---
name: ctu-can-fd-rtl-architecture
description: Architecture map of the CTU CAN FD VHDL RTL in src/ — subsystem responsibilities, data flow, key state machines, and known refactor-in-progress spots. Load this before making non-trivial changes to src/, reviewing a diff under src/, or answering questions about how the core works (protocol_control_fsm, bit stuffing, fault confinement, bit timing/SSP, RX/TX buffers, TXT buffer backup mode, frame filters, register/interrupt infrastructure). Triggers on: can_core, protocol_control, bus_sampling, prescaler, rx_buffer, tx_arbitrator, txt_buffer, frame_filters, memory_registers, interrupt_manager, bit stuffing/destuffing, SSP/secondary sample point, fault confinement, TEC/REC, bus-off, backup buffer / TXBBM.
---

# CTU CAN FD RTL Architecture

Scope: `/WORK/ctu_can_fd/src/` only. This is the CTU CAN FD IP core — a CAN/CAN-FD bus
controller, VHDL-93, fully synchronous single-clock-domain design, ISO 11898-1:2015
compliant, FPGA and ASIC targets. ~36,500 lines across 90 files. This skill is a
condensed map built from a full read-through of the subsystem; use it to get oriented
fast, then read the actual files for anything you're about to change.

**PSL assertions convention**: `-- psl ...` comment blocks throughout this codebase are
**live, active PSL assertions/covers** checked by PSL-aware simulators (NVC, Synopsys
VCS — both used in this repo's regression flow). VHDL-93 has no native assertion
statement, so embedding PSL inside specially-tagged comments is the standard mechanism,
not disabled or dead code. Never describe them as "commented out."

## Top level

`can_top_level.vhd` instantiates everything and is the integration point. Fully
synchronous, single clock (`clk_sys`) + async reset (`res_n`) synchronized internally by
`rst_sync`. `can_top_ahb.vhd` / `can_top_apb.vhd` wrap it with AMBA AHB-Lite / APB4
adaptors — both translate to the *same* native internal register bus
(`data_in`/`data_out`/`adress`/`scs`/`srd`/`swr`/`sbe`), so the two front-ends are
interchangeable and `memory_registers.vhd` does its own internal address-block decode
regardless of which front-end is attached.

Nine sub-blocks wired in `can_top_level.vhd`: memory registers, interrupt manager,
prescaler, bus sampling, RX buffer, TXT buffers, TX arbitrator, frame filters, reset
synchroniser — plus `can_core` (the protocol engine) which is the hub everything else
talks to.

Generics of note: `rx_buffer_size`, `txt_buffer_count` (2-8), `sup_filtA/B/C/range`,
`sup_parity` (adds parity to TXT/RX buffer RAMs), `sup_traffic_ctrs`, `target_technology`
(`C_TECH_FPGA`/`C_TECH_ASIC` — gates clock gating and other ASIC-only features).

## can_core/ — the protocol engine

`can_core.vhd` (~1000 lines) is purely structural (no protocol logic) — instantiates and
wires: `protocol_control` (frame-level engine wrapper), `operation_control` (tiny
transmitter/receiver/idle role FSM), `fault_confinement`, `can_crc`, `bit_stuffing` /
`bit_destuffing`, `bus_traffic_counters` (optional), `trigger_mux` (derives all internal
pipeline clock-enables from the prescaler's `rx_triggers`/`tx_trigger`).

Data flow: CAN bus RX (`rx_data_wbs`) → `bit_destuffing` → feeds `protocol_control` and
CRC. `protocol_control.tx_data_nbs` → `bit_stuffing` → drives `tx_data_wbs` to the bus.
Loopback path muxes `bds_data_in` between own TX (self-reception for SSP), bus-monitor
loopback, and plain RX.

### protocol_control / protocol_control_fsm.vhd (3300 lines, the largest file in the repo)

State enum: `t_protocol_control_state` in `packages/can_types_pkg.vhd:106-145` (~37
states). Two processes: `next_state_proc` (pure transition logic) and `curr_state_proc`
(per-state output/side-effects, defaults reset each cycle).

Phases: idle/integration (`s_pc_off`→`s_pc_integrating`→`s_pc_idle`, plus
`s_pc_reintegrating_wait`/`s_pc_reintegrating` for bus-off recovery gated by a 128×11-bit
`reintegration_counter`) → arbitration (`s_pc_sof`→`s_pc_base_id`→...→ext ID if IDE) →
control field (`s_pc_edl_r1`/`s_pc_edl_r0` = FDF/EDL, the CAN-FD-vs-2.0 branch point;
`s_pc_brs` switches `sp_control` to `DATA_SAMPLE`; `s_pc_esi`; `s_pc_dlc`) → data
(`s_pc_data`) → stuff-count (`s_pc_stuff_count`, ISO CAN FD only) → CRC → CRC delim → ACK
(1 bit CAN 2.0, 2 bits CAN FD) → ACK delim → EOF → intermission → suspend (error-passive
transmitters only) or back to SOF. Error handling: `s_pc_act_err_flag`/`s_pc_pas_err_flag`
→ `s_pc_err_delim_wait` → delim or "too long" state. Overload mirrors error handling.

**ISO vs non-ISO CAN FD** (`mr_settings_nisofd`): the Stuff-Count+parity field
(`s_pc_stuff_count`) only exists for ISO CAN FD. CRC17/21 init-vector MSB differs
(`can_crc.vhd:206-216`). Stuff-count field is only CRC-checked in ISO mode
(`err_detector.vhd:307-310`).

**Protocol exception** (`mr_settings_pex`): if FDE disabled and a recessive FDF/EDL bit is
sampled, or FDE enabled and a recessive reserved bit is sampled, the unit silently goes to
`s_pc_integrating` instead of raising Form Error (`mr_status_pexs` latched status).

Non-obvious things worth knowing before touching this file:
- RX data is processed one cycle *behind* the sample point ("Process" pipeline stage)
  while the TX shift register is loaded in the *same* cycle — the whole timing model
  depends on this offset (documented in the file header, ~lines 74-93).
- `crc_calc_from_rx` forces CRC to be computed from RX data during arbitration for *both*
  transmitter and receiver, so a transmitter that loses arbitration doesn't redo CRC.
- `txtb_gate_mem_read`/`txtb_num_words_gate` exist purely to prevent spurious TXT-buffer
  RAM parity errors from reading past the last valid word.
- `retr_ctr_add_block` is a latch-like guard preventing the retransmit counter from
  double-incrementing across multiple error frames within one logical frame; must be
  cleared exactly once per intermission.

### Bit stuffing / destuffing

`bit_stuffing.vhd` / `bit_destuffing.vhd` are near-mirror modules: a same-bits counter
(reset at 5 consecutive equal bits) plus a mod-8 stuffed/destuffed-bit counter (`bst_ctr`/
`dst_ctr`, feeds the ISO stuff-count field). Normal stuffing: complementary bit after 5
equal bits. Fixed stuffing (CRC field for FD, and the stuff-count field): every 4 bits
regardless of polarity; the normal→fixed transition forces one extra bit. A violated fixed
stuff rule is reported as **Form Error**, not Stuff Error (`err_detector.vhd:265-268`).
`data_halt`/`destuffed` stall `trigger_mux`'s CRC/process triggers for one bit time when a
stuff bit was just inserted/removed. `trigger_mux` produces 4 separate CRC data/trigger
pairs so CRC15 is computed from unstuffed data while CRC17/21 include normal stuff bits
but exclude fixed ones, per spec.

### Fault confinement

Three layers: `err_counters.vhd` (independent 9-bit TX/RX error counters — RX saturates at
0x1FF per ISO 12.1.4.3 rather than wrapping, and is set to 120 rather than 0 when
decrementing from >127; separate informational 16-bit nominal/data bitrate error
counters) → `fault_confinement_rules.vhd` (pure combinational ISO 11898-1 §12.1.4 rules:
`inc_one`/`inc_eight`/`dec_one`, gated off entirely in bus-monitor/ROM mode) →
`fault_confinement_fsm.vhd` (3-state FSM: error-active/passive/bus-off, purely threshold
comparisons; bus-off recovery only via `set_err_active` after 128×11 recessive bits
post-reintegration). `protocol_control_fsm` never touches counters directly — it only
asserts semantic signals (`primary_err`, `err_delim_late`, `decrement_rec`,
`err_ctrs_unchanged`, etc.) that the rules module translates.

### Key can_core external interfaces
Prescaler (`rx_triggers`, `tx_trigger`, `sync_control`, `sp_control`, `no_pos_resync`,
`nbt_ctrs_en`/`dbt_ctrs_en`) · Bus sampling (`rx_data_wbs`/`tx_data_wbs`, `bit_err`,
SSP-related signals) · TX arbitrator/TXT buffers (`tran_*` fields, `txtb_hw_cmd` record:
lock/valid/err/arbl/failed) · RX buffer/filters (`rec_*` fields,
`store_metadata`/`store_data`/`rec_valid`/`rec_abort`) · Interrupt manager
(`arbitration_lost`, `tran_valid`, `err_detected`, `fcs_changed`, ...) · Memory registers:
aggregate status via `cc_stat` (`t_can_core_stat`) and `pc_dbg` (`t_protocol_control_dbg`).

## prescaler/ + bus_sampling/ — bit timing and physical sampling

**Split of responsibility**: prescaler generates bit-time segments/triggers with no
notion of the CAN bus; bus_sampling owns the physical RX/TX interface and bit error
detection. They interact through a small signal set (`tx_trigger`, `rx_triggers`,
`tq_edge` prescaler→sampling; `sync_edge` sampling→prescaler; `sync_control`/`sp_control`
from can_core to both).

Both NBT (nominal) and DBT (data) bit-time counter datapaths in `bit_time_counters.vhd`
run **in parallel continuously** — BRS doesn't switch a mode, it just changes which
counter's output `sp_control` selects (`prescaler.vhd:474`). Counter widths are
asymmetric by design: NBT is 8 bits (covers up to 222), DBT is 7 bits (up to 126).
`bit_time_cfg_capture.vhd` captures nominal/data timing config once on `SETTINGS[ENA]`
rising edge to avoid long combinational paths every bit. `bit_time_fsm.vhd` is a simple
3-state FSM (`s_bt_reset`→`s_bt_tseg1`⇄`s_bt_tseg2`). `bit_segment_meter.vhd` is the
densest/highest-risk module here — computes expected segment length folding in
resync extension/shortening, SJW clamping (`phase_err_mt_sjw`/`_eq_sjw`/`_sjw_by_one`,
`exit_ph2_immediate`); read its header ASCII diagrams before touching it.

`synchronisation_checker.vhd` decides hard-sync vs resync: a `sync_flag` enforces "only
one sync per bit"; resync is valid in TSEG2 unconditionally, in TSEG1 only if
`no_pos_resync='0'` (CAN rule: a transmitter ignores early edges caused by its own
dominant bit — `no_pos_resync` is computed in `can_core.vhd` as
`tx_data_wbs = DOMINANT`).

**Secondary Sample Point (SSP)**: needed because in BRS data phase, transceiver TX→RX
loopback delay can exceed one data bit time. `trv_delay_meas.vhd` measures the loop delay
(TX edge to RX edge); `ssp_generator.vhd` uses `trv_delay + offset` for the *first* SSP
after BRS, then a measured bit-time-counter value for subsequent SSPs within the data
phase (so later SSPs track any within-frame resync). `tx_data_cache.vhd` is a small FIFO
holding TX values so `bit_err_detector` compares against the TX value belonging to the
correct (SSP-offset) bit.

**Bit error detection** (`bit_err_detector.vhd`): two conditions ORed — normal (TX≠RX at
regular sample point, when not in SECONDARY_SAMPLE) and SSP (TX≠RX at the SSP trigger,
latched since it can occur before the next regular sample point). `bit_err` itself is a
registered value held between sample points, not a single-cycle pulse.

Known sharp edges: `tx_data_cache.vhd` hardcodes 3-bit RAM addressing even though its
generics claim a configurable depth/pointer-width — only safe because the sole
instantiation fixes depth=8. `trigger_generator.vhd` declares
`G_SAMPLE_TRIGGER_COUNT : natural range 2 to 8` but only ever drives 2 of the vector
elements — fine only because `C_SAMPLE_TRIGGER_COUNT := 2` is the only value used anywhere.
Git history shows active WIP iteration on BRP=1 edge-case counting in
`bit_time_counters.vhd` — double check that corner case if timing bugs are suspected.

## rx_buffer/ + tx_arbitrator/ + txt_buffer/ + frame_filters/

### RX buffer
Circular FIFO of 32-bit words (`G_RX_BUFF_SIZE`, power-of-2). `rx_buffer_fsm.vhd`
sequences storage: frame-format → identifier → timestamp placeholder (skip states) →
data words → timestamp overwrite at frame end (separate write pointer, so SW-configurable
RTSOP — timestamp-at-begin-vs-end — doesn't stall the raw write pointer). Optional
error-frame logging path (`mr_mode_erfm`) stores FRAME_FORMAT/IDENTIFIER/TIMESTAMP plus
ERF_IND/TYPE/POS/ERP fields for logged error frames.

Overrun: `write_pointer_raw` (in-progress) vs `write_pointer` (committed). On overrun the
frame is never committed and `write_pointer_raw` rolls back. Two overrun flags: SW-visible
sticky one (cleared by `mr_command_cdo`) and an internal one that auto-clears at next
frame's idle state. **RRB (Release Receive Buffer) mid-storage** is a subtle,
correctness-critical case: pointers are wiped immediately but the RX FSM/CAN Core can't be
told to stop mid-frame, so the internal overrun flag silently absorbs the discrepancy
(`rx_buffer.vhd` ~lines 773-800 has the explanatory comment — read it before touching
pointer-reset logic).

`STATUS[RXPE]` (parity error) is set at *read* time gated by `MODE[RXBAM]`, not at
read-pointer-move time (changed in commit `5ecc5dad` — see refactor notes below).

RWCNT (remaining word count) computed combinationally from DLC with special cases:
RTR/ERF frames → 3 words; Normal CAN DLC>8 capped at 5. Loopback frames store
`curr_txtb_index` (from TX arbitrator) so SW can tell which TXT buffer a looped-back frame
came from.

### TX arbitrator
`priority_decoder.vhd`: pure combinational comparator tree (supports up to 8 buffers),
ties resolve to the **lower index**. `tx_arbitrator_fsm.vhd`: multi-stage validation FSM
(`s_arb_idle`→select low/upper timestamp→frame-test/format/ID words→`s_arb_validated`→
lock→`s_arb_locked`) with one-cycle wait states for synchronous RAM reads; restarts from
almost any state if a higher-priority frame becomes ready mid-validation
(`select_index_changed`). Timestamp-triggered TX (`mr_mode_tttm`) compares frame timestamp
against the free-running `timestamp` input before allowing selection. Metadata is
captured into "double buffer" registers and only atomically committed to CAN-Core-visible
registers once the whole frame validates parity-clean.

**Backup buffer (TXBBM) mechanism** — the key non-obvious design in this cluster: buffers
are indexed 0..N-1; **even buffers are "original," odd buffers (i+1) are their paired
"backup."** When `mr_mode_txbbm='1'`, the odd buffer's arbitration priority is overridden
to equal its paired even buffer's priority. SW loads the *same* frame into both; once the
even (original) buffer's transmission completes (success or exhaustion), the odd (backup)
buffer auto-skips itself — implemented via `txtb_is_bb` computed in `tx_arbitrator.vhd`,
consumed in `txt_buffer_odd.vhd`'s `buffer_skipped` logic. There's no HW enforcement that
SW actually loaded identical content into both buffers of a pair — correctness here is a
software convention, only checked by a sim-only PSL assertion.

### TXT buffer FSM (`t_txt_buf_state` in `can_types_pkg.vhd:189-198`)
`s_txt_empty`→`s_txt_ready`→(`s_txt_tx_prog`|`s_txt_ab_prog`)→(`s_txt_ok`|`s_txt_failed`|
`s_txt_aborted`|`s_txt_parity_err`)→back to ready/empty via SW commands.
`txtb_available` (visible to TX arbitrator) is asserted **only** in `s_txt_ready` with no
pending abort — deliberately avoids a HW/SW race. `txtb_allow_bb` = ready OR tx_prog OR
ab_prog (content is "committed" and can be mirrored).

**`txt_buffer_even.vhd` vs `txt_buffer_odd.vhd`**: recently split from (presumably) one
generic entity. Even variant has no backup-specific ports. Odd variant adds `txtb_is_bb`
input, `buffer_skipped` logic, and `txtb_bb_parity_error` output (STATUS[TXDPE] for
parity errors detected while operating as backup). If you add a port to one, check
whether it belongs on both.

### Frame filters
Filter A/B/C (bit mask+value, `bit_filter.vhd`) + range filter (`range_filter.vhd`), each
independently enabled per frame-type combination (FD/2.0 × Basic/Extended) via one-hot
enable bits (FAFE/FAFB/FANE/FANB style, per filter). Frame passes if `rec_ivld='0'`
(unconfirmed identifier — always accept), else rejected if RTR and `mr_settings_fdrf`
drops RTR frames, else accepted if any enabled filter matches. `mr_mode_afm` is the master
enable — if off, everything passes. If **no** filters are synthesized at all
(`G_SUP_FILTx=false` for all), acceptance is hardwired true even overriding `mr_mode_afm`,
so disabling filters at synthesis time can never accidentally reject frames. Sits
between can_core's raw `store_metadata`/`store_data`/`rec_valid`/`rec_abort` and RX
buffer's `_f`-suffixed filtered versions — a pure pass-through gate.

## memory_registers/ + interrupt_manager/ + interface/ + common_blocks/ + packages/

### Registers
`memory_registers.vhd` (hand-written) wraps two **generated** register-map entities
(`generated/control_registers_reg_map.vhd` ~3300 lines, `generated/test_registers_reg_map.vhd`)
produced by an external pyXact-based generator (see `scripts/pyXact_generator` in repo
root) — don't hand-edit generated files. Address bits select block (control/test/TXT
buffer RAM blocks); reads are registered one cycle (Avalon-style: address/cs cycle N,
data valid cycle N+1). Per-block clock gating for ASIC power. Two reset registers: soft
reset (MODE[RST]) and core reset (soft reset OR core disabled).

Generic register primitives (`memory_reg_rw`, `memory_reg_os`, and `_lock` variants):
- `memory_reg_rw`: a true flip-flop, holds value until overwritten.
- `memory_reg_os` ("one-shot"): **not a register** — combinationally equals the write
  data during the write cycle and snaps back to a fixed reset value otherwise. Used for
  self-clearing command/pulse bits (MODE[RST], TX commands, INT_ENA/MASK set/clear).
- `_lock` variants additionally gate writes off while locked. Two lock conditions computed
  once in `memory_registers.vhd`: locked unless MODE[TSTM] (test mode) is set; locked once
  SETTINGS[ENA]=1 (core running) — protects config from being mutated live.

`can_registers_pkg.vhd` (generated) declares the big `control_registers_out_t` /
`Control_registers_in_t` records — the typed-struct API the rest of the core consumes
instead of raw address decoding.

### Interrupt manager
`int_module.vhd`: one instance per interrupt source (12 total, `G_INT_COUNT`). Status bit
sets only if not masked, clears on SW write-1; **set wins over clear** if simultaneous.
Mask: set wins over clear. Enable: **clear wins over set** (asymmetric from status/mask —
worth noting if debugging enable-bit races). Note: the file's header comment describes a
`clear_priority` generic that doesn't actually exist on the entity — stale documentation
from an earlier design, not a live parameter.

`int_manager.vhd` maps 12 named causes (RXI/TXI/EWLI/DOI/FCSI/ALI/BEI/OFI/RXFI/BSI/
RBNEI/TXBHCI) to `int_module` instances, asserts `int` output whenever any enabled status
bit is set, registered through one extra flip-flop for glitch-free output.

### Bus interfaces
`ahb_ifc.vhd` (AMBA AHB-Lite): tracks a registered "mid-write" flag since AHB is
pipelined (address phase, then data phase next cycle); asserts a wait state on
read-after-write hazard. `apb_ifc.vhd` (AMBA APB4): simpler, not pipelined, always
`pready='1'`; `reg_rden_o` pulses exactly 1 cycle during SETUP phase — matters for
FIFO-like registers (RX buffer read pointer) that must advance only once per transaction.

### common_blocks/ (one-liners)
`clk_gate` (ASIC clock gate, transparent on FPGA, gated by scan_enable) ·
`dff_arst`/`dff_arst_ce` (base flip-flop primitives) · `dlc_decoder` (DLC→byte length,
CAN2.0/FD aware) · `dp_inf_ram`/`dp_inf_ram_be` (generic inferred dual-port RAM, `_be`
variant adds byte enables) · `majority_decoder_3` (3-input majority vote) · `mux2` ·
`parity_calculator` (XOR-tree parity, drives the `sup_parity` feature) · `rst_reg`
(reset register with scan-mode bypass — DFT support) · `rst_sync` (2-FF CDC reset
synchronizer) · `shift_reg`/`shift_reg_byte`/`shift_reg_preload` (shift register variants)
· `sig_sync` (2-FF CDC bit synchronizer).

### Packages
`can_config_pkg.vhd`: the tunable "knobs" — version constants, buffer/interrupt counts,
bit-timing field widths, CRC polynomials, SSP sizing, device ID.
`can_constants_pkg.vhd`: protocol-level fixed constants — DOMINANT/RECESSIVE encodings,
sync/sample-point-type encodings, per-field bit-duration constants for the FSM,
`C_TECH_ASIC`/`C_TECH_FPGA` selectors.
`can_types_pkg.vhd`: shared record/enum types — all the FSM state enums, TXT buffer array
types, `t_can_core_stat`/`t_protocol_control_dbg` cross-module status records, DLC tables.
`can_fd_frame_format.vhd` / `can_fd_register_map.vhd`: large, mostly field-offset/bit
constants — grep rather than read linearly.

## Known refactor-in-progress / loose ends (as of branch `rtl-refactoring`)

- Endian swapper module (`common_blocks/endian_swapper.vhd`) was deleted; the byte-swap
  it did is now inlined directly in `can_core/tx_shift_reg.vhd` (commit `8a719f99`).
- `STATUS[RXPE]` semantics changed from "set on read-pointer move" to "set on RX_DATA read
  gated by MODE[RXBAM]" (commit `5ecc5dad`, `rx_buffer.vhd` ~line 904).
- `tx_arbitrator.vhd:528` has a leftover `-- TODO: Convert pointers to std_logic!`.
- `int_module.vhd` header comment references a nonexistent `clear_priority` generic
  (stale doc from a prior design change).
- `txt_buffer_even.vhd`/`txt_buffer_odd.vhd` split is itself a recent refactor (was
  presumably one generic entity before) — check both when adding ports to either.
- `tx_data_cache.vhd` and `trigger_generator.vhd` generics claim more flexibility
  (configurable depth/pointer-width, up to 8 sample triggers) than the implementation
  actually delivers — currently harmless because only one configuration is ever
  instantiated, but don't assume the generics are safe to vary without re-checking.

## Where to look for more detail

- Full port-level wiring: `can_top_level.vhd`.
- Frame timing/protocol spec cross-reference: `doc/datasheet/ctu_can_fd_datasheet.lyx` and
  `doc/system_architecture/ctu_can_fd_architecture.lyx` (LyX format).
- Test structure / how to run regressions: repo root `README.md`.
