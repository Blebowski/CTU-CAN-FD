---
name: ctu-can-fd-naming-conventions
description: VHDL naming/coding-style conventions enforced across this repo's RTL (src/) and testbench (test/, excluding test/main_tb/iso-16845-compliance-tests) — prefixes for processes, instances, generate statements, generics, constants, types, FSM states, and signal/register naming idioms. Load this to check whether new or existing VHDL code in this repo follows the established style, or before renaming/refactoring identifiers. Triggers on: naming convention, coding style, style check, lint, p_ prefix, i_ prefix, g_ prefix, G_ generic, C_ constant, t_ type.
---

# CTU CAN FD VHDL Naming Conventions

Scope: `src/` (RTL) and `test/` excluding `test/main_tb/iso-16845-compliance-tests/`
(a vendored/external compliance-test library with its own conventions, out of scope).
These conventions were established/normalized through an explicit repo-wide renaming
effort on branch `rtl-refactoring` and are the current target style — use this skill
to check whether code (new or existing) conforms, not to describe legacy style.

## Prefix conventions for VHDL constructs

| Construct | Convention | Example |
|---|---|---|
| Process (labeled) | `p_` prefix, lowercase | `p_next_state : process(...)`, `p_ram_write : process(clk_sys)` |
| Component/entity instantiation (labeled) | `i_` prefix, lowercase | `i_can_core : entity ctu_can_fd_rtl.can_core`, `i_rst_sync : entity ...` |
| Generate statement (`for`/`if`/`case` ... `generate`) | `g_` prefix, lowercase | `g_txt_buf_even : if ((i mod 2) = 0) generate`, `g_bit : for i in 0 to data_width - 1 generate` |
| Generic | `G_` prefix, ALL CAPS | `G_RX_BUF_SIZE`, `G_TXT_BUF_COUNT`, `G_FILT_A_EN`, `G_TECHNOLOGY` |
| Constant | `C_` prefix, ALL CAPS | `C_TXT_BUFFER_COUNT`, `C_TECH_FPGA`, `C_CAN_DEVICE_ID`, `C_CRC15_POL` |
| Type (record/enum/array) | `t_` prefix, lowercase | `t_can_core_stat`, `t_protocol_control_state`, `t_txtb_hw_cmd`, `t_com_channel` |
| FSM state (enum literal) | `s_<module>_<state>`, lowercase | `s_pc_off`, `s_pc_integrating` (protocol_control_fsm), `s_rxb_idle` (rx_buffer_fsm), `s_txt_empty` (txt_buffer_fsm), `s_arb_idle` (tx_arbitrator_fsm), `s_fc_err_active` (fault_confinement_fsm), `s_bt_reset` (bit_time_fsm) |
| Signal / port | lowercase snake_case, no mandated prefix | `clk_sys`, `rst_n`, `scan_mode`, `tx_data_wbs`, `rec_ident` |

### Rules for stripping old-style suffixes when applying a prefix

When a process/instance/generate label already carries the *old* suffix-based name,
convert it by **stripping the exact suffix and prepending the prefix** — do not just
prepend the prefix and leave the suffix in place:

- Process: strip a trailing `_proc` (or `_process`) before adding `p_`.
  `next_state_proc` → `p_next_state` (not `p_next_state_proc`).
  Also collapse any leftover `proc`/`process` **infix**, not just suffix:
  `p_pli_listener_process` → `p_pli_listener`.
- Instance: strip a trailing `_inst` before adding `i_`.
  `rst_sync_inst` → `i_rst_sync`.
- Generate: strip a trailing `_gen` (or `_g`) before adding `g_`; also collapse a
  `_gen_` **infix** down to a single `_`: `test_registers_gen_true` → first pass
  `g_test_registers_gen_true`, then infix collapse → `g_test_registers_true`.
  `gen_filt_pos` (old *prefix*-style name, no trailing `_gen`) → `g_gen_filt_pos` on
  first pass, then infix collapse (since `g_` + `gen_` forms `_gen_`) →
  **`g_filt_pos`**.

If a label has **no** recognizable old suffix at all, just prepend the prefix as-is
(e.g. `data_reader` → `p_data_reader`, `cons_check` → `p_cons_check`,
`fault_conf_state_reg` → `p_fault_conf_state_reg`). Never leave a label unprefixed,
and never double-prefix a label that's already compliant (`g_tech_asic` stays
`g_tech_asic`, not `g_g_tech_asic`).

Unlabeled processes/generates found during a renaming pass should be given a new,
descriptive `p_`/`g_`-prefixed label based on what they do — don't leave them
unlabeled if the surrounding pass is establishing this convention.

## Register / pipeline signal idioms (not strict prefixes, but consistent patterns)

- **`_i` suffix**: an internal signal that shadows an output port of the same base
  name, used because VHDL output ports can't be read back on the RHS of an
  assignment. E.g. `arbitration_lost_i`, `is_bus_off_i`, `sp_control_i` — the port
  itself is a plain read-only alias/assignment of the `_i` signal.
- **`_q` / `_d` suffix pair**: classic register naming — `_d` is the combinational
  "next value" input to a flip-flop, `_q` is the registered/current output.
  E.g. `same_bits_d`/`same_bits_q`, `dst_ctr_d`/`dst_ctr_q`, `destuffed_d`/`destuffed_q`.

## Package and file naming

- One package per file; **package name matches the file's basename** (e.g.
  `can_config_pkg.vhd` declares `package can_config_pkg`, `tb_report_pkg.vhd`
  declares `package tb_report_pkg`).
- Package files end in `_pkg` (RTL: `can_config_pkg`, `can_constants_pkg`,
  `can_types_pkg`; TB: `tb_report_pkg`, `tb_communication_pkg`, `tb_random_pkg`, etc.)
  — exception: `can_fd_register_map.vhd` / `can_fd_frame_format.vhd` (RTL) and
  `can_fd_tb_register_map.vhd` (TB) don't carry the `_pkg` suffix despite being
  packages, for historical/generator reasons.
- **Entity name matches the file's basename** (e.g. `protocol_control_fsm.vhd` →
  `entity protocol_control_fsm`), with the top-level entities being the main
  intentional exception where file name carries extra qualifiers
  (`ctu_can_fd_top_apb.vhd` → `entity ctu_can_fd_top_apb`, consistent) — always
  1:1, no divergence observed.

## Known, intentional exceptions (do not flag these as style violations)

- **Generated files are exempt.** `src/memory_registers/generated/*.vhd` (register
  map, address decoder, generic register primitives) and
  `test/main_tb/pkg/can_fd_tb_register_map.vhd` are produced by an external
  pyXact-based generator (see `scripts/pyXact_generator`). Their constant naming
  (e.g. register-field constants like `ALC_BASE_ID`, `ACF_ENABLED`, `AFM_IND` — no
  `C_` prefix) follows the generator's own scheme, not the hand-written `C_`
  convention, and should not be renamed/flagged. `control_registers_reg_map.vhd`'s
  generate labels similarly keep generator-assigned mixed-case base names even after
  getting a `g_` prefix (e.g. `g_FILTER_A_MASK_present_gen_t` →
  `g_FILTER_A_MASK_present_t` — only the prefix/infix convention is enforced, the
  generator's own field-name casing is left alone).
- **Legacy type names without `t_` prefix**: `dlc_type`, `length_type`,
  `logger_state_type` in `src/packages/` use an older `..._type` suffix style
  instead of `t_` prefix. These are pre-existing/legacy and have not been part of
  the renaming sweep — flag only if asked to fully normalize types, not by default.
- **`test/main_tb/iso-16845-compliance-tests/`** is an external/vendored compliance
  test suite (C++ + its own VHDL cosimulation shims) and is entirely out of scope
  for these conventions.
- **Gate-level netlist boundary**: `test/main_tb/tb_top_gates_ctu_can_fd.vhd`
  instantiates a frozen, already-synthesized `ctu_can_fd_gates.ctu_can_fd_top`
  netlist. If/when RTL-side port or generic names are renamed in the future, this
  file's DUT-facing side may lag behind until the netlist is regenerated — that's
  expected, not a bug.
- **VIP/agent generics not exposed as `G_`-prefixed**: TB-only generics that aren't
  meant to mirror an RTL generic (e.g. `test_name`, `test_type`, `seed`,
  `cfg_sys_clk_period`, `func_cov_en` in `ctu_can_fd_vip.vhd`) are lowercase by
  convention — only generics that exist to directly propagate a same-named RTL
  generic value get the `G_` treatment (e.g. `G_RX_BUF_SIZE`, `G_TXT_BUF_COUNT`
  on `ctu_can_fd_vip`, mirroring `ctu_can_fd_top`'s generics of the same name).

## Cross-cutting hazard when renaming instances/generates in RTL

Renaming an `i_`/`g_`-prefixed hierarchy element in `src/` has ripple effects that
must be updated in lockstep, or simulation breaks silently until elaboration:

1. **VHDL-2008 external names** (`<< signal .path.to.signal : type >>`) in
   `test/main_tb/agents/functional_coverage_agent/*.vhd` and the whitebox
   force/release paths in `test/main_tb/ctu_can_fd_vip.vhd` hardcode full
   hierarchical paths (e.g. `.tb_top_ctu_can_fd.i_dut.i_mac_top.g_bus_traffic_ctrs.
   i_mac_bus_traffic_counters.tx_frame_ctr_i`). These are plain-text paths, not
   resolved by the compiler until elaboration/simulation — a stale path fails
   silently at elaboration, not compile time. By convention these paths are written
   **all lower-case** in both locations (VHDL identifiers are case-insensitive, so
   this is purely a style convention, but keep it consistent when editing/renaming).
2. **The NVC coverage exclusion file** `test/exclude_files/nvc_exclusions_3v0.txt`
   encodes the same kind of hierarchical paths in upper-case
   (`CTU_CAN_FD_TB.TB_TOP_CTU_CAN_FD.I_DUT.I_CAN_CORE...`) — VHDL identifiers are
   case-insensitive so NVC canonicalizes to upper case there, but the identifier
   text must still track any rename.
3. Both of the above must be greped for the old name (case-insensitively) and
   updated whenever an instance/generate label on the path changes.

## How to use this skill for a style check

1. Grep for labeled processes/instances/generates not matching `^\s*p_`/`^\s*i_`/
   `^\s*g_` respectively (see the exact detection regexes used historically:
   `^\s*[A-Za-z_][A-Za-z0-9_]*\s*:\s*process\b`,
   `^\s*[A-Za-z_][A-Za-z0-9_]*\s*:\s*(entity|component)\b` plus the bare
   `label : component_name` form (no `entity`/`component` keyword, valid VHDL when a
   matching `component` declaration is visible — used throughout
   `ctu_can_fd_vip.vhd` and the `tb_top_*.vhd` files), and
   `^\s*[A-Za-z_][A-Za-z0-9_]*\s*:\s*(for|if|case)\b`).
2. Cross-check any hit against the "known exceptions" list above before flagging it.
3. For generics/constants, check for ALL CAPS + correct prefix; for types, check for
   `t_` prefix; for FSM enum literals, check for `s_<module>_` prefix consistency
   within the same state type.
4. If a rename is warranted, always check `test/main_tb/agents/functional_coverage_agent/`
   external names, `test/main_tb/ctu_can_fd_vip.vhd`'s own whitebox force/release
   external names (a separate, easy-to-forget hardcoded-path spot — see hazard
   section above), and `test/exclude_files/nvc_exclusions_3v0.txt` for the same
   identifier before considering the rename complete.
