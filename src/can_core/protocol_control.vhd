--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2021-present Ondrej Ille
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- educational, research, evaluation, self-interest purposes. Using the
-- Component for commercial purposes is forbidden unless previously agreed with
-- Copyright holder.
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Component.
--
-- THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS
-- IN THE COMPONENT.
--
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN
-- protocol license from Bosch.
--
-- -------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2020 MIT License
--
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <martin.jerabek01@gmail.com>
--
-- Project advisors:
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
--
-- Department of Measurement         (http://meas.fel.cvut.cz/)
-- Faculty of Electrical Engineering (http://www.fel.cvut.cz)
-- Czech Technical University        (http://www.cvut.cz/)
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to deal in the Component without restriction, including without limitation
-- the rights to use, copy, modify, merge, publish, distribute, sublicense,
-- and/or sell copies of the Component, and to permit persons to whom the
-- Component is furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Component.
--
-- THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS
-- IN THE COMPONENT.
--
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN
-- protocol license from Bosch.
--
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Module:
--  Protocol control module.
--
-- Sub-modules:
--  1. Protocol control FSM.
--  2. TX Shift register.
--  3. RX Shift register.
--  4. Error detector.
--  5. Control counter
--  6. Re-integration counter.
--  7. Retransmitt counter.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity protocol_control is
    generic (
        -- Control counter width
        G_CTRL_CTR_WIDTH        :     natural;

        -- Retransmitt limit counter width
        G_RETR_LIM_CTR_WIDTH    :     natural;

        -- Insert pipeline on "error_valid"
        G_ERR_VALID_PIPELINE    :     boolean
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        res_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- DFT support
        -------------------------------------------------------------------------------------------
        scan_enable             : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_mode_acf             : in  std_logic;
        mr_mode_stm             : in  std_logic;
        mr_mode_bmm             : in  std_logic;
        mr_mode_fde             : in  std_logic;
        mr_mode_rom             : in  std_logic;
        mr_mode_tstm            : in  std_logic;
        mr_mode_sam             : in  std_logic;

        mr_settings_ena         : in  std_logic;
        mr_settings_nisofd      : in  std_logic;
        mr_settings_rtrth       : in  std_logic_vector(G_RETR_LIM_CTR_WIDTH - 1 downto 0);
        mr_settings_rtrle       : in  std_logic;
        mr_settings_ilbp        : in  std_logic;
        mr_settings_pex         : in  std_logic;

        mr_command_ercrst       : in  std_logic;
        mr_command_cpexs        : in  std_logic;

        mr_ssp_cfg_ssp_src      : in  std_logic_vector(1 downto 0);

        -- ALC register
        alc_alc_bit             : out std_logic_vector(4 downto 0);
        alc_alc_id_field        : out std_logic_vector(2 downto 0);

        -- ERR_CAPT register
        err_capt_err_type       : out std_logic_vector(2 downto 0);
        err_capt_err_pos        : out std_logic_vector(3 downto 0);
        err_capt_err_erp        : out std_logic;

        -- Protocol exception status
        mr_status_pexs          : out std_logic;

        -- Protocol control debug information
        pc_dbg                  : out t_protocol_control_dbg;

        -------------------------------------------------------------------------------------------
        -- TXT Buffers interface
        -------------------------------------------------------------------------------------------
        -- TX Data word
        tran_word               : in  std_logic_vector(31 downto 0);

        -- TX Data length code
        tran_dlc                : in  std_logic_vector(3 downto 0);

        -- TX Remote transmission request flag
        tran_is_rtr             : in  std_logic;

        -- TX Identifier type (0-Basic, 1-Extended)
        tran_ident_type         : in  std_logic;

        -- TX Frame type (0-CAN 2.0, 1-CAN FD)
        tran_frame_type         : in  std_logic;

        -- TX Bit rate shift
        tran_brs                : in  std_logic;

        -- TX Identifier
        tran_identifier         : in  std_logic_vector(28 downto 0);

        -- TX frame test word
        tran_frame_test         : in  t_frame_test_w;

        -- Frame in TXT Buffer is valid any can be transmitted.
        tran_frame_valid        : in  std_logic;

        -- Transmission frame
        tran_frame_parity_error : in  std_logic;

        -- HW Commands for TX Arbitrator and TXT Buffers
        txtb_hw_cmd             : out t_txtb_hw_cmd;

        -- Pointer to TXT buffer memory
        txtb_ptr                : out natural range 0 to 20;

        -- Clock enable for TXT Buffer memory
        txtb_clk_en             : out std_logic;

        -- Selected TXT Buffer index changed
        txtb_changed            : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- RX Buffer interface
        -------------------------------------------------------------------------------------------
        -- RX CAN Identifier
        rec_ident               : out std_logic_vector(28 downto 0);

        -- RX Data length code
        rec_dlc                 : out std_logic_vector(3 downto 0);

        -- RX Remote transmission request flag
        rec_is_rtr              : out std_logic;

        -- RX Recieved identifier type (0-BASE Format, 1-Extended Format);
        rec_ident_type          : out std_logic;

        -- RX frame type (0-CAN 2.0, 1- CAN FD)
        rec_frame_type          : out std_logic;

        -- Received Loopback frame
        rec_lbpf                : out std_logic;

        -- RX Bit rate shift Flag
        rec_brs                 : out std_logic;

        -- RX Error state indicator
        rec_esi                 : out std_logic;

        -- RX Identifier is valid
        rec_ivld                : out std_logic;

        -- Store Metadata in RX Buffer
        store_metadata          : out std_logic;

        -- Abort storing of frame in RX Buffer. Revert to last frame.
        rec_abort               : out std_logic;

        -- Store data word to RX Buffer.
        store_data              : out std_logic;

        -- Data words to be stored to RX Buffer.
        store_data_word         : out std_logic_vector(31 downto 0);

        -- Pulse in Start of Frame
        sof_pulse               : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Operation control FSM Interface
        -------------------------------------------------------------------------------------------
        -- Unit is transmitter
        is_transmitter          : in  std_logic;

        -- Unit is receiver
        is_receiver             : in  std_logic;

        -- Loss of arbitration -> Turn receiver!
        arbitration_lost        : out std_logic;

        -- Set unit to be transmitter (in SOF)
        set_transmitter         : out std_logic;

        -- Set unit to be receiver
        set_receiver            : out std_logic;

        -- Set unit to be idle
        set_idle                : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Fault confinement Interface
        -------------------------------------------------------------------------------------------
        -- Unit is error active
        is_err_active           : in  std_logic;

        -- Unit is error passive
        is_err_passive          : in  std_logic;

        -- Unit is Bus-off
        is_bus_off              : in  std_logic;

        -- Error detected
        err_detected            : out std_logic;

        -- Primary Error
        primary_err             : out std_logic;

        -- Active Error or Overload flag is being tranmsmitted
        act_err_ovr_flag        : out std_logic;

        -- Error delimiter too late
        err_delim_late          : out std_logic;

        -- Set unit to be error active
        set_err_active          : out  std_logic;

        -- Error counters should remain unchanged
        err_ctrs_unchanged      : out  std_logic;

        -------------------------------------------------------------------------------------------
        -- TX and RX Trigger signals to Sample and Transmitt Data
        -------------------------------------------------------------------------------------------
        -- TX Trigger (in SYNC segment)
        tx_trigger              : in  std_logic;

        -- RX Trigger (one clock cycle delayed after Sample point)
        rx_trigger              : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- CAN Bus serial data stream
        -------------------------------------------------------------------------------------------
        -- TX Data
        tx_data_nbs             : out std_logic;

        -- TX Data (post bit stuffing)
        tx_data_wbs             : in  std_logic;

        -- RX Data
        rx_data_nbs             : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Bit Stuffing Interface
        -------------------------------------------------------------------------------------------
        -- Bit Stuffing enabled
        stuff_enable            : out std_logic;

        -- Bit De-stuffing enabled
        destuff_enable          : out std_logic;

        -- Bit Stuffing type (0-Normal, 1-Fixed)
        fixed_stuff             : out std_logic;

        -- Frame transmission without SOF started
        tx_frame_no_sof         : out std_logic;

        -- Number of de-stuffed bits modulo 8
        dst_ctr                 : in  std_logic_vector(2 downto 0);

        -- Number of stuffed bits modulo 8
        bst_ctr                 : in  std_logic_vector(2 downto 0);

        -- Stuff Error
        stuff_err               : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Bus Sampling Interface
        -------------------------------------------------------------------------------------------
        -- Bit Error detected
        bit_err                 : in  std_logic;

        -- Reset Bit time measurement counter
        btmc_reset              : out  std_logic;

        -- Start Measurement of data bit time (in TX Trigger)
        dbt_measure_start       : out std_logic;

        -- First SSP generated (in ESI bit)
        gen_first_ssp           : out std_logic;

        -- Synchronization edge
        sync_edge               : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- CRC Interface
        -------------------------------------------------------------------------------------------
        -- Enable CRC calculation
        crc_enable              : out std_logic;

        -- CRC calculation - speculative enable
        crc_spec_enable         : out  std_logic;

        -- Use RX Data for CRC calculation
        crc_calc_from_rx        : out  std_logic;

        -- Load CRC Initialization vector
        load_init_vect          : out  std_logic;

        -- Calculated CRC 15
        crc_15                  : in  std_logic_vector(14 downto 0);

        -- Calculated CRC 17
        crc_17                  : in  std_logic_vector(16 downto 0);

        -- Calculated CRC 21
        crc_21                  : in  std_logic_vector(20 downto 0);

        -------------------------------------------------------------------------------------------
        -- Control signals
        -------------------------------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control              : out std_logic_vector(1 downto 0);

        -- Sample control (Registered)
        sp_control_q            : out std_logic_vector(1 downto 0);

        -- Enable Nominal Bit time counters.
        nbt_ctrs_en             : out  std_logic;

        -- Enable Data Bit time counters.
        dbt_ctrs_en             : out  std_logic;

        -- Synchronisation control (No synchronisation, Hard Synchronisation, Resynchronisation
        sync_control            : out std_logic_vector(1 downto 0);

        -- Clear the Shift register for secondary sampling point.
        ssp_reset               : out std_logic;

        -- Enable measurement of Transmitter delay
        tran_delay_meas         : out std_logic;

        -- Transmitted frame is valid
        tran_valid              : out std_logic;

        -- Received frame is valid
        rec_valid               : out std_logic;

        -- Decrement Receive Error counter
        decrement_rec           : out std_logic;

        -- Bit Error in passive error flag after ACK Error
        bit_err_after_ack_err   : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Status signals
        -------------------------------------------------------------------------------------------
        -- Bit rate shifted
        br_shifted              : out std_logic;

        -- Form Error has occurred
        form_err                : out std_logic;

        -- ACK Error has occurred
        ack_err                 : out std_logic;

        -- CRC Error has occurred
        crc_err                 : out std_logic;

        -- Status of retransmit counter (for observation purpose)
        retr_ctr                : out std_logic_vector(G_RETR_LIM_CTR_WIDTH - 1 downto 0)
    );
end entity;

architecture rtl of protocol_control is

    -- TXT Buffer word (endianity swapped)
    signal tran_word_swapped        :     std_logic_vector(31 downto 0);

    -- Error frame request
    signal err_frm_req              :     std_logic;

    -- Load commands for TX Shift register
    signal tx_load_base_id          :     std_logic;
    signal tx_load_ext_id           :     std_logic;
    signal tx_load_dlc              :     std_logic;
    signal tx_load_data_word        :     std_logic;
    signal tx_load_stuff_count      :     std_logic;
    signal tx_load_crc              :     std_logic;

    -- TX Shift register enabled
    signal tx_shift_ena             :     std_logic;

    -- Transmit dominant bit
    signal tx_dominant              :     std_logic;

    -- Clear all registers in RX Shift register
    signal rx_clear                :      std_logic;

    -- Store commands for RX Shift register
    signal rx_store_base_id        :     std_logic;
    signal rx_store_ext_id         :     std_logic;
    signal rx_store_ide            :     std_logic;
    signal rx_store_rtr            :     std_logic;
    signal rx_store_edl            :     std_logic;
    signal rx_store_dlc            :     std_logic;
    signal rx_store_esi            :     std_logic;
    signal rx_store_brs            :     std_logic;
    signal rx_store_stuff_count    :     std_logic;

    -- Clock Enable RX Shift register for each byte.
    signal rx_shift_ena            :     std_logic_vector(3 downto 0);

    -- Selector for inputs of each byte of shift register (0-Previous byte output, 1-RX Data input)
    signal rx_shift_in_sel         :     std_logic;

    -- RX value of Remote transmission request
    signal rec_is_rtr_i            :     std_logic;

    -- RX value of DLC (combinational), valid only in last bit of DLC
    signal rec_dlc_d               :     std_logic_vector(3 downto 0);

    -- RX value of DLC (captured)
    signal rec_dlc_q               :     std_logic_vector(3 downto 0);

    -- RX frame type (0-CAN 2.0, 1- CAN FD)
    signal rec_frame_type_i        :     std_logic;

    -- Preload control counter
    signal ctrl_ctr_pload          :      std_logic;

    -- Control counter preload value
    signal ctrl_ctr_pload_val      :      std_logic_vector(8 downto 0);

    -- Control counter is enabled
    signal ctrl_ctr_ena            :      std_logic;

    -- Control counter is zero
    signal ctrl_ctr_zero           :      std_logic;

    -- Control counter is equal to 1
    signal ctrl_ctr_one            :      std_logic;

    -- Control counter counted multiple of 8 bits
    signal ctrl_counted_byte       :      std_logic;

    -- Control counter byte index within a memory word
    signal ctrl_counted_byte_index :      std_logic_vector(1 downto 0);

    -- Control counter - TXT Buffer memory index
    signal ctrl_ctr_mem_index      :      std_logic_vector(4 downto 0);

    -- Complementary counter enable
    signal compl_ctr_ena           :      std_logic;

    -- Reintegration counter Clear (synchronous)
    signal reinteg_ctr_clr         :      std_logic;

    -- Enable counting (with RX Trigger)
    signal reinteg_ctr_enable      :      std_logic;

    -- Reintegration counter expired (reached 128)
    signal reinteg_ctr_expired     :      std_logic;

    -- Clear Retransmitt counter
    signal retr_ctr_clear          :      std_logic;

    -- Increment Retransmitt counter by 1
    signal retr_ctr_add            :      std_logic;

    -- Retransmitt limit was reached
    signal retr_limit_reached      :      std_logic;

    -- Form Error has occurred
    signal form_err_i              :      std_logic;

    -- ACK Error has occurred
    signal ack_err_i               :      std_logic;

    -- Perform CRC check
    signal crc_check               :      std_logic;

    -- Bit Error in arbitration field
    signal bit_err_arb             :      std_logic;

    -- Calculated CRC and Stuff count are matching received ones
    signal crc_match               :     std_logic;

    -- CRC error signalling
    signal crc_err_i               :     std_logic;

    -- Clear CRC Match flag
    signal crc_clear_match_flag    :      std_logic;

    -- CRC Source (CRC15, CRC17, CRC21)
    signal crc_src                 :      std_logic_vector(1 downto 0);

    -- Error position field (for Error capture)
    signal err_pos                 :      std_logic_vector(3 downto 0);

    -- Bit error detection enabled
    signal bit_err_enable          :      std_logic;

    -- Received CRC (driven from RX Shift register)
    signal rx_crc                  :      std_logic_vector(20 downto 0);

    -- RX Stuff count (grey coded) + RX parity
    signal rx_stuff_count          :      std_logic_vector(3 downto 0);

    -- Fixed Stuff (internal)
    signal fixed_stuff_i           :      std_logic;

    -- Arbitration lost (internal)
    signal arbitration_lost_i      :      std_logic;

    -- Part of arbitration currently transmitted
    signal arbitration_part        :      std_logic_vector(2 downto 0);

    -- Debug status of protocol controller
    signal pc_dbg_i                :      t_protocol_control_dbg;

begin

    -----------------------------------------------------------------------------------------------
    -- TX Data word endian swapper
    -----------------------------------------------------------------------------------------------
    endian_swapper_tx_inst : entity ctu_can_fd_rtl.endian_swapper
    generic map(
        G_SWAP_GEN              => true,
        G_WORD_SIZE             => 4,                       -- Number of Groups
        G_GROUP_SIZE            => 8                        -- Group size (bits)
    )
    port map(
        input                   => tran_word,               -- IN
        output                  => tran_word_swapped        -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- Protocol control FSM
    -----------------------------------------------------------------------------------------------
    protocol_control_fsm_inst : entity ctu_can_fd_rtl.protocol_control_fsm
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        -- Signals which cause state change
        rx_trigger              => rx_trigger,              -- IN
        err_frm_req             => err_frm_req,             -- IN

        -- Memory registers interface
        mr_settings_ena         => mr_settings_ena,         -- IN
        mr_settings_nisofd      => mr_settings_nisofd,      -- IN
        mr_command_ercrst       => mr_command_ercrst,       -- IN
        mr_mode_acf             => mr_mode_acf,             -- IN
        mr_mode_stm             => mr_mode_stm,             -- IN
        mr_mode_bmm             => mr_mode_bmm,             -- IN
        mr_settings_rtrle       => mr_settings_rtrle,       -- IN
        mr_settings_ilbp        => mr_settings_ilbp,        -- IN
        mr_mode_fde             => mr_mode_fde,             -- IN
        mr_ssp_cfg_ssp_src      => mr_ssp_cfg_ssp_src,      -- IN
        mr_settings_pex         => mr_settings_pex,         -- IN
        mr_command_cpexs        => mr_command_cpexs,        -- IN
        mr_mode_rom             => mr_mode_rom,             -- IN
        mr_mode_sam             => mr_mode_sam,             -- IN

        mr_status_pexs          => mr_status_pexs,          -- OUT

        pc_dbg                  => pc_dbg_i,                -- OUT

        -- Data-path interface
        tx_data_wbs             => tx_data_wbs,             -- IN
        rx_data_nbs             => rx_data_nbs,             -- IN

        -- RX Buffer interface
        store_metadata          => store_metadata,          -- OUT
        store_data              => store_data,              -- OUT
        rec_valid               => rec_valid,               -- OUT
        rec_abort               => rec_abort,               -- OUT
        sof_pulse               => sof_pulse,               -- OUT

        -- TXT Buffer, TX Arbitrator interface
        tran_frame_valid        => tran_frame_valid,        -- IN
        txtb_hw_cmd             => txtb_hw_cmd,             -- OUT
        txtb_ptr                => txtb_ptr,                -- OUT
        txtb_clk_en             => txtb_clk_en,             -- OUT
        tran_dlc                => tran_dlc,                -- IN
        tran_is_rtr             => tran_is_rtr,             -- IN
        tran_frame_type         => tran_frame_type,         -- IN
        tran_ident_type         => tran_ident_type,         -- IN
        tran_brs                => tran_brs,                -- IN

        -- TX Shift register interface
        tx_load_base_id         => tx_load_base_id,         -- OUT
        tx_load_ext_id          => tx_load_ext_id,          -- OUT
        tx_load_dlc             => tx_load_dlc,             -- OUT
        tx_load_data_word       => tx_load_data_word,       -- OUT
        tx_load_stuff_count     => tx_load_stuff_count,     -- OUT
        tx_load_crc             => tx_load_crc,             -- OUT
        tx_shift_ena            => tx_shift_ena,            -- OUT
        tx_dominant             => tx_dominant,             -- OUT

        -- RX Shift register interface
        rx_clear                => rx_clear,                -- OUT
        rx_store_base_id        => rx_store_base_id,        -- OUT
        rx_store_ext_id         => rx_store_ext_id,         -- OUT
        rx_store_ide            => rx_store_ide,            -- OUT
        rx_store_rtr            => rx_store_rtr,            -- OUT
        rx_store_edl            => rx_store_edl,            -- OUT
        rx_store_dlc            => rx_store_dlc,            -- OUT
        rx_store_esi            => rx_store_esi,            -- OUT
        rx_store_brs            => rx_store_brs,            -- OUT
        rx_store_stuff_count    => rx_store_stuff_count,    -- OUT
        rx_shift_ena            => rx_shift_ena,            -- OUT
        rx_shift_in_sel         => rx_shift_in_sel,         -- OUT
        rec_is_rtr              => rec_is_rtr_i,            -- IN
        rec_dlc_d               => rec_dlc_d,               -- IN
        rec_dlc_q               => rec_dlc_q,               -- IN
        rec_frame_type          => rec_frame_type_i,        -- IN
        rec_lbpf                => rec_lbpf,                -- OUT
        rec_ivld                => rec_ivld,                -- OUT

        -- Control counter interface
        ctrl_ctr_pload          => ctrl_ctr_pload,          -- OUT
        ctrl_ctr_pload_val      => ctrl_ctr_pload_val,      -- OUT
        ctrl_ctr_ena            => ctrl_ctr_ena,            -- OUT
        ctrl_ctr_zero           => ctrl_ctr_zero,           -- IN
        ctrl_ctr_one            => ctrl_ctr_one,            -- IN
        ctrl_counted_byte       => ctrl_counted_byte,       -- IN
        ctrl_counted_byte_index => ctrl_counted_byte_index, -- IN
        ctrl_ctr_mem_index      => ctrl_ctr_mem_index,      -- IN
        compl_ctr_ena           => compl_ctr_ena,           -- OUT
        arbitration_part        => arbitration_part,        -- OUT

        -- Reintegration counter interface
        reinteg_ctr_clr         => reinteg_ctr_clr,         -- OUT
        reinteg_ctr_enable      => reinteg_ctr_enable,      -- OUT
        reinteg_ctr_expired     => reinteg_ctr_expired,     -- IN

        -- Retransmitt counter interface
        retr_ctr_clear          => retr_ctr_clear,          -- OUT
        retr_ctr_add            => retr_ctr_add,            -- OUT
        retr_limit_reached      => retr_limit_reached,      -- IN

        -- Error detector interface
        form_err                => form_err_i,              -- OUT
        ack_err                 => ack_err_i,               -- OUT
        crc_check               => crc_check,               -- OUT
        bit_err_arb             => bit_err_arb,             -- OUT
        bit_err_after_ack_err   => bit_err_after_ack_err,   -- OUT
        crc_match               => crc_match,               -- IN
        crc_err                 => crc_err_i,               -- OUT
        crc_clear_match_flag    => crc_clear_match_flag,    -- OUT
        crc_src                 => crc_src,                 -- OUT
        err_pos                 => err_pos,                 -- OUT

        -- Bit Stuffing/Destuffing control signals
        stuff_enable            => stuff_enable,            -- OUT
        destuff_enable          => destuff_enable,          -- OUT
        fixed_stuff             => fixed_stuff_i,           -- OUT
        tx_frame_no_sof         => tx_frame_no_sof,         -- OUT

        -- Operation control interface
        is_transmitter          => is_transmitter,          -- IN
        is_receiver             => is_receiver,             -- IN
        arbitration_lost        => arbitration_lost_i,      -- OUT
        set_transmitter         => set_transmitter,         -- OUT
        set_receiver            => set_receiver,            -- OUT
        set_idle                => set_idle,                -- OUT

        -- Fault confinement interface
        primary_err             => primary_err,             -- OUT
        act_err_ovr_flag        => act_err_ovr_flag,        -- OUT
        set_err_active          => set_err_active,          -- OUT
        err_delim_late          => err_delim_late,          -- OUT
        is_err_active           => is_err_active,           -- IN
        is_err_passive          => is_err_passive,          -- IN
        is_bus_off              => is_bus_off,              -- IN
        decrement_rec           => decrement_rec,           -- OUT

        -- Other control signals
        sp_control              => sp_control,              -- OUT
        sp_control_q            => sp_control_q,            -- OUT
        nbt_ctrs_en             => nbt_ctrs_en,             -- OUT
        dbt_ctrs_en             => dbt_ctrs_en,             -- OUT
        sync_control            => sync_control,            -- OUT
        ssp_reset               => ssp_reset,               -- OUT
        tran_delay_meas         => tran_delay_meas,         -- OUT
        tran_valid              => tran_valid,              -- OUT
        crc_enable              => crc_enable,              -- OUT
        crc_spec_enable         => crc_spec_enable,         -- OUT
        crc_calc_from_rx        => crc_calc_from_rx,        -- OUT
        load_init_vect          => load_init_vect,          -- OUT
        bit_err_enable          => bit_err_enable,          -- OUT
        br_shifted              => br_shifted,              -- OUT
        btmc_reset              => btmc_reset,              -- OUT
        dbt_measure_start       => dbt_measure_start,       -- OUT
        gen_first_ssp           => gen_first_ssp,           -- OUT
        sync_edge               => sync_edge                -- IN
    );

    -----------------------------------------------------------------------------------------------
    -- Control counter
    -----------------------------------------------------------------------------------------------
    control_counter_inst : entity ctu_can_fd_rtl.control_counter
    generic map(
        G_CTRL_CTR_WIDTH        => G_CTRL_CTR_WIDTH
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        -- Control signals
        rx_trigger              => rx_trigger,              -- IN
        ctrl_ctr_ena            => ctrl_ctr_ena,            -- IN
        ctrl_ctr_pload          => ctrl_ctr_pload,          -- IN
        ctrl_ctr_pload_val      => ctrl_ctr_pload_val,      -- IN
        compl_ctr_ena           => compl_ctr_ena,           -- IN
        arbitration_lost        => arbitration_lost_i,      -- IN
        arbitration_part        => arbitration_part,        -- IN

        -- Status signals
        ctrl_ctr_zero           => ctrl_ctr_zero,           -- OUT
        ctrl_ctr_one            => ctrl_ctr_one,            -- OUT
        ctrl_counted_byte       => ctrl_counted_byte,       -- OUT
        ctrl_counted_byte_index => ctrl_counted_byte_index, -- OUT
        ctrl_ctr_mem_index      => ctrl_ctr_mem_index,      -- OUT

        -- Memory registers interface
        alc_alc_bit             => alc_alc_bit,             -- OUT
        alc_alc_id_field        => alc_alc_id_field         -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- Reintegration counter
    -----------------------------------------------------------------------------------------------
    reintegration_counter_inst : entity ctu_can_fd_rtl.reintegration_counter
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        -- Control signals
        reinteg_ctr_clr         => reinteg_ctr_clr,         -- IN
        reinteg_ctr_enable      => reinteg_ctr_enable,      -- IN
        rx_trigger              => rx_trigger,              -- IN

        -- Status signals
        reinteg_ctr_expired     => reinteg_ctr_expired      -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- Retransmitt counter
    -----------------------------------------------------------------------------------------------
    retransmitt_counter_inst : entity ctu_can_fd_rtl.retransmitt_counter
    generic map(
        G_RETR_LIM_CTR_WIDTH    => G_RETR_LIM_CTR_WIDTH
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        -- Control signals
        txtb_changed            => txtb_changed,            -- IN
        retr_ctr_clear          => retr_ctr_clear,          -- IN
        retr_ctr_add            => retr_ctr_add,            -- IN

        -- Memory registers interface
        mr_settings_rtrth       => mr_settings_rtrth,       -- IN

        -- Status signals
        retr_limit_reached      => retr_limit_reached,      -- OUT
        retr_ctr                => retr_ctr                 -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- Error detector
    -----------------------------------------------------------------------------------------------
    err_detector_inst : entity ctu_can_fd_rtl.err_detector
    generic map(
        G_ERR_VALID_PIPELINE    => G_ERR_VALID_PIPELINE
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        -- Data-path interface
        tx_data                 => tx_data_wbs,             -- IN
        rx_data                 => rx_data_nbs,             -- IN

        -- Error sources
        bit_err                 => bit_err,                 -- IN
        bit_err_arb             => bit_err_arb,             -- IN
        stuff_err               => stuff_err,               -- IN
        form_err                => form_err_i,              -- IN
        ack_err                 => ack_err_i,               -- IN
        crc_err                 => crc_err_i,               -- IN
        tran_frame_parity_error => tran_frame_parity_error, -- IN

        -- Memory registers interface
        mr_settings_nisofd      => mr_settings_nisofd,      -- IN
        err_capt_err_type       => err_capt_err_type,       -- OUT
        err_capt_err_pos        => err_capt_err_pos,        -- OUT
        err_capt_err_erp        => err_capt_err_erp,        -- OUT

        -- CRC comparison data
        rx_crc                  => rx_crc,                  -- IN
        crc_15                  => crc_15,                  -- IN
        crc_17                  => crc_17,                  -- IN
        crc_21                  => crc_21,                  -- IN
        rx_stuff_count          => rx_stuff_count,          -- IN
        dst_ctr                 => dst_ctr,                 -- IN

        -- Control signals
        bit_err_enable          => bit_err_enable,          -- IN
        fixed_stuff             => fixed_stuff_i,           -- IN
        err_pos                 => err_pos,                 -- IN
        crc_check               => crc_check,               -- IN
        crc_clear_match_flag    => crc_clear_match_flag,    -- IN
        crc_src                 => crc_src,                 -- IN
        is_arbitration          => pc_dbg_i.is_arbitration, -- IN
        is_transmitter          => is_transmitter,          -- IN
        is_err_passive          => is_err_passive,          -- IN

        -- Status output
        err_frm_req             => err_frm_req,             -- OUT
        err_detected            => err_detected,            -- OUT
        crc_match               => crc_match,               -- OUT
        err_ctrs_unchanged      => err_ctrs_unchanged       -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- TX Shift register
    -----------------------------------------------------------------------------------------------
    tx_shift_reg_inst : entity ctu_can_fd_rtl.tx_shift_reg
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        mr_mode_tstm            => mr_mode_tstm,            -- IN

        tx_trigger              => tx_trigger,              -- IN
        tx_data_nbs             => tx_data_nbs,             -- OUT

        -- Protocol control FSM interface
        tx_load_base_id         => tx_load_base_id,         -- IN
        tx_load_ext_id          => tx_load_ext_id,          -- IN
        tx_load_dlc             => tx_load_dlc,             -- IN
        tx_load_data_word       => tx_load_data_word,       -- IN
        tx_load_stuff_count     => tx_load_stuff_count,     -- IN
        tx_load_crc             => tx_load_crc,             -- IN
        tx_shift_ena            => tx_shift_ena,            -- IN
        tx_dominant             => tx_dominant,             -- IN
        crc_src                 => crc_src,                 -- IN

        -- CAN CRC Interface
        crc_15                  => crc_15,                  -- IN
        crc_17                  => crc_17,                  -- IN
        crc_21                  => crc_21,                  -- IN

        err_frm_req             => err_frm_req,             -- IN
        is_err_active           => is_err_active,           -- IN
        bst_ctr                 => bst_ctr,                 -- IN
        tran_identifier         => tran_identifier,         -- IN
        tran_frame_test         => tran_frame_test,         -- IN
        tran_word_swapped       => tran_word_swapped,       -- IN
        tran_dlc                => tran_dlc                 -- IN
    );

    -----------------------------------------------------------------------------------------------
    -- RX Shift register
    -----------------------------------------------------------------------------------------------
    rx_shift_reg_inst : entity ctu_can_fd_rtl.rx_shift_reg
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        -- DFT support
        scan_enable             => scan_enable,             -- IN

        rx_trigger              => rx_trigger,              -- IN

        -- Data-path interface
        rx_data_nbs             => rx_data_nbs,             -- IN

        -- Protocol control FSM interface
        rx_clear                => rx_clear,                -- IN
        rx_shift_ena            => rx_shift_ena,            -- IN
        rx_shift_in_sel         => rx_shift_in_sel,         -- IN
        rx_store_base_id        => rx_store_base_id,        -- IN
        rx_store_ext_id         => rx_store_ext_id,         -- IN
        rx_store_ide            => rx_store_ide,            -- IN
        rx_store_rtr            => rx_store_rtr,            -- IN
        rx_store_edl            => rx_store_edl,            -- IN
        rx_store_dlc            => rx_store_dlc,            -- IN
        rx_store_esi            => rx_store_esi,            -- IN
        rx_store_brs            => rx_store_brs,            -- IN
        rx_store_stuff_count    => rx_store_stuff_count,    -- IN

        -- RX Buffer interface
        rec_ident               => rec_ident,               -- OUT
        rec_dlc_d               => rec_dlc_d,               -- OUT
        rec_dlc                 => rec_dlc_q,               -- OUT
        rec_is_rtr              => rec_is_rtr_i,            -- OUT
        rec_ident_type          => rec_ident_type,          -- OUT
        rec_frame_type          => rec_frame_type_i,        -- OUT
        rec_brs                 => rec_brs,                 -- OUT
        rec_esi                 => rec_esi,                 -- OUT
        store_data_word         => store_data_word,         -- OUT

        -- CRC information for CRC comparison
        rx_crc                  => rx_crc,                  -- OUT
        rx_stuff_count          => rx_stuff_count           -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- Internal signals propagation to output
    -----------------------------------------------------------------------------------------------
    rec_frame_type <= rec_frame_type_i;
    rec_is_rtr <= rec_is_rtr_i;
    rec_dlc <= rec_dlc_q;
    form_err <= form_err_i;
    ack_err <= ack_err_i;
    crc_err <= crc_err_i;
    fixed_stuff <= fixed_stuff_i;
    arbitration_lost <= arbitration_lost_i;
    pc_dbg <= pc_dbg_i;


    -- <RELEASE_OFF>
    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- psl no_invalid_ack_err_asrt : assert never
    --  ((ack_err = '1' or crc_err = '1' or stuff_err = '1') and (pc_dbg_i.is_err = '1'))
    -- report "ACK, Stuff, CRC Errors can't occur during Error or overload flag";

    -- psl sample_sec_proper_asrt : assert never
    --  (sp_control = SECONDARY_SAMPLE and is_transmitter = '0')
    --  report "Secondary sampling is allowed only for transmitter!";

    -- psl no_simul_tx_rx_trigger_asrt : assert never
    --  (tx_trigger = '1' and rx_trigger = '1')
    --  report "RX Trigger and TX Trigger can't be active at once!";

    -- psl no_simul_transmitter_receiver_asrt : assert never
    --  (is_transmitter = '1' and is_receiver = '1')
    --  report "Unit can't be transmitter and receiver simultaneously!";

    -- psl no_h_sync_in_data_bit_rate_asrt : assert always
    --  (sync_control = HARD_SYNC) -> (sp_control = NOMINAL_SAMPLE)
    --  report "Hard synchronisation shall be used in Nominal bit rate only!";

    -- psl no_simul_err_req_asrt : assert never
    --  (tran_valid = '1' and err_frm_req = '1')
    -- report "Tranmission OK and Error frame request can't occur at once!";

    -- <RELEASE_ON>

end architecture;