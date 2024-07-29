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
--  CAN Core
--
-- Sub-modules:
--   1. Protocol control
--   2. Bit stuffing
--   3. Bit destuffing
--   4. Fault confinement
--   5. CAN CRC
--   6. Operation control
--
--  Note:
--   Status bus assignments are implemented in this module.
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

entity can_core is
    generic (
        -- Number of signals in Sample trigger
        G_SAMPLE_TRIGGER_COUNT  :     natural range 2 to 8;

        -- Control counter width
        G_CTRL_CTR_WIDTH        :     natural;

        -- Retransmitt limit counter width
        G_RETR_LIM_CTR_WIDTH    :     natural;

        -- Insert pipeline on "error_valid"
        G_ERR_VALID_PIPELINE    :     boolean;

        -- CRC 15 polynomial
        G_CRC15_POL             :     std_logic_vector(15 downto 0);

        -- CRC 17 polynomial
        G_CRC17_POL             :     std_logic_vector(19 downto 0);

        -- CRC 15 polynomial
        G_CRC21_POL             :     std_logic_vector(23 downto 0);

        -- Support traffic counters
        G_SUP_TRAFFIC_CTRS      :     boolean
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
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

        mr_settings_ena         : in  std_logic;
        mr_settings_nisofd      : in  std_logic;
        mr_settings_rtrth       : in  std_logic_vector(3 downto 0);
        mr_settings_rtrle       : in  std_logic;
        mr_settings_ilbp        : in  std_logic;
        mr_settings_pex         : in  std_logic;

        mr_command_ercrst       : in  std_logic;
        mr_command_rxfcrst      : in  std_logic;
        mr_command_txfcrst      : in  std_logic;
        mr_command_cpexs        : in  std_logic;

        mr_ssp_cfg_ssp_src      : in  std_logic_vector(1 downto 0);

        mr_ewl_ew_limit         : in  std_logic_vector(7 downto 0);
        mr_erp_erp_limit        : in  std_logic_vector(7 downto 0);

        mr_ctr_pres_ctpv        : in  std_logic_vector(8 downto 0);
        mr_ctr_pres_ptx         : in  std_logic;
        mr_ctr_pres_prx         : in  std_logic;
        mr_ctr_pres_enorm       : in  std_logic;
        mr_ctr_pres_efd         : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Status signals
        -------------------------------------------------------------------------------------------
        cc_stat                 : out t_can_core_stat;
        pc_dbg                  : out t_protocol_control_dbg;

        -------------------------------------------------------------------------------------------
        -- Tx Arbitrator and TXT Buffers interface
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

        -- TX Bit Rate Shift
        tran_brs                : in  std_logic;

        -- TX Identifier
        tran_identifier         : in  std_logic_vector(28 downto 0);

        -- TX frame test word
        tran_frame_test         : in  t_frame_test_w;

        -- Frame in TXT Buffer is valid any can be transmitted.
        tran_frame_valid        : in  std_logic;

        -- Parity Error occurred in TXT Buffer RAMs during transmission of data words
        tran_frame_parity_error : in  std_logic;

        -- HW Commands for TX Arbitrator and TXT Buffers
        txtb_hw_cmd             : out t_txtb_hw_cmd;

        -- Selected TXT Buffer index changed
        txtb_changed            : in  std_logic;

        -- Pointer to TXT buffer memory
        txtb_ptr                : out natural range 0 to 20;

        -- Clock enable for TXT Buffer memory
        txtb_clk_en             : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Recieve Buffer and Message Filter Interface
        -------------------------------------------------------------------------------------------
        -- RX CAN Identifier
        rec_ident               : out std_logic_vector(28 downto 0);

        -- RX Data length code
        rec_dlc                 : out std_logic_vector(3 downto 0);

        -- RX Recieved identifier type (0-BASE Format, 1-Extended Format);
        rec_ident_type          : out std_logic;

        -- RX frame type (0-CAN 2.0, 1- CAN FD)
        rec_frame_type          : out std_logic;

        -- Received Loopback frame
        rec_lbpf                : out std_logic;

        -- RX Remote transmission request Flag
        rec_is_rtr              : out std_logic;

        -- RX Bit Rate Shift bit
        rec_brs                 : out std_logic;

        -- RX Error state indicator
        rec_esi      	        : out std_logic;

        -- RX Identifier is valid
        rec_ivld                : out std_logic;

        -- RX Frame received succesfully, can be commited to RX Buffer.
        rec_valid               : out std_logic;

        -- Metadata are received OK, and can be stored in RX Buffer.
        store_metadata          : out std_logic;

        -- Store data word to RX Buffer.
        store_data              : out std_logic;

        -- Data words to be stored to RX Buffer.
        store_data_word         : out std_logic_vector(31 downto 0);

        -- Abort storing of frame in RX Buffer. Revert to last frame.
        rec_abort               : out std_logic;

        -- Pulse in Start of Frame
        sof_pulse               : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Interrupt Manager Interface
        -------------------------------------------------------------------------------------------
        -- Arbitration was lost
        arbitration_lost        : out std_logic;

        -- Frame stored in CAN Core was sucessfully transmitted
        tran_valid              : out std_logic;

        -- Bit Rate Was Shifted
        br_shifted              : out std_logic;

        -- Error is detected (Error frame will be transmitted)
        err_detected            : out std_logic;

        -- Fault confinement state changed
        fcs_changed             : out std_logic;

        -- Error warning limit reached
        err_warning_limit_pulse : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Prescaler interface
        -------------------------------------------------------------------------------------------
        -- RX Triggers (in Sample Point)
        rx_triggers             : in  std_logic_vector(G_SAMPLE_TRIGGER_COUNT - 1 downto 0);

        -- TX Trigger
        tx_trigger              : in  std_logic;

        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation
        sync_control            : out std_logic_vector(1 downto 0);

        -- No positive resynchronisation
        no_pos_resync           : out std_logic;

        -- Sample control (Nominal, Data, Secondary)
        sp_control              : out std_logic_vector(1 downto 0);

        -- Enable Nominal Bit time counters.
        nbt_ctrs_en             : out std_logic;

        -- Enable Data Bit time counters.
        dbt_ctrs_en             : out std_logic;

        -------------------------------------------------------------------------------------------
        -- CAN Bus serial data stream
        -------------------------------------------------------------------------------------------
        -- RX Data from CAN Bus
        rx_data_wbs             : in  std_logic;

        -- TX Data to CAN Bus
        tx_data_wbs             : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Others
        -------------------------------------------------------------------------------------------
        -- Secondary sample point reset
        ssp_reset               : out std_logic;

        -- Enable measurement of Transmitter delay
        tran_delay_meas         : out std_logic;

        -- Bit Error detected
        bit_err                 : in  std_logic;

        -- Reset Bit time measurement counter
        btmc_reset              : out std_logic;

        -- Start Measurement of data bit time (in TX Trigger)
        dbt_measure_start       : out std_logic;

        -- First SSP generated (in ESI bit)
        gen_first_ssp           : out std_logic;

        -- Synchronization edge
        sync_edge               : in  std_logic;

        -- RX Trigger of Protocol control FSM (sample point)
        pc_rx_trigger           : out std_logic
    );
end entity;

architecture rtl of can_core is

    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Internal signals
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------

    -- TXT Buffer control
    signal txtb_hw_cmd_i            : t_txtb_hw_cmd;

    -- Received frame
    signal rec_ident_i              : std_logic_vector(28 downto 0);
    signal rec_dlc_i                : std_logic_vector(3 downto 0);
    signal rec_ident_type_i         : std_logic;
    signal rec_frame_type_i         : std_logic;
    signal rec_lbpf_i               : std_logic;
    signal rec_is_rtr_i             : std_logic;
    signal rec_brs_i                : std_logic;
    signal rec_esi_i                : std_logic;
    signal rec_ivld_i               : std_logic;

    -- Arbitration lost capture
    signal alc_alc_bit              : std_logic_vector(4 downto 0);
    signal alc_alc_id_field         : std_logic_vector(2 downto 0);

    -- Error code capture
    signal err_capt_err_type        : std_logic_vector(2 downto 0);
    signal err_capt_err_pos         : std_logic_vector(3 downto 0);
    signal err_capt_err_erp         : std_logic;

    -- Operation control interface
    signal is_transmitter           : std_logic;
    signal is_receiver              : std_logic;
    signal is_idle                  : std_logic;
    signal arbitration_lost_i       : std_logic;
    signal set_transmitter          : std_logic;
    signal set_receiver             : std_logic;
    signal set_idle                 : std_logic;

    -- Fault confinement Interface
    signal is_err_active            : std_logic;
    signal is_err_passive           : std_logic;
    signal is_bus_off_i             : std_logic;
    signal err_detected_i           : std_logic;
    signal primary_err              : std_logic;
    signal act_err_ovr_flag         : std_logic;
    signal err_delim_late           : std_logic;
    signal set_err_active           : std_logic;
    signal err_ctrs_unchanged       : std_logic;

    -- Bit Stuffing Interface
    signal stuff_enable             : std_logic;
    signal destuff_enable           : std_logic;
    signal fixed_stuff              : std_logic;
    signal tx_frame_no_sof          : std_logic;
    signal dst_ctr                  : std_logic_vector(2 downto 0);
    signal bst_ctr                  : std_logic_vector(2 downto 0);
    signal stuff_err                : std_logic;

    -- CRC Interface
    signal crc_enable               : std_logic;
    signal crc_spec_enable          : std_logic;
    signal crc_calc_from_rx         : std_logic;
    signal crc_15                   : std_logic_vector(14 downto 0);
    signal crc_17                   : std_logic_vector(16 downto 0);
    signal crc_21                   : std_logic_vector(20 downto 0);

    -- Protocol control - control outputs
    signal sp_control_i             : std_logic_vector(1 downto 0);
    signal sp_control_q             : std_logic_vector(1 downto 0);
    signal sync_control_i           : std_logic_vector(1 downto 0);
    signal ssp_reset_i              : std_logic;
    signal tran_delay_meas_i        : std_logic;
    signal tran_valid_i             : std_logic;
    signal rec_valid_i              : std_logic;
    signal br_shifted_i             : std_logic;

    -- Fault confinement status signals
    signal fcs_changed_i            : std_logic;

    signal tx_err_ctr               : std_logic_vector(8 downto 0);
    signal rx_err_ctr               : std_logic_vector(8 downto 0);
    signal norm_err_ctr             : std_logic_vector(15 downto 0);
    signal data_err_ctr             : std_logic_vector(15 downto 0);

    -- Protocol control triggers
    signal pc_tx_trigger            : std_logic;
    signal pc_rx_trigger_i          : std_logic;

    -- Protocol control data inputs/outputs
    signal pc_tx_data_nbs           : std_logic;
    signal pc_rx_data_nbs           : std_logic;

    -- CRC Data inputs
    signal crc_data_tx_wbs          : std_logic;
    signal crc_data_tx_nbs          : std_logic;
    signal crc_data_rx_wbs          : std_logic;
    signal crc_data_rx_nbs          : std_logic;

    -- CRC Trigger inputs
    signal crc_trig_tx_wbs          : std_logic;
    signal crc_trig_tx_nbs          : std_logic;
    signal crc_trig_rx_wbs          : std_logic;
    signal crc_trig_rx_nbs          : std_logic;

    -- Bit stuffing signals
    signal bst_data_in              : std_logic;
    signal bst_data_out             : std_logic;
    signal bst_trigger              : std_logic;
    signal data_halt                : std_logic;

    -- Bit destuffing signals
    signal bds_data_in              : std_logic;
    signal bds_data_out             : std_logic;
    signal bds_trigger              : std_logic;
    signal destuffed                : std_logic;

    -- Bus traffic counters
    signal tx_frame_ctr             : std_logic_vector(31 downto 0);
    signal rx_frame_ctr             : std_logic_vector(31 downto 0);

    signal tx_data_wbs_i            : std_logic;

    -- Looped back data for bus monitoring mode
    signal lpb_dominant             : std_logic;

    -- Error indication
    signal form_err                 : std_logic;
    signal ack_err                  : std_logic;
    signal crc_err                  : std_logic;

    -- Start of frame indication
    signal sof_pulse_i              : std_logic;

    signal load_init_vect           : std_logic;
    signal retr_ctr                 : std_logic_vector(G_RETR_LIM_CTR_WIDTH - 1 downto 0);

    -- Decrement Receive Error counter
    signal decrement_rec            : std_logic;

    -- Bit Error in passive error flag following ACK error!
    signal bit_err_after_ack_err    : std_logic;

    -- Protocol exception status
    signal mr_status_pexs           : std_logic;

    -- Error warning limit status
    signal mr_status_ewl            : std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- Protocol control
    -----------------------------------------------------------------------------------------------
    protocol_control_inst : entity ctu_can_fd_rtl.protocol_control
    generic map (
        G_CTRL_CTR_WIDTH        => G_CTRL_CTR_WIDTH,
        G_RETR_LIM_CTR_WIDTH    => G_RETR_LIM_CTR_WIDTH,
        G_ERR_VALID_PIPELINE    => G_ERR_VALID_PIPELINE
    )
    port map (
        clk_sys                 => clk_sys,                     -- IN
        res_n                   => res_n,                       -- IN

        -- DFT support
        scan_enable             => scan_enable,                 -- IN

        -- Memory registers interface
        mr_mode_acf             => mr_mode_acf,                 -- IN
        mr_mode_stm             => mr_mode_stm,                 -- IN
        mr_mode_bmm             => mr_mode_bmm,                 -- IN
        mr_mode_fde             => mr_mode_fde,                 -- IN
        mr_mode_rom             => mr_mode_rom,                 -- IN
        mr_mode_tstm            => mr_mode_tstm,                -- IN
        mr_settings_ena         => mr_settings_ena,             -- IN
        mr_settings_nisofd      => mr_settings_nisofd,          -- IN
        mr_settings_rtrth       => mr_settings_rtrth,           -- IN
        mr_settings_rtrle       => mr_settings_rtrle,           -- IN
        mr_settings_ilbp        => mr_settings_ilbp,            -- IN
        mr_settings_pex         => mr_settings_pex,             -- IN
        mr_command_ercrst       => mr_command_ercrst,           -- IN
        mr_command_cpexs        => mr_command_cpexs,            -- IN
        mr_ssp_cfg_ssp_src      => mr_ssp_cfg_ssp_src,          -- IN

        alc_alc_bit             => alc_alc_bit,                 -- OUT
        alc_alc_id_field        => alc_alc_id_field,            -- OUT

        err_capt_err_type       => err_capt_err_type,           -- OUT
        err_capt_err_pos        => err_capt_err_pos,            -- OUT
        err_capt_err_erp        => err_capt_err_erp,            -- OUT

        pc_dbg                  => pc_dbg,                      -- OUT
        mr_status_pexs          => mr_status_pexs,              -- OUT

        -- TXT Buffers interface
        tran_word               => tran_word,                   -- IN
        tran_dlc                => tran_dlc,                    -- IN
        tran_is_rtr             => tran_is_rtr,                 -- IN
        tran_ident_type         => tran_ident_type,             -- IN
        tran_frame_type         => tran_frame_type,             -- IN
        tran_brs                => tran_brs,                    -- IN
        tran_identifier         => tran_identifier,             -- IN
        tran_frame_test         => tran_frame_test,             -- IN
        tran_frame_valid        => tran_frame_valid,            -- IN
        tran_frame_parity_error => tran_frame_parity_error,     -- IN
        txtb_hw_cmd             => txtb_hw_cmd_i,               -- IN
        txtb_ptr                => txtb_ptr,                    -- OUT
        txtb_clk_en             => txtb_clk_en,                 -- OUT
        txtb_changed            => txtb_changed,                -- IN

        -- RX Buffer interface
        rec_ident               => rec_ident_i,                 -- OUT
        rec_dlc                 => rec_dlc_i,                   -- OUT
        rec_is_rtr              => rec_is_rtr_i,                -- OUT
        rec_ident_type          => rec_ident_type_i,            -- OUT
        rec_frame_type          => rec_frame_type_i,            -- OUT
        rec_lbpf                => rec_lbpf_i,                  -- OUT
        rec_brs                 => rec_brs_i,                   -- OUT
        rec_esi                 => rec_esi_i,                   -- OUT
        rec_ivld                => rec_ivld_i,                  -- OUT
        store_metadata          => store_metadata,              -- OUT
        rec_abort               => rec_abort,                   -- OUT
        store_data              => store_data,                  -- OUT
        store_data_word         => store_data_word,             -- OUT
        sof_pulse               => sof_pulse_i,                 -- OUT

        -- Operation control FSM Interface
        is_transmitter          => is_transmitter,              -- IN
        is_receiver             => is_receiver,                 -- IN
        arbitration_lost        => arbitration_lost_i,          -- OUT
        set_transmitter         => set_transmitter,             -- OUT
        set_receiver            => set_receiver,                -- OUT
        set_idle                => set_idle,                    -- OUT

        -- Fault confinement Interface
        is_err_active           => is_err_active,               -- IN
        is_err_passive          => is_err_passive,              -- IN
        is_bus_off              => is_bus_off_i,                -- IN
        err_detected            => err_detected_i,              -- OUT
        primary_err             => primary_err,                 -- OUT
        act_err_ovr_flag        => act_err_ovr_flag,            -- OUT
        err_delim_late          => err_delim_late,              -- OUT
        set_err_active          => set_err_active,              -- OUT
        err_ctrs_unchanged      => err_ctrs_unchanged,          -- OUT

        -- TX and RX Trigger signals to Sample and Transmitt Data
        tx_trigger              => pc_tx_trigger,               -- IN
        rx_trigger              => pc_rx_trigger_i,             -- IN

        -- CAN Bus serial data stream
        tx_data_nbs             => pc_tx_data_nbs,              -- OUT
        tx_data_wbs             => tx_data_wbs_i,               -- IN
        rx_data_nbs             => pc_rx_data_nbs,              -- IN

        -- Bit Stuffing Interface
        stuff_enable            => stuff_enable,                -- OUT
        destuff_enable          => destuff_enable,              -- OUT
        fixed_stuff             => fixed_stuff,                 -- OUT
        tx_frame_no_sof         => tx_frame_no_sof,             -- OUT
        dst_ctr                 => dst_ctr,                     -- IN
        bst_ctr                 => bst_ctr,                     -- IN
        stuff_err               => stuff_err,                   -- IN

        -- Bus Sampling Interface
        bit_err                 => bit_err,                     -- IN
        btmc_reset              => btmc_reset,                  -- OUT
        dbt_measure_start       => dbt_measure_start,           -- OUT
        gen_first_ssp           => gen_first_ssp,               -- OUT
        sync_edge               => sync_edge,                   -- IN

        -- CRC Interface
        crc_enable              => crc_enable,                  -- OUT
        crc_spec_enable         => crc_spec_enable,             -- OUT
        crc_calc_from_rx        => crc_calc_from_rx,            -- OUT
        load_init_vect          => load_init_vect,              -- OUT
        crc_15                  => crc_15,                      -- IN
        crc_17                  => crc_17,                      -- IN
        crc_21                  => crc_21,                      -- IN

        -- Control signals
        sp_control              => sp_control_i,                -- OUT
        sp_control_q            => sp_control_q,                -- OUT
        nbt_ctrs_en             => nbt_ctrs_en,                 -- OUT
        dbt_ctrs_en             => dbt_ctrs_en,                 -- OUT
        sync_control            => sync_control_i,              -- OUT
        ssp_reset               => ssp_reset_i,                 -- OUT
        tran_delay_meas         => tran_delay_meas_i,           -- OUT
        tran_valid              => tran_valid_i,                -- OUT
        rec_valid               => rec_valid_i,                 -- OUT
        decrement_rec           => decrement_rec,               -- OUT
        bit_err_after_ack_err   => bit_err_after_ack_err,       -- OUT

        -- Status signals
        br_shifted              => br_shifted_i,                -- OUT
        form_err                => form_err,                    -- OUT
        ack_err                 => ack_err,                     -- OUT
        crc_err                 => crc_err,                     -- OUT
        retr_ctr                => retr_ctr                     -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- Operation control FSM
    -----------------------------------------------------------------------------------------------
    operation_control_inst : entity ctu_can_fd_rtl.operation_control
    port map (
        clk_sys                 => clk_sys,                     -- IN
        res_n                   => res_n,                       -- IN

        -- Prescaler Interface
        rx_trigger              => pc_rx_trigger_i,             -- IN

        -- Fault confinement Interface
        is_bus_off              => is_bus_off_i,                -- IN

        -- Protocol Control Interface
        arbitration_lost        => arbitration_lost_i,          -- IN
        set_transmitter         => set_transmitter,             -- IN
        set_receiver            => set_receiver,                -- IN
        set_idle                => set_idle,                    -- IN
        is_transmitter          => is_transmitter,              -- OUT
        is_receiver             => is_receiver,                 -- OUT
        is_idle                 => is_idle                      -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- Fault confinement
    -----------------------------------------------------------------------------------------------
    fault_confinement_inst : entity ctu_can_fd_rtl.fault_confinement
    port map (
        clk_sys                 => clk_sys,                     -- IN
        res_n                   => res_n,                       -- IN

        -- DFT support
        scan_enable             => scan_enable,                 -- IN

        mr_mode_rom             => mr_mode_rom,                 -- IN
        mr_ewl_ew_limit         => mr_ewl_ew_limit,             -- IN
        mr_erp_erp_limit        => mr_erp_erp_limit,            -- IN
        mr_ctr_pres_ctpv        => mr_ctr_pres_ctpv,            -- IN
        mr_ctr_pres_ptx         => mr_ctr_pres_ptx,             -- IN
        mr_ctr_pres_prx         => mr_ctr_pres_prx,             -- IN
        mr_ctr_pres_enorm       => mr_ctr_pres_enorm,           -- IN
        mr_ctr_pres_efd         => mr_ctr_pres_efd,             -- IN
        mr_status_ewl           => mr_status_ewl,               -- OUT

        -- Error signalling for interrupts
        fcs_changed             => fcs_changed_i,               -- OUT
        err_warning_limit_pulse => err_warning_limit_pulse,     -- OUT

        -- Operation control Interface
        is_transmitter          => is_transmitter,              -- IN
        is_receiver             => is_receiver,                 -- IN

        -- Protocol control Interface
        sp_control              => sp_control_i,                -- IN
        set_err_active          => set_err_active,              -- IN
        err_detected            => err_detected_i,              -- IN
        err_ctrs_unchanged      => err_ctrs_unchanged,          -- IN
        primary_err             => primary_err,                 -- IN
        act_err_ovr_flag        => act_err_ovr_flag,            -- IN
        err_delim_late          => err_delim_late,              -- IN
        tran_valid              => tran_valid_i,                -- IN
        decrement_rec           => decrement_rec,               -- IN
        bit_err_after_ack_err   => bit_err_after_ack_err,       -- IN

        -- Fault confinement State indication
        is_err_active           => is_err_active,               -- OUT
        is_err_passive          => is_err_passive,              -- OUT
        is_bus_off              => is_bus_off_i,                -- OUT

        -- Error counters
        tx_err_ctr              => tx_err_ctr,                  -- OUT
        rx_err_ctr              => rx_err_ctr,                  -- OUT
        norm_err_ctr            => norm_err_ctr,                -- OUT
        data_err_ctr            => data_err_ctr                 -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- CAN CRC
    -----------------------------------------------------------------------------------------------
    can_crc_inst : entity ctu_can_fd_rtl.can_crc
    generic map (
        G_CRC15_POL             => G_CRC15_POL,
        G_CRC17_POL             => G_CRC17_POL,
        G_CRC21_POL             => G_CRC21_POL
    )
    port map (
        clk_sys                 => clk_sys,                     -- IN
        res_n                   => res_n,                       -- IN

        -- Memory registers interface
        mr_settings_nisofd      => mr_settings_nisofd,          -- IN

        -- Data inputs for CRC calculation
        data_tx_wbs             => crc_data_tx_wbs,             -- IN
        data_tx_nbs             => crc_data_tx_nbs,             -- IN
        data_rx_wbs             => crc_data_rx_wbs,             -- IN
        data_rx_nbs             => crc_data_rx_nbs,             -- IN

        -- Trigger signals to process the data on each CRC input.
        trig_tx_wbs             => crc_trig_tx_wbs,             -- IN
        trig_tx_nbs             => crc_trig_tx_nbs,             -- IN
        trig_rx_wbs             => crc_trig_rx_wbs,             -- IN
        trig_rx_nbs             => crc_trig_rx_nbs,             -- IN

        -- Control signals
        crc_enable              => crc_enable,                  -- IN
        crc_spec_enable         => crc_spec_enable,             -- IN
        crc_calc_from_rx        => crc_calc_from_rx,            -- IN
        load_init_vect          => load_init_vect,              -- IN

        -- CRC Outputs
        crc_15                  => crc_15,                      -- OUT
        crc_17                  => crc_17,                      -- OUT
        crc_21                  => crc_21                       -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- Bit Stuffing
    -----------------------------------------------------------------------------------------------
    bit_stuffing_inst : entity ctu_can_fd_rtl.bit_stuffing
    port map (
        clk_sys                 => clk_sys,                     -- IN
        res_n                   => res_n,                       -- IN

        -- Data-path
        data_in                 => bst_data_in,                 -- IN
        data_out                => bst_data_out,                -- OUT

        -- Control signals
        bst_trigger             => bst_trigger,                 -- IN
        stuff_enable            => stuff_enable,                -- IN
        fixed_stuff             => fixed_stuff,                 -- IN
        tx_frame_no_sof         => tx_frame_no_sof,             -- IN

        -- Status signals
        bst_ctr                 => bst_ctr,                     -- OUT
        data_halt               => data_halt                    -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- Bit Destuffing
    -----------------------------------------------------------------------------------------------
    bit_destuffing_inst : entity ctu_can_fd_rtl.bit_destuffing
    port map (
        clk_sys                 => clk_sys,                     -- IN
        res_n                   => res_n,                       -- IN

        -- Data-path
        data_in                 => bds_data_in,                 -- IN
        data_out                => bds_data_out,                -- OUT

        -- Control signals
        bds_trigger             => bds_trigger,                 -- IN
        destuff_enable          => destuff_enable,              -- IN
        fixed_stuff             => fixed_stuff,                 -- IN

        -- Status Outpus
        stuff_err               => stuff_err,                   -- OUT
        destuffed               => destuffed,                   -- OUT
        dst_ctr                 => dst_ctr                      -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- Bus traffic counters
    -----------------------------------------------------------------------------------------------
    bus_traffic_ctrs_gen : if (G_SUP_TRAFFIC_CTRS = true) generate

        bus_traffic_counters_inst : entity ctu_can_fd_rtl.bus_traffic_counters
        port map (
            clk_sys             => clk_sys,                     -- IN
            res_n               => res_n,                       -- IN
            scan_enable         => scan_enable,                 -- IN

            -- Memory registers interface
            mr_command_rxfcrst  => mr_command_rxfcrst,          -- IN
            mr_command_txfcrst  => mr_command_txfcrst,          -- IN

            tran_valid          => tran_valid_i,                -- IN
            rec_valid           => rec_valid_i,                 -- IN

            -- Counter outputs
            tx_frame_ctr        => tx_frame_ctr,                -- OUT
            rx_frame_ctr        => rx_frame_ctr                 -- OUT
        );

    end generate bus_traffic_ctrs_gen;

    no_bus_traffic_ctrs_gen : if (G_SUP_TRAFFIC_CTRS = false) generate
        tx_frame_ctr <= (others => '0');
        rx_frame_ctr <= (others => '0');
    end generate;

    -----------------------------------------------------------------------------------------------
    -- Trigger multiplexor
    -----------------------------------------------------------------------------------------------
    trigger_mux_inst : entity ctu_can_fd_rtl.trigger_mux
    generic map (
        G_SAMPLE_TRIGGER_COUNT  => G_SAMPLE_TRIGGER_COUNT
    )
    port map (
        -- Clock and Asynchronous reset
        clk_sys                => clk_sys,                      -- IN
        res_n                  => res_n,                        -- IN

        -- Input triggers
        rx_triggers            => rx_triggers,                  -- IN
        tx_trigger             => tx_trigger,                   -- IN

        -- Control signals
        data_halt              => data_halt,                    -- IN
        destuffed              => destuffed,                    -- IN
        fixed_stuff            => fixed_stuff,                  -- IN
        bds_data_in            => bds_data_in,                  -- IN

        -- Output triggers
        pc_tx_trigger          => pc_tx_trigger,                -- OUT
        pc_rx_trigger          => pc_rx_trigger_i,              -- OUT
        bst_trigger            => bst_trigger,                  -- OUT
        bds_trigger            => bds_trigger,                  -- OUT
        crc_trig_rx_nbs        => crc_trig_rx_nbs,              -- OUT
        crc_trig_tx_nbs        => crc_trig_tx_nbs,              -- OUT
        crc_trig_rx_wbs        => crc_trig_rx_wbs,              -- OUT
        crc_trig_tx_wbs        => crc_trig_tx_wbs,              -- OUT

        -- Status signals
        crc_data_rx_wbs        => crc_data_rx_wbs               -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Datapath connection
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------

    -----------------------------------------------------------------------------------------------
    -- Protocol control datapath connection:
    --  1. RX Data - Output of bit destuffing.
    --  2. TX Data - Input to bit stuffing.
    -----------------------------------------------------------------------------------------------
    pc_rx_data_nbs <= bds_data_out;
    bst_data_in <= pc_tx_data_nbs;

    -----------------------------------------------------------------------------------------------
    -- CRC 15 (No bit stuffing) data inputs:
    --  1. TX Data from Protocol control.
    --  2. RX Data after bit destuffing.
    -----------------------------------------------------------------------------------------------
    crc_data_tx_nbs <= pc_tx_data_nbs;
    crc_data_rx_nbs <= bds_data_out;

    -----------------------------------------------------------------------------------------------
    -- CRC 17,21 (With bit stuffing) data inputs:
    --  1. TX Data after Bit stuffing.
    --  2. RX Data before Bit destuffing.
    -----------------------------------------------------------------------------------------------
    crc_data_tx_wbs <= bst_data_out;

    lpb_dominant <= rx_data_wbs and bst_data_out;

    -----------------------------------------------------------------------------------------------
    -- Bit Stuffing data input:
    --  1. Bit Destuffing output for secondary sampling. This-way core will automatically receive
    --     what it transmitts without loop over Transceiver. Bit Error is detected by Bus sampling
    --     properly.
    --  2. Looped back dominant Bit for Bus monitoring Mode.
    --  3. Regular RX Data
    -----------------------------------------------------------------------------------------------
    bds_data_in <= bst_data_out when (sp_control_q = SECONDARY_SAMPLE) else
                   lpb_dominant when (mr_mode_bmm = '1') else
                    rx_data_wbs;

    -----------------------------------------------------------------------------------------------
    -- In Bus monitoring mode or when core is disabled, transmitted data to the bus are only
    -- recessive. Otherwise transmitted data are stuffed data!
    -----------------------------------------------------------------------------------------------
    tx_data_wbs_i <= RECESSIVE when (mr_settings_ena = CTU_CAN_DISABLED) else
                     RECESSIVE when (mr_mode_bmm = '1') else
                     bst_data_out;

    -----------------------------------------------------------------------------------------------
    -- Node transmitting dominant bit does shall not re-synchronize as a result of dominant
    -- transmitted bit.
    -----------------------------------------------------------------------------------------------
    no_pos_resync <= '1' when (tx_data_wbs_i = DOMINANT) else
                     '0';

    -----------------------------------------------------------------------------------------------
    -- CAN Core status record connections
    -----------------------------------------------------------------------------------------------
    cc_stat.is_err_active   <= is_err_active;
    cc_stat.is_err_passive  <= is_err_passive;
    cc_stat.is_bus_off      <= is_bus_off_i;
    cc_stat.is_transmitter  <= is_transmitter;
    cc_stat.is_receiver     <= is_receiver;
    cc_stat.is_idle         <= is_idle;
    cc_stat.tx_err_ctr      <= tx_err_ctr;
    cc_stat.rx_err_ctr      <= rx_err_ctr;
    cc_stat.status_pexs     <= mr_status_pexs;
    cc_stat.norm_err_ctr    <= norm_err_ctr;
    cc_stat.data_err_ctr    <= data_err_ctr;
    cc_stat.err_type        <= err_capt_err_type;
    cc_stat.err_erp         <= err_capt_err_erp;
    cc_stat.err_pos         <= err_capt_err_pos;
    cc_stat.retr_ctr        <= retr_ctr;
    cc_stat.alc_bit         <= alc_alc_bit;
    cc_stat.alc_id_field    <= alc_alc_id_field;
    cc_stat.rx_frame_ctr    <= rx_frame_ctr;
    cc_stat.tx_frame_ctr    <= tx_frame_ctr;
    cc_stat.bst_ctr         <= bst_ctr;
    cc_stat.dst_ctr         <= dst_ctr;
    cc_stat.status_ewl      <= mr_status_ewl;

    -----------------------------------------------------------------------------------------------
    -- Internal signals to output propagation
    -----------------------------------------------------------------------------------------------
    txtb_hw_cmd             <= txtb_hw_cmd_i;
    rec_ident               <= rec_ident_i;
    rec_dlc                 <= rec_dlc_i;
    rec_ident_type          <= rec_ident_type_i;
    rec_frame_type          <= rec_frame_type_i;
    rec_lbpf                <= rec_lbpf_i;
    rec_is_rtr              <= rec_is_rtr_i;
    rec_brs                 <= rec_brs_i;
    rec_esi                 <= rec_esi_i;
    rec_ivld                <= rec_ivld_i;
    rec_valid               <= rec_valid_i;
    arbitration_lost        <= arbitration_lost_i;
    tran_valid              <= tran_valid_i;
    br_shifted              <= br_shifted_i;
    err_detected            <= err_detected_i;
    fcs_changed             <= fcs_changed_i;
    sync_control            <= sync_control_i;
    tx_data_wbs             <= tx_data_wbs_i;
    sp_control              <= sp_control_i;
    ssp_reset               <= ssp_reset_i;
    tran_delay_meas         <= tran_delay_meas_i;
    sof_pulse               <= sof_pulse_i;

    -- Test signals observation
    pc_rx_trigger           <= pc_rx_trigger_i;

    -- <RELEASE_OFF>
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl no_stuff_bit_in_error_frame_1_asrt : assert never
    --  (data_halt = '1' and pc_dbg.is_err = '1')
    --  report "Stuff bits not allowed in Error frame!";

    -- Note: In following assertion, we can't check at the same clock cycle
    --       because Data halt will be cleared one clock cycle later than
    --       Error frame transmission starts!

    -- psl no_stuff_bit_in_error_frame_2_asrt : assert never
    --  ({pc_dbg.is_err = '1'; pc_dbg.is_err = '1' and destuffed = '1'})
    --  report "Stuff bits not allowed in Error frame!";

    -- psl no_stuff_bit_in_overload_frame_asrt : assert never
    --  ((destuffed = '1' or data_halt = '1') and pc_dbg.is_overload = '1')
    --  report "Stuff bits not allowed in Overload frame!";

    -- psl no_stuff_bit_in_eof_asrt : assert never
    --  ((destuffed = '1' or data_halt = '1') and pc_dbg.is_eof = '1')
    --  report "Stuff bits not allowed in End of frame!";

    -- psl no_stuff_bit_in_intermission_asrt : assert never
    --  ((destuffed = '1' or data_halt = '1') and pc_dbg.is_intermission = '1')
    --  report "Stuff bits not allowed in Intermission!";

    -- psl no_stuff_bit_in_idle_asrt : assert never
    --  ((destuffed = '1' or data_halt = '1') and is_idle = '1' and mr_mode_rom = '0')
    --  report "Stuff bits not allowed in Bus idle!";

    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Functional coverage
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------

    -- Transmitted frame combinations (no RTR)

    -- psl tx_base_id_can_2_0_cov : cover
    --  {tran_ident_type = BASE and tran_frame_type = NORMAL_CAN and tran_is_rtr = '0'};

    -- psl tx_extended_id_can_2_0_cov : cover
    --  {tran_ident_type = EXTENDED and tran_frame_type = NORMAL_CAN and tran_is_rtr = '0'};

    -- psl tx_base_id_can_fd_cov : cover
    --  {tran_ident_type = BASE and tran_frame_type = FD_CAN and tran_is_rtr = '0'};

    -- psl tx_extended_id_can_fd_cov : cover
    --  {tran_ident_type = EXTENDED and tran_frame_type = FD_CAN and tran_is_rtr = '0'};

    -- RTR frames (in combination with FD_CAN, this is ignored!)

    -- psl tx_base_id_can_2_0_rtr_cov : cover
    --  {tran_ident_type = BASE and tran_frame_type = NORMAL_CAN and tran_is_rtr = '1'};

    -- psl tx_extended_id_can_2_0_rtr_cov : cover
    --  {tran_ident_type = EXTENDED and tran_frame_type = NORMAL_CAN and tran_is_rtr = '1'};

    -- psl tx_base_id_can_fd_rtr_cov : cover
    --  {tran_ident_type = BASE and tran_frame_type = FD_CAN and tran_is_rtr = '1'};

    -- psl tx_extended_id_can_fd_rtr_cov : cover
    --  {tran_ident_type = EXTENDED and tran_frame_type = FD_CAN and tran_is_rtr = '1'};

    -- <RELEASE_ON>

end architecture;