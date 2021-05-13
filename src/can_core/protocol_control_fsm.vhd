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
--  Protocol control FSM.
--
-- Purpose:
--  State machine handling CAN FD protocol according to CAN FD 1.0 and ISO CAN
--  FD (ISO 11898-1 2015) standard.
--  Processes RX Data in "Process" pipeline stage with "rx_trigger". This
--  corresponds to moment one clock cycle behind sample point. In the same clock
--  cycle loads TX Shift register. FSM communicates with following modules:
--    1. TXT Buffers (HW Commands)
--    2. TX Arbitrator (HW Commands)
--    3. RX Buffer (Storing protocol)
--    4. TX Shift Register (Load)
--    5. RX Shift Register (Store commands)
--    6. Error detector (Error enabling, CRC check command)
--    7. Prescaler (Sample control, Synchronisation control)
--    8. Bus sampling (Transceiver Delay measurement)
--    9. Bit Stuffing (Stuffing method, Length of Stuff rule, Data halt)
--   10. Bit De-Stuffing (Stuffing method, Length of Stuff rule, Destuffed bit)
--   11. CAN CRC (Enable, speculative enable, Calculated CRC sequence)
--   12. Control, Retransmitt, Re-integration counters.
--   13. Fault confinement (Fault confinement protocol as in ISO 11898-1 2015.
--   14. Operation control (Setting transmitter/receiver idle).
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity protocol_control_fsm is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;
        
        -- Asynchronous reset
        res_n                   :in   std_logic;
        
        -----------------------------------------------------------------------
        -- Signals which cause state change
        -----------------------------------------------------------------------
        -- RX Trigger
        rx_trigger              :in   std_logic;

        -- Error frame request
        err_frm_req             :in   std_logic;

        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- CTU CAN FD is enabled
        drv_ena                 :in   std_logic;
        
        -- CAN FD type (ISO / NON-ISO)
        drv_fd_type             :in   std_logic;
        
        -- Command to start re-integration in Bus-off
        drv_bus_off_reset       :in   std_logic;
        
        -- Forbidding acknowledge mode
        drv_ack_forb            :in   std_logic;
        
        -- Self Test Mode enabled
        drv_self_test_ena       :in   std_logic;

        -- Bus Monitoring mode enabled
        drv_bus_mon_ena         :in   std_logic;
        
        -- Retransmition limit enabled for errornous frames
        drv_retr_lim_ena        :in   std_logic;
        
        -- Internal Loopback enabled
        drv_int_loopback_ena    :in   std_logic;
        
        -- Reception of CAN FD Frames is enabled
        drv_can_fd_ena          :in   std_logic;
        
        -- Secondary sampling point delay select
        drv_ssp_delay_select    :in   std_logic_vector(1 downto 0);
        
        -- Protocol exception handling
        drv_pex                 :in   std_logic;
        
        -- Protocol exception status clear
        drv_cpexs               :in   std_logic;
        
        -- ROM mode enabled
        drv_rom_ena             :in   std_logic;
        
        -- Control field is being transmitted
        is_control              :out  std_logic;

        -- Data field is being transmitted
        is_data                 :out  std_logic;

        -- Stuff Count field is being transmitted
        is_stuff_count          :out  std_logic;

        -- CRC field is being transmitted
        is_crc                  :out  std_logic;
        
        -- CRC Delimiter is being transmitted
        is_crc_delim            :out  std_logic;
        
        -- ACK field is being transmitted
        is_ack_field            :out  std_logic;
        
        -- ACK Delimiter is being transmitted
        is_ack_delim            :out  std_logic;
        
        -- End of Frame field is being transmitted
        is_eof                  :out  std_logic;
        
        -- Intermission is being transmitted
        is_intermission         :out  std_logic;
        
        -- Suspend transmission is being transmitted
        is_suspend              :out  std_logic;

        -- Error frame is being transmitted
        is_err_frm              :out  std_logic;
        
        -- Overload frame is being transmitted
        is_overload             :out  std_logic;
        
        -- Start of Frame
        is_sof                  :out  std_logic;
        
        -- Protocol exception status
        is_pexs                 :out  std_logic;

        -----------------------------------------------------------------------
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual TX Data (post bit stuffing)
        tx_data_wbs             :in   std_logic;
        
        -- Actual RX Data
        rx_data_nbs             :in   std_logic;
        
        -----------------------------------------------------------------------
        -- RX Buffer interface
        -----------------------------------------------------------------------
        -- Command to store CAN frame metadata to RX Buffer
        store_metadata          :out  std_logic;

        -- Command to store word of CAN Data
        store_data              :out  std_logic;
        
        -- Received frame valid
        rec_valid               :out  std_logic;
        
        -- Command to abort storing of RX frame (due to Error frame)
        rec_abort               :out  std_logic;
        
        -- Start of Frame pulse
        sof_pulse               :out  std_logic;

        -----------------------------------------------------------------------
        -- TXT Buffer, TX Arbitrator interface
        -----------------------------------------------------------------------
        -- There is a valid frame for transmission
        tran_frame_valid        :in   std_logic;
        
        -- HW Commands to TXT Buffers
        txtb_hw_cmd             :out  t_txtb_hw_cmd;
        
        -- Pointer to TXT Buffer memory
        txtb_ptr                :out  natural range 0 to 19;
        
        -- Clock enable for TXT Buffer memory
        txtb_clk_en             :out  std_logic;
        
        -- TX Data length code
        tran_dlc                :in   std_logic_vector(3 downto 0);
        
        -- TX Remote transmission request flag
        tran_is_rtr             :in   std_logic;
        
        -- TX Frame type (0-CAN 2.0, 1-CAN FD)
        tran_frame_type         :in   std_logic;
        
        -- Identifier type (BASIC, EXTENDED)
        tran_ident_type         :in   std_logic;

        -- TX Bit rate shift
        tran_brs                :in   std_logic;
                
        -----------------------------------------------------------------------
        -- TX Shift register interface
        -----------------------------------------------------------------------
        -- Load Base Identifier to TX Shift register
        tx_load_base_id         :out  std_logic;

        -- Load extended Identifier to TX Shift register
        tx_load_ext_id          :out  std_logic;

        -- Load DLC
        tx_load_dlc             :out  std_logic;

        -- Load Data word to TX Shift register
        tx_load_data_word       :out  std_logic;
        
        -- Load Stuff count
        tx_load_stuff_count     :out  std_logic;

        -- Load CRC to TX Shift register
        tx_load_crc             :out  std_logic;

        -- Shift register enable (shifts with TX Trigger)
        tx_shift_ena            :out  std_logic;

        -- Force Dominant value instead of value from shift register
        tx_dominant             :out  std_logic;
        
        -----------------------------------------------------------------------
        -- RX Shift register interface
        -----------------------------------------------------------------------
        -- Clear all registers in RX Shift register
        rx_clear                :out  std_logic;
        
        -- Store Base Identifier 
        rx_store_base_id        :out  std_logic;
        
        -- Store Extended Identifier
        rx_store_ext_id         :out  std_logic;
        
        -- Store Identifier extension
        rx_store_ide            :out  std_logic;
        
        -- Store Remote transmission request
        rx_store_rtr            :out  std_logic;
        
        -- Store EDL (FDF) bit
        rx_store_edl            :out  std_logic;
        
        -- Store DLC
        rx_store_dlc            :out  std_logic;
        
        -- Store ESI
        rx_store_esi            :out  std_logic;
        
        -- Store BRS
        rx_store_brs            :out  std_logic;
        
        -- Store stuff count and Stuff Count parity
        rx_store_stuff_count    :out  std_logic;
        
        -- Clock Enable RX Shift register for each byte.
        rx_shift_ena            :out  std_logic_vector(3 downto 0);
        
        -- Selector for inputs of each byte of shift register
        -- (0-Previous byte output, 1- RX Data input)
        rx_shift_in_sel         :out  std_logic;
        
        -- RX value of Remote transmission request
        rec_is_rtr              :in   std_logic;

        -- RX value of DLC (combinational), valid only in last bit of DLC
        rec_dlc_d               :in   std_logic_vector(3 downto 0);
        
        -- RX value of DLC (captured)
        rec_dlc_q               :in   std_logic_vector(3 downto 0);
        
        -- RX frame type (0-CAN 2.0, 1- CAN FD)
        rec_frame_type          :in   std_logic;

        -----------------------------------------------------------------------
        -- Control counter interface
        -----------------------------------------------------------------------
        -- Preload control counter
        ctrl_ctr_pload          :out   std_logic;
        
        -- Control counter preload value
        ctrl_ctr_pload_val      :out   std_logic_vector(8 downto 0);
        
        -- Control counter is enabled
        ctrl_ctr_ena            :out   std_logic;
        
        -- Control counter is zero
        ctrl_ctr_zero           :in    std_logic;
        
        -- Control counter is equal to 1
        ctrl_ctr_one            :in    std_logic;

        -- Control counter counted multiple of 8 bits
        ctrl_counted_byte       :in    std_logic;

        -- Control counter byte index within a memory word
        ctrl_counted_byte_index :in    std_logic_vector(1 downto 0);
        
        -- Control counter - TXT Buffer memory index
        ctrl_ctr_mem_index      :in    std_logic_vector(4 downto 0);
        
        -- Complementary counter enable
        compl_ctr_ena           :out   std_logic;
        
        -- Arbitration lost capture ID field
        alc_id_field            :out   std_logic_vector(2 downto 0);

        -----------------------------------------------------------------------
        -- Reintegration counter interface
        -----------------------------------------------------------------------
        -- Reintegration counter Clear (synchronous)
        reinteg_ctr_clr         :out   std_logic;

        -- Enable counting (with RX Trigger)
        reinteg_ctr_enable      :out   std_logic;
        
        -- Reintegration counter expired (reached 128)
        reinteg_ctr_expired     :in    std_logic;

        -----------------------------------------------------------------------
        -- Retransmitt counter interface
        -----------------------------------------------------------------------
        -- Clear Retransmitt counter
        retr_ctr_clear          :out   std_logic;

        -- Increment Retransmitt counter by 1
        retr_ctr_add            :out   std_logic;

        -- Retransmitt limit was reached
        retr_limit_reached      :in    std_logic;

        -----------------------------------------------------------------------
        -- Error detector interface
        -----------------------------------------------------------------------
        -- Form Error has occurred
        form_err                :out   std_logic;

        -- ACK Error has occurred
        ack_err                 :out   std_logic;

        -- Perform CRC check
        crc_check               :out   std_logic;
        
        -- Bit Error in arbitration field
        bit_err_arb             :out   std_logic;
        
        -- Calculated CRC and Stuff count are matching received ones
        crc_match               :in    std_logic;

        -- CRC error signalling
        crc_err                 :out   std_logic;

        -- Clear CRC Match flag
        crc_clear_match_flag    :out   std_logic;

        -- CRC Source (CRC15, CRC17, CRC21)
        crc_src                 :out   std_logic_vector(1 downto 0);
        
        -- Error position field (for Error capture)
        err_pos                 :out   std_logic_vector(4 downto 0);
        
        -- Arbitration field is being transmitted / received
        is_arbitration          :out   std_logic;
        
        -----------------------------------------------------------------------
        -- Bit Stuffing/Destuffing control signals
        -----------------------------------------------------------------------
        -- Bit Stuffing is enabled
        stuff_enable            :out   std_logic;
        
        -- Bit De-stuffing is enabled
        destuff_enable          :out   std_logic;

        -- Length of Bit stuffing rule
        stuff_length            :out   std_logic_vector(2 downto 0);
        
        -- Fixed Bit stuffing method
        fixed_stuff             :out   std_logic;
        
        -- Frame transmission without SOF started
        tx_frame_no_sof         :out   std_logic;
        
        -----------------------------------------------------------------------
        -- Operation control interface
        -----------------------------------------------------------------------
        -- Unit is transmitter
        is_transmitter          :in   std_logic;
        
        -- Unit is receiver
        is_receiver             :in   std_logic;

        -- Loss of arbitration -> Turn receiver!
        arbitration_lost        :out  std_logic;

        -- Set unit to be transmitter (in SOF)
        set_transmitter         :out  std_logic;

        -- Set unit to be receiver
        set_receiver            :out  std_logic;

        -- Set unit to be idle
        set_idle                :out  std_logic;

        -----------------------------------------------------------------------
        -- Fault confinement interface
        -----------------------------------------------------------------------
        -- Primary Error
        primary_err             :out  std_logic;
        
        -- Active Error or Overload flag is being tranmsmitted
        act_err_ovr_flag        :out  std_logic;

        -- Set unit to be error active
        set_err_active          :out   std_logic;

        -- Error delimiter too late
        err_delim_late          :out  std_logic;

        -- Unit is error active
        is_err_active           :in   std_logic;
        
        -- Unit is error passive
        is_err_passive          :in   std_logic;
        
        -- Unit is Bus off
        is_bus_off              :in   std_logic;
        
        -- Decrement REC (by 1)
        decrement_rec           :out  std_logic;
        
        -- Bit Error in passive error flag after ACK Error
        bit_err_after_ack_err   :out   std_logic;

        -----------------------------------------------------------------------
        -- Other control signals
        -----------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control              :out   std_logic_vector(1 downto 0);
        
        -- Sample control (Registered)
        sp_control_q            :out   std_logic_vector(1 downto 0);
        
        -- Enable Nominal Bit time counters.
        nbt_ctrs_en             :out   std_logic;
        
        -- Enable Data Bit time counters.
        dbt_ctrs_en             :out   std_logic;

        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation)
        sync_control            :out   std_logic_vector(1 downto 0);

        -- Clear the Shift register for secondary sampling point.
        ssp_reset               :out   std_logic;

        -- Enable measurement of Transmitter delay
        tran_delay_meas         :out   std_logic;

        -- Protocol control FSM state output
        pc_state                :out   t_protocol_control_state;

        -- Transmitted frame is valid
        tran_valid              :out   std_logic;

        -- CRC calculation enabled
        crc_enable              :out   std_logic;
        
        -- CRC calculation - speculative enable
        crc_spec_enable         :out   std_logic;
        
        -- Use RX Data for CRC calculation
        crc_calc_from_rx        :out   std_logic;

        -- Load CRC Initialization vector
        load_init_vect          :out   std_logic;

        -- Bit error enable
        bit_err_enable          :out   std_logic;

        -- Bit rate shifted
        br_shifted              :out   std_logic;
        
        -- Reset Bit time measurement counter
        btmc_reset              :out   std_logic;
    
        -- Start Measurement of data bit time (in TX Trigger)
        dbt_measure_start       :out  std_logic;
    
        -- First SSP generated (in ESI bit)
        gen_first_ssp           :out  std_logic;
        
        -- Synchronization edge
        sync_edge               :in   std_logic
    );
end entity;

architecture rtl of protocol_control_fsm is

    ---------------------------------------------------------------------------
    -- FSM related signals
    ---------------------------------------------------------------------------
    -- Protocol control FSM
    signal curr_state               :   t_protocol_control_state;
    signal next_state               :   t_protocol_control_state;

    -- Clock enable for state register
    signal state_reg_ce             :   std_logic;
    
    ---------------------------------------------------------------------------
    -- Internal combinational signals
    ---------------------------------------------------------------------------
    -- No data field should be transmitted
    signal no_data_transmitter      :   std_logic;
    signal no_data_receiver         :   std_logic;
    signal no_data_field            :   std_logic;

    -- Preload control counter internal signal
    signal ctrl_ctr_pload_i          :  std_logic;
    
    -- Used when control counter should be preloaded elsewhere than in sample
    -- point. This is used during integration to reset control counter upon
    -- synchronisation edge!
    signal ctrl_ctr_pload_unaliged   :  std_logic;
    
    -- CRC Selection
    signal crc_use_21                :  std_logic;
    signal crc_use_17                :  std_logic;
    signal crc_src_i                 :  std_logic_vector(1 downto 0);
    signal crc_length_i              :  std_logic_vector(8 downto 0);

    -- Length of data field (decoded from DLC, does not take RTR into account)
    signal tran_data_length          :  std_logic_vector(6 downto 0);
    signal rec_data_length           :  std_logic_vector(6 downto 0);
    signal rec_data_length_c         :  std_logic_vector(6 downto 0);

    signal data_length_c             :  std_logic_vector(6 downto 0);
    signal data_length_shifted_c     :  std_logic_vector(9 downto 0);
    signal data_length_sub_c         :  unsigned(9 downto 0);
    signal data_length_bits_c        :  std_logic_vector(8 downto 0);
    
    -- FD Frame is being transmitted/received
    signal is_fd_frame               :  std_logic;
    
    -- Frame transmission/reception can be started from idle or intermission!
    signal frame_start               :  std_logic;
    
    -- There is TX Frame ready for transmission
    signal tx_frame_ready            :  std_logic;
    
    -- IDE bit is part of arbitration
    signal ide_is_arbitration        :  std_logic;
    
    -- Arbitration lost condition
    signal arbitration_lost_condition : std_logic;

    -- Loss of arbitration -> Turn receiver!
    signal arbitration_lost_i        :  std_logic;
    
    -- Transmission failed (due to reached number of retransmissions), or
    -- first error, arb lost when there are 0 retransmissions allowed!
    signal tx_failed                 :  std_logic;
    
    -- Internal commands for RX Buffer
    signal store_metadata_d          :  std_logic;
    signal store_data_d              :  std_logic;
    signal rec_valid_d               :  std_logic;
    signal rec_abort_d               :  std_logic;

    -- Internal commands for TXT Buffers
    signal txtb_hw_cmd_d             :  t_txtb_hw_cmd;
    signal txtb_hw_cmd_q             :  t_txtb_hw_cmd;
    
    -- Unit should go to suspend transmission field!
    signal go_to_suspend             :  std_logic;
    
    -- Unit should go to stuff count field
    signal go_to_stuff_count         :  std_logic;
    
    -- Internal store commands for RX Shift register
    signal rx_store_base_id_i        :  std_logic;
    signal rx_store_ext_id_i         :  std_logic;
    signal rx_store_ide_i            :  std_logic;
    signal rx_store_rtr_i            :  std_logic;
    signal rx_store_edl_i            :  std_logic;
    signal rx_store_dlc_i            :  std_logic;
    signal rx_store_esi_i            :  std_logic;
    signal rx_store_brs_i            :  std_logic;
    signal rx_store_stuff_count_i    :  std_logic;
    
    signal rx_clear_i                :  std_logic;

    -- Internal commands for TX Shift register
    signal tx_load_base_id_i         :  std_logic;
    signal tx_load_ext_id_i          :  std_logic;
    signal tx_load_dlc_i             :  std_logic;
    signal tx_load_data_word_i       :  std_logic;
    signal tx_load_stuff_count_i     :  std_logic;
    signal tx_load_crc_i             :  std_logic;

    signal tx_shift_ena_i            :  std_logic;

    -- Internal signals for detected errors
    signal form_err_i                :  std_logic;
    signal ack_err_i                 :  std_logic;
    signal ack_err_flag              :  std_logic; 
    signal ack_err_flag_clr          :  std_logic;
    signal crc_err_i                 :  std_logic;
    signal bit_err_arb_i             :  std_logic;

    -- Sample control (Bit Rate) signals
    signal sp_control_switch_data    :  std_logic;
    signal sp_control_switch_nominal :  std_logic;
    
    -- Secondary sampling point is used
    signal switch_to_ssp             :  std_logic;
    
    signal sp_control_ce             :  std_logic;
    signal sp_control_d              :  std_logic_vector(1 downto 0);
    signal sp_control_q_i            :  std_logic_vector(1 downto 0);

    -- Secondary sampling point shift register reset
    signal ssp_reset_i               :  std_logic;
    
    -- Synchronisation control
    signal sync_control_d            :  std_logic_vector(1 downto 0);
    signal sync_control_q            :  std_logic_vector(1 downto 0);
    
    -- Hard synchronisation should be performed
    signal perform_hsync             :  std_logic;
    
    -- Fault confinemnt interface
    signal primary_err_i             :  std_logic;
    signal err_delim_late_i          :  std_logic;
    signal set_err_active_i          :  std_logic;
    
    -- Operation state handling internal
    signal set_transmitter_i         :  std_logic;
    signal set_receiver_i            :  std_logic;
    signal set_idle_i                :  std_logic;
    
    -- Flag which holds whether FSM is in first bit of error delimiter 
    signal first_err_delim_d         :  std_logic;
    signal first_err_delim_q         :  std_logic;
    
    -- Bit stuffing 
    signal stuff_enable_set          :  std_logic;
    signal stuff_enable_clear        :  std_logic;
    
    -- Bit stuffing disable
    signal destuff_enable_set        :  std_logic;
    signal destuff_enable_clear      :  std_logic;

    -- Bit error disable (internal)
    -- Note: Bit Error is rather disabled than enabled, since it is disabled on less
    -- places than enabled!
    signal bit_err_disable           :  std_logic;
    
    -- Bit Error is disabled for receiver in most of the frame!
    signal bit_err_disable_receiver  :  std_logic;
    
    -- TXT Buffer pointer
    signal txtb_ptr_d                :  natural range 0 to 19;
    signal txtb_ptr_q                :  natural range 0 to 19;
    
    -- Start of frame pulse
    signal sof_pulse_i               :  std_logic;
    
    -- Complementary counter enable
    signal compl_ctr_ena_i           :  std_logic;
    
    -- Logic for clocking FSM state register
    signal tick_state_reg_on_off     :  std_logic;
    signal tick_state_reg            :  std_logic;
    
    -- Bit-rate shifted (internal value)
    signal br_shifted_i              :  std_logic;
    
    -- Arbitration field is being transmitted / received
    signal is_arbitration_i          :  std_logic;
    
    -- CRC calculation - speculative enable
    signal crc_spec_enable_i         :  std_logic;
    
    -- CRC Load initialization vector - internal value
    signal load_init_vect_i          :  std_logic;
    
    -- Capture register to synchronize Bus off reset request till next Sample point
    signal drv_bus_off_reset_q       :  std_logic;
    
    -- Retransmitt counter clear (internal value)
    signal retr_ctr_clear_i          :  std_logic;
    -- Increment Retransmitt counter by 1
    signal retr_ctr_add_i            :  std_logic;

    -- Decrement Receive error counter (internal value)
    signal decrement_rec_i           :  std_logic;

    -- Blocking register for retransmitt counter add signal.
    signal retr_ctr_add_block        :  std_logic;
    signal retr_ctr_add_block_clr    :  std_logic;
    
    -- Blocking HW command for Unlock.
    signal block_txtb_unlock         :  std_logic;
    
    -- No SOF transmitted
    signal tx_frame_no_sof_d         :  std_logic;
    signal tx_frame_no_sof_q         :  std_logic;
    
    -- Control signal should be updated!
    signal ctrl_signal_upd           :  std_logic;
    
    -- Clear bus-off reset flag
    signal clr_bus_off_rst_flg       :  std_logic;
    
    -- Protocol exception signals
    signal pex_on_fdf_enable         :  std_logic;
    signal pex_on_res_enable         :  std_logic;
    
    -- Counting of consecutive bits during passive error flag
    signal rx_data_nbs_prev          :  std_logic;

    -- Protocol exception status
    signal pexs_set                  :  std_logic;
    
    -- Internal TX frame type
    signal tran_frame_type_i         :  std_logic;
    
    -- Clock enable for TXT Buffer RAM
    signal txtb_clk_en_d             :  std_logic;
    signal txtb_clk_en_q             :  std_logic;

begin

    tx_frame_ready <= '1' when (tran_frame_valid = '1' and
                                drv_bus_mon_ena = BMM_DISABLED and
                                drv_rom_ena = ROM_DISABLED)
                          else
                      '0';
                      
    tran_frame_type_i <= FD_CAN when (tran_frame_type = FD_CAN and drv_can_fd_ena = '1')
                                else
                         NORMAL_CAN;

    no_data_transmitter <= '1' when (tran_dlc = "0000" or 
                                    (tran_is_rtr = RTR_FRAME and tran_frame_type_i = NORMAL_CAN))
                               else
                           '0';

    no_data_receiver <= '1' when (rec_is_rtr = RTR_FRAME or rec_dlc_d = "0000")
                            else
                        '0';

    no_data_field <= '1' when (is_transmitter = '1' and no_data_transmitter = '1')
                         else
                     '1' when (is_receiver = '1' and no_data_receiver = '1')
                         else
                     '0';

    go_to_suspend <= '1' when (is_err_passive = '1' and is_transmitter = '1')
                         else
                     '0';

    ide_is_arbitration <= '1' when (tran_ident_type = EXTENDED or is_receiver = '1')
                              else
                          '0';

    arbitration_lost_condition <= '1' when (is_transmitter = '1' and 
                                            tx_data_wbs = RECESSIVE and
                                            rx_data_nbs = DOMINANT and
                                            rx_trigger = '1')
                                      else
                                  '0';

    tx_failed <= '1' when (drv_retr_lim_ena = '1' and retr_limit_reached = '1')
                     else
                 '0';

    is_fd_frame <= '1' when (is_transmitter = '1' and tran_frame_type_i = FD_CAN)
                       else
                   '1' when (is_receiver = '1' and rec_frame_type = FD_CAN)
                       else
                   '0';

    go_to_stuff_count <= '1' when (drv_fd_type = ISO_FD and is_fd_frame = '1')
                             else
                         '0';

    frame_start <= '1' when (tx_frame_ready = '1' and go_to_suspend = '0') else
                   '1' when (rx_data_nbs = DOMINANT) else
                   '0';

    ---------------------------------------------------------------------------
    -- Signal is not decoded inside curr_state process, because it is sensitive
    -- to this signal! Done here to avoid loops!
    ---------------------------------------------------------------------------
    block_txtb_unlock <= '1' when (curr_state = s_pc_act_err_flag or
                                   curr_state = s_pc_pas_err_flag or
                                   curr_state = s_pc_err_delim_wait or
                                   curr_state = s_pc_err_delim or
                                   curr_state = s_pc_ovr_flag or
                                   curr_state = s_pc_ovr_delim_wait or
                                   curr_state = s_pc_ovr_delim)
                             else
                         '0';

    pex_on_fdf_enable <= '1' when (drv_can_fd_ena = FDE_DISABLE and
                                   drv_pex = PROTOCOL_EXCEPTION_ENABLED) 
                             else
                         '0';

    pex_on_res_enable <= '1' when (drv_can_fd_ena = FDE_ENABLE and
                                   drv_pex = PROTOCOL_EXCEPTION_ENABLED)
                             else
                         '0';

    ---------------------------------------------------------------------------
    -- CRC sequence selection
    ---------------------------------------------------------------------------
    crc_use_21 <= '1' when (is_transmitter = '1' and tran_frame_type_i = FD_CAN and
                            to_integer(unsigned(tran_data_length)) > 16)
                      else
                  '1' when (is_receiver = '1' and rec_frame_type = FD_CAN and
                            to_integer(unsigned(rec_data_length)) > 16)
                      else
                  '0';

    crc_use_17 <= '1' when (is_transmitter = '1' and tran_frame_type_i = FD_CAN and
                            crc_use_21 = '0')
                      else
                  '1' when (is_receiver = '1' and rec_frame_type = FD_CAN and
                            crc_use_21 = '0')
                      else
                  '0';
                            

    crc_src_i <= C_CRC21_SRC when (crc_use_21 = '1') else
                 C_CRC17_SRC when (crc_use_17 = '1') else
                 C_CRC15_SRC;

    crc_length_i <= C_CRC15_DURATION when (crc_src_i = C_CRC15_SRC) else
                    C_CRC17_DURATION when (crc_src_i = C_CRC17_SRC) else
                    C_CRC21_DURATION;

    ---------------------------------------------------------------------------
    -- DLC to Data length decoders
    ---------------------------------------------------------------------------
    dlc_decoder_tx_inst : dlc_decoder
    port map(
        dlc           => tran_dlc,
        frame_type    => tran_frame_type_i,

        data_length   => tran_data_length,
        is_valid      => open
    );

    dlc_decoder_rx_inst : dlc_decoder
    port map(
        dlc           => rec_dlc_q,
        frame_type    => rec_frame_type,

        data_length   => rec_data_length,
        is_valid      => open
    );
    
    dlc_decoder_rx_inst_comb : dlc_decoder
    port map(
        dlc           => rec_dlc_d,
        frame_type    => rec_frame_type,

        data_length   => rec_data_length_c,
        is_valid      => open
    );

    -- Data field length (valid only in Sample point of last bit of DLC)
    data_length_c <= tran_data_length when (is_transmitter = '1') else
                     rec_data_length_c;
                     
    -- Shift by 3 (Multiply by 8)
    data_length_shifted_c <= data_length_c & "000";
    
    -- Subtract 1 (control counter counts till field length minus 1)
    data_length_sub_c <= unsigned(data_length_shifted_c) - 1;
    
    -- Convert to length of control counter
    data_length_bits_c <= std_logic_vector(
            data_length_sub_c(ctrl_ctr_pload_val'length - 1 downto 0));
    
    ---------------------------------------------------------------------------
    -- Bus off reset request capture register.
    -- Capture when there is write to memory registers, clear when request is
    -- processed (in next Sample Point).
    ---------------------------------------------------------------------------
    bus_off_req_capt_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            drv_bus_off_reset_q <= '0';
        elsif (rising_edge(clk_sys)) then
            if (drv_bus_off_reset = '1') then
                drv_bus_off_reset_q <= '1';
            elsif (rx_trigger = '1' and clr_bus_off_rst_flg = '1') then
                drv_bus_off_reset_q <= '0';
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Next state process
    ---------------------------------------------------------------------------
    next_state_proc : process(
        curr_state, drv_ena, err_frm_req, ctrl_ctr_zero, no_data_field,
        drv_fd_type, is_receiver, is_fd_frame,
        is_bus_off, go_to_suspend, tx_frame_ready, drv_bus_off_reset_q,
        reinteg_ctr_expired, rx_data_nbs, is_err_active, go_to_stuff_count,
        pex_on_fdf_enable, pex_on_res_enable, drv_rom_ena)
    begin
        next_state <= curr_state;

        if (drv_ena = CTU_CAN_DISABLED) then
            next_state <= s_pc_off;
            
        elsif (err_frm_req = '1') then
            if (drv_rom_ena = ROM_DISABLED) then
                if (is_err_active = '1') then
                    next_state <= s_pc_act_err_flag;
                else
                    next_state <= s_pc_pas_err_flag;
                end if;
            else
                next_state <= s_pc_integrating;
            end if;
            
        else
            case curr_state is
    
            -------------------------------------------------------------------
            -- Unit is Off (drv_ena = '0')
            -------------------------------------------------------------------
            when s_pc_off =>
                next_state <= s_pc_integrating;
            
            -------------------------------------------------------------------
            -- Unit is integrating (first integration after enabling)
            -------------------------------------------------------------------
            when s_pc_integrating =>
                if (ctrl_ctr_zero = '1') then
                    next_state <= s_pc_idle;
                end if;

            -------------------------------------------------------------------
            -- Start of frame
            -------------------------------------------------------------------
            when s_pc_sof =>
                next_state <= s_pc_base_id;
                
            -------------------------------------------------------------------
            -- Base identifier
            -------------------------------------------------------------------
            when s_pc_base_id =>
                if (ctrl_ctr_zero = '1') then
                    next_state <= s_pc_rtr_srr_r1;
                end if;
            
            -------------------------------------------------------------------
            -- RTR/SRR/R1 bit. First bit after Base identifier.
            -------------------------------------------------------------------
            when s_pc_rtr_srr_r1 =>
                next_state <= s_pc_ide;
            
            -------------------------------------------------------------------
            -- IDE bit
            -------------------------------------------------------------------
            when s_pc_ide =>
                if (rx_data_nbs = DOMINANT) then
                   next_state <= s_pc_edl_r0;
                else
                   next_state <= s_pc_ext_id; 
                end if;
            
            -------------------------------------------------------------------
            -- Extended identifier
            -------------------------------------------------------------------
            when s_pc_ext_id =>
                if (ctrl_ctr_zero = '1') then    
                    next_state <= s_pc_rtr_r1;
                end if;

            -------------------------------------------------------------------
            -- RTR/R1 bit after the Extended identifier
            -------------------------------------------------------------------
            when s_pc_rtr_r1 =>
                next_state <= s_pc_edl_r1;    
                
            -------------------------------------------------------------------
            -- EDL/r1 bit after RTR/r1 bit in Extended Identifier
            -------------------------------------------------------------------
            when s_pc_edl_r1 =>
                if (rx_data_nbs = DOMINANT) then
                    next_state <= s_pc_r0_ext;
                else
                    if (pex_on_fdf_enable = '1') then
                        next_state <= s_pc_integrating;
                    else
                        next_state <= s_pc_r0_fd;
                    end if; 
                end if;
                    
            -------------------------------------------------------------------
            -- r0 bit after EDL/r1 bit in Extended CAN Frames.
            -------------------------------------------------------------------
            when s_pc_r0_ext =>
                next_state <= s_pc_dlc;

            -------------------------------------------------------------------
            -- r0(res) bit in CAN FD Frames (both Base and Extended identifier)
            ------------------------------------------------------------------- 
            when s_pc_r0_fd =>
                if (rx_data_nbs = RECESSIVE and pex_on_res_enable = '1') then
                    next_state <= s_pc_integrating;
                else
                    next_state <= s_pc_brs;
                end if;    
                
            -------------------------------------------------------------------
            -- EDL/r0 bit in CAN 2.0 and CAN FD Frames with BASE identifier
            -- only!
            -------------------------------------------------------------------
            when s_pc_edl_r0 =>
                if (rx_data_nbs = DOMINANT) then
                    next_state <= s_pc_dlc;
                else
                    -- Protocol exception on recessive FDF/EDL in "Classical CAN"
                    -- configuration
                    if (pex_on_fdf_enable = '1') then
                        next_state <= s_pc_integrating;
                    else
                        next_state <= s_pc_r0_fd;
                    end if;
                end if; 
            
            -------------------------------------------------------------------
            -- BRS (Bit rate shift) Bit
            -------------------------------------------------------------------
            when s_pc_brs =>
                next_state <= s_pc_esi;
            
            -------------------------------------------------------------------
            -- ESI (Error State Indicator) Bit
            ------------------------------------------------------------------- 
            when s_pc_esi =>
                next_state <= s_pc_dlc;
            
            -------------------------------------------------------------------
            -- DLC (Data length code)
            -------------------------------------------------------------------
            when s_pc_dlc =>
                if (ctrl_ctr_zero = '1') then 
                    if (no_data_field = '1') then
                        if (go_to_stuff_count = '1') then
                            next_state <= s_pc_stuff_count;
                        else
                            next_state <= s_pc_crc;
                        end if;
                    else
                        next_state <= s_pc_data;
                    end if;
                end if;

            -------------------------------------------------------------------
            -- Data field
            -------------------------------------------------------------------
            when s_pc_data =>
                if (ctrl_ctr_zero = '1') then
                    if (go_to_stuff_count = '1') then
                        next_state <= s_pc_stuff_count;
                    else
                        next_state <= s_pc_crc;
                    end if;
                end if;

            -------------------------------------------------------------------
            -- Stuff count + Stuff parity field
            -------------------------------------------------------------------
            when s_pc_stuff_count =>
                if (ctrl_ctr_zero = '1') then
                    next_state <= s_pc_crc;
                end if;

            -------------------------------------------------------------------
            -- CRC field
            -------------------------------------------------------------------
            when s_pc_crc =>
                if (ctrl_ctr_zero = '1') then
                    next_state <= s_pc_crc_delim;
                end if;
           
            -------------------------------------------------------------------
            -- CRC Delimiter
            -------------------------------------------------------------------
            when s_pc_crc_delim =>
                if (is_fd_frame = '1') then
                    next_state <= s_pc_ack_fd_1;
                else
                    next_state <= s_pc_ack;
                end if;

            -------------------------------------------------------------------
            -- ACK Slot of CAN 2.0 frame
            -------------------------------------------------------------------
            when s_pc_ack =>
                next_state <= s_pc_ack_delim;

            -------------------------------------------------------------------
            -- First bit of CAN FD Frame ACK
            -------------------------------------------------------------------
            when s_pc_ack_fd_1 =>
                next_state <= s_pc_ack_fd_2;
                
            -------------------------------------------------------------------
            -- Second bit of CAN FD Frame ACK
            -------------------------------------------------------------------
            when s_pc_ack_fd_2 =>
                next_state <= s_pc_ack_delim;
            
            -------------------------------------------------------------------
            -- ACK Delimiter
            -------------------------------------------------------------------
            when s_pc_ack_delim =>
                next_state <= s_pc_eof;

            -------------------------------------------------------------------
            -- End of Frame. Receiver sampling DOMINANT in last bit interprets
            -- this as Overload flag!
            -------------------------------------------------------------------
            when s_pc_eof =>
                if (ctrl_ctr_zero = '1') then
                    if (rx_data_nbs = RECESSIVE) then
                        next_state <= s_pc_intermission;
                    elsif (is_receiver = '1') then
                        if (drv_rom_ena = ROM_DISABLED) then
                            next_state <= s_pc_ovr_flag;
                        else
                            next_state <= s_pc_integrating;
                        end if;
                    end if;
                end if;

            -------------------------------------------------------------------
            -- Intermission field
            -------------------------------------------------------------------
            when s_pc_intermission =>
                if (is_bus_off = '1') then
                    next_state <= s_pc_reintegrating_wait;
                    
                -- Last bit of intermission!
                elsif (ctrl_ctr_zero = '1') then
                    if (rx_data_nbs = DOMINANT) then
                        next_state <= s_pc_base_id;
                    elsif (go_to_suspend = '1') then
                        next_state <= s_pc_suspend;
                    elsif (tx_frame_ready = '1') then
                        next_state <= s_pc_sof;
                    else
                        next_state <= s_pc_idle;
                    end if;
                
                -- First or second bit of intermission!
                elsif (rx_data_nbs = DOMINANT) then
                    if (drv_rom_ena = ROM_DISABLED) then
                        next_state <= s_pc_ovr_flag;
                    else
                        next_state <= s_pc_integrating;
                    end if;
                end if;

            -------------------------------------------------------------------
            -- Suspend transmission
            -------------------------------------------------------------------
            when s_pc_suspend =>
                if (rx_data_nbs = DOMINANT) then
                    next_state <= s_pc_base_id;
                elsif (ctrl_ctr_zero = '1') then
                    -- Start transmission after suspend if we have what to
                    -- transmitt!
                    if (tx_frame_ready = '1') then
                        next_state <= s_pc_sof;
                    else    
                        next_state <= s_pc_idle;
                    end if;
                end if;

            -------------------------------------------------------------------
            -- Unit is in Bus idle period.
            -------------------------------------------------------------------
            when s_pc_idle =>
               if (is_bus_off = '1') then
                   next_state <= s_pc_reintegrating_wait;
               elsif (rx_data_nbs = DOMINANT) then
                   next_state <= s_pc_base_id;
               elsif (tx_frame_ready = '1') then
                   next_state <= s_pc_sof;
               end if;

            -------------------------------------------------------------------
            -- Wait till command from User to start re-integration!
            -------------------------------------------------------------------
            when s_pc_reintegrating_wait =>
                if (drv_bus_off_reset_q = '1') then
                    next_state <= s_pc_reintegrating;    
                end if;

            -------------------------------------------------------------------
            -- Unit is re-integrating, waiting till re-integration counter
            -- expires!
            -------------------------------------------------------------------
            when s_pc_reintegrating =>            
                if (reinteg_ctr_expired = '1' and ctrl_ctr_zero = '1') then
                    next_state <= s_pc_idle;
                end if;
                
            -------------------------------------------------------------------
            -- Active error flag.
            -------------------------------------------------------------------
            when s_pc_act_err_flag =>
                if (ctrl_ctr_zero = '1') then
                    next_state <= s_pc_err_delim_wait;
                end if;
            
            -------------------------------------------------------------------
            -- Passive error flag.
            -------------------------------------------------------------------
            when s_pc_pas_err_flag =>
                if (ctrl_ctr_zero = '1') then
                    next_state <= s_pc_err_delim_wait;
                end if;
            
            -------------------------------------------------------------------
            -- Wait till Error delimiter (detection of recessive bit)
            -------------------------------------------------------------------
            when s_pc_err_delim_wait =>
                if (rx_data_nbs = RECESSIVE) then
                    next_state <= s_pc_err_delim;
                elsif (ctrl_ctr_zero = '1') then
                    next_state <= s_pc_err_flag_too_long;
                end if;

            -------------------------------------------------------------------
            -- 13 dominant bits (6 Error flag + 7 Error delimiter) has been
            -- detected.
            -------------------------------------------------------------------
            when s_pc_err_flag_too_long =>
                if (rx_data_nbs = RECESSIVE) then
                    next_state <= s_pc_err_delim;
                end if;

            -------------------------------------------------------------------
            -- 13 dominant bits (6 Overload flag + 7 Overload delimiter) has
            -- been detected.
            -------------------------------------------------------------------
            when s_pc_ovr_flag_too_long =>
                if (rx_data_nbs = RECESSIVE) then
                    next_state <= s_pc_ovr_delim;
                end if;

            -------------------------------------------------------------------
            -- Error delimiter
            -------------------------------------------------------------------
            when s_pc_err_delim =>
                if (ctrl_ctr_zero = '1') then
                    if (rx_data_nbs = DOMINANT) then
                        next_state <= s_pc_ovr_flag;
                    else
                        next_state <= s_pc_intermission;
                    end if;
                end if;
            
            -------------------------------------------------------------------
            -- Overload flag
            -------------------------------------------------------------------
            when s_pc_ovr_flag =>
                if (ctrl_ctr_zero = '1') then
                    next_state <= s_pc_ovr_delim_wait;
                end if;
            
            -------------------------------------------------------------------
            -- Wait till overload delimiter.
            -------------------------------------------------------------------
            when s_pc_ovr_delim_wait =>
                if (rx_data_nbs = RECESSIVE) then
                    next_state <= s_pc_ovr_delim;
                elsif (ctrl_ctr_zero = '1') then
                    next_state <= s_pc_ovr_flag_too_long;
                end if;

            -------------------------------------------------------------------
            -- Overload delimiter
            -------------------------------------------------------------------
            when s_pc_ovr_delim  =>
                if (ctrl_ctr_zero = '1') then
                    if (rx_data_nbs = DOMINANT) then
                        next_state <= s_pc_ovr_flag;
                    else
                        next_state <= s_pc_intermission;
                    end if;
                end if;
            end case;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Current state process
    ---------------------------------------------------------------------------
    curr_state_proc : process(
        curr_state, err_frm_req, sp_control_q_i, tx_failed, drv_ena, rx_data_nbs,
        ctrl_ctr_zero, arbitration_lost_condition, tx_data_wbs, is_transmitter,
        tran_ident_type, tran_frame_type_i, tran_is_rtr, ide_is_arbitration,
        drv_can_fd_ena, tran_brs, rx_trigger, is_err_active, no_data_field,
        drv_fd_type, ctrl_counted_byte, ctrl_counted_byte_index, is_fd_frame,
        is_receiver, crc_match, drv_ack_forb, drv_self_test_ena, tx_frame_ready,
        go_to_suspend, frame_start, ctrl_ctr_one, drv_bus_off_reset_q,
        reinteg_ctr_expired, first_err_delim_q, go_to_stuff_count,
        crc_length_i, data_length_bits_c, ctrl_ctr_mem_index, is_bus_off,
        block_txtb_unlock, drv_pex, rx_data_nbs_prev, sync_edge)
    begin

        -----------------------------------------------------------------------
        -- Default values
        -----------------------------------------------------------------------
        
        -- Control counter
        ctrl_ctr_pload_i     <= '0';
        ctrl_ctr_pload_val   <= (OTHERS => '0');
        ctrl_ctr_pload_unaliged <= '0';
        ctrl_ctr_ena         <= '0';
        compl_ctr_ena_i      <= '0';
        alc_id_field         <= ALC_RSVD; 
        
        -- RX Buffer storing protocol
        store_metadata_d <= '0';
        store_data_d <= '0';
        rec_abort_d <= '0';
        rec_valid_d <= '0';

        sof_pulse_i <= '0';
        
        -- TXT Buffer HW Commands
        txtb_hw_cmd_d.lock    <= '0';
        txtb_hw_cmd_d.unlock  <= '0';
        txtb_hw_cmd_d.valid   <= '0';
        txtb_hw_cmd_d.err     <= '0';
        txtb_hw_cmd_d.arbl    <= '0';
        txtb_hw_cmd_d.failed  <= '0';

        -- RX Shift register interface
        rx_store_base_id_i        <= '0';
        rx_store_ext_id_i         <= '0';
        rx_store_ide_i            <= '0';
        rx_store_rtr_i            <= '0';
        rx_store_edl_i            <= '0';
        rx_store_dlc_i            <= '0';
        rx_store_esi_i            <= '0';
        rx_store_brs_i            <= '0';
        rx_store_stuff_count_i    <= '0';

        rx_shift_ena            <= "0000";
        rx_shift_in_sel         <= '0';
        rx_clear_i              <= '0';

        -- TX Shift register interface
        tx_load_base_id_i         <= '0';
        tx_load_ext_id_i          <= '0';
        tx_load_dlc_i             <= '0';
        tx_load_data_word_i       <= '0';
        tx_load_stuff_count_i     <= '0';
        tx_load_crc_i             <= '0';
        
        tx_shift_ena_i            <= '0';
        tx_dominant               <= '0';
        
        reinteg_ctr_clr      <= '0';
        reinteg_ctr_enable   <= '0';
        is_arbitration_i <= '0';
        tx_dominant    <= '0';
        crc_check      <= '0';
        
        -- Error signalling
        form_err_i <= '0';
        ack_err_i <= '0';            
        crc_err_i <= '0';
        bit_err_arb_i <= '0';
        bit_err_disable <= '0';
        bit_err_disable_receiver <= '0';
        crc_clear_match_flag <= '0';
        err_pos <= ERC_POS_OTHER;
        
        arbitration_lost_i <= '0';
        set_transmitter_i <= '0';
        set_receiver_i <= '0';
        set_idle_i <= '0';
        
        sp_control_switch_data <= '0';
        sp_control_switch_nominal <= '0';
        
        -- Transceiver delay measurement
        ssp_reset_i <= '0';
        tran_delay_meas <= '0';
        
        -- Secondary sampling point control
        btmc_reset        <= '0';
        dbt_measure_start <= '0';
        gen_first_ssp     <= '0';
        
        -- Fault confinement
        primary_err_i <= '0';
        err_delim_late_i <= '0';
        first_err_delim_d <= '0';
        set_err_active_i <= '0';
        
        br_shifted_i <= '0';

        -- Bit Stuffing/Destuffing control
        stuff_length <= std_logic_vector(to_unsigned(5, 3));
        fixed_stuff <= '0';
        stuff_enable_set <= '0';
        stuff_enable_clear <= '0';
        destuff_enable_set <= '0';
        destuff_enable_clear <= '0';
        tx_frame_no_sof_d <= '0';
        
        -- Synchronisation control
        perform_hsync <= '0';

        -- TXT Buffer pointer
        txtb_ptr_d <= 0;

        -- CRC control
        crc_enable <= '0';
        crc_spec_enable_i <= '0';
        load_init_vect_i <= '0';
        
        -- Bit time counters enabling
        nbt_ctrs_en <= '1';
        dbt_ctrs_en <= '0';

        -- Clear block register for retransmitt counter add signal.
        retr_ctr_add_block_clr <= '0';
        tick_state_reg <= '0';

        -- Status signals for debug
        is_control      <= '0';
        is_data         <= '0';
        is_stuff_count  <= '0';
        is_crc          <= '0';
        is_crc_delim    <= '0';
        is_ack_field    <= '0';
        is_ack_delim    <= '0';
        is_eof          <= '0';
        is_suspend      <= '0';
        is_err_frm      <= '0';
        is_overload     <= '0';
        is_intermission <= '0';
        is_sof          <= '0';
        
        clr_bus_off_rst_flg <= '0';
        decrement_rec_i <= '0';
        ack_err_flag_clr <= '0';
        bit_err_after_ack_err <= '0';
        
        pexs_set <= '0';

        if (err_frm_req = '1') then
            tick_state_reg <= '1';
            ctrl_ctr_pload_i   <= '1';
            if (drv_rom_ena = ROM_DISABLED) then
                ctrl_ctr_pload_val <= C_ERR_FLG_DURATION;
            else
                ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                set_idle_i <= '1';
            end if;
            rec_abort_d <= '1';
            
            crc_clear_match_flag <= '1';
            destuff_enable_clear <= '1';
            stuff_enable_clear <= '1';

            if (sp_control_q_i = DATA_SAMPLE or 
                sp_control_q_i = SECONDARY_SAMPLE)
            then
                sp_control_switch_nominal <= '1';
                br_shifted_i <= '1';
            end if;

            if (is_transmitter = '1' and block_txtb_unlock = '0') then
                txtb_hw_cmd_d.unlock <= '1';
                if (tx_failed = '1') then
                    txtb_hw_cmd_d.failed  <= '1';
                else
                    txtb_hw_cmd_d.err     <= '1';
                end if;
            end if;
            
            -- Keep both counters enabled to make sure that Error frame starts
            -- at proper time when error occurred in Data Bit-rate.
            dbt_ctrs_en <= '1';

        else
            case curr_state is

            -------------------------------------------------------------------
            -- Unit is Off (drv_ena = '0')
            -------------------------------------------------------------------
            when s_pc_off =>
                nbt_ctrs_en <= '0';
                if (drv_ena = CTU_CAN_ENABLED) then
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                end if;

            -------------------------------------------------------------------
            -- Unit is integrating (first integration after enabling)
            -------------------------------------------------------------------
            when s_pc_integrating =>
                bit_err_disable <= '1';
                ctrl_ctr_ena <= '1';
                perform_hsync <= '1';
                
                -- Restart integration upon reception of DOMINANT bit or upon
                -- synchronization edge detected!
                if (rx_data_nbs = DOMINANT or sync_edge = '1') then
                    ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                end if;
                
                -- When preloaded due to synchronisation edge, this is
                -- outside of sample point!
                if (rx_data_nbs = DOMINANT) then
                    ctrl_ctr_pload_i <= '1';
                end if;

                if (sync_edge = '1' and
                   -- Third reset condition shall be valid for nodes which are
                   -- CAN FD tolerant or CAN FD enabled!
                   (not(drv_pex = '0' and drv_can_fd_ena = '0')))
                then
                    ctrl_ctr_pload_unaliged <= '1';
                end if;

                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    set_idle_i <= '1';
                    
                    -- Device can be integrating right after start or after
                    -- protocol exception. Only after start, it is bus off,
                    -- then it shall be set to error active and error counters
                    -- shall be cleared. Otherwise, error counters shall retain
                    -- their value!
                    if (is_bus_off = '1') then
                        set_err_active_i <= '1';
                    end if;
                    load_init_vect_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- Start of frame
            -------------------------------------------------------------------
            when s_pc_sof =>
                tick_state_reg <= '1';
                bit_err_disable <= '1';
                ctrl_ctr_pload_i <= '1';
                ctrl_ctr_pload_val <= C_BASE_ID_DURATION;
                tx_load_base_id_i <= '1';
                sof_pulse_i <= '1';
                tx_dominant <= '1';
                err_pos <= ERC_POS_SOF;
                crc_enable <= '1';
                is_sof <= '1';
                
                -- If we have transmission pending, FSM goes to SOF in sample
                -- point of third bit of intermission or in idle. But till the
                -- end of bit, it is still Intermission/Idle from bus
                -- perspective, so we must allow hard-synchronization!
                -- Then, even during TSEG1 of SOF bit, we can keep hard-sync
                -- enabled, because it should be disabled due to no_pos_resync.
                -- since DUT sends dominant bit, and sync edge during TSEG1
                -- means positive phase error! Therefore any sync_edge will
                -- be ignored by prescaler!
                perform_hsync <= '1';
                
                if (rx_data_nbs = RECESSIVE) then
                    form_err_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- Base identifier
            -------------------------------------------------------------------
            when s_pc_base_id =>
                bit_err_disable <= '1';
                ctrl_ctr_ena <= '1';
                rx_shift_ena <= "1111";
                is_arbitration_i <= '1';
                tx_shift_ena_i <= '1';
                err_pos <= ERC_POS_ARB;
                crc_enable <= '1';
                alc_id_field <= ALC_BASE_ID;
                
                if (arbitration_lost_condition = '1') then
                    txtb_hw_cmd_d.unlock <= '1';
                    arbitration_lost_i <= '1';
                    stuff_enable_clear <= '1';
                    if (tx_failed = '1') then
                        txtb_hw_cmd_d.failed  <= '1';
                    else
                        txtb_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    rx_store_base_id_i <= '1';
                end if;
                
                if (tx_data_wbs = DOMINANT and rx_data_nbs = RECESSIVE) then
                    bit_err_arb_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- RTR/SRR/R1 bit. First bit after Base identifier.
            -------------------------------------------------------------------
            when s_pc_rtr_srr_r1 =>
                tick_state_reg <= '1';
                is_arbitration_i <= '1';
                bit_err_disable <= '1';
                crc_enable <= '1';
                rx_store_rtr_i <= '1';
                err_pos <= ERC_POS_ARB;
                alc_id_field <= ALC_SRR_RTR;
                
                if (arbitration_lost_condition = '1') then
                    txtb_hw_cmd_d.unlock <= '1';
                    arbitration_lost_i <= '1';
                    stuff_enable_clear <= '1';
                    if (tx_failed = '1') then
                        txtb_hw_cmd_d.failed  <= '1';
                    else
                        txtb_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                if (is_transmitter = '1' and tran_ident_type = BASE) then
                    if (tran_frame_type_i = FD_CAN or
                        tran_is_rtr = NO_RTR_FRAME)
                    then
                        tx_dominant <= '1';
                    end if;
                end if;
                
                if (tx_data_wbs = DOMINANT and rx_data_nbs = RECESSIVE) then
                    bit_err_arb_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- IDE bit
            -------------------------------------------------------------------
            when s_pc_ide =>
                tick_state_reg <= '1';
                rx_store_ide_i <= '1';
                crc_enable <= '1';
                alc_id_field <= ALC_IDE;
                
                if (rx_data_nbs = RECESSIVE) then
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_EXT_ID_DURATION;
                    tx_load_ext_id_i <= '1';
                end if;
                
                if (ide_is_arbitration = '1' and arbitration_lost_condition = '1') then
                    txtb_hw_cmd_d.unlock <= '1';
                    arbitration_lost_i <= '1';
                    stuff_enable_clear <= '1';
                    if (tx_failed = '1') then
                        txtb_hw_cmd_d.failed  <= '1';
                    else
                        txtb_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                if (ide_is_arbitration = '1') then
                    is_arbitration_i <= '1';
                    bit_err_disable <= '1';
                else
                    is_control <= '1';
                end if;
                
                if (tx_data_wbs = DOMINANT and rx_data_nbs = RECESSIVE) then
                    bit_err_arb_i <= '1';
                end if;

                if (is_transmitter = '1' and tran_ident_type = BASE) then
                    tx_dominant <= '1';
                end if;
                
                if (ide_is_arbitration = '1') then
                    err_pos <= ERC_POS_ARB;
                else
                    err_pos <= ERC_POS_CTRL;
                end if;
    
            -------------------------------------------------------------------
            -- Extended identifier
            -------------------------------------------------------------------
            when s_pc_ext_id =>
                ctrl_ctr_ena <= '1';
                rx_shift_ena <= "1111";
                is_arbitration_i <= '1';
                tx_shift_ena_i  <= '1';
                err_pos <= ERC_POS_ARB;
                bit_err_disable <= '1';
                crc_enable <= '1';
                alc_id_field <= ALC_EXTENSION;
                
                if (arbitration_lost_condition = '1') then
                    txtb_hw_cmd_d.unlock <= '1';
                    arbitration_lost_i <= '1';
                    stuff_enable_clear <= '1';
                    if (tx_failed = '1') then
                        txtb_hw_cmd_d.failed  <= '1';
                    else
                        txtb_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    rx_store_ext_id_i         <= '1';
                end if;

                if (tx_data_wbs = DOMINANT and rx_data_nbs = RECESSIVE) then
                    bit_err_arb_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- RTR/R1 bit after the Extended identifier
            -------------------------------------------------------------------
            when s_pc_rtr_r1 =>
                tick_state_reg <= '1';
                is_arbitration_i <= '1';
                bit_err_disable <= '1';
                crc_enable <= '1';                
                rx_store_rtr_i <= '1';
                err_pos <= ERC_POS_ARB;
                alc_id_field <= ALC_RTR;
                
                if (arbitration_lost_condition = '1') then
                    txtb_hw_cmd_d.unlock <= '1';
                    arbitration_lost_i <= '1';
                    stuff_enable_clear <= '1';
                    if (tx_failed = '1') then
                        txtb_hw_cmd_d.failed  <= '1';
                    else
                        txtb_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                if (is_transmitter = '1') then
                    if (tran_frame_type_i = FD_CAN) then
                        tx_dominant <= '1';
                    elsif (tran_is_rtr = NO_RTR_FRAME) then
                        tx_dominant <= '1';
                    end if;
                end if;
                
                if (tx_data_wbs = DOMINANT and rx_data_nbs = RECESSIVE) then
                    bit_err_arb_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- EDL/r1 bit after RTR/r1 bit in Extended Identifier
            -------------------------------------------------------------------
            when s_pc_edl_r1 =>
                tick_state_reg <= '1';
                rx_store_edl_i <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                is_control <= '1';
                bit_err_disable_receiver <= '1';
                
                if (is_transmitter = '1') then
                    if (tran_frame_type_i = NORMAL_CAN) then
                        tx_dominant <= '1';
                    else
                        ssp_reset_i <= '1';
                    end if;
                end if;
                
                -- Sample recessive but CAN FD is disabled -> Form error or
                -- protocol exception!
                if (rx_data_nbs = RECESSIVE and
                    drv_can_fd_ena = FDE_DISABLE)
                then
                    if (drv_pex = PROTOCOL_EXCEPTION_DISABLED) then
                        form_err_i <= '1';
                    
                    -----------------------------------------------------------
                    -- Detect protocol exception. Disable bit stuffing, and
                    -- unlock TXT Buffer if needed in case user attempted to
                    -- transmitt frame on which its own protocol exception is
                    -- detected!
                    -----------------------------------------------------------
                    else
                        if (is_transmitter = '1') then
                            txtb_hw_cmd_d.unlock <= '1';
                            stuff_enable_clear <= '1';
                            if (tx_failed = '1') then
                                txtb_hw_cmd_d.failed  <= '1';
                            else
                                txtb_hw_cmd_d.arbl    <= '1';
                            end if;
                        end if;
                        destuff_enable_clear <= '1';
                        ctrl_ctr_pload_i <= '1';
                        ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                        pexs_set <= '1';
                    end if;
                end if;

            -------------------------------------------------------------------
            -- r0 bit after EDL/r1 bit in Extended CAN Frames.
            -------------------------------------------------------------------
            when s_pc_r0_ext =>
                tick_state_reg <= '1';
                ctrl_ctr_pload_i <= '1';
                ctrl_ctr_pload_val <= C_DLC_DURATION;
                tx_load_dlc_i <= '1';
                err_pos <= ERC_POS_CTRL;
                tran_delay_meas <= '1';
                crc_enable <= '1';
                is_control <= '1';
                bit_err_disable_receiver <= '1';
                
                if (is_transmitter = '1') then
                    tx_dominant <= '1';
                end if;

            -------------------------------------------------------------------
            -- r0 bit in CAN FD Frames (both Base and Extended identifier)
            ------------------------------------------------------------------- 
            when s_pc_r0_fd =>
                tick_state_reg <= '1';
                tran_delay_meas <= '1';
                err_pos <= ERC_POS_CTRL;
                perform_hsync <= '1';
                crc_enable <= '1';
                is_control <= '1';
                bit_err_disable_receiver <= '1';
                
                if (is_transmitter = '1') then
                    tx_dominant <= '1';
                end if;
                
                -- Here recessive would mean further extending beyond CAN FD
                -- protocol (CAN XL in future). Now we don't have protocol
                -- exception, so we throw error here!
                if (rx_data_nbs = RECESSIVE) then
                    if (drv_pex = PROTOCOL_EXCEPTION_DISABLED) then
                        form_err_i <= '1';

                    -----------------------------------------------------------
                    -- Detect protocol exception. Disable bit stuffing, and
                    -- unlock TXT Buffer if needed in case user attempted to
                    -- transmitt frame on which its own protocol exception is
                    -- detected!
                    -----------------------------------------------------------
                    else
                        if (is_transmitter = '1') then
                            txtb_hw_cmd_d.unlock <= '1';
                            stuff_enable_clear <= '1';
                            if (tx_failed = '1') then
                                txtb_hw_cmd_d.failed  <= '1';
                            else
                                txtb_hw_cmd_d.arbl    <= '1';
                            end if;
                        end if;
                        destuff_enable_clear <= '1';
                        ctrl_ctr_pload_i <= '1';
                        ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                        pexs_set <= '1';
                    end if;
                end if;
                
            -------------------------------------------------------------------
            -- EDL/r0 bit in CAN 2.0 and CAN FD Frames with BASE identifier
            -- only!
            -------------------------------------------------------------------
            when s_pc_edl_r0 =>
                tick_state_reg <= '1';
                rx_store_edl_i <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                is_control <= '1';
                bit_err_disable_receiver <= '1';
            
                if (rx_data_nbs = DOMINANT) then
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_DLC_DURATION;
                    tx_load_dlc_i <= '1';
                end if;
                
                if (is_transmitter = '1' and tran_frame_type_i = NORMAL_CAN) then
                    tx_dominant <= '1';
                else
                    ssp_reset_i <= '1';
                end if;
                
                -- Sample recessive but CAN FD is disabled -> Form error or
                -- protocol exception!
                if (rx_data_nbs = RECESSIVE and
                    drv_can_fd_ena = FDE_DISABLE)
                then
                    if (drv_pex = PROTOCOL_EXCEPTION_DISABLED) then
                        form_err_i <= '1';

                    -----------------------------------------------------------
                    -- Detect protocol exception. Disable bit stuffing, and
                    -- unlock TXT Buffer if needed in case user attempted to
                    -- transmitt frame on which its own protocol exception is
                    -- detected!
                    -----------------------------------------------------------
                    else
                        if (is_transmitter = '1') then
                            txtb_hw_cmd_d.unlock <= '1';
                            stuff_enable_clear <= '1';
                            if (tx_failed = '1') then
                                txtb_hw_cmd_d.failed  <= '1';
                            else
                                txtb_hw_cmd_d.arbl    <= '1';
                            end if;
                        end if;
                        destuff_enable_clear <= '1';
                        ctrl_ctr_pload_i <= '1';
                        ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                        pexs_set <= '1';
                    end if;
                end if;

            -------------------------------------------------------------------
            -- BRS (Bit rate shift) Bit
            -------------------------------------------------------------------
            when s_pc_brs =>
                tick_state_reg <= '1';
                rx_store_brs_i <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                is_control <= '1';
                bit_err_disable_receiver <= '1';
                dbt_ctrs_en <= '1';
                btmc_reset  <= '1';

                if (is_transmitter = '1' and tran_brs = BR_NO_SHIFT) then
                    tx_dominant <= '1';
                end if;
                
                if (rx_data_nbs = RECESSIVE and rx_trigger = '1') then
                    sp_control_switch_data <= '1';
                    br_shifted_i <= '1';
                end if;
                
            -------------------------------------------------------------------
            -- ESI (Error State Indicator) Bit
            ------------------------------------------------------------------- 
            when s_pc_esi =>
                tick_state_reg <= '1';
                ctrl_ctr_pload_i <= '1';
                ctrl_ctr_pload_val <= C_DLC_DURATION;
                tx_load_dlc_i <= '1';
                rx_store_esi_i <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                is_control <= '1';
                bit_err_disable_receiver <= '1';
                dbt_ctrs_en <= '1';
                
                if (is_transmitter = '1' and is_err_active = '1') then
                    tx_dominant <= '1';
                end if;
                
                -- Transmitter transmitts via SSP
                if (sp_control_q_i = SECONDARY_SAMPLE) then
                    dbt_measure_start <= '1';
                    gen_first_ssp     <= '1';
                end if;

            -------------------------------------------------------------------
            -- DLC (Data length code)
            -------------------------------------------------------------------
            when s_pc_dlc =>
                ctrl_ctr_ena <= '1';
                rx_shift_ena <= "1111";
                tx_shift_ena_i  <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                is_control <= '1';
                bit_err_disable_receiver <= '1';
                
                if (sp_control_q_i /= NOMINAL_SAMPLE) then
                    dbt_ctrs_en <= '1';
                end if;
                
                -- Address first Data Word in TXT Buffer RAM in advance to
                -- account for DFF delay and RAM delay! Do it only when tran-
                -- smitting to avoid toggling of RAM signals during reception
                -- (possible power consideration)
                if (is_transmitter = '1') then 
                    txtb_ptr_d <= 4;
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';

                    if (no_data_field = '1') then
                        if (go_to_stuff_count = '1') then
                            ctrl_ctr_pload_val <= C_STUFF_COUNT_DURATION;
                            tx_load_stuff_count_i <= '1';
                        else
                            ctrl_ctr_pload_val <= crc_length_i;
                            tx_load_crc_i <= '1';
                        end if;
                    else
                        ctrl_ctr_pload_val <= data_length_bits_c;
                        tx_load_data_word_i <= '1';
                    end if;

                    store_metadata_d <= '1';
                    rx_store_dlc_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- Data field
            -------------------------------------------------------------------
            when s_pc_data =>
                ctrl_ctr_ena <= '1';
                rx_shift_ena(to_integer(unsigned(ctrl_counted_byte_index))) <= '1';
                rx_shift_in_sel <= '1';
                tx_shift_ena_i <= '1';
                err_pos <= ERC_POS_DATA;
                crc_enable <= '1';
                is_data <= '1';
                compl_ctr_ena_i <= '1';
                bit_err_disable_receiver <= '1';
                
                if (sp_control_q_i /= NOMINAL_SAMPLE) then
                    dbt_ctrs_en <= '1';
                end if;
                
                -- Address next word (the one after actually transmitted one),
                -- so that when current word ends, TXT Buffer RAM already
                -- provides data on its output! Counter is divided by 32 since
                -- each memory word contains 32 bits!
                if (is_transmitter = '1') then
                    txtb_ptr_d <= to_integer(unsigned(ctrl_ctr_mem_index));
                end if;

                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';

                    if (go_to_stuff_count = '1') then
                        ctrl_ctr_pload_val <= C_STUFF_COUNT_DURATION;
                        tx_load_stuff_count_i <= '1';
                    else
                        ctrl_ctr_pload_val <= crc_length_i;
                        tx_load_crc_i <= '1';
                    end if;
                    
                    -- Store data word at the end of data field.
                    store_data_d <= '1';
                end if;

                -- Store data word when multiple of 4 data bytes were counted!
                -- Avoid storing at the end of Data field, because CRC must be
                -- preloaded then!
                if (ctrl_counted_byte = '1' and 
                    ctrl_counted_byte_index = "11" and
                    ctrl_ctr_zero = '0')
                then
                    store_data_d <= '1';
                    tx_load_data_word_i <= '1';
                end if;
                    
            -------------------------------------------------------------------
            -- Stuff count + Stuff parity field
            -------------------------------------------------------------------
            when s_pc_stuff_count =>
                ctrl_ctr_ena <= '1';
                rx_shift_ena <= "1111";
                tx_shift_ena_i <= '1';
                err_pos <= ERC_POS_CRC;
                crc_enable <= '1';
                is_stuff_count <= '1';
                bit_err_disable_receiver <= '1';
                
                if (sp_control_q_i /= NOMINAL_SAMPLE) then
                    dbt_ctrs_en <= '1';
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_val <= crc_length_i;
                    ctrl_ctr_pload_i <= '1';
                    tx_load_crc_i <= '1';
                    rx_store_stuff_count_i <= '1';
                end if;
    
                if (is_fd_frame = '1') then
                    stuff_length <= std_logic_vector(to_unsigned(4, 3));
                    fixed_stuff <= '1';
                end if;

            -------------------------------------------------------------------
            -- CRC field
            -------------------------------------------------------------------
            when s_pc_crc =>
                ctrl_ctr_ena <= '1';
                rx_shift_ena <= "1111";
                tx_shift_ena_i <= '1';
                err_pos <= ERC_POS_CRC;
                is_crc <= '1';
                bit_err_disable_receiver <= '1';
                
                if (sp_control_q_i /= NOMINAL_SAMPLE) then
                    dbt_ctrs_en <= '1';
                end if;

                if (is_fd_frame = '1') then
                    stuff_length <= std_logic_vector(to_unsigned(4, 3));
                    fixed_stuff <= '1';
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                end if;

            -------------------------------------------------------------------
            -- CRC Delimiter
            -------------------------------------------------------------------
            when s_pc_crc_delim =>
                tick_state_reg <= '1';
                err_pos <= ERC_POS_ACK;
                is_crc_delim  <= '1';
                dbt_ctrs_en <= '1';
                bit_err_disable <= '1';
                destuff_enable_clear <= '1';
                stuff_enable_clear <= '1';

                if (rx_trigger = '1') then
                    if (is_receiver = '1') then
                        crc_check <= '1';
                    end if;
                    
                    if (rx_data_nbs = DOMINANT) then
                        form_err_i <= '1';
                    end if;
                    
                    if (sp_control_q_i = DATA_SAMPLE or 
                        sp_control_q_i = SECONDARY_SAMPLE)
                    then
                        sp_control_switch_nominal <= '1';
                        br_shifted_i <= '1';
                    end if;
                end if;

            -------------------------------------------------------------------
            -- ACK Slot of CAN 2.0 frame
            -------------------------------------------------------------------
            when s_pc_ack =>
                tick_state_reg <= '1';
                err_pos <= ERC_POS_ACK;
                is_ack_field  <= '1';
                dbt_ctrs_en <= '1';
                
                if (is_receiver = '1' and crc_match = '1' and
                    drv_ack_forb = '0')
                then
                    tx_dominant <= '1';

                -- Bit Error still shall be detected when unit sends dominant
                -- (receiver) and receives recessive!
                else
                    bit_err_disable <= '1';
                end if;

                if (is_receiver = '1' and crc_match = '1' and
                    rx_data_nbs = DOMINANT)
                then
                    decrement_rec_i <= '1';
                end if;

                if (is_transmitter = '1' and drv_self_test_ena = '0' and
                    rx_data_nbs = RECESSIVE)
                then
                    ack_err_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- First bit of CAN FD Frame ACK - Receiver sends ACK
            -------------------------------------------------------------------
            when s_pc_ack_fd_1 =>
                tick_state_reg <= '1';
                err_pos <= ERC_POS_ACK;
                is_ack_field  <= '1';
                dbt_ctrs_en <= '1';

                if (is_receiver = '1' and crc_match = '1' and
                    drv_ack_forb = '0')
                then
                    tx_dominant <= '1';

                -- Bit Error still shall be detected when unit sends dominant
                -- (receiver) and receives recessive!
                else
                    bit_err_disable <= '1';
                end if;
                
                if (is_receiver = '1' and crc_match = '1' and
                    rx_data_nbs = DOMINANT)
                then
                    decrement_rec_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- Second bit of CAN FD Frame ACK
            -------------------------------------------------------------------
            when s_pc_ack_fd_2 =>
                tick_state_reg <= '1';
                err_pos <= ERC_POS_ACK;
                is_ack_field  <= '1';
                dbt_ctrs_en <= '1';
                
                -- No ACK sent now, but dominant or recessive should be tolerated.
                bit_err_disable <= '1';

                -- Transmitter not detecting dominant bit now, nor at previous
                -- bit -> Ack Error
                if (is_transmitter = '1' and drv_self_test_ena = '0' and
                    rx_data_nbs = RECESSIVE and rx_data_nbs_prev = RECESSIVE)
                then
                    ack_err_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- ACK Delimiter
            -------------------------------------------------------------------
            when s_pc_ack_delim =>
                tick_state_reg <= '1';
                ctrl_ctr_pload_i <= '1';
                ctrl_ctr_pload_val <= C_EOF_DURATION;
                err_pos <= ERC_POS_ACK;
                is_ack_delim  <= '1';
                bit_err_disable <= '1';
                
                if (rx_data_nbs = DOMINANT) then
                    form_err_i <= '1';
                end if;
                
                if (is_receiver = '1' and crc_match = '0') then
                    crc_err_i <= '1';
                end if;
    
            -------------------------------------------------------------------
            -- End of Frame. Receiver sampling DOMINANT in last bit interprets
            -- this as Overload flag!
            -------------------------------------------------------------------
            when s_pc_eof =>
                ctrl_ctr_ena <= '1';
                is_eof <= '1';
                err_pos <= ERC_POS_EOF;
                bit_err_disable <= '1';

                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    
                    if (rx_data_nbs = RECESSIVE) then
                        ctrl_ctr_pload_val <= C_INTERMISSION_DURATION;
                        
                        -- No Error until the end of EOF means frame is valid
                        -- for transmitter!
                        if (is_transmitter = '1') then
                            txtb_hw_cmd_d.unlock <= '1';
                            txtb_hw_cmd_d.valid  <= '1';
                        end if;
                        
                    elsif (is_receiver = '1') then
                        if (drv_rom_ena = ROM_DISABLED) then
                            ctrl_ctr_pload_val <= C_OVR_FLG_DURATION;
                        else
                            ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                            set_idle_i <= '1';
                        end if;
                    end if;
                    
                    crc_clear_match_flag <= '1';
                end if;

                -- If there is no error (RX Recessive) in one bit before end
                -- of EOF, signal valid Frame reception!
                if (ctrl_ctr_one = '1' and rx_data_nbs = RECESSIVE) then
                    rec_valid_d <= '1';
                end if;
                
                -- DOMINANT during EOF. All bits before last -> Form error!
                -- Last bit -> Receiver treats it as overload condition, so
                -- no error frame will be transmitted. Transmitter treats it
                -- as Form error!
                if (rx_data_nbs = DOMINANT) then
                    if (ctrl_ctr_zero = '0') then
                        form_err_i <= '1';
                    elsif (is_transmitter = '1') then
                        form_err_i <= '1';
                    end if;
                end if;
    
            -------------------------------------------------------------------
            -- Intermission field
            -------------------------------------------------------------------
            when s_pc_intermission =>
                ctrl_ctr_ena <= '1';
                is_intermission <= '1';
                retr_ctr_add_block_clr <= '1';
                bit_err_disable <= '1';
                
                -- If we are bus-off, go to reintegration wait!
                if (is_bus_off = '1') then
                    tick_state_reg <= '1';
                end if;

                -- Last (third) bit of intermission
                if (ctrl_ctr_zero = '1' and is_bus_off = '0') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    crc_spec_enable_i <= '1';
                    
                    -- Here FSM goes to Base ID (sampling of DOMINANT in the
                    -- third bit of intermission)!
                    if (rx_data_nbs = DOMINANT) then
                        ctrl_ctr_pload_val <= C_BASE_ID_DURATION;
                        tx_load_base_id_i <= '1';
                        sof_pulse_i <= '1';
                        
                    -- Here FSM goes to either IDLE, Suspend, or to SOF, when
                    -- it has sth. to transmitt. We preload SUSPEND length in
                    -- any case, since other states don't care about control
                    -- counter.
                    else
                        ctrl_ctr_pload_val <= C_SUSPEND_DURATION;
                    end if;

                    -- Lock TXT Buffer when there is what to transmitt, and no
                    -- suspend! Unit becomes transmitter! If not, and DOMINANT
                    -- is received, become receiver! 
                    if (tx_frame_ready = '1' and go_to_suspend = '0') then
                        txtb_hw_cmd_d.lock <= '1';
                        set_transmitter_i <= '1';
                        stuff_enable_set <= '1';

                        if (rx_data_nbs = DOMINANT) then
                            tx_frame_no_sof_d <= '1';
                        end if;
                        
                    elsif (rx_data_nbs = DOMINANT) then
                        set_receiver_i   <= '1';
                    end if;
                    
                    -- Transmission/reception started -> Enable Bit stuffing!
                    -- Clear RX Shift Register!
                    if (frame_start = '1') then
                        destuff_enable_set <= '1';
                        rx_clear_i <= '1';
                    end if;
                    
                    -- If we dont sample dominant, nor we have sth ready for
                    -- transmission, we go to Idle! Don't become idle when we
                    -- go to suspend!
                    if (rx_data_nbs = RECESSIVE and tx_frame_ready = '0' and
                        go_to_suspend = '0')
                    then
                        set_idle_i <= '1';
                    end if;
    
                -- First or second bit of intermission!
                elsif (rx_data_nbs = DOMINANT and is_bus_off = '0') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    if (drv_rom_ena = ROM_DISABLED) then
                        ctrl_ctr_pload_val <= C_OVR_FLG_DURATION;
                    else
                        ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                        set_idle_i <= '1';
                    end if;
                end if;
                
                -- Second or third bit of intermission, Hard Synchronisation
                if (ctrl_ctr_zero = '1' or ctrl_ctr_one = '1') then
                    perform_hsync <= '1';
                end if;
                
                -- First or second bit of Intermission, pre-load CRC Init vector
                -- for next frame.
                if (ctrl_ctr_zero = '0') then
                    load_init_vect_i <= '1';
                end if;
    
            -------------------------------------------------------------------
            -- Suspend transmission
            -------------------------------------------------------------------
            when s_pc_suspend =>
                ctrl_ctr_ena <= '1';
                perform_hsync <= '1';
                crc_spec_enable_i <= '1';
                bit_err_disable <= '1';
                is_suspend <= '1';
                
                if (rx_data_nbs = DOMINANT) then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_BASE_ID_DURATION;
                    tx_load_base_id_i <= '1';
                    sof_pulse_i <= '1';
                    set_receiver_i <= '1';
                    destuff_enable_set <= '1';
                    rx_clear_i <= '1';

                -- End of Suspend -> Unit goes to IDLE if there is nothing to
                -- transmitt, otherwise it goes to SOF and transmitts
                elsif (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    if (tx_frame_ready = '1') then
                        set_transmitter_i <= '1';
                        txtb_hw_cmd_d.lock <= '1';
                        rx_clear_i <= '1';
                        destuff_enable_set <= '1';
                        stuff_enable_set <= '1';
                    else
                        set_idle_i <= '1';
                    end if;
                end if;
    
            -------------------------------------------------------------------
            -- Unit is in Bus idle period.
            -------------------------------------------------------------------
            when s_pc_idle =>
                perform_hsync <= '1';
                crc_spec_enable_i <= '1';
                bit_err_disable <= '1';
                
                if (is_bus_off = '0') then
                    if (rx_data_nbs = DOMINANT) then
                        tick_state_reg <= '1';
                        ctrl_ctr_pload_i <= '1';
                        ctrl_ctr_pload_val <= C_BASE_ID_DURATION;
                        sof_pulse_i <= '1';
                        crc_enable <= '1';
                    end if;

                    if (tx_frame_ready = '1') then
                        tick_state_reg <= '1';
                        txtb_hw_cmd_d.lock <= '1';
                        set_transmitter_i <= '1';
                        tx_load_base_id_i <= '1';
                        stuff_enable_set <= '1';
    
                        if (rx_data_nbs = DOMINANT) then
                            tx_frame_no_sof_d <= '1';
                        end if;
    
                    elsif (rx_data_nbs = DOMINANT) then
                        set_receiver_i <= '1';
                    end if;

                    -- Transmission/reception started -> Enable Bit de-stuffing!
                    -- Clear RX Shift register!
                    if (frame_start = '1') then
                        destuff_enable_set <= '1';
                        rx_clear_i <= '1';
                    end if;
                    
                -- If we are bus-off we need to move to wait for reintegration command!
                else
                    tick_state_reg <= '1';
                end if;

            -------------------------------------------------------------------
            -- Wait till command from User to start re-integration!
            -------------------------------------------------------------------
            when s_pc_reintegrating_wait =>
                bit_err_disable <= '1';
                
                if (drv_bus_off_reset_q = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    reinteg_ctr_clr <= '1';
                    ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                    clr_bus_off_rst_flg <= '1';
                end if;

            -------------------------------------------------------------------
            -- Unit is re-integrating, waiting till re-integration counter
            -- expires!
            -------------------------------------------------------------------
            when s_pc_reintegrating =>
                ctrl_ctr_ena <= '1';
                perform_hsync <= '1';
                bit_err_disable <= '1';

                -- Restart integration upon reception of DOMINANT bit or upon
                -- synchronization edge detected!
                if (rx_data_nbs = DOMINANT or sync_edge = '1') then
                    ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;
                end if;
                
                -- When preloaded due to synchronisation edge, this is
                -- outside of sample point!
                if (rx_data_nbs = DOMINANT) then
                    ctrl_ctr_pload_i <= '1';
                end if;

                if (sync_edge = '1' and
                   -- Third reset condition shall be valid for nodes which are
                   -- CAN FD tolerant or CAN FD enabled!
                   (not(drv_pex = '0' and drv_can_fd_ena = '0')))
                then
                    ctrl_ctr_pload_unaliged <= '1';
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    reinteg_ctr_enable <= '1';
                end if;

                if (ctrl_ctr_zero = '1' and reinteg_ctr_expired = '0') then
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_INTEGRATION_DURATION;    
                end if;

                if (reinteg_ctr_expired = '1' and ctrl_ctr_zero = '1' and
                    rx_trigger = '1')
                then
                    tick_state_reg <= '1';
                    set_idle_i <= '1';
                    set_err_active_i <= '1';
                    load_init_vect_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- Active error flag.
            -------------------------------------------------------------------
            when s_pc_act_err_flag =>
                ctrl_ctr_ena <= '1';
                is_err_frm <= '1';
                tx_dominant <= '1';
                err_pos <= ERC_POS_ERR;

                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_DELIM_WAIT_DURATION;
                    first_err_delim_d <= '1';
                end if;

            -------------------------------------------------------------------
            -- Passive error flag.
            -------------------------------------------------------------------
            when s_pc_pas_err_flag =>
                ctrl_ctr_ena <= '1';
                is_err_frm <= '1';
                err_pos <= ERC_POS_ERR;
                
                -- Node sending Passive error flag may receive RECESSIVE or
                -- DOMINANT, and DOMINANT shall not be treated as bit error!
                bit_err_disable <= '1';
                
                -- Reseting control counter if different bit that previous
                -- is detected. Passive error flag must be completed after
                -- 6 bits of equal polarity!
                if (rx_data_nbs_prev /= rx_data_nbs) then
                    ctrl_ctr_pload_i   <= '1';
                    ctrl_ctr_pload_val <= C_SHORTENED_ERR_FLG_DURATION;
                elsif (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_DELIM_WAIT_DURATION;
                    first_err_delim_d <= '1';
                end if;
                
                -- If dominant bit is detected, and previous error was ACK, then
                -- TEC shall be still incremented!
                if (ack_err_flag = '1' and rx_data_nbs = DOMINANT and rx_trigger = '1') then
                    bit_err_after_ack_err <= '1';
                    ack_err_flag_clr <= '1';
                end if;

            -------------------------------------------------------------------
            -- Wait till Error delimiter (detection of recessive bit)
            -------------------------------------------------------------------
            when s_pc_err_delim_wait =>
                is_err_frm <= '1';
                err_pos <= ERC_POS_ERR;
                ack_err_flag_clr <= '1';
                
                -- When waiting for RECESSIVE bit after Error flag, unit
                -- may receive DOMINANT and not interpret this as Bit error!
                bit_err_disable <= '1';

                if (ctrl_ctr_zero = '0') then
                    ctrl_ctr_ena <= '1';
                else
                    tick_state_reg <= '1';
                end if;

                if (rx_data_nbs = RECESSIVE) then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_ERR_DELIM_DURATION;
                end if;

                -- Node received dominant bit as first bit after Error flag!
                -- This shall be treated as primary error
                if (rx_data_nbs = DOMINANT and first_err_delim_q = '1') then
                    primary_err_i <= '1';
                    first_err_delim_d <= '0';
                end if;

            -------------------------------------------------------------------
            -- 13 dominant bits (6 error flag + 7 error delimiter) has been
            -- detected (active error flag), or 7 has been detected (passive
            -- error flag).
            -------------------------------------------------------------------
            when s_pc_err_flag_too_long =>
                is_err_frm <= '1';
                err_pos <= ERC_POS_ERR;
                bit_err_disable <= '1';
                ctrl_ctr_ena <= '1';

                if (rx_data_nbs = RECESSIVE) then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_ERR_DELIM_DURATION;

                -- This indicates that either 14th dominant bit was detected,
                -- or each next consecutive 8 DOMINANT bits were detected!
                elsif (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_DOMINANT_REPEAT_DURATION;
                    err_delim_late_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- 13 dominant bits (6 overload flag + 7 overload delimiter) has 
            -- been detected.
            -------------------------------------------------------------------
            when s_pc_ovr_flag_too_long =>
                is_overload <= '1';
                err_pos <= ERC_POS_OVRL;
                bit_err_disable <= '1';
                ctrl_ctr_ena <= '1';

                if (rx_data_nbs = RECESSIVE) then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_OVR_DELIM_DURATION;

                -- This indicates that either 14th dominant bit was detected,
                -- or each next consecutive 8 DOMINANT bits were detected!
                elsif (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_DOMINANT_REPEAT_DURATION;
                    err_delim_late_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- Error delimiter
            -------------------------------------------------------------------
            when s_pc_err_delim =>
                is_err_frm <= '1';
                ctrl_ctr_ena <= '1';
                err_pos <= ERC_POS_ERR;
                bit_err_disable <= '1';
                                
                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';

                    if (rx_data_nbs = DOMINANT) then
                        ctrl_ctr_pload_val <= C_OVR_FLG_DURATION;
                    else
                        ctrl_ctr_pload_val <= C_INTERMISSION_DURATION;
                    end if;
                elsif (rx_data_nbs = DOMINANT) then
                    form_err_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- Overload flag
            -------------------------------------------------------------------
            when s_pc_ovr_flag =>
                is_overload <= '1';
                ctrl_ctr_ena <= '1';
                tx_dominant <= '1';
                err_pos <= ERC_POS_OVRL;
                
                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_DELIM_WAIT_DURATION;
                end if;
                
            -------------------------------------------------------------------
            -- Wait till overload delimiter.
            -------------------------------------------------------------------
            when s_pc_ovr_delim_wait =>
                is_overload <= '1';
                err_pos <= ERC_POS_OVRL;
                
                if (ctrl_ctr_zero = '0') then
                    ctrl_ctr_ena <= '1';
                else
                    tick_state_reg <= '1';
                end if;
                
                -- When waiting for RECESSIVE bit after Overload flag, unit
                -- may receive DOMINANT and not interpret this as Bit error!
                bit_err_disable <= '1';
                
                if (rx_data_nbs = RECESSIVE) then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';
                    ctrl_ctr_pload_val <= C_OVR_DELIM_DURATION;
                end if;

            -------------------------------------------------------------------
            -- Overload delimiter
            -------------------------------------------------------------------
            when s_pc_ovr_delim  =>
                ctrl_ctr_ena <= '1';
                is_overload <= '1';
                err_pos <= ERC_POS_OVRL;
                bit_err_disable <= '1';

                if (ctrl_ctr_zero = '1') then
                    tick_state_reg <= '1';
                    ctrl_ctr_pload_i <= '1';

                    if (rx_data_nbs = DOMINANT) then
                        ctrl_ctr_pload_val <= C_OVR_FLG_DURATION;
                    else
                        ctrl_ctr_pload_val <= C_INTERMISSION_DURATION;
                    end if;
                elsif (rx_data_nbs = DOMINANT) then
                    form_err_i <= '1';
                end if;

            end case;
        end if;

    end process;
    
    -----------------------------------------------------------------------
    -- Turn on/off of whole controller is not synchronized with any
    -- other event! Therefore it creates separate clock enable condition
    -- for FSM state register
    -----------------------------------------------------------------------
    tick_state_reg_on_off <=
        '1' when (curr_state = s_pc_off and drv_ena = CTU_CAN_ENABLED) else
        '1' when (curr_state /= s_pc_off and drv_ena = CTU_CAN_DISABLED) else
        '0';

    -----------------------------------------------------------------------
    -- FSM State register
    -----------------------------------------------------------------------
    state_reg_ce <=
        '1' when (tick_state_reg = '1' and ctrl_signal_upd = '1') else
        '1' when (tick_state_reg_on_off = '1') else
        '0';

    fsm_state_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            curr_state <= s_pc_off;
        elsif (rising_edge(clk_sys)) then
            if (state_reg_ce = '1') then
                curr_state <= next_state;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------
    -- Control counter is preloaded:
    --  1. When core is off and becomes non-off
    --  2. When preloaded by any state and RX trigger is active. This saves
    --     gating with RX Trigger in each FSM state!
    --  3. When counter is reset during integration due to synchronisation
    --     edge. This can be anytime, not just in sample point!
    -----------------------------------------------------------------------
    ctrl_ctr_pload <= ctrl_ctr_pload_i when (curr_state = s_pc_off) else
                      ctrl_ctr_pload_i when (ctrl_signal_upd = '1') else
                      '1' when (ctrl_ctr_pload_unaliged = '1') else
                      '0';

    -----------------------------------------------------------------------
    -- Registering control commands to RX Buffer due to following reasons
    --  1. In last bit of DLC, DLC is not yet sampled in RX Shift register,
    --     thus we need to delay storing of metadata word by one clock
    --     cycle!
    --  2. Break possible long combinational paths between RX Buffer and
    --     Protocol control FSM! 
    -----------------------------------------------------------------------
    rx_buf_cmds_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            store_metadata     <= '0';
            store_data         <= '0';
            rec_valid          <= '0';
            rec_abort          <= '0';
        elsif (rising_edge(clk_sys)) then
            
            -- Frame is stored to RX Buffer when unit is either receiver
            -- or loopback mode is enabled.
            -- Each command is active only for one clock cycle!
            if ((is_receiver = '1' or drv_int_loopback_ena = '1') and
                ((rx_trigger = '1') or (err_frm_req = '1')))
            then
                store_metadata     <= store_metadata_d;
                store_data         <= store_data_d;
                rec_valid          <= rec_valid_d;
                rec_abort          <= rec_abort_d;
            else
                store_metadata     <= '0';
                store_data         <= '0';
                rec_valid          <= '0';
                rec_abort          <= '0';
            end if;
        end if;
    end process;
    
    ctrl_signal_upd <= '1' when (rx_trigger = '1' or err_frm_req = '1')
                           else
                       '0';

    -----------------------------------------------------------------------
    -- TXT Buffer HW commands pipeline
    -----------------------------------------------------------------------
    txtb_hw_cmd_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            txtb_hw_cmd_q.lock    <= '0';
            txtb_hw_cmd_q.unlock  <= '0';
            txtb_hw_cmd_q.valid   <= '0';
            txtb_hw_cmd_q.err     <= '0';
            txtb_hw_cmd_q.arbl    <= '0';
            txtb_hw_cmd_q.failed  <= '0';
        elsif (rising_edge(clk_sys)) then
            if (ctrl_signal_upd = '1') then
                txtb_hw_cmd_q <= txtb_hw_cmd_d;
            else
                txtb_hw_cmd_q <= ('0', '0', '0', '0', '0', '0');
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------
    -- RX Shift register commands gating. Each command can be active only
    -- in sample point (rx_trigger = '1')!
    -----------------------------------------------------------------------
    rx_store_base_id <= rx_store_base_id_i and rx_trigger;
    rx_store_ext_id <= rx_store_ext_id_i and rx_trigger;
    rx_store_ide <= rx_store_ide_i and rx_trigger;
    rx_store_rtr <= rx_store_rtr_i and rx_trigger;
    rx_store_edl <= rx_store_edl_i and rx_trigger;
    rx_store_dlc <= rx_store_dlc_i and rx_trigger;
    rx_store_esi <= rx_store_esi_i and rx_trigger;
    rx_store_brs <= rx_store_brs_i and rx_trigger;
    rx_store_stuff_count <= rx_store_stuff_count_i and rx_trigger;

    -----------------------------------------------------------------------
    -- TX Shift register commands gating. Each command can be active only
    -- in sample point (rx_trigger = '1')!
    -----------------------------------------------------------------------
    tx_load_base_id <= tx_load_base_id_i and rx_trigger;        
    tx_load_ext_id <= tx_load_ext_id_i and rx_trigger;         
    tx_load_dlc <= tx_load_dlc_i and rx_trigger;
    tx_load_data_word <= tx_load_data_word_i and rx_trigger;
    tx_load_stuff_count <= tx_load_stuff_count_i and rx_trigger;
    tx_load_crc <= tx_load_crc_i and rx_trigger;
   
    -----------------------------------------------------------------------
    -- TX Shift register is enabled only when the unit is transmitter!
    -----------------------------------------------------------------------
    tx_shift_ena <= '1' when (tx_shift_ena_i = '1' and is_transmitter = '1')
                        else
                    '0';
    
    -----------------------------------------------------------------------
    -- Error signalling gating. Each command can be active only in sample 
    -- point (rx_trigger = '1')!
    -----------------------------------------------------------------------
    form_err <= form_err_i and rx_trigger;
    ack_err <= ack_err_i and rx_trigger; 
    crc_err <= crc_err_i and rx_trigger;
    bit_err_arb <= bit_err_arb_i and rx_trigger;
    decrement_rec <= decrement_rec_i and rx_trigger;

    -----------------------------------------------------------------------
    -- Switching of Bit-rate
    -----------------------------------------------------------------------
    switch_to_ssp <= '1' when (sp_control_switch_data = '1' and
                               is_transmitter = '1' and
                               drv_ssp_delay_select /= SSP_SRC_NO_SSP)
                         else
                     '0';
    
    sp_control_d <=   NOMINAL_SAMPLE when (sp_control_switch_nominal = '1')
                                     else
                    SECONDARY_SAMPLE when (switch_to_ssp = '1')
                                     else
                         DATA_SAMPLE when (sp_control_switch_data = '1')
                                     else
                        sp_control_q_i;

    sp_control_ce <= '1' when (sp_control_switch_nominal = '1') else
                     '1' when (sp_control_switch_data = '1') else
                     '0';

    sp_control_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            sp_control_q_i <= NOMINAL_SAMPLE;
        elsif (rising_edge(clk_sys)) then
            if (sp_control_ce = '1') then
                sp_control_q_i <= sp_control_d;
            end if;
        end if;
    end process;

    sp_control <= sp_control_d when (br_shifted_i = '1') else
                  sp_control_q_i;

    ---------------------------------------------------------------------------
    -- Indicates that Active Error or Overload flag is being transmitted!
    -- Can't be part of current state, since it must be valid also during
    -- error condition to distiguish error during error flag!
    ---------------------------------------------------------------------------
    act_err_ovr_flag <= '1' when (curr_state = s_pc_act_err_flag) else
                        '1' when (curr_state = s_pc_ovr_flag) else
                        '0';

    first_err_delim_flag_reg : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            first_err_delim_q <= '0';
        elsif (rising_edge(clk_sys)) then
            if (rx_trigger = '1') then
                first_err_delim_q <= first_err_delim_d;
            end if;
        end if;        
    end process;

    ---------------------------------------------------------------------------
    -- Detection of primary error and late error delimiter must be active only
    -- for one clock cycle in Sample point (rx_trigger)!
    ---------------------------------------------------------------------------
    primary_err <= '1' when (primary_err_i = '1' and rx_trigger = '1')
                       else
                   '0';

    err_delim_late <= '1' when (err_delim_late_i = '1' and rx_trigger = '1')
                          else
                      '0';

    set_err_active <= '1' when (set_err_active_i = '1' and rx_trigger = '1')
                          else
                      '0';

    rx_clear <= '1' when (rx_clear_i = '1' and rx_trigger = '1')
                    else
                '0'; 

    ---------------------------------------------------------------------------
    -- Bit error is disabled:
    --  1. In arbitration field, there it is detected extra since only
    --     transmitting dominant and receiving recessive is trated as bit error.
    --  2. For receiver during control, data, CRC fields!
    ---------------------------------------------------------------------------                 
    bit_err_enable <= '0' when (bit_err_disable = '1') else
                      '0' when (bit_err_disable_receiver = '1' and
                                is_receiver = '1')
                          else
                      '1';

    ---------------------------------------------------------------------------
    -- Retransmitt counter is incremented when error frame is detected, or
    -- when arbitration loss occurs!
    -- Active only when:
    --  1. Counter is not cleared (clear has priority)
    --  2. Retransmitt limitation is enabled. Not counting when disabled.
    --  3. Unit is reciever. Only transmitter counts re-transmissions!
    ---------------------------------------------------------------------------
    retr_ctr_add_i <= '0' when (retr_ctr_clear_i = '1' or drv_retr_lim_ena = '0'
                              or is_receiver = '1' or retr_ctr_add_block = '1') else
                    '1' when (arbitration_lost_i = '1' and rx_trigger = '1') else
                    '1' when (err_frm_req = '1') else
                    '0';

    ---------------------------------------------------------------------------
    -- Retransmitt counter is cleared when:
    --  1. Transmission is valid.
    --  2. Transmission failed (TXT Buffer is moving to TX Error)!
    ---------------------------------------------------------------------------
    retr_ctr_clear_i <= '1' when (txtb_hw_cmd_d.valid = '1' and rx_trigger = '1')
                            else
                        '1' when (txtb_hw_cmd_d.failed = '1')
                            else
                        '0';

    ---------------------------------------------------------------------------
    -- Retransmitt counter must be modified only once if multiple Error frames
    -- are requested during single frame.
    ---------------------------------------------------------------------------
    retr_ctr_add_block_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            retr_ctr_add_block <= '0';
        elsif (rising_edge(clk_sys)) then
            if (retr_ctr_add_i = '1') then
                retr_ctr_add_block <= '1';
            elsif (retr_ctr_add_block_clr = '1') then
                retr_ctr_add_block <= '0';
            end if;
        end if;
    end process;


    -- Start of frame pulse is active in Sample point of SOF only!
    sof_pulse <= '1' when (sof_pulse_i = '1' and rx_trigger = '1')
                     else
                 '0';

    ---------------------------------------------------------------------------
    -- Complementary counter counts only in Sample point once per bit time.
    ---------------------------------------------------------------------------
    compl_ctr_ena <= '1' when (compl_ctr_ena_i = '1' and rx_trigger = '1')
                         else
                     '0';
                     
    ---------------------------------------------------------------------------
    -- Operation control commands active in Sample point only!
    ---------------------------------------------------------------------------
    set_transmitter <= '1' when (set_transmitter_i = '1' and rx_trigger = '1')
                           else
                       '0';

    set_receiver <= '1' when (set_receiver_i = '1' and rx_trigger = '1')
                        else
                    '0';

    ---------------------------------------------------------------------------
    -- Idle must be un-gated also by Error frame request, since in ROM mode
    -- it is possible that upon any kind of error, unit should go to integrating
    -- and therefore stop being transmitter or receiver!
    ---------------------------------------------------------------------------
    set_idle <= '1' when (set_idle_i = '1' and (rx_trigger = '1' or err_frm_req = '1'))
                    else
                '0';

    ---------------------------------------------------------------------------
    -- CRC select source for calculation:
    --  1. When speculative enable is selected, always use RX Data. This is
    --     in idle/intermission/suspend when dominant is sampled and cosidered
    --     as SOF.
    --  2. When we are in arbitration, always use idle. This is to make sure
    --     that transmitting recessive and receiving dominant (loosing arbi-
    --     tration) will calculate data from DOMINANT value
    --  3. In other cases Transmitter uses TX Data, Receiver uses RX Data.
    ---------------------------------------------------------------------------
    crc_calc_from_rx <= '1' when (crc_spec_enable_i = '1') else
                        '1' when (is_arbitration_i = '1') else
                        '1' when (is_receiver = '1') else
                        '0';

    load_init_vect <= '1' when (load_init_vect_i = '1' and rx_trigger = '1')
                          else
                      '0';

    ---------------------------------------------------------------------------
    -- Bit Stuffing enable
    ---------------------------------------------------------------------------
    stuff_ena_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            stuff_enable <= '0';
        elsif (rising_edge(clk_sys)) then
            if (ctrl_signal_upd = '1') then
                if (stuff_enable_set = '1') then
                   stuff_enable <= '1';
                elsif (stuff_enable_clear = '1') then
                   stuff_enable <= '0';
               end if;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Bit DeStuffing enable, Stuff Error
    ---------------------------------------------------------------------------
    destuff_ena_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            destuff_enable <= '0';
        elsif (rising_edge(clk_sys)) then
            if (ctrl_signal_upd = '1') then
                if (destuff_enable_set = '1') then
                    destuff_enable <= '1';
                elsif (destuff_enable_clear = '1') then
                    destuff_enable <= '0';
                end if;
            end if;
        end if;    
    end process;
    
    
    ---------------------------------------------------------------------------
    -- Synchronisation type
    ---------------------------------------------------------------------------
    sync_control_d <= NO_SYNC when ((sp_control_switch_data = '1' and
                                     is_transmitter = '1') or                                    
                                    sp_control_q_i = SECONDARY_SAMPLE or
                                    (sp_control_q_i = DATA_SAMPLE and
                                     is_transmitter = '1'))
                              else
                    HARD_SYNC when (perform_hsync = '1')
                              else
                      RE_SYNC;
    
    sync_control_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            sync_control_q <= HARD_SYNC;
        elsif (rising_edge(clk_sys)) then
            sync_control_q <= sync_control_d;
        end if;
    end process;
    
    -----------------------------------------------------------------------
    -- TXT Buffer pointer registering
    -----------------------------------------------------------------------
    txtb_ptr_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            txtb_ptr_q <= 0;
        elsif (rising_edge(clk_sys)) then
            if (txtb_clk_en_d = '1') then
                txtb_ptr_q <= txtb_ptr_d;
            end if;
        end if;
    end process;
    
    -- Enable memory only when pointer changes. This allows clocking the
    -- memory only when new read data are to be read!
    txtb_clk_en_d <= '1' when (txtb_ptr_q /= txtb_ptr_d) else
                     '0';

    -----------------------------------------------------------------------
    -- Register clock enable! Data need to be loaded from TXT Buffer RAM
    -- after address has changed!
    -----------------------------------------------------------------------
    txtb_ce_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            txtb_clk_en_q <= '0';
        elsif (rising_edge(clk_sys)) then
            txtb_clk_en_q <= txtb_clk_en_d;
        end if;
    end process;

    -----------------------------------------------------------------------
    -- Frame transmission (transmitter) started without SOF!
    -----------------------------------------------------------------------
    tx_frame_no_sof_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tx_frame_no_sof_q <= '0';
        elsif (rising_edge(clk_sys)) then
            if (rx_trigger = '1') then
                tx_frame_no_sof_q <= tx_frame_no_sof_d;
            end if;
        end if;
    end process;
    
    -----------------------------------------------------------------------
    -- Registering value from previous bit of CAN_RX signal
    -----------------------------------------------------------------------
    prev_rx_data_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            rx_data_nbs_prev <= RECESSIVE;
        elsif (rising_edge(clk_sys)) then
            if (rx_trigger = '1') then
                rx_data_nbs_prev <= rx_data_nbs;
            end if;
        end if;
    end process;
    
    -----------------------------------------------------------------------
    -- Remembering ACK error. Needed by transmitted sending passive error
    -- frame due to ACK error.
    -----------------------------------------------------------------------
    ack_err_flag_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            ack_err_flag <= '0';
        elsif (rising_edge(clk_sys)) then
            if (ack_err_i = '1' and rx_trigger = '1') then
                ack_err_flag <= '1';
            elsif (ack_err_flag_clr = '1') then
                ack_err_flag <= '0';
            end if;
        end if;    
    end process;

    -----------------------------------------------------------------------
    -- Protocol exception status
    -----------------------------------------------------------------------
    pexs_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            is_pexs <= '0';
        elsif (rising_edge(clk_sys)) then
            if (pexs_set = '1') then
                is_pexs <= '1';
            elsif (drv_cpexs = '1') then
                is_pexs <= '0';
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------
    -- Internal signals to output propagation
    -----------------------------------------------------------------------
    crc_src <= crc_src_i;
    txtb_hw_cmd <= txtb_hw_cmd_q;
    tran_valid <= txtb_hw_cmd_q.valid;
    ssp_reset <= ssp_reset_i; 
    sync_control <= sync_control_q;
    txtb_ptr <= txtb_ptr_q;
    pc_state <= curr_state;
    br_shifted <= br_shifted_i;
    sp_control_q <= sp_control_q_i;
    is_arbitration <= is_arbitration_i;
    crc_spec_enable <= crc_spec_enable_i;
    retr_ctr_clear <= retr_ctr_clear_i;
    arbitration_lost <= arbitration_lost_i;
    retr_ctr_add <= retr_ctr_add_i;
    tx_frame_no_sof <= tx_frame_no_sof_q;
    txtb_clk_en <= txtb_clk_en_q;

    -- <RELEASE_OFF>
    -----------------------------------------------------------------------
    -----------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------
    -----------------------------------------------------------------------
    
    -- psl default clock is rising_edge(clk_sys);

    -- psl no_simul_crc_17_crc_21_asrt : assert never
    --  (crc_use_17 = '1' and crc_use_21 = '1')
    --  report "Can't use simultaneously CRC 17 and CRC 21";
    
    -- psl no_simul_rx_trigger_err_req_asrt : assert never
    --  (rx_trigger = '1' and err_frm_req = '1')
    --  report "RX Trigger and Error frame request can't be active at once " &
    --  " since they should occur in different pipeline stages!";

    -- psl no_simul_rx_rtr_and_fd_frame_asrt : assert never
    --  (rec_is_rtr = RTR_FRAME and rec_frame_type = FD_CAN)
    --  report "RTR and FDF can't be received simultaneously!";


    -- Error frame requests can't arrive during following frame fields:
    --  OFF, Integrating, reintegrating, Idle, Intermission (dominant bit
    --  is interpreted as Overload or SOF of new frame), Suspend
    --  (dominant bit is new frame), Error delimiter wait (Accepts 
    --  both dominant and recessive and waits for recessive)
    
    -- psl no_err_frm_req_in_off : assert never
    --  (err_frm_req = '1') and
    --  (curr_state = s_pc_off or curr_state = s_pc_integrating or
    --   curr_state = s_pc_idle or curr_state = s_pc_intermission or
    --   curr_state = s_pc_suspend or curr_state = s_pc_reintegrating)   
    --  report "Error frame request in invalid Protocol control field!";
    
    -- psl no_secondary_sample_receiver : assert never
    --  (sp_control_q_i = SECONDARY_SAMPLE) and (is_receiver = '1')
    --  report "Receiver shall never use secondary sample point!";

    -- psl no_sof_receiver : assert never
    --  (curr_state = s_pc_sof and is_receiver = '1')
    --  report "SOF state shall never be entered when receiver!";

    -- psl no_err_ovr_in_rom_mode : assert never
    --  (curr_state = s_pc_act_err_flag or curr_state = s_pc_pas_err_flag or
    --   curr_state = s_pc_err_delim or curr_state = s_pc_ovr_delim)
    --  and (drv_rom_ena = ROM_ENABLED)
    -- report "Error or Overload frames shall never be transmitted in ROM mode!"; 

    -- psl no_tx_in_rom_mode : assert never
    --  (drv_rom_ena = '1' and is_transmitter = '1')
    -- report "Device shall not transmit frames in ROM mode!";

    -- psl no_stuff_destuff_in_eof : assert never
    --  (stuff_enable = '1' or destuff_enable = '1') and
    --  (curr_state = s_pc_eof)
    -- report "Bit stuffing destuffin should be never enabled during EOF!";


    -----------------------------------------------------------------------
    -----------------------------------------------------------------------
    -- Functional coverage
    -----------------------------------------------------------------------
    -----------------------------------------------------------------------

    -- Error frame request in various parts of CAN frame!

    -- Note: SOF must be actually previous cycle, since at time when error
    --       frame request arrives, it is already next state!
    -- psl err_frm_req_in_sof_cov : cover
    --  {curr_state = s_pc_sof; err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_base_id_in_base_cov : cover
    --  {curr_state = s_pc_base_id and err_frm_req = '1'};
    
    -- psl err_frm_req_in_s_pc_ext_id_in_ext_id_cov : cover
    --  {curr_state = s_pc_ext_id and err_frm_req = '1'};
    
    -- psl err_frm_req_in_s_pc_ext_id_in_rtr_srr_r1_cov : cover
    --  {curr_state = s_pc_rtr_srr_r1 and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ext_id_in_ide_cov : cover
    --  {curr_state = s_pc_ide and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_rtr_r1_cov : cover
    --  {curr_state = s_pc_rtr_r1 and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_edl_r1_cov : cover
    --  {curr_state = s_pc_edl_r1 and err_frm_req = '1'};
    
    -- psl err_frm_req_in_s_pc_r0_ext_cov : cover
    --  {curr_state = s_pc_r0_ext and err_frm_req = '1'};
    
    -- psl err_frm_req_in_s_pc_r0_fd_cov : cover
    --  {curr_state = s_pc_r0_fd and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_edl_r0_cov : cover
    --  {curr_state = s_pc_edl_r0 and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_esi_cov : cover
    --  {curr_state = s_pc_esi and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_dlc_cov : cover
    --  {curr_state = s_pc_dlc and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_data_cov : cover
    --  {curr_state = s_pc_data and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_stuff_count_cov : cover
    --  {curr_state = s_pc_stuff_count and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_crc_cov : cover
    --  {curr_state = s_pc_crc and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_crc_delim_cov : cover
    --  {curr_state = s_pc_crc_delim and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ack_cov : cover
    --  {curr_state = s_pc_ack and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_eof_cov : cover
    --  {curr_state = s_pc_eof and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_act_err_flag_cov : cover
    --  {curr_state = s_pc_act_err_flag and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ovr_flag_cov : cover
    --  {curr_state = s_pc_ovr_flag and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ovr_delim_cov : cover
    --  {curr_state = s_pc_ovr_delim and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_err_delim_cov : cover
    --  {curr_state = s_pc_err_delim and err_frm_req = '1'};


    -- Overload frame requests
    
    -- psl ovr_from_eof_cov : cover
    --  {curr_state = s_pc_eof and next_state = s_pc_ovr_flag};
    
    -- psl ovr_from_intermission_cov : cover
    --  {curr_state = s_pc_intermission and next_state = s_pc_ovr_flag};
    
    -- psl ovr_from_err_delim : cover
    --  {curr_state = s_pc_err_delim and next_state = s_pc_ovr_flag};
    
    -- psl ovr_from_ovr_delim_cov : cover
    --  {curr_state = s_pc_ovr_delim and next_state = s_pc_ovr_flag};


    -- Protocol exception
    
    -- psl pex_on_fdf_enable_cov : cover
    --  {pex_on_fdf_enable = '1' and is_pexs = '1'};
    
    -- psl pex_on_res_enable_cov : cover
    --  {pex_on_res_enable = '1' and is_pexs = '1'};
    
    -- psl pex_in_s_pc_r0_fd_cov : cover
    --  {curr_state = s_pc_r0_fd and pexs_set = '1'};
    
    -- psl pex_in_s_pc_edl_r1_cov : cover
    --  {curr_state = s_pc_edl_r1 and pexs_set = '1'};


    -- Arbitration lost
    
    -- psl arb_lost_base_id_cov : cover
    --  {curr_state = s_pc_base_id and arbitration_lost_i = '1'};
    
    -- psl arb_lost_rtr_srr_r1_cov : cover
    --  {curr_state = s_pc_rtr_srr_r1 and arbitration_lost_i = '1'};
    
    -- psl arb_lost_ide_cov : cover
    --  {curr_state = s_pc_ide and arbitration_lost_i = '1'};
    
    -- psl arb_lost_ext_id_cov : cover
    --  {curr_state = s_pc_ext_id and arbitration_lost_i = '1'};
    
    -- psl arb_lost_rtr_r1_cov : cover
    --  {curr_state = s_pc_rtr_r1 and arbitration_lost_i = '1'};
    
    -- <RELEASE_ON>

end architecture;
