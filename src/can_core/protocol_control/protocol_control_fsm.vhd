--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
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
-- Purpose:
--  Protocol control state machine according to CAN FD protocol specification. 
--  Processes input data with "rx_trigger" signal (in Sample point of Bit).
--------------------------------------------------------------------------------
-- Revision History:
--
--    July 2015   Created file
--    24.3.2019   Stashed all previous notes due to re-implementation.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.endian_swap.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

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

        -----------------------------------------------------------------------
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual TX Data
        tx_data                 :in   std_logic;
        
        -- Actual RX Data
        rx_data                 :in   std_logic;
        
        -----------------------------------------------------------------------
        -- RX Buffer interface
        -----------------------------------------------------------------------
        -- Command to store CAN frame metadata to RX Buffer
        store_metadata          :out  std_logic;

        -- Store whole word of CAN Data
        store_data              :out  std_logic;
        
        -- Received frame valid
        rec_valid               :out  std_logic;
        
        -- Abort storing of RX frame (due to Error frame)
        rec_abort               :out  std_logic;
        
        -- Start of Frame pulse
        sof_pulse               :out  std_logic;

        -----------------------------------------------------------------------
        -- TXT Buffer, TX Arbitrator interface
        -----------------------------------------------------------------------
        -- There is a valid frame for transmission
        tran_frame_valid        :in   std_logic;
        
        -- HW Commands to TXT Buffers
        txt_hw_cmd              :out  txt_hw_cmd_type;
        
        -- Pointer to TXT Buffer memory
        txt_buf_ptr             :out  natural range 0 to 19;
        
        -- TX Data length code
        tran_dlc                :in   std_logic_vector(3 downto 0);
        
        -- TX Remote transmission request flag
        tran_is_rtr             :in   std_logic;
        
        -- TX Frame type (0-CAN 2.0, 1-CAN FD)
        tran_frame_type         :in   std_logic;

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
        rx_shift_ena            :out  std_logic(3 downto 0);
        
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
        ctrl_count_pload        :out   std_logic;
        
        -- Control counter preload value
        ctrl_count_pload_val    :out   std_logic_vector(8 downto 0);
        
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
        form_error              :out   std_logic;

        -- ACK Error has occurred
        ack_error               :out   std_logic;

        -- Perform CRC check
        crc_check               :out   std_logic;
        
        -- Bit Error in arbitration field
        bit_error_arb           :out   std_logic;
        
        -- Calculated CRC and Stuff count are matching received ones
        crc_match               :in   std_logic;

        -- CRC error signalling
        crc_error               :out  std_logic;

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

        -- Length of Bit stuffing rule
        stuff_length            :out   std_logic_vector(2 downto 0);
        
        -- Fixed Bit stuffing method
        fixed_stuff             :out   std_logic;
        
        -- Bit Stuff error detection enabled
        stuff_error_enable      :out   std_logic;
        
        -----------------------------------------------------------------------
        -- Operation control interface
        -----------------------------------------------------------------------
        -- Unit is transmitter
        is_transmitter          :in   std_logic;
        
        -- Unit is receiver
        is_receiver             :in   std_logic;

        -- Unit is idle
        is_idle                 :in   std_logic;

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
        primary_error           :out  std_logic;
        
        -- Active Error or Overload flag is being tranmsmitted
        act_err_ovr_flag        :out  std_logic;

        -- Error delimiter too late
        err_delim_late          :out  std_logic;

        -- Unit is error active
        is_err_active           :in   std_logic;
        
        -- Unit is error passive
        is_err_passive          :in   std_logic;
        
        -- Unit is Bus off
        is_bus_off              :in   std_logic;

        -----------------------------------------------------------------------
        -- Other control signals
        -----------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control              :out   std_logic_vector(1 downto 0);

        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation)
        sync_control            :out   std_logic_vector(1 downto 0);

        -- No Resynchronisation due to positive phase error
        no_pos_resync           :out   std_logic;

        -- Clear the Shift register for secondary sampling point.
        ssp_reset               :out   std_logic;

        -- Enable measurement of Transciever delay
        trv_delay_calib         :out   std_logic;

        -- Protocol control FSM state output
        pc_state                :out   t_protocol_control_state;

        -- Transmitted frame is valid
        tran_valid              :out   std_logic;

        -- ACK received
        ack_received            :out   std_logic;

        -- CRC calculation enabled
        crc_enable              :out   std_logic;
        
        -- CRC calculation - speculative enable
        crc_spec_enable         :out   std_logic;

        -- Bit error enable
        bit_error_enable        :out   std_logic;

        -- Bit rate shifted
        br_shifted              :out   std_logic
    );
end entity;

architecture rtl of protocol_control_fsm is

    ---------------------------------------------------------------------------
    -- FSM related signals
    ---------------------------------------------------------------------------
    -- Protocol control FSM
    signal curr_state : t_protocol_control_state;
    signal next_state : t_protocol_control_state;

    -- Clock enable for state register
    signal state_reg_ce : std_logic;
    
    ---------------------------------------------------------------------------
    -- Internal combinational signals
    ---------------------------------------------------------------------------
    -- No data field should be transmitted
    signal no_data_transmitter : std_logic;
    signal no_data_receiver : std_logi;
    signal no_data_field : std_logic;
    
    -- Allow 2 bit long CRC delimiter
    signal allow_2bit_crc_delim : std_logic;
    
    -- Allow 2 bit long ACK slot
    signal allow_2bit_ack : std_logic;

    -- Preload control counter internal signal
    signal ctrl_count_pload_i : std_logic;
    
    -- CRC Selection
    signal crc_use_21         : std_logic;
    signal crc_use_17         : std_logic;
    signal crc_src_i          : std_logic_vector(1 downto 0);
    signal crc_length_i       : std_logic_vector(8 downto 0);

    -- Length of data field (decoded from DLC, does not take RTR into account)
    signal tran_data_length   : std_logic_vector(6 downto 0);
    signal rec_data_length    : std_logic_vector(6 downto 0);
    signal rec_data_length_c  : std_logic_vector(6 downto 0);

    signal data_length_c      : std_logic;
    
    -- FD Frame is being transmitted/received
    signal is_fd_frame        : std_logic;
    
    -- Frame transmission/reception can be started from idle or intermission!
    signal frame_start        : std_logic;
    
    -- There is TX Frame ready for transmission
    signal tx_frame_ready     : std_logic;
    
    -- IDE bit is part of arbitration
    signal ide_is_arbitration : std_logic;
    
    -- Arbitration lost condition
    signal arbitration_lost_condition : std_logic;
    
    -- Transmission failed (due to reached number of retransmissions), or
    -- first error, arb lost when there are 0 retransmissions allowed!
    signal tx_failed            : std_logic;
    
    -- Internal commands for RX Buffer
    signal store_metadata_d     : std_logic;
    signal store_data_d         : std_logic;
    signal rec_valid_d          : std_logic;
    signal rec_abort_d          : std_logic;

    -- Internal commands for TXT Buffers
    signal txt_hw_cmd_d         : txt_hw_cmd_type;
    signal txt_hw_cmd_q         : txt_hw_cmd_type;
    
    -- Unit should go to suspend transmission field!
    signal go_to_suspend        : std_logic;
    
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
    
    signal rx_clear_d                :  std_logic;
    signal rx_clear_q                :  std_logic;

    -- Internal commands for TX Shift register
    signal tx_load_base_id_i         :  std_logic;
    signal tx_load_ext_id_i          :  std_logic;
    signal tx_load_dlc_i             :  std_logic;
    signal tx_load_data_word_i       :  std_logic;
    signal tx_load_stuff_count_i     :  std_logic;
    signal tx_load_crc_i             :  std_logic;

    signal tx_shift_ena_i            :  std_logic;

    -- Internal signals for detected errors
    signal form_error_i              :  std_logic;
    signal ack_error_i               :  std_logic;  
    signal crc_error_i               :  std_logic;
    signal bit_error_arb_i           :  std_logic;

    -- Sample control (Bit Rate) signals
    signal sp_control_switch_data    :  std_logic;
    signal sp_control_switch_nominal :  std_logic;
    
    signal sp_control_ce             :  std_logic;
    signal sp_control_d              :  std_logic;
    signal sp_control_q              :  std_logic;

    -- Secondary sampling point shift register reset
    signal ssp_reset_d               :  std_logic;
    signal ssp_reset_q               :  std_logic;
    
    -- Synchronisation control
    signal sync_control_d            :  std_logic_vector(1 downto 0);
    signal sync_control_q            :  std_logic_vector(1 downto 0);
    
    -- Hard synchronisation should be performed
    signal perform_hsync             :  std_logic;
    
    -- Fault confinemnt interface
    signal primary_error_i           :  std_logic;
    signal err_delim_late_i          :  std_logic;
    
    -- Flag which holds whether FSM is in first bit of error delimiter 
    signal first_err_delim_d         :  std_logic;
    signal first_err_delim_q         :  std_logic;
    
    -- Bit stuffing 
    signal stuff_enable_set          :  std_logic;
    signal stuff_enable_clear        :  std_logic;

    -- Bit error disable (internal)
    -- Note: Bit Error is rather disabled than enabled, since it is disabled on less
    -- places than enabled!
    signal bit_err_disable           :  std_logic;
    
    -- TXT Buffer pointer
    signal txt_buf_ptr_d             :  natural range 0 to 19;
    signal txt_buf_ptr_q             :  natural range 0 to 19;

begin

    tx_frame_ready <= '1' when (tran_frame_valid = '1' and drv_bus_mon_ena = '0')
                          else
                      '0';

    no_data_transmitter <= '1' when (tran_dlc = "0000" or tran_is_rtr = RTR_FRAME)
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

    allow_2bit_crc_delim <= '1' when (is_transmitter = '1' and 
                                      tran_frame_type = FD_CAN)
                                else
                            '0';

    allow_2bit_ack <= '1' when (is_transmitter = '1' and tran_frame_type = FD_CAN)
                          else
                      '1' when (is_receiver = '1' and rec_frame_type = FD_CAN)
                          else
                      '0';

    go_to_suspend <= '1' when (is_err_passive = '1' and is_transmitter = '1')
                         else
                     '0';

    ide_is_arbitration <= '1' when (tran_frame_type = EXTENDED)
                              else
                          '0';

    arbitration_lost_condition <= '1' when (is_transmitter = '1' and 
                                            tx_data = RECESSIVE and
                                            rx_data = DOMINANT)
                                      else
                                  '0';

    tx_failed <= '1' when (drv_retr_lim_ena = '1' and retr_limit_reached = '1')
                     else
                 '0';

    is_fd_frame <= '1' when (is_transmitter = '1' and tran_frame_type = FD_CAN)
                       else
                   '1' when (is_receiver = '1' and rec_frame_type = FD_CAN)
                       else
                   '0';
                   
    frame_start <= '1' when (tx_frame_ready = '1' and go_to_suspend = '0') else
                   '1' when (rx_data = DOMINANT) else
                   '0';

    ---------------------------------------------------------------------------
    -- CRC sequence selection
    ---------------------------------------------------------------------------
    crc_use_21 <= '1' when (is_transmitter = '1' and tran_frame_type = FD_CAN and
                            to_integer(unsigned(tran_data_length)) > 16)
                      else
                  '1' when (is_receiver = '1' and rec_frame_type = FD_CAN and
                            to_integer(unsigned(rec_data_length)) > 16)
                      else
                  '0';

    crc_use_17 <= '1' when (is_transmitter = '1' and tran_frame_type = FD_CAN and
                            crc_use_21 = '0')
                      else
                  '1' when (is_receiver = '1' and rec_frame_type = FD_CAN and
                            crc_use_21 = '0')
                      else
                  '0';
                            

    crc_src_i <= CRC21 when (crc_use_21 = '1') else
                 CRC17 when (crc_use_17 = '1') else
                 CRC15;

    crc_length_i <= C_CRC15_DURATION when (crc_src_i = CRC15) else
                    C_CRC17_DURATION when (crc_src_i = CRC17) else
                    C_CRC21_DURATION;

    ---------------------------------------------------------------------------
    -- DLC to Data length decoders
    ---------------------------------------------------------------------------
    dlc_decoder_tx_inst : dlc_decoder
    port map(
        dlc           => tran_dlc,
        frame_type    => tran_frame_type,

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

    ---------------------------------------------------------------------------
    -- Next state process
    ---------------------------------------------------------------------------
    next_state_proc : process
    begin
        next_state <= curr_state;

        if (drv_ena = CTU_CAN_DISABLED) then
            next_state <= s_pc_off;
            
        elsif (err_frm_req = '1') then
            if (is_err_active = '1') then
                next_state <= s_pc_act_err_flag;
            else
                next_state <= s_pc_pas_err_flag;
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
                if (rx_data = DOMINANT) then
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
                if (rx_data = DOMINANT) then
                    next_state <= s_pc_r0_ext;
                else
                    next_state <= s_pc_r0_fd; 
                end if;
                    
            -------------------------------------------------------------------
            -- r0 bit after EDL/r1 bit in Extended CAN Frames.
            -------------------------------------------------------------------
            when s_pc_r0_ext =>
                next_state <= s_pc_dlc;

            -------------------------------------------------------------------
            -- r0 bit in CAN FD Frames (both Base and Extended identifier)
            ------------------------------------------------------------------- 
            when s_pc_r0_fd =>
                next_state <= s_pc_brs;    
                
            -------------------------------------------------------------------
            -- EDL/r0 bit in CAN 2.0 and CAN FD Frames with BASE identifier
            -- only!
            -------------------------------------------------------------------
            when s_pc_edl_r0 =>
                if (rx_data = DOMINANT) then
                    next_state <= s_pc_dlc;
                else
                    next_state <= s_pc_r0_fd;
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
                        if (drv_fd_type = ISO_FD) then
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
                    if (drv_fd_type = ISO_FD) then
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
                    next_state <= s_pc_prim_crc_delim;
                end if;
           
            -------------------------------------------------------------------
            -- CRC Delimiter
            -------------------------------------------------------------------
            when s_pc_crc_delim =>
                if (allow_2bit_crc_delim = '1') then
                    next_state <= s_pc_crc_delim_sec;
                else
                    next_state <= s_pc_ack;
                end if;

            -------------------------------------------------------------------
            -- Secondary CRC Delimiter, or an ACK Slot if DOMINANT.
            -------------------------------------------------------------------
            when s_pc_crc_delim_sec =>
                if (rx_data = DOMINANT) then
                    next_state <= s_pc_ack_sec;
                else
                    next_state <= s_pc_ack;
                end if;

            -------------------------------------------------------------------
            -- ACK Slot
            -------------------------------------------------------------------
            when s_pc_ack =>
                if (allow_2bit_ack = '1') then
                    next_state <= s_pc_ack_sec;
                else
                    next_state <= s_pc_ack_delim;
                end if;

            -------------------------------------------------------------------
            -- Secondary ACK field (in FD Frames),or ACK Delimiter if RECESSIVE
            -------------------------------------------------------------------
            when s_pc_ack_sec =>
                if (rx_data = DOMINANT) then
                    next_state <= s_pc_ack_delim;
                else
                    next_state <= s_pc_eof;
                end if;

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
                    if (rx_data = RECESSIVE) then
                        next_state <= s_pc_itermission;
                    elsif (is_receiver = '1') then
                        next_state <= s_pc_ovr_flag;
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
                    if (rx_data = DOMINANT) then
                        next_state <= s_pc_base_id;
                    elsif (go_to_suspend = '1') then
                        next_state <= s_pc_suspend;
                    elsif (tx_frame_ready = '1') then
                        next_state <= s_pc_sof;
                    else
                        next_state <= s_pc_idle;
                    end if;
                
                -- First or second bit of intermission!
                elsif (rx_data = DOMINANT) then
                    next_state <= s_pc_ovr_flag;
                end if;

            -------------------------------------------------------------------
            -- Suspend transmission
            -------------------------------------------------------------------
            when s_pc_suspend =>
                if (rx_data = DOMINANT) then
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
               -- Dominant during idle is interpreted as SOF!
               if (rx_data = DOMINANT) then
                   next_state <= s_pc_base_id;
               elsif (tx_frame_ready = '1') then
                   next_state <= s_pc_sof;
               end if;

            -------------------------------------------------------------------
            -- Wait till command from User to start re-integration!
            -------------------------------------------------------------------
            when s_pc_reintegrating_wait =>
                if (drv_bus_off_reset = '1') then
                    next_state <= s_pc_reintegrating;    
                end if;

            -------------------------------------------------------------------
            -- Unit is re-integrating, waiting till re-integration counter
            -- expires!
            -------------------------------------------------------------------
            when s_pc_reintegrating =>            
                if (reinteg_ctr_expired = '1') then
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
                if (rx_data = RECESSIVE) then
                    next_state <= s_pc_err_delim;
                end if;
            
            -------------------------------------------------------------------
            -- Error delimiter
            -------------------------------------------------------------------
            when s_pc_err_delim =>
                if (ctrl_ctr_zero = '1') then
                    if (rx_data = DOMINANT) then
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
                if (rx_data = RECESSIVE) then
                    next_state <= s_pc_ovr_delim;
                end if;

            -------------------------------------------------------------------
            -- Overload delimiter
            -------------------------------------------------------------------
            when s_pc_ovr_delim  =>
                if (ctrl_ctr_zero = '1') then
                    if (rx_data = DOMINANT) then
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
    curr_state_proc : process
    begin

        -----------------------------------------------------------------------
        -- Default values
        -----------------------------------------------------------------------
        ctrl_count_pload       <= '0';
        ctrl_count_pload_val   <= (OTHERS => '0');
        ctrl_ctr_ena           <= '0';
        
        -- RX Buffer storing protocol
        store_metadata_d <= '0';
        store_data_d <= '0';
        rec_abort_d <= '0';
        rec_valid_d <= '0';

        sof_pulse <= '0';
        
        -- TXT Buffer HW Commands
        txt_hw_cmd_d.lock    <= '0';
        txt_hw_cmd_d.unlock  <= '0';
        txt_hw_cmd_d.valid   <= '0';
        txt_hw_cmd_d.err     <= '0';
        txt_hw_cmd_d.arbl    <= '0';
        txt_hw_cmd_d.failed  <= '0';

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

        rx_shift_ena            <= '0';
        rx_shift_in_sel         <= '0';
        rx_clear_d              <= '0';

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
        retr_ctr_clear <= '0';
        retr_ctr_add   <= '0';
        is_arbitration <= '0';
        tx_dominant    <= '0';
        crc_check      <= '0';
        
        -- Error signalling
        form_error_i <= '0';
        ack_error_i <= '0';            
        crc_error_i <= '0';
        bit_error_arb_i <= '0';
        bit_err_disable <= '0';
        crc_clear_match_flag <= '0';
        err_pos <= ERC_POS_OTHER;
        
        arbitration_lost <= '0';
        set_transmitter <= '0';
        set_receiver <= '0';
        set_idle <= '0';
        
        sp_control_switch_data <= '0';
        sp_control_switch_nominal <= '0';
        
        -- Secondary sampling point measurement
        ssp_reset_d <= '0';
        trv_delay_calib <= '0';
        
        -- Fault confinement
        primary_error_i <= '0';
        err_delim_late_i <= '0';
        first_err_delim_d <= '0';
        
        br_shifted <= '0';
        ack_received <= '0';

        -- Bit Stuffing control
        stuff_length <= 5;
        fixed_stuff <= '0';
        stuff_enable_set <= '0';
        stuff_enable_clear <= '0';
        
        -- Synchronisation control
        perform_hsync <= '0';

        -- TXT Buffer pointer
        txt_buf_ptr_d <= 0;

        -- CRC control
        crc_enable <= '0';
        crc_spec_enable <= '0';

        if (err_frm_req = '1') then
            ctrl_count_pload_i   <= '1';
            ctrl_count_pload_val <= C_ERR_FLG_DURATION;
            rec_abort_d <= '1';
            
            txt_hw_cmd_d.unlock <= '1';
            retr_ctr_add <= '1';
            crc_clear_match_flag <= '1';
            stuff_enable_clear <= '1';

            if (sp_control_q = DATA_SAMPLE or 
                sp_control_q = SECONDARY_SAMPLE)
            then
                sp_control_switch_nominal <= '1';
                br_shifted <= '1';
            end if;

            if (tx_failed = '1') then
                txt_hw_cmd_d.failed  <= '1';
            else
                txt_hw_cmd_d.err     <= '1';
            end if;

        else
            case curr_state is
    
            -------------------------------------------------------------------
            -- Unit is Off (drv_ena = '0')
            -------------------------------------------------------------------
            when s_pc_off =>
                if (drv_ena = CTU_CAN_ENABLED) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_INTEGRATION_DURATION;
                end if;

            -------------------------------------------------------------------
            -- Unit is integrating (first integration after enabling)
            -------------------------------------------------------------------
            when s_pc_integrating =>
                bit_err_disable <= '1';
                ctrl_ctr_ena <= '1';
                perform_hsync <= '1';
                
                -- Restart integration upon reception of DOMINANT bit!
                if (rx_data = DOMINANT) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_INTEGRATION_DURATION;
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    set_idle <= '1';
                end if;

            -------------------------------------------------------------------
            -- Start of frame
            -------------------------------------------------------------------
            when s_pc_sof =>
                ctrl_count_pload_i <= '1';
                ctrl_count_pload_val <= C_BASE_ID_DURATION;
                tx_load_base_id_i <= '1';
                sof_pulse <= '1';
                tx_dominant <= '1';
                err_pos <= ERC_POS_SOF;
                crc_enable <= '1';
                
                if (rx_data = RECESSIVE) then
                    form_error_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- Base identifier
            -------------------------------------------------------------------
            when s_pc_base_id =>
                bit_err_disable <= '1';
                ctrl_ctr_ena <= '1';
                rx_shift_ena <= '1';
                is_arbitration <= '1';
                tx_shift_ena_i <= '1';
                err_pos <= ERC_POS_ARB;
                crc_enable <= '1';
                
                if (arbitration_lost_condition = '1') then
                    txt_hw_cmd_d.unlock <= '1';
                    retr_ctr_add <= '1';
                    arbitration_lost <= '1';
                    if (tx_failed = '1') then
                        txt_hw_cmd_d.failed  <= '1';
                    else
                        txt_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    rx_store_base_id_i <= '1';
                end if;
                
                if (tx_data = DOMINANT and rx_data = RECESSIVE) then
                    bit_error_arb_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- RTR/SRR/R1 bit. First bit after Base identifier.
            -------------------------------------------------------------------
            when s_pc_rtr_srr_r1 =>
                is_arbitration <= '1';
                bit_err_disable <= '1';
                crc_enable <= '1';
                
                if (arbitration_lost_condition = '1') then
                    txt_hw_cmd_d.unlock <= '1';
                    retr_ctr_add <= '1';
                    arbitration_lost <= '1';
                    if (tx_failed = '1') then
                        txt_hw_cmd_d.failed  <= '1';
                    else
                        txt_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                rx_store_rtr_i <= '1';
                err_pos <= ERC_POS_ARB;
                
                if (is_transmitter = '1' and tran_ident_type = BASE) then
                    if (tran_frame_type = FD_CAN) then
                        tx_dominant <= '1';
                    elsif (tran_is_rtr = NO_RTR) then
                        tx_dominant <= '1';
                    end if;
                end if;
                
                if (tx_data = DOMINANT and rx_data = RECESSIVE) then
                    bit_error_arb_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- IDE bit
            -------------------------------------------------------------------
            when s_pc_ide =>
                if (rx_data = RECESSIVE) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_EXT_ID_DURATION;
                    tx_load_ext_id_i <= '1';
                end if;
                
                if (ide_is_arbitration = '1' and arbitration_lost_condition = '1') then
                    txt_hw_cmd_d.unlock <= '1';
                    retr_ctr_add <= '1';
                    arbitration_lost <= '1';
                    if (tx_failed = '1') then
                        txt_hw_cmd_d.failed  <= '1';
                    else
                        txt_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                rx_store_ide_i <= '1';
                crc_enable <= '1';
                
                if (ide_is_arbitration = '1') then
                    is_arbitration <= '1';
                    bit_err_disable <= '1';
                    
                    if (tx_data = DOMINANT and rx_data = RECESSIVE) then
                        bit_error_arb_i <= '1';
                    end if;
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
                rx_shift_ena <= '1';
                is_arbitration <= '1';
                tx_shift_ena_i  <= '1';
                err_pos <= ERC_POS_ARB;
                bit_err_disable <= '1';
                crc_enable <= '1';
                
                if (arbitration_lost_condition = '1') then
                    txt_hw_cmd_d.unlock <= '1';
                    retr_ctr_add <= '1';
                    arbitration_lost <= '1';
                    if (tx_failed = '1') then
                        txt_hw_cmd_d.failed  <= '1';
                    else
                        txt_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    rx_store_ext_id_i         <= '1';
                end if;

                if (tx_data = DOMINANT and rx_data = RECESSIVE) then
                    bit_error_arb_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- RTR/R1 bit after the Extended identifier
            -------------------------------------------------------------------
            when s_pc_rtr_r1 =>
                is_arbitration <= '1';
                bit_err_disable <= '1';
                crc_enable <= '1';
                
                if (arbitration_lost_condition = '1') then
                    txt_hw_cmd_d.unlock <= '1';
                    retr_ctr_add <= '1';
                    arbitration_lost <= '1';
                    if (tx_failed = '1') then
                        txt_hw_cmd_d.failed  <= '1';
                    else
                        txt_hw_cmd_d.arbl    <= '1';
                    end if;
                end if;
                
                rx_store_rtr_i <= '1';
                err_pos <= ERC_POS_ARB;
                
                if (is_transmitter = '1') then
                    if (tran_frame_type = FD_CAN or tran_ident_type = BASE) then
                        tx_dominant <= '1';
                    end if;
                end if;
                
                if (tx_data = DOMINANT and rx_data = RECESSIVE) then
                    bit_error_arb_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- EDL/r1 bit after RTR/r1 bit in Extended Identifier
            -------------------------------------------------------------------
            when s_pc_edl_r1 =>
                rx_store_edl_i <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                
                if (is_transmitter = '1') then
                    if (tran_frame_type = NORMAL_CAN) then
                        tx_dominant <= '1';
                    else
                        ssp_reset_d <= '1';
                    end if;
                end if;

            -------------------------------------------------------------------
            -- r0 bit after EDL/r1 bit in Extended CAN Frames.
            -------------------------------------------------------------------
            when s_pc_r0_ext =>
                ctrl_count_pload_i <= '1';
                ctrl_count_pload_val <= C_DLC_DURATION;
                tx_load_dlc_i <= '1';
                err_pos <= ERC_POS_CTRL;
                trv_delay_calib <= '1';
                crc_enable <= '1';
                
                if (is_transmitter = '1') then
                    tx_dominant <= '1';
                end if;
                
                -- Here recessive would mean further extending beyond CAN FD
                -- protocol (CAN XL in future). Now we don't have protocol
                -- exception, so we throw error here!
                if (rx_data = RECESSIVE) then
                    form_error_i <= '1';
                end if;

            -------------------------------------------------------------------
            -- r0 bit in CAN FD Frames (both Base and Extended identifier)
            ------------------------------------------------------------------- 
            when s_pc_r0_fd =>
                if (is_transmitter = '1') then
                    tx_dominant <= '1';
                end if;
                
                -- Here recessive would mean further extending beyond CAN FD
                -- protocol (CAN XL in future). Now we don't have protocol
                -- exception, so we throw error here!
                if (rx_data = RECESSIVE) then
                    form_error_i <= '1';
                end if;

                trv_delay_calib <= '1';
                err_pos <= ERC_POS_CTRL;
                perform_hsync <= '1';
                crc_enable <= '1';
                
            -------------------------------------------------------------------
            -- EDL/r0 bit in CAN 2.0 and CAN FD Frames with BASE identifier
            -- only!
            -------------------------------------------------------------------
            when s_pc_edl_r0 =>
                if (rx_data = DOMINANT) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_DLC_DURATION;
                    tx_load_dlc_i <= '1';
                end if;
            
                rx_store_edl_i <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                
                if (is_transmitter = '1' and tran_frame_type = NORMAL_CAN) then
                    tx_dominant <= '1';
                else
                    ssp_reset_d <= '1';
                end if;

            -------------------------------------------------------------------
            -- BRS (Bit rate shift) Bit
            -------------------------------------------------------------------
            when s_pc_brs =>
                rx_store_brs_i <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                
                if (is_transmitter = '1' and tran_brs = BR_NO_SHIFT) then
                    tx_dominant <= '1';
                end if;
                
                if (rx_data = DOMINANT and rx_trigger = '1') then
                    sp_control_switch_data <= '1';
                    br_shifted <= '1';
                end if;
                
            -------------------------------------------------------------------
            -- ESI (Error State Indicator) Bit
            ------------------------------------------------------------------- 
            when s_pc_esi =>
                ctrl_count_pload_i <= '1';
                ctrl_count_pload_val <= C_EXT_ID_DURATION;
                rx_store_esi_i <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                
                if (is_transmitter = '1' and is_err_active = '1') then
                    tx_dominant <= '1';
                end if;

            -------------------------------------------------------------------
            -- DLC (Data length code)
            -------------------------------------------------------------------
            when s_pc_dlc =>
                ctrl_ctr_ena <= '1';
                rx_shift_ena <= '1';
                tx_shift_ena_i  <= '1';
                err_pos <= ERC_POS_CTRL;
                crc_enable <= '1';
                
                -- Address first Data Word in TXT Buffer RAM in advance to
                -- account for DFF delay and RAM delay! Do it only when tran-
                -- smitting to avoid toggling of RAM signals during reception
                -- (possible power consideration)
                if (is_transmitter = '1') then 
                    txt_buf_ptr_d <= 4;
                end if;
                
                if (ctrl_ctr_zero = '1') then
                    ctrl_count_pload_i <= '1';
                    if (no_data_field = '1') then
                        if (drv_fd_type = ISO_FD) then
                            ctrl_count_pload_val <= C_STUFF_COUNT_DURATION;
                            tx_load_stuff_count_i <= '1';
                        else
                            ctrl_count_pload_val <= crc_length_i;
                        end if;
                    else
                        ctrl_count_pload_val <= data_length_c;
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
                rx_shift_ena <= '1';
                rx_shift_in_sel <= '1';
                tx_shift_ena_i <= '1';
                err_pos <= ERC_POS_DATA;
                crc_enable <= '1';
                
                -- Address next word (the one after actually transmitted one),
                -- so that when current word ends, TXT Buffer RAM already
                -- provides data on its output! Counter is divided by 32 since
                -- each memory word contains 32 bits!
                if (is_transmitter = '1') then
                    txt_buf_ptr_d <= to_integer(unsigned(ctrl_ctr_mem_index));
                end if;

                if (ctrl_ctr_zero = '1') then
                    if (drv_fd_type = ISO_FD) then
                        ctrl_count_pload_val <= C_STUFF_COUNT_DURATION;
                        tx_load_stuff_count_i <= '1';
                    else
                        ctrl_count_pload_val <= crc_length_i;
                        tx_load_crc_i <= '1';
                    end if;
                    -- Store data word at the end of data field.
                    store_data_d <= '1';
                end if;

                -- Store data word when multiple of 4 data bytes were counted!
                if (ctrl_counted_byte = '1' and 
                    ctrl_counted_byte_index = "11")
                then
                    store_data_d <= '1';
                    tx_load_data_word_i <= '1';
                end if;
                    
            -------------------------------------------------------------------
            -- Stuff count + Stuff parity field
            -------------------------------------------------------------------
            when s_pc_stuff_count =>
                ctrl_ctr_ena <= '1';
                rx_shift_ena <= '1';
                tx_shift_ena_i <= '1';
                err_pos <= ERC_POS_CRC;
                crc_enable <= '1';
                
                if (ctrl_ctr_zero = '1') then
                    ctrl_count_pload_val <= crc_length_i;
                    tx_load_crc_i <= '1';
                    rx_store_stuff_count_i <= '1';
                end if;
    
                if (is_fd_frame = '1') then
                    stuff_length <= 4;
                    fixed_stuff <= '1';
                end if;

            -------------------------------------------------------------------
            -- CRC field
            -------------------------------------------------------------------
            when s_pc_crc =>
                ctrl_ctr_ena <= '1';
                rx_shift_ena <= '1';
                tx_shift_ena_i <= '1';
                err_pos <= ERC_POS_CRC;

                if (is_fd_frame = '1') then
                    stuff_length <= 4;
                    fixed_stuff <= '1';
                end if;

            -------------------------------------------------------------------
            -- CRC Delimiter
            -------------------------------------------------------------------
            when s_pc_crc_delim =>
                if (is_receiver = '1' and rx_trigger = '1') then
                    crc_check <= '1';
                end if;

                if (rx_data = DOMINANT) then
                    form_error_i <= '1';
                end if;
                err_pos <= ERC_POS_ACK;
                
                if (sp_control_q = DATA_SAMPLE or 
                    sp_control_q = SECONDARY_SAMPLE)
                then
                    sp_control_switch_nominal <= '1';
                    br_shifted <= '1';
                end if;
                stuff_enable_clear <= '1';
                
            -------------------------------------------------------------------
            -- Secondary CRC Delimiter, or an ACK Slot if DOMINANT.
            -------------------------------------------------------------------
            when s_pc_crc_delim_sec =>
                err_pos <= ERC_POS_ACK;

            -------------------------------------------------------------------
            -- ACK Slot, or a ACK delim, if previous two bits were recessive!
            -------------------------------------------------------------------
            when s_pc_ack =>
                if (is_receiver = '1' and crc_match = '1' and
                    drv_ack_forb = '0')
                then
                    tx_dominant <= '1';
                end if;
                
                if (is_transmitter = '1' and drv_self_test_ena = '0' and
                    rx_data = RECESSIVE)
                then
                    ack_error_i <= '1';
                end if;
                err_pos <= ERC_POS_ACK;
                
                if (rx_data = DOMINANT) then
                    ack_received <= '1';
                end if;
                
                -- If Transmitter sends ACK, receiving dominant is not bit
                -- error!
                if (is_transmitter = '1') then
                    bit_err_disable <= '1';
                end if;

            -------------------------------------------------------------------
            -- Secondary ACK field (in FD Frames),or ACK Delimiter if RECESSIVE
            -------------------------------------------------------------------
            when s_pc_ack_sec =>

                if (rx_data = RECESSIVE) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_EOF_DURATION;
                end if;

                if (is_receiver = '1' and crc_match = '0') then
                    crc_error_i <= '1';
                end if;
                err_pos <= ERC_POS_ACK;

                if (rx_data = DOMINANT) then
                    ack_received <= '1';
                end if;
                
                -- If Transmitter sends ACK, receiving dominant is not bit
                -- error!
                if (is_transmitter = '1') then
                    bit_err_disable <= '1';
                end if;

            -------------------------------------------------------------------
            -- ACK Delimiter
            -------------------------------------------------------------------
            when s_pc_ack_delim =>
                ctrl_count_pload <= '1';
                ctrl_count_pload_val <= C_EOF_DURATION;
                
                if (rx_data = DOMINANT) then
                    form_error_i <= '1';
                end if;
                
                if (is_receiver = '1' and crc_match = '0') then
                    crc_error_i <= '1';
                end if;
                err_pos <= ERC_POS_ACK;
    
            -------------------------------------------------------------------
            -- End of Frame. Receiver sampling DOMINANT in last bit interprets
            -- this as Overload flag!
            -------------------------------------------------------------------
            when s_pc_eof =>
                ctrl_ctr_ena <= '1';
                if (ctrl_ctr_zero = '1') then
                    ctrl_count_pload_i <= '1';
                    if (rx_data = RECESSIVE) then
                        ctrl_count_pload_val <= C_INTERMISSION_DURATION;
                        
                        -- No Error until the end of EOF means frame is valid
                        -- for transmitter!
                        if (is_transmitter = '1') then
                            txt_hw_cmd_d.unlock <= '1';
                            txt_hw_cmd_d.valid  <= '1';
                            retr_ctr_clear <= '1';
                        end if;
                        
                    elsif (is_receiver = '1') then
                        ctrl_count_pload_val <= C_OVR_FLG_DURATION;
                    end if;
                    
                    crc_clear_match_flag <= '1';
                end if;

                -- If there is no error (RX Recessive) in one bit before end
                -- of EOF, signal valid Frame reception!
                if (ctrl_ctr_one = '1' and rx_data = RECESSIVE) then
                    rec_valid_d <= '1';
                end if;
                
                -- DOMINANT during EOF (apart from last bit) means error!
                if (rx_data = DOMINANT and ctrl_ctr_zero = '0') then
                    form_error_i <= '1';
                end if;
                err_pos <= ERC_POS_INTF;
    
            -------------------------------------------------------------------
            -- Intermission field
            -------------------------------------------------------------------
            when s_pc_intermission =>
                ctrl_ctr_ena <= '1';
                
                -- Address Identifier Word in TXT Buffer RAM in advance to
                -- account for DFF delay and RAM delay! 
                txt_buf_ptr_d <= 1;

                -- Last (third) bit of intermission
                if (ctrl_ctr_zero = '1') then
                    ctrl_count_pload_i <= '1';
                    crc_spec_enable <= '1';
                    
                    -- Here FSM goes to Base ID (sampling of DOMINANT in the
                    -- third bit of intermission)!
                    if (rx_data = DOMINANT) then
                        ctrl_count_pload_val <= C_BASE_ID_DURATION;
                        tx_load_base_id_i <= '1';
                        sof_pulse <= '1';
                        
                    -- Here FSM goes to either IDLE, Suspend, or to SOF, when
                    -- it has sth. to transmitt. We preload SUSPEND length in
                    -- any case, since other states don't care about control
                    -- counter.
                    else
                        ctrl_count_pload_val <= C_SUSPEND_DURATION;
                    end if;

                    -- Lock Buffer when there is what to transmitt, and no
                    -- suspend! Unit becomes transmitter! If not, and DOMINANT
                    -- is received, become receiver! 
                    if (tx_frame_ready = '1' and go_to_suspend = '0') then
                        txt_hw_cmd_d.lock <= '1';
                        set_transmitter   <= '1';
                    elsif (rx_data = DOMINANT) then
                        set_receiver   <= '1';
                    end if;
                    
                    -- Transmission/reception started -> Enable Bit stuffing!
                    -- Clear RX Shift Register!
                    if (frame_start = '1') then
                        stuff_enable_set <= '1';
                        rx_clear_d <= '1';
                    end if;
    
                -- First or second bit of intermission!
                elsif (rx_data = DOMINANT) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_OVR_FLG_DURATION;
                end if;
                
                -- Second or third bit of intermission, Hard Synchronisation
                if (ctrl_ctr_zero = '1' or ctrl_ctr_one = '1') then
                    perform_hsync <= '1';
                end if;
    
            -------------------------------------------------------------------
            -- Suspend transmission
            -------------------------------------------------------------------
            when s_pc_suspend =>
                ctrl_ctr_ena <= '1';
                perform_hsync <= '1';
                crc_spec_enable <= '1';
                
                -- Address Identifier Word in TXT Buffer RAM in advance to
                -- account for DFF delay and RAM delay! 
                txt_buf_ptr_d <= 1;
                
                if (rx_data = DOMINANT) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_BASE_ID_DURATION;
                    tx_load_base_id_i <= '1';
                    sof_pulse <= '1';
                    set_receiver <= '1';
                    stuff_enable_set <= '1';
                    rx_clear_d <= '1';

                -- End of Suspend -> Unit goes to IDLE!
                elsif (ctrl_ctr_zero = '1') then
                    set_idle <= '1';
                end if;
    
            -------------------------------------------------------------------
            -- Unit is in Bus idle period.
            -------------------------------------------------------------------
            when s_pc_idle =>
                perform_hsync <= '1';
                crc_spec_enable <= '1';
                                
                -- Address Identifier Word in TXT Buffer RAM in advance to
                -- account for DFF delay and RAM delay! 
                txt_buf_ptr_d <= 1;
                
                if (rx_data = DOMINANT) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_BASE_ID_DURATION;
                    tx_load_base_id_i <= '1';
                    sof_pulse <= '1';
                    crc_enable <= '1';
                end if;

                if (tx_frame_ready = '1') then
                    txt_hw_cmd_d.lock <= '1';
                    set_transmitter <= '1';
                elsif (rx_data = DOMINANT) then
                    set_receiver <= '1';
                end if;

                -- Transmission/reception started -> Enable Bit stuffing!
                -- Clear RX Shift register!
                if (frame_start = '1') then
                    stuff_enable_set <= '1';
                    rx_clear_d <= '1';
                end if;

            -------------------------------------------------------------------
            -- Wait till command from User to start re-integration!
            -------------------------------------------------------------------
            when s_pc_reintegrating_wait =>
                bit_err_disable <= '1';
                if (drv_bus_off_reset = '1') then
                    ctrl_count_pload_i <= '1';
                    reinteg_ctr_clr    <= '1';
                    ctrl_count_pload_val <= C_INTEGRATION_DURATION;
                end if;
                    
            -------------------------------------------------------------------
            -- Unit is re-integrating, waiting till re-integration counter
            -- expires!
            -------------------------------------------------------------------
            when s_pc_reintegrating =>
                ctrl_ctr_ena <= '1';
                reinteg_ctr_enable <= '1';
                perform_hsync <= '1';
                bit_err_disable <= '1';
                
                if (ctrl_ctr_zero = '1' and reinteg_ctr_expired = '0') then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_INTEGRATION_DURATION;    
                end if;

                if (reinteg_ctr_expired = '1') then
                    set_idle <= '1';
                end if;

            -------------------------------------------------------------------
            -- Active error flag.
            -------------------------------------------------------------------
            when s_pc_act_err_flag =>
                ctrl_ctr_ena <= '1';
                if (ctrl_ctr_zero = '1') then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_DELIM_WAIT_DURATION;
                    first_err_delim_d <= '1';
                end if;
                
                tx_dominant <= '1';
                
                if (rx_data = RECESSIVE) then
                    form_error_i <= '1';
                end if;
                err_pos <= ERC_POS_ERR;

            -------------------------------------------------------------------
            -- Passive error flag.
            -------------------------------------------------------------------
            when s_pc_pas_err_flag =>
                ctrl_ctr_ena <= '1';
                if (ctrl_ctr_zero = '1') then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_DELIM_WAIT_DURATION;
                    first_err_delim_d <= '1';
                end if;
                err_pos <= ERC_POS_ERR;
                
                -- Node sending Passive error flag may receive RECESSIVE or
                -- DOMINANT, and DOMINANT shall not be treated as bit error!
                bit_err_disable <= '1';

            -------------------------------------------------------------------
            -- Wait till Error delimiter (detection of recessive bit)
            -------------------------------------------------------------------
            when s_pc_err_delim_wait =>
                if (ctrl_ctr_zero = '0') then
                    ctrl_ctr_ena <= '1';
                end if;

                if (rx_data = RECESSIVE) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_ERR_DELIM_DURATION;
                
                -- Err flag (6 bits) + Delimiter wait (7 bits) elapsed and
                -- dominant is detected, this signals 14 consecutive dominant
                -- bits!
                elsif (ctrl_ctr_zero = '1') then
                    err_delim_late_i <= '1';
                end if;
                err_pos <= ERC_POS_ERR;

                -- When waiting for RECESSIVE bit after Error delimiter, unit
                -- may receive DOMINANT and not interpret this as Bit error!
                bit_err_disable <= '1';

                -- Node received dominant bit as first bit after Error flag!
                -- This shall be treated as primamry error
                if (rx_data = DOMINANT and first_err_delim_q = '1') then
                    primary_error_i <= '1';
                    first_err_delim_d <= '0';
                end if;
                
            -------------------------------------------------------------------
            -- Error delimiter
            -------------------------------------------------------------------
            when s_pc_err_delim =>
                ctrl_ctr_ena <= '1';
                if (ctrl_ctr_zero = '1') then
                    ctrl_count_pload_i <= '1';
                    if (rx_data = DOMINANT) then
                        ctrl_count_pload_val <= C_OVR_FLG_DURATION;
                    else
                        ctrl_count_pload_val <= C_INTERMISSION_DURATION;
                    end if;
                end if;

                if (rx_data = DOMINANT) then
                    form_error_i <= '1';
                end if;
                err_pos <= ERC_POS_ERR;

            -------------------------------------------------------------------
            -- Overload flag
            -------------------------------------------------------------------
            when s_pc_ovr_flag =>
                ctrl_ctr_ena <= '1';
                tx_dominant <= '1';
                err_pos <= ERC_POS_OVRL;
                
                if (rx_data = RECESSIVE) then
                    form_error_i <= '1';
                end if;
                
            -------------------------------------------------------------------
            -- Wait till overload delimiter.
            -------------------------------------------------------------------
            when s_pc_ovr_delim_wait =>
                err_pos <= ERC_POS_OVRL;
                if (rx_data = RECESSIVE) then
                    ctrl_count_pload_i <= '1';
                    ctrl_count_pload_val <= C_OVR_DELIM_DURATION;
                end if;

            -------------------------------------------------------------------
            -- Overload delimiter
            -------------------------------------------------------------------
            when s_pc_ovr_delim  =>
                ctrl_ctr_ena <= '1';
                if (ctrl_ctr_zero = '1') then
                    ctrl_count_pload_i <= '1';
                    if (rx_data = DOMINANT) then
                        ctrl_count_pload_val <= C_OVR_FLG_DURATION;
                    else
                        ctrl_count_pload_val <= C_INTERMISSION_DURATION;
                    end if;
                end if;
                
                if (rx_data = DOMINANT) then
                    form_error_i <= '1';
                end if;
                err_pos <= ERC_POS_OVRL;

            end case;
        end if;

    end process;

    -----------------------------------------------------------------------
    -- FSM State register
    -----------------------------------------------------------------------
    state_reg_ce <= '1' when (next_state /= curr_state and
                              (rx_trigger = '1' or drv_ena = '0' or
                               err_frm_req = '1'))
                        else
                    '0';

    fsm_state_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
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
    -----------------------------------------------------------------------
    ctrl_count_pload <= ctrl_count_pload_i when (curr_state = s_pc_off) else
                        ctrl_count_pload_i when (rx_trigger = '1') else
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
        if (res_n = G_RESET_POLARITY) then
            store_metadata     <= '0';
            store_data         <= '0';
            rec_valid          <= '0';
            rec_abort          <= '0';
        elsif (rising_edge(clk_sys)) then
            
            -- Frame is stored to RX Buffer when unit is either receiver
            -- or self test mode is enabled (loopback).
            if (is_receiver = '1' or drv_self_test_ena = '1') then
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
    
    -----------------------------------------------------------------------
    -- TXT Buffer HW commands pipeline
    -----------------------------------------------------------------------
    txt_hw_cmd_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            txt_hw_cmd_q.lock    <= '0';
            txt_hw_cmd_q.unlock  <= '0';
            txt_hw_cmd_q.valid   <= '0';
            txt_hw_cmd_q.err     <= '0';
            txt_hw_cmd_q.arbl    <= '0';
            txt_hw_cmd_q.failed  <= '0';
        elsif (rising_edge(clk_sys)) then
            txt_hw_cmd_q <= txt_hw_cmd_d;
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
    -- RX Shift regsiter clear
    -----------------------------------------------------------------------
    rx_shift_reg_clear_inst : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,
        input              => rx_clear_d,
        load               => '1',
        output             => rx_clear_q
    );


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
    form_error <= form_error_i and rx_trigger;
    ack_error <= ack_error_i and rx_trigger; 
    crc_error <= crc_error_i and rx_trigger;
    bit_error_arb <= bit_error_arb_i and rx_trigger;

    -----------------------------------------------------------------------
    -- Switching of Bit-rate
    -----------------------------------------------------------------------
    sp_control_d <= NOMINAL_SAMPLE when (sp_control_switch_nominal = '1') else
                    DATA_SAMPLE when (sp_control_switch_data = '1' and
                                      is_receiver = '1')
                                else
                    SECONDARY_SAMPLE when (sp_control_switch_data = '1') else
                    sp_control_q;

    sp_control_ce <= '1' when (sp_control_switch_nominal = '1') else
                     '1' when (sp_control_switch_data = '1') else
                     '0';

    sp_control_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            sp_control_q <= NOMINAL_SAMPLE;
        elsif (rising_edge(clk_sys)) then
            if (sp_control_ce = '1') then
                sp_control_q <= sp_control_d;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------
    -- Secondary sampling point reset
    -----------------------------------------------------------------------
    dff_arst_ssp_reset_inst : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,
        input              => ssp_reset_d,
        load               => '1',
        output             => ssp_reset_q
    );
    
    ---------------------------------------------------------------------------
    -- Indicates that Error or Overload flag is being transmitted! Can't
    -- be part of current state, since it must be valid also during
    -- error condition to distiguish error during error flag!
    ---------------------------------------------------------------------------
    act_err_ovr_flag <= '1' when (curr_state = s_pc_act_err_flag) else
                        '1' when (curr_state = s_pc_pas_err_flag) else
                        '1' when (curr_state = s_pc_ovr_flag) else
                        '0';

    first_err_delim_flag_reg : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
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
    primary_error <= '1' when (primary_error_i = '1' and rx_trigger = '1')
                         else
                     '0';

    err_delim_late <= '1' when (err_delim_late_i = '1' and rx_trigger = '1')
                          else
                      '0';

    -- No positive resynchronisation for transmitter of dominant bit!
    no_pos_resync <= '1' when (is_transmitter = '1' and tx_data = DOMINANT)
                         else
                     '0';

    ---------------------------------------------------------------------------
    -- Bit Stuffing / Destuffing enable
    ---------------------------------------------------------------------------
    stuff_ena_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            stuff_enable <= '0';
            stuff_error_enable <= '0';
        elsif (rising_edge(clk_sys)) then
            if (stuff_enable_set = '1') then
               stuff_enable <= '1';
               stuff_error_enable <= '1';
            elsif (stuff_enable_clear = '1') then
               stuff_enable <= '0';
               stuff_error_enable <= '0';
           end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Synchronisation type
    ---------------------------------------------------------------------------
    sync_control_d <= NO_SYNC when (is_transmitter = '1' and
                                    tran_frame_type ='1' and
                                    sp_control_q = SECONDARY_SAMPLE)
                              else
                    HARD_SYNC when (perform_hsync = '1')
                              else
                      RE_SYNC;
    
    sync_control_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            sync_control_q <= HARD_SYNC;
        elsif (rising_edge(clk_sys)) then
            sync_control_q <= sync_control_d;
        end if;
    end process;
    
    -----------------------------------------------------------------------
    -- TXT Buffer pointer registering
    -----------------------------------------------------------------------
    txt_buf_ptr_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            txt_buf_ptr_q <= 0;
        elsif (rising_edge(clk_sys)) then
            txt_buf_ptr_q <= txt_buf_ptr_d;
        end if;
    end process;

    -----------------------------------------------------------------------
    -- Internal signals to output propagation
    -----------------------------------------------------------------------
    crc_src <= crc_src_i;
    txt_hw_cmd <= txt_hw_cmd_q;
    sp_control <= sp_control_q;
    tran_valid <= txt_hw_cmd_q.valid;
    ssp_reset <= ssp_reset_q; 
    pc_state <= curr_state;
    rx_clear <= rx_clear_q;
    sync_control <= sync_control_q;
    
    bit_error_enable <= not bit_err_disable;

end architecture;