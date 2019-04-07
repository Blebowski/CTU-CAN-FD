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
        
        -- Error condition (invokes transmission of Error frame)
        error_condition         :in   std_logic;
        
        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- CTU CAN FD is enabled
        drv_ena                 :in   std_logic;
        
        -- CAN FD type (ISO / NON-ISO)
        drv_fd_type             :in   std_logic;
        
        -- Command to start re-integration in Bus-off
        drv_bus_off_reset       :in   std_logic;

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
        store_data_word         :out  std_logic;
        
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
        
        -- TX Data length code
        tran_dlc                :in   std_logic_vector(3 downto 0);
        
        -- TX Remote transmission request flag
        tran_is_rtr             :in   std_logic;
        
        -- TX Frame type (0-CAN 2.0, 1-CAN FD)
        tran_frame_type         :in   std_logic;
        
        -----------------------------------------------------------------------
        -- TX Shift register interface
        -----------------------------------------------------------------------
        -- Load Identifier to TX Shift register
        tx_load_id              :out  std_logic;
        
        -- Load Data word to TX Shift register
        tx_load_data_word       :out  std_logic;
        
        -- Load CRC to TX Shift register
        tx_load_crc             :out  std_logic;
        
        -----------------------------------------------------------------------
        -- RX Shift register interface
        -----------------------------------------------------------------------
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
        rec_dlc_comb            :in   std_logic_vector(3 downto 0);
        
        -- RX frame type (0-CAN 2.0, 1- CAN FD)
        rec_frame_type          :in   std_logic;
        
        -----------------------------------------------------------------------
        -- CRC Comparator / CAN CRC interface
        -----------------------------------------------------------------------
        -- Perform CRC check
        crc_eval                :out  std_logic;
        
        -- CRC and Stuff count are valid
        crc_valid               :in   std_logic;
        
        -- CRC calculation enabled
        crc_enable              :out  std_logic;
        
        -----------------------------------------------------------------------
        -- Control counter interface
        -----------------------------------------------------------------------
        -- Preload control counter
        ctrl_count_pload        :out   std_logic;
        
        -- Control counter preload value
        ctrl_count_pload_val    :out   std_logic_vector(8 downto 0);
        
        -- Control counter is zero
        ctrl_count_zero         :in    std_logic;
        
        -- Control counter is equal to 1
        ctrl_ctr_one            :in    std_logic;
        
        -- Control counter is divisible by 8 (aligned to byte)
        ctrl_ctr_mod_8          :in    std_logic;
        
        -----------------------------------------------------------------------
        -- Reintegration counter interface
        -----------------------------------------------------------------------
        -- Reintegration counter Clear (synchronous)
        reinteg_ctr_clr         :out   std_logic;

        -- Enable counting (with RX Trigger)
        reinteg_ctr_enable      :out  std_logic;
        
        -- Reintegration counter expired (reached 128)
        reinteg_ctr_expired     :in   std_logic;
        
        -----------------------------------------------------------------------
        -- Arbitrator interface
        -----------------------------------------------------------------------
        -- Protocol control is in Arbitration field.
        is_arbitation           :out   std_logic;
        
        -----------------------------------------------------------------------
        -- Error detector interface
        -----------------------------------------------------------------------
        -- Form Error has occurred
        form_error              :out   std_logic;
        
        -- ACK Error has occurred
        ack_error               :out   std_logic;
        
        -- Perform CRC check
        crc_check               :out   std_logic;
        
        -- Clear CRC Error flag
        crc_clear_error_flag    :out   std_logic;        
        
        -- CRC Source (CRC15, CRC17, CRC21)
        crc_src                 :out   std_logic_vector(1 downto 0);
        
        -- Error position field (from Protocol control)
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
        -- Resynchronisation
        sync_control            :out   std_logic_vector(1 downto 0);
        
        -- Clear the Shift register for secondary sampling point.
        ssp_reset               :out   std_logic;
        
        -- Enable measurement of Transciever delay
        trv_delay_calib         :out   std_logic;
        
        -- Protocol control FSM state output
        PC_State_out            :out   protocol_type;
        
        -- Transmitted frame is valid
        tran_valid              :out   std_logic;
        
        -- ACK received
        ack_recieved_out        :out   std_logic;
        
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

begin

    no_data_transmitter <= '1' when (tran_dlc = "0000" or tran_is_rtr = RTR_FRAME)
                               else
                           '0';

    no_data_receiver <= '1' when (rec_is_rtr = RTR_FRAME or rec_dlc_comb = "0000")
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

    ---------------------------------------------------------------------------
    -- Next state process
    ---------------------------------------------------------------------------
    next_state_proc : process
    begin
        next_state <= curr_state;

        if (drv_ena = CTU_CAN_DISABLED) then
            next_state <= s_pc_off;
            
        elsif (error_condition = '1') then
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
                if (ctrl_count_zero = '1') then
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
                if (ctrl_count_zero = '1') then
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
                if (ctrl_count_zero = '1') then    
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
                if (ctrl_count_zero = '1') then 
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
                if (ctrl_count_zero = '1') then
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
                if (ctrl_count_zero = '1') then
                    next_state <= s_pc_crc;
                end if;

            -------------------------------------------------------------------
            -- CRC field
            -------------------------------------------------------------------
            when s_pc_crc =>
                if (ctrl_count_zero = '1') then
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
                if (ctrl_count_zero = '1') then
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
                elsif (ctrl_count_zero = '1') then
                    if (rx_data = DOMINANT) then
                        next_state <= s_pc_base_id;
                    elsif (tran_frame_valid = '1') then
                        next_state <= s_pc_sof;
                    elsif (is_err_passive = '1' and is_transmitter = '1') then
                        next_state <= s_pc_suspend;
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
                elsif (ctrl_count_zero = '1') then
                    -- Start transmission after suspend if we have what to
                    -- transmitt!
                    if (tran_frame_valid = '1') then
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
               elsif (tran_frame_valid = '1') then
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
                if (ctrl_count_zero = '1') then
                    next_state <= s_pc_err_delim_wait;
                end if;
            
            -------------------------------------------------------------------
            -- Passive error flag.
            -------------------------------------------------------------------
            when s_pc_pas_err_flag =>
                if (ctrl_count_zero = '1') then
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
                if (ctrl_count_zero = '1') then
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
                if (ctrl_count_zero = '1') then
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
                if (ctrl_count_zero = '1') then
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
    curr_state_proc : process()
    begin
        
        
    end process;

end architecture;