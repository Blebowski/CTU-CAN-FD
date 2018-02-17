--------------------------------------------------------------------------------
-- 
-- CAN with Flexible Data-Rate IP Core 
-- 
-- Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisor: Jiri Novak <jnovak@fel.cvut.cz>
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
--  Error detection and signalling. Logic for error counters incrementing imple-
--  mented. Possible to forbid accepting of FD Frames for reciever. ESD bit de-
-- tected as bit error then and error_flag is sent.
--------------------------------------------------------------------------------
-- Revision History:
--
--    July 2015   Created file
--    19.12.2015  Added enable disable for whole controller. Default state chan-
--                ged from interframe to off. ISO CAN FD still needs to be im-
--                plemented
--    17.1.2016   Added permanent PC_State stuck to "off" while drv_ena=1. Whole
--                controller is disabled
--    21.5.2016   Added crc_state substate to cover ISO FD functionality. CRC 
--                state now has two substates stuff_count and real_crc. Stuff 
--                count represents added field in ISO protocol. Number of de-
--                stuffed bits is provided to protocolControl by bit Destuffing
--                circuit. Additionaly parity decoder and Grey decoder added.
--    26.5.2016   drv_fd_type signal connected to the CAN protocol. Stuff bit 
--                count field of ISO FD CAN is now transmitted only when ISO 
--                option is selected!! 
--    14.6.2016   ack_recieved register reset to 0 in SOF. No function change, 
--                but more clear behaviour
--    21.6.2016   1.Arbitration_lost pulled low every cycle except setting in 
--	                Arbitration field. Before arbitration_lost always kept its 
--	                previous value! Due to that when arbitration was lost in last
--                  possible bit (same identifier BASE vs EXTENDED), then 
--                  "arbitration_lost" remained set for whole duration of 
--                  transmittion!!!
--                2.Added delay_control_trans register as bugfix. When arbitra-
--                  tion was lost in last bit as described in previous case, 
--                  OP_State did not manage to be acutalize and thus FSMpreset 
--                  branch was executed for transmitter! THus control_pointer 
--                  was set totally wrong, and reciever was confused... Error 
--                  frame was later on detected OK. This error in some cases 
--                  behaved just like CRC error! Now transition from arbitration
--                  to control is always done one clock cycle later than imme-
--                  diately after rec_trig! It is OK, there is plenty of time, 
--                  since we are still in NOMINAL bit time at this point! 
--                  delay_control_trans register is used for this delay!
--    22.6.2016   Bug fix. Added detection of recieved RTR and recived frame 
--                type to setting dlc_int register. Previous behaviour caused 
--                that when RTR frame with DLC e.g. 12 was transmitted (no data
--                field but, DLC =12, as special feature of RTR preffered beha-
--                viour), then reciever accepted this dlc and ignored the fact 
--                there is going to be no data phase. CRC length decision was
--                then made based on this recieved value and reciever did expect
--                longer CRC field (17 or 21) than there actually was! Thus it 
--                did not send the acknowledge and error ocurred!
--    23.6.2016   1. Added is_idle_r<='1' when transmittion is aborted. OP State
--                   should be immediately idle not after the end of the inter-
--                   frame space, since transmittion was aborted and node shhould
--                   be now as if there was no activity one the bus for long 
--                   time...
--                2. RTR prefered behaviour bug fix. Active bit was flippped. 
--                   Correctly active in logic 1
--    24.6.2016   Bug fix in self test mode. Reciever did not check 
--                drv_self_test_ena='1' before going to error counter. Thus 
--                transmitter did accept the frame without ACK in self test mode
--                , but reciever did not!
--    27.6.2016   Changed handling of secondary sampling point reset. ssp_reset
--                pulled inactive permanently instead of moments where it is 
--                reset. Since ssp_reset achieves reset in one clock cycle,
--                it is more clear behaviour to only reset SSP register at BRS,
--                instead of holding it active for long time before.
--    28.6.2016   Added set_reciever signal activation in suspend transmittion 
--                field in interframe state. Now if DOMINANT bit is detected unit
--                immediately turns reciever of the frame instead of only going
--                to SOF. Before it only went to SOF tranmsmitted one bit (SOF)
--                and turned reciever. We dont want recieving unit transmitt
--                anyhing. Not even SOF!
--    30.6.2016   1.Corrected Overload detection in interframe intermission! 
--                  Until now only synchronization edge was able to cause over-
--                  load frame or SOF! This covers the option of immediate start
--                  of the SOF field at any part of bit time. Second option added
--                  with recieve trigger! Without this fix some overload condi-
--                  tions might not have been detected or even SOFs missed!!
--                2.Bit error detection during sending active error flag fixed.
--                  Before only reciever detected the bit error during active 
--                  error flag!
--     5.7.2016   Code formatting and replacement of some literals by constants 
--                for easier readability!
--    12.7.2016   Fixed ack error detection. CRC_error signal was activated 
--                instead of ack_error signal
--    18.7.2016   1. crc_check signal erased in SOF
--                2. Added crc_check to the condition of starting Error frame 
--                    after Delim_ACK! Until now only received ACK was monitored!
--                    This was wrong behaviour. If CRC check fails ERROR frame
--                    has to start even if other node CRC Check was ok and ack-
--                    nowledge was recieved!
--    1.8.2016    Bug fix. sync_type changed to RE_SYNC in the sample point of 
--                SOF not in the beginning. Other wise edge did not arrive yet 
--                and no hard synchronization could have happened!
--    12.1.2017   1. Added CRC fix for ISO FD CAN. CRC was stopped before the 
--                   stuff count field. Due to this Stuff count was not included
--                   into CRC which made the calculated CRC always wrong!
--                2. Fixed CRC length for small FD frames to be always 17 
--                   instead of 15!
--    29.11.2017  1. Optimized storing of received data. Data stored into 16*32 
--                   RAM (array) after each byte was received. Since RX Buffer
--                   is reading the data serially, it does not need to have
--                   the data available in parallel! Removed signal "rec_data_r"
--                   and replaced it with "rec_dram". RX buffer now provides 
--                   address signal which combinationally reads the data on RAM 
--                   output!
--                   This approach saved approx. 1000 LC combinationals of Altera
--                   device. No RAM was inferred, and the memory was stored in 
--                   LUT combinational memory! An additional effect of this 
--                   change is that Received Data are not erased in the SOF of 
--                   next frame and thus it stays on the output of CAN Core until
--                   it is rewritten by next data.
--    4.12.2017   Added support for addressing of transmitted data directly from
--                TXT buffer with "txt_buf_ptr", instead of fetching data from 
--                "Tran Buffer" in CAN Core.
--    9.12.2017   1. Change reception of CRC from direct addressing to shift re-
--                   gister. Saved approx. 60 LUTs.
--                2. Split the "rec_ident_in" into two separate shift registers
--                   "rec_ident_base_sr" and "rec_ident_ext_sr". Base and exten-
--                   ded identifiers are not addressed by "tran_pointer" anymore
--                   but received in shift registers. The output value is com-
--                   bined from these two shift registers, thus interface to RX
--                   buffer remained unchanged! Saved approx 100 LUTs.
--   27.12.2017   Added "tran_lock", "tran_unlock", "tran_drop" signals for
--                implementation of frame swapping feature. Replaced 
--                "tran_data_ack" with "tran_lock" signal.
--   15.02.2018   1. Removed "tran_lock", "tran_unlock" and "tran_drop" signals
--                   and replaced them with "txt_hw_cmd" record signal
--                2. Removed "rettransmitt" signal. It is not needed anymore.
--                   Since the Core is now transmitting from the TXT Buffer
--                   directly, the core will unlock the buffer at arbitration
--                   lost or error frame. Thus in intermission idle, the Core 
--                   will automatically start rettransmitting, since it will
--                   have valid frame signalled by "tran_frame_valid_in"!
--                3. If different buffer is decided for transmission, 
--                   "txt_buf_changed" signal will be active. This signal is
--                   implemented to be valid in the same clock as "tran_frame_
--                   valid_in", and thus when it is sampled, "txt_buf_changed"
--                   is used to find out if "retr_counter" should be erased.
--                4. Added bugfix. If frame is locked for transmission from
--                   tran_frame_valid, it must be locked at the same clock
--                   cycle as "tran_frame_valid_in" is active. Since SW commands
--                   are introduced to the TXT Buffers, one can no longer rely
--                   on transiting to SOF from BUS IDLE and locking the frame
--                   only then! "is_txt_locked" signal is introduced, to not
--                   perform additional locking in SOF if lock was already per-
--                   formed in BUS IDLE.
--   17.02.2018   Removed obsolete "frame_store", its functionality is fully
--                replaced with frame_lock
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CAN_FD_frame_format.all;
use work.CAN_FD_register_map.all;

entity protocolControl is
  port(
    -------------------
    --Clock and reset--
    -------------------
    signal clk_sys                :in   std_logic; --System clock
    signal res_n                  :in   std_logic; --Async reset
    
    --Driving bus signals
    signal drv_bus                :in   std_logic_vector(1023 downto 0);
    
    -------------------------------
    --Transcieve buffer interface--
    -------------------------------
    signal tran_data              :in   std_logic_vector(31 downto 0);
    signal tran_ident_base        :in   std_logic_vector(10 downto 0);
    signal tran_ident_ext         :in   std_logic_vector(17 downto 0);
    signal tran_dlc               :in   std_logic_vector(3 downto 0);
    signal tran_is_rtr            :in   std_logic;
    signal tran_ident_type        :in   std_logic;
    signal tran_frame_type        :in   std_logic;
    signal tran_brs               :in   std_logic; 
    
    --Valid frame ready to be stored into Transcieeve Buffer
    signal tran_frame_valid_in    :in   std_logic;
    
    -- Commands for TX Arbitrator and TXT Buffers signalling
    -- locking, unlocking and course of the transmission
    signal txt_hw_cmd             :out  txt_hw_cmd_type;
    
    --Pointer to TXT buffer memory
    signal txt_buf_ptr            :out  natural range 0 to 15;
    
    signal txtb_changed           :in std_logic;
    
    -------------------------
    --Recieved data output --
    -------------------------
    signal rec_ident              :out  std_logic_vector(28 downto 0);
    signal rec_dlc                :out  std_logic_vector(3 downto 0);
    signal rec_is_rtr             :out  std_logic;
    signal rec_ident_type         :out  std_logic;
    signal rec_frame_type         :out  std_logic;
    signal rec_brs                :out  std_logic;
    signal rec_crc                :out  std_logic_vector(20 downto 0);
    signal rec_esi                :out  std_logic;
    
    --Added interface for aux SRAM
    signal rec_dram_word          :out  std_logic_vector(31 downto 0);
    signal rec_dram_addr          :in   natural range 0 to 15;
    
    --------------------------------
    --Operation mode FSM Interface--
    --------------------------------
    
    --Operation mode state
    signal OP_state               :in   oper_mode_type;
    
    --Signal for Operational mode state mahine about loosing arbitration
    signal arbitration_lost       :out  std_logic;
    
    --Indicates transcieve or recieve finished and bus is idle
    signal is_idle                :out  std_logic;
    
    --Set OP_State FSM into transciever state (Used at SOF)
    signal set_transciever        :out  std_logic;
    
    --Set OP_State FSM into reciever state
    signal set_reciever           :out  std_logic;
    
    --Arbitration lost capture
    signal alc                    :out  std_logic_vector(4 downto 0);
    
    -------------------------------
    --Fault confinement Interface--
    -------------------------------
    --Fault confinement state
    signal error_state            :in   error_state_type;
    
    --Error signals for fault confinement
    signal form_Error             :out  std_logic; --Form Error
    signal CRC_Error              :out  std_logic; --CRC Error
    signal ack_Error              :out  std_logic; --Acknowledge error
    
    --Some of the state machines, 
    --or signals reached unknown state!!
    signal unknown_state_Error    :out  std_logic; 
    
    --Error signal for PC control FSM from fault confinement 
    --unit (Bit error or Stuff Error appeared)
    signal bit_Error_valid        :in   std_logic; 
    signal stuff_Error_valid      :in   std_logic; 
    
    --Note: New Interface for fault confinement incrementation
    signal inc_one                :out  std_logic;
    signal inc_eight              :out  std_logic;
    signal dec_one                :out  std_logic;
    
    signal tran_valid             :out  std_logic;
    signal rec_valid              :out  std_logic;
    signal ack_recieved_out       :out  std_logic;
        
    signal br_shifted             :out  std_logic;
        
    --------------------------------------------
    --Transcieve and recieve triggering signals-
    --------------------------------------------
    
    --Transcieve triggerring signal (sync_nbt,sync_dbt) 
    --multiplexed in core_top (CAN Core)
    signal tran_trig              :in   std_logic;
    
    --Recieve triggerring signal (sample_2_nbt,sample_2_dbt) 
    --multiplexed in core_top
    signal rec_trig               :in   std_logic;
    
    -------------------------------------------
    --Transcieved and recieved data interface--
    -------------------------------------------
    --Transcieve
    signal data_tx                :out  std_logic; --Transcieved data on CAN Bus
    signal stuff_enable           :out  std_logic;
    
    --Log 1 - Fixed Stuffing, Log 0 - Normal stuffing
    signal fixed_stuff            :out  std_logic;
    
    --Stuffing length
    signal stuff_length           :out  std_logic_vector(2 downto 0);
    
    --Recieved data
    signal data_rx                :in   std_logic;
    
    --Enabling destuffing
    signal destuff_enable         :out  std_logic;
    
    --Enabling firing of destuffing error
    signal stuff_error_enable     :out  std_logic;
    
    --Fixed stuffing method (log. 1), Normal stuffing (log 0);
    signal fixed_destuff          :out  std_logic;
    
    --Number of equal consequent bits before destuffed bit 
    signal destuff_length         :out  std_logic_vector(2 downto 0);
    
    --Number of stuffed bits modulo 8
    signal dst_ctr                :in   natural range 0 to 7;
    
    ------------------
    --CRC Interface --
    ------------------
    
    --Transition from 0 to 1 erases the CRC and operation holds as 
    -- long as enable=1
    signal crc_enable             :out  std_logic;
    signal crc15                  :in   std_logic_vector(14 downto 0); --CRC 15
    signal crc17                  :in   std_logic_vector(16 downto 0); --CRC 17
    signal crc21                  :in   std_logic_vector(20 downto 0); --CRC 21
    
    -----------------------
    --Prescaler interface--
    -----------------------
    signal sync_control           :out  std_logic_vector(1 downto 0); 
    --00-no synchronisation, 10-Hard synchronisation, 11-Resynchronisation
    
    -----------------------
    --Bus synchronisation--
    -----------------------
    --00 nominal, 01-data, 10 -secondary
    signal sp_control             :out  std_logic_vector(1 downto 0);
    
    --Clear the Shift register at the  beginning of Data Phase!!!
    signal ssp_reset              :out  std_logic;
    
    --Calibration command for transciever delay compenstation (counter)
    signal trv_delay_calib        :out  std_logic;
    
    --Bit Error detection enable (Ex. disabled when recieving data)
    signal bit_err_enable         :out  std_logic;
    --Note: In the end bit Error detection is always enabled, Fault confinement 
    -- module decides whenever the bit Error is VALID!!!
    
    --Synchronisation edge validated by prescaler!!!
    signal hard_sync_edge         :in   std_logic;
    
    --Internal loopBack enabled (for Bus monitoring mode)
    signal int_loop_back_ena      :out  std_logic;
    
    -- Protocol state output
    signal PC_State_out           :out  protocol_type
    
    );
  -----------------------
  --Driving bus aliases--
  -----------------------
  
  --RTR behavior setting
  signal drv_rtr_pref             :     std_logic;
  
  --Whenever FD Frames are supported for reciever
  signal drv_CAN_fd_ena           :     std_logic;
  
  --Bus Monitoring mode enabled
  signal drv_bus_mon_ena          :     std_logic;
  
  --Retransmition limit enabled for errornous frames
  signal drv_retr_lim_ena         :     std_logic;
  
  --Retransmittion treshold
  signal drv_retr_th              :     std_logic_vector(3 downto 0);
  
  --Self Test Mode enabled
  signal drv_self_test_ena        :     std_logic;
  
  --Immediately abort transmittion
  signal drv_abort_tran           :     std_logic;
  
  --Forbidding acknowledge mode
  signal drv_ack_forb             :     std_logic;
  
  --Enabling the whole controller
  signal drv_ena                  :     std_logic;
  
  --Type of FD Format Frame (ISO,non-ISO)
  signal drv_fd_type              :     std_logic;
  
  --Frame swapping behaviour of TX Arbitrator
  signal drv_frame_swap           :     std_logic;
  
  ----------------------
  --Internal registers--
  ----------------------
  
  --Protocol register
  signal PC_state                 :     protocol_type;
  
  --Internal loopBack enabled (for Bus monitoring mode)
  signal int_loop_back_ena_r      :     std_logic;
  
  -- Marks that TXT Buffer is locked and does not have
  -- to be locked anymore
  signal is_txt_locked            :     std_logic;
  
  ----------------------------------------
  --Retransmittion counters
  ----------------------------------------
  signal retr_count               :     natural range 0 to 15;
  
  --Registered values of output
  signal data_tx_r                :     std_logic;
  signal arbitration_lost_r       :     std_logic; 
  signal crc_enable_r             :     std_logic;
  signal stuff_enable_r           :     std_logic;
  signal fixed_stuff_r            :     std_logic;
  signal stuff_length_r           :     std_logic_vector(2 downto 0);
  signal destuff_enable_r         :     std_logic;
  signal fixed_destuff_r          :     std_logic;
  signal destuff_length_r         :     std_logic_vector(2 downto 0);
  signal stuff_error_enable_r     :     std_logic;
  signal is_idle_r                :     std_logic;
  signal set_transciever_r        :     std_logic;
  signal set_reciever_r           :     std_logic;
  
  --00 nominal, 01-data, 10 -secondary
  signal sp_control_r             :     std_logic_vector(1 downto 0);
  
  --Clear the Shift register at the  beginning of Data Phase!!!    
  signal ssp_reset_r              :     std_logic;
  
  --Calibration command for transciever delay compenstation (counter)
  signal trv_delay_calib_r        :     std_logic;
  
  --Bit Error detection enable (Ex. disabled when recieving data)
  signal bit_err_enable_r         :     std_logic;
  signal sync_control_r           :     std_logic_vector(1 downto 0);
  signal alc_r                    :     std_logic_vector(4 downto 0);

  signal form_Error_r             :     std_logic; --Form Error
  signal CRC_Error_r              :     std_logic; --CRC Error
  signal ack_Error_r              :     std_logic; --Acknowledge error
  signal unknown_state_Error_r    :     std_logic; --Unknown state Error
  
  signal inc_one_r                :     std_logic;
  signal inc_eight_r              :     std_logic;
  signal dec_one_r                :     std_logic;
  
  signal tran_valid_r             :     std_logic;
  signal rec_valid_r              :     std_logic;
  
  ---------------
  --SOF signals--
  ---------------
  --Signal whenever transcieve of SOF bit should be skypped 
  --(detection of DOMINANT in intermission)
  signal sof_skip                 :     std_logic; 
  
  -----------------------
  --Auxiliarly signals --
  -----------------------
  
  --data_rx & data_tx;
  signal aux_tx_rx                :     std_logic_vector(1 downto 0);
  
  --tran_frame_type & tran_ident_type
  signal aux_tran_frame_ident_type:     std_logic_vector(1 downto 0);
 
  ---------------------------
  --Recieved data registers--
  ---------------------------
  signal rec_dlc_r                :     std_logic_vector(3 downto 0);
 	signal rec_is_rtr_r             :     std_logic;
  signal rec_ident_type_r         :     std_logic;
  signal rec_frame_type_r         :     std_logic;
  signal rec_brs_r        	       :     std_logic;
  
  signal rec_ident_ext_sr         :     std_logic_vector(17 downto 0);
  signal rec_ident_base_sr        :     std_logic_vector(10 downto 0);
  
  --Recieved CRC value
  signal rec_crc_r                :     std_logic_vector(20 downto 0);
  
  --Recieved Error state indicator
  signal rec_esi_r                :     std_logic;
    
  -------------------------
  --Arbitration registers--
  -------------------------
  
  --Pointer on actual value of identifier
  signal tran_pointer             :     natural range 0 to 28;
  
  --Arbitration control state machine
  signal arb_state                :     arb_type;
  
  signal arb_two_bits             :     std_logic_vector(1 downto 0); 
  --First bit after the base identifier (RTR,r1,SRR)
  --Second bit after the base identifier (IDE)
  
  --Stored value of bit behind Identifier extension (RTR,r1)
  signal arb_one_bit              :     std_logic;
  
  --Stuff Error appeared in arbitration field!!
  signal stuff_err_arb_int        :     std_logic;
  
  -- Delay transition to conrol field by one clock cycle!
  -- This is to fix lost of arbitration in last bit!!!
  signal delay_control_trans      :     std_logic; 
  
  --Transceive identifier shift registers
  signal tran_ident_base_sr       :     std_logic_vector(10 downto 0);
  signal tran_ident_ext_sr        :     std_logic_vector(17 downto 0);  
  
  --------------------------
  --Control field registers-
  --------------------------
  --Pointer for counting DLC bits 
  signal control_pointer          :     natural range 0 to 7;
  
  --Signal for presetting the state machine of control field into correct
  -- state  based on type of transcieved/recieved frame
  signal FSM_preset               :     std_logic; 
  
  --State machine for managing the bits inside the control field
  signal control_state            :     control_type;
  
  --Register for transcieving the data in control field bits
  signal ctrl_tran_reg            :     std_logic_vector(7 downto 0);
  
  --Register for transcieving the data in control field bits
  signal dlc_int                  :     std_logic_vector(3 downto 0);
 
 
  ------------------------
  --Data field registers--
  ------------------------
  
  --Pointer for transcieving the data
  signal data_pointer             :     natural range 0 to 511;
  
  --Pointer for transcieving the stuf length field
  signal stl_pointer              :     natural range 0 to 3;
  signal data_size                :     natural range 0 to 511;
  
  -- Signals for optimalization of data reception usage
  -- Refer to Revision comment: 29.11.2017
  type rec_data_RAM_type is array (0 to 15) of std_logic_vector(31 downto 0); 
  
  --Shift register for data reception
  signal rec_data_sr              :     std_logic_vector(7 downto 0);
  
  --Register for counting received bytes in shift register
  signal rec_dram_ptr             :     natural range 0 to 7;
  
  --Byte index into RAM
  signal rec_dram_bind            :     natural range 0 to 3;
  signal rec_dram                 :     rec_data_RAM_type;
  
  --Pointer directly to TXT buffer to get the data
  signal txt_buf_ptr_r            :     natural range 0 to 15;
  
  -----------------------
  --CRC field registers--
  -----------------------
  
  --CRC Source , 00-CRC15, 01-CRC17, 10-CRC21,11-Invalid
  signal crc_src                  :     std_logic_vector(1 downto 0);
  
  --Recieved CRC matches the calculated one
  signal crc_check                :     std_logic;
  
  --Fixed stuff bit before CRC of FD Frame
  signal fixed_CRC_FD             :     std_logic;
  
  --Fixed stuff bit before CRC of FD Frame, for reciever
  signal fixed_CRC_FD_rec         :     std_logic;
  
  
  -----------------------------------------
  --Added signals for ISO FD type
  -----------------------------------------
  
  --Subfield of CRC (stuff count, real_crc)
  signal crc_state                :     crc_type;
  
  --Parity of the stuff count
  signal stuff_parity             :     std_logic;
  
  --Number of stuffed bits grey coded!!
  signal stuff_count_grey         :     std_logic_vector(2 downto 0);
  
  --Received value of stuff length parity field bit
  signal rx_parity                :     std_logic;
  
  --Received number of stuffed bits!
  signal rx_count_grey            :     std_logic_vector(2 downto 0);
  
  -----------------------
  --Delimiter registers--
  -----------------------
  
  --Acknowledge was recieved;
  signal ack_recieved             :     std_logic;
  
  --Whenever one acknowledge recessive bit already was monitorred by transciever
  -- (Delay compensation) 
  signal sec_ack                  :     std_logic;
  
  --------------------------
  --Intermission registers--
  --------------------------
  signal interm_state             :     interm_spc_type;
  
  -------------------------
  --Error frame registers--
  -------------------------
  signal err_frame_state          :     err_frame_type;
  
  --Register for detection of 6 consecutive equal bits!
  signal err_pas_bit_val          :     std_logic;

  -----------------------
  --Overload registers --
  -----------------------
  signal ovr_frame_state          :     ovr_frame_type; 

end entity;


architecture rtl of protocolControl is 
begin
  --Driving bus aliases
  drv_rtr_pref          <=  drv_bus(DRV_RTR_PREF_INDEX);
  drv_CAN_fd_ena        <=  drv_bus(DRV_CAN_FD_ENA_INDEX);
  drv_bus_mon_ena       <=  drv_bus(DRV_BUS_MON_ENA_INDEX);
  drv_retr_lim_ena      <=  drv_bus(DRV_RETR_LIM_ENA_INDEX);
  drv_retr_th           <=  drv_bus(DRV_RETR_TH_HIGH downto DRV_RETR_TH_LOW);
  drv_self_test_ena     <=  drv_bus(DRV_SELF_TEST_ENA_INDEX);
  drv_abort_tran        <=  drv_bus(DRV_ABORT_TRAN_INDEX);
  drv_ack_forb          <=  drv_bus(DRV_ACK_FORB_INDEX);
  drv_ena               <=  drv_bus(DRV_ENA_INDEX);
  drv_fd_type           <=  drv_bus(DRV_FD_TYPE_INDEX);
  drv_frame_swap        <=  drv_bus(DRV_FRAME_SWAP_INDEX);
  
  -----------------------------------
  --Registers to output propagation--
  -----------------------------------
  PC_State_out          <=  PC_State;
  alc                   <=  alc_r;
  data_tx       	       <=  data_tx_r;
  arbitration_lost      <=  arbitration_lost_r;
  crc_enable            <=  crc_enable_r;
  stuff_enable          <=  stuff_enable_r;
  fixed_stuff           <=  fixed_stuff_r;
  stuff_length          <=  stuff_length_r;
  destuff_enable        <=  destuff_enable_r;
  fixed_destuff         <=  fixed_destuff_r;
  destuff_length        <=  destuff_length_r;
  stuff_error_enable    <=  stuff_error_enable_r;
  is_idle               <=  is_idle_r;
  set_transciever       <=  set_transciever_r;
  set_reciever          <=  set_reciever_r;
  
  form_Error            <=  form_Error_r;
  CRC_Error             <=  CRC_Error_r;
  ack_Error             <=  ack_Error_r;
  unknown_state_Error   <=  unknown_state_Error_r;
  int_loop_back_ena     <=  int_loop_back_ena_r;
  
  inc_one               <=  inc_one_r;
  inc_eight             <=  inc_eight_r;
  dec_one               <=  dec_one_r;
  
  tran_valid            <=  tran_valid_r;
  rec_valid             <=  rec_valid_r;
  
  
  --Bus synchronisation interface registers
  sp_control            <=  sp_control_r;
  ssp_reset             <=  ssp_reset_r;
  trv_delay_calib       <=  trv_delay_calib_r;
  bit_err_enable        <=  bit_err_enable_r;
  --Synchronisation control
  sync_control          <=  sync_control_r;
  
  --Recieved data registers to output propagation
  rec_ident             <=  rec_ident_ext_sr&rec_ident_base_sr;
  rec_dlc               <=  rec_dlc_r;
  rec_is_rtr            <=  rec_is_rtr_r;
  rec_ident_type        <=  rec_ident_type_r;
  rec_frame_type        <=  rec_frame_type_r;
  rec_brs               <=  rec_brs_r;
  rec_crc               <=  rec_crc_r;
  rec_esi               <=  rec_esi_r;
  ack_recieved_out      <=  ack_recieved;
  
  --Pointer into TXT Buffer
  txt_buf_ptr           <=  txt_buf_ptr_r;
  
  -----------------------
  --Auxiliarly vectors
  -----------------------
  aux_tx_rx                 <=  data_tx_r & data_rx;
  aux_tran_frame_ident_type <=  tran_frame_type&tran_ident_type;
  
  -------------------------------------
  --Gray coding of stuff bit counter  
  -------------------------------------
   with dst_ctr select stuff_count_grey <=
      "000" when 0,
      "001" when 1,
      "011" when 2,
      "010" when 3,
      "110" when 4,
      "111" when 5,
      "101" when 6,
      "100" when 7,
      "000" when others;
  
   -------------------------------------
   --Parity of the stuff length field
   -------------------------------------
   stuff_parity <= '0' when (dst_ctr mod 2)=0 else
                   '1';  
  
   -------------------------------------
   -- Output of receive data RAM
   ------------------------------------- 
   rec_dram_word <= rec_dram(rec_dram_addr);
  
  ---------------------------------------
  ---------------------------------------
  --Protocol control process
  ---------------------------------------
  ---------------------------------------
  PC_proc:process(clk_sys,res_n)
  begin
    if(res_n=ACT_RESET)then
      --Presetting the state
      PC_State                <=  off;
      interm_state            <=  interm_idle;
      int_loop_back_ena_r     <=  '0';
      retr_count              <=  0;
      is_txt_locked           <=  '0';
      
      --------------------------------
      --Configuring output registers--
      --------------------------------
      txt_hw_cmd.lock         <=  '0';
      txt_hw_cmd.unlock       <=  '0';
      txt_hw_cmd.valid        <=  '0';
      txt_hw_cmd.err          <=  '0';
      txt_hw_cmd.arbl         <=  '0';
      txt_hw_cmd.failed       <=  '0';
      
      --FSM starts from intermission.interm_idle state, we dont need 
      --preseting for intermission then!!! We CANT preeset then!!
      FSM_preset              <=  '0';
      crc_enable_r            <=  '0';
      data_tx_r               <=  RECESSIVE;
      arbitration_lost_r      <=  '0';
      is_idle_r               <=  '0';
      
      --Configure Bit Stuffing
      stuff_enable_r          <=  '0';
      fixed_stuff_r           <=  '0';
      stuff_length_r          <=  std_logic_vector(
                                  to_unsigned(BASE_STUFF_LENGTH,3));
            
      --Configuring Bit Destuffing
      destuff_enable_r        <=  '0';
      fixed_destuff_r         <=  '0';
      destuff_length_r        <=  std_logic_vector(
                                  to_unsigned(BASE_STUFF_LENGTH,3));
      stuff_error_enable_r    <=  '0';
      
      inc_one_r               <=  '0';
      inc_eight_r             <=  '0';
      dec_one_r               <=  '0';

      br_shifted              <=  '0';
      
      tran_valid_r            <=  '0';
      rec_valid_r             <=  '0';
      
      err_pas_bit_val         <=  RECESSIVE;
      stuff_err_arb_int       <=  '0';
      
      --------------------------------
      --Prestting internal registers--
      -------------------------------- 
      rec_brs_r               <=  '0';
      rec_crc_r               <=  (OTHERS=>'0');
      rec_esi_r               <=  '0';
      
      sof_skip                <=  '0';
           
      arb_two_bits            <=  (OTHERS=>'0');
      arb_one_bit             <=  '0';
      
      ctrl_tran_reg           <=  (OTHERS =>'0');
      dlc_int                 <=  (OTHERS=>'0');
      
      crc_src                 <=  "11";
      crc_check               <=  '0';
      
      ack_recieved            <=  '0';
      sec_ack                 <=  '0';
      
      tran_pointer            <=  0;
      alc_r                   <=  (OTHERS=>'0');
      data_size               <=  0;
      
      tran_ident_base_sr      <= (OTHERS => '0');
      tran_ident_ext_sr       <= (OTHERS => '0');
      
      --Nulling recieve registers
      rec_ident_base_sr       <=  (OTHERS=>'0');
      rec_ident_ext_sr        <=  (OTHERS=>'0');
      rec_dlc_r               <=  (OTHERS=>'0');
      rec_is_rtr_r            <=  '0';
      rec_ident_type_r        <=  '0';
      rec_frame_type_r        <=  '0';
      
      -- Receive data RAM
      rec_dram_ptr            <= 0;
      rec_dram_bind           <= 0;
      rec_data_sr             <= (OTHERS => '0');
      
      -- Pointer directly to TXT Buffer RAM
      txt_buf_ptr_r           <= 0;
      
      --Presetting the sampling point control
      sp_control_r            <=  NOMINAL_SAMPLE;
      ssp_reset_r             <=  '0';
      trv_delay_calib_r       <=  '0';
      bit_err_enable_r        <=  '0';
      fixed_CRC_FD            <=  '0';
      fixed_CRC_FD_rec        <=  '0';
      sync_control_r          <=  NO_SYNC;
      
      --Error presetting
      form_Error_r            <=  '0';
      CRC_Error_r             <=  '0';
      ack_Error_r             <=  '0';
      unknown_state_Error_r   <=  '0';
      set_transciever_r       <=  '0';
      set_reciever_r          <=  '0';
      
      delay_control_trans     <=  '0';
      
      rx_parity               <=  '0';
      rx_count_grey           <=  (OTHERS =>'0');
       
    elsif rising_edge(clk_sys)then
        
      -----------------------------------------------------
      --Assigning previous values to avoid latch creation--
      -----------------------------------------------------
      
       PC_state               <=  PC_state; --Protocol register
       data_tx_r              <=  data_tx_r; --Registered value of tx data
       arbitration_lost_r     <=  '0'; 
       crc_enable_r           <=  crc_enable_r;
       is_txt_locked          <=  is_txt_locked;
       
       -- These TX arbitrator control signals are set only for one
       -- clock cycle
       txt_hw_cmd.lock         <=  '0';
       txt_hw_cmd.unlock       <=  '0';
       txt_hw_cmd.valid        <=  '0';
       txt_hw_cmd.err          <=  '0';
       txt_hw_cmd.arbl         <=  '0';
       txt_hw_cmd.failed       <=  '0';
     
       stuff_enable_r         <=  stuff_enable_r;
       fixed_stuff_r          <=  fixed_stuff_r;
       stuff_length_r         <=  stuff_length_r;
       destuff_enable_r       <=  destuff_enable_r;
       fixed_destuff_r        <=  fixed_destuff_r;
       destuff_length_r       <=  destuff_length_r;
       stuff_error_enable_r   <=  stuff_error_enable_r;
       rec_ident_base_sr      <=  rec_ident_base_sr;
       rec_ident_ext_sr       <=  rec_ident_ext_sr;
       rec_dlc_r              <=  rec_dlc_r;
       rec_is_rtr_r           <=  rec_is_rtr_r;
       rec_ident_type_r       <=  rec_ident_type_r;
       rec_frame_type_r       <=  rec_frame_type_r;
       rec_brs_r              <=  rec_brs_r;
       rec_crc_r              <=  rec_crc_r;
       rec_esi_r              <=  rec_esi_r;
       tran_pointer           <=  tran_pointer;
       arb_state              <=  arb_state;--Arbitration control state machine
       arb_two_bits           <=  arb_two_bits;
       
       tran_ident_base_sr     <= tran_ident_base_sr;
       tran_ident_ext_sr      <= tran_ident_ext_sr;
       
       --Stored value of bit behind Identifier extension (RTR,r1)
       arb_one_bit            <=  arb_one_bit;
       
       --Pointer for counting DLC bits
       control_pointer        <=  control_pointer;
       
       --Signal for presetting the state machine of control field into 
       --correct state
       FSM_preset             <=  FSM_preset;
       
       --State machine for managing the bits inside the control field
       control_state          <=  control_state;
       
       --Register for transcieving the data in control field bits
       ctrl_tran_reg          <=  ctrl_tran_reg;
       
       --Internal registered value of DLC field (transcieved or recieved)
       dlc_int                <=  dlc_int;
       
       --Pointer for transcieving the data
       data_pointer           <=  data_pointer;
       
       --CRC Source , 00-CRC15, 01-CRC17, 10-CRC21,11-Invalid
       crc_src                <=  crc_src;
       
       --Recieved CRC matches the calculated one
       crc_check              <=  crc_check;
       
       --Acknowledge was recieved;
       ack_recieved           <=  ack_recieved;
       
       --Whenever one acknowledge recessive bit already was monitorred
       --by transciever (Delay compensation)
       sec_ack                <=  sec_ack;
       sof_skip               <=  sof_skip;
       interm_state           <=  interm_state;
       err_frame_state        <=  err_frame_state;
       fixed_CRC_FD           <=  fixed_CRC_FD;
       fixed_CRC_FD_rec       <=  fixed_CRC_FD_rec;
       err_pas_bit_val        <=  err_pas_bit_val;
       data_size              <=  data_size;
       
       --Retransmittion signals
       retr_count            <=  retr_count;
  
       --Control signals for OP_State FSM
       is_idle_r              <=  '0';
       set_transciever_r      <=  '0';
       set_reciever_r         <=  '0';
       
       --Error signals(are in logic one only for one clk_sys cycle!)
       form_Error_r           <=  '0';
       CRC_Error_r            <=  '0';
       ack_Error_r            <=  '0';
       unknown_state_Error_r  <=  '0';
       int_loop_back_ena_r    <=  int_loop_back_ena_r;
       crc_state              <=  crc_state;
       
       inc_one_r              <=  '0';
       inc_eight_r            <=  '0';
       dec_one_r              <=  '0';
       
       tran_valid_r           <=  '0';
       rec_valid_r            <=  '0';
       
       br_shifted             <=  '0';
       
       stuff_err_arb_int      <=  '0'; --Stuff error appeared during arbitration
       
       --Bus synchronisation interface registers
       sp_control_r           <=  sp_control_r;
       ssp_reset_r            <=  '0';
       trv_delay_calib_r      <=  trv_delay_calib_r;
       bit_err_enable_r       <=  bit_err_enable_r;
      
       sync_control_r         <=  sync_control_r;
       
       delay_control_trans    <=  '0';    
    
       rx_parity              <=  rx_parity;
       rx_count_grey          <=  rx_count_grey;
    
       rec_data_sr            <=  rec_data_sr;
       rec_dram_ptr           <=  rec_dram_ptr;
       rec_dram_bind          <=  rec_dram_bind;
        
       txt_buf_ptr_r          <=  txt_buf_ptr_r;
    
    if(drv_ena='0')then
      PC_State                <=  off;
      
    elsif(bit_Error_valid='1' or stuff_Error_valid='1')then     
      PC_State                <=  error;
      FSM_preset              <=  '1';
      
      if(OP_State=reciever)then
				
        --Bit Error or Stuff Error detected by reciever (Control,data,CRC) , 
        -- Increase by one
        inc_one_r             <=  '1';
        
      elsif(OP_State=transciever and PC_State=arbitration)then
        stuff_err_arb_int     <=  '1';
      end if;
      
    elsif(OP_State=transciever and drv_abort_tran='1')then 
      PC_State                <=  interframe;
      FSM_Preset              <=  '1';
      CRC_enable_r            <=  '0';
      stuff_enable_r          <=  '0';
      destuff_enable_r        <=  '0';
      is_idle_r               <=  '1'; 
    
        --Bug fix 21.6.2016
    elsif(delay_control_trans  =  '1')then
      PC_State                <= control;
    
    else
      case PC_state is 
      
      --------------------------------------------------------------------------
      --Start of frame 
      --------------------------------------------------------------------------
      when sof => 
            if(FSM_preset='1')then
                
                ack_recieved <= '0';
                crc_check    <= '0';
                
                -- Bus monitoring mode is disabled
                if(drv_bus_mon_ena='0')then
                    
                    -- If we already have frame locked, or we have frame to lock
                    -- available
                    if (is_txt_locked = '1' or (tran_frame_valid_in = '1')) then
                      set_transciever_r <=  '1';
                      stuff_enable_r    <=  '1';
                      fixed_stuff_r     <=  '0';
                      stuff_length_r    <=  std_logic_vector(
                                            to_unsigned(BASE_STUFF_LENGTH,3));
                    end if;
                    
                    -- If we dont have frame locked, but we have one available
                    -- the we just lock it!
                    if (is_txt_locked = '0' and (tran_frame_valid_in = '1')) then
                       txt_hw_cmd.lock   <=  '1';
           	           is_txt_locked     <=  '1';
           	           
           	           -- In case that TX Arbitrator provides different frame for
           	           -- us, we need to erase the retranmsitt counter
           	           if (txtb_changed = '1') then
                          retr_count <= 0;
                        end if;
                    end if;
                    
                    -- If we dont have anything to lock, and have nothing locked
          	     	   if (is_txt_locked = '0' and (tran_frame_valid_in = '0')) then
          	     	     set_reciever_r      <=  '1';
                    end if;
                
                else 
                  set_reciever_r          <=  '1';
                end if;
                
                --If this one bit should be skipped go directly to 
                --arbitration field
                if(sof_skip='1')then
                  FSM_preset              <=  '1';
                  PC_State                <=  arbitration;
                  sync_control_r          <=  RE_SYNC; 
                else
                  FSM_preset              <=  '0';
                end if;
                
                --Bus synchronisation settings
                sp_control_r              <=  NOMINAL_SAMPLE;
                ssp_reset_r               <=  '1';
                trv_delay_calib_r         <=  '0';
                bit_err_enable_r          <=  '1'; 
                
                --Erasing internal DLC
                dlc_int                   <=  (OTHERS =>'0');
                
                --Configuration Bit Destuffing (Both transciever and reciever)
                destuff_enable_r          <=  '1';
                fixed_destuff_r           <=  '0';
                destuff_length_r          <=  std_logic_vector(
                                              to_unsigned(BASE_STUFF_LENGTH,3));
                stuff_error_enable_r      <=  '1'; 
              
                --Clearing arbitration transcieve pointer for transcieving
                --identifier. Restarting arbitration state machine
                
                --The index in tran_ident in where MSB bit of the 
                --base ident is 10
                tran_pointer              <=  10;
                
                --Load the Identifier transmission shift registers
                tran_ident_base_sr        <= tran_ident_base;
                tran_ident_ext_sr         <= tran_ident_ext;
                
                arb_state                 <=  base_id;
                crc_enable_r              <=  '1';
                
                --Erasing the recieved data registers
                rec_ident_base_sr         <=  (OTHERS =>'0');
                rec_ident_ext_sr          <=  (OTHERS =>'0');
                rec_dlc_r                 <=  (OTHERS =>'0');
 	              rec_is_rtr_r              <=  '0';
                rec_ident_type_r          <=  '0';
                rec_frame_type_r          <=  '0';
                rec_brs_r                 <=  '0';
                rec_crc_r                 <=  (OTHERS =>'0');
                rec_esi_r                 <=  '0';
                
                rx_parity                 <=  '0';
                rx_count_grey             <=  (OTHERS =>'0');
      
                control_pointer           <=  0;
                
            else
                
                --Transcieving the data if we have what to transcieve
                if(tran_trig='1')then
                  if(OP_State=transciever or (tran_frame_valid_in='1'))then
                    data_tx_r             <=  DOMINANT;
                  else
                    data_tx_r             <=  RECESSIVE;
                  end if;
                end if;
                --Note: OP_State machine has to react and set the state to
                --       transciever when SOF state and data are availiable!
              
                --Recieving the data
                if(rec_trig='1')then
                  if(data_rx=DOMINANT)then
                    PC_state              <=  arbitration;
                    sync_control_r        <=  RE_SYNC; 
                  else
                  
                    --First bit detected recessive!
                    PC_state              <=  error;
                    if(OP_state=reciever)then
                      inc_one_r           <=  '1';
                    end if;
                  end if;
                  FSM_Preset              <=  '1';
                end if; 
                 
            end if;
    
    ----------------------------------------------------------------------------
    --Arbitration
    ----------------------------------------------------------------------------
    when arbitration =>
          if(FSM_Preset='1')then
            FSM_Preset<='0';
           else
            --Losing arbitration when sending recessive and sampling dominant
            if(OP_state=transciever)then
              if(rec_trig='1')then
                case aux_tx_rx is
                when DOMINANT_DOMINANT => 
                      arbitration_lost_r  <=  '0';
                when DOMINANT_RECESSIVE =>
                      arbitration_lost_r  <=  '0';
                      PC_State            <=  error;
                      FSM_Preset          <=  '1';
                when RECESSIVE_DOMINANT =>
                      arbitration_lost_r  <=  '1';
                      
                      --When switching to reciever only recessive bits will 
                      --be sent, then no Stuff bits are inserted
                      stuff_enable_r      <=  '0';
                      
                      --Current frame should be retransmitted!
                      txt_hw_cmd.unlock   <=  '1';
                      txt_hw_cmd.arbl     <=  '1';
                      is_txt_locked       <=  '0';
                      
                when RECESSIVE_RECESSIVE =>
                      arbitration_lost_r  <=  '0';
                when others => 
                      unknown_state_Error_r <=  '1';
                      arbitration_lost_r    <=  '0';
                      PC_State              <=  error;
                      FSM_preset            <=  '1';
                end case;
              else
                arbitration_lost_r        <=  arbitration_lost_r;
              end if;
            else
              arbitration_lost_r          <=  '0';
            end if;
              
            --Arbitration state machine
            case arb_state is
            when base_id =>
                  if(tran_trig='1')then
                    if(OP_state=transciever)then
                      
                      --Direct addressing replaced by shift register
                      data_tx_r          <= tran_ident_base_sr(10);
                      tran_ident_base_sr <= tran_ident_base_sr(9 downto 0)&'0';
                      --data_tx_r     <=  tran_ident(tran_pointer);
                    else
                      data_tx_r     <=  RECESSIVE;
                    end if;
                  end if;   
                  if(rec_trig='1')then
                    if(tran_pointer=0)then
                      --Nulling the pointer for two_bits state
                      tran_pointer  <=  1;
                      arb_state     <=  two_bits;
                    else
                      tran_pointer  <=  tran_pointer-1;
                    end if;
                    
                    --Replaced direct addressing with Shift register
                    rec_ident_base_sr <= rec_ident_base_sr(9 downto 0)&data_rx;
                  end if;
                  if(arbitration_lost_r='1')then
                    alc_r           <=  std_logic_vector(
                                        to_unsigned(9-tran_pointer,5));
                  end if;
                  --Note: possible optimalization, using tran_pointer from 
                  --			0 to 10 instead of from 10 to 0
            when two_bits =>
                 if(tran_trig='1')then --Transcieving two bits
                    if(OP_state=transciever)then
                      case tran_pointer is
                      when 1 => --Sending first bit
                            case aux_tran_frame_ident_type is
                             when "00" => data_tx_r   <=  tran_is_rtr;  --RTR Bit
                             when "10" => data_tx_r   <=  DOMINANT; --r1 Bit
                             when "01" => data_tx_r   <=  RECESSIVE; --SRR Bit
                             when "11" => data_tx_r   <=  RECESSIVE;  --SRR Bit
                             when others=> data_tx_r  <=  RECESSIVE;
                                           unknown_state_Error_r  <=  '1'; 
                                           PC_State               <=  error;
                                           FSM_preset             <=  '1'; 
                            end case;
                      when 0 => --Sending second bit (IDE)
                          --Note: IDE for Base frame is part of Control field ,
                          -- but is it is dominant it can also be part of arbit-
                          -- ration field, because then node which sends it 
                          -- always wins arbitration! 
                          data_tx_r   <=  tran_ident_type; 
                          --IDE Bit in extended format
                      when others => 
                            unknown_state_Error_r <=  '1'; 
                            PC_State              <=  error;
                            FSM_preset            <=  '1';
                      end case; 
                    else
                      data_tx_r                   <=  RECESSIVE;
                    end if;
                 end if;
                 
                 if(rec_trig='1')then --Sampling (recieving) two bits
                   case tran_pointer is
                   when 1 => --First bit of two is sampled
                        arb_two_bits(1)   <=  data_rx;
                        tran_pointer      <=  0;
                   when 0 => --Second bit of two is sampled (IDE)
                        --IDE bit value decides whenever we go to control 
                        --field or extended identifier
                        if(data_rx=DOMINANT)then
                          
                          --Bug fix 21.6.2016
                          delay_control_trans     <=  '1';
                         -- PC_State<=control;
                         
                          arb_state               <=  arb_state;
                          tran_pointer            <=  tran_pointer;
                          FSM_preset              <=  '1';
                          rec_ident_type_r        <=  '0';
                        else
                          arb_state               <=  ext_id;
                          --For extending frame pointer need to be set to 
                          --beginning of extended frame!
                          tran_pointer            <=  28; 
                          rec_ident_type_r        <=  '1';
                        end if;
                        arb_two_bits(0)           <=  data_rx;
                   when others =>
                          unknown_state_Error_r   <=  '1'; 
                          PC_State                <=  error;
                          FSM_preset              <=  '1';
                   end case;
                 end if;
                 --Note: IDE bit for Base CAN is part of the arbitration field in
                 --      this implementation. If it is recessive then the frame 
                 --      is Extended and arbitration goes on normally If it is 
                 --      dominant it is last bit of arbitration therefore the 
                 --      arbitration ends!
                 
                 if(arbitration_lost_r='1')then
                    alc_r         <=  std_logic_vector(
                                      to_unsigned(1-tran_pointer,5));
                  end if;
            when ext_id=>
                 if(tran_trig='1')then
                    if(OP_state=transciever)then
                      
                      --Replaced direct access by shift register
                      data_tx_r          <= tran_ident_ext_sr(17);
                      tran_ident_ext_sr  <= tran_ident_ext_sr(16 downto 0)&'0';
                      
                      --data_tx_r   <=  tran_ident(tran_pointer);
                    else
                      data_tx_r   <=  RECESSIVE;
                    end if;
                  end if;
                  
                  if(rec_trig='1')then
                    if(tran_pointer=11)then
                      tran_pointer  <=  0; --The value doesnt matter now
                      arb_state     <=  one_bit; --Following 
                    else
                      tran_pointer  <=  tran_pointer-1;
                      arb_state     <=  arb_state;
                    end if;
                    
                    --Storing recieved extended identifier
                    --Replaced direct addressing with shift register
                    rec_ident_ext_sr <= rec_ident_ext_sr(16 downto 0)&data_rx;
                    
                  end if;    
                  
                  if(arbitration_lost_r='1')then
                    alc_r   <=  std_logic_vector(to_unsigned(42-tran_pointer,5));
                  end if;
                  
            when one_bit=> --RTR bit of CAN Base, r0 bit of CAN FD
                 if(tran_trig='1')then
                    if(OP_State=transciever)then
                      if(tran_frame_type=FD_CAN or tran_is_rtr=NO_RTR_FRAME)then
                        data_tx_r <=  DOMINANT; 
                      else
                        data_tx_r <=  RECESSIVE;
                      end if;
                    else
                      data_tx_r   <=  RECESSIVE;
                    end if; 
                 end if;
                 if(rec_trig='1')then
                 
                    --Storing the last bit of arbitration field
                    arb_one_bit   <=  data_rx;
                    
                    --Bug fix 21.6.2016
                    delay_control_trans     <=  '1';
                    --PC_State<=control;
                    
                    --Erasing the value so that FSM can be preset in next 
                    --state first clock cycle
                    FSM_preset    <=  '1'; 
                 end if;
                 if(arbitration_lost_r='1')then
                    alc_r         <=  std_logic_vector(to_unsigned(31,5));
                  end if;
            when others=>
                  unknown_state_Error_r   <=  '1'; 
                  PC_State                <=  error;
                  FSM_preset              <=  '1';
            end case;
          end if;  
          
      --------------------------------------------------------------------------
      --Control frame field
      --------------------------------------------------------------------------
      when control => 
            
            if(OP_State=transciever)then 
              --Presetting the Shift-Register for transcieving control bits
              if(FSM_preset='1')then
                  
                  if(tran_frame_type=NORMAL_CAN and tran_is_rtr=RTR_FRAME)then
                    dlc_int <=  (OTHERS =>'0');
                  else
                    dlc_int <=  tran_dlc; --Storing Internal value of DLC 
                  end if;
                  
                  case aux_tran_frame_ident_type is
                  
                  --CAN Base format
                  when NORMAL_CAN & BASE =>
                          ctrl_tran_reg(4)    <=  DOMINANT; --r0 bit
                          control_pointer     <=  4;
                  
                  --CAN FD Format
                  when FD_CAN & BASE =>
                          ctrl_tran_reg(7)    <=  RECESSIVE; --EDL Bit
                          ctrl_tran_reg(6)    <=  DOMINANT; --r0 Bit
                          ctrl_tran_reg(5)    <=  tran_brs;--BRS Bit
                          if(error_state=error_active)then
                            ctrl_tran_reg(4)  <=  DOMINANT;
                          else
                            ctrl_tran_reg(4)  <=  RECESSIVE;
                          end if;
                          control_pointer     <=  7;
                  
                  --CAN Extended format
                  when NORMAL_CAN & EXTENDED => 
                          ctrl_tran_reg(5)    <=  DOMINANT; --r1 Bit
                          ctrl_tran_reg(4)    <=  DOMINANT; --r0 Bit
                          control_pointer     <=  5;
                          
                  --CAN FD Extended Format
                  when FD_CAN & EXTENDED =>
                          ctrl_tran_reg(7)    <=  RECESSIVE; --EDL Bit
                          ctrl_tran_reg(6)    <=  DOMINANT; --r0 Bit
                          ctrl_tran_reg(5)    <=  tran_brs;--BRS Bit
                          if(error_state=error_active)then
                            ctrl_tran_reg(4)  <=  DOMINANT;
                          else
                            ctrl_tran_reg(4)  <=  RECESSIVE;
                          end if;
                          control_pointer     <=  7;
                          
                  when others=>
                          unknown_state_Error_r <=  '1'; 
                          PC_State              <=  error;
                          FSM_preset            <=  '1';
                  end case;
                  
                --If Frame is RTR then user has chance whenever to send DLC=0000
                --or custom DLC as usual. No data is sent in both cases in RTR 
                --Frame.
                if(tran_frame_type=NORMAL_CAN   and 
									 tran_is_rtr=RTR_FRAME        and 
									 drv_rtr_pref='1')
								then
                  ctrl_tran_reg(3 downto 0) <=  (OTHERS =>'0'); 
                else
                  --DLC bits are common for control fields of all frame types
                  ctrl_tran_reg(3 downto 0) <=  tran_dlc;  
                end if;
                FSM_preset                  <=  '0';
                bit_err_enable_r            <=  '1';
              else 
                
                
                dlc_int                     <=  dlc_int;
                if(tran_trig='1')then
                  
                  --Sending Control bits on the bus
                  data_tx_r                 <=  ctrl_tran_reg(control_pointer);
                  
                end if;
                
                if(rec_trig='1')then   --Necessary delay compensation !!
                  
                  if(control_pointer>0)then
                    control_pointer         <=  control_pointer-1;
                  else
                    control_pointer         <=  0;
                  end if;
                  
                  if(control_pointer=0)then
                    if(tran_is_rtr=RTR_FRAME and tran_frame_type=NORMAL_CAN) or 
											(tran_dlc="0000")
										then
                        PC_State            <=  crc;
                    else
                        --Moving to the data phase when ALL DLC bits are 
                        --transcieved (or CRC Phase for RTR Frame)
                        PC_State            <=  data;     
                    end if;
                    FSM_preset              <=  '1';
                  end if;

                   
                  --Transciever delay compensation calibration for FD Frames.
                  --Note: Following condition is satisfied only when transcieve 
                  -- of FD frame appears!!
                  if(control_pointer=7)then --EDL bit
										
                    --Clearing the shift register for output data
                    ssp_reset_r           <=  '1';
                    
                    trv_delay_calib_r     <=  '1';
                    
                    --sync_control_r      <=  HARD_SYNC;  
                    --Hard synchronisation has to be performed in the bits bet-
                    --ween EDL and r0 of CAN FD Note: hard synchronisation so 
                    -- far ommited. Newest implementation of prescaler didnt 
                    -- support hard synchronisation in middle of data transfer!
                  else
                    trv_delay_calib_r     <=  '0';
                    --Note:ssp_reset released when data phase starts!
                  end if;         
                  
                  --Switching bit-Rate for FD Frames:
                  if(control_pointer=5      and 
								     tran_brs=BR_SHIFT      and
								     tran_frame_type=FD_CAN)
								  then --BRS bit of FD Frame
                    sp_control_r    <=  SECONDARY_SAMPLE;
                    br_shifted      <=  '1';
                    sync_control_r  <=  NO_SYNC;
                    ssp_reset_r     <=  '0';
                  end if;
                  
                end if;
              end if;
            else
                data_tx_r           <=  RECESSIVE;
            end if;         
            
            
            if(OP_State=reciever)then --Recieving control bits 
              if(rec_trig='1')then
                
                if(FSM_preset='1')then --Detecting first bit in control field
									
                  --EDL bit -> CAN FD Frame, r0 bit ->CAN Frame
                  rec_frame_type_r  <=  data_rx;
                  
                  if(data_rx=RECESSIVE)then --IF is FD Frame
                  
                      --If FD Frames are supported,go on, otherwise 
                      -- throw Form error
                      if(drv_CAN_fd_ena='1')then
                        control_pointer   <=  6; --r0,BRS,ESI,4DLC bits
                        rec_is_rtr_r      <=  '0';
                        FSM_preset        <=  '0';
                        
                        --sync_control_r  <=  HARD_SYNC;
                        --Note: hard synchronisation so far ommited. Newest 
                        -- implementation of prescaler didnt support hard
                        -- synchronisation in middle of data transfer!
                        --report "Reciever EDL";
                      else
                        PC_State      <=  error;
                        form_Error_r  <=  '1';
                        inc_one_r     <=  '1';
                        FSM_preset    <=  '1';
                      end if;
                  else
                    FSM_preset<='0';
                    if(rec_ident_type_r=EXTENDED)then
                      control_pointer <=  4; --r0 bit,4 DLC
                      rec_is_rtr_r    <=  arb_one_bit;
                    else
                      control_pointer <=  3; --DLC
                      rec_is_rtr_r    <=  arb_two_bits(1);
                    end if;
                  end if;
                  
                else
                  case control_pointer is
                  when 6 =>
                       control_pointer<=control_pointer-1;
                       
                       --r0 bit of FD Frame detected recessive
                       if(data_rx=RECESSIVE)then
                          form_Error_r  <=  '1';
                          inc_one_r     <=  '1';
                          PC_State      <=  error;
                          FSM_Preset    <=  '1';
                       end if;
                       sync_control_r   <=  RE_SYNC;   
                  when 5 => --BRS Bit
                      control_pointer   <=  control_pointer-1;
                      rec_brs_r         <=  data_rx;
                      if(data_rx=RECESSIVE)then
                        sp_control_r    <=  DATA_SAMPLE; --Switching bit rate
                        br_shifted      <=  '1';
                      end if;
                  when 4 => 
                      control_pointer<=control_pointer-1;
                      if(rec_frame_type_r=FD_CAN)then
                        rec_esi_r       <=  data_tx_r;       
                      else
                        
                        --r0 bit of extended frame detected recessive
                        if(rec_ident_type_r=EXTENDED and data_rx=RECESSIVE)then
                          form_Error_r  <=  '1';
                          inc_one_r     <=  '1';
                          PC_State      <=  error;
                          FSM_Preset    <=  '1';
                        end if;
                    end if;
                  when 3 => 
                      control_pointer   <=  control_pointer-1;
                      rec_dlc_r(3)      <=  data_rx;  
                  when 2 => 
                      control_pointer   <=  control_pointer-1;
                      rec_dlc_r(2)      <=  data_rx; 
                  when 1 => 
                      control_pointer   <=  control_pointer-1;
                      rec_dlc_r(1)      <=  data_rx;
                  when 0 => 
                      rec_dlc_r(0)      <=  data_rx;
                      FSM_preset        <=  '1';
                      
                      --If frame is RTR Frame or dat length is zero
                      if((rec_is_rtr_r=RTR_FRAME and rec_frame_type_r=NORMAL_CAN) or 
                        (rec_dlc_r(3 downto 1)="000" and data_rx='0'))then 
                        PC_State        <=  crc;
                      else
                        PC_State<=data;
                      end if;
                      
                      --Bug fix 22.6.2016
                      --If RTR frame is recieved, than actual DLC which is re-
                      --cieved depends on rtr_pref behaviour of transciever! Thus
                      --we can recieve DLC of e.g 12 bytes but frame is RTR so we
                      --should decide about CRC length from RTR flag not recieved
                      --DLC!!!
                      if(rec_is_rtr_r=RTR_FRAME and rec_frame_type_r=NORMAL_CAN)then
                        dlc_int             <= (OTHERS => '0');
                      else
                        dlc_int(3 downto 1) <=  rec_dlc_r(3 downto 1);
                        dlc_int(0)          <=  data_rx;
                      end if;
                      
                  when others=> 
                      unknown_state_Error_r <=  '1'; 
                      PC_State              <=  error;
                      FSM_preset            <=  '1';
                  end case;
                                    
                end if;    
              end if;
            end if;
            
            
      --------------------------------------------------------------------------
      --Data Phase
      --------------------------------------------------------------------------
      when data => 
          if(FSM_Preset='1')then
            FSM_Preset  <=  '0';
            
            --Note: We dont have to ask whenever frame is RTR frame, in RTR frame 
            --we never get into Data Phase!
            case dlc_int is
            when "0000" => data_size  <=  0; --Zero bits
            when "0001" => data_size  <=  7; --1 byte
            when "0010" => data_size  <=  15; --2 bytes
            when "0011" => data_size  <=  23; --3 bytes
            when "0100" => data_size  <=  31; --4 bytes
            when "0101" => data_size  <=  39; --5 bytes
            when "0110" => data_size  <=  47; --6 bytes
            when "0111" => data_size  <=  55; --7 bytes
            when "1000" => data_size  <=  63; --8 bytes
            when "1001" => data_size  <=  95; --12 bytes
            when "1010" => data_size  <=  127; --16 bytes
            when "1011" => data_size  <=  159; --20 bytes
            when "1100" => data_size  <=  191; --24 bytes
            when "1101" => data_size  <=  255; --32 bytes
            when "1110" => data_size  <=  383; --48 bytes
            when "1111" => data_size  <=  511; --64 bytes
            when others => data_size  <=  0;
                           unknown_state_Error_r  <=  '1'; 
                           PC_State               <=  error;
                           FSM_preset             <=  '1';
            end case;
            
            data_pointer    <=  511;
            
            if(OP_State=transciever and tran_frame_type=FD_CAN)then
             --Transmitter shall not synchronize in data phase of CAN FD Frame!
             sync_control_r <=  NO_SYNC;
            end if;
            
            --Receive RAM signals
            rec_dram_ptr            <= 0;
            rec_dram_bind           <= 0;
            rec_data_sr             <= (OTHERS => '0');
            
            -- Pointer directly to TXT Buffer
            txt_buf_ptr_r           <= 0;
            
          else

            if(OP_State=transciever)then
              if(tran_trig='1')then
              
                --data_tx_r <=  tran_data(data_pointer);
                data_tx_r <= tran_data(data_pointer mod 32);
                
                --Move to the next word
                if ((data_pointer mod 32) = 0) then
                  txt_buf_ptr_r <= (txt_buf_ptr_r+1) mod 16;
                end if;
                
              end if;
            else
              data_tx_r   <=  RECESSIVE;
            end if;
            
            --Recieving data (also transmitter recieves the same data)
            if(rec_trig='1')then
              
              -- Shift register and storing to local RAM
              rec_data_sr               <=  rec_data_sr(6 downto 0)&data_rx; 
              rec_dram_ptr              <=  (rec_dram_ptr+1) mod 8;
              
              -- If the whole byte was received
              if (rec_dram_ptr=7) then
                rec_dram_bind <= (rec_dram_bind+1) mod 4;
                case rec_dram_bind is
                  when  0 =>
                    rec_dram(data_pointer/32) <= rec_data_sr(6 downto 0) &
                                                 data_rx &
                                                 "000000000000000000000000";
                  when  1 =>
                    rec_dram(data_pointer/32)(23 downto 0) <= 
                                                rec_data_sr(6 downto 0) &
                                                data_rx &
                                                "0000000000000000";
                  when  2 =>
                    rec_dram(data_pointer/32)(15 downto 0) <= 
                                                rec_data_sr(6 downto 0) &
                                                data_rx &
                                                "00000000";
                  when  3 =>
                    rec_dram(data_pointer/32)(7 downto 0) <= 
                                              rec_data_sr(6 downto 0)&
                                              data_rx;
                  when others =>
                      report "Unknown state" severity error;
                      PC_State <= error;
                end case;   
              end if;
              
              if(data_pointer>511-data_size)then
              --if(data_pointer>0)then
                data_pointer            <=  data_pointer-1;
              else
                PC_State      <=  crc;
                FSM_Preset    <=  '1';
              end if;
            end if;
            
          end if;
          
          
      --------------------------------------------------------------------------
      -- CRC Field
      --------------------------------------------------------------------------
      when crc =>
        
          if(FSM_Preset='1')then
            
            ssp_reset_r     <=  '0';
            
            --CRC for normal CAN is always 15 bit, even if non-standard frame
            -- longer than 8 bytes is on the bus!
            if ((OP_State  = transciever and tran_frame_type = NORMAL_CAN) or
                (OP_State = reciever    and rec_frame_type_r = NORMAL_CAN))then
              data_pointer  <=  14; --CRC 15
              crc_src       <=  CRC_15_SRC;
            else    
              if(unsigned(dlc_int)>10)then --More than 16 bytes, CRC 21
                data_pointer  <=  20;
                crc_src<=CRC_21_SRC;
              else --Less than 16 bytes, CRC 17
                data_pointer  <=  16;
                crc_src       <=  CRC_17_SRC;   
              end if;
            end if;
           
            
            --Chaning the bit Stuffing for CRC of CAN FD
            if(OP_State=transciever and tran_frame_type=FD_CAN) or 
              (OP_State=reciever    and rec_frame_type_r=FD_CAN)
            then
              fixed_stuff_r     <=  '1';
              fixed_destuff_r   <=  '1';
              stuff_length_r    <=  std_logic_vector(
                                    to_unsigned(FD_STUFF_LENGTH,3));
              destuff_length_r  <=  std_logic_vector(
                                    to_unsigned(FD_STUFF_LENGTH,3));
              fixed_CRC_FD      <=  '1';
              fixed_CRC_FD_rec  <=  '1';
              
              --Stuff count is transmitted only if ISO option is configured!!
              if(drv_fd_type=ISO_FD)then
                crc_state       <= stuff_count;
                stl_pointer     <= 3;
              else
                crc_state       <= real_crc;
                crc_enable_r    <= '0';
              end if;
              
            else
              fixed_CRC_FD      <=  '0';
              fixed_CRC_FD_rec  <=  '0';
              crc_state         <=  real_crc;
              crc_enable_r      <=  '0';
            end if;
            
            rec_crc_r           <=  (OTHERS => '0');
            FSM_Preset          <=  '0';
            --sync_control_r<=RE_SYNC;
          else
            
            case crc_state is
              when stuff_count =>
                                
                if(OP_State=transciever)then
                  if(tran_trig='1')then 
                      --Transmitting stuff count field and the parity
                      if (stl_pointer>0)then
                        data_tx_r   <= stuff_count_grey(stl_pointer-1);
                      else
                        data_tx_r   <= stuff_parity;
                      end if;                                 
                  end if;
                else
                  data_tx_r<=RECESSIVE;
                end if;
                
                if(rec_trig='1')then          
                    if (stl_pointer>0)then
                      stl_pointer                   <=  stl_pointer-1;
                      rx_count_grey(stl_pointer-1)  <=  data_rx;
                    else 
                      crc_state                     <=  real_crc;
                      crc_enable_r                  <=  '0';
                      rx_parity                     <=  data_rx; 
                    end if;
                end if; 
                                
              when real_crc =>
                
                if(OP_State=transciever)then
                 if(tran_trig='1')then
                    case crc_src is
                      when CRC_15_SRC => data_tx_r  <=  crc15(data_pointer);
                      when CRC_17_SRC => data_tx_r  <=  crc17(data_pointer);
                      when CRC_21_SRC => data_tx_r  <=  crc21(data_pointer);
                      when others=> 
                            data_tx_r             <=  data_tx_r;
                            unknown_state_Error_r <=  '1'; 
                            PC_State              <=  error;
                            FSM_preset            <=  '1';
                    end case;
                  end if;
                end if;
                
                if(rec_trig='1')then
                    rec_crc_r <= rec_crc_r(19 downto 0)&data_rx;
                    if(data_pointer=0)then
                      PC_State              <=  delim_ack;
                      FSM_Preset            <=  '1';
                    else
                      data_pointer          <=  data_pointer-1;
                    end if;
                end if; 
                   
              when others =>
             end case;
        end if;
      
      
      --------------------------------------------------------------------------
      --CRC Delimiter, Acknowledge and Acknowledge delim
      --------------------------------------------------------------------------
      when delim_ack =>
          if(FSM_Preset='1')then
            control_pointer   <=  0;
            FSM_Preset        <=  '0';
            ack_recieved      <=  '0';
            sec_ack           <=  '0';
            --Ack field is not coded by bit stuffing
            stuff_enable_r    <=  '0';
            destuff_enable_r  <=  '0';
            
            fixed_stuff_r     <=  '0';
            fixed_destuff_r   <=  '0';
                  
            --Crc checking (for both reciever, and also for transciever if 
            -- loopbacked CRC matches the calculated one!
            if((rec_crc_r="000000"&crc15) or
						   (rec_crc_r="0000"&crc17) or 
						   (rec_crc_r=crc21))
						then
              --Checking stuff count and parity of ISO FD
              if( ((OP_State=transciever  and tran_frame_type   = FD_CAN) or 
                   (OP_State=reciever     and rec_frame_type_r  = FD_CAN)
                   )and 
                   (drv_fd_type = ISO_FD)
                 )
               then
                  
                  if(rx_parity=stuff_parity and stuff_count_grey=rx_count_grey)
                  then
                    crc_check <=  '1';
                  else
                    crc_check <=  '0';
                  end if;
                  
               else    
                crc_check     <=  '1';
               end if;
                             
            else
              crc_check       <=  '0';
            end if;        
             
          else
            if(OP_State=transciever)then
              if(tran_trig='1')then --Sending data as transciever
                --As transciever we send only recessive bits in these fields
                data_tx_r <=  RECESSIVE; 
              end if;
            
              if(rec_trig='1')then --Monitoring data as transciever
                case control_pointer is
                when 0 => sp_control_r    <=  NOMINAL_SAMPLE;
                          if(tran_brs=BR_SHIFT and tran_frame_type=FD_CAN)then
                            br_shifted    <=  '1';
                          end if;
                          --Note : no condition is necessary because in normal 
                          --frame sp_control remains NORMAL_SAMPLE!
                          control_pointer <=  control_pointer+1;
                          
                          --Bit Error detected on CRC delimiter bit
                          --if(data_rx=DOMINANT) then
                          --  PC_State<=error;
                          --  FSM_Preset<='1';
                          --end if;
                when 1 => if(data_rx=DOMINANT or drv_self_test_ena='1')then 
                            ack_recieved    <=  '1';
                            control_pointer <=  control_pointer+1;
                          else
                            if(sec_ack='0')then
                              --Still OK, just one recesive sampled by transciever
                              sec_ack       <=  '1';
                              ack_recieved  <=  ack_recieved;
                            else
                              --Three recessive bits registered -->
                              -- no acknowledge, ACK Error
                              ack_recieved  <=  '0';
                              sec_ack       <=  sec_ack;
                              PC_State      <=  error;
                              ack_Error_r   <=  '1'; 
                              FSM_Preset    <=  '1';
                            end if;  
                          end if;
                when 2 => --This state represents ack delimiter
                          if(ack_recieved='1')then
                            PC_State        <=  eof;
                          else
                            PC_State        <=  error;
                          end if;
                          FSM_Preset        <=  '1';
                when others=> unknown_state_Error_r <=  '1'; 
                              PC_State              <=  error;
                              FSM_preset            <=  '1';
                end case;
              end if;
            elsif(OP_State=reciever)then
              if(tran_trig='1')then
                case control_pointer is
                
                  --CRC delimiter bit
                  when 0 => data_tx_r       <=  RECESSIVE;
                            --Switching the bit rate back
                            --sp_control_r  <=  NOMINAL_SAMPLE;
                            --
                  
                  --Send acknowledge if CRC Match and is not forbidden
                  when 1 => if(crc_check='1' and drv_ack_forb='0')then
                              data_tx_r     <=  DOMINANT;
                            else
                              data_tx_r     <=  RECESSIVE;
                            end if;
                            
                            --If Bus Monitoring mode is enabled then data has to
                            --be looped back before sending on the bus!
                            if(drv_bus_mon_ena='1')then 
                              int_loop_back_ena_r <=  '1';
                            end if;
                            
                  when 2 => data_tx_r                   <=  RECESSIVE;
                            --Loop-Back is turned off either in Bus Mon mode or 
                            --normal mode
                            int_loop_back_ena_r         <=  '0'; 
                            
                  when others => unknown_state_Error_r  <=  '1'; 
                                 PC_State               <=  error;
                                 FSM_preset             <=  '1';
                end case;
              end if;
              
              if(rec_trig='1')then
                case control_pointer is
                  when 2 =>  if(ack_recieved='1' and crc_check='1')then
                              PC_State      <=  eof;
                             else
                              PC_State      <=  error;
                              ack_Error_r   <=  '1';
                              inc_one_r     <=  '1';
                             end if; 
                             FSM_preset     <=  '1';
                  
                  --Acknowledge sent but recessive monitored
                  when 1 =>  if(data_tx_r=DOMINANT and data_rx=RECESSIVE)then
                              PC_State      <=  error;
                              FSM_Preset    <=  '1';
                             end if;
                             if(data_rx=DOMINANT or drv_self_test_ena='1')then
                              ack_recieved  <=  '1';
                             end if;
                  when 0 =>  sp_control_r   <=  NOMINAL_SAMPLE;
                    
                             if(rec_brs_r=BR_SHIFT and rec_frame_type_r=FD_CAN)then
                              br_shifted    <=  '1';
                             end if;
                   
                             if(data_rx=DOMINANT)then --CRC Delimiter bit
                                PC_State      <=  error;
                                FSM_Preset    <=  '1';
                                form_Error_r  <=  '1';
                                --Increment recieve error counter by one!
                                inc_one_r     <=  '1';
                             end if;
                  when others=>
                end case;        
                control_pointer               <=  control_pointer+1;
              end if;
            else
              --If we get here it is absolute fail... 
              --(Not transciever, Not reciever) in ACK field...
              unknown_state_Error_r <=  '1'; 
              PC_State              <=  error;
              FSM_preset            <=  '1';
            end if;
          end if;
      
      --------------------------------------------------------------------------
      --End of Frame
      --------------------------------------------------------------------------
      when eof =>
          if(FSM_Preset='1')then
            FSM_Preset        <=  '0';
            --Eof is not bit-Stuffed
            stuff_enable_r    <=  '0';
            destuff_enable_r  <=  '0';
            control_pointer   <=  6;
          else
            if(tran_trig='1')then
              data_tx_r       <=  RECESSIVE;
            end if;           
            if(rec_trig='1')then
            
             --Detection of dominant bit during EOF means error, 
             --or Overload condition 
             if(data_rx=DOMINANT)then
                 --if(control_pointer>1)then
                  PC_State      <=  error;  
                  FSM_Preset    <=  '1';
                  if(OP_State=reciever)then
                    --Increment recieve error counter by one!
                    inc_one_r   <=  '1';
                  end if;
                -- else
                --  PC_State      <=  overload;  
                --  FSM_Preset    <=  '1';
                --end if;
             else 
              if(control_pointer>0)then
                control_pointer <=  control_pointer-1;
                --Message is recieved as valid one bit before the end of frame
                if(control_pointer=1 and OP_State=reciever)then
                  rec_valid_r   <=  '1';
                  dec_one_r     <=  '1';
                end if; 
              else
                if(OP_State=transciever)then --Message is sucessfully transcieved
                  tran_valid_r        <=  '1';
                  dec_one_r           <=  '1';            
                  txt_hw_cmd.unlock   <=  '1';
                  txt_hw_cmd.valid    <=  '1';
                  is_txt_locked       <=  '0';
                  retr_count          <=  0;
                end if;
                PC_State        <=  interframe; 
                FSM_Preset      <=  '1';
              end if;
             end if;
            end if;
           end if;
           
      --------------------------------------------------------------------------
      --Interframe space
      --------------------------------------------------------------------------
      when interframe =>
          if(FSM_Preset='1')then
            FSM_Preset        <=  '0';
            control_pointer   <=  2;
            interm_state      <=  intermission;
            stuff_enable_r    <=  '0';
            destuff_enable_r  <=  '0';
          else
            case interm_state is
            when intermission =>
                  if(tran_trig='1')then --Transmition of intermission
                    data_tx_r <=  RECESSIVE;
                    if(control_pointer<2)then 
                      --Hard synchronisation during second or third bit of 
                      -- intermission!
                      sync_control_r  <=  HARD_SYNC; 
                    end if;
                  end if;
                        
                  --------------------------------------------------------------
                  -- We transfer to SOF When sample dominant or detect edge
                  --------------------------------------------------------------
                  if( hard_sync_edge = '1')then
                    PC_State          <=  sof;
                    sof_skip          <=  '0'; 
                    crc_enable_r      <=  '1';
                    FSM_preset        <=  '1';
                  elsif(rec_trig='1')then --Reciving intermission bits
                    
                    if(control_pointer>0)then
                      control_pointer <=  control_pointer-1;
                    end if;
                    
                    if(data_rx=RECESSIVE)then 
                      if(control_pointer=0)then --Third recessive bit sampled
                        if(OP_State=transciever and 
                           error_state=error_passive)
                        then
                          --Transmitting error passive node always suspends
                          -- after transmition
                          interm_state    <=  suspend;
                          --Preseting the pointer for Suspend field. In error
                          --field it is preset by FSM_preset
                          control_pointer <=  7;   
                        else
                          interm_state    <=  interm_idle; 
                        end if;
                     end if;
                    
                    --Bugfix 30.6.2016
                    elsif(data_rx=DOMINANT)then
                      if(control_pointer=0)then --Third recessive bit sampled
                        PC_State          <=  sof;
                        sof_skip          <=  '0'; 
                        crc_enable_r      <=  '1';
                        FSM_preset        <=  '1';
                      else
                        PC_State          <=  overload;
                        FSM_preset        <=  '1'; 
                      end if;
                    end if;  
                    
                  end if;  
                  
            when suspend =>
                  sync_control_r<=HARD_SYNC;
                  if(tran_trig='1')then
                    data_tx_r             <=  RECESSIVE;
                  end if;
                  
                  if( hard_sync_edge = '1')then
                    PC_State          <=  sof;
                    sof_skip          <=  '0'; 
                    crc_enable_r      <=  '1';
                    FSM_preset        <=  '1';
                    set_reciever_r    <=  '1';
                  elsif(rec_trig='1')then
                    if(control_pointer>0)then
                      control_pointer     <=  control_pointer-1;
                    end if;
                     if(control_pointer=0)then  
                      if ((drv_bus_mon_ena       = '0') and
                         --Next data are availiable
                         (tran_frame_valid_in  = '1'))
                      then
                          PC_State        <=  sof;
                          is_txt_locked   <=  '1';
                          txt_hw_cmd.lock <=  '1';
                          sof_skip        <=  '0';
                          crc_enable_r    <=  '1';
                          FSM_preset      <=  '1';
                          
                          --Bug fix 28.6.2016
                          --Preset reciever already here, not in SOF, otherwise
                          -- if there is nothing to transmitt SOF will be trans-
                          -- mitted anyway. If we were transmitter of previous 
                          -- message and we have nothing more to transmitt and 
                          -- we turn reciever, we dont want SOF to be
                          -- tranmsmitted by reciever!!
                          set_reciever_r      <=  '1';
                      
                      else
                          interm_state    <=  interm_idle;
                      end if;
                    end if;
                  end if;
                  
            when interm_idle =>
                  --Note : Integrating condition has to be checked, otherwise 
                  --       any dominant bit can be interpreted as SOF, therefore 
                  --       causing transmittion of Error_frame
                  --Signal for OP_State machine that bus is idle
                  is_idle_r         <=  '1';
                  
                  if(tran_trig='1')then
                    data_tx_r       <=  RECESSIVE;
                  end if;  
                  if(OP_State /= integrating )then
                    sync_control_r  <=  hard_sync;
                  end if;
                  
                 if( hard_sync_edge = '1' and (OP_State /= integrating) )then
                    PC_State          <=  sof;
                    sof_skip          <=  '0';
                    crc_enable_r      <=  '1';
                    FSM_preset        <=  '1';
                    
                 elsif(rec_trig='1' and (OP_State /= integrating) )then 
                      
                      -- If any frame is available here for transmission 
                      -- we lock it already here. If we moved to SOF and
                      -- locked only there, the frame might have been
                      -- aborted in that one clock cycle! Thus we would
                      -- then lock invalid frame!
                      if ((drv_bus_mon_ena = '0') and 
  												 -- Next data are availiable
                         (tran_frame_valid_in  = '1')) 
                      then
                        PC_State        <=  sof;
                        is_txt_locked   <=  '1';
                        txt_hw_cmd.lock <=  '1';
                        sof_skip        <=  '0';                  
                        crc_enable_r    <=  '1';
                        FSM_Preset      <=  '1';
                        
                        -- In case that TX Arbitrator provides different frame for
           	            -- us, we need to erase the retranmsitt counter
           	            if (txtb_changed = '1') then
                          retr_count    <= 0;
                        end if;
                      else
                        FSM_Preset      <=  '0';                    
                      end if;
                    
                 end if;
                
            when others =>
                  unknown_state_Error_r <=  '1'; 
                  PC_State              <=  error;
                  FSM_preset            <=  '1';
            end case;
          end if;
          
      --------------------------------------------------------------------------
      --Error frame
      --------------------------------------------------------------------------
      when error =>
            if(FSM_Preset='1')then
              FSM_Preset        <=  '0';
              control_pointer   <=  6; --Pointer for sending the error flag
              stuff_enable_r    <=  '0';
              fixed_stuff_r     <=  '0';
              fixed_destuff_r   <=  '0';
              
              --Pointer for recieving the superposition of error flags
              tran_pointer      <=  12; 
              err_frame_state   <=  err_flg_sup;
              destuff_enable_r  <=  '0';
              
              --If Error appears within FD Data Phase node has to switch back
              sp_control_r      <=  NOMINAL_SAMPLE;
              crc_enable_r      <=  '0';
              
              --Here we force data to be dominant event if not trigger is used! 
              --This helps in situations when error is detected during Data bit 
              --rates!!
              data_tx_r         <=  DOMINANT;
              
              --If unit is transciever and Error appears then rettransmitt
              -- counter should be incremented
              if(OP_State=transciever)then
                if ((drv_retr_lim_ena='0') or --Retransmitt limit is disabled 
                    (drv_retr_lim_ena='1' and --Enabled, but not reached
                     retr_count<to_integer(unsigned(drv_retr_th))))
                then
                  retr_count         <=  retr_count + 1 mod 16;            
                  txt_hw_cmd.unlock  <=  '1';
                  txt_hw_cmd.err     <=  '1';
                else
                  
                  -- Retransmitt limit reached, signal transmission failure
                  -- Erase the retransmitt counter, since the next frame
                  -- can be from the same buffer, but it can be different frame!
                  -- Thus retr_counter wont be erased on "txt_buf_changed"!
                  retr_count          <=  0;
                  txt_hw_cmd.unlock   <=  '1';
                  txt_hw_cmd.failed   <=  '1';
                end if;
                
                is_txt_locked         <=  '0';
               
                --Transmitter started to transmitt error flag -> increase by 8 
                -- except ack error for error passive
                --Or Stuff Error appeared during arbitration!
                if((error_state=error_passive and ack_error_r='1') or 
									(stuff_err_arb_int='1'))
							  then
                  inc_eight_r   <=  '0';
                else  
                  inc_eight_r   <=  '1';
                end if;
              end if;
              
              --If Bus Monitoring mode is enabled then data has to be looped 
              -- back before sending on the bus!
              if(drv_bus_mon_ena='1')then 
                int_loop_back_ena_r <=  '1';
              end if;
              
            else
              
              case  err_frame_state is
              
                --Transmition of error flag and reception of Error 
                --flag superposition 
                when err_flg_sup =>
                                              
                        if(tran_trig='1')then
                          if(control_pointer>0 and error_state=error_active)then
                            data_tx_r <=  DOMINANT; --Sending active error flag
                          else 
                            --Sending passive error flag or one bit after
                            --active error flag!
                            data_tx_r <=  RECESSIVE;
                          end if;
                        end if;
                        
                        if(rec_trig='1')then 
                          
                          --Bit error detection during active error flag
                          if(data_tx_r=DOMINANT and data_rx=RECESSIVE)then
                            FSM_Preset    <=  '1';
                            
                            if(OP_State=reciever)then
                              --Reciever error counter increased by 8.s
                              inc_eight_r <=  '1';
                            end if;
                          else
                            if(error_state=error_active)then
                              
                              --Decreasing counters
                              if(control_pointer>0)then
                                control_pointer <=  control_pointer-1;
                              end if;
                              if(tran_pointer>0)then
                                tran_pointer    <=  tran_pointer-1;
                              end if;
                              
                              --Only in the last bit, if detected earlier then 
                              --bit error apeared
                              if(data_rx=RECESSIVE)then
                                err_frame_state <=  err_delim;
                                control_pointer <=  6; 
                                --Note: this has to be 6 not 7 (duration of 
                                -- err_delim is 8) because one bit is sent 
                                -- recessive and detected
                                
                               --We accepted 13-th consecutive DOMINANT bit -> 
                               -- Error again??
                              elsif(data_rx=DOMINANT and tran_pointer=0)then
                                FSM_preset            <=  '1';
                                int_loop_back_ena_r   <=  '0';
                                if(OP_State=reciever)then
                                  inc_eight_r         <=  '1'; 
                                  --For reciever error counter increased by 8. 
                                  --For transciever this is done in FSM_Preset!
                                end if;
                              end if;
                              
                              --This condition is causes that Reciever that was 
                              -- the first to detect the error has error counter
                              -- increased by 8! Transciever counter is increased
                              -- in FSM preset! Any next reciever that will hook
                              -- up, will hook up at the end of error flag super-
                              -- position Thus after transmitting its error flag
                              -- there will be recessive bit, not dominant!
                              if((tran_pointer=5) and (OP_State=reciever))then
                               --First bit detected Dominant after active error
                               --flag was sent!
                                inc_eight_r   <=  '1';
                              end if;
                            
                            elsif(error_state=error_passive)then
                              
                              --Storing last recieved data
                              err_pas_bit_val <=  data_rx;
                                
                              --Detecting 6 consecutive bits of equal polarity
                              if(control_pointer=6)then
                                control_pointer <=  control_pointer-1;
                              else
                                if(data_rx=err_pas_bit_val)then
                                  if(control_pointer>1)then
                                    control_pointer   <=  control_pointer-1;
                                  else --Six equal consecutive bits detected
                                    err_frame_state   <=  err_delim;
                                    control_pointer   <=  6; 
                                  end if;
                                else
                                  control_pointer     <=  5; 
                                  --Restart the detection (with one bit less
                                  --because then the first bit is already the 
                                  --bit where mismatch appeared)
                                end if;
                              end if;                             
                             else --Node must be Bus-off here
                                PC_State    <=  off;
                                FSM_Preset  <=  '1';
                             end if;
                          end if;
                        end if;  
                        
                when err_delim =>
                        if(tran_trig='1')then 
                          data_tx_r   <=  RECESSIVE;
                        end if;
                        if(rec_trig='1')then
                          if(control_pointer>0)then
                            control_pointer <=  control_pointer-1;
                          else
                            if(data_rx=DOMINANT)then
                              PC_State  <=  overload;
                            else
                              PC_State  <=  interframe;
                            end if;
                            FSM_Preset          <=  '1';
                            int_loop_back_ena_r <=  '0';
                          end if;
                        end if;
                when others=>
                      unknown_state_Error_r   <=  '1'; 
                      PC_State                <=  error;
                      FSM_preset              <=  '1';
                      int_loop_back_ena_r     <=  '0';
              end case;
            end if;
      
      
      --------------------------------------------------------------------------
      --Overload frame
      --------------------------------------------------------------------------
      when overload =>  
            if(FSM_Preset='1')then
              FSM_Preset        <=  '0';
              control_pointer   <=  6; --Pointer for sending the overload flag
              --Pointer for recieving the superposition of ovverload flags
              tran_pointer      <=  12;
              ovr_frame_state   <=  ovr_flg_sup;
              stuff_enable_r    <=  '0';
              destuff_enable_r  <=  '0'; 
              crc_enable_r      <=  '0';
              
              --If Bus Monitoring mode is enabled then data has to be 
              --looped back before sending on the bus!
              if(drv_bus_mon_ena='1')then 
                int_loop_back_ena_r <=  '1';
              end if;
                    
            else
              case  ovr_frame_state is
                --Transmition of overload flag and reception of 
                --overload flag superposition 
                when ovr_flg_sup =>
                        
                        if(tran_trig='1')then
                          if(control_pointer>0)then
                            data_tx_r <=  DOMINANT; --Sending overload flag
                          else 
                            data_tx_r <=  RECESSIVE;
                          end if;
                        end if;
                        
                        if(rec_trig='1')then
                          
                          
                          if(control_pointer>0)then
                            control_pointer <=  control_pointer-1;
                          end if;
                          if(tran_pointer>0)then
                            tran_pointer    <=  tran_pointer-1;
                          end if;
                          
                          if(data_rx=RECESSIVE)then
                            --Still sending overload flag, but recessive 
                            --detected -> error frame + increase counter 
                            if(control_pointer>0)then
                              PC_State        <=  error; 
                              if(OP_State=reciever)then
                                --For reciever error counter increased by 8.
                                --For transciever this is done in FSM_Preset!
                                inc_eight_r   <=  '1';
                              end if;
                              FSM_Preset      <=  '1';
                            else
                              ovr_frame_state <=  ovr_delim;
                              control_pointer <=  7; 
                            end if;
                          
                          --We accepted 13-th consecutive DOMINANT bit in 
                          --superposition --> 
                          elsif(data_rx=DOMINANT and tran_pointer=0)then
                            if(OP_State=reciever)then
                              --For reciever error counter increased by 8. For 
                              --transciever this is done in FSM_Preset!
                              inc_eight_r <=  '1'; 
                            end if;
                            PC_State            <=  error;
                            FSM_preset          <=  '1';
                            int_loop_back_ena_r <=  '0';
                          end if;
                        end if;
                        
                when ovr_delim => --Overlad delimiter
                        if(tran_trig='1')then 
                          data_tx_r   <=  RECESSIVE;
                        end if;
                        if(rec_trig='1')then
                          if(control_pointer>0)then
                            control_pointer <=  control_pointer-1;
                          else
                            PC_State            <=  interframe;
                            FSM_preset          <=  '1';
                            int_loop_back_ena_r <=  '0';
                          end if;  
                        end if;
                when others=>
                      unknown_state_Error_r <=  '1'; 
                      PC_State              <=  error;
                      FSM_preset            <=  '1';
                      int_loop_back_ena_r   <=  '0';
              end case;
            end if;
            
      --------------------------------------------------------------------------
      --Unit is turned off
      --------------------------------------------------------------------------
      when off =>
           
            if(drv_ena=ENABLED)then
              if(not (error_state=bus_off))then
                FSM_Preset    <=  '1';
                PC_State      <=  interframe;  
              end if;
            end if;
  
      when others=>
            unknown_state_Error_r <=  '1'; 
            PC_State              <=  error;
            FSM_preset            <=  '1';
      end case;
    end if;
    end if;
  end process;

end architecture;