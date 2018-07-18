--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisors and co-authors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--                  OP_State did not manage to be acutalized and thus FSMpreset 
--                  branch was executed for transmitter! THus control_pointer 
--                  was set totally wrong, and reciever was confused... Error 
--                  frame was later on detected OK. This error in some cases 
--                  behaved just like CRC error! Now transition from arbitration
--                  to control is always done one clock cycle later than imme-
--                  diately after "rec_trig"! It is OK, there is plenty of time, 
--                  since we are still in NOMINAL bit time at this point! 
--                  "delay_control_trans" register is used for this delay!
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
--                   "txtb_changed" signal will be active. This signal is
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
--   17.02.2018   1. Removed obsolete "frame_store", its functionality is fully
--                   replaced with frame_lock
--                2. Added increment of the rettransmitt counter on arbitration
--                   lost. This is desirable for new implementation of the
--                   TXT Buffer finite state machine.
--   21.02.2018   Removed obsolete "frame_swap" since it is not necessary with
--                prioritized TX buffers.
--   22.02.2018   Added "sof_pulse" to signalize start of frame for rest of the
--                design.
--   23.02.2018   Swapped Identifier Base and Extended on received identifier.
--     6.4.2018   Added direct addressing of identifier from Protocol control.
--                In SOF TXT buffer pointer is set to identifier word and
--                Identifier is stored in the first cycle of Arbitration field.
--   19.5.2018    1. Added "store_data", "store_metadata", "rec_abort" signals
--                   as a storing protocol between CAN Core and RX Buffer for
--                   continous storing of CAN frame during reception.
--                2. Added endian swap for transceived and received data to 
--                   have Data byte 0 at address 0.
--   29.5.2018    Removed obsolete "sof_skip" signal. Transition from Interframe
--                to SOF is by received synchronsation edge!
--    5.6.2018    Added "data_tx_index". Separated CRC check to "crc_valid"
--                signal! Added "parity_valid" signal. Added "alc_val" for
--                arbitration lost capture combinational decoder! Added
--                "control_pointer_non_zero". Changed counting on
--                "control_pointer" to 0 in delim_ack! Now counting is done
--                from value to zero on all instances of control_pointer!
--	 22.6.2018    Bug-fix of bit stuffing, bit destuffing. Turned of upon
--                reception of first bit of delim_ack. Thisway if last 5
--                bits of CRC are matching one stuff bit is still inserted as
--                expected!
--   10.7.2018    Changed length of data length field for DLC > 8 in case of
--                CAN 2.0 frame! For CAN 2.0 frame DLC higher than 8 should be
--                interpreted as 8!
--   13.7.2018    1. Removed "unknown_state_Error_r" since it was unused. Unified
--                   all "when others" statements to go to "error" state an cause
--                   error frame transmission! This is however synthesis tool
--                   dependent! If FSM synthesis on invalid state (e.g. glitch)
--                   would jump to reset state, then node would become off, and
--                   communication would not continue! If synthesis tool would
--                   use "others" to detect invalid FSM state, and go to "error"
--                   state, then this would be "safety" feature for possible
--                   glitches. From CAN Node perspective, this would be another
--                   "internal" error, which would cause transmission of error
--                   frame! Such a behaviour is not defined by standard, but it
--                   is logical to do it like so!
--                2. Removed "bit_err_ena" since it was unused! Selection of
--                   valid bit error is done by Fault confinement!
--   13.8.2018    fixed_CRC_FD removed since fixed stuff bit before CRC field
--                is inserted by Bit Stuffing and discarded by Bit Destuffing
--                automatically upon detection of 0 -> 1 transition on
--                "fixed_stuff" / "fixed_destuff" signals.
--   2.9.2018     1. Change from "off" to "idle" directly to "interm_idle", to
--                   avoid interpretation of SOF as Overload flag direclty
--                   after integration!
--                2. Added explicit Form Error detection in "delim_ack"!
--   7.9.2018     Fixed duration of overload flag superposition and error flag
--                superposition. After accepting 14 DOMINANT Bits (not 13
--                as before), error counter is incremented again!
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CAN_FD_frame_format.all;
use work.CAN_FD_register_map.all;

use work.endian_swap.all;

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
    signal txt_buf_ptr            :out  natural range 0 to 19;
    
    signal txtb_changed           :in   std_logic;
    
    -------------------------
    -- RX Buffer interface
    -------------------------
    signal rec_ident              :out  std_logic_vector(28 downto 0);
    signal rec_dlc                :out  std_logic_vector(3 downto 0);
    signal rec_is_rtr             :out  std_logic;
    signal rec_ident_type         :out  std_logic;
    signal rec_frame_type         :out  std_logic;
    signal rec_brs                :out  std_logic;
    signal rec_crc                :out  std_logic_vector(20 downto 0);
    signal rec_esi                :out  std_logic;
    
    -- Metadata are received OK, and can be stored in RX Buffer!
    signal store_metadata         :out  std_logic;

    -- Cancel storing of frame in RX Buffer.
    signal rec_abort              :out  std_logic;

    -- Data words is available and can be stored in RX Buffer!
    signal store_data             :out  std_logic;
    signal store_data_word        :out  std_logic_vector(31 downto 0);

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
    signal alc                    :out  std_logic_vector(7 downto 0);

    --Unknown Operational state -> Error frame!
    signal unknown_OP_state		  :in 	std_logic;
    
    -------------------------------
    --Fault confinement Interface--
    -------------------------------
    --Fault confinement state
    signal error_state            :in   error_state_type;
    
    --Error signals for fault confinement
    signal form_Error             :out  std_logic; --Form Error
    signal CRC_Error              :out  std_logic; --CRC Error
    signal ack_Error              :out  std_logic; --Acknowledge error    
    
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
    
    --Synchronisation edge validated by prescaler!!!
    signal hard_sync_edge         :in   std_logic;
    
    --Internal loopBack enabled (for Bus monitoring mode)
    signal int_loop_back_ena      :out  std_logic;
    
    -- One clock cycle long pulse in SOF
    signal sof_pulse              :out  std_logic;
    
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
  signal sync_control_r           :     std_logic_vector(1 downto 0);
  signal alc_r                    :     std_logic_vector(7 downto 0);

  signal form_Error_r             :     std_logic; --Form Error
  signal CRC_Error_r              :     std_logic; --CRC Error
  signal ack_Error_r              :     std_logic; --Acknowledge error
  
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
  signal sof_pulse_r              :     std_logic;
  
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

  -- Metadata are received OK, and can be stored in RX Buffer!
  signal store_metadata_r         :     std_logic;

  -- Cancel storing of frame in RX Buffer.
  signal rec_abort_r              :     std_logic;

  -- Data words is available and can be stored in RX Buffer!
  signal store_data_r             :     std_logic;
  signal store_data_word_r        :     std_logic_vector(31 downto 0);
    
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
  
  -- Arbitration lost capture value
  signal alc_val                  :     std_logic_vector(7 downto 0);
  
  --------------------------
  --Control field registers-
  --------------------------
  --Pointer for counting DLC bits 
  signal control_pointer          :     natural range 0 to 7;
  signal control_pointer_non_zero :     boolean;
  
  --Signal for presetting the state machine of control field into correct
  -- state  based on type of transcieved/recieved frame
  signal FSM_preset               :     std_logic; 
  
  --Register for transcieving the data in control field bits
  signal ctrl_tran_reg            :     std_logic_vector(7 downto 0);
  
  --Register for transcieving the data in control field bits
  signal dlc_int                  :     std_logic_vector(3 downto 0);
 
 
  ------------------------
  --Data field registers--
  ------------------------
  
  -- Pointer for data transmission
  signal data_pointer             :     natural range 0 to 511;

  -- Pointer for TX data index within TX word.
  -- (Shift register not used, due to additional delay after load of
  --  txt word on output). 
  signal data_tx_index            :     natural range 0 to 31;


  --Shift register for data reception
  signal rec_data_sr              :     std_logic_vector(7 downto 0);
  
  --Register for counting received bytes in shift register
  signal rec_word_ptr             :     natural range 0 to 7;
  
  --Byte index into RAM word (store_data_word)
  signal rec_word_bind            :     natural range 0 to 3;

  
  --Pointer directly to TXT buffer to get the data
  signal txt_buf_ptr_r            :     natural range 0 to 19;

  -- Data word to transmit after endian swapping.
  signal tx_data_word             :     std_logic_vector(31 downto 0);
  
  -----------------------
  --CRC field registers--
  -----------------------
  
  --CRC Source , 00-CRC15, 01-CRC17, 10-CRC21,11-Invalid
  signal crc_src                  :     std_logic_vector(1 downto 0);
  
  --Recieved CRC matches the calculated one
  signal crc_check                :     std_logic;
  
  --Pointer for transcieving the stuf length field
  signal stl_pointer              :     natural range 0 to 3;

  -- True if received CRC is matching TX CRC
  signal crc_and_parity_valid     :     boolean;
  signal crc_valid                :     boolean;
  signal parity_valid             :     boolean;

  
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
  
  -----------------------------------
  --Registers to output propagation--
  -----------------------------------
  PC_State_out          <=  PC_State;
  alc                   <=  alc_r;
  data_tx               <=  data_tx_r;
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
  --Synchronisation control
  sync_control          <=  sync_control_r;
  
  --Recieved data registers to output propagation
  rec_ident(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) <= rec_ident_base_sr;
  rec_ident(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L)   <= rec_ident_ext_sr;
  
  rec_dlc               <=  rec_dlc_r;
  rec_is_rtr            <=  rec_is_rtr_r;
  rec_ident_type        <=  rec_ident_type_r;
  rec_frame_type        <=  rec_frame_type_r;
  rec_brs               <=  rec_brs_r;
  rec_crc               <=  rec_crc_r;
  rec_esi               <=  rec_esi_r;

  store_metadata        <=  store_metadata_r;
  rec_abort             <=  rec_abort_r;
  store_data            <=  store_data_r;
  store_data_word       <=  store_data_word_r;

  ack_recieved_out      <=  ack_recieved;
  

  --Pointer into TXT Buffer
  txt_buf_ptr           <=  txt_buf_ptr_r;
  
  sof_pulse             <=  sof_pulse_r;

  -- TX Data word endian swap
  tx_data_word          <= endian_swap_32(tran_data);
  
  -----------------------
  --Auxiliarly vectors
  -----------------------
  aux_tx_rx                 <=  data_tx_r & data_rx;
  aux_tran_frame_ident_type <=  tran_frame_type&tran_ident_type;

  -------------------------------------------
  -- Auxiliarly control pointer signals
  -------------------------------------------
  control_pointer_non_zero  <= true when (control_pointer > 0)
                                    else
                               false;
  
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
   stuff_parity <= '0' when (dst_ctr mod 2) = 0 else
                   '1';

   -----------------------------------------------------------------------------
   -- Comparison of received CRC with calculated CRC
   -----------------------------------------------------------------------------
   crc_and_parity_valid <= crc_valid and parity_valid;

   crc_valid <= true when ((crc_src = CRC_15_SRC) and
                           (rec_crc_r(14 downto 0) = crc15)) 
                          or
                          ((crc_src = CRC_17_SRC) and
                           (rec_crc_r(16 downto 0) = crc17)) 
                          or
                          ((crc_src = CRC_21_SRC) and
                           (rec_crc_r = crc21))
                     else
                false;


   parity_valid <= false when 
                    ((OP_State = transciever and tran_frame_type = FD_CAN) or 
                     (OP_State = reciever    and rec_frame_type_r = FD_CAN)) and
                     (drv_fd_type = ISO_FD) and ((rx_parity /= stuff_parity) or 
                      (stuff_count_grey /= rx_count_grey))
                         else
                   true;

    ----------------------------------------------------------------------------
    -- Arbitration lost capture decoder
    ----------------------------------------------------------------------------
    alc_val(7 downto 5) <=
        ALC_BASE_ID    when (arb_state = base_id) else
        ALC_EXTENSION  when (arb_state = ext_id) else
        ALC_SRR_RTR    when (arb_state = two_bits and tran_pointer = 1) else
        ALC_IDE        when (arb_state = two_bits and tran_pointer = 0) else
        ALC_RTR        when (arb_state = one_bit) else
        (OTHERS => '0');

    alc_val(4 downto 0) <= std_logic_vector(to_unsigned(tran_pointer, 5));


  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  ---- Protocol control process
  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  PC_proc : process(clk_sys, res_n)
  begin
    if (res_n = ACT_RESET) then
        -- Presetting the state
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
                                  to_unsigned(BASE_STUFF_LENGTH, 3));
            
        --Configuring Bit Destuffing
        destuff_enable_r        <=  '0';
        fixed_destuff_r         <=  '0';
        destuff_length_r        <=  std_logic_vector(
                                  to_unsigned(BASE_STUFF_LENGTH, 3));
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
        rec_crc_r               <=  (OTHERS => '0');
        rec_esi_r               <=  '0';
           
        arb_two_bits            <=  (OTHERS => '0');
        arb_one_bit             <=  '0';

        ctrl_tran_reg           <=  (OTHERS => '0');
        dlc_int                 <=  (OTHERS => '0');

        crc_src                 <=  "11";
        crc_check               <=  '0';

        ack_recieved            <=  '0';
        sec_ack                 <=  '0';

        tran_pointer            <=  0;
        alc_r                   <=  (OTHERS => '0');
        
        data_pointer            <=  0;
        data_tx_index           <=  0;

        tran_ident_base_sr      <= (OTHERS => '0');
        tran_ident_ext_sr       <= (OTHERS => '0');

        -- Nulling recieve registers
        rec_ident_base_sr       <=  (OTHERS => '0');
        rec_ident_ext_sr        <=  (OTHERS => '0');
        rec_dlc_r               <=  (OTHERS => '0');
        rec_is_rtr_r            <=  '0';
        rec_ident_type_r        <=  '0';
        rec_frame_type_r        <=  '0';

        -- Commands for RX Buffer for storing received frame
        store_metadata_r        <=  '0';
        rec_abort_r             <=  '0';
        store_data_r            <=  '0';
        store_data_word_r       <=  (OTHERS => '0');

        -- Receive data RAM
        rec_word_ptr            <= 0;
        rec_word_bind           <= 0;
        rec_data_sr             <= (OTHERS => '0');

        -- Pointer directly to TXT Buffer RAM
        txt_buf_ptr_r           <= to_integer(unsigned(
                                    IDENTIFIER_W_ADR(11 downto 2)));

        --Presetting the sampling point control
        sp_control_r            <=  NOMINAL_SAMPLE;
        ssp_reset_r             <=  '0';
        trv_delay_calib_r       <=  '0';
        sync_control_r          <=  NO_SYNC;

        --Error presetting
        form_Error_r            <=  '0';
        CRC_Error_r             <=  '0';
        ack_Error_r             <=  '0';
        set_transciever_r       <=  '0';
        set_reciever_r          <=  '0';

        delay_control_trans     <=  '0';

        rx_parity               <=  '0';
        rx_count_grey           <=  (OTHERS => '0');

        sof_pulse_r             <=  '0';
       
    elsif rising_edge(clk_sys) then
        
        ------------------------------------------------------
        -- Assigning previous values to avoid latch creation
        ------------------------------------------------------

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

        store_metadata_r       <=  '0';
        rec_abort_r            <=  '0';
        store_data_r           <=  '0';
        store_data_word_r      <=  store_data_word_r;

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

        --Register for transcieving the data in control field bits
        ctrl_tran_reg          <=  ctrl_tran_reg;

        -- Internal registered value of DLC field (transcieved or recieved)
        -- Always corresponds to length of Data lenght field!
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
        interm_state           <=  interm_state;
        err_frame_state        <=  err_frame_state;
        err_pas_bit_val        <=  err_pas_bit_val;
        data_tx_index          <=  data_tx_index;

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

        sync_control_r         <=  sync_control_r;

        delay_control_trans    <=  '0';    

        rx_parity              <=  rx_parity;
        rx_count_grey          <=  rx_count_grey;

        rec_data_sr            <=  rec_data_sr;
        rec_word_ptr           <=  rec_word_ptr;
        rec_word_bind          <=  rec_word_bind;

        txt_buf_ptr_r          <=  txt_buf_ptr_r;

        sof_pulse_r            <=  '0';
    
        if (drv_ena = '0') then
            PC_State                    <= off;
          
        elsif (bit_Error_valid = '1' or stuff_Error_valid = '1' or
			   unknown_OP_state = '1')
		then     
            PC_State                    <= error;
            FSM_preset                  <= '1';

            if (OP_State = reciever) then

                -- Bit Error or Stuff Error detected by reciever 
                -- (Control,data,CRC), Increase by one
                inc_one_r               <= '1';

            elsif (OP_State = transciever and PC_State = arbitration) then
                stuff_err_arb_int       <= '1';
            end if;

        elsif (OP_State = transciever and drv_abort_tran = '1') then 
            PC_State                    <= interframe;
            FSM_Preset                  <= '1';
            CRC_enable_r                <= '0';
            stuff_enable_r              <= '0';
            destuff_enable_r            <= '0';
            is_idle_r                   <= '1';  
            txt_hw_cmd.unlock           <= '1';
            txt_hw_cmd.failed           <= '1';
            is_txt_locked               <= '0';

        -- Bug fix 21.6.2016
        elsif (delay_control_trans = '1') then
          PC_State                      <= control;

        else


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    ---- Protocol control state machine
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    case PC_state is 

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Start of frame 
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when sof =>
        if (FSM_preset = '1') then
            
            --------------------------------------------------------------------
            -- Erase internal registers for CAN frame
            --------------------------------------------------------------------
            ack_recieved        <= '0';
            crc_check           <= '0';
            sof_pulse_r         <= '1';
            FSM_preset          <= '0';
            ssp_reset_r         <= '1';
            trv_delay_calib_r   <= '0';
            control_pointer     <=  0;

            -- Erasing the recieved for ID and metadata
            rec_ident_base_sr         <=  (OTHERS => '0');
            rec_ident_ext_sr          <=  (OTHERS => '0');
            rec_dlc_r                 <=  (OTHERS => '0');
            rec_is_rtr_r              <=  '0';
            rec_ident_type_r          <=  '0';
            rec_frame_type_r          <=  '0';
            rec_brs_r                 <=  '0';
            rec_crc_r                 <=  (OTHERS => '0');
            rec_esi_r                 <=  '0';
            rx_parity                 <=  '0';
            rx_count_grey             <=  (OTHERS => '0');

            -- Erasing internal DLC
            dlc_int                   <=  (OTHERS => '0');


            --------------------------------------------------------------------
            -- Go to transceiver or receiver depending on TX frame availability,
            -- Bus monitoring mode. Lock frame if necessary. Note that frame
            -- could have already been locked in interframe space!
            --------------------------------------------------------------------

            -- Bus monitoring mode is disabled! In Bus monitoring mode, frames
            -- are not transmitted! Also, if unit was forced to be receiver
            -- during transition from SUSPEND to SOF, we must NOT lock buffer
            -- for transmission!
            if (drv_bus_mon_ena = '0' and set_reciever_r = '0') then
                
                -- If frame is already locked, or there is on to lock available,
                -- Start transceiving!
                if (is_txt_locked = '1' or (tran_frame_valid_in = '1')) then
                    set_transciever_r   <=  '1';
                    stuff_enable_r      <=  '1';
                    fixed_stuff_r       <=  '0';
                    stuff_length_r      <=  std_logic_vector(
                                             to_unsigned(BASE_STUFF_LENGTH, 3));
                end if;
                
                -- If we dont have frame locked, but we have one available
                -- the we just lock it!
                if (is_txt_locked = '0' and (tran_frame_valid_in = '1')) then
                    txt_hw_cmd.lock     <=  '1';
                    is_txt_locked       <=  '1';
                   
                    -- In case that TX Arbitrator provides frame from different
                    -- buffer, we must erase error counter!
                    if (txtb_changed = '1') then
                        retr_count      <= 0;
                    end if;
                end if;
                
                -- If we dont have anything to lock, and have nothing locked
             	if (is_txt_locked = '0' and (tran_frame_valid_in = '0')) then
             	     set_reciever_r     <=  '1';
                end if;

            else 
                set_reciever_r          <=  '1';
            end if;


            --------------------------------------------------------------------
            -- Configure other circuits for CAN Frame transmission / reception
            --------------------------------------------------------------------

            -- Bus synchronisation settings
            sp_control_r              <=  NOMINAL_SAMPLE;

            -- Configuration of Bit Destuffing (Both transciever and reciever)
            destuff_enable_r          <=  '1';
            stuff_error_enable_r      <=  '1'; 
            fixed_destuff_r           <=  '0';
            destuff_length_r          <=  std_logic_vector(
                                          to_unsigned(BASE_STUFF_LENGTH, 3));


            -- Clearing arbitration transcieve pointer for transcieving
            -- identifier. Restarting arbitration state machine

            -- Configuration of Arbitration field:
            --      1. Restart "tran_pointer" counter for ID bits
            --      2. Configure IDENTIFIER WORD address in TXT Buffer so that
            --         Arbitration state have ID available on TXT Buffer output.
            --      3. Restart Arbitration state machine
            tran_pointer              <=  10;
            txt_buf_ptr_r             <=  to_integer(unsigned(
                                            IDENTIFIER_W_ADR(11 downto 2)));
            arb_state                 <=  base_id;
            crc_enable_r              <=  '1';

        else
                
            -- Transcieving the data if we have what to transcieve
            if (tran_trig = '1') then
                if (OP_State = transciever or (tran_frame_valid_in = '1')) then
                    data_tx_r             <=  DOMINANT;
                else
                    data_tx_r             <=  RECESSIVE;
                end if;
            end if;
            --Note: OP_State machine has to react and set the state to
            --       transciever when SOF state and data are availiable!

            -- Recieving the data
            if (rec_trig = '1') then
                if (data_rx = DOMINANT) then
                    PC_state              <=  arbitration;
                    sync_control_r        <=  RE_SYNC;

                -- First bit detected recessive!
                else
                    PC_state              <=  error;
                    if (OP_state = reciever) then
                        inc_one_r         <=  '1';
                    end if;
                end if;
                FSM_Preset                <=  '1';
            end if; 

        end if;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Arbitration
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when arbitration =>
        if (FSM_Preset = '1') then
            FSM_Preset             <= '0';

            -- Loading shift registers with Identifier, It was addressed in
            -- SOF and is available on TXT Buffer output!
            tran_ident_base_sr     <= tran_data(IDENTIFIER_BASE_H downto   
                                             IDENTIFIER_BASE_L);
            tran_ident_ext_sr      <= tran_data(IDENTIFIER_EXT_H downto
                                             IDENTIFIER_EXT_L);
        else

            if (OP_state = transciever and rec_trig = '1') then

                case aux_tx_rx is

                    -- Losing arbitration when sending recessive and sampling 
                    -- dominant!
                    when RECESSIVE_DOMINANT =>
                        arbitration_lost_r  <=  '1';

                        -- When switching to reciever only recessive bits will 
                        -- be sent, then no Stuff bits are inserted
                        stuff_enable_r      <=  '0';

                        -- Current frame should be retransmitted!
                        txt_hw_cmd.unlock   <=  '1';
                        is_txt_locked       <=  '0';

                        -- Retransmitt limit is disabled, or enabled but not
                        -- yet reached...
                        if ((drv_retr_lim_ena = '0') or
                            (drv_retr_lim_ena = '1' and
                            retr_count < to_integer(unsigned(drv_retr_th))))
                        then
                            retr_count         <=  (retr_count + 1) mod 16;
                            txt_hw_cmd.arbl    <=  '1';
                        else

                            -- Retransmitt limit reached, transmission failed,
                            -- Erase the retransmitt counter, since the next
                            -- frame can be from the same buffer, but it can be
                            -- different frame! Thus retr_counter wont be erased
                            -- on "txt_buf_changed"!
                            retr_count          <=  0;
                            txt_hw_cmd.failed   <=  '1';
                        end if;

                    -- BIT Error, sending dominant, receiving recessive
                    when DOMINANT_RECESSIVE =>
                          PC_State            <=  error;
                          FSM_Preset          <=  '1';
                      
                    when RECESSIVE_RECESSIVE =>
                    when DOMINANT_DOMINANT =>
                    when others =>
                          PC_State              <=  error;
                          FSM_preset            <=  '1';
                end case;
            end if;
          
            --------------------------------------------------------------------
            -- Arbitration state machine
            --------------------------------------------------------------------
            case arb_state is

                ----------------------------------------------------------------
                -- BASE Identifier
                ----------------------------------------------------------------
                when base_id =>

                    -- Transceive Base ID from Shift register when transmitter,
                    -- otherwise, send RECESSIVE.
                    if (tran_trig = '1' and OP_state = transciever) then
                        data_tx_r           <= tran_ident_base_sr(10);
                        tran_ident_base_sr  <= tran_ident_base_sr(9 downto 0) &
                                               '0';
                    elsif (OP_State = reciever) then
                        data_tx_r           <= RECESSIVE;
                    end if;

                    if (rec_trig = '1') then
                        -- Counting down from 10 to 0 for each bit of BASE ID.
                        if (tran_pointer = 0) then
                            tran_pointer    <= 1;
                            arb_state       <= two_bits;
                        else
                            tran_pointer    <= tran_pointer - 1;
                        end if;

                        -- Receiving Base ID to shift register
                        rec_ident_base_sr   <= rec_ident_base_sr(9 downto 0) &
                                                data_rx;
                    end if;

                    -- Marking arbitration lost capture
                    if (arbitration_lost_r = '1') then
                        alc_r <= alc_val;
                    end if;

                ----------------------------------------------------------------
                -- Two bits between BASE and EXTENSION. Covers following cases:
                --      1. RTR and IDE for CAN 2.0 Base Frame
                --      2. r1 and IDE for CAN FD Base Frame
                --      3. SRR and IDE for CAN 2.0 and CAN FD Extended frames!
                ----------------------------------------------------------------
                when two_bits =>

                    -- Transmitting two bits based on frame type
                    if (tran_trig = '1' and OP_State = transciever) then
                        
                        --------------------------------------------------------
                        -- "tran_pointer" can be only 0 or 1 since the state has
                        -- only 2 bits! "tran_pointer" is preset to 1 in 
                        -- "base_id" state!
                        --------------------------------------------------------

                        -- RTR, r1, SRR bits
                        if (tran_pointer = 1) then
                            case aux_tran_frame_ident_type is

                                -- CAN 2.0 Base Frame -> RTR Bit
                                when "00" =>
                                    data_tx_r   <= tran_is_rtr;

                                -- CAN FD Base Frame -> r1 Bit
                                when "10" =>
                                    data_tx_r   <= DOMINANT;

                                -- CAN 2.0 Extended Frame -> SRR Bit
                                when "01" =>
                                    data_tx_r   <= RECESSIVE;
                                
                                -- CAN FD Extended Frame -> SRR Bit
                                when "11" =>
                                    data_tx_r   <= RECESSIVE;

                                -- Error if undefined
                                when others =>
                                    data_tx_r               <= RECESSIVE;
                                    PC_State                <= error;
                                    FSM_preset              <= '1'; 
                            end case;

                        --------------------------------------------------------
                        -- IDE bit
                        --
                        -- Note: IDE for Base frame is part of Control field ,
                        -- but is it is dominant it can also be part of arbit-
                        -- ration field, because then node which sends it 
                        -- always wins arbitration, thus it will never loose
                        -- during it!
                        --------------------------------------------------------
                        else
                            data_tx_r               <= tran_ident_type; 
                        end if;

                    elsif (OP_State = reciever) then
                        data_tx_r                   <= RECESSIVE;
                    end if;
                    
                    -- Sampling (receiving) two bits
                    if (rec_trig = '1') then

                        -- Store received value to "arb_two_bits".
                        arb_two_bits(tran_pointer)  <= data_rx;

                        --------------------------------------------------------
                        -- "tran_pointer" can be only 0 or 1 since the state has
                        -- only 2 bits! "tran_pointer" is preset to 1 in 
                        -- "base_id" state!
                        --------------------------------------------------------

                        -- First bit sampled (RTR, r1, SRR)
                        if (tran_pointer = 1) then
                            tran_pointer                <= 0;

                        -- Second bit sampled (IDE)
                        -- IDE bit value decides whenever we go to control 
                        -- field, or extended ID
                        else

                            ----------------------------------------------------
                            -- Go to control field, but via "delay_control_trans"
                            -- which is one clock cycle delayed, to give time
                            -- to Operation control to update "OP_State"!
                            -- (Bug fix 21.6.2016)
                            ----------------------------------------------------
                            if (data_rx = DOMINANT) then
                                delay_control_trans     <= '1';
                                arb_state               <= arb_state;
                                FSM_preset              <= '1';
                                rec_ident_type_r        <= '0';

                            -- RECESSIVE -> Start receiving Extended Identifier.
                            -- Preset counters accordingly!
                            else
                                arb_state               <= ext_id;
                                tran_pointer            <= 17; 
                                rec_ident_type_r        <= '1';
                            end if;
                        end if;
                    end if;

                    -- Marking arbitration lost capture
                    if (arbitration_lost_r = '1') then
                        alc_r <= alc_val;
                    end if;

                ----------------------------------------------------------------
                -- Identifier EXTENSION
                ----------------------------------------------------------------
                when ext_id =>

                    -- Transmitting Identifier Extension from TX Shift register!
                    if (tran_trig = '1' and OP_state = transciever) then
                        data_tx_r           <= tran_ident_ext_sr(17);
                        tran_ident_ext_sr   <= tran_ident_ext_sr(16 downto 0) &
                                              '0';

                    elsif (OP_State = reciever) then
                        data_tx_r           <=  RECESSIVE;
                    end if;

                    -- Receiving extended identifier to RX Shift register.
                    if (rec_trig = '1') then

                        rec_ident_ext_sr    <= rec_ident_ext_sr(16 downto 0) &
                                                data_rx;

                        -- Count 18 bits of Identifier extension, upon the end
                        -- move to "one bit"!
                        if (tran_pointer = 0) then
                            arb_state       <=  one_bit;
                        else
                            tran_pointer    <=  tran_pointer - 1;
                            arb_state       <=  arb_state;
                        end if;                        
                    end if;    

                    -- Marking arbitration lost capture
                    if (arbitration_lost_r = '1') then
                        alc_r <= alc_val;
                    end if;
            
                ----------------------------------------------------------------
                -- Extra bit after EXTENDED Identifier.
                -- (RTR bit of CAN 2.0, r0 bit of CAN FD Frame)
                ----------------------------------------------------------------
                when one_bit =>

                    -- Transmitting last bit of Arbitration field.
                    if (tran_trig = '1' and OP_state = transciever) then

                        if (tran_frame_type = FD_CAN) then
                            data_tx_r   <=  DOMINANT; 
                        else
                            data_tx_r   <=  tran_is_rtr;
                        end if;

                    elsif (OP_State = reciever) then
                        data_tx_r       <=  RECESSIVE;                    
                    end if;

                    -- Storing last bit of arbitration field and transfering to
                    -- control field
                    if (rec_trig = '1') then
                        arb_one_bit             <=  data_rx;
                        delay_control_trans     <=  '1';
                        FSM_preset              <=  '1';
                    end if;

                    -- Marking arbitration lost capture
                    if (arbitration_lost_r = '1') then
                        alc_r <= alc_val;
                    end if;

                when others =>
                    PC_State                <=  error;
                    FSM_preset              <=  '1';
                end case;
            end if;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Control frame field
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when control => 
        
        ------------------------------------------------------------------------
        -- Transmitting control bits 
        ------------------------------------------------------------------------
        if (OP_State = transciever) then 
            
            --------------------------------------------------------------------
            -- First bit of control field as transmitter:
            --      1. Enable bit error detection.
            --      1. Store internal DLC.
            --      2. Build shift register for transmission of control field
            --         bits!
            --------------------------------------------------------------------
            if (FSM_preset = '1') then
                FSM_preset                      <=  '0';

                ----------------------------------------------------------------
                -- Calculate real length of data field, which does not
                -- always correspond to DLC!
                ----------------------------------------------------------------
                if (tran_frame_type = NORMAL_CAN) then

                    ------------------------------------------------------------
                    -- For RTR frames data length is 0
                    ------------------------------------------------------------
                    if (tran_is_rtr = RTR_FRAME) then
                        dlc_int     <= (OTHERS => '0');

                    ------------------------------------------------------------
                    -- For CAN Frames with DLC > 8. Data length is only 8!
                    ------------------------------------------------------------
                    elsif (tran_dlc(3) = '1') then
                        dlc_int     <= "1000";

                    ------------------------------------------------------------
                    -- DLC matching data length value!
                    ------------------------------------------------------------
                    else
                        dlc_int     <= tran_dlc;
                    end if;

                ----------------------------------------------------------------
                -- For FD Frames, Data length corresponds to DLC.
                ----------------------------------------------------------------
                else
                    dlc_int         <= tran_dlc;
                end if;


                ----------------------------------------------------------------
                -- Building shift register for transmission of control field
                -- bits.
                ----------------------------------------------------------------
                case aux_tran_frame_ident_type is

                    -- CAN 2.0 BASE format
                    when NORMAL_CAN & BASE =>
                        ctrl_tran_reg(4)        <= DOMINANT; -- r0 bit
                        control_pointer         <= 4;

                    -- CAN FD BASE Format
                    when FD_CAN & BASE =>
                        ctrl_tran_reg(7)        <= RECESSIVE; -- EDL Bit
                        ctrl_tran_reg(6)        <= DOMINANT;  -- r0 Bit
                        ctrl_tran_reg(5)        <= tran_brs;  -- BRS Bit

                        if (error_state = error_active) then
                            ctrl_tran_reg(4)    <= DOMINANT;
                        else
                            ctrl_tran_reg(4)    <= RECESSIVE;
                        end if;
                        control_pointer         <= 7;

                    -- CAN 2.0 Extended format
                    when NORMAL_CAN & EXTENDED => 
                        ctrl_tran_reg(5)        <= DOMINANT; -- r1 Bit
                        ctrl_tran_reg(4)        <= DOMINANT; -- r0 Bit
                        control_pointer         <= 5;
                          
                    -- CAN FD Extended Format
                    when FD_CAN & EXTENDED =>
                        ctrl_tran_reg(7)        <= RECESSIVE; -- EDL Bit
                        ctrl_tran_reg(6)        <= DOMINANT;  -- r0 Bit
                        ctrl_tran_reg(5)        <= tran_brs;  -- BRS Bit

                        if (error_state = error_active) then
                            ctrl_tran_reg(4)    <= DOMINANT;
                        else
                            ctrl_tran_reg(4)    <= RECESSIVE;
                        end if;

                        control_pointer         <= 7;
                          
                    when others =>
                          PC_State              <=  error;
                          FSM_preset            <=  '1';
                end case;


                ----------------------------------------------------------------
                -- If Frame is RTR then user has chance to send DLC = 0000 or
                -- custom DLC as usual. No data is sent in both cases in RTR 
                -- Frame.
                ----------------------------------------------------------------
                if (tran_frame_type = NORMAL_CAN   and 
                    tran_is_rtr = RTR_FRAME        and 
                    drv_rtr_pref = '1')
                then
                    ctrl_tran_reg(3 downto 0) <= (OTHERS => '0'); 
                else
                    -- DLC bits are common for control fields of all frame types
                    ctrl_tran_reg(3 downto 0) <= tran_dlc;  
                end if;


            --------------------------------------------------------------------
            -- Remaining bits of control field as transmitter. Transmitt
            -- Contents of "ctrl_tran_reg" register.
            --------------------------------------------------------------------
            else 
                
                -- Transmitting control field bits on the bus
                if (tran_trig = '1') then
                    data_tx_r                 <= ctrl_tran_reg(control_pointer);
                end if;

                -- Receiving control field bits as Transmitter
                if (rec_trig = '1') then

                    ------------------------------------------------------------
                    -- Count down on "control_pointer" till 0. When end of
                    -- control field was reached -> switch state.
                    ------------------------------------------------------------
                    if (control_pointer_non_zero) then
                        control_pointer        <= control_pointer - 1;

                    else
                        -- Move to CRC field when RTR frame is transmitted or
                        -- DLC is 0. Move to Data field otherwise!
                        if ((tran_is_rtr = RTR_FRAME and 
                             tran_frame_type = NORMAL_CAN) or 
                            (tran_dlc = "0000"))
                        then
                            PC_State            <= crc;
                        else
                            PC_State            <= data;     
                        end if;
                        FSM_preset              <= '1';
                    end if;

                    
                    ------------------------------------------------------------
                    -- Transciever delay compensation calibration is executed
                    -- in EDL bit. Note that "control_pointer" is 7 only
                    -- in EDL bit of CAN FD Frame. Otherwise control field has
                    -- less bits.
                    -- Following is executed:
                    --      1. Reset secondary sampling point shift register
                    --      2. Start Transceiver delay calibration.
                    ------------------------------------------------------------
                    if (control_pointer = 7) then
					
                        -- Clearing the shift register for output data
                        ssp_reset_r           <= '1';
                        trv_delay_calib_r     <= '1';

                    else
                        trv_delay_calib_r     <= '0';
                    end if;         

                    ------------------------------------------------------------
                    -- Switching bit-Rate for FD Frames in BRS bit:
                    --      1. Bit rate is switched
                    --      2. No synchronisation (FD Transmitter does not
                    --         re-synchronize)
                    --      3. Release reset of secondary sampling point
                    --         shift register.
                    ------------------------------------------------------------
                    if (control_pointer = 5 and tran_brs = BR_SHIFT and
                        tran_frame_type = FD_CAN)
                    then
                        sp_control_r    <= SECONDARY_SAMPLE;
                        br_shifted      <= '1';
                        sync_control_r  <= NO_SYNC;
                        ssp_reset_r     <= '0';
                    end if;
                end if;

            end if;

        -- Receiver does not transmitt anymore
        else
            data_tx_r           <=  RECESSIVE;
        end if;         


        ------------------------------------------------------------------------
        -- Recieving control bits 
        ------------------------------------------------------------------------
        if (OP_State = reciever and rec_trig = '1') then

            --------------------------------------------------------------------
            -- First bit -> Set "control pointer" based on received bit (
            --------------------------------------------------------------------
            if (FSM_preset = '1') then
								
                -- EDL bit -> CAN FD Frame, r0 bit -> CAN Frame
                rec_frame_type_r  <= data_rx;

                ----------------------------------------------------------------
                -- CAN FD Frame (EDL = RECESSIVE)
                ----------------------------------------------------------------
                if (data_rx = RECESSIVE) then

                    ------------------------------------------------------------    
                    -- Check if CAN FD Frames are supported. If not, throw
                    -- FORM Error.
                    ------------------------------------------------------------
                    if (drv_CAN_fd_ena = '1') then
                        control_pointer     <= 6; -- r0,BRS,ESI,4DLC bits
                        rec_is_rtr_r        <= '0';
                        FSM_preset          <= '0';

                        -- HARD Synchronisation in EDL -> r0 transition for
                        -- receivers! (Transmitter does not necessarily have to
                        -- perform Hard synchronisation, since now it is the
                        -- only node which is transmitting!)
                        sync_control_r      <= HARD_SYNC;
                    else
                        PC_State            <= error;
                        form_Error_r        <= '1';
                        inc_one_r           <= '1';
                        FSM_preset          <= '1';
                    end if;

                ----------------------------------------------------------------
                -- CAN 2.0 Frame (EDL = DOMINANT)
                ----------------------------------------------------------------
                else

                    FSM_preset              <= '0';

                    -- Difference for EXTENDED and BASE frames.
                    if (rec_ident_type_r = EXTENDED) then
                        control_pointer     <= 4; -- r0 bit, 4 bits DLC
                        rec_is_rtr_r        <= arb_one_bit;
                    else
                        control_pointer     <= 3; -- 4 bits DLC
                        rec_is_rtr_r        <= arb_two_bits(1);
                    end if;
                end if;
            
            --------------------------------------------------------------------
            -- Receiving remaining bits
            --------------------------------------------------------------------
            else

                -- Decrement control pointer till the end of control field
                if (control_pointer_non_zero) then
                    control_pointer        <= control_pointer - 1;
                end if;

                case control_pointer is

                    ------------------------------------------------------------
                    -- r0 bit of FD Frame -> If detected RECESSIVE -> Error
                    -- Return synchronisation back to RE_SYNC after HARD_SYNC
                    -- was performed between EDL and r0 bits.
                    ------------------------------------------------------------
                    when 6 =>
                        if (data_rx = RECESSIVE) then
                            form_Error_r    <= '1';
                            inc_one_r       <= '1';
                            PC_State        <= error;
                            FSM_Preset      <= '1';
                        end if;
                        sync_control_r      <= RE_SYNC;
   
                    ------------------------------------------------------------
                    -- BRS bit: Switch bit-rate if RECESSIVE
                    ------------------------------------------------------------
                    when 5 =>
                        rec_brs_r           <= data_rx;
                        if (data_rx = RECESSIVE) then
                            sp_control_r    <= DATA_SAMPLE;
                            br_shifted      <= '1';
                        end if;

                    ------------------------------------------------------------
                    -- CAN FD Frame -> ESI bit
                    -- CAN 2.0 Frame -> We get to "control_pointer=4" only if
                    --      the frame is EXTENDED. Then it is r0 bit. If dete-
                    --      cted RECESSIVE, FORM Error!
                    ------------------------------------------------------------
                    when 4 =>
                        if (rec_frame_type_r = FD_CAN) then
                            rec_esi_r       <= data_tx_r;       
                        else
                            if (data_rx = RECESSIVE) then
                                form_Error_r    <= '1';
                                inc_one_r       <= '1';
                                PC_State        <= error;
                                FSM_Preset      <= '1';
                            end if;
                        end if;

                    ------------------------------------------------------------
                    -- Last 4 bits of Control -> DLC field
                    ------------------------------------------------------------
                    when 3 => 
                        rec_dlc_r(3)            <= data_rx;

                    when 2 => 
                        rec_dlc_r(2)            <= data_rx;
 
                    when 1 => 
                        rec_dlc_r(1)            <= data_rx;

                    when 0 => 
                        rec_dlc_r(0)            <= data_rx;
                        FSM_preset              <= '1';
                        
                        --------------------------------------------------------
                        -- Update PC State. For RTR Frames or DLC = 0000,
                        -- go directly to CRC Phase!
                        --------------------------------------------------------
                        if ((rec_is_rtr_r = RTR_FRAME and
                             rec_frame_type_r = NORMAL_CAN) or 
                            (rec_dlc_r(3 downto 1) = "000" and data_rx = '0'))
                        then 
                            PC_State            <= crc;
                        else
                            PC_State            <= data;
                        end if;

                        --------------------------------------------------------
                        -- Real length of data field does not always correspond
                        -- to received DLC!
                        --------------------------------------------------------
                        if (rec_frame_type_r = NORMAL_CAN) then

                            ----------------------------------------------------
                            -- RTR Frame -> Data length = 0
                            ----------------------------------------------------
                            if (rec_is_rtr_r = RTR_FRAME) then
                                dlc_int             <= (OTHERS => '0');

                            ----------------------------------------------------
                            -- CAN Frame with more than 8 bytes -> 8 only!
                            ----------------------------------------------------
                            elsif (rec_dlc_r(3) = '1') then
                                dlc_int             <= "1000";

                            ----------------------------------------------------
                            -- Data length as received DLC!
                            ----------------------------------------------------
                            else
                                dlc_int(3 downto 1) <= rec_dlc_r(3 downto 1);
                                dlc_int(0)          <= data_rx;
                            end if;

                        --------------------------------------------------------
                        -- For FD Frames, there is no RTR flag, and full length
                        -- of 64 bytes is supported.
                        --------------------------------------------------------
                        else                            
                            dlc_int(3 downto 1)     <= rec_dlc_r(3 downto 1);
                            dlc_int(0)              <= data_rx;
                        end if;

                        --------------------------------------------------------
                        -- Sending commands to RX Buffer, to store metadata, 
                        -- identifier and frame format words
                        --------------------------------------------------------
                        store_metadata_r            <= '1';

                    when others =>
                        PC_State                    <= error;
                        FSM_preset                  <= '1';
                end case;   
                    
            end if;
        end if;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Data Phase
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when data => 
        if (FSM_Preset = '1') then
            FSM_Preset  <= '0';
            
            --------------------------------------------------------------------
            -- Set data pointer based on data size to be transmitted or received
            -------------------------------------------------------------------- 
            case dlc_int is
                when "0000" => data_pointer  <= 0; -- Zero bits
                when "0001" => data_pointer  <= 7; -- 1 byte
                when "0010" => data_pointer  <= 15; -- 2 bytes
                when "0011" => data_pointer  <= 23; -- 3 bytes
                when "0100" => data_pointer  <= 31; -- 4 bytes
                when "0101" => data_pointer  <= 39; -- 5 bytes
                when "0110" => data_pointer  <= 47; -- 6 bytes
                when "0111" => data_pointer  <= 55; -- 7 bytes
                when "1000" => data_pointer  <= 63; -- 8 bytes
                when "1001" => data_pointer  <= 95; -- 12 bytes
                when "1010" => data_pointer  <= 127; -- 16 bytes
                when "1011" => data_pointer  <= 159; -- 20 bytes
                when "1100" => data_pointer  <= 191; -- 24 bytes
                when "1101" => data_pointer  <= 255; -- 32 bytes
                when "1110" => data_pointer  <= 383; -- 48 bytes
                when "1111" => data_pointer  <= 511; -- 64 bytes
                when others => 
                        data_pointer           <= 0;
                        PC_State               <= error;
                        FSM_preset             <= '1';
            end case;
            data_tx_index           <= 31;
            
            -- Transmitter shall not synchronize in data phase of CAN FD Frame!
            if (OP_State = transciever and tran_frame_type = FD_CAN) then
                sync_control_r      <= NO_SYNC;
            end if;
            
            -- Receive RAM signals
            rec_word_ptr            <= 0;
            rec_word_bind           <= 0;
            rec_data_sr             <= (OTHERS => '0');
            
            -- Pointer directly to TXT Buffer, First data word
            txt_buf_ptr_r           <= to_integer(unsigned(
                                        DATA_1_4_W_ADR(11 downto 2)));

        else

            --------------------------------------------------------------------
            -- Transmitting data
            --------------------------------------------------------------------
            if (OP_State = transciever) then
                if (tran_trig = '1') then
                    data_tx_r               <= tx_data_word(data_tx_index);

                    ------------------------------------------------------------
                    -- Move to the next word :
                    --      1. Increment adress in TXT Buffer
                    --      2. Set "data_tx_index" to first bit of next word
                    ------------------------------------------------------------
                    if (data_tx_index = 0) then
                        if (txt_buf_ptr_r < 19) then
                            txt_buf_ptr_r   <= txt_buf_ptr_r + 1;
                        end if;
                        data_tx_index       <= 31;
                    else
                        data_tx_index       <= data_tx_index - 1;
                    end if;
                end if;
            else
                data_tx_r   <=  RECESSIVE;
            end if;

            --------------------------------------------------------------------
            -- Recieving data (also transmitter recieves the same data)
            --------------------------------------------------------------------
            if (rec_trig = '1') then
              
                -- Shift register and bits within one byte
                rec_data_sr               <= rec_data_sr(6 downto 0) & data_rx; 
                rec_word_ptr              <= (rec_word_ptr + 1) mod 8;

                ----------------------------------------------------------------
                -- If whole byte was received store it to "store_data_word_r".
                ----------------------------------------------------------------
                if (rec_word_ptr = 7) then
                    rec_word_bind         <= (rec_word_bind + 1) mod 4;

                    case rec_word_bind is
                        --------------------------------------------------------
                        -- First byte of word, whole word is written to avoid
                        -- bytes from old frames!
                        --------------------------------------------------------
                        when  0 =>
                            store_data_word_r <= "000000000000000000000000" &
                                                  rec_data_sr(6 downto 0) &
                                                  data_rx;
                        when  1 =>
                            store_data_word_r(15 downto 8) <= 
                                                rec_data_sr(6 downto 0) &
                                                data_rx;
                        when  2 =>
                            store_data_word_r(23 downto 16) <= 
                                                rec_data_sr(6 downto 0) &
                                                data_rx;
                        when  3 =>
                            store_data_word_r(31 downto 24) <= 
                                                rec_data_sr(6 downto 0) &
                                                data_rx;
                        when others =>
                            PC_State      <= error;
                            FSM_Preset    <= '1';
                    end case;

                    ------------------------------------------------------------
                    -- Give command to RX Buffer to store data word whole word
                    -- was received (4 bytes aligned).
                    ------------------------------------------------------------
                    if (rec_word_bind = 3 and OP_State = reciever) then
                        store_data_r          <= '1';
                    end if;
                end if;

                ----------------------------------------------------------------
                -- Data pointer counts till 0 (end of data field).
                ----------------------------------------------------------------
                if (data_pointer > 0) then
                    data_pointer              <= data_pointer - 1;
                else

                    ------------------------------------------------------------
                    -- If we finish data field, and we did not receive 4 byte 
                    -- aligned data, we still did not store data since last 
                    -- aligned word! Remaining bytes must be stored.
                    ------------------------------------------------------------
                    if (rec_word_bind /= 3 and OP_State = reciever) then
                        store_data_r          <= '1';
                    end if;
                    PC_State                  <= crc;
                    FSM_Preset                <= '1';
                end if;

            end if;            
        end if;
  
          
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- CRC field
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when crc =>
        
        if (FSM_Preset = '1') then
            ssp_reset_r     <= '0';

            --------------------------------------------------------------------
            -- Calculate length of CRC sequence:
            --  1. For all CAN 2.0 Frames it is 15
            --  2. For CAN FD frames less than 16 bytes (dlc=0xA), it is 17
            --  3. For longer CAN FD frames it is 21.
            --------------------------------------------------------------------
            if ((OP_State = transciever and tran_frame_type = NORMAL_CAN) or
                (OP_State = reciever    and rec_frame_type_r = NORMAL_CAN))
            then
                data_pointer  <=  14; --CRC 15
                crc_src       <=  CRC_15_SRC;
            else

                if (unsigned(dlc_int) > 10) then
                    data_pointer  <= 20;
                    crc_src       <= CRC_21_SRC;
                else
                    data_pointer  <= 16;
                    crc_src       <= CRC_17_SRC;   
                end if;

            end if;

            --------------------------------------------------------------------
            -- Change the bit Stuffing for CRC of CAN FD:
            --  1. Fixed stuffing
            --  2. Stuff rule lenght is FD_STUFF_LENGTH (4)
            --------------------------------------------------------------------
            if (OP_State = transciever and tran_frame_type = FD_CAN) or 
               (OP_State = reciever    and rec_frame_type_r = FD_CAN)
            then
                fixed_stuff_r       <= '1';
                fixed_destuff_r     <= '1';
                stuff_length_r      <= std_logic_vector(
                                            to_unsigned(FD_STUFF_LENGTH, 3));
                destuff_length_r    <= std_logic_vector(
                                            to_unsigned(FD_STUFF_LENGTH, 3));

                ----------------------------------------------------------------
                -- Go to stuff count transmission if ISO FD is configured.
                ----------------------------------------------------------------
                if (drv_fd_type = ISO_FD) then
                    crc_state       <= stuff_count;
                    stl_pointer     <= 3;
                else
                    crc_state       <= real_crc;
                    crc_enable_r    <= '0';
                end if;

            else
                crc_state           <= real_crc;
                crc_enable_r        <= '0';
            end if;

            rec_crc_r               <= (OTHERS => '0');
            FSM_Preset              <= '0';

        else
            case crc_state is

                ----------------------------------------------------------------
                -- Stuff count field + stuff parity
                ----------------------------------------------------------------
                when stuff_count =>
                                
                    if (OP_State = transciever) then
                        if (tran_trig = '1') then 
                            -- Transmitting stuff count field and the parity
                            if (stl_pointer > 0) then
                                data_tx_r   <= stuff_count_grey(stl_pointer - 1);
                            else
                                data_tx_r   <= stuff_parity;
                            end if;                                 
                        end if;
                    else
                        data_tx_r           <= RECESSIVE;
                    end if;

                    if (rec_trig = '1') then          
                        if (stl_pointer > 0) then
                            stl_pointer                     <= stl_pointer - 1;
                            rx_count_grey(stl_pointer - 1)  <= data_rx;
                        else 
                            crc_state                       <= real_crc;
                            crc_enable_r                    <= '0';
                            rx_parity                       <= data_rx; 
                        end if;
                    end if; 

                ----------------------------------------------------------------
                -- Real CRC sequence
                ----------------------------------------------------------------
                when real_crc =>

                    ------------------------------------------------------------
                    -- Transmitt CRC sequence
                    ------------------------------------------------------------
                    if (OP_State = transciever and tran_trig = '1') then
                        case crc_src is
                            when CRC_15_SRC =>
                                data_tx_r   <= crc15(data_pointer);

                            when CRC_17_SRC =>
                                data_tx_r   <= crc17(data_pointer);

                            when CRC_21_SRC =>
                                data_tx_r   <= crc21(data_pointer);

                            when others => 
                                data_tx_r             <=  RECESSIVE;
                                PC_State              <=  error;
                                FSM_preset            <=  '1';
                        end case;
                    end if;

                    ------------------------------------------------------------
                    -- Receive CRC sequence
                    ------------------------------------------------------------
                    if (rec_trig = '1') then
                        rec_crc_r <= rec_crc_r(19 downto 0) & data_rx;
                        if (data_pointer = 0) then
                            PC_State              <= delim_ack;
                            FSM_Preset            <= '1';
                        else
                            data_pointer          <= data_pointer - 1;
                        end if;
                    end if;

                when others =>
                    PC_State                      <= error;
                    FSM_preset                    <= '1';
             end case;
        end if;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- CRC Delimiter, Acknowledge and Acknowledge delimiter
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when delim_ack =>
        if (FSM_Preset = '1') then
            control_pointer   <= 2;
            FSM_Preset        <= '0';
            ack_recieved      <= '0';
            sec_ack           <= '0';

            --------------------------------------------------------------------
            -- CRC check (for both reciever, and also for transciever if 
            -- loopbacked CRC matches the calculated one!
            --------------------------------------------------------------------
            if (crc_and_parity_valid) then
                crc_check     <= '1';
            else
                crc_check     <= '0';
            end if;        
             
        else

            --------------------------------------------------------------------
            -- Disable bit stuffing, ACK field is not coded by bit stuffing.
            -- Disable in first bit of ACK field, so that if there is stuff
            -- bit at the end of CRC field, it is still destuffed/stuffed!
            --------------------------------------------------------------------
            if (rec_trig = '1' and control_pointer = 2) then
                stuff_enable_r    <= '0';
                destuff_enable_r  <= '0';
                fixed_stuff_r     <= '0';
                fixed_destuff_r   <= '0';
            end if;


            if (OP_State = transciever) then

                ----------------------------------------------------------------
                -- Transmitter sends only recessice during CRC Delim, ACK and
                -- ACK delim
                ----------------------------------------------------------------
                if (tran_trig = '1') then
                    data_tx_r <=  RECESSIVE; 
                end if;

                ----------------------------------------------------------------
                -- Monitoring received data 
                ----------------------------------------------------------------
                if (rec_trig = '1') then
                    case control_pointer is

                        --------------------------------------------------------
                        -- CRC delimiter -> Switch back to Nominal bit rate
                        --------------------------------------------------------
                        when 2 =>
                            if (tran_brs = BR_SHIFT and
                                tran_frame_type = FD_CAN)
                            then
                                br_shifted      <= '1';
                                sp_control_r    <= NOMINAL_SAMPLE;
                            end if;
                            
                            control_pointer     <= control_pointer - 1;

                            -- Form Error detection
                            if (data_rx = DOMINANT) then
                                form_Error_r    <= '1';
                                PC_State        <= error;
                                FSM_Preset      <= '1';
                            end if;

                        --------------------------------------------------------
                        -- ACK field -> When dominant is received, ACK is valid.
                        --------------------------------------------------------
                        when 1 =>
                            if (data_rx = DOMINANT or 
                                drv_self_test_ena = '1')
                            then 
                                ack_recieved        <= '1';
                                control_pointer     <= control_pointer - 1;
                            else

                                ------------------------------------------------
                                -- Allow receiving first bit RECEESIVE, only
                                -- if second consecutive RECESSIVE is received,
                                -- ACK is considered as ERROR. This is defined
                                -- by spec. and allows for compensation of
                                -- mismatch caused by bit-rate switching!
                                ------------------------------------------------
                                if (sec_ack = '0') then
                                    sec_ack         <= '1';
                                    ack_recieved    <= '0';
                                else
                                    ack_recieved    <= '0';
                                    sec_ack         <= sec_ack;
                                    PC_State        <= error;
                                    ack_Error_r     <= '1'; 
                                    FSM_Preset      <= '1';
                                end if;  
                            end if;

                        --------------------------------------------------------
                        -- ACK Delimiter. If ACK was received -> OK, Error
                        -- frame otherwise. If sampled
                        --------------------------------------------------------
                        when 0 =>
                            if (ack_recieved = '1' and data_rx = RECESSIVE) then
                                PC_State            <= eof;
                            else
                                PC_State            <= error;
                                if (data_rx = DOMINANT) then
                                    form_Error_r    <= '1';
                                end if;
                            end if;
                            FSM_Preset              <= '1';

                        when others =>
                            PC_State                <= error;
                            FSM_preset              <= '1';
                    end case;
                end if;
            end if;


            if (OP_State = reciever) then

                ----------------------------------------------------------------
                -- Receiver sends acknowledge if it is allowed to and if CRC
                -- was checked OK!
                ----------------------------------------------------------------
                if (tran_trig = '1') then
                    case control_pointer is

                        --------------------------------------------------------
                        -- CRC delimiter bit
                        --------------------------------------------------------
                        when 2 => 
                            data_tx_r           <= RECESSIVE;
                            
                        --------------------------------------------------------
                        -- ACK field. Send if CRC match and ACK is not forbidden
                        --------------------------------------------------------
                        when 1 => 
                            if (crc_check = '1' and drv_ack_forb = '0') then
                                data_tx_r       <= DOMINANT;
                            else
                                data_tx_r       <= RECESSIVE;
                            end if;

                            ----------------------------------------------------
                            -- If Bus Monitoring mode is enabled then data has
                            -- to be looped back before sending on the bus!
                            -- Thisway node itself will NOT detec error due to
                            -- missing acknowledge!
                            ----------------------------------------------------
                            if (drv_bus_mon_ena = '1') then
                                int_loop_back_ena_r     <= '1';
                            end if;

                        --------------------------------------------------------
                        -- ACK field. Send if CRC match and ACK is not forbidden
                        --------------------------------------------------------
                        when 0 =>
                            data_tx_r               <= RECESSIVE;
                            int_loop_back_ena_r     <= '0'; 
  
                        when others =>
                            PC_State                <= error;
                            FSM_preset              <= '1';
                    end case;
                end if;

                ----------------------------------------------------------------
                -- Receiver also monitors if ACK was received!
                ----------------------------------------------------------------
                if (rec_trig = '1') then
                    case control_pointer is

                        --------------------------------------------------------
                        -- CRC delimiter, switch back to NOMINAL bit rate.
                        --------------------------------------------------------
                        when 2 =>
                            if (rec_brs_r = BR_SHIFT and
                                rec_frame_type_r = FD_CAN)
                            then
                                br_shifted          <= '1';
                                sp_control_r        <= NOMINAL_SAMPLE;
                            end if;

                            -- Receiving DOMINANT means error!
                            if (data_rx = DOMINANT) then
                                PC_State            <= error;
                                FSM_Preset          <= '1';
                                form_Error_r        <= '1';
                                -- Increment recieve error counter by one!
                                inc_one_r           <= '1';
                            end if;

                        --------------------------------------------------------
                        -- ACK field. Sending DOMINANT and receiving RECESSIVE
                        -- signals ERROR. Receiving DOMINANT means ACK present.
                        --------------------------------------------------------
                        when 1 =>
                            if (data_tx_r = DOMINANT and data_rx = RECESSIVE) then
                                PC_State            <= error;
                                FSM_Preset          <= '1';
                            end if;
                            if (data_rx = DOMINANT or drv_self_test_ena = '1') then
                                ack_recieved        <= '1';
                            end if;

                        --------------------------------------------------------
                        -- ACK delimiter
                        --------------------------------------------------------
                        when 0 => 
                            if (ack_recieved = '1' and crc_check = '1' and
                                data_rx = RECESSIVE) then
                                PC_State            <= eof;
                            else
                                PC_State            <= error;

                                if (ack_recieved = '0') then
                                    ack_Error_r         <= '1';
                                end if;

                                if (data_rx = DOMINANT) then
                                    form_Error_r        <= '1';
                                end if;
                                
                                inc_one_r           <= '1';
                            end if; 
                            FSM_preset              <= '1';

                        when others =>
                            PC_State                <= error;
                            FSM_preset              <= '1';
                    end case;

                    if (control_pointer_non_zero) then
                        control_pointer             <= control_pointer - 1;
                    end if;
              end if;

            end if;
        end if;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- End of frame
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when eof =>
        if (FSM_Preset = '1') then
            FSM_Preset                          <= '0';
            control_pointer                     <= 6;
        else

            --------------------------------------------------------------------
            -- Sending only RECESSIVE during EOF
            --------------------------------------------------------------------
            if (tran_trig = '1') then
                data_tx_r                       <= RECESSIVE;
            end if;           

            --------------------------------------------------------------------
            -- Receiving EOF
            --------------------------------------------------------------------
            if (rec_trig = '1') then

                ----------------------------------------------------------------
                -- DOMINANT bit during EOF. In last bit -> Overload , In
                -- previous bits -> Error frame
                ----------------------------------------------------------------
                if (data_rx = DOMINANT) then

                    if (control_pointer_non_zero) then
                        PC_State                    <= error;
                        if (OP_State = reciever) then
                            inc_one_r               <= '1';
                        end if;
                    else
                        PC_State                    <= overload;
                    end if;
                    FSM_Preset <= '1';

                ----------------------------------------------------------------
                -- RECESSIVE bit during EOF.
                ----------------------------------------------------------------
                else
                    ------------------------------------------------------------
                    -- RX Frame is considered as valid one bit before finishing
                    -- EOF. Thisway Data are stored even if Overload flag is
                    -- present!
                    ------------------------------------------------------------
                    if (control_pointer = 1 and OP_State = reciever) then
                        rec_valid_r         <= '1';
                        dec_one_r           <= '1';
                    end if; 

                    ------------------------------------------------------------
                    -- TX Frame is considered as valid at the end of EOF!
                    ------------------------------------------------------------
                    if (control_pointer = 0 and OP_State = transciever) then
                        tran_valid_r        <= '1';
                        dec_one_r           <= '1';            
                        txt_hw_cmd.unlock   <= '1';
                        txt_hw_cmd.valid    <= '1';
                        is_txt_locked       <= '0';
                        retr_count          <= 0;
                    end if; 

                    ------------------------------------------------------------
                    -- Count till 0 -> Go to interframe then.
                    ------------------------------------------------------------
                    if (control_pointer_non_zero) then
                        control_pointer         <= control_pointer - 1;                        
                    else
                        PC_State                <= interframe; 
                        FSM_Preset              <= '1';
                    end if;
                end if;
            end if;

        end if;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Interframe space
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when interframe =>
        if (FSM_Preset = '1') then
            FSM_Preset                          <= '0';
            control_pointer                     <= 2;
            interm_state                        <= intermission;
            stuff_enable_r                      <= '0';
            destuff_enable_r                    <= '0';
        else
            case interm_state is

                ----------------------------------------------------------------
                -- Intermission
                ----------------------------------------------------------------
                when intermission =>

                    ------------------------------------------------------------
                    -- Transmitting RECESSIVE. From second bit on, react on
                    -- edge by HARD_SYNC.
                    ------------------------------------------------------------
                    if (tran_trig = '1') then 
                        data_tx_r               <= RECESSIVE;
                        if (control_pointer < 2) then 
                            sync_control_r      <= HARD_SYNC; 
                        end if;
                    end if;
                        
                    ------------------------------------------------------------
                    -- We transfer to SOF When sample dominant or detect edge
                    ------------------------------------------------------------
                    if (hard_sync_edge = '1') then
                        PC_State                    <= sof; 
                        crc_enable_r                <= '1';
                        FSM_preset                  <= '1';

                    ------------------------------------------------------------
                    -- Receiving intermission bits
                    ------------------------------------------------------------
                    elsif (rec_trig = '1') then

                        if (control_pointer_non_zero) then
                            control_pointer         <= control_pointer - 1;
                        end if;

                        --------------------------------------------------------
                        -- Sampling RECESSIVE bit. In the last RECESSIVE bit
                        -- Transmitting error passive node goes to SUSPEND,
                        -- otherwise goes to intermission idle.
                        --------------------------------------------------------
                        if (data_rx = RECESSIVE) then

                            if (not control_pointer_non_zero) then
                                if (OP_State = transciever and 
                                    error_state = error_passive)
                                then
                                  interm_state      <= suspend;
                                  control_pointer   <= 7;   
                                else
                                  interm_state      <= interm_idle; 
                                end if;
                            end if;

                        --------------------------------------------------------
                        -- Sampling DOMINANT bit. Third bit should be
                        -- interpreted as SOF, previous bits as Overload
                        -- condition. (Bugfix 30.6.2016)
                        --------------------------------------------------------
                        else
                            if (control_pointer_non_zero) then
                                PC_State            <= overload;
                            else
                                PC_State            <= sof;
                                crc_enable_r        <= '1';
                           end if;
                           FSM_preset          <= '1';
                        end if;  
                    end if;  


                ----------------------------------------------------------------
                -- Suspend transmission. If Hard sync edge comes, go to SOF,
                -- but turn receiver! 
                ----------------------------------------------------------------
                when suspend =>
                    sync_control_r                  <= HARD_SYNC;
                    if (tran_trig = '1') then
                        data_tx_r                   <= RECESSIVE;
                    end if;

                    if (hard_sync_edge = '1') then
                        PC_State                    <= sof; 
                        crc_enable_r                <= '1';
                        FSM_preset                  <= '1';
                        set_reciever_r              <= '1';

                    elsif (rec_trig = '1') then

                        if (control_pointer_non_zero) then
                            control_pointer         <= control_pointer - 1;
                        else
                            interm_state            <= interm_idle;
                        end if;

                    end if;

                ----------------------------------------------------------------
                -- Intermission idle
                ----------------------------------------------------------------
                when interm_idle =>
                    is_idle_r               <=  '1';

                    if (tran_trig = '1') then
                        data_tx_r           <= RECESSIVE;
                    end if;

                    sync_control_r      <= HARD_SYNC;
                    
                    if (hard_sync_edge = '1') then
                        PC_State            <= sof;
                        crc_enable_r        <= '1';
                        FSM_preset          <= '1';

                    elsif (rec_trig = '1') then 
                      
                        -- If any frame is available here for transmission 
                        -- we lock it already here. If we moved to SOF and
                        -- locked only there, the frame might have been
                        -- aborted in that one clock cycle! Thus we would
                        -- then lock invalid frame!
                        if ((drv_bus_mon_ena = '0') and 
                            (tran_frame_valid_in  = '1')) -- Next data are availiable 
                        then
                            PC_State        <= sof;
                            is_txt_locked   <= '1';
                            txt_hw_cmd.lock <= '1';                  
                            crc_enable_r    <= '1';
                            FSM_Preset      <= '1';

                            -- In case that TX Arbitrator provides different frame 
                            -- for us, we need to erase the retranmsitt counter
                            if (txtb_changed = '1') then
                                retr_count   <= 0;
                            end if;
                        else
                            FSM_Preset       <= '0';                    
                        end if;
                    end if;
                
            when others =>
                  PC_State                  <= error;
                  FSM_preset                <= '1';
            end case;
          end if;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Error frame
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when error =>
        if (FSM_Preset = '1') then
            FSM_Preset          <= '0';
            control_pointer     <= 6; -- Pointer for sending error flag
            stuff_enable_r      <= '0';
            fixed_stuff_r       <= '0';
            fixed_destuff_r     <= '0';

            --------------------------------------------------------------------
            -- Pointer for recieving the superposition of error flags
            --------------------------------------------------------------------
            tran_pointer        <= 13;
            err_frame_state     <= err_flg_sup;
            destuff_enable_r    <= '0';

            --------------------------------------------------------------------
            -- If Error appears within FD Data Phase node has to switch back
            --------------------------------------------------------------------
            sp_control_r        <= NOMINAL_SAMPLE;
            crc_enable_r        <= '0';

            --------------------------------------------------------------------
            -- Here we force data to be dominant event if not trigger is used! 
            -- This helps in situations when error is detected during Data bit 
            -- rates!!
            --------------------------------------------------------------------
            data_tx_r           <= DOMINANT;

            -- Storing in RX Buffer must be aborted regardless of OP State.
            rec_abort_r         <= '1';

            --------------------------------------------------------------------
            -- If unit is transciever and Error appears then rettransmitt
            -- counter should be incremented
            --------------------------------------------------------------------
            if (OP_State = transciever) then

                ----------------------------------------------------------------
                -- Retransmitt limit is disabled, or enabled and not reached.
                ----------------------------------------------------------------
                if ((drv_retr_lim_ena = '0') or 
                    (drv_retr_lim_ena = '1' and
                     retr_count < to_integer(unsigned(drv_retr_th))))
                then
                    retr_count         <= (retr_count + 1) mod 16;
                    txt_hw_cmd.err     <= '1';
                else

                    ------------------------------------------------------------
                    -- Retransmitt limit reached, signal transmission failure
                    -- Erase the retransmitt counter, since the next frame
                    -- can be from the same buffer, but it can be different frame!
                    -- Thus retr_counter wont be erased on "txt_buf_changed"!
                    ------------------------------------------------------------
                    retr_count            <= 0;
                    txt_hw_cmd.failed     <= '1';
                end if;

                txt_hw_cmd.unlock           <= '1';
                is_txt_locked               <= '0';

                ----------------------------------------------------------------
                -- Transmitter started to transmitt error flag -> increase by 8 
                -- except ACK error for error passive
                -- Or Stuff Error appeared during arbitration!
                ----------------------------------------------------------------
                if ((error_state = error_passive and ack_error_r = '1') or 
                    (stuff_err_arb_int = '1'))
                then
                    inc_eight_r             <= '0';
                else  
                    inc_eight_r             <= '1';
                end if;
            end if;

            --------------------------------------------------------------------
            -- If Bus Monitoring mode is enabled then data has to be looped 
            -- back before sending on the bus!
            --------------------------------------------------------------------
            if (drv_bus_mon_ena = '1') then 
                int_loop_back_ena_r         <= '1';
            end if;
          
        else
          
            case  err_frame_state is

                ----------------------------------------------------------------
                -- Transmition of error flag and Reception of Error
                -- flag superposition 
                ----------------------------------------------------------------
                when err_flg_sup =>

                    if (tran_trig = '1') then
                        if (control_pointer_non_zero and
                            error_state = error_active)

                        -- Sending active error flag
                        then
                            data_tx_r <=  DOMINANT;

                        -- Sending passive error flag or one bit after
                        -- active error flag!
                        else  
                            data_tx_r <=  RECESSIVE;
                        end if;
                    end if;

                    if (rec_trig = '1') then 

                        --------------------------------------------------------
                        -- Bit error detection during active error flag ->
                        -- next Error and increase counter by 8!
                        --------------------------------------------------------
                        if (data_tx_r = DOMINANT and data_rx = RECESSIVE) then
                            FSM_Preset      <= '1';

                            -- Reciever error counter increased by 8.s
                            if (OP_State = reciever) then
                                inc_eight_r   <= '1';
                            end if;
                        else
                            if (error_state = error_active) then

                                ------------------------------------------------
                                -- Decreasing counters
                                ------------------------------------------------
                                if (control_pointer_non_zero) then
                                    control_pointer     <= control_pointer - 1;
                                end if;

                                if (tran_pointer > 0) then
                                    tran_pointer        <= tran_pointer - 1;
                                end if;

                                ------------------------------------------------
                                -- Only in the last bit, if detected earlier  
                                -- then bit error apeared
                                ------------------------------------------------
                                if (data_rx = RECESSIVE) then
                                    err_frame_state     <= err_delim;
                                    control_pointer     <= 6; 
                                    -- Note: this has to be 6 not 7 (duration of 
                                    -- err_delim is 8) because one bit is sent 
                                    -- recessive and detected

                                ------------------------------------------------
                                -- We accepted 14-th consecutive DOMINANT bit ->
                                -- Error again??
                                ------------------------------------------------
                                elsif (data_rx = DOMINANT and tran_pointer = 0)
                                then
                                    FSM_preset              <= '1';
                                    int_loop_back_ena_r     <= '0';

                                    -- For reciever error counter increased by 8. 
                                    -- For transciever this is done in FSM_Preset!
                                    if (OP_State = reciever) then
                                        inc_eight_r         <= '1'; 
                                    end if;
                                end if;

                                ------------------------------------------------
                                -- This condition is causes that Reciever that 
                                -- was the first to detect the error has error 
                                -- counter increased by 8! Transciever counter 
                                -- is increased in FSM preset! Any next reciever
                                -- that will hook up, will hook up at the end of
                                -- error flag superposition Thus after
                                -- transmitting its error flag there will be
                                -- recessive bit, not dominant!
                                ------------------------------------------------
                                if ((tran_pointer = 6) and (OP_State = reciever))
                                then
                                    -- First bit detected Dominant after active 
                                    -- error flag was sent!
                                    inc_eight_r             <= '1';
                                end if;
                            end if;

                            if (error_state = error_passive) then
                          
                                -- Storing last recieved data
                                err_pas_bit_val <=  data_rx;
                                
                                ------------------------------------------------
                                -- Detecting 6 consecutive bits of
                                -- equal polarity
                                ------------------------------------------------
                                if (control_pointer = 6) then
                                    control_pointer   <= control_pointer - 1;
                                else
                                    if (data_rx = err_pas_bit_val) then

                                        -- So far less than 6 bits -> count down
                                        if (control_pointer > 1) then
                                            control_pointer <= 
                                                control_pointer - 1;

                                        -- Six equal consecutive bits detected
                                        else
                                            err_frame_state   <= err_delim;
                                            control_pointer   <= 6; 
                                        end if;

                                    -- Restart the detection (with one bit less
                                    -- because then the first bit is already the 
                                    -- bit where mismatch appeared)
                                    else
                                        control_pointer       <=  5; 
                                    end if;
                                end if;

                            end if;

                            -- Transition to bus-off                           
                            if (error_state = bus_off) then
                                PC_State    <= off;
                                FSM_Preset  <= '1';
                            end if;
                        end if;
                    end if;  

                ----------------------------------------------------------------
                -- Error delimiter
                ----------------------------------------------------------------
                when err_delim =>
                    if (tran_trig = '1') then 
                        data_tx_r   <=  RECESSIVE;
                    end if;

                    if (rec_trig = '1') then
                        if (control_pointer_non_zero) then
                            control_pointer <=  control_pointer - 1;
                        else
                            ----------------------------------------------------
                            -- DOMINANT in last bit of Error delimiter is
                            -- overload condition! Otherwise Interframe!
                            ----------------------------------------------------
                            if (data_rx = DOMINANT) then
                                PC_State    <= overload;
                            else
                                PC_State    <= interframe;
                            end if;
                            FSM_Preset          <= '1';
                            int_loop_back_ena_r <= '0';
                        end if;
                    end if;

                ----------------------------------------------------------------
                -- Other, invalid states!
                ----------------------------------------------------------------
                when others =>
                      PC_State                  <= error;
                      FSM_preset                <= '1';
                      int_loop_back_ena_r       <= '0';
            end case;
        end if;
      
      
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Overload frame
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when overload =>  
        if (FSM_Preset = '1') then
            FSM_Preset          <= '0';

            -- Pointer for sending the overload flag
            control_pointer     <= 6;

            -- Pointer for recieving the superposition of ovverload flags
            tran_pointer        <= 13;

            ovr_frame_state     <= ovr_flg_sup;
            stuff_enable_r      <= '0';
            destuff_enable_r    <= '0'; 
            crc_enable_r        <= '0';

            --------------------------------------------------------------------
            -- If Bus Monitoring mode is enabled then data has to be 
            -- looped back before sending on the bus!
            --------------------------------------------------------------------
            if (drv_bus_mon_ena = '1') then 
                int_loop_back_ena_r     <= '1';
            end if;

        else
            case  ovr_frame_state is

                ----------------------------------------------------------------
                -- Transmition of overload flag and reception of 
                -- overload flag superposition!
                ----------------------------------------------------------------
                when ovr_flg_sup =>

                    ------------------------------------------------------------
                    -- Transmitting overload flag
                    ------------------------------------------------------------
                    if (tran_trig = '1') then
                        if (control_pointer_non_zero) then
                            data_tx_r   <= DOMINANT;
                        else 
                            data_tx_r   <= RECESSIVE;
                        end if;
                    end if;
                    
                    ------------------------------------------------------------
                    -- Receiving Overload flag superposition                    
                    ------------------------------------------------------------
                    if (rec_trig = '1') then

                        --------------------------------------------------------
                        -- Decreasing counters
                        --------------------------------------------------------
                        if (control_pointer_non_zero) then
                            control_pointer     <= control_pointer - 1;
                        end if;

                        if (tran_pointer > 0) then
                            tran_pointer        <= tran_pointer - 1;
                        end if;

                        --------------------------------------------------------
                        -- Still sending overload flag, but recessive 
                        -- detected -> error frame + increase counter
                        --------------------------------------------------------
                        if (data_rx = RECESSIVE) then
                            if (control_pointer_non_zero) then
                                PC_State            <= error;

                                ------------------------------------------------
                                -- For reciever error counter increased by 8.
                                -- For transciever this is done in FSM_Preset! 
                                ------------------------------------------------
                                if (OP_State = reciever) then
                                    inc_eight_r     <= '1';
                                end if;
                                FSM_Preset          <= '1';
                            else
                                ovr_frame_state     <= ovr_delim;
                                control_pointer     <= 7; 
                            end if;

                        --------------------------------------------------------
                        -- We accepted 14-th consecutive DOMINANT bit in 
                        -- superposition --> Error!
                        --------------------------------------------------------
                        elsif (data_rx = DOMINANT and tran_pointer = 0) then

                            ----------------------------------------------------
                            -- For reciever error counter increased by 8. 
                            -- For transciever this is done in FSM_Preset!
                            ----------------------------------------------------
                            if (OP_State = reciever) then
                                inc_eight_r     <= '1'; 
                            end if;

                            PC_State            <= error;
                            FSM_preset          <= '1';
                            int_loop_back_ena_r <= '0';
                        end if;
                    end if;

                ----------------------------------------------------------------
                -- Overlad delimiter
                ----------------------------------------------------------------
                when ovr_delim =>
                    if (tran_trig = '1') then 
                        data_tx_r   <=  RECESSIVE;
                    end if;
                    if (rec_trig = '1') then
                        if (control_pointer_non_zero) then
                            control_pointer     <= control_pointer - 1;
                        else
                            PC_State            <= interframe;
                            FSM_preset          <= '1';
                            int_loop_back_ena_r <= '0';
                        end if;  
                    end if;

                when others =>
                      PC_State                  <= error;
                      FSM_preset                <= '1';
                      int_loop_back_ena_r       <= '0';
              end case;
            end if;

        
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Unit is turned off
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when off =>     
        if (drv_ena = ENABLED) then
            if (error_state /= bus_off and OP_State /= integrating) then

                -- Note that here we don't want to execute FSM_Preset! We want
                -- to go directly to "interm_idle", thus we don't think of
                -- SOF bit as of Overload flag!
                FSM_Preset          <= '0';
                PC_State            <= interframe;
                interm_state        <= interm_idle;
            end if;
        end if;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Unknown state
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    when others =>
        PC_State                <= error;
        FSM_preset              <= '1';
    end case;

    end if;

end if;
end process;

end architecture;
