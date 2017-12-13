--------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy 
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is 
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in 
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
-- IN THE SOFTWARE.
--
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN 
-- protocol license from Bosch.
--
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Purpose:
--  Unit test for Protocol Control. The architecture of this test is depicted in picture below. It is inherited
--  from original Protocol Control testbench created during the implementation of the CAN FD IP Core. This 
--  testbench only tests the state machine of protocol control! 
--  Following features of CAN are NOT tested here:
--    - bit stuffing and destuffing (frames transmitted from protocol control is without bit stuffing)
--    - CRC calculation  (CRC is provided to protocol control and in this circuit always fixed value is provided)
--    - stuff and destuff counters are always zero provided to the Protocol control
--    - Synchronization - triggerring signals are fixed in this testbench, no synchronization is happenning!
--    - error detection!
--  This functionality is tested in according feature tests!!
--
-- Architecture:
--   
--            -----------        ---------------------
--      |-----| Receive |<-------| Protocol control 2|-----|
--      |     |  frame  |        ---------------------     |
--      |     -----------                                  |  bus_level
--      |                                                  |
--      |    -----------         ---------------------     |                  ---------------------
--      |    | Generate|-------->| Protocol control 1|-----|----------------->|Record the frame as|
--      |    |   frame |    |    ---------------------                        |   bit sequence    |
--      |    -----------    |                                                 ---------------------
--      |                   |         -----------------------                           |
--      |                   |-------->| SW CAN Imlementation|---------------------|     | 
--      |                        |    -----------------------   expcted frame     v     v recorded frame
--      |                        |                                               ----------
--      -------------------      |                                               | compare|
--                        |      |                                               ----------
--                        v      v                                                    |
--                        ---------                                                   |
--                        |compare|                                                   |
--                        ---------                                                   v
--                            |                                                    Error if
--                            v                                                  not matching
--                         Error if
--                       not matching
--
--
--  Test sequence:
--    1. Generate random frame on input of Protocol Control 1
--    2. Calculate the expected bit sequence (frame) on the CAN bus including ACK and EOF!
--    3. Transmitt the frame and record the bit sequence!
--    4. Compare if Expected bit sequence is equal to recorded one
--    5. Compare if Generated frame (data,ident,type of frame...) is equal to received one!
--    6. If points 4 or 5 give mismatch increase error counter
--    7. Loop points 1 to 6 until the number of iterations was reached!
--
--    Note that since additional function for buidling CAN frame in "SW" is used it verifies the
--    protocol control towards errors which can be not detected when both PC1 and PC2 have the
--    error. E.g if PC has missing bit then it wont transmitt it, it wont receive it and it will
--    communicate hapilly further! But the transmitted frame would not be according to specification!s
--    Especially this was reason to implemnt CAN Frame buildup as SW(behavioral) procedure!
--
-----------------------------------------------------------------------------------------------------------------
-- Revision History:
--    13.6.2016   Created file
--    22.6.2016   Modified tb to be compliant with latest bugfixes in protocol control!
--
-----------------------------------------------------------------------------------------------------------------


-----------------------------------------------------------------------------------------------------------------
-- Test implementation                                            
-----------------------------------------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.ID_transfer.all;

architecture Protocol_Control_unit_test of CAN_test is
    
    -----------------------------------------------------------------------------------------------------------------
    -- Common Signals from/to PC1 and PC2
    -----------------------------------------------------------------------------------------------------------------  
    signal clk_sys                  :  std_logic:='0'; --System clock
    signal res_n                    :  std_logic:='0'; --Async reset   
    signal drv_bus                  :  std_logic_vector(1023 downto 0)  :=  (OTHERS => '0');  --Driving bus signals
    signal crc15                    :  std_logic_vector(14 downto 0)    :=  (OTHERS => '0');  --CRC 15
    signal crc17                    :  std_logic_vector(16 downto 0)    :=  (OTHERS => '0');  --CRC 17
    signal crc21                    :  std_logic_vector(20 downto 0)    :=  (OTHERS => '0');  --CRC 21
      
    -----------------------------------------------------------------------------------------------------------------
    -- Signals from/to PC1
    -----------------------------------------------------------------------------------------------------------------  
    signal tran_data_1              :  std_logic_vector(511 downto 0):=  (OTHERS => '0');
    signal tran_ident_1             :  std_logic_vector(28 downto 0):=  (OTHERS => '0');
    signal tran_dlc_1               :  std_logic_vector(3 downto 0):=  (OTHERS => '0');
    signal tran_is_rtr_1            :  std_logic:='0';
    signal tran_ident_type_1        :  std_logic:='0';
    signal tran_frame_type_1        :  std_logic:='0';
    signal tran_brs_1               :  std_logic:='0';     
    signal frame_store_1            :  std_logic:='0'; --Store frame from TX Arbitrator to the Transcieve Buffer
    signal tran_frame_valid_in_1    :  std_logic:='0'; --Valid frame ready to be stored into Transcieeve Buffer
    signal tran_data_ack_1          :  std_logic:='0'; --Acknowledge that the frame was stored
    signal rec_data_1               :  std_logic_vector(511 downto 0);
    signal rec_ident_1              :  std_logic_vector(28 downto 0);
    signal rec_dlc_1                :  std_logic_vector(3 downto 0);
    signal rec_is_rtr_1             :  std_logic;
    signal rec_ident_type_1         :  std_logic;
    signal rec_frame_type_1         :  std_logic;
    signal rec_brs_1                :  std_logic;
    signal rec_crc_1                :  std_logic_vector(20 downto 0); --Recieved CRC value
    signal rec_esi_1                :  std_logic; --Recieved Error state indicator
    signal OP_state_1               :  oper_mode_type; --Operation mode state
    signal arbitration_lost_1       :  std_logic; --Signal for Operational mode state mahine about loosing arbitration
    signal is_idle_1                :  std_logic; --Signal to indicate transcieve or recieve finished and bus is idle
    signal set_transciever_1        :  std_logic; --Set OP_State FSM into transciever state (Used at SOF)
    signal set_reciever_1           :  std_logic; --Set OP_State FSM into reciever state
    signal alc_1                    :  std_logic_vector(4 downto 0);
    signal error_state_1            :  error_state_type; --Fault confinement state
    signal form_Error_1             :  std_logic; --Form Error
    signal CRC_Error_1              :  std_logic; --CRC Error
    signal ack_Error_1              :  std_logic; --Acknowledge error
    signal unknown_state_Error_1    :  std_logic; --Some of the state machines, 
    signal bit_stuff_Error_valid_1  :  std_logic; 
    signal inc_one_1                :  std_logic;
    signal inc_eight_1              :  std_logic;
    signal dec_one_1                :  std_logic;    
    signal tran_valid_1             :  std_logic;
    signal rec_valid_1              :  std_logic;
    signal ack_recieved_out_1       :  std_logic;        
    signal br_shifted_1             :  std_logic;
    signal data_tx_1                :  std_logic; --Transcieved data on CAN Bus
    signal stuff_enable_1           :  std_logic;
    signal fixed_stuff_1            :  std_logic; --Log 1 - Fixed Stuffing, Log 0 - Normal stuffing
    signal stuff_length_1           :  std_logic_vector(2 downto 0); --Stuffing length
    signal destuff_enable_1         :  std_logic; --Enabling destuffing
    signal stuff_error_enable_1     :  std_logic; --Enabling firing of destuffing error
    signal fixed_destuff_1          :  std_logic; --Fixed stuffing method (log. 1), Normal stuffing (log 0);
    signal destuff_length_1         :  std_logic_vector(2 downto 0); --Number of equal consequent bits before destuffed bit 
    signal dst_ctr_1                :  natural range 0 to 7; --Number of stuffed bits modulo 8
    signal crc_enable_1             :  std_logic; --Transition from 0 to 1 erases the CRC and operation holds as long as enable=1
    signal sync_control_1           :  std_logic_vector(1 downto 0); 
    signal sp_control_1             :  std_logic_vector(1 downto 0); --00 nominal, 01-data, 10 -secondary
    signal ssp_reset_1              :  std_logic; --Clear the Shift register at the  beginning of Data Phase!!!    
    signal trv_delay_calib_1        :  std_logic; --Calibration command for transciever delay compenstation (counter)
    signal bit_err_enable_1         :  std_logic; --Bit Error detection enable (Ex. disabled when recieving data)
    signal hard_sync_edge_1         :  std_logic; --Synchronisation edge validated by prescaler!!!
    signal int_loop_back_ena_1      :  std_logic; --Internal loopBack enabled (for Bus monitoring mode)
    signal PC_State_out_1           :  protocol_type;
    
    -----------------------------------------------------------------------------------------------------------------
    -- Signals from/to PC2
    -----------------------------------------------------------------------------------------------------------------  
    signal tran_data_2              :  std_logic_vector(511 downto 0):=  (OTHERS => '0');
    signal tran_ident_2             :  std_logic_vector(28 downto 0):=  (OTHERS => '0');
    signal tran_dlc_2               :  std_logic_vector(3 downto 0):=  (OTHERS => '0');
    signal tran_is_rtr_2            :  std_logic:='0';
    signal tran_ident_type_2        :  std_logic:='0';
    signal tran_frame_type_2        :  std_logic:='0';
    signal tran_brs_2               :  std_logic:='0';     
    signal frame_store_2            :  std_logic; --Store frame from TX Arbitrator to the Transcieve Buffer
    signal tran_frame_valid_in_2    :  std_logic:='0'; --Valid frame ready to be stored into Transcieeve Buffer
    signal tran_data_ack_2          :  std_logic; --Acknowledge that the frame was stored
    signal rec_data_2               :  std_logic_vector(511 downto 0);
    signal rec_ident_2              :  std_logic_vector(28 downto 0);
    signal rec_dlc_2                :  std_logic_vector(3 downto 0);
    signal rec_is_rtr_2             :  std_logic;
    signal rec_ident_type_2         :  std_logic;
    signal rec_frame_type_2         :  std_logic;
    signal rec_brs_2                :  std_logic;
    signal rec_crc_2                :  std_logic_vector(20 downto 0); --Recieved CRC value
    signal rec_esi_2                :  std_logic; --Recieved Error state indicator
    signal OP_state_2               :  oper_mode_type; --Operation mode state
    signal arbitration_lost_2       :  std_logic; --Signal for Operational mode state mahine about loosing arbitration
    signal is_idle_2                :  std_logic; --Signal to indicate transcieve or recieve finished and bus is idle
    signal set_transciever_2        :  std_logic; --Set OP_State FSM into transciever state (Used at SOF)
    signal set_reciever_2           :  std_logic; --Set OP_State FSM into reciever state
    signal alc_2                    :  std_logic_vector(4 downto 0);
    signal error_state_2            :  error_state_type; --Fault confinement state
    signal form_Error_2             :  std_logic; --Form Error
    signal CRC_Error_2              :  std_logic; --CRC Error
    signal ack_Error_2              :  std_logic; --Acknowledge error
    signal unknown_state_Error_2    :  std_logic; --Some of the state machines, 
    signal bit_stuff_Error_valid_2  :  std_logic; 
    signal inc_one_2                :  std_logic;
    signal inc_eight_2              :  std_logic;
    signal dec_one_2                :  std_logic;    
    signal tran_valid_2             :  std_logic;
    signal rec_valid_2              :  std_logic;
    signal ack_recieved_out_2       :  std_logic;        
    signal br_shifted_2             :  std_logic;
    signal data_tx_2                :  std_logic; --Transcieved data on CAN Bus
    signal stuff_enable_2           :  std_logic;
    signal fixed_stuff_2            :  std_logic; --Log 1 - Fixed Stuffing, Log 0 - Normal stuffing
    signal stuff_length_2           :  std_logic_vector(2 downto 0); --Stuffing length
    signal destuff_enable_2          :  std_logic; --Enabling destuffing
    signal stuff_error_enable_2     :  std_logic; --Enabling firing of destuffing error
    signal fixed_destuff_2          :  std_logic; --Fixed stuffing method (log. 1), Normal stuffing (log 0);
    signal destuff_length_2         :  std_logic_vector(2 downto 0); --Number of equal consequent bits before destuffed bit 
    signal dst_ctr_2                :  natural range 0 to 7; --Number of stuffed bits modulo 8
    signal crc_enable_2             :  std_logic; --Transition from 0 to 1 erases the CRC and operation holds as long as enable=1
    signal sync_control_2           :  std_logic_vector(1 downto 0); 
    signal sp_control_2             :  std_logic_vector(1 downto 0); --00 nominal, 01-data, 10 -secondary
    signal ssp_reset_2              :  std_logic; --Clear the Shift register at the  beginning of Data Phase!!!    
    signal trv_delay_calib_2        :  std_logic; --Calibration command for transciever delay compenstation (counter)
    signal bit_err_enable_2         :  std_logic; --Bit Error detection enable (Ex. disabled when recieving data)
    signal hard_sync_edge_2         :  std_logic; --Synchronisation edge validated by prescaler!!!
    signal int_loop_back_ena_2      :  std_logic; --Internal loopBack enabled (for Bus monitoring mode)
    signal PC_State_out_2           :  protocol_type;
    
    -----------------------------------------------------------------------------------------------------------------
    -- Internal testbench signals
    -----------------------------------------------------------------------------------------------------------------  
    signal bus_level                :  std_logic;
    signal rnd_ctr_tr               :  natural range 0 to RAND_POOL_SIZE:=0;
    signal tx_trig                  :  std_logic;
    signal rx_trig                  :  std_logic;
    signal tx_frame                 :  CAN_frame_type;
    signal rx_frame                 :  CAN_frame_type;
    signal prev_bus                 :  std_logic;
    signal error_st                 :  error_state_type:=error_active;
    
    -----------------------
    --Driving bus aliases--
    -----------------------
    signal drv_rtr_pref             :     std_logic; --RTR behavior setting
    signal drv_CAN_fd_ena           :     std_logic; --Whenever FD Frames are supported for reciever
    signal drv_bus_mon_ena          :     std_logic; --Bus Monitoring mode enabled
    signal drv_retr_lim_ena         :     std_logic; --Retransmition limit enabled for errornous frames
    signal drv_retr_th              :     std_logic_vector(3 downto 0); --Retransmittion treshold
    signal drv_self_test_ena        :     std_logic; --Self Test Mode enabled
    signal drv_abort_tran           :     std_logic; --Immediately abort transmittion
    signal drv_ack_forb             :     std_logic; --Forbidding acknowledge mode
    signal drv_ena                  :     std_logic; --Enabling the whole controller
    signal drv_fd_type              :     std_logic; --Type of FD Format Frame (ISO,non-ISO)
    
    -----------------------------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------------------------
    -- Testbench procedures and functions
    -----------------------------------------------------------------------------------------------------------------  
    -----------------------------------------------------------------------------------------------------------------
    
    -----------------------------------------------------------------------------------------------------------------
    --Generagtes Random CAN frame to be transmitted by PC1
    -----------------------------------------------------------------------------------------------------------------
    procedure generate_tx_frame(
      signal rand_ctr   :inout     natural range 0 to RAND_POOL_SIZE;
      signal tx_frame   :inout     CAN_frame_type
    )is
    begin
      rand_logic_vect (rand_ctr, tx_frame.rec_ident_in,      0.3);
      rand_logic_vect (rand_ctr, tx_frame.rec_data_in,       0.5);
      rand_logic_vect (rand_ctr, tx_frame.rec_dlc_in,        0.5);
      
      rand_logic      (rand_ctr, tx_frame.rec_ident_type_in, 0.5);
      rand_logic      (rand_ctr, tx_frame.rec_frame_type_in, 0.5);
      rand_logic      (rand_ctr, tx_frame.rec_is_rtr,        0.5);
      rand_logic      (rand_ctr, tx_frame.rec_brs,           0.5);
      
      if(tx_frame.rec_frame_type_in=NORMAL_CAN)then
        tx_frame.rec_brs<='0';
      end if;
      
      if(tx_frame.rec_frame_type_in=FD_CAN)then
        tx_frame.rec_is_rtr<='0';
      end if;
      
      --Here we test only the Protocol correct RTR behaviour
      --That is if we send RTR FRAME then we send always zero DLC
      if(tx_frame.rec_is_rtr='1')then
        tx_frame.rec_dlc_in<="0000";
      end if;
      
    end procedure;
    
    -----------------------------------------------------------------------------------------------------------------
    --Compare two frames
    -----------------------------------------------------------------------------------------------------------------
    procedure compare_frame(
      signal    tx_frame    :in     CAN_frame_type;
      signal    rx_frame    :in     CAN_frame_type;
      variable  outcome     :out    boolean;
      signal    log_level   :in     log_lvl_type
    )is
    variable    data_length :       natural;
    begin
      outcome:=true;
      
      --Frame type signals
      if(tx_frame.rec_dlc_in          /= rx_frame.rec_dlc_in          OR
         tx_frame.rec_frame_type_in   /= rx_frame.rec_frame_type_in   OR
         tx_frame.rec_ident_type_in   /= rx_frame.rec_ident_type_in   OR
         tx_frame.rec_is_rtr          /= rx_frame.rec_is_rtr          OR
         tx_frame.rec_brs             /= rx_frame.rec_brs             )
      then
         log("Control signals mismatch between TX and RX frame",error_l,log_level);
         outcome:=false;
      end if;
      
      --Identifier comparison
      if(tx_frame.rec_ident_type_in=FRAME_BASIC)then
        if(tx_frame.rec_ident_in(10 downto 0) /= rx_frame.rec_ident_in(10 downto 0))then
          log("Identifier mismatch between TX and RX frame",error_l,log_level);
          outcome:=false;
        end if;
      else
        if(tx_frame.rec_ident_in(28 downto 0) /= rx_frame.rec_ident_in(28 downto 0))then
          log("Identifier mismatch between TX and RX frame",error_l,log_level);
          outcome:=false;
        end if;
      end if;
      
      --Data comparison
      decode_dlc(tx_frame.rec_dlc_in,data_length);
      if(data_length>0)then
        for i in 1 to data_length loop
            if(tx_frame.rec_data_in(511-((i-1)*8) downto 504-((i-1)*8)) /= 
               rx_frame.rec_data_in(511-((i-1)*8) downto 504-((i-1)*8)))then
              log("Data mismatch between TX and RX frame",error_l,log_level);
              outcome:=false; 
            end if;
        end loop;  
      end if;
      
    end procedure;
    
    -----------------------------------------------------------------------------------------------------------------
    --Generate expected bit sequence on the otput of Protocol Control
    -- THis method is SW implementation of CAN protocol
    -----------------------------------------------------------------------------------------------------------------
    procedure gen_sw_CAN(
      signal   tx_frame   :in   CAN_frame_type;
      signal   error_st   :in   error_state_type;
      signal   iso_fd     :in   std_logic;
      variable seq        :out  std_logic_vector(639 downto 0); --Generated bit sequence
      variable length     :out  natural  --Length of the generated sequence
    )is
    variable join         :     std_logic_vector(1 downto 0);
    variable pointer      :     natural;
    variable data_length  :     natural;
    variable crc_length   :     natural;
    begin
      
      --SOF bit
      seq(0)  := DOMINANT;
      
      --Base identifier
      for i in 0 to 10 loop
        seq(1+i) := tx_frame.rec_ident_in(10-i);
      end loop;
      
      --All fields until begginning of data phase
      join       := tx_frame.rec_frame_type_in & tx_frame.rec_ident_type_in;
      case join is
      when "00" =>  --CAN Base
              seq(12) := tx_frame.rec_is_rtr;
              seq(13) := DOMINANT; --IDE bit
              seq(14) := DOMINANT; --r0 bit
              for j in 0 to 3 loop --DLC bits
                seq(15+j):= tx_frame.rec_dlc_in(3-j);
              end loop;
              pointer:= 19;
              
      when "01" =>  --CAN Extended
              seq(12) := RECESSIVE;
              seq(13) := RECESSIVE;
              for j in 0 to 17 loop
                seq(14+j):= tx_frame.rec_ident_in(28-j);
              end loop;
              seq(32) := tx_frame.rec_is_rtr;
              seq(33) := DOMINANT;
              seq(34) := DOMINANT;
              for j in 0 to 3 loop --DLC bits
                seq(35+j):= tx_frame.rec_dlc_in(3-j);
              end loop;
              pointer:= 39;
              
      when "10" =>  --CAN FD Base
              seq(12) := DOMINANT;
              seq(13) := DOMINANT;
              seq(14) := RECESSIVE; --EDL bit
              seq(15) := DOMINANT;  --r0 bit
              seq(16) := tx_frame.rec_brs;
              
              --ESI bit
              if(error_st = error_active)then
                seq(17) := DOMINANT;
              else
                seq(17) := RECESSIVE;
              end if;
              
              for j in 0 to 3 loop --DLC bits
                seq(18+j):= tx_frame.rec_dlc_in(3-j);
              end loop;
              pointer:= 22;
              
      when "11" =>  --CAN FD Extended
              seq(12) := RECESSIVE;
              seq(13) := RECESSIVE;
              for j in 0 to 17 loop
                seq(14+j):= tx_frame.rec_ident_in(28-j);
              end loop;
              seq(32) := DOMINANT;
              seq(33) := RECESSIVE;
              seq(34) := DOMINANT;
              seq(35) := tx_frame.rec_brs;
              --ESI bit
              if(error_st = error_active)then
                seq(36) := DOMINANT;
              else
                seq(36) := RECESSIVE;
              end if;
               for j in 0 to 3 loop --DLC bits
                seq(37+j):= tx_frame.rec_dlc_in(3-j);
              end loop;
              pointer:= 41;
              
      when others =>
      end case;
      
      --Data field
      decode_dlc(tx_frame.rec_dlc_in,data_length);
      if(data_length>0 and tx_frame.rec_is_rtr='0')then
        
        --Copy in the data by bits
        for i in 0 to (data_length*8)-1 loop
          seq(pointer):= tx_frame.rec_data_in(511-i);
          pointer     := pointer+1;
        end loop;
        
      end if;
      
      --CRC field
      if(iso_fd = '0' and tx_frame.rec_frame_type_in=FD_CAN)then  --Stuff count       
         --Stuff count is inserted fixed, we dont test CRC error detection here
         seq(pointer)   :=  '0';
         seq(pointer+1) :=  '0'; 
         seq(pointer+2) :=  '0';
         seq(pointer+3) :=  '0';
         pointer        :=  pointer+4;
      end if;
      
      if(data_length>16)then
        crc_length      :=  21;
      elsif(data_length>8)then
        crc_length      :=  17;
      else
        crc_length      :=  15;
      end if;
      
      --Here we put constant CRC, since we dont test CRC functionality here...
      for i in 0 to crc_length-1 loop
        if (i mod 2 = 0)then
          seq(pointer)  := '1';
        else
          seq(pointer)  := '0';
        end if;
        pointer         :=  pointer+1;
      end loop;
      
      --CRC delimiter
      seq(pointer)      := RECESSIVE;
      pointer           := pointer+1;
      
      --ACK bit
      --We assume frame will be received correctly
      seq(pointer)      := DOMINANT;
      pointer           := pointer+1;
      
      --ACK delimiter
      seq(pointer)      := RECESSIVE;
      pointer           := pointer+1;
      
      --EOF
      for i in 0 to 6 loop
        seq(pointer)    := RECESSIVE;
        pointer         := pointer+1;
      end loop;
      
      --Now we propagate the length to the output
      length              := pointer+1;
      
    end procedure;
    
    
    -----------------------------------------------------------------------------------------------------------------
    --Record what is on the bus! With known length from SW CAN what should be there
    -----------------------------------------------------------------------------------------------------------------
    procedure record_bit_seq(
      signal   bus_line             :in   std_logic; --Bus level to be recorded
      signal   sample               :in   std_logic; --Signal to sample the bus level
      variable length               :in   natural;   --How many bits should be recorded
      variable recorded             :out  std_logic_vector(639 downto 0);
      signal tran_frame_valid_in_1  :out std_logic -- Frame valid must be disabled during the transmittion
    )is
    begin
      
      --First wait until we are on SOF bit
      wait until bus_line=DOMINANT;
      
      for i in 1 to length loop
        
        if(i=10)then
          tran_frame_valid_in_1<='0';
        end if;
        
        wait until rising_edge(sample);
        recorded(i-1):= bus_line;
      end loop;
      
    end procedure;
    
    -------------------------------------------------------------------------------
    --Compare what is expected to be transmitted and what is acutally transmitted
    -------------------------------------------------------------------------------
    procedure compare_bit_seq(
      variable bs1        :in   std_logic_vector(639 downto 0);
      variable bs2        :in   std_logic_vector(639 downto 0);
      variable mut_length :in   natural; --Mutual length of both vectors
      variable outcome    :out  boolean
    )is
    begin
      outcome:= true;
      
      for i in 1 to mut_length-1 loop
        if(bs1(i-1) /= bs2(i-1))then
          outcome:=false;
        end if;
      end loop;
      
    end procedure;
    
begin
  
    protocolControl_1_Comp:protocolControl
    port map(
     clk_sys                =>  clk_sys,
     res_n                  =>  res_n,
     drv_bus                =>  drv_bus,
     tran_data              =>  tran_data_1,
     tran_ident             =>  tran_ident_1,
     tran_dlc               =>  tran_dlc_1,
     tran_is_rtr            =>  tran_is_rtr_1,
     tran_ident_type        =>  tran_ident_type_1,
     tran_frame_type        =>  tran_frame_type_1,
     tran_brs               =>  tran_brs_1,
     frame_store            =>  frame_store_1,
     tran_frame_valid_in    =>  tran_frame_valid_in_1,
     tran_data_ack          =>  tran_data_ack_1,
     rec_data               =>  rec_data_1,
     rec_ident              =>  rec_ident_1,
     rec_dlc                =>  rec_dlc_1,
     rec_is_rtr             =>  rec_is_rtr_1,
     rec_ident_type         =>  rec_ident_type_1,
     rec_frame_type         =>  rec_frame_type_1,
     rec_brs                =>  rec_brs_1,
     rec_crc                =>  rec_crc_1,
     rec_esi                =>  rec_esi_1,
     OP_state               =>  OP_state_1,
     arbitration_lost       =>  arbitration_lost_1,
     is_idle                =>  is_idle_1,
     set_transciever        =>  set_transciever_1,
     set_reciever           =>  set_reciever_1,
     alc                    =>  alc_1,
     error_state            =>  error_state_1,
     form_Error             =>  form_Error_1,
     CRC_Error              =>  CRC_Error_1,
     ack_Error              =>  ack_Error_1,
     unknown_state_Error    =>  unknown_state_Error_1,
     bit_stuff_Error_valid  =>  bit_stuff_Error_valid_1,
     inc_one                =>  inc_one_1,
     inc_eight              =>  inc_eight_1,
     dec_one                =>  dec_one_1,
     tran_valid             =>  tran_valid_1,
     rec_valid              =>  rec_valid_1,
     ack_recieved_out       =>  ack_recieved_out_1,
     br_shifted             =>  br_shifted_1,
     tran_trig              =>  tx_trig,
     rec_trig               =>  rx_trig,
     data_tx                =>  data_tx_1,
     stuff_enable           =>  stuff_enable_1,
     fixed_stuff            =>  fixed_stuff_1,
     stuff_length           =>  stuff_length_1,
     data_rx                =>  bus_level,
     destuff_enable         =>  destuff_enable_1,
     stuff_error_enable     =>  stuff_error_enable_1,
     fixed_destuff          =>  fixed_destuff_1,
     destuff_length         =>  destuff_length_1,
     dst_ctr                =>  dst_ctr_1,
     crc_enable             =>  crc_enable_1,
     crc15                  =>  crc15,
     crc17                  =>  crc17,
     crc21                  =>  crc21,
     sync_control           =>  sync_control_1,
     sp_control             =>  sp_control_1,
     ssp_reset              =>  ssp_reset_1,
     trv_delay_calib        =>  trv_delay_calib_1,
     bit_err_enable         =>  bit_err_enable_1,
     hard_sync_edge         =>  hard_sync_edge_1,
     int_loop_back_ena      =>  int_loop_back_ena_1,
     PC_State_out           =>  PC_State_out_1
    );
    
    protocolControl_2_Comp:protocolControl
    port map(
     clk_sys                =>  clk_sys,
     res_n                  =>  res_n,
     drv_bus                =>  drv_bus,
     tran_data              =>  tran_data_2,
     tran_ident             =>  tran_ident_2,
     tran_dlc               =>  tran_dlc_2,
     tran_is_rtr            =>  tran_is_rtr_2,
     tran_ident_type        =>  tran_ident_type_2,
     tran_frame_type        =>  tran_frame_type_2,
     tran_brs               =>  tran_brs_2,
     frame_store            =>  frame_store_2,
     tran_frame_valid_in    =>  tran_frame_valid_in_2,
     tran_data_ack          =>  tran_data_ack_2,
     rec_data               =>  rec_data_2,
     rec_ident              =>  rec_ident_2,
     rec_dlc                =>  rec_dlc_2,
     rec_is_rtr             =>  rec_is_rtr_2,
     rec_ident_type         =>  rec_ident_type_2,
     rec_frame_type         =>  rec_frame_type_2,
     rec_brs                =>  rec_brs_2,
     rec_crc                =>  rec_crc_2,
     rec_esi                =>  rec_esi_2,
     OP_state               =>  OP_state_2,
     arbitration_lost       =>  arbitration_lost_2,
     is_idle                =>  is_idle_2,
     set_transciever        =>  set_transciever_2,
     set_reciever           =>  set_reciever_2,
     alc                    =>  alc_2,
     error_state            =>  error_state_2,
     form_Error             =>  form_Error_2,
     CRC_Error              =>  CRC_Error_2,
     ack_Error              =>  ack_Error_2,
     unknown_state_Error    =>  unknown_state_Error_2,
     bit_stuff_Error_valid  =>  bit_stuff_Error_valid_2,
     inc_one                =>  inc_one_2,
     inc_eight              =>  inc_eight_2,
     dec_one                =>  dec_one_2,
     tran_valid             =>  tran_valid_2,
     rec_valid              =>  rec_valid_2,
     ack_recieved_out       =>  ack_recieved_out_2,
     br_shifted             =>  br_shifted_2,
     tran_trig              =>  tx_trig,
     rec_trig               =>  rx_trig,
     data_tx                =>  data_tx_2,
     stuff_enable           =>  stuff_enable_2,
     fixed_stuff            =>  fixed_stuff_2,
     stuff_length           =>  stuff_length_2,
     data_rx                =>  bus_level,
     destuff_enable         =>  destuff_enable_2,
     stuff_error_enable     =>  stuff_error_enable_2,
     fixed_destuff          =>  fixed_destuff_2,
     destuff_length         =>  destuff_length_2,
     dst_ctr                =>  dst_ctr_2,
     crc_enable             =>  crc_enable_2,
     crc15                  =>  crc15,
     crc17                  =>  crc17,
     crc21                  =>  crc21,
     sync_control           =>  sync_control_2,
     sp_control             =>  sp_control_2,
     ssp_reset              =>  ssp_reset_2,
     trv_delay_calib        =>  trv_delay_calib_2,
     bit_err_enable         =>  bit_err_enable_2,
     hard_sync_edge         =>  hard_sync_edge_2,
     int_loop_back_ena      =>  int_loop_back_ena_2,
     PC_State_out           =>  PC_State_out_2
    );
  
    
    -------------------------------------------------------------------
    -- Creating bus level
    -------------------------------------------------------------------
    bus_level             <= data_tx_2 AND data_tx_1;
    
    -------------------------------------------------------------------
    -- Assigning record types
    -------------------------------------------------------------------
    tran_data_1           <= tx_frame.rec_data_in;
    tran_ident_1          <= tx_frame.rec_ident_in;
    tran_dlc_1            <= tx_frame.rec_dlc_in;
    tran_is_rtr_1         <= tx_frame.rec_is_rtr;
    tran_ident_type_1     <= tx_frame.rec_ident_type_in;
    tran_frame_type_1     <= tx_frame.rec_frame_type_in;
    tran_brs_1            <= tx_frame.rec_brs;
    
    rx_frame.rec_data_in        <=  rec_data_2         ;
    rx_frame.rec_ident_in       <=  rec_ident_2        ;
    rx_frame.rec_dlc_in         <=  rec_dlc_2          ;
    rx_frame.rec_is_rtr         <=  rec_is_rtr_2       ;
    rx_frame.rec_ident_type_in  <=  rec_ident_type_2   ;
    rx_frame.rec_frame_type_in  <=  rec_frame_type_2   ;
    rx_frame.rec_brs            <=  rec_brs_2          ;   
    
    
    --Driving bus aliases
    drv_bus(DRV_RTR_PREF_INDEX)                       <=  drv_rtr_pref ;
    drv_bus(DRV_CAN_FD_ENA_INDEX)                     <=  drv_CAN_fd_ena ;
    drv_bus(DRV_BUS_MON_ENA_INDEX)                    <=  drv_bus_mon_ena ;
    drv_bus(DRV_RETR_LIM_ENA_INDEX)                   <=  drv_retr_lim_ena  ;
    drv_bus(DRV_RETR_TH_HIGH downto DRV_RETR_TH_LOW)  <=  drv_retr_th    ;
    drv_bus(DRV_SELF_TEST_ENA_INDEX)                  <=  drv_self_test_ena ;
    drv_bus(DRV_ABORT_TRAN_INDEX)                     <=  drv_abort_tran ;
    drv_bus(DRV_ACK_FORB_INDEX)                       <=  drv_ack_forb ;
    drv_bus(DRV_ENA_INDEX)                            <=  drv_ena ;
    drv_bus(DRV_FD_TYPE_INDEX)                        <=  drv_fd_type;
    
    --Some configurations are extra
    drv_rtr_pref                                      <= '0';
    drv_CAN_fd_ena                                    <= '1';
    drv_bus_mon_ena                                   <= '0';
    drv_retr_lim_ena                                  <= '0';
    drv_retr_th                                       <= (OTHERS =>'0');
    drv_self_test_ena                                 <= '0';
    drv_abort_tran                                    <= '0';
    drv_ack_forb                                      <= '0';
    drv_ena                                           <= '1';
    
    
    
    --Constant values
    OP_state_2                <= reciever;
    OP_state_1                <= transciever;
    error_state_2             <= error_active;
    error_state_1             <= error_active;
    bit_stuff_Error_valid_1   <= '0';
    bit_stuff_Error_valid_2   <= '0';
    dst_ctr_2                 <= 0;
    dst_ctr_1                 <= 0;
    
    crc15                     <= "101010101010101";
    crc17                     <= "10101010101010101";
    crc21                     <= "101010101010101010101";
    
    ---------------------------------
    --Clock generation
    ---------------------------------
    clock_gen:process
    variable period   :natural:=f100_Mhz;
    variable duty     :natural:=50;
    variable epsilon  :natural:=0;
    begin
      generate_clock(period,duty,epsilon,clk_sys);
    end process;
    
    ---------------------------------
    -- Sampling signals generation
    ---------------------------------
    sample_gen:process
    
    variable min_diff: natural := 4;
    --Note: With this setting there must be minnimally two clock cycles
    --      between the RX trig and TX trig. When 2 or 1 is set here
    --      test fails since gap between sample and sync is shorter
    --      than information processing time!!
    --      Note that official IPT is 4 clock cycles (3 between) thus
    --      in this test PC can do it with 3. One clock cycle is
    --      reserved and is of use in situations where both nodes
    --      are transmitting at the same time!
    
    begin
      generate_simple_trig(rnd_ctr_tr,tx_trig,rx_trig,clk_sys,min_diff);    
    end process;
    
    ---------------------------------
    -- Hard sync edge generation
    --  emulates the bus_sync
    ---------------------------------
    hs_edge_gen:process(res_n,clk_sys)
    begin
      if(res_n = '0')then
        hard_sync_edge_2 <= '0';
        hard_sync_edge_1 <= '0';
        prev_bus         <= RECESSIVE;
      elsif rising_edge(clk_sys)then
         
         prev_bus<=bus_level;
                
         if(prev_bus /= bus_level and bus_level=DOMINANT)then
           hard_sync_edge_2 <= '1';
           hard_sync_edge_1 <= '1';
         else
           hard_sync_edge_2 <= '0';
           hard_sync_edge_1 <= '0';
        end if;
        
      end if;
       
    end process;
    
    
    ---------------------------------
    ---------------------------------
    --Main Test process
    ---------------------------------
    ---------------------------------
    test_proc:process
    variable sw_seq     : std_logic_vector(639 downto 0):=(OTHERS => '0');
    variable rec_seq    : std_logic_vector(639 downto 0):=(OTHERS => '0');
    variable seq_length : natural;
    variable out_seq    : boolean;
    variable out_frm    : boolean;
    begin
      log("Restarting Protocol control unit test!",info_l,log_level);
      wait for 5 ns;
      reset_test(res_n,status,run,error_ctr);
      log("Restarted Protocol control unit test",info_l,log_level);
      print_test_info(iterations,log_level,error_beh,error_tol);
      
      -------------------------------
      --Main loop of the test
      -------------------------------
      log("Starting Protocol control main loop",info_l,log_level);
      
      while (loop_ctr<iterations  or  exit_imm)
      loop
        log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
        
        --Erase the sequences
        sw_seq     :=(OTHERS => '0');
        rec_seq    :=(OTHERS => '0');
        
        --Generate random frame
        log("Generating frame for transmittion",info_l,log_level);
        generate_tx_frame(rand_ctr,tx_frame);
        rand_logic(rand_ctr,drv_fd_type,0.5);
        
        wait for 0 ns;
        log("Calculating expected frame by SW CAN",info_l,log_level);
        gen_sw_CAN(tx_frame,error_st,drv_fd_type,sw_seq,seq_length);
        
        --Start transmitting
        log("Starting transmittion and recording on bus",info_l,log_level);
        tran_frame_valid_in_1<='1';
        
        --Record what comes out of the controller
        record_bit_seq(bus_level,rx_trig,seq_length,rec_seq,tran_frame_valid_in_1);
           
        --Compare the results
        log("Comparing the results",info_l,log_level);
        compare_frame(tx_frame,rx_frame,out_frm,log_level);
        compare_bit_seq(sw_seq,rec_seq,seq_length,out_seq);
        
        --Evaluate the error
        if(out_seq=false or out_frm=false)then
          process_error(error_ctr,error_beh,exit_imm);
        end if;
        
        loop_ctr<=loop_ctr+1;
      end loop;
      
      evaluate_test(error_tol,error_ctr,status);
    end process;
  
end architecture;


-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture Protocol_Control_unit_test_wrapper of CAN_test_wrapper is
  
  --Test component itself
  component CAN_test is
  port (
    signal run            :in   boolean;                -- Input trigger, test starts running when true
    signal iterations     :in   natural;                -- Number of iterations that test should do
    signal log_level      :in   log_lvl_type;           -- Logging level, severity which should be shown
    signal error_beh      :in   err_beh_type;           -- Test behaviour when error occurs: Quit, or Go on
    signal error_tol      :in   natural;                -- Error tolerance, error counter should not
                                                         -- exceed this value in order for the test to pass
    signal status         :out  test_status_type;      -- Status of the test
    signal errors         :out  natural                -- Amount of errors which appeared in the test
    --TODO: Error log results 
  );
  end component;
  
  --Select architecture of the test
  for test_comp : CAN_test use entity work.CAN_test(Protocol_Control_unit_test);
  
    signal run              :   boolean;                -- Input trigger, test starts running when true                                                        -- exceed this value in order for the test to pass
    signal status_int       :   test_status_type;      -- Status of the test
    signal errors           :   natural;                -- Amount of errors which appeared in the test

begin
  
  --In this test wrapper generics are directly connected to the signals
  -- of test entity
  test_comp:CAN_test
  port map(
     run              =>  run,
     --iterations       =>  10000,
     iterations       =>  iterations , 
     log_level        =>  log_level,
     error_beh        =>  error_beh,
     error_tol        =>  error_tol,                                                     
     status           =>  status_int,
     errors           =>  errors
  );
  
  status              <= status_int;
  
  ------------------------------------
  --Starts the test and lets it run
  ------------------------------------
  test:process
  begin
    run               <= true;
    wait for 1 ns;
    
    --Wait until the only test finishes and then propagate the results
    wait until (status_int=passed or status_int=failed);  
    
    wait for 100 ns;
    run               <= false;
        
  end process;
  
  
end;

