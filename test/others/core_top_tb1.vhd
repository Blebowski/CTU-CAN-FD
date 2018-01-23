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

-------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Testbench for testing top level entity of CAN Core.                                                 --
--  Various test are availiable. Arbitration test. Error Test. Transcieve - Recieve test.               --
--  Enabling the test must be done pre-compilation by constants under "Availiable tests"                --
--  Only one test can be enabled (according constant in logic one).                                     --
--  In all test two nodes are instantiated and messages are sent on the bus. Bus level is created with  --
--  DOMINANT and RECESSIVE values as on real CAN bus. This enables testing of acknowledge, Error flag   --
--  signalisation, arbitration mechanism testing and collisions when same identifier apears.            --
--  Due to many signals in automatic script: script1 in scripts folder can be used for automatic testing--
--------------------------------------------------------------------------------------------------------

--Note: This testbench doesnt have binary result. For checking if State machine isnt stuck only waveforms
--      can are used. Signal DATA_mismatch in logic 1 indicates that data which were sebt by one node 
--      and recieved by another node are not equal. 

--------------------------------------------------------------------------------
-- Revision History:
--
--  July 2015  Original version
-------------------------------------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANcomponents.ALL;
use work.CANconstants.all;
use work.CAN_FD_frame_format.all;

entity core_top_tb1 is
 end entity;

architecture behav of core_top_tb1 is
 
  --Note: signals with nd1 are of instanced node 1, signals with nd2 are of instanced node 2
    ------------------
    --COMMON SINGALS--
    ------------------
    signal sample_nbt_del_2: std_logic; --Sampling signal 2 clk_sys delayed from sample_nbt
    signal sample_dbt_del_2: std_logic; --Sampling signal 2 clk_sys delayed from sample_dbt
    signal sample_nbt_del_1: std_logic; --Sampling signal of NBT for Bit destuffing
    signal sample_dbt_del_1: std_logic; --Sampling signal of DBT for Bit destuffing
    signal sync_nbt: std_logic; --Beginning of Nominal bit time (transcieving next bit)
    signal sync_dbt: std_logic; --Beginning of Data bit time (transcieving next bit)
    signal sync_nbt_del_1: std_logic; --Bit stuffing trigger NBT
    signal sync_dbt_del_1: std_logic; --Bit Stuffing trigger DBT
    
    signal sample_sec: std_logic; --Secondary sample signal 
    signal sample_sec_del_1: std_logic; --Bit destuffing trigger for secondary sample point
    signal sample_sec_del_2: std_logic; --Rec trig for secondary sample point
    
    signal clk_sys: std_logic;
    signal res_n: std_logic;
    signal drv_bus:std_logic_vector(1023 downto 0);
    signal bit_Error_sec_sam: std_logic; --Bit error with secondary sampling transciever!
    
    ------------------
    --NODE 1 signals--
    ------------------
    signal tran_data_in_nd1: std_logic_vector(511 downto 0);
    signal tran_ident_in_nd1: std_logic_vector(28 downto 0);
    signal tran_dlc_in_nd1: std_logic_vector(3 downto 0);
    signal tran_is_rtr_in_nd1: std_logic;
    signal tran_ident_type_in_nd1: std_logic; --TX Identifier type (0-Basic,1-Extended);
    signal tran_frame_type_in_nd1: std_logic; --TX Frame type (0-CAN Normal, 1-CAN FD)
    signal tran_brs_in_nd1: std_logic; --Frame should be transcieved with BRS value
    signal tran_frame_valid_in_nd1: std_logic; --Signal for CAN Core that frame on the output is valid and can be stored for transmitting
    signal tran_data_ack_out_nd1: std_logic; --Acknowledge from CAN core that acutal message was stored into internal buffer for transmitting   
 
    signal rec_ident_out_nd1: std_logic_vector(28 downto 0); --Message Identifier
    signal rec_data_out_nd1: std_logic_vector(511 downto 0); --Message Data (up to 64 bytes);
    signal rec_dlc_out_nd1: std_logic_vector(3 downto 0); --Data length code
    signal rec_ident_type_out_nd1: std_logic; --Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_frame_type_out_nd1: std_logic; --Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_is_rtr_out_nd1: std_logic; --Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_brs_out_nd1: std_logic; --Frame was recieved with Bit rate shift
    signal rec_message_valid_out_nd1: std_logic; --Output from acceptance filters (out_ident_valid) if message fits the filters
    signal rec_message_ack_out_nd1: std_logic; --Acknowledge for CAN Core about accepted data
    
    signal arbitration_lost_out_nd1: std_logic; --Arbitration was lost input
    signal wake_up_valid_nd1: std_logic; --Wake up appeared
    signal tx_finished_nd1: std_logic; --Message stored in CAN Core was sucessfully transmitted
    signal br_shifted_nd1: std_logic; --Bit Rate Was Shifted
    
    signal error_valid_nd1: std_logic; --At least one error appeared
    signal error_passive_changed_nd1: std_logic; --Error passive state changed
    signal error_warning_limit_nd1: std_logic; --Error warning limit was reached
      
    signal sync_control_nd1: std_logic_vector(1 downto 0); --Synchronisation control signal (Hard sync, Re Sync)
    
    signal data_rx_nd1: std_logic; --Recieved data input (valid with sample_nbt_del_1)
    signal data_tx_nd1: std_logic; --Data transcieved on CAN Bus (valid with sync_nbt_del_1)
    
    signal sp_control_nd1: std_logic_vector(1 downto 0); --Control signal for Sampling source
    signal ssp_reset_nd1: std_logic; --Secondary sample point reset
    signal trv_delay_calib_nd1: std_logic; --Enable calibration of transciever delay compenstation
    signal stat_bus_nd1: std_logic_vector(511 downto 0);
    
    ------------------
    --NODE 2 signals--
    ------------------
    signal tran_data_in_nd2: std_logic_vector(511 downto 0);
    signal tran_ident_in_nd2: std_logic_vector(28 downto 0);
    signal tran_dlc_in_nd2: std_logic_vector(3 downto 0);
    signal tran_is_rtr_in_nd2: std_logic;
    signal tran_ident_type_in_nd2: std_logic; --TX Identifier type (0-Basic,1-Extended);
    signal tran_frame_type_in_nd2: std_logic; --TX Frame type (0-CAN Normal, 1-CAN FD)
    signal tran_brs_in_nd2: std_logic; --Frame should be transcieved with BRS value
    signal tran_frame_valid_in_nd2: std_logic; --Signal for CAN Core that frame on the output is valid and can be stored for transmitting
    signal tran_data_ack_out_nd2: std_logic; --Acknowledge from CAN core that acutal message was stored into internal buffer for transmitting   
 
    signal rec_ident_out_nd2: std_logic_vector(28 downto 0); --Message Identifier
    signal rec_data_out_nd2: std_logic_vector(511 downto 0); --Message Data (up to 64 bytes);
    signal rec_dlc_out_nd2: std_logic_vector(3 downto 0); --Data length code
    signal rec_ident_type_out_nd2: std_logic; --Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_frame_type_out_nd2: std_logic; --Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_is_rtr_out_nd2: std_logic; --Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_brs_out_nd2: std_logic; --Frame was recieved with Bit rate shift
    signal rec_message_valid_out_nd2: std_logic; --Output from acceptance filters (out_ident_valid) if message fits the filters
    signal rec_message_ack_out_nd2: std_logic; --Acknowledge for CAN Core about accepted data
    
    signal arbitration_lost_out_nd2: std_logic; --Arbitration was lost input
    signal wake_up_valid_nd2: std_logic; --Wake up appeared
    signal tx_finished_nd2: std_logic; --Message stored in CAN Core was sucessfully transmitted
    signal br_shifted_nd2: std_logic; --Bit Rate Was Shifted
    
    signal error_valid_nd2: std_logic; --At least one error appeared
    signal error_passive_changed_nd2: std_logic; --Error passive state changed
    signal error_warning_limit_nd2: std_logic; --Error warning limit was reached
      
    signal sync_control_nd2: std_logic_vector(1 downto 0); --Synchronisation control signal (Hard sync, Re Sync)
    
    signal data_rx_nd2: std_logic; --Recieved data input (valid with sample_nbt_del_1)
    signal data_tx_nd2: std_logic; --Data transcieved on CAN Bus (valid with sync_nbt_del_1)
    
    signal sp_control_nd2: std_logic_vector(1 downto 0); --Control signal for Sampling source
    signal ssp_reset_nd2: std_logic; --Secondary sample point reset
    signal trv_delay_calib_nd2: std_logic; --Enable calibration of transciever delay compenstation
    signal stat_bus_nd2: std_logic_vector(511 downto 0);
    
     -------------------
    --TestBench Data --
    -------------------
    --Timing parameters
    constant clk_per:time:=10 ns;
    constant nbt_bit_time:time:=500 ns;
    constant nbt_sample:time:=400 ns;
    constant dbt_bit_time:time:=500 ns;
    constant dbt_sample:time:=400 ns;
    constant test_time:time:=5000 us;
    
    --Data Parameters
    constant max_data_length:natural:=64;
    
    
    --Random tresholds 
    constant rand_data_th:real:=0.5; --Treshold for generating random data for data field! 0.5 is eqal chance for 1 or 0
    constant is_fd_chance:real:=0.5;
    constant is_rtr_chance:real:=0.5; --Note : frame first have to be Normal (non FD) to become RTR. Therefore in case of 0.5, 0.5 the real chance is 0.25 
    constant is_ext_chance:real:=0.5;
    constant brs_chance:real:=0.5;
    
    signal random_pointer: integer:=10;
    
    --TestBench signals
    signal bus_level:std_logic;
    signal DATA_MISMATCH:std_logic;
    signal force_level:std_logic;
    signal force_level_valid:std_logic;
    
    --------------------
    --Availiable Tests--
    --------------------
    constant tran_rec_test:std_logic:='0'; --Node 1 Transciever, Node 2 Reciever, Random data beeing generated sent, ACKed  and verified if recieved the same data
    constant arbit_test_1:std_logic:='0'; --Several arbitration test, always both nodes get identifiers different in few fields (RTR vs normal Frame...). Start of each test is reported!
    constant error_test:std_logic:='1'; --Simple error testbench forcing error value on bus. Veryfing of active error frame and passive error frame
    --Note:TODO implement the exceptions for rule 2 and tesbench it properly! (in real hardware)

    --Node numbers: 1 , 2 supported here
    procedure fill_Random_Data
      (signal frame_type: out std_logic;
       signal ident_type: out std_logic;
       signal dlc: out std_logic_vector(3 downto 0);
       signal is_rtr:out std_logic;
       signal ident:out std_logic_vector(28 downto 0);
       signal data:out std_logic_vector(511 downto 0);
       signal brs:out std_logic) is
      VARIABLE seed1, seed2: positive; -- Seed and state values for random generator
      variable act_length:natural range 0 to 16;
      variable frame_type_var:std_logic;
      variable is_rtr_var:std_logic;
      variable ident_type_var:std_logic;
      VARIABLE int_rand: integer;
      
    begin     
              --Generating Frame  type
              if(randData(random_pointer)>is_fd_chance)then frame_type_var:=FD_CAN; else frame_type_var:=NORMAL_CAN; end if;--Generating FD or non FD Frame
              frame_type<=frame_type_var;
              
              --Generating Frame format (Identifier type)
              if(randData(random_pointer+1)>is_ext_chance)then ident_type_var:=BASE; else ident_type_var:=EXTENDED; end if;
              ident_type<=ident_type_var;
              
              --Generating RTR frame
              if(frame_type_var=BASE)then --only CAN Frames have RTR Frames
                if(randData(random_pointer+2)>is_rtr_chance)then is_rtr_var:='0'; else is_rtr_var:='1'; end if;
              else
                is_rtr_var:='0';
              end if;
              is_rtr<=is_rtr_var;
            
              --Filling Random Identifier
              ident<=(OTHERS=>'0');
              if(ident_type_var=BASE)then
                int_rand := INTEGER(TRUNC(randData(random_pointer+3)*2048.0));
              else
                int_rand := INTEGER(TRUNC(randData(random_pointer+3)*536870912.0));                  
              end if;
              ident<= std_logic_vector(to_unsigned(int_rand,29));
            
              --Filling with random Data
               data<=(Others =>'0'); --Nulling the current data
              if(is_rtr_var='0')then
                if(frame_type_var=FD_CAN)then
                  int_rand := INTEGER(TRUNC(16.0*randData(random_pointer+4)));
                  act_length:=natural(int_rand); --Getting random number scaled 0 to 16
                else
                  int_rand := INTEGER(TRUNC(8.0*randData(random_pointer+4)));
                  act_length:=natural(int_rand); --Getting random number scaled 0 to 16
                end if;  
              end if;
                
                dlc<=dlc_codes(act_length); --Assigning DLC Code
              
                for I in 0 to dlc_length(act_length)*8-1 loop
                  if(randData(random_pointer+4+I)>0.5)then
                    data(I)<='1';
                  else
                    data(I)<='0';
                  end if; 
                end loop;
              
              --Generating random Bit Rate shift
              if(frame_type_var=FD_CAN and randData(random_pointer+5)>brs_chance)then brs<='0'; else  brs<='0'; end if;       
    
    end procedure;
    
begin
  node1:core_top
  port map(
      clk_sys=>clk_sys,
      res_n=>res_n,
      drv_bus=>drv_bus,
      stat_bus=>stat_bus_nd1,
    
      sample_nbt_del_2=>sample_nbt_del_2,
      sample_dbt_del_2=>sample_dbt_del_2,
      sample_nbt_del_1=>sample_nbt_del_1,
      sample_dbt_del_1=>sample_dbt_del_1,
      sync_nbt=>sync_nbt,
      sync_dbt=>sync_dbt,
      sync_nbt_del_1=>sync_nbt_del_1,
      sync_dbt_del_1=>sync_dbt_del_1,
    
      sample_sec=>sample_sec,
      sample_sec_del_1=>sample_sec_del_1,
      sample_sec_del_2=>sample_sec_del_2,
    
      tran_data_in=>tran_data_in_nd1,
      tran_ident_in=>tran_ident_in_nd1,
      tran_dlc_in=>tran_dlc_in_nd1,
      tran_is_rtr_in=>tran_is_rtr_in_nd1,
      tran_ident_type_in=>tran_ident_type_in_nd1,
      tran_frame_type_in=>tran_frame_type_in_nd1,
      tran_brs_in=>tran_brs_in_nd1,
      tran_frame_valid_in=>tran_frame_valid_in_nd1,
      tran_data_ack_out=>tran_data_ack_out_nd1,
 
      rec_ident_out=>rec_ident_out_nd1,
      rec_data_out=>rec_data_out_nd1,
      rec_dlc_out=>rec_dlc_out_nd1,
      rec_ident_type_out=>rec_ident_type_out_nd1,
      rec_frame_type_out=>rec_frame_type_out_nd1,
      rec_is_rtr_out=>rec_is_rtr_out_nd1,
      rec_brs_out=>rec_brs_out_nd1,
      rec_message_valid_out=>rec_message_valid_out_nd1,
      rec_message_ack_out=>rec_message_ack_out_nd1,
    
      arbitration_lost_out=>arbitration_lost_out_nd1,
      wake_up_valid=>wake_up_valid_nd1,
      tx_finished=>tx_finished_nd1,
      br_shifted=>br_shifted_nd1,
    
      error_valid=>error_valid_nd1,
      error_passive_changed=>error_passive_changed_nd1,
      error_warning_limit=>error_warning_limit_nd1,
          
      sync_control=>sync_control_nd1,
    
      data_rx=>data_rx_nd1,
      data_tx=>data_tx_nd1,
    
      sp_control=>sp_control_nd1,
      ssp_reset=>ssp_reset_nd1,
      trv_delay_calib=>trv_delay_calib_nd1,
      bit_Error_sec_sam=>bit_Error_sec_sam,
      timestamp => (OTHERS => '0'),
      hard_sync_edge => '0'  --In this legacy testbench hard synchronisation is not tested
   );
   
  node2:core_top
  port map(
      clk_sys=>clk_sys,
      res_n=>res_n,
      drv_bus=>drv_bus,
      stat_bus=>stat_bus_nd2,
    
      sample_nbt_del_2=>sample_nbt_del_2,
      sample_dbt_del_2=>sample_dbt_del_2,
      sample_nbt_del_1=>sample_nbt_del_1,
      sample_dbt_del_1=>sample_dbt_del_1,
      sync_nbt=>sync_nbt,
      sync_dbt=>sync_dbt,
      sync_nbt_del_1=>sync_nbt_del_1,
      sync_dbt_del_1=>sync_dbt_del_1,
    
      sample_sec=>sample_sec,
      sample_sec_del_1=>sample_sec_del_1,
      sample_sec_del_2=>sample_sec_del_2,
    
      tran_data_in=>tran_data_in_nd2,
      tran_ident_in=>tran_ident_in_nd2,
      tran_dlc_in=>tran_dlc_in_nd2,
      tran_is_rtr_in=>tran_is_rtr_in_nd2,
      tran_ident_type_in=>tran_ident_type_in_nd2,
      tran_frame_type_in=>tran_frame_type_in_nd2,
      tran_brs_in=>tran_brs_in_nd2,
      tran_frame_valid_in=>tran_frame_valid_in_nd2,
      tran_data_ack_out=>tran_data_ack_out_nd2,
 
      rec_ident_out=>rec_ident_out_nd2,
      rec_data_out=>rec_data_out_nd2,
      rec_dlc_out=>rec_dlc_out_nd2,
      rec_ident_type_out=>rec_ident_type_out_nd2,
      rec_frame_type_out=>rec_frame_type_out_nd2,
      rec_is_rtr_out=>rec_is_rtr_out_nd2,
      rec_brs_out=>rec_brs_out_nd2,
      rec_message_valid_out=>rec_message_valid_out_nd2,
      rec_message_ack_out=>rec_message_ack_out_nd2,
    
      arbitration_lost_out=>arbitration_lost_out_nd2,
      wake_up_valid=>wake_up_valid_nd2,
      tx_finished=>tx_finished_nd2,
      br_shifted=>br_shifted_nd2,
    
      error_valid=>error_valid_nd2,
      error_passive_changed=>error_passive_changed_nd2,
      error_warning_limit=>error_warning_limit_nd2,
          
      sync_control=>sync_control_nd2,
    
      data_rx=>data_rx_nd2,
      data_tx=>data_tx_nd2,
    
      sp_control=>sp_control_nd2,
      ssp_reset=>ssp_reset_nd2,
      trv_delay_calib=>trv_delay_calib_nd2,
      bit_Error_sec_sam=>bit_Error_sec_sam,
      timestamp => (OTHERS => '0'),
      hard_sync_edge => '0'  --In this legacy testbench hard synchronisation is not tested
   ); 
  
  clock_gen:process
  begin
    clk_sys<='1';
    wait for clk_per/2;
    clk_sys<='0';
    wait for clk_per/2;
  end process; 
  
  nbt_gen:process
  begin
   
   sync_nbt<='0';
   sync_nbt_del_1<='0';
   sample_nbt_del_2<='0';
   sample_nbt_del_1<='0';    
   
   wait for 100 ns;
   
   while(2>1) loop
    sync_nbt<='1';
    wait for clk_per;
    sync_nbt<='0';
    sync_nbt_del_1<='1';
    wait for clk_per;
    sync_nbt_del_1<='0';
    wait for nbt_sample;
    sample_nbt_del_1<='1';
    wait for clk_per;
    sample_nbt_del_1<='0';
    sample_nbt_del_2<='1';
    wait for clk_per;
    sample_nbt_del_2<='0';
    wait for nbt_bit_time-nbt_sample-4*clk_per;
    end loop;
    
  end process;
 
  dbt_gen:process
  begin
    sync_dbt<='0';
    sync_dbt_del_1<='0';
    sample_dbt_del_2<='0';
    sample_dbt_del_1<='0';
    sample_sec_del_1<='0';
    sample_sec_del_2<='0';
    sample_sec<='0';
   wait for 100 ns;
   
   while(2>1) loop
    sync_dbt<='1';
    wait for clk_per;
    sync_dbt<='0';
    sync_dbt_del_1<='1';
    wait for clk_per;
    sync_dbt_del_1<='0';
    wait for dbt_sample;
    sample_dbt_del_1<='1';
    sample_sec_del_1<='1';
    wait for clk_per;
    sample_dbt_del_1<='0';
    sample_sec_del_1<='0';
    sample_dbt_del_2<='1';
    sample_sec_del_2<='1';
    wait for clk_per;
    sample_dbt_del_2<='0';
    sample_sec_del_2<='0';
    wait for dbt_bit_time-dbt_sample-2*clk_per;
   end loop;
  end process;
  --Note: So far secondary sample point is the same as normal data phase sample point
  
  --Restart
  rest_process:process
  begin
   
   wait for test_time;
  end process;
 ---------------------------------------------------------------
 --Virtual bus creation--RECESSIVE AND DOMINANT Representation--
 ---------------------------------------------------------------
 bus_level<=force_level when force_level_valid='1' else
            '0'         when (data_tx_nd1='0' or data_tx_nd2='0') else 
            '1';
 data_rx_nd1<=bus_level;
 data_rx_nd2<=bus_level; 
 
 tests_proc:process
 variable rnd:integer;
 begin
    res_n<=ACT_RESET;
    fill_Random_Data(tran_frame_type_in_nd1,tran_ident_type_in_nd1,tran_dlc_in_nd1,
                    tran_is_rtr_in_nd1,tran_ident_in_nd1,tran_data_in_nd1,tran_brs_in_nd1);
    fill_Random_Data(tran_frame_type_in_nd2,tran_ident_type_in_nd2,tran_dlc_in_nd2,
                    tran_is_rtr_in_nd2,tran_ident_in_nd2,tran_data_in_nd2,tran_brs_in_nd2);
    tran_frame_valid_in_nd1<='0';
    tran_frame_valid_in_nd2<='0';
    drv_bus<=(OTHERS =>'0');
    bit_Error_sec_sam<='0';
    drv_bus(DRV_CAN_FD_ENA_INDEX)<='1'; --Enable accepting FD Frames
    
    drv_bus(DRV_EWL_HIGH downto DRV_EWL_LOW)<="00100000";
    drv_bus(DRV_ERP_HIGH downto DRV_ERP_LOW)<="01100000";
    drv_bus(DRV_CAN_FD_ENA_INDEX)<='1';
    drv_bus(DRV_RTR_PREF_INDEX)<='0';
    drv_bus(DRV_RETR_LIM_ENA_INDEX)<='0';
    drv_bus(DRV_RETR_TH_LOW downto DRV_RETR_TH_LOW)<=(OTHERS =>'0');
  
    rec_message_ack_out_nd2<='0';
    rec_message_ack_out_nd1<='0';
        
    wait for 17 ns;
    res_n<=not ACT_RESET;
   
    ----------------------------------
    --Node 1 random data generation:--
    ----------------------------------
 
    --Simple test with both nodes connected on virtual BUS. Node 1 sending random messages, node 2 recieving messages and sending acknowledge.
    --CRC calculation is debugged.
    --Note BIT Rate shifting is not working properly. Since sample points are not at the same point when switched then Bugs appears.
    --Two types of bugs are appearing: 
    --     1. When shifting to faster and bug appear then form error is detected and sending node starts Error frame
    --     2. When Shifting to slower, bits in CRC_DELIM, ACK Field are missinterpreted and false data can be stored in Node 2.
    --Both problems will be solved by proper synchronisation of triggering signals between switching by prescaler!!
    --Note: Both nodes use the same clock source for simplicity
   
   force_level_valid<='0';
   force_level<='0';
   
   if(tran_rec_test='1')then --Assign 1 into constant in the condition to run the test
   DATA_MISMATCH<='0';
   wait for 17 ns;
   
   while 2>1 loop
      
      --Fill the random data to send, Node 1
      fill_Random_Data(tran_frame_type_in_nd1,tran_ident_type_in_nd1,tran_dlc_in_nd1,
                    tran_is_rtr_in_nd1,tran_ident_in_nd1,tran_data_in_nd1,tran_brs_in_nd1);
      random_pointer<=random_pointer+44;
      
      --Restarting the random data values 
      if(random_pointer>3200)then random_pointer<=0; end if;
      tran_frame_valid_in_nd1<='1';    
      wait for 10 us;
      
      --Wait until message is accepted by node 2
      wait until rec_message_valid_out_nd2='1';
      
      --Compare the data
      if(tran_frame_type_in_nd1=rec_frame_type_out_nd2 and
         tran_ident_type_in_nd1=rec_ident_type_out_nd2 and
         tran_data_in_nd1=rec_data_out_nd2 and
         tran_ident_in_nd1=rec_ident_out_nd2 and
         tran_is_rtr_in_nd1=rec_is_rtr_out_nd2 and
         tran_brs_in_nd1=rec_brs_out_nd2)then
        
        --Log sucessfull frame 
         assert false report "MESSAGE RECIEVED CORRECTLY!!!" severity note;
         if(tran_frame_type_in_nd1='1') then assert false report "FRAME TYPE: FD"  severity note;
                                       else assert false report "FRAME TYPE: NORMAL" severity note; end if;
        
        if(tran_ident_type_in_nd1='1') then assert false report "IDENTIFIER TYPE: BASIC"  severity note;
                                       else assert false report "IDENTIFIER TYPE: EXTENDED" severity note; end if;
        
        if(tran_is_rtr_in_nd1='1') then assert false report "RTR FRAME: YES"  severity note;
                               else assert false report "RTR FRAME: NO" severity note; end if;
        
        if(tran_brs_in_nd1='1') then assert false report "BIT RATE SHIFTED: YES"  severity note;
                               else assert false report "BIT RATE SHIFTED: NO" severity note; end if;                              
        
        DATA_MISMATCH<='0'; 
      else   
        assert false report "Data mismatch appeared" severity note;
        DATA_MISMATCH<='1'; 
      end if;
    end loop; 
   end if;
   
   ----------------------------------------------------------------------------
   --Direct arbitration test, testing various extreme arbitration conditions --
   ----------------------------------------------------------------------------
   
   if(arbit_test_1='1')then
      --Note: In this test the Identifiers are differed in first bit of identifier
      random_pointer<=0;
      wait for 17 ns;
      
       --Filling data for node 1
       fill_Random_Data(tran_frame_type_in_nd1,tran_ident_type_in_nd1,tran_dlc_in_nd1,
                    tran_is_rtr_in_nd1,tran_ident_in_nd1,tran_data_in_nd1,tran_brs_in_nd1);
       random_pointer<=random_pointer+19; --Note: 19 has no particular sense...
       wait for 1 ns;
        
       --Filling the data for node 2
       fill_Random_Data(tran_frame_type_in_nd2,tran_ident_type_in_nd2,tran_dlc_in_nd2,
                    tran_is_rtr_in_nd2,tran_ident_in_nd2,tran_data_in_nd2,tran_brs_in_nd2);
       random_pointer<=random_pointer+19; --Note: 19 has no particular sense...
     
       --Commanding both nodes to transmitt
       tran_frame_valid_in_nd1<='1';
       tran_frame_valid_in_nd2<='1';
       assert false report "Test with Identifiers differed in first bit starts!" severity note;
        
       wait for 10 us;
       tran_frame_valid_in_nd1<='0';
       tran_frame_valid_in_nd2<='0';
       wait for 400 us;
       
       -------------------------------------------------------
       --Identifiers are differed in last bit of identifier---
       -------------------------------------------------------
        tran_frame_type_in_nd1<='0';
        tran_ident_type_in_nd1<='0';
        tran_is_rtr_in_nd1<='0'; --Node 1 is RTR Frame
        tran_data_in_nd1<=(OTHERS =>'0');
        tran_dlc_in_nd1<="0001";
        tran_ident_in_nd1<="00000000000000000000000000001";
        tran_brs_in_nd1<='1';
        tran_frame_valid_in_nd1<='1';
       
        tran_frame_type_in_nd2<='0';
        tran_ident_type_in_nd2<='0';
        tran_is_rtr_in_nd2<='0'; --Node 1 is RTR Frame
        tran_data_in_nd2<=(OTHERS =>'0');
        tran_dlc_in_nd2<="0001";
        tran_ident_in_nd2<="00000000000000000000000000000";
        tran_brs_in_nd2<='1';
        tran_frame_valid_in_nd2<='1';        
        assert false report "Test with Identifiers differed in last bit starts!" severity note;
        wait for 5 us;
        
        tran_frame_valid_in_nd1<='0';
        tran_frame_valid_in_nd2<='0';
        wait for 400 us;
        
        -------------------------------------------------------------------------------
        --RTR Frame vs data frame with the same identifier testing (Basic identifier)--
        -------------------------------------------------------------------------------
        tran_frame_type_in_nd1<='0'; --Basic CAN, Note if RTR Frame of type FD is sent the RTR Flag is ignored!
        tran_ident_type_in_nd1<='0';
        tran_is_rtr_in_nd1<='1';
        tran_data_in_nd1<=(OTHERS =>'0');
        tran_dlc_in_nd1<="0000";
        tran_ident_in_nd1<="00000000000000000000000001101";
        tran_brs_in_nd1<='0'; --Doesnt matter now. In CAN Frame bit rate cant be shifted!
        
        tran_frame_type_in_nd2<='0'; --Basic CAN, Note if RTR Frame of type FD is sent the RTR Flag is ignored!
        tran_ident_type_in_nd2<='0';
        tran_is_rtr_in_nd2<='0';
        tran_data_in_nd2<=(OTHERS =>'0');
        tran_dlc_in_nd2<="0001";
        tran_ident_in_nd2<="00000000000000000000000001101";
        tran_brs_in_nd2<='0'; --Doesnt matter now. In CAN Frame bit rate cant be shifted!
        
        tran_frame_valid_in_nd1<='1';
        tran_frame_valid_in_nd2<='1';
        assert false report "RTR CAN frame (NODE 1) vs Data CAN (NODE 2) frame with identical identifier test started" severity note;
        wait for 5 us;
        
        tran_frame_valid_in_nd1<='0';
        tran_frame_valid_in_nd2<='0';
        wait for 400 us;
        
        ------------------------------------------------------
        --Basic identifier vs Extended Identifier Basic CAN --
        ------------------------------------------------------
        tran_frame_type_in_nd1<='0'; --Basic CAN, Note if RTR Frame of type FD is sent the RTR Flag is ignored!
        tran_ident_type_in_nd1<='1';
        tran_is_rtr_in_nd1<='0';
        tran_data_in_nd1<=(OTHERS =>'0');
        tran_dlc_in_nd1<="0001";
        tran_ident_in_nd1<="00000011000000111001100001101";
        tran_brs_in_nd1<='0'; --Doesnt matter now. In CAN Frame bit rate cant be shifted!
        
        tran_frame_type_in_nd2<='0'; --Basic CAN, Note if RTR Frame of type FD is sent the RTR Flag is ignored!
        tran_ident_type_in_nd2<='0';
        tran_is_rtr_in_nd2<='0';
        tran_data_in_nd2<=(OTHERS =>'0');
        tran_dlc_in_nd2<="0001";
        tran_ident_in_nd2<="00000000000000000001100001101";
        tran_brs_in_nd2<='0'; --Doesnt matter now. In CAN Frame bit rate cant be shifted!
        
        tran_frame_valid_in_nd1<='1';
        tran_frame_valid_in_nd2<='1';
        assert false report "Extended CAN Identifier (Node 1) vs BASIC CAN Identifier (Node 2) with same first 11 bits test starts here!" severity note;
        wait for 5 us;
        
        tran_frame_valid_in_nd1<='0';
        tran_frame_valid_in_nd2<='0';
        wait for 400 us;
        
        --------------------------------------------------------------------
        --Confilct in data. The same identifiers with different data sent --
        --------------------------------------------------------------------
        --Note: Error detection and error frame sending not working properly yet 
        tran_frame_type_in_nd1<='0'; --Basic CAN, Note if RTR Frame of type FD is sent the RTR Flag is ignored!
        tran_ident_type_in_nd1<='0';
        tran_is_rtr_in_nd1<='0';
        tran_data_in_nd1<=(OTHERS =>'0');
        tran_data_in_nd1(6)<='1';
        tran_data_in_nd1(5)<='1';
        tran_dlc_in_nd1<="0001";
        tran_ident_in_nd1<="00000000000000000001100001101";
        tran_brs_in_nd1<='0'; --Doesnt matter now. In CAN Frame bit rate cant be shifted!
        
        tran_frame_type_in_nd2<='0'; --Basic CAN, Note if RTR Frame of type FD is sent the RTR Flag is ignored!
        tran_ident_type_in_nd2<='0';
        tran_is_rtr_in_nd2<='0';
        tran_data_in_nd2<=(OTHERS =>'0');
        tran_dlc_in_nd2<="0001";
        tran_ident_in_nd2<="00000000000000000001100001101";
        tran_brs_in_nd2<='0'; --Doesnt matter now. In CAN Frame bit rate cant be shifted!
        
        tran_frame_valid_in_nd1<='1';
        tran_frame_valid_in_nd2<='1';
        assert false report "Same identifiers different data test started here! Bus conflict simulation" severity note;
        wait for 5 us;
        
        tran_frame_valid_in_nd1<='0';
        tran_frame_valid_in_nd2<='0';
        wait for 400 us;     
   end if;
   
   --------------------------------------
   --Error detection and handling test --
   --------------------------------------
   
   if(error_test='1')then
    
       DATA_MISMATCH<='0';
        wait for 17 ns;
   
        while 2>1 loop
      
            --Fill the random data to send, Node 1
            fill_Random_Data(tran_frame_type_in_nd1,tran_ident_type_in_nd1,tran_dlc_in_nd1,
                    tran_is_rtr_in_nd1,tran_ident_in_nd1,tran_data_in_nd1,tran_brs_in_nd1);
            fill_Random_Data(tran_frame_type_in_nd2,tran_ident_type_in_nd2,tran_dlc_in_nd2,
                    tran_is_rtr_in_nd2,tran_ident_in_nd2,tran_data_in_nd2,tran_brs_in_nd2);
            tran_frame_valid_in_nd2<='0';
            random_pointer<=random_pointer+44;
      
            --Restarting the random data values 
            if(random_pointer>3200)then random_pointer<=0; end if;
            tran_frame_valid_in_nd1<='1';    
            wait for 10 us;
      
            --Testing error at SOF
            wait until (protocol_type'VAL(to_integer(unsigned(stat_bus_nd1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))=sof);
            
            assert false report "SOF error testing" severity note;
            force_level<='1';
            force_level_valid<='1';
            wait for nbt_bit_time;
            force_level_valid<='0';
             
            --Testing Bit error in Arbitration field (one transmitter loses the arbitration due to error->both nodes recieves -> 
            --   noone sending data -> stuff error after 6 same bits) 
            wait until (protocol_type'VAL(to_integer(unsigned(stat_bus_nd1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))=arbitration);
            wait for 3*nbt_bit_time;
            assert false report "Arbitration bit error testing (Losing arbitration due to error)" severity note;
            force_level<=not bus_level;
            force_level_valid<='1';
            wait for nbt_bit_time;
            force_level_valid<='0';
            
             --Testing the FD Error counters!
            drv_bus(DRV_ACK_FORB_INDEX)<='0';
            wait until (protocol_type'VAL(to_integer(unsigned(stat_bus_nd1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))=interframe);
            tran_frame_type_in_nd1<='1';
            tran_brs_in_nd1<='1';
            tran_ident_type_in_nd1<='1';
            tran_frame_valid_in_nd1<='1';
            wait until (protocol_type'VAL(to_integer(unsigned(stat_bus_nd1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))=data);
            assert false report "Testing error in data bit rate !" severity note;
            force_level<=RECESSIVE;
            force_level_valid<='1';
            wait for 4*nbt_bit_time; 
            force_level_valid<='0';
            
            wait for 100 ns;
            
            
            --Testing Bit error in Arbitration field (one transmitter loses the arbitration due to error->both nodes recieves -> 
            --   noone sending data -> stuff error after 6 same bits) 
            wait until (protocol_type'VAL(to_integer(unsigned(stat_bus_nd1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))=arbitration) and (bus_level=DOMINANT);
            wait for 7*nbt_bit_time;
            assert false report "Arbitration bit error testing (Sending dominant getting recesive)" severity note;
            force_level<=RECESSIVE;
            force_level_valid<='1';
            wait for 2*nbt_bit_time;
            force_level_valid<='0';
            
            --Testing Bit error in Control field (one transmitter loses the arbitration due to error->both nodes recieves -> 
            --   noone sending data -> stuff error after 6 same bits) 
            wait until (protocol_type'VAL(to_integer(unsigned(stat_bus_nd1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))=arbitration) and (bus_level=DOMINANT);
            wait for 7*nbt_bit_time;
            assert false report "Arbitration bit error testing (Sending dominant getting recesive)" severity note;
            force_level<=RECESSIVE;
            force_level_valid<='1';
            wait for nbt_bit_time; --TODO: discuss situation when 2*nbt_bit_time is waited! 
            force_level_valid<='0';
            
            wait for 100 us;
            
            ---Presetting the error passive state
            drv_bus(DRV_CTR_VAL_HIGH downto DRV_CTR_VAL_LOW)<="011100000";
            drv_bus(DRV_CTR_SEL_HIGH downto DRV_CTR_SEL_LOW)<="0001";
            wait for 2*clk_per;
            drv_bus(DRV_CTR_SEL_HIGH downto DRV_CTR_SEL_LOW)<="0000";
            assert false report "Assigning error passive node!" severity note;
            
            wait until (protocol_type'VAL(to_integer(unsigned(stat_bus_nd1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))=data) and (bus_level=DOMINANT);
            assert false report "Testing passive error flag!" severity note;
            force_level<=RECESSIVE;
            force_level_valid<='1';
            wait for nbt_bit_time; 
            force_level_valid<='0';
            
            --Setting to forbidding acknowledge mode
            drv_bus(DRV_ACK_FORB_INDEX)<='1';
            
            wait for 10 us;    
            
           
            
                  
        end loop; 
     
   end if;

   
 end process tests_proc;

end architecture;