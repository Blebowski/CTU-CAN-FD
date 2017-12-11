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
-- Revision History:
--
--    1.6.2016   Created file
--   22.6.2016   Updated testbench to cover also the modified functionality of RX Buffer. Now ESI bit is also stored
--                and compared. Also RTR frame of CAN normal frame does not store any data words into the buffer.
--               
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Unit test for the RX Buffer circuit.
--
--  Following test instantiates RX Buffer. It stimuli generator generates input frames as CAN_Core would do.
--  Then it checks whether frame was stored into the buffer! Another process reads the data as user would
--  do by memory access. Both, data written into the buffer, and data read from the buffer are stored into 
--  test memories (in_mem,out_mem). When test memory is full content of both memories is compared! When mismatch
--  occurs test fails. Each time memory is filled test moves to the next iteration.
--
--  Additionally Rx_buffer_model is implemented in separate file. This model is not yet debugged and used. It
--  is commented in following code. The purpose of the model is to provide behavioural description of the RX
--  buffer! Additional process, status_controller (now stalled) checks at operation time difference between
--  RX_Buffer and its model! It it intended that data read from the model are stored into third memory and mismatch
--  check will be done between all three memories. However this is not yet used in this unit test!                                      
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Test implementation                                           
-----------------------------------------------------------------------------------------------------------------

architecture rx_buf_unit_test of CAN_test is
  
    signal clk_sys              :   std_logic:='0';                       --System clock
    signal res_n                :   std_logic:='0';                         --Async. reset
    signal rec_ident_in         :   std_logic_vector(28 downto 0);    --Message Identifier
    signal rec_dlc_in           :   std_logic_vector(3 downto 0);     --Data length code
    signal rec_ident_type_in    :   std_logic:='0';                                      --Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_frame_type_in    :   std_logic:='0';                                      --Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_is_rtr           :   std_logic:='0';                                      --Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_brs              :   std_logic:='0';                                      --Whenever frame was recieved with BIT Rate shift 
    signal rec_esi              :   std_logic:='0';                                      --Recieved error state indicator
    signal rec_message_valid    :   std_logic:='0';                                      --Output from acceptance filters (out_ident_valid) if message fits the filters   
    signal timestamp            :   std_logic_vector(63 downto 0):=(OTHERS =>'0');
    signal drv_bus              :   std_logic_vector(1023 downto 0):=(OTHERS =>'0');   --Driving bus from registers 

    signal rec_dram_word        :   std_logic_vector(31 downto 0);
    signal rec_dram_addr        :   natural range 0 to 15;

    signal rec_message_ack_b      :   std_logic:='0';                          --Acknowledge for CAN Core about accepted data    
    signal rx_buf_size_b          :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Actual size of synthetised message buffer (in 32 bit words)
    signal rx_full_b              :   std_logic:='0';                          --Signal whenever buffer is full
    signal rx_empty_b             :   std_logic:='0';                          --Signal whenever buffer is empty
    signal rx_message_count_b     :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Number of messaged stored in recieve buffer
    signal rx_mem_free_b          :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Number of free 32 bit wide ''windows''
    signal rx_read_pointer_pos_b  :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Position of read pointer
    signal rx_write_pointer_pos_b :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Position of write pointer
    signal rx_message_disc_b      :   std_logic:='0';                          --Message was discarded since Memory is full
    signal rx_data_overrun_b      :   std_logic:='0';                          --Some data were discarded, register    
    signal rx_read_buff_b         :   std_logic_vector(31 downto 0):=(OTHERS =>'0');    --Actually loaded data for reading
    
    signal rec_message_ack_m      :   std_logic:='0';                          --Acknowledge for CAN Core about accepted data    
    signal rx_buf_size_m          :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Actual size of synthetised message buffer (in 32 bit words)
    signal rx_full_m              :   std_logic:='0';                          --Signal whenever buffer is full
    signal rx_empty_m             :   std_logic:='0';                          --Signal whenever buffer is empty
    signal rx_message_count_m     :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Number of messaged stored in recieve buffer
    signal rx_mem_free_m          :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Number of free 32 bit wide ''windows''
    signal rx_read_pointer_pos_m  :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Position of read pointer
    signal rx_write_pointer_pos_m :   std_logic_vector(7 downto 0):=(OTHERS =>'0');     --Position of write pointer
    signal rx_message_disc_m      :   std_logic:='0';                          --Message was discarded since Memory is full
    signal rx_data_overrun_m      :   std_logic:='0';                          --Some data were discarded, register    
    signal rx_read_buff_m         :   std_logic_vector(31 downto 0):=(OTHERS =>'0');    --Actually loaded data for reading
  

  
  --------------------------------------------------------------------------
  --Test specific signals
  --------------------------------------------------------------------------
  signal input_frame              :   CAN_frame_type := ((OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),'0','0','0','0','0','0');   
  signal iteration_done           :   boolean := false;
  signal in_mem_full              :   boolean := false;
  signal out_mem_full             :   boolean := false;
  signal mod_mem_full             :   boolean := false;
  signal aux_read                 :   std_logic:='0';
  
  --error counters
  signal stim_errs                :   natural:=0;
  signal read_errs                :   natural:=0;
  signal status_errs              :   natural:=0; 
  signal cons_errs                :   natural:=0;  
  
  --Dummy signals
  signal exit_imm_d               : boolean :=false;
  signal exit_imm_d_2             : boolean :=false;
  signal exit_imm_d_3             : boolean :=false;
  
  --------------------------------------------------------------------------
  --Memory declarations for memories where data are read out
  --------------------------------------------------------------------------
  type eval_mem_test is array (0 to 1023) of std_logic_vector(31 downto 0);
  signal in_mem                   : eval_mem_test := (OTHERS => (OTHERS => '0'));  --Frames stored into the buffer
  signal out_mem                  : eval_mem_test := (OTHERS => (OTHERS => '0'));  --Frames read from the buffer
  signal mod_mem                  : eval_mem_test := (OTHERS => (OTHERS => '0'));  --Frames read from the model
  signal in_pointer               : natural:=0;
  signal out_pointer              : natural:=0;
  signal mod_pointer              : natural:=0;
  
  constant buff_size              : natural:=32;
  
  -----------------------------------------------------
  -- Generates random frame on input of the buffers
  -----------------------------------------------------
  procedure generate_frame
  (signal   rand_ctr  :inout  natural range 0 to RAND_POOL_SIZE;
   signal   frame     :inout  CAN_frame_type;
   variable size      :out    natural
  )is
  variable int_size :natural;
  begin
   rand_logic(rand_ctr,frame.rec_ident_type_in  ,0.5);
   rand_logic(rand_ctr,frame.rec_frame_type_in  ,0.5);
   rand_logic(rand_ctr,frame.rec_is_rtr         ,0.5);
   rand_logic(rand_ctr,frame.rec_brs            ,0.5);
   rand_logic(rand_ctr,frame.rec_esi            ,0.5);
   
   rand_logic_vect(rand_ctr,frame.rec_dlc_in     ,0.5);
   rand_logic_vect(rand_ctr,frame.rec_data_in    ,0.5);
   rand_logic_vect(rand_ctr,frame.rec_ident_in   ,0.5);
   wait for 0 ns;
   
   --Note: Here we test only the buffer! So we can generate also invalid combinations of
   --      frame formats and types. E.g FD RTR frame...
   
   decode_dlc_buff(frame.rec_dlc_in,int_size);
   
   if(frame.rec_is_rtr='1' and frame.rec_frame_type_in='0')then
     int_size:=4;
   end if;
   
   size := int_size;
   wait for 0 ns;
   
  end procedure;
  
  -----------------------------------------------------
  -- Insert frame into the RX buffer and its model
  -- and also common memory
  -----------------------------------------------------
  procedure insert_frame
  (signal   frame           :inout  CAN_frame_type;  
   signal   timestamp       :in     std_logic_vector(63 downto 0);
   variable insert_cmn      :in     boolean;         --Whether frame should be inserted also into common memory
   variable was_inserted    :inout  boolean;
   signal   data_overrun_b  :in     std_logic;
   signal   data_overrun_m  :in     std_logic;
   signal   memory          :inout  eval_mem_test;
   signal   in_pointer      :inout  natural;
   signal   log_level       :in     log_lvl_type
   )
  is
  variable int_length       : natural;
  begin
    --First we say that frame is valid to the buffer and its model
    frame.rec_message_valid <= '1';
    wait for 10 ns; --Wait for one clock cycle to trigger the storing
    frame.rec_message_valid <= '0';
    wait for 0 ns;
    
    --Here we ask if data overrun did not appear
    --if(data_overrun_b='1' or data_overrun_m='1')then
    if(data_overrun_b='1')then
      was_inserted:=false;
    else  
      was_inserted:=true; 
   end if;
    
    -----------------------------------------------------------------------
    --When frame was truly inserted then add it to the common input memory
    -----------------------------------------------------------------------
    if was_inserted then
      memory(in_pointer)   <= "000000000000000000000"&frame.rec_esi&frame.rec_brs&'1'&frame.rec_frame_type_in&
                                frame.rec_ident_type_in&frame.rec_is_rtr&'0'&frame.rec_dlc_in;
      memory(in_pointer+1) <= timestamp(63 downto 32);
      
      --Note that here we have to store timestamp increased by two, because timestamp is
      --in this test increasing by one every clock cycle!! thus when timestamp is acutally
      --stored into RX buffer it is two clock cycles later!!!
      memory(in_pointer+2) <= std_logic_vector(unsigned(timestamp(31 downto 0))+2);
      memory(in_pointer+3) <= "000"&frame.rec_ident_in;
      in_pointer <= in_pointer+4;
      
      wait for 0 ns;
      
      decode_dlc(frame.rec_dlc_in, int_length);
      if(frame.rec_is_rtr='1' and frame.rec_frame_type_in='0')then
        int_length:=0;
      end if;
      
      --Store the data
      if(int_length>0)then
        for i in 0 to (int_length-1)/4 loop
          memory(in_pointer)   <= frame.rec_data_in(511-i*32 downto 480-i*32);
          in_pointer           <= in_pointer +1;
          wait for 0 ns;
        end loop;
      end if; 
         
      --At the end we need to move one more time
      --in_pointer              <= in_pointer+1;
      wait for 0 ns;   
            
   end if;
    
    --Here we check whether frame was not discarded by either buffer
      if (was_inserted=false and insert_cmn=true) or
         (was_inserted=true  and insert_cmn=false)
      then
        log("Incorrect discard value from buffers returned!",error_l,log_level);
      end if; 
    
    
    --Test waits long enough to store maximum frame size (20 words = 20 clock cycles)
    --Note that when can frame is accepted then eof field and interframe space are
    --always longer that 20 cycles!! So this is reasonable asumption
    wait for 20*10 ns;
    
  end procedure;
  
  ------------------------------------------------------------------------------------
  -- Read frame from the RX buffer and its model
  -- and stores it into the common model and output memory!
  ------------------------------------------------------------------------------------
  procedure read_frame
  (signal buff_out        :in    std_logic_vector(31 downto 0);
   signal mod_out         :in    std_logic_vector(31 downto 0);
   signal out_mem         :out   eval_mem_test;
   signal mod_mem         :out   eval_mem_test;
   signal out_read        :inout std_logic;
   signal out_pointer     :inout natural;
   signal mod_pointer     :inout natural
  )is
  variable out_length     : natural;
  variable mod_length     : natural;
  begin
    
    --Now in the first word we are pointed on frame format word
    decode_dlc_buff(buff_out(3 downto 0),out_length);
    decode_dlc_buff(mod_out(3 downto 0) ,mod_length);
    
    --RTR frame has no data and is normal CAN frame
    if(buff_out(5)='1' and buff_out(7)='0')then
      out_length:=4;
    end if;
    
    --Reading all the words in cycles and storing
    --Here we assume that out_length and mod length are equal!
    for i in 0 to out_length-1 loop
      out_read              <= '1';
      out_mem(out_pointer)  <= buff_out;
      mod_mem(mod_pointer)  <= mod_out;
      out_pointer           <= out_pointer+1;
      mod_pointer           <= mod_pointer+1;
      wait for 10 ns;
      out_read              <= '0';
      wait for 10 ns;
    end loop;  
      
  end procedure;
  
  procedure compare_data
  (signal in_mem          :in   eval_mem_test;
   signal out_mem         :in   eval_mem_test;
   signal mod_mem         :in   eval_mem_test;
   variable cons_res      :out  boolean
   )is
   begin
     cons_res:=true;
     
     for i in 0 to in_mem'length-1 loop
       --if(in_mem(i) /= out_mem(i) or in_mem(i) /= mod_mem(i))then
       if(in_mem(i) /= out_mem(i))then
          cons_res:=false;
       end if;
     end loop;
     
   end procedure;
  
  for rx_Buffer_comp : rxBuffer use entity work.rxBuffer(rtl);
 -- for rx_Buffer_mod  : rxBuffer use entity work.rxBuffer(behav);
    
begin
   
-----------------------------------------------------------------------------------------------------------------
-- Buffer component
-----------------------------------------------------------------------------------------------------------------
   rx_Buffer_comp:rxBuffer 
    generic map(
      buff_size                 =>  buff_size
    )
    port map(
     clk_sys                    =>  clk_sys,
     res_n                      =>  res_n ,
     rec_ident_in               =>  rec_ident_in,
     rec_dram_word              =>  rec_dram_word,
     rec_dram_addr              =>  rec_dram_addr,   
     rec_dlc_in                 =>  rec_dlc_in,
     rec_ident_type_in          =>  rec_ident_type_in,
     rec_frame_type_in          =>  rec_frame_type_in,
     rec_is_rtr                 =>  rec_is_rtr,
     rec_brs                    =>  rec_brs,
     rec_esi                    =>  rec_esi,
     rec_message_ack            =>  rec_message_ack_b,
     rec_message_valid          =>  rec_message_valid,
     rx_buf_size                =>  rx_buf_size_b,
     
     timestamp                  =>  timestamp,
     
     rx_full                    =>  rx_full_b,
     rx_empty                   =>  rx_empty_b,
     rx_message_count           =>  rx_message_count_b,
     rx_mem_free                =>  rx_mem_free_b,
     rx_read_pointer_pos        =>  rx_read_pointer_pos_b,
     rx_write_pointer_pos       =>  rx_write_pointer_pos_b,
     rx_message_disc            =>  rx_message_disc_b,
     rx_data_overrun            =>  rx_data_overrun_b,
     rx_read_buff               =>  rx_read_buff_b,
     
     drv_bus                    =>  drv_bus
  );  
  
------------------------------------------------------------------------------------------------------------------
-- Buffer model
----------------------------------------------------------------------------------------------------------------- 
  --rx_Buffer_mod:rxBuffer 
--    generic map(
--      buff_size                 =>  32
--    )
--    port map(
--     clk_sys                    =>  clk_sys,
--     res_n                      =>  res_n ,
--     rec_ident_in               =>  rec_ident_in,
--     rec_dram_word              =>  rec_dram_word,
--     rec_dram_addr              =>  rec_dram_addr,     
--     rec_dlc_in                 =>  rec_dlc_in,
--     rec_ident_type_in          =>  rec_ident_type_in,
--     rec_frame_type_in          =>  rec_frame_type_in,
--     rec_is_rtr                 =>  rec_is_rtr,
--     rec_brs                    =>  rec_brs,
--     rec_message_ack            =>  rec_message_ack_m,
--     rec_message_valid          =>  rec_message_valid,
--     rx_buf_size                =>  rx_buf_size_m,
--     
--     timestamp                  =>  timestamp,
--     
--     rx_full                    =>  rx_full_m,
--     rx_empty                   =>  rx_empty_m,
--     rx_message_count           =>  rx_message_count_m,
--     rx_mem_free                =>  rx_mem_free_m,
--     rx_read_pointer_pos        =>  rx_read_pointer_pos_m,
--     rx_write_pointer_pos       =>  rx_write_pointer_pos_m,
--     rx_message_disc            =>  rx_message_disc_m,
--     rx_data_overrun            =>  rx_data_overrun_m,
--     rx_read_buff               =>  rx_read_buff_m,
--     
--     drv_bus                    =>  drv_bus
--  ); 
  
  -------------------------------------------
  --Connect input frame to stimuli generator
  -------------------------------------------
  rec_ident_in          <=  input_frame.rec_ident_in;
  rec_dlc_in            <=  input_frame.rec_dlc_in;
  rec_ident_type_in     <=  input_frame.rec_ident_type_in;
  rec_frame_type_in     <=  input_frame.rec_frame_type_in;
  rec_is_rtr            <=  input_frame.rec_is_rtr;
  rec_brs               <=  input_frame.rec_brs;
  rec_esi               <=  input_frame.rec_esi;
  rec_message_valid     <=  input_frame.rec_message_valid;
  
  -- Change to RAM usage for internal data of Protocol control!
  -- Only 32 bytes of data are provided at a time !!!
  rec_dram_word         <=  input_frame.rec_data_in((rec_dram_addr+1)*32-1 downto rec_dram_addr*32); 
  
  ---------------------------------
  --Clock and timestamp generation
  ---------------------------------
  clock_gen:process
  variable period   :natural:=f100_Mhz;
  variable duty     :natural:=50;
  variable epsilon  :natural:=0;
  begin
    generate_clock(period,duty,epsilon,clk_sys);
    timestamp <= std_logic_vector(unsigned(timestamp)+1);
  end process; 
  
  --Overall amount of errors is sum of errors from all processes
  error_ctr   <=  stim_errs+read_errs+status_errs+cons_errs;               
  
  --Common input memory is considered full once there is less space
  -- than RX buffer size!
  in_mem_full <= true when in_pointer+buff_size+1>1024  else
                 false;
                 
  out_mem_full <= true when out_pointer+buff_size+1>1024  else
                 false;
  
  mod_mem_full <= true when mod_pointer+buff_size+1>1024  else
                  false;
  
  --Auxiliarly process for read detection
  aux:process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      aux_read <= drv_bus(DRV_READ_START_INDEX);
    else
       aux_read <=  aux_read;
    end if;
  end process;
                  
  
  ------------------------------------------
  -- Stimuli generator - Main test process
  ------------------------------------------
  stim_gen:process
  variable gen_size     : natural:=0; --Size of generated frame in 32 bit words
  variable enough_space : boolean:=true;
  variable was_inserted : boolean:=false;
  begin
    log("Restarting RX Bufrer test!",info_l,log_level);
    wait for 5 ns;
    reset_test(res_n,status,run,stim_errs);
    log("Restarted RX Bufrer test",info_l,log_level);
    print_test_info(iterations,log_level,error_beh,error_tol);
    log("Consider adding RX Buffer model into this testbench!",warning_l,log_level);
    log("Note that RX buffer content is NOT initialized on purpose!"&
         "If initialized during reset, it is impossible to do synthesis into RAM!",warning_l,log_level);
             
    -------------------------------
    --Main loop of the test
    -------------------------------
    log("Starting RX buffer main loop",info_l,log_level);
    
    while (loop_ctr<iterations  or  exit_imm)
    loop
      
      --Start generating the frames on Input
      --as long as there is enough space available in the common memory
      while (in_mem_full=false) loop
          generate_frame(rand_ctr,input_frame,gen_size);
          
          --Check whether we wil have enough space in the buffer         
          if( (gen_size<=rx_mem_free_b) or 
              (gen_size=rx_mem_free_b+1 and aux_read='1' and drv_bus(DRV_READ_START_INDEX)='0')
          )then
            enough_space:=true;
          else
            enough_space:=false;
          end if;
          
          --Now buffer has for sure space
          --Frame is inserted into the RX Buffer, Model and stored
          --also into common memory
          was_inserted:=false;
          insert_frame(input_frame,timestamp,enough_space,was_inserted,rx_message_disc_b,
                       rx_message_disc_m,in_mem,in_pointer,log_level);
                                       
          --Here we check whether frame was not discarded by either buffer
          if (was_inserted=false and enough_space=true) or
             (was_inserted=true  and enough_space=false)
          then
            process_error(stim_errs,error_beh,exit_imm);
            log("Incorrect discard value from buffers returned!",error_l,log_level);
          end if;  
      end loop;
      
      --Now input memory is full
      --We need to wait for Data reader to read all frames into common memory from
      --rx buffer and its model. Then it checks data consistency and next iteration
      --can start
      wait until iteration_done=true;
      
      --Now common input memory is erased
      in_mem <= (OTHERS => (OTHERS => '0'));
      in_pointer <= 0;
      loop_ctr<= loop_ctr+1;
      
      wait for 100 ns;      
      
    end loop;
    
    
    --This is the main process loop so we evaluate test here
    evaluate_test(error_tol,error_ctr,status);
     
  end process;
  
  
  ---------------------------------
  -- Data reader
  ---------------------------------
  data_reader:process
  variable sanity_check   :boolean  :=  true;
  variable sanity_counter :natural  :=  0;
  begin
    --Offset in time only in first clock cycle
    if(loop_ctr=0)then
        wait for 5 ns;
    end if;
    
    while (out_mem_full=false and mod_mem_full=false and sanity_check=true) loop
      
        if(rx_empty_b='0')then
          read_frame(rx_read_buff_b,rx_read_buff_m,out_mem,mod_mem,
                     drv_bus(DRV_READ_START_INDEX),out_pointer,mod_pointer);
          sanity_counter:=0;
        else         
          wait for 10 ns;
          
          --If buffer is empty then wait a little
          sanity_counter:=sanity_counter+1;
          
          --There is nothing to read from the buffers...
          if(sanity_counter=50)then
            sanity_check := false;
            process_error(read_errs,error_beh,exit_imm_d);
            log("There is nothing to read for too long!",error_l,log_level);
          end if;
          
        end if;
            
    end loop;
    
    --Now output memory is full
    --We need to wait for Status block to compare the data 
    --consistency
    wait until iteration_done=true;
    
    sanity_check    :=  true;
    sanity_counter  :=  0;
    
    --Erase the common memories
    out_mem <= (OTHERS => (OTHERS => '0'));
    out_pointer <= 0;
    mod_mem <= (OTHERS => (OTHERS => '0'));
    mod_pointer <= 0;
    
    
    wait for 100 ns;
    
  end process;
  
  
  ---------------------------------
  -- Status controller
  ---------------------------------
  status_contr:process
  begin
    --Offset in time only in first clock cycle
    if(loop_ctr=0)then
        wait for 5 ns;
    end if;
    
    --Test is at the same time inserting the data
    --into model and real buffer! Thus status
    --signals should be at each time equal!
    --
    --This process detects any differences
    
    --Temporary stopped
    wait;
    
     if(rx_full_b /= rx_full_m) then
        process_error(status_errs,error_beh,exit_imm_d_2);
        log("Model and Circuit mismatch in signal: rx_full",error_l,log_level);
     end if;
      
     if(rx_empty_b /= rx_empty_m) then
        process_error(status_errs,error_beh,exit_imm_d_2);
        log("Model and Circuit mismatch in signal: rx_empty",error_l,log_level);
     end if; 
    
     if(rx_message_count_b /= rx_message_count_m) then
        process_error(status_errs,error_beh,exit_imm_d_2);
        log("Model and Circuit mismatch in signal: rx_message_count",error_l,log_level);
     end if;
     
     if(rx_mem_free_b /= rx_mem_free_m) then
        process_error(status_errs,error_beh,exit_imm_d_2);
        log("Model and Circuit mismatch in signal: rx_mem_free",error_l,log_level);
     end if;
     
     if(rx_data_overrun_b /= rx_data_overrun_m) then
        process_error(status_errs,error_beh,exit_imm_d_2);
        log("Model and Circuit mismatch in signal: rx_data_overrun",error_l,log_level);
     end if;
     
     if(rx_read_pointer_pos_b /= rx_read_pointer_pos_m) then
        process_error(status_errs,error_beh,exit_imm_d_2);
        log("Model and Circuit mismatch in signal: rx_read_pointer_pos",error_l,log_level);
     end if;
     
     if(rx_write_pointer_pos_b /= rx_write_pointer_pos_m) then
        process_error(status_errs,error_beh,exit_imm_d_2);
        log("Model and Circuit mismatch in signal: rx_write_pointer_pos",error_l,log_level);
     end if;
     
     if(rx_read_buff_b /= rx_read_buff_m) then
        process_error(status_errs,error_beh,exit_imm_d_2);
        log("Model and Circuit mismatch in signal: rx_read_buff",error_l,log_level);
     end if;
     
     wait for 10 ns;
     
  end process;
  
  ---------------------------------
  -- Data consistency checker
  ---------------------------------
  cons_check:process
  variable cons_res:boolean:=false;
  variable clk_time:time:=10 ns;
  begin
    
    iteration_done <= false;
    
    --Wait until data we inserted from the buffer and model
    wait until in_mem_full=true;
    
    --Now that all the data wer inserted into the buffer
    --we will wait until it it for  sure read back OK
    --That means buffer_size*2 clock cycles!
    wait for buff_size*2*clk_time;
    
    --Now compare the data
    cons_res:=false;
    compare_data(in_mem,out_mem,mod_mem,cons_res);
    
    if(cons_res=false)then
      process_error(cons_errs,error_beh,exit_imm_d_3);
      log("Data consistency check failed !",error_l,log_level); 
    end if;
    
    --Now we can tell to the other circuits that one iteration is over
    iteration_done <= true;
    wait for 100 ns;
    
  end process;
  
  
end architecture;



-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture rx_buf_unit_test_wrapper of CAN_test_wrapper is
  
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
  for test_comp : CAN_test use entity work.CAN_test(rx_buf_unit_test);
  
  signal run              :   boolean;                -- Input trigger, test starts running when true                                                        -- exceed this value in order for the test to pass
  signal status_int       :   test_status_type;      -- Status of the test
  signal errors           :   natural;                -- Amount of errors which appeared in the test
  
begin
  
  --In this test wrapper generics are directly connected to the signals
  -- of test entity
  test_comp:CAN_test
  port map(
     run              =>  run,
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
  

end architecture;

