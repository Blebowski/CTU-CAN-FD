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
--  Unit test for TX_Arbitrator circuit                                                 
--------------------------------------------------------------------------------
-- Revision History:
--    30.5.2016   Created file
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Test implementation                                            
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.ID_transfer.all;

architecture tx_arb_unit_test of CAN_test is
  
    signal txt1_buffer_in         :  std_logic_vector(639 downto 0):=(OTHERS => '0');     
    signal txt1_buffer_empty      :  std_logic:='0';                        
    signal txt1_buffer_ack        :  std_logic:='0';                          
    signal txt2_buffer_in         :  std_logic_vector(639 downto 0):=(OTHERS => '0');       
    signal txt2_buffer_empty      :  std_logic:='0';                          
    signal txt2_buffer_ack        :  std_logic:='0';                         
    signal tran_data_out          :  std_logic_vector(511 downto 0):=(OTHERS => '0');      
    signal tran_ident_out         :  std_logic_vector(28 downto 0):=(OTHERS => '0');       
    signal tran_dlc_out           :  std_logic_vector(3 downto 0):=(OTHERS => '0');        
    signal tran_is_rtr            :  std_logic:='0';                         
    signal tran_ident_type_out    :  std_logic:='0';                         
    signal tran_frame_type_out    :  std_logic:='0';                    
    signal tran_brs_out           :  std_logic:='0';                    
    signal tran_frame_valid_out   :  std_logic:='0';                    
    signal tran_data_ack          :  std_logic:='0';                                                
                                                                          
    signal drv_bus                :  std_logic_vector(1023 downto 0):= (OTHERS => '0'); 
    signal timestamp              :  std_logic_vector(63 downto 0):= 
                                      "0000000000000000000000000000000011111111111111111111111111111111";  
    
    --Here reset is only to have the same framework for each test
    -- Circuit is only combinational
    signal res_n                  :  std_logic:='0';
    signal clk_sys                :  std_logic:='0';
    
    signal drv_allow_txt1         :   std_logic:='0';                             --Allow transmit of messages from tx buffer
    signal drv_allow_txt2         :   std_logic:='0';                             --Allow transmit of messages from txt buffer
    
    -------------------------------------------------------
    -- Generatees random input frame as TX buffet would do
    -------------------------------------------------------
    procedure gen_random_input(
      signal  rand_ctr            :inout  natural range 0 to RAND_POOL_SIZE;
      signal  buffer_in           :inout  std_logic_vector(639 downto 0);
      signal  buffer_empty        :inout  std_logic
    )is
    variable aux_log_vector       :std_logic_vector(31 downto 0);
    begin
      --Generate frame format
      rand_logic_vect_v(rand_ctr,aux_log_vector,0.5);
      buffer_in(639 downto 608)<=aux_log_vector;
      
      --Generate identifier and data
      rand_logic_vect(rand_ctr,buffer_in(575 downto 0),0.5);
      rand_logic(rand_ctr,buffer_empty,0.5);
      
      --Uppest 32 bits of timestamp is always zero
      --buffer_in(607 downto 576) <= (OTHERS => '0');
      --buffer_in(575 downto 544) <= (OTHERS => '0');
      --buffer_in(543 downto 512) <= (OTHERS => '0');
      wait for 0 ns;
      
    end procedure;
    
    --Comparing procedure for two 64 bit std logic vectors
    function less_than(
      signal   a       : in std_logic_vector(63 downto 0);
      signal   b       : in std_logic_vector(63 downto 0)
    )return boolean is
    begin
       if (unsigned(a(63 downto 32)) < unsigned(b(63 downto 32))) or 
          ((a(63 downto 32) = b(63 downto 32)) and (unsigned(a(31 downto 0)) < unsigned(b(31 downto 0))))then
          return true;
      else
         return false;
      end if;
   
    end function;
    
    procedure compare_frame(
      signal buffer_in              :in  std_logic_vector(639 downto 0); 
      signal tran_data_out          :in  std_logic_vector(511 downto 0);    
      signal tran_ident_out         :in  std_logic_vector(28 downto 0);     
      signal tran_dlc_out           :in  std_logic_vector(3 downto 0);      
      signal tran_is_rtr            :in  std_logic;                         
      signal tran_ident_type_out    :in  std_logic;                         
      signal tran_frame_type_out    :in  std_logic;                         
      signal tran_brs_out           :in  std_logic;
      variable outcome              :out boolean
    )is
    begin
      outcome:=true;
      
      if(buffer_in(611 downto 608) /= tran_dlc_out)then
        outcome:=false;
      end if; 
      
      if(buffer_in(613) /= tran_is_rtr)then
        outcome:=false;
      end if; 
      
      if(buffer_in(614) /= tran_ident_type_out)then
        outcome:=false;
      end if;
      
      if(buffer_in(615) /= tran_frame_type_out)then
        outcome:=false;
      end if;
      
      if(buffer_in(617) /= tran_brs_out)then
        outcome:=false;
      end if;  
      
      if(buffer_in(540 downto 512) /= tran_ident_out)then
        outcome:=false;
      end if;
      
      if(buffer_in(511 downto 0) /= tran_data_out)then
        outcome:=false;
      end if;
      
    end procedure;
    
    procedure check_output(
      signal txt1_buffer_in         :in  std_logic_vector(639 downto 0);     
      signal txt1_buffer_empty      :in  std_logic;                          
      signal txt1_buffer_ack        :in  std_logic;                          
      signal txt2_buffer_in         :in  std_logic_vector(639 downto 0);     
      signal txt2_buffer_empty      :in  std_logic;                          
      signal txt2_buffer_ack        :in  std_logic;
      signal tran_data_out          :in  std_logic_vector(511 downto 0);    
      signal tran_ident_out         :in  std_logic_vector(28 downto 0);     
      signal tran_dlc_out           :in  std_logic_vector(3 downto 0);      
      signal tran_is_rtr            :in  std_logic;                         
      signal tran_ident_type_out    :in  std_logic;                         
      signal tran_frame_type_out    :in  std_logic;                         
      signal tran_brs_out           :in  std_logic;                         
      signal tran_frame_valid_out   :in  std_logic;                         
      signal tran_data_ack          :in  std_logic;
      signal drv_allow_txt1         :in  std_logic;
      signal drv_allow_txt2         :in  std_logic;
      variable outcome              :out boolean
    )is
      variable estim_src              :std_logic:='0';
      variable ts1_valid              :boolean:=false;
      variable ts2_valid              :boolean:=false;
      variable id_1_dec               :natural;
      variable id_2_dec               :natural;
      variable base                   :std_logic_vector(10 downto 0);
      variable ext                    :std_logic_vector(17 downto 0);
      variable conc                   :std_logic_vector(28 downto 0);
      variable ts_val_1               :boolean:=false;
      variable ts_val_2               :boolean:=false;
      variable no_frame               :boolean:=true;
      variable imm                    :boolean:=false;
    begin
      outcome:=true;
      
      --Calculate the decimal values of identifiers
      base      :=  txt1_buffer_in(522 downto 512);
      ext       :=  txt1_buffer_in(540 downto 523);
      conc      :=  base&ext;
      id_1_dec  :=  to_integer(unsigned(conc));
      
      base      :=  txt2_buffer_in(522 downto 512);
      ext       :=  txt2_buffer_in(540 downto 523);
      conc      :=  base&ext;
      id_2_dec  :=  to_integer(unsigned(conc));     
      
      if(txt1_buffer_empty='1' and txt2_buffer_empty='1')then
        no_frame:=true;
      else
        no_frame:=false;
      end if;
          
      --No frame should be put on output if none on input
      if(txt1_buffer_empty='1' and txt2_buffer_empty='1' 
         and tran_frame_valid_out='1')then
        outcome:=false;
        log("Both buffers empty but frame_valid is active!",error_l,log_level);
      else
        --Here we dont car what data are on output as long as frame_valid is inactive
        outcome:=true;
      end if;
      
      ----------------------------------
      --Determine the predicted source
      ----------------------------------
      
      --Only buffer 1 has message and is allowed
      if(txt1_buffer_empty='1' and txt2_buffer_empty='0' and drv_allow_txt2='1' )then
        estim_src:='1';
        
      --Only buffer two has frame ans is allowed
      elsif(txt1_buffer_empty='0' and txt2_buffer_empty='1' and drv_allow_txt1='1' )then
        estim_src:='0';
        
      --Both buffers have message but only 1 is allowed
      elsif(txt1_buffer_empty='0' and txt2_buffer_empty='0' and drv_allow_txt1='1' and drv_allow_txt2='0')then
        estim_src:='0';
      
      --Both buffers have message but only 2 is allowed
      elsif(txt1_buffer_empty='0' and txt2_buffer_empty='0' and drv_allow_txt1='0' and drv_allow_txt2='1')then
        estim_src:='1';
      
      --Both buffers are allowed and non empty...
      elsif(txt1_buffer_empty='0' and txt2_buffer_empty='0' and 
            less_than(txt1_buffer_in(607 downto 544),txt2_buffer_in(607 downto 544))=true)then
        estim_src:='0';
        
      elsif(txt1_buffer_empty='0' and txt2_buffer_empty='0' and 
            txt1_buffer_in(607 downto 544)=txt2_buffer_in(607 downto 544) and
             (id_1_dec   < id_2_dec))then
        estim_src:='0';  
      else
        estim_src:='1';
      end if;
      
      --See whether timestamps are valid
      if(less_than(txt1_buffer_in(607 downto 544),timestamp) and drv_allow_txt1='1')then
        ts_val_1:=true;
      else
        ts_val_1:=false;
      end if;
      
      --See whether timestamps are valid
      if(less_than(txt2_buffer_in(607 downto 544),timestamp) and drv_allow_txt2='1')then
        ts_val_2:=true;
      else
        ts_val_2:=false;
      end if;
      
      if(no_frame=false)then
        
        if(estim_src='0' and ts_val_1=true and txt1_buffer_empty='0') then
          compare_frame(txt1_buffer_in,tran_data_out, tran_ident_out, tran_dlc_out, tran_is_rtr ,                         
                         tran_ident_type_out , tran_frame_type_out, tran_brs_out, imm);
          if(tran_frame_valid_out='0')then
            log("Frame_valid is inactive expecting active",error_l,log_level);
            outcome:=false;
          end if;
          
          if(imm=false)then
            log("Comparison between input and expected output failed",error_l,log_level);
            outcome:=false;
          end if;
          
        elsif (estim_src='0' and ts_val_1=false) then
          
          if(tran_frame_valid_out='1')then
            outcome:=false;
            log("Frame_valid active but timestamp does not have according value",error_l,log_level);
          end if;
          
        elsif (estim_src='1' and ts_val_2=true and txt2_buffer_empty='0') then
          
          compare_frame(txt2_buffer_in,tran_data_out, tran_ident_out, tran_dlc_out, tran_is_rtr ,                         
                         tran_ident_type_out , tran_frame_type_out, tran_brs_out, imm);
          if(imm=false or tran_frame_valid_out='0')then
            outcome:=false;
            log("Frame_valid invalid when it should be valid or data comparison failed",error_l,log_level);
          end if;
          
        elsif (estim_src='1' and ts_val_2=false ) then
          
          if(tran_frame_valid_out='1')then
            log("Frame_valid valid when it should be invalid",error_l,log_level);
            outcome:=false;
          end if;
          
        end if;     
         
      end if;
      
    end procedure;
        
begin
  
  tx_Arbitrator_comp:txArbitrator  
  port map( 
     txt1_buffer_in       =>  txt1_buffer_in,      
     txt1_buffer_empty    =>  txt1_buffer_empty , 
     txt1_buffer_ack      =>  txt1_buffer_ack,
     txt2_buffer_in       =>  txt2_buffer_in, 
     txt2_buffer_empty    =>  txt2_buffer_empty,
     txt2_buffer_ack      =>  txt2_buffer_ack,
     tran_data_out        =>  tran_data_out,  
     tran_ident_out       =>  tran_ident_out,
     tran_dlc_out         =>  tran_dlc_out,
     tran_is_rtr          =>  tran_is_rtr,
     tran_ident_type_out  =>  tran_ident_type_out,
     tran_frame_type_out  =>  tran_frame_type_out,
     tran_brs_out         =>  tran_brs_out ,                       
     tran_frame_valid_out =>  tran_frame_valid_out ,                   
     tran_data_ack        =>  tran_data_ack,                                                                     
     drv_bus              =>  drv_bus,
     timestamp            =>  timestamp
  );
  
  --Driving bus aliases
  drv_bus(DRV_ALLOW_TXT1_INDEX) <= drv_allow_txt1;
  drv_bus(DRV_ALLOW_TXT2_INDEX) <= drv_allow_txt2;
  
  ---------------------------------
  --Clock generation
  ---------------------------------
  clock_gen:process
  variable period   :natural:=f100_Mhz;
  variable duty     :natural:=50;
  variable epsilon  :natural:=0;
  begin
    generate_clock(period,duty,epsilon,clk_sys);
    timestamp <= std_logic_vector(unsigned(timestamp)+1);
  end process;
  
  ---------------------------------
  -- Input generator
  ---------------------------------
  input_gen:process
  begin
    wait until falling_edge(clk_sys);
    gen_random_input(rand_ctr,txt1_buffer_in,txt1_buffer_empty);
    gen_random_input(rand_ctr,txt2_buffer_in,txt2_buffer_empty);
    rand_logic(rand_ctr,drv_allow_txt1,0.5);
    rand_logic(rand_ctr,drv_allow_txt2,0.5);
  end process;
  
  ---------------------------------
  ---------------------------------
  --Main Test process
  ---------------------------------
  ---------------------------------
  test_proc:process
  variable outcome:boolean;
  begin
    log("Restarting TX Arbitrator test!",info_l,log_level);
    wait for 5 ns;
    reset_test(res_n,status,run,error_ctr);
    log("Restarted TX Arbitrator test",info_l,log_level);
    print_test_info(iterations,log_level,error_beh,error_tol);
    
    -------------------------------
    --Main loop of the test
    -------------------------------
    log("Starting main loop",info_l,log_level);
    
    while (loop_ctr<iterations  or  exit_imm)
    loop
      log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
     
      --TODO: Test processing
      check_output( txt1_buffer_in, txt1_buffer_empty ,txt1_buffer_ack, txt2_buffer_in, txt2_buffer_empty,                      
                     txt2_buffer_ack , tran_data_out, tran_ident_out , tran_dlc_out ,tran_is_rtr ,                      
                     tran_ident_type_out  , tran_frame_type_out , tran_brs_out, tran_frame_valid_out,                         
                     tran_data_ack , drv_allow_txt1 , drv_allow_txt2 , outcome);
      
      if(outcome=false)then
        log("Predicted and actual output not matching!",error_l,log_level);
        process_error(error_ctr,error_beh,exit_imm);  
      end if;
      
      wait until rising_edge(clk_sys);
      
      loop_ctr<=loop_ctr+1;
    end loop;
    
    evaluate_test(error_tol,error_ctr,status);
  end process;
  
  
end architecture;




-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture tx_arb_unit_test_wrapper of CAN_test_wrapper is
  
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
  for test_comp : CAN_test use entity work.CAN_test(tx_arb_unit_test);
  
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
  
  
end;

