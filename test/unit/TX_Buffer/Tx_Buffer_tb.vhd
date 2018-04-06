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
--  Unit test for the TX Buffer circuit                                                 
--------------------------------------------------------------------------------
-- Revision History:
--    14.6.2016   Created file
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

architecture tx_buf_unit_test of CAN_test is
  
    -- Clocking and reset
    signal clk_sys                :   std_logic:='0';
    signal res_n                  :   std_logic:='0';
    
    -------------------------------
    --Driving Registers Interface--
    -------------------------------
    
    -- Data and address for SW access into the RAM of TXT Buffer
    signal tran_data              :     std_logic_vector(31 downto 0);
    signal tran_addr              :     std_logic_vector(4 downto 0);
    signal tran_cs                :     std_logic;
    
    -- SW commands from user registers
    signal txt_sw_cmd             :     txt_sw_cmd_type;
    signal txt_sw_buf_cmd_index   :     std_logic_vector(buf_count - 1 downto 0);
  
    ------------------     
    --Status signals--
    ------------------
    signal txtb_state             :     txt_fsm_type;
    
    ------------------------------------
    --CAN Core and TX Arbiter Interface-
    ------------------------------------
    
    -- Commands from the CAN Core for manipulation of the CAN 
    signal txt_hw_cmd             :     txt_hw_cmd_type;  
    signal txt_hw_cmd_buf_index   :     natural range 0 to buf_count - 1;
  
    -- Buffer output and pointer to the RAM memory
    signal txt_word               :     std_logic_vector(31 downto 0);
    signal txt_addr               :     natural range 0 to 19;
    
    -- Signals to the TX Arbitrator that it can be selected for transmission
    -- (used as input to priority decoder)
    signal txt_buf_ready          :     std_logic

begin
   
    ----------------------------------------------------------------------------
    -- Buffer components - create only one instance
    ----------------------------------------------------------------------------
    txt_Buf_comp : txtBuffer 
    generic map(
        buf_count               => 4,
        ID                      => 0
    )
    port map(
        clk_sys                 => clk_sys,    
        res_n                   => res_n,
        tran_data               => tran_data,
        tran_addr               => tran_addr,
        tran_cs                 => tran_cs,
        txt_sw_cmd              => txt_sw_cmd,
        txt_sw_buf_cmd_index    => txt_sw_buf_cmd_index,
        txtb_state              => txtb_state,
        txt_hw_cmd              => txt_hw_cmd,
        txt_hw_cmd_buf_index    => txt_hw_cmd_buf_index,
        txt_word                => txt_word,
        txt_addr                => txt_addr,
        txt_buf_ready           => txt_buf_ready
    );
  
    ---------------------------------
    -- Clock generation
    ---------------------------------
    clock_gen : process
    variable period   :natural := f100_Mhz;
    variable duty     :natural := 50;
    variable epsilon  :natural := 0;
    begin
      generate_clock(period, duty, epsilon, clk_sys);
    end process;
    
    --------------------------------------------
    -- Data generation
    -------------------------------------------- 
    data_gen_proc : process
        variable rand_nr    : real;
        variable rand_time  : time;
    begin
      
    end process;
  
      
    ---------------------------------
    ---------------------------------
    --Main Test process
    ---------------------------------
    ---------------------------------
    test_proc : process
        variable rand_nr    : real;
        variable rand_time  : time;
    begin
        log("Restarting TXT Buffer test!", info_l, log_level);
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        log("Restarted TXT Buffer test", info_l, log_level);
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        log("Starting TXT Buffer main loop", info_l, log_level);

        while (loop_ctr < iterations  or  exit_imm)
        loop
            log("Starting loop nr " & integer'image(loop_ctr),
                                        info_l, log_level);
            wait until falling_edge(clk_sys);

            


            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol,error_ctr,status);
    end process;
      
end architecture;




-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture tx_buf_unit_test_wrapper of CAN_test_wrapper is
  
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
  for test_comp : CAN_test use entity work.CAN_test(tx_buf_unit_test);
  
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
