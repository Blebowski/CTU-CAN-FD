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
--  Unit test for TX Arbitrator circuit                                                 
--------------------------------------------------------------------------------
-- Revision History:
--    30.5.2016   Created file
--    23.4.2018   Updated test to cover TX Arbitrator with continous timestamp
--                load.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

use work.ID_transfer.all;

architecture tx_arb_unit_test of CAN_test is
    
    ------------------------
    -- DUT signals    
    ------------------------
    signal clk_sys                :  std_logic;
    signal res_n                  :  std_logic := '0';
    signal txt_buf_in             :  txtb_output_type :=
                                        (OTHERS => (OTHERS => '0'));

    signal txt_buf_ready          :  std_logic_vector(TXT_BUFFER_COUNT - 1 downto 0)
                                        := (OTHERS => '0');

    signal txtb_ptr               :  natural range 0 to 19;
    signal tran_data_word_out     :  std_logic_vector(31 downto 0);
    signal tran_dlc_out           :  std_logic_vector(3 downto 0);
    signal tran_is_rtr            :  std_logic;
    signal tran_ident_type_out    :  std_logic;
    signal tran_frame_type_out    :  std_logic;
    signal tran_brs_out           :  std_logic;
    signal tran_frame_valid_out   :  std_logic;
    signal txt_hw_cmd             :  txt_hw_cmd_type := ('0','0','0','0','0','0');
    signal txtb_changed           :  std_logic;
    signal txt_hw_cmd_buf_index   :  natural range 0 to TXT_BUFFER_COUNT - 1;
    signal txtb_core_pointer      :  natural range 0 to 19 := 0;
    signal drv_bus                :  std_logic_vector(1023 downto 0);
    signal txt_buf_prio           :  txtb_priorities_type :=
                                        (OTHERS => (OTHERS => '0'));
    signal timestamp              :  std_logic_vector(63 downto 0) :=
                                        (OTHERS => '0');

    ------------------------
    -- Internal TB signals   
    ------------------------

    -- Memories as if connected to TXT Buffers
    type txtb_test_mem_type is array (0 to 19) of std_logic_vector(31 downto 0);
    type txtb_multi_test_type is array (0 to TXT_BUFFER_COUNT - 1) of
         txtb_test_mem_type;
    signal shadow_mem             : txtb_multi_test_type :=
                                    (OTHERS => (OTHERS => (OTHERS => '0')));

    -- Random pool pointers
    signal rand_ctr_1             : natural range 0 to RAND_POOL_SIZE;
    signal rand_ctr_2             : natural range 0 to RAND_POOL_SIZE;
    signal rand_ctr_3             : natural range 0 to RAND_POOL_SIZE;

    -- Highest priority buffer which is ready
    signal high_prio_buf_index    : natural range 0 to TXT_BUFFER_COUNT - 1;
    
     -- Modeled outputs
     signal mod_dlc_out           :  std_logic_vector(3 downto 0) := "0000";
     signal mod_is_rtr            :  std_logic := '0';
     signal mod_ident_type_out    :  std_logic := '0';
     signal mod_frame_type_out    :  std_logic := '0';
     signal mod_brs_out           :  std_logic := '0';
     signal mod_frame_valid_out   :  std_logic := '0';
     signal mod_buf_index         :  natural range 0 to TXT_BUFFER_COUNT - 1 :=
                                     0;

    -- Model is locked (as if transmission in progress)
    signal mod_locked             :  boolean := false;

    -- Error counters
    signal cmp_err_ctr            :  natural;


    -- Comparing procedure for two 64 bit std logic vectors
    function less_than(
      signal   a       : in std_logic_vector(63 downto 0);
      signal   b       : in std_logic_vector(63 downto 0)
    )return boolean is
    begin
       if (unsigned(a(63 downto 32)) < unsigned(b(63 downto 32))) or 
          ((a(63 downto 32) = b(63 downto 32)) and 
          (unsigned(a(31 downto 0)) < unsigned(b(31 downto 0))))
       then
          return true;
       else
          return false;
       end if;
   
     end function;


    -- Setting TXT Buffer priorities as from user registers
    procedure set_priorities(
        signal rand_ptr               :inout natural range 0 to RAND_POOL_SIZE;
        signal txt_buf_prio           :out   txtb_priorities_type
    )is
        variable tmp                  : std_logic_vector(2 downto 0);
    begin
        for i in 0 to TXT_BUFFER_COUNT - 1 loop
            rand_logic_vect_v(rand_ptr, tmp, 0.5);
            txt_buf_prio(i)      <= tmp;
        end loop;
    end procedure;
    

begin

  ---------------------------------
  -- DUT
  ---------------------------------
  txArbitrator_comp : txArbitrator
  generic map(
    buf_count               => TXT_BUFFER_COUNT,
    tx_time_sup             => true
  )
  port map( 
     clk_sys                => clk_sys,
     res_n                  => res_n,
     txt_buf_in             => txt_buf_in,
     txt_buf_ready          => txt_buf_ready,
     txtb_ptr               => txtb_ptr,
     tran_data_word_out     => tran_data_word_out,
     tran_dlc_out           => tran_dlc_out,
     tran_is_rtr            => tran_is_rtr,
     tran_ident_type_out    => tran_ident_type_out,
     tran_frame_type_out    => tran_frame_type_out,
     tran_brs_out           => tran_brs_out,
     tran_frame_valid_out   => tran_frame_valid_out,
     txt_hw_cmd             => txt_hw_cmd,
     txtb_changed           => txtb_changed,
     txt_hw_cmd_buf_index   => txt_hw_cmd_buf_index,
     txtb_core_pointer      => txtb_core_pointer,
     drv_bus                => drv_bus,
     txt_buf_prio           => txt_buf_prio,
     timestamp              => timestamp
  );


  ----------------------------------------------
  -- Emulate content and state of TXT Buffers
  ----------------------------------------------
  buf_em_proc : process
    variable wait_time_r    : real;
    variable wait_time      : time;
    variable buf_index      : real;
    variable tmp            : std_logic_vector(31 downto 0);
    variable extra_time     : std_logic_vector(7 downto 0);
    variable time_to_tx     : std_logic_vector(31 downto 0);
  begin

    -- Wait up to 100 ns
    rand_real_v(rand_ctr_1, wait_time_r);
    wait_time_r := wait_time_r * 100.0;
    wait_time   := wait_time_r * 1 ns;
    wait for wait_time;

    -- Choose random TXT Buffer
    rand_real_v(rand_ctr_1, buf_index);
    buf_index := buf_index * 3.0;

    -- Make sure buffer is not "ready", only then it can be accessed
    txt_buf_ready(integer(buf_index)) <= '0';
    wait for 10 ns;

    -- Fill the buffer with random data
    for i in 0 to 19 loop
        rand_logic_vect_v(rand_ctr_1, tmp, 0.5);
        shadow_mem(integer(buf_index))(i) <= tmp;
    end loop;

    -- Make sure that timestamp words in the buffer have some normal value,
    -- otherwise no buffer would ever get on output...
    -- Set the time to transmit to actual timestamp + some extra time
    shadow_mem(integer(buf_index))(3) <= timestamp(63 downto 32);
    rand_logic_vect_v(rand_ctr_1, extra_time, 0.3);
    time_to_tx  := std_logic_vector(to_unsigned(
                    to_integer(unsigned(timestamp(31 downto 0))) +
                    to_integer(unsigned(extra_time)), 32));
    shadow_mem(integer(buf_index))(2) <= time_to_tx;

    -- Make the buffer ready again
    txt_buf_ready(integer(buf_index)) <= '1';

    wait for 150 ns;

  end process;


  ------------------------------------------------------------------------------
  -- Connect TX Arbitrator to the shadow memories which emulate TXT Buffers
  ------------------------------------------------------------------------------
  buf_access_emu_proc : process (res_n, clk_sys)
  begin
    if (res_n = ACT_RESET) then
         txt_buf_in(0) <= (OTHERS => '0');
         txt_buf_in(1) <= (OTHERS => '0');
         txt_buf_in(2) <= (OTHERS => '0');
         txt_buf_in(3) <= (OTHERS => '0');
    elsif (rising_edge(clk_sys)) then
         txt_buf_in(0) <= shadow_mem(0)(txtb_ptr);
         txt_buf_in(1) <= shadow_mem(1)(txtb_ptr);
         txt_buf_in(2) <= shadow_mem(2)(txtb_ptr);
         txt_buf_in(3) <= shadow_mem(3)(txtb_ptr);
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- Model TX Arbitrator. Choose Highest priority "ready" TXT Buffer
  ------------------------------------------------------------------------------
  tx_arb_model_proc : process
    variable tmp_index    : natural;
    variable tmp_prio     : natural;
  begin
    
    -- Choose highest priority TXT buffer 
    tmp_index           := 0;
    tmp_prio            := 0;
    for i in 0 to TXT_BUFFER_COUNT - 1 loop
        if ((to_integer(unsigned(txt_buf_prio(i))) >= tmp_prio) and
            txt_buf_ready(i) = '1')
        then
            tmp_index   := i;
            tmp_prio    := to_integer(unsigned(txt_buf_prio(i)));
        end if;
    end loop;

    -- Update the buffer
    high_prio_buf_index <= tmp_index;

    wait for 10 ns;
  end process;


  ------------------------------------------------------------------------------
  -- Update data on output based on the highest priority buffer and timestamp
  ------------------------------------------------------------------------------
  tx_time_proc : process
    variable ts_elapsed : boolean       := false;
    variable ts         : std_logic_vector(63 downto 0);
  begin

    while res_n = ACT_RESET loop
        wait until rising_edge(clk_sys);
    end loop;

    -- Wait until timestamp is reached
    while ts_elapsed = false or mod_locked = true loop
        ts := shadow_mem(high_prio_buf_index)(3) &
              shadow_mem(high_prio_buf_index)(2);
        wait for 10 ns;
        if (to_integer(unsigned(ts)) < to_integer(unsigned(timestamp))) then
            ts_elapsed := true;
        else
            ts_elapsed := false;
        end if;
    end loop;

    -- Emulate two clock cycles delay!
    wait for 20 ns;

    -- Propagate metadata to the output
    mod_buf_index        <= high_prio_buf_index;
    mod_dlc_out          <= shadow_mem(high_prio_buf_index)(0)
                                       (DLC_H downto DLC_L);  
    mod_is_rtr           <= shadow_mem(high_prio_buf_index)(0)
                                       (RTR_IND); 
    mod_ident_type_out   <= shadow_mem(high_prio_buf_index)(0)
                                       (ID_TYPE_IND); 
    mod_frame_type_out   <= shadow_mem(high_prio_buf_index)(0)
                                       (FR_TYPE_IND); 
    mod_brs_out          <= shadow_mem(high_prio_buf_index)(0)
                                       (BRS_IND); 
    mod_frame_valid_out  <= '1';

    -- Wait until the buffer gets unlocked again
    while mod_locked = true loop
        wait until rising_edge(clk_sys);
    end loop;
  end process;
  

  ------------------------------------------------------------------------------
  -- Model LOCK and UNLOCK commands as if coming from CAN Core
  ------------------------------------------------------------------------------
  cmd_mod_proc : process
    variable wait_time      : time;
    variable wait_time_r    : real;
  begin
    while mod_frame_valid_out = '0' loop
        wait until rising_edge(clk_sys);
    end loop;

    wait until rising_edge(clk_sys);

    -- Lock the Buffer
    txt_hw_cmd.lock <= '1';
    mod_locked      <= true;
    wait until rising_edge(clk_sys);
    txt_hw_cmd.lock <= '0';

    -- Wait random time
    rand_real_v(rand_ctr_3, wait_time_r);
    wait_time_r := wait_time_r * 200.0;
    wait_time   := wait_time_r * 1 ns;
    wait for wait_time;
    
    -- Unlock the Buffer
    txt_hw_cmd.unlock <= '1';
    mod_locked        <= false;
    wait until rising_edge(clk_sys);
    txt_hw_cmd.unlock <= '0';

    -- Before the next possible LOCK command, buffers must be evaluated.
    -- This takes up to 3 states of TX Arbitrator FSM. Note that this condition
    -- is always satisfied by CAN Core since between unlock and lock there must
    -- be 3 whole bit times of interframe space!    
    wait until rising_edge(clk_sys);
    wait until rising_edge(clk_sys);
    wait until rising_edge(clk_sys);
    wait until rising_edge(clk_sys);
    wait until rising_edge(clk_sys);
    
  end process;


  ------------------------------------------------------------------------------
  -- Compare DUT outputs with model outputs
  ------------------------------------------------------------------------------
  cmp_proc : process
  begin
    
    if ((mod_dlc_out            /= tran_dlc_out) or
        (mod_is_rtr             /= tran_is_rtr) or
        (mod_ident_type_out     /= tran_ident_type_out) or
        (mod_frame_type_out     /= tran_frame_type_out) or
        (mod_frame_valid_out    /= tran_frame_valid_out))
    then
        log("DUT and Model metadata not matching!", error_l, log_level);
        cmp_err_ctr          <= cmp_err_ctr + 1;
    end if;

    if (txt_hw_cmd_buf_index   /= mod_buf_index)
    then
        log("DUT and Model buffer index not matching!", error_l, log_level);
        cmp_err_ctr          <= cmp_err_ctr + 1;
    end if;

    wait until falling_edge(clk_sys);
  end process;


  -----------------------------------
  -- Clock and timestamp generation
  -----------------------------------
  clock_gen : process
  variable period   :natural := f100_Mhz;
  variable duty     :natural := 50;
  variable epsilon  :natural := 0;
  begin
    generate_clock(period, duty, epsilon, clk_sys);
    timestamp <= std_logic_vector(unsigned(timestamp) + 1);
  end process;
  
  errors <= error_ctr;

  ---------------------------------
  ---------------------------------
  -- Main Test process
  ---------------------------------
  ---------------------------------
  test_proc : process
    variable outcome : boolean;
  begin
    log("Restarting TX Arbitrator test!", info_l, log_level);
    wait for 5 ns;
    reset_test(res_n, status, run, error_ctr);
    log("Restarted TX Arbitrator test", info_l, log_level);
    print_test_info(iterations, log_level, error_beh, error_tol);
    
    -------------------------------
    --Main loop of the test
    -------------------------------
    log("Starting main loop", info_l, log_level);
    
    while (loop_ctr < iterations  or  exit_imm)
    loop
      log("Starting loop nr " & integer'image(loop_ctr), info_l, log_level);
     
      -- Configure TXT Buffer priorities!
	  set_priorities(rand_ctr_2, txt_buf_prio);

      wait for 1000 ns;
 
      error_ctr <= cmp_err_ctr;

      wait until rising_edge(clk_sys);

      loop_ctr <= loop_ctr + 1;
    end loop;

    evaluate_test(error_tol, error_ctr, status);
  end process;
  
  
end architecture;



--------------------------------------------------------------------------------
-- Test wrapper and control signals generator        
--------------------------------------------------------------------------------
architecture tx_arb_unit_test_wrapper of CAN_test_wrapper is
  
  -- Select architecture of the test
  for test_comp : CAN_test use entity work.CAN_test(tx_arb_unit_test);
  
	-- Input trigger, test starts running when true
    signal run              :   boolean;

	-- Status of the test
    signal status_int       :   test_status_type;

    -- Amount of errors which appeared in the test
    signal errors           :   natural;

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

