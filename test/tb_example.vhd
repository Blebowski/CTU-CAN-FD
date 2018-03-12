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
--    VUnit wrapper for sanity test.
--------------------------------------------------------------------------------
-- Revision History:
--    February 2018   First Implementation - Martin Jerabek
--------------------------------------------------------------------------------

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
library work;
use ieee.std_logic_1164.all;
USE work.CANtestLib.All;

entity tb_example is
  generic (
    runner_cfg : string;
    iterations : natural := 1;
    log_level  : log_lvl_type := info_l;
    error_beh  : err_beh_type := quit;           -- Test behaviour when error occurs: Quit, or Go on
    error_tol  : natural := 0                    -- Error tolerance, error counter should not
                                                 -- exceed this value in order for the test to pass
  );
end entity;

architecture tb of tb_example is
  --signal status : test_status_type;
  --signal do_run : boolean := false;

  signal t_sanity_errors   : natural;
  signal t_sanity_status   : test_status_type;
  signal t_sanity_run      : boolean;

  signal t_unit_bitstuf_errors   : natural;
  signal t_unit_bitstuf_status   : test_status_type;
  signal t_unit_bitstuf_run      : boolean;

  procedure run_test(
    variable errors : inout natural;
    signal do_run : out boolean;
    signal status : in test_status_type;
    signal t_errors : in natural
  ) is
  begin
    report "running";
    wait for 1 ns;
    do_run <= true;
    wait until status = passed or status = failed;
    report "Done";
    report to_string(t_errors);
    wait for 100 ns;
    do_run <= false;
    errors := errors + t_errors;
  end;
begin
  -- WARNING: bit_stuffing_unit_test is evil and reports failures even though
  --          it is not run
  -- Maybe this is partially caused by shared entity and signals defined
  -- in entity declaration?
  t_unit_bitstuf: entity work.CAN_test(bit_stuffing_unit_test)
    port map (
      iterations => iterations,
      log_level  => log_level,
      error_beh  => error_beh,
      error_tol  => error_tol,
      errors     => t_unit_bitstuf_errors,
      status     => t_unit_bitstuf_status,
      run        => t_unit_bitstuf_run
    );
--   t_sanity: entity work.CAN_test(sanity_test)
--     port map (
--       iterations => iterations,
--       log_level  => log_level,
--       error_beh  => error_beh,
--       error_tol  => error_tol,
--       errors     => t_sanity_errors,
--       status     => t_sanity_status,
--       run        => t_sanity_run
--     );
  main:process
    variable errors : natural := 0;
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("unit_bit_stuffing") then
        run_test(errors, t_unit_bitstuf_run, t_unit_bitstuf_status, t_sanity_errors);
      --elsif run("sanity_test") then
      --  run_test(errors, t_sanity_run, t_sanity_status, t_sanity_errors);
      end if;
    end loop;

    test_runner_cleanup(runner, errors > 0);
  end process;
end architecture;
