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

entity tb_sanity is
  generic (
    runner_cfg : string;
    log_level  : log_lvl_type := info_l;
    error_beh  : err_beh_type := quit;           -- Test behaviour when error occurs: Quit, or Go on
    error_tol  : natural := 0                    -- Error tolerance, error counter should not
                                                 -- exceed this value in order for the test to pass
  );
end entity;

architecture tb of tb_sanity is
  signal t_sanity_errors   : natural;
  signal t_sanity_status   : test_status_type;
  signal t_sanity_run      : boolean;

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

  function strtolen(n : natural; src : string) return string is
    variable s : string(1 to n) := (others => ' ');
  begin
    assert src'length <= n report "String too long." severity failure;
    s(src'range) := src;
    return s;
  end;

  type bus_length_type is array(1 to 6) of real;
  type config_t is record
    topology    : string (1 to 50);
    bus_length_v: bus_length_type;
    trv_del_v   : anat_nc_t;
    osc_tol_v   : anat_nc_t; -- epsilon_v

    -- Noise parameters
    nw_mean     : real;
    nw_var      : real;
    ng_mean     : real;
    ng_var      : real;

    -- Bit time config
    -- brp_nbt brp_dbt prop_nbt ph1_nbt ph2_nbt sjw_nbt prop_dbt ph1_dbt ph2_dbt sjw_dbt
    timing_config : bit_time_config_type;

    -- Name and iterations amount
    name        : string (1 to 50);
    niter       : natural;
  end record;

  function str_equal(a : string; b : string) return boolean is
    variable l : integer;
    variable r : integer;
  begin
    if a'left > b'left then
      l := a'left;
    else
      l := b'left;
    end if;
    if a'right < b'right then
      r := a'right;
    else
      r := b'right;
    end if;
    --report to_string(l) severity note;
    --report to_string(r) severity note;
    --report a(l to r) severity note;
    --report b(l to r) severity note;
    return a(l to r) = b(l to r);
  end function;

  function len_to_matrix(topology : string; l : bus_length_type)
    return bus_matrix_type is
    variable bm : bus_matrix_type;
  begin
    if str_equal(topology, "bus") then
      bm := ((0.0,            l(1),           l(1)+l(2),      l(1)+l(2)+l(3)),
             (l(1),           0.0,            l(2),           l(2)+l(3)),
             (l(1)+l(2),      l(2),           0.0,            l(3)),
             (l(1)+l(2)+l(3), l(2)+l(3),      l(3),           0.0));
    elsif str_equal(topology, "star") then
      bm := ((0.0,            l(1)+l(2),      l(1)+l(3),      l(1)+l(4)),
             (l(1)+l(2),      0.0,            l(2)+l(3),      l(2)+l(4)),
             (l(1)+l(3),      l(2)+l(3),      0.0,            l(3)+l(4)),
             (l(1)+l(4),      l(2)+l(4),      l(3)+l(4),      0.0));
    elsif str_equal(topology, "tree") then
      bm := ((0.0,            l(1)+l(2),      l(1)+l(3)+l(5), l(1)+l(4)+l(5)),
             (l(1)+l(2),      0.0,            l(2)+l(3)+l(5), l(2)+l(4)+l(5)),
             (l(1)+l(3)+l(5), l(2)+l(3)+l(5), 0.0,            l(3)+l(4)),
             (l(1)+l(4)+l(5), l(2)+l(4)+l(5), l(3)+l(4),      0.0));
    elsif str_equal(topology, "ring") then
      assert false report "Ring topology not implemented." severity failure;
      -- TODO: Ring topology with min functions
    elsif str_equal(topology, "custom") then
      bm := ((0.0,  l(1), l(2), l(3)),
             (l(1), 0.0,  l(4), l(5)),
             (l(2), l(4), 0.0,  l(6)),
             (l(3), l(6), l(6), 0.0));
    else
      assert false report "Invalid bus topology!" severity failure;
    end if;
    return bm;
  end len_to_matrix;

  signal config : config_t;
  signal bm : bus_matrix_type;

  -- ***** Configurations *****
  constant config1 : config_t := (
    topology      => strtolen(50, "star"),
    bus_length_v  => (10.0, 10.0, 10.0, 10.0, 0.0, 0.0),
    trv_del_v     => (10, 10, 10, 10),
    osc_tol_v     => (0, 5, 10, 15),
    nw_mean       => 70.0,
    nw_var        => 5.0,
    ng_mean       => 300000.0,
    ng_var        => 100000.0,
    timing_config => (4, 1, 8, 8, 8, 3, 3, 1, 5, 2),
    name          => strtolen(50, "1Mb/10Mb 20 m Star"),
    niter         => 5
  );
begin
  main:process
    variable errors : natural := 0;
    variable var_config : config_t;
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("1Mb/10Mb 20 m Star") then
        var_config := config1;
      end if;
      config <= var_config;
      bm <= len_to_matrix(var_config.topology, var_config.bus_length_v);
      run_test(errors, t_sanity_run, t_sanity_status, t_sanity_errors);
    end loop;

    test_runner_cleanup(runner, errors > 0);
  end process;

  t_sanity: entity work.sanity_test
    port map (
      iterations => config.niter,
      log_level  => log_level,
      error_beh  => error_beh,
      error_tol  => error_tol,
      errors     => t_sanity_errors,
      status     => t_sanity_status,
      run        => t_sanity_run,
      -- test params
      epsilon_v  => config.osc_tol_v,
      trv_del_v  => config.trv_del_v,
      bus_matrix => bm,
      iter_am    => config.niter,
      nw_mean    => config.nw_mean,
      nw_var     => config.nw_var,
      ng_mean    => config.ng_mean,
      ng_var     => config.ng_var,
      topology   => config.topology,
      timing_config => config.timing_config
    );

end architecture;
