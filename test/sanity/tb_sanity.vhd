--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
-- 
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <martin.jerabek01@gmail.com>
-- 
-- Project advisors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 
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

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

package mypkg is
    type bus_length_type is array(1 to 6) of real;
    subtype natural_vector is anat_t;
end package;

library vunit_lib;
context vunit_lib.vunit_context;

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;
use ctu_can_fd_tb.mypkg.all;

entity tb_sanity is
    generic (
        -- ghdl accepts only string, integral and enum top-level generics
        runner_cfg    : string := runner_cfg_default;
        log_level     : log_lvl_type := info_l;

        -- Test behaviour when error occurs: Quit, or Go on
        error_beh     : err_beh_type := quit;
        -- Error tolerance, error counter should not exceed this value
        -- in order for the test to pass
        error_tol     : natural := 0;
        -- Timeout in simulation time. 0 means no limit.
        timeout       : string := "0 ms";

        seed          : natural := 0;

        topology      : string;
        bus_len_v     : string; --bus_length_type;
        trv_del_v     : string; --anat_nc_t;
        osc_tol_v     : string; --anat_nc_t;

        -- Noise parameters
        nw_mean       : string; -- real;
        nw_var        : string; -- real;
        ng_mean       : string; -- real;
        ng_var        : string; -- real;

        gauss_iter    : natural := 40;

        -- brp_nbt brp_dbt prop_nbt ph1_nbt ph2_nbt sjw_nbt prop_dbt ph1_dbt ph2_dbt sjw_dbt
        timing_config : string; --bit_time_config_type;
        iterations    : natural
    );
end entity;
architecture tb of tb_sanity is
    impure function decode_real_vec(s : string) return real_vector is
        variable parts : lines_t := split(s(s'low+1 to s'high-1), ", ");
        variable return_value : real_vector(parts'range);
    begin
        for i in parts'range loop
            return_value(i) := real'value(parts(i).all);
        end loop;
        return return_value;
    end function decode_real_vec;

    impure function decode_integer_vec(s : string) return integer_vector is
        variable parts : lines_t := split(s(s'low+1 to s'high-1), ", ");
        variable return_value : integer_vector(parts'range);
    begin
        for i in parts'range loop
            return_value(i) := integer'value(parts(i).all);
        end loop;
        return return_value;
    end function decode_integer_vec;

    impure function decode_natural_vec(s : string) return natural_vector is
        variable parts : lines_t := split(s(s'low+1 to s'high-1), ", ");
        variable return_value : natural_vector(parts'range);
    begin
        for i in parts'range loop
            return_value(i) := natural'value(parts(i).all);
        end loop;
        return return_value;
    end function decode_natural_vec;

    function to_bit_time_config_type(v : natural_vector)
      return bit_time_config_type is
        variable ret : bit_time_config_type;
    begin
        ret.tq_nbt   := v(0);
        ret.tq_dbt   := v(1);
        ret.prop_nbt := v(2);
        ret.ph1_nbt  := v(3);
        ret.ph2_nbt  := v(4);
        ret.sjw_nbt  := v(5);
        ret.prop_dbt := v(6);
        ret.ph1_dbt  := v(7);
        ret.ph2_dbt  := v(8);
        ret.sjw_dbt  := v(9);
        return ret;
    end function to_bit_time_config_type;

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
            -- LCOV_EXCL_START
            assert false report "Invalid bus topology!" severity failure;
            -- LCOV_EXCL_STOP
        end if;
        return bm;
    end len_to_matrix;

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
    end procedure run_test;

    constant bm : bus_matrix_type
        := len_to_matrix(topology, bus_length_type(decode_real_vec(bus_len_v)));
    constant epsilon_v : epsilon_type
        := epsilon_type(decode_integer_vec(osc_tol_v));
    constant decoded_trv_del_v : trv_del_type
        := trv_del_type(decode_integer_vec(trv_del_v));
    constant decoded_timing_config : bit_time_config_type
        := to_bit_time_config_type(decode_natural_vec(timing_config));
    constant padded_topology : string(1 to 50) := strtolen(50, topology);
    constant decoded_ng_mean : real := real'value(ng_mean);
    constant decoded_ng_var  : real := real'value(ng_var);
    constant decoded_nw_mean : real := real'value(nw_mean);
    constant decoded_nw_var  : real := real'value(nw_var);

    signal t_sanity_errors   : natural;
    signal t_sanity_status   : test_status_type;
    signal t_sanity_run      : boolean;
begin
    main:process
        variable errors : natural := 0;
    begin
        test_runner_setup(runner, runner_cfg);
        run_test(errors, t_sanity_run, t_sanity_status, t_sanity_errors);
        test_runner_cleanup(runner, errors > 0);
    end process;

    watchdog: if time'value(timeout) > 0 ns generate
        test_runner_watchdog(runner, time'value(timeout));
    end generate;

    t_sanity: entity work.sanity_test
    generic map (
        seed       => seed,
        iterations => iterations,
        log_level  => log_level,
        error_beh  => error_beh,
        error_tol  => error_tol,
        -- test params
        epsilon_v  => epsilon_v,
        trv_del_v  => decoded_trv_del_v,
        bus_matrix => bm,
        iter_am    => gauss_iter, -- gauss iteration count
        nw_mean    => decoded_nw_mean,
        nw_var     => decoded_nw_var,
        ng_mean    => decoded_ng_mean,
        ng_var     => decoded_ng_var,
        topology   => padded_topology,
        timing_config => decoded_timing_config
    )
    port map (
        errors     => t_sanity_errors,
        status     => t_sanity_status,
        run        => t_sanity_run
    );
end architecture;
