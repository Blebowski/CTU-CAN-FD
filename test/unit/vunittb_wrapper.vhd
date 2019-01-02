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

library vunit_lib;
context vunit_lib.vunit_context;

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

entity vunittb_wrapper is
    generic (
        nested_runner_cfg : string;
        iterations : natural := 50;
        log_level  : log_lvl_type := info_l;

        -- Test behaviour when error occurs: Quit, or Go on
        error_beh  : err_beh_type := quit;

        -- Error tolerance, error counter should not exceed this value
        -- in order for the test to pass
        error_tol  : natural := 0;

        -- Timeout in simulation time. 0 means no limit
        timeout    : string := "0 ms";

        seed       : natural := 0
    );
end entity;

architecture tb of vunittb_wrapper is
    signal t_errors   : natural := 0;
    signal t_status   : test_status_type;
    signal t_run      : boolean;
begin
    i_test: CAN_test
        generic map (
            seed => seed
        )
        port map (
            iterations => iterations,
            log_level  => log_level,
            error_beh  => error_beh,
            error_tol  => error_tol,
            errors     => t_errors,
            status     => t_status,
            run        => t_run
        );
    main:process
    begin
        test_runner_setup(runner, nested_runner_cfg);
        while test_suite loop
            if run("all") then
                t_run <= true;
                wait until t_status = passed or t_status = failed;
                report "Done";
                report to_string(t_errors);
                wait for 100 ns;
                t_run <= false;
            end if;
        end loop;
        test_runner_cleanup(runner, t_errors > error_tol);
    end process;

    watchdog: if time'value(timeout) > 0 ns generate
        test_runner_watchdog(runner, time'value(timeout));
    end generate;

end architecture;
