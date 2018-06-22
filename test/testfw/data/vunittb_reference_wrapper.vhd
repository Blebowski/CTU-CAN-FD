library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
library work;
use ieee.std_logic_1164.all;
USE work.CANtestLib.All;

{% for data_set in data_sets %}

entity vunittb_reference_wrapper_{{data_set}} is
    generic (
        nested_runner_cfg   : string;
        iterations          : natural := 50;
        log_level           : log_lvl_type := info_l;

        -- Test behaviour when error occurs: Quit, or Go on
        error_beh           : err_beh_type := quit;

        -- Error tolerance, error counter should not exceed this value
        -- in order for the test to pass
        error_tol           : natural := 0;

        -- Timeout in simulation time. 0 means no limit
        timeout             : string := "0 ms";

        seed                : natural := 0;

        path                : string := " ";
    );
end entity;

architecture tb of vunittb_reference_wrapper_{{data_set}} is
    signal t_errors         : natural := 0;
    signal t_status         : test_status_type;
    signal t_run            : boolean;
begin

    ----------------------------------------------------------------------------
    -- Test component 
    ----------------------------------------------------------------------------
    i_test : CAN_reference_test
        generic map (
            seed        => seed,
            config_path => config_path
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


    ----------------------------------------------------------------------------
    -- Test driver
    ----------------------------------------------------------------------------
    main : process
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

    
    ----------------------------------------------------------------------------
    -- Timeout watchdog
    ----------------------------------------------------------------------------
    watchdog : if time'value(timeout) > 0 ns generate
        test_runner_watchdog(runner, time'value(timeout));
    end generate;

end architecture;
-- -----------------------------------------------------------------------------
{% endfor %}
