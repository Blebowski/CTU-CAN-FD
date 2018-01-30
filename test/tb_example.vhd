library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
library work;
use ieee.std_logic_1164.all;
USE work.CANtestLib.All;

entity tb_example is
  generic (runner_cfg : string);
end entity;

architecture tb of tb_example is
  signal errors : natural;
  signal status : test_status_type;
  signal do_run    : boolean := false;
begin
  test: entity work.CAN_test(sanity_test)
        port map (
          status => status,
          errors => errors,
          run => do_run,
          iterations => 1,
          log_level => info_l,
          error_beh => quit,
          error_tol => 0
        );
  main:process
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop

      if run("test_pass") then
        report "running";
        wait for 1 ns;
        do_run <= true;
        wait until status = passed or status = failed;
        report "Done";
        report to_string(errors);
        wait for 100 ns;
        do_run <= false;
      end if;
    end loop;

    test_runner_cleanup(runner, errors > 0);
  end process;
end architecture;
