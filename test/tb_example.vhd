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
