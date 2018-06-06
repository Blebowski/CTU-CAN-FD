-- Configurations
{% for test in tests %}

configuration tbconf_{{test}} of vunittb_wrapper is
for tb
    for i_test : CAN_test use entity work.CAN_test({{test}}); end for;
end for;
end configuration;
-- -----------------------------------------------------------------------------
{% endfor %}

-- Entities
{% for test in tests %}

library work;
use work.CANtestLib.all;
library vunit_lib;
context vunit_lib.vunit_context;

entity tb_{{test}} is generic (
    runner_cfg : string := runner_cfg_default;
    iterations : natural := 1;
    log_level  : log_lvl_type := info_l;
    error_beh  : err_beh_type := quit;
    error_tol  : natural := 0;
    timeout    : string := "0 ms"
); end entity;
architecture tb of tb_{{test}} is
    component vunittb_wrapper is generic (
        nested_runner_cfg : string;
        iterations : natural;
        log_level  : log_lvl_type;
        error_beh  : err_beh_type;
        error_tol  : natural;
        timeout    : string
    ); end component;
    for all:vunittb_wrapper use configuration work.tbconf_{{test}};
begin
    tb:vunittb_wrapper generic map(
        nested_runner_cfg => runner_cfg,
        iterations        => iterations,
        log_level         => log_level,
        error_beh         => error_beh,
        error_tol         => error_tol,
        timeout           => timeout);
end architecture;
-- -----------------------------------------------------------------------------
{% endfor %}
