Library ieee;
library vunit_lib;
library work;
context vunit_lib.vunit_context;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;


--Testbench packages
{% for test in tests %}
use work.{{ test }}_feature.All;
{% endfor %}


package body pkg_feature_exec_dispath is
--Procedure for processing the feature tests!
procedure exec_feature_test(
    --Common test parameters
    signal   test_name    : in     string;
    variable outcome      : inout  boolean;
    signal   rand_ctr     : inout  natural range 0 to RAND_POOL_SIZE;
    --Additional signals for tests
    --Pretty much everything can be read out of stat bus...
    signal   mem_bus_1    : inout  Avalon_mem_type;
    signal   mem_bus_2    : inout  Avalon_mem_type;
    signal   int_1        : in     std_logic;
    signal   int_2        : in     std_logic;
    signal   bus_level    : in     std_logic;
    signal   drv_bus_1    : in     std_logic_vector(1023 downto 0);
    signal   drv_bus_2    : in     std_logic_vector(1023 downto 0);
    signal   stat_bus_1   : in     std_logic_vector(511 downto 0);
    signal   stat_bus_2   : in     std_logic_vector(511 downto 0);
    signal   bl_inject    : inout  std_logic;
    signal   bl_force     : inout  boolean
) is
begin
    outcome:=false;

    -- TODO: generate this procedure
    if false then
    {% for test in tests %}
    elsif run("{{ test }}") then
        {{ test }}_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                 drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
    {% endfor %}
    end if;
end procedure;

end package body;
