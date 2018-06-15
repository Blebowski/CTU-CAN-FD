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
    constant test_name    : in     string;
    variable o            : out    feature_outputs_t;
    signal   so           : out    feature_signal_outputs_t;
    signal   rand_ctr     : inout  natural range 0 to RAND_POOL_SIZE;
    signal   iout         : in     instance_inputs_arr_t;
    signal   mem_bus      : inout  mem_bus_arr_t;
    signal   bus_level    : in     std_logic
) is
begin
    o.outcome := false;

    if false then
    {% for test in tests %}
    elsif str_equal(test_name, "{{ test }}") then
        {{ test }}_feature_exec(
            o => o,
            rand_ctr => rand_ctr,
            mem_bus => mem_bus,
            iout => iout,
            bus_level => bus_level,
            so => so
        );
    {% endfor %}
    end if;
end procedure;

end package body;
