Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

package pkg_feature_exec_dispath is
    --Procedure for processing the feature tests!

    type feature_outputs_t is record
        outcome      : boolean;
    end record;

    type feature_signal_outputs_t is record
        bl_inject    : std_logic;
        bl_force     : boolean;
    end record;

    constant NINST : natural := 2;

    type instance_inputs_t is record
        drv_bus    : std_logic_vector(1023 downto 0);
        stat_bus   : std_logic_vector(511 downto 0);
        irq        : std_logic;
    end record;

    type instance_outputs_t is record
        hw_reset   : std_logic;
    end record;

    type instance_inputs_arr_t is array (1 to NINST) of instance_inputs_t;
    type instance_outputs_arr_t is array (1 to NINST) of instance_outputs_t;
    type mem_bus_arr_t is array (1 to NINST) of Avalon_mem_type;

    procedure exec_feature_test(
        --Common test parameters
        signal   test_name    : in     string;
        variable o            : out    feature_outputs_t;
        signal   so           : out    feature_signal_outputs_t;
        signal   rand_ctr     : inout  natural range 0 to RAND_POOL_SIZE;
        signal   iout         : in     instance_inputs_arr_t;
        signal   mem_bus      : inout  mem_bus_arr_t;
        signal   bus_level    : in     std_logic
    );
end package;
