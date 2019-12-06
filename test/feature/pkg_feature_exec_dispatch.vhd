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

context work.ctu_can_synth_context;

use work.CANtestLib.All;
use work.randomLib.All;

use STD.textio.all;
use IEEE.std_logic_textio.all;

package pkg_feature_exec_dispath is
    --Procedure for processing the feature tests!

    type feature_signal_outputs_t is record
        bl_inject           : std_logic;
        bl_force            : boolean;
        ftr_tb_trv_delay    : t_ftr_tx_delay;
        ts_preset           : std_logic_vector(2 downto 1);
        ts_preset_val       : std_logic_vector(63 downto 0);
    end record;

    constant NINST : natural := 2;

    type instance_outputs_t is record
        drv_bus    : std_logic_vector(1023 downto 0);
        stat_bus   : std_logic_vector(511 downto 0);
        irq        : std_logic;
        hw_reset   : std_logic;
        can_tx     : std_logic;
    end record;

    type instance_outputs_arr_t is array (1 to NINST) of instance_outputs_t;
    type mem_bus_arr_t is array (1 to NINST) of Avalon_mem_type;

    procedure exec_feature_test(
        --Common test parameters
        constant test_name    : in     string;
        signal   so           : out    feature_signal_outputs_t;
        signal   rand_ctr     : inout  natural range 0 to RAND_POOL_SIZE;
        signal   iout         : in     instance_outputs_arr_t;
        signal   mem_bus      : inout  mem_bus_arr_t;
        signal   bus_level    : in     std_logic
    );
end package;
