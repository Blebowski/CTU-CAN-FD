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
-- @TestInfoStart
--
-- @Purpose:
--  Glitch filtering during Bus idle feature test.
--
-- @Verifies:
--  @1. Device is able to filter out glitches shorter than duration of TSEG1
--      during Bus idle (it will not interpret them as SOF).
--
-- @Test sequence:
--  @1. Read bit timing config from Node 1. Calculate duration of TSEG1.
--      Force CAN RX of Node 1 low for TSEG1-1 Tim quanta Release CAN TX,
--      and check that Node 1 did NOT turn receiver.
--  @2. Force CAN RX of Node 1 low for TSEG1 Time quanta. Release CAN TX and
--      check that Node 1 did turn receiver.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    12.01.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package glitch_filtering_feature is
    procedure glitch_filtering_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body glitch_filtering_feature is
    procedure glitch_filtering_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is

        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                               (OTHERS => '0');
        variable bus_timing         :     bit_time_config_type;
        variable tseg1              :     natural;
        variable tseg1_minus_1_tq   :     natural;
        variable stat_1             :     SW_status;
    begin

        -----------------------------------------------------------------------
        -- @1. Read bit timing config from Node 1. Calculate duration of TSEG1.
        --     Force CAN RX of Node 1 low for TSEG1-1 clock cycles. Release CAN
        --     TX, and check that Node 1 did NOT turn receiver.
        -----------------------------------------------------------------------
        info("Step 1");
        
        CAN_read_timing_v(bus_timing, ID_1, mem_bus(1));
        tseg1 := bus_timing.tq_nbt *
                    (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt);
        tseg1_minus_1_tq := bus_timing.tq_nbt *
                                (bus_timing.prop_nbt + bus_timing.ph1_nbt);

        force_can_rx(DOMINANT, ID_1, so.crx_force, so.crx_inject, so.crx_index);
        for i in 1 to tseg1_minus_1_tq loop
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;
        release_can_rx(so.crx_force);
        wait for 50 ns;
        
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check_false(stat_1.receiver, "Node 1 not receiver!");
        check(stat_1.bus_status, "Node 1 Idle");
        
        -----------------------------------------------------------------------
        -- @1. Force CAN RX of Node 1 low for TSEG1 clock cycles. Release CAN
        --     TX and check that Node 1 did turn receiver.
        -----------------------------------------------------------------------
        info("Step 2");
        wait for 100 ns;
        
        force_can_rx(DOMINANT, ID_1, so.crx_force, so.crx_inject, so.crx_index);
        for i in 1 to tseg1 loop
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;
        release_can_rx(so.crx_force);
        wait for 50 ns;
        
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.receiver, "Node 1 receiver!");
        check_false(stat_1.bus_status, "Node 1 not Idle");

        wait for 100 ns;

  end procedure;

end package body;
