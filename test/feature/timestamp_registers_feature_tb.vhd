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
--  Feature test for Timestamp options on RX frame in RX Buffer!
--
--------------------------------------------------------------------------------
-- Revision History:
--
--    29.6.2018     Created file 
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package timestamp_registers_feature is
    procedure timestamp_registers_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
	);
end package;


package body timestamp_registers_feature is
    procedure timestamp_registers_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :        std_logic_vector(31 downto 0) :=
                                                 (OTHERS => '0');
        variable ID_1               :        natural := 1;
        variable diff               :        unsigned(63 downto 0);

        variable ts_input           :        std_logic_vector(63 downto 0);
        variable ts_read            :        std_logic_vector(63 downto 0);
    begin
        o.outcome := true;

        wait for 1000 ns;

        for i in 0 to 50 loop
            -------------------------------------------------------------------
            -- Read and save timestamp from registers
            -------------------------------------------------------------------
            ts_input := iout(1).stat_bus(STAT_TS_HIGH downto STAT_TS_LOW);

            -------------------------------------------------------------------
            -- Read timestamp with lib function
            -------------------------------------------------------------------
            CAN_read_timestamp(ts_read, ID_1, mem_bus(1));

            -------------------------------------------------------------------
            -- Compare both values
            -------------------------------------------------------------------
            info("Timestamp input: 0x" & to_hstring(ts_input));
            info("Timestamp read: 0x" & to_hstring(ts_read));

            -------------------------------------------------------------------
            -- Substract Read timestamp from Input timestamp. Input timestamp is
            -- stored first, thus it will be always lower than read timestamp 
            -- (memory access lasts one clock cycle). When substracting in this
            -- order, we avoid underflow on "unsigned" data type, which is
            -- defined from 0.
            -------------------------------------------------------------------
            diff := unsigned(ts_read) - unsigned(ts_input);

            check(to_integer(diff) < 10, "Timestamp difference is too big! " & 
                  "Difference " & integer'image(to_integer(diff)));
        end loop;

    end procedure;

end package body;
