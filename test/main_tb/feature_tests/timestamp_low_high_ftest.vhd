--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2021-present Ondrej Ille
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- educational, research, evaluation, self-interest purposes. Using the
-- Component for commercial purposes is forbidden unless previously agreed with
-- Copyright holder.
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
-- -------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2015-2020 MIT License
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
--  Feature test for Timestamp options on RX frame in RX Buffer!
--
-- @Verifies:
--  @1. TIMESTAMP_LOW and TIMESTAMP_HIGH registers functionality.
--
-- @Test sequence:
--  @1. Preset Timestamp value in TB. Read values from TIMESTAMP_LOW and
--      TIMESTAMP_HIGH registers. Check read value matches value which was
--      preset.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    29.6.2018     Created file
--   22.11.2019     Add support for timestamp randomization! 
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.timestamp_agent_pkg.all;

package timestamp_low_high_ftest is
    procedure timestamp_low_high_ftest_exec(
        signal      chn             : inout  t_com_channel
	);
end package;


package body timestamp_low_high_ftest is
    procedure timestamp_low_high_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable diff               :        unsigned(63 downto 0);

        variable ts_input           :        std_logic_vector(63 downto 0);
        variable ts_read            :        std_logic_vector(63 downto 0);
        
        variable ts_rand            :        std_logic_vector(63 downto 0);
    begin

        -----------------------------------------------------------------------
        -- @1. Preset Timestamp value in TB. Read values from TIMESTAMP_LOW and
        --    TIMESTAMP_HIGH registers. Check read value matches value which
        --    was preset.
        -----------------------------------------------------------------------
        info_m("Step 1");

        -- Force random timestamp so that we are sure that both words of the
        -- timestamp are clocked properly!
        rand_logic_vect_v(ts_rand, 0.5);
        
        -- Keep highest bit 0 to avoid complete overflow during the test!
        ts_rand(63) := '0';

        -- Additionally, keep bit 31=0. This is because timestamp is internally
        -- in TB implemented from two naturals which are 0 .. 2^31 - 1. If we
        -- would generate bit 31=1, conversion "to_integer" from such unsigned
        -- value is out of scope of natural!
        ts_rand(31) := '0';

        info_m("Forcing start timestamp in DUT to: " & to_hstring(ts_rand));
        ftr_tb_set_timestamp(ts_rand, chn);

        wait for 100 ns;

        for i in 0 to 50 loop
            -- Get timestamp from Timestamp agent
            timestamp_agent_get_timestamp(chn, ts_input);

            -- Get timestamp from DUT
            CAN_read_timestamp(ts_read, DUT_NODE, chn);

            -------------------------------------------------------------------
            -- Compare both values
            -------------------------------------------------------------------
            -- Calculate difference. Two separate cases are needed to avoid
            -- underflow
            if (ts_input > ts_read) then
                diff := unsigned(ts_input) - unsigned(ts_read);
            else
                diff := unsigned(ts_read) - unsigned(ts_input);
            end if;

            info_m("Timestamp input: 0x" & to_hstring(ts_input));
            info_m("Timestamp read: 0x" & to_hstring(ts_read));

            check_m(to_integer(diff) < 10, "Timestamp difference is too big! " & 
                  "Difference " & integer'image(to_integer(diff)));

            wait for 100 ns;

        end loop;

    end procedure;

end package body;
