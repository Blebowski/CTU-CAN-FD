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
--  Glitch filtering during Bus idle feature test.
--
-- @Verifies:
--  @1. Device is able to filter out glitches shorter than duration of TSEG1
--      during Bus idle (it will not interpret them as SOF).
--
-- @Test sequence:
--  @1. Read bit timing config from DUT. Calculate duration of TSEG1.
--      Force CAN RX of DUT low for TSEG1-1 Tim quanta Release CAN TX,
--      and check that DUT did NOT turn receiver.
--  @2. Force CAN RX of DUT low for TSEG1 Time quanta. Release CAN TX and
--      check that DUT did turn receiver.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    12.01.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package glitch_filtering_ftest is
    procedure glitch_filtering_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body glitch_filtering_ftest is
    procedure glitch_filtering_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                               (OTHERS => '0');
        variable bus_timing         :     bit_time_config_type;
        variable tseg1              :     natural;
        variable tseg1_minus_1_tq   :     natural;
        variable stat_1             :     SW_status;
    begin

        -----------------------------------------------------------------------
        -- @1. Read bit timing config from DUT. Calculate duration of TSEG1.
        --     Force CAN RX of DUT low for TSEG1-1 clock cycles. Release CAN
        --     TX, and check that DUT did NOT turn receiver.
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        CAN_read_timing_v(bus_timing, DUT_NODE, chn);
        tseg1 := bus_timing.tq_nbt *
                    (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt);
        tseg1_minus_1_tq := bus_timing.tq_nbt *
                                (bus_timing.prop_nbt + bus_timing.ph1_nbt);

        force_can_rx(DOMINANT, DUT_NODE, chn);
        for i in 1 to tseg1_minus_1_tq loop
            clk_agent_wait_cycle(chn);
        end loop;
        release_can_rx(chn);
        wait for 50 ns;
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.receiver, "DUT not receiver!");
        check_m(stat_1.bus_status, "DUT Idle");
        
        -----------------------------------------------------------------------
        -- @1. Force CAN RX of DUT low for TSEG1 clock cycles. Release CAN
        --     TX and check that DUT did turn receiver.
        -----------------------------------------------------------------------
        info_m("Step 2");
        wait for 100 ns;
        
        force_can_rx(DOMINANT, DUT_NODE, chn);
        for i in 1 to tseg1 loop
            clk_agent_wait_cycle(chn);
        end loop;
        release_can_rx(chn);
        wait for 50 ns;
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.receiver, "DUT receiver!");
        check_false_m(stat_1.bus_status, "DUT not Idle");

        wait for 100 ns;

  end procedure;

end package body;
