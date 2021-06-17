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
--  BTR (Bit timing register) - maximal settings feature test.
--
-- @Verifies:
--  @1. Node is able to send frame succesfully with minimal possible bit-rate
--      (nominal and data), as specified by datasheet.
--
-- @Test sequence:
--  @1. Configure slowest possible bit-rate as given by datasheet in both nodes.
--      Disable Secondary sampling point.
--  @3. Generate random frame which is FD frame with bit-rate shift. Send the
--      frame by DUT, receive it by Test node and check it.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--   26.12.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package btr_maximal_ftest is
    procedure btr_maximal_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body btr_maximal_ftest is
    procedure btr_maximal_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame_1        :       SW_CAN_frame_type;
        variable CAN_frame_2        :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        
        variable bus_timing         :       bit_time_config_type;

        variable clock_per_bit      :       natural := 0;

        variable clock_meas         :       natural := 0;
        variable frames_equal       :       boolean;
        
        variable tx_delay           :       time;
    begin

        -----------------------------------------------------------------------
        -- @1. Configure lowest possible bit-rate as given by datasheet in
        --     both nodes. Disable Secondary sampling point.
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);

        bus_timing.prop_nbt := 127;
        bus_timing.ph1_nbt := 63;
        bus_timing.ph2_nbt := 63;
        bus_timing.sjw_nbt := 5;
        bus_timing.tq_nbt := 255;

        bus_timing.prop_dbt := 63;
        bus_timing.ph1_dbt := 31;
        bus_timing.ph2_dbt := 31;
        bus_timing.sjw_nbt := 5;
        bus_timing.tq_dbt := 255;

        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);

        info_m("CAN bus nominal bit-rate:");
        info_m("PROP: " & integer'image(bus_timing.prop_nbt));
        info_m("PH1: " & integer'image(bus_timing.ph1_nbt));
        info_m("PH2: " & integer'image(bus_timing.ph2_nbt));
        info_m("SJW: " & integer'image(bus_timing.sjw_nbt));

        info_m("CAN bus data bit-rate:");
        info_m("PROP: " & integer'image(bus_timing.prop_dbt));
        info_m("PH1: " & integer'image(bus_timing.ph1_dbt));
        info_m("PH2: " & integer'image(bus_timing.ph2_dbt));
        info_m("SJW: " & integer'image(bus_timing.sjw_dbt));

        -----------------------------------------------------------------------
        -- @2. Generate random frame which is FD frame with bit-rate shift.
        --     Send the frame by DUT, receive it by Test node and check it.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        CAN_generate_frame(CAN_frame_1);
        info_m("Generated frame");
        -- Make frame as short as possible not to have too long test time.
        CAN_frame_1.frame_format := FD_CAN;
        CAN_frame_1.brs := BR_SHIFT;
        CAN_frame_1.rtr := NO_RTR_FRAME;
        CAN_frame_1.ident_type := BASE;
        CAN_frame_1.data_length := 0;
        CAN_frame_1.identifier := CAN_frame_1.identifier mod (2 ** 11);
        CAN_frame_1.dlc := "0000";
        decode_dlc_rx_buff(CAN_frame_1.dlc, CAN_frame_1.rwcnt);
        
        CAN_send_frame(CAN_frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(TEST_NODE, chn);
        CAN_read_frame(CAN_frame_2, TEST_NODE, chn);
        
        CAN_compare_frames(CAN_frame_1, CAN_frame_2, false, frames_equal);
        check_m(frames_equal, "TX/RX frame equal!");

  end procedure;

end package body;