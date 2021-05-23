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
--  BTR (Bit timing register) - minimal settings feature test.
--
-- @Verifies:
--  @1. Node is able to send frame succesfully with maximal possible bit-rate
--      (nominal and data), as specified by datasheet.
--
-- @Test sequence:
--  @1. Configure highest possible bit-rate as given by datasheet in both nodes.
--      Disable Secondary sampling point.
--  @2. Configure small Transmitter delay, so that sampling in nominal bit rate
--      will not fire an error.
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

package btr_minimal_ftest is
    procedure btr_minimal_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body btr_minimal_ftest is
    procedure btr_minimal_ftest_exec(
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
        -- @1. Configure highest possible bit-rate as given by datasheet in
        --     both nodes. Disable Secondary sampling point.
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);

        bus_timing.prop_nbt := 1;
        bus_timing.ph1_nbt := 3;
        bus_timing.ph2_nbt := 3;
        bus_timing.sjw_nbt := 2;
        bus_timing.tq_nbt := 1;

        bus_timing.prop_dbt := 0;
        bus_timing.ph1_dbt := 2;
        bus_timing.ph2_dbt := 2;
        bus_timing.sjw_nbt := 1;
        bus_timing.tq_dbt := 1;

        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        CAN_configure_ssp(ssp_no_ssp, x"00", DUT_NODE, chn);
        CAN_configure_ssp(ssp_no_ssp, x"00", TEST_NODE, chn);

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
        -- @2. Configure small Transmitter delay, so that sampling in nominal
        --     bit rate will not fire an error.
        -----------------------------------------------------------------------
        info_m("Step 2");

        ftr_tb_set_tran_delay(1 ns, DUT_NODE, chn);
        ftr_tb_set_tran_delay(1 ns, TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @3. Generate random frame which is FD frame with bit-rate shift.
        --     Send the frame by DUT, receive it by Test node and check it.
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        CAN_generate_frame(CAN_frame_1);
        info_m("Generated frame");
        CAN_frame_1.frame_format := FD_CAN;
        CAN_frame_1.brs := BR_SHIFT;
        CAN_frame_1.rtr := NO_RTR_FRAME;
        
        CAN_send_frame(CAN_frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(TEST_NODE, chn);
        CAN_read_frame(CAN_frame_2, TEST_NODE, chn);
        
        CAN_compare_frames(CAN_frame_1, CAN_frame_2, false, frames_equal);
        check_m(frames_equal, "TX/RX frame equal!");

  end procedure;

end package body;