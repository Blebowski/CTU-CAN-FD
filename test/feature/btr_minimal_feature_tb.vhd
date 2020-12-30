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
--      frame by Node 1, receive it by Node 2 and check it.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--   26.12.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package btr_minimal_feature is
    procedure btr_minimal_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body btr_minimal_feature is
    procedure btr_minimal_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
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
        info("Step 1");
        CAN_turn_controller(false, ID_1, mem_bus(1));
        CAN_turn_controller(false, ID_2, mem_bus(2));

        bus_timing.prop_nbt := 3;
        bus_timing.ph1_nbt := 2;
        bus_timing.ph2_nbt := 2;
        bus_timing.sjw_nbt := 2;
        bus_timing.tq_nbt := 1;
        
        bus_timing.prop_dbt := 0;
        bus_timing.ph1_dbt := 2;
        bus_timing.ph2_dbt := 2;
        bus_timing.sjw_nbt := 1;
        bus_timing.tq_dbt := 1;

        CAN_configure_timing(bus_timing, ID_1, mem_bus(1));
        CAN_configure_timing(bus_timing, ID_2, mem_bus(2));

        CAN_configure_ssp(ssp_no_ssp, x"00", ID_1, mem_bus(1));
        CAN_configure_ssp(ssp_no_ssp, x"00", ID_2, mem_bus(2));

        CAN_turn_controller(true, ID_1, mem_bus(1));
        CAN_turn_controller(true, ID_2, mem_bus(2));

        CAN_wait_bus_on(ID_1, mem_bus(1));
        CAN_wait_bus_on(ID_2, mem_bus(2));

        info("CAN bus nominal bit-rate:");
        info("PROP: " & integer'image(bus_timing.prop_nbt));
        info("PH1: " & integer'image(bus_timing.ph1_nbt));
        info("PH2: " & integer'image(bus_timing.ph2_nbt));
        info("SJW: " & integer'image(bus_timing.sjw_nbt));

        info("CAN bus data bit-rate:");
        info("PROP: " & integer'image(bus_timing.prop_dbt));
        info("PH1: " & integer'image(bus_timing.ph1_dbt));
        info("PH2: " & integer'image(bus_timing.ph2_dbt));
        info("SJW: " & integer'image(bus_timing.sjw_dbt));

        -----------------------------------------------------------------------
        -- @2. Configure small Transmitter delay, so that sampling in nominal
        --     bit rate will not fire an error.
        -----------------------------------------------------------------------
        info("Step 2");

        ftr_tb_set_tran_delay(1 ns, ID_1, so.ftr_tb_trv_delay);
        ftr_tb_set_tran_delay(1 ns, ID_2, so.ftr_tb_trv_delay);

        -----------------------------------------------------------------------
        -- @3. Generate random frame which is FD frame with bit-rate shift.
        --     Send the frame by Node 1, receive it by Node 2 and check it.
        -----------------------------------------------------------------------
        info("Step 3");
        
        CAN_generate_frame(rand_ctr, CAN_frame_1);
        info("Generated frame");
        CAN_frame_1.frame_format := FD_CAN;
        CAN_frame_1.brs := BR_SHIFT;
        CAN_frame_1.rtr := NO_RTR_FRAME;
        
        CAN_send_frame(CAN_frame_1, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_2, mem_bus(2));
        CAN_read_frame(CAN_frame_2, ID_2, mem_bus(2));
        
        CAN_compare_frames(CAN_frame_1, CAN_frame_2, false, frames_equal);
        check(frames_equal, "TX/RX frame equal!");

  end procedure;

end package body;