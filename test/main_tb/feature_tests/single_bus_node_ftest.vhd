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
--  Single bus node test
--
-- @Verifies:
--  @1. Node which is single on bus (other node is not transmitting, acking,
--      sending error frames), will turn error passive and not bus-off when
--      trying to transmitt a frame!
--
-- @Test sequence:
--  @1. Disable Test node, disable retransmitt limit in DUT.
--  @2. Transmitt frame by DUT.
--  @3. Wait until error frame starts and check that error counter is incremented
--      by 8. Repeat until node turns error passive!
--  @4. Wait for several times that node transmitts a frame and check that after
--      each, TX Error counter is not incremented.
--
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    18.7.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package single_bus_node_ftest is
    procedure single_bus_node_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body single_bus_node_ftest is
    procedure single_bus_node_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable alc                :       natural;

        variable fault_state_1      :     SW_fault_state;
        
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_2            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        variable mode_1             :     SW_mode;

        -- Node status
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        
        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;
        
        variable err_counters       :     SW_error_counters;
        
        constant id_template        :     std_logic_vector(10 downto 0) :=
                "01010101010";
        variable id_var             :     std_logic_vector(10 downto 0) :=
                 (OTHERS => '0');
        variable retr_index         :     natural := 0;
        variable tec_at_err_passive :     natural;
        variable frame_sent         :     boolean := false;
    begin

        ------------------------------------------------------------------------
        -- @1. Disable Test node, disable retransmitt limit in DUT.
        ------------------------------------------------------------------------
        info_m("Step 1: Disabling Test node");

        CAN_turn_controller(false, TEST_NODE, chn);        
        CAN_enable_retr_limit(false, 0, DUT_NODE, chn);
        
        --get_core_mode(mode_1, DUT_NODE, chn);
        --mode_1.self_test := true;
        --set_core_mode(mode_1, DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @2. Transmitt frame by DUT.
        ------------------------------------------------------------------------
        info_m("Step 2: Transmit frame by DUT");

        CAN_generate_frame(frame_1);
        frame_1.rtr := NO_RTR_FRAME;
        frame_1.frame_format := NORMAL_CAN;
        frame_1.dlc := "0001";
        frame_1.data_length := 1;
        frame_1.identifier := 256;
        frame_1.data(0) := x"AA";      

        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);

        ------------------------------------------------------------------------
        -- @3.  Wait until error frame starts and check that error counter is
        --      incremented by 8. Repeat until node turns error passive!
        ------------------------------------------------------------------------
        info_m("Step 3: Looping till error passive");

        get_fault_state(fault_state_1, DUT_NODE, chn);
        while (fault_state_1 = fc_error_active) loop
            CAN_wait_error_frame(DUT_NODE, chn);
        
            -- Wait till error frame is for sure over
            for i in 0 to 13 loop
                CAN_wait_sample_point(DUT_NODE, chn);
            end loop;

            read_error_counters(err_counters, DUT_NODE, chn);
            retr_index := retr_index + 1;
            check_m(err_counters.tx_counter = retr_index * 8, "TEC incremented");
                        
            get_fault_state(fault_state_1, DUT_NODE, chn);
        end loop;

        ------------------------------------------------------------------------
        -- @4. Wait for several times that node transmitts a frame and check
        --     that after each, TX Error counter is not incremented.
        ------------------------------------------------------------------------
        info_m("Step 4: Checking TEC stays!");

        read_error_counters(err_counters, DUT_NODE, chn);
        tec_at_err_passive := err_counters.tx_counter;
        
        for i in 0 to 10 loop
            info_m("Frame in Error passive nr:" & integer'image(i));

            CAN_wait_error_frame(DUT_NODE, chn);
        
            -- Wait till error frame is for sure over
            for j in 0 to 13 loop
                CAN_wait_sample_point(DUT_NODE, chn);
            end loop;

            read_error_counters(err_counters, DUT_NODE, chn);
            check_m(err_counters.tx_counter = tec_at_err_passive, "TEC stable");
        end loop;

  end procedure;

end package body;
