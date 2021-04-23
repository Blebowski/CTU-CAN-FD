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
--  @Purpose:
--    Reference test agent. Executes reference tests.
--
--    Reference test agent is started by test controller agent, when test_type
--    is configured to "reference". Reference test agent resets DUT, configures
--    it, enables it and runs reference test sequence based on selected data
--    set.
--
--    Reference test agent uses CAN agent to apply reference test sequence to
--    DUT and Memory bus agent to access DUT.
--
--    Reference tests are ALWAYS run on 2 MBit/500 KBit with 80 % sample point!
--
--------------------------------------------------------------------------------
-- Revision History:
--    09.4.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_agents_context;

use ctu_can_fd_tb.reference_data_set_1.all;
use ctu_can_fd_tb.reference_data_set_2.all;
use ctu_can_fd_tb.reference_data_set_3.all;
use ctu_can_fd_tb.reference_data_set_4.all;
use ctu_can_fd_tb.reference_data_set_5.all;
use ctu_can_fd_tb.reference_data_set_6.all;
use ctu_can_fd_tb.reference_data_set_7.all;
use ctu_can_fd_tb.reference_data_set_8.all;
use ctu_can_fd_tb.reference_data_set_9.all;
use ctu_can_fd_tb.reference_data_set_10.all;


entity reference_test_agent is
    generic(
        -- Test details
        test_name               : string;
        test_type               : string;
        stand_alone_vip_mode    : boolean;
        reference_iterations    : natural range 1 to 1000
    );
end entity;


architecture tb of reference_test_agent is

begin
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Test control process
    --
    -- Waits on start request from Test controller agent and runs a test.
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    test_process : process
        -- 2 Mbit / 500 Kbit, 80 % sample point
        variable bus_timing     : bit_time_config_type := 
            (2, 1, 40, 39, 20, 10, 20, 14, 15, 10);
        variable data_set : t_reference_data_set;
        variable driver_item : t_can_driver_entry :=
            ('0', 0 ns, false, (OTHERS => '0'));
        variable rx_frame : SW_CAN_frame_type;
        variable result : boolean;
        variable reference_offset : natural;
    begin
        wait until reference_start = '1';

        -- Pre-set test to be "passed", any error will make it fail
        ctu_vip_test_result.set_result(true);

        -- Configure bit timing
        CAN_configure_timing(bus_timing, DUT_NODE, default_channel);

        -- Enable CAN controllers
        CAN_turn_controller(true, DUT_NODE, default_channel);
        info_m("DUT is ON");
        
        -- Wait till integration is over in both nodes
        CAN_wait_bus_on(DUT_NODE, default_channel);
        info_m("Bus integration finished");

        if (test_name = "data_set_1") then
            data_set := C_reference_data_set_1;
        elsif (test_name = "data_set_2") then
            data_set := C_reference_data_set_2;
        elsif (test_name = "data_set_3") then
            data_set := C_reference_data_set_3;
        elsif (test_name = "data_set_4") then
            data_set := C_reference_data_set_4;
        elsif (test_name = "data_set_5") then
            data_set := C_reference_data_set_5;
        elsif (test_name = "data_set_6") then
            data_set := C_reference_data_set_6;
        elsif (test_name = "data_set_7") then
            data_set := C_reference_data_set_7;
        elsif (test_name = "data_set_8") then
            data_set := C_reference_data_set_8;
        elsif (test_name = "data_set_9") then
            data_set := C_reference_data_set_9;
        elsif (test_name = "data_set_10") then
            data_set := C_reference_data_set_10;
        else
            error_m("Invalid reference test data set");
        end if;

        -----------------------------------------------------------------------
        -- Test sequence itself
        -----------------------------------------------------------------------
        rand_int_v(899, reference_offset);

        for frame_index in reference_offset to reference_offset + reference_iterations loop
            
            info_m("Testing frame nr: " & integer'image(frame_index - reference_offset));
            info_m("Frame position in dataset: " & integer'image(frame_index));

            info_m("Pushing frame to CAN agent...");
            can_agent_driver_flush(default_channel);
            for seq_ind in 1 to data_set(frame_index).seq_len loop
                
                driver_item.value := data_set(frame_index).seq(seq_ind).value;
                driver_item.drive_time := data_set(frame_index).seq(seq_ind).drive_time;
                can_agent_driver_push_item(default_channel, driver_item);
            end loop;

            info_m("Running CAN Agent driver...");
            can_agent_driver_start(default_channel);
            can_agent_driver_wait_finish(default_channel);

            info_m("Reading out CAN frame...");
            CAN_read_frame(rx_frame, DUT_NODE, default_channel);
            
            info_m("Comparing received vs golden frame...");
            -- Pre-calculate RWCNT of TX frame
            decode_dlc_rx_buff(data_set(frame_index).frame.dlc, data_set(frame_index).frame.rwcnt);
            CAN_compare_frames(rx_frame, data_set(frame_index).frame, false, result);
            
            check_m(result, "Frames equal");
        end loop; 

        -- Signal test is done.
        reference_result <= ctu_vip_test_result.get_result;
        wait for 0 ns;
        reference_done <= '1';
        wait until reference_start = '0';
        reference_done <= '0';
        wait for 0 ns;

    end process;

end architecture;