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
--  Data length Code CAN 2.0 more than 8 bytes feature test.
--
-- @Verifies:
--  @1. When transmission of CAN 2.0 frame with DLC higher than 8 is requested,
--      only 8 bytes are transmitted! 
--
-- @Test sequence:
--   @1. Generate CAN 2.0 Frame and set DLC higher than 8. Set higher data
--       bytes accordingly!
--   @2. Send the CAN Frame via Node 1. Monitor the bus and check that only
--       8 bytes are sent!
--   @3. Verify that frame received by Node 2, has the same DLC, but is has
--       received only 8 bytes of Data!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    14.7.2018   Created file
--   21.10.2018   Add check monitoring data field length.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package dlc_can20_8_64_bytes_feature is
    procedure dlc_can20_8_64_bytes_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body dlc_can20_8_64_bytes_feature is
    procedure dlc_can20_8_64_bytes_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        constant ID_1               :        natural := 1;
        constant ID_2               :        natural := 2;
        variable CAN_frame          :        SW_CAN_frame_type;
        variable CAN_frame_2        :        SW_CAN_frame_type  := 
                    (0, (OTHERS => (OTHERS => '0')), "0000", 0, '0', '0',
                     '0', '0', '0', (OTHERS => '0'), 0);
        variable frame_sent         :        boolean;
        variable pc_dbg             :        SW_PC_Debug;
    begin

        ------------------------------------------------------------------------
        -- @1. Generate CAN 2.0 Frame and set DLC higher than 8. Set higher data
        --    bytes accordingly!
        ------------------------------------------------------------------------
        info("Step 1: Generate frame");
        CAN_generate_frame(rand_ctr, CAN_frame);
        rand_logic_vect_v(rand_ctr, CAN_frame.dlc, 0.5);
        -- Set highest bit to 1 -> DLC will be more than 8!
        CAN_frame.dlc(3) := '1';
        CAN_frame.rtr := NO_RTR_FRAME;
        CAN_frame.frame_format := NORMAL_CAN;
        decode_dlc(CAN_frame.dlc, CAN_frame.data_length);
        for i in 0 to CAN_frame.data_length - 1 loop
            rand_logic_vect_v(rand_ctr, CAN_frame.data(i), 0.5);
        end loop;

        ------------------------------------------------------------------------
        -- @2. Send the CAN Frame via Node 1. Monitor the bus and check that only
        --    8 bytes are sent!
        ------------------------------------------------------------------------
        info("Step 2: Send frame");
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_data, ID_1, mem_bus(1));
        for i in 0 to 63 loop
            CAN_wait_sample_point(iout(1).stat_bus);
            wait for 11 ns; -- for DFF to flip
            
            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            
            if (i = 63) then
                check_false(pc_dbg = pc_deb_data,
                    "After 64 bytes data field ended!");
            else
                check(pc_dbg = pc_deb_data,
                    "Before 64 bytes data field goes on!");
            end if;
        end loop;
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        wait for 500 ns;

        ------------------------------------------------------------------------
        -- @3. Verify that frame received by Node 2, has the same DLC, but is has
        --    received only 8 bytes of Data!
        ------------------------------------------------------------------------
        info("Step 3: Check frame received!");
        CAN_read_frame(CAN_frame_2, ID_2, mem_bus(2));
        check(CAN_frame_2.dlc = CAN_frame.dlc, "Invalid DLC received!");
        check(CAN_frame_2.rwcnt = 5, "Invalid DLC received!");
          
        for i in 8 to 63 loop
            check(CAN_frame_2.data(i) = "00000000",
                  "Byte index " & integer'image(i) & " not zero!");
        end loop;

  end procedure;

end package body;
