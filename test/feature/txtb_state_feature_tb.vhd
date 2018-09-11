--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Project advisors and co-authors:
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--  Buffer operation feature test. Verifies states of TXT Buffer and application
--  of "set_ready", "set_empty" and "set_abort" commands on TXT Buffer.
--
--  Test sequence:
--      Part 1 (set_empty):
--          1. Generate CAN Frame and insert to Node 1 for transmission.
--          2. Wait till transmission starts. Check that TXT Buffer is in
--             "TX in progress".
--          3. Wait till transmission ends. Check that TXT Buffer is in "Done".
--          4. Send "set_empty" command.
--          5. Verify that TXT Buffer ends up "Empty" state!
--      Part 2 (set_abort):
--          1. Insert CAN Frame to Node 1 and send "set_ready" command.
--          2. Wait till Transmission starts.
--          3. Send "set_abort" command.
--          4. Verify that TX Buffer is in "Abort in Progress".
--          5. Wait till transmission ends.
--          6. Verify that TX Buffer ended up in "Aborted".
--      Part 3 (set_ready, set_abort):
--          1. Insert 1 frame to TXT Buffer "n" of Node 1.
--          2. Send "set_ready" command on TXT Buffer "n".
--          3. Wait till transmission starts.
--          4. Insert the frame into "n+1" TXT Buffer.
--          5. Send "set_ready" command on TXT Buffer "n+1".
--          6. Verify that TXT Buffer "n+1" is in "TX Ready".
--          7. Send "send_abort" command to TXT Buffer "n+1".
--          8. Verify that TXT Buffer is "Aborted". Wait till bus is idle.
--      Part 4 (Error state):
--          1. Configure Retransmitt limit to e.g. 5 on Node 1.
--          2. Forbid Acknowledge on Node 2.
--          3. Erase error counters on Node 1 and Node 2.
--          4. Insert Frame to TXT Buffer of Node 1.
--          4. Give "set_ready" command to Node 1.
--          5. Wait till transmission starts and ends for 5 times. After this
--             amount TXT Buffer with the frame should be in "TX Error".
--      Part 5 (Different TXT Buffers):
--          Repeat Parts 1 to 4 for all TXT Buffers via simple for loop!
--
--      One iteration of this test is enough. This test is namely to improve
--      code coverage for TXT Buffer registers!
--
--------------------------------------------------------------------------------
-- Revision History:
--    11.9.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
use work.CAN_FD_frame_format.all;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

use work.CAN_FD_register_map.all;

package txtb_state_feature is
    procedure txtb_state_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body txtb_state_feature is
    procedure txtb_state_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable mode               :       SW_mode := (false, false, false,
                                                false, true, false, false,
                                                false, false, false);
        variable rx_state           :       SW_RX_Buffer_info;
        variable txt_state          :       SW_TXT_Buffer_state_type;
        variable error_counters     :       SW_error_counters := (0, 0, 0, 0);
        variable nxt_buffer         :       natural := 0;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Generate CAN Frame to be used during the whole test.
        -- Use RTR frame to make the test shorter.
        -- Wait till the end of unit integration!
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.rtr := RTR_FRAME;
        CAN_frame.frame_format := NORMAL_CAN;
        wait_rand_cycles(rand_ctr, mem_bus(1).clk_sys, 1600, 1601);

        for i in 1 to TXT_BUFFER_COUNT loop

            report "Starting TXT Buffer " & integer'image(i) & " test!";

            --------------------------------------------------------------------
            -- Part 1 (set_empty)
            --------------------------------------------------------------------
            report "Starting " & integer'image(i) & ".1";

            --------------------------------------------------------------------
            -- Insert frame to Node 1, Wait until it starts transmitting
            --------------------------------------------------------------------
            CAN_send_frame(CAN_frame, i, ID_1, mem_bus(1), frame_sent);
            CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));

            --------------------------------------------------------------------
            -- Check that TXT Buffer state is "TX in progress"
            --------------------------------------------------------------------
            get_tx_buf_state(i, txt_state, ID_1, mem_bus(1));
            if (txt_state /= buf_tx_progress) then
                 -- LCOV_EXCL_START
                o.outcome := false;
                report "TXT Buffer " & integer'image(i) & " is not " &
                       "'TX in Progress' as expected!" severity error;
                -- LCOV_EXCL_STOP
            end if;
            CAN_wait_bus_idle(ID_1, mem_bus(1));

            --------------------------------------------------------------------
            -- Check that TXT Buffer state is "TX Done" after the transmission!
            --------------------------------------------------------------------
            get_tx_buf_state(i, txt_state, ID_1, mem_bus(1));
            if (txt_state /= buf_done) then
                 -- LCOV_EXCL_START
                o.outcome := false;
                report "TXT Buffer " & integer'image(i) & " is not " &
                       "'TX Done' as expected!" severity error;
                -- LCOV_EXCL_STOP
            end if;

            --------------------------------------------------------------------
            -- Issue "set_empty" command and check that buffer is in "Empty"!
            --------------------------------------------------------------------
            send_TXT_buf_cmd(buf_set_empty, i, ID_1, mem_bus(1));
            get_tx_buf_state(i, txt_state, ID_1, mem_bus(1));
            if (txt_state /= buf_empty) then
                 -- LCOV_EXCL_START
                o.outcome := false;
                report "TXT Buffer " & integer'image(i) & " is not " &
                       "'Empty' as expected!" severity error;
                -- LCOV_EXCL_STOP
            end if;


            --------------------------------------------------------------------
            -- Part 2 (set_abort)
            --------------------------------------------------------------------
            report "Starting " & integer'image(i) & ".2";
            --------------------------------------------------------------------
            -- Insert CAN Frame and wait till transmission starts.
            --------------------------------------------------------------------
            CAN_send_frame(CAN_frame, i, ID_1, mem_bus(1), frame_sent);
            CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));

            --------------------------------------------------------------------
            -- Send "set_abort" command
            --------------------------------------------------------------------
            send_TXT_buf_cmd(buf_set_abort, i, ID_1, mem_bus(1));
            get_tx_buf_state(i, txt_state, ID_1, mem_bus(1));
            if (txt_state /= buf_ab_progress) then
                 -- LCOV_EXCL_START
                o.outcome := false;
                report "TXT Buffer " & integer'image(i) & " is not " &
                       "'Abort in progress' as expected!" severity error;
                -- LCOV_EXCL_STOP
            end if;

            --------------------------------------------------------------------
            -- Wait till transmission end, check that TXT Buffer is in "Aborted"
            --------------------------------------------------------------------
            CAN_wait_bus_idle(ID_1, mem_bus(1));
            get_tx_buf_state(i, txt_state, ID_1, mem_bus(1));
            if (txt_state /= buf_aborted) then
                 -- LCOV_EXCL_START
                o.outcome := false;
                report "TXT Buffer " & integer'image(i) & " is not " &
                       "'Aborted' as expected!" severity error;
                -- LCOV_EXCL_STOP
            end if;


            --------------------------------------------------------------------
            -- Part 3 (set_ready)
            --------------------------------------------------------------------
            report "Starting " & integer'image(i) & ".3";
            --------------------------------------------------------------------
            -- Send CAN Frame by Node 1, TXT Buffer i.
            -- Wait till transmission starts!
            --------------------------------------------------------------------
            CAN_send_frame(CAN_frame, i, ID_1, mem_bus(1), frame_sent);
            CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));

            --------------------------------------------------------------------
            -- Insert CAN Frame to Node 1, TXT Buffer i + 1.
            -- Send "set_ready" command.
            --------------------------------------------------------------------
            nxt_buffer := ((i + 1) mod TXT_BUFFER_COUNT) + 1;
            CAN_insert_TX_frame(CAN_frame, nxt_buffer, ID_1, mem_bus(1));
            send_TXT_buf_cmd(buf_set_ready, nxt_buffer, ID_1, mem_bus(1));

            --------------------------------------------------------------------
            -- Check that Buffer "n+1" is in "TX Ready"!
            --------------------------------------------------------------------
            get_tx_buf_state(nxt_buffer, txt_state, ID_1, mem_bus(1));
            if (txt_state /= buf_ready) then
                 -- LCOV_EXCL_START
                o.outcome := false;
                report "TXT Buffer " &
                        integer'image(nxt_buffer) &
                       " is not 'TX Ready' as expected!" severity error;
                -- LCOV_EXCL_STOP
            end if;

            --------------------------------------------------------------------
            -- Send "set_abort" command on Buffer "n+1" and check that
            -- TXT Buffer ends up in "Aborted" state. 
            --------------------------------------------------------------------
            send_TXT_buf_cmd(buf_set_abort, nxt_buffer, ID_1, mem_bus(1));
            if (txt_state /= buf_aborted) then
                 -- LCOV_EXCL_START
                o.outcome := false;
                report "TXT Buffer " &
                       integer'image(nxt_buffer) &
                       " is not 'Aborted' as expected!" severity error;
                -- LCOV_EXCL_STOP
            end if;
            CAN_wait_bus_idle(ID_1, mem_bus(1));



            --------------------------------------------------------------------
            -- Part 4 (Error state)
            --------------------------------------------------------------------
            report "Starting " & integer'image(i) & ".4";
            --------------------------------------------------------------------
            -- Set "ACF" - Acknowledge forbidden on Node 2.
            -- Set Retransmitt limit to 5 on Node 1.
            --------------------------------------------------------------------
            get_core_mode(mode, ID_2, mem_bus(2));
            mode.acknowledge_forbidden := true;
            set_core_mode(mode, ID_2, mem_bus(2));
            CAN_enable_retr_limit(true, 5, ID_1, mem_bus(1));
        
            --------------------------------------------------------------------
            -- Erase error counters on both nodes.
            --------------------------------------------------------------------
            set_error_counters(error_counters, ID_1, mem_bus(1));
            set_error_counters(error_counters, ID_2, mem_bus(2));

            --------------------------------------------------------------------
            -- Send frame from Node 1. Wait till the frame is retransmitted
            -- 5 times!
            --------------------------------------------------------------------
            CAN_send_frame(CAN_frame, i, ID_1, mem_bus(1), frame_sent);
            for i in 0 to 4 loop
                CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));
                CAN_wait_bus_idle(ID_1, mem_bus(1));
            end loop;

            --------------------------------------------------------------------
            -- Check that state of TXT Buffer is "TX Failed".
            --------------------------------------------------------------------
            get_tx_buf_state(i, txt_state, ID_1, mem_bus(1));
            if (txt_state /= buf_failed) then
                 -- LCOV_EXCL_START
                o.outcome := false;
                report "TXT Buffer " & integer'image(i) &
                       " is not 'TX Failed' as expected!" severity error;
                -- LCOV_EXCL_STOP
            end if;
        end loop;

  end procedure;

end package body;
