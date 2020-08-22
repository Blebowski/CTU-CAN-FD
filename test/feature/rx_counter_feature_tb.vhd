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
--  RX Traffic counter feature test implementation.
--
-- @Verifies:
--  @1. RX Counter is incremented after each succesfully received frame.
--  @2. RX Counter is not incremented when error frame is transmitted.
--  @3. TX Counter is not incremented when frame is succesfully received.
--  @4. RX Counter is cleared by COMMAND[RXFRCRST].
--  @5. RX Counter is NOT cleared by COMMAND[TXFRCRST].
--  @6. RX Counter is incremented when frame is transmitted in Loopback mode.
--
-- @Test sequence:
--  @1. Read TX Counter from Node 1. Set One-shot mode (no retransmission) in
--      Node 1.
--  @2. Send frame from Node 2. Wait until EOF field. Read RX counter from Node 1
--      and check it did not change yet.
--  @3. Wait until the end of EOF. Read RX counter and check it was incremented.
--      Read TX counter and check it is not incremented!
--  @4. Send Frame from Node 2. Wait till ACK field. Corrupt ACK field to be
--      recessive. Wait till Node 2 is not in ACK field anymore. Check Node 2
--      is transmitting Error frame.
--  @5. Wait until Node 1 also starts transmitting error frame. Wait until bus 
--      is idle, check that RX Counter was not incremented in Node 1.
--  @6. Send random amount of frames by Node 2 and wait until they are sent.
--  @7. Check that RX counter was incremented by N in Node 1!
--  @8. Issue COMMAND[TXFRCRST] and check RX counter was NOT cleared in Node 1.
--  @9. Issue COMMAND[RXFRCRST] and check RX counter was cleared in Node 1.
-- @10. Read all frames from RX buffer in Node 1.
-- @11. Set Loopback mode in Node 1. Send frame by Node 1 and wait until it is
--      sent. Check there is a frame received in RX buffer. Check that RX frame
--      counter was incremented.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    28.9.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package rx_counter_feature is
    procedure rx_counter_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );

end package;


package body rx_counter_feature is
    procedure rx_counter_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :       natural := 1;
        variable ID_2               :       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable RX_CAN_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable rand_value         :       natural;

        variable ctrs_1             :       SW_traffic_counters;
        variable ctrs_2             :       SW_traffic_counters;
        variable ctrs_3             :       SW_traffic_counters;
        variable ctrs_4             :       SW_traffic_counters;
        variable ctrs_5             :       SW_traffic_counters;
        
        variable status             :       SW_status;
        variable command            :       SW_command := SW_command_rst_val;
        
        variable rx_buf_info        :       SW_RX_Buffer_info;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
    begin

        ------------------------------------------------------------------------
        -- @1. Read TX Counter from Node 1. Set One-shot mode (no retransmission)
        --    in Node 1.
        ------------------------------------------------------------------------
        info("Step 1: Read initial counter values.");
        read_traffic_counters(ctrs_1, ID_1, mem_bus(1));
        CAN_enable_retr_limit(true, 0, ID_2, mem_bus(2));
        
        ------------------------------------------------------------------------
        -- @2. Send frame from Node 2. Wait until EOF field. Read RX counter from
        --    Node 1 and check it did not change yet.
        ------------------------------------------------------------------------        
        info("Step 2: Send frame by Node 2!");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 2, ID_2, mem_bus(2), frame_sent);
        CAN_wait_pc_state(pc_deb_eof, ID_1, mem_bus(1));
        read_traffic_counters(ctrs_2, ID_1, mem_bus(1));
        check(ctrs_1.tx_frames = ctrs_2.tx_frames,
            "TX counter unchanged before EOF!");
        check(ctrs_1.rx_frames = ctrs_2.rx_frames,
            "RX counter unchanged before EOF!");

        ------------------------------------------------------------------------
        -- @3. Wait until the end of EOF. Read RX counter and check it was
        --    incremented. Read TX counter and check it is not incremented!
        ------------------------------------------------------------------------
        info("Step 3: Check TX, RX counters after frame.");
        CAN_wait_not_pc_state(pc_deb_eof, ID_1, mem_bus(1));
        read_traffic_counters(ctrs_3, ID_1, mem_bus(1));
        check(ctrs_1.tx_frames = ctrs_3.tx_frames,
            "TX counter unchanged after EOF!");
        check(ctrs_1.rx_frames + 1 = ctrs_3.rx_frames,
            "RX counter changed after EOF!");
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @4. Send Frame from Node 2. Wait till ACK field. Corrupt ACK field to
        --    be recessive. Wait till Node 2 is not in ACK field anymore. Check
        --    Node 2 is transmitting Error frame.
        ------------------------------------------------------------------------
        info("Step 4: Send frame and force ACK recessive!");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.frame_format := NORMAL_CAN;
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_pc_state(pc_deb_ack, ID_2, mem_bus(2));

        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_not_pc_state(pc_deb_ack, ID_2, mem_bus(2));
        get_controller_status(status, ID_2, mem_bus(2));
        check(status.error_transmission, "Error frame is being transmitted!");
        release_bus_level(so.bl_force);

        ------------------------------------------------------------------------
        -- @5. Wait until Node 1 also starts transmitting error frame. Wait until
        --    bus is idle, check that RX Counter was not incremented in Node 1.
        ------------------------------------------------------------------------
        info("Step 5: Wait until error frame!");
        CAN_wait_error_frame(ID_1, mem_bus(1));    
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        read_traffic_counters(ctrs_4, ID_1, mem_bus(1));
        check(ctrs_3.tx_frames = ctrs_4.tx_frames,
            "TX counter unchanged after Error frame!");
        check(ctrs_3.rx_frames = ctrs_4.rx_frames,
            "RX counter unchanged after Error frame!");

        ------------------------------------------------------------------------
        -- @6. Send random amount of frames by Node 2 and wait until they are
        --    sent.
        ------------------------------------------------------------------------
        info("Step 6: Send N random frames!");
        rand_int_v(rand_ctr, 6, rand_value);
        for i in 0 to rand_value - 1 loop
            CAN_generate_frame(rand_ctr, CAN_frame);
            CAN_send_frame(CAN_frame, 3, ID_2, mem_bus(2), frame_sent);
            CAN_wait_frame_sent(ID_2, mem_bus(2));
        end loop;

        ------------------------------------------------------------------------
        -- @7. Check that RX counter was incremented by N in Node 1!
        ------------------------------------------------------------------------
        info("Step 7: Check RX counter was incremented by N!");
        read_traffic_counters(ctrs_5, ID_1, mem_bus(1));
        check(ctrs_4.rx_frames + rand_value = ctrs_5.rx_frames,
              "RX Frames counter incremented by: " & integer'image(rand_value));

        ------------------------------------------------------------------------
        -- @8. Issue COMMAND[TXFRCRST] and check RX counter was NOT cleared in 
        --    Node 1.
        ------------------------------------------------------------------------
        info("Step 8: Issue COMMAND[TXFRCRST]");
        command.tx_frame_ctr_rst := true;
        give_controller_command(command, ID_1, mem_bus(1));
        read_traffic_counters(ctrs_1, ID_1, mem_bus(1));
        check(ctrs_1.rx_frames = ctrs_5.rx_frames,
              "RX counter not cleared by COMMAND[TXFRCRST]");

        ------------------------------------------------------------------------
        -- @9. Issue COMMAND[RXFRCRST] and check RX counter was cleared in 
        --     Node 1.
        ------------------------------------------------------------------------
        info("Step 9: Issue COMMAND[RXFRCRST]");
        command.rx_frame_ctr_rst := true;
        command.tx_frame_ctr_rst := false;
        give_controller_command(command, ID_1, mem_bus(1));
        read_traffic_counters(ctrs_1, ID_1, mem_bus(1));
        check(ctrs_1.rx_frames = 0, "RX counter cleared by COMMAND[RXFRCRST]");

        ------------------------------------------------------------------------
        -- @10. Read all frames from RX buffer in Node 1.
        ------------------------------------------------------------------------
        info("Step 10: Read all frames from RX Buffer!");
        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        if (rx_buf_info.rx_frame_count > 0) then
            for i in 0 to rx_buf_info.rx_frame_count - 1 loop
                CAN_read_frame(RX_CAN_frame, ID_1, mem_bus(1));
            end loop;
        end if;
        
        ------------------------------------------------------------------------
        -- @11. Set Loopback mode in Node 1. Send frame by Node 1 and wait until
        --     it is sent. Check there is a frame received in RX buffer. Check
        --     that RX frame counter was incremented.
        ------------------------------------------------------------------------
        info("Step 11: Check RX counter is incremented in Loopback mode!");
        mode_1.internal_loopback := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 4, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        read_traffic_counters(ctrs_1, ID_1, mem_bus(1));
        check(ctrs_1.rx_frames = 1,
            "RX counter incremented when frame sent in Loopback mode!");

    end procedure;

end package body;