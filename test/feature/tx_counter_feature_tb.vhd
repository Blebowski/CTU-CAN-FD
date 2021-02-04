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
--  TX Traffic counter feature test implementation.
--
-- @Verifies:
--  @1. TX Counter is incremented after each succesfully transmitted frame.
--  @2. TX Counter is not incremented when error frame is transmitted.
--  @3. RX Counter is not incremented when frame is succesfully transmitted.
--  @4. TX Counter is cleared by COMMAND[TXFRCRST].
--  @5. TX Counter is NOT cleared by COMMAND[RXFRCRST].
--
-- @Test sequence:
--  @1. Read TX Counter from Node 1. Set One-shot mode (no retransmission) in
--      Node 1.
--  @2. Send frame from Node 1. Wait until EOF field. Read TX counter and check
--      it did not change yet.
--  @3. Wait until the end of EOF. Read TX counter and check it was incremented.
--      Read RX counter and check it is not incremented!
--  @4. Send Frame from Node 1. Wait till ACK field. Corrupt ACK field to be
--      recessive. Wait till Node 2 is not in ACK field anymore. Check Node 1
--      is transmitting Error frame.
--  @5. Wait until Node 2 also starts transmitting error frame. Wait until bus 
--      is idle, check that TX Counter was not incremented in Node 1.
--  @6. Send random amount of frames by Node 1 and wait until they are sent.
--  @7. Check that TX counter was incremented by N!
--  @8. Issue COMMAND[RXFRCRST] and check TX counter was NOT cleared in Node 1.
--  @9. Issue COMMAND[TXFRCRST] and check TX counter was cleared in Node 1.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    26.9.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package tx_counter_feature is
    procedure tx_counter_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );

end package;


package body tx_counter_feature is
    procedure tx_counter_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :       natural := 1;
        variable ID_2               :       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable rand_value         :       natural;

        variable ctrs_1             :       SW_traffic_counters;
        variable ctrs_2             :       SW_traffic_counters;
        variable ctrs_3             :       SW_traffic_counters;
        variable ctrs_4             :       SW_traffic_counters;
        variable ctrs_5             :       SW_traffic_counters;
        
        variable status             :       SW_status;
        variable command            :       SW_command := SW_command_rst_val;
    begin

        ------------------------------------------------------------------------
        --  @1. Read TX Counter from Node 1. Set One-shot mode (no retransmission)
        --     in Node 2.
        ------------------------------------------------------------------------
        info("Step 1: Read initial counter values.");
        read_traffic_counters(ctrs_1, ID_1, mem_bus(1));
        CAN_enable_retr_limit(true, 0, ID_2, mem_bus(2));
        
        ------------------------------------------------------------------------
        --  @2. Send frame from Node 1. Wait until EOF field. Read TX counter and
        --     check it did not change yet.
        ------------------------------------------------------------------------
        info("Step 2: Send frame by Node 1!");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_eof, ID_1, mem_bus(1));
        read_traffic_counters(ctrs_2, ID_1, mem_bus(1));
        check(ctrs_1.tx_frames = ctrs_2.tx_frames,
            "TX counter unchanged before EOF!");
        check(ctrs_1.rx_frames = ctrs_2.rx_frames,
            "RX counter unchanged before EOF!");
        
        ------------------------------------------------------------------------
        --  @3. Wait until the end of EOF. Read TX counter and check it was 
        --     incremented. Read RX counter and check it is not incremented!       
        ------------------------------------------------------------------------
        info("Step 3: Check TX, RX counters after frame.");
        CAN_wait_not_pc_state(pc_deb_eof, ID_1, mem_bus(1));
        read_traffic_counters(ctrs_3, ID_1, mem_bus(1));
        check(ctrs_1.tx_frames + 1 = ctrs_3.tx_frames,
            "TX counter changed after EOF!");
        check(ctrs_1.rx_frames = ctrs_3.rx_frames,
            "RX counter unchanged after EOF!");
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        --  @4. Send Frame from Node 1. Wait till ACK field. Corrupt ACK field to
        --     be recessive. Wait till Node 1 is not in ACK field anymore. Check
        --     Node 1 is transmitting Error frame.
        ------------------------------------------------------------------------
        info("Step 4: Send frame and force ACK recessive!");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.frame_format := NORMAL_CAN;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_ack, ID_1, mem_bus(1));

        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_not_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission, "Error frame is being transmitted!");
        release_bus_level(so.bl_force);
        
        ------------------------------------------------------------------------
        -- @5. Wait until Node 2 also starts transmitting error frame. Wait until
        --    bus is idle, check that TX Counter was not incremented.
        ------------------------------------------------------------------------
        info("Step 5: Wait until error frame!");
        CAN_wait_error_frame(ID_2, mem_bus(2));    
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        read_traffic_counters(ctrs_4, ID_1, mem_bus(1));
        check(ctrs_3.tx_frames = ctrs_4.tx_frames,
            "TX counter unchanged after Error frame!");
        check(ctrs_3.rx_frames = ctrs_4.rx_frames,
            "RX counter unchanged after Error frame!");
        
        ------------------------------------------------------------------------
        -- @6. Send random amount of frames by Node 1 and check that TX counter
        --    was incremented by N!
        ------------------------------------------------------------------------
        info("Step 6: Send N random frames!");
        rand_int_v(rand_ctr, 6, rand_value);
        for i in 0 to rand_value - 1 loop
            CAN_generate_frame(rand_ctr, CAN_frame);
            CAN_send_frame(CAN_frame, 2, ID_1, mem_bus(1), frame_sent);
            CAN_wait_frame_sent(ID_1, mem_bus(1));
        end loop;

        ------------------------------------------------------------------------
        -- @7. Check that TX counter was incremented by N!
        ------------------------------------------------------------------------
        info("Step 7: Check TX counter was incremented by N!");
        read_traffic_counters(ctrs_5, ID_1, mem_bus(1));
        check(ctrs_4.tx_frames + rand_value = ctrs_5.tx_frames,
              "TX Frames counter incremented by: " & integer'image(rand_value));

        ------------------------------------------------------------------------
        --  @8. Issue COMMAND[RXFRCRST] and check TX counter was NOT cleared in 
        --     Node 1.
        ------------------------------------------------------------------------
        command.rx_frame_ctr_rst := true;
        give_controller_command(command, ID_1, mem_bus(1));
        read_traffic_counters(ctrs_1, ID_1, mem_bus(1));
        check(ctrs_1.tx_frames = ctrs_5.tx_frames,
              "TX counter not cleared by COMMAND[RXFRCRST]");

        ------------------------------------------------------------------------
        --  @8. Issue COMMAND[TXFRCRST] and check TX counter was cleared in 
        --     Node 1.
        ------------------------------------------------------------------------
        command.rx_frame_ctr_rst := false;
        command.tx_frame_ctr_rst := true;
        give_controller_command(command, ID_1, mem_bus(1));
        read_traffic_counters(ctrs_1, ID_1, mem_bus(1));
        check(ctrs_1.tx_frames = 0, "TX counter cleared by COMMAND[TXFRCRST]");

    end procedure;

end package body;