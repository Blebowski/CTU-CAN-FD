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
-- Purpose:
--  Retransmitt limit feature test 2 (cornercases).
--
-- Verifies:
--  1. When TXT Buffer from which core transmits is changed between two
--     consecutive transmissions, retransmitt counter is cleared! 
--  2. When there are multiple errors during single frame, retransmitt counter
--     is incremented only by 1!
--  3. When there is Arbitration lost and Error frame during single frame,
--     retransmitt counter is incremented only by 1!
--  4. When unit is a receiver without attempt to transmitt a frame
--     (TXT Buffer is ready, unit is Error passive and dominant bit is detected
--      during Suspend field), if an error occurs during such a frame,
--     retransmitt counter is not incremented!
--
-- Test sequence:
--  1. Configure retransmitt limit in Node 1, enable retransmitt limitation.
--     Enable Test Mode in Node 1 to be able manipulate with Error counters. 
--     Configure Node 2 to Acknowledge Forbidden Mode to invoke transmission of
--     Error frames during test. Configure TXT Buffer 1 in Node 1 with priority
--     1, TXT Buffer 2 in Node 1 with priority 2.
--  2. Send frame from TXT Buffer 1 by Node 1. After 2 transmit attempts, insert
--     frame to TXT Buffer 2 and mark TXT Buffer 2 as ready! Check that
--     retransmitt counter is equal to 2!
--  3. Wait until transmission starts by Node 1. Check that TXT Buffer 2 was
--     selected for transmission. Check that retransmitt limit is 0 now
--     (because TXT Buffer used is changed).
--  4. Check that 'retransmit limit - 1' times TXT Buffer 2 ends up in TX Ready
--     after transmission of an Error frame. Check that after 'retransmit limit'
--     retransmissions, TXT Buffer 2 ends in TX Error. This verifies that
--     previous retransmissions were not counted in transmission from TXT Buffer
--     2. Meanwhile, issue Abort command to TXT Buffer 1, so that we don't
--     continue transmissions from TXT Buffer 1 after retransmitt limit was
--     reached on TXT Buffer 2.
--  5. Check that Retransmitt counter is 0 now!
--  6. Clear TX Error counter in Node 1. Check it is 0. Send frame from TXT
--     Buffer 1. Send frame from TXT Buffer by Node 2.
--  7. Wait until error frame (due to ACF in Node 2), corrupt bus level for
--     duration of 6 bits, force bus level to be recessive.
--  8. Wait until transmission is over. Check that TXT Buffer 1 is in TX Ready,
--     Retransmitt counter is 1. Check that TX Error counter is (6+1)*8 = 56
--     (One increment as original missing ACK, 6 increments due to six errors
--      in Error frame). Wait until TXT Buffer 1 ends up in TX Error state.
--     Check Retransmitt counter is 0. Wait Until bus is idle.
--  9. Insert frame with CAN ID = 10 to Node 1, CAN ID = 9 to Node 2. Send frame
--     by both Nodes! Wait until Arbitration field. Check that both Nodes are
--     transmitters. Wait until Control field. Check that Node 1 is receiver now
--     and Node 2 is still transmitter. Check that Retransmitt counter in Node
--     1 is now 1. 
-- 10. Wait until ACK field. Force ACK field to be low during whole duration
--     of ACK field. Wait until change in Protocol control, check that
--     Error frame is being transmitted. Wait until Intermission field, check
--     that Retransmitt counter in Node 1 is still equal to 1. Wait until TXT
--     Buffer ends up in TX Error (after some retransmissions). Check that
--     retransmitt counter is now 0 in Node 1. Wait until bus is idle.
--------------------------------------------------------------------------------
-- Revision History:
--    11.7.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package retr_limit_2_feature is
    procedure retr_limit_2_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body retr_limit_2_feature is
    procedure retr_limit_2_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ctr_1              :       natural;
        variable ctr_2              :       natural;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable rand_val           :       real;
        variable retr_th            :       natural;
        variable mode_backup        :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable buf_state          :       SW_TXT_Buffer_state_type;
        variable status             :       SW_status;
        variable retr_ctr           :       natural;
        variable fault_state        :       SW_fault_state;
    begin
        
        -- Hard coded threshold is enough for this test!
        retr_th := 5;

        ------------------------------------------------------------------------
        -- 1. Configure retransmitt limit in Node 1, enable retransmitt
        --    limitation. Enable Test Mode in Node 1 to be able manipulate with
        --    Error counters. Configure Node 2 to Acknowledge Forbidden Mode to
        --    invoke transmission of Error frames during test. Configure TXT
        --    Buffer 1 in Node 1 with priority 1, TXT Buffer 2 in Node 1 with
        --    priority 2.
        ------------------------------------------------------------------------
        info("Step 1: Configuring retransmitt limit to 1 (Node 1), ACF (Node 2)");
        CAN_enable_retr_limit(true, retr_th, ID_1, mem_bus(1));
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        
        CAN_configure_tx_priority(1, 1, ID_1, mem_bus(1));
        CAN_configure_tx_priority(2, 2, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- 2. Send frame from TXT Buffer 2 by Node 1. After 2 transmit attempts,
        --    insert frame to TXT Buffer 1 and mark TXT Buffer 1 as ready! Check
        --    that retransmitt counter is equal to 2!
        ------------------------------------------------------------------------
        info("Step 2: Sending frame by Node 1");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        for i in 0 to 1 loop
            CAN_wait_error_frame(ID_1, mem_bus(1));
            CAN_wait_pc_state(pc_deb_intermission, ID_1, mem_bus(1));
        end loop;
        
        CAN_send_frame(CAN_frame, 2, ID_1, mem_bus(1), frame_sent);
        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 2, "Retransmitt counter equal to 2!");
        CAN_wait_not_pc_state(pc_deb_intermission, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- 3. Wait until transmission starts by Node 1. Check that TXT Buffer 2
        --    was selected for transmission. Check that retransmitt limit is 0
        --    now (because TXT Buffer used is changed).
        ------------------------------------------------------------------------
        info("Step 3: Checking TXT Buffer 2 selected!");
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));
        get_tx_buf_state(2, buf_state, ID_1, mem_bus(1));
        check(buf_state = buf_tx_progress, "Buffer 2 selected!");
        get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
        check(buf_state = buf_ready, "Buffer 1 not selected!");
        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 0, "Retransmitt counter equal to 0!");

        ------------------------------------------------------------------------
        -- 4. Check that 'retransmit limit - 1' times TXT Buffer 1 ends up in
        --    TX Ready after transmission of an Error frame. Check that after
        --    'retransmit limit' retransmissions, TXT Buffer 2 ends in TX Error.
        --    This verifies that previous retransmissions were not counted in
        --    transmission from TXT Buffer 2. Meanwhile, issue Abort command to
        --    TXT Buffer 1, so that we don't continue transmissions from TXT
        --    Buffer 1 after retransmitt limit was reached on TXT Buffer 2.
        ------------------------------------------------------------------------
        info("Step 4: Checking N reatransmissions!");
        for i in 0 to retr_th loop
            
            -- Additionally, check intermediate value of retransmitt counter!
            retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
            check(retr_ctr = i,
                "Retransmitt counter equal to number of Errors!");
            
            CAN_wait_frame_sent(ID_1, mem_bus(1));
            get_tx_buf_state(2, buf_state, ID_1, mem_bus(1));
            if (i /= retr_th) then
                check(buf_state = buf_ready, "TXT Buffer ready");
            else
                check(buf_state = buf_failed, "TXT Buffer failed");
            end if;

            if (i = 1) then
                send_TXT_buf_cmd(buf_set_abort, 1, ID_1, mem_bus(1));
            end if;

        end loop;

        ------------------------------------------------------------------------
        -- 5. Check that Retransmitt counter is 0 now!
        ------------------------------------------------------------------------
        info("Step 5: Check retransmitt counter is 0 after TXT Buffer in TX Error!");
        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 0, "Retransmitt counter equal to 0!");

        ------------------------------------------------------------------------
        -- 6. Clear TX Error counter in Node 1. Check it is 0. Send frame from
        --    TXT Buffer 1. Send frame from TXT Buffer by Node 2.
        ------------------------------------------------------------------------
        info("Step 6: Clear TX Error counter in Node 1. Send frame from Node 2.");
        err_counters.tx_counter := 0;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        read_error_counters(err_counters, ID_1, mem_bus(1));
        check(err_counters.tx_counter = 0, "Error counter cleared!");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        ------------------------------------------------------------------------
        -- 7. Wait until error frame (due to ACF in Node 2), corrupt bus level
        --    for duration of 6 bits, force bus level to be recessive.
        ------------------------------------------------------------------------
        info("Step 7: Corrupting bus level for 6 bits during Error frame!");
        CAN_wait_error_frame(ID_1, mem_bus(1));
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        for i in 0 to 5 loop
            info("Bit number: " & integer'image(i));
            CAN_wait_sample_point(iout(1).stat_bus);
            wait for 15 ns; -- Wait till sample point is not active anymore!
        end loop;
        release_bus_level(so.bl_force);

        ------------------------------------------------------------------------
        -- 8. Wait until transmission is over. Check that TXT Buffer 1 is in TX
        --    Ready, Retransmitt counter is 1. Check that TX Error counter is
        --    (6+1)*8 = 56 (One increment as original missing ACK, 6 increments
        --    due to six errors in Error frame). Wait until TXT Buffer 1 ends
        --    up in TX Error state. Check Retransmitt counter is 0. Wait until
        --    Bus is idle.
        ------------------------------------------------------------------------
        info("Step 8: Checking Error counters!");
        CAN_wait_pc_state(pc_deb_intermission, ID_1, mem_bus(1));
        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 1,
            "Retransmitt counter incremented only once during multiple error frames!");
        get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
        check(buf_state = buf_ready, "TXT Buffer ready");
        read_error_counters(err_counters, ID_1, mem_bus(1));
        check(err_counters.tx_counter = 56, "Error counter cleared!");

        while (buf_state /= buf_failed) loop
            get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
            wait for 1000 ns;
        end loop;

        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 0,
            "Retransmitt counter cleared after TX Error!");
        
        -- Wait until bus is idle, clear Error counters to avoid going to error
        -- Passive.
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        err_counters.tx_counter := 0;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- 9. Insert frame with CAN ID = 10 to Node 1, CAN ID = 9 to Node 2.
        --    Send frame by both Nodes! Wait until Arbitration field. Check that
        --    both Nodes are transmitters. Wait until Control field. Check that
        --    Node 1 is receiver now and Node 2 is still transmitter. Check that
        --    Retransmitt counter in Node 1 is now 1. 
        ------------------------------------------------------------------------
        info("Step 9: Invoking Arbitration!");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.identifier := 10;
        CAN_insert_TX_frame(CAN_frame, 1, ID_1, mem_bus(1));
        CAN_frame.identifier := 9;
        CAN_insert_TX_frame(CAN_frame, 1, ID_2, mem_bus(2));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));
        
        CAN_wait_pc_state(pc_deb_arbitration, ID_1, mem_bus(1));
        CAN_wait_pc_state(pc_deb_arbitration, ID_2, mem_bus(2));

        get_controller_status(status, ID_1, mem_bus(1));
        check(status.transmitter, "Node 1 transmitter");
        get_controller_status(status, ID_2, mem_bus(2));
        check(status.transmitter, "Node 2 transmitter");

        CAN_wait_pc_state(pc_deb_control, ID_1, mem_bus(1));
        CAN_wait_pc_state(pc_deb_control, ID_2, mem_bus(2));

        get_controller_status(status, ID_1, mem_bus(1));
        check(status.receiver, "Node 1 receiver");
        get_controller_status(status, ID_2, mem_bus(2));
        check(status.transmitter, "Node 2 transmitter");

        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 1,
            "Retransmitt counter incremented after arbitration loss!");

        ------------------------------------------------------------------------
        -- 10. Wait until ACK field. Force ACK field to be low during whole
        --     duration of ACK field. Wait until change in Protocol control,
        --     check that Error frame is being transmitted. Wait until Intermi-
        --     ssion field, check that Retransmitt counter in Node 1 is still
        --     equal to 1. Wait until TXT Buffer ends up in TX Error (after some
        --     retransmissions). Check that retransmitt counter is now 0 in
        --     Node 1. Wait until bus is idle.
        ------------------------------------------------------------------------
        info("Step 10: Wait until ACK field");
        CAN_wait_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_not_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        release_bus_level(so.bl_force);
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission, "Error frame being transmitted!");
        
        CAN_wait_pc_state(pc_deb_intermission, ID_1, mem_bus(1));
        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 1,
            "Retransmitt counter incremented only once during arbitration loss" &
            " and Error frame during single frame!");
        
        get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
        while (buf_state /= buf_failed) loop
            get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
            wait for 1000 ns;
        end loop;
        
        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 0,
            "Retransmitt counter 0 when TXT Buffer goes to TX Error!");
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        wait for 1000 ns;
        
  end procedure;

end package body;