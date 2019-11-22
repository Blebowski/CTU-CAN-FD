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
--  TX Status feature test.
--
-- Verifies:
--  1. TXT Buffer is Empty after reset
--  2. TXT Buffer is OK after successfull transmission.
--  2. TXT Buffer is Failed after transmission fails from this buffer!
--  3. TXT Buffer is Aborted after Set Abort command was issued during
--     transmission.
--
-- Test sequence:
--  1. Reset Node, Enable it, and wait until it integrates. Pick random TXT
--     Buffer which will be used during this test. Check that TXT Buffer is
--     empty.
--  2. Transmitt CAN Frame by Node 1 and wait until it is received in Node 2.
--     Check that TXT Buffer is OK.
--  3. Set ACK Forbidden mode in Node 2. Set One shot mode in Node 1. Send frame
--     by Node 1 and wait until it is sent. Check that TXT Buffer is in TX
--     Failed.
--  4. Send CAN frame and when it starts, issue Set Abort Command. Wait until
--     frame is sent and check that TXT Buffer is in Aborted.
--
-- Note:
--  Ready, TX in Progress and Abort in Progress are not tested here as they are
--  checked in tx_cmd_set_ready/empty/abort test case.
--------------------------------------------------------------------------------
-- Revision History:
--     22.11.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package tx_status_feature is
    procedure tx_status_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;

package body tx_status_feature is

    procedure tx_status_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        
        variable data               :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable address            :       std_logic_vector(11 downto 0) :=
                                                (OTHERS => '0');
        
        variable ID_1               :       natural := 1;   -- Transmiter
        variable ID_2               :       natural := 2;   -- Receiver          
        
        variable CAN_frame_rx       :       SW_CAN_frame_type;
        variable CAN_frame_tx       :       SW_CAN_frame_type;
        
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;

        variable frame_equal        :       boolean := false;
        variable frame_sent         :       boolean := false;
        variable tmp_int            :       natural := 0;

        variable found              :       boolean;

        variable bus_timing         :       bit_time_config_type;
        variable txt_state          :       SW_TXT_Buffer_state_type;

        variable txt_buf_num        :       natural;

    begin
        o.outcome := true;

        -----------------------------------------------------------------------
        -- 1. Reset Node, Enable it, and wait until it integrates. Pick random
        --    TXT Buffer which will be used during this test. Check that TXT
        --    Buffer is empty.
        -----------------------------------------------------------------------
        info("Step 1");
        
        rand_int_v(rand_ctr, 4, txt_buf_num);
        if (txt_buf_num = 0) then
            txt_buf_num := 1;
        end if;

        CAN_read_timing_v(bus_timing, ID_1, mem_bus(1));
        exec_SW_reset(ID_1, mem_bus(1));
        exec_SW_reset(ID_2, mem_bus(2));
        CAN_configure_timing(bus_timing, ID_1, mem_bus(1));
        CAN_configure_timing(bus_timing, ID_2, mem_bus(2));

        CAN_turn_controller(true, ID_1, mem_bus(1));
        CAN_turn_controller(true, ID_2, mem_bus(2));

        -- Wait till integration is over!
        CAN_wait_bus_on(ID_1, mem_bus(1));
        CAN_wait_bus_on(ID_2, mem_bus(2));

        get_tx_buf_state(txt_buf_num, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_empty, "TX Empty after reset!");

        -----------------------------------------------------------------------
        -- 2. Transmitt CAN Frame by Node 1 and wait until it is received in
        --    Node 2. Check that TXT Buffer is OK.
        -----------------------------------------------------------------------
        info("Step 2");
        
        CAN_generate_frame(rand_ctr, CAN_frame_tx);
        CAN_send_frame(CAN_frame_tx, txt_buf_num, ID_1, mem_bus(1), frame_sent);

        CAN_wait_frame_sent(ID_1, mem_bus(1));

        get_tx_buf_state(txt_buf_num, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_done, "TX OK after frame sent!");

        -----------------------------------------------------------------------
        -- 3. Set ACK Forbidden mode in Node 2. Set One shot mode in Node 1. 
        --    Send frame by Node 1 and wait until it is sent. Check that TXT
        --    Buffer is in TX Failed.
        -----------------------------------------------------------------------
        info("Step 3");

        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));

        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));

        CAN_generate_frame(rand_ctr, CAN_frame_tx);
        CAN_send_frame(CAN_frame_tx, txt_buf_num, ID_1, mem_bus(1), frame_sent);
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        
        get_tx_buf_state(txt_buf_num, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_failed, "TX Failed after error!");

        -----------------------------------------------------------------------
        -- Send CAN frame and when it starts, issue Set Abort Command. Wait
        -- until frame is sent and check that TXT Buffer is in Aborted.
        -----------------------------------------------------------------------
        info("Step 4");

        -- One shot must be disabled now, otherwise this would lead to
        -- TX Failed. We need transmission to be failed, but retransmitt limit
        -- not reached!
        CAN_enable_retr_limit(false, 5, ID_1, mem_bus(1));

        CAN_generate_frame(rand_ctr, CAN_frame_tx);
        CAN_send_frame(CAN_frame_tx, txt_buf_num, ID_1, mem_bus(1), frame_sent);
        
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_abort, txt_buf_num, ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        get_tx_buf_state(txt_buf_num, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_aborted, "TX Aborted after Set Abort!");

    end procedure;
end package body;
