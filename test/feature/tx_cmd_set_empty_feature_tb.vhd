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
--  TXT Buffer Set empty - SW command (TX_COMMAND) feature test.
--
-- Verifies:
--  1. Set Empty command moves TXT Buffer from TX OK, TX Error, Aborted to Empty.
--
-- Test sequence:
--  1. Check TXT Buffer is empty. Issue Set empty and check it is still empty.
--  2. Generate random CAN frame, send it from TXT Buffer and wait until it is
--     received. Check TXT Buffer is in TX OK. Issue Set Empty command and
--     check TXT Buffer is empty.
--  3. Generate random CAN frame, send it fom TXT Buffer and issue Set Abort
--     command. Wait until frame is over and check TXT Buffer is Aborted. Issue
--     Set Empty command and check it becomes Empty.
--  4. Set One shot mode in Node 1. Forbid ACK in Node 2. Send CAN frame by Node
--     1. Wait until CAN frame is sent and check TXT Buffer from which it was
--     sent ended in TX Error. Issue Set empty command and check TXT Buffer is
--     Empty.
--------------------------------------------------------------------------------
-- Revision History:
--   16.11.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package tx_cmd_set_empty_feature is
    procedure tx_cmd_set_empty_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body tx_cmd_set_empty_feature is
    procedure tx_cmd_set_empty_feature_exec(
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
        variable mode               :       SW_mode := SW_mode_rst_val;
        variable rx_state           :       SW_RX_Buffer_info;
        variable txt_state          :       SW_TXT_Buffer_state_type;
        variable error_counters     :       SW_error_counters := (0, 0, 0, 0);
        variable nxt_buffer         :       natural := 0;
        variable buf_nr             :       natural;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
    begin

        -- For whole test random TXT Buffer will be used!
        rand_int_v(rand_ctr, 4, buf_nr);
        if (buf_nr = 0) then
            buf_nr := 1;
        end if;
        info("Testing with TXT Buffer: " & integer'image(buf_nr));

        -----------------------------------------------------------------------
        -- 1. Check TXT Buffer is empty. Issue Set empty and check it is still
        --    empty.
        -----------------------------------------------------------------------
        info("Step 1");
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_empty, "TXT Buffer empty upon start");
        send_TXT_buf_cmd(buf_set_empty, buf_nr, ID_1, mem_bus(1));
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_empty, "TXT Buffer still empty even post restart!");

        ------------------------------------------------------------------------
        -- 2. Generate random CAN frame, send it from TXT Buffer and wait until
        --    it is received. Check TXT Buffer is in TX OK. Issue Set Empty
        --    command and check TXT Buffer is empty.
        ------------------------------------------------------------------------
        info("Step 2");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, buf_nr, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));

        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_done, "TXT Buffer OK after frame sent!");
        send_TXT_buf_cmd(buf_set_empty, buf_nr, ID_1, mem_bus(1));
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_empty, "Set Empty: TX OK -> Empty");

        ------------------------------------------------------------------------
        -- 3. Generate random CAN frame, send it fom TXT Buffer and issue Set
        --    Abort command. Wait until frame is over and check TXT Buffer is
        --    Aborted. Issue Set Empty command and check it becomes Empty.
        ------------------------------------------------------------------------
        info("Step 2");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, buf_nr, ID_1, mem_bus(1), frame_sent);
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_abort, buf_nr, ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_aborted, "TXT Buffer Aborted!");
        send_TXT_buf_cmd(buf_set_empty, buf_nr, ID_1, mem_bus(1));
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_empty, "Set Empty: Aborted -> Empty");

        -----------------------------------------------------------------------
        -- 4. Set One shot mode in Node 1. Forbid ACK in Node 2. Send CAN frame
        --    by Node 1. Wait until CAN frame is sent and check TXT Buffer from
        --    which it was sent ended in TX Error. Issue Set empty command and
        --    check TXT Buffer is Empty.
        -----------------------------------------------------------------------
        info("Step 3");
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));

        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, buf_nr, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));

        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_failed, "TXT Buffer Failed!");

        send_TXT_buf_cmd(buf_set_empty, buf_nr, ID_1, mem_bus(1));
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_empty, "Set Empty: TX Failed -> Empty");

  end procedure;

end package body;