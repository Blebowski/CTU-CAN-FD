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
--  TXT Buffer Set ready - SW command (TX_COMMAND) feature test.
--
-- @Verifies:
--  @1. Set Ready command moves TXT Buffer from TX OK, TX Error, Aborted and 
--      Empty to Ready.
--
-- @Test sequence:
--  @1. Check TXT Buffer is empty. Wait until sample point and issue Set ready
--      and check it becomes Ready. Wait until frame is sent.
--  @2. Check TXT Buffer state is TX OK. Wait until sample point and issue
--      Set ready command. Check TXT Buffer is ready. Wait until frame is sent.
--  @3. Wait until Sample point. Issue Set Ready and consecutively Set Abort.
--      Check TXT Buffer is aborted. Issue Set Ready again and check it is
--      Ready now!
--  @4. Set One shot mode in Node 1. Forbid ACK in Node 2. Send CAN frame
--      by Node 1. Wait until CAN frame is sent and check TXT Buffer from
--      which it was sent ended in TX Error. Wait unitl sample point. Issue Set
--      Ready command and check TXT Buffer is Ready
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--   17.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package tx_cmd_set_ready_feature is
    procedure tx_cmd_set_ready_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body tx_cmd_set_ready_feature is
    procedure tx_cmd_set_ready_feature_exec(
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
        variable txt_state          :       SW_TXT_Buffer_state_type;
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
        -- @1. Check TXT Buffer is empty. Wait until sample point and issue
        --    Set ready and check it becomes Ready. Wait until frame is sent.
        -----------------------------------------------------------------------
        info("Step 1");
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_empty, "TXT Buffer empty upon start");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_insert_TX_frame(CAN_frame, buf_nr, ID_1, mem_bus(1));

        CAN_wait_sample_point(iout(1).stat_bus);
        send_TXT_buf_cmd(buf_set_ready, buf_nr, ID_1, mem_bus(1));
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_ready, "Set Ready: Empty -> Ready");
        CAN_wait_frame_sent(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @2. Check TXT Buffer state is TX OK. Wait until sample point and issue
        --    Set ready command. Check TXT Buffer is ready. Wait until frame is
        --    sent.
        ------------------------------------------------------------------------
        info("Step 2");
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_done, "TXT Buffer OK after frame sent!");
        
        CAN_wait_sample_point(iout(1).stat_bus);

        send_TXT_buf_cmd(buf_set_ready, buf_nr, ID_1, mem_bus(1));
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_ready, "Set Ready: TX OK -> Ready");
        CAN_wait_frame_sent(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @3. Wait until Sample point. Issue Set Ready and consecutively Set
        --    Abort. Check TXT Buffer is aborted. Issue Set Ready again and
        --    check it is Ready now!
        ------------------------------------------------------------------------
        info("Step 3");
        CAN_wait_sample_point(iout(1).stat_bus);
        send_TXT_buf_cmd(buf_set_ready, buf_nr, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_abort, buf_nr, ID_1, mem_bus(1));
        
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_aborted, "TXT Buffer Aborted!");
        
        CAN_wait_sample_point(iout(1).stat_bus);
        send_TXT_buf_cmd(buf_set_ready, buf_nr, ID_1, mem_bus(1));
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_ready, "Set Ready: Aborted -> Ready");        
        CAN_wait_frame_sent(ID_1, mem_bus(1));

        return;
        
        -----------------------------------------------------------------------
        -- @4. Set One shot mode in Node 1. Forbid ACK in Node 2. Send CAN frame
        --    by Node 1. Wait until CAN frame is sent and check TXT Buffer from
        --    which it was sent ended in TX Error. Wait unitl sample point.
        --    Issue Set Ready command and check TXT Buffer is Ready
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

        CAN_wait_sample_point(iout(1).stat_bus);
        send_TXT_buf_cmd(buf_set_ready, buf_nr, ID_1, mem_bus(1));
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, ID_1, mem_bus(1));
        check(txt_state = buf_ready, "Set Ready: TX Failed -> Ready");

  end procedure;

end package body;