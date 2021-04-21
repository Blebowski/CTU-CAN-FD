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
--  STATUS[TXNF] feature test.
--
-- @Verifies:
--  @1. When no TXT Buffer is in Empty state, STATUS[TXNF] is not set.
--  @2. When at least on TXT Buffer is in Empty state STATUS[TXNF] is set.
--
-- @Test sequence:
--  @1. Set BMM mode in DUT. Check that STATUS[TXNF] is set (all TXT Buffers
--      should be empty).
--  @2. Issue Set ready consecutively to all TXT Buffers. Check that STATUS[TXNF]
--      is set before last buffer. Check that after last buffer STATUS[TXNF] is
--      not set.
--  @3. Check that all TXT Buffers are Failed now. Move always single buffer to
--      empty and check that STATUS[TXNF] is set. Move this buffer to Failed again
--      and check that STATUS[TXNF] is not set. Repeat with each TXT Buffer.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    31.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package status_txnf_ftest is
    procedure status_txnf_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body status_txnf_ftest is
    procedure status_txnf_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        
        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable mode_1             :     SW_mode;
        
        variable num_txt_bufs       :     natural;
    begin
        
        -- Read number of TXT buffers to adapt based on number of TXT buffers.
        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);
        
        -----------------------------------------------------------------------
        -- @1. Set BMM mode in DUT. Check that STATUS[TXNF] is set (all TXT
        --    Buffers should be empty).
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        mode_1.bus_monitoring := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.tx_buffer_empty, "STATUS[TXNF] set!");

        -----------------------------------------------------------------------
        -- @2. Issue Set ready consecutively to all TXT Buffers. Check that
        --     STATUS[TXNF] is set before last buffer. Check that after last
        --     buffer STATUS[TXNF] is not set.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        for i in 1 to num_txt_bufs loop
            send_TXT_buf_cmd(buf_set_ready, i, DUT_NODE, chn);
            
            -- Waiting time for FSM to change state!
            wait for 20 ns;
            
            get_controller_status(stat_1, DUT_NODE, chn);

            if (i = num_txt_bufs) then
                check_false_m(stat_1.tx_buffer_empty,
                    "STATUS[TXNF] not set after last TXT Buffer");
            else
                check_m(stat_1.tx_buffer_empty,
                    "STATUS[TXNF] set before last TXT Buffer");
            end if;
        end loop;

        -----------------------------------------------------------------------
        -- @3. Check that all TXT Buffers are Failed now. Move always single
        --    buffer to empty and check that STATUS[TXNF] is set. Move this
        --    buffer to Failed again and check that STATUS[TXNF] is not set.
        --    Repeat with each TXT Buffer.
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        for i in 1 to num_txt_bufs loop
            get_tx_buf_state(i, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_failed, "TXT Buffer: " &
                integer'image(i) & " failed!");
        end loop;

        for i in 1 to num_txt_bufs loop
            send_TXT_buf_cmd(buf_set_empty, i, DUT_NODE, chn);
            
            -- Waiting time for FSM to change state!
            wait for 20 ns;
            
            get_controller_status(stat_1, DUT_NODE, chn);
            check_m(stat_1.tx_buffer_empty, "STATUS[TXNF] set!");
            
            send_TXT_buf_cmd(buf_set_ready, i, DUT_NODE, chn);
            
            -- Waiting time for FSM to change state!
            wait for 20 ns;
            
            get_controller_status(stat_1, DUT_NODE, chn);
            check_false_m(stat_1.tx_buffer_empty, "STATUS[TXNF] not set!");
            
            get_tx_buf_state(i, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_failed, "TXT Buffer: " &
                integer'image(i) & " ready!");
        end loop;

  end procedure;

end package body;
