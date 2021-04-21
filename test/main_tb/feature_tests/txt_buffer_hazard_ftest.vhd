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
--  TXT Buffer datapath hazard feature test.
--
-- @Verifies:
--  @1. When Lock command is issued on Protocol control at the same time as
--      Set Abort command, Lock command will have priority.
--  @2. TXT Buffer will never end up Aborted when Protocol control succesfully
--      locks TXT Buffer for transmission! 
--
-- @Test sequence:  
--  @1. Insert frame to TXT Buffer
--  @2. Mark the buffer as ready and wait for incrementing time.
--  @3. Send set_abort command.
--  @4. Readout status of TXT buffer.
--  @5. If the buffer is "Aborted", check that no transmission is in progress
--      (via STATUS), throw an error if not. If this happend, it would mean
--      CTU CAN FD would be transmitting from Aborted buffer which is hazard!
--  @6. If the buffer is "Abort in progress" check that transmission is 
--      in progress and wait till its end. Throw an error if not.
--  @7. If buffer is in any other state, throw an error. Set Abort command should
--      not be missed by HW!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--      17.1.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package txt_buffer_hazard_ftest is
    procedure txt_buffer_hazard_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body txt_buffer_hazard_ftest is

    procedure txt_buffer_hazard_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable command            :       SW_command := SW_command_rst_val;
        variable status             :       SW_status;
	    variable txt_buf_state	    :	    SW_TXT_Buffer_state_type;	
    begin
        -- Generate CAN frame
        CAN_generate_frame(CAN_frame);
    
        -- Insert the frame for transmittion
        CAN_insert_TX_frame(CAN_frame, 1, DUT_NODE, chn);
    
        -- Repeat test several times
        for i in 1 to 150 loop
    
            -- Give "Set ready" command to the buffer
            send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);
    
            -- Wait for some clock cycles before sending abort command.
            for j in 0 to i loop
                clk_agent_wait_cycle(chn);
            end loop;
    
            -- Give "Set abort" command to the buffer
            send_TXT_buf_cmd(buf_set_abort, 1, DUT_NODE, chn);
    
            -- Wait for some clock cycles before reading buffer and controller state
            for j in 0 to 20 loop
                clk_agent_wait_cycle(chn);
            end loop;
    
            -- Read "TXT Buffer state"
            get_tx_buf_state(1, txt_buf_state, DUT_NODE, chn);
    
            -- Read status of CTU CAN FD controller.
            get_controller_status(status, DUT_NODE, chn);
    
            -- Is controller transmitting?
            if (status.transmitter) then
                -- Is also TXT buffer in transmitting state? Or if set_abort command
                -- comes just at the time when the core LOCKs the buffer, it goes to 
                -- "Abort in Progress".
                if (txt_buf_state = buf_tx_progress) or 
                    (txt_buf_state = buf_ab_progress) then
                    -- Everything is ok
                    info_m("Transmitting: Both TXT buffer and controller " & 
                         "in consistence state." & " [" & to_string(i) & "]");
             
                    -- Wait until bus is idle 
                    CAN_wait_bus_idle(DUT_NODE, chn);
    
                    -- Is the unit now in idle since it is after transmittion already?
                    get_controller_status(status, DUT_NODE, chn);
                    check_m(status.bus_status, "Unit is not Idle!");
                else
                    -- Inconsistency happened
                    error_m("Inconsistence: CAN controller transmitter=TRUE, TXT " &
                            "buffer=" & to_string(txt_buf_state) &
                            " [" & to_string(i) & "]");
                end if;
            else 
                -- Is also TXT buffer in aborted state?
                if (txt_buf_state = buf_aborted) then
                    -- Everything is ok
                    info_m("Both TXT buffer and controller in aborted state" &
                         " [" & to_string(i) & "]");		 
                else
                    -- Inconsistency happened
                   error_m("Inconsistence: CAN controller transmitter=FALSE, TXT " &
                           "buffer=" & to_string(txt_buf_state) & 
                           " [" & to_string(i) & "]");
                end if;
            end if;
    	end loop;
  end procedure;
end package body;
