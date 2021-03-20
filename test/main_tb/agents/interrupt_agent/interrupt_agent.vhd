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
--  @Purpose:
--    Interrupt agent - Can check values of interrupt pin  
--
--------------------------------------------------------------------------------
-- Revision History:
--    12.3.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.interrupt_agent_pkg.all;


entity interrupt_agent is
    port (
        int   :   in std_logic
    );
end entity;


architecture tb of interrupt_agent is
    
    ---------------------------------------------------------------------------
    -- Parameters configured over communication library
    ---------------------------------------------------------------------------
    signal interrupt_polarity   :   std_logic := '1';

begin
    
    ---------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable cmd : integer;
        variable reply_code : integer;
    begin
        receive_start(default_channel, C_INTERRUPT_AGENT_ID);

        -- Command is sent as message type
        cmd := com_channel_data.get_msg_code;
        reply_code := C_REPLY_CODE_OK;
         
        case cmd is
        when INTERRUPT_AGNT_CMD_POLARITY_SET =>
            interrupt_polarity <= com_channel_data.get_param;

        when INTERRUPT_AGNT_CMD_POLARITY_GET =>
            com_channel_data.set_param(interrupt_polarity);

        when INTERRUPT_AGNT_CMD_CHECK_ASSERTED =>
            check_m(int = interrupt_polarity, INTERRUPT_AGENT_TAG &" ** Interrupt asserted **");

        when INTERRUPT_AGNT_CMD_CHECK_NOT_ASSERTED =>
            check_false_m(int = interrupt_polarity, INTERRUPT_AGENT_TAG &" ** Interrupt NOT asserted **");

        when others =>
            info_m("Invalid message type: " & integer'image(cmd));
            reply_code := C_REPLY_CODE_ERR;

        end case;
        receive_finish(default_channel, reply_code);
    end process;
    
end architecture;