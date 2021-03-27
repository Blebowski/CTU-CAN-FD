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
--    Package with API of test controller agent.
--------------------------------------------------------------------------------
-- Revision History:
--    31.1.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

package test_probe_agent_pkg is

    component test_probe_agent is
    port (
        -- VIP test control / status signals
        dut_test_probe          : in t_ctu_can_fd_test_probe;
        test_node_test_probe    : in t_ctu_can_fd_test_probe 
    );
    end component;

    ---------------------------------------------------------------------------
    -- Wait till next sample point of DUT.
    --
    -- Note: This function blocks communication channel during waiting!!
    --
    -- @param channel           Channel on which to send the request
    -- @param node              Node index on which to wait (0 - DUT, 1 - Test)
    -- @param skip_stuff_bits   If true, sample points of stuff bits are
    --                          ignored.
    ---------------------------------------------------------------------------
    procedure test_probe_agent_wait_sample(
        signal channel          : inout t_com_channel;
               node             : in    natural;
               skip_stuff_bits  : in    boolean
    );

    ---------------------------------------------------------------------------
    -- Wait till start of next bit of DUT (Sync segment)
    --
    -- Note: This function blocks communication channel during waiting!!
    --
    -- @param channel           Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure test_probe_agent_wait_sync(
        signal channel          : inout t_com_channel;
               node             : in    natural
    );

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Private declarations 
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- Supported commands
    constant TEST_PROBE_AGNT_WAIT_SAMPLE_NO_STUFF         : integer := 0;
    constant TEST_PROBE_AGNT_WAIT_SAMPLE_STUFF            : integer := 1;
    constant TEST_PROBE_AGNT_WAIT_SYNC                    : integer := 2;
    
    -- Reset agent tag (for messages)
    constant TEST_PROBE_AGENT_TAG : string := "Test probe Agent: ";

end package;


package body test_probe_agent_pkg is

    procedure test_probe_agent_wait_sample(
        signal channel          : inout t_com_channel;
               node             : in    natural;
               skip_stuff_bits  : in    boolean
    ) is
    begin
        info_m(TEST_PROBE_AGENT_TAG & "Waiting till sample point");
        com_channel_data.set_param(node);
        if (skip_stuff_bits) then
            send(channel, C_TEST_PROBE_AGENT_ID, TEST_PROBE_AGNT_WAIT_SAMPLE_NO_STUFF);
        else
            send(channel, C_TEST_PROBE_AGENT_ID, TEST_PROBE_AGNT_WAIT_SAMPLE_STUFF);
        end if;
        info_m(TEST_PROBE_AGENT_TAG & "Waited till sample point");
    end procedure;


    procedure test_probe_agent_wait_sync(
        signal channel          : inout t_com_channel;
               node             : in    natural
    ) is
    begin
        info_m(TEST_PROBE_AGENT_TAG & "Waiting till SYNC segment");
        com_channel_data.set_param(node);
        send(channel, C_TEST_PROBE_AGENT_ID, TEST_PROBE_AGNT_WAIT_SYNC);
        info_m(TEST_PROBE_AGENT_TAG & "Waited till SYNC segment");
    end procedure;

end package body;

