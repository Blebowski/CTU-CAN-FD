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
--    Package with API for Reset generator agent.
--
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--    04.2.2021   Adjusted to work without Vunits COM library.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;


package reset_agent_pkg is

    ---------------------------------------------------------------------------
    -- Reset generator component    
    ---------------------------------------------------------------------------
    component reset_agent is
    port (
        -- Generated reset
        reset   :   out std_logic
    );
    end component;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Reset generator agent API    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Assert reset by Reset generator agent.
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure rst_agent_assert(
        signal channel  : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- De-assert reset by Reset generator agent.
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure rst_agent_deassert(
        signal channel  : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Set polarity of Reset generator agent.
    --
    -- @param channel   Channel on which to send the request
    -- @param polarity  Polarity to be set.
    ---------------------------------------------------------------------------
    procedure rst_agent_polarity_set(
        signal   channel     : inout t_com_channel;
        constant polarity    : in    std_logic
    );

    ---------------------------------------------------------------------------
    -- Get polarity of Reset generator agent.
    --
    -- @param channel   Channel on which to send the request
    -- @param polarity  Obtained polarity.   
    ---------------------------------------------------------------------------
    procedure rst_agent_polarity_get(
        signal   channel     : inout t_com_channel;
        variable polarity    : out   std_logic
    );

   
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Private declarations 
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- Supported commands
    constant RST_AGNT_CMD_ASSERT         : integer := 0;
    constant RST_AGNT_CMD_DEASSERT       : integer := 1;
    constant RST_AGNT_CMD_POLARITY_SET   : integer := 2;
    constant RST_AGNT_CMD_POLARITY_GET   : integer := 3;
    
    -- Reset agent tag (for messages)
    constant RESET_AGENT_TAG : string := "Reset Agent: ";

end package;


package body reset_agent_pkg is
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Reset generator agent API
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
   
    procedure rst_agent_assert(
        signal channel  : inout t_com_channel
    ) is
    begin
        info_m(RESET_AGENT_TAG & "Asserting reset");
        send(channel, C_RESET_AGENT_ID, RST_AGNT_CMD_ASSERT);
        debug_m(RESET_AGENT_TAG & "Asserted reset");
    end procedure;


    procedure rst_agent_deassert(
        signal channel  : inout t_com_channel
    ) is
    begin
        info_m(RESET_AGENT_TAG & "De-Asserting reset");
        send(channel, C_RESET_AGENT_ID, RST_AGNT_CMD_DEASSERT);
        debug_m(RESET_AGENT_TAG & "De-Asserted reset");
    end procedure;


    procedure rst_agent_polarity_set(
        signal   channel    : inout t_com_channel;
        constant polarity   : in    std_logic
    ) is
    begin
        info_m(RESET_AGENT_TAG & "Setting reset polarity to: " & std_logic'image(polarity));
        com_channel_data.set_param(polarity);
        send(channel, C_RESET_AGENT_ID, RST_AGNT_CMD_POLARITY_SET);
        debug_m(RESET_AGENT_TAG & "Polarity set");
    end procedure;

    procedure rst_agent_polarity_get(
        signal   channel    : inout t_com_channel;
        variable polarity   : out   std_logic
    ) is
    begin
        info_m(RESET_AGENT_TAG & "Getting reset polarity");
        send(channel, C_RESET_AGENT_ID, RST_AGNT_CMD_POLARITY_GET);
        polarity := com_channel_data.get_param;
        debug_m(RESET_AGENT_TAG & "Polarity got");
    end procedure;

end package body;
