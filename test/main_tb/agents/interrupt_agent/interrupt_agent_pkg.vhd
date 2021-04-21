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
--    12.3.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;


package interrupt_agent_pkg is

    ---------------------------------------------------------------------------
    -- Interrupt agent component    
    ---------------------------------------------------------------------------
    component interrupt_agent is
    port (
        int   :   in std_logic
    );
    end component;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Interrupt agent API    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Set polarity of active Interrupt value.
    --
    -- @param channel   Channel on which to send the request
    -- @param polarity  Polarity to be set.
    ---------------------------------------------------------------------------
    procedure interrupt_agent_polarity_set(
        signal   channel     : inout t_com_channel;
        constant polarity    : in    std_logic
    );

    ---------------------------------------------------------------------------
    -- Get polarity of Interrupt value
    --
    -- @param channel   Channel on which to send the request
    -- @param polarity  Obtained polarity.   
    ---------------------------------------------------------------------------
    procedure interrupt_agent_polarity_get(
        signal   channel     : inout t_com_channel;
        variable polarity    : out   std_logic
    );

    ---------------------------------------------------------------------------
    -- Checks if interrupt is asserted
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure interrupt_agent_check_asserted(
        signal   channel     : inout t_com_channel
    );
    
    ---------------------------------------------------------------------------
    -- Checks if interrupt is not asserted
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure interrupt_agent_check_not_asserted(
        signal   channel     : inout t_com_channel
    );
   
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Private declarations 
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- Supported commands
    constant INTERRUPT_AGNT_CMD_POLARITY_SET            : integer := 0;
    constant INTERRUPT_AGNT_CMD_POLARITY_GET            : integer := 1;
    constant INTERRUPT_AGNT_CMD_CHECK_ASSERTED          : integer := 2;
    constant INTERRUPT_AGNT_CMD_CHECK_NOT_ASSERTED      : integer := 3;
    
    -- Tag for messages
    constant INTERRUPT_AGENT_TAG : string := "Interrupt Agent: ";

end package;


package body interrupt_agent_pkg is
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Interrupt agent API
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    procedure interrupt_agent_polarity_set(
        signal   channel     : inout t_com_channel;
        constant polarity    : in    std_logic
    ) is
    begin
        info_m(INTERRUPT_AGENT_TAG & "Setting polarity");
        com_channel_data.set_param(polarity);
        send(channel, C_INTERRUPT_AGENT_ID, INTERRUPT_AGNT_CMD_POLARITY_SET);
        debug_m(INTERRUPT_AGENT_TAG & "Polarity set");
    end procedure;


    procedure interrupt_agent_polarity_get(
        signal   channel     : inout t_com_channel;
        variable polarity    : out   std_logic
    )is
    begin
        info_m(INTERRUPT_AGENT_TAG & "Getting polarity");
        send(channel, C_INTERRUPT_AGENT_ID, INTERRUPT_AGNT_CMD_POLARITY_GET);
        polarity := com_channel_data.get_param;
        debug_m(INTERRUPT_AGENT_TAG & "Polarity got");
    end procedure;
   
   
    procedure interrupt_agent_check_asserted(
        signal   channel     : inout t_com_channel
    )is
    begin
        info_m(INTERRUPT_AGENT_TAG & "Checking asserted");
        send(channel, C_INTERRUPT_AGENT_ID, INTERRUPT_AGNT_CMD_CHECK_ASSERTED);
        debug_m(INTERRUPT_AGENT_TAG & "Asserted checked");
    end procedure;


    procedure interrupt_agent_check_not_asserted(
        signal   channel     : inout t_com_channel
    )is
    begin
        info_m(INTERRUPT_AGENT_TAG & "Checking not asserted");
        send(channel, C_INTERRUPT_AGENT_ID, INTERRUPT_AGNT_CMD_CHECK_NOT_ASSERTED);
        debug_m(INTERRUPT_AGENT_TAG & "Not asserted checked");
    end procedure;

end package body;
