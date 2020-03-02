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
--  @Purpose:
--    Package with API for Reset generator agent.
--
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package rst_gen_agent_pkg is

    ---------------------------------------------------------------------------
    -- Reset generator component    
    ---------------------------------------------------------------------------
    component rst_gen_agent is
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
    -- @param net Network on which Reset agent listens (use "net").    
    ---------------------------------------------------------------------------
    procedure rst_agent_assert(
        signal      net         : inout network_t
    );

    ---------------------------------------------------------------------------
    -- De-assert reset by Reset generator agent.
    --
    -- @param net Network on which Reset agent listens (use "net").
    ---------------------------------------------------------------------------
    procedure rst_agent_deassert(
        signal      net         : inout network_t
    );

    ---------------------------------------------------------------------------
    -- Set polarity of Reset generator agent.
    --
    -- @param net       Network on which Reset agent listens (use "net").
    -- @param polarity  Polarity to be set.
    ---------------------------------------------------------------------------
    procedure rst_agent_polarity_set(
        signal      net         : inout network_t;
        constant    polarity    : in    std_logic
    );

    ---------------------------------------------------------------------------
    -- Get polarity of Reset generator agent.
    --
    -- @param net       Network on which Reset agent listens (use "net").
    -- @param polarity  Obtained polarity.   
    ---------------------------------------------------------------------------
    procedure rst_agent_polarity_get(
        signal      net         : inout network_t;
        variable    polarity    : out   std_logic
    );

   
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Private declarations 
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- Supported commands for clock agent (sent as message types)
    constant RST_AGNT_CMD_ASSERT         : integer := 0;
    constant RST_AGNT_CMD_DEASSERT       : integer := 1;
    constant RST_AGNT_CMD_POLARITY_SET   : integer := 2;
    constant RST_AGNT_CMD_POLARITY_GET   : integer := 3;
    
    constant RST_AGNT_CMD_REPLY_OK      : integer := 256;
    constant RST_AGNT_CMD_REPLY_ERR     : integer := 257;
    
    -- Reset gen actor
    constant actor_rst_gen_agent : actor_t := new_actor("actor_rst_gen_agent");

    -- Reset agent tag (for messages)
    constant RESET_AGENT_TAG : string := "Reset Agent: ";

end package;


library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package body rst_gen_agent_pkg is
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Reset generator agent API
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
   
    procedure rst_agent_assert(
        signal      net         : inout network_t
    ) is
        constant rst_gen_rec : actor_t := find("actor_rst_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(RESET_AGENT_TAG & "Asserting reset");
        req_msg := new_msg(msg_type => (p_code => RST_AGNT_CMD_ASSERT));
        send(net         => net,
             receiver    => rst_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = RST_AGNT_CMD_REPLY_OK,
              RESET_AGENT_TAG & "Reset asserted");
    end procedure;


    procedure rst_agent_deassert(
        signal      net         : inout network_t
    ) is
        constant rst_gen_rec : actor_t := find("actor_rst_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(RESET_AGENT_TAG & "De-Asserting reset");
        req_msg := new_msg(msg_type => (p_code => RST_AGNT_CMD_DEASSERT));
        send(net         => net,
             receiver    => rst_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = RST_AGNT_CMD_REPLY_OK,
              RESET_AGENT_TAG & "Reset de-asserted");
    end procedure;


    procedure rst_agent_polarity_set(
        signal      net         : inout network_t;
        constant    polarity    : in    std_logic
    ) is
        constant rst_gen_rec : actor_t := find("actor_rst_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(RESET_AGENT_TAG & "Setting reset polarity to: " & std_logic'image(polarity));
        req_msg := new_msg(msg_type => (p_code => RST_AGNT_CMD_POLARITY_SET));
        push(req_msg, polarity);
        send(net         => net,
             receiver    => rst_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = RST_AGNT_CMD_REPLY_OK,
              RESET_AGENT_TAG & "Reset polarity set");
    end procedure;

    procedure rst_agent_polarity_get(
        signal      net         : inout network_t;
        variable    polarity    : out   std_logic
    ) is
        constant rst_gen_rec : actor_t := find("actor_rst_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(RESET_AGENT_TAG & "Getting reset polarity");
        req_msg := new_msg(msg_type => (p_code => RST_AGNT_CMD_POLARITY_GET));
        send(net         => net,
             receiver    => rst_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        polarity := pop(reply_msg);
        check(message_type(reply_msg).p_code = RST_AGNT_CMD_REPLY_OK,
              RESET_AGENT_TAG & "Reset polarity got");
    end procedure;

end package body;
