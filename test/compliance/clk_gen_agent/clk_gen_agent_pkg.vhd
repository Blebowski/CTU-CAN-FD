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
--    Package with declarations for clock generator agent accessible over
--    Vunit communication library!
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

package clk_gen_agent_pkg is

    ---------------------------------------------------------------------------
    -- Clock generator component    
    ---------------------------------------------------------------------------
    component clk_gen_agent is
    port (
        -- Generated clock output
        clock   :   out std_logic
    );
    end component;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Clock generator agent API    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure start_clk_gen_agent(
        signal      net         : inout network_t
    );

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure stop_clk_gen_agent(
        signal      net         : inout network_t
    );
        
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure set_period_clk_agent(
        signal      net         : inout network_t;
        constant    period      : in    time
    );
    
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure get_period_clk_agent(
        signal      net         : inout network_t;
        variable    period      : out   time
    );
    
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure set_jitter_clk_agent(
        signal      net         : inout network_t;
        constant    jitter      : in    time
    );
    
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure get_jitter_clk_agent(
        signal      net         : inout network_t;
        variable    jitter      : out   time
    );

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure set_duty_clk_agent(
        signal      net         : inout network_t;
        constant    duty        : in    integer range 0 to 100
    );

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure get_duty_clk_agent(
        signal      net         : inout network_t;
        variable    duty        : out   integer range 0 to 100
    );

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Private declarations 
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- Supported commands for clock agent (sent as message types)
    constant CLK_AGNT_CMD_START         : integer := 0;
    constant CLK_AGNT_CMD_STOP          : integer := 1;
    constant CLK_AGNT_CMD_PERIOD_SET    : integer := 2;
    constant CLK_AGNT_CMD_PERIOD_GET    : integer := 3;
    constant CLK_AGNT_CMD_JITTER_SET    : integer := 4;
    constant CLK_AGNT_CMD_JITTER_GET    : integer := 5;
    constant CLK_AGNT_CMD_DUTY_SET      : integer := 6;
    constant CLK_AGNT_CMD_DUTY_GET      : integer := 7;
    
    constant CLK_AGNT_CMD_REPLY_OK      : integer := 256;
    constant CLK_AGNT_CMD_REPLY_ERR     : integer := 257;
    
    -- Clock gen actor
    constant actor_clk_gen_agent : actor_t := new_actor("actor_clk_gen_agent");

end package;


library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package body clk_gen_agent_pkg is
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Clock generator agent API
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    procedure start_clk_gen_agent(
        signal net              : inout network_t
    ) is
        constant clk_gen_rec : actor_t := find("actor_clk_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info("Starting clock generator agent!");
        req_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_START));
        send(net         => net,
             receiver    => clk_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg);
        check(message_type(reply_msg).p_code = CLK_AGNT_CMD_REPLY_OK,
            "Clock generator agent started!");
    end procedure;


    procedure stop_clk_gen_agent(
        signal net : inout network_t
    ) is
        constant clk_gen_rec : actor_t := find("actor_clk_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info("Stopping clock generator agent!");
        req_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_STOP));
        send(net         => net,
             receiver    => clk_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CLK_AGNT_CMD_REPLY_OK,
            "Clock generator agent stopped");
    end procedure;

 
    procedure set_period_clk_agent(
        signal      net         : inout network_t;
        constant    period      : in    time
    ) is
        constant clk_gen_rec : actor_t := find("actor_clk_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info("Setting clock agent period to: " & time'image(period));
        req_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_PERIOD_SET));
        push(req_msg, period);
        send(net         => net,
             receiver    => clk_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CLK_AGNT_CMD_REPLY_OK,
            "Clock generator period set");
    end procedure;


    procedure get_period_clk_agent(
        signal      net         : inout network_t;
        variable    period      : out   time
    ) is
    constant clk_gen_rec : actor_t := find("actor_clk_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info("Getting clock agent period");
        req_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_PERIOD_GET));
        send(net         => net,
             receiver    => clk_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg);
        period := pop(reply_msg);
        check(message_type(reply_msg).p_code = CLK_AGNT_CMD_REPLY_OK,
            "Clock generator period got");
    end procedure;
 
 
    procedure set_jitter_clk_agent(
        signal      net         : inout network_t;
        constant    jitter      : in    time
    ) is
        constant clk_gen_rec : actor_t := find("actor_clk_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info("Setting clock agent jitter to: " & time'image(jitter));
        req_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_JITTER_SET));
        push(req_msg, jitter);
        send(net         => net,
             receiver    => clk_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CLK_AGNT_CMD_REPLY_OK,
            "Clock generator period set");
    end procedure;


    procedure get_jitter_clk_agent(
        signal      net         : inout network_t;
        variable    jitter      : out   time
    ) is
    constant clk_gen_rec : actor_t := find("actor_clk_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info("Getting clock agent jitter");
        req_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_JITTER_GET));
        send(net         => net,
             receiver    => clk_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg);
        jitter := pop(reply_msg);
        check(message_type(reply_msg).p_code = CLK_AGNT_CMD_REPLY_OK,
            "Clock generator jitter got");
    end procedure;
 
 
    procedure set_duty_clk_agent(
        signal      net         : inout network_t;
        constant    duty        : in    integer range 0 to 100
    ) is
        constant clk_gen_rec : actor_t := find("actor_clk_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info("Setting clock agent duty cycle to: " & integer'image(duty));
        req_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_DUTY_SET));
        push(req_msg, duty);
        send(net         => net,
             receiver    => clk_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CLK_AGNT_CMD_REPLY_OK,
            "Clock generator period set");
    end procedure;
    
    
    procedure get_duty_clk_agent(
        signal      net         : inout network_t;
        variable    duty        : out   integer range 0 to 100
    ) is
    constant clk_gen_rec : actor_t := find("actor_clk_gen_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info("Getting clock agent duty cycle");
        req_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_DUTY_GET));
        send(net         => net,
             receiver    => clk_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg);
        duty := pop(reply_msg);
        check(message_type(reply_msg).p_code = CLK_AGNT_CMD_REPLY_OK,
            "Clock generator duty cycle got");
    end procedure;
 
 
end package body;