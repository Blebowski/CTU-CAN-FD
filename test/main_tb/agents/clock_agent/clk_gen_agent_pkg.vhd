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
--    Package with API for Clock generator agent.
--
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--    04.2.2021   Adjusted to work without Vunits COM library.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;


package clk_gen_agent_pkg is

    ---------------------------------------------------------------------------
    -- Clock generator component    
    ---------------------------------------------------------------------------
    component clk_gen_agent is
    port (
        -- Generated clock output
        clock_out   :   out std_logic := '0';
        clock_in    :   in  std_logic
    );
    end component;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Clock generator agent API    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Start clock generator agent.
    --
    -- @param channel   Channel on which to send the request 
    ---------------------------------------------------------------------------
    procedure clk_gen_agent_start(
        signal channel          : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Stop clock generator agent.
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure clk_gen_agent_stop(
        signal channel          : inout t_com_channel
    );
        
    ---------------------------------------------------------------------------
    -- Set clock period of clock generator agent.
    --
    -- @param channel   Channel on which to send the request
    -- @param period    Clock period to be set.
    ---------------------------------------------------------------------------
    procedure clk_agent_set_period(
        signal      channel     : inout t_com_channel;
        constant    period      : in    time
    );
    
    ---------------------------------------------------------------------------
    -- Get clock period of clock generator agent.
    --
    -- @param channel   Channel on which to send the request
    -- @param period    Obtained clock period.
    ---------------------------------------------------------------------------
    procedure clk_agent_get_period(
        signal      channel     : inout t_com_channel;
        variable    period      : out   time
    );
    
    ---------------------------------------------------------------------------
    -- Set clock generator jitter.
    --
    -- @param channel   Channel on which to send the request
    -- @param jitter    Jitter to be set.    
    ---------------------------------------------------------------------------
    procedure clk_agent_set_jitter(
        signal      channel     : inout t_com_channel;
        constant    jitter      : in    time
    );
    
    ---------------------------------------------------------------------------
    -- Get clock generator jitter.
    --
    -- @param channel   Channel on which to send the request
    -- @param jitter    Obtained jitter.  
    ---------------------------------------------------------------------------
    procedure clk_agent_get_jitter(
        signal      channel     : inout t_com_channel;
        variable    jitter      : out   time
    );

    ---------------------------------------------------------------------------
    -- Set clock generator duty cycle.
    --
    -- @param channel   Channel on which to send the request
    -- @param duty      Duty cycle to be set.
    ---------------------------------------------------------------------------
    procedure clk_agent_set_duty(
        signal      channel     : inout t_com_channel;
        constant    duty        : in    integer range 0 to 100
    );

    ---------------------------------------------------------------------------
    -- Get clock generator duty cycle.
    --
    -- @param channel   Channel on which to send the request
    -- @param duty      Obtained duty cycle.  
    ---------------------------------------------------------------------------
    procedure clk_agent_get_duty(
        signal      channel     : inout t_com_channel;
        variable    duty        : out   integer range 0 to 100
    );

    ---------------------------------------------------------------------------
    -- Wait 1 clock period of clock. Waits on clock_in, not on clock_out.
    -- So when clocks are not generated by clock generator, it still works
    -- properly!
    --
    -- During the waiting, communication channel is blocked and shall not be
    -- used by other processes/agents. 
    --
    -- @param channel    Channel on which to send the request
    -- @param num_cycles Number  
    ---------------------------------------------------------------------------
    procedure clk_agent_wait_cycle(
        signal      channel     : inout t_com_channel
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
    constant CLK_AGNT_CMD_WAIT_CYCLE    : integer := 8;

end package;


package body clk_gen_agent_pkg is
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Clock generator agent API
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    procedure clk_gen_agent_start(
        signal      channel     : inout t_com_channel
    ) is
    begin
        info_m("Starting clock generator agent!");
        send(channel, C_CLOCK_AGENT_ID, CLK_AGNT_CMD_START);
        debug_m("Clock generator agent started!");
    end procedure;


    procedure clk_gen_agent_stop(
        signal      channel     : inout t_com_channel
    ) is
    begin
        info_m("Stopping clock generator agent!");
        send(channel, C_CLOCK_AGENT_ID, CLK_AGNT_CMD_STOP);
        debug_m("Clock generator agent stopped");
    end procedure;

 
    procedure clk_agent_set_period(
        signal      channel     : inout t_com_channel;
        constant    period      : in    time
    ) is
    begin
        info_m("Setting clock agent period to: " & time'image(period));
        com_channel_data.set_param(period);
        send(channel, C_CLOCK_AGENT_ID, CLK_AGNT_CMD_PERIOD_SET);
        debug_m("Clock generator period set");
    end procedure;


    procedure clk_agent_get_period(
        signal      channel     : inout t_com_channel;
        variable    period      : out   time
    ) is
    begin
        info_m("Getting clock agent period");
        send(channel, C_CLOCK_AGENT_ID, CLK_AGNT_CMD_PERIOD_GET);
        period := com_channel_data.get_param;
        debug_m("Clock generator period got");
    end procedure;
 
 
    procedure clk_agent_set_jitter(
        signal      channel     : inout t_com_channel;
        constant    jitter      : in    time
    ) is
    begin
        info_m("Setting clock agent jitter to: " & time'image(jitter));
        com_channel_data.set_param(jitter);
        send(channel, C_CLOCK_AGENT_ID, CLK_AGNT_CMD_JITTER_SET);
        debug_m("Clock generator period set");
    end procedure;


    procedure clk_agent_get_jitter(
        signal      channel     : inout t_com_channel;
        variable    jitter      : out   time
    ) is
    begin
        info_m("Getting clock agent jitter");
        send(channel, C_CLOCK_AGENT_ID, CLK_AGNT_CMD_JITTER_GET);
        jitter := com_channel_data.get_param;
        debug_m("Clock generator jitter got");
    end procedure;
 
 
    procedure clk_agent_set_duty(
        signal      channel     : inout t_com_channel;
        constant    duty        : in    integer range 0 to 100
    ) is
    begin
        info_m("Setting clock agent duty cycle to: " & integer'image(duty));
        com_channel_data.set_param(duty);
        send(channel, C_CLOCK_AGENT_ID, CLK_AGNT_CMD_DUTY_SET);
        debug_m("Clock generator period set");
    end procedure;
    
    
    procedure clk_agent_get_duty(
        signal      channel     : inout t_com_channel;
        variable    duty        : out   integer range 0 to 100
    ) is
    begin
        info_m("Getting clock agent duty cycle");
        send(channel, C_CLOCK_AGENT_ID, CLK_AGNT_CMD_DUTY_GET);
        duty := com_channel_data.get_param;
        debug_m("Clock generator duty cycle got");
    end procedure;
    
    
    procedure clk_agent_wait_cycle(
        signal      channel     : inout t_com_channel
    ) is
    begin
        debug_m("Waiting one clock cycle");
        send(channel, C_CLOCK_AGENT_ID, CLK_AGNT_CMD_WAIT_CYCLE);
        debug_m("Waited one clock cycle");
    end procedure;
 
 
end package body;
