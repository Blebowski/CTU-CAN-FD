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
--    Package with API for Timestamp agent.
--
--------------------------------------------------------------------------------
-- Revision History:
--    12.3.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;


package timestamp_agent_pkg is

    ---------------------------------------------------------------------------
    -- Interrupt agent component    
    ---------------------------------------------------------------------------
    component timestamp_agent is
    port (
        clk_sys     : in std_logic;
        timestamp   : out std_logic_vector(63 downto 0)
    );
    end component;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Timestamp agent API    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Start timestamp agent
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure timestamp_agent_start(
        signal   channel     : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Stop timestamp agent
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure timestamp_agent_stop(
        signal   channel     : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Set step of timestamp agent
    --
    -- @param channel   Channel on which to send the request
    -- @param step      Natural
    ---------------------------------------------------------------------------
    procedure timestamp_agent_set_step(
        signal   channel     : inout t_com_channel;
        constant step        : in    natural
    );

    ---------------------------------------------------------------------------
    -- Set prescaler of timestamp agent
    --
    -- @param channel   Channel on which to send the request
    -- @param prescaler Natural
    ---------------------------------------------------------------------------
    procedure timestamp_agent_set_prescaler(
        signal   channel     : inout t_com_channel;
        constant prescaler   : in    natural
    );
    
    ---------------------------------------------------------------------------
    -- Preset timestamp value of timestamp agent
    --
    -- @param channel   Channel on which to send the request
    -- @param timestamp Timestamp value to be preset
    ---------------------------------------------------------------------------
    procedure timestamp_agent_timestamp_preset(
        signal   channel     : inout t_com_channel;
        constant timestamp   : in    std_logic_vector(63 downto 0)
    );
   
    
    ---------------------------------------------------------------------------
    -- Gets current value of timestamp
    --
    -- @param channel   Channel on which to send the request
    -- @param timestamp Return value of timestamp
    ---------------------------------------------------------------------------
    procedure timestamp_agent_get_timestamp(
        signal   channel     : inout t_com_channel;
        variable timestamp   : out   std_logic_vector(63 downto 0)
    );

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Private declarations 
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- Supported commands
    constant TIMESTAMP_AGENT_CMD_START                  : integer := 0;
    constant TIMESTAMP_AGENT_CMD_STOP                   : integer := 1;
    constant TIMESTAMP_AGENT_CMD_STEP_SET               : integer := 2;
    constant TIMESTAMP_AGENT_CMD_PRESCALER_SET          : integer := 3;
    constant TIMESTAMP_AGENT_CMD_TIMESTAMP_PRESET       : integer := 4;
    constant TIMESTAMP_AGENT_CMD_GET_TIMESTAMP          : integer := 5;
    
    -- Tag for messages
    constant TIMESTAMP_AGENT_TAG : string := "Timestamp Agent: ";

end package;


package body timestamp_agent_pkg is
    
    procedure timestamp_agent_start(
        signal   channel     : inout t_com_channel
    ) is
    begin
        info_m(TIMESTAMP_AGENT_TAG & "Starting");
        send(channel, C_TIMESTAMP_AGENT_ID, TIMESTAMP_AGENT_CMD_START);
        debug_m(TIMESTAMP_AGENT_TAG & "Started");
    end procedure;


    procedure timestamp_agent_stop(
        signal   channel     : inout t_com_channel
    ) is
    begin
        info_m(TIMESTAMP_AGENT_TAG & "Stopping");
        send(channel, C_TIMESTAMP_AGENT_ID, TIMESTAMP_AGENT_CMD_STOP);
        debug_m(TIMESTAMP_AGENT_TAG & "Stopped");
    end procedure;


    procedure timestamp_agent_set_step(
        signal   channel     : inout t_com_channel;
        constant step        : in    natural
    ) is
    begin
        info_m(TIMESTAMP_AGENT_TAG & "Setting step");
        com_channel_data.set_param(step);
        send(channel, C_TIMESTAMP_AGENT_ID, TIMESTAMP_AGENT_CMD_STEP_SET);
        debug_m(TIMESTAMP_AGENT_TAG & "Step set");
    end procedure;


    procedure timestamp_agent_set_prescaler(
        signal   channel     : inout t_com_channel;
        constant prescaler   : in    natural
    ) is
    begin
        info_m(TIMESTAMP_AGENT_TAG & "Setting Prescaler");
        com_channel_data.set_param(prescaler);
        send(channel, C_TIMESTAMP_AGENT_ID, TIMESTAMP_AGENT_CMD_PRESCALER_SET);
        debug_m(TIMESTAMP_AGENT_TAG & "Prescaler set");
    end procedure;


    procedure timestamp_agent_timestamp_preset(
        signal   channel     : inout t_com_channel;
        constant timestamp   : in    std_logic_vector(63 downto 0)
    ) is
    begin
        info_m(TIMESTAMP_AGENT_TAG & "Presetting timestamp");
        com_channel_data.set_param(timestamp);
        send(channel, C_TIMESTAMP_AGENT_ID, TIMESTAMP_AGENT_CMD_TIMESTAMP_PRESET);
        debug_m(TIMESTAMP_AGENT_TAG & "Timestamp preset");
    end procedure;

    procedure timestamp_agent_get_timestamp(
        signal   channel     : inout t_com_channel;
        variable timestamp   : out   std_logic_vector(63 downto 0)
    ) is
        variable tmp : std_logic_vector(127 downto 0);
    begin
        info_m(TIMESTAMP_AGENT_TAG & "Reading timestamp");
        send(channel, C_TIMESTAMP_AGENT_ID, TIMESTAMP_AGENT_CMD_GET_TIMESTAMP);
        tmp := com_channel_data.get_param;
        timestamp := tmp(63 downto 0);
        debug_m(TIMESTAMP_AGENT_TAG & "Timestamp read");
    end procedure;

end package body;
