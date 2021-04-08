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
--    Timestamp agent - Generates timestamp on DUT input.
--
--    Has following features:
--      - Initialization of timestamp
--      - Step of timestamp (next value bigger than previous by step)
--      - Prescaler of timestamp (how many cycles needed to increment by step).
--      - Start/Stop counting.
--
--------------------------------------------------------------------------------
-- Revision History:
--    12.3.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.timestamp_agent_pkg.all;


entity timestamp_agent is
    port (
        clk_sys     : in std_logic;
        timestamp   : out std_logic_vector(63 downto 0)
    );
end entity;


architecture tb of timestamp_agent is
    
    ---------------------------------------------------------------------------
    -- Parameters configured over communication library
    ---------------------------------------------------------------------------
    signal step         : unsigned(63 downto 0);
    signal prescaler    : natural := 1;
    
    signal running      : boolean := false;

    signal timestamp_i  : unsigned(63 downto 0) := (OTHERS => '0');

    signal timestamp_preset_val : std_logic_vector(63 downto 0);
    signal timestamp_preset     : std_logic;
    
    signal prescaler_ctr        : natural := 0;
    signal last_clk_event       : time;
    
begin

    ---------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable cmd : integer;
        variable reply_code : integer;
        variable tmp : std_logic_vector(127 downto 0);
        variable tmp_int : integer;
    begin
        receive_start(default_channel, C_TIMESTAMP_AGENT_ID);

        -- Command is sent as message type
        cmd := com_channel_data.get_msg_code;
        reply_code := C_REPLY_CODE_OK;
         
        case cmd is
        when TIMESTAMP_AGENT_CMD_START =>
            running <= true;
            
        when TIMESTAMP_AGENT_CMD_STOP =>
            running <= false;

        when TIMESTAMP_AGENT_CMD_STEP_SET =>
            tmp_int := com_channel_data.get_param;
            step <= to_unsigned(tmp_int, 64);
            
        when TIMESTAMP_AGENT_CMD_PRESCALER_SET =>
            prescaler <= com_channel_data.get_param;
            
        when TIMESTAMP_AGENT_CMD_TIMESTAMP_PRESET =>
            tmp := com_channel_data.get_param;
            timestamp_preset_val <= tmp(63 downto 0);
            wait for 0 ns;
            timestamp_preset <= '1';
            wait for 0 ns;
            timestamp_preset <= '0';
            wait for 0 ns;
            
        when TIMESTAMP_AGENT_CMD_GET_TIMESTAMP =>
            com_channel_data.set_param(std_logic_vector(timestamp_i));

        when others =>
            info_m("Invalid message type: " & integer'image(cmd));
            reply_code := C_REPLY_CODE_ERR;

        end case;
        receive_finish(default_channel, reply_code);
    end process;
    
    ---------------------------------------------------------------------------
    -- Timestamp generation
    ---------------------------------------------------------------------------
    timestamp_gen_proc : process
    begin
        if (running = false) then
            wait until running = true;
        end if;

        if (timestamp_preset'last_event < (now - last_clk_event)) then
            timestamp_i <= unsigned(timestamp_preset_val);
        end if;
    
        last_clk_event <= now;
        wait until rising_edge(clk_sys);

        if (prescaler > 1) then
            if (prescaler_ctr = prescaler - 1) then
                prescaler_ctr <= 0;
                timestamp_i <= timestamp_i + step;
            else
                prescaler_ctr <= prescaler_ctr + 1;
            end if;
        else
            timestamp_i <= timestamp_i + step;
        end if;

    end process;

    timestamp <= std_logic_vector(timestamp_i);
    
end architecture;