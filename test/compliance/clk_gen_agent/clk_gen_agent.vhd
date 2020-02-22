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
--    Clock generator agent. Configurable over Vunit Communication library.
--
--    More on Vunit and its communication library can be found at:
---     https://vunit.github.io/documentation.html
--      https://vunit.github.io/com/user_guide.html

--    Clock generator agent generates clock (whoooa thats surprise right).
--    
--    Clock generator agent only generates clock when it is enabled. When it is
--    disabled, its clock output remains in value which clock output had at 
--    time when it was disabled.
--    
--    Clock generator agent has following configurable parameters:
--      Clock period        Obvious who does not know what clock period is should
--                          not be reading this text.
--      Jitter              Cycle to cycle jitter. Each clock cycle can last:
--                          period +- "rand_val"  where "rand_val" is less than
--                          or equal to jitter.
--      Duty cycle          Again, obvious, if you don't know what duty cycle
--                          is, seriously, stop reading!
--  
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use ieee.math_real.uniform;
use ieee.math_real.floor;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

Library work;
use work.clk_gen_agent_pkg.all;

entity clk_gen_agent is
    port (
        -- Generated clock output
        clock   :   out std_logic := '0'
    );
end entity;

architecture tb of clk_gen_agent is
    
    ---------------------------------------------------------------------------
    -- Parameters configured over communication library
    ---------------------------------------------------------------------------
    
    -- Clock generator is enabled, clocks are being generated!
    -- Note: This must be enabled by default when testbench is controlled from 
    --       SW, otherwise simulator will run out of events and timeout!
    signal enabled          :    boolean := true;
    
    -- Generated clock period
    signal period           :    time := 10 ns;
    
    -- Duty cycle of generated clock (in %)
    signal duty             :    integer range 0 to 100 := 50;

    -- Jitter of generated clock (short time -> cycle to cycle)
    signal jitter           :    time := 1 fs;

    -- High / Low times
    signal t_low            :    time;
    signal t_high           :    time;

    procedure recalc_parameters(
        p_period            :   in  time;
        p_duty              :   in  integer range 0 to 100;
        signal p_t_low      :   out time;
        signal p_t_high     :   out time
    )is
    begin
       p_t_low  <= p_period * (real(p_duty) / 100.0);
       p_t_high <= p_period * (1.0 - (real(p_duty) / 100.0));
    end;

begin
    
    ---------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable msg, ack_msg : msg_t := new_msg;
        variable cmd : integer;
    begin
        receive(net, actor_clk_gen_agent, msg);
        
        -- Command is sent as message type
        cmd := message_type(msg).p_code;
        ack_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_REPLY_OK));

        case cmd is
        when CLK_AGNT_CMD_START =>
            enabled <= true;

        when CLK_AGNT_CMD_STOP =>
            enabled <= false;

        when CLK_AGNT_CMD_PERIOD_SET =>
            period <= pop(msg);
            recalc_parameters(period, duty, t_low, t_high);

        when CLK_AGNT_CMD_PERIOD_GET =>
            push(ack_msg, period);

        when CLK_AGNT_CMD_JITTER_SET =>
            jitter <= pop(msg);

        when CLK_AGNT_CMD_JITTER_GET =>
            push(ack_msg, period);

        when CLK_AGNT_CMD_DUTY_SET =>
            duty <= pop(msg);
            recalc_parameters(period, duty, t_low, t_high);

        when CLK_AGNT_CMD_DUTY_GET =>
            push(ack_msg, duty);

        when others =>
            info ("Invalid message type: " & integer'image(cmd));
            ack_msg := new_msg(msg_type => (p_code => CLK_AGNT_CMD_REPLY_ERR));
        end case;

        reply(net, msg, ack_msg);
    end process;
    
    ---------------------------------------------------------------------------
    -- Clock generation process
    ---------------------------------------------------------------------------
    clk_gen_proc : process
        variable t_high_with_jitter : time := t_high;
        variable t_low_with_jitter : time := t_low;
        variable rand_real : real;
        variable rand_jitter : time;
        variable seed1 : positive := 1;
        variable seed2 : positive := 1;
    begin
        if (enabled) then
            
            if (jitter > 0 fs) then
                uniform(seed1, seed2, rand_real);
                rand_jitter := jitter * (0.5 - rand_real);
                t_high_with_jitter := (period + rand_jitter) * (real(duty) / 100.0);
                t_low_with_jitter := (period + rand_jitter) * (1.0 - (real(duty) / 100.0));
            end if;
            
            clock <= '1';
            wait for t_high_with_jitter;
            clock <= '0';
            wait for t_low_with_jitter;
        else
            wait until enabled;
        end if;
    end process;

end architecture;