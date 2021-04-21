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
--    Clock generator agent.

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
--    04.2.2021   Adjusted to work without Vunits COM library.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.clk_gen_agent_pkg.all;


entity clk_gen_agent is
    port (
        -- Generated clock output
        clock_out   :   out std_logic := '0';
        clock_in    :   in  std_logic
    );
end entity;

architecture tb of clk_gen_agent is
    
    ---------------------------------------------------------------------------
    -- Parameters configured over communication library
    ---------------------------------------------------------------------------
    
    -- Clock generator is enabled, clocks are being generated!
    -- Note: This must be enabled by default when testbench is controlled from 
    --       SW, otherwise simulator will run out of events and timeout!
    signal enabled          :    boolean := false;
    
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
        variable cmd : integer;
        variable reply_code : integer;
    begin
        receive_start(default_channel, C_CLOCK_AGENT_ID);
        
        -- Command is sent as message type
        reply_code := C_REPLY_CODE_OK;
        cmd := com_channel_data.get_msg_code;

        case cmd is
        when CLK_AGNT_CMD_START =>
            enabled <= true;

        when CLK_AGNT_CMD_STOP =>
            enabled <= false;

        when CLK_AGNT_CMD_PERIOD_SET =>
            period <= com_channel_data.get_param;
            recalc_parameters(period, duty, t_low, t_high);

        when CLK_AGNT_CMD_PERIOD_GET =>
            com_channel_data.set_param(period);

        when CLK_AGNT_CMD_JITTER_SET =>
            jitter <= com_channel_data.get_param;

        when CLK_AGNT_CMD_JITTER_GET =>
            com_channel_data.set_param(jitter);

        when CLK_AGNT_CMD_DUTY_SET =>
            duty <= com_channel_data.get_param;
            recalc_parameters(period, duty, t_low, t_high);

        when CLK_AGNT_CMD_DUTY_GET =>
            com_channel_data.set_param(duty);
            
        when CLK_AGNT_CMD_WAIT_CYCLE =>
            wait until rising_edge(clock_in) for 1 us;

        when others =>
            reply_code := C_REPLY_CODE_ERR;
        end case;

        receive_finish(default_channel, reply_code);
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
        variable jitter_div_2 : time := jitter / 2;
    begin
        if (enabled) then
            
            if (jitter > 0 fs) then
                uniform(seed1, seed2, rand_real);
                rand_jitter := jitter * rand_real;
                t_high_with_jitter := (period + rand_jitter - jitter_div_2) * (real(duty) / 100.0);
                t_low_with_jitter := (period + rand_jitter - jitter_div_2) * (1.0 - (real(duty) / 100.0));

                clock_out <= '1';
                wait for t_high_with_jitter;
                clock_out <= '0';
                wait for t_low_with_jitter;
            else
                clock_out <= '1';
                wait for t_high;
                clock_out <= '0';
                wait for t_low;
            end if;
        else
            wait until enabled;
        end if;
    end process;

end architecture;