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
-- Module:
--  Byte shift register.
--
-- Purpose:
--  Shift register which is split into bytes. Operates in two modes: Linear mode
--  and Byte mode. Number of Bytes is configurable. In Linear mode, shift register
--  forms one long shift register and input to each byte is output from previous
--  byte. In Byte mode, shift register forms N byte shift regsiters and input to
--  each byte is directly from input of shift register. Each byte has separate
--  clock enable. No pre-load is implemented.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

entity shift_reg_byte is
    generic (
        -- Reset polarity
        G_RESET_POLARITY     :       std_logic;
        
        -- Reset value
        G_RESET_VALUE        :       std_logic_vector;
        
        -- Shift register width
        G_NUM_BYTES          :       natural
    );
    port (
        -----------------------------------------------------------------------
        -- Clock and Asyncrhonous reset
        -----------------------------------------------------------------------
        -- Clock
        clk             : in    std_logic;

        -- Asynchronous reset
        res_n           : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Shift register input        
        input           : in    std_logic;

        -- Clock enable for shifting each byte of the shift register.
        byte_clock_ena  : in    std_logic_vector(G_NUM_BYTES - 1 downto 0);

        -- Input source selector for each byte
        -- (0-Previous byte output, 1- Shift reg input)
        byte_input_sel  : in    std_logic_vector(G_NUM_BYTES - 1 downto 0);

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Shift register status
        reg_stat        : out   std_logic_vector(8 * G_NUM_BYTES - 1 downto 0)
    );
end shift_reg_byte;

architecture rtl of shift_reg_byte is

    type t_byte_shift_reg is array (0 to G_NUM_BYTES - 1) of
        std_logic_vector(7 downto 0);

    signal shift_reg_q : t_byte_shift_reg;
    signal shift_reg_in   : std_logic_vector(G_NUM_BYTES - 1 downto 0);
    
begin

    byte_shift_reg_gen : for i in 0 to G_NUM_BYTES - 1 generate
    begin

        first_byte_gen : if (i = 0) generate
            shift_reg_in(i) <= input;
        end generate;

        -- Shift register input mux
        next_bytes_gen : if (i > 0) generate
            shift_reg_in(i) <= shift_reg_q(i - 1)(7) when (byte_input_sel(i) = '0')
                                                     else
                                               input;
        end generate;

        -----------------------------------------------------------------------
        -- Shift register assignment
        -----------------------------------------------------------------------
        shift_reg_proc : process(clk, res_n)
        begin
            if (res_n = G_RESET_POLARITY) then
                shift_reg_q(i) <= (OTHERS => '0'); -- G_RESET_VALUE(i * 8 + 7 downto i * 8);
            elsif (rising_edge(clk)) then
                if (byte_clock_ena(i) = '1') then
                    shift_reg_q(i) <= shift_reg_q(i)(6 downto 0) &
                                         shift_reg_in(i);
                end if;
            end if;
        end process;
        
        -- Propagation to output
        reg_stat(i * 8 + 7 downto i * 8) <= shift_reg_q(i);
        
    end generate;

end rtl;
