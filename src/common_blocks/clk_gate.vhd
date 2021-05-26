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
--  Clock gate - Latch + AND based clock gate
--
--  If ASIC technology is chosen, gate clocks. Otherwise, do not gate. Latch is
--  without reset since ICGs do not usually have reset. 
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.common_blocks_pkg.all;

entity clk_gate is
    generic (
        G_TECHNOLOGY       :       natural := 0
    );    
    port (
        -- Clock input
        clk_in             : in    std_logic;
        
        -- Clock Enable
        clk_en             : in    std_logic;

        -- Gated clocks
        clk_out            : out   std_logic
    );
end clk_gate;

architecture rtl of clk_gate is
    
    signal clk_en_q        :       std_logic;
    
begin

    g_tech_asic : if (G_TECHNOLOGY = C_TECH_ASIC) generate
        
        -- Latching enable - transparent in zero since we use AND gating
        clk_en_latch_proc : process(clk_in, clk_en)
        begin
            if (clk_in = '0') then
                clk_en_q <= clk_en;
            end if;
        end process;

        -- Gating
        clk_out <= clk_in AND clk_en_q;
        
    end generate g_tech_asic;


    g_tech_fpga : if (G_TECHNOLOGY = C_TECH_FPGA) generate
        
        -- Connect directly (always enabled, no gating)
        clk_out <= clk_in;
        
        -- Drive just to avoid un-driven signals warning by Synplify
        clk_en_q <= '1';
        
    end generate g_tech_fpga;

    assert (G_TECHNOLOGY = C_TECH_ASIC or G_TECHNOLOGY = C_TECH_FPGA);

end rtl;