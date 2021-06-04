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
--  Endian swapper
-- 
-- Purpose:
--  Swaps endianity of input vector. Size of byte (group) is configurable. Word
--  size and selection by generic or input signal is configurable. Output is
--  combinatorial.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity endian_swapper is 
    generic (

        -- When true, output word is endian swapped as long as "swap_by_signal"
        -- is true. Otherwise it has no meaning.
        G_SWAP_GEN              :     boolean := false;

        -- Size of word (in groups)
        G_WORD_SIZE             :     natural := 4;
        
        -- Size of group (in bits)
        G_GROUP_SIZE            :     natural := 8  
    );  
    port (
        -- Data input
        input   : in  std_logic_vector(G_WORD_SIZE * G_GROUP_SIZE - 1 downto 0);
        
        -- Data output
        output  : out std_logic_vector(G_WORD_SIZE * G_GROUP_SIZE - 1 downto 0)
    );
end entity;

architecture rtl of endian_swapper is
    
    -- Endian swapped input word
    signal swapped :  std_logic_vector(G_WORD_SIZE * G_GROUP_SIZE - 1 downto 0);

begin
    
    ---------------------------------------------------------------------------
    -- Endian swap implementation
    ---------------------------------------------------------------------------
    swap_proc : process(input)
        variable l_ind_orig  : natural;
        variable u_ind_orig  : natural;
        variable l_ind_swap  : natural;
        variable u_ind_swap  : natural;
        variable i_inv       : natural;
    begin
        for i in 0 to G_WORD_SIZE - 1 loop
            l_ind_orig := i * G_GROUP_SIZE;
            u_ind_orig := (i + 1) * G_GROUP_SIZE - 1;
            i_inv := G_WORD_SIZE - i - 1;
            l_ind_swap := i_inv * G_GROUP_SIZE;
            u_ind_swap := (i_inv + 1) * G_GROUP_SIZE - 1;
            swapped(u_ind_swap downto l_ind_swap) <=
                input(u_ind_orig downto l_ind_orig);
        end loop;
    end process;

    ---------------------------------------------------------------------------
    -- Swapping by generic
    ---------------------------------------------------------------------------

    -- Swap
    swap_by_generic_true_gen : if (G_SWAP_GEN) generate
        output <= swapped;    
    end generate swap_by_generic_true_gen;
    
    -- Don't Swap
    swap_by_generic_false_gen : if (not G_SWAP_GEN) generate
        output <= input;    
    end generate swap_by_generic_false_gen;

end architecture;
