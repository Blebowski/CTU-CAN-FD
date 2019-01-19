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
--  Jiri Novak <jnovak@fel.cvut.cz>
--  Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
-- Purpose:
--  Generic endian swapper. Selectable word size, group(byte) size.
--  Selectable generic swapping, or swapping by input signal.
--
--  Output is combinational
--------------------------------------------------------------------------------
--    19.1.2018  Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.endian_swap.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity endian_swapper is 
    generic (
        
        -- If true, "swap_in" signal selects between swapping/non-swapping.
        -- If false "swap_gen" generic selects bewtween swapping/non-swapping.
        constant swap_by_signal        :     boolean := false;
        
        -- When true, output word is endian swapped as long as "swap_by_signal"
        -- is true. Otherwise it has no meaning.
        constant swap_gen              :     boolean := false;

        -- Size of word (in groups)
        constant word_size             :     natural := 4;
        
        -- Size of group (in bits)
        constant group_size            :     natural := 8  
    );  
    port (
        
        -- Data input/output
        signal input   : in  std_logic_vector(word_size * group_size - 1 downto 0);
        signal output  : out std_logic_vector(word_size * group_size - 1 downto 0);
        
        -- Swap signal (used only when "swap_by_signal=true")
        -- Swaps endian when '1', keeps otherwise.
        signal swap_in : in  std_logic
    );
end entity;

architecture rtl of endian_swapper is
    
    -- Endian swapped input word
    signal swapped         :  std_logic_vector(word_size * group_size - 1 downto 0);

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
        for i in 0 to word_size - 1 loop
            l_ind_orig := i * group_size;
            u_ind_orig := (i + 1) * group_size - 1;
            i_inv := word_size - i - 1;
            l_ind_swap := i_inv * group_size;
            u_ind_swap := (i_inv + 1) * group_size - 1;
            swapped(u_ind_swap downto l_ind_swap) <=
                input(u_ind_orig downto l_ind_orig);
        end loop;
    end process;
    
    ---------------------------------------------------------------------------
    -- Swapping by generic
    ---------------------------------------------------------------------------
    swap_by_generic_gen : if (swap_by_signal = false) generate
        
        -- Swap
        swap_by_generic_true_gen : if (swap_gen) generate
            output <= swapped;    
        end generate swap_by_generic_true_gen;
        
        -- Don't Swap
        swap_by_generic_false_gen : if (not swap_gen) generate
            output <= input;    
        end generate swap_by_generic_false_gen;

    end generate swap_by_generic_gen;

    
    ---------------------------------------------------------------------------
    -- Swapping by input    
    ---------------------------------------------------------------------------    
    swap_by_input_gen : if (swap_by_signal = true) generate
        output <= swapped when (swap_in = '1') else
                  input;
    end generate swap_by_input_gen; 
end architecture;