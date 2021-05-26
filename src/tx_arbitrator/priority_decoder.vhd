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
--  Priority decoder.
-- 
-- Purpose:
--  Combinational decoder for TXT BufferS priority. Considers priority, bufffer
--  validity. Generic amount of buffers is available (up to 8). Decoder consists
--  of 3 levels of comparators (4+2+1). If two frames have the same priority,
--  a frame with lower index is selected.                                                                                                                                               
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity priority_decoder is
    generic(
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT     : natural range 2 to 8
    );
    port( 
        ------------------------------------------------------------------------
        -- TXT Buffer information
        ------------------------------------------------------------------------
        -- TXT Buffer priority
        prio             : in  t_txt_bufs_priorities(G_TXT_BUFFER_COUNT - 1 downto 0);
        
        -- TXT Buffer is valid for selection
        prio_valid       : in  std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        ------------------------------------------------------------------------
        -- Output interface
        ------------------------------------------------------------------------
        -- Whether selected buffer is valid 
        -- (at least one of the buffers must be non-empty and allowed)
        output_valid     : out  std_logic;

        -- Index of highest priority buffer which is non-empty and allowed
        -- for transmission
        output_index     : out  natural range 0 to G_TXT_BUFFER_COUNT - 1
    );
end entity;

architecture rtl of priority_decoder is

    ----------------------------------------------------------------------------
    -- Level 0 aliases for input signals to provide variable signal width
    ----------------------------------------------------------------------------
    type level0_priority_type  is array (7 downto 0) of
          std_logic_vector(2 downto 0);
    signal l0_prio        : level0_priority_type;
    signal l0_valid       : std_logic_vector(7 downto 0);


    ----------------------------------------------------------------------------
    -- Level 1 priorities and valid indicators
    ----------------------------------------------------------------------------
    type level1_priority_type  is array (3 downto 0) of
          std_logic_vector(2 downto 0);
    type level1_comp_valid_type is array (3 downto 0) of
          std_logic_vector(1 downto 0);
    signal l1_prio        : level1_priority_type;
    signal l1_valid       : std_logic_vector(3 downto 0);
    signal l1_winner      : std_logic_vector(3 downto 0);


    ----------------------------------------------------------------------------
    -- Level 2 priorities and valid indicators
    ----------------------------------------------------------------------------
    type level2_priority_type  is array (1 downto 0) of
          std_logic_vector(2 downto 0);
    type level2_comp_valid_type is array (1 downto 0) of
          std_logic_vector(1 downto 0);
    signal l2_prio        : level2_priority_type;
    signal l2_valid       : std_logic_vector(1 downto 0);
    signal l2_winner      : std_logic_vector(1 downto 0);

    ----------------------------------------------------------------------------
    -- Level 3, we dont need the priorities, we only need the
    -- outcome which of them is bigger (since there is no next stage)
    ----------------------------------------------------------------------------
    signal l3_valid       : std_logic;
    signal l3_winner      : std_logic;

    constant LOWER_TREE   : std_logic := '0';
    constant UPPER_TREE   : std_logic := '1';  

begin
  
    ----------------------------------------------------------------------------
    -- Level 0 - aliases
    ----------------------------------------------------------------------------
    l0_gen : for i in 0 to G_TXT_BUFFER_COUNT - 1 generate

        -- pragma translate_off
        
        -- Since we cover "00" as inactive value, instead of active values 
        -- "01", "10" or "11", rather make sure that input values are defined
        l0_val_proc : process(prio_valid(i))
        begin
            if (prio_valid(i) /= '0' and prio_valid(i) /= '1' and now /= 0 fs) then
                report "Input values not exactly defined" severity error;
            end if;
        end process;
        -- pragma translate_on

        l0_prio(i)  <= prio(i);
        l0_valid(i) <= prio_valid(i);

    end generate;

  
    fill_zeroes_gen : if (G_TXT_BUFFER_COUNT < 8) generate
        l0_prio(7 downto G_TXT_BUFFER_COUNT)  <= (OTHERS => (OTHERS => '0'));
        l0_valid(7 downto G_TXT_BUFFER_COUNT) <= (OTHERS => '0');
    end generate;
    
    
    ----------------------------------------------------------------------------
    -- Level 1 comparators
    ----------------------------------------------------------------------------
    l1_prio_dec_proc : process(l0_valid, l0_prio)
        variable tmp : level1_comp_valid_type := (OTHERS => (OTHERS => '0'));
    begin
        for i in 0 to 3 loop
            tmp(i) := l0_valid(2 * i + 1 downto 2 * i);
            case tmp(i) is
            when "01" =>
                l1_prio(i)      <= l0_prio(2 * i);
                l1_valid(i)     <= '1'; 
                l1_winner(i)    <= LOWER_TREE;

            when "10" =>
                l1_prio(i)      <= l0_prio(2 * i + 1);
                l1_valid(i)     <= '1'; 
                l1_winner(i)    <= UPPER_TREE;

            when "11" => 
                if (unsigned(l0_prio(2 * i + 1)) > unsigned(l0_prio(2 * i))) then
                    l1_prio(i)    <= l0_prio(2 * i + 1);
                    l1_winner(i)  <= UPPER_TREE;
                else
                    l1_prio(i)    <= l0_prio(2 * i);
                    l1_winner(i)  <= LOWER_TREE;
                end if;
                l1_valid(i)     <= '1';  

            when "00" =>
                l1_valid(i)     <= '0';
                l1_prio(i)      <= l0_prio(2 * i + 1);
                l1_winner(i)    <= UPPER_TREE;

            when others => 
                l1_valid(i)     <= '0';
                l1_prio(i)      <= l0_prio(2 * i + 1);
                l1_winner(i)    <= UPPER_TREE;

            end case;
        end loop;
    end process; 


    ----------------------------------------------------------------------------
    -- Level 2 comparators
    ----------------------------------------------------------------------------
    l2_prio_dec_proc : process(l1_valid, l1_prio)
        variable tmp : level2_comp_valid_type := (OTHERS => (OTHERS => '0'));
    begin
        for i in 0 to 1 loop
            tmp(i) := l1_valid(2 * i + 1 downto 2 * i);

            case tmp(i) is
            when "01" => 
                l2_prio(i)      <= l1_prio(2 * i);
                l2_valid(i)     <= '1'; 
                l2_winner(i)    <= LOWER_TREE;

            when "10" => 
                l2_prio(i)      <= l1_prio(2 * i + 1);
                l2_valid(i)     <= '1'; 
                l2_winner(i)    <= UPPER_TREE;

            when "11" => 
                if (unsigned(l1_prio(2 * i + 1)) > unsigned(l1_prio(2 * i))) then
                    l2_prio(i)    <= l1_prio(2 * i + 1);
                    l2_winner(i)  <= UPPER_TREE;
                else
                    l2_prio(i)    <= l1_prio(2 * i);
                    l2_winner(i)  <= LOWER_TREE;
                end if;
                l2_valid(i)     <= '1';  
             
            when "00" =>
                l2_valid(i)     <= '0';
                l2_prio(i)      <= l1_prio(2 * i + 1);
                l2_winner(i)    <= UPPER_TREE;

            when others => 
                l2_valid(i)     <= '0';
                l2_prio(i)      <= l1_prio(2 * i + 1);
                l2_winner(i)    <= UPPER_TREE;

            end case;
        end loop;  
    end process;
  
  
    ----------------------------------------------------------------------------
    -- Level 3 comparators
    ----------------------------------------------------------------------------
  
    -- Here we have only one comparator, plus we dont need
    -- the priority on the output...  

    -- Validity of level 3 is also the output validity
    l3_valid  <= '0' when l2_valid(1 downto 0) = "00" 
                   else
                '1';
    output_valid <= '1' when l3_valid = '1' else '0';

    -- Priority comparator of level 3
    l3_winner  <= LOWER_TREE when l2_valid(1 downto 0) = "01" else
                UPPER_TREE when l2_valid(1 downto 0) = "10" else
                UPPER_TREE when (l2_valid(1 downto 0) = "11" and
                                 unsigned(l2_prio(1)) > unsigned(l2_prio(0))) 
                           else 
                LOWER_TREE;
   

    ----------------------------------------------------------------------------
    -- Output index decoder
    ----------------------------------------------------------------------------
    -- Just find out the winning buffer index from decisions in the tree

    -- Note that modulo is used only for purpose of getting rid of
    -- compiler warnings. If lower amount of buffers is on input 
    -- (TXT_BUFFER_COUNT), then higher indices of input priorities and
    -- valid sinals are set to 0. This leads to the case that "buf_index" 
    -- will NEVER be assigned value higher than its available
    -- range.
   
    out_index_proc : process(l3_winner, l2_winner, l1_winner)
    begin
        if (l3_winner = LOWER_TREE) then
            if (l2_winner(0) = LOWER_TREE) then
                if (l1_winner(0) = LOWER_TREE) then
                    output_index <= 0;
                else
                    output_index <= 1;
                end if;
            else
                if (l1_winner(1) = LOWER_TREE) then
                    output_index <= 2 mod G_TXT_BUFFER_COUNT;
                else
                    output_index <= 3 mod G_TXT_BUFFER_COUNT;
                end if;
            end if;      
        else
            if (l2_winner(1) = LOWER_TREE) then
                if (l1_winner(2) = LOWER_TREE) then
                    output_index <= 4 mod G_TXT_BUFFER_COUNT;
                else
                    output_index <= 5 mod G_TXT_BUFFER_COUNT;
                end if;
            else
                if (l1_winner(3) = LOWER_TREE) then
                    output_index <= 6 mod G_TXT_BUFFER_COUNT;
                else
                    output_index <= 7 mod G_TXT_BUFFER_COUNT;
                end if;
            end if;   
        end if;
    end process;     
  
end architecture;