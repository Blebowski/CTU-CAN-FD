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
--  Single Interrupt module.
--
-- Purpose:
--  Contains Interrupt enable, Interrupt mask and Interrupt status.
--
--  Interrupt status can be set by "int_set" and cleared by "int_clear".
--  Preference is selected by "clear_priority". If "clear_priority = true",
--  clear has priority over set, otherwise set has priority over clear.
--
--  Interrupt mask is set by "int_mask_set" and cleared by "int_mask_clear".
--  Simulteneous set/clear of interrupt mask is not allowed!
--
--  Interrupt enable is set by "int_ena_set" and cleared by "int_ena_clear".
--  Simulteneous set/clear of interrupt mask is not allowed!
--
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

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

entity int_module is
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System Clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous Reset
        res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Control control signals
        ------------------------------------------------------------------------
        -- Interrupt Status Set
        int_status_set         :in   std_logic;
        
        -- Interrupt Status Clear
        int_status_clear       :in   std_logic;

        -- Interrupt Mask Set
        int_mask_set           :in   std_logic;
        
        -- Interrupt Mask Clear
        int_mask_clear         :in   std_logic;

        -- Interrupt Enable Set
        int_ena_set            :in   std_logic;
        
        -- Interrupt Enable Clear
        int_ena_clear          :in   std_logic;

        ------------------------------------------------------------------------
        -- Interrupt output signals
        ------------------------------------------------------------------------
        -- Interrupt status (Interrupt vector)
        int_status             :out  std_logic;
        
        -- Interrupt mask
        int_mask               :out  std_logic;
        
        -- Interrupt enable
        int_ena                :out  std_logic
    );  
end entity;

architecture rtl of int_module is

    -- Internal values 
    signal int_mask_i               : std_logic;
    signal int_ena_i                : std_logic;

    -- Interrupt mask handling signals
    signal int_mask_load            : std_logic;
    signal int_mask_next            : std_logic;

begin

    ------------------------------------------------------------------------
    -- Interrupt status - Set priority 
    ------------------------------------------------------------------------
    int_stat_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            int_status <= '0';

        elsif rising_edge(clk_sys) then
          
            -- Setting Interrupt
            if (int_status_set = '1' and int_mask_i = '0') then
                int_status <= '1';

            -- Clearing Interrupt
            elsif (int_status_clear = '1') then
                int_status <= '0';

            end if;
        end if;
    end process;


    ------------------------------------------------------------------------
    -- Interrupt mask
    ------------------------------------------------------------------------

    int_mask_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            int_mask_i <= '0';

        elsif rising_edge(clk_sys) then
          
            -- Setting / Clearing Interrupt Mask
            if (int_mask_load = '1') then
                int_mask_i <= int_mask_next;
            end if;

        end if;
    end process;

    int_mask_load        <= int_mask_set or int_mask_clear;
    int_mask_next        <= '1' when (int_mask_set = '1')
                                else
                            '0';

    ------------------------------------------------------------------------
    -- Interrupt Enable
    ------------------------------------------------------------------------
    int_ena_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            int_ena_i <= '0';

        elsif rising_edge(clk_sys) then
          
            -- Setting Interrupt Mask
            if (int_ena_set = '1') then
                int_ena_i <= '1';

            -- Clearing Interrupt Mask
            elsif (int_ena_clear = '1') then
                int_ena_i <= '0';

            end if;
        end if;
    end process;

    -- Propagation to outputs
    int_mask <= int_mask_i;
    int_ena  <= int_ena_i;

end architecture;