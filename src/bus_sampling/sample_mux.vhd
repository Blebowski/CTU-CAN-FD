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
--  Sample multiplexor.
--
-- Purpose:
--  Maintains a value sampled in previous sample point. This is then used
--  by other modules within Bus sampling.
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

entity sample_mux is
    port(
        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in   std_logic;
        
        -- Asynchronous reset
        res_n                :in   std_logic;       
        
        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- CTU CAN FD enabled
        drv_ena              :in   std_logic;
        
        -- Sample control (nominal, data, secondary)
        sp_control           :in   std_logic_vector(1 downto 0);
        
        -- RX Trigger
        rx_trigger           :in   std_logic;
        
        -- RX Trigger - Secondary Sampling
        sample_sec           :in   std_logic;

        -----------------------------------------------------------------------
        -- Datapath
        -----------------------------------------------------------------------
        -- RX Data (synchronised)
        data_rx_synced       :in   std_logic;

        -- Sampled value of RX pin in Sample point (DFF output)
        prev_sample          :out  std_logic
    );
end entity;

architecture rtl of sample_mux is

    -- Internal sample signal (muxed for NBT, DBT and SAMPLE)
    signal sample           : std_logic;

    -- Bit error detected value
    signal prev_sample_d    : std_logic;
    signal prev_sample_q    : std_logic;

begin

    ----------------------------------------------------------------------------
    -- Sample point multiplexor
    ----------------------------------------------------------------------------
    sample <= sample_sec when (sp_control = SECONDARY_SAMPLE) else
              rx_trigger;

    ----------------------------------------------------------------------------
    -- Previous sample register 
    ----------------------------------------------------------------------------
    prev_sample_d <= data_rx_synced when (sample = '1') else
                     prev_sample_q;

    sample_prev_req_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            prev_sample_q <= RECESSIVE;
        elsif (rising_edge(clk_sys)) then
            if (drv_ena = '1') then
                prev_sample_q <= prev_sample_d;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Internal signal to output propagation
    ----------------------------------------------------------------------------
    
    -- Internal signal to output propagation
    prev_sample <= prev_sample_q;

end architecture;