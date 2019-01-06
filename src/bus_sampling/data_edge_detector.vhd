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
-- Purpose:
--  Detection of edge on TX and RX data. Selectable DFF may be inserted on 
--  output.
-- 
--  Edge on RX Data is signaled only when:
--    1. Edge is detected on "rx_edge" input
--    2. Previously sampled value of RX data is different from value after
--       the edge.
--    3. Actual value of data is DOMINANT.
--  By these conditions it is satisfied that only RECESSIE to DOMINANT edge
--  with previous bit value detected as RECESSIVE is signalled. In CAN, this
--  is the only valid, HARD SYNCHRONISATION or RE-SYNCHRONISATION edge.
--
--  Edge on TX Data is signalled only when:
--    1. There is an edge on TX Data.
--    2. New TX-Data are dominant.
--
--  TX Edge is used for TRV DELAY measurement which is in EDL to R0 edge.
--  Thus only RECESSIVE to DOMINANT edge is needed.
--
--------------------------------------------------------------------------------
--    05.01.2018  Created file
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

entity data_edge_detector is
    generic(
        -- Reset polarity
        constant reset_polarity         :     std_logic;
        
        -- Pipeline TX edge output (insert DFF)
        constant tx_edge_pipeline       :     boolean := true;
        
        -- Pipeline RX edge output (insert DFF)
        constant rx_edge_pipeline       :     boolean := true
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        signal clk_sys                  :in   std_logic;
        signal res_n                    :in   std_logic;

        ------------------------------------------------------------------------
        -- Inputs
        ------------------------------------------------------------------------
        -- TX Data from CAN Core
        signal tx_data                  :in   std_logic;
        
        -- RX Data (synced from CAN Bus)
        signal rx_data                  :in   std_logic;
        
        -- RX Data value sampled in previous Sample point.
        signal prev_rx_sample           :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Outputs
        ------------------------------------------------------------------------
        -- Edge detected on TX
        signal tx_edge                  :out  std_logic;

        -- Edge detected on RX                                                
        signal rx_edge                  :out  std_logic
        );
end entity;


architecture rtl of data_edge_detector is
    
    -- Previous values on rx_data, tx_data inputs to detect edge
    signal rx_data_prev                 :       std_logic;
    signal tx_data_prev                 :       std_logic;

    -- Immediate edges on tx_data, rx_data (not yet finally valid)
    signal rx_edge_immediate            :       std_logic;
    signal tx_edge_immediate            :       std_logic;
    
    -- Valid edge
    signal rx_edge_valid                :       std_logic;
    signal tx_edge_valid                :       std_logic;
    
    -- Internal value of output signals
    signal rx_edge_i                    :       std_logic;
    signal tx_edge_i                    :       std_logic;
    
begin
    
    ----------------------------------------------------------------------------
    -- Registering previous value of rx_data, tx_data to detect edge in
    -- the stream
    ----------------------------------------------------------------------------
    data_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            rx_data_prev        <= RECESSIVE;
            tx_data_prev        <= RECESSIVE;
        elsif (rising_edge(clk_sys)) then
            rx_data_prev        <= rx_data;
            tx_data_prev        <= tx_data;
        end if;        
    end process;


    ----------------------------------------------------------------------------
    -- Immediate edges are detected when there is difference between input
    -- data (TX,RX) and its registered version.
    ----------------------------------------------------------------------------
    rx_edge_immediate <= '1' when (rx_data_prev /= rx_data) else
                         '0';
                         
    tx_edge_immediate <= '1' when (tx_data_prev /= tx_data) else
                         '0';


    ----------------------------------------------------------------------------
    -- Valid TX Edge:
    --  1. Edge on tx_data
    --  2. RECESSIVE to DOMINANT
    ----------------------------------------------------------------------------
    tx_edge_valid <= '1' when (tx_edge_immediate = '1') and 
                              (tx_data_prev = RECESSIVE)
                         else
                     '0';

    ----------------------------------------------------------------------------
    -- Valid RX Edge:
    --  1. Edge on rx_data
    --  2. RECESSIVE to DOMINANT
    --  3. Data sampled in previous Sample point are different from actual
    --     rx_data immediately after edge. 
    ----------------------------------------------------------------------------
    rx_edge_valid <= '1' when (rx_edge_immediate = '1') and 
                              (rx_data_prev = RECESSIVE) and
                              (prev_rx_sample /= rx_data)
                         else
                     '0';

    ----------------------------------------------------------------------------
    -- Driving rx_edge output
    ----------------------------------------------------------------------------
    rx_edge_pipeline_gen_true : if (rx_edge_pipeline) generate
        rx_edge_pipeline_proc : process(res_n, clk_sys)
        begin
            if (res_n = reset_polarity) then
                rx_edge_i <= '0';
            elsif (rising_edge(clk_sys)) then
                rx_edge_i <= rx_edge_valid;
            end if;
        end process;
    end generate rx_edge_pipeline_gen_true;

    rx_edge_pipeline_gen_false : if (not rx_edge_pipeline) generate
        rx_edge_i <= rx_edge_valid;
    end generate rx_edge_pipeline_gen_false;

    rx_edge <= rx_edge_i;
    

    ----------------------------------------------------------------------------
    -- Driving tx_edge output
    ----------------------------------------------------------------------------
    tx_edge_pipeline_gen_true : if (tx_edge_pipeline) generate
        tx_edge_pipeline_proc : process(res_n, clk_sys)
        begin
            if (res_n = reset_polarity) then
                tx_edge_i <= '0';
            elsif (rising_edge(clk_sys)) then
                tx_edge_i <= tx_edge_valid;
            end if;
        end process;
    end generate tx_edge_pipeline_gen_true;

    tx_edge_pipeline_gen_false : if (not tx_edge_pipeline) generate
        tx_edge_i <= tx_edge_valid;
    end generate tx_edge_pipeline_gen_false;
    
    tx_edge <= tx_edge_i;

end architecture;
