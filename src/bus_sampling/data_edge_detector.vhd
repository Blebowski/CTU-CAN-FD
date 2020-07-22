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
-- Module:
--  Data edge detector.
--
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
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity data_edge_detector is
    generic(
        -- Reset polarity
        G_RESET_POLARITY         :     std_logic
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                  :in   std_logic;
        
        -- Asynchronous Reset
        res_n                    :in   std_logic;

        ------------------------------------------------------------------------
        -- Inputs
        ------------------------------------------------------------------------
        -- TX Data from CAN Core
        tx_data                  :in   std_logic;
        
        -- RX Data (from CAN Bus)
        rx_data                  :in   std_logic;
        
        -- RX Data value from previous Sample point.
        prev_rx_sample           :in   std_logic;
        
        -- Time quanta edge
        tq_edge                  :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Outputs
        ------------------------------------------------------------------------
        -- Edge detected on TX Data
        tx_edge                  :out  std_logic;

        -- Edge detected on RX Data                                             
        rx_edge                  :out  std_logic;
        
        -- Synchronisation edge
        sync_edge                :out  std_logic
    );
end entity;


architecture rtl of data_edge_detector is
    
    -- Previous values on rx_data, tx_data inputs to detect edge
    signal rx_data_prev                 :       std_logic;
    signal tx_data_prev                 :       std_logic;
    signal rx_data_sync_prev            :       std_logic;
    
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
        if (res_n = G_RESET_POLARITY) then
            rx_data_prev        <= RECESSIVE;
            tx_data_prev        <= RECESSIVE;
            rx_data_sync_prev   <= RECESSIVE;
        elsif (rising_edge(clk_sys)) then
            rx_data_prev        <= rx_data;
            tx_data_prev        <= tx_data;
            
            if (tq_edge = '1') then
                rx_data_sync_prev <= rx_data;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Valid TX Edge (Used to start transceiver delay measurement):
    --  1. Edge on tx_data
    --  2. RECESSIVE to DOMINANT
    ----------------------------------------------------------------------------
    tx_edge_i <= '1' when (tx_data_prev /= tx_data) and 
                          (tx_data_prev = RECESSIVE)
                     else
                 '0';

    ----------------------------------------------------------------------------
    -- Valid RX Edge (used to stop transceiver delay measurement)
    --  1. Edge on rx_data
    --  2. RECESSIVE to DOMINANT
    ----------------------------------------------------------------------------
    rx_edge_i <= '1' when (rx_data_prev /= rx_data) and 
                          (rx_data_prev = RECESSIVE)
                     else
                 '0';

    ----------------------------------------------------------------------------
    -- Synchronisation edge:
    --  1. Edge on RX data, aligned with Time Quanta
    --  2. Recessive to Dominant
    --  3. Data sampled in previous Sample point are different from actual
    --     rx_data immediately after edge!
    --  4. Aligned with time quanta!
    ----------------------------------------------------------------------------
    sync_edge <= '1' when (rx_data_sync_prev /= rx_data) and
                          (rx_data_sync_prev = RECESSIVE) and
                          (prev_rx_sample /= rx_data) and
                          (tq_edge = '1')
                     else
                 '0';

    ----------------------------------------------------------------------------
    -- Internal signals to output propagation
    ----------------------------------------------------------------------------
    rx_edge <= rx_edge_i;
    tx_edge <= tx_edge_i;


end architecture;