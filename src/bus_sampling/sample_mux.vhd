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
--  Detects bit error.
--------------------------------------------------------------------------------
--    28.02.2019  Created file
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

entity sample_mux is
    generic(
        -- Reset polarity
        constant reset_polarity         :     std_logic;
        
        -- Pipeline data output
        constant pipeline_sampled_data  :     boolean := true
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        signal clk_sys                  :in   std_logic;
        signal res_n                    :in   std_logic;       
        
        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- CTU CAN FD enabled
        signal drv_ena                  :in   std_logic;
        
        -- Sample control (nominal, data, secondary)
        signal sp_control               :in   std_logic_vector(1 downto 0);
        
        -- Input sample signals
        signal sample_nbt               :in   std_logic;
        signal sample_dbt               :in   std_logic;
        signal sample_sec               :in   std_logic;

        -----------------------------------------------------------------------
        -- RX Data inputs
        -----------------------------------------------------------------------
        -- Receieved data in Nominal Bit time (either directly sampled data,
        -- or tripple sampling output)
        signal data_rx_nbt              :in   std_logic;

        -- Received data (Nominal and Data)
        signal can_rx_i                 :in   std_logic;

        ------------------------------------------------------------------------
        -- Outputs
        ------------------------------------------------------------------------
        -- Sampled value of RX in Sample point (DFF output)
        signal prev_sample              : out std_logic;
        
        -- Sampled value of RX in Sample point (either DFF or direct output)
        signal data_rx                  : out std_logic
        );
end entity;

architecture rtl of sample_mux is

    -- Internal sample signal (muxed for NBT, DBT and SAMPLE)
    signal sample           : std_logic;
    
    -- RX Data
    signal rx_data_i        : std_logic;

    -- Bit error detected value
    signal sample_prev_d    : std_logic;
    signal sample_prev_q    : std_logic;

begin

    ----------------------------------------------------------------------------
    -- Sample point multiplexor
    ----------------------------------------------------------------------------
    sample <= sample_nbt when (sp_control = NOMINAL_SAMPLE) else
              sample_dbt when (sp_control = DATA_SAMPLE) else
              sample_sec when (sp_control = SECONDARY_SAMPLE) else
              '0';

    ----------------------------------------------------------------------------
    -- RX data mux.
    ----------------------------------------------------------------------------
    rx_data_i <= data_rx_nbt when (sp_control = NOMINAL_SAMPLE) else
                 can_rx_i;

    ----------------------------------------------------------------------------
    -- Previous sample register 
    ----------------------------------------------------------------------------
    sample_prev_d <= rx_data_i when (sample = '1') else
                     sample_prev_q;

    sample_prev_req_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            sample_prev_q <= RECESSIVE;
        elsif (rising_edge(clk_sys)) then
            if (drv_ena = '1') then
                sample_prev_q <= sample_prev_d;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Receive data. If pipeline is inserted, then use directly sample_prev,
    -- if not, then pipe the input to output directly! 
    ----------------------------------------------------------------------------
    insert_pipeline_true_gen : if (pipeline_sampled_data = true) generate
        data_rx <= sample_prev_q;
    end generate insert_pipeline_true_gen;
    
    insert_pipeline_false_gen : if (pipeline_sampled_data = false) generate
        data_rx <= rx_data_i;
    end generate insert_pipeline_false_gen;
    
    -- Internal signal to output propagation
    prev_sample <= sample_prev_q;

end architecture;