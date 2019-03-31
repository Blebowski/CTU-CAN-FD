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

entity bit_error_detector is
    generic(
        -- Reset polarity
        G_RESET_POLARITY         :     std_logic
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                  :in   std_logic;
        
        -- Asynchronous reset
        res_n                    :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- CTU CAN FD Core is enabled
        drv_ena                  :in   std_logic;
        
        -- Sample control
        sp_control               :in   std_logic_vector(1 downto 0);
        
        -- RX Trigger - Nominal Bit time
        sample_nbt               :in   std_logic;
        
        -- RX Trigger - Data Bit time
        sample_dbt               :in   std_logic;
        
        -- RX Trigger - Secondary Sample
        sample_sec               :in   std_logic;

        -----------------------------------------------------------------------
        -- TX / RX Datapath
        -----------------------------------------------------------------------
        -- Actually transmitted data on CAN bus
        data_tx                  :in   std_logic;
        
        -- Delayed transmitted data (for detection in secondary sampling point)
        data_tx_delayed          :in   std_logic;
        
        -- Receieved data in Nominal Bit time
        data_rx_nbt              :in   std_logic;

        -- Received data (both Nominal Bit time and Data Bit time)
        can_rx_i                 :in   std_logic;

        -----------------------------------------------------------------------
        -- Status outputs
        -----------------------------------------------------------------------
        -- Bit error detected
        bit_error                : out std_logic
        );
end entity;

architecture rtl of bit_error_detector is

    -- Internal sample signal (muxed for NBT, DBT and SAMPLE)
    signal sample           : std_logic;
    
    -- Expected bit value (TX, from SYNC)
    signal exp_data         : std_logic;
    
    -- Actual data value (RX, from Sample point)
    signal act_data         : std_logic;

    -- Bit error detected value
    signal bit_error_d      : std_logic;
    signal bit_error_q      : std_logic;

begin

    ----------------------------------------------------------------------------
    -- Sample point multiplexor
    ----------------------------------------------------------------------------
    sample <= sample_nbt when (sp_control = NOMINAL_SAMPLE) else
              sample_dbt when (sp_control = DATA_SAMPLE) else
              sample_sec when (sp_control = SECONDARY_SAMPLE) else
              '0';
    
    ----------------------------------------------------------------------------
    -- Expected data mux. Choose between TX data and delayed TX Data
    ----------------------------------------------------------------------------
    exp_data <= data_tx_delayed when (sp_control = SECONDARY_SAMPLE) else
                data_tx;
    
    ----------------------------------------------------------------------------
    -- Actual data. Consider tripple sampling for Nominal Bit-rate
    ----------------------------------------------------------------------------
    act_data <= data_rx_nbt when (sp_control = NOMINAL_SAMPLE) else
                can_rx_i;
                
    ----------------------------------------------------------------------------
    -- Bit Error detection. If expected data is not equal to actual data in
    -- sample point -> Bit Error!
    ----------------------------------------------------------------------------
    bit_error_d <= '0' when (drv_ena = CTU_CAN_DISABLED) else
                   '1' when (exp_data /= act_data and sample = '1') else
                   '0' when (exp_data = act_data and sample = '1') else
                   bit_error_q;
    
    ----------------------------------------------------------------------------
    -- Bit error register
    ----------------------------------------------------------------------------
    bit_error_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            bit_error_q <= '0';
        elsif (rising_edge(clk_sys)) then
            bit_error_q <= bit_error_d;
        end if;
    end process;
    
    -- Propagation to output
    bit_error <= bit_error_q;

end architecture;