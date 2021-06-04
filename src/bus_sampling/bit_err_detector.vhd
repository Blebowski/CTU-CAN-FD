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
--  Bit error detector.
--
-- Purpose:
--  Detects bit error in:
--   1. Regular sampling point (compares actual TX data to actual RX data),
--      in Nominal Bit Rate.
--   2. Secondary sampling point (compares actual RX data to delayed TX data),
--      in Data Bit-Rate of Transmitter (Secondary sampling).
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity bit_err_detector is
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
        
        -- RX Trigger
        rx_trigger               :in   std_logic;
        
        -- RX Trigger - Secondary Sample
        sample_sec               :in   std_logic;

        -----------------------------------------------------------------------
        -- TX / RX Datapath
        -----------------------------------------------------------------------
        -- Actually transmitted data on CAN bus
        data_tx                  :in   std_logic;
        
        -- Delayed transmitted data (for detection in secondary sampling point)
        data_tx_delayed          :in   std_logic;
        
        -- RX Data (Synchronised)
        data_rx_synced           :in   std_logic;

        -----------------------------------------------------------------------
        -- Status outputs
        -----------------------------------------------------------------------
        -- Bit error detected
        bit_err                  : out std_logic
    );
end entity;

architecture rtl of bit_err_detector is

    -- Bit error detected value
    signal bit_err_d      : std_logic;
    signal bit_err_q      : std_logic;

    -- Capture register for Secondary sampling point bit error
    signal bit_err_ssp_capt_d  : std_logic;
    signal bit_err_ssp_capt_q  : std_logic;
    
    -- Valid Bit error detected by Secondary sampling
    signal bit_err_ssp_valid        : std_logic;
    signal bit_err_ssp_condition    : std_logic; 
    
    -- Valid Bit Error detected by regular sampling
    signal bit_err_norm_valid  : std_logic;

begin

    ----------------------------------------------------------------------------
    -- Condition for SSP bit error is valid when TX Data cache output is not
    -- equal to CAN RX in the moment of SSP!
    ----------------------------------------------------------------------------    
    bit_err_ssp_condition <= '1' when (data_tx_delayed /= data_rx_synced and
                                       sample_sec = '1')
                                 else
                             '0';

    ----------------------------------------------------------------------------
    -- Capture register for secondary sampling point bit error:
    --  1. Clear upon next regular sample point.
    --  2. Set when Bit error is detected by secondary sampling point.
    ----------------------------------------------------------------------------
    bit_err_ssp_capt_d <= '0' when (rx_trigger = '1') else
                          '1' when (bit_err_ssp_condition = '1') else
          bit_err_ssp_capt_q;

    bit_error_ssp_capt_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            bit_err_ssp_capt_q <= '0';
        elsif (rising_edge(clk_sys)) then
            bit_err_ssp_capt_q <= bit_err_ssp_capt_d;
        end if;
    end process;
    
    ----------------------------------------------------------------------------
    -- Bit Error for SSP is processed in Sample point (RX Trigger). We must look
    -- at capture register (bit_err_ssp_capt_q) and also SSP condition because
    -- SSP might occur at the same cycle as RX Trigger and in this case bit
    -- error is processed immediately and not captured!
    ----------------------------------------------------------------------------
    bit_err_ssp_valid <= '1' when (sp_control = SECONDARY_SAMPLE and
                                   rx_trigger = '1' and
                                   (bit_err_ssp_capt_q = '1' or
                                    bit_err_ssp_condition = '1'))
                             else
                         '0';

    bit_err_norm_valid <= '1' when (sp_control /= SECONDARY_SAMPLE and
                                    data_rx_synced /= data_tx and
                                    rx_trigger = '1')      
                              else
                          '0';

    ----------------------------------------------------------------------------
    -- Bit Error detection. If expected data is not equal to actual data in
    -- sample point -> Bit Error!
    ----------------------------------------------------------------------------
    bit_err_d <= '0' when (drv_ena = CTU_CAN_DISABLED) else
                 '1' when (bit_err_ssp_valid = '1') else
                 '1' when (bit_err_norm_valid = '1') else
                 '0';
    
    ----------------------------------------------------------------------------
    -- Bit error register
    ----------------------------------------------------------------------------
    bit_err_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            bit_err_q <= '0';
        elsif (rising_edge(clk_sys)) then
            bit_err_q <= bit_err_d;
        end if;
    end process;
    
    -- Propagation to output
    bit_err <= bit_err_q;


    -- <RELEASE_OFF>
    ----------------------------------------------------------------------------
    -- Functional coverge
    ----------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- Note: Following cover points does not yet mean that error will cause
    --       transmission of error frame! It is up to protocol control FSM to
    --       decide this!

    -- psl bit_err_normal_cov : cover
    --  {bit_err_norm_valid = '1'};
    
    -- psl bit_err_secondary_cov : cover
    --  {bit_err_ssp_valid = '1' and bit_err_norm_valid = '0'};

    -- psl bit_err_secondary_capt_cov : cover
    --  {bit_err_ssp_valid = '1' and bit_err_ssp_capt_q = '1' and
    --   bit_err_ssp_condition = '0'};

    -- psl bit_err_secondary_direct_cov : cover
    --  {bit_err_ssp_valid = '1' and bit_err_ssp_capt_q = '0' and
    --   bit_err_ssp_condition = '1'};

    -- <RELEASE_ON>

end architecture;