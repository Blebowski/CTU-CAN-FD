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
--  Trigger multiplexor.
--
-- Purpose:
--  Creates trigger (clock enable) signals for pipeline stages within CAN Core.
--  Creates following trigger signals:
--      1. Protocol control TX Trigger - Stuff pipeline stage. Gated when a bit
--         was stuffed.
--      2. Protocol control RX Trigger - Process pipeline state. Gated when a
--         bit was destuffed.
--      3. Bit stuffing trigger - Stuff pipeline stage.
--      4. Bit destuffing trigger - Destuff pipeline stage
--      5. CRC RX With bit stuffing trigger - Process pipeline stage. Gated
--         when fixed stuff bit is destuffed since CRC 17, 21 shall not be
--         calculated from fixed stuff bits.
--      6. CRC RX No bit stuffing trigger - Process pipeline stage. Gated when
--         a bit is destuffed.
--      7. CRC TX No bit stuffing trigger - Stuff pipeline stage. Gated when
--         a stuff bit was inserted after previous bit.
--      7. CRC TX With bit stuffing trigger - Stuff pipeline stage + 1 clock
--         cycle. Gated when fixed stuff bit was inserted.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer.all;
use ctu_can_fd_rtl.can_constants.all;
use ctu_can_fd_rtl.can_components.all;
use ctu_can_fd_rtl.can_types.all;
use ctu_can_fd_rtl.cmn_lib.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.reduce_lib.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity trigger_mux is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :    std_logic := '0';
        
        -- Number of Sample Triggers
        G_SAMPLE_TRIGGER_COUNT  :    natural := 2
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous reset
        res_n                  :in   std_logic;
        
        ------------------------------------------------------------------------    
        -- Input triggers
        ------------------------------------------------------------------------
        -- RX Triggers
        rx_triggers            :in   std_logic_vector(G_SAMPLE_TRIGGER_COUNT - 1 downto 0);

        -- TX Trigger
        tx_trigger             :in   std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Stuff bit is inserted, Protocol control operation to be halted for
        -- one bit time
        data_halt              :in   std_logic;
        
        -- Data output is not valid, actual bit is stuff bit.
        destuffed              :in   std_logic;
        
        -- Fixed bit stuffing method is used
        fixed_stuff            :in   std_logic;
        
        -- Bit Destuffing Data input
        bds_data_in            :in   std_logic;

        ------------------------------------------------------------------------
        -- Output triggers
        ------------------------------------------------------------------------
        -- Protocol control TX Trigger
        pc_tx_trigger          :out  std_logic;
        
        -- Protocol control RX Trigger
        pc_rx_trigger          :out  std_logic;
        
        -- Bit Stuffing Trigger
        bst_trigger            :out  std_logic;
        
        -- Bit De-Stuffing Trigger
        bds_trigger            :out  std_logic;
        
        -- CRC Trigger RX - No bit stuffing
        crc_trig_rx_nbs        :out  std_logic;
        
        -- CRC Trigger TX - No bit stuffing
        crc_trig_tx_nbs        :out  std_logic;
        
        -- CRC Trigger RX - With bit stuffing
        crc_trig_rx_wbs        :out  std_logic;
        
        -- CRC Trigger TX - With bit stuffing
        crc_trig_tx_wbs        :out  std_logic;
        
        ------------------------------------------------------------------------
        -- Status signals
        ------------------------------------------------------------------------
        -- CRC RX With Bit Stuffing - Data input
        crc_data_rx_wbs        :out  std_logic
    );
end entity;

architecture rtl of trigger_mux is
    
    signal tx_trigger_q       :      std_logic;
    
begin
  
      ---------------------------------------------------------------------------
    -- Protocol control triggers:
    --  1. Protocol control trigger (TX) - shifts TX Shift register, is enabled 
    --     when stuff bit is not inserted! Active in Stuff pipeline stage.
    --  2. Protocol control trigger (RX) - shifts RX Shift register, is enabled
    --     when stuff bit is not destuffed! Active in Process pipeline stage. 
    ---------------------------------------------------------------------------
    pc_tx_trigger <= '1' when (tx_trigger = '1' and data_halt = '0')
                         else
                     '0';

    pc_rx_trigger <= '1' when (rx_triggers(0) = '1' and destuffed = '0')
                         else
                     '0';
                     
    ---------------------------------------------------------------------------
    -- Bit stuffing/destuffing triggers:
    --  1. Bit Stuffing Trigger (TX) - Processes data on Bit stuffing input,
    --     active in Stuff pipeline stage.
    --  2. Bit Destuffing Trigger (RX) - Processes data on Bit Destuffin input,
    --     active in Destuff pipeline stage.
    ---------------------------------------------------------------------------
    bst_trigger <= tx_trigger;
    bds_trigger <= rx_triggers(1);
    
    ---------------------------------------------------------------------------
    -- CRC Triggers for CRC 15 (CRC without stuff bits):
    --  1. CRC RX NBS - Trigger for CRC15 from RX data without bit stuffing.
    --     Trigger must be gated when bit was destuffed, because CRC15 for 
    --     CAN 2.0 frames shall not take stuff bits into account! Active in 
    --     Process pipeline stage.
    --  2. CRC TX NBS - Trigger for CRC15 from TX data without bit stuffing.
    --     Must be gated when stuff bit is inserted! Active in Stuff pipeline
    --     stage.
    ---------------------------------------------------------------------------
    crc_trig_rx_nbs <= '1' when (rx_triggers(0) = '1' and destuffed = '0')
                           else
                       '0';

    crc_trig_tx_nbs <= '1' when (tx_trigger = '1' and data_halt = '0')
                           else
                       '0';

    ---------------------------------------------------------------------------
    -- CRC Trigger for CRC 17, 21 (with bit stuffing):
    --  1. CRC TX WBS - Trigger for CRC17, CRC21 from TX Data with bit stuffing.
    --     This trigger must be gated for fixed stuff bits since CRC17, CRC21
    --     shall not contain fixed stuff bits! Active one clock cycle after
    --     Stuff pipeline stage.
    --  2. CRC RX WBS - Trigger for CRC17, CRC21 from RX Data with bit stuffing.
    --     Fixed stuff bits must be left out! Active in Process pipeline stage.
    --     (see next comment).
    ---------------------------------------------------------------------------
    crc_trig_tx_wbs_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,            -- IN
        clk                => clk_sys,          -- IN
        input              => tx_trigger,       -- IN
        
        output             => tx_trigger_q      -- OUT
    );

    crc_trig_tx_wbs <= '0' when (fixed_stuff = '1' and data_halt = '1') else
                       '1' when (tx_trigger_q = '1') else
                       '0';

    ---------------------------------------------------------------------------
    -- We must gate fixed stuff bit for CRC from RX With Bit Stuffing. But we
    -- don't know if it is stuff bit in Stuff pipeline stage. So we must delay
    -- the information to Process pipeline stage. We sample the data 
    -- (Bit Destuffing input) to avoid possible change, and calculate the CRC
    -- with rx_trigger(0) (in Process pipeline stage).
    ---------------------------------------------------------------------------
    crc_data_rx_wbs_reg : dff_arst_ce
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,            -- IN
        clk                => clk_sys,          -- IN
        input              => bds_data_in,      -- IN
        ce                 => rx_triggers(1),   -- IN
        
        output             => crc_data_rx_wbs   -- OUT
    );
    
    crc_trig_rx_wbs <= '0' when (fixed_stuff = '1' and destuffed = '1') else
                       '1' when (rx_triggers(0) = '1') else
                       '0';

end architecture;