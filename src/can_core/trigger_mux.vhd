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
--  Trigger multiplexor        
--------------------------------------------------------------------------------
-- Revision History:
--    26.5.2019  Created file
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
    --  1. TX Trigger which shifts TX Shift register is enabled when
    --     stuff bit is not inserted!
    --  2. RX Trigger which shifts RX Shift register is enabled when
    --     stuff bit is not destuffed!
    ---------------------------------------------------------------------------
    pc_tx_trigger <= '1' when (tx_trigger = '1' and data_halt = '0')
                         else
                     '0';

    pc_rx_trigger <= '1' when (rx_triggers(0) = '1' and destuffed = '0')
                         else
                     '0';
                     
    ---------------------------------------------------------------------------
    -- Bit stuffing/destuffing triggers:
    --  1. Bit Stuffing - TX Trigger, stuff bit does not make any change here
    --     since also stuff bit must be processed by Bit Stuffing.
    --  2. Bit Destuffing - RX Trigger, one clock cycle in advance of TX
    --     Trigger for protocol control, since Bit stuffing is pipelined!
    --     Destuffed bits shall not block bit destuffing since these must also
    --     be processed by Bit destuffing.
    ---------------------------------------------------------------------------
    bst_trigger <= tx_trigger;    
    bds_trigger <= rx_triggers(1);
    
    ---------------------------------------------------------------------------
    -- CRC Triggers for CRC 15 (no bit stuffing):
    --  1. CRC RX NBS - Trigger for CRC15 from RX data without bit stuffing.
    --     The same trigger as for Protocol control reception in sample point.
    --     Trigger must be gated when bit was destuffed, because CRC15 for 
    --     CAN 2.0 frames shall not take stuff bits into account!
    --  2. CRC TX NBS - Trigger for CRC15 from TX data without bit stuffing.
    --     The same trigger as TX Trigger (inserts stuff bit). Must be gated
    --     when stuff bit is inserted!
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
    --     Trigger one clock cycle delayed from TX Trigger. Note that this
    --     trigger may be delayed since resynchronisation will never shorten
    --     phase 1 (between TX and RX triggers). This trigger must be gated
    --     for fixed stuff bits!!
    --  2. CRC RX WBS Trigger is the same trigger as the one used to process
    --     data by bit destuffing (one clock cycle in advance of Protocol 
    --     control sampling)! Fixed stuff bits must be left out!
    ---------------------------------------------------------------------------
    crc_trig_tx_wbs_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => tx_trigger,
        ce                 => '1',
        output             => tx_trigger_q
    );

    crc_trig_tx_wbs <= '0' when (fixed_stuff = '1' and data_halt = '1') else
                       '1' when (tx_trigger_q = '1') else
                       '0';

    ---------------------------------------------------------------------------
    -- We must gate fixed stuff bit for CRC from RX With Bit Stuffing. But we
    -- don't know if it is stuff bit, because this should be calculated at the
    -- same clock cycle as bit destuffing! So we must belay the information
    -- here! We sample the data (Bit Destuffing input) to avoid possible change,
    -- and calculate the CRC with rx_trigger(0) (the same trigger as sample
    -- point).
    ---------------------------------------------------------------------------
    crc_data_rx_wbs_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => bds_data_in,
        ce                 => rx_triggers(1),
        output             => crc_data_rx_wbs
    );
    
    crc_trig_rx_wbs <= '0' when (fixed_stuff = '1' and destuffed = '1') else
                       '1' when (rx_triggers(0) = '1') else
                       '0';

end architecture;