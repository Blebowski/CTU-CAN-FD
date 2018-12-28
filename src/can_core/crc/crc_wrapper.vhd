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
--  CRC Wrapper for CTU CAN FD. Contains 4 CRC circuits. Following CRCs are
--  calculated:
--      1. From TX data before bit-stuffing (tx_nbs)
--      2. From TX data after bit-stuffing (tx_wbs)
--      3. From RX data before bit-destuffing (rx_wbs)
--      4. From RX data after bit-destuffing (rx_nbs)
--
--  CRCs are multiplexed combinationally to output of wrapper via signals:
--   use_rx_crc, use_wbs_crc.  
--  4 combinations on these signals choose the final CRC.
--------------------------------------------------------------------------------
-- Revision History:
--    28.12.2018    Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.can_constants.all;
use work.CAN_FD_register_map.all;
use work.can_components.all;


entity crc_wrapper is
    generic(
        constant crc15_pol :     std_logic_vector(15 downto 0) := x"C599";
        constant crc17_pol :     std_logic_vector(19 downto 0) := x"3685B";
        constant crc21_pol :     std_logic_vector(23 downto 0) := x"302899"  
    );
    port(
        ------------------------------------------------------------------------
        -- Reset and clock
        ------------------------------------------------------------------------
        signal res_n            :in   std_logic;
        signal clk_sys          :in   std_logic;

        ------------------------------------------------------------------------
        -- Serial data inputs
        ------------------------------------------------------------------------
        signal data_tx_nbs      :in   std_logic;
        signal data_tx_wbs      :in   std_logic;
        signal data_rx_wbs      :in   std_logic;
        signal data_rx_nbs      :in   std_logic;

        ------------------------------------------------------------------------
        -- Trigger signals to process the data on each CRC input.
        ------------------------------------------------------------------------
        signal trig_tx_nbs      :in   std_logic;
        signal trig_tx_wbs      :in   std_logic;
        signal trig_rx_wbs      :in   std_logic;
        signal trig_rx_nbs      :in   std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------

        -- Enable for all CRC circuits.
        signal enable           :in   std_logic;

        -- Driving bus
        signal drv_bus          :in   std_logic_vector(1023 downto 0);

        -- When '1' CRCs from RX path will be chosen, otherwise TX
        signal use_rx_crc       :in   std_logic;

        -- When '1' CRCs with bit stuffing will be chosen, otherwise no bit 
        -- stuffing!
        signal use_wbs_crc      :in   std_logic;

        ------------------------------------------------------------------------
        -- Outputs
        ------------------------------------------------------------------------
        signal crc15            :out  std_logic_vector(14 downto 0);
        signal crc17            :out  std_logic_vector(16 downto 0);
        signal crc21            :out  std_logic_vector(20 downto 0)
    );
end entity;


architecture rtl of crc_wrapper is

    ---------------------------------------------------------------------------
    -- Immediate outputs of CRC circuits
    ---------------------------------------------------------------------------

    -- CRC calculated with bit Stuffing from RX Data
    signal crc15_wbs_rx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_wbs_rx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_wbs_rx            :     std_logic_vector(20 downto 0); --CRC 21

    -- CRC calculated without bit Stuffing from RX Data
    signal crc15_nbs_rx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_nbs_rx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_nbs_rx            :     std_logic_vector(20 downto 0); --CRC 21

    -- CRC calculated with bit Stuffing from TX Data
    signal crc15_wbs_tx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_wbs_tx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_wbs_tx            :     std_logic_vector(20 downto 0); --CRC 21

    -- CRC calculated without bit Stuffing from TX Data
    signal crc15_nbs_tx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_nbs_tx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_nbs_tx            :     std_logic_vector(20 downto 0); --CRC 21


    ---------------------------------------------------------------------------
    -- Intermediate results of first level muxes (after decision between
    -- WBS and NBS)
    ---------------------------------------------------------------------------
    signal crc15_tx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_tx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_tx            :     std_logic_vector(20 downto 0); --CRC 21
    
    signal crc15_rx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_rx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_rx            :     std_logic_vector(20 downto 0); --CRC 21

begin


    ----------------------------------------------------------------------------
    -- CRC with bit stuffing from RX Data
    ----------------------------------------------------------------------------
    crc_wbs_rx_comp : can_crc 
        generic map(
            crc15_pol              =>  crc15_pol,
            crc17_pol              =>  crc17_pol,
            crc21_pol              =>  crc21_pol
        )
        port map(
            data_in                =>  data_rx_wbs,
            clk_sys                =>  clk_sys,
            trig                   =>  trig_rx_wbs,
            res_n                  =>  res_n,
            enable                 =>  enable,
            drv_bus                =>  drv_bus,
            crc15                  =>  crc15_wbs_rx,
            crc17                  =>  crc17_wbs_rx,
            crc21                  =>  crc21_wbs_rx
        ); 


    ----------------------------------------------------------------------------
    -- CRC no bit stuffing from RX Data
    ----------------------------------------------------------------------------
    crc_nbs_rx_comp : can_crc 
        generic map(
            crc15_pol              =>  crc15_pol,
            crc17_pol              =>  crc17_pol,
            crc21_pol              =>  crc21_pol
        )
        port map(
            data_in                =>  data_rx_nbs,
            clk_sys                =>  clk_sys,
            trig                   =>  trig_rx_nbs,
            res_n                  =>  res_n,
            enable                 =>  enable,
            drv_bus                =>  drv_bus,
            crc15                  =>  crc15_nbs_rx,
            crc17                  =>  crc17_nbs_rx,
            crc21                  =>  crc21_nbs_rx
        );


    ----------------------------------------------------------------------------
    -- CRC with bit stuffing from TX Data
    ----------------------------------------------------------------------------
    crc_wbs_tx_comp : can_crc 
        generic map(
            crc15_pol              =>  crc15_pol,
            crc17_pol              =>  crc17_pol,
            crc21_pol              =>  crc21_pol
        )
        port map(
            data_in                =>  data_tx_wbs,
            clk_sys                =>  clk_sys,
            trig                   =>  trig_tx_wbs,
            res_n                  =>  res_n,
            enable                 =>  enable,
            drv_bus                =>  drv_bus,
            crc15                  =>  crc15_wbs_tx,
            crc17                  =>  crc17_wbs_tx,
            crc21                  =>  crc21_wbs_tx
        );


    ----------------------------------------------------------------------------
    -- CRC no bit stuffing from TX Data
    ----------------------------------------------------------------------------
    crc_nbs_tx_comp : can_crc 
        generic map(
            crc15_pol              =>  crc15_pol,
            crc17_pol              =>  crc17_pol,
            crc21_pol              =>  crc21_pol
        )
        port map(
            data_in                =>  data_tx_nbs,
            clk_sys                =>  clk_sys,
            trig                   =>  trig_tx_nbs,
            res_n                  =>  res_n,
            enable                 =>  enable,
            drv_bus                =>  drv_bus,
            crc15                  =>  crc15_nbs_tx,
            crc17                  =>  crc17_nbs_tx,
            crc21                  =>  crc21_nbs_tx
        ); 


    ----------------------------------------------------------------------------
    -- First stage muxes. Selecting between WBS and NBS CRCs on both TX and
    -- RX datapaths.
    ----------------------------------------------------------------------------
    -- RX
    crc15_rx <= crc15_wbs_rx when (use_wbs_crc = '1') else
                crc15_nbs_rx;

    crc17_rx <= crc17_wbs_rx when (use_wbs_crc = '1') else
                crc17_nbs_rx;

    crc21_rx <= crc21_wbs_rx when (use_wbs_crc = '1') else
                crc21_nbs_rx;

    -- TX
    crc15_tx <= crc15_wbs_tx when (use_wbs_crc = '1') else
                crc15_nbs_tx;

    crc17_tx <= crc17_wbs_tx when (use_wbs_crc = '1') else
                crc17_nbs_tx;

    crc21_tx <= crc21_wbs_tx when (use_wbs_crc = '1') else
                crc21_nbs_tx;


    ----------------------------------------------------------------------------
    -- Second stage muxes. Selecting between RX and TX CRCs.
    ----------------------------------------------------------------------------
    crc15 <= crc15_rx when (use_rx_crc = '1') else
             crc15_tx;

    crc17 <= crc17_rx when (use_rx_crc = '1') else
             crc17_tx;

    crc21 <= crc21_rx when (use_rx_crc = '1') else
             crc21_tx;

end architecture;
