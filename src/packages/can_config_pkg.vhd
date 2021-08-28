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
-- Package:
--  CAN Configuration
-- 
-- Purpose:
--  Package with Configuration of CTU CAN FD. Contains all contstants which
--  drive top level generics.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

package can_config_pkg is

    -- IP Core version
    constant C_CTU_CAN_FD_VERSION_MINOR : std_logic_vector(7 downto 0) := x"04";
    constant C_CTU_CAN_FD_VERSION_MAJOR : std_logic_vector(7 downto 0) := x"02";

    -- Number of TXT Buffers
    constant C_TXT_BUFFER_COUNT     : natural := 4;
    
    -- Number of Interrupts
    constant C_INT_COUNT            : natural := 12;  
  
    -- Number of Sample Triggers
    constant C_SAMPLE_TRIGGER_COUNT : natural range 2 to 8 := 2;
    
    -- Width of control counter
    constant C_CTRL_CTR_WIDTH       : natural := 9;
    
    -- Width of retransmitt counter
    constant C_RETR_LIM_CTR_WIDTH   : natural := 4;
    
    -- Insert pipeline on Error valid signal
    constant C_ERR_VALID_PIPELINE   : boolean := true;
    
    -- TSEG1 Width - Nominal Bit Time
    constant C_TSEG1_NBT_WIDTH      : natural := 8;
        
    -- TSEG2 Width - Nominal Bit Time
    constant C_TSEG2_NBT_WIDTH      : natural := 8;
        
    -- Baud rate prescaler Width - Nominal Bit Time
    constant C_BRP_NBT_WIDTH        : natural := 8;
        
    -- Synchronisation Jump width Width - Nominal Bit Time
    constant C_SJW_NBT_WIDTH        : natural := 5;
        
    -- TSEG1 Width - Data Bit Time
    constant C_TSEG1_DBT_WIDTH      : natural := 8;
        
    -- TSEG2 Width - Data Bit Time
    constant C_TSEG2_DBT_WIDTH      : natural := 8;
        
    -- Baud rate prescaler width - Data Bit Time
    constant C_BRP_DBT_WIDTH        : natural := 8;
        
    -- Synchronisation Jump Width width - Data Bit Time
    constant C_SJW_DBT_WIDTH        : natural := 5;
    
    -- Secondary sampling point Shift registers length
    constant C_SSP_DELAY_SAT_VAL    : natural := 255;

    -- Depth of FIFO Cache for TX Data
    constant C_TX_CACHE_DEPTH       : natural := 8;
        
    -- Width (number of bits) in transceiver delay measurement counter
    constant C_TRV_CTR_WIDTH        : natural := 7;
    
    -- Secondary sample point position width
    constant C_SSP_POS_WIDTH        : natural := 8;

    -- Optional usage of saturated value of ssp_delay 
    constant C_USE_SSP_SATURATION   : boolean := true;

    -- Width of SSP counters
    constant C_SSP_CTRS_WIDTH       : natural := 15;

    -- CRC polynomials
    constant C_CRC15_POL : std_logic_vector(15 downto 0) := x"C599";
    constant C_CRC17_POL : std_logic_vector(19 downto 0) := x"3685B";
    constant C_CRC21_POL : std_logic_vector(23 downto 0) := x"302899";
    
    -- Device ID
    constant C_CAN_DEVICE_ID : std_logic_vector(15 downto 0) := x"CAFD";
  
end package;
