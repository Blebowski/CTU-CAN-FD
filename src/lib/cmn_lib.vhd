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
--  Library with component declarations for common design entities.
--------------------------------------------------------------------------------
-- Revision History:
--    10.12.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

package cmn_lib is

    ----------------------------------------------------------------------------
    -- Reset synchronizer
    ----------------------------------------------------------------------------
    component rst_sync is
        generic (
            constant reset_polarity     :       std_logic
        );    
        port (
            signal clk                  : in    std_logic;
            signal arst                 : in    std_logic;
            signal rst                  : out   std_logic
        );
    end component rst_sync;


    ----------------------------------------------------------------------------
    -- Signal synchroniser - synchronisation chain
    ----------------------------------------------------------------------------
    component sig_sync is
        generic (
            constant timing_check       :       boolean := true
        );
        port (
            signal clk                  : in    std_logic;
            signal async                : in    std_logic;
            signal sync                 : out   std_logic
        );
    end component sig_sync;


    ----------------------------------------------------------------------------
    -- Shift register with preload
    ----------------------------------------------------------------------------
    component shift_reg_preload is
        generic (
            constant reset_polarity     :       std_logic;
            constant reset_value        :       std_logic_vector;
            constant width              :       natural;
            constant shift_down         :       boolean
        );
        port (
            signal clk                  : in    std_logic;
            signal res_n                : in    std_logic;
            signal input                : in    std_logic;
            signal preload              : in    std_logic;
            signal preload_val          : in    std_logic_vector(width - 1 downto 0);
            signal enable               : in    std_logic;
            signal reg_stat             : out   std_logic_vector(width - 1 downto 0);
            signal output               : out   std_logic
        );
    end component shift_reg_preload;


    ----------------------------------------------------------------------------
    -- Shift register
    ----------------------------------------------------------------------------
    component shift_reg is
        generic (
            constant reset_polarity     :       std_logic;
            constant reset_value        :       std_logic_vector;
            constant width              :       natural;
            constant shift_down         :       boolean
        );
        port (
            signal clk                  : in    std_logic;
            signal res_n                : in    std_logic;
            signal input                : in    std_logic;
            signal enable               : in    std_logic;
            signal reg_stat             : out   std_logic_vector(width - 1 downto 0);
            signal output               : out   std_logic
        );
    end component shift_reg;


    ----------------------------------------------------------------------------
    -- Majority out of 3 decoder.
    ----------------------------------------------------------------------------
    component majority_decoder_3 is
        port (
            signal input                : in    std_logic_vector(2 downto 0);
            signal output               : out   std_logic
        );
    end component majority_decoder_3;


    ----------------------------------------------------------------------------
    -- Simple DFF with configurable width with asynchronous reset.
    ----------------------------------------------------------------------------
    component dff_arst is
    generic (
        constant reset_polarity     :       std_logic;
        constant rst_val            :       std_logic
    );    
    port (
        signal arst                 : in    std_logic;
        signal clk                  : in    std_logic;

        signal input                : in    std_logic;
        signal load                 : in    std_logic;
        signal output               : out   std_logic
    );
    end component dff_arst;



end package cmn_lib;

