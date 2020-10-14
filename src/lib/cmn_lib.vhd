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
-- Package:
--  Common Library.
-- 
-- Purpose:
--  Package with component declarations for common design entities.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

package cmn_lib is

    component dff_arst_ce is
    generic (
        -- Reset polarity
        G_RESET_POLARITY   :       std_logic;
        
        -- Reset value
        G_RST_VAL          :       std_logic
    );    
    port (
        -- Asynchronous reset
        arst               : in    std_logic;
        
        -- Clock
        clk                : in    std_logic;

        -- Data input (D)
        input              : in    std_logic;
        
        -- Clock enable (CE)
        ce                 : in    std_logic;
        
        -- Data output (Q)
        output             : out   std_logic
    );
    end component dff_arst_ce;

    component dff_arst is
    generic (
        -- Reset polarity
        G_RESET_POLARITY   :       std_logic;
        
        -- Reset value
        G_RST_VAL          :       std_logic
    );    
    port (
        -- Asynchronous reset
        arst               : in    std_logic;
        
        -- Clock
        clk                : in    std_logic;

        -- Data input (D)
        input              : in    std_logic;
        
        -- Data output (Q)
        output             : out   std_logic
    );
    end component dff_arst;


    component dlc_decoder is
    port (
        -- DLC Input (as in CAN Standard)
        dlc              :   in std_logic_vector(3 downto 0);
        
        -- Frame Type (0 - CAN 2.0, 1 - CAN FD)
        frame_type       :   in std_logic;

        -- Data length (decoded)
        data_length      :   out std_logic_vector(6 downto 0);
        
        -- Validity indication (0 for CAN 2.0 frames with dlc > 0) 
        is_valid         :   out std_logic
    );
    end component dlc_decoder;

    component endian_swapper is 
    generic (
        
        -- When true, output word is endian swapped as long as "swap_by_signal"
        -- is true. Otherwise it has no meaning.
        G_SWAP_GEN              :     boolean := false;

        -- Size of word (in groups)
        G_WORD_SIZE             :     natural := 4;
        
        -- Size of group (in bits)
        G_GROUP_SIZE            :     natural := 8  
    );  
    port (
        -- Data input
        input   : in  std_logic_vector(G_WORD_SIZE * G_GROUP_SIZE - 1 downto 0);
        
        -- Data output
        output  : out std_logic_vector(G_WORD_SIZE * G_GROUP_SIZE - 1 downto 0)
    );
    end component;

    component g_inf_ram_wrapper is
    generic(
        -- Reset polarity
        G_RESET_POLARITY       :     std_logic := '1';
        
        -- Width of memory word (in bits)
        G_WORD_WIDTH           :     natural := 32;

        -- Memory depth (in words)
        G_DEPTH                :     natural := 32;

        -- Address width (in bits)
        G_ADDRESS_WIDTH        :     natural := 8;

        -- Synchronous read
        G_SYNC_READ            :     boolean := true
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Reset
        ------------------------------------------------------------------------
        clk_sys     :in   std_logic;
        res_n       :in   std_logic;

        ------------------------------------------------------------------------
        -- Port A - Data input
        ------------------------------------------------------------------------
        -- Address
        addr_A      :in   std_logic_vector(G_ADDRESS_WIDTH - 1 downto 0);
        
        -- Write signal
        write       :in   std_logic;
        
        -- Data input
        data_in     :in   std_logic_vector(G_WORD_WIDTH - 1 downto 0);

        ------------------------------------------------------------------------   
        -- Port B - Data output
        ------------------------------------------------------------------------
        -- Address
        addr_B      :in   std_logic_vector(G_ADDRESS_WIDTH - 1 downto 0);
        
        -- Data output
        data_out    :out  std_logic_vector(G_WORD_WIDTH - 1 downto 0)
    );
    end component;

    component majority_decoder_3 is
    port (
        -- Input
        input      : in    std_logic_vector(2 downto 0);
        
        -- Output
        output     : out   std_logic
    );
    end component majority_decoder_3;

    component rst_sync is
    generic (
        -- Reset polarity
        G_RESET_POLARITY    :       std_logic
    );    
    port (
        -- Clock
        clk                 : in    std_logic;
        
        -- Asynchronous reset
        arst                : in    std_logic;
        
        -- Synchronous reset
        rst                 : out   std_logic
    );
    end component rst_sync;

    component shift_reg_byte is
    generic (
        -- Reset polarity
        G_RESET_POLARITY     :       std_logic;
        
        -- Reset value
        G_RESET_VALUE        :       std_logic_vector;
        
        -- Shift register width
        G_NUM_BYTES          :       natural
    );
    port (
        -----------------------------------------------------------------------
        -- Clock and Asyncrhonous reset
        -----------------------------------------------------------------------
        -- Clock
        clk             : in    std_logic;

        -- Asynchronous reset
        res_n           : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Shift register input        
        input           : in    std_logic;

        -- Clock enable for shifting each byte of the shift register.
        byte_clock_ena  : in    std_logic_vector(G_NUM_BYTES - 1 downto 0);

        -- Input source selector for each byte
        -- (0-Previous byte output, 1- Shift reg input)
        byte_input_sel  : in    std_logic_vector(G_NUM_BYTES - 1 downto 0);

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Shift register status
        reg_stat        : out   std_logic_vector(8 * G_NUM_BYTES - 1 downto 0)
    );
    end component shift_reg_byte;

    component shift_reg_preload is
    generic (
        -- Reset polarity
        G_RESET_POLARITY     :       std_logic;
        
        -- Reset value
        G_RESET_VALUE        :       std_logic_vector;
        
        -- Shift register width
        G_WIDTH              :       natural;

        -- True - Shift from Highest index, False - Shift from lowest Index
        G_SHIFT_DOWN         :       boolean
    );
    port (
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        clk                  : in    std_logic;
        res_n                : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Pre-load shift register
        preload              : in    std_logic;

        -- Value to be pre-load to the shift register
        preload_val          : in    std_logic_vector(G_WIDTH - 1 downto 0);

        -- Enable for shift register. When enabled, shifted each clock, when
        -- disabled, register keeps its state.
        enable               : in    std_logic;

        -- Input to a shift register 
        input                : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Shift register value
        reg_stat             : out   std_logic_vector(G_WIDTH - 1 downto 0);

        -- Shift register output
        output               : out   std_logic
    );
    end component shift_reg_preload;

    component shift_reg is
    generic (
        -- Reset polarity
        G_RESET_POLARITY     :       std_logic;

        -- Reset value
        G_RESET_VALUE        :       std_logic_vector;

        -- Shift register width
        G_WIDTH              :       natural;

        -- True - Shift from Highest index, False - Shift from lowest Index
        G_SHIFT_DOWN         :       boolean
    );
    port (
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        clk                  : in    std_logic;
        res_n                : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Shift register input        
        input                : in    std_logic;

        -- Enable for shift register. When enabled, shifted each clock, when
        -- disabled, register keeps its state.
        enable               : in    std_logic;

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Shift register value
        reg_stat             : out   std_logic_vector(G_WIDTH - 1 downto 0);

        -- Register output
        output               : out   std_logic
    );
    end component shift_reg;

    component sig_sync is
    generic(
        -- Reset polarity
        G_RESET_POLARITY     : std_logic := '0';
        
        -- Reset value
        G_RESET_VALUE        : std_logic := '1'
    );
    port (
        -- Reset
        res_n                : in    std_logic;
        
        -- Clock
        clk                  : in    std_logic;
        
        -- Asychronous signal
        async                : in    std_logic;
        
        -- Synchronous signal
        sync                 : out   std_logic
    );
    end component sig_sync;

end package cmn_lib;
