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
--  TX Shift register
--------------------------------------------------------------------------------
-- Revision History:
--
--    27.3.2019   Created file
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

entity tx_shift_reg is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;

        -- Asynchronous reset
        res_n                   :in   std_logic;

        -----------------------------------------------------------------------
        -- Trigger signals
        -----------------------------------------------------------------------
        -- RX Trigger
        tx_trigger              :in   std_logic;

        -----------------------------------------------------------------------
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual TX Data
        tx_data                 :out   std_logic;

        -----------------------------------------------------------------------
        -- Protocol control FSM interface
        -----------------------------------------------------------------------
        -- Load Base Identifier to TX Shift register
        tx_load_base_id         :in  std_logic;

        -- Load extended Identifier to TX Shift register
        tx_load_ext_id          :in  std_logic;

        -- Load DLC to TX Shift register
        tx_load_dlc             :in  std_logic;

        -- Load Data word to TX Shift register
        tx_load_data_word       :in  std_logic;

        -- Load CRC to TX Shift register
        tx_load_crc             :in  std_logic;
        
        -- Shift register enable (shifts with TX Trigger)
        tx_shift_ena            :in  std_logic;

        -- Force Dominant value instead of value from shift register
        tx_dominant             :in  std_logic;

        -- CRC Source (CRC15, CRC17, CRC21)
        crc_src                 :in  std_logic_vector(1 downto 0);

        -----------------------------------------------------------------------
        -- CAN CRC Interface
        -----------------------------------------------------------------------
        -- Calculated CRC 15
        crc_15                  :in   std_logic_vector(14 downto 0);

        -- Calculated CRC 17
        crc_17                  :in   std_logic_vector(16 downto 0);
        
        -- Calculated CRC 21
        crc_21                  :in   std_logic_vector(20 downto 0);

        -----------------------------------------------------------------------
        -- Error detector Interface
        -----------------------------------------------------------------------
        -- Error frame request
        err_frm_req             :in  std_logic;
        
        -----------------------------------------------------------------------
        -- Fault confinement Interface
        -----------------------------------------------------------------------
        -- Unit is error active
        is_err_active           :in  std_logic;

        -----------------------------------------------------------------------
        -- TXT Buffers interface
        -----------------------------------------------------------------------
        -- TXT Buffer RAM word
        txt_buffer_word         :in   std_logic_vector(31 downto 0);

        -- TX Data length code
        tran_dlc                :in   std_logic_vector(3 downto 0)
    );
end entity;

architecture rtl of tx_shift_reg is
   
    -- Shift register output 
    signal tx_sr_output : std_logic;

    -- Shift register clock enable
    signal tx_sr_ce     : std_logic;
    
    -- Shift register preload
    signal tx_sr_pload      : std_logic;
    signal tx_sr_pload_val  : std_logic;

    -- ID Loaded from TXT Buffer RAM
    signal tx_base_id : std_logic_vector(10 downto 0);
    signal tx_ext_id : std_logic_vector(17 downto 0);

    -- Selected CRC to be transmitted
    signal tx_crc : std_logic_vector(20 downto 0);

begin
    
    -- Tick shift register in Sync (TX Trigger)!
    tx_sr_ce <= '1' when (tx_shift_ena = '1' and tx_trigger = '1')
                    else
                '0';
                
    -- Shift register pre-load
    tx_sr_pload <= '1' when (tx_load_base_id = '1' or                  
                             tx_load_ext_id = '1' or
                             tx_load_dlc = '1' or
                             tx_load_data_word = '1' or
                             tx_load_crc = '1')
                       else
                   '0';

    -- CRC to be transmitted
    tx_crc <= crc_15 & "000000" when (crc_src = CRC15) else
                crc_17 & "0000" when (crc_src = CRC17) else
                        crc_21;
    

    -- Choosing Base and Ext IDs from TXT Buffer RAM memory words!
    tx_base_id <= txt_buffer_word(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L);
    tx_ext_id <= txt_buffer_word(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L);

    ---------------------------------------------------------------------------
    -- Shift register pre-load value:
    --  1. Base ID is loaded from TXT Buffer memory.
    --  2. Extended ID is loaded from TXT Buffer memory;
    --  3. DLC is loaded from Output of TX Arbitrator.
    --  4. TXT Buffer word is loaded from TXT Buffer memory.
    --  5. Calculated CRC is loaded from output of CAN CRC.
    ---------------------------------------------------------------------------
    tx_sr_pload_val <=
         tx_base_id & "0000000000000000000000" when (tx_load_base_id = '1') else
                  tx_ext_id & "00000000000000" when (tx_load_ext_id = '1') else
     tran_dlc & "0000000000000000000000000000" when (tx_load_dlc = '1') else
                               txt_buffer_word when (tx_load_data_word = '1') else
                      tx_crc & "0000000000000" when (tx_load_crc = '1') else
                               (OTHERS => '0');

    ---------------------------------------------------------------------------
    -- TX Shift register instance
    ---------------------------------------------------------------------------
    tx_shift_reg_inst : shift_reg_preload
    generic map(
        G_RESET_POLARITY     => G_RESET_POLARITY,
        G_RESET_VALUE        => x"000000000",
        G_WIDTH              => 32,
        G_SHIFT_DOWN         => false
    )
    port map(
        clk                  => clk_sys,
        res_n                => res_n,
        preload              => tx_sr_pload,
        preload_val          => tx_sr_pload_val,
        enable               => tx_sr_ce,
        input                => '0',
        
        reg_stat             => open,
        output               => tx_sr_output
    );

    ---------------------------------------------------------------------------
    -- Calculation of next data bit value!
    ---------------------------------------------------------------------------
    tx_data <= DOMINANT when (err_frm_req = '1' and is_err_active = '1') else
               RECESSIVE when (err_frm_req = '1') else
               DOMINANT when (tx_dominant = '1') else
               tx_sr_output when (tx_shift_ena = '1') else
               RECESSIVE;

end architecture;