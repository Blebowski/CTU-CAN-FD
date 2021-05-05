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
--  TX Shift register
--
-- Purpose:
--  Creates TX Serial data stream in multi-bit fields on CAN bus. Controlled
--  by Protocol control FSM. Preloaded in "Process" pipeline stage. Shifted in
--  "Stuff" pipeline stage. Single bit fields are transmitted via forcing the
--  TX output by Protocol control FSM to Dominant value.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

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
        -- Actual TX Data (no bit stuffing)
        tx_data_nbs             :out   std_logic;

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

        -- Load Stuff count
        tx_load_stuff_count     :in  std_logic;
        
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
        -- Bit Stuffing / Destuffing Interface
        -----------------------------------------------------------------------
        -- Stuff counter modulo 8
        bst_ctr                 :in   std_logic_vector(2 downto 0);

        -----------------------------------------------------------------------
        -- TXT Buffers interface
        -----------------------------------------------------------------------
        -- TX Identifier
        tran_identifier         :in   std_logic_vector(28 downto 0);
        
        -- TXT Buffer RAM word (byte endianity swapped)
        tran_word_swapped       :in   std_logic_vector(31 downto 0);

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
    signal tx_sr_pload_val  : std_logic_vector(31 downto 0);

    -- ID Loaded from TXT Buffer RAM
    signal tx_base_id : std_logic_vector(10 downto 0);
    signal tx_ext_id : std_logic_vector(17 downto 0);

    -- Selected CRC to be transmitted
    signal tx_crc : std_logic_vector(20 downto 0);

    -- Stuff counter (grey coded)
    signal bst_ctr_grey : std_logic_vector(2 downto 0);
    signal bst_parity   : std_logic;
    signal stuff_count  : std_logic_vector(3 downto 0);
    
    constant C_RX_SHIFT_REG_RST_VAL : std_logic_vector(31 downto 0) :=
        x"00000000";
    
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
                             tx_load_stuff_count = '1' or
                             tx_load_crc = '1')
                       else
                   '0';

    -- CRC to be transmitted
    tx_crc <= crc_15 & "000000" when (crc_src = C_CRC15_SRC) else
                crc_17 & "0000" when (crc_src = C_CRC17_SRC) else
                        crc_21;
                        
    -- Stuff counter grey coding
    with bst_ctr select bst_ctr_grey <=
        "000" when "000",
        "001" when "001",
        "011" when "010",
        "010" when "011",
        "110" when "100",
        "111" when "101",
        "101" when "110",
        "100" when "111",
        "000" when others;
    
    bst_parity <= bst_ctr_grey(0) xor bst_ctr_grey(1) xor bst_ctr_grey(2);
    
    stuff_count <= bst_ctr_grey & bst_parity;

    -- Choosing Base and Ext IDs from TXT Buffer RAM memory words!
    tx_base_id <= tran_identifier(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L);
    tx_ext_id <= tran_identifier(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L);

    ---------------------------------------------------------------------------
    -- Shift register pre-load value:
    --  1. Base ID is loaded from TXT Buffer memory.
    --  2. Extended ID is loaded from TXT Buffer memory;
    --  3. DLC is loaded from Output of TX Arbitrator.
    --  4. TXT Buffer word is loaded from TXT Buffer memory.
    --  5. Calculated CRC is loaded from output of CAN CRC.
    ---------------------------------------------------------------------------
    tx_sr_pload_val <=
          tx_base_id & "000000000000000000000" when (tx_load_base_id = '1') else
                  tx_ext_id & "00000000000000" when (tx_load_ext_id = '1') else
     tran_dlc & "0000000000000000000000000000" when (tx_load_dlc = '1') else
                             tran_word_swapped when (tx_load_data_word = '1') else
  stuff_count & "0000000000000000000000000000" when (tx_load_stuff_count = '1') else         
                        tx_crc & "00000000000" when (tx_load_crc = '1') else
                               (OTHERS => '0');

    ---------------------------------------------------------------------------
    -- TX Shift register instance
    ---------------------------------------------------------------------------
    tx_shift_reg_inst : shift_reg_preload
    generic map(
        G_RESET_POLARITY     => '0',
        G_RESET_VALUE        => C_RX_SHIFT_REG_RST_VAL,
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
    tx_data_nbs <= DOMINANT when (err_frm_req = '1' and is_err_active = '1') else
                   RECESSIVE when (err_frm_req = '1') else
                   DOMINANT when (tx_dominant = '1') else
                   tx_sr_output when (tx_shift_ena = '1') else
                   RECESSIVE;

end architecture;
