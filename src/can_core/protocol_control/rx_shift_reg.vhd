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
--  RX Shift register
--------------------------------------------------------------------------------
-- Revision History:
--
--    28.3.2019   Created file
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

entity rx_shift_reg is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :     std_logic := '0'
    );
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
        rx_trigger              :in   std_logic;

        -----------------------------------------------------------------------
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual TX Data
        tx_data                 :out  std_logic;

        -- Actual RX Data
        rx_data                 :in   std_logic;

        -----------------------------------------------------------------------
        -- Protocol control FSM interface
        -----------------------------------------------------------------------
        -- Clear all registers in Shift register (Glitch free)
        rx_clear                :in  std_logic;

        -- Clock Enable RX Shift register for each byte.
        rx_shift_ena            :in  std_logic(3 downto 0);

        -- Selector for inputs of each byte of shift register
        -- (0-Previous byte output, 1- RX Data input)
        rx_shift_in_sel         :in  std_logic;

        -- Store Base Identifier 
        rx_store_base_id        :in  std_logic;

        -- Store Extended Identifier
        rx_store_ext_id         :in  std_logic;

        -- Store Identifier extension
        rx_store_ide            :in  std_logic;
        
        -- Store Remote transmission request
        rx_store_rtr            :in  std_logic;
        
        -- Store EDL (FDF) bit
        rx_store_edl            :in  std_logic;
        
        -- Store DLC
        rx_store_dlc            :in  std_logic;
        
        -- Store ESI
        rx_store_esi            :in  std_logic;
        
        -- Store BRS
        rx_store_brs            :in  std_logic;

        -- Store stuff count
        rx_store_stuff_count    :in  std_logic;
        
        -----------------------------------------------------------------------
        -- RX Buffer interface
        -----------------------------------------------------------------------
        -- RX CAN Identifier
        rec_ident               :out std_logic_vector(28 downto 0);
        
        -- RX Data length code
        rec_dlc                 :out std_logic_vector(3 downto 0);
        
        -- RX Remote transmission request flag
        rec_is_rtr              :out std_logic;
        
        -- RX Recieved identifier type (0-BASE Format, 1-Extended Format);
        rec_ident_type          :out std_logic;
        
        -- RX frame type (0-CAN 2.0, 1- CAN FD)
        rec_frame_type          :out std_logic;
        
        -- RX Bit rate shift Flag
        rec_brs                 :out std_logic;
        
        -- RX Error state indicator
        rec_esi                 :out std_logic;
        
        -- Data words to be stored to RX Buffer. Valid only when rx_trigger='1'
        -- in last bit of data word stored
        store_data_word         :out std_logic_vector(31 downto 0);
        
        -----------------------------------------------------------------------
        -- CRC information for CRC comparison
        -----------------------------------------------------------------------
        -- Received CRC
        rx_crc                  :out std_logic_vector(20 downto 0);
        
        -- Received Stuff count + Stuff Parity
        rx_stuff_count          :out std_logic_vector(3 downto 0)
    );
end entity;

architecture rtl of rx_shift_reg is

    -- Internal reset
    signal res_n_internal : std_logic;

    -- Shift register status
    signal rx_shift_reg   : std_logic_vector(31 downto 0);

    -- RX Shift register shift
    signal rx_shift_cmd   : std_logic; 

begin

     -- Internal reset: Async reset + reset by design!
    res_n_internal <= G_RESET_POLARITY when (res_n = G_RESET_POLARITY or 
                                             rx_clear = '1')
                                       else
                      not (G_RESET_POLARITY);

    ---------------------------------------------------------------------------
    -- Shift the register when it is enabled and RX Trigger is active!
    -- Protocol control keeps the register disabled when e.g Bus is idle
    -- to save power!
    ---------------------------------------------------------------------------
    rx_shift_cmd <= '1' when (rx_trigger = '1' and rx_shift_ena = '1') else
                    '0';

    ---------------------------------------------------------------------------
    -- RX Shift register
    ---------------------------------------------------------------------------
    shift_reg_byte_inst : shift_reg_byte
    generic map(
        G_RESET_POLARITY     => G_RESET_POLARITY,
        G_RESET_VALUE        => x"00000000",
        G_NUM_BYTES          => 4
    )
    port map(
        clk                  => clk_sys,
        res_n                => res_n_internal,
        input                => rx_data,
        byte_clock_ena       => rx_shift_ena,
        byte_input_sel       => rx_shift_in_sel,
        reg_stat             => rx_shift_reg
    );

    ---------------------------------------------------------------------------
    -- Store Identifier
    ---------------------------------------------------------------------------
    id_store_proc : process(clk_sys, res_n_internal)
    begin
        if (res_n_internal = G_RESET_POLARITY) then
            rec_ident <= (OTHERS => '0');    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_base_id = '1' and rx_trigger = '1') then
                rec_ident(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) <=
                    rx_shift_reg(9 downto 0) & rx_data;
            end if;

            if (rx_store_ext_id = '1' and rx_trigger = '1') then
                rec_ident(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) <= 
                    rx_shift_reg(16 downto 0) & rx_data;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Store IDE bit (Identifier type)
    ---------------------------------------------------------------------------
    ide_store_proc : process(clk_sys, res_n_internal)
    begin
        if (res_n_internal = G_RESET_POLARITY) then
            rec_ident_type <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_ide = '1' and rx_trigger = '1') then
                rec_ident_type <= rx_data;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- RX Store RTR bit (Remote transmission request bit)
    ---------------------------------------------------------------------------
    rx_store_proc : process(clk_sys, res_n_internal)
    begin
        if (res_n_internal = G_RESET_POLARITY) then
            rec_is_rtr <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_rtr = '1' and rx_trigger = '1') then
                rec_is_rtr <= rx_data;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Store EDL/FDF bit (Extended data length or Flexible data-rate format)
    ---------------------------------------------------------------------------
    edl_store_proc : process(clk_sys, res_n_internal)
    begin
        if (res_n_internal = G_RESET_POLARITY) then
            rec_frame_type <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_edl = '1' and rx_trigger = '1') then
                rec_frame_type <= rx_data;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Store ESI bit (Error state indicator)
    ---------------------------------------------------------------------------
    esi_store_proc : process(clk_sys, res_n_internal)
    begin
        if (res_n_internal = G_RESET_POLARITY) then
            rec_esi <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_esi = '1' and rx_trigger = '1') then
                rec_esi <= rx_data;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Store BRS bit (Bit rate shift)
    ---------------------------------------------------------------------------
    brs_store_proc : process(clk_sys, res_n_internal)
    begin
        if (res_n_internal = G_RESET_POLARITY) then
            rec_brs <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_brs = '1' and rx_trigger = '1') then
                rec_brs <= rx_data;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Store DLC (Data length code)
    ---------------------------------------------------------------------------
    dlc_store_proc : process(clk_sys, res_n_internal)
    begin
        if (res_n_internal = G_RESET_POLARITY) then
            rec_dlc <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_dlc = '1' and rx_trigger = '1') then
                rec_dlc <= rx_shift_reg(2 downto 0) & rx_data;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Store RX Stuff Count
    ---------------------------------------------------------------------------
    stuff_count_store_proc : process(clk_sys, res_n_internal)
    begin
        if (res_n_internal = G_RESET_POLARITY) then
            rx_stuff_count <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_dlc = '1' and rx_trigger = '1') then
                rx_stuff_count <= rx_shift_reg(2 downto 0) & rx_data;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- CRC Value propagation to output. Valid only when rx_trigger='1' in
    -- the last clock cycle of last CRC bit.
    ---------------------------------------------------------------------------
    rx_crc <= rx_shift_reg(20 downto 0);
    
    ---------------------------------------------------------------------------
    -- Output data word. Valid one clock cycle after rx_trigger='1'. Whole 
    -- output data word is taken from shift reg. instead of taking the lowest
    -- byte from data input (as in case of ID, DLC, etc.). This is because
    -- the lowest bit does not have fixed position if data word is being
    -- stored! Due to this, when storing data word to RX Buffer, it must be
    -- stored one clock cycle after rx_trigger='1'!
    ---------------------------------------------------------------------------
    store_data_word <= rx_shift_reg;


    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    
    -- psl default clock is rising_edge(clk_sys);

    -- psl rx_shift_reg_byte_config : assert never
    --  (rx_shift_in_sel = '1') and 
    --  (rx_store_base_id = '1' or rx_store_ext_id = '1' or 
    --   rx_store_ide = '1' or rx_store_rtr = '1' or rx_store_edl = '1' or
    --   rx_store_dlc = '1' or rx_store_esi = '1' or rx_store_brs = '1' or
    --   rx_store_stuff_count = '1')
    --  report "RX Shift register should be configured as Byte shift " &
    --  "register only during DATA phase of CAN frame"
    --  severity error;
    
    -- psl no_simul_capture_and_clear : assert never
    --  (res_n_internal = G_RESET_POLARITY) and
    --  (rx_store_base_id = '1' or rx_store_ext_id = '1' or 
    --   rx_store_ide = '1' or rx_store_rtr = '1' or rx_store_edl = '1' or
    --   rx_store_dlc = '1' or rx_store_esi = '1' or rx_store_brs = '1' or
    --   rx_store_stuff_count = '1')
    --  report "RX Shift register should not be cleared when RX Data should" &
    --         " be stored!"
    --  severity error;
    
end architecture;