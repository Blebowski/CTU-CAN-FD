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
--  RX Shift register.
--
-- Purpose:
--  Implements 32-bits shift register for reception of CAN serial data stream.
--  Receives CAN ID (Base, Extended), DLC, Data, Stuff count and CRC.
--  Controlled by Protocol control FSM, stored sequences are stored into capture
--  registers (CAN IDs, DLC, flags like RTR, BRS etc...), or RX Buffer FIFO
--  directly. Operates in Linear mode or Byte mode. Byte mode is used for
--  reception of Data field. Linear mode is used in all other multi-bit fields.
--  RX Shift register is shifted in "Process" pipeline stage.
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

entity rx_shift_reg is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;

        -- Asynchronous reset
        res_n                   :in   std_logic;

        ------------------------------------------------------------------------
        -- DFT support
        ------------------------------------------------------------------------
        scan_enable             :in   std_logic;

        -----------------------------------------------------------------------
        -- Trigger signals
        -----------------------------------------------------------------------
        -- RX Trigger
        rx_trigger              :in   std_logic;

        -----------------------------------------------------------------------
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual RX Data
        rx_data_nbs             :in   std_logic;

        -----------------------------------------------------------------------
        -- Protocol control FSM interface
        -----------------------------------------------------------------------
        -- Clear all registers in Shift register (Glitch free)
        rx_clear                :in  std_logic;

        -- Clock Enable RX Shift register for each byte.
        rx_shift_ena            :in  std_logic_vector(3 downto 0);

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
        
        -- RX Data length code (D input)
        rec_dlc_d               :out std_logic_vector(3 downto 0);
        
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
    signal res_n_i_d        : std_logic;
    signal res_n_i_q        : std_logic;
    signal res_n_i_q_scan   : std_logic;

    -- Shift register status
    signal rx_shift_reg_q : std_logic_vector(31 downto 0);

    -- RX Shift register shift
    signal rx_shift_cmd   : std_logic_vector(3 downto 0);
    
    -- Shift register input selector demuxed
    signal rx_shift_in_sel_demuxed : std_logic_vector(3 downto 0);

    signal rec_is_rtr_i : std_logic;
    signal rec_frame_type_i : std_logic;

begin

     -- Internal reset: Async reset + reset by design!
    res_n_i_d <= '0' when (rx_clear = '1' or res_n = '0') else
                 '1';

    ---------------------------------------------------------------------------
    -- Registering reset to avoid glitches
    ---------------------------------------------------------------------------
    rx_shift_res_reg_inst : entity ctu_can_fd_rtl.dff_arst
    generic map(
        G_RESET_POLARITY   => '0',

        -- Reset to the same value as is polarity of reset so that other DFFs
        -- which are reset by output of this one will be reset too!
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,          -- IN
        clk                => clk_sys,        -- IN
        input              => res_n_i_d,      -- IN

        output             => res_n_i_q       -- OUT
    );

    ---------------------------------------------------------------------------
    -- Registering reset to avoid glitches
    ---------------------------------------------------------------------------
    mux2_res_tst_inst : entity ctu_can_fd_rtl.mux2
    port map(
        a                  => res_n_i_q, 
        b                  => '1',
        sel                => scan_enable,

        -- Output
        z                  => res_n_i_q_scan
    );

    ---------------------------------------------------------------------------
    -- Shift the register when it is enabled and RX Trigger is active!
    -- Protocol control keeps the register disabled when e.g Bus is idle
    -- to save power!
    ---------------------------------------------------------------------------
    rx_shift_cmd_gen : for i in 0 to 3 generate
        rx_shift_cmd(i) <= '1' when (rx_trigger = '1' and rx_shift_ena(i) = '1')
                               else
                           '0';
    end generate rx_shift_cmd_gen;

    ---------------------------------------------------------------------------
    -- D input of received DLC is needed by Protocol control FSM in the last
    -- bit of DLC. Thus it is needed at the same clock cycle as it is stored
    -- so Q value is not yet there! 
    ---------------------------------------------------------------------------
    rec_dlc_d <= rx_shift_reg_q(2 downto 0) & rx_data_nbs;

    rx_shift_in_sel_demuxed <= rx_shift_in_sel & rx_shift_in_sel &
                               rx_shift_in_sel & rx_shift_in_sel;

    ---------------------------------------------------------------------------
    -- RX Shift register
    ---------------------------------------------------------------------------
    shift_reg_byte_inst : entity ctu_can_fd_rtl.shift_reg_byte
    generic map(
        G_RESET_POLARITY     => '0',
        G_RESET_VALUE        => x"00000000",
        G_NUM_BYTES          => 4
    )
    port map(
        clk                  => clk_sys,
        res_n                => res_n_i_q_scan,
        input                => rx_data_nbs,
        byte_clock_ena       => rx_shift_cmd,
        byte_input_sel       => rx_shift_in_sel_demuxed,
        reg_stat             => rx_shift_reg_q
    );

    ---------------------------------------------------------------------------
    -- Store Identifier
    ---------------------------------------------------------------------------
    id_store_proc : process(clk_sys, res_n_i_q_scan)
    begin
        if (res_n_i_q_scan = '0') then
            rec_ident <= (OTHERS => '0');    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_base_id = '1') then
                rec_ident(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) <=
                    rx_shift_reg_q(9 downto 0) & rx_data_nbs;
            end if;

            if (rx_store_ext_id = '1') then
                rec_ident(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) <= 
                    rx_shift_reg_q(16 downto 0) & rx_data_nbs;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Store IDE bit (Identifier type)
    ---------------------------------------------------------------------------
    ide_store_proc : process(clk_sys, res_n_i_q_scan)
    begin
        if (res_n_i_q_scan = '0') then
            rec_ident_type <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_ide = '1') then
                rec_ident_type <= rx_data_nbs;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- RX Store RTR bit (Remote transmission request bit)
    ---------------------------------------------------------------------------
    rx_store_proc : process(clk_sys, res_n_i_q_scan)
    begin
        if (res_n_i_q_scan = '0') then
            rec_is_rtr_i <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_rtr = '1') then
                rec_is_rtr_i <= rx_data_nbs;
            end if;
        end if;
    end process;
    
    -- RTR flag can't be active at the same time as FDF. FDF has priority!
    rec_is_rtr <= rec_is_rtr_i when (rec_frame_type_i = NORMAL_CAN) else
                  NO_RTR_FRAME;

    ---------------------------------------------------------------------------
    -- Store EDL/FDF bit (Extended data length or Flexible data-rate format)
    ---------------------------------------------------------------------------
    edl_store_proc : process(clk_sys, res_n_i_q_scan)
    begin
        if (res_n_i_q_scan = '0') then
            rec_frame_type_i <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_edl = '1') then
                rec_frame_type_i <= rx_data_nbs;
            end if;
        end if;
    end process;

    rec_frame_type <= rec_frame_type_i;


    ---------------------------------------------------------------------------
    -- Store ESI bit (Error state indicator)
    ---------------------------------------------------------------------------
    esi_store_proc : process(clk_sys, res_n_i_q_scan)
    begin
        if (res_n_i_q_scan = '0') then
            rec_esi <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_esi = '1') then
                rec_esi <= rx_data_nbs;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Store BRS bit (Bit rate shift)
    ---------------------------------------------------------------------------
    brs_store_proc : process(clk_sys, res_n_i_q_scan)
    begin
        if (res_n_i_q_scan = '0') then
            rec_brs <= '0';    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_brs = '1') then
                rec_brs <= rx_data_nbs;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Store DLC (Data length code)
    ---------------------------------------------------------------------------
    dlc_store_proc : process(clk_sys, res_n_i_q_scan)
    begin
        if (res_n_i_q_scan = '0') then
            rec_dlc <= (OTHERS => '0');    
        elsif (rising_edge(clk_sys)) then
            if (rx_store_dlc = '1') then
                rec_dlc <= rx_shift_reg_q(2 downto 0) & rx_data_nbs;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Store RX Stuff Count
    ---------------------------------------------------------------------------
    stuff_count_store_proc : process(clk_sys, res_n_i_q_scan)
    begin
        if (res_n_i_q_scan = '0') then
            rx_stuff_count <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (rx_store_stuff_count = '1') then
                rx_stuff_count <= rx_shift_reg_q(2 downto 0) & rx_data_nbs;
            end if;
        end if;
    end process;


    ---------------------------------------------------------------------------
    -- CRC Value propagation to output. Valid only when rx_trigger='1' in
    -- the last clock cycle of last CRC bit.
    ---------------------------------------------------------------------------
    rx_crc <= rx_shift_reg_q(20 downto 0);
    
    ---------------------------------------------------------------------------
    -- Output data word. Valid one clock cycle after rx_trigger='1'. Whole 
    -- output data word is taken from shift reg. instead of taking the lowest
    -- byte from data input (as in case of ID, DLC, etc.). This is because
    -- the lowest bit does not have fixed position if data word is being
    -- stored! Due to this, when storing data word to RX Buffer, it must be
    -- stored one clock cycle after rx_trigger='1'!
    ---------------------------------------------------------------------------
    store_data_word <= rx_shift_reg_q;

    -- <RELEASE_OFF>
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
    --  (res_n_i_q_scan = '0') and
    --  (rx_store_base_id = '1' or rx_store_ext_id = '1' or 
    --   rx_store_ide = '1' or rx_store_rtr = '1' or rx_store_edl = '1' or
    --   rx_store_dlc = '1' or rx_store_esi = '1' or rx_store_brs = '1' or
    --   rx_store_stuff_count = '1')
    --  report "RX Shift register should not be cleared when RX Data should" &
    --         " be stored!"
    --  severity error;

    -- psl rx_shift_reg_clear_cov : cover
    --  {rx_clear = '1'};
    
    -- In linear mode, all bytes are shifting at once!
    -- psl rx_shift_reg_linear_mode_cov : cover
    --  {rx_shift_in_sel = '0' and rx_shift_ena = "1111"};

    -- psl rx_shift_reg_byte_mode_byte_1_cov : cover
    --  {rx_shift_in_sel = '1' and rx_shift_ena = "0001"};

    -- psl rx_shift_reg_byte_mode_byte_2_cov : cover
    --  {rx_shift_in_sel = '1' and rx_shift_ena = "0010"};

    -- psl rx_shift_reg_byte_mode_byte_3_cov : cover
    --  {rx_shift_in_sel = '1' and rx_shift_ena = "0100"};

    -- psl rx_shift_reg_byte_mode_byte_4_cov : cover
    --  {rx_shift_in_sel = '1' and rx_shift_ena = "1000"};

    -- psl rx_shift_reg_store_base_id_cov : cover
    --  {rx_store_base_id = '1'};
    
    -- psl rx_shift_reg_store_ext_id_cov : cover
    --  {rx_store_ext_id = '1'};
    
    -- psl rx_shift_reg_store_ide_cov : cover
    --  {rx_store_ide = '1'};

    -- psl rx_shift_reg_store_rtr_cov : cover
    --  {rx_store_rtr = '1'};
    
    -- psl rx_shift_reg_store_edl_cov : cover
    --  {rx_store_edl = '1'};
    
    -- psl rx_shift_reg_store_dlc_cov : cover
    --  {rx_store_dlc = '1'};

    -- psl rx_shift_reg_store_esi_cov : cover
    --  {rx_store_esi = '1'};

    -- psl rx_shift_reg_store_brs_cov : cover
    --  {rx_store_brs = '1'};

    -- psl rx_shift_reg_store_stuff_count_cov : cover
    --  {rx_store_stuff_count = '1'};

    -- <RELEASE_ON>

end architecture;