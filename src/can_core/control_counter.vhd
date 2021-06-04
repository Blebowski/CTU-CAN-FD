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
--  Control counter.
--
-- Purpose:
--  Measures duration of CAN Frame fields which last more bits. Pre-loaded by
--  Protocol control FSM and counts till 0. Signals reaching 1 and 0. Contains
--  complementary counter which counts from 0 and indicates that whole byte
--  elapsed or whole memory word elapsed (4 bytes). Provides byte index within
--  a memory word for addressing of CAN Data byte. Contains arbitration lost
--  capture register which stores bit position in which unit lost arbitration.
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

entity control_counter is
    generic(
        -- Width of control counter
        G_CTRL_CTR_WIDTH        :     natural := 9 
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys         :in   std_logic;

        -- Asynchronous reset
        res_n           :in   std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- RX Trigger (Decrements the counter)
        rx_trigger            :in   std_logic;

        -- Control counter counting is enabled
        ctrl_ctr_ena          :in   std_logic;

        -- Pre-load control counter
        ctrl_ctr_pload        :in   std_logic;
  
        -- Pre-load value for control counter
        ctrl_ctr_pload_val    :in   std_logic_vector(G_CTRL_CTR_WIDTH - 1 downto 0);
        
        -- Complementary counter enable
        compl_ctr_ena         :in    std_logic;
        
        -- Arbitration lost
        arbitration_lost      :in    std_logic;
        
        -- Arbitration lost
        alc_id_field          :in    std_logic_vector(2 downto 0);

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Control counter is equal to zero
        ctrl_ctr_zero           :out std_logic;

        -- Control counter is equal to one
        ctrl_ctr_one            :out std_logic;

        -- Control counter counted multiple of 8 bits
        ctrl_counted_byte       :out std_logic;
        
        -- Control counter byte index within a memory word
        ctrl_counted_byte_index :out std_logic_vector(1 downto 0);
        
        -- Index of memory word in TXT Buffer
        ctrl_ctr_mem_index      :out std_logic_vector(4 downto 0);
        
        -- Arbitration lost capture
        alc                     :out std_logic_vector(7 downto 0)
    );
end entity;

architecture rtl of control_counter is

    -- Control counter
    signal ctrl_ctr_d : unsigned(G_CTRL_CTR_WIDTH - 1 downto 0);
    signal ctrl_ctr_q : unsigned(G_CTRL_CTR_WIDTH - 1 downto 0);

    -- Clock enable
    signal ctrl_ctr_ce : std_logic;

    -- Complementary counter
    signal compl_ctr_d  : unsigned(G_CTRL_CTR_WIDTH - 1 downto 0);
    signal compl_ctr_q  : unsigned(G_CTRL_CTR_WIDTH - 1 downto 0);
    signal compl_ctr_div_32 : unsigned(G_CTRL_CTR_WIDTH - 6 downto 0);
    signal compl_ctr_div_32_plus_5 : integer range 0 to 20;
    signal compl_ctr_div_32_plus_5_sat : integer range 0 to 19;
    signal compl_ctr_ce : std_logic;
    
    constant C_CTRL_CTR_ZEROES : unsigned(G_CTRL_CTR_WIDTH - 1 downto 0) :=
        (OTHERS => '0');

begin

    -- Next value
    ctrl_ctr_d <= unsigned(ctrl_ctr_pload_val) when (ctrl_ctr_pload = '1') else
                              (ctrl_ctr_q - 1) when (rx_trigger = '1') else
                                   ctrl_ctr_q;
                 
    -- Clock enable
    ctrl_ctr_ce <= '1' when (rx_trigger = '1' and ctrl_ctr_ena = '1') else
                   '1' when (ctrl_ctr_pload = '1') else
                   '0';

    ctrl_ctr_zero <= '1' when (ctrl_ctr_q = 0) else
                     '0';

    ctrl_ctr_one <= '1' when (ctrl_ctr_q = 1) else
                    '0';
    
    ---------------------------------------------------------------------------
    -- Control Counter register
    ---------------------------------------------------------------------------                   
    retr_ctr_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            ctrl_ctr_q <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (ctrl_ctr_ce = '1') then
                ctrl_ctr_q <= ctrl_ctr_d;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Complementary counter.
    --
    -- This counter counts bits during data field. This is done to calculate
    -- address of Data word in TXT Buffer, byte index within memory word and
    -- indicate whole byte of data elapsed!
    -- 
    -- Counter is erased when control counter is preloaded (upon data field),
    -- and counts only during data field.
    ---------------------------------------------------------------------------   
    compl_ctr_d <= (OTHERS => '0') when (ctrl_ctr_pload = '1') else
                   compl_ctr_q + 1;

    compl_ctr_ce <= '1' when (ctrl_ctr_pload = '1') else
                    '1' when (compl_ctr_ena = '1') else
                    '0';

    compl_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            compl_ctr_q <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (compl_ctr_ce = '1') then
                compl_ctr_q <= compl_ctr_d;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Status signals calculated from complementary counter
    ---------------------------------------------------------------------------
    -- Control counter counted number of bits is on last bit within a byte!
    ctrl_counted_byte <= '1' when (compl_ctr_q(2 downto 0) = "111")
                             else
                         '0';

    -- Byte index within memory word!
    ctrl_counted_byte_index <= std_logic_vector(compl_ctr_q(4 downto 3));
    
    -- Complementary counter divided by 32
    compl_ctr_div_32 <= compl_ctr_q(G_CTRL_CTR_WIDTH - 1 downto 5);

    -- Complementary counter divided by 32, + 5
    compl_ctr_div_32_plus_5 <= to_integer(compl_ctr_div_32) + 5;
    
    -- Saturate to 19
    compl_ctr_div_32_plus_5_sat <=
        compl_ctr_div_32_plus_5 when (compl_ctr_div_32_plus_5 < 19) else
        19;
    
    ---------------------------------------------------------------------------
    -- Index of word in TXT Buffer memory. Always Index one word further than
    -- we are transmitting to allow loading data on TXT Buffer RAM output:
    --  Data Bytes 1 - 4 (0 - 32) = Address word 5
    --  Data Bytes 5 - 8 (33 - 64) = Address word 6
    --  ...
    --  Data Bytes 61 - 64 () = Address word 19
    ---------------------------------------------------------------------------
    ctrl_ctr_mem_index <= 
        std_logic_vector(to_unsigned(compl_ctr_div_32_plus_5_sat, 5));


    ---------------------------------------------------------------------------
    -- Arbitration lost capture register
    ---------------------------------------------------------------------------
    alc_capt_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            alc <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (arbitration_lost = '1') then
                alc(4 downto 0) <= std_logic_vector(ctrl_ctr_q(4 downto 0));
                alc(7 downto 5) <= alc_id_field;
            end if;
        end if;
    end process;

    -- <RELEASE_OFF>
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);
    
    -- <RELEASE_ON>
end architecture;