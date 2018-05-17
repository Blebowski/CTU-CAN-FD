--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Project advisors and co-authors:
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--    Adaptor from APB4 to internal bus.
--------------------------------------------------------------------------------
-- Revision History:
--    May 2018   First Implementation - Martin Jerabek
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.CANconstants.all;

entity apb_ifc is
    generic (
        -- ID (bits  19-16 of reg_addr_o)
        ID : natural := 1
    );
    port (
        aclk             : in  std_logic;
        arstn            : in  std_logic;

        reg_data_in_o    : out std_logic_vector(31 downto 0);
        reg_data_out_i   : in  std_logic_vector(31 downto 0);
        reg_addr_o       : out std_logic_vector(23 downto 0);
        reg_be_o         : out std_logic_vector(3 downto 0);
        reg_rden_o       : out std_logic;
        reg_wren_o       : out std_logic;

        s_apb_paddr      : in  std_logic_vector(31 downto 0);
        s_apb_penable    : in  std_logic;
        s_apb_pprot      : in  std_logic_vector(2 downto 0);
        s_apb_prdata     : out std_logic_vector(31 downto 0);
        s_apb_pready     : out std_logic;
        s_apb_psel       : in  std_logic;
        s_apb_pslverr    : out std_logic;
        s_apb_pstrb      : in  std_logic_vector(3 downto 0);
        s_apb_pwdata     : in  std_logic_vector(31 downto 0);
        s_apb_pwrite     : in  std_logic
    );
end entity;

architecture rtl of apb_ifc is
    signal rst_countdown_reg : natural range 0 to 2;
begin
    reg_data_in_o <= s_apb_pwdata;
    s_apb_prdata  <= reg_data_out_i;

    reg_addr_o(COMP_TYPE_ADRESS_HIGHER downto COMP_TYPE_ADRESS_LOWER) <= CAN_COMPONENT_TYPE;
    reg_addr_o(ID_ADRESS_HIGHER downto ID_ADRESS_LOWER) <= std_logic_vector(to_unsigned(ID,4));
    -- forward only aligned addresses (the bridge/CPU will then pick the bytes itself)
    reg_addr_o(ID_ADRESS_LOWER-1 downto 2) <= s_apb_paddr(ID_ADRESS_LOWER-1 downto 2);
    reg_addr_o(1 downto 0) <= (others => '0');

    reg_be_o   <= s_apb_pstrb when s_apb_pwrite = '1' else (others => '1'); -- path can be shortened by registering
    reg_rden_o <= s_apb_psel and not s_apb_pwrite;
    reg_wren_o <= s_apb_psel and s_apb_pwrite and s_apb_penable; -- path can be shortened by registering it
    -- ignore s_apb_pprot

    p_rready:process(arstn, aclk)
    begin
        if arstn = '0' then
            rst_countdown_reg <= 2;
        elsif rising_edge(aclk) then
            if rst_countdown_reg > 0 then
                rst_countdown_reg <= rst_countdown_reg - 1;
            end if;
        end if;
    end process;

    s_apb_pready <= '1' when rst_countdown_reg = 0 else '0';
    s_apb_pslverr <= '0';
end architecture rtl;
