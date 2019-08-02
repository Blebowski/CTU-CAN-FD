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
-- Module:
--  ABP Interface
--
-- Purpose:
--  Adaptor from APB4 to internal bus of CTU CAN FD.
--
-- Note: 
--  This is not strictly APB conformant as the read data stays only for the
--  next cycle; after that they are zeroed.
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
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity apb_ifc is
    generic (
        -- ID (bits  19-16 of reg_addr_o)
        ID : natural := 1
    );
    port (
        aclk             : in  std_logic;
        arstn            : in  std_logic;

        -----------------------------------------------------------------------
        -- CTU CAN FD Interface
        -----------------------------------------------------------------------
        reg_data_in_o    : out std_logic_vector(31 downto 0);
        reg_data_out_i   : in  std_logic_vector(31 downto 0);
        reg_addr_o       : out std_logic_vector(15 downto 0);
        reg_be_o         : out std_logic_vector(3 downto 0);
        reg_rden_o       : out std_logic;
        reg_wren_o       : out std_logic;

        -----------------------------------------------------------------------
        -- APB interface 
        -----------------------------------------------------------------------
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
    signal rst_countdown_reg : natural range 0 to 3;
    signal next_apb_pready   : std_logic;
    signal ready_for_read    : std_logic;

    function to_std_logic(a : boolean) return std_logic is
    begin
        if a then
            return '1';
        else
            return '0';
        end if;
    end function to_std_logic;
begin
    
    reg_data_in_o <= s_apb_pwdata;
    s_apb_prdata  <= reg_data_out_i;

    reg_addr_o(ID_ADRESS_HIGHER downto ID_ADRESS_LOWER) <=
        std_logic_vector(to_unsigned(ID, 4));

    -- forward only aligned addresses
    -- (the bridge/CPU will then pick the bytes itself)
    reg_addr_o(ID_ADRESS_LOWER - 1 downto 2) <=
        s_apb_paddr(ID_ADRESS_LOWER - 1 downto 2);

    reg_addr_o(1 downto 0) <= (others => '0');

    -- path can be shortened by registering
    reg_be_o   <= s_apb_pstrb when s_apb_pwrite = '1'
                              else
                  (others => '1');


    -- Read must be issued one cycle before finishing the transaction
    -- and must be active only for 1 cycle so that when reading from a FIFO
    -- the pointer is only incremented once.
    -- rst_countdown_reg = 0 -> not s_apb_penable
    -- otherwise rst_countdown_reg = 1 (s_apb_penable will stay in '1', so double trigger won't happen)
    ready_for_read <= (not s_apb_penable and next_apb_pready) or to_std_logic(rst_countdown_reg = 1);
    reg_rden_o <= s_apb_psel and not s_apb_pwrite and ready_for_read;

    -- path can be shortened by registering it
    -- ignore s_apb_pprot
    reg_wren_o <= s_apb_psel and s_apb_pwrite and s_apb_penable;

    p_rready : process(arstn, aclk)
    begin
        if arstn = '0' then
            rst_countdown_reg <= 3;
        elsif rising_edge(aclk) then
            if rst_countdown_reg > 0 then
                rst_countdown_reg <= rst_countdown_reg - 1;
            end if;
        end if;
    end process;

    next_apb_pready <= '1' when rst_countdown_reg = 0 else '0';
    s_apb_pready <= next_apb_pready;
    s_apb_pslverr <= '0';
    
    
    -- ASRT_START
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    -- psl default clock is rising_edge (aclk);
    
    -- psl onecycle_rden_asrt : assert always reg_rden_o = '1' -> next reg_rden_o = '0'
    --  report "Read enable shall be active only for one clock cycle"
    --  severity error;
      
    -- ASRT_END
    
end architecture rtl;