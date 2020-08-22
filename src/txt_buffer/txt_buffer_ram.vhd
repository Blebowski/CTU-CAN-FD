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
--  TXT Buffer RAM
--
-- Purpose:
--  Wrapper for dual port RAM in TXT Buffer.
--
-- Memory parameters:
--  Depth: 20
--  Word size: 32 bits
--  Read: Synchronous
--  Write: Synchronous
--  Port A: Write Only
--  Port B: Read only
--
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer.all;
use ctu_can_fd_rtl.can_constants.all;
use ctu_can_fd_rtl.can_components.all;
use ctu_can_fd_rtl.can_types.all;
use ctu_can_fd_rtl.cmn_lib.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.reduce_lib.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity txt_buffer_ram is
    generic(
        -- Reset polarity
        G_RESET_POLARITY       :     std_logic := '0'
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous reset
        res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Port A - Write (from Memory registers)
        ------------------------------------------------------------------------
        -- Address
        port_a_address       :in     std_logic_vector(4 downto 0);
        
        -- Data
        port_a_data_in       :in     std_logic_vector(31 downto 0);
        
        -- Write signal
        port_a_write         :in     std_logic;

        -----------------------------------------------------------------------
        -- Port B - Read (from CAN Core)
        -----------------------------------------------------------------------
        -- Address
        port_b_address       :in     std_logic_vector(4 downto 0);
        
        -- Data
        port_b_data_out      :out    std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of txt_buffer_ram is

begin
    
    ---------------------------------------------------------------------------
    -- RAM is implemented as synchronous inferred RAM for FPGAs.
    -- Synchronous RAM is chosen since some FPGA families does not provide
    -- inferred RAM for asynchronously read data (in the same clock cycle).
    ---------------------------------------------------------------------------
    txt_buf_ram_inst : inf_RAM_wrapper 
    generic map (
        G_WORD_WIDTH           => 32,
        G_DEPTH                => 20,
        G_ADDRESS_WIDTH        => port_a_address'length,
        G_RESET_POLARITY       => G_RESET_POLARITY,
        G_SYNC_READ            => true
    )
    port map(
        clk_sys              => clk_sys,
        res_n                => res_n,
        
        addr_A               => port_a_address,
        write                => port_a_write, 
        data_in              => port_a_data_in,
        
        addr_B               => port_b_address,
        data_out             => port_b_data_out
    );
  
end architecture;