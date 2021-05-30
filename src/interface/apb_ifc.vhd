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
--  ABP Interface
--
-- Purpose:
--  Adaptor from APB4 to internal bus of CTU CAN FD.
--
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

entity apb_ifc is
    generic (
        -- ID (bits  19-16 of reg_addr_o)
        ID : natural := 1
    );
    port (
        aclk             : in  std_logic;

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
    reg_be_o   <= s_apb_pstrb when (s_apb_pwrite = '1') else
                  (others => '1');

    ---------------------------------------------------------------------------
    -- Read must be issued one cycle before finishing the transaction and
    -- must be active only for 1 cycle, so that when reading from a FIFO the 
    -- pointer is only incremented once.
    ---------------------------------------------------------------------------
    reg_rden_o <= '1' when (s_apb_psel = '1' and s_apb_pwrite = '0' and
                            s_apb_penable = '0')
                      else
                  '0';

    reg_wren_o <= '1' when (s_apb_psel = '1' and s_apb_pwrite = '1' and
                            s_apb_penable = '1')
                      else
                  '0';

    s_apb_pready  <= '1';
    s_apb_pslverr <= '0';
    
    -- <RELEASE_OFF>
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    -- psl default clock is rising_edge (aclk);
    
    -- psl onecycle_rden_asrt : assert always reg_rden_o = '1' -> next reg_rden_o = '0'
    --  report "Read enable shall be active only for one clock cycle"
    --  severity error;
      
    -- <RELEASE_ON>
    
end architecture rtl;