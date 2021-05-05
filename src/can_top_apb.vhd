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
-- Purpose:
--    Top-level entity using APB4.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

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

entity can_top_apb is
    generic(
        rx_buffer_size      : natural range 32 to 4098 := 128;
        txt_buffer_count    : natural range 2 to 8     := 4; 
        sup_filtA           : boolean                  := true;
        sup_filtB           : boolean                  := true;
        sup_filtC           : boolean                  := true;
        sup_range           : boolean                  := true;
        sup_traffic_ctrs    : boolean                  := true;
        sup_test_registers  : boolean                  := true
    );
    port(
        aclk             : in  std_logic;
        arstn            : in  std_logic;
        scan_enable      : in  std_logic;

        irq              : out std_logic;
        CAN_tx           : out std_logic;
        CAN_rx           : in  std_logic;
        timestamp        : in std_logic_vector(63 downto 0);

        -- Ports of APB4
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
end entity can_top_apb;

architecture rtl of can_top_apb is
 
    signal reg_data_in      : std_logic_vector(31 downto 0);
    signal reg_data_out     : std_logic_vector(31 downto 0);
    signal reg_addr         : std_logic_vector(15 downto 0);
    signal reg_be           : std_logic_vector(3 downto 0);
    signal reg_rden         : std_logic;
    signal reg_wren         : std_logic;
    
begin

    can_inst: CAN_top_level
        generic map (
            rx_buffer_size      => rx_buffer_size,
            txt_buffer_count    => txt_buffer_count,
            sup_filtA           => sup_filtA,
            sup_filtB           => sup_filtB,
            sup_filtC           => sup_filtC,
            sup_range           => sup_range,
            sup_traffic_ctrs    => sup_traffic_ctrs
        )
        port map (
            clk_sys         => aclk,
            res_n           => arstn,
            
            scan_enable     => scan_enable,

            data_in         => reg_data_in,
            data_out        => reg_data_out,
            adress          => reg_addr,
            scs             => '1',
            srd             => reg_rden,
            swr             => reg_wren,
            sbe             => reg_be,

            int             => irq,

            CAN_tx          => CAN_tx,
            CAN_rx          => CAN_rx,

            timestamp       => timestamp
        );

    apb_inst : apb_ifc
        port map (
            aclk           => aclk,

            reg_data_in_o  => reg_data_in,
            reg_data_out_i => reg_data_out,
            reg_addr_o     => reg_addr,
            reg_be_o       => reg_be,
            reg_rden_o     => reg_rden,
            reg_wren_o     => reg_wren,

            s_apb_paddr    => s_apb_paddr,
            s_apb_penable  => s_apb_penable,
            s_apb_pprot    => s_apb_pprot,
            s_apb_prdata   => s_apb_prdata,
            s_apb_pready   => s_apb_pready,
            s_apb_psel     => s_apb_psel,
            s_apb_pslverr  => s_apb_pslverr,
            s_apb_pstrb    => s_apb_pstrb,
            s_apb_pwdata   => s_apb_pwdata,
            s_apb_pwrite   => s_apb_pwrite
        );
        
end architecture rtl;