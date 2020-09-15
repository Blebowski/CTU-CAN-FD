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
--    Top-level entity using AHB.
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

entity can_top_ahb is
    generic(
        rx_buffer_size   : natural range 32 to 4098 := 128;
        sup_filtA        : boolean                  := true;
        sup_filtB        : boolean                  := true;
        sup_filtC        : boolean                  := true;
        sup_range        : boolean                  := true;
        sup_traffic_ctrs : boolean                  := true
    );
    port(
        -----------------------------------------------------------------------
        -- AHB interface 
        -----------------------------------------------------------------------
        hresetn          : in std_logic;
        hclk             : in std_logic;
        haddr            : in std_logic_vector(31 downto 0);
        hwdata           : in std_logic_vector(31 downto 0);
        hsel             : in std_logic;
        hwrite           : in std_logic;
        hsize            : in std_logic_vector(2 downto 0);
        hburst           : in std_logic_vector(2 downto 0);
        hprot            : in std_logic_vector(3 downto 0);
        htrans           : in std_logic_vector(1 downto 0);
        hmastlock        : in std_logic;
        hready           : in std_logic;
        hreadyout        : out std_logic;
        hresp            : out std_logic;
        hrdata           : out std_logic_vector(31 downto 0);
        
        -----------------------------------------------------------------------
        -- CAN Bus 
        -----------------------------------------------------------------------
        can_tx           : out std_logic;
        can_rx           : in  std_logic;
        
        -----------------------------------------------------------------------
        -- Timestamp 
        -----------------------------------------------------------------------
        timestamp        : in  std_logic_vector(63 downto 0);

        -----------------------------------------------------------------------
        -- Interrupt
        -----------------------------------------------------------------------
        int              : out std_logic      
    );
end entity can_top_ahb;

architecture rtl of can_top_ahb is
 
    signal ctu_can_data_in   : std_logic_vector(31 downto 0);
    signal ctu_can_data_out  : std_logic_vector(31 downto 0);
    signal ctu_can_adress    : std_logic_vector(15 downto 0);
    
    signal ctu_can_scs       : std_logic;
    signal ctu_can_srd       : std_logic;
    signal ctu_can_swr       : std_logic;
    signal ctu_can_sbe       : std_logic_vector(3 downto 0);
    
begin

    can_inst : CAN_top_level
    generic map (
        rx_buffer_size  => rx_buffer_size,
        sup_filtA       => sup_filtA,
        sup_filtB       => sup_filtB,
        sup_filtC       => sup_filtC,
        sup_range       => sup_range,
        sup_traffic_ctrs=> sup_traffic_ctrs
    )
    port map (
        clk_sys         => hclk,
        res_n           => hresetn,

        data_in         => ctu_can_data_in,
        data_out        => ctu_can_data_out,
        adress          => ctu_can_adress,
        scs             => ctu_can_scs,
        srd             => ctu_can_srd,
        swr             => ctu_can_swr,
        sbe             => ctu_can_sbe,

        int             => int,

        CAN_tx          => CAN_tx,
        CAN_rx          => CAN_rx,

        timestamp       => timestamp
    );

    ahb_ifc_inst : ahb_ifc
    port map(
        -- CTU CAN FD Interface
        data_in          => ctu_can_data_in,
        data_out         => ctu_can_data_out,
        adress           => ctu_can_adress,
        sbe              => ctu_can_sbe,
        scs              => ctu_can_scs,
        swr              => ctu_can_swr,
        srd              => ctu_can_srd,

        -- AHB interface 
        hresetn          => hresetn,
        hclk             => hclk,
        haddr            => haddr,
        hwdata           => hwdata,
        hsel             => hsel,
        hwrite           => hwrite,
        hsize            => hsize,
        hburst           => hburst,
        hprot            => hprot,
        htrans           => htrans,
        hmastlock        => hmastlock,
        hready           => hready,
        hreadyout        => hreadyout,
        hresp            => hresp,
        hrdata           => hrdata
    );
  
end architecture rtl;