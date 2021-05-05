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
--  AHB Interface.
--
-- Purpose:
--  Adaptor from AHB to internal bus of CTU CAN FD.
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

entity ahb_ifc is
    port (
        -----------------------------------------------------------------------
        -- CTU CAN FD Interface
        -----------------------------------------------------------------------
        data_in          : out std_logic_vector(31 downto 0);
        data_out         : in  std_logic_vector(31 downto 0);
        adress           : out std_logic_vector(15 downto 0);
        sbe              : out std_logic_vector(3 downto 0);
        scs              : out std_logic;
        swr              : out std_logic;
        srd              : out std_logic;

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
        hrdata           : out std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of ahb_ifc is
    
    -- Transfer type busy
    constant TT_BUSY   : std_logic_vector(1 downto 0) := "01";
    
    signal hsel_valid  : std_logic;

    signal write_acc_d : std_logic;
    signal write_acc_q : std_logic;
    
    signal haddr_q     : std_logic_vector(15 downto 0);

    -- HReady signals
    signal h_ready_raw : std_logic;

    -- Byte enable decoding
    signal sbe_d       : std_logic_vector(3 downto 0);
    signal sbe_q       : std_logic_vector(3 downto 0);

    signal swr_i       : std_logic;
    signal srd_i       : std_logic;


begin
    
    -- Only accept transaction if previous one completed OK!
    hsel_valid <= '1' when (hsel = '1' and hready = '1' and
                            htrans /= TT_BUSY)
                      else
                  '0';

    write_acc_d <= '1' when (hsel_valid = '1' and hwrite = '1')
                       else
                   '0' when (htrans /= TT_BUSY)
                       else
                   write_acc_q; 
    
    write_acc_reg_proc : process(hclk, hresetn)
    begin
        if (hresetn = '0') then
            write_acc_q <= '0';
        elsif (rising_edge(hclk)) then
            write_acc_q <= write_acc_d;
        end if;
    end process;
    
    haddr_reg_proc : process(hclk, hresetn)
    begin
        if (hresetn = '0') then
            haddr_q <= (OTHERS => '0');
            sbe_q   <= "0000";
        elsif (rising_edge(hclk)) then
            if (write_acc_d = '1') then
                haddr_q <= haddr(15 downto 0);
                sbe_q <= sbe_d;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Mux for address / Byte enables:
    --  1. Choose registered address for writes because data are available
    --     one clock cycle later.
    --  2. Otherwise choose direct adress.
    ---------------------------------------------------------------------------
    adress <= haddr_q when (write_acc_q = '1') else  
              haddr(15 downto 0);
    sbe <= sbe_q when (write_acc_q = '1') else
           sbe_d;
    
    ---------------------------------------------------------------------------
    -- Decoding HSIZE to Byte enables
    ---------------------------------------------------------------------------
    h_size_dec_proc : process(hsize)
    begin
        sbe_d <= "0000";
        case (hsize) is
        when "000" =>
            case haddr(1 downto 0) is
            when "00" => sbe_d <= "0001";
            when "01" => sbe_d <= "0010";
            when "10" => sbe_d <= "0100";
            when "11" => sbe_d <= "1000";
            when others => sbe_d <= "0000";
            end case;
        when "001" =>
            if (haddr(1) = '0') then
                sbe_d <= "0011";
            else
                sbe_d <= "1100";
            end if;
        when "010" =>
            sbe_d <= "1111";
        when others =>
            sbe_d <= "0000";
        end case;
    end process;
    
    ---------------------------------------------------------------------------
    -- Write control:
    --  When write access was registered, but master is not busy.
    ---------------------------------------------------------------------------
    swr_i <= '1' when (write_acc_q = '1' and htrans /= TT_BUSY)
               else
            '0';
    
    srd_i <= '1' when (hsel_valid = '1' and htrans /= TT_BUSY and hwrite = '0')
               else
           '0';
    
    scs <= '1' when (swr_i = '1' or srd_i = '1') else
           '0';

    swr <= swr_i;
    srd <= srd_i;

    ---------------------------------------------------------------------------
    -- We need to stall the master when there is Read after write because we
    -- cant deliver the read data in the same clock cycle as we are writing!
    ---------------------------------------------------------------------------
    hreadyout <= '0' when (write_acc_q = '1' and hsel_valid = '1' and
                           hwrite = '0')
                     else
                 '1';
        
    data_in <= hwdata;
    hrdata <= data_out;
    
    -- No errors
    hresp <= '0';
    
    -- <RELEASE_OFF>
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    -- psl default clock is rising_edge (hclk);
    
    -- psl ahb_read_after_write_cov : 
    --      cover {hwrite = '1' and hsel = '1'; hwrite = '0' and hsel = '1'};
    
    -- psl ahb_write_after_read_cov : 
    --      cover {hwrite = '0' and hsel = '1'; hwrite = '1' and hsel = '1'};
    
    -- psl ahb_write_after_write_cov : 
    --      cover {hwrite = '1' and hsel = '1'; hwrite = '1' and hsel = '1'};
    
    -- psl ahb_read_after_read_cov : 
    --      cover {hwrite = '0' and hsel = '1'; hwrite = '0' and hsel = '1'};
    
    -- <RELEASE_ON>
    
end architecture rtl;
