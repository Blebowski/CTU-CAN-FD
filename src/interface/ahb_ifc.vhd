--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2021-2023 Ondrej Ille
-- Copyright (C) 2023-     Logic Design Services Ltd.s
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- non-commercial purposes. Using the Component for commercial purposes is
-- forbidden unless previously agreed with Copyright holder.
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
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity ahb_ifc is
    port (
        -------------------------------------------------------------------------------------------
        -- CTU CAN FD Interface
        -------------------------------------------------------------------------------------------
        data_in          : out std_logic_vector(31 downto 0);
        data_out         : in  std_logic_vector(31 downto 0);
        adress           : out std_logic_vector(15 downto 0);
        sbe              : out std_logic_vector(3 downto 0);
        scs              : out std_logic;
        swr              : out std_logic;
        srd              : out std_logic;

        -------------------------------------------------------------------------------------------
        -- AHB interface
        -------------------------------------------------------------------------------------------
        hresetn          : in  std_logic;
        hclk             : in  std_logic;
        haddr            : in  std_logic_vector(31 downto 0);
        hwdata           : in  std_logic_vector(31 downto 0);
        hsel             : in  std_logic;
        hwrite           : in  std_logic;
        hsize            : in  std_logic_vector(2 downto 0);
        hburst           : in  std_logic_vector(2 downto 0);
        hprot            : in  std_logic_vector(3 downto 0);
        htrans           : in  std_logic_vector(1 downto 0);
        hmastlock        : in  std_logic;
        hready           : in  std_logic;
        hreadyout        : out std_logic;
        hresp            : out std_logic_vector(1 downto 0);
        hrdata           : out std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of ahb_ifc is

    -- Registered address phase signals
    signal haddr_q          : std_logic_vector(15 downto 0);
    signal hsel_q           : std_logic;
    signal hwrite_q         : std_logic;
    signal hsize_q          : std_logic_vector(2 downto 0);

    -- Registered ready
    signal hready_q         : std_logic;

    -- Stall indication
    signal hreadyout_d      : std_logic;

begin

    p_addr_phase_reg : process(hresetn, hclk)
    begin
        if (hresetn = '0') then
            haddr_q   <= (others => '0');
            hsel_q    <= '0';
            hwrite_q  <= '0';
            hsize_q   <= (others => '0');
        elsif (rising_edge(hclk)) then
            if (hready = '1') then
                haddr_q   <= haddr(15 downto 0);
                hsel_q    <= hsel;
                hwrite_q  <= hwrite;
                hsize_q   <= hsize;
            end if;
        end if;
    end process;

    -- Stall upon read
    hreadyout_d <= '0' when (hsel = '1' and hwrite = '0' and hready = '1')
                       else
                   '1';

    p_hready_regs : process(hresetn, hclk)
    begin
        if (hresetn = '0') then
            hreadyout <= '1';
            hready_q <= '1';
        elsif (rising_edge(hclk)) then
            hreadyout <= hreadyout_d;
            hready_q <= hready;
        end if;
    end process;

    -- Activate on CTU CAN FDs RAM-like bus only upon first cycle
    -- of current AHB transfer. If "hready" is low, then the AHB
    -- master keeps the address phase valid, and we don't sample
    -- the next address phase, therefore we would execute a
    -- transfer twice. Since read may have side effect, this is
    -- undesirable!
    scs <= '1' when (hsel_q = '1' and hready_q = '1') else
           '0';

    swr <= '1' when (hsel_q = '1' and hready_q = '1' and hwrite_q = '1') else
           '0';

    srd <= '1' when (hsel_q = '1' and hready_q = '1' and hwrite_q = '0') else
           '0';

    adress <= haddr_q;

    data_in <= hwdata;
    hrdata <= data_out;
    hresp <= "00";

    -----------------------------------------------------------------------------------------------
    -- Decoding HSIZE to Byte enables
    -----------------------------------------------------------------------------------------------
    p_h_size_dec : process(hsize_q, haddr_q)
    begin
        sbe <= "0000";
        case (hsize_q) is
        when "000" =>
            case haddr_q(1 downto 0) is
            when "00" => sbe <= "0001";
            when "01" => sbe <= "0010";
            when "10" => sbe <= "0100";
            when "11" => sbe <= "1000";
            when others => sbe <= "0000";
            end case;
        when "001" =>
            if (haddr_q(1) = '0') then
                sbe <= "0011";
            else
                sbe <= "1100";
            end if;
        when "010" =>
            sbe <= "1111";
        when others =>
            sbe <= "0000";
        end case;
    end process;

    -- <RELEASE_OFF>
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------

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
