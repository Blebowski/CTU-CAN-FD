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
--  CRC Checking for CAN flexible data Rate. Three CRC are calculated simulta-
--  neously. Serial Data input. Operation starts with enable transition from 0 
--  to 1. Valid input data has to be present then. Circuit processes the data on
--  trig signal in logic 1. Circuit operation finishes when 1 to 0 transition on
--  enable signal appears. The output CRC is valid then. CRC stays valid until
--  following 0 to 1 enable transition. This also erases CRC registers.
--
--  Refer to CAN 2.0 or CAN FD Specification for CRC calculation algorithm
--------------------------------------------------------------------------------
-- Revision History:
--    June 2015   Created file
--    28.5.2016   Starting polynomial changed for crc 17 and crc 21. Highest bit
--                is now fixed in logic one to be compliant with CAN ISO FD. It
--                will be needed to implement both ways still since ISO and 
--                non-ISO FD will be changable via configuration bit! 
--    4.6.2016    Added drv_is_fd to cover differencce in highest bit of crc17
--                and crc21 polynomial.
--   13.7.2018    Replaced "crc15_nxt", "crc17_nxt", "crc21_nxt" by
--                signals instead of variable inside process.
--  15.11.2018    Replaced hard-coded CRC calculation with generic CRC entity.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.endian_swap.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity can_crc is
    generic(
        constant crc15_pol :     std_logic_vector(15 downto 0) := x"C599";
        constant crc17_pol :     std_logic_vector(19 downto 0) := x"3685B";
        constant crc21_pol :     std_logic_vector(23 downto 0) := x"302899"  
    );
    port(
        ------------------------------------------------------------------------
        -- Inputs
        ------------------------------------------------------------------------

        -- Serial data input
        signal data_in    :in   std_logic;

        -- System clock input 
        signal clk_sys    :in   std_logic;

        -- Trigger to sample the input value
        -- Note: trigger for CAN FD should be 1 clk_sys behind normal CAN 
        --       since for CAN FD bit stuffing is made before CRC calculation
        signal trig       :in   std_logic;
 
        -- Asynchronous reset
        signal res_n      :in   std_logic;

        -- By transition from 0 to 1 on enable sampled on clk_sys rising edge 
        -- (and with trig='1') operation is started. First bit of data already 
        -- has to be on data_in input.
        -- Circuit works as long as enable=1.
        signal enable     :in   std_logic; 

        -- Driving bus
        signal drv_bus    :in   std_logic_vector(1023 downto 0);

        ------------------------------------------------------------------------
        -- Outputs
        ------------------------------------------------------------------------
        signal crc15      :out  std_logic_vector(14 downto 0);
        signal crc17      :out  std_logic_vector(16 downto 0);
        signal crc21      :out  std_logic_vector(20 downto 0)
    );
  
end entity;


architecture rtl of can_crc is

    -- ISO CAN FD or NON ISO CAN FD Value
    signal drv_fd_type      :     std_logic;

    -- Initialization vectors
    signal init_vect_15     :     std_logic_vector(14 downto 0);
    signal init_vect_17     :     std_logic_vector(16 downto 0);
    signal init_vect_21     :     std_logic_vector(20 downto 0); 

begin

    -- ISO vs NON-ISO FD for selection of initialization vectors of 17 and 21.
    drv_fd_type         <= drv_bus(DRV_FD_TYPE_INDEX);

    -- For CRC 15 Init vector is constant zeroes
    init_vect_15        <= (OTHERS => '0');

    -- For CRC 17 and 21, Init vector depends on ISO/NON-ISO type
    init_vect_17(16)    <= '1' when (drv_fd_type = ISO_FD)
                               else
                            '0';
    init_vect_17(15 downto 0) <= (OTHERS => '0');

    init_vect_21(20)    <= '1' when (drv_fd_type = ISO_FD)
                               else
                            '0';
    init_vect_21(19 downto 0) <= (OTHERS => '0');


    ----------------------------------------------------------------------------
    -- CRC instances
    ----------------------------------------------------------------------------
    crc_calc_15_comp : crc_calc
    generic map(
        crc_width       => 15,
        reset_polarity  => ACT_RESET,
        polynomial      => crc15_pol
    )
    port map(
        res_n           => res_n,
        clk_sys         => clk_sys,

        data_in         => data_in,
        trig            => trig,
        enable          => enable,
        init_vect       => init_vect_15,
        crc             => crc15
    );

    crc_calc_17_comp : crc_calc
    generic map(
        crc_width       => 17,
        reset_polarity  => ACT_RESET,
        polynomial      => crc17_pol
    )
    port map(
        res_n           => res_n,
        clk_sys         => clk_sys,

        data_in         => data_in,
        trig            => trig,
        enable          => enable,
        init_vect       => init_vect_17,
        crc             => crc17
    );

    crc_calc_21_comp : crc_calc
    generic map(
        crc_width       => 21,
        reset_polarity  => ACT_RESET,
        polynomial      => crc21_pol
    )
    port map(
        res_n           => res_n,
        clk_sys         => clk_sys,

        data_in         => data_in,
        trig            => trig,
        enable          => enable,
        init_vect       => init_vect_21,
        crc             => crc21
    );
       
end architecture;
