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
--  Generic CRC calculation module.
-- 
-- Purpose:
--  Calculates CRC sequence of generic length from Serial Data input. Processes
--  input data with trigger signal only when enabled. CRC value remains on 
--  output when module is disabled. Upon start of calculation, CRC register 
--  is loaded with initial value.
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

entity crc_calc is
    generic(        
        -- Width of CRC sequence
        G_CRC_WIDTH         : natural;

        -- CRC Polynomial
        G_POLYNOMIAL        : std_logic_vector
    );
    port(
        ------------------------------------------------------------------------
        -- System clock and Asynchronous Reset
        ------------------------------------------------------------------------
        -- System clock input 
        clk_sys    :in   std_logic;

        -- Asynchronous reset
        res_n      :in   std_logic;

        ------------------------------------------------------------------------
        -- CRC Calculation control
        ------------------------------------------------------------------------
        -- Serial data input for CRC calculation
        data_in    :in   std_logic;

        -- Trigger to sample the input data
        trig       :in   std_logic;
 
        -- CRC calculation enabled
        enable     :in   std_logic;

        -- Initialization vector for CRC calculation
        init_vect  :in   std_logic_vector(G_CRC_WIDTH - 1 downto 0);

        -- Load CRC Initialization vector
        load_init_vect : in std_logic;

        ------------------------------------------------------------------------
        -- CRC output
        ------------------------------------------------------------------------
        crc         :out  std_logic_vector(G_CRC_WIDTH - 1 downto 0)
    );    
end entity;

architecture rtl of crc_calc is

    ----------------------------------------------------------------------------
    -- Internal registers
    ----------------------------------------------------------------------------
    -- CRC register
    signal crc_q            :     std_logic_vector(G_CRC_WIDTH - 1 downto 0);

    -- Signal if next value of CRC should be shifted and XORed or only shifted!
    signal crc_nxt          :     std_logic;

    -- Shifted value of CRC register. Insert 0 from left
    signal crc_shift        :     std_logic_vector(G_CRC_WIDTH - 1 downto 0);

    -- XORed value 
    signal crc_shift_n_xor  :     std_logic_vector(G_CRC_WIDTH - 1 downto 0);

    -- Combinational value of next CRC value
    signal crc_d            :     std_logic_vector(G_CRC_WIDTH - 1 downto 0);

    -- Clock enable for CRC register
    signal crc_ce           :     std_logic;

begin
   
    ----------------------------------------------------------------------------
    -- Calculation of next CRC value
    ----------------------------------------------------------------------------
    crc_nxt         <= data_in xor crc_q(G_CRC_WIDTH - 1);
  
    crc_shift       <= crc_q(G_CRC_WIDTH - 2 downto 0) & '0';
    
    crc_shift_n_xor <= crc_shift xor G_POLYNOMIAL(G_CRC_WIDTH - 1 downto 0);

    crc_d           <=       init_vect when (load_init_vect = '1') else
                       crc_shift_n_xor when (crc_nxt = '1') else
                            crc_shift;

    crc_ce <= '1' when (load_init_vect = '1') else
              '1' when (enable = '1' and trig = '1') else
              '0';

    ----------------------------------------------------------------------------
    -- Calculation of CRC value 
    ----------------------------------------------------------------------------
    crc_calc_proc : process(res_n, clk_sys)
    begin 
        if (res_n = '0') then
            crc_q             <= (OTHERS => '0');
        elsif rising_edge(clk_sys) then 
            if (crc_ce = '1') then
                crc_q <= crc_d;            
            end if;
        end if;
    end process crc_calc_proc;

    -- Register to output propagation.
    crc <= crc_q;

    -- <RELEASE_OFF>
    ----------------------------------------------------------------------------
    -- Assertions on input settings
    ----------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl no_simul_load_and_calc_asrt : assert never
    --   (enable = '1' and trig = '1' and load_init_vect = '1')
    -- report "Can't load CRC init vector and execute CRC calculation at once!"
    -- severity error;
    
    -- psl crc_load_init_vec_cov : cover
    --  {load_init_vect = '1'};
    
    -- <RELEASE_ON>
end architecture;