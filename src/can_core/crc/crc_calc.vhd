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
--  Generic CRC calculation circuit. Serial Data input. Operation starts with 
--  enable transition from 0 to 1. Valid input data has to be present then.
--  Circuit processes the data on trig signal in logic 1. Circuit operation 
--  finishes when 1 to 0 transition on enable signal appears. The output CRC is
--  valid then. CRC stays valid until following 0 to 1 enable transition. This 
--  also erases CRC registers.
--
--------------------------------------------------------------------------------
-- Revision History:
--   15.11.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CAN_FD_register_map.all;

entity crc_calc is
    generic(

        -- Width of CRC sequence
        constant crc_width      :       natural;

        -- Reset Polarity
        constant reset_polarity :       std_logic := '0';

        -- CRC Polynomial
        constant polynomial     :       std_logic_vector
    );
    port(
        ------------------------------------------------------------------------
        -- Inputs
        ------------------------------------------------------------------------

        -- Asynchronous reset
        signal res_n      :in   std_logic;

        -- System clock input 
        signal clk_sys    :in   std_logic;

        -- Serial data input
        signal data_in    :in   std_logic;

        -- Trigger to sample the input value
        signal trig       :in   std_logic;
 
        -- By transition from 0 to 1 on enable sampled on clk_sys rising edge 
        -- (and with trig='1') operation is started. First bit of data already 
        -- has to be on data_in input.
        -- Circuit works as long as enable=1.
        signal enable     :in   std_logic; 

        -- Initialization vector for CRC calculation
        signal init_vect  :in   std_logic_vector(crc_width - 1 downto 0);

        ------------------------------------------------------------------------
        -- CRC value
        ------------------------------------------------------------------------
        signal crc         :out  std_logic_vector(crc_width - 1 downto 0)
    );    
  
end entity;


architecture rtl of crc_calc is

    ----------------------------------------------------------------------------
    -- Internal registers
    ----------------------------------------------------------------------------
    signal crc_reg          :     std_logic_vector(crc_width - 1 downto 0);
    
    -- Holds previous value of enable input. Detects 0 to 1 transition
    signal start_reg        :     std_logic;

    -- Signal if next value of CRC should be shifted and XORed or only shifted!
    signal crc_nxt          :     std_logic;

    -- Shifted value of CRC register. Insert 0 from left
    signal crc_shift        :     std_logic_vector(crc_width - 1 downto 0);

    -- XORed value 
    signal crc_shift_n_xor  :     std_logic_vector(crc_width - 1 downto 0);

    -- Combinational value of next CRC value
    signal crc_nxt_val      :     std_logic_vector(crc_width - 1 downto 0);


begin
   
    ----------------------------------------------------------------------------
    -- Calculation of next CRC value
    ----------------------------------------------------------------------------
    crc_nxt         <= data_in xor crc_reg(crc_width - 1);
  
    crc_shift       <= crc_reg(crc_width - 2 downto 0) & '0';
    
    crc_shift_n_xor <= crc_shift xor polynomial(crc_width - 1 downto 0);

    crc_nxt_val     <= crc_shift_n_xor when (crc_nxt = '1') 
                                       else
                       crc_shift;

    ----------------------------------------------------------------------------
    -- Registering previous value of enable input to detect 0 to 1 transition
    ----------------------------------------------------------------------------
    start_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = reset_polarity) then
            start_reg       <= '0';
        elsif rising_edge(clk_sys) then
            start_reg       <= enable;
        end if;
    end process start_reg_proc;

    ----------------------------------------------------------------------------
    -- Calculation of CRC value 
    ----------------------------------------------------------------------------
    crc_calc_proc : process(res_n, clk_sys)
    begin 
        if (res_n = reset_polarity) then
            crc_reg             <= (OTHERS => '0');
        elsif rising_edge(clk_sys) then 

            -- Load CRC init vector to CRC register
            if (start_reg = '0' and enable = '1') then
                crc_reg         <= init_vect;
            else

                -- Calculate the next value when triggered
                if (enable = '1' and trig = '1') then
                    crc_reg     <= crc_nxt_val;
                end if;

            end if;
        end if;
    end process crc_calc_proc;

    -- Register to output propagation.
    crc <= crc_reg;

end architecture;
