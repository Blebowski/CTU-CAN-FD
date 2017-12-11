Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;

--------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy 
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is 
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in 
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
-- IN THE SOFTWARE.
--
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN 
-- protocol license from Bosch.
--
-- Revision History:
--
--    17.1.2016   Created file
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Purpose:
--  Package for converting between Register format of CAN Identifier and decimal
--  format of Identifier. Needed by TX arbitrator and message filter when fil-
--  tering data based on identifier decimal value. When acessing CAN Controller
--  from software driver should take care of this conversion!
--------------------------------------------------------------------------------

package ID_transfer is

  --Register value to decimal value
  procedure ID_reg_to_decimal
        (signal ID_reg:in std_logic_vector(28 downto 0);
         signal ID_dec : out natural);

  --Decimal value to register value
  procedure ID_decimal_to_reg
    (signal ID_dec : in  natural;
     signal ID_reg : out std_logic_vector(28 downto 0));

end package ID_transfer;

package body ID_transfer is
  procedure ID_reg_to_decimal
    (signal ID_reg : in  std_logic_vector(28 downto 0);
     signal ID_dec : out natural) is
    variable base : std_logic_vector(10 downto 0);
    variable ext  : std_logic_vector(17 downto 0);
    variable conc : std_logic_vector(28 downto 0);
  begin
    base   := ID_reg(10 downto 0);
    ext    := ID_reg(28 downto 11);
    conc   := base&ext;
    ID_dec <= to_integer(unsigned(conc));
  end procedure ID_reg_to_decimal;

  procedure ID_decimal_to_reg
    (signal ID_dec : in  natural;
     signal ID_reg : out std_logic_vector(28 downto 0)) is
    variable vector : std_logic_vector(28 downto 0);
  begin
    vector := std_logic_vector(to_unsigned(ID_dec, 29));
    ID_reg <= vector(18 downto 0)&vector(28 downto 19);
  end procedure ID_decimal_to_reg;

end ID_transfer;
