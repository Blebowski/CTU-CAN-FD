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
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Purpose:
--  Asynchronouse reset synchroniser to avoid problems with Reset recovery time.
--------------------------------------------------------------------------------
-- Revision History:
--    27.11.2017   Created file
--
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

entity rst_sync is
  port (
      signal clk    : in  std_logic;
      signal arst_n : in  std_logic;
      signal rst_n  : out std_logic
  );
end rst_sync;

architecture rtl of rst_sync is
  signal rff : std_logic; 	
begin
 process (clk, arst_n)
 begin
  if (arst_n = '0') then
    rff <= '0';
    rst_n <= '0';
  elsif (rising_edge(clk)) then
    rff <= '1';
    rst_n <= rff;
  end if;
 end process;
end rtl;
