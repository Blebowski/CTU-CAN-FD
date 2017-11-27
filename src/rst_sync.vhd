Library ieee;
use ieee.std_logic_1164.all;

-------------------------------------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- This program is free software; you can redistribute it and/or
-- modify it under the terms of the GNU General Public License
-- as published by the Free Software Foundation; either version 2
-- of the License, or (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- The CAN protocol is developed by Robert Bosch GmbH and     
-- protected by patents. Anybody who wants to implement this    
-- IP core on silicon has to obtain a CAN protocol license
-- from Bosch.
--
--
-- Revision History:
--
--    27.11.2017   Created file
--
-------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Asynchronouse reset synchroniser to avoid problems with Reset recovery time.
-------------------------------------------------------------------------------------------------------------

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
