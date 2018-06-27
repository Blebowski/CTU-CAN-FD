Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;

-------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
--
--    17.1.2016   Created file
-------------------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Package for converting between Register format of CAN Identifier and decimal format of Identifier.
--  Needed by TX arbitrator and message filter when filtering data based on identifier decimal value.
--  When acessing CAN Controller from software driver should take care of this conversion!
---------------------------------------------------------------------------------------------------------------

package ID_transfer is

--Register value to decimal value
procedure ID_reg_to_decimal
      (signal ID_reg:in std_logic_vector(28 downto 0);
       signal ID_dec:out natural);
       
--Decimal value to register value
procedure ID_decimal_to_reg
      (signal ID_dec:in natural;
       signal ID_reg:out std_logic_vector(28 downto 0));
   
end package ID_transfer;

package body ID_transfer          is
  procedure ID_reg_to_decimal
      (signal ID_reg:in std_logic_vector(28 downto 0);
       signal ID_dec:out natural) is
    variable base:std_logic_vector(10 downto 0);
    variable ext:std_logic_vector(17 downto 0);
    variable conc:std_logic_vector(28 downto 0);
  begin
     base   :=  ID_reg(10 downto 0);
     ext    :=  ID_reg(28 downto 11);
     conc   :=  base&ext;
     ID_dec <=  to_integer(unsigned(conc));
  end procedure ID_reg_to_decimal;

procedure ID_decimal_to_reg
      (signal ID_dec:in natural;
       signal ID_reg:out std_logic_vector(28 downto 0)) is
    variable vector:std_logic_vector(28 downto 0);
  begin
    vector  :=  std_logic_vector(to_unsigned(ID_dec,29));
    ID_reg  <=  vector(18 downto 0)&vector(28 downto 19);
  end procedure ID_decimal_to_reg;
   
end ID_transfer;
