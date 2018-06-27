Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

-----------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
--
--    30.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Feature test for setting error counters from user and its appropriate fault
--  confinement state manipulation!
--                                      
-----------------------------------------------------------------------------------------------------------------


package fault_conf_feature is
  
  procedure fault_conf_feature_exec(
    variable   outcome      : inout boolean;
    signal      rand_ctr        :inout  natural range 0 to RAND_POOL_SIZE;
    signal      mem_bus_1       :inout  Avalon_mem_type;
    signal      mem_bus_2       :inout  Avalon_mem_type;
    --Additional signals for tests
    --Pretty much everything can be read out of stat bus...
    signal      bus_level       :in     std_logic;
    signal      drv_bus_1       :in     std_logic_vector(1023 downto 0);
    signal      drv_bus_2       :in     std_logic_vector(1023 downto 0);
    signal      stat_bus_1      :in     std_logic_vector(511 downto 0);
    signal      stat_bus_2      :in     std_logic_vector(511 downto 0) 
  );
  
end package;


package body fault_conf_feature is
  
   procedure fault_conf_feature_exec(
    variable   outcome      : inout boolean;
    signal      rand_ctr        :inout  natural range 0 to RAND_POOL_SIZE;
    signal      mem_bus_1       :inout  Avalon_mem_type;
    signal      mem_bus_2       :inout  Avalon_mem_type;
    --Additional signals for tests
    --Pretty much everything can be read out of stat bus...
    signal      bus_level       :in     std_logic;
    signal      drv_bus_1       :in     std_logic_vector(1023 downto 0);
    signal      drv_bus_2       :in     std_logic_vector(1023 downto 0);
    signal      stat_bus_1      :in     std_logic_vector(511 downto 0);
    signal      stat_bus_2      :in     std_logic_vector(511 downto 0) 
  )is
  variable r_data               :     std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable CAN_frame            :     SW_CAN_frame_type;
  variable frame_sent           :     boolean:=false;
  variable ctr_1                :     natural;
  variable ctr_2                :     natural;
  variable ID_1           	     :     natural:=1;
  variable ID_2           	     :     natural:=2;
  variable rand_val             :     real;
  variable th_1                 :     natural:=0;
  variable rxc                  :     natural:=0;
  variable txc                  :     natural:=0;
  begin
    outcome:=true;
    
    -----------------------------------------------
    -- Generate random setting of ERP treshold
    -- and RX counters to preset
    -----------------------------------------------
    rand_real_v(rand_ctr,rand_val);
    th_1:=integer(rand_val*254.0);
    
    rand_real_v(rand_ctr,rand_val);
    rxc:=integer(rand_val*257.0);
    
    rand_real_v(rand_ctr,rand_val);
    txc:=integer(rand_val*257.0);
    
    
    -----------------------------------------------
    -- Set the counter and tresholds
    -----------------------------------------------
    r_data := (OTHERS => '0');
    r_data(15 downto 8):= std_logic_vector(to_unsigned(th_1,8));
    CAN_write(r_data,ERROR_TH_ADR,ID_1,mem_bus_1);
    
    r_data := (OTHERS => '0');
    r_data(8 downto 0):= std_logic_vector(to_unsigned(txc,9));
    r_data(9):='1';
    CAN_write(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
    
    r_data := (OTHERS => '0');
    r_data(8 downto 0):= std_logic_vector(to_unsigned(rxc,9));
    r_data(10):='1';
    CAN_write(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
    
    
    -----------------------------------------------
    -- Read counters back
    -----------------------------------------------
    CAN_read(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
    
   if( to_integer(unsigned(r_data(8 downto 0))) /= rxc )then
      outcome:=false;
    end if;
    
    if( to_integer(unsigned(r_data(24 downto 16))) /= txc )then
      outcome:=false;
    end if;
    
    -----------------------------------------------
    -- Read fault confinement state
    -----------------------------------------------
    CAN_read(r_data,ERROR_TH_ADR,ID_1,mem_bus_1);
    
    if(txc>255 or rxc>255)then
      if(r_data(16)='1' or r_data(17)='1' or r_data(18)='0')then
        outcome:=false;
      end if;
    elsif(txc<th_1 and rxc<th_1) then
      if(r_data(16)='0' or r_data(17)='1' or r_data(18)='1')then
        outcome:=false;
      end if;
    else
      if(r_data(16)='1' or r_data(17)='0' or r_data(18)='1')then
        outcome:=false;
      end if;
      
    end if;
    
    
  end procedure;
  
end package body;
