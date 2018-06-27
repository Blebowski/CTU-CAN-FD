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
--  Feature test for retransmitt limitation
--                                      
-----------------------------------------------------------------------------------------------------------------


package retr_limit_feature is
  
  procedure retr_limit_feature_exec(
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


package body retr_limit_feature is
  
   procedure retr_limit_feature_exec(
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
  variable retr_th              :     natural;
  variable mode_backup          :     std_logic_vector(31 downto 0):=(OTHERS => '0');
  begin
    outcome:=true;
    
    -----------------------------------------------
    --Set node  2 to forbid acknowledge
    -----------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data(7) := '1';
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    
    -----------------------------------------------
    -- Erase error counters node 1
    -----------------------------------------------    
    r_data :=(OTHERS => '0');
    r_data(9):='1';
    CAN_write(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);    
        
        
    -----------------------------------------------
    --Set node 1 retransmitt limit
    ----------------------------------------------- 
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    mode_backup:=r_data;   
    r_data(24):='1';
    rand_real_v(rand_ctr,rand_val);
    retr_th:=integer(15.0*rand_val);
    r_data(28 downto 25):= std_logic_vector(to_unsigned(retr_th,4));
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
           
    CAN_generate_frame(rand_ctr,CAN_frame);     
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent); 
    
    for i in 0 to retr_th loop
      CAN_wait_frame_sent(ID_1,mem_bus_1);
   end loop;
     
    -----------------------------------------------
    --Read TX Counter, it should be equal to 
    -- 8 times number of retransmitts plus one
    -- original transmittion does not count
    -- as retransmittion
    -----------------------------------------------  
     CAN_read(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
     
     if(to_integer(unsigned(r_data(31 downto 16))) /= 8*(retr_th+1))then       
       outcome:=false;
     end if;
     
     
    -----------------------------------------------
    --Set node  2 to allow acknowledge again
    -----------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data(7) := '0';
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);  
    CAN_write(mode_backup,MODE_REG_ADR,ID_1,mem_bus_1); 
    
  end procedure;
  
end package body;
