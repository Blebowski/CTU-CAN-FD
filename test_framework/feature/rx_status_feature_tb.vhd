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
--    21.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  RX Buffer status feature test implementation.
-- 
--  Test sequence is like so:
--    1. RX Buffer size is read and buffer is cleared,
--    2. Free memory, buffer status and message count is checked
--    3. Random frames are sent on the bus by node 2 and recieved by node 1
--    4. After each frame amount of remaining memory is checked towards expected value
--    5. When buffer is filled Data overrun flag is checked and cleared
--    6. After clearing Overrun flag, it is checked it was really cleared
--
-----------------------------------------------------------------------------------------------------------------


package rx_status_feature is
  
  procedure rx_status_feature_exec(
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


package body rx_status_feature is
  
  procedure rx_status_feature_exec(
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
  variable r_data               :       std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable w_data               :       std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable size_of_buf          :       natural;
  variable ID_1           	     :     natural:=1;
  variable ID_2           	     :     natural:=2;
  variable CAN_frame            :     SW_CAN_frame_type;
  variable send_more            :     boolean:=true;
  variable in_RX_buf            :     natural range 0 to 1023;
  variable frame_sent           :     boolean:=false;
  variable number_frms_sent     :     natural range 0 to 1023;
  variable aux2                 :     natural range 0 to 1023;
  variable aux                  :     std_logic_vector(7 downto 0):=(OTHERS => '0');
  begin
    outcome:= true;
    
    -------------------------------------------
    -- Read the size of the synthesized buffer
    -------------------------------------------
    CAN_read(r_data,RX_INFO_2_ADR,ID_1,mem_bus_1);
    size_of_buf:= to_integer(unsigned(r_data(7 downto 0)));
    
    -------------------------------------------
    --Restart the content of the buffer...
    -------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data(10) := '1';  --Release recieve buffer bit
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    -------------------------------------------
    --Check that buffer is empty
    --Check that mem free is equal to buff size
    --Check that both pointers are 0 as well
    -- as message count
    -------------------------------------------
    CAN_read(r_data,RX_INFO_1_ADR,ID_1,mem_bus_1);
    if(r_data(1 downto 0) /= "01")then
      outcome:= false;
    end if;
    
    if(to_integer(unsigned(r_data(23 downto 16))) /= size_of_buf)then
      outcome:= false;
    end if;
    
    if(to_integer(unsigned(r_data(15 downto 8))) /= 0)then
      outcome:= false;
    end if;
    
    CAN_read(r_data,RX_INFO_2_ADR,ID_1,mem_bus_1);
    if(r_data(23 downto 8) /= "0000000000000000")then
      outcome:= false;
    end if;
    
    --------------------------------------------
    -- Generate the CAN frames to send
    --------------------------------------------
    while send_more=true loop
      CAN_generate_frame(rand_ctr,CAN_frame);
      
      
      if(CAN_frame.rtr='1' and CAN_frame.frame_format='0')then
        if(in_RX_buf+4 > size_of_buf)then
          send_more:= false;
        end if;
      else
         if(CAN_frame.data_length mod 4 = 0)then
            if(in_RX_buf+CAN_frame.data_length/4+4 > size_of_buf)then
              send_more:= false;
            end if;
          else
            if(in_RX_buf+CAN_frame.data_length/4+5 > size_of_buf)then
              send_more:= false;
            end if;
          end if;
      end if;
      --TODO:change to arbitrary BRS
      --CAN_frame.brs:='0';
      
      CAN_send_frame(CAN_frame,1,ID_2,mem_bus_2,frame_sent);
      CAN_wait_frame_sent(ID_1,mem_bus_1);
      number_frms_sent:=number_frms_sent+1;
      if((CAN_frame.rtr='1' and CAN_frame.frame_format='0') or CAN_frame.data_length=0)then
        in_RX_buf:=in_RX_buf+4;
      else
        
        if(CAN_frame.data_length mod 4 = 0)then
          in_RX_buf:=in_RX_buf+(CAN_frame.data_length/4)+4;
        else
          in_RX_buf:=in_RX_buf+(CAN_frame.data_length/4+1)+4;
        end if;
        
      end if;
      
      
      --Wait until frame is for sure stored in the RX Buffer
      for i in 0 to 19 loop
        wait until rising_edge(mem_bus_1.clk_sys);
      end loop;
      
      --Check that mc was incremented and memfree is according
      CAN_read(r_data,RX_INFO_1_ADR,ID_1,mem_bus_1);
      aux:=r_data(15 downto 8);
      aux2:=to_integer(unsigned(aux));
      if(number_frms_sent /= aux2 and send_more=true)then
        outcome:= false;
      end if;   
      if(to_integer(unsigned(r_data(23 downto 16)))+in_RX_buf /= size_of_buf and send_more=true)then
        outcome:= false;
      end if; 
        
    end loop;
    
    --------------------------------------------
    -- Here we should check the Data overrun
    --  status!
    -- Now data oveerrun flag should be set
    --------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    if(r_data(17)='0')then
      outcome:=false;
    end if;
     
    -------------------------------------------
    -- Clear the data overrun flag
    -------------------------------------------
    r_data(11):='1';
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    --------------------------------------------
    -- Check that overrun flag was cleared
    --------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    if(r_data(17)='1')then
      outcome:=false;
    end if;
    
    
  end procedure;
  
end package body;