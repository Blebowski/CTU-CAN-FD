Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;

--Time stamp generator generates  2xtime stamp value from source clock signal 
--(either sys_clock,clk_tq_nbt,clk_tq_dbt,clk_nbt,clk_dbt)

--This component originally intended as internal component of CAN controller!
--Not Used anymore, tsGen used instead on system level.
--Therefore all controllers in the system share the same TimeStamp Generator
--NOTE: DRIVING BUS SIGNALS controlling this circuit were deleted!

entity timeStampGen is
  port(
    ----------
    --INPUTS--
    ----------
    --Source clock signals
    signal clk_sys: in std_logic; --System clock
    signal clk_tq_nbt :in std_logic; --NBT time quantum clock
    signal clk_tq_dbt :in std_logic; --DBT time qunatum clock
    signal clk_nbt :in std_logic; --NBT Bit time clock
    signal clk_dbt :in std_logic; --DBT Bit time clock
    
    signal res_n:in std_logic; --Async reset
    
    --Driving bus from user registers
    signal drv_bus:in std_logic_vector(1023 downto 0);
    
    ----------
    --OUTPUTS-
    ----------
    --TimeStamp counter values
    signal ts_1:out std_logic_vector(49 downto 0);
    signal ts_2:out std_logic_vector(49 downto 0);
    --TimeStamp registers output
    signal ts_reg:out std_logic_vector(199 downto 0)
  );
  
  -------------------
  --INTERNAL SIGNALS- 
  -------------------
  --TimeStamp counter Registers
  signal ts_1_value:std_logic_vector(49 downto 0):=(OTHERS=>'0');
  signal ts_2_value:std_logic_vector(49 downto 0):=(OTHERS=>'0');
  
  --Stamped values registers
  signal ts_A_reg:std_logic_vector(49 downto 0):=(OTHERS=>'0');
  signal ts_B_reg:std_logic_vector(49 downto 0):=(OTHERS=>'0');
  signal ts_C_reg:std_logic_vector(49 downto 0):=(OTHERS=>'0');
  signal ts_D_reg:std_logic_vector(49 downto 0):=(OTHERS=>'0');
   
  --Driving signals aliases
  signal drv_ts_1_src:std_logic_vector(2 downto 0); --Source of ts_1 counter
  signal drv_ts_1_rst:std_logic;  --Reset of counter 1;
  signal drv_ts_1_make:std_logic_vector(3 downto 0); --Log the data of ts_1
  
  signal drv_ts_2_src:std_logic_vector(2 downto 0); --Source of ts_1 counter
  signal drv_ts_2_rst:std_logic;  --Reset of counter 1;
  signal drv_ts_2_make:std_logic_vector(3 downto 0); --Log the data of ts_1
  
  --Registers for previous values of clock inputs
  signal r_clk_tq_nbt :std_logic;
  signal r_clk_tq_dbt :std_logic;
  signal r_clk_nbt :std_logic;
  signal r_clk_dbt :std_logic;
  
end entity;


architecture rtl of timeStampGen is
  begin
  --Outputs
  ts_1<=ts_1_value;
  ts_2<=ts_2_value;
  ts_reg<=ts_D_reg&ts_C_reg&ts_B_reg&ts_A_reg;
  
  --Driving signals aliases
  --Note: If this circuit is to be used following driving signals have to be assigned (or put as input control signals)
  drv_ts_1_src<=(OTHERS=>'0');
  drv_ts_1_rst<='0';
  drv_ts_1_make<=(OTHERS=>'0');

  drv_ts_2_src<=(OTHERS=>'0');
  drv_ts_2_rst<='0';
  drv_ts_2_make<=(OTHERS=>'0');
    
 --CLK_SYS process Async reset, Sync Reset, and counter driving
 clk_sys_proc:process(clk_sys)
 begin
  if(res_n='0')then
    ts_1_value<=(OTHERS=>'0');
    ts_2_value<=(OTHERS=>'0');
    ts_A_reg<=(OTHERS=>'0');
    ts_B_reg<=(OTHERS=>'0');
    ts_C_reg<=(OTHERS=>'0');
    ts_D_reg<=(OTHERS=>'0');
    r_clk_tq_nbt<='0';
    r_clk_tq_dbt<='0';
    r_clk_nbt<='0';
    r_clk_dbt<='0';
  elsif rising_edge(clk_sys)then   
    --Detecting rising edges on other clocks than clk_sys still synchronous to clk_sys
    r_clk_tq_nbt<=clk_tq_nbt;
    r_clk_tq_dbt<=clk_tq_dbt;
    r_clk_nbt<=clk_nbt;
    r_clk_dbt<=r_clk_dbt;
    
    --Counter 1 erase or increment
    if(drv_ts_1_rst='1')then
      ts_1_value<=(OTHERS =>'0');     
    elsif ((drv_ts_1_src="0000") or --clk_sys is source clock for timeStamp 1
          ((drv_ts_1_src="0001") and (clk_tq_nbt='1') and r_clk_tq_nbt='0') or --clk_tq_nbt is source clock for timeStamp 1
          ((drv_ts_1_src="0010") and (clk_tq_dbt='1') and r_clk_tq_dbt='0') or --clk_tq_dbt is source clock for timeStamp 1
          ((drv_ts_1_src="0011") and (clk_nbt='1') and r_clk_nbt='0') or --clk_nbt is source clock for timeStamp 1
          ((drv_ts_1_src="0100") and (clk_dbt='1') and r_clk_dbt='0')) --clk_dbt is source clock for timeStamp 1
          then       
      ts_1_value<=std_logic_vector(unsigned(ts_1_value)+1);
    else 
      ts_1_value<=ts_1_value;
    end if; 
    
     --Counter 2 erase or increment
    if(drv_ts_2_rst='1')then
      ts_2_value<=(OTHERS =>'0');     
    elsif ((drv_ts_2_src="0000") or --clk_sys is source clock for timeStamp 2
          ((drv_ts_2_src="0001") and (clk_tq_nbt='1') and r_clk_tq_nbt='0') or --clk_tq_nbt is source clock for timeStamp 2
          ((drv_ts_2_src="0010") and (clk_tq_dbt='1') and r_clk_tq_dbt='0') or --clk_tq_dbt is source clock for timeStamp 2
          ((drv_ts_2_src="0011") and (clk_nbt='1') and r_clk_nbt='0') or --clk_nbt is source clock for timeStamp 2
          ((drv_ts_2_src="0100") and (clk_dbt='1') and r_clk_dbt='0')) --clk_dbt is source clock for timeStamp 2
     then       
      ts_2_value<=std_logic_vector(unsigned(ts_2_value)+1);
    else 
      ts_2_value<=ts_2_value;
    end if; 
     
    --Saving time from timeStamps into registers
    if(drv_ts_1_make(0)='1')then
      ts_A_reg<=ts_1_value;
    elsif (drv_ts_2_make(0)='1') then
      ts_A_reg<=ts_2_value;
    else
      ts_A_reg<=ts_A_reg;
    end if;
    
    if(drv_ts_1_make(1)='1')then
      ts_B_reg<=ts_1_value;
    elsif (drv_ts_2_make(1)='1') then
      ts_B_reg<=ts_2_value;
    else
      ts_B_reg<=ts_B_reg;
    end if;
    
    if(drv_ts_1_make(2)='1')then
      ts_C_reg<=ts_1_value;
    elsif (drv_ts_2_make(2)='1') then
      ts_C_reg<=ts_2_value;
    else
      ts_C_reg<=ts_C_reg;
    end if;
    
    if(drv_ts_1_make(3)='1')then
      ts_D_reg<=ts_1_value;
    elsif (drv_ts_2_make(3)='1') then
      ts_D_reg<=ts_2_value;
    else
      ts_D_reg<=ts_D_reg;
    end if;    
        
  end if; 
 end process clk_sys_proc;
    
end architecture;