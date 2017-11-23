Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
USE WORK.CANconstants.ALL;

--Author: Ille Ondrej
--CAN FD IP function project 2015

entity prescaler_v3_tb is
end entity;

architecture behav of prescaler_v3_tb is
  
  component prescaler_v3 is
  PORT(
    signal clk_sys:in std_logic;  --System clock
    signal res_n:in std_logic;   --Async reset
    
    signal sync_edge:in std_logic; --Edge for synchronisation
    signal OP_State:in oper_mode_type; --Protocol control state
    
    signal drv_bus:in std_logic_vector(1023 downto 0); 
    
    signal clk_tq_nbt:out std_logic; --Time quantum clock - Nominal bit time
    signal clk_tq_dbt:out std_logic; --bit time - Nominal bit time
    
    signal sample_nbt:out std_logic; --Sample signal for nominal bit time
    signal sample_dbt:out std_logic; --Sample signal of data bit time
    signal sample_nbt_del_1:out std_logic;
    signal sample_dbt_del_1:out std_logic;
    signal sample_nbt_del_2:out std_logic;
    signal sample_dbt_del_2:out std_logic;
    signal bt_FSM_out:out bit_time_type;
    
    signal sync_nbt:out std_logic;
    signal sync_dbt:out std_logic;
    signal sync_nbt_del_1:out std_logic;
    signal sync_dbt_del_1:out std_logic;
    
    signal sp_control:in std_logic_vector(1 downto 0);
    signal sync_control:in std_logic_vector(1 downto 0)
  );
  end component;
  
    signal clk_sys: std_logic;  --System clock
    signal res_n: std_logic;   --Async reset
    
    signal sync_edge: std_logic; --Edge for synchronisation
    signal OP_State: oper_mode_type; --Protocol control state
    
    signal drv_bus: std_logic_vector(1023 downto 0); 
    
    signal clk_tq_nbt: std_logic; --Time quantum clock - Nominal bit time
    signal clk_tq_dbt: std_logic; --bit time - Nominal bit time
    
    signal sample_nbt: std_logic; --Sample signal for nominal bit time
    signal sample_dbt: std_logic; --Sample signal of data bit time
    signal sample_nbt_del_1: std_logic;
    signal sample_dbt_del_1: std_logic;
    signal sample_nbt_del_2: std_logic;
    signal sample_dbt_del_2: std_logic;
    signal bt_FSM_out: bit_time_type;
    
    signal sync_nbt: std_logic;
    signal sync_dbt: std_logic;
    signal sync_nbt_del_1: std_logic;
    signal sync_dbt_del_1: std_logic;
    
    signal sp_control: std_logic_vector(1 downto 0);
    signal sync_control: std_logic_vector(1 downto 0);

begin
  comp:prescaler_v3
  port map(
    clk_sys=>clk_sys,
    res_n=>res_n,
    sync_edge=>sync_edge,
    drv_bus=>drv_bus,
    clk_tq_nbt=>clk_tq_nbt,
    clk_tq_dbt=>clk_tq_dbt,
    sample_nbt=>sample_nbt,
    sample_dbt=>sample_dbt,
    sample_nbt_del_1=>sample_nbt_del_1,
    sample_nbt_del_2=>sample_nbt_del_2,
    sample_dbt_del_1=>sample_dbt_del_1,
    sample_dbt_del_2=>sample_dbt_del_2,
    sync_nbt=>sync_nbt,
    sync_dbt=>sync_dbt,
    bt_FSM_out=>bt_FSM_out,
    sync_nbt_del_1=>sync_nbt_del_1,
    sync_dbt_del_1=>sync_dbt_del_1,
    sp_control=>sp_control,
    sync_control=>sync_control,
    OP_State=>OP_State
    );
  
  clk_gen:process
  begin
   clk_sys<='1';
   wait for 5 ns;
   clk_sys<='0';
   wait for 5 ns;
  end process;
  
  test1:process
  variable random_pointer:natural:=0;
  begin
    res_n<=ACT_RESET;
    drv_bus<=(OTHERS=>'0');
    sp_control<=NOMINAL_SAMPLE;
    sync_control<=NO_SYNC;
    OP_State<=reciever;
    sync_edge<='0';
    wait for 37 ns;
    res_n<=not ACT_RESET;
    random_pointer:=1;
    
    -------------------------
    --Time segments setting--
    -------------------------
    drv_bus(DRV_TQ_NBT_HIGH downto DRV_TQ_NBT_LOW)<="000100"; --4 clock cycles per time quantum
    drv_bus(DRV_TQ_DBT_HIGH downto DRV_TQ_DBT_LOW)<="000010"; --2 clock cycles per time quantum
    --Nominal bit time -- 10 Time quanta
    drv_bus(DRV_PRS_NBT_HIGH downto DRV_PRS_NBT_LOW)<="000100";
    drv_bus(DRV_PH1_NBT_HIGH downto DRV_PH1_NBT_LOW)<="000010";
    drv_bus(DRV_PH2_NBT_HIGH downto DRV_PH2_NBT_LOW)<="000011";
    --Data bit time
    drv_bus(DRV_PRS_DBT_HIGH downto DRV_PRS_DBT_LOW)<="0011";
    drv_bus(DRV_PH1_DBT_HIGH downto DRV_PH1_DBT_LOW)<="0010";
    drv_bus(DRV_PH2_DBT_HIGH downto DRV_PH2_DBT_LOW)<="0010";
    --Synchronisation jump width
    drv_bus(DRV_SJW_HIGH downto DRV_SJW_LOW)<="0011";
    
    assert false report "Nominal bit time clock generation!" severity note;
    wait for 1500 ns;
    sp_control<=DATA_SAMPLE;
    assert false report "Data bit time clock generation" severity note;
    wait for 250 ns;
    sp_control<=SECONDARY_SAMPLE;
    assert false report "Secondary sample point time clock generation (the same as data bit time)" severity note;
    wait for 250 ns;
    
    wait for 1 us;
    
    report "Hard synchronisation test";
    sync_control<=HARD_SYNC;
     sp_control<=NOMINAL_SAMPLE;
    
    --Random snchronisation signals
    for I in 1 to 100 loop
      wait for randData(random_pointer)*1000.0  ns;
      sync_edge<='1';
      wait for 10 ns;
      sync_edge<='0';
      random_pointer:=random_pointer+1;
    end loop;
    
    
    
    wait for 5000 ns;
    
  end process;
end architecture;
