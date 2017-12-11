Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANcomponents.ALL;
USE WORK.CANconstants.ALL;

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
--  July 2015  Original version
-------------------------------------------------------------------------------------------------------------
entity prescaler_v3_tb is
end entity;

architecture behav of prescaler_v3_tb is
  
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
    
    signal data_tx: std_logic := '0';
    
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
    data_tx=>data_tx,
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
