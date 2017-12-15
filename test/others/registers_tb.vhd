--------------------------------------------------------------------------------
-- 
-- CAN with Flexible Data-Rate IP Core 
-- 
-- Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisor: Jiri Novak <jnovak@fel.cvut.cz>
-- Department of Measurement         (http://meas.fel.cvut.cz/)
-- Faculty of Electrical Engineering (http://www.fel.cvut.cz)
-- Czech Technical University        (http://www.cvut.cz/)
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy 
-- of this VHDL component and associated documentation files (the "Component"), 
-- to deal in the Component without restriction, including without limitation 
-- the rights to use, copy, modify, merge, publish, distribute, sublicense, 
-- and/or sell copies of the Component, and to permit persons to whom the 
-- Component is furnished to do so, subject to the following conditions:
-- 
-- The above copyright notice and this permission notice shall be included in 
-- all copies or substantial portions of the Component.
-- 
-- THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS 
-- IN THE COMPONENT.
-- 
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN 
-- protocol license from Bosch.
-- 
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Revision History:
--
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;

entity registers_tb is
end entity;

architecture behav of registers_tb is
  component registers is
  generic(
    constant compType:std_logic_vector(3 downto 0):= CAN_COMPONENT_TYPE;
    constant ID:natural:=1 --ID of the component
  );
  port(
    --Clock and asynchronous reset
    signal clk_sys:in std_logic;
    signal res_n:in std_logic;
    signal res_out:out std_logic;
    
    --------------------
    --Memory Interface--
    --------------------
    signal data_in:in std_logic_vector(31 downto 0);
    signal data_out:out std_logic_vector(31 downto 0);
    signal adress:in std_logic_vector(23 downto 0);
    signal scs:in std_logic;
    signal srd:in std_logic;
    signal swr:in std_logic;
    --Note: Memory interface is avalon compatible!
    --Note: In 32-bit system, every register is at 0x4 higher adress. At FPGA the lowest two bits of adress are considered
    --to be cut, therefore +0x04 means one bit higher adress!!
      
    --Driving and Status Bus
    signal drv_bus:out std_logic_vector(1023 downto 0);
    signal stat_bus:in std_logic_vector(511 downto 0);
    
    -----------------------
    --RX Buffer Interface--
    -----------------------
    signal rx_read_buff:in std_logic_vector(31 downto 0); --Actually loaded data for reading
    signal rx_buf_size:in std_logic_vector(7 downto 0); --Actual size of synthetised message buffer (in 32 bit words)
    signal rx_full: in std_logic; --Signal whenever buffer is full
    signal rx_empty: in std_logic; --Signal whenever buffer is empty
    signal rx_message_count: in std_logic_vector(7 downto 0); --Number of messaged stored in recieve buffer
    signal rx_mem_free:in std_logic_vector(7 downto 0); --Number of free 32 bit wide ''windows''
    signal rx_read_pointer_pos:in std_logic_vector(7 downto 0); --Position of read pointer
    signal rx_write_pointer_pos:in std_logic_vector(7 downto 0); --Position of write pointer
    signal rx_message_disc: in std_logic; --Message was discarded since Memory is full
    signal rx_data_overrun: in std_logic; --Some data were discarded, register
    
    -----------------------
    --TX Buffer Interface--
    -----------------------
    signal tran_data_in:out std_logic_vector(639 downto 0);  --Transcieve data (Common for TX Buffer and TXT Buffer)
    
    signal tx_buff_size:in std_logic_vector(7 downto 0); --Size of transcieve Buffer in 32 bit words
    signal tx_full: in std_logic; --Trascieve buffer is full
    signal tx_message_count :in std_logic_vector(7 downto 0); --Number of messages in the TX buffer
    signal tx_empty:in std_logic; --Buffer empty;
    signal tx_mem_free:in std_logic_vector (7 downto 0); --Number of free words in TX counter
    signal tx_read_pointer_pos: in std_logic_vector(7 downto 0); --Read pointer value propagated
    signal tx_write_pointer_pos: in std_logic_vector(7 downto 0); --Write pointer value propagated
    signal tx_message_disc:in std_logic; --Signal that acutal message was discarded and not stored for transcieving
    
    ------------------------
    --TXT Buffer Interface--
    ------------------------
    signal txt_empty:in std_logic; --Logic 1 signals empty TxTime buffer
    signal txt_disc:in std_logic; --Info that message store into buffer from driving registers failed because buffer is full
    
    
    signal trv_delay_out: in std_logic_vector(15 downto 0);
    
    ------------------------
    --Interrrupt Interface--
    ------------------------
    signal int_vector:in std_logic_vector(10 downto 0) --Interrupt vector (Interrupt register of SJA1000)
  );
  
  end component;
  
    signal clk_sys: std_logic;
    signal res_n: std_logic;
    signal res_out: std_logic;
    signal data_in: std_logic_vector(31 downto 0);
    signal data_out: std_logic_vector(31 downto 0);
    signal adress: std_logic_vector(23 downto 0);
    signal scs: std_logic;
    signal srd: std_logic;
    signal swr: std_logic;
    --Driving and Status Bus
    signal drv_bus: std_logic_vector(1023 downto 0);
    signal stat_bus: std_logic_vector(511 downto 0);
    
    signal rx_read_buff: std_logic_vector(31 downto 0); --Actually loaded data for reading
    signal rx_buf_size: std_logic_vector(7 downto 0); --Actual size of synthetised message buffer (in 32 bit words)
    signal rx_full:  std_logic; --Signal whenever buffer is full
    signal rx_empty:  std_logic; --Signal whenever buffer is empty
    signal rx_message_count:  std_logic_vector(7 downto 0); --Number of messaged stored in recieve buffer
    signal rx_mem_free: std_logic_vector(7 downto 0); --Number of free 32 bit wide ''windows''
    signal rx_read_pointer_pos: std_logic_vector(7 downto 0); --Position of read pointer
    signal rx_write_pointer_pos: std_logic_vector(7 downto 0); --Position of write pointer
    signal rx_message_disc:  std_logic; --Message was discarded since Memory is full
    signal rx_data_overrun:  std_logic; --Some data were discarded, register
    
    signal tran_data_in:std_logic_vector(639 downto 0);  --Transcieve data (Common for TX Buffer and TXT Buffer)
    
    signal tx_buff_size: std_logic_vector(7 downto 0); --Size of transcieve Buffer in 32 bit words
    signal tx_full:  std_logic; --Trascieve buffer is full
    signal tx_message_count : std_logic_vector(7 downto 0); --Number of messages in the TX buffer
    signal tx_empty: std_logic; --Buffer empty;
    signal tx_mem_free: std_logic_vector (7 downto 0); --Number of free words in TX counter
    signal tx_read_pointer_pos:  std_logic_vector(7 downto 0); --Read pointer value propagated
    signal tx_write_pointer_pos:  std_logic_vector(7 downto 0); --Write pointer value propagated
    signal tx_message_disc: std_logic; --Signal that acutal message was discarded and not stored for transcieving
    
    signal txt_empty: std_logic; --Logic 1 signals empty TxTime buffer
    signal txt_disc: std_logic; --Info that message store into buffer from driving registers failed because buffer is full
    
    signal int_vector: std_logic_vector(10 downto 0); --Interrupt vector (Interrupt register of SJA1000)
    
    signal trv_delay_out:  std_logic_vector(15 downto 0);
    
     constant compType:std_logic_vector(3 downto 0):= CAN_COMPONENT_TYPE;
     constant ID:natural:=1; --ID of the component
     
       -----------------------
        --Testbench constants--
        -----------------------
        constant clk_per:time:=10 ns;
 
begin
  reg_comp:registers
  generic map(
     compType=>compType,
     ID=>ID
  )
  port map(
     clk_sys=>clk_sys,
     res_n=>res_n,
     res_out=>res_out,
     data_in=>data_in,
     data_out=>data_out,
     adress=>adress,
     scs=>scs,
     srd=>srd,
     swr=>swr,
     drv_bus=>drv_bus,
     stat_bus=>stat_bus,
     rx_read_buff=>rx_read_buff,
     rx_buf_size=>rx_buf_size,
     rx_full=>rx_full,
     rx_empty=>rx_empty,
     rx_message_count=>rx_message_count,
     rx_mem_free=>rx_mem_free,
     rx_read_pointer_pos=>rx_read_pointer_pos,
     rx_write_pointer_pos=>rx_write_pointer_pos,
     rx_message_disc=>rx_message_disc,
     rx_data_overrun=>rx_data_overrun,
     tran_data_in =>tran_data_in,
     tx_buff_size=>tx_buff_size,
     tx_full=>tx_full,
     tx_message_count =>tx_message_count,
     tx_empty=>tx_empty,
     tx_mem_free=>tx_mem_free,
     tx_read_pointer_pos=>tx_read_pointer_pos,
     tx_write_pointer_pos=>tx_write_pointer_pos,
     tx_message_disc=>tx_message_disc,
     txt_empty=>txt_empty,
     trv_delay_out=>trv_delay_out,
     txt_disc=>txt_disc,
     int_vector=>int_vector
  );
  
   clock_gen:process
    begin
     clk_sys<='1';
     wait for clk_per;
     clk_sys<='0';
     wait for clk_per;
    end process;
  
  test_proc:process
  begin
  res_n<=ACT_reset;
  data_in<=(OTHERS=>'0');
  scs<='0';
  swr<='0';
  srd<='0';
  adress<=(OTHERS=>'0');
  stat_bus<=(OTHERS=>'0');
  
  --Setting information about RX Buffer
   rx_read_buff<=(OTHERS=>'0');
   rx_buf_size<=(OTHERS=>'0');
   rx_full<='0';
   rx_empty<='1';
   rx_message_count<=(OTHERS=>'0');
   rx_mem_free<=(OTHERS=>'0');
   rx_read_pointer_pos<=(OTHERS=>'0');
   rx_write_pointer_pos<=(OTHERS=>'0');
   rx_message_disc<='0';
   rx_data_overrun<='0';
  
  
--Setting info about TX Buffer
  
  int_vector<=(OTHERS=>'0');
  tran_data_in<=(OTHERS=>'0');
     tx_buff_size<=(OTHERS=>'0');
     tx_full<='0';
     tx_message_count<=(OTHERS=>'0');
     tx_empty<='0';
     tx_mem_free<=(OTHERS=>'0');
     tx_read_pointer_pos<=(OTHERS=>'0');
     tx_write_pointer_pos<=(OTHERS=>'0');
     tx_message_disc<='0';
     txt_empty<='0';
     txt_disc<='0';
     int_vector<=(OTHERS=>'0');
  
  wait for 17 ns;
  res_n<=not ACT_reset;
  
  adress<=TO_STDLOGICVECTOR(X"410000");
  scs<='1';
  srd<='1';
  wait for 20 ns;
  
  
  wait for 10 us;
  end process;  
  
  
end architecture;
