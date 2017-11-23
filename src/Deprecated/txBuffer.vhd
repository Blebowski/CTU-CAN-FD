Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;

--Comment: TODO

entity txBuffer is 
  GENERIC (
    buff_size:natural:=32 --Only powers of 2 allowed as buff_size!! (32,64,128,256)
  );
  PORT(
    -------------------
    --Clock and Reset--
    -------------------
    signal clk_sys:in std_logic; --System clock
    signal res_n:in std_logic; --Async Reset
    
    -------------------------------
    --Driving registers inferface--
    -------------------------------
    signal drv_bus:in std_logic_vector(1023 downto 0);  --Driving bus from registers
    signal tran_data_in:in std_logic_vector(639 downto 0);  --Transcieve data    

    ---------------------------
    --TX Arbitrator interface--
    ---------------------------
    signal tx_buffer_out: out std_logic_vector(639 downto 0); --Output frame with TX buffer structure
    signal tx_buffer_valid:out std_logic; --Valid data on the output of TX Buffer
    signal tx_buffer_ack:in std_logic; --Acknowledge from TX Arbitrator that message was loaded into CAN Core and can be erased
    
    -------------------
    --Status signals --
    -------------------
    signal tx_buff_size:out std_logic_vector(7 downto 0); --Size of transcieve Buffer in 32 bit words
    signal tx_full: out std_logic; --Trascieve buffer is full
    signal tx_message_count :out std_logic_vector(7 downto 0); --Number of messages in the TX buffer
    signal tx_empty:out std_logic; --Buffer empty;
    signal tx_mem_free:out std_logic_vector (7 downto 0); --Number of free words in TX counter
    signal tx_read_pointer_pos: out std_logic_vector(7 downto 0); --Read pointer value propagated
    signal tx_write_pointer_pos: out std_logic_vector(7 downto 0); --Write pointer value propagated
    signal tx_message_disc:out std_logic --Signal that acutal message was discarded and not stored for transcieving
  );
  
  -----------------------
  --Driving bus aliases--
  -----------------------
  signal drv_erase_tx: std_logic; --Logic 1 Erases the data in transcieve buffer
  signal drv_write_tx: std_logic; --Logic 1 Stores the value on tran_data_in into the buffer (Format A Message format)
  
  ----------------
  --FIFO Memory --
  ----------------
  constant data_width:natural:=32;
  
  type tx_memory is array (0 to buff_size-1) of std_logic_vector(data_width-1 downto 0); --TX Buffer memory type
  signal memory:tx_memory; --Memory declaration
  signal read_pointer : natural range 0 to buff_size-1; --Read Pointer for data
  signal write_pointer : natural range 0 to buff_size-1; --Write pointer
  signal prev_store:std_logic; --Previous value of store signal. Used for edge detection of drv_store_tx
  --Note: by write in user registers, the drv_store_tx can be in logic one for more cycles (depending on length of write cycle). In order
  --      not to store the data into the buffer more times, edge detection has to be performed!
  --Note 2: Every command of write into the the buffer has to be separate write command and have the edge on drv_store_tx. Otherwise data will
  --       be stored just oonce!
  
end entity;

architecture rtl of txBuffer is
begin
  --Driving bus aliases
  drv_erase_tx<=drv_bus(DRV_ERASE_TXT1_INDEX);
  drv_write_tx<=drv_bus(DRV_STORE_TXT1_INDEX);
  
  --Registers to output propagation
  tx_read_pointer_pos<=std_logic_vector(to_unsigned(read_pointer,8)); --Output information about pointer position
  tx_write_pointer_pos<=std_logic_vector(to_unsigned(write_pointer,8));  
  tx_buff_size<=std_logic_vector(to_unsigned(buff_size,8));  --Buffer size to output
  
  
  mem_acess:process(res_n,clk_sys)
  variable data_length_read:natural range 0 to 16;
  variable data_length_write:natural range 0 to 16;
  variable mem_free:natural range 0 to buff_size;
  variable message_count:natural range 0 to 255;
  begin
  if(res_n=ACT_RESET) or (drv_erase_tx='1')then
    read_pointer<=0;
    write_pointer<=0;
    memory<=(OTHERS=>(OTHERS=>'0'));
    data_length_read:=0;
    data_length_write:=0;
    mem_free:=buff_size;
    message_count:=0;
    prev_store<='0';
    
    tx_full<='0';
    tx_message_count<=(OTHERS=>'0');
    tx_empty<='1';
    tx_mem_free<=std_logic_vector(to_unsigned(buff_size,8));
    tx_message_disc<='0';
    
    tx_buffer_out<=(OTHERS=>'0');
    tx_buffer_valid<='0';
  elsif rising_edge(clk_sys) then
   prev_store<=drv_write_tx;  --Registering the value for edge detection
      
   --Data length encoding, data_length=number of bytes/4 (FIFO is 32 bits)
    case memory(read_pointer)(3 downto 0) is
        when "0000" => data_length_read:=0;
        when "0001" => data_length_read:=1;
        when "0010" => data_length_read:=1;
        when "0011" => data_length_read:=1;
        when "0100" => data_length_read:=1;
        when "0101" => data_length_read:=2;
        when "0110" => data_length_read:=2;
        when "0111" => data_length_read:=2;
        when "1000" => data_length_read:=2;
        when "1001" => data_length_read:=3;
        when "1010" => data_length_read:=4;
        when "1011" => data_length_read:=5; 
        when "1100" => data_length_read:=6;
        when "1101" => data_length_read:=8;
        when "1110" => data_length_read:=12;
        when "1111" => data_length_read:=16; 
        when others => data_length_read:=0;
    end case;    
    
   --Oldest message to the TX Arbitrator
    if(message_count>0)then
      tx_buffer_valid<='1';
      tx_buffer_out(TX_FFW_HIGH downto TX_FFW_LOW)<=memory(read_pointer);  --Frame format Word
      tx_buffer_out(TX_IDW_HIGH downto TX_IDW_LOW)<=memory(read_pointer+1); --Identifier Word
      
   --Data Words
      for I in 0 to 15 loop
        if(I<data_length_read)then
          tx_buffer_out(TX_DATAW_HIGH-I*32 downto TX_DATA1W_LOW-I*32)<=memory(read_pointer+2+I); 
        end if;
      end loop;  
    else
      tx_buffer_valid<='0';
      tx_buffer_out<=(OTHERS=>'0');
    end if;
    
   --Moving to the next message by acknowledge from CAN Core (TX Arbitrator)
    if(tx_buffer_ack='1' and message_count>0)then
      message_count:=message_count-1;
      read_pointer<=read_pointer+2+data_length_read;
      mem_free:=mem_free+2+data_length_read;
    end if;
      
   --Data length encoding for stored message
    case tran_data_in(611 downto 608) is
        when "0000" => data_length_write:=0;
        when "0001" => data_length_write:=1;
        when "0010" => data_length_write:=1;
        when "0011" => data_length_write:=1;
        when "0100" => data_length_write:=1;
        when "0101" => data_length_write:=2;
        when "0110" => data_length_write:=2;
        when "0111" => data_length_write:=2;
        when "1000" => data_length_write:=2;
        when "1001" => data_length_write:=3;
        when "1010" => data_length_write:=4;
        when "1011" => data_length_write:=5; 
        when "1100" => data_length_write:=6;
        when "1101" => data_length_write:=8;
        when "1110" => data_length_write:=12;
        when "1111" => data_length_write:=16; 
        when others => data_length_write:=0;
    end case;    
      
   --Storing the message (Format A) in the buffer
    if(drv_write_tx='1' and prev_store='0') then
     if(mem_free>=data_length_write+2)then    
      tx_message_disc<='0';
      memory(write_pointer)<=tran_data_in(TX_FFW_HIGH downto TX_FFW_LOW); --Frame format word
      memory(write_pointer+1)<=tran_data_in(TX_IDW_HIGH downto TX_IDW_LOW); --Identifier word
      
   --Data words
      for I in 0 to 15 loop
        if(I<data_length_write)then
          memory(write_pointer+2+I)<=tran_data_in(TX_DATAW_HIGH-I*32 downto TX_DATA1W_LOW-I*32);
        end if;
      end loop;
      write_pointer<=write_pointer+2+data_length_write; 
      mem_free:=mem_free-2-data_length_write;
      message_count:=message_count+1; 
     else
      tx_message_disc<='1';
     end if;
     tx_message_disc<='0'; 
    end if;
   
   --Buffer full
   if(mem_free=0)then
    tx_full<='1';
   else
    tx_full<='0';
   end if;
   
   --Buffer empty
   if(mem_free=buff_size)then
    tx_empty<='1';
   else
    tx_empty<='0';
   end if;  
   
   --Message count and free memory
   tx_message_count<=std_logic_vector(to_unsigned(message_count,8));
   tx_mem_free<=std_logic_vector(to_unsigned(mem_free,8));
   
   end if;
  end process mem_acess;
end architecture;