Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
USE WORK.CANconstants.ALL;

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
--    15.11.2017   Created file
--    27.11.2017   Added "rst_sync" asynchronous rest synchroniser circuit
-------------------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Package for components declarations to avoid writing component declarations every time into
--  architecture itself.
---------------------------------------------------------------------------------------------------------------

package CANcomponents is
  
  --------------------------------------------------------------------------------
  --------------------------------------------------------------------------------
  ---- CAN FD Core top level entity
  --------------------------------------------------------------------------------
  --------------------------------------------------------------------------------
  component CAN_top_level is
  generic(
      constant use_logger     :boolean  :=true; --Whenever event logger should be synthetised (not yet implemented)
      constant rx_buffer_size :natural  :=128; --Transcieve Buffer size
      constant useFDSize      :boolean  :=true; --Transcieve buffer size should be synthetised as FD Size (640 bits) or normal CAN (128 bits)
      constant use_sync       :boolean  :=true; --Whenever internal synchroniser chain should be used for incoming bus signals
                                                --Dont turn off unless external synchronisation chain is put on input of FPGA by
                                                --synthetiser
      constant ID             :natural  range 0 to 15:=1; --ID (bits  19-16 of adress) 
      constant logger_size    :natural --range 0 to 512:=8
  );
  port(
      signal clk_sys:in std_logic;
      signal res_n:in std_logic;
      signal data_in:in std_logic_vector(31 downto 0);
      signal data_out:out std_logic_vector(31 downto 0);
      signal adress:in std_logic_vector(23 downto 0);
    	 signal scs:in std_logic; --Chip select
      signal srd:in std_logic; --Serial read
      signal swr:in std_logic; --Serial write
      signal int:out std_logic;
      signal CAN_tx:out std_logic;
      signal CAN_rx:in std_logic;
      signal time_quanta_clk:out std_logic; --Time Quantum clocks possible to be used for synchronisation
      signal timestamp:in std_logic_vector(63 downto 0)
  );
  end component;
  

  --------------------------------------------------------------------------------
  --------------------------------------------------------------------------------
  ---- CAN Top level components
  --------------------------------------------------------------------------------
  --------------------------------------------------------------------------------

  --------------
  --Registers --
  --------------
  component registers is
  generic(
    constant compType           :     std_logic_vector(3 downto 0):= CAN_COMPONENT_TYPE;
    constant use_logger         :     boolean                     := true; --Whenever event logger is present
    constant ID                 :     natural --ID of the component
  );
  port(
    signal clk_sys              :in   std_logic;
    signal res_n                :in   std_logic;
    signal res_out              :out  std_logic;
    
    signal data_in              :in   std_logic_vector(31 downto 0);
    signal data_out             :out  std_logic_vector(31 downto 0);
    signal adress               :in   std_logic_vector(23 downto 0);
    signal scs                  :in   std_logic;
    signal srd                  :in   std_logic;
    signal swr                  :in   std_logic;
      
    signal drv_bus              :out  std_logic_vector(1023 downto 0);
    signal stat_bus             :in   std_logic_vector(511 downto 0);
    
    signal rx_read_buff         :in   std_logic_vector(31 downto 0); --Actually loaded data for reading
    signal rx_buf_size          :in   std_logic_vector(7 downto 0); --Actual size of synthetised message buffer (in 32 bit words)
    signal rx_full              :in   std_logic; --Signal whenever buffer is full
    signal rx_empty             :in   std_logic; --Signal whenever buffer is empty
    signal rx_message_count     :in   std_logic_vector(7 downto 0); --Number of messaged stored in recieve buffer
    signal rx_mem_free          :in   std_logic_vector(7 downto 0); --Number of free 32 bit wide ''windows''
    signal rx_read_pointer_pos  :in   std_logic_vector(7 downto 0); --Position of read pointer
    signal rx_write_pointer_pos :in   std_logic_vector(7 downto 0); --Position of write pointer
    signal rx_message_disc      :in   std_logic; --Message was discarded since Memory is full
    signal rx_data_overrun      :in   std_logic; --Some data were discarded, register
    
    signal tran_data_in         :out  std_logic_vector(639 downto 0);  --Transcieve data (Common for TX Buffer and TXT Buffer)
    
    signal txt1_empty           :in   std_logic; --Logic 1 signals empty TxTime buffer
    signal txt1_disc            :in   std_logic; --Info that message store into buffer from driving registers failed because buffer is full
    
    signal txt2_empty           :in   std_logic; --Logic 1 signals empty TxTime buffer
    signal txt2_disc            :in   std_logic; --Info that message store into buffer from driving registers failed because buffer is full

    signal trv_delay_out        :in   std_logic_vector(15 downto 0);

    signal int_vector           :in   std_logic_vector(10 downto 0); --Interrupt vector (Interrupt register of SJA1000)
    
    signal loger_act_data       :in   std_logic_vector(63 downto 0);
    signal log_write_pointer    :in   std_logic_vector(7 downto 0);
    signal log_read_pointer     :in   std_logic_vector(7 downto 0);
    signal log_size             :in   std_logic_vector(7 downto 0);
    signal log_state_out        :in   logger_state_type
  );
  end component;
  
  ---------------------
  --RX Buffer module --
  ---------------------
  component rxBuffer is
  generic(
      buff_size                 :     natural range 4 to 512
  );
  port(
    signal clk_sys              :in   std_logic; --System clock
    signal res_n                :in   std_logic; --Async. reset
    
    signal rec_ident_in         :in   std_logic_vector(28 downto 0); --Message Identifier
    signal rec_data_in          :in   std_logic_vector(511 downto 0); --Message Data (up to 64 bytes);
    signal rec_dlc_in           :in   std_logic_vector(3 downto 0); --Data length code
    signal rec_ident_type_in    :in   std_logic; --Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_frame_type_in    :in   std_logic; --Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_is_rtr           :in   std_logic; --Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_message_valid    :in   std_logic; --Output from acceptance filters (out_ident_valid) if message fits the filters
    signal rec_brs              :in   std_logic; --Whenever frame was recieved with BIT Rate shift 
    signal rec_esi              :in   std_logic;                        --Recieved error state indicator
    signal rec_message_ack      :out  std_logic; --Acknowledge for CAN Core about accepted data
    
    signal rx_buf_size          :out  std_logic_vector(7 downto 0); --Actual size of synthetised message buffer (in 32 bit words)
    signal rx_full              :out  std_logic; --Signal whenever buffer is full
    signal rx_empty             :out  std_logic; --Signal whenever buffer is empty
    signal rx_message_count     :out  std_logic_vector(7 downto 0); --Number of messaged stored in recieve buffer
    signal rx_mem_free          :out  std_logic_vector(7 downto 0); --Number of free 32 bit wide ''windows''
    signal rx_read_pointer_pos  :out  std_logic_vector(7 downto 0); --Position of read pointer
    signal rx_write_pointer_pos :out  std_logic_vector(7 downto 0); --Position of write pointer
    signal rx_message_disc      :out  std_logic; --Message was discarded since Memory is full
    signal rx_data_overrun      :out  std_logic; --Some data were discarded, register
    
    signal timestamp            :in   std_logic_vector(63 downto 0);
    
    signal rx_read_buff         :out  std_logic_vector(31 downto 0); --Actually loaded data for reading
    signal drv_bus              :in   std_logic_vector(1023 downto 0) --Driving bus from registers
    
  );
  end component;
  
  ----------------------
  --TX Buffer  module --
  ----------------------
  component txBuffer is 
  GENERIC (
    buff_size                   :     natural --Only powers of 2 allowed as buff_size!! (32,64,128,256)
  );
  PORT(
    signal clk_sys              :in   std_logic; --System clock
    signal res_n                :in   std_logic; --Async Reset
    
    signal drv_bus              :in   std_logic_vector(1023 downto 0);  --Driving bus from registers
    signal tran_data_in         :in   std_logic_vector(639 downto 0);  --Transcieve data    

    signal tx_buffer_out        :out  std_logic_vector(639 downto 0); --Output frame with TX buffer structure
    signal tx_buffer_valid      :out  std_logic; --Valid data on the output of TX Buffer
    signal tx_buffer_ack        :in   std_logic; --Acknowledge from TX Arbitrator that message was loaded into CAN Core and can be erased
    
    signal tx_buff_size         :out  std_logic_vector(7 downto 0); --Size of transcieve Buffer in 32 bit words
    signal tx_full              :out  std_logic; --Trascieve buffer is full
    signal tx_message_count     :out  std_logic_vector(7 downto 0); --Number of messages in the TX buffer
    signal tx_empty             :out  std_logic; --Buffer empty;
    signal tx_mem_free          :out  std_logic_vector (7 downto 0); --Number of free words in TX counter
    signal tx_read_pointer_pos  :out  std_logic_vector(7 downto 0); --Read pointer value propagated
    signal tx_write_pointer_pos :out  std_logic_vector(7 downto 0); --Write pointer value propagated
    signal tx_message_disc      :out  std_logic --Signal that acutal message was discarded and not stored for transcieving
  );
  end component;
  
  ----------------------
  --TXT Buffer module --
  ---------------------- 
  component txtBuffer is 
    generic  (
      constant ID               :     natural:=1;
      constant useFDsize        :     boolean:=false
    );
    PORT(
      signal clk_sys            :in   std_logic;
      signal res_n              :in   std_logic; --Async reset

      signal drv_bus            :in   std_logic_vector(1023 downto 0); --Driving bus
      signal tran_data_in       :in   std_logic_vector(639 downto 0); --Input data frame (Format B value of transcieve register of driving registers) 
      
      signal txt_empty          :out  std_logic; --Logic 1 signals empty TxTime buffer
      signal txt_disc           :out  std_logic; --Info that message store into buffer from driving registers failed because buffer is full
            
      signal txt_buffer_out     :out  std_logic_vector(639 downto 0); --Output value of message in the buffer  
      signal txt_data_ack       :in   std_logic --Signal from TX Arbiter that data were sent and buffer can be erased     
      );
  end component;  
  
  -------------------------
  --TXT Arbitrator module--
  -------------------------
  component txArbitrator is 
  port( 
    ------------------------
    --TX Buffers interface--
    ------------------------
    --TXT Buffer 1
    signal txt1_buffer_in       :in   std_logic_vector(639 downto 0); --Time TX1 buffer input
    signal txt1_buffer_ack      :out  std_logic; --Time buffer acknowledge that message can be erased
    signal txt1_buffer_empty    :in   std_logic; --No message in Time TX Buffer
    
    --TXT Buffer 2
    signal txt2_buffer_in       :in   std_logic_vector(639 downto 0); --Time TX1 buffer input
    signal txt2_buffer_empty    :in   std_logic; --No message in Time TX Buffer
    signal txt2_buffer_ack      :out  std_logic; --Acknowledge for message that it can erased
  
    -----------------------
    --CAN Core Interface---
    -----------------------
    signal tran_data_out        :out  std_logic_vector(511 downto 0); --TX Message data
    signal tran_ident_out       :out  std_logic_vector(28 downto 0); --TX Identifier 
    signal tran_dlc_out         :out  std_logic_vector(3 downto 0); --TX Data length code
    signal tran_is_rtr          :out  std_logic; --TX is remote frame
    signal tran_ident_type_out  :out  std_logic; --TX Identifier type (0-Basic,1-Extended);
    signal tran_frame_type_out  :out  std_logic; --TX Frame type
    signal tran_brs_out         :out  std_logic; --Bit rate shift for CAN FD frames
    
    signal tran_frame_valid_out :out  std_logic; --Signal for CAN Core that frame on the output is valid and can be stored for transmitting
    signal tran_data_ack        :in   std_logic; --Acknowledge from CAN core that acutal message was stored into internal buffer for transmitting   
      
    ---------------------
    --Driving interface--
    ---------------------
    signal drv_bus              :in   std_logic_vector(1023 downto 0); --Driving bus from registers
    signal timestamp            :in   std_logic_vector(63 downto 0) --TimeStamp value
    
  );
  end component;
  
  -------------------------
  --Message filter module--
  -------------------------
  component messageFilter is
  PORT(
    signal clk_sys              :in   std_logic; --System clock
    signal res_n                :in   std_logic; --Async reset
    
    signal rec_ident_in         :in   std_logic_vector(28 downto 0);--Receieved identifier
    signal ident_type           :in   std_logic; --Input message identifier type (0-BASE Format, 1-Extended Format);
    signal frame_type           :in   std_logic; --Input frame type (0-Normal CAN, 1- CAN FD) 
    signal rec_ident_valid      :in   std_logic; --Identifier valid (active log 1)
    
    signal drv_bus              :in   std_logic_vector(1023 downto 0);
    
    signal out_ident_valid      :out  std_logic --Signal whenever identifier matches the filter identifiers
    );
  end component;
  
  ----------------------------
  --Interrupt manager module--
  ----------------------------
  component intManager is 
  GENERIC(
    constant int_length         :     natural range 0 to 10:=5 --Lenght in clock cycles how long will interrupt stay active
    );
  PORT(
    signal clk_sys              :in   std_logic; --System Clock
    signal res_n                :in   std_logic; --Async Reset
    
    signal error_valid          :in   std_logic; --Valid Error appeared for interrupt
    signal error_passive_changed:in   std_logic; --Error pasive /Error acitve functionality changed
    signal error_warning_limit  :in   std_logic; --Error warning limit reached
    
    signal arbitration_lost     :in   std_logic; --Arbitration was lost input
    signal wake_up_valid        :in   std_logic; --Wake up appeared
    signal tx_finished          :in   std_logic; --Message stored in CAN Core was sucessfully transmitted
    signal br_shifted           :in   std_logic; --Bit Rate Was Shifted
    
     signal rx_message_disc     :in   std_logic; --Income message was discarded
    signal rec_message_valid    :in   std_logic; --Message recieved!
    --Note : use the "out_ident_valid" signal of messageFilters. Therefore only
    --interrupt is started for signals which pass income filters
    
    signal rx_full              :in   std_logic; --RX Buffer is full (the last income message filled the remaining space)
    --NOTE! rec_message_valid will be in logic one for two clock cycles
    signal loger_finished       :in   std_logic;  --Event logging finsihed

    signal drv_bus              :in   std_logic_vector(1023 downto 0);
    signal int_out              :out  std_logic; --Interrupt output
    
    signal int_vector           :out  std_logic_vector(10 downto 0) --Interrupt vector (Interrupt register of SJA1000)
  );
 	end component;
 	
 	--------------------
 	--CAN Core module --
 	--------------------
 	component core_top is
  PORT(
    signal clk_sys              :in   std_logic;
    signal res_n                :in   std_logic;
    
    signal drv_bus              :in   std_logic_vector(1023 downto 0);    
    signal stat_bus             :out  std_logic_vector(511 downto 0);
    
    signal tran_data_in         :in   std_logic_vector(511 downto 0);
    signal tran_ident_in        :in   std_logic_vector(28 downto 0);
    signal tran_dlc_in          :in   std_logic_vector(3 downto 0);
    signal tran_is_rtr_in       :in   std_logic;
    signal tran_ident_type_in   :in   std_logic; --TX Identifier type (0-Basic,1-Extended);
    signal tran_frame_type_in   :in   std_logic; --TX Frame type (0-CAN Normal, 1-CAN FD)
    signal tran_brs_in          :in   std_logic; --Frame should be transcieved with BRS value
    signal tran_frame_valid_in  :in   std_logic; --Signal for CAN Core that frame on the output is valid and can be stored for transmitting
    signal tran_data_ack_out    :out  std_logic; --Acknowledge from CAN core that acutal message was stored into internal buffer for transmitting   
 
    signal rec_ident_out        :out  std_logic_vector(28 downto 0); --Message Identifier
    signal rec_data_out         :out  std_logic_vector(511 downto 0); --Message Data (up to 64 bytes);
    signal rec_dlc_out          :out  std_logic_vector(3 downto 0); --Data length code
    signal rec_ident_type_out   :out  std_logic; --Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_frame_type_out   :out  std_logic; --Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_is_rtr_out       :out  std_logic; --Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_brs_out          :out  std_logic; --Frame was recieved with Bit rate shift
    signal rec_esi_out          :out  std_logic; --Error state indicator
    signal rec_message_valid_out:out  std_logic;
    signal rec_message_ack_out  :in   std_logic; --Acknowledge for CAN Core about accepted data
    
    signal arbitration_lost_out :out  std_logic; --Arbitration was lost input
    signal wake_up_valid        :out  std_logic; --Wake up appeared
    signal tx_finished          :out  std_logic; --Message stored in CAN Core was sucessfully transmitted
    signal br_shifted           :out  std_logic; --Bit Rate Was Shifted
    
    signal error_valid          :out  std_logic; --At least one error appeared
    signal error_passive_changed:out  std_logic; --Error passive state changed
    signal error_warning_limit  :out  std_logic; --Error warning limit was reached
  
    signal sample_nbt_del_2     :in   std_logic; --Sampling signal 2 clk_sys delayed from sample_nbt
    signal sample_dbt_del_2     :in   std_logic; --Sampling signal 2 clk_sys delayed from sample_dbt
    signal sample_nbt_del_1     :in   std_logic; --Sampling signal of NBT for Bit destuffing
    signal sample_dbt_del_1     :in   std_logic; --Sampling signal of DBT for Bit destuffing
    signal sync_nbt             :in   std_logic; --Beginning of Nominal bit time (transcieving next bit)
    signal sync_dbt             :in   std_logic; --Beginning of Data bit time (transcieving next bit)
    signal sync_nbt_del_1       :in   std_logic; --Bit stuffing trigger NBT
    signal sync_dbt_del_1       :in   std_logic; --Bit Stuffing trigger DBT
    
    signal sample_sec           :in   std_logic; --Secondary sample signal 
    signal sample_sec_del_1     :in   std_logic; --Bit destuffing trigger for secondary sample point
    signal sample_sec_del_2     :in   std_logic; --Rec trig for secondary sample point
        
    signal sync_control         :out  std_logic_vector(1 downto 0); --Synchronisation control signal (Hard sync, Re Sync)
    
    signal data_rx              :in   std_logic; --Recieved data input (valid with sample_nbt_del_1)
    signal data_tx              :out  std_logic; --Data transcieved on CAN Bus (valid with sync_nbt_del_1)
    
    signal timestamp            :in   std_logic_vector(63 downto 0);
    
    signal sp_control           :out  std_logic_vector(1 downto 0); --Control signal for Sampling source
    signal ssp_reset            :out  std_logic; --Secondary sample point reset
    signal trv_delay_calib      :out  std_logic; --Enable calibration of transciever delay compenstation
    signal bit_Error_sec_sam    :in   std_logic; --Bit error with secondary sampling transciever!
    signal hard_sync_edge       :in   std_logic
   );
  end component;
  
  ---------------------
  --Prescaler module --
  ---------------------
  component prescaler_v3 is
  PORT(
    signal clk_sys              :in   std_logic;  --System clock
    signal res_n                :in   std_logic;   --Async reset
    
    signal sync_edge            :in   std_logic; --Edge for synchronisation
    signal OP_State             :in   oper_mode_type; --Protocol control state
    
    signal drv_bus              :in   std_logic_vector(1023 downto 0); 
    
    signal clk_tq_nbt           :out  std_logic; --Time quantum clock - Nominal bit time
    signal clk_tq_dbt           :out  std_logic; --bit time - Nominal bit time
    
    signal sample_nbt           :out  std_logic; --Sample signal for nominal bit time
    signal sample_dbt           :out  std_logic; --Sample signal of data bit time
    signal sample_nbt_del_1     :out  std_logic;
    signal sample_dbt_del_1     :out  std_logic;
    signal sample_nbt_del_2     :out  std_logic;
    signal sample_dbt_del_2     :out  std_logic;
    
    signal sync_nbt             :out  std_logic;
    signal sync_dbt             :out  std_logic;
    signal sync_nbt_del_1       :out  std_logic;
    signal sync_dbt_del_1       :out  std_logic;
    signal bt_FSM_out           :out  bit_time_type;
    
    signal data_tx              :in   std_logic;
    
    signal hard_sync_edge_valid :out  std_logic; --Validated hard synchronisation edge to start Protocol control FSM
          --Note: Sync edge from busSync.vhd cant be used! If it comes during sample nbt, sequence it causes
          --      errors! It needs to be strictly before or strictly after this sequence!!! 
  
    signal sp_control           :in   std_logic_vector(1 downto 0);
    signal sync_control         :in   std_logic_vector(1 downto 0)    
  );
  end component;
  
  ----------------------------
  --Bus synchroniser module--
  ---------------------------
  component busSync is 
  GENERIC (
      use_Sync                  :     boolean 
      --Whenever Synchronisation chain should be used for sampled data from the bus. 
      --Turn off only when Synthetizer puts synchronisation chain automatically on the
      --output pins! Otherwise metastability issues will occur!
    );  
  PORT(
    signal clk_sys              :in   std_logic; --System clock
    signal res_n                :in   std_logic; --Async Reset
    
    signal CAN_rx               :in   std_logic; --CAN data input from transciever
    signal CAN_tx               :out  std_logic; --CAN data output to transciever  
    
    signal drv_bus              :in   std_logic_vector(1023 downto 0); --Driving bus
      
    signal sample_nbt           :in   std_logic; --Sample command for nominal bit time
    signal sample_dbt           :in   std_logic; --Sample command for data bit tim    
    signal sync_edge            :out  std_logic; --Synchronisation edge appeared
        
    signal data_tx              :in   std_logic; --Transcieve data value
    signal data_rx              :out  std_logic; --Recieved data value
    
    signal sp_control           :in   std_logic_vector(1 downto 0); --Control sequence for sampling 
                    --00:sample_nbt used for sampling (Nominal bit time sampling, Transciever and Reciever)
                    --01:sample_dbt used for sampling (Data bit time sampling, only Reciever)
                    --10:Sampling with transciever delay compensation (Data bit time, transciever)
    signal ssp_reset            :in   std_logic; --Clear the Shift register at the  beginning of Data Phase!!!
    signal trv_delay_calib      :in   std_logic; --Calibration command for transciever delay compenstation (counter)
    
    signal bit_err_enable       :in   std_logic; 
    
    signal sample_sec_out       :out  std_logic; --Secondary sample signal 
    signal sample_sec_del_1_out :out  std_logic; --Bit destuffing trigger for secondary sample point
    signal sample_sec_del_2_out :out  std_logic; --Rec trig for secondary sample point
    
    signal trv_delay_out        :out  std_logic_vector(15 downto 0);
  
    signal bit_Error            :out  std_logic --Bit Error appeared (monitored value different than transcieved value)   
   ); 
  end component;
  
  component CAN_logger is 
  generic(
    constant memory_size        :     natural:=16 --Only 2^k possible!
  );
  port(
    signal clk_sys              :in   std_logic;
    signal res_n                :in   std_logic;
    
    signal drv_bus              :in   std_logic_vector(1023 downto 0);
    signal stat_bus             :in   std_logic_vector(511 downto 0);
    signal sync_edge            :in   std_logic;
    signal data_overrun         :in   std_logic;
    signal timestamp            :in   std_logic_vector(63 downto 0);
    signal bt_FSM               :in   bit_time_type;
    
    signal loger_finished       :out  std_logic; --Logger finished interrrupt output
    signal loger_act_data       :out  std_logic_vector(63 downto 0);
    signal log_write_pointer    :out  std_logic_vector(7 downto 0);
    signal log_read_pointer     :out  std_logic_vector(7 downto 0);
    signal log_size             :out  std_logic_vector(7 downto 0);
    signal log_state_out        :out  logger_state_type
    
  ); 
end component;
  
  
  --------------------------------------------------------------------------------
  --------------------------------------------------------------------------------
  ---- CORE Top level components
  --------------------------------------------------------------------------------
  --------------------------------------------------------------------------------
  
  ---------------
  --CRC module --
  ---------------
  component canCRC is
  generic(
    constant crc15_pol :     std_logic_vector(15 downto 0):=std_logic_vector'(X"C599");
    constant crc17_pol :     std_logic_vector(19 downto 0):=std_logic_vector'(X"3685B");
    constant crc21_pol :     std_logic_vector(23 downto 0):=std_logic_vector'(X"302899")  
  );
  port(
    signal data_in                :in   std_logic; --Serial data input
    signal clk_sys                :in   std_logic; --System clock input
    
    --Trigger to sample the input value . Note: Trigger generated by prescaler
    --Note 2: trigger for CAN FD should be 1 clk_sys behind normal CAN since for CAN FD bit stuffing is made
    --before CRC calculation
    signal trig                   :in   std_logic;
    
    --Asynchronous reset
    signal res_n                  :in   std_logic; 
    
    --By transition from 0 to 1 on enable sampled on clk_sys rising edge (and with trig='1')
    --operation is started. First bit of data already has to be on data_in input.
    --Circuit works as long as enable=1.
    signal enable                 :in   std_logic; 
    signal drv_bus                :in   std_logic_vector(1023 downto 0);
    
    signal crc15                  :out  std_logic_vector(14 downto 0);
    signal crc17                  :out  std_logic_vector(16 downto 0);
    signal crc21                  :out  std_logic_vector(20 downto 0)
  ); 
  end component;
  
  ---------------------
  --Transcieve buffer--
  ---------------------
  component tranBuffer is 
  port(
    signal clk_sys                :in   std_logic; --System clock
    signal res_n                  :in   std_logic;
    
    signal tran_data_in           :in   std_logic_vector(511 downto 0);
    signal tran_ident_in          :in   std_logic_vector(28 downto 0);
    signal tran_dlc_in            :in   std_logic_vector(3 downto 0);
    signal tran_is_rtr_in         :in   std_logic;
    signal tran_ident_type_in     :in   std_logic; --TX Identifier type (0-Basic,1-Extended);
    signal tran_frame_type_in     :in   std_logic; --TX Frame type (0-CAN Normal, 1-CAN FD)
    signal tran_brs_in            :in   std_logic;
    
    signal frame_store            :in   std_logic; --Store the data on input
    
    signal tran_data              :out  std_logic_vector(511 downto 0);
    signal tran_ident             :out  std_logic_vector(28 downto 0);
    signal tran_dlc               :out  std_logic_vector(3 downto 0);
    signal tran_is_rtr            :out  std_logic;
    signal tran_ident_type        :out  std_logic;
    signal tran_frame_type        :out  std_logic;
    signal tran_brs               :out  std_logic  
  );
  end component;

  -----------------
  --Bit Stuffing --
  -----------------
  component bitStuffing_v2 is 
  port(
    signal clk_sys                :in   std_logic;
    signal res_n                  :in   std_logic;
    signal tran_trig_1            :in   std_logic; --Trigger signal for propagating the data (one clk_sys delayed behind beginning of bit time) 
    signal enable                 :in   std_logic; --Enabling the operation of the circuit
    signal data_in                :in   std_logic; --Data Input sampled 
    signal fixed_stuff            :in   std_logic; --Whenever fixed bit stuffing should be used (CAN FD Option)
    signal data_halt              :out  std_logic; --Logic 1 signals stuffed bit for CAN Core. CAN Core has to halt the data sending for one bit-time  
    signal length                 :in   std_logic_vector(2 downto 0); --Length of Bit Stuffing
    signal bst_ctr                :out  natural range 0 to 7; --Bit stuffing counter
    signal data_out               :out  std_logic --Data output        
  );
  end component;
  
  ------------------
  --Bit Destuffing--
  ------------------
  component bitDestuffing is
  PORT(
    signal clk_sys                :in   std_logic; --System clock
    signal res_n                  :in   std_logic; --Async Reset
    signal data_in                :in   std_logic; --Sampled data from busSync.vhd
    signal trig_spl_1             :in   std_logic; --Triggering signal with one clk_sys delay behind the used sampling signal
    signal stuff_Error            :out  std_logic; --Stuff Error
    signal data_out               :out  std_logic; --Data output for CAN Core
    signal destuffed              :out  std_logic; --Signal that data on output are not valid but it is a stuff bit
    signal enable                 :in   std_logic; --Enable of the circuit
    signal stuff_Error_enable     :in   std_logic; --Enable stuff Error logging
    signal fixed_stuff            :in   std_logic; --Whenever fixed bit Destuffing method is used    
    signal length                 :in   std_logic_vector(2 downto 0); --Length of bit stuffing rule
    signal dst_ctr                :out  natural range 0 to 7 --Bit destuffing length
  );
  end component;
  
  -------------------------
  --Operation control FSM--
  -------------------------
  component operationControl is
  PORT(
      signal clk_sys              :in   std_logic; 
      signal res_n                :in   std_logic;
      signal drv_bus              :in   std_logic_vector(1023 downto 0);
      signal arbitration_lost     :in   std_logic;
      signal PC_State             :in   protocol_type;
      signal tran_data_valid_in   :in   std_logic;
      signal set_transciever      :in   std_logic; --Set OP_State FSM into transciever state (Used at SOF)
      signal set_reciever         :in   std_logic; --Set OP_State FSM into reciever state
      signal is_idle              :in   std_logic; --Unit is idle
      signal tran_trig            :in   std_logic;
      signal rec_trig             :in   std_logic;
      signal data_rx              :     std_logic;
      signal OP_State             :out  oper_mode_type
    );
  end component;
  
  ------------------------
  --Protocol Control FSM--
  ------------------------
  component protocolControl is
  port(
    signal clk_sys                :in   std_logic; --System clock
    signal res_n                  :in   std_logic; --Async reset
    signal drv_bus                :in   std_logic_vector(1023 downto 0); --Driving bus signals
    
    signal int_loop_back_ena      :out  std_logic; --Internal loopBack enabled (for Bus monitoring mode)
    signal PC_State_out           :out  protocol_type;
    signal alc                    :out  std_logic_vector(4 downto 0);
    
    signal tran_data              :in   std_logic_vector(511 downto 0);
    signal tran_ident             :in   std_logic_vector(28 downto 0);
    signal tran_dlc               :in   std_logic_vector(3 downto 0);
    signal tran_is_rtr            :in   std_logic;
    signal tran_ident_type        :in   std_logic;
    signal tran_frame_type        :in   std_logic;
    signal tran_brs               :in   std_logic; 
    
    signal frame_store            :out  std_logic; --Store frame from TX Arbitrator to the Transcieve Buffer
    signal tran_frame_valid_in    :in   std_logic; --Valid frame ready to be stored into Transcieeve Buffer
    signal tran_data_ack          :out  std_logic; --Acknowledge that the frame was stored
    signal br_shifted             :out  std_logic; --Bit Rate Was Shifted
    
    signal rec_data               :out  std_logic_vector(511 downto 0);
    signal rec_ident              :out  std_logic_vector(28 downto 0);
    signal rec_dlc                :out  std_logic_vector(3 downto 0);
    signal rec_is_rtr             :out  std_logic;
    signal rec_ident_type         :out  std_logic;
    signal rec_frame_type         :out  std_logic;
    signal rec_brs                :out  std_logic;
    signal rec_crc                :out  std_logic_vector(20 downto 0); --Recieved CRC value
    signal rec_esi                :out  std_logic; --Recieved Error state indicator
      
    signal OP_state               :in   oper_mode_type; --Operation mode state
    signal arbitration_lost       :out  std_logic; --Signal for Operational mode state mahine about loosing arbitration
    signal is_idle                :out  std_logic; --Signal to indicate transcieve or recieve finished and bus is idle
    signal set_transciever        :out  std_logic; --Set OP_State FSM into transciever state (Used at SOF)
    signal set_reciever           :out  std_logic; --Set OP_State FSM into reciever state
    
    signal ack_recieved_out       :out  std_logic;
    
    signal error_state            :in   error_state_type; --Fault confinement state
    --Error signals for fault confinement
    signal form_Error             :out  std_logic; --Form Error
    signal CRC_Error              :out  std_logic; --CRC Error
    signal ack_Error              :out  std_logic; --Acknowledge error
    signal unknown_state_Error    :out  std_logic; --Some of the state machines, or signals reached unknown state!! Shouldnt happend!!
    signal bit_stuff_Error_valid  :in   std_logic; --Error signal for PC control FSM from fault confinement unit (Bit error or Stuff Error appeared)
        
    --Note: New Interface for fault confinement incrementation
    signal inc_one                :out  std_logic;
    signal inc_eight              :out  std_logic;
    signal dec_one                :out  std_logic;
    
    signal tran_valid             :out  std_logic;
    signal rec_valid              :out  std_logic;
    
    signal tran_trig              :in   std_logic; --Transcieve triggerring signal (sync_nbt,sync_dbt) ->multiplexed in core_top (CAN Core)
    signal rec_trig               :in   std_logic; --Recieve triggerring signal (sample_2_nbt,sample_2_dbt) ->multiplexed in core_top
     
    signal data_tx                :out  std_logic; --Transcieved data on CAN Bus
    signal stuff_enable           :out  std_logic;
    signal fixed_stuff            :out  std_logic; --Log 1 - Fixed Stuffing, Log 0 - Normal stuffing
    signal stuff_length           :out  std_logic_vector(2 downto 0); --Stuffing length
    signal data_rx                :in   std_logic; --Recieved data
    signal destuff_enable         :out  std_logic; --Enabling destuffing
    signal stuff_error_enable     :out  std_logic; --Enabling firing of destuffing error
    signal fixed_destuff          :out  std_logic; --Fixed stuffing method (log. 1), Normal stuffing (log 0);
    signal destuff_length         :out  std_logic_vector(2 downto 0); --Number of equal consequent bits before destuffed bit 
    signal dst_ctr                :in   natural range 0 to 7; --Number of stuffed bits modulo 8
    
    signal crc_enable             :out  std_logic; --Transition from 0 to 1 erases the CRC and operation holds as long as enable=1
    signal crc15                  :in   std_logic_vector(14 downto 0); --CRC 15
    signal crc17                  :in   std_logic_vector(16 downto 0); --CRC 17
    signal crc21                  :in   std_logic_vector(20 downto 0); --CRC 21
    
    signal sync_control           :out  std_logic_vector(1 downto 0); --00-no synchronisation, 10-Hard synchronisation, 11-Resynchronisation
    
    signal sp_control             :out  std_logic_vector(1 downto 0); --00 nominal, 01-data, 10 -secondary
    signal ssp_reset              :out  std_logic; --Clear the Shift register at the  beginning of Data Phase!!!    
    signal trv_delay_calib        :out  std_logic; --Calibration command for transciever delay compenstation (counter)
    signal bit_err_enable         :out  std_logic; --Bit Error detection enable (Ex. disabled when recieving data)
    signal hard_sync_edge         :in   std_logic
    );
  end component;

  ------------------------
  --Fault confinement   --
  ------------------------
  component faultConf is 
  PORT(
    signal clk_sys                :in   std_logic; --System clock
    signal res_n                  :in   std_logic; --Async reset
    signal drv_bus                :in   std_logic_vector(1023 downto 0);
    
    signal stuff_Error            :in   std_logic; --Stuffing Error from bit destuffing
      
    signal error_valid            :out  std_logic; --At least one error appeared
    signal error_passive_changed  :out  std_logic; --Error passive state changed
    signal error_warning_limit    :out  std_logic; --Error warning limit was reached
    
    signal OP_State               :in   oper_mode_type;
    
    signal data_rx                :in   std_logic; --Recieved data. Valid with the same signal as rec_trig in CAN Core
    signal data_tx                :in   std_logic; --Transcieved data by CAN Core. Valid with one clk_sys delay from tran_trig! The same trigger signal as Bit-Stuffing!
    signal rec_trig               :in   std_logic; --Recieve data trigger
    signal tran_trig_1            :in   std_logic; --Transcieve data trigger one clk_sys delayed behind the tran_trig
    
    signal PC_State               :in   protocol_type;
    signal sp_control             :in   std_logic_vector(1 downto 0);
    signal form_Error             :in   std_logic; --Form Error from PC State
    signal CRC_Error              :in   std_logic; --CRC Error from PC State
    signal ack_Error              :in   std_logic; --Acknowledge Error from PC State
    signal unknown_state_Error    :in   std_logic; --Some of the state machines, or signals reached unknown state!! Shouldnt happend!!
    signal bit_stuff_Error_valid  :out  std_logic; --Error signal for PC control FSM from fault confinement unit (Bit error or Stuff Error appeared)
        
    --Note: This new interface is used for error incrementation, decrementation!!
    signal inc_one                :in   std_logic;
    signal inc_eight              :in   std_logic;
    signal dec_one                :in   std_logic;
    
    signal enable                 :in   std_logic; --Enable for error counting
    signal bit_Error_sec_sam      :in   std_logic; --Bit Error detected with secondary sampling point at busSync.vhd
    signal bit_Error_out          :out  std_logic;
    
    signal tx_counter_out         :out  std_logic_vector(8 downto 0);
    signal rx_counter_out         :out  std_logic_vector(8 downto 0);
    signal err_counter_norm_out   :out  std_logic_vector(15 downto 0);
    signal err_counter_fd_out     :out  std_logic_vector(15 downto 0);
    
    signal error_state_out        :out  error_state_type
  );
 end component;

  ---------------------------------------
  --Asynchronous resset synchroniser   --
  ---------------------------------------
  component rst_sync is
    port (
        signal clk    : in  std_logic;
        signal arst_n : in  std_logic;
        signal rst_n  : out std_logic
    );
  end component;


   
end package;
