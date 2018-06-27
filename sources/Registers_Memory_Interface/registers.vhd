Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;

-------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
-- 
--    July 2015   Created file
--    19.12.2015  RETR register changed for settings register, added configuration options
--                for enabling and disabling whole controller, and selecting ISO FD option.
--                (Not yet implemented)
--    16.5.2016   Added restart function. Code formatting and constant replacement
--    19.6.2016   Changed tx_data reg to be array 5*128 bits instead of 640 std_logic vector
--                This should ease the automatic inference into RAM memory...
--    20.6.2016   Added ET bit in status register to monitor transmittion of error frame!
--    21.6.2016   Fixed SETTINGS registers some of the bits were not read back correctly
--    23.6.2016   Added DEBUG_REG for some additional debugging purposes
--    15.7.2016   Added "RX_buff_read_first" and "aux_data" signals. Changed handling of moving to
--                next word in RX buffer RX_DATA. Now first cycle of memory access is detected,
--                here and  "rx_read_start" is set to active value for only one clock cycle!
--                Even if bus access lasts several clock cycles data output is captured only in
--                the first cycle and then held until the end of access. Additionally "rx_read_start"
--                signal is now combinationall, not registered output. Thisway latency is shortened.
--                Without this precaution it was necessary to add empty cycles between reads from
--                RX_DATA!!!        
--    24.8.2016   Added "use_logger" generic and LOG_EXIST bit to the LOG_STATUS register to provide
--                way how to find out from SW if logger is actually present. Size is not deciding since
--                HW developer can set the size to e.g. 32 and use_logger to false!
--    1.9.2016    Moved SJW values to separate register! Now SJW has 4 bits instead of two bits! This is
--                compliant with CAN FD specification.
--
-------------------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Memory registers which control functionality of CAN FD core. Memory interface is 32 bit avalon compatible.
--  Registers create drv_bus signal which is used in whole CAN FD IP function to control all modules.         
--  Memory Reads and writes to any location need to be executed as one read, write. No extended cycles are    
--  allowed.                                                                                                  
--  Write to register as following:
--    1. SCS<='1' , data_in<=valid_data, adress<=valid_adress
--    2. SWR<='0' , wait at least one clock cycle
--    3. SWR<='1' SCS<='0'
--  Read from register as following:
--    1. SCS<='1' , adress<=valid_adress
--    2. SRD<='0' , wait at least one clock cycle
--    3. Capture valid data on data_out output
--    4. SWR<='1' SCS<='0'
---------------------------------------------------------------------------------------------------------------
--Note: All control signals which command any event execution which lasts one clock cycle has negative edge  --
--      detection. Therefore once srd or swr is active to finish the read or write it has to become inactive!--
---------------------------------------------------------------------------------------------------------------


entity registers is
  generic(
    constant compType              :std_logic_vector(3 downto 0)    := CAN_COMPONENT_TYPE;
    constant use_logger            :boolean                         := true; --Whenever event logger is present
    constant ID                    :natural                         := 1 --ID of the component
  );
  port(
    --Clock and asynchronous reset
    signal clk_sys              :in   std_logic;
    signal res_n                :in   std_logic;
    signal res_out              :out  std_logic;
    
    --------------------
    --Memory Interface--
    --------------------
    signal data_in              :in   std_logic_vector(31 downto 0);
    signal data_out             :out  std_logic_vector(31 downto 0);
    signal adress               :in   std_logic_vector(23 downto 0);
    signal scs                  :in   std_logic;
    signal srd                  :in   std_logic;
    signal swr                  :in   std_logic;
    --Memory interface is Avalon bus compatible!
    --In 32-bit system, every register is 0x4 higher adress. 
    --At FPGA the lowest two bits of adress are considered
    --to be cut, therefore +0x04 means one bit higher adress!!
    --Only 32 bit native access is supported!! 
      
    --Driving and Status Bus
    signal drv_bus              :out  std_logic_vector(1023 downto 0) := (OTHERS=>'0');
    signal stat_bus             :in   std_logic_vector(511 downto 0);
    
    -----------------------
    --RX Buffer Interface--
    -----------------------
    signal rx_read_buff         :in   std_logic_vector(31 downto 0);  --Actually loaded data for reading
    signal rx_buf_size          :in   std_logic_vector(7 downto 0);   --Size of  message buffer (in words)
    signal rx_full              :in   std_logic;                      --Signal whenever buffer is full
    signal rx_empty             :in   std_logic;                      --Signal whenever buffer is empty
    signal rx_message_count     :in   std_logic_vector(7 downto 0);   --Number of frames in recieve buffer
    signal rx_mem_free          :in   std_logic_vector(7 downto 0);   --Number of free 32 bit wide ''windows''
    signal rx_read_pointer_pos  :in   std_logic_vector(7 downto 0);   --Position of read pointer
    signal rx_write_pointer_pos :in   std_logic_vector(7 downto 0);   --Position of write pointer
    signal rx_message_disc      :in   std_logic;                      --Frame was discarded due full Memory
    signal rx_data_overrun      :in   std_logic;                      --Some data were discarded, register
    
    ----------------------------------
    --TXT1 and TXT2 Buffer Interface--
    ----------------------------------
    --Transcieve data (Common for TX Buffer and TXT Buffer)
    signal tran_data_in         :out  std_logic_vector(639 downto 0); 
    
    signal txt1_empty           :in   std_logic;                      --Logic 1 signals empty TxTime buffer
    signal txt1_disc            :in   std_logic;                      --Info that frame store into buffer from
                                                                      -- driving registers failed because buffer is full
                                                                      
    signal txt2_empty           :in   std_logic;                      --Logic 1 signals empty TxTime buffer
    signal txt2_disc            :in   std_logic;                      --Info that frame store into buffer from 
                                                                      -- driving registers failed because buffer is full
    
    signal trv_delay_out        :in   std_logic_vector(15 downto 0);
    
    --------------------------
    --Event logger interface--
    --------------------------
    signal loger_act_data       :in   std_logic_vector(63 downto 0);
    signal log_write_pointer    :in   std_logic_vector(7 downto 0);
    signal log_read_pointer     :in   std_logic_vector(7 downto 0);
    signal log_size             :in   std_logic_vector(7 downto 0);
    signal log_state_out        :in   logger_state_type;
        
    ------------------------
    --Interrrupt Interface--
    ------------------------
    --Interrupt vector (Interrupt register of SJA1000)
    signal int_vector           :in   std_logic_vector(10 downto 0) 
  );
  ----------------------
  --Internal registers--
  ----------------------
  signal int_reset              :     std_logic:='1';
  
  --Command registers
  signal clear_overrun          :     std_logic;
  signal release_recieve        :     std_logic;
  signal abort_transmittion     :     std_logic;
  signal ack_forb               :     std_logic;
  
  --Retransmitt registers
  signal retr_lim_ena           :     std_logic;                      --Retransmit limited is enabled
  signal retr_lim_th            :     std_logic_vector(3 downto 0);   --Retransmit treshold
  
  --Interrupt registers
  signal interrupt_vector_erase :     std_logic;
  
  --Timing registers
  signal sjw_norm               :     std_logic_vector(3 downto 0);
  signal brp_norm               :     std_logic_vector(5 downto 0);
  signal ph1_norm               :     std_logic_vector(4 downto 0);
  signal ph2_norm               :     std_logic_vector(4 downto 0);
  signal prop_norm              :     std_logic_vector(5 downto 0);
  signal sjw_fd                 :     std_logic_vector(3 downto 0);
  signal brp_fd                 :     std_logic_vector(5 downto 0);
  signal ph1_fd                 :     std_logic_vector(4 downto 0);
  signal ph2_fd                 :     std_logic_vector(4 downto 0);
  signal prop_fd                :     std_logic_vector(5 downto 0);
   --Tripple sampling for normal data rate
  signal sam_norm               :     std_logic;             
  
  --Error treshold registers
  signal ewl                    :     std_logic_vector(7 downto 0);
  signal erp                    :     std_logic_vector(7 downto 0);
  
  --Error counter preset registers
  signal erctr_pres_value       :     std_logic_vector(8 downto 0);
  signal erctr_pres_mask        :     std_logic_vector(3 downto 0);
  
  --Message filters
  signal filter_A_mask          :     std_logic_vector(28 downto 0);
  signal filter_B_mask          :     std_logic_vector(28 downto 0);
  signal filter_C_mask          :     std_logic_vector(28 downto 0);
  signal filter_A_value         :     std_logic_vector(28 downto 0);
  signal filter_B_value         :     std_logic_vector(28 downto 0);
  signal filter_C_value         :     std_logic_vector(28 downto 0);
  signal filter_ran_low         :     std_logic_vector(28 downto 0); 
  signal filter_ran_high        :     std_logic_vector(28 downto 0);
  signal filter_A_ctrl          :     std_logic_vector(3 downto 0);
  signal filter_B_ctrl          :     std_logic_vector(3 downto 0);
  signal filter_C_ctrl          :     std_logic_vector(3 downto 0);
  signal filter_ran_ctrl        :     std_logic_vector(3 downto 0);
  
  --RX Buffer control signals
  -- Transition from logic 1 to logic zero on this signal 
  -- causes rx_reading_pointer increment by one
  signal rx_read_start          :     std_logic; 
  
  --TX Arbitrator allow
  signal txt1_arbit_allow       :     std_logic;
  signal txt2_arbit_allow       :     std_logic;
  
  --Transcieve data registers
  type tx_data_memory_type is array (1 to 5) of std_logic_vector(127 downto 0);
  signal tx_data_reg            :     tx_data_memory_type;
  
  
  signal intLoopbackEna         :     std_logic;
  
  --Event logger registers
  signal log_trig_config        :     std_logic_vector(31 downto 0);
  signal log_capt_config        :     std_logic_vector(31 downto 0);
  signal log_cmd                :     std_logic_vector(3 downto 0);
  
  signal txt1_commit            :     std_logic;
  signal txt2_commit            :     std_logic;
  
  --Recieve transcieve message counters
  signal rx_ctr_set             :     std_logic;
  signal tx_ctr_set             :     std_logic;
  signal ctr_val_set            :     std_logic_vector(31 downto 0);
  
  --Enable of whole CAN controller
  signal CAN_enable             :     std_logic;
  
  --Type of FD controller used (ISO CAN FD or FD1.0)
  signal FD_type                :     std_logic; 
  
  --------------------
  --Memory registers--
  --------------------
  signal mode_reg               :     std_logic_vector(5 downto 0);   --Mode register
  signal status_reg             :     std_logic_vector(7 downto 0);   --Status Register
  signal int_ena_reg            :     std_logic_vector(10 downto 0);  --Interrupt Enable register
  
  --Auxiliarly signals
  signal PC_state               :     protocol_type;
  signal PC_state_reg_vect      :     std_logic_vector(6 downto 0);
  
  --Reading from RX buffer, detection of first cycle to move the pointer
  signal RX_buff_read_first     :     boolean;
  signal aux_data               :     std_logic_Vector(31 downto 0);
  
end entity;


architecture rtl of registers is

  --------------------------------------------
  -- Default value assignment to registers --
  -------------------------------------------
  procedure reg_reset (
    signal int_reset              :out  std_logic;
    signal clear_overrun          :out  std_logic;
    signal release_recieve        :out  std_logic;
    signal abort_transmittion     :out  std_logic;
    signal ack_forb               :out  std_logic;
    signal retr_lim_ena           :out  std_logic;                      --Retransmit limited is enabled
    signal retr_lim_th            :out  std_logic_vector(3 downto 0);   --Retransmit treshold
    signal interrupt_vector_erase :out  std_logic;
    signal sjw_norm               :out  std_logic_vector(3 downto 0);
    signal brp_norm               :out  std_logic_vector(5 downto 0);
    signal ph1_norm               :out  std_logic_vector(4 downto 0);
    signal ph2_norm               :out  std_logic_vector(4 downto 0);
    signal prop_norm              :out  std_logic_vector(5 downto 0);
    signal sjw_fd                 :out  std_logic_vector(3 downto 0);
    signal brp_fd                 :out  std_logic_vector(5 downto 0);
    signal ph1_fd                 :out  std_logic_vector(4 downto 0);
    signal ph2_fd                 :out  std_logic_vector(4 downto 0);
    signal prop_fd                :out  std_logic_vector(5 downto 0);
    signal sam_norm               :out  std_logic;             
    signal ewl                    :out  std_logic_vector(7 downto 0);
    signal erp                    :out  std_logic_vector(7 downto 0);
    signal erctr_pres_value       :out  std_logic_vector(8 downto 0);
    signal erctr_pres_mask        :out  std_logic_vector(3 downto 0);
    signal filter_A_mask          :out  std_logic_vector(28 downto 0);
    signal filter_B_mask          :out  std_logic_vector(28 downto 0);
    signal filter_C_mask          :out  std_logic_vector(28 downto 0);
    signal filter_A_value         :out  std_logic_vector(28 downto 0);
    signal filter_B_value         :out  std_logic_vector(28 downto 0);
    signal filter_C_value         :out  std_logic_vector(28 downto 0);
    signal filter_ran_low         :out  std_logic_vector(28 downto 0); 
    signal filter_ran_high        :out  std_logic_vector(28 downto 0);
    signal filter_A_ctrl          :out  std_logic_vector(3 downto 0);
    signal filter_B_ctrl          :out  std_logic_vector(3 downto 0);
    signal filter_C_ctrl          :out  std_logic_vector(3 downto 0);
    signal filter_ran_ctrl        :out  std_logic_vector(3 downto 0);
    signal txt1_arbit_allow       :out  std_logic;
    signal txt2_arbit_allow       :out  std_logic;
    signal intLoopbackEna         :out  std_logic;
    signal log_trig_config        :out  std_logic_vector(31 downto 0);
    signal log_capt_config        :out  std_logic_vector(31 downto 0);
    signal log_cmd                :out  std_logic_vector(3 downto 0);    
    signal txt1_commit            :out  std_logic;
    signal txt2_commit            :out  std_logic;
    signal rx_ctr_set             :out  std_logic;
    signal tx_ctr_set             :out  std_logic;
    signal ctr_val_set            :out  std_logic_vector(31 downto 0);
    signal CAN_enable             :out  std_logic;
    signal FD_type                :out  std_logic; 
    signal mode_reg               :out  std_logic_vector(5 downto 0);   
    signal int_ena_reg            :out  std_logic_vector(10 downto 0)
  ) is
  begin
    
    --Command registers
    clear_overrun           <=  NO_ACTION;
    release_recieve         <=  NO_ACTION;
    abort_transmittion      <=  NO_ACTION;
    interrupt_vector_erase  <=  NO_ACTION;
    
    erctr_pres_value        <=  (OTHERS=>'0');
    erctr_pres_mask         <=  (OTHERS=>'0');
    txt1_commit             <=  NO_ACTION;
    txt2_commit             <=  NO_ACTION;
    ctr_val_set             <=  (OTHERS =>'0');
    rx_ctr_set              <=  NO_ACTION;
    tx_ctr_set              <=  NO_ACTION;
    ack_forb                <=  ACK_ALLOWED;
    intLoopbackEna          <=  LOOPBACK_DIS;
   
    --Enable register
    CAN_enable              <=  DISABLED;
    FD_type                 <=  ISO_FD;
    
    --Mode register 
    mode_reg(RST_IND)       <=  NO_ACTION;
    mode_reg(LOM_IND)       <=  DISABLED;   --Listen only mod
    mode_reg(STM_IND)       <=  DISABLED;   --Self test mode
    mode_reg(AFM_IND)       <=  DISABLED;   --Acceptance filters mode 
    mode_reg(FDE_IND)       <=  ENABLED;    --Flexible datarate enable
    mode_reg(RTR_PREF_IND)  <=  ENABLED;    --RTR Preffered behaviour   
    
    --Interrupt enable register
    int_ena_reg(RI_IND)     <=  DISABLED;
    int_ena_reg(TI_IND)     <=  DISABLED;
    int_ena_reg(EI_IND)     <=  ENABLED;
    int_ena_reg(DOI_IND)    <=  ENABLED;
    int_ena_reg(4)          <=  DISABLED;
    int_ena_reg(EPI_IND)    <=  ENABLED;
    int_ena_reg(ALI_IND)    <=  DISABLED;
    int_ena_reg(BEI_IND)    <=  DISABLED;
    int_ena_reg(LFI_IND)    <=  DISABLED;
    int_ena_reg(RFI_IND)    <=  DISABLED;
    int_ena_reg(BSI_IND)    <=  DISABLED;      
    
    --Retransmitt limit enable
    retr_lim_ena            <=  RETR_LIM_DIS;
    retr_lim_th             <=  (OTHERS=>'0'); --Retr. limit treshold zeroes   
    
    sjw_norm                <=  "0010";         --2
    brp_norm                <=  "001010";       --10
    ph1_norm                <=  "00011";        --3
    ph2_norm                <=  "00101";        --5 
    prop_norm               <=  "000101";       --5
    
    sjw_fd                  <=  "0010";         --2
    brp_fd                  <=  "000100";       --4
    ph1_fd                  <=  "00011";        --3
    ph2_fd                  <=  "00011";        --3
    prop_fd                 <=  "000011";       --3
    
    sam_norm                <=  SINGLE_SAMPLING;
    ewl                     <=  std_logic_vector(to_unsigned(96,ewl'length));
    erp                     <=  std_logic_vector(to_unsigned(128,erp'length));
    
    --Message filters
    filter_A_mask           <=  (OTHERS=>'0');
    filter_B_mask           <=  (OTHERS=>'0');
    filter_C_mask           <=  (OTHERS=>'0');
    filter_A_value          <=  (OTHERS=>'0');
    filter_B_value          <=  (OTHERS=>'0');
    filter_C_value          <=  (OTHERS=>'0');
    filter_ran_low          <=  (OTHERS=>'0');
    filter_ran_high         <=  (OTHERS=>'0'); 
    filter_A_ctrl           <=  (OTHERS=>'1'); --Only filter A is enabled to pass all message types with any identifier
    filter_B_ctrl           <=  (OTHERS=>'0');
    filter_C_ctrl           <=  (OTHERS=>'0');
    filter_ran_ctrl         <=  (OTHERS=>'0');
    
    txt1_arbit_allow        <=  ALLOW_BUFFER;
    txt2_arbit_allow        <=  ALLOW_BUFFER;
    
    log_cmd                 <=  (OTHERS =>'0');
    log_trig_config         <=  (OTHERS =>'0');
    log_capt_config         <=  (OTHERS =>'0');
  end procedure;
  


begin
  
  --------------------------------------------------------
  --Reset propagation to output
  --Note: this works only for reset active in logic zero
  --------------------------------------------------------
  res_out                   <=  res_n and int_reset; 
  --data_out                  <=  (OTHERS=>'0');
 
  
  --------------------------------------------------------
  -- Main memory access process
  --------------------------------------------------------
  mem_acess:process(clk_sys,res_n,int_reset)
  begin
  if(res_n=ACT_RESET)then
      
      --Internal synced reset
      int_reset               <=  '1';
      data_out                <=  (OTHERS=>'0');
      
      --Reset the rest of registers
      reg_reset (
       int_reset         ,clear_overrun        ,release_recieve         ,abort_transmittion    ,ack_forb        ,
       retr_lim_ena      ,retr_lim_th          ,interrupt_vector_erase  ,sjw_norm              ,brp_norm        ,            
       ph1_norm          ,ph2_norm             ,prop_norm               ,sjw_fd                ,brp_fd          ,             
       ph1_fd            ,ph2_fd               ,prop_fd                 ,sam_norm              ,ewl             ,            
       erp               ,erctr_pres_value     ,erctr_pres_mask         ,filter_A_mask         ,filter_B_mask   ,         
       filter_C_mask     ,filter_A_value       ,filter_B_value          ,filter_C_value        ,filter_ran_low  ,        
       filter_ran_high   ,filter_A_ctrl        ,filter_B_ctrl           ,filter_C_ctrl         ,filter_ran_ctrl ,        
       txt1_arbit_allow     ,txt2_arbit_allow        ,intLoopbackEna  ,        
       log_trig_config   ,log_capt_config      ,log_cmd                 ,txt1_commit           ,txt2_commit     ,         
       rx_ctr_set        ,tx_ctr_set           ,ctr_val_set             ,CAN_enable            ,FD_type         ,         
       mode_reg          ,int_ena_reg            
      );
      
      tx_data_reg(5)        <=  (OTHERS=>'0');
      tx_data_reg(4)        <=  (OTHERS=>'0');
      tx_data_reg(3)        <=  (OTHERS=>'0');
      tx_data_reg(2)        <=  (OTHERS=>'0');
      tx_data_reg(1)        <=  (OTHERS=>'0');
      
      RX_buff_read_first    <= false;
      aux_data              <=  (OTHERS=>'0');
      
  elsif rising_edge(clk_sys)then
	if(int_reset=ACT_RESET)then --Synchronous reset
		
  		  --Internal synced reset
  		  int_reset               <=  not ACT_RESET;
  	   data_out                <=  (OTHERS=>'0');
  	   
  	   --Reset of the other registers
 	    reg_reset (
         int_reset         ,clear_overrun        ,release_recieve         ,abort_transmittion    ,ack_forb        ,
         retr_lim_ena      ,retr_lim_th          ,interrupt_vector_erase  ,sjw_norm              ,brp_norm        ,            
         ph1_norm          ,ph2_norm             ,prop_norm               ,sjw_fd                ,brp_fd          ,             
         ph1_fd            ,ph2_fd               ,prop_fd                 ,sam_norm              ,ewl             ,            
         erp               ,erctr_pres_value     ,erctr_pres_mask         ,filter_A_mask         ,filter_B_mask   ,         
         filter_C_mask     ,filter_A_value       ,filter_B_value          ,filter_C_value        ,filter_ran_low  ,        
         filter_ran_high   ,filter_A_ctrl        ,filter_B_ctrl           ,filter_C_ctrl         ,filter_ran_ctrl ,        
         txt1_arbit_allow     ,txt2_arbit_allow        ,intLoopbackEna  ,        
         log_trig_config   ,log_capt_config      ,log_cmd                 ,txt1_commit           ,txt2_commit     ,         
         rx_ctr_set        ,tx_ctr_set           ,ctr_val_set             ,CAN_enable            ,FD_type         ,         
         mode_reg          ,int_ena_reg            
       ) ;
       
      tx_data_reg(5)        <=  (OTHERS=>'0');
      tx_data_reg(4)        <=  (OTHERS=>'0');
      tx_data_reg(3)        <=  (OTHERS=>'0');
      tx_data_reg(2)        <=  (OTHERS=>'0');
      tx_data_reg(1)        <=  (OTHERS=>'0');
      
      RX_buff_read_first    <= false;
      aux_data              <=  (OTHERS=>'0');

	else
		--Internal registers holding its value
		filter_A_mask             <=  filter_A_mask;
		filter_B_mask             <=  filter_B_mask;
		filter_C_mask             <=  filter_C_mask;
		filter_A_value            <=  filter_A_value;
		filter_B_value            <=  filter_B_value;
		filter_C_value            <=  filter_C_value;
		filter_ran_low            <=  filter_ran_low;
		filter_ran_high           <=  filter_ran_high;
		filter_A_ctrl             <=  filter_A_ctrl;
		filter_B_ctrl             <=  filter_B_ctrl;
		filter_C_ctrl             <=  filter_C_ctrl;
		filter_ran_ctrl           <=  filter_ran_ctrl;
		
		int_ena_reg               <=  int_ena_reg;
		retr_lim_ena              <=  retr_lim_ena;
		retr_lim_th               <=  retr_lim_th;
		sjw_norm                  <=  sjw_norm;
		brp_norm                  <=  brp_norm;
		ph1_norm                  <=  ph1_norm;
		ph2_norm                  <=  ph2_norm;
		prop_norm                 <=  prop_norm;
		sjw_fd                    <=  sjw_fd;
		brp_fd                    <=  brp_fd;
		ph1_fd                    <=  ph1_fd;
		ph2_fd                    <=  ph2_fd;
		prop_fd                   <=  prop_fd;
		sam_norm                  <=  sam_norm;
		ewl                       <=  ewl;
		erp                       <=  erp;
		mode_reg                  <=  mode_reg;
		intLoopbackEna            <=  intLoopbackEna;
		log_trig_config           <=  log_trig_config;
		log_capt_config           <=  log_capt_config;
		
		--Internal registers manipulation
		int_reset                 <=  not ACT_RESET;
		clear_overrun             <=  '0';
		release_recieve           <=  '0';
		abort_transmittion        <=  '0';
		interrupt_vector_erase    <=  '0';
		erctr_pres_value          <=  (OTHERS=>'0');
		erctr_pres_mask           <=  "0000";
		txt2_commit               <=  '0';
		txt1_commit               <=  '0';
		ctr_val_set               <=  (OTHERS =>'0');
		rx_ctr_set                <=  '0';
		tx_ctr_set                <=  '0';
		ack_forb                  <=  ack_forb;
		data_out                  <=  (OTHERS=>'0');
		log_cmd                   <=  (OTHERS =>'0');
		
		RX_buff_read_first        <= false;
		aux_data                  <=  (OTHERS=>'0');
		
		--Chip select active and our device is selected (Component type and ID)
		if((scs=ACT_CSC) and 
		   (adress(COMP_TYPE_ADRESS_HIGHER downto COMP_TYPE_ADRESS_LOWER)=CAN_COMPONENT_TYPE) and 
		   (adress(ID_ADRESS_HIGHER downto ID_ADRESS_LOWER)=std_logic_vector(to_unsigned(ID,4))) )then 
		  
		  --------------------
		  --Writing the data--
		  --------------------
		  if(swr=ACT_SWR)then 
    			case adress(13 downto 2) is
    			
    			---------------------------------------------------------  
    			--MODE Register (Mode, Command, Status as in SJA1000)
    			---------------------------------------------------------   
    			when MODE_REG_ADR =>    
    			    
    			     --Mode register
    					  mode_reg(5 downto 1)     <=  data_in(5 downto 1);  --RTR_PREF,FDE,AFM,STM,LOM Bits
    					  sam_norm                 <=  data_in(6);           --Tripple sampling 
    					  ack_forb                 <=  data_in(7);           --Acknowledge forbidden
    					  if(data_in(0)='1')then                             --Reset by memory access
    					   int_reset               <=  ACT_RESET; 
    					  end if;      
    					            
    					  --Command register
    					  clear_overrun            <=  data_in(11);
    					  release_recieve          <=  data_in(10);
    					  abort_transmittion       <=  data_in(9);
    					  
    					  --Status register is read only!
    					  
    					  --Retransmitt limit register
    					  retr_lim_ena             <=  data_in(24);
    					  retr_lim_th              <=  data_in(28 downto 25);
    					  intLoopbackEna           <=  data_in(29);
    					  CAN_enable               <=  data_in(30);
    					  FD_type                  <=  data_in(31);
    			
			 ------------------------------------------------------------	  
 			 --INT_REG (Interrupt register, Interrupt enable register)
 			 ------------------------------------------------------------
    			when INTERRUPT_REG_ADR =>               
    					   int_ena_reg(10 downto 0)<=  data_in(26 downto 16);--Interrupt enable register
    					   --Interrupt register (interrupt vector) is read only! (By read it is also erased)  
 			 
 			 ----------------------
 			 --Bit timing register
 			 -----------------------        
    			when TIMING_REG_ADR => 
    					   prop_norm               <=  data_in(5 downto 0);
    					   ph1_norm                <=  data_in(10 downto 6);
    					   ph2_norm                <=  data_in(15 downto 11);
    					   prop_fd                 <=  data_in(21 downto 16);
    					   ph1_fd                  <=  data_in(26 downto 22);
    					   ph2_fd                  <=  data_in(31 downto 27);
			
			 ----------------------------------------------------
    			--Arbitration lost capture and error code Capture  
    			--and Baud Rate prescaler		 
    			----------------------------------------------------
    			when ARB_ERROR_PRESC_ADR => 
    					   --Arbitration lost, Error code are read only
    					   
    					    --Baud rate prescaler register
    					   brp_norm                <=  data_in(21 downto 16);
    					   sjw_norm                <=  data_in(11 downto 8);
    					   brp_fd                  <=  data_in(29 downto 24);
    					   sjw_fd                  <=  data_in(15 downto 12); 
    			
    			---------------------------------------------------- 
  			 --Error warning limit, error passive treshold	
  			 ----------------------------------------------------	   
    			when ERROR_TH_ADR =>
    					   ewl                     <=  data_in(7 downto 0); --Error warning limit 
    					   erp                     <=  data_in(15 downto 8); --Error passive treshold
    			
    			----------------------------------------------------	   
    			--Error counters, presetting
    			----------------------------------------------------	   
    			when ERROR_COUNTERS_ADR => 
    					  erctr_pres_value         <=  data_in(8 downto 0);
    					  erctr_pres_mask          <=  "00"&data_in(10 downto 9);
    			
    			----------------------------------------------------	   
    			--Special error counters. Only erasable!
    			----------------------------------------------------	
    			when ERROR_COUNTERS_SPEC_ADR => 
    					  erctr_pres_value         <=  (OTHERS=>'0');
    					  erctr_pres_mask          <=  data_in(12 downto 11)&"00"; 
    		
    		 ----------------------------------------------------	   
    			--Acceptance filters
    			----------------------------------------------------	
    			when FILTER_A_VAL_ADR    => filter_A_mask    <=  data_in(28 downto 0);
    			when FILTER_A_MASK_ADR   => filter_A_value   <=  data_in(28 downto 0);     
    			when FILTER_B_VAL_ADR    => filter_B_mask    <=  data_in(28 downto 0);     
    			when FILTER_B_MASK_ADR   => filter_B_value   <=  data_in(28 downto 0);     
    			when FILTER_C_VAL_ADR    => filter_C_mask    <=  data_in(28 downto 0);     
    			when FILTER_C_MASK_ADR   => filter_C_value   <=  data_in(28 downto 0);     
    			when FILTER_RAN_LOW_ADR  => filter_ran_low   <=  data_in(28 downto 0);
    			when FILTER_RAN_HIGH_ADR => filter_ran_high  <=  data_in(28 downto 0);
    			when FILTER_CONTROL_ADR  => 
    					  filter_A_ctrl            <=  data_in(3 downto 0);
    					  filter_B_ctrl            <=  data_in(7 downto 4);
    					  filter_C_ctrl            <=  data_in(11 downto 8);
    					  filter_ran_ctrl          <=  data_in(15 downto 12);
    			
    			----------------------------------------------------
    			--TX Settings register
    			----------------------------------------------------
    			when TX_SETTINGS_ADR => 
    					  txt1_arbit_allow         <=  data_in(0);
    					  txt2_arbit_allow         <=  data_in(1);
    					  txt1_commit              <=  data_in(2); 
    					  txt2_commit              <=  data_in(3); 
    					  
    			----------------------------------------------------
    			--TX Data registers
    			----------------------------------------------------		  
    			when TX_DATA_1_ADR => tx_data_reg(5)(127 downto 96)    <=  data_in;
    			when TX_DATA_2_ADR => tx_data_reg(5)(95 downto 64)     <=  data_in;
    			when TX_DATA_3_ADR => tx_data_reg(5)(63 downto 32)     <=  data_in;
    			when TX_DATA_4_ADR => tx_data_reg(5)(31 downto 0)      <=  data_in;
    			when TX_DATA_5_ADR => tx_data_reg(4)(127 downto 96)    <=  data_in;
    			when TX_DATA_6_ADR => tx_data_reg(4)(95 downto 64)     <=  data_in;
    			when TX_DATA_7_ADR => tx_data_reg(4)(63 downto 32)     <=  data_in;
    			when TX_DATA_8_ADR => tx_data_reg(4)(31 downto 0)      <=  data_in;
    			when TX_DATA_9_ADR => tx_data_reg(3)(127 downto 96)    <=  data_in;
    			when TX_DATA_10_ADR => tx_data_reg(3)(95 downto 64)    <=  data_in;
    			when TX_DATA_11_ADR => tx_data_reg(3)(63 downto 32)    <=  data_in;
    			when TX_DATA_12_ADR => tx_data_reg(3)(31 downto 0)     <=  data_in;
    			when TX_DATA_13_ADR => tx_data_reg(2)(127 downto 96)   <=  data_in;
    			when TX_DATA_14_ADR => tx_data_reg(2)(95 downto 64)    <=  data_in;
    			when TX_DATA_15_ADR => tx_data_reg(2)(63 downto 32)    <=  data_in;
    			when TX_DATA_16_ADR => tx_data_reg(2)(31 downto 0)     <=  data_in;
    			when TX_DATA_17_ADR => tx_data_reg(1)(127 downto 96)   <=  data_in;
    			when TX_DATA_18_ADR => tx_data_reg(1)(95 downto 64)    <=  data_in;
    			when TX_DATA_19_ADR => tx_data_reg(1)(63 downto 32)    <=  data_in;
    			when TX_DATA_20_ADR => tx_data_reg(1)(31 downto 0)     <=  data_in;
    			
    			--------------------------------------
    			--Recieve frame counter presetting
    			--------------------------------------
    			when RX_COUNTER_ADR => 
    					  ctr_val_set                <=  data_in;
    					  rx_ctr_set                 <=  '1';
    					  
    			--------------------------------------
    			--Transcieve frame counter presetting
    			--------------------------------------		  
    			when TX_COUNTER_ADR => 
    					  ctr_val_set                <=  data_in;
    					  tx_ctr_set                 <=  '1';
    			
    			--------------------------------------
    			--Logger configuration registers
    			--------------------------------------		  
    			when LOG_TRIG_CONFIG_ADR=>
    					  log_trig_config            <=  data_in;
    			when LOG_CAPT_CONFIG_ADR=>
    					 log_capt_config             <=  data_in;
    			when LOG_CMD_ADR => 
    			    --LOG_DOWN,LOG_UP,LOG_ABT,LOG_STR
    					 log_cmd                     <=  data_in(3 downto 0);       
    			when others =>
    			  
    			end case;
		  end if;
		  
		  --------------------
		  --Reading the data--
		  --------------------
		  if(srd=ACT_SRD)then 
    			case adress(13 downto 2) is
    			  
    			  --------------------------------------
    			  --Device ID
			   --------------------------------------	    			  
  			   when DEVICE_ID_ADR =>     
  			       data_out                  <=  CAN_DEVICE_ID;
  			     
  			   --------------------------------------
    			  --MODE Register (Mode, Command, Status of SJA1000)
			   --------------------------------------  
    			  when MODE_REG_ADR => 
    			     
    			     --Mode register
    					  data_out                   <=  (OTHERS=>'0');
    					  data_out(6)                <=  sam_norm;
    					  data_out(7)                <=  ack_forb;
    					  data_out(5 downto 1)       <=  mode_reg(5 downto 1);
    					  
    					  --Command register is write only!
    					  --(Read in these bytes wont return previous value)
    					  
    					  --Status register
    					  data_out(23 downto 16)     <=  status_reg;
    					  
    					  --Retransmitt limit register
    					  data_out(24)               <=  retr_lim_ena;
    					  data_out(28 downto 25)     <=  retr_lim_th;
    					  data_out(29)               <=  intLoopbackEna;
 				    data_out(30)    					      <=  CAN_enable;
    					  data_out(31)               <=  FD_type;
    					  
 				 ---------------------------------------------------------
 				 --INT_REG (Interrupt register, Interrupt enable register)
 				 ---------------------------------------------------------
    			  when INTERRUPT_REG_ADR => 
    					  data_out                   <=  (OTHERS=>'0');
    					  data_out(26 downto 16)     <=  int_ena_reg(10 downto 0);--Interrupt enable register
    					  data_out(10 downto 0)      <=  int_vector; --Interrupt register
    					  interrupt_vector_erase     <=  '1'; --By reading interrupt vector it is erased
    			  
    			  ---------------------------------------------------------
 				 --Bit timing registers
    			  ---------------------------------------------------------
 				 when TIMING_REG_ADR => 
    					   data_out(5 downto 0)      <=  prop_norm;
    					   data_out(10 downto 6)     <=  ph1_norm;
    					   data_out(15 downto 11)    <=  ph2_norm;
    					   data_out(21 downto 16)    <=  prop_fd;
    					   data_out(26 downto 22)    <=  ph1_fd;
    					   data_out(31 downto 27)    <=  ph2_fd;
    			  
    			  ----------------------------------------------------------
    			  --Arbitration lost capture register
         -- Baud rate prescaler register
         ----------------------------------------------------------
    			  when ARB_ERROR_PRESC_ADR =>
    					   data_out                  <=  (OTHERS =>'0');
    					   data_out(4 downto 0)      <=  stat_bus(STAT_ALC_HIGH downto STAT_ALC_LOW); 
    					   --TODO : Error code capture (data out 15 to 0)
    					   data_out(21 downto 16)    <=  brp_norm; --Baud rate prescaler register
    					   data_out(11 downto 8)    <=  sjw_norm;
    					   data_out(29 downto 24)    <=  brp_fd;
    					   data_out(15 downto 12)    <=  sjw_fd;
    					   
				 ----------------------------------------------------------
    			  --Error warning limit, error passive treshold
         -- Fault confinement state
         ----------------------------------------------------------
    			   when ERROR_TH_ADR =>
    					  data_out(7 downto 0)       <=  ewl; --Error warning limit 
    					  data_out(15 downto 8)      <=  erp; --Error passive treshold
    					  
    					  --Fault confinment state
    					   if(error_state_type'VAL(to_integer(unsigned(stat_bus(STAT_ERROR_STATE_HIGH downto STAT_ERROR_STATE_LOW))))=error_active)then
    					     data_out(16)            <=  '1'; 
					   else 
					     data_out(16)            <=  '0'; 
					   end if;
					   
    					   if(error_state_type'VAL(to_integer(unsigned(stat_bus(STAT_ERROR_STATE_HIGH downto STAT_ERROR_STATE_LOW))))=error_passive)then
    					     data_out(17)            <=  '1';
					   else 
					     data_out(17)            <=  '0';
					   end if;
    					   
    					   if(error_state_type'VAL(to_integer(unsigned(stat_bus(STAT_ERROR_STATE_HIGH downto STAT_ERROR_STATE_LOW))))=bus_off)then
    					     data_out(18)            <=  '1'; 
					   else 
					     data_out(18)            <=  '0';
					    end if;  
    					  
    					   data_out(31 downto 19)<=(OTHERS=>'0');
    			   
    			   ----------------------------------------------------------
    			   --Error counters (NORMAL)
    			   ----------------------------------------------------------
    			   when ERROR_COUNTERS_ADR => 
    					  data_out                   <=  (OTHERS=>'0');
    					  data_out(8 downto 0)       <=  stat_bus(STAT_RX_COUNTER_HIGH downto STAT_RX_COUNTER_LOW);
    					  data_out(24 downto 16)     <=  stat_bus(STAT_TX_COUNTER_HIGH downto STAT_TX_COUNTER_LOW);
    					
    					----------------------------------------------------------  
    					--Error counters special  
    					----------------------------------------------------------
    			   when ERROR_COUNTERS_SPEC_ADR => 
    					  data_out                   <=  (OTHERS=>'0');
    					  data_out(15 downto 0)      <=  stat_bus(STAT_ERROR_COUNTER_NORM_HIGH downto STAT_ERROR_COUNTER_NORM_LOW);
    					  data_out(31 downto 16)     <=  stat_bus(STAT_ERROR_COUNTER_FD_HIGH downto STAT_ERROR_COUNTER_FD_LOW);
    			   
    			   ----------------------------------------------------------  
    					--Acceptance filters  
    					----------------------------------------------------------
    			   when FILTER_A_VAL_ADR => 
    			     data_out(28 downto 0)       <=  filter_A_mask;
						data_out(31 downto 29)      <=  (OTHERS=>'0');
    			   when FILTER_A_MASK_ADR => 
    			     data_out(28 downto 0)       <=  filter_A_value;
						data_out(31 downto 29)      <=  (OTHERS=>'0');     
    			   when FILTER_B_VAL_ADR => 
    			     data_out(28 downto 0)       <=  filter_B_mask;
    						data_out(31 downto 29)      <=  (OTHERS=>'0');
    			   when FILTER_B_MASK_ADR => 
    			     data_out(28 downto 0)       <=  filter_B_value;
    						data_out(31 downto 29)      <=  (OTHERS=>'0');   
    			   when FILTER_C_VAL_ADR =>
    			     data_out(28 downto 0)       <=  filter_C_mask;
    						data_out(31 downto 29)      <=  (OTHERS=>'0');    
    			   when FILTER_C_MASK_ADR =>
    			     data_out(28 downto 0)       <=  filter_C_value;
    						data_out(31 downto 29)      <=  (OTHERS=>'0');     
    			   when FILTER_RAN_LOW_ADR =>
    			     data_out(28 downto 0)       <=  filter_ran_low;
    						data_out(31 downto 29)      <=  (OTHERS=>'0');
    			   when FILTER_RAN_HIGH_ADR =>
    			     data_out(28 downto 0)       <=  filter_ran_high;
    						data_out(31 downto 29)      <=  (OTHERS=>'0');
    					
    					-------------------------------------------------------	
					--Acceptance filter configuration register
					-------------------------------------------------------
    			   when FILTER_CONTROL_ADR => 
    					  data_out(3 downto 0)       <=  filter_A_ctrl;
    					  data_out(7 downto 4)       <=  filter_B_ctrl;
    					  data_out(11 downto 8)      <=  filter_B_ctrl;
    					  data_out(15 downto 12)     <=  filter_ran_ctrl;
    					  data_out(31 downto 16)     <=  (OTHERS=>'0');
    					
    					-------------------------------------------------------
    					--RX_INFO_1 register
    					-------------------------------------------------------  
    			   when RX_INFO_1_ADR => 
    					  data_out(31 downto 0)      <=  (OTHERS=>'0');
    					  data_out(0)                <=  rx_empty;
    					  data_out(1)                <=  rx_full;
    					  data_out(15 downto 8)      <=  rx_message_count;
    					  data_out(23 downto 16)     <=  rx_mem_free;
    			   
    			   -------------------------------------------------------
    					--RX_INFO_2 register
    					-------------------------------------------------------  
    			   when RX_INFO_2_ADR =>
    					  data_out(31 downto 0)      <=  (OTHERS=>'0');
    					  data_out(7 downto 0)       <=  rx_buf_size;
    					  data_out(15 downto 8)      <=  rx_write_pointer_pos;
    					  data_out(23 downto 16)     <=  rx_read_pointer_pos;
 					
 					-------------------------------------------------------
 					--RX_DATA register
 					-------------------------------------------------------
    			   when RX_DATA_ADR => 
    			     if(RX_buff_read_first=false)then
    					   data_out(31 downto 0)      <=  rx_read_buff;
    					   aux_data                   <=  rx_read_buff;
					  else
					   data_out(31 downto 0)      <=  aux_data;
					  end if;
					  
    					  RX_buff_read_first         <=  true;
    			   
    			   -------------------------------------------------------
   			    --Transciever delay adress  
    			   -------------------------------------------------------
    			   when TRV_DELAY_ADR =>
    			      data_out(31 downto 16)     <=  (OTHERS=>'0');
    			      data_out(15 downto 0)      <=  trv_delay_out; 
 			    
 			    -------------------------------------------------------
 			    --TXT Buffers status
    			   -------------------------------------------------------
    			   when TX_STATUS_ADR => 
    			      data_out(31 downto 2)      <=  (OTHERS=>'0');
    			      data_out(1)                <=  txt2_empty;
    			      data_out(0)                <=  txt1_empty;
    			      
 			    ------------------------------------------------------- 
 			    --TX_Settings register
 			    -------------------------------------------------------
    			   when TX_SETTINGS_ADR => 
    					  data_out                     <=  (OTHERS =>'0');
    					  data_out(0)                  <=  txt1_arbit_allow;
    					  data_out(1)                  <=  txt2_arbit_allow;
    					
    					------------------------------------------------------- 
 			    --Frame counters registers
 			    -------------------------------------------------------  
    			   when RX_COUNTER_ADR => --Recieve message counter 
    					  data_out                     <=  stat_bus(STAT_RX_CTR_HIGH downto STAT_RX_CTR_LOW);
    			   when TX_COUNTER_ADR => --Transcieve message counter 
    					  data_out                     <=  stat_bus(STAT_TX_CTR_HIGH downto STAT_TX_CTR_LOW);
    			   
    			   ------------------------------------------------------- 
 			    --Logger configuration registers
 			    -------------------------------------------------------  
    			   when LOG_TRIG_CONFIG_ADR=>
    					  data_out                     <=  log_trig_config;
    			   when LOG_CAPT_CONFIG_ADR=>
    					  data_out                     <=  log_capt_config;
    			   when LOG_STATUS_ADR=>
    					  --Logger status
    					  if(log_state_out=config)then 
    					     data_out(0)               <=  '1'; 
					  else 
					     data_out(0)               <=  '0'; 
					  end if;
					  
    					  if(log_state_out=ready)then 
    					     data_out(1)               <=  '1'; 
    					  else 
    					     data_out(1)               <=  '0'; 
    					  end if;
    					  
    					  if(log_state_out=running)then 
    					     data_out(2)               <=  '1'; 
    					  else 
    					     data_out(2)               <=  '0'; 
    					  end if;
    					  
    					  if(use_logger=true)then
    					     data_out(6)               <=  '1';
					  else
					     data_out(6)               <=  '0';
					  end if;       
    			
    					  data_out(6 downto 3)         <=  (OTHERS =>'0');
    					  data_out(15 downto 8)        <=  log_size;
    					  data_out(23 downto 16)       <=  log_write_pointer;
    					  data_out(31 downto 24)       <=  log_read_pointer;
    			   when LOG_CAPT_EVENT1_ADR=>
    					  data_out                     <=  loger_act_data(63 downto 32);
    			   when LOG_CAPT_EVENT2_ADR=>
    					  data_out                     <=  loger_act_data(31 downto 0);
    			      			   
 			   ------------------------------------------------------- 
 			   --DEBUG register
 			   -------------------------------------------------------  
  			    when DEBUG_REG_ADR =>
  			      data_out(7 downto 3)         <= (OTHERS =>'0');
  			      data_out(2 downto 0)         <= stat_bus(STAT_BS_CTR_HIGH downto STAT_BS_CTR_LOW);
  			      data_out(5 downto 3)         <= stat_bus(STAT_BD_CTR_HIGH downto STAT_BD_CTR_LOW);
    			     data_out(12 downto 6)        <= PC_state_reg_vect;
    			   
  			   ------------------------------------------------------- 
 			   --YOOOOLOOOO REGISTER
 			   ------------------------------------------------------- 
    				 when YOLO_REG_ADR =>
    				     data_out                    <=  std_logic_vector'(X"DEADBEEF");
    			   when others=>
    			  end case;    
		  end if;
		  
		end if;
	end if;
  end if;  
  end process mem_acess;
  
  --Combinational logic for incrementing read pointer in RX buffer!
 	rx_read_start         <=  '1' when (srd=ACT_SRD and 
 	                                    scs=ACT_CSC and   
		                                  adress(COMP_TYPE_ADRESS_HIGHER downto COMP_TYPE_ADRESS_LOWER)=CAN_COMPONENT_TYPE and 
		                                  adress(ID_ADRESS_HIGHER downto ID_ADRESS_LOWER)=std_logic_vector(to_unsigned(ID,4)) and
		                                  adress(13 downto 2)=RX_DATA_ADR and
		                                  RX_buff_read_first = false)
		                            else
		                         '0';
  
  
  --------------------------------
  --Register logic and structure--
  --------------------------------
  PC_state              <=  protocol_type'VAL(to_integer(unsigned(stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))));
 
  --Note: Flip flops are not used for most of the logic because all the blocks have registered information on output, therefore it is enough to read it directly!
  
  --Status register
  status_reg(BS_IND)<= '1' when  error_state_type'VAL(to_integer(unsigned(stat_bus(STAT_ERROR_STATE_HIGH downto STAT_ERROR_STATE_LOW))))=bus_off else 
                       '1' when  oper_mode_type'VAL(to_integer(unsigned(stat_bus(STAT_OP_STATE_HIGH downto STAT_OP_STATE_LOW))))=integrating else
                       '1' when  oper_mode_type'VAL(to_integer(unsigned(stat_bus(STAT_OP_STATE_HIGH downto STAT_OP_STATE_LOW))))=idle else
                       '0';
  status_reg(ES_IND)<='1' when (ewl<stat_bus(STAT_TX_COUNTER_HIGH downto STAT_TX_COUNTER_LOW) or
                                ewl<stat_bus(STAT_RX_COUNTER_HIGH downto STAT_RX_COUNTER_LOW)) else '0';
                                
  status_reg(TS_IND)<='1' when oper_mode_type'VAL(to_integer(unsigned(stat_bus(STAT_OP_STATE_HIGH downto STAT_OP_STATE_LOW))))=transciever else '0';
  status_reg(RS_IND)<='1' when oper_mode_type'VAL(to_integer(unsigned(stat_bus(STAT_OP_STATE_HIGH downto STAT_OP_STATE_LOW))))=reciever else '0';
  status_reg(TBS_IND)<='1' when (txt1_empty='0' and txt2_empty='0') else '0'; --When buffer is not full there is one. However still might be not enough place in buffer
  status_reg(RBS_IND)<=not rx_empty; --When at least one message is availiable in the buffer
  status_reg(DOS_IND)<=rx_data_overrun;
  status_reg(ET_IND) <='1' when PC_state=error else '0';
  
  tran_data_in<=tx_data_reg(5)&tx_data_reg(4)&tx_data_reg(3)&tx_data_reg(2)&tx_data_reg(1);
  
  --Debug register
  PC_state_reg_vect(0)    <= '1' when PC_State=arbitration else '0';
  PC_state_reg_vect(1)    <= '1' when PC_State=control else '0';
  PC_state_reg_vect(2)    <= '1' when PC_State=data else '0';
  PC_state_reg_vect(3)    <= '1' when PC_State=crc else '0';
  PC_state_reg_vect(4)    <= '1' when PC_State=eof else '0';
  PC_state_reg_vect(5)    <= '1' when PC_State=overload else '0';
  PC_state_reg_vect(6)    <= '1' when PC_State=interframe else '0';
  
  ---------------------------
  --Driving bus assignment --
  ---------------------------
  --Note:  All unused signals indices should be assigned to zero!
  drv_bus(80 downto 50)                             <=  (OTHERS=>'0');
  drv_bus(349 downto 330)                           <=  (OTHERS=>'0');
  drv_bus(351)                                      <=  '0';
  drv_bus(355 downto 354)                           <=  (OTHERS=>'0');
  drv_bus(360 downto 358)                           <=  (OTHERS=>'0');
  drv_bus(365 downto 363)                           <=  (OTHERS=>'0');
  drv_bus(370 downto 368)                           <=  (OTHERS=>'0');
  drv_bus(371)                                      <=  '0';
  drv_bus(375 downto 373)                           <=  (OTHERS=>'0');
  drv_bus(399 downto 388)                           <=  (OTHERS=>'0');
  drv_bus(459 downto 445)                           <=  (OTHERS=>'0');
  drv_bus(464 downto 462)                           <=  (OTHERS=>'0');
  drv_bus(1023 downto 614)                          <=  (OTHERS=>'0');
  drv_bus(609 downto 601)                           <=  (OTHERS=>'0');
  drv_bus(579 downto 570)                           <=  (OTHERS=>'0');
  drv_bus(519 downto 511)                           <=  (OTHERS=>'0');
  drv_bus(444 downto 429)                           <=  (OTHERS=>'0');
  
  --Prescaler data and bus timing
  drv_bus(DRV_TQ_NBT_HIGH downto DRV_TQ_NBT_LOW)    <=  brp_norm;
  drv_bus(DRV_TQ_DBT_HIGH downto DRV_TQ_DBT_LOW)    <=  brp_fd;
  drv_bus(DRV_PRS_NBT_HIGH downto DRV_PRS_NBT_LOW)  <=  prop_norm;
  drv_bus(DRV_PRS_DBT_HIGH downto DRV_PRS_DBT_LOW)  <=  prop_fd(3 downto 0);
  drv_bus(DRV_PH1_NBT_HIGH downto DRV_PH1_NBT_LOW)  <=  '0'&ph1_norm;
  drv_bus(DRV_PH1_DBT_HIGH downto DRV_PH1_DBT_LOW)  <=  ph1_fd(3 downto 0);
  drv_bus(DRV_PH2_NBT_HIGH downto DRV_PH2_NBT_LOW)  <=  '0'&ph2_norm;
  drv_bus(DRV_PH2_DBT_HIGH downto DRV_PH2_DBT_LOW)  <=  ph2_fd(3 downto 0);
  drv_bus(DRV_SJW_HIGH downto DRV_SJW_LOW)          <=  sjw_norm;
  drv_bus(DRV_SJW_DBT_HIGH downto DRV_SJW_DBT_LOW)  <=  sjw_fd;
  
  --Acceptance filters
  drv_bus(DRV_FILTERS_ENA_INDEX)                                      <=  mode_reg(AFM_IND);
  drv_bus(DRV_FILTER_A_MASK_HIGH downto DRV_FILTER_A_MASK_LOW)        <=  filter_A_mask;
  drv_bus(DRV_FILTER_A_BITS_HIGH downto DRV_FILTER_A_BITS_LOW)        <=  filter_A_value;
  drv_bus(DRV_FILTER_A_CTRL_HIGH downto DRV_FILTER_A_CTRL_LOW)        <=  filter_A_ctrl;
  drv_bus(DRV_FILTER_B_MASK_HIGH downto DRV_FILTER_B_MASK_LOW)        <=  filter_B_mask;
  drv_bus(DRV_FILTER_B_BITS_HIGH downto DRV_FILTER_B_BITS_LOW)        <=  filter_B_value;
  drv_bus(DRV_FILTER_B_CTRL_HIGH downto DRV_FILTER_B_CTRL_LOW)        <=  filter_B_ctrl;
  drv_bus(DRV_FILTER_C_MASK_HIGH downto DRV_FILTER_C_MASK_LOW)        <=  filter_C_mask;
  drv_bus(DRV_FILTER_C_BITS_HIGH downto DRV_FILTER_C_BITS_LOW)        <=  filter_C_value;
  drv_bus(DRV_FILTER_C_CTRL_HIGH downto DRV_FILTER_C_CTRL_LOW)        <=  filter_C_ctrl;
  drv_bus(DRV_FILTER_RAN_CTRL_HIGH downto DRV_FILTER_RAN_CTRL_LOW)    <=  filter_ran_ctrl;
  drv_bus(DRV_FILTER_RAN_LO_TH_HIGH downto DRV_FILTER_RAN_LO_TH_LOW)  <=  filter_ran_low;
  drv_bus(DRV_FILTER_RAN_HI_TH_HIGH downto DRV_FILTER_RAN_HI_TH_LOW)  <=  filter_ran_high;
  
  --Rx Buffer
  drv_bus(DRV_ERASE_RX_INDEX)                       <=  release_recieve;
  drv_bus(DRV_READ_START_INDEX)                     <=  rx_read_start;
  drv_bus(DRV_CLR_OVR_INDEX)                        <=  clear_overrun;
  
  --TXT Buffer and TX Buffer
  drv_bus(DRV_ERASE_TXT1_INDEX)                     <=  '0';--TODO: Add chance to erase also TXT Buffer
  drv_bus(DRV_STORE_TXT1_INDEX)                     <=  txt1_commit;
  
  drv_bus(DRV_STORE_TXT2_INDEX)                     <=  txt2_commit;
  drv_bus(DRV_ERASE_TXT2_INDEX)                     <=  '0'; --TODO: Add chance to erase also TX Buffer
  
  --TX Arbitrator
  drv_bus(DRV_ALLOW_TXT1_INDEX)                     <=  txt1_arbit_allow;
  drv_bus(DRV_ALLOW_TXT2_INDEX)                     <=  txt2_arbit_allow;
  
  --Tripple sampling
  drv_bus(DRV_SAM_INDEX)                            <=  sam_norm;
  
  --Interrupts
  drv_bus(DRV_BUS_ERR_INT_ENA_INDEX)                <=  int_ena_reg(BEI_IND);
  drv_bus(DRV_ARB_LST_INT_ENA_INDEX)                <=  int_ena_reg(ALI_IND);
  drv_bus(DRV_ERR_PAS_INT_ENA_INDEX)                <=  int_ena_reg(EPI_IND);
  drv_bus(DRV_WAKE_INT_ENA_INDEX)                   <=  '0'; --Wake interrupt is not supported, no sleep mode implemented
  drv_bus(DRV_DOV_INT_ENA_INDEX)                    <=  int_ena_reg(DOI_IND);
  drv_bus(DRV_ERR_WAR_INT_ENA_INDEX)                <=  int_ena_reg(EI_IND);
  drv_bus(DRV_TX_INT_ENA_INDEX)                     <=  int_ena_reg(TI_IND);  
  drv_bus(DRV_RX_INT_ENA_INDEX)                     <=  int_ena_reg(RI_IND);
  drv_bus(DRV_LOG_FIN_INT_ENA_INDEX)                <=  int_ena_reg(LFI_IND);
  drv_bus(DRV_RX_FULL_INT_ENA_INDEX)                <=  int_ena_reg(RFI_IND);
  drv_bus(DRV_BRS_INT_ENA_INDEX)                    <=  int_ena_reg(BSI_IND);
  
  drv_bus(DRV_INT_VECT_ERASE_INDEX)                 <=  interrupt_vector_erase;
  
  --Falt confinement
  drv_bus(DRV_EWL_HIGH downto DRV_EWL_LOW)          <=  ewl;  
  drv_bus(DRV_ERP_HIGH downto DRV_ERP_LOW)          <=  erp;
  
  drv_bus(DRV_CTR_VAL_HIGH downto DRV_CTR_VAL_LOW)  <=  erctr_pres_value;
  drv_bus(DRV_CTR_SEL_HIGH downto DRV_CTR_SEL_LOW)  <=  erctr_pres_mask;
  
  --CAN Core
  drv_bus(DRV_ABORT_TRAN_INDEX)                     <=  abort_transmittion;
  
  drv_bus(DRV_CAN_FD_ENA_INDEX)                     <=  mode_reg(FDE_IND);
  drv_bus(DRV_RTR_PREF_INDEX)                       <=  mode_reg(RTR_PREF_IND);
  drv_bus(DRV_BUS_MON_ENA_INDEX)                    <=  mode_reg(LOM_IND); --Bus monitoring = listen only mode
  drv_bus(DRV_SELF_TEST_ENA_INDEX)                  <=  mode_reg(STM_IND);
  
  drv_bus(DRV_RETR_LIM_ENA_INDEX)                   <=  retr_lim_ena;
  drv_bus(DRV_RETR_TH_HIGH downto DRV_RETR_TH_LOW)  <=  retr_lim_th;
  drv_bus(DRV_ENA_INDEX)                            <=  CAN_enable;
  drv_bus(DRV_FD_TYPE_INDEX)                        <=  FD_Type;
  
  --Bus traffic counters
  drv_bus(DRV_SET_CTR_VAL_HIGH downto DRV_SET_CTR_VAL_LOW)  <=  ctr_val_set;
  drv_bus(DRV_SET_RX_CTR_INDEX)                     <=  rx_ctr_set;
  drv_bus(DRV_SET_TX_CTR_INDEX)                     <=  tx_ctr_set;

  drv_bus(DRV_ACK_FORB_INDEX)                       <=  ack_forb;
  drv_bus(DRV_INT_LOOBACK_ENA_INDEX)                <=  intLoopbackEna;
  
  --Event logger
  drv_bus(DRV_TRIG_CONFIG_DATA_HIGH downto DRV_TRIG_CONFIG_DATA_LOW)<=(OTHERS =>'0');
   
  drv_bus(DRV_TRIG_SOF_INDEX)                       <=  log_trig_config(0);
  drv_bus(DRV_TRIG_ARB_LOST_INDEX)                  <=  log_trig_config(1);
  drv_bus(DRV_TRIG_REC_VALID_INDEX)                 <=  log_trig_config(2);
  drv_bus(DRV_TRIG_TRAN_VALID_INDEX)                <=  log_trig_config(3);
  drv_bus(DRV_TRIG_OVL_INDEX)                       <=  log_trig_config(4);
  drv_bus(DRV_TRIG_ERROR_INDEX)                     <=  log_trig_config(5);
  drv_bus(DRV_TRIG_BRS_INDEX)                       <=  log_trig_config(6);
  drv_bus(DRV_TRIG_USER_WRITE_INDEX)                <=  log_trig_config(7);
  drv_bus(DRV_TRIG_ARB_START_INDEX)                 <=  log_trig_config(8);
  drv_bus(DRV_TRIG_CONTR_START_INDEX)               <=  log_trig_config(9);
  drv_bus(DRV_TRIG_DATA_START_INDEX)                <=  log_trig_config(10);
  drv_bus(DRV_TRIG_CRC_START_INDEX)                 <=  log_trig_config(11);
  drv_bus(DRV_TRIG_ACK_REC_INDEX)                   <=  log_trig_config(12);
  drv_bus(DRV_TRIG_ACK_N_REC_INDEX)                 <=  log_trig_config(13);
  drv_bus(DRV_TRIG_EWL_REACHED_INDEX)               <=  log_trig_config(14);
  drv_bus(DRV_TRIG_ERP_CHANGED_INDEX)               <=  log_trig_config(15);
  drv_bus(DRV_TRIG_TRAN_START_INDEX)                <=  log_trig_config(16);
  drv_bus(DRV_TRIG_REC_START_INDEX)                 <=  log_trig_config(17);
  
  drv_bus(DRV_CAP_SOF_INDEX)                        <=  log_capt_config(0);
  drv_bus(DRV_CAP_ARB_LOST_INDEX)                   <=  log_capt_config(1);
  drv_bus(DRV_CAP_REC_VALID_INDEX)                  <=  log_capt_config(2);
  drv_bus(DRV_CAP_TRAN_VALID_INDEX)                 <=  log_capt_config(3);
  drv_bus(DRV_CAP_OVL_INDEX)                        <=  log_capt_config(4);
  drv_bus(DRV_CAP_ERROR_INDEX)                      <=  log_capt_config(5);
  drv_bus(DRV_CAP_BRS_INDEX)                        <=  log_capt_config(6);
  drv_bus(DRV_CAP_ARB_START_INDEX)                  <=  log_capt_config(7);
  drv_bus(DRV_CAP_CONTR_START_INDEX)                <=  log_capt_config(8);
  drv_bus(DRV_CAP_DATA_START_INDEX)                 <=  log_capt_config(9);
  drv_bus(DRV_CAP_CRC_START_INDEX)                  <=  log_capt_config(10);
  drv_bus(DRV_CAP_ACK_REC_INDEX)                    <=  log_capt_config(11);
  drv_bus(DRV_CAP_ACK_N_REC_INDEX)                  <=  log_capt_config(12);
  drv_bus(DRC_CAP_EWL_REACHED_INDEX)                <=  log_capt_config(13);
  drv_bus(DRV_CAP_ERP_CHANGED_INDEX)                <=  log_capt_config(14);
  drv_bus(DRV_CAP_TRAN_START_INDEX)                 <=  log_capt_config(15);
  drv_bus(DRV_CAP_REC_START_INDEX)                  <=  log_capt_config(16);
  drv_bus(DRV_CAP_SYNC_EDGE_INDEX)                  <=  log_capt_config(17);
  drv_bus(DRV_CAP_STUFFED_INDEX)                    <=  log_capt_config(18);
  drv_bus(DRV_CAP_DESTUFFED_INDEX)                  <=  log_capt_config(19);
  drv_bus(DRV_CAP_OVR_INDEX)                        <=  log_capt_config(20);
     
  drv_bus(DRV_LOG_CMD_STR_INDEX)                    <=  log_cmd(0);
  drv_bus(DRV_LOG_CMD_ABT_INDEX)                    <=  log_cmd(1);
  drv_bus(DRV_LOG_CMD_UP_INDEX)                     <=  log_cmd(2);
  drv_bus(DRV_LOG_CMD_DOWN_INDEX)                   <=  log_cmd(3);

end architecture;