--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisors and co-authors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
-- Purpose:
--  Memory registers which control functionality of CAN FD core. Memory inter-
--  face is 32 bit avalon compatible. Registers create drv_bus signal which is 
--  used in whole CAN FD IP function to control all modules. Memory Reads and 
--  writes to any location need to be executed as one read, write. No extended 
--  cycles are allowed.
--  Write to register as following:
--    1. SCS <= ACT_SCS, data_in <= valid_data, adress <= valid_adress
--    2. SWR <= ACT_SWR, wait at least one clock cycle
--    3. SWR <= not ACT_SWR SCS <= not ACT_SCS
--  Read from register as following:
--    1. SCS <= ACT_SCS, adress<=valid_adress
--    2. SRD <= ACT_SRD, wait at least one clock cycle
--    3. Capture valid data on data_out output
--    4. SRD <= not ACT_SRD, SCS <= not ACT_SCS
--------------------------------------------------------------------------------
-- Note: You must wait at least 1 cycle after deasserting async reset, since
--       a synchronous reset is performed the next cycle.
--------------------------------------------------------------------------------
--Note: All control signals which command any event execution which lasts one
--      clock cycle has negative edge detection. Therefore once srd or swr is 
--      active to finish the read or write it has to become inactive!--
-- 2018-04-10 MJ: this looks untrue!
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--    19.12.2015  RETR register changed for settings register, added configura-
--                tion options for enabling and disabling whole controller, and 
--                selecting ISO FD option. (Not yet implemented)
--    16.5.2016   Added restart function. Code formatting and constant replace-
--                ment
--    19.6.2016   Changed tx_data reg to be array 5*128 bits instead of 640 
--                std_logic vector. This should ease the automatic inference 
--                into RAM memory...
--    20.6.2016   Added ET bit in status register to monitor transmittion of 
--                error frame!
--    21.6.2016   Fixed SETTINGS registers some of the bits were not read back 
--                correctly
--    23.6.2016   Added DEBUG_REG for some additional debugging purposes
--    15.7.2016   Added "RX_buff_read_first" and "aux_data" signals. Changed han-
--                dling of moving to next word in RX buffer RX_DATA. Now first 
--                cycle of memory access is detected, here and  "rx_read_start" 
--                is set to active value for only one clock cycle! Even if bus 
--                access lasts several clock cycles data output is captured only
--                in the first cycle and then held until the end of access. 
--                Additionally "rx_read_start" signal is now combinationall, not
--                registered output. Thisway latency is shortened. Without this
--                precaution it was necessary to add empty cycles between reads
--                from RX_DATA!!!        
--    24.8.2016   Added "use_logger" generic and LOG_EXIST bit to the LOG_STATUS
--                register to provide way how to find out from SW if logger is 
--                actually present. Size is not deciding since HW developer can 
--                set the size to e.g. 32 and use_logger to false!
--    1.9.2016    Moved SJW values to separate register! Now SJW has 4 bits 
--                instead of two bits! This is compliant with CAN FD specifi-
--                cation.
--    30.11.2017  Changed implementation of TX_DATA registers. Registers removed
--                and access into these registers is now directly accessing RAM 
--                in TXT buffer. Note that buffer must be first forbidden in 
--                TX_SETTINGS register so that half written frame is not commi-
--                tted to CAN Core for transmission. Added BUF_DIR bit and remo-
--                ved TXT1_COMMIT and TXT2_COMMIT bits
--    12.12.2017  Renamed entity to  "canfd_registers" instead of "registers"
--                to avoid possible name conflicts.
--    20.12.2017  Removed obsolete tran_data_in signal. Removed obsolete 
--                tx_data_reg. Added supoort for byte enable signal on register
--                writes and reads.
--    27.12.2017  Added "txt_frame_swap" bit for frame swapping after the
--                frame retransmission.
--    28.12.2017  Added support for "tx_time_suport" and Filter Status register.
--    18.01.2018  Removed txt1_disc, txt2_disc, txt1_commit and txt2_disc 
--                obsolete signals
--    21.02.2018  Removed "txt_frame_swap" since it is not needed with new,
--                priority based implementation of TX Buffers.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CAN_FD_register_map.all;

entity canfd_registers is
  generic(
    constant compType     :std_logic_vector(3 downto 0)    := CAN_COMPONENT_TYPE;
    
    --Whenever event logger is present
    constant use_logger   :boolean                         := true; 
    
    --Optional synthesis of received message filters
    constant sup_filtA    :boolean                         := true; 
    
    -- By default the behaviour is as if all the filters are present
    constant sup_filtB    :boolean                         := true; 
    constant sup_filtC    :boolean                         := true;
    constant sup_range    :boolean                         := true;
    
    --Support of byte enable signal on memory bus interface
    constant sup_be       :boolean                         := false;
    
    --Support of Transmission at given time
    constant tx_time_sup  : boolean                        := true;
    
    -- Number of TXT Buffers
    constant buf_count    : natural range 0 to 7           := 4;
    
    --ID of the component
    constant ID           :natural                         := 1 
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
    signal sbe                  :in   std_logic_vector(3 downto 0);
      
    --Driving and Status Bus
    signal drv_bus              :out  std_logic_vector(1023 downto 0) 
                                        := (OTHERS=>'0');
    signal stat_bus             :in   std_logic_vector(511 downto 0);
    
    -----------------------
    --RX Buffer Interface--
    -----------------------
    
    --Actually loaded data for reading
    signal rx_read_buff         :in   std_logic_vector(31 downto 0); 
    
    --Size of  message buffer (in words)
    signal rx_buf_size          :in   std_logic_vector(12 downto 0);   
    
    --Signal whenever buffer is full
    signal rx_full              :in   std_logic;                      
    
    --Signal whenever buffer is empty
    signal rx_empty             :in   std_logic;                      
    
    --Number of frames in recieve buffer
    signal rx_message_count     :in   std_logic_vector(10 downto 0);   
    
    --Number of free 32 bit wide ''windows''
    signal rx_mem_free          :in   std_logic_vector(12 downto 0);   
    
    --Position of read pointer
    signal rx_read_pointer_pos  :in   std_logic_vector(11 downto 0);   
    
    --Position of write pointer
    signal rx_write_pointer_pos :in   std_logic_vector(11 downto 0);   
    
    --Frame was discarded due full Memory
    signal rx_message_disc      :in   std_logic;                     
    
     --Some data were discarded, register 
    signal rx_data_overrun      :in   std_logic;                     
    
    --------------------------------------------------------
    -- Optimized, direct interface to TXT1 and TXT2 buffers
    --------------------------------------------------------
    
    -- Data and address for access to RAM of TXT Buffer
    signal tran_data            :out  std_logic_vector(31 downto 0);    
    signal tran_addr            :out  std_logic_vector(4 downto 0);  
    signal txtb_cs              :out  std_logic_vector(buf_count - 1 downto 0);   
  
    -- Buffer status signals
    signal txtb_fsms            :in   txt_fsms_type;
    
    -- Buffer commands + command index
    signal txt_sw_cmd           :out  txt_sw_cmd_type;
    signal txt_buf_cmd_index    :out  std_logic_vector(buf_count - 1 downto 0);
    signal txt_buf_prior_out    :out  txtb_priorities_type;
     
    ----------------------------------
    -- Bus synchroniser interface
    ----------------------------------
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
    signal int_vector           :in   std_logic_vector(INT_COUNT - 1 downto 0);
    signal int_ena              :in   std_logic_vector(INT_COUNT - 1 downto 0);
    signal int_mask             :in   std_logic_vector(INT_COUNT - 1 downto 0)
  );
  ----------------------
  --Internal registers--
  ----------------------
  signal int_reset              :     std_logic:='1';
  
  -- Internal read data for byte enabled reads
  signal data_out_int           :     std_logic_vector(31 downto 0);
  signal sbe_reg                :     std_logic_vector(3 downto 0);
  
  --Command registers
  signal clear_overrun          :     std_logic;
  signal release_recieve        :     std_logic;
  signal abort_transmittion     :     std_logic;
  signal ack_forb               :     std_logic;
  
  --Retransmitt registers
  --Retransmit limited is enabled
  signal retr_lim_ena           :     std_logic;                     
  
  --Retransmit treshold
  signal retr_lim_th            :     std_logic_vector(3 downto 0);   
  
  --Interrupt registers
  signal int_vect_clear         :     std_logic_vector(INT_COUNT - 1 downto 0);
  signal int_ena_set            :     std_logic_vector(INT_COUNT - 1 downto 0);
  signal int_ena_clear          :     std_logic_vector(INT_COUNT - 1 downto 0);
  signal int_mask_set           :     std_logic_vector(INT_COUNT - 1 downto 0);
  signal int_mask_clear         :     std_logic_vector(INT_COUNT - 1 downto 0);
  
  --Timing registers
  signal sjw_norm               :     std_logic_vector(4 downto 0);
  signal brp_norm               :     std_logic_vector(7 downto 0);
  signal ph1_norm               :     std_logic_vector(5 downto 0);
  signal ph2_norm               :     std_logic_vector(5 downto 0);
  signal prop_norm              :     std_logic_vector(6 downto 0);
  
  signal sjw_fd                 :     std_logic_vector(4 downto 0);
  signal brp_fd                 :     std_logic_vector(7 downto 0);
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
  
  -- RX Buffer control signals
  -- Transition from logic 1 to logic zero on this signal 
  -- causes rx_reading_pointer increment by one
  signal rx_read_start          :     std_logic; 
  
  ---------------------------------------------------
  -- TXT Buffer settings
  ---------------------------------------------------
  
  -- TXT Buffer priorities
  signal txt_buf_prior          :     txtb_priorities_type;
  
  --One of the TX Buffers is accessed
  signal txt_buf_access         :     boolean;
  
    
  signal intLoopbackEna         :     std_logic;
  
  --Event logger registers
  signal log_trig_config        :     std_logic_vector(31 downto 0);
  signal log_capt_config        :     std_logic_vector(31 downto 0);
  signal log_cmd                :     std_logic_vector(3 downto 0);
     
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
  
  --Mode register
  signal mode_reg               :     std_logic_vector(5 downto 0);   
  
  --Status Register
  signal status_reg             :     std_logic_vector(7 downto 0);   
  
  --Auxiliarly signals
  signal PC_state               :     protocol_type;
  signal PC_state_reg_vect      :     std_logic_vector(6 downto 0);
  
  --Reading from RX buffer, detection of first cycle to move the pointer
  signal RX_buff_read_first     :     boolean;
  signal aux_data               :     std_logic_Vector(31 downto 0);
  
  -- Receive Timestamp options
  signal rtsopt                 :     std_logic;
  
end entity;


architecture rtl of canfd_registers is

  --------------------------------------------
  -- Default value assignment to registers --
  -------------------------------------------
  procedure reg_reset (
    signal int_reset              :out  std_logic;
    signal clear_overrun          :out  std_logic;
    signal release_recieve        :out  std_logic;
    signal abort_transmittion     :out  std_logic;
    signal ack_forb               :out  std_logic;
    
    --Retransmit limited is enabled
    signal retr_lim_ena           :out  std_logic;                      
    
    --Retransmit treshold
    signal retr_lim_th            :out  std_logic_vector(3 downto 0);
       
    signal int_vect_clear         :out  std_logic_vector(INT_COUNT - 1 downto 0);
    signal int_ena_set            :out  std_logic_vector(INT_COUNT - 1 downto 0);
    signal int_ena_clear          :out  std_logic_vector(INT_COUNT - 1 downto 0);
    signal int_mask_set           :out  std_logic_vector(INT_COUNT - 1 downto 0);
    signal int_mask_clear         :out  std_logic_vector(INT_COUNT - 1 downto 0);
    
    signal sjw_norm               :out  std_logic_vector(4 downto 0);
    signal brp_norm               :out  std_logic_vector(7 downto 0);
    signal ph1_norm               :out  std_logic_vector(5 downto 0);
    signal ph2_norm               :out  std_logic_vector(5 downto 0);
    signal prop_norm              :out  std_logic_vector(6 downto 0);
    
    signal sjw_fd                 :out  std_logic_vector(4 downto 0);
    signal brp_fd                 :out  std_logic_vector(7 downto 0);
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
    
    signal txt_buf_set_empty      :out  std_logic;
    signal txt_buf_set_ready      :out  std_logic;
    signal txt_buf_set_abort      :out  std_logic;
    signal txt_buf_cmd_index      :out  std_logic_vector(buf_count - 1 downto 0);
    signal txt_buf_prior          :out  txtb_priorities_type;

    signal intLoopbackEna         :out  std_logic;
    signal log_trig_config        :out  std_logic_vector(31 downto 0);
    signal log_capt_config        :out  std_logic_vector(31 downto 0);
    signal log_cmd                :out  std_logic_vector(3 downto 0);    
    signal rx_ctr_set             :out  std_logic;
    signal tx_ctr_set             :out  std_logic;
    signal ctr_val_set            :out  std_logic_vector(31 downto 0);
    signal CAN_enable             :out  std_logic;
    signal FD_type                :out  std_logic; 
    signal mode_reg               :out  std_logic_vector(5 downto 0);   
    signal rtsopt                 :out  std_logic
  ) is
  begin
    
    --Command registers
    clear_overrun           <=  CDO_RSTVAL;
    release_recieve         <=  RRB_RSTVAL;
    abort_transmittion      <=  AT_RSTVAL;
    
    erctr_pres_value        <=  (OTHERS=>'0');
    erctr_pres_mask         <=  (OTHERS=>'0');
    ctr_val_set             <=  (OTHERS =>'0');
    rx_ctr_set              <=  NO_ACTION;
    tx_ctr_set              <=  NO_ACTION;
    ack_forb                <=  ACK_ALLOWED;
    intLoopbackEna          <=  LOOPBACK_DIS;
   
    --Enable register
    CAN_enable              <=  ENA_RSTVAL;
    FD_type                 <=  FD_TYPE_RSTVAL;
    
    --Mode register 
    mode_reg(RST_IND)       <=  RST_RSTVAL;
    mode_reg(LOM_IND)       <=  LOM_RSTVAL;   --Listen only mode
    mode_reg(STM_IND)       <=  STM_RSTVAL;   --Self test mode
    mode_reg(AFM_IND)       <=  AFM_RSTVAL;   --Acceptance filters mode 
    mode_reg(FDE_IND)       <=  FDE_RSTVAL;    --Flexible datarate enable
    mode_reg(RTR_PREF_IND)  <=  RTR_PREF_RSTVAL;    --RTR Preffered behaviour   
        
    --Retransmitt limit enable
    retr_lim_ena            <=  RTRLE_RSTVAL;
    retr_lim_th             <=  RTR_TH_RSTVAL; --Retr. limit treshold zeroes   
    
    sjw_norm                <=  SJW_RSTVAL;
    brp_norm                <=  BRP_RSTVAL;
    ph1_norm                <=  PH1_RSTVAL;
    ph2_norm                <=  PH2_RSTVAL;
    prop_norm               <=  PROP_RSTVAL;
    
    sjw_fd                  <=  SJW_FD_RSTVAL;
    brp_fd                  <=  BRP_FD_RSTVAL;
    ph1_fd                  <=  PH1_FD_RSTVAL;
    ph2_fd                  <=  PH2_FD_RSTVAL;
    prop_fd                 <=  PROP_FD_RSTVAL;
    
    sam_norm                <=  TSM_RSTVAL;
    ewl                     <=  EWL_LIMIT_RSTVAL;
    erp                     <=  ERP_LIMIT_RSTVAL;
    
    --Message filters
    if (sup_filtA = true) then
      filter_A_mask           <=  BIT_MASK_A_VAL_RSTVAL;
      filter_A_value          <=  BIT_VAL_A_VAL_RSTVAL;
      
       --Only filter A is enabled to pass all message types with any identifier
      filter_A_ctrl           <=  (OTHERS=>'1');
      
    end if;
    
    if (sup_filtB = true) then
      filter_B_mask           <=  BIT_MASK_B_VAL_RSTVAL;
      filter_B_value          <=  BIT_MASK_B_VAL_RSTVAL;
      filter_B_ctrl           <=  (OTHERS=>'0');
    end if;
    
    if (sup_filtB = true) then
      filter_C_mask           <=  BIT_MASK_C_VAL_RSTVAL;
      filter_C_value          <=  BIT_MASK_C_VAL_RSTVAL;
      filter_C_ctrl           <=  (OTHERS=>'0');
    end if;
    
    if (sup_range = true) then
      filter_ran_low          <=  BIT_RAN_LOW_VAL_RSTVAL;
      filter_ran_high         <=  BIT_RAN_HIGH_VAL_RSTVAL; 
      filter_ran_ctrl         <=  (OTHERS=>'0');
    end if;
    
    log_cmd                 <=  (OTHERS =>'0');
    log_trig_config         <=  (OTHERS =>'0');
    log_capt_config         <=  (OTHERS =>'0');
    
    txt_buf_set_empty      <= TXCE_RSTVAL;
    txt_buf_set_ready      <= TXCR_RSTVAL;
    txt_buf_set_abort      <= TXCA_RSTVAL;
    
    txt_buf_cmd_index(0)   <= TXI1_RSTVAL;
    txt_buf_cmd_index(1)   <= TXI2_RSTVAL;
    txt_buf_cmd_index(2)   <= TXI3_RSTVAL;
    txt_buf_cmd_index(3)   <= TXI4_RSTVAL;
    
    txt_buf_prior(0)       <= TXT1P_RSTVAL;
    txt_buf_prior(1)       <= TXT2P_RSTVAL;
    txt_buf_prior(2)       <= TXT3P_RSTVAL;
    txt_buf_prior(3)       <= TXT4P_RSTVAL;
    
    int_vect_clear         <= (OTHERS => '0');
    int_ena_set            <= INT_ENA_SET_RSTVAL;
    int_ena_clear          <= INT_ENA_CLR_RSTVAL;
    int_mask_set           <= INT_MASK_SET_RSTVAL;
    int_mask_clear         <= INT_MASK_CLR_RSTVAL;
    
    rtsopt                 <= RTSOP_RSTVAL;
  end procedure;
  
  
  ------------------------------------------------
  -- Write into register of single bit with byte
  -- enable support - variable input
  ------------------------------------------------
  procedure write_be_v(
    variable  dest_reg   : out std_logic;
    constant  bit_index  : in natural range 0 to 31;
    variable  data_in    : in std_logic_vector(31 downto 0);
    signal    be         : in std_logic_vector(3 downto 0)
  )is
  begin
    if (sup_be = true) then
      if (bit_index<8 and be(0)='1') then
        dest_reg := data_in(bit_index);
      elsif (bit_index<16 and bit_index>7 and be(1)='1') then
        dest_reg := data_in(bit_index);
      elsif (bit_index<24 and bit_index>15 and be(2)='1') then
        dest_reg := data_in(bit_index);
      elsif (bit_index<32 and bit_index>23 and be(3)='1') then
        dest_reg := data_in(bit_index);
      end if;
    else
      dest_reg := data_in(bit_index);
    end if;
  end procedure;
  
  ------------------------------------------------
  -- Write into register of single bit with byte
  -- enable support - signal input
  ------------------------------------------------
  procedure write_be_s(
    signal    dest_reg     : out std_logic;
    constant  bit_index    : in natural range 0 to 31;
    signal    data_in      : in std_logic_vector(31 downto 0);
    signal    be           : in std_logic_vector(3 downto 0)
  )is
  begin
    if (sup_be = true) then
      if (bit_index<8 and be(0)='1') then
        dest_reg <= data_in(bit_index);
      elsif (bit_index<16 and bit_index>7 and be(1)='1') then
        dest_reg <= data_in(bit_index);
      elsif (bit_index<24 and bit_index>15 and be(2)='1') then
        dest_reg <= data_in(bit_index);
      elsif (bit_index<32 and bit_index>23 and be(3)='1') then
        dest_reg <= data_in(bit_index);
      end if;
    else
      dest_reg <= data_in(bit_index);
    end if;
  end procedure;
  
  ------------------------------------------------
  -- Write into register with byte enable support
  ------------------------------------------------
  procedure write_be_vect(
    signal   dest_reg    : out std_logic_vector;
    constant low_rindex  : in natural range 31 downto 0;
    constant high_rindex : in natural range 31 downto 0;
    
    signal   data_in     : in std_logic_vector(31 downto 0);
    constant low_dindex  : in natural range 31 downto 0;
    constant high_dindex : in natural range 31 downto 0;
    
    signal   be          : in std_logic_vector(3 downto 0)
  )is
  variable reg_val  : std_logic;
  variable data_val : std_logic_vector(31 downto 0);
  variable j : natural;
  begin
    
    -- Check if input data range to write corresponds to register indices
    if (high_rindex-low_rindex /= high_dindex-low_dindex) then
      report "Mismatching data and register size";
    end if;
    
    j := low_rindex;
    for i in low_dindex to high_dindex loop
      data_val := data_in;
      write_be_v(reg_val,i,data_val,be);
      dest_reg(j) <= reg_val;
      j := j+1;
    end loop;
  end procedure;
  

begin
  
  --------------------------------------------------------
  --Reset propagation to output
  --Note: this works only for reset active in logic zero
  --------------------------------------------------------
  res_out               <=  res_n and int_reset; 
 
  --------------------------------------------------------
  -- Propagation of Avalon address to TXT Buffer RAM
  --------------------------------------------------------
  tran_data             <= data_in;
  
  --Since TX_DATA registers are in separate region, which
  -- is aligned it is enough to take the lowest bits to 
  -- create the address offset
  tran_addr             <= adress(6 downto 2);
  
  --------------------------------------------------------
  -- Decoding of TXT buffer signals...
  --------------------------------------------------------
  txt_buf_access   <= true when (((adress(11 downto 8) = TX_BUFFER_1_BLOCK) or
                                  (adress(11 downto 8) = TX_BUFFER_2_BLOCK) or
                                  (adress(11 downto 8) = TX_BUFFER_3_BLOCK) or
                                  (adress(11 downto 8) = TX_BUFFER_4_BLOCK)) and 
                                 scs='1' and swr='1')
                           else
                      false;
                      
  -- We have to hard-code the chip select signals since we cant define array of
  -- memory regions in IP-Xact
  txtb_cs(0)       <= '1' when ((adress(11 downto 8) = TX_BUFFER_1_BLOCK) and
                                txt_buf_access)
                          else
                      '0';
  
  txtb_cs(1)       <= '1' when ((adress(11 downto 8) = TX_BUFFER_2_BLOCK) and
                                txt_buf_access)
                          else
                      '0';
                      
  txtb_cs(2)       <= '1' when ((adress(11 downto 8) = TX_BUFFER_3_BLOCK) and
                                txt_buf_access)
                          else
                      '0';
                      
  txtb_cs(3)       <= '1' when ((adress(11 downto 8) = TX_BUFFER_4_BLOCK) and
                                txt_buf_access)
                          else
                      '0';
   
  txt_buf_prior_out <= txt_buf_prior;

  --------------------------------------------------------
  -- Main memory access process
  --------------------------------------------------------
  mem_acess:process(clk_sys,res_n,int_reset)
  begin
  if(res_n=ACT_RESET)then
      
      --Internal synced reset
      int_reset               <= '1';
      data_out_int            <= (OTHERS=>'0');
      sbe_reg                 <= (OTHERS => '0');
      
      --Reset the rest of registers
      reg_reset (
       int_reset          ,clear_overrun          ,release_recieve         ,
       abort_transmittion ,ack_forb               ,retr_lim_ena            ,
       retr_lim_th        ,
       
       int_vect_clear     ,int_ena_set            ,int_ena_clear           ,
       int_mask_set       ,int_mask_clear         ,
       
       sjw_norm           ,
       brp_norm           ,ph1_norm               ,ph2_norm                ,
       prop_norm          ,sjw_fd                 ,brp_fd                  ,             
       ph1_fd             ,ph2_fd                 ,prop_fd                 ,
       sam_norm           ,ewl                    ,erp                     ,
       erctr_pres_value   ,erctr_pres_mask        ,filter_A_mask           ,
       filter_B_mask      ,filter_C_mask          ,filter_A_value          ,
       filter_B_value     ,filter_C_value         ,filter_ran_low          ,        
       filter_ran_high    ,filter_A_ctrl          ,filter_B_ctrl           ,
       filter_C_ctrl      ,filter_ran_ctrl        ,
       txt_sw_cmd.set_ety ,txt_sw_cmd.set_rdy     ,     
       txt_sw_cmd.set_abt ,txt_buf_cmd_index      ,txt_buf_prior           ,     
       intLoopbackEna     ,log_trig_config        ,
       log_capt_config    ,log_cmd                ,rx_ctr_set              ,
       tx_ctr_set         ,ctr_val_set            ,CAN_enable              ,
       FD_type            ,mode_reg               ,rtsopt         
      );
      
      RX_buff_read_first    <= false;
      aux_data              <=  (OTHERS=>'0');
      
  elsif rising_edge(clk_sys)then
	if(int_reset=ACT_RESET)then --Synchronous reset
		
  		  --Internal synced reset
  		  int_reset               <=  not ACT_RESET;
  	   data_out_int            <= (OTHERS=>'0');
  	   sbe_reg                 <= (OTHERS => '0');
  	   
  	    --Reset the rest of registers
      reg_reset (
       int_reset          ,clear_overrun          ,release_recieve         ,
       abort_transmittion ,ack_forb               ,retr_lim_ena            ,
       retr_lim_th        ,
       
       int_vect_clear     ,int_ena_set            ,int_ena_clear           ,
       int_mask_set       ,int_mask_clear         ,

       sjw_norm           ,
       brp_norm           ,ph1_norm               ,ph2_norm                ,
       prop_norm          ,sjw_fd                 ,brp_fd                  ,             
       ph1_fd             ,ph2_fd                 ,prop_fd                 ,
       sam_norm           ,ewl                    ,erp                     ,
       erctr_pres_value   ,erctr_pres_mask        ,filter_A_mask           ,
       filter_B_mask      ,filter_C_mask          ,filter_A_value          ,
       filter_B_value     ,filter_C_value         ,filter_ran_low          ,        
       filter_ran_high    ,filter_A_ctrl          ,filter_B_ctrl           ,
       filter_C_ctrl      ,filter_ran_ctrl        ,
       txt_sw_cmd.set_ety ,txt_sw_cmd.set_rdy     ,     
       txt_sw_cmd.set_abt ,txt_buf_cmd_index      ,txt_buf_prior           ,      
       intLoopbackEna     ,log_trig_config        ,
       log_capt_config    ,log_cmd                ,
       rx_ctr_set         ,tx_ctr_set             ,
       ctr_val_set        ,CAN_enable             ,FD_type                 ,         
       mode_reg           ,rtsopt           
      );
           
      RX_buff_read_first    <= false;
      aux_data              <=  (OTHERS=>'0');

	else
		--Internal registers holding its value
		
		 --Message filters
    if (sup_filtA = true) then
      filter_A_mask           <=  filter_A_mask;
      filter_A_value          <=  filter_A_value;
      filter_A_ctrl           <=  filter_A_ctrl;
    end if;
    
    if (sup_filtB = true) then
      filter_B_mask           <=  filter_B_mask;
      filter_B_value          <=  filter_B_value;
      filter_B_ctrl           <=  filter_B_ctrl;
    end if;
    
    if (sup_filtB = true) then
      filter_C_mask           <=  filter_C_mask;
      filter_C_value          <=  filter_C_value;
      filter_C_ctrl           <=  filter_C_ctrl;
    end if;
    
    if (sup_range = true) then
      filter_ran_low          <=  filter_ran_low;
      filter_ran_high         <=  filter_ran_high;
      filter_ran_ctrl         <=  filter_ran_ctrl;
    end if;
		
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
		erctr_pres_value          <=  (OTHERS=>'0');
		erctr_pres_mask           <=  "0000";
		ctr_val_set               <=  (OTHERS =>'0');
		rx_ctr_set                <=  '0';
		tx_ctr_set                <=  '0';
		ack_forb                  <=  ack_forb;
		data_out_int              <=  (OTHERS=>'0');
		log_cmd                   <=  (OTHERS =>'0');
		
		RX_buff_read_first        <= false;
		aux_data                  <=  (OTHERS=>'0');
		sbe_reg                   <= sbe;
		
		txt_sw_cmd.set_ety        <= '0';
    txt_sw_cmd.set_rdy        <= '0';
    txt_sw_cmd.set_abt        <= '0';
    txt_buf_cmd_index         <= (OTHERS => '0');
		txt_buf_prior             <= txt_buf_prior;
		
		--Chip select active and our device is selected (Component type and ID)
		if((scs=ACT_CSC) and 
		   (adress(COMP_TYPE_ADRESS_HIGHER downto 
		    COMP_TYPE_ADRESS_LOWER)=compType) and
		   (adress(ID_ADRESS_HIGHER downto ID_ADRESS_LOWER)=
		   std_logic_vector(to_unsigned(ID,4))) )
		then 
		  
		  --------------------
		  --Writing the data--
		  --------------------
		  if(swr=ACT_SWR)then 
    			case adress(11 downto 0) is
    			
    			---------------------------------------------------------  
    			-- MODE, COMMAND and SETTINGS registers
    			---------------------------------------------------------   
    			when MODE_ADR =>    
    
    			     --RTR_PREF,FDE,AFM,STM,LOM Bits
    					  write_be_vect(mode_reg, 1, 5, data_in, 1, 5, sbe);
    					  
					  --Tripple sampling
					  write_be_s(sam_norm, TSM_IND, data_in, sbe);
    					  		  
    					  --Acknowledge forbidden
    					  write_be_s(ack_forb, ACF_IND, data_in, sbe);       
    					  
    					  --Reset by memory access
    					  if(data_in(RST_IND)='1' and sbe(0)='1')then                              
    					   int_reset               <=  ACT_RESET; 
    					  end if;      
    					            
    					  --Command register
    					  write_be_s(clear_overrun, CDO_IND, data_in, sbe);
 					  write_be_s(release_recieve, RRB_IND, data_in, sbe);
 					  write_be_s(abort_transmittion, AT_IND, data_in, sbe);
 					 		  
    					  --Status register is read only!
    					  
    					  --Settings register
    					  write_be_s(retr_lim_ena, RTRLE_IND, data_in, sbe);     
    					  write_be_vect(retr_lim_th, 0, 3, data_in, RTR_TH_L, RTR_TH_H, sbe);
    					  write_be_s(intLoopbackEna, INT_LOOP_IND, data_in, sbe);     
    					  write_be_s(CAN_enable, ENA_IND, data_in, sbe);     
    					  write_be_s(FD_type, FD_TYPE_IND, data_in, sbe);     
    			
			 ------------------------------------------------------------	  
 			 -- INT_STATUS register
 			 ------------------------------------------------------------
    			when INT_STAT_ADR =>               
 			      write_be_vect(int_vect_clear, 0, INT_COUNT - 1, data_in,
 			                                    0, INT_COUNT - 1, sbe);
    
       ------------------------------------------------------------	  
 			 -- INT_ENA_SET register
 			 ------------------------------------------------------------
    			when INT_ENA_SET_ADR =>
    			     write_be_vect(int_ena_set, 0, INT_COUNT - 1, data_in,
 			                                 0, INT_COUNT - 1, sbe);

       ------------------------------------------------------------	  
 			 -- INT_ENA_CLEAR register
 			 ------------------------------------------------------------
    			when INT_ENA_CLR_ADR =>
    			     write_be_vect(int_ena_clear, 0, INT_COUNT - 1, data_in,
 			                                   0, INT_COUNT - 1, sbe);
       
       ------------------------------------------------------------	  
 			 -- INT_MASK_SET register
 			 ------------------------------------------------------------
    			when INT_MASK_SET_ADR =>
    			     write_be_vect(int_mask_set, 0, INT_COUNT - 1, data_in,
 			                                  0, INT_COUNT - 1, sbe);
       
       ------------------------------------------------------------	  
 			 -- INT_MASK_CLEAR register
 			 ------------------------------------------------------------
    			when INT_MASK_CLR_ADR =>
    			     write_be_vect(int_mask_clear, 0, INT_COUNT - 1, data_in,
 			                                    0, INT_COUNT - 1, sbe);
 
 			 ----------------------
 			 -- BTR
 			 ----------------------
    			when BTR_ADR =>
    			      write_be_vect(prop_norm, 0, prop_norm'length - 1, data_in, 
			                     PROP_L, PROP_H, sbe);
    			      write_be_vect(ph1_norm, 0, ph1_norm'length - 1, data_in,
  			                     PH1_L, PH1_H, sbe);
    			      write_be_vect(ph2_norm, 0, ph2_norm'length - 1, data_in,
  			                     PH2_L, PH2_H, sbe);
    			      write_be_vect(brp_norm, 0, brp_norm'length - 1, data_in,
  			                     BRP_L, BRP_H, sbe);
 					   write_be_vect(sjw_norm, 0, sjw_norm'length - 1, data_in,
 					                 SJW_L, SJW_H, sbe);
 					   
  			 ----------------------
 			 -- BTR_FD
 			 ----------------------
    			when BTR_FD_ADR =>
    			      write_be_vect(prop_fd, 0, prop_fd'length - 1, data_in,
    			                     PROP_FD_L, PROP_FD_H, sbe);
    			      write_be_vect(ph1_fd, 0, ph1_fd'length - 1, data_in,
    			                     PH1_FD_L, PH1_FD_H, sbe);
    			      write_be_vect(ph2_fd, 0, ph2_fd'length - 1, data_in,
    			                     PH2_FD_L, PH2_FD_H, sbe);
             write_be_vect(brp_fd, 0, brp_fd'length - 1, data_in,
                            BRP_FD_L, BRP_FD_H, sbe);
 					   write_be_vect(sjw_fd, 0, sjw_fd'length - 1, data_in,
 					                  SJW_FD_L, SJW_FD_H, sbe);
    					   
    			---------------------------------------------------- 
  			 -- EWL, ERP
  			 ----------------------------------------------------	   
    			when EWL_ADR =>
				           
				     --Error warning limit
				     write_be_vect(ewl, 0, 7, data_in, EWL_LIMIT_L, EWL_LIMIT_H, sbe);
    					   
 					   --Error passive treshold
 					   write_be_vect(erp, 0, 7, data_in, ERP_LIMIT_L, ERP_LIMIT_H, sbe);
    			
    			----------------------------------------------------	   
    			-- CTR_PRES 
    			----------------------------------------------------	   
    			when CTR_PRES_ADR => 
    			  
    			     write_be_vect(erctr_pres_value, 0, 8, data_in, CTPV_L, CTPV_H, sbe);
    			     write_be_vect(erctr_pres_mask, 0, 3, data_in, 9, 12, sbe);
    			   
    			----------------------------------------------------	   
    			-- ERR_NORM_ADR
    			----------------------------------------------------	
    			when ERR_NORM_ADR => 
    			  
    			     if (sbe(1) = '1') then
    					    erctr_pres_value         <=  (OTHERS=>'0');
    					    erctr_pres_mask          <=  data_in(12 downto 11)&"00"; 
  		        end if;
    		
    		 ----------------------------------------------------	   
    			--Acceptance filters
    			----------------------------------------------------	
    			when FILTER_A_VAL_ADR    => 
    			       if (sup_filtA) then
    			         write_be_vect(filter_A_value, 0, 28, data_in, 
			                  BIT_VAL_A_VAL_L, BIT_VAL_A_VAL_H, sbe);
  			        end if;
    			when FILTER_A_MASK_ADR   =>
    			       if (sup_filtA) then
    			         write_be_vect(filter_A_mask, 0, 28, data_in,
			                  BIT_MASK_A_VAL_L, BIT_MASK_A_VAL_H, sbe);
			        end if;     
    			when FILTER_B_VAL_ADR    => 
    			     if (sup_filtB) then
    			       write_be_vect(filter_B_value, 0, 28, data_in,
    			               BIT_VAL_B_VAL_L, BIT_VAL_B_VAL_H, sbe);
  			       end if;
    			when FILTER_B_MASK_ADR   => 
    			     if (sup_filtB) then
    			       write_be_vect(filter_B_mask, 0, 28, data_in,
    			               BIT_MASK_B_VAL_L, BIT_MASK_B_VAL_H, sbe);
			      end if;
    			when FILTER_C_VAL_ADR    =>
    			     if (sup_filtC) then
    			       write_be_vect(filter_C_value, 0, 28, data_in,
    			               BIT_VAL_C_VAL_L, BIT_VAL_C_VAL_H, sbe);
			      end if;
    			when FILTER_C_MASK_ADR   =>
    			     if (sup_filtC) then
    			       write_be_vect(filter_C_mask, 0, 28, data_in,
    			               BIT_MASK_C_VAL_L, BIT_MASK_C_VAL_H, sbe);
			      end if;
    			when FILTER_RAN_LOW_ADR  => 
    			     if (sup_range) then
    			       write_be_vect(filter_ran_low, 0, 28, data_in,
    			              BIT_RAN_LOW_VAL_L, BIT_RAN_LOW_VAL_H, sbe);
  			       end if;
    			when FILTER_RAN_HIGH_ADR => 
			      if (sup_range) then
			        write_be_vect(filter_ran_high, 0, 28, data_in,
			                BIT_RAN_HIGH_VAL_L, BIT_RAN_HIGH_VAL_H, sbe);
			      end if;
    			when FILTER_CONTROL_ADR  =>
   			      if (sup_filtA) then
   			         write_be_vect(filter_A_ctrl, 0, 3, data_in, 0, 3, sbe);
					  end if;
					  if (sup_filtB) then
					     write_be_vect(filter_B_ctrl, 0, 3, data_in, 4, 7, sbe); 
  					  end if;
  					  if (sup_filtC) then
  					     write_be_vect(filter_C_ctrl, 0, 3, data_in, 8, 11, sbe);
  				    end if;	     
					  if (sup_range) then
					     write_be_vect(filter_ran_ctrl, 0, 3, data_in, 12, 15, sbe); 
					  end if;
    			
    			----------------------------------------------------
    			-- TX_COMMAND (TX_SETTINGS and TX_COMMAND)
    			----------------------------------------------------
    			when TX_COMMAND_ADR =>
    			 
    			     -- TXT Buffers commands
    			     write_be_s(txt_sw_cmd.set_ety, TXCE_IND, data_in, sbe);
    			     write_be_s(txt_sw_cmd.set_rdy, TXCR_IND, data_in, sbe);
    			     write_be_s(txt_sw_cmd.set_abt, TXCA_IND, data_in, sbe);
    			   
            -- Vector index for which buffer the command is active
            write_be_vect(txt_buf_cmd_index, 0, TXT_BUFFER_COUNT - 1, data_in,
                          TXI1_IND, TXI1_IND + txt_buf_cmd_index'length - 1, sbe); 
      	    
   	   ----------------------------------------------------
    			-- TX_PRIORITY
    			----------------------------------------------------
 	     when TX_PRIORITY_ADR =>
 	          write_be_vect(txt_buf_prior(0), 0, 2, data_in,
                        TXT1P_L, TXT1P_H, sbe);   
 	          write_be_vect(txt_buf_prior(1), 0, 2, data_in,
                        TXT2P_L, TXT2P_H, sbe);
            write_be_vect(txt_buf_prior(2), 0, 2, data_in,
                        TXT3P_L, TXT3P_H, sbe);
            write_be_vect(txt_buf_prior(3), 0, 2, data_in,
                        TXT4P_L, TXT4P_H, sbe);   
 	     
 	     -------------------------------------------------------
			 -- RX_STATUS (RX_SETTINGS is write)
			 -------------------------------------------------------  
			 when RX_STATUS_ADR =>
    			     write_be_s(rtsopt, RTSOP_IND, data_in, sbe);
 			      
    			--------------------------------------
    			--Recieve frame counter presetting
    			--------------------------------------
    			when RX_COUNTER_ADR =>
    			     write_be_vect(ctr_val_set, 0, 31, data_in, 
    			             RX_COUNTER_VAL_L, RX_COUNTER_VAL_H, sbe);
    					  rx_ctr_set                 <=  '1';
    					  
    			--------------------------------------
    			--Transcieve frame counter presetting
    			--------------------------------------		  
    			when TX_COUNTER_ADR => 
            write_be_vect(ctr_val_set, 0, 31, data_in,
                    TX_COUNTER_VAL_L, TX_COUNTER_VAL_H, sbe);
    					  tx_ctr_set                 <=  '1';
    			
    			--------------------------------------
    			--Logger configuration registers
    			--------------------------------------		  
    			when LOG_TRIG_CONFIG_ADR=>
            write_be_vect(log_trig_config, 0, 31, data_in, 0, 31, sbe);
    			when LOG_CAPT_CONFIG_ADR=>
    			     write_be_vect(log_capt_config, 0, 31, data_in, 0, 31, sbe);
    			when LOG_COMMAND_ADR => 
    			    --LOG_DOWN,LOG_UP,LOG_ABT,LOG_STR
    			    write_be_vect(log_cmd, 0, 3, data_in, 0, 3, sbe);     
    			when others =>
    			  
    			end case;
		  end if;
		  
		  --------------------
		  --Reading the data--
		  --------------------
		  if(srd=ACT_SRD)then 
		   
		   data_out_int      <=  (OTHERS=>'0');
		    
    			case adress(11 downto 0) is
    			  
    			  --------------------------------------
    			  --Device_ID and VERSION
			   --------------------------------------	    			  
  			   when DEVICE_ID_ADR =>     
  			       data_out_int(DEVICE_ID_H downto DEVICE_ID_L)
  			             <=  DEVICE_ID_RSTVAL;
  			       data_out_int(VER_MINOR_H downto VER_MINOR_L)
  			             <=  CTU_CAN_FD_VERSION_MINOR;
			       data_out_int(VER_MAJOR_H downto VER_MAJOR_L)
  			             <=  CTU_CAN_FD_VERSION_MAJOR;
  			     
  			   --------------------------------------
    			  --MODE Register (Mode, Command, Status of SJA1000)
			   --------------------------------------  
    			  when MODE_ADR => 
    			     
    			     --Mode register
    					  data_out_int               <=  (OTHERS=>'0');
    					  data_out_int(TSM_IND)      <=  sam_norm;
    					  data_out_int(ACF_IND)      <=  ack_forb;
    					  data_out_int(5 downto 1)   <=  mode_reg(5 downto 1);
    					  
    					  --Command register is write only!
    					  --(Read in these bytes wont return previous value)
    					  
    					  --Status register
    					  data_out_int(23 downto 16) <=  status_reg;
    					  
    					  --Retransmitt limit register
    					  data_out_int(RTRLE_IND)                <=  retr_lim_ena;
    					  data_out_int(RTR_TH_H downto RTR_TH_L) <=  retr_lim_th;
    					  data_out_int(INT_LOOP_IND)             <=  intLoopbackEna;
 				    data_out_int(ENA_IND)    					         <=  CAN_enable;
    					  data_out_int(FD_TYPE_IND)              <=  FD_type;
    					  
 				 ---------------------------------------------------------
 				 -- INT_STAT
 				 ---------------------------------------------------------
    			  when INT_STAT_ADR => 
    					  data_out_int               <=  (OTHERS => '0');
    					  
    					   --Interrupt register
    					  data_out_int(INT_COUNT - 1 downto 0)  <=  int_vector;

 				 ---------------------------------------------------------
 				 -- INT_ENA_SET
 				 ---------------------------------------------------------
    			  when INT_ENA_SET_ADR => 
    					  data_out_int               <=  (OTHERS=>'0');
    					  
    					   -- Reading this register returns the value of interrupt
    					   -- enable
    					   data_out_int(INT_COUNT - 1 downto 0)  <=  int_ena;

 				 ---------------------------------------------------------
 				 -- INT_MASK_SET
 				 ---------------------------------------------------------
    			  when INT_MASK_SET_ADR => 
    					  data_out_int               <=  (OTHERS=>'0');

    					  -- Reading this register returns the value of interrupt
					  -- mask
    					  data_out_int(INT_COUNT - 1 downto 0)  <=  int_mask;

    			  ---------------------------------------------------------
 				 -- BTR
    			  ---------------------------------------------------------
 				 when BTR_ADR => 
    					   data_out_int(PROP_H downto PROP_L)          <=  prop_norm;
    					   data_out_int(PH1_H downto PH1_L)            <=  ph1_norm;
    					   data_out_int(PH2_H downto PH2_L)            <=  ph2_norm;
 			       data_out_int(BRP_H downto BRP_L)            <=  brp_norm; 
    					   data_out_int(SJW_H downto SJW_L)            <=  sjw_norm;
    					   
 			   ---------------------------------------------------------
 				 -- BTR_FD
    			  ---------------------------------------------------------
 				 when BTR_FD_ADR =>
    					   data_out_int(PROP_FD_H downto PROP_FD_L)    <=  prop_fd;
    					   data_out_int(PH1_FD_H downto PH1_FD_L)      <=  ph1_fd;
    					   data_out_int(PH2_FD_H downto PH2_FD_L)      <=  ph2_fd;
    					   data_out_int(BRP_FD_H downto BRP_FD_L)      <=  brp_fd;
    					   data_out_int(SJW_FD_H downto SJW_FD_L)      <=  sjw_fd;
    			     
				  ----------------------------------------------------------
    			   -- EWL, ERP and FAULT_STATE
          ----------------------------------------------------------
    			   when EWL_ADR =>
    			   
    			     --Error warning limit 
    					  data_out_int(EWL_LIMIT_H downto EWL_LIMIT_L)    <=  ewl; 
    					  
    					  --Error passive treshold
    					  data_out_int(ERP_LIMIT_H downto ERP_LIMIT_L)    <=  erp; 
    					  
    					  --Fault confinment state
    					   if(error_state_type'VAL(to_integer(unsigned(
    					      stat_bus(STAT_ERROR_STATE_HIGH downto 
    					      STAT_ERROR_STATE_LOW))))=error_active)
    					   then
    					     data_out_int(ERA_IND)            <=  '1'; 
					   else 
					     data_out_int(ERA_IND)            <=  '0'; 
					   end if;
					   
    					   if(error_state_type'VAL(to_integer(unsigned(stat_bus(
    					      STAT_ERROR_STATE_HIGH downto STAT_ERROR_STATE_LOW))))
    					      =error_passive)
    					   then
    					     data_out_int(ERP_IND)            <=  '1';
					   else 
					     data_out_int(ERP_IND)            <=  '0';
					   end if;
    					   
    					   if(error_state_type'VAL(to_integer(unsigned(stat_bus(
    					     STAT_ERROR_STATE_HIGH downto STAT_ERROR_STATE_LOW))))
    					     =bus_off)
    					   then
    					     data_out_int(BOF_IND)            <=  '1'; 
					   else 
					     data_out_int(BOF_IND)            <=  '0';
					    end if;  
    					  
    					   data_out_int(31 downto 19)<=(OTHERS=>'0');
    			   
    			   ----------------------------------------------------------
    			   -- RXC, TXC 
    			   ----------------------------------------------------------
    			   when RXC_ADR => 
    					  data_out_int                   <=  (OTHERS=>'0');
    					  data_out_int(8 downto 0)       <=  
    					       stat_bus(STAT_RX_COUNTER_HIGH downto 
    					                STAT_RX_COUNTER_LOW);
    					  data_out_int(24 downto 16)     <=  
    					       stat_bus(STAT_TX_COUNTER_HIGH downto 
    					                STAT_TX_COUNTER_LOW);
    					
    					-------------------------------------------------------- 
    					-- ERR_NORM, ERR_FD
    					--------------------------------------------------------
    			   when ERR_NORM_ADR => 
    					  data_out_int                   <=  (OTHERS=>'0');
    					  data_out_int(ERR_NORM_VAL_H downto ERR_NORM_VAL_L)  <=  
    					           stat_bus(STAT_ERROR_COUNTER_NORM_HIGH downto 
    					                    STAT_ERROR_COUNTER_NORM_LOW);
    					  data_out_int(ERR_FD_VAL_H downto ERR_FD_VAL_L)     <=  
    					           stat_bus(STAT_ERROR_COUNTER_FD_HIGH downto 
    					                    STAT_ERROR_COUNTER_FD_LOW);
    			   
    			   -------------------------------------------------------- 
    					--Acceptance filters  
    					--------------------------------------------------------
    			   when FILTER_A_VAL_ADR => 
    			     if (sup_filtA) then 
    			       data_out_int(28 downto 0)       <=  filter_A_mask;
						  data_out_int(31 downto 29)      <=  (OTHERS=>'0');
						else
						  data_out_int <= (OTHERS => '0');  
						end if;
    			   when FILTER_A_MASK_ADR =>
    			     if (sup_filtA) then  
    			       data_out_int(28 downto 0)       <=  filter_A_value;
						  data_out_int(31 downto 29)      <=  (OTHERS=>'0');
						else
						  data_out_int <= (OTHERS => '0');  
						end if;  
    			   when FILTER_B_VAL_ADR =>
    			     if (sup_filtB) then 
    			       data_out_int(28 downto 0)       <=  filter_B_mask;
    						  data_out_int(31 downto 29)      <=  (OTHERS=>'0');
						else
						  data_out_int <= (OTHERS => '0');  
						end if; 
    			   when FILTER_B_MASK_ADR =>
    			     if (sup_filtB) then  
    			       data_out_int(28 downto 0)       <=  filter_B_value;
    						  data_out_int(31 downto 29)      <=  (OTHERS=>'0'); 
						else
						  data_out_int <= (OTHERS => '0');  
						end if;   
    			   when FILTER_C_VAL_ADR =>
			      if (sup_filtC) then 
    			       data_out_int(28 downto 0)       <=  filter_C_mask;
    						  data_out_int(31 downto 29)      <=  (OTHERS=>'0'); 
						else
						  data_out_int <= (OTHERS => '0');  
						end if;    
    			   when FILTER_C_MASK_ADR =>
    			     if (sup_filtC) then 
    			       data_out_int(28 downto 0)       <=  filter_C_value;
    						  data_out_int(31 downto 29)      <=  (OTHERS=>'0'); 
						else
						  data_out_int <= (OTHERS => '0');  
						end if;     
    			   when FILTER_RAN_LOW_ADR =>
    			     if (sup_range) then 
    			       data_out_int(28 downto 0)       <=  filter_ran_low;
    						  data_out_int(31 downto 29)      <=  (OTHERS=>'0');
						else
						  data_out_int <= (OTHERS => '0');  
						end if; 
    			   when FILTER_RAN_HIGH_ADR =>
    			     if (sup_range) then 
    			       data_out_int(28 downto 0)       <=  filter_ran_high;
    						  data_out_int(31 downto 29)      <=  (OTHERS=>'0');
						else
						  data_out_int <= (OTHERS => '0');  
						end if; 
    					
    			   -------------------------------------------------------
				  --Acceptance filter configuration and status register
			    -------------------------------------------------------
    			   when FILTER_CONTROL_ADR => 
    					  data_out_int(3 downto 0)       <=  filter_A_ctrl;
    					  data_out_int(7 downto 4)       <=  filter_B_ctrl;
    					  data_out_int(11 downto 8)      <=  filter_B_ctrl;
    					  data_out_int(15 downto 12)     <=  filter_ran_ctrl;
    					  
    					  if (sup_filtA) then
    					    data_out_int(SFA_IND) <= '1';
 					  else
 					    data_out_int(SFA_IND) <= '0';
 					  end if;
 					  
 					  if (sup_filtB) then
    					    data_out_int(SFB_IND) <= '1';
 					  else
 					    data_out_int(SFB_IND) <= '0';
 					  end if;
 					  
 					  if (sup_filtC) then
    					    data_out_int(SFC_IND) <= '1';
 					  else
 					    data_out_int(SFC_IND) <= '0';
 					  end if;
 					  
 					  if (sup_range) then
    					    data_out_int(SFR_IND) <= '1';
 					  else
 					    data_out_int(SFR_IND) <= '0';
 					  end if;
 					  
    					  data_out_int(31 downto 20)     <=  (OTHERS=>'0');
    					
    					-------------------------------------------------------
    			   -- RX_MEM_INFO
    			   -------------------------------------------------------  
    			   when RX_MEM_INFO_ADR =>
    					  data_out_int(31 downto 0)      <=  (OTHERS=>'0');
    					  data_out_int(RX_BUFF_SIZE_H downto RX_BUFF_SIZE_L)
    					           <= rx_buf_size;
    					  data_out_int(RX_MEM_FREE_H downto RX_MEM_FREE_L)
    					            <=  rx_mem_free;         
    					  
    					  
    			   -------------------------------------------------------
    			   -- RX_POINTERS
    			   -------------------------------------------------------  
    			   when RX_POINTERS_ADR =>
    			     data_out_int(31 downto 0)      <=  (OTHERS=>'0');
    			     data_out_int(RX_WPP_H downto RX_WPP_L)
    					           <= rx_write_pointer_pos;
    					  data_out_int(RX_RPP_H downto RX_RPP_L)
    					           <= rx_read_pointer_pos;
    					
    					
    					-------------------------------------------------------
    			   -- RX_STATUS
    			   -------------------------------------------------------  
    			   when RX_STATUS_ADR =>
    			     data_out_int                             <= (OTHERS => '0');
    					  data_out_int(RX_EMPTY_IND)               <=  rx_empty;
    					  data_out_int(RX_FULL_IND)                <=  rx_full;
    					  
    					  data_out_int(RX_FRC_H downto RX_FRC_L)
    					            <= rx_message_count;
 					  data_out_int(RTSOP_IND)                  <= rtsopt;
    					
    					
 				  -------------------------------------------------------
 				  --RX_DATA register
 				  -------------------------------------------------------
    			   when RX_DATA_ADR => 
    			     if(RX_buff_read_first=false)then
    					   data_out_int(RX_DATA_H downto RX_DATA_L) <= rx_read_buff;
					  else
					   data_out_int(RX_DATA_H downto RX_DATA_L) <= aux_data;
					  end if;
					  
    					  RX_buff_read_first              <=  true;
    			   
    			   -------------------------------------------------------
			    --Transciever delay adress  
    			   -------------------------------------------------------
    			   when TRV_DELAY_ADR =>
    			      data_out_int(31 downto 16)     <=  (OTHERS=>'0');
    			      data_out_int(TRV_DELAY_VALUE_H downto TRV_DELAY_VALUE_L)
			                <= trv_delay_out; 
 			    
 			    -------------------------------------------------------
 			    --TXT Buffers status
    			   -------------------------------------------------------
    			   when TX_STATUS_ADR => 
    			      data_out_int(31 downto 3)      <=  (OTHERS=>'0');
    			      
    			      -- We encode state here, later this will be abstracted to
    			      -- higher level module. So far we unrool it and do it
    			      -- for every Buffer state separately so that we dont have
    			      -- problems with dependencies between indices! (e.g if
    			      -- we used loop and used offset from first buffer state)
			       case txtb_fsms(0) is
			       when txt_empty =>
			            data_out_int(TX1S_H downto TX1S_L) <= TXT_ETY;
			       when txt_ready =>
			            data_out_int(TX1S_H downto TX1S_L) <= TXT_RDY;
			       when txt_tx_prog =>
			            data_out_int(TX1S_H downto TX1S_L) <= TXT_TRAN;
			       when txt_ab_prog =>
			            data_out_int(TX1S_H downto TX1S_L) <= TXT_ABTP;
			       when txt_ok =>
			            data_out_int(TX1S_H downto TX1S_L) <= TXT_TOK;
			       when txt_error =>
			            data_out_int(TX1S_H downto TX1S_L) <= TXT_ERR;
			       when txt_aborted =>
			            data_out_int(TX1S_H downto TX1S_L) <= TXT_ABT;
			       when others =>
			            data_out_int(TX1S_H downto TX1S_L) <= (OTHERS => '0');
			       end case;
			       
			       case txtb_fsms(1) is
			       when txt_empty =>
			            data_out_int(TX2S_H downto TX2S_L) <= TXT_ETY;
			       when txt_ready =>
			            data_out_int(TX2S_H downto TX2S_L) <= TXT_RDY;
			       when txt_tx_prog =>
			            data_out_int(TX2S_H downto TX2S_L) <= TXT_TRAN;
			       when txt_ab_prog =>
			            data_out_int(TX2S_H downto TX2S_L) <= TXT_ABTP;
			       when txt_ok =>
			            data_out_int(TX2S_H downto TX2S_L) <= TXT_TOK;
			       when txt_error =>
			            data_out_int(TX2S_H downto TX2S_L) <= TXT_ERR;
			       when txt_aborted =>
			            data_out_int(TX2S_H downto TX2S_L) <= TXT_ABT;
			       when others =>
			            data_out_int(TX2S_H downto TX2S_L) <= (OTHERS => '0');
			       end case;
			       
			       case txtb_fsms(2) is
			       when txt_empty =>
			            data_out_int(TX3S_H downto TX3S_L) <= TXT_ETY;
			       when txt_ready =>
			            data_out_int(TX3S_H downto TX3S_L) <= TXT_RDY;
			       when txt_tx_prog =>
			            data_out_int(TX3S_H downto TX3S_L) <= TXT_TRAN;
			       when txt_ab_prog =>
			            data_out_int(TX3S_H downto TX3S_L) <= TXT_ABTP;
			       when txt_ok =>
			            data_out_int(TX3S_H downto TX3S_L) <= TXT_TOK;
			       when txt_error =>
			            data_out_int(TX3S_H downto TX3S_L) <= TXT_ERR;
			       when txt_aborted =>
			            data_out_int(TX3S_H downto TX3S_L) <= TXT_ABT;
			       when others =>
			            data_out_int(TX3S_H downto TX3S_L) <= (OTHERS => '0');
			       end case;
			       
			       case txtb_fsms(3) is
			       when txt_empty =>
			            data_out_int(TX4S_H downto TX4S_L) <= TXT_ETY;
			       when txt_ready =>
			            data_out_int(TX4S_H downto TX4S_L) <= TXT_RDY;
			       when txt_tx_prog =>
			            data_out_int(TX4S_H downto TX4S_L) <= TXT_TRAN;
			       when txt_ab_prog =>
			            data_out_int(TX4S_H downto TX4S_L) <= TXT_ABTP;
			       when txt_ok =>
			            data_out_int(TX4S_H downto TX4S_L) <= TXT_TOK;
			       when txt_error =>
			            data_out_int(TX4S_H downto TX4S_L) <= TXT_ERR;
			       when txt_aborted =>
			            data_out_int(TX4S_H downto TX4S_L) <= TXT_ABT;
			       when others =>
			            data_out_int(TX4S_H downto TX4S_L) <= (OTHERS => '0');
			       end case;
    			
    			-- TODO: Shouldn we add this to the register map as before??     
    			      --if (tx_time_sup) then   
--			         data_out_int(TXTS_IND) <= '1';
--			       else
--			         data_out_int(TXTS_IND) <= '0';
--			       end if;
 			    
 			    ----------------------------------------------------
   			    -- TX_COMMAND (TX_SETTINGS and TX_COMMAND)
			    ----------------------------------------------------
			    when TX_COMMAND_ADR =>
    			     
    			     -- Commands and indices are read only (TX_COMMAND register)
    			     
     	      -- Buffer direction and Frame swap (TX_SETTINGS)
     	      data_out_int           <= (OTHERS => '0');
     	    
          ----------------------------------------------------
    			   -- TX_PRIORITY
    			   ----------------------------------------------------
 	        when TX_PRIORITY_ADR =>
 	          data_out_int                         <= (OTHERS => '0');
     	      data_out_int(TXT1P_H downto TXT1P_L) <= txt_buf_prior(0);
            data_out_int(TXT2P_H downto TXT2P_L) <= txt_buf_prior(1);
        	
    					------------------------------------------------------- 
 			    --Error capture register and ALC
 			    -------------------------------------------------------
  					when ERR_CAPT_ADR =>
  					  data_out_int                      <=  (OTHERS =>'0');
  					  data_out_int(7 downto 0)          <=
  					       stat_bus(STAT_ERC_HIGH downto STAT_ERC_LOW);
 					  data_out_int(ALC_VAL_H downto ALC_VAL_L)      <=  
    					        stat_bus(STAT_ALC_HIGH downto STAT_ALC_LOW); 
    					   
    		    ------------------------------------------------------- 
 			    --Frame counters registers
 			    -------------------------------------------------------  
    			   when RX_COUNTER_ADR => --Recieve message counter 
    					  data_out_int                     <=  
    					      stat_bus(STAT_RX_CTR_HIGH downto STAT_RX_CTR_LOW);
    					      
    			   when TX_COUNTER_ADR => --Transcieve message counter 
    					  data_out_int                     <=  
    					      stat_bus(STAT_TX_CTR_HIGH downto STAT_TX_CTR_LOW);
    			   
    			   ------------------------------------------------------- 
 			    --Logger configuration registers
 			    -------------------------------------------------------  
    			   when LOG_TRIG_CONFIG_ADR=>
    					  data_out_int                     <=  log_trig_config;
    			   when LOG_CAPT_CONFIG_ADR=>
    					  data_out_int                     <=  log_capt_config;
 					
 					------------------------------------------------------- 
 			    --LOG_STATUS, LOG_WPP, LOG_RPP
 			    -------------------------------------------------------  
    			   when LOG_STATUS_ADR=>
    					  --Logger status
    					  if(log_state_out=config)then 
    					     data_out_int(LOG_CFG_IND)     <=  '1'; 
					  else 
					     data_out_int(LOG_CFG_IND)     <=  '0'; 
					  end if;
					  
    					  if(log_state_out=ready)then 
    					     data_out_int(LOG_RDY_IND)     <=  '1'; 
    					  else 
    					     data_out_int(LOG_RDY_IND)     <=  '0'; 
    					  end if;
    					  
    					  if(log_state_out=running)then 
    					     data_out_int(LOG_RUN_IND)     <=  '1'; 
    					  else 
    					     data_out_int(LOG_RUN_IND)     <=  '0'; 
    					  end if;
    					  
    					  if(use_logger=true)then
    					     data_out_int(LOG_EXIST_IND)   <=  '1';
					  else
					     data_out_int(LOG_EXIST_IND)   <=  '0';
					  end if;       
    			
    					  data_out_int(6 downto 3)                         <= (OTHERS =>'0');
    					  data_out_int(LOG_SIZE_H downto LOG_SIZE_L)       <= log_size;
    					  data_out_int(LOG_WPP_H downto LOG_WPP_L) <= log_write_pointer;
    					  data_out_int(LOG_RPP_H downto LOG_RPP_L) <= log_read_pointer;
    			  
    			   when LOG_CAPT_EVENT_1_ADR=>
    					  data_out_int                     <=  
    					      loger_act_data(63 downto 32);
    					      
    			   when LOG_CAPT_EVENT_2_ADR=>
    					  data_out_int                     <=  
    					      loger_act_data(31 downto 0);
    			      			   
 			   ------------------------------------------------------- 
 			   --DEBUG register
 			   -------------------------------------------------------  
  			    when DEBUG_REGISTER_ADR =>
  			      data_out_int(7 downto 3)         <= (OTHERS =>'0');
  			      data_out_int(STUFF_COUNT_H downto STUFF_COUNT_L) <= 
  			          stat_bus(STAT_BS_CTR_HIGH downto STAT_BS_CTR_LOW);
  			      data_out_int(DESTUFF_COUNT_H downto DESTUFF_COUNT_L) <= 
  			           stat_bus(STAT_BD_CTR_HIGH downto STAT_BD_CTR_LOW);
   			      data_out_int(12 downto 6)        <= PC_state_reg_vect;
    			   
  			   ------------------------------------------------------- 
 			   --YOOOOLOOOO REGISTER
 			   ------------------------------------------------------- 
    				 when YOLO_REG_ADR =>
    				     data_out_int             <=  x"DEADBEEF";
    				     
    			   when others=>
    			  end case;    
		  end if;
		  
		end if;
	end if;
  end if;  
  end process mem_acess;
  
  
  -------------------------------------------------------------------------------
  -- Combinational logic for incrementing read pointer in RX buffer!
 	-------------------------------------------------------------------------------
  rx_read_start   <=  '1' when 
 	                        (srd=ACT_SRD and 
 	                         scs=ACT_CSC and   
		                     adress(COMP_TYPE_ADRESS_HIGHER downto 
		                            COMP_TYPE_ADRESS_LOWER)=CAN_COMPONENT_TYPE and 
		                     adress(ID_ADRESS_HIGHER downto ID_ADRESS_LOWER)=
		                                std_logic_vector(to_unsigned(ID,4)) and
		                     adress(11 downto 0)=RX_DATA_ADR and
		                     RX_buff_read_first = false)
		                     else
		                '0';
  
  -------------------------------------------------------------------------------
  -- Read data set by byte enable combinationally
  -- Registered value of the byte enable must be taken. According to Avalon
  -- spec. the byte enable signal does not have to be valid in the clock
  -- cycle when the data are returned!!
 	-------------------------------------------------------------------------------
  data_out(7 downto 0) <= data_out_int(7 downto 0) when sbe_reg(0)='1' 
                                                   else
                          (OTHERS => '0');
  
  data_out(15 downto 8) <= data_out_int(15 downto 8) when sbe_reg(1)='1' 
                                                   else
                          (OTHERS => '0');
  
  data_out(23 downto 16) <= data_out_int(23 downto 16) when sbe_reg(2)='1' 
                                                   else
                          (OTHERS => '0');
   
  data_out(31 downto 24) <= data_out_int(31 downto 24) when sbe_reg(3)='1' 
                                                   else
                          (OTHERS => '0');                       
  
  --------------------------------
  --Register logic and structure--
  --------------------------------
  PC_state <=  protocol_type'VAL(to_integer(
               unsigned(stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))));
 
  --Note: Flip flops are not used for most of the logic because all the blocks 
  --      have registered information on output, therefore it is enough to read 
  --      it directly!
  
  --Status register
  status_reg(BS_IND mod 8)<= '1' when  error_state_type'VAL(
                                   to_integer(unsigned(
                                       stat_bus(STAT_ERROR_STATE_HIGH downto 
                                                STAT_ERROR_STATE_LOW))))=bus_off 
                           else 
                       '1' when  oper_mode_type'VAL(
                                   to_integer(unsigned(
                                        stat_bus(STAT_OP_STATE_HIGH downto 
                                                 STAT_OP_STATE_LOW))))=integrating
                           else
                       '1' when  oper_mode_type'VAL(
                                    to_integer(unsigned(
                                         stat_bus(STAT_OP_STATE_HIGH downto 
                                                  STAT_OP_STATE_LOW))))=idle 
                           else
                       '0';
  status_reg(ES_IND mod 8)<='1' when (ewl<stat_bus(STAT_TX_COUNTER_HIGH downto 
                                             STAT_TX_COUNTER_LOW) or
                                ewl<stat_bus(STAT_RX_COUNTER_HIGH downto 
                                             STAT_RX_COUNTER_LOW)) else '0';
                                
  status_reg(TS_IND mod 8)<='1' when oper_mode_type'VAL(to_integer(
                               unsigned(stat_bus(STAT_OP_STATE_HIGH downto 
                                        STAT_OP_STATE_LOW))))=transciever 
                          else
                      '0';
  status_reg(RS_IND mod 8)<='1' when oper_mode_type'VAL(to_integer(
                               unsigned(stat_bus(STAT_OP_STATE_HIGH downto
                                                 STAT_OP_STATE_LOW))))=reciever
                          else
                      '0';
  
  
  status_reg(TBS_IND mod 8)<='1' when (txtb_fsms(0) = txt_empty or
                                       txtb_fsms(1) = txt_empty or
                                       txtb_fsms(2) = txt_empty or
                                       txtb_fsms(3) = txt_empty)
                                 else
                            '0';
  
  --When at least one message is availiable in the buffer
  status_reg(RBS_IND mod 8)<=not rx_empty; 
  status_reg(DOS_IND mod 8)<=rx_data_overrun;
  status_reg(ET_IND mod 8) <='1' when PC_state=error else '0';
   
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
  drv_bus(80 downto 61)                             <=  (OTHERS=>'0');
  drv_bus(349 downto 330)                           <=  (OTHERS=>'0');
  drv_bus(355 downto 354)                           <=  (OTHERS=>'0');
  drv_bus(360 downto 358)                           <=  (OTHERS=>'0');
  drv_bus(362 downto 361)                           <=  (OTHERS=>'0');
  drv_bus(365 downto 363)                           <=  (OTHERS=>'0');
  drv_bus(370 downto 368)                           <=  (OTHERS=>'0');
  drv_bus(371)                                      <=  '0';
  drv_bus(375 downto 373)                           <=  (OTHERS=>'0');
  drv_bus(399 downto 388)                           <=  (OTHERS=>'0');
  drv_bus(459 downto 445)                           <=  (OTHERS=>'0');
  drv_bus(464 downto 462)                           <=  (OTHERS=>'0');
  drv_bus(609 downto 601)                           <=  (OTHERS=>'0');
  drv_bus(579 downto 570)                           <=  (OTHERS=>'0');
  drv_bus(519 downto 511)                           <=  (OTHERS=>'0');
  drv_bus(444 downto 429)                           <=  (OTHERS=>'0');
  
  drv_bus(1023 downto 876)                          <=  (OTHERS=>'0');
  
  drv_bus(863 downto 844)                          <=  (OTHERS=>'0');
  drv_bus(831 downto 812)                          <=  (OTHERS=>'0');
  drv_bus(799 downto 780)                          <=  (OTHERS=>'0');
  drv_bus(767 downto 748)                          <=  (OTHERS=>'0');
  drv_bus(735 downto 614)                          <=  (OTHERS=>'0');


  
  --Prescaler data and bus timing
  drv_bus(DRV_TQ_NBT_HIGH downto DRV_TQ_NBT_LOW)    <=  brp_norm;
  drv_bus(DRV_TQ_DBT_HIGH downto DRV_TQ_DBT_LOW)    <=  brp_fd;
  drv_bus(DRV_PRS_NBT_HIGH downto DRV_PRS_NBT_LOW)  <=  prop_norm;
  drv_bus(DRV_PRS_DBT_HIGH downto DRV_PRS_DBT_LOW)  <=  prop_fd;
  drv_bus(DRV_PH1_NBT_HIGH downto DRV_PH1_NBT_LOW)  <=  ph1_norm;
  drv_bus(DRV_PH1_DBT_HIGH downto DRV_PH1_DBT_LOW)  <=  ph1_fd;
  drv_bus(DRV_PH2_NBT_HIGH downto DRV_PH2_NBT_LOW)  <=  ph2_norm;
  drv_bus(DRV_PH2_DBT_HIGH downto DRV_PH2_DBT_LOW)  <=  ph2_fd;
  drv_bus(DRV_SJW_HIGH downto DRV_SJW_LOW)          <=  sjw_norm;
  drv_bus(DRV_SJW_DBT_HIGH downto DRV_SJW_DBT_LOW)  <=  sjw_fd;
  
  --Acceptance filters
  drv_bus(DRV_FILTERS_ENA_INDEX)        <=  mode_reg(AFM_IND);
  drv_bus(DRV_FILTER_A_MASK_HIGH downto 
          DRV_FILTER_A_MASK_LOW)        <=  filter_A_mask;
  drv_bus(DRV_FILTER_A_BITS_HIGH downto 
          DRV_FILTER_A_BITS_LOW)        <=  filter_A_value;
  drv_bus(DRV_FILTER_A_CTRL_HIGH downto 
          DRV_FILTER_A_CTRL_LOW)        <=  filter_A_ctrl;
  drv_bus(DRV_FILTER_B_MASK_HIGH downto 
          DRV_FILTER_B_MASK_LOW)        <=  filter_B_mask;
  drv_bus(DRV_FILTER_B_BITS_HIGH downto 
          DRV_FILTER_B_BITS_LOW)        <=  filter_B_value;
  drv_bus(DRV_FILTER_B_CTRL_HIGH downto 
          DRV_FILTER_B_CTRL_LOW)        <=  filter_B_ctrl;
  drv_bus(DRV_FILTER_C_MASK_HIGH downto 
          DRV_FILTER_C_MASK_LOW)        <=  filter_C_mask;
  drv_bus(DRV_FILTER_C_BITS_HIGH downto 
          DRV_FILTER_C_BITS_LOW)        <=  filter_C_value;
  drv_bus(DRV_FILTER_C_CTRL_HIGH downto 
          DRV_FILTER_C_CTRL_LOW)        <=  filter_C_ctrl;
  drv_bus(DRV_FILTER_RAN_CTRL_HIGH downto 
          DRV_FILTER_RAN_CTRL_LOW)    <=  filter_ran_ctrl;
  drv_bus(DRV_FILTER_RAN_LO_TH_HIGH downto 
          DRV_FILTER_RAN_LO_TH_LOW)  <=  filter_ran_low;
  drv_bus(DRV_FILTER_RAN_HI_TH_HIGH downto 
          DRV_FILTER_RAN_HI_TH_LOW)  <=  filter_ran_high;
  
  --Rx Buffer
  drv_bus(DRV_ERASE_RX_INDEX)                       <=  release_recieve;
  drv_bus(DRV_READ_START_INDEX)                     <=  rx_read_start;
  drv_bus(DRV_CLR_OVR_INDEX)                        <=  clear_overrun;
  drv_bus(DRV_RTSOPT_INDEX)                         <=  rtsopt;
  
  --TXT Buffer and TX Buffer
  drv_bus(DRV_ERASE_TXT1_INDEX)                     <=  '0';
  drv_bus(DRV_ERASE_TXT2_INDEX)                     <=  '0';
  
  --Tripple sampling
  drv_bus(DRV_SAM_INDEX)                            <=  sam_norm;
  
  
  --Interrupts
  drv_bus(DRV_INT_CLR_HIGH downto DRV_INT_CLR_LOW)          
            <= int_vect_clear;
            
  drv_bus(DRV_INT_ENA_SET_HIGH downto DRV_INT_ENA_SET_LOW)      
            <= int_ena_set;
            
  drv_bus(DRV_INT_ENA_CLR_HIGH downto DRV_INT_ENA_CLR_LOW)  
            <= int_ena_clear;
            
  drv_bus(DRV_INT_MASK_SET_HIGH downto DRV_INT_MASK_SET_LOW)    
            <= int_mask_set;
            
  drv_bus(DRV_INT_MASK_CLR_HIGH downto DRV_INT_MASK_CLR_LOW)
            <= int_mask_clear;
  
  
  --Falt confinement
  drv_bus(DRV_EWL_HIGH downto DRV_EWL_LOW)          <=  ewl;  
  drv_bus(DRV_ERP_HIGH downto DRV_ERP_LOW)          <=  erp;
  
  drv_bus(DRV_CTR_VAL_HIGH downto DRV_CTR_VAL_LOW)  <=  erctr_pres_value;
  drv_bus(DRV_CTR_SEL_HIGH downto DRV_CTR_SEL_LOW)  <=  erctr_pres_mask;
  
  --CAN Core
  drv_bus(DRV_ABORT_TRAN_INDEX)                     <=  abort_transmittion;
  
  drv_bus(DRV_CAN_FD_ENA_INDEX)                     <=  mode_reg(FDE_IND);
  drv_bus(DRV_RTR_PREF_INDEX)                       <=  mode_reg(RTR_PREF_IND);
  
  --Bus monitoring = listen only mode
  drv_bus(DRV_BUS_MON_ENA_INDEX)                    <=  mode_reg(LOM_IND);
  
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
  drv_bus(DRV_TRIG_CONFIG_DATA_HIGH downto 
          DRV_TRIG_CONFIG_DATA_LOW)<=(OTHERS =>'0');
   
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
