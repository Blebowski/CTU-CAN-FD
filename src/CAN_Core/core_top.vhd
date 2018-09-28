--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
-- 
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <martin.jerabek01@gmail.com>
-- 
-- Project advisors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 
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
--  Top level entity of CAN Core covering whole functionality of CAN FD Protocol
--  Instantiates: 1*protocol_control, 1*operation_control, 1*bitStuffing_v2,    
--  1*bitDestuffing, 2*CRC, 1* tranBuffer                                       
--  Logic for switching sampling and synchronisation signals implemented here   
--  stat_bus assignment implemented here!                                       
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015  Created file
--    4.6.2016   Added drv_bus connection to the crc circuit to cover the diffe-
--               rence between ISO FD and NON ISO FD. CRC polynomial changed to 
--               be generic setting of CRC entity
--    6.6.2016   Added two more CRC circuits for CRC calculation from transmitted
--               data! This way transmitter in Data phase does not have to wait 
--               for data to be sampled to calculate CRC!! Now when unit is 
--               transmitter TX CRC is always provided to ProtocolControl and 
--               when unit is receiver RX CRC is provided!!
--   22.6.2016   1. Added rec_esi signal for error state propagation into the RX
--                  buffer.
--               2. Added explicit architecture selection for each component 
--                  (RTL)
--   23.6.2016   1. Added bit stuffing and bit destuffing counters into the 
--                  Status bus
--               2. Added timestamp into status bus for easier testing...
--   24.6.2016   1.Added data_tx_from PC to uniquely distinguish data source for
--                 internal loopback,and LOM mode. Bug fix added. Before internal
--                 loopback did not loop back the acknowledge properly! Additio-
--                 naly it still sent the data on the bus!
--               2.Added data_tx signal connection to prescaler
--   12.1.2017   Added fix of processing the FD crc. FD CRC should process only 
--               dynamic stuff bits. Another condition added into the triggerring
--               signal for CRC engines. The condition halts crc step as long as
--               data is destuffed or data_halt and fixed_destuffing method is 
--               being used!
--   3.4.2018   Removed "tranBuffer" since TX frame metadata are now available
--              on the output of TX Arbitrator combinationally ! During the
--              transmission  metadata will not change since TX arbitrator is
--              locked!
--  13.7.2018   Removed obsolete "drv_bus_mon_ena". Bus monitoring mode (or 
--              Listen only mode, it is the same thing) is realized by Protocol
--              Control which sets "int_loop_back_ena" to perform internal
--              loopback upon transmission of DOMINANT bit which should not
--              get to the bus!
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CANcomponents.ALL;
use work.CAN_FD_frame_format.ALL;
use work.CAN_FD_frame_format.all;

entity core_top is
    port(
        ------------------------------------------------------------------------
        -- System clock and Reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        -- Driving bus
        signal drv_bus                :in   std_logic_vector(1023 downto 0);

        -- Status bus
        signal stat_bus               :out  std_logic_vector(511 downto 0);

        ------------------------------------------------------------------------
        -- Tx Arbitrator interface
        ------------------------------------------------------------------------
        signal tran_data_in           :in   std_logic_vector(31 downto 0);
        signal tran_dlc_in            :in   std_logic_vector(3 downto 0);
        signal tran_is_rtr_in         :in   std_logic;

        -- TX Identifier type (0-Basic,1-Extended);
        signal tran_ident_type_in     :in   std_logic;

        -- TX Frame type (0-CAN Normal, 1-CAN FD)
        signal tran_frame_type_in     :in   std_logic;

        -- Frame should be transcieved with BRS value
        signal tran_brs_in            :in   std_logic;

        -- Signal for CAN Core that frame on the output is valid and can be 
        -- stored for transmitting
        signal tran_frame_valid_in    :in   std_logic; 

        -- CAN Core control signals for TX Arbitrator for manipulation with
        -- TXT Buffers
        signal txt_hw_cmd             :out  txt_hw_cmd_type;
        signal txtb_changed           :in   std_logic;

        -- Pointer to TXT buffer memory
        signal txt_buf_ptr            :out  natural range 0 to 19;

        -- Transition to bus off has occurred
        signal bus_off_start          :out  std_logic;

        ------------------------------------------------------------------------
        -- Recieve Buffer and Message Filter Interface
        ------------------------------------------------------------------------

        -- Message Identifier
        signal rec_ident_out          :out  std_logic_vector(28 downto 0);

        -- Data length code
        signal rec_dlc_out            :out  std_logic_vector(3 downto 0);

        -- Recieved identifier type (0-BASE Format, 1-Extended Format);
        signal rec_ident_type_out     :out  std_logic;

        -- Recieved frame type (0-Normal CAN, 1- CAN FD) 
        signal rec_frame_type_out     :out  std_logic;

        -- Recieved frame is RTR Frame(0-No, 1-Yes)
        signal rec_is_rtr_out         :out  std_logic;

        -- Frame was recieved with Bit rate shift
        signal rec_brs_out            :out  std_logic;

        -- Error state indicator
        signal rec_esi_out  	      :out  std_logic;

        -- Acknowledge that CAN frame was received succesfully and can be
        -- committed in the RX Buffer
        signal rec_message_valid_out  :out  std_logic; 

        -- Metadata are received OK, and can be stored in RX Buffer!
        signal store_metadata         :out  std_logic;

        -- Data words is available and can be stored in RX Buffer!
        signal store_data             :out  std_logic;
        signal store_data_word        :out  std_logic_vector(31 downto 0);

        -- Cancel storing of frame in RX Buffer.
        signal rec_abort              :out  std_logic;

        ------------------------------------------------------------------------
        -- Interrupt Manager Interface 
        ------------------------------------------------------------------------

        -- Arbitration was lost input
        signal arbitration_lost_out   :out  std_logic;

        -- Message stored in CAN Core was sucessfully transmitted
        signal tx_finished            :out  std_logic; 

        -- Bit Rate Was Shifted
        signal br_shifted             :out  std_logic;

        -- At least one error appeared
        signal error_valid            :out  std_logic;

        -- Error passive state changed
        signal error_passive_changed  :out  std_logic;

        -- Error warning limit reached
        signal error_warning_limit    :out  std_logic;

        ------------------------------------------------------------------------
        -- Prescaler interface 
        ------------------------------------------------------------------------

        -- Sampling signal 2 clk_sys delayed from sample_nbt
        signal sample_nbt_del_2       :in   std_logic;

        -- Sampling signal 2 clk_sys delayed from sample_dbt
        signal sample_dbt_del_2       :in   std_logic;

        -- Sampling signal of NBT for Bit destuffing
        signal sample_nbt_del_1       :in   std_logic;

        -- Sampling signal of DBT for Bit destuffing 
        signal sample_dbt_del_1       :in   std_logic;

        -- Beginning of Nominal bit time (transcieving next bit)
        signal sync_nbt               :in   std_logic;

        -- Beginning of Data bit time (transcieving next bit)
        signal sync_dbt               :in   std_logic;

        -- Bit stuffing trigger NBT
        signal sync_nbt_del_1         :in   std_logic;

        -- Bit Stuffing trigger DBT 
        signal sync_dbt_del_1         :in   std_logic;

        -- Secondary sample signal 
        signal sample_sec             :in   std_logic; 

        -- Bit destuffing trigger for secondary sample point
        signal sample_sec_del_1       :in   std_logic; 

        -- Rec trig for secondary sample point
        signal sample_sec_del_2       :in   std_logic;

        -- Command that valid hard synchronisation edge came and PC state 
        -- can be started by SOF
        signal hard_sync_edge         :in   std_logic; 

        -- Note: This signal is from Bus Sync, not Prescaler but is sample,  
        --       trigger signal therefore logically belongs here

        -- Synchronisation control signal (Hard sync, Re Sync)
        signal sync_control           :out  std_logic_vector(1 downto 0); 

        ------------------------------------------------------------------------
        -- Recieve and transcieved data interface
        ------------------------------------------------------------------------

        -- Recieved data input (valid with sample_nbt_del_1)
        signal data_rx                :in   std_logic; 

        -- Data transcieved on CAN Bus (valid with sync_nbt_del_1)
        signal data_tx                :out  std_logic; 

        ------------------------------------------------------------------------
        -- Timestamp input
        ------------------------------------------------------------------------
        signal timestamp              :in   std_logic_vector(63 downto 0);

        ------------------------------------------------------------------------
        -- BUS Synchroniser interface
        ------------------------------------------------------------------------

        -- Control signal for Sampling source
        signal sp_control             :out  std_logic_vector(1 downto 0); 

        -- Secondary sample point reset
        signal ssp_reset              :out  std_logic; 

        -- Enable calibration of transciever delay compenstation
        signal trv_delay_calib        :out  std_logic;

        -- Bit error with secondary sampling transciever! 
        signal bit_Error_sec_sam      :in   std_logic;

        signal sof_pulse              :out  std_logic

    );

    ----------------------------------------------------------------------------
    -- Driving bus aliases
    ----------------------------------------------------------------------------
    signal drv_set_ctr_val         :     std_logic_vector(31 downto 0);
    signal drv_set_rx_ctr          :     std_logic;
    signal drv_set_tx_ctr          :     std_logic;
    signal drv_int_loopback_ena    :     std_logic;
   
   
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Internal signals
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    -- Trigerring signals
    ----------------------------------------------------------------------------

    -- Transmitt trigger: Sync, beginning of bit time
    signal tran_trig               :     std_logic;

    -- Receive trigger: Sample point
    signal rec_trig                :     std_logic;

    -- Bit Stuffing trigger, 1 clk sys delayed from sample
    signal bs_trig                 :     std_logic; 

    -- Bit Destuffing trigger
    signal bds_trig                :     std_logic; 

    -- Bit Destuffing trigger delayed one clock cycle
    signal bds_trig_del_1          :     std_logic; 

    -- CRC trigger for tx chain with bit stuffing
    signal crc_tx_wbs_trig         :     std_logic; 

    -- Sample point control signals
    signal sp_control_int          :     std_logic_vector(1 downto 0);
    signal ssp_reset_int           :     std_logic;
    signal trv_delay_calib_int     :     std_logic;

    -- Synchronisation control signal
    signal sync_control_int        :     std_logic_vector(1 downto 0);

    -- Bit Rate Was Shifted
    signal br_shifted_int          :     std_logic;
   
    ----------------------------------------------------------------------------
    -- Data signals
    ----------------------------------------------------------------------------

    -- Destuffed data valid with bs_trig
    signal data_destuffed          :     std_logic; 

    -- Data for bit stuffing, either data from Core or recessive when
    -- internal loopback
    signal data_tx_before_stuff    :     std_logic; 

    -- Internal data before transcieve
    signal data_tx_int             :     std_logic; 

    -- Internal data before destuffing (output from Loopbakc multiplexor)
    signal data_rx_int             :     std_logic; 

    -- Transcieved data from CAN Core, before bit stuffing
    signal data_tx_from_PC         :     std_logic; 

    -- Internal loopBack enabled (for Bus monitoring mode)
    signal int_loop_back_ena       :     std_logic; 
   
    ----------------------------------------------------------------------------
    -- Bit Stuffing signals
    ----------------------------------------------------------------------------
    signal bs_enable               :     std_logic; --Bit Stuffing enable

    -- Whenever fixed bit stuffing should be used (CAN FD Option)
    signal fixed_stuff             :     std_logic; 

    -- Data should be halted for transcieve
    signal data_halt               :     std_logic; 

    -- Length of Bit Stuffing
    signal bs_length               :     std_logic_vector(2 downto 0); 
    signal bst_ctr                 :     natural range 0 to 7;
   
    ----------------------------------------------------------------------------
    -- Bit destuffing signals
    ----------------------------------------------------------------------------
    signal stuff_Error             :     std_logic; --Stuff Error

    -- Signal that data on output are not valid but it is a stuff bit
    signal destuffed               :     std_logic;

    signal bds_enable              :     std_logic; --Enable of the circuit
    signal stuff_Error_enable      :     std_logic; --Enable stuff Error logging

    -- Whenever fixed bit Destuffing method is used    
    signal fixed_destuff           :     std_logic; 

    -- Length of bit stuffing rule 
    signal bds_length              :     std_logic_vector(2 downto 0);

    signal dst_ctr                 :     natural range 0 to 7;

    -- Resolved signal from bit stuffing counter, or bit destuffing counter
    signal st_ctr_resolved         :     natural range 0 to 7;

    -- OP State and PC state signals
    signal OP_state                :     oper_mode_type;
    signal PC_state                :     protocol_type;
    signal arbitration_lost        :     std_logic;
    signal set_transciever         :     std_logic;
    signal set_reciever            :     std_logic;
    signal is_idle                 :     std_logic;
    signal alc                     :     std_logic_vector(7 downto 0);

    signal unknown_OP_state        :     std_logic;

    -- Transcieve buffer output
    signal tran_dlc                :     std_logic_vector(3 downto 0);
    signal tran_is_rtr             :     std_logic;
    signal tran_ident_type         :     std_logic;
    signal tran_frame_type         :     std_logic;
    signal tran_brs                :     std_logic;

    signal txt_hw_cmd_i            :  txt_hw_cmd_type;
   
    -- Fault confinement signals
    -- Error counters
    signal tx_counter_out          :     std_logic_vector(8 downto 0);
    signal rx_counter_out          :     std_logic_vector(8 downto 0);
    signal err_counter_norm_out    :     std_logic_vector(15 downto 0);
    signal err_counter_fd_out      :     std_logic_vector(15 downto 0);

    signal error_state             :     error_state_type;

    -- Form Error from PC State
    signal form_Error              :     std_logic;

    -- CRC Error from PC State
    signal CRC_Error               :     std_logic;

    signal err_capt                :     std_logic_vector(7 downto 0);   

    -- Acknowledge Error from PC State
    signal ack_Error               :     std_logic;

    -- Some of the state machines, or signals reached unknown state!! 
    -- Shouldnt happend!!
    signal unknown_state_Error     :     std_logic;

    -- Error signal for PC control FSM from fault confinement unit (Bit error 
    -- or Stuff Error appeared)
    signal bit_Error_valid         :     std_logic;
    signal stuff_Error_valid       :     std_logic;

    signal bit_Error_out           :     std_logic;

    signal tran_valid              :     std_logic;
    signal rec_valid               :     std_logic;

    -- At least one error appeared
    signal error_valid_int         :     std_logic;

    -- Error passive state changed
    signal error_passive_changed_int:   std_logic;

    -- Error warning limit was reached
    signal error_warning_limit_int:     std_logic; 

    -- Note: New Interface for fault confinement incrementation
    signal inc_one                :     std_logic;
    signal inc_eight              :     std_logic;
    signal dec_one                :     std_logic;

    signal bus_off_start_int       :     std_logic;
    
    -- Protocol control signals
    signal rec_ident               :     std_logic_vector(28 downto 0);
    signal rec_dlc                 :     std_logic_vector(3 downto 0);
    signal rec_is_rtr              :     std_logic;
    signal rec_ident_type          :     std_logic;
    signal rec_frame_type          :     std_logic;
    signal rec_brs                 :     std_logic;

    -- Recieved CRC value
    signal rec_crc                 :     std_logic_vector(20 downto 0); 

    -- Recieved Error state indicator
    signal rec_esi                 :     std_logic;

    signal ack_recieved_out        :     std_logic;

    -- CRC Interfaces  
    -- Transition from 0 to 1 erases the CRC and operation holds as long 
    -- as enable=1
    signal crc_enable              :     std_logic; 
   
    -- CRC calculated with bit Stuffing from RX Data
    signal crc15_wbs_rx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_wbs_rx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_wbs_rx            :     std_logic_vector(20 downto 0); --CRC 21

    -- CRC calculated without bit Stuffing from RX Data
    signal crc15_nbs_rx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_nbs_rx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_nbs_rx            :     std_logic_vector(20 downto 0); --CRC 21

    -- CRC calculated with bit Stuffing from TX Data
    signal crc15_wbs_tx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_wbs_tx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_wbs_tx            :     std_logic_vector(20 downto 0); --CRC 21

    -- CRC calculated without bit Stuffing from TX Data
    signal crc15_nbs_tx            :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17_nbs_tx            :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21_nbs_tx            :     std_logic_vector(20 downto 0); --CRC 21
   
   
    -- Final CRC chosen based on the type of transcieved/recieved frame
    signal crc15                   :     std_logic_vector(14 downto 0); --CRC 15
    signal crc17                   :     std_logic_vector(16 downto 0); --CRC 17
    signal crc21                   :     std_logic_vector(20 downto 0); --CRC 21

    signal data_crc_wbs            :     std_logic;
    signal data_crc_nbs            :     std_logic;

    signal crc_wbs_trig            :     std_logic;
    signal crc_nbs_trig            :     std_logic;
   
    -- Triggering signal created in this unit
    signal sync_dbt_del_2          :     std_logic; 
    signal sync_nbt_del_2          :     std_logic;

    signal tran_trig_del_1         :     std_logic;
   
    -- Bus traffic measurment
    signal tx_counter              :     std_logic_vector(31 downto 0);
    signal rx_counter              :     std_logic_vector(31 downto 0);

    -- Signals start of frame to rest of the design
    signal sof_pulse_r             :     std_logic;
    
end entity;


architecture rtl of core_top is
  
    for OP_State_comp    : operationControl  
            use entity work.operationControl(rtl);

    for PC_State_comp    : protocolControl
            use entity work.protocolControl(rtl);

    for faultConf_comp   : faultConf
            use entity work.faultConf(rtl);

    for crc_wbs_rx_comp  : canCRC
            use entity work.canCRC(rtl);

    for crc_nbs_rx_comp  : canCRC
            use entity work.canCRC(rtl);

    for crc_wbs_tx_comp  : canCRC
            use entity work.canCRC(rtl);

    for crc_nbs_tx_comp  : canCRC
            use entity work.canCRC(rtl);

    for bs_comp          : bitStuffing_v2
            use entity work.bitStuffing_v2(rtl);

    for bitDest_comp     : bitDestuffing
            use entity work.bitDestuffing(rtl);
   
begin
  
    -- Internal signals to output propagation
    data_tx               <= data_tx_int;
    sp_control            <= sp_control_int;
    sync_control          <= sync_control_int;
    txt_hw_cmd            <= txt_hw_cmd_i;

    rec_ident_out         <= rec_ident;
    rec_dlc_out           <= rec_dlc;
    rec_ident_type_out    <= rec_ident_type;
    rec_frame_type_out    <= rec_frame_type;
    rec_is_rtr_out        <= rec_is_rtr;
    rec_brs_out           <= rec_brs;
    rec_esi_out           <= rec_esi;
  
    -- Confirmation about valid recieved data for RX Buffer
    rec_message_valid_out <=  rec_valid; 
    --NOTE: HandShake protocol with acknowledge not used in the end

    -- Transceive CAN frame is routed directly
    tran_dlc              <= tran_dlc_in;
    tran_is_rtr           <= tran_is_rtr_in;
    tran_ident_type       <= tran_ident_type_in;
    tran_frame_type       <= tran_frame_type_in;
    tran_brs              <= tran_brs_in;


    ----------------------------------------------------------------------------
    -- Operation control state machine
    ----------------------------------------------------------------------------
    OP_State_comp : operationControl
        port map(
            clk_sys             =>  clk_sys,
            res_n               =>  res_n,
            drv_bus             =>  drv_bus,
            arbitration_lost    =>  arbitration_lost,
            PC_State            =>  PC_State,
            tran_data_valid_in  =>  tran_frame_valid_in,
            set_transciever     =>  set_transciever,
            set_reciever        =>  set_reciever,
            unknown_OP_state    =>  unknown_OP_state,
            is_idle             =>  is_idle,
            tran_trig           =>  tran_trig,
            rec_trig            =>  rec_trig,
            data_rx             =>  data_rx,
            OP_State            =>  OP_State
        );


    ----------------------------------------------------------------------------
    -- Protocol control state machine
    ----------------------------------------------------------------------------
    PC_State_comp : protocolControl
        port map(
            clk_sys            =>  clk_sys,
            res_n              =>  res_n,
            drv_bus            =>  drv_bus,

            int_loop_back_ena  =>  int_loop_back_ena,
            PC_State_out       =>  PC_State,
            alc                =>  alc,
             
            tran_data          =>  tran_data_in,
            tran_dlc           =>  tran_dlc,
            tran_is_rtr        =>  tran_is_rtr,
            tran_ident_type    =>  tran_ident_type,
            tran_frame_type    =>  tran_frame_type,
            tran_brs           =>  tran_brs,
            br_shifted         =>  br_shifted_int,
            txt_buf_ptr        =>  txt_buf_ptr,

            hard_sync_edge     =>  hard_sync_edge,

            tran_frame_valid_in=>  tran_frame_valid_in,

            txt_hw_cmd         =>  txt_hw_cmd_i,
            txtb_changed       =>  txtb_changed,

            rec_ident          =>  rec_ident,
            rec_dlc            =>  rec_dlc,
            rec_is_rtr         =>  rec_is_rtr,
            rec_ident_type     =>  rec_ident_type,
            rec_frame_type     =>  rec_frame_type,
            rec_brs            =>  rec_brs,
            rec_crc            =>  rec_crc,
            rec_esi            =>  rec_esi,

            store_metadata     =>  store_metadata,
            rec_abort          =>  rec_abort,
            store_data         =>  store_data,
            store_data_word    =>  store_data_word,

            OP_state           =>  OP_state,
            arbitration_lost   =>  arbitration_lost,
            is_idle            =>  is_idle,
            set_transciever    =>  set_transciever,
            set_reciever       =>  set_reciever,

            error_state            =>  error_state,
            form_Error             =>  form_Error,
            CRC_Error              =>  CRC_Error,
            ack_Error              =>  ack_Error,

            bit_Error_valid        =>  bit_Error_valid,
            stuff_Error_valid      =>  stuff_Error_valid,

            inc_one            =>  inc_one,
            inc_eight          =>  inc_eight,
            dec_one            =>  dec_one,

            tran_trig          =>  tran_trig,
            rec_trig           =>  rec_trig,

            data_tx            =>  data_tx_from_PC,
            data_rx            =>  data_destuffed,

            ack_recieved_out   =>  ack_recieved_out,

            stuff_enable       =>  bs_enable,
            fixed_stuff        =>  fixed_stuff,
            stuff_length       =>  bs_length,

            destuff_enable     =>  bds_enable,
            stuff_error_enable =>  stuff_error_enable,
            fixed_destuff      =>  fixed_destuff,
            destuff_length     =>  bds_length,
            dst_ctr            =>  st_ctr_resolved,

            unknown_OP_state   =>  unknown_OP_state,

            crc_enable         =>  crc_enable,
            crc15              =>  crc15,
            crc17              =>  crc17,
            crc21              =>  crc21,

            rec_valid          =>  rec_valid,
            tran_valid         =>  tran_valid,

            sync_control       =>  sync_control_int,
            sp_control         =>  sp_control_int,
            ssp_reset          =>  ssp_reset_int,
            trv_delay_calib    =>  trv_delay_calib_int,

            sof_pulse          =>  sof_pulse_r
        );


    ----------------------------------------------------------------------------
    -- Fault confinement
    ----------------------------------------------------------------------------
    faultConf_comp : faultConf 
        port map(
            clk_sys                =>  clk_sys,
            res_n                  =>  res_n,
            drv_bus                =>  drv_bus,

            stuff_Error            =>  stuff_Error,

            error_valid            =>  error_valid_int,
            error_passive_changed  =>  error_passive_changed_int,
            error_warning_limit    =>  error_warning_limit_int,

            OP_State               =>  OP_State,

            data_rx                =>  data_destuffed,
            data_tx                =>  data_tx_before_stuff,
            rec_trig               =>  rec_trig,
            tran_trig_1            =>  bs_trig,

            inc_one                =>  inc_one,
            inc_eight              =>  inc_eight,
            dec_one                =>  dec_one,
            bit_Error_out          =>  bit_Error_out,

            PC_State               =>  PC_state,
            sp_control             =>  sp_control_int,
            form_Error             =>  form_Error,
            CRC_Error              =>  CRC_Error,
            ack_Error              =>  ack_Error,

            bit_Error_valid        => bit_Error_valid,
            stuff_Error_valid      => stuff_Error_valid,

            err_capt               => err_capt,
            bus_off_start          => bus_off_start_int,

            enable                 =>  '1',
            bit_Error_sec_sam      =>  bit_Error_sec_sam,

            tx_counter_out         =>  tx_counter_out,
            rx_counter_out         =>  rx_counter_out,
            err_counter_norm_out   =>  err_counter_norm_out,
            err_counter_fd_out     =>  err_counter_fd_out,

            error_state_out        =>  error_state
        );


    ----------------------------------------------------------------------------
    -- CRC with bit stuffing from RX Data
    ----------------------------------------------------------------------------
    crc_wbs_rx_comp : canCRC 
        generic map(
            crc15_pol              =>  CRC15_POL,
            crc17_pol              =>  CRC17_POL,
            crc21_pol              =>  CRC21_POL
        )
        port map(
            data_in                =>  data_crc_wbs,
            clk_sys                =>  clk_sys,
            trig                   =>  crc_wbs_trig,
            res_n                  =>  res_n,
            enable                 =>  crc_enable,
            drv_bus                =>  drv_bus,
            crc15                  =>  crc15_wbs_rx,
            crc17                  =>  crc17_wbs_rx,
            crc21                  =>  crc21_wbs_rx
        ); 


    ----------------------------------------------------------------------------
    -- CRC no bit stuffing from RX Data
    ----------------------------------------------------------------------------
    crc_nbs_rx_comp : canCRC 
        generic map(
            crc15_pol              =>  CRC15_POL,
            crc17_pol              =>  CRC17_POL,
            crc21_pol              =>  CRC21_POL
        )
        port map(
            data_in                =>  data_crc_nbs,
            clk_sys                =>  clk_sys,
            trig                   =>  crc_nbs_trig,
            res_n                  =>  res_n,
            enable                 =>  crc_enable,
            drv_bus                =>  drv_bus,
            crc15                  =>  crc15_nbs_rx,
            crc17                  =>  crc17_nbs_rx,
            crc21                  =>  crc21_nbs_rx
        );


    ----------------------------------------------------------------------------
    -- CRC with bit stuffing from TX Data
    ----------------------------------------------------------------------------
    crc_wbs_tx_comp : canCRC 
        generic map(
            crc15_pol              =>  CRC15_POL,
            crc17_pol              =>  CRC17_POL,
            crc21_pol              =>  CRC21_POL
        )
        port map(
            data_in                =>  data_tx_int,
            clk_sys                =>  clk_sys,
            trig                   =>  crc_tx_wbs_trig,
            res_n                  =>  res_n,
            enable                 =>  crc_enable,
            drv_bus                =>  drv_bus,
            crc15                  =>  crc15_wbs_tx,
            crc17                  =>  crc17_wbs_tx,
            crc21                  =>  crc21_wbs_tx
        );


    ----------------------------------------------------------------------------
    -- CRC no bit stuffing from TX Data
    ----------------------------------------------------------------------------
    crc_nbs_tx_comp : canCRC 
        generic map(
         crc15_pol              =>  CRC15_POL,
         crc17_pol              =>  CRC17_POL,
         crc21_pol              =>  CRC21_POL
        )
        port map(
         data_in                =>  data_tx_before_stuff,
         clk_sys                =>  clk_sys,
         
         -- TX no bit stuffing crc is calculated with the same
         -- trigger as bit stuffing
         trig                   =>  tran_trig_del_1,  
         
         res_n                  =>  res_n,
         enable                 =>  crc_enable,
         drv_bus                =>  drv_bus,
         crc15                  =>  crc15_nbs_tx,
         crc17                  =>  crc17_nbs_tx,
         crc21                  =>  crc21_nbs_tx
        ); 


    ----------------------------------------------------------------------------
    -- Bit Stuffing
    ----------------------------------------------------------------------------
    bs_comp : bitStuffing_v2  
        port map(
            clk_sys             =>  clk_sys,
            res_n               =>  res_n,
            tran_trig_1         =>  bs_trig,
            enable              =>  bs_enable,
            data_in             =>  data_tx_before_stuff,
            fixed_stuff         =>  fixed_stuff,
            data_halt           =>  data_halt,
            length              =>  bs_length,
            bst_ctr             =>  bst_ctr,
            data_out            =>  data_tx_int   
        );


    ----------------------------------------------------------------------------
    -- Bit De-Stuffing
    ----------------------------------------------------------------------------
    bitDest_comp : bitDestuffing 
        port map(
            clk_sys                =>  clk_sys,
            res_n                  =>  res_n,
            data_in                =>  data_rx_int,
            trig_spl_1             =>  bds_trig,
            stuff_Error            =>  stuff_Error,
            data_out               =>  data_destuffed,
            destuffed              =>  destuffed,
            enable                 =>  bds_enable,
            stuff_Error_enable     =>  stuff_Error_enable,
            fixed_stuff            =>  fixed_stuff,
            length                 =>  bds_length,
            dst_ctr                =>  dst_ctr
        );
  


    ----------------------------------------------------------------------------
    -- CRC From RX Data can't be always calculated from RX Data! In case of
    -- secondary sampling, bit length might be shorter than Propagation from TX
    -- to RX over transceiver! Thus in case of secondary sampling, even CRC
    -- from RX data is calculated from TX Data!
    ----------------------------------------------------------------------------
    data_crc_wbs  <=  data_tx_int when sp_control_int = SECONDARY_SAMPLE else 
                      data_rx_int;
                    
    data_crc_nbs  <=  data_tx_before_stuff when sp_control_int = SECONDARY_SAMPLE
                                           else 
                      data_destuffed;


    ----------------------------------------------------------------------------
    -- Driving bus aliases
    ----------------------------------------------------------------------------
    drv_set_ctr_val       <=  drv_bus(DRV_SET_CTR_VAL_HIGH downto 
                                      DRV_SET_CTR_VAL_LOW);
    drv_set_rx_ctr        <=  drv_bus(DRV_SET_RX_CTR_INDEX);
    drv_set_tx_ctr        <=  drv_bus(DRV_SET_TX_CTR_INDEX);
    drv_int_loopback_ena  <=  drv_bus(DRV_INT_LOOBACK_ENA_INDEX);


    ----------------------------------------------------------------------------
    -- Output propagation
    ----------------------------------------------------------------------------
    arbitration_lost_out   <=  arbitration_lost;
    tx_finished            <=  tran_valid;
    sof_pulse              <=  sof_pulse_r;
    bus_off_start          <=  bus_off_start_int;
    br_shifted             <=  br_shifted_int;
    ssp_reset              <=  ssp_reset_int;
    trv_delay_calib        <=  trv_delay_calib_int;


    ----------------------------------------------------------------------------
    -- CRC Multiplexors.
    -- 
    -- CRC data sources like so:
    --  1. Transceiver, CAN FD Frame  -> TX Data after bit stuffing.
    --  2. Transceiver, CAN 2.0 Frame -> TX Data before bit stuffing.
    --  3. Receiver,    CAN FD Frame  -> RX Data before bit destuffing.
    --  4. Receiver,    CAN 2.0 Frame -> RX Data after bit destuffing.
    ----------------------------------------------------------------------------
    crc15   <=  crc15_wbs_tx when (OP_State = transciever and 
                                   tran_frame_type = FD_CAN) else

                crc15_nbs_tx when (OP_State = transciever and 
                                   tran_frame_type = NORMAL_CAN) else

                crc15_wbs_rx when (OP_State = reciever and
                                   rec_frame_type  = FD_CAN) else

                crc15_nbs_rx when (OP_State = reciever and
                                   rec_frame_type  = NORMAL_CAN) else

                "000000000000000";
              
    crc17   <=  crc17_wbs_tx when (OP_State  = transciever and 
                                   tran_frame_type = FD_CAN) else

                crc17_nbs_tx when (OP_State  = transciever and 
                                   tran_frame_type = NORMAL_CAN) else

                crc17_wbs_rx when (OP_State  = reciever and
                                   rec_frame_type  = FD_CAN) else

                crc17_nbs_rx when (OP_State  = reciever and
                                   rec_frame_type  = NORMAL_CAN) else

                "00000000000000000";
        
    crc21   <=  crc21_wbs_tx when (OP_State = transciever and 
                                   tran_frame_type = FD_CAN) else

                crc21_nbs_tx when (OP_State = transciever and
                                   tran_frame_type = NORMAL_CAN) else

                crc21_wbs_rx when (OP_State = reciever and 
                                   rec_frame_type  = FD_CAN) else

                crc21_nbs_rx when (OP_State = reciever and 
                                   rec_frame_type  = NORMAL_CAN) else

                "000000000000000000000";
 
 
    ----------------------------------------------------------------------------
    -- Multiplexing of stuff counter and destuff counter
    ----------------------------------------------------------------------------
    st_ctr_resolved <= dst_ctr when (OP_State = reciever) else
                       bst_ctr when (OP_State = transciever) else
                       0;
      	 	           
 
    ----------------------------------------------------------------------------
    -- Trigger signals registering. Creating delayed signals
    ----------------------------------------------------------------------------
    trig_cr : process(clk_sys, res_n)
    begin
        if (res_n = ACT_RESET) then
            sync_dbt_del_2        <=  '0';
            sync_nbt_del_2        <=  '0';
            tran_trig_del_1       <=  '0';
            bds_trig_del_1        <=  '0';
        elsif rising_edge(clk_sys) then
            sync_dbt_del_2        <=  sync_dbt_del_1;
            sync_nbt_del_2        <=  sync_nbt_del_1;
            tran_trig_del_1       <=  tran_trig;
            bds_trig_del_1        <=  bds_trig;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Multiplexors for TX/RX Triggers. These Triggers are used to
    -- transmitt/receive data from/to Protocol Control.
    -- Note that Secondary sampling point is used only by Bus sync for bit
    -- error detection!
    ----------------------------------------------------------------------------
    tran_trig   <=  sync_nbt and (not data_halt) 
                        when (sp_control_int = NOMINAL_SAMPLE) else

                    sync_dbt and (not data_halt) 
                        when (sp_control_int = DATA_SAMPLE) else

                    sync_dbt and (not data_halt)
                        when (sp_control_int = SECONDARY_SAMPLE) else
 
                    '0';

    rec_trig   <=   sample_nbt_del_2 and (not destuffed) 
                        when (sp_control_int = NOMINAL_SAMPLE) else

                    sample_dbt_del_2 and (not destuffed) 
                        when (sp_control_int = DATA_SAMPLE) else

                    sample_dbt_del_2 and (not data_halt) 
                        when (sp_control_int = SECONDARY_SAMPLE) else

                    '0';


    ----------------------------------------------------------------------------
    -- Multiplexors for Bit Stuffing/Destuffing triggers. These are used to
    -- process data by Bit Stuffing/Destuffing circuits.
    -- Note that Secondary sampling point is used only by Bus sync for bit
    -- error detection!
    ----------------------------------------------------------------------------
    bs_trig  <= sync_nbt_del_1 when (sp_control_int = NOMINAL_SAMPLE)     else
                sync_dbt_del_1 when (sp_control_int = DATA_SAMPLE)        else
                sync_dbt_del_1 when (sp_control_int = SECONDARY_SAMPLE)   else
                '0';
          
    bds_trig <= sample_nbt_del_1  when (sp_control_int = NOMINAL_SAMPLE)    else
                sample_dbt_del_1  when (sp_control_int = DATA_SAMPLE)       else
                sync_dbt_del_1    when (sp_control_int = SECONDARY_SAMPLE)  else
                '0';


    ----------------------------------------------------------------------------
    -- According to CAN FD specification fixed stuff bits are never included in
    -- CRC calculation. With NON ISO there is no problem. CRC calculation in the
    -- end of data phase! With ISO FD CRC should be still calculated from
    -- stuff_count and parity, but NOT from the preceding fixed stuff bit. 
    -- CRC calculation must be halted for every fixed stuff bit, since CRC for
    -- FD should be calculated only from dynamic stuff bits!! Additionally CRC
    -- with bit stuffing must be calculated one clock cycle later, since it 
    -- needs information whether there was a fixed bit destuffed which is 
    -- provided at the same time as bit destuffing! Thus if the same trigger as
    -- is used as for destuffing, CRC calculation is wrong. 
    -- We additionally delay the bds_trigger!
    ----------------------------------------------------------------------------
    crc_wbs_trig <=  '0'          when (fixed_stuff = '1' and destuffed = '1')
                                  else
                  sync_dbt_del_2  when (sp_control_int = SECONDARY_SAMPLE)
                                  else 
                  bds_trig_del_1;

                  
    crc_nbs_trig <=  (sync_dbt and (not data_halt))  
                     when (sp_control_int = SECONDARY_SAMPLE) 
                     else  rec_trig;

    crc_tx_wbs_trig <= '0'         when (fixed_stuff = '1' and data_halt = '1') else
                    sync_nbt_del_2 when (sp_control_int = NOMINAL_SAMPLE) else
                    sync_dbt_del_2 when (sp_control_int = DATA_SAMPLE) else
                    sync_dbt_del_2 when (sp_control_int = SECONDARY_SAMPLE) else
                    '0';
              
    error_valid            <=  error_valid_int;
    error_passive_changed  <=  error_passive_changed_int;
    error_warning_limit    <=  error_warning_limit_int;


    ----------------------------------------------------------------------------
    -- Data Received by Protocol control are from Bit Destuffing, apart from
    -- following cases:
    --  1. Internal loopback is set by Protocol control (due to some special
    --     mode such as LOM), or rerouting acknowledge internally in case of
    --     STM mode.
    --  2. Internal loopback is permanently turned on from SW for debugging!
    --  3. Secondary sampling point is set. This is ONLY in Data phase of
    --     CAN FD Transmitter after Bit rate shift! This is needed for proper
    --     reception of own data in the same bit! Note that Bit error in this
    --     case is detected by Bus Sync circuit which has shift registers for
    --     secondary sampling point!   
    ----------------------------------------------------------------------------
    data_rx_int <= data_tx_from_PC when (int_loop_back_ena = '1' or 
                                         drv_int_loopback_ena = '1' or 
                                         sp_control_int = SECONDARY_SAMPLE) 
                                   else
                   data_rx;
 
    ----------------------------------------------------------------------------
    -- Data transmitted by Protocol control are sent out of CAN Core to
    -- Bit stuffing, apart from following cases:
    --  1. Protocol Control has "int_loop_back_ena" which forbids bit to be
    --     transmitted further!
    --  2. User has set (for debugging purposes) permanent Internal Loopback!
    ----------------------------------------------------------------------------
    data_tx_before_stuff <= RECESSIVE when (int_loop_back_ena = '1' or 
                                            drv_int_loopback_ena = '1') 
                                      else
                            data_tx_from_PC;


    ----------------------------------------------------------------------------
    -- Bus traffic measurement
    ----------------------------------------------------------------------------
    bus_tra_proc : process(clk_sys, res_n)
    begin
        if (res_n = ACT_RESET) then
            tx_counter           <= (OTHERS => '0');
            rx_counter           <= (OTHERS => '0');
        elsif rising_edge(clk_sys) then
            tx_counter           <= tx_counter;
            rx_counter           <= rx_counter;

            if (drv_set_rx_ctr = '1') then
                rx_counter        <=  drv_set_ctr_val;
            elsif (rec_valid = '1') then
                rx_counter        <=  std_logic_vector(to_unsigned(
                                        to_integer(unsigned(rx_counter)) + 1,
                                        rx_counter'length));

            end if;

            if (drv_set_tx_ctr = '1') then
                tx_counter        <=  drv_set_ctr_val;
            elsif (tran_valid = '1') then
                tx_counter        <=  std_logic_vector(to_unsigned(
                                        to_integer(unsigned(tx_counter)) + 1,
                                        tx_counter'length));
            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- STATUS Bus Implementation
    ----------------------------------------------------------------------------
    stat_bus(511 downto 370)                              <= (OTHERS => '0');
    stat_bus(299 downto 297)                              <= (OTHERS => '0');
    stat_bus(187 downto 183)                              <= (OTHERS => '0');
    stat_bus(99 downto 90)                                <= (OTHERS => '0');
    stat_bus(60 downto 32)                                <= (OTHERS => '0');

    stat_bus(STAT_BR_SHIFTED)                             <= br_shifted_int;

    stat_bus(STAT_ERC_HIGH downto STAT_ERC_LOW)           <= err_capt;

    stat_bus(STAT_OP_STATE_HIGH downto STAT_OP_STATE_LOW)      
        <=  std_logic_vector(to_unsigned(oper_mode_type'pos(OP_State),2));
        
    stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW)      
        <=  std_logic_vector(to_unsigned(protocol_type'pos(PC_State),4));
        
    stat_bus(STAT_ARB_LOST_INDEX)                         <=  arbitration_lost;
    stat_bus(STAT_SET_TRANSC_INDEX)                       <=  set_transciever;
    stat_bus(STAT_SET_REC_INDEX)                          <=  set_reciever;
    stat_bus(STAT_IS_IDLE_INDEX)                          <=  is_idle;

    stat_bus(STAT_SP_CONTROL_HIGH downto STAT_SP_CONTROL_LOW)
        <=  sp_control_int;

    stat_bus(STAT_SSP_RESET_INDEX)                        <=  ssp_reset_int;

    stat_bus(STAT_TRV_DELAY_CALIB_INDEX)
        <=  trv_delay_calib_int;

    stat_bus(STAT_SYNC_CONTROL_HIGH downto STAT_SYNC_CONTROL_LOW)
        <=  sync_control_int;

    stat_bus(STAT_DATA_TX_INDEX)                               <=  data_tx_int;
    stat_bus(STAT_DATA_RX_INDEX)                               <=  data_rx_int;

    stat_bus(STAT_BS_ENABLE_INDEX)                             <=  bs_enable;
    stat_bus(STAT_FIXED_STUFF_INDEX)                           <=  fixed_stuff;
    stat_bus(STAT_DATA_HALT_INDEX)                             <=  data_halt;
    stat_bus(STAT_BS_LENGTH_HIGH downto STAT_BS_LENGTH_LOW)    <=  bs_length;

    stat_bus(STAT_STUFF_ERROR_INDEX)                           <=  stuff_Error;
    stat_bus(STAT_DESTUFFED_INDEX)                             <=  destuffed;
    stat_bus(STAT_BDS_ENA_INDEX)                               <=  bds_enable;

    stat_bus(STAT_STUFF_ERRROR_ENA_INDEX)                      
        <=  stuff_Error_enable;

    stat_bus(STAT_FIXED_DESTUFF_INDEX)                         <=  fixed_destuff;
    stat_bus(STAT_BDS_LENGTH_HIGH downto STAT_BDS_LENGTH_LOW)  <=  bds_length;
 
    stat_bus(STAT_TRAN_DLC_HIGH downto STAT_TRAN_DLC_LOW)
        <=  tran_dlc;

    stat_bus(STAT_TRAN_IS_RTR_INDEX)
        <=  tran_is_rtr;

    stat_bus(STAT_TRAN_IDENT_TYPE_INDEX)
        <=  tran_ident_type;

    stat_bus(STAT_TRAN_FRAME_TYPE_INDEX)
        <=  tran_frame_type;

    stat_bus(STAT_TRAN_DATA_ACK_INDEX)
        <=  txt_hw_cmd_i.lock;

    stat_bus(STAT_TRAN_BRS_INDEX)
        <=  tran_brs;

    stat_bus(STAT_FRAME_STORE_INDEX)
        <=  txt_hw_cmd_i.lock;

    -- Error counters and state
    stat_bus(STAT_TX_COUNTER_HIGH downto STAT_TX_COUNTER_LOW)
        <=  tx_counter_out;

    stat_bus(STAT_RX_COUNTER_HIGH downto STAT_RX_COUNTER_LOW)
        <=  rx_counter_out;

    stat_bus(STAT_ERROR_COUNTER_NORM_HIGH downto STAT_ERROR_COUNTER_NORM_LOW)
        <=  err_counter_norm_out;

    stat_bus(STAT_ERROR_COUNTER_FD_HIGH downto STAT_ERROR_COUNTER_FD_LOW)
        <=  err_counter_fd_out;

    stat_bus(STAT_ERROR_STATE_HIGH downto STAT_ERROR_STATE_LOW)                             
        <= std_logic_vector(to_unsigned(error_state_type'pos(error_state), 2));
   
    -- Error signals
    stat_bus(STAT_FORM_ERROR_INDEX)                      <=  form_Error;
    stat_bus(STAT_CRC_ERROR_INDEX)                       <=  CRC_Error;
    stat_bus(STAT_ACK_ERROR_INDEX)                       <=  ack_Error;
    stat_bus(STAT_UNKNOWN_STATE_ERROR_INDEX)             <=  '0';
    stat_bus(STAT_BIT_STUFF_ERROR_INDEX)                 
        <=  bit_Error_valid or stuff_Error_valid;
 
    stat_bus(STAT_FIRST_BIT_AFTER_INDEX)                       <=  '0';
    stat_bus(STAT_REC_VALID_INDEX)                             <=  rec_valid;
    stat_bus(STAT_TRAN_VALID_INDEX)                            <=  tran_valid;
    stat_bus(STAT_CONST7_INDEX)                                <=  '0';
    stat_bus(STAT_CONST14_INDEX)                               <=  '0';

    stat_bus(STAT_TRANSM_ERROR_INDEX)                          <=  '0'; 
       
    -- Recieved data interfac
    stat_bus(STAT_REC_IDENT_HIGH downto STAT_REC_IDENT_LOW)    <=  rec_ident;
    stat_bus(STAT_REC_DLC_HIGH downto STAT_REC_DLC_LOW)        <=  rec_dlc;
    stat_bus(STAT_REC_IS_RTR_INDEX)                            <=  rec_is_rtr;
    stat_bus(STAT_REC_IDENT_TYPE_INDEX)                        <=  rec_ident_type;
    stat_bus(STAT_REC_FRAME_TYPE_INDEX)                        <=  rec_frame_type;
    stat_bus(STAT_REC_BRS_INDEX)                               <=  rec_brs;
    stat_bus(STAT_REC_CRC_HIGH downto STAT_REC_CRC_LOW)        <=  rec_crc;
    stat_bus(STAT_REC_ESI_INDEX)                               <=  rec_esi;

    stat_bus(STAT_CRC_ENA_INDEX)                               <=  crc_enable;

    stat_bus(STAT_TRAN_TRIG)                                   <=  tran_trig;
    stat_bus(STAT_REC_TRIG)                                    <=  rec_trig;
    stat_bus(STAT_ALC_HIGH downto STAT_ALC_LOW)                <=  alc;

    stat_bus(STAT_RX_CTR_HIGH downto STAT_RX_CTR_LOW)          <=  rx_counter;
    stat_bus(STAT_TX_CTR_HIGH downto STAT_TX_CTR_LOW)          <=  tx_counter;

    stat_bus(STAT_ERP_CHANGED_INDEX)             <=  error_passive_changed_int;
    stat_bus(STAT_EWL_REACHED_INDEX)             <=  error_warning_limit_int;
    stat_bus(STAT_ERROR_VALID_INDEX)             <=  error_valid_int;
 
    stat_bus(STAT_ACK_RECIEVED_OUT_INDEX)                  <=  ack_recieved_out;
    stat_bus(STAT_BIT_ERROR_VALID_INDEX)                   <=  bit_Error_out;

    stat_bus(STAT_BS_CTR_HIGH downto STAT_BS_CTR_LOW)          
          <=  std_logic_vector(to_unsigned(bst_ctr, 3));
          
    stat_bus(STAT_BD_CTR_HIGH downto STAT_BD_CTR_LOW)          
          <=  std_logic_vector(to_unsigned(dst_ctr, 3)); 

    stat_bus(STAT_TS_HIGH downto STAT_TS_LOW)              <=  timestamp;
 
 
end architecture;


