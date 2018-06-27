Library ieee;
use ieee.std_logic_1164.all;

-------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
--
--    June 2015   Created file
--
-------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Package with Constants, types and other defintions for CAN FD IP Core
-------------------------------------------------------------------------------------------------------------

package CANconstants is
       
  --Active value of asynchronous reset 
  constant ACT_RESET:std_logic:='0';
  
  --Definition of basic logic levels for CAN bus
  constant DOMINANT:std_logic:='0';
  constant RECESSIVE:std_logic:='1';
    
  --Definition of frame types identifiers (Basic, Extended ), equal to IDE bit of Arbitration field
  constant FRAME_BASIC:std_logic:='0';
  constant FRAME_EXTENDED:std_logic:='1';
  
  constant INTEGRATING_DURATION:natural:=11;
  constant TRAN_BUFF_SIZE:natural:=600;
  
  constant BASE_STUFF_LENGTH:natural:=5;
  constant FD_STUFF_LENGTH:natural:=4;
  
  constant CAN_BASE_ID_LENGTH:natural:=11;
  constant CAN_EXT_ID_LENGTH:natural:=18; --Length Identifier extension only
  
  constant BASE:std_logic:='0';
  constant EXTENDED:std_logic:='1';
  constant NORMAL_CAN:std_logic:='0';
  constant FD_CAN:std_logic:='1';
  
  constant NO_SYNC:std_logic_vector:="00";
  constant HARD_SYNC:std_logic_vector:="01";
  constant RE_SYNC:std_logic_vector:="10";
  
  --CRC sources
  constant CRC_15_SRC:std_logic_vector:="00";
  constant CRC_17_SRC:std_logic_vector:="01";
  constant CRC_21_SRC:std_logic_vector:="10";
  
  --Sample point control constants
  constant NOMINAL_SAMPLE:std_logic_vector(1 downto 0):="00";
  constant DATA_SAMPLE:std_logic_vector(1 downto 0):="01";
  constant SECONDARY_SAMPLE:std_logic_vector(1 downto 0):="10";
  
  --Tuples definition for older compiler (less than 2008)
  constant DOMINANT_DOMINANT:std_logic_vector:=DOMINANT&DOMINANT;
  constant DOMINANT_RECESSIVE:std_logic_vector:=DOMINANT&RECESSIVE;
  constant RECESSIVE_DOMINANT:std_logic_vector:=RECESSIVE&DOMINANT;
  constant RECESSIVE_RECESSIVE:std_logic_vector:=RECESSIVE&RECESSIVE;
  
  --Error flag definitions 
  constant PASSIVE_ERR_FLAG:std_logic:=RECESSIVE;
  constant ACTIVE_ERR_FLAG:std_logic:=DOMINANT;
  
  constant ERROR_FLAG_LENGTH:natural:=6;
  
  constant INC_ONE_CON:std_logic_vector(2 downto 0):="100";
  constant INC_EIGHT_CON:std_logic_vector(2 downto 0):="010";
  constant DEC_ONE_CON:std_logic_vector(2 downto 0):="001";
  
  --Values for enabling of whole controller
  --and interrupts
  constant ENABLED: std_logic:='1';
  constant DISABLED: std_logic:='0';
  
  --Values for the type of FD frame type which whould be used
  constant ISO_FD: std_logic:='0';
  constant NON_ISO_FD: std_logic:='1';
  
  --DLC Types
  type dlc_type is array (0 to 15) of std_logic_vector(3 downto 0);
  type length_type is array (0 to 15) of natural;
  constant dlc_codes:dlc_type:=("0000","0001","0010","0011","0100","0101","0110","0111",
                                  "1000","1001","1010","1011","1100","1101","1110","1111"); 
  constant dlc_length:length_type:=(0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64);

  constant ZERO: std_logic := '0';
  constant NO_ACTION: std_logic := '0';
  
  constant ACK_ALLOWED : std_logic := '0';
  constant ACK_FORBIDEN : std_logic := '1';
  
  constant LOOPBACK_ENA : std_logic := '1';
  constant LOOPBACK_DIS : std_logic := '0';
  
  constant RETR_LIM_DIS : std_logic := '0'; 
  constant RETR_LIM_ENA : std_logic := '1'; 
       
  constant SINGLE_SAMPLING : std_logic := '0'; 
  constant TRIPPLE_SAMPLING : std_logic := '1'; 
  
  constant ALLOW_BUFFER : std_logic := '1'; 
  constant FORBID_BUFFER : std_logic := '0'; 
  
  --CRC polynomials
  constant CRC15_POL :     std_logic_vector(15 downto 0):=std_logic_vector'(X"C599");
  constant CRC17_POL :     std_logic_vector(19 downto 0):=std_logic_vector'(X"3685B");
  constant CRC21_POL :     std_logic_vector(23 downto 0):=std_logic_vector'(X"302899");
  
  -----------------------
  --State Machine types--
  -----------------------
  
  --Error state of node
  type error_state_type is (
    error_active,
    error_passive,
    bus_off
  ); 
    
  --Operation mode of the Node
  type oper_mode_type is (
    integrating,
    idle,
    transciever,
    reciever
  );
  
  --Protocol control
  type protocol_type is (
    sof,
    arbitration,
    control,
    data,
    crc,
    delim_ack,
    eof,
    interframe,
    overload,
    error,
    off
  );
  
  --Note: two bits are two bits between Base and Extended identifier
  --Note: one bit is the last remaining bit after the identifier extension
  type arb_type is (
    base_id,
    two_bits,
    ext_id,
    one_bit
  );
  
  --State machine type for sending control field bits
  --Note: control_type is only for sending. For recieving frame type is unknown until EDL bit
  type control_type is (
    r0,
    r1,
    edl,
    brs,
    esi,
    dlc
  );
  
  --Within ISO CAN FD new field stuff count is needed!
  type crc_type is(
    stuff_count,
    real_crc
  );
  
  --Intermission field sub-State
  type interm_spc_type is (
    intermission,
    suspend,
    interm_idle
  ); 
  
  --Error frame subtype
  type err_frame_type is (
    err_flg_sup,
    err_delim
  );
  
   --Overload frame subtype
  type ovr_frame_type is (
    ovr_flg_sup,
    ovr_delim
  ); 
  
  type bit_time_type is (
    sync,
    prop,
    ph1,
    ph2,
    h_sync,
    reset
  );
  
  --Logger state machine type 
  type logger_state_type is (
    config,
    ready,
    running
  ); 
    
  
  ----------------------------
  --Driving bus signal ranges-
  ----------------------------
  --Prescaler
  constant DRV_TQ_NBT_LOW:natural:=0;
  constant DRV_TQ_NBT_HIGH:natural:=5;
  
  constant DRV_TQ_DBT_LOW:natural:=6;
  constant DRV_TQ_DBT_HIGH:natural:=11;
  
  constant DRV_PRS_NBT_LOW:natural:=12;
  constant DRV_PRS_NBT_HIGH:natural:=17; 
  
  constant DRV_PH1_NBT_LOW:natural:=18; 
  constant DRV_PH1_NBT_HIGH:natural:=23; 
  
  constant DRV_PH2_NBT_LOW:natural:=24;
  constant DRV_PH2_NBT_HIGH:natural:=29;

  constant DRV_PRS_DBT_LOW:natural:=30;
  constant DRV_PRS_DBT_HIGH:natural:=33;
  
  constant DRV_PH1_DBT_LOW:natural:=34;
  constant DRV_PH1_DBT_HIGH:natural:=37;
  
  constant DRV_PH2_DBT_LOW:natural:=38;
  constant DRV_PH2_DBT_HIGH:natural:=41;
  
  constant DRV_SJW_LOW:natural:=42;
  constant DRV_SJW_HIGH:natural:=45;
  
  constant DRV_SJW_DBT_LOW:natural:=46;
  constant DRV_SJW_DBT_HIGH:natural:=49;
  
  --TimeStampGen
  constant DRV_TS_1_SRC_LOW:natural:=61;
  constant DRV_TS_1_SRC_HIGH:natural:=63;
  
  constant DRV_TS_1_RST_INDEX:natural:=64;
  
  constant DRV_TS_1_MAKE_LOW:natural:=65;
  constant DRV_TS_1_MAKE_HIGH:natural:=68;
  
  constant DRV_TS_2_SRC_LOW:natural:=69;
  constant DRV_TS_2_SRC_HIGH:natural:=71;
  
  constant DRV_TS_2_RST_INDEX:natural:=72;
  
  constant DRV_TS_2_MAKE_LOW:natural:=73;
  constant DRV_TS_2_MAKE_HIGH:natural:=76;
  
  --Message Filter
  constant DRV_FILTER_A_MASK_LOW:natural:=81;
  constant DRV_FILTER_A_MASK_HIGH:natural:=109;
  
  constant DRV_FILTER_A_CTRL_LOW:natural:=110;
  constant DRV_FILTER_A_CTRL_HIGH:natural:=113;
  
  constant DRV_FILTER_A_BITS_LOW:natural:=114;
  constant DRV_FILTER_A_BITS_HIGH:natural:=142;
  
  constant DRV_FILTER_B_MASK_LOW:natural:=143;
  constant DRV_FILTER_B_MASK_HIGH:natural:=171;
  
  constant DRV_FILTER_B_CTRL_LOW:natural:=172;
  constant DRV_FILTER_B_CTRL_HIGH:natural:=175;
  
  constant DRV_FILTER_B_BITS_LOW:natural:=176;
  constant DRV_FILTER_B_BITS_HIGH:natural:=204;
  
  constant DRV_FILTER_C_MASK_LOW:natural:=205;
  constant DRV_FILTER_C_MASK_HIGH:natural:=233;
  
  constant DRV_FILTER_C_CTRL_LOW:natural:=234;
  constant DRV_FILTER_C_CTRL_HIGH:natural:=237;
  
  constant DRV_FILTER_C_BITS_LOW:natural:=238;
  constant DRV_FILTER_C_BITS_HIGH:natural:=266;
  
  constant DRV_FILTER_RAN_CTRL_LOW:natural:=267;
  constant DRV_FILTER_RAN_CTRL_HIGH:natural:=270;
  
  constant DRV_FILTER_RAN_LO_TH_LOW:natural:=271;
  constant DRV_FILTER_RAN_LO_TH_HIGH:natural:=299;
  
  constant DRV_FILTER_RAN_HI_TH_LOW:natural:=300;
  constant DRV_FILTER_RAN_HI_TH_HIGH:natural:=328;
  
  constant DRV_FILTERS_ENA_INDEX:natural:=329;
  
  --RX Buffer
  constant DRV_ERASE_RX_INDEX:natural:=350;
  constant DRV_OVR_RX_INDEX:natural:=351; 
  constant DRV_READ_START_INDEX:natural:=352;
  constant DRV_CLR_OVR_INDEX:natural:=353;
  
  --TXT Buffer
  constant DRV_ERASE_TXT2_INDEX:natural:=356;
  constant DRV_STORE_TXT2_INDEX:natural:=357;
  
  --TX Buffer
  constant DRV_ERASE_TXT1_INDEX:natural:=366;
  constant DRV_STORE_TXT1_INDEX:natural:=367;
  
  --TX Arbitrator
  constant DRV_ALLOW_TXT1_INDEX:natural:=361;
  constant DRV_ALLOW_TXT2_INDEX:natural:=362;

  --Interrupt manager indices 
  constant DRV_BUS_ERR_INT_ENA_INDEX:natural:=376;
  constant DRV_ARB_LST_INT_ENA_INDEX:natural:=377;
  constant DRV_ERR_PAS_INT_ENA_INDEX:natural:=378;
  constant DRV_WAKE_INT_ENA_INDEX:natural:=379;
  constant DRV_DOV_INT_ENA_INDEX:natural:=380;
  constant DRV_ERR_WAR_INT_ENA_INDEX:natural:=381;
  constant DRV_TX_INT_ENA_INDEX:natural:=382;
  constant DRV_RX_INT_ENA_INDEX:natural:=383;
  constant DRV_LOG_FIN_INT_ENA_INDEX:natural:=384;
  constant DRV_RX_FULL_INT_ENA_INDEX:natural:=385;
  constant DRV_BRS_INT_ENA_INDEX:natural:=386;
  
  constant DRV_INT_VECT_ERASE_INDEX:natural:=387;
  
  constant DRV_SAM_INDEX:natural:=372;
  
  -------------
  --CAN Core --
  -------------
  
  --Fault Confinement
  constant DRV_EWL_LOW:natural:=400;
  constant DRV_EWL_HIGH:natural:=407;
    
  constant DRV_ERP_LOW:natural:=408;
  constant DRV_ERP_HIGH:natural:=415;
    
  constant DRV_CTR_VAL_LOW:natural:=416;
  constant DRV_CTR_VAL_HIGH:natural:=424;
  
  constant DRV_CTR_SEL_LOW:natural:=425;
  constant DRV_CTR_SEL_HIGH:natural:=428;
  
  --Operation control FSM
  constant DRV_CAN_FD_ENA_INDEX:natural:=460;
  constant DRV_RTR_PREF_INDEX:natural:=461;
  constant DRV_BUS_MON_ENA_INDEX:natural:=470;
  constant DRV_SELF_TEST_ENA_INDEX:natural:=471;
  
  constant DRV_RETR_LIM_ENA_INDEX:natural:=465;
  
  constant DRV_RETR_TH_LOW:natural:=466;
  constant DRV_RETR_TH_HIGH:natural:=469;
  
  constant DRV_ABORT_TRAN_INDEX:natural:=472;
  
  constant DRV_SET_RX_CTR_INDEX:natural:=473;
  constant DRV_SET_TX_CTR_INDEX:natural:=474;
  
  constant DRV_SET_CTR_VAL_HIGH:natural:=506;
  constant DRV_SET_CTR_VAL_LOW:natural:=475;
  
  constant DRV_ACK_FORB_INDEX:natural:=507; 
  constant DRV_INT_LOOBACK_ENA_INDEX:natural:=508; 
  
  constant DRV_ENA_INDEX:natural:=509;
  constant DRV_FD_TYPE_INDEX:natural:=510;
  
  ----------------
  --Event logger--
  ----------------
  constant   DRV_TRIG_CONFIG_DATA_HIGH:natural:=551;
  constant   DRV_TRIG_CONFIG_DATA_LOW:natural:=520;
  
  constant   DRV_TRIG_SOF_INDEX:natural:=552;
  constant   DRV_TRIG_ARB_LOST_INDEX:natural:=553;
  constant   DRV_TRIG_REC_VALID_INDEX:natural:=554;
  constant   DRV_TRIG_TRAN_VALID_INDEX:natural:=555;
  constant   DRV_TRIG_OVL_INDEX:natural:=556;
  constant   DRV_TRIG_ERROR_INDEX:natural:=557;
  constant   DRV_TRIG_BRS_INDEX:natural:=558;
  constant   DRV_TRIG_USER_WRITE_INDEX:natural:=559;
  constant   DRV_TRIG_ARB_START_INDEX:natural:=560;
  constant   DRV_TRIG_CONTR_START_INDEX:natural:=561;
  constant   DRV_TRIG_DATA_START_INDEX:natural:=562;
  constant   DRV_TRIG_CRC_START_INDEX:natural:=563;
  constant   DRV_TRIG_ACK_REC_INDEX:natural:=564;
  constant   DRV_TRIG_ACK_N_REC_INDEX:natural:=565;
  constant   DRV_TRIG_EWL_REACHED_INDEX:natural:=566;
  constant   DRV_TRIG_ERP_CHANGED_INDEX:natural:=567;
  constant   DRV_TRIG_TRAN_START_INDEX:natural:=568;
  constant   DRV_TRIG_REC_START_INDEX:natural:=569;

  constant   DRV_CAP_SOF_INDEX:natural:=580;
  constant   DRV_CAP_ARB_LOST_INDEX:natural:=581;
  constant   DRV_CAP_REC_VALID_INDEX:natural:=582;
  constant   DRV_CAP_TRAN_VALID_INDEX:natural:=583;
  constant   DRV_CAP_OVL_INDEX:natural:=584;
  constant   DRV_CAP_ERROR_INDEX:natural:=585;
  constant   DRV_CAP_BRS_INDEX:natural:=586;
  constant   DRV_CAP_ARB_START_INDEX:natural:=587;
  constant   DRV_CAP_CONTR_START_INDEX:natural:=588;
  constant   DRV_CAP_DATA_START_INDEX:natural:=589;
  constant   DRV_CAP_CRC_START_INDEX:natural:=590;
  constant   DRV_CAP_ACK_REC_INDEX:natural:=591;
  constant   DRV_CAP_ACK_N_REC_INDEX:natural:=592;
  constant   DRC_CAP_EWL_REACHED_INDEX:natural:=593;
  constant   DRV_CAP_ERP_CHANGED_INDEX:natural:=594;
  constant   DRV_CAP_TRAN_START_INDEX:natural:=595;
  constant   DRV_CAP_REC_START_INDEX:natural:=596;
  constant   DRV_CAP_SYNC_EDGE_INDEX:natural:=597;
  constant   DRV_CAP_STUFFED_INDEX:natural:=598;
  constant   DRV_CAP_DESTUFFED_INDEX:natural:=599;
  constant   DRV_CAP_OVR_INDEX:natural:=600;

  constant DRV_LOG_CMD_STR_INDEX:natural:=610;
  constant DRV_LOG_CMD_ABT_INDEX:natural:=611;
  constant DRV_LOG_CMD_UP_INDEX:natural:=612;
  constant DRV_LOG_CMD_DOWN_INDEX:natural:=613;
  
  ----------------------------------------------------
  --RX, TX and TXT Buffer frame format signal indexes-  
  ----------------------------------------------------
  --Tx Message format (Format A)
  constant TX_FFW_HIGH:natural:=639;
  constant TX_FFW_LOW:natural:=608;
  
  constant TX_IDW_HIGH:natural:=607;
  constant TX_IDW_LOW:natural:=576;
  
  constant TX_DATAW_HIGH:natural:=575;  --16 Data words for up to 64 bytes of data
  constant TX_DATA1W_LOW:natural:=544; 
  constant TX_DATAW_LOW:natural:=64;
  
  --Txt message format (Format B)
  constant TXT_FFW_HIGH:natural:=639;
  constant TXT_FFW_LOW:natural:=608;
  
  constant TXT_TSUPP_HIGH:natural:=607;
  constant TXT_TSUPP_LOW:natural:=576;
  
  constant TXT_TSLOW_HIGH:natural:=575;
  constant TXT_TSLOW_LOW:natural:=544;
  
  constant TXT_IDW_HIGH:natural:=543;
  constant TXT_IDW_LOW:natural:=512;
  
  constant TXT_DATAW_HIGH:natural:=511;
  constant TXT_DATAW_LOW:natural:=0;
  
    
  ----------------------------
  --Interrupt vector indices--
  ----------------------------
  constant BUS_ERR_INT:natural:=7;
  constant ARB_LST_INT:natural:=6;
  constant ERR_PAS_INT:natural:=5;
  constant WAKE_INT:natural:=4;
  constant DOV_INT:natural:=3;
  constant ERR_WAR_INT:natural:=2;
  constant TX_INT:natural:=1;
  constant RX_INT:natural:=0;
  constant LOG_FIN_INT:natural:=8;
  constant RX_FULL_INT:natural:=9;
  constant BRS_INT:natural:=10;

  
  -----------------------
  --STATUS BUS INDICES --
  -----------------------
  constant STAT_OP_STATE_LOW:natural:=0;
  constant STAT_OP_STATE_HIGH:natural:=1;
  
  constant STAT_PC_STATE_LOW:natural:=2;
  constant STAT_PC_STATE_HIGH:natural:=5;
  
  constant STAT_ARB_LOST_INDEX:natural:=6;
  
  constant STAT_SET_TRANSC_INDEX:natural:=7;
  constant STAT_SET_REC_INDEX:natural:=8;
  constant STAT_IS_IDLE_INDEX:natural:=9;
  
  constant STAT_SP_CONTROL_HIGH:natural:=11;
  constant STAT_SP_CONTROL_LOW:natural:=10;
  
  constant STAT_SSP_RESET_INDEX:natural:=12;
  constant STAT_TRV_DELAY_CALIB_INDEX:natural:=13;
  constant STAT_SYNC_CONTROL_HIGH:natural:=15;
  constant STAT_SYNC_CONTROL_LOW:natural:=14;
  
  constant STAT_DATA_TX_INDEX:natural:=16;
  constant STAT_DATA_RX_INDEX:natural:=17;
  constant STAT_BS_ENABLE_INDEX:natural:=18;
  constant STAT_FIXED_STUFF_INDEX:natural:=19;
  constant STAT_DATA_HALT_INDEX:natural:=20;
  constant STAT_BS_LENGTH_HIGH:natural:=23;
  constant STAT_BS_LENGTH_LOW:natural:=21;
  
  --Error indices
  constant STAT_STUFF_ERROR_INDEX:natural:=24;
  constant STAT_DESTUFFED_INDEX:natural:=25;
  constant STAT_BDS_ENA_INDEX:natural:=26;
  constant STAT_STUFF_ERRROR_ENA_INDEX:natural:=27;
  constant STAT_FIXED_DESTUFF_INDEX:natural:=28;
  constant STAT_BDS_LENGTH_HIGH:natural:=31;
  constant STAT_BDS_LENGTH_LOW:natural:=29;
  
  --Transcieve data
  constant STAT_TRAN_IDENT_HIGH:natural:=60;
  constant STAT_TRAN_IDENT_LOW:natural:=32;
  
  constant STAT_TRAN_DLC_HIGH:natural:=64;
  constant STAT_TRAN_DLC_LOW:natural:=61;
  
  constant STAT_TRAN_IS_RTR_INDEX:natural:=65;
  constant STAT_TRAN_IDENT_TYPE_INDEX:natural:=66;
  constant STAT_TRAN_FRAME_TYPE_INDEX:natural:=67;
  constant STAT_TRAN_DATA_ACK_INDEX:natural:=68;
  constant STAT_TRAN_BRS_INDEX:natural:=69;
  constant STAT_FRAME_STORE_INDEX:natural:=70;
  
  --Error counters and error state
  constant STAT_TX_COUNTER_HIGH:natural:=79;
  constant STAT_TX_COUNTER_LOW:natural:=71;
  constant STAT_RX_COUNTER_HIGH:natural:=89;
  constant STAT_RX_COUNTER_LOW:natural:=81;
  constant STAT_ERROR_COUNTER_NORM_HIGH:natural:=272;
  constant STAT_ERROR_COUNTER_NORM_LOW:natural:=257;
  constant STAT_ERROR_COUNTER_FD_HIGH:natural:=288;
  constant STAT_ERROR_COUNTER_FD_LOW:natural:=273;
  
  constant STAT_ERROR_STATE_HIGH:natural:=109;
  constant STAT_ERROR_STATE_LOW:natural:=108;
  
  --Error signals
  constant STAT_FORM_ERROR_INDEX:natural:=110;
  constant STAT_CRC_ERROR_INDEX:natural:=111;
  constant STAT_ACK_ERROR_INDEX:natural:=112;
  constant STAT_UNKNOWN_STATE_ERROR_INDEX:natural:=113;
  constant STAT_BIT_STUFF_ERROR_INDEX:natural:=114;
  constant STAT_FIRST_BIT_AFTER_INDEX:natural:=115;
  constant STAT_REC_VALID_INDEX:natural:=116;
  constant STAT_TRAN_VALID_INDEX:natural:=117;
  constant STAT_CONST7_INDEX:natural:=118;
  constant STAT_CONST14_INDEX:natural:=119;
  constant STAT_TRANSM_ERROR_INDEX:natural:=120;
  
  --Recieved data interface
  constant STAT_REC_IDENT_HIGH:natural:=149;
  constant STAT_REC_IDENT_LOW:natural:=121;
  constant STAT_REC_DLC_HIGH:natural:=153;
  constant STAT_REC_DLC_LOW:natural:=150;
  constant STAT_REC_IS_RTR_INDEX:natural:=154;
  constant STAT_REC_IDENT_TYPE_INDEX:natural:=155;
  constant STAT_REC_FRAME_TYPE_INDEX:natural:=156;
  constant STAT_REC_BRS_INDEX:natural:=157;
  constant STAT_REC_CRC_HIGH:natural:=178;
  constant STAT_REC_CRC_LOW:natural:=158;
  constant STAT_REC_ESI_INDEX:natural:=179;
  constant STAT_CRC_ENA_INDEX:natural:=180;
  constant STAT_TRAN_TRIG:natural:=181;
  constant STAT_REC_TRIG:natural:=182;
  
  --Arbitration lost capture
  constant STAT_ALC_HIGH:natural:=187;
  constant STAT_ALC_LOW:natural:=183;
  
  --Bus traffic registers
  constant STAT_RX_CTR_HIGH:natural:=219;
  constant STAT_RX_CTR_LOW:natural:=188;
  
  constant STAT_TX_CTR_HIGH:natural:=251;
  constant STAT_TX_CTR_LOW:natural:=220;
  
  constant STAT_ERP_CHANGED_INDEX:natural:=252;
  constant STAT_EWL_REACHED_INDEX:natural:=253;
  constant STAT_ERROR_VALID_INDEX:natural:=254;
  
  constant STAT_ACK_RECIEVED_OUT_INDEX:natural:=255;
  
  constant STAT_BIT_ERROR_VALID_INDEX:natural:=256;
  
  constant STAT_BS_CTR_HIGH:natural:=302;
  constant STAT_BS_CTR_LOW:natural:=300;
  
  constant STAT_BD_CTR_HIGH:natural:=305;
  constant STAT_BD_CTR_LOW:natural:=303;
  
  constant STAT_TS_HIGH:natural:=369;
  constant STAT_TS_LOW:natural:=306;
  
  ------------------
  --MEMORY Acesss --
  ------------------
  constant GPR_COMPONENT_TYPE:std_logic_vector(3 downto 0):=std_logic_vector'(X"1"); --General Purpose register
  constant OUTMUX_COMPONENT_TYPE:std_logic_vector(3 downto 0):=std_logic_vector'(X"2"); --OutPut Multiplexor component type
  constant FLEXRAY_COMPONENT_TYPE:std_logic_vector(3 downto 0):=std_logic_vector'(X"3"); --FlexRay Node
  constant CAN_COMPONENT_TYPE:std_logic_vector(3 downto 0):=std_logic_vector'(X"4"); --CAN Node has 4.
  constant LIN_COMPONENT_TYPE:std_logic_vector(3 downto 0):=std_logic_vector'(X"5"); --LIN Node
 
  constant ACT_CSC:std_logic:='1';  
  constant ACT_SRD:std_logic:='1';  
  constant ACT_SWR:std_logic:='1';  
  
  --Address ranges for component type and identifier
  constant COMP_TYPE_ADRESS_HIGHER:natural:=23;
  constant COMP_TYPE_ADRESS_LOWER:natural:=20;
  constant ID_ADRESS_HIGHER:natural:=19;
  constant ID_ADRESS_LOWER:natural:=16;
  
  constant CAN_DEVICE_ID:std_logic_vector(31 downto 0):=std_logic_vector'(X"0000CAFD");
  
  --Adress Offsets
  --Note: It is considered that lowest two bits are cut!! Therefore every register is in processor mapped to 0x4 higher adress!!
  --This is done to achieve the uniform acess, when someone acesses 0x0000,0x0001,0x0002,0x0003 adresses. This way lower bits will
  --be cut and no adress alignment problems will appear.
  --The real adress(in processor) offset is 4 times value of adress constant!
  constant DEVICE_ID_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"000"); 
  constant MODE_REG_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"001"); 
  constant INTERRUPT_REG_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"002"); 
  constant TIMING_REG_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"003"); 
  constant ARB_ERROR_PRESC_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"004"); 
  constant ERROR_TH_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"005"); 
  constant ERROR_COUNTERS_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"006"); 
  constant ERROR_COUNTERS_SPEC_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"007"); 
  constant FILTER_A_VAL_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"008"); 
  constant FILTER_A_MASK_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"009");
  constant FILTER_B_VAL_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"00A"); 
  constant FILTER_B_MASK_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"00B"); 
  constant FILTER_C_VAL_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"00C"); 
  constant FILTER_C_MASK_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"00D"); 
  constant FILTER_RAN_LOW_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"00E"); --Mode, Status, Control Register
  constant FILTER_RAN_HIGH_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"00F"); --Mode, Status, Control Register
  constant FILTER_CONTROL_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"010"); --Mode, Status, Control Register
  constant RX_INFO_1_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"011"); 
  constant RX_INFO_2_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"012"); 
  constant RX_DATA_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"013"); 
  constant TRV_DELAY_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"014"); 
  
  constant TX_STATUS_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"015"); 
  constant TX_SETTINGS_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"016"); 
  
  constant TX_DATA_1_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"017"); 
  constant TX_DATA_2_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"018"); 
  constant TX_DATA_3_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"019"); 
  constant TX_DATA_4_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"01A"); 
  constant TX_DATA_5_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"01B"); 
  constant TX_DATA_6_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"01C"); 
  constant TX_DATA_7_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"01D"); 
  constant TX_DATA_8_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"01E"); 
  constant TX_DATA_9_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"01F"); 
  constant TX_DATA_10_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"020"); 
  constant TX_DATA_11_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"021"); 
  constant TX_DATA_12_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"022"); 
  constant TX_DATA_13_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"023"); 
  constant TX_DATA_14_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"024"); 
  constant TX_DATA_15_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"025"); 
  constant TX_DATA_16_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"026"); 
  constant TX_DATA_17_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"027"); 
  constant TX_DATA_18_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"028"); 
  constant TX_DATA_19_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"029"); 
  constant TX_DATA_20_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"02A"); 
  
  constant RX_COUNTER_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"02B"); 
  constant TX_COUNTER_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"02C"); 
  
  constant LOG_TRIG_CONFIG_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"02E"); 
  constant LOG_TRIG_CONFIG_DATA_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"02F"); 
  constant LOG_CAPT_CONFIG_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"030"); 
  constant LOG_STATUS_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"031"); 
  constant LOG_CMD_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"032"); 
  constant LOG_CAPT_EVENT1_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"033"); 
  constant LOG_CAPT_EVENT2_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"034"); 
  
  constant DEBUG_REG_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"035");
  constant YOLO_REG_ADR:std_logic_vector(11 downto 0):=std_logic_vector'(X"036"); 
  
  ----------------
  --Register Bits-
  ----------------
  --Status register
  constant BS_IND:natural:=7; --Bus status
  constant ES_IND:natural:=6; --Error Status
  constant TS_IND:natural:=5; --Transmit Status
  constant RS_IND:natural:=4; --Recieve status
  constant ET_IND:natural:=3; --Transmit Buffer Status
  constant TBS_IND:natural:=2; --Transmit Buffer Status
  constant DOS_IND:natural:=1; --Data OverRun Status
  constant RBS_IND:natural:=0; -- Recieve Buffer Status
  
  --Mode register
  constant FDE_IND:natural:=4; --Flexible datarate enable
  constant RTR_PREF_IND:natural:=5; --RTR Preffered behaviour
  constant AFM_IND:natural:=3; --Acceptance filters mode (acceptance filters enabled)
  constant LOM_IND:natural:=1;
  constant STM_IND:natural:=2;
  constant TSM_IND:natural:=6;
  constant RST_IND:natural:=0;
  
  --Interrupt enable indices
  constant RI_IND:natural:=0;
  constant TI_IND:natural:=1;
  constant EI_IND:natural:=2;
  constant DOI_IND:natural:=3;
  constant EPI_IND:natural:=5;
  constant ALI_IND:natural:=6;
  constant BEI_IND:natural:=7;
  constant LFI_IND:natural:=8;
  constant RFI_IND:natural:=9;
  constant BSI_IND:natural:=10;
    
    
  ----------------------------------------------  
  --Logger event types 
  ----------------------------------------------
  constant SOF_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"01");
  constant ALO_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"02");    
  constant REC_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"03");    
  constant TRAN_EVNT:std_logic_vector(7 downto 0)   := std_logic_vector'(X"04");  
  constant OVLD_EVNT:std_logic_vector(7 downto 0)   := std_logic_vector'(X"05");  
  constant ERR_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"06");  
  constant BRS_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"07");   
  constant ARB_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"08");   
  constant CTRL_EVNT:std_logic_vector(7 downto 0)   := std_logic_vector'(X"09");   
  constant DATA_EVNT:std_logic_vector(7 downto 0)   := std_logic_vector'(X"0A");   
  constant CRC_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"0B");   
  constant ACK_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"0C");
  constant NACK_EVNT:std_logic_vector(7 downto 0)   := std_logic_vector'(X"0D");   
  constant EWL_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"0E");   
  constant ERP_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"0F"); 
  constant TXS_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"10"); 
  constant RXS_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"11"); 
  constant SYNC_EVNT:std_logic_vector(7 downto 0)   := std_logic_vector'(X"12");  
  constant STUF_EVNT:std_logic_vector(7 downto 0)   := std_logic_vector'(X"13"); 
  constant DSTF_EVNT:std_logic_vector(7 downto 0)   := std_logic_vector'(X"14"); 
  constant OVR_EVNT:std_logic_vector(7 downto 0)    := std_logic_vector'(X"15"); 
  -----------------------------------------------------
  --constant ERR_EVNT:std_logic_vector(7 downto 0)  := std_logic_vector'(X"01"); SO far pseudo random numbers for testing the data--
  -----------------------------------------------------
  
  type rand_length is array (0 to 3809) of real;
  constant randData:rand_length:= 
  (0.82197,0.63702,0.95391,0.94693,0.96661,
0.067344,0.43753,0.32084,0.13405,0.13459,
0.80595,0.52476,0.94426,0.98834,0.40989,
0.37119,0.22685,0.44603,0.26622,0.4591,
0.43291,0.25962,0.13371,0.41923,0.50686,
0.32432,0.68469,0.44309,0.43566,0.79302,
0.81556,0.75211,0.78926,0.50127,0.55518,
0.63075,0.097989,0.24568,0.61573,0.30496,
0.76697,0.26723,0.039506,0.29656,0.55636,
0.96907,0.68912,0.71788,0.55903,0.53335,
0.87572,0.3931,0.45807,0.20825,0.75727,
0.54669,0.35739,0.701,0.10922,0.0066108,
0.5973,0.65918,0.58001,0.90995,0.63601,
0.52556,0.25962,0.051171,0.73195,0.16429,
0.2804,0.25943,0.5471,0.54127,0.78811,
0.86961,0.78754,0.96943,0.18047,0.9306,
0.045152,0.24065,0.008855,0.67159,0.90481,
0.57242,0.15546,0.50237,0.56773,0.18827,
0.3242,0.71604,0.55293,0.14228,0.38036,
0.39657,0.57674,0.019402,0.57758,0.9322,
0.10687,0.73215,0.97052,0.60889,0.71967,
0.30275,0.45902,0.048029,0.38535,0.36172,
0.28758,0.81671,0.45053,0.80663,0.79017,
0.28296,0.068309,0.054931,0.63752,0.42429,
0.90553,0.41732,0.15406,0.54,0.93709,
0.66096,0.39466,0.25899,0.84791,0.94506,
0.377,0.06728,0.18158,0.57575,0.18589,
0.29145,0.46166,0.34698,0.31817,0.45991,
0.23589,0.027752,0.65845,0.1588,0.80266,
0.40858,0.32739,0.74601,0.74635,0.17395,
0.11754,0.17404,0.62741,0.84189,0.51008,
0.16576,0.71431,0.90704,0.21854,0.87096,
0.21184,0.83666,0.8593,0.52338,0.47736,
0.88988,0.065076,0.50946,0.6208,0.73357,
0.23,0.021872,0.13899,0.76951,0.96977,
0.38679,0.99343,0.32639,0.13716,0.38475,
0.56264,0.63384,0.54161,0.31499,0.15933,
0.15262,0.13695,0.7098,0.46486,0.11327,
0.70088,0.17999,0.80367,0.51396,0.54844,
0.20784,0.78459,0.52649,0.57102,0.42204,
0.72116,0.073138,0.59486,0.86198,0.4488,
0.65258,0.30347,0.60744,0.2789,0.79956,
0.79617,0.95409,0.44434,0.45688,0.59982,
0.84262,0.0312,0.18727,0.94359,0.94792,
0.45298,0.81083,0.92888,0.67272,0.37233,
0.4057,0.43882,0.67865,0.46507,0.95326,
0.3547,0.33902,0.89586,0.54543,0.74928,
0.12488,0.45323,0.074749,0.66334,0.70365,
0.91895,0.66007,0.6901,0.85372,0.4679,
0.45848,0.8061,0.82477,0.19044,0.025673,
0.056815,0.14294,0.17142,0.62584,0.029515,
0.47233,0.67844,0.11479,0.23606,0.28908,
0.17276,0.32371,0.80111,0.29963,0.77563,
0.55277,0.55469,0.73065,0.77362,0.90085,
0.13817,0.79408,0.18941,0.028974,0.12739,
0.13374,0.12831,0.93526,0.27323,0.94268,
0.63815,0.87253,0.36707,0.2362,0.18732,
0.54565,0.25512,0.30582,0.015549,0.58749,
0.96256,0.84985,0.0079403,0.63403,0.35929,
0.11406,0.54082,0.41642,0.5171,0.88612,
0.1494,0.43468,0.059039,0.38103,0.72237,
0.095119,0.66719,0.2964,0.59856,0.15185,
0.43638,0.012682,0.22902,0.26368,0.51139,
0.21508,0.34611,0.74782,0.41362,0.055753,
0.39003,0.47448,0.82532,0.30364,0.82179,
0.56568,0.054382,0.26001,0.5891,0.47973,
0.19865,0.23901,0.78019,0.61731,0.14413,
0.71613,0.40151,0.46239,0.70728,0.40121,
0.014385,0.07464,0.59107,0.446,0.92662,
0.094893,0.37542,0.546,0.11168,0.90446,
0.63328,0.90541,0.63055,0.014228,0.31648,
0.11187,0.62946,0.060709,0.67399,0.47744,
0.30555,0.51633,0.70703,0.81361,0.31581,
0.3113,0.34498,0.6663,0.86107,0.76178,
0.87584,0.87124,0.17279,0.85022,0.95959,
0.77021,0.87502,0.067413,0.64679,0.3241,
0.64035,0.87975,0.37364,0.76674,0.16809,
0.51972,0.62746,0.71391,0.3064,0.26369,
0.916,0.61503,0.093175,0.6277,0.19203,
0.77696,0.8645,0.33358,0.13541,0.76547,
0.31861,0.25238,0.20007,0.06901,0.55191,
0.40381,0.75012,0.48718,0.38479,0.061409,
0.21369,0.54387,0.41064,0.90096,0.05629,
0.44351,0.53781,0.13406,0.54095,0.85737,
0.19802,0.15561,0.061378,0.66107,0.018603,
0.2911,0.97382,0.76464,0.24368,0.68212,
0.13785,0.62981,0.85701,0.8998,0.34837,
0.48631,0.67952,0.70412,0.46088,0.36427,
0.28027,0.076204,0.44462,0.16571,0.39875,
0.92058,0.51133,0.91414,0.091934,0.99304,
0.096434,0.31315,0.78538,0.6024,0.46591,
0.29813,0.13317,0.29501,0.16663,0.3171,
0.10984,0.83209,0.97159,0.21827,0.70608,
0.039014,0.6163,0.66936,0.037202,0.0033346,
0.14246,0.86241,0.27604,0.53169,0.52222,
0.56762,0.33303,0.41342,0.41435,0.98392,
0.057739,0.39654,0.79132,0.59419,0.30956,
0.9018,0.093127,0.31903,0.88696,0.65743,
0.68452,0.47395,0.14124,0.95092,0.88263,
0.43744,0.83496,0.32514,0.36764,0.79484,
0.099309,0.95178,0.0014739,0.2954,0.048455,
0.44274,0.78985,0.91352,0.53325,0.80408,
0.56266,0.75088,0.0092304,0.47678,0.25033,
0.30792,0.96695,0.2088,0.52048,0.22555,
0.5672,0.99816,0.13187,0.95468,0.12388,
0.18624,0.6465,0.12817,0.081321,0.65923,
0.027399,0.98518,0.53933,0.37383,0.70673,
0.94741,0.38228,0.69291,0.60207,0.77526,
0.59183,0.37618,0.85065,0.22574,0.79696,
0.99688,0.28131,0.71038,0.66464,0.41484,
0.49827,0.94912,0.95318,0.73292,0.38468,
0.040083,0.58293,0.56471,0.35518,0.8802,
0.62453,0.62402,0.29574,0.07468,0.29371,
0.23474,0.3459,0.84849,0.16036,0.15786,
0.50866,0.6033,0.1614,0.63545,0.84394,
0.78227,0.26457,0.3147,0.1832,0.44747,
0.32668,0.27982,0.93176,0.39969,0.37942,
0.59285,0.068507,0.20524,0.72362,0.57515,
0.20023,0.84348,0.42373,0.54475,0.52797,
0.18511,0.08169,0.46409,0.030555,0.43498,
0.55786,0.63878,0.034217,0.7099,0.16932,
0.59338,0.60806,0.77236,0.056274,0.85473,
0.38428,0.39962,0.32542,0.55539,0.29542,
0.36612,0.34905,0.63022,0.66444,0.9921,
0.94436,0.35035,0.193,0.91958,0.28869,
0.55086,0.91932,0.090049,0.2577,0.42705,
0.57772,0.89952,0.21823,0.96703,0.43398,
0.78482,0.52523,0.33128,0.43159,0.7179,
0.91621,0.89003,0.13471,0.11991,0.89345,
0.65309,0.040278,0.50472,0.89445,0.38573,
0.29205,0.23404,0.20095,0.38031,0.59479,
0.26839,0.62244,0.80457,0.10397,0.72924,
0.64859,0.47465,0.93291,0.096444,0.59911,
0.23356,0.032328,0.57986,0.8422,0.5569,
0.83988,0.20495,0.62125,0.17402,0.28954,
0.018503,0.7015,0.95209,0.74901,0.75673,
0.54209,0.28206,0.24487,0.28632,0.96313,
0.2307,0.53732,0.20501,0.43404,0.14221,
0.37558,0.79356,0.81282,0.90379,0.54038,
0.81786,0.70842,0.043209,0.14595,0.23334,
0.2467,0.17028,0.23506,0.27546,0.95161,
0.34667,0.29732,0.40443,0.30224,0.75731,
0.35974,0.1249,0.61718,0.35552,0.36294,
0.068488,0.86716,0.4579,0.077615,0.90492,
0.2817,0.61388,0.66186,0.20002,0.95999,
0.66511,0.5413,0.86897,0.55705,0.021398,
0.48268,0.80799,0.73601,0.5723,0.0089849,
0.71828,0.44943,0.65962,0.75321,0.80474,
0.029156,0.77982,0.56735,0.076117,0.25162,
0.13345,0.56447,0.54098,0.068921,0.98843,
0.2511,0.31547,0.30066,0.042039,0.5279,
0.25602,0.40871,0.94751,0.91928,0.12124,
0.59194,0.35965,0.71931,0.52355,0.26084,
0.49308,0.85584,0.72441,0.19911,0.15729,
0.37048,0.86227,0.68476,0.6342,0.14132,
0.079302,0.87615,0.42043,0.48766,0.46033,
0.51568,0.27199,0.23158,0.89953,0.9087,
0.60364,0.36524,0.59859,0.66849,0.89456,
0.087336,0.53901,0.42845,0.61715,0.55888,
0.22585,0.10452,0.0099777,0.059153,0.32265,
0.77948,0.33548,0.61957,0.99289,0.64801,
0.53978,0.23225,0.73983,0.88899,0.85981,
0.59706,0.65475,0.91501,0.43318,0.28976,
0.63188,0.29542,0.62203,0.047534,0.99461,
0.20676,0.60736,0.34763,0.71774,0.027993,
0.066842,0.92706,0.087772,0.3324,0.52618,
0.24664,0.5429,0.78087,0.52188,0.93195,
0.14711,0.41677,0.28029,0.5981,0.036475,
0.063687,0.32288,0.098379,0.17004,0.37116,
0.039762,0.70924,0.64134,0.17406,0.062152,
0.40666,0.46306,0.20267,0.86955,0.59794,
0.023014,0.89943,0.45295,0.058026,0.10627,
0.99843,0.86633,0.61521,0.026944,0.32252,
0.46378,0.099017,0.57099,0.32588,0.45049,
0.57784,0.074844,0.057343,0.30096,0.52172,
0.56188,0.24155,0.91272,0.82573,0.44455,
0.98206,0.57827,0.23442,0.81059,0.45127,
0.24997,0.95544,0.14265,0.51256,0.97193,
0.64832,0.61467,0.46965,0.57778,0.91131,
0.37622,0.22876,0.42352,0.2736,0.44457,
0.62752,0.53464,0.38544,0.87345,0.30035,
0.40003,0.51772,0.061825,0.23137,0.11849,
0.09878,0.89028,0.033389,0.83903,0.50727,
0.11372,0.49045,0.59944,0.090216,0.97822,
0.65302,0.46113,0.86376,0.26283,0.82396,
0.32898,0.9413,0.24411,0.95711,0.51082,
0.56458,0.99372,0.77098,0.3138,0.057873,
0.044074,0.81294,0.41207,0.38416,0.52313,
0.89215,0.40592,0.60439,0.094804,0.33598,
0.14484,0.24488,0.37897,0.27032,0.2156,
0.63373,0.79972,0.20851,0.89626,0.74571,
0.53639,0.96997,0.56488,0.21769,0.85776,
0.862,0.3133,0.32698,0.84001,0.49253,
0.051902,0.77802,0.42667,0.27996,0.33351,
0.36694,0.79479,0.038691,0.72667,0.87268,
0.28581,0.65685,0.23189,0.62195,0.075124,
0.96676,0.60997,0.38372,0.03057,0.85757,
0.60354,0.84784,0.50462,0.0079108,0.91907,
0.4107,0.73237,0.15111,0.81523,0.9096,
0.77794,0.75712,0.4052,0.7019,0.56546,
0.58474,0.34491,0.70423,0.16086,0.0015139,
0.562,0.54648,0.55083,0.69454,0.92749,
0.94421,0.90326,0.60411,0.92621,0.11695,
0.3408,0.03673,0.53837,0.73723,0.18122,
0.42625,0.098168,0.30322,0.78012,0.53696,
0.76907,0.63894,0.89315,0.06069,0.17577,
0.41633,0.73979,0.89295,0.025853,0.13756,
0.42409,0.76457,0.52444,0.75448,0.1698,
0.67273,0.61865,0.0068461,0.74023,0.99174,
0.12818,0.36058,0.19107,0.7144,0.17787,
0.98685,0.016999,0.039796,0.80433,0.85979,
0.56625,0.75411,0.51951,0.5859,0.19073,
0.50344,0.050915,0.056096,0.3352,0.6309,
0.89198,0.6734,0.68527,0.69574,0.79983,
0.66061,0.51994,0.33174,0.93417,0.24648,
0.51121,0.74032,0.25417,0.84558,0.53815,
0.91014,0.35071,0.93532,0.92772,0.5877,
0.10176,0.36688,0.27585,0.2661,0.22654,
0.0013208,0.89301,0.4535,0.57842,0.31561,
0.99402,0.98389,0.96455,0.66627,0.72616,
0.33404,0.52277,0.27335,0.71838,0.77802,
0.081064,0.22158,0.20396,0.6241,0.72521,
0.83436,0.018895,0.20212,0.46909,0.3784,
0.34039,0.063851,0.76134,0.40266,0.67429,
0.55108,0.051457,0.3075,0.96544,0.93148,
0.37803,0.61767,0.56253,0.83056,0.9577,
0.075506,0.88802,0.56245,0.19436,0.22145,
0.70576,0.59676,0.5863,0.96849,0.58174,
0.0998,0.1666,0.10209,0.14625,0.67156,
0.63985,0.37244,0.16287,0.38952,0.79998,
0.39979,0.75512,0.29521,0.64001,0.88512,
0.20964,0.48036,0.11299,0.13237,0.064002,
0.079297,0.62695,0.41133,0.63859,0.85656,
0.76358,0.97616,0.78154,0.93558,0.73915,
0.25362,0.71925,0.69351,0.77332,0.74722,
0.31838,0.51043,0.77307,0.57282,0.95385,
0.17157,0.90724,0.75222,0.28618,0.62741,
0.46283,0.12944,0.54938,0.96951,0.44214,
0.59096,0.11097,0.19985,0.16296,0.036877,
0.27272,0.23013,0.35854,0.13434,0.99859,
0.51352,0.38785,0.24985,0.36475,0.39906,
0.9263,0.49547,0.60919,0.0048085,0.58773,
0.77746,0.65718,0.52831,0.82577,0.96243,
0.31374,0.79709,0.28518,0.014895,0.09415,
0.32872,0.30557,0.018022,0.16221,0.44399,
0.76677,0.68126,0.71454,0.45978,0.91927,
0.98894,0.9326,0.46152,0.90492,0.38645,
0.60302,0.56032,0.84584,0.2848,0.66332,
0.60226,0.65646,0.30993,0.33164,0.1882,
0.10068,0.28656,0.35483,0.53576,0.99083,
0.028178,0.70953,0.90516,0.86583,0.11917,
0.95528,0.4409,0.87576,0.86495,0.35515,
0.63115,0.86455,0.020992,0.07682,0.37671,
0.14919,0.034097,0.78225,0.32725,0.81748,
0.17362,0.67612,0.8756,0.75716,0.22963,
0.35894,0.36316,0.26744,0.3373,0.086961,
0.45156,0.45466,0.029036,0.6371,0.059459,
0.1692,0.68468,0.55457,0.0060358,0.28809,
0.37661,0.14649,0.074534,0.46074,0.37039,
0.82456,0.53754,0.81752,0.46038,0.014034,
0.0049832,0.16623,0.36594,0.71821,0.15942,
0.27877,0.64494,0.2872,0.32193,0.15542,
0.38766,0.89573,0.88777,0.39361,0.67546,
0.25221,0.94999,0.62468,0.20695,0.10966,
0.56585,0.27433,0.071165,0.15845,0.04952,
0.68088,0.78176,0.80737,0.26505,0.8959,
0.60799,0.014401,0.34403,0.53366,0.62776,
0.44674,0.81111,0.14483,0.97483,0.83376,
0.3401,0.61624,0.30376,0.089219,0.52153,
0.82529,0.76393,0.94764,0.33355,0.38974,
0.15041,0.3337,0.55368,0.55029,0.16042,
0.11709,0.39855,0.83138,0.185,0.50079,
0.12631,0.86463,0.76665,0.56435,0.38957,
0.49014,0.8869,0.90505,0.49839,0.52923,
0.9097,0.5786,0.77579,0.66139,0.46966,
0.21977,0.60259,0.18426,0.19751,0.86199,
0.12565,0.64558,0.43732,0.61261,0.73748,
0.30328,0.042968,0.83551,0.36931,0.66134,
0.89572,0.2741,0.9979,0.83435,0.79134,
0.65661,0.54367,0.38668,0.82223,0.59533,
0.78164,0.98767,0.78825,0.85292,0.48561,
0.87376,0.33385,0.20532,0.49436,0.031209,
0.82757,0.26426,0.6775,0.79386,0.68184,
0.65076,0.23734,0.4774,0.93642,0.2411,
0.20911,0.27247,0.77582,0.33141,0.60307,
0.18401,0.087459,0.30892,0.23088,0.90919,
0.93688,0.031879,0.59365,0.043818,0.4249,
0.52158,0.84034,0.62503,0.2552,0.90469,
0.7673,0.56267,0.89715,0.36808,0.33773,
0.61886,0.086895,0.3755,0.18738,0.81053,
0.72325,0.67467,0.91398,0.82096,0.54872,
0.015542,0.094686,0.51476,0.95248,0.84306,
0.93501,0.97341,0.37253,0.3209,0.78325,
0.21222,0.95915,0.93023,0.36573,0.34491,
0.69268,0.7166,0.80034,0.59367,0.097175,
0.55951,0.04309,0.75763,0.65965,0.59946,
0.22699,0.71371,0.7245,0.34422,0.032751,
0.84362,0.7703,0.50965,0.70696,0.79117,
0.33067,0.91368,0.5767,0.28791,0.26686,
0.89018,0.3661,0.15002,0.94633,0.65084,
0.98221,0.19383,0.82874,0.4938,0.59078,
0.044891,0.56446,0.63354,0.4981,0.059624,
0.51516,0.053195,0.84363,0.59467,0.69102,
0.64723,0.62344,0.95386,0.17694,0.47491,
0.8424,0.49696,0.7624,0.17988,0.69162,
0.91442,0.16266,0.4281,0.041852,0.22575,
0.66615,0.87134,0.34473,0.027252,0.90528,
0.47106,0.91046,0.32798,0.6911,0.017448,
0.51235,0.33181,0.33611,0.95335,0.75051,
0.33392,0.63283,0.41555,0.5765,0.9595,
0.61585,0.15058,0.83827,0.72617,0.10885,
0.16135,0.29701,0.42358,0.52841,0.2751,
0.82796,0.79278,0.27375,0.61569,0.10155,
0.66996,0.3474,0.6695,0.2953,0.68043,
0.39377,0.39182,0.20696,0.72452,0.81931,
0.92647,0.51412,0.19314,0.6029,0.19818,
0.41039,0.85307,0.42908,0.92566,0.37308,
0.69825,0.58863,0.10087,0.4124,0.92471,
0.05555,0.38405,0.24064,0.22781,0.35712,
0.9241,0.5025,0.80324,0.35027,0.45868,
0.9511,0.13298,0.070307,0.70608,0.9317,
0.1089,0.6517,0.14606,0.69396,0.62561,
0.043723,0.79792,0.98621,0.43674,0.34475,
0.048806,0.2252,0.58101,0.60138,0.25252,
0.65434,0.060096,0.94051,0.092531,0.33398,
0.65105,0.50391,0.53093,0.37912,0.76891,
0.28251,0.64927,0.92218,0.33067,0.81344,
0.78764,0.12432,0.15739,0.415,0.68733,
0.6577,0.67998,0.0093867,0.12861,0.76391,
0.84192,0.74695,0.94452,0.91971,0.22307,
0.50977,0.78774,0.52245,0.61739,0.55824,
0.73099,0.063402,0.49279,0.22992,0.33739,
0.55164,0.74369,0.41231,0.50762,0.12897,
0.21484,0.29283,0.44997,0.24028,0.26326,
0.89158,0.58614,0.93867,0.61808,0.25072,
0.10254,0.54636,0.44265,0.69151,0.75944,
0.39942,0.56741,0.20558,0.94353,0.51281,
0.27625,0.22914,0.95293,0.68942,0.70787,
0.65692,0.77897,0.58047,0.50323,0.87873,
0.39295,0.24264,0.28163,0.90477,0.70268,
0.85306,0.37868,0.63302,0.031557,0.92003,
0.46975,0.5199,0.76631,0.33622,0.62599,
0.3269,0.31111,0.64608,0.43369,0.10289,
0.23615,0.97178,0.6989,0.65945,0.3261,
0.019822,0.57047,0.036335,0.26474,0.19066,
0.4493,0.76866,0.62794,0.3147,0.29204,
0.078238,0.17194,0.72128,0.089449,0.91781,
0.42968,0.079723,0.57214,0.83177,0.34737,
0.77989,0.70834,0.59094,0.37699,0.39383,
0.42831,0.42303,0.21892,0.49676,0.03987,
0.56482,0.049143,0.87228,0.86002,0.33826,
0.97403,0.95478,0.98862,0.15772,0.0029184,
0.24818,0.34629,0.23334,0.75589,0.22593,
0.44844,0.25944,0.34579,0.90295,0.012701,
0.13222,0.38806,0.84062,0.21327,0.93523,
0.9698,0.37526,0.98486,0.12209,0.76096,
0.57936,0.62116,0.4899,0.22045,0.14413,
0.63518,0.80242,0.66964,0.4716,0.94448,
0.59148,0.22675,0.68384,0.32064,0.74892,
0.99814,0.26973,0.93187,0.69639,0.095259,
0.24587,0.93466,0.47978,0.88253,0.7793,
0.018284,0.77076,0.6858,0.86767,0.58051,
0.6261,0.11115,0.21369,0.03637,0.44454,
0.32658,0.28729,0.49645,0.18171,0.93379,
0.94018,0.59146,0.00080862,0.90301,0.68289,
0.073851,0.99638,0.31446,0.80922,0.46145,
0.58714,0.4776,0.55245,0.17532,0.58724,
0.1548,0.44545,0.53762,0.4619,0.83411,
0.99617,0.97224,0.74593,0.19996,0.036329,
0.1886,0.57755,0.62827,0.36477,0.63656,
0.89129,0.25352,0.28636,0.12542,0.52857,
0.82077,0.29978,0.78768,0.43701,0.55054,
0.98611,0.68978,0.97014,0.095966,0.89809,
0.058443,0.12332,0.63073,0.3701,0.31957,
0.79569,0.3017,0.60746,0.46197,0.29259,
0.83242,0.57053,0.070508,0.26631,0.17087,
0.56031,0.20209,0.49237,0.72937,0.955,
0.76237,0.080068,0.93825,0.97447,0.60711,
0.95749,0.33065,0.90122,0.00036562,0.48328,
0.14747,0.31499,0.17138,0.59291,0.65761,
0.34896,0.053693,0.047168,0.055858,0.071512,
0.015676,0.97368,0.7348,0.75239,0.033314,
0.11761,0.2613,0.89336,0.97206,0.99407,
0.34679,0.2923,0.18725,0.74315,0.71802,
0.18889,0.43394,0.045593,0.65547,0.78104,
0.8629,0.78535,0.43349,0.34782,0.055389,
0.81944,0.10604,0.006074,0.27504,0.94277,
0.67426,0.38566,0.85695,0.67482,0.6081,
0.43943,0.96129,0.82291,0.010499,0.14162,
0.48281,0.58062,0.29299,0.43887,0.39827,
0.13731,0.95276,0.36004,0.83698,0.041336,
0.42383,0.075638,0.31381,0.94598,0.8545,
0.17353,0.53588,0.29489,0.90287,0.081387,
0.78747,0.17441,0.71999,0.23201,0.20714,
0.21613,0.97667,0.38185,0.63529,0.046564,
0.23207,0.87484,0.46014,0.0030636,0.46494,
0.82407,0.87189,0.30107,0.23601,0.83158,
0.43775,0.50709,0.18547,0.1515,0.90681,
0.99275,0.39008,0.82681,0.95406,0.89384,
0.84101,0.68328,0.1718,0.23755,0.43429,
0.025062,0.26865,0.98016,0.31269,0.57175,
0.61166,0.90154,0.39604,0.69277,0.20449,
0.062853,0.62239,0.68802,0.29679,0.57156,
0.31622,0.8095,0.3655,0.69879,0.63343,
0.96458,0.2008,0.59818,0.083703,0.54441,
0.84615,0.040698,0.32936,0.70935,0.86031,
0.5009,0.99476,0.42864,0.26735,0.47295,
0.16085,0.1589,0.26834,0.75712,0.60917,
0.96693,0.77332,0.57286,0.45022,0.28706,
0.75239,0.094358,0.10679,0.73489,0.35459,
0.58063,0.29886,0.71368,0.36049,0.71846,
0.99532,0.0837,0.46821,0.093803,0.72517,
0.91205,0.086414,0.51995,0.55263,0.68515,
0.29965,0.60188,0.1985,0.65676,0.69836,
0.45968,0.15766,0.42179,0.59605,0.32233,
0.83073,0.12269,0.25118,0.93761,0.65521,
0.75291,0.81086,0.048427,0.41471,0.72549,
0.13904,0.63256,0.24683,0.83997,0.83667,
0.45307,0.39476,0.9616,0.0081783,0.39844,
0.48777,0.61814,0.07001,0.14269,0.65423,
0.21832,0.11913,0.043137,0.16556,0.51111,
0.86727,0.76626,0.13495,0.064921,0.67244,
0.48221,0.49542,0.31348,0.3235,0.9091,
0.45034,0.74864,0.18525,0.084922,0.20703,
0.55892,0.6617,0.93921,0.040245,0.16366,
0.43812,0.32567,0.089932,0.81032,0.39956,
0.65531,0.56684,0.0031219,0.65135,0.3171,
0.26593,0.46045,0.62316,0.80185,0.56116,
0.024804,0.79354,0.27309,0.82291,0.68855,
0.6039,0.38695,0.065535,0.99859,0.66104,
0.22917,0.092255,0.24098,0.90257,0.21735,
0.47952,0.90923,0.19378,0.44799,0.6178,
0.59421,0.66588,0.26768,0.42387,0.79932,
0.93231,0.46972,0.3063,0.43937,0.30686,
0.27501,0.13875,0.16655,0.49395,0.086584,
0.92684,0.14848,0.16048,0.71772,0.67489,
0.35852,0.090458,0.008248,0.37278,0.78831,
0.44729,0.099625,0.80583,0.65945,0.45236,
0.53336,0.52666,0.62968,0.64656,0.19183,
0.80173,0.19657,0.41129,0.23569,0.27221,
0.0086685,0.92828,0.52022,0.46413,0.52353,
0.33681,0.33238,0.98829,0.47372,0.74686,
0.66996,0.79559,0.27254,0.28392,0.71726,
0.18464,0.8337,0.16967,0.95327,0.72255,
0.13425,0.85309,0.93169,0.56418,0.39882,
0.4416,0.70168,0.26004,0.028529,0.94797,
0.99617,0.77948,0.9894,0.48755,0.12798,
0.46437,0.74259,0.67339,0.87398,0.75825,
0.61718,0.27482,0.33638,0.83572,0.38589,
0.080347,0.86558,0.43537,0.17208,0.0042964,
0.6525,0.36969,0.3761,0.44134,0.51612,
0.30011,0.47832,0.0062469,0.17144,0.30356,
0.74446,0.58089,0.70121,0.80466,0.85199,
0.11772,0.65726,0.75295,0.70213,0.60512,
0.038267,0.19515,0.33068,0.64283,0.2174,
0.07691,0.98059,0.20676,0.98325,0.61037,
0.24798,0.51796,0.71283,0.25987,0.11469,
0.54967,0.2189,0.31218,0.97313,0.30919,
0.94625,0.41056,0.010334,0.31007,0.15521,
0.99408,0.45024,0.54522,0.26233,0.88254,
0.10306,0.82356,0.42718,0.92034,0.52801,
0.87083,0.35281,0.40015,0.59792,0.91152,
0.13299,0.042167,0.40104,0.59571,0.91968,
0.29378,0.54504,0.78168,0.3274,0.095263,
0.44168,0.029077,0.67539,0.11159,0.19964,
0.53016,0.62736,0.3736,0.60645,0.84639,
0.70195,0.39287,0.57187,0.24977,0.72735,
0.34333,0.078635,0.30223,0.3004,0.023221,
0.17611,0.86092,0.65595,0.85654,0.27094,
0.89595,0.20879,0.96417,0.41436,0.22957,
0.81837,0.36044,0.50152,0.74767,0.83185,
0.39447,0.096636,0.26539,0.79195,0.93686,
0.56826,0.93802,0.59725,0.98795,0.76235,
0.18562,0.51677,0.64015,0.86895,0.60012,
0.99967,0.12918,0.29337,0.47688,0.12958,
0.22812,0.18107,0.67202,0.32578,0.79059,
0.54601,0.23056,0.68676,0.049083,0.5726,
0.99376,0.36297,0.55872,0.92728,0.25988,
0.83248,0.17513,0.87984,0.27963,0.84948,
0.80672,0.18913,0.97856,0.25373,0.77855,
0.81078,0.49795,0.91572,0.10098,0.13582,
0.85832,0.018646,0.72833,0.81666,0.11622,
0.48856,0.34745,0.77389,0.28529,0.8244,
0.17306,0.86364,0.92605,0.1911,0.050352,
0.54845,0.0095179,0.82836,0.90854,0.7053,
0.24333,0.58923,0.9389,0.8955,0.60623,
0.79238,0.83617,0.56014,0.0006951,0.25128,
0.43195,0.43065,0.87523,0.067641,0.33991,
0.17706,0.24604,0.72782,0.2882,0.23734,
0.13966,0.22073,0.81755,0.37823,0.66552,
0.93581,0.42648,0.60516,0.96958,0.36465,
0.66445,0.31476,0.88572,0.29554,0.29537,
0.90708,0.49288,0.20714,0.025819,0.17021,
0.30234,0.96144,0.78366,0.67326,0.84072,
0.42085,0.70374,0.48129,0.82058,0.43739,
0.85997,0.0088972,0.99603,0.2752,0.033516,
0.1521,0.4796,0.88884,0.28342,0.78413,
0.17039,0.73129,0.24662,0.081841,0.85291,
0.68655,0.95811,0.98506,0.53472,0.90598,
0.47665,0.88967,0.02385,0.12872,0.74064,
0.39102,0.39363,0.60185,0.023885,0.38767,
0.056746,0.68957,0.52039,0.076086,0.40626,
0.43021,0.65931,0.21839,0.081089,0.46767,
0.72867,0.61398,0.9006,0.25278,0.2201,
0.2358,0.81013,0.70241,0.8785,0.71101,
0.66108,0.43958,0.56279,0.99031,0.95732,
0.33682,0.15546,0.41708,0.55448,0.029752,
0.86671,0.89441,0.49719,0.20929,0.7216,
0.69972,0.46075,0.17059,0.79437,0.8885,
0.14251,0.93967,0.00026086,0.43881,0.011168,
0.23452,0.70698,0.69416,0.26512,0.32657,
0.093234,0.041599,0.75928,0.32487,0.52182,
0.95189,0.66898,0.47312,0.64937,0.66564,
0.23015,0.80489,0.50279,0.43403,0.70903,
0.55526,0.255,0.075201,0.5891,0.67424,
0.58801,0.5337,0.47262,0.32381,0.43099,
0.7649,0.99646,0.26502,0.79207,0.13461,
0.035536,0.45395,0.56298,0.23088,0.14476,
0.6771,0.35407,0.51047,0.69471,0.59529,
0.50611,0.7764,0.6852,0.31949,0.83214,
0.66082,0.58707,0.019078,0.83695,0.114,
0.84042,0.3375,0.42944,0.79378,0.14157,
0.6081,0.93063,0.63644,0.047771,0.69835,
0.25711,0.6909,0.36318,0.54188,0.22214,
0.90549,0.37354,0.078816,0.03093,0.6269,
0.937,0.31994,0.74517,0.28893,0.12435,
0.51408,0.38751,0.97861,0.29182,0.59234,
0.57136,0.61586,0.56091,0.27986,0.15185,
0.73301,0.039699,0.80162,0.40642,0.83001,
0.97104,0.42248,0.053905,0.78704,0.63008,
0.50882,0.56769,0.30919,0.74714,0.33666,
0.88335,0.33366,0.79601,0.80478,0.050239,
0.40251,0.96519,0.41438,0.33052,0.63637,
0.85803,0.78828,0.53841,0.74167,0.73972,
0.44479,0.85041,0.49453,0.94816,0.79779,
0.13304,0.23777,0.12772,0.53412,0.056697,
0.46857,0.98269,0.68222,0.8709,0.49621,
0.28766,0.18302,0.96122,0.75626,0.95035,
0.67127,0.18605,0.28984,0.058208,0.044066,
0.27956,0.17047,0.6771,0.90242,0.35199,
0.10016,0.793,0.46843,0.68734,0.68308,
0.42144,0.65934,0.63343,0.52515,0.19128,
0.51191,0.74833,0.41983,0.73748,0.68647,
0.95573,0.30791,0.3423,0.38703,0.026176,
0.63355,0.56525,0.36551,0.42146,0.4622,
0.31444,0.22566,0.11594,0.13397,0.94652,
0.49243,0.74821,0.31094,0.0077053,0.61562,
0.35757,0.64883,0.84598,0.10135,0.5883,
0.58238,0.5218,0.4546,0.9667,0.18231,
0.55939,0.62647,0.03013,0.64227,0.83816,
0.39514,0.43734,0.084883,0.58138,0.87168,
0.20691,0.09461,0.18795,0.66784,0.73242,
0.63101,0.46821,0.31436,0.64147,0.56203,
0.66373,0.12488,0.67403,0.68031,0.93012,
0.82882,0.1781,0.94924,0.19962,0.21768,
0.0085219,0.30063,0.27447,0.58645,0.39655,
0.1864,0.39282,0.15459,0.8323,0.14975,
0.67955,0.75009,0.01489,0.14225,0.030656,
0.8087,0.047146,0.66618,0.45603,0.803,
0.35844,0.025334,0.21859,0.71091,0.54892,
0.86969,0.73884,0.10323,0.97143,0.73871,
0.013858,0.67912,0.4855,0.034718,0.11444,
0.076625,0.60782,0.23347,0.14201,0.53017,
0.9303,0.6169,0.68989,0.23443,0.20684,
0.037366,0.66336,0.50725,0.4858,0.8617,
0.96551,0.75969,0.24668,0.67627,0.75931,
0.58619,0.82244,0.92684,0.63757,0.018938,
0.18434,0.026741,0.935,0.48447,0.79479,
0.45684,0.49421,0.42571,0.89177,0.87429,
0.63315,0.29228,0.29168,0.2455,0.70435,
0.77461,0.82265,0.80619,0.25377,0.8005,
0.62971,0.2315,0.38108,0.4532,0.50056,
0.54057,0.34515,0.29143,0.70353,0.93408,
0.59417,0.59309,0.73873,0.37264,0.45327,
0.50938,0.92871,0.45079,0.63743,0.45936,
0.55377,0.041233,0.8022,0.67151,0.20033,
0.014586,0.76317,0.56751,0.65421,0.63474,
0.73433,0.88733,0.66591,0.12067,0.71374,
0.44723,0.64098,0.6489,0.67838,0.058828,
0.22395,0.16798,0.92853,0.51523,0.67336,
0.6678,0.081461,0.22507,0.28419,0.91376,
0.2291,0.24862,0.67629,0.64849,0.70368,
0.8371,0.7273,0.019434,0.58375,0.92088,
0.091678,0.55456,0.9308,0.19945,0.52532,
0.12674,0.74873,0.31467,0.18717,0.9804,
0.71283,0.82565,0.68322,0.6894,0.53088,
0.18885,0.49302,0.55954,0.82828,0.20287,
0.52775,0.80781,0.3551,0.54776,0.71744,
0.20251,0.64376,8.9016e-05,0.51318,0.56983,
0.0070367,0.89247,0.83319,0.77517,0.78814,
0.37363,0.15231,0.35226,0.6451,0.93194,
0.093321,0.73878,0.055267,0.75716,0.46349,
0.045004,0.84203,0.16471,0.11507,0.27156,
0.31439,0.60609,0.67747,0.98764,0.99326,
0.75754,0.27506,0.95401,0.41108,0.21662,
0.62913,0.014869,0.0433,0.18026,0.20047,
0.71936,0.44287,0.84549,0.38979,0.83755,
0.74842,0.58386,0.16057,0.52884,0.46267,
0.37954,0.09323,0.25915,0.33584,0.37511,
0.14626,0.31677,0.28123,0.82012,0.34209,
0.872,0.26803,0.77543,0.60921,0.017969,
0.70151,0.015752,0.68389,0.8779,0.43181,
0.63134,0.58604,0.22741,0.78465,0.28764,
0.92438,0.29848,0.5363,0.33365,0.23729,
0.54524,0.10769,0.13716,0.094427,0.31088,
0.81264,0.51356,0.86648,0.98049,0.32665,
0.21962,0.51822,0.35301,0.85948,0.26002,
0.84372,0.5828,0.71941,0.34352,0.0092739,
0.19245,0.50739,0.024242,0.55004,0.27618,
0.69624,0.88135,0.024681,0.34121,0.42141,
0.080011,0.079354,0.072376,0.90034,0.96931,
0.39131,0.31359,0.55331,0.79205,0.79831,
0.86326,0.79796,0.0063764,0.9199,0.017962,
0.029416,0.71145,0.53841,0.52459,0.50224,
0.065672,0.23004,0.11704,0.98977,0.070516,
0.060024,0.66158,0.34406,0.12851,0.63714,
0.74649,0.80527,0.26903,0.43398,0.40181,
0.60814,0.77009,0.94112,0.13146,0.25572,
0.37831,0.99374,0.34095,0.89955,0.23748,
0.22,0.99108,0.95111,0.63822,0.50413,
0.35812,0.76845,0.78436,0.028918,0.052441,
0.32342,0.79969,0.63023,0.9828,0.15825,
0.61668,0.30872,0.085079,0.87683,0.5907,
0.30312,0.16807,0.33988,0.067446,0.65349,
0.24429,0.75792,0.29901,0.41882,0.055832,
0.030856,0.18658,0.27091,0.066243,0.33349,
0.049906,0.10131,0.07187,0.90451,0.3118,
0.59177,0.37763,0.95156,0.7424,0.88709,
0.5989,0.68963,0.39112,0.22327,0.13126,
0.17248,0.9386,0.034482,0.87956,0.52233,
0.80825,0.44758,0.34654,0.75715,0.36021,
0.55661,0.88872,0.84016,0.24403,0.43562,
0.82855,0.1527,0.75795,0.87651,0.50212,
0.57027,0.85148,0.98464,0.79021,0.35068,
0.5019,0.25521,0.50987,0.07862,0.055899,
0.63499,0.6169,0.081473,0.60952,0.54467,
0.85592,0.64269,0.27163,0.10675,0.060513,
0.62188,0.69907,0.27494,0.42197,0.36582,
0.082159,0.06977,0.12089,0.67785,0.21894,
0.23728,0.0086346,0.72711,0.56103,0.55197,
0.74225,0.82463,0.39339,0.13594,0.19574,
0.7203,0.66963,0.05985,0.38428,0.49124,
0.77416,0.14781,0.48553,0.071835,0.78744,
0.39373,0.23616,0.080568,0.079005,0.92743,
0.20472,0.95658,0.33974,0.95997,0.34315,
0.063423,0.61247,0.55561,0.90645,0.20123,
0.78769,0.34772,0.97215,0.83605,0.71181,
0.2149,0.015448,0.21004,0.53647,0.10846,
0.85376,0.91085,0.33373,0.46914,0.0028969,
0.31091,0.48522,0.26217,0.68311,0.91119,
0.5705,0.26528,0.92847,0.064245,0.6263,
0.94828,0.51639,0.9567,0.27202,0.52699,
0.91119,0.21469,0.82679,0.13833,0.43775,
0.87528,0.70431,0.49702,0.6096,0.077828,
0.24143,0.025784,0.84648,0.014863,0.55042,
0.11967,0.58795,0.62221,0.74702,0.30993,
0.9165,0.78227,0.10105,0.2811,0.90751,
0.11109,0.78982,0.10228,0.76794,0.66206,
0.7731,0.93198,0.17688,0.020422,0.83364,
0.47319,0.10865,0.089054,0.11513,0.16797,
0.97251,0.286,0.59902,0.85221,0.24478,
0.72717,0.40079,0.3106,0.6538,0.39183,
0.11934,0.97854,0.51825,0.50777,0.32502,
0.19727,0.86289,0.26787,0.53868,0.55147,
0.96326,0.26958,0.26572,0.30999,0.5573,
0.37819,0.24577,0.67618,0.26129,0.96149,
0.49758,0.42777,0.56779,0.61304,0.80699,
0.88078,0.15998,0.089177,0.31313,0.60735,
0.37874,0.92554,0.057013,0.99402,0.66696,
0.61715,0.14789,0.028996,0.57247,0.029982,
0.71153,0.21404,0.25519,0.95859,0.18189,
0.79846,0.040043,0.30916,0.96709,0.26699,
0.12188,0.60738,0.64535,0.87442,0.89733,
0.053178,0.75951,0.8589,0.48172,0.075096,
0.090721,0.20022,0.77829,0.40728,0.82704,
0.14797,0.73291,0.14989,0.83843,0.54537,
0.96117,0.075495,0.93383,0.35942,0.06693,
0.96593,0.64945,0.42062,0.59644,0.34709,
0.21763,0.037841,0.15343,0.37338,0.87682,
0.44115,0.63777,0.38781,0.8372,0.76632,
0.12558,0.092661,0.66032,0.37767,0.10779,
0.62381,0.6216,0.91163,0.60613,0.48535,
0.57674,0.9821,0.51857,0.3487,0.98287,
0.99201,0.89616,0.74158,0.66474,0.73336,
0.43592,0.39963,0.21789,0.88163,0.94207,
0.94895,0.10644,0.057975,0.16372,0.52747,
0.069748,0.010146,0.09168,0.60744,0.056085,
0.6581,0.57582,0.073835,0.24157,0.3512,
0.12674,0.73941,0.44568,0.017612,0.68171,
0.88083,0.13982,0.81085,0.25286,0.024386,
0.53072,0.93485,0.76933,0.40475,0.71781,
0.48132,0.44118,0.94236,0.57878,0.78361,
0.71481,0.14217,0.69292,0.39365,0.12777,
0.6409,0.49751,0.063973,0.52354,0.31454,
0.85489,0.0090151,0.20259,0.8806,0.21805,
0.31769,0.34396,0.80008,0.57108,0.81324,
0.08618,0.19214,0.6295,0.063863,0.74296,
0.6034,0.79844,0.16059,0.73793,0.75704,
0.88131,0.42924,0.18748,0.30303,0.32076,
0.44201,0.36188,0.6256,0.93883,0.2869,
0.2446,0.92085,0.92622,0.30994,0.076475,
0.78088,0.13075,0.46148,0.44017,0.605,
0.12702,0.59904,0.13213,0.11534,0.44107,
0.2711,0.94411,0.51664,0.44529,0.30859,
0.4696,0.77944,0.16881,0.022451,0.70873,
0.60515,0.41368,0.91477,0.47718,0.91622,
0.49691,0.92755,0.70365,0.69464,0.0045936,
0.54022,0.41297,0.62813,0.54354,0.216,
0.5521,0.86275,0.0049393,0.12508,0.77507,
0.020386,0.14263,0.32041,0.90475,0.61416,
0.015898,0.31139,0.19277,0.27547,0.02671,
0.83064,0.76428,0.72116,0.82272,0.76454,
0.12057,0.38433,0.027923,0.0013045,0.006998,
0.16778,0.15044,0.51549,0.41888,0.59381,
0.27698,0.59283,0.58514,0.12159,0.078594,
0.032066,0.16817,0.52417,0.84137,0.30641,
0.19116,0.065119,0.94601,0.24477,0.6254,
0.20667,0.24174,0.74341,0.156,0.82317,
0.093985,0.9798,0.65322,0.28411,0.86723,
0.03014,0.76094,0.89618,0.61887,0.72075,
0.46385,0.75889,0.007142,0.19349,0.091452,
0.16793,0.94225,0.84943,0.13728,0.66073,
0.03849,0.14816,0.18476,0.64536,0.85655,
0.85338,0.49404,0.78088,0.27465,0.96738,
0.99053,0.084449,0.32979,0.2683,0.21592,
0.76702,0.36505,0.15402,0.16502,0.89336,
0.94579,0.51747,0.018373,0.41931,0.12209,
0.95185,0.27018,0.89365,0.89519,0.77553,
0.90882,0.01668,0.95929,0.075253,0.28513,
0.48812,0.97057,0.56811,0.27397,0.12908,
0.15519,0.26881,0.43389,0.3959,0.42525,
0.61439,0.44681,0.012023,0.034271,0.39877,
0.97945,0.54277,0.91239,0.32416,0.29052,
0.87897,0.77914,0.39441,0.88233,0.42567,
0.46638,0.42493,0.45521,0.69542,0.095578,
0.73346,0.13002,0.49653,0.25966,0.59167,
0.22762,0.95512,0.06301,0.18325,0.62424,
0.29141,0.13719,0.18893,0.95662,0.29661,
0.2372,0.66016,0.004296,0.078168,0.30958,
0.22397,0.69529,0.70023,0.85182,0.55906,
0.57033,0.53092,0.85889,0.44635,0.38338,
0.59536,0.30504,0.10111,0.37194,0.22216,
0.64323,0.77666,0.43003,0.62106,0.27907,
0.47805,0.31474,0.82717,0.74316,0.14431,
0.068265,0.53214,0.74396,0.83573,0.72066,
0.88762,0.16285,0.20124,0.88196,0.46196,
0.18705,0.72488,0.88355,0.79801,0.84297,
0.53805,0.20959,0.53045,0.8433,0.21534,
0.71673,0.86267,0.47126,0.4605,0.94253,
0.62834,0.80639,0.40154,0.32139,0.37881,
0.71897,0.19306,0.48999,0.12077,0.13992,
0.9472,0.63094,0.43866,0.70776,0.52965,
0.65243,0.77818,0.61802,0.50209,0.48537,
0.11015,0.91655,0.43262,0.427,0.029286,
0.51043,0.9637,0.27535,0.74341,0.1168,
0.76933,0.76572,0.54077,0.16932,0.93012,
0.20821,0.019761,0.60161,0.60737,0.46885,
0.59631,0.72346,0.40811,0.59761,0.42571,
0.12184,0.34995,0.54675,0.1271,0.081497,
0.28946,0.67682,0.45704,0.49922,0.058478,
0.073606,0.33788,0.33607,0.98496,0.75166,
0.64653,0.88287,0.51189,0.3762,0.88376);
  
  
end package;