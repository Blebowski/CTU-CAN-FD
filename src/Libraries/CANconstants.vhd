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
--  Package with Constants, types and other defintions for CAN FD IP Core.
--------------------------------------------------------------------------------
-- Revision History:
--    June 2015   Created file
--    08.12.2017  Code formatting. Replaced hexadecimal values with shorter
--                notation
--    19.12.2017  Moved TX_DATA registers into separate memory location
--                TX_DATA_REGION to make the direct addressing inside the TXT
--                buffer easier.
--    27.12.2017  Added "DRV_FRAME_SWAP_INDEX" into driving bus.
--    20.1.2018   Removed CAN Frame constants FRAME_BASIC and FRAME_EXTENDED
--                Properly only signals from CAN_FD_frame_format package
--                BASE and EXTENDED should be used!
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

package CANconstants is

  -- IP Core version related constants
  constant CTU_CAN_FD_VERSION_MINOR : std_logic_vector(7 downto 0) := x"01";
  constant CTU_CAN_FD_VERSION_MAJOR : std_logic_vector(7 downto 0) := x"02";

  --Active value of asynchronous reset 
  constant ACT_RESET : std_logic := '0';

  --Definition of basic logic levels for CAN bus
  constant DOMINANT  : std_logic := '0';
  constant RECESSIVE : std_logic := '1';

  constant INTEGRATING_DURATION : natural := 11;
  constant TRAN_BUFF_SIZE       : natural := 600;

  constant BASE_STUFF_LENGTH : natural := 5;
  constant FD_STUFF_LENGTH   : natural := 4;

  constant CAN_BASE_ID_LENGTH : natural := 11;
  constant CAN_EXT_ID_LENGTH  : natural := 18;  --Length Identifier extension only

  constant NO_SYNC   : std_logic_vector(1 downto 0) := "00";
  constant HARD_SYNC : std_logic_vector(1 downto 0) := "01";
  constant RE_SYNC   : std_logic_vector(1 downto 0) := "10";

  --CRC sources
  constant CRC_15_SRC : std_logic_vector(1 downto 0) := "00";
  constant CRC_17_SRC : std_logic_vector(1 downto 0) := "01";
  constant CRC_21_SRC : std_logic_vector(1 downto 0) := "10";

  --Sample point control constants
  constant NOMINAL_SAMPLE   : std_logic_vector(1 downto 0) := "00";
  constant DATA_SAMPLE      : std_logic_vector(1 downto 0) := "01";
  constant SECONDARY_SAMPLE : std_logic_vector(1 downto 0) := "10";

  --Tuples definition for older compiler (less than 2008)
  constant DOMINANT_DOMINANT   : std_logic_vector(1 downto 0) := DOMINANT&DOMINANT;
  constant DOMINANT_RECESSIVE  : std_logic_vector(1 downto 0) := DOMINANT&RECESSIVE;
  constant RECESSIVE_DOMINANT  : std_logic_vector(1 downto 0) := RECESSIVE&DOMINANT;
  constant RECESSIVE_RECESSIVE : std_logic_vector(1 downto 0) := RECESSIVE&RECESSIVE;

  --Error flag definitions 
  constant PASSIVE_ERR_FLAG : std_logic := RECESSIVE;
  constant ACTIVE_ERR_FLAG  : std_logic := DOMINANT;

  constant ERROR_FLAG_LENGTH : natural := 6;

  constant INC_ONE_CON   : std_logic_vector(2 downto 0) := "100";
  constant INC_EIGHT_CON : std_logic_vector(2 downto 0) := "010";
  constant DEC_ONE_CON   : std_logic_vector(2 downto 0) := "001";

  constant TXT_BUFFER_COUNT     : natural := 4;  
  
  constant INT_COUNT            : natural := 12;
  
  ------------------------------------------------------------------------------   
  --DLC Types
  ------------------------------------------------------------------------------
  type dlc_type is array (0 to 15) of std_logic_vector(3 downto 0);
  type length_type is array (0 to 15) of natural;
  constant dlc_codes : dlc_type := ("0000", "0001", "0010", "0011", "0100", "0101",
                                    "0110", "0111", "1000", "1001", "1010", "1011",
                                    "1100", "1101", "1110", "1111");
  constant dlc_length : length_type := (0, 1, 2, 3, 4, 5, 6, 7, 8,
                                        12, 16, 20, 24, 32, 48, 64);
                                        

  constant ZERO      : std_logic := '0';
  constant NO_ACTION : std_logic := '0';

  constant ACK_ALLOWED  : std_logic := '0';
  constant ACK_FORBIDEN : std_logic := '1';

  constant LOOPBACK_ENA : std_logic := '1';
  constant LOOPBACK_DIS : std_logic := '0';

  constant RETR_LIM_DIS : std_logic := '0';
  constant RETR_LIM_ENA : std_logic := '1';

  constant SINGLE_SAMPLING  : std_logic := '0';
  constant TRIPPLE_SAMPLING : std_logic := '1';

  constant ALLOW_BUFFER  : std_logic := '1';
  constant FORBID_BUFFER : std_logic := '0';

  --Definition of register directions for TXT1 and TXT2 buffers
  constant TXT1_DIR : std_logic := '0';
  constant TXT2_DIR : std_logic := '1';

  --CRC polynomials
  constant CRC15_POL : std_logic_vector(15 downto 0) := x"C599";
  constant CRC17_POL : std_logic_vector(19 downto 0) := x"3685B";
  constant CRC21_POL : std_logic_vector(23 downto 0) := x"302899";

  ------------------------------------------------------------------------------
  -- State Machine types
  ------------------------------------------------------------------------------

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
  --Note: control_type is only for sending. For recieving frame 
  -- type is unknown until EDL bit
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

  -- TX arbitrator state type
  type tx_arb_state_type is (
    arb_sel_low_ts,
    arb_sel_upp_ts,
    arb_sel_ffw,
    arb_locked
    );
  
  -- TXT buffer state type
  type txt_fsm_type is (
    txt_empty,
    txt_ready,
    txt_tx_prog,
    txt_ab_prog,
    txt_ok,
    txt_error,
    txt_aborted
  );
  
  ------------------------------------------------------------------------------
  -- TXT Buffer types
  ------------------------------------------------------------------------------
  type txtb_priorities_type is array (0 to TXT_BUFFER_COUNT - 1) of
        std_logic_vector(2 downto 0);
  
  type txtb_output_type is array (0 to TXT_BUFFER_COUNT - 1) of
        std_logic_vector(31 downto 0);
  
  type txt_fsms_type is array (0 to TXT_BUFFER_COUNT - 1) of
        txt_fsm_type;
  
  type txt_sw_cmd_type is record
     set_rdy   : std_logic;
     set_ety   : std_logic;
     set_abt   : std_logic;
  end record;
  
  type txt_hw_cmd_type is record
     lock      : std_logic;
     unlock    : std_logic;
     valid     : std_logic;
     err       : std_logic;
     arbl      : std_logic;
     failed    : std_logic;
  end record;

  ------------------------------------------------------------------------------
  -- Driving bus signal ranges
  ------------------------------------------------------------------------------
  --Prescaler
  constant DRV_TQ_NBT_LOW  : natural := 0;
  constant DRV_TQ_NBT_HIGH : natural := 7;

  constant DRV_TQ_DBT_LOW  : natural := 8;
  constant DRV_TQ_DBT_HIGH : natural := 15;

  constant DRV_PRS_NBT_LOW  : natural := 16;
  constant DRV_PRS_NBT_HIGH : natural := 22;

  constant DRV_PH1_NBT_LOW  : natural := 23;
  constant DRV_PH1_NBT_HIGH : natural := 28;

  constant DRV_PH2_NBT_LOW  : natural := 29;
  constant DRV_PH2_NBT_HIGH : natural := 34;

  constant DRV_PRS_DBT_LOW  : natural := 35;
  constant DRV_PRS_DBT_HIGH : natural := 40;

  constant DRV_PH1_DBT_LOW  : natural := 41;
  constant DRV_PH1_DBT_HIGH : natural := 45;

  constant DRV_PH2_DBT_LOW  : natural := 46;
  constant DRV_PH2_DBT_HIGH : natural := 50;

  constant DRV_SJW_LOW  : natural := 51;
  constant DRV_SJW_HIGH : natural := 55;

  constant DRV_SJW_DBT_LOW  : natural := 56;
  constant DRV_SJW_DBT_HIGH : natural := 60;

  --TimeStampGen
  constant DRV_TS_1_SRC_LOW  : natural := 61;
  constant DRV_TS_1_SRC_HIGH : natural := 63;

  constant DRV_TS_1_RST_INDEX : natural := 64;

  constant DRV_TS_1_MAKE_LOW  : natural := 65;
  constant DRV_TS_1_MAKE_HIGH : natural := 68;

  constant DRV_TS_2_SRC_LOW  : natural := 69;
  constant DRV_TS_2_SRC_HIGH : natural := 71;

  constant DRV_TS_2_RST_INDEX : natural := 72;

  constant DRV_TS_2_MAKE_LOW  : natural := 73;
  constant DRV_TS_2_MAKE_HIGH : natural := 76;

  --Message Filter
  constant DRV_FILTER_A_MASK_LOW  : natural := 81;
  constant DRV_FILTER_A_MASK_HIGH : natural := 109;

  constant DRV_FILTER_A_CTRL_LOW  : natural := 110;
  constant DRV_FILTER_A_CTRL_HIGH : natural := 113;

  constant DRV_FILTER_A_BITS_LOW  : natural := 114;
  constant DRV_FILTER_A_BITS_HIGH : natural := 142;

  constant DRV_FILTER_B_MASK_LOW  : natural := 143;
  constant DRV_FILTER_B_MASK_HIGH : natural := 171;

  constant DRV_FILTER_B_CTRL_LOW  : natural := 172;
  constant DRV_FILTER_B_CTRL_HIGH : natural := 175;

  constant DRV_FILTER_B_BITS_LOW  : natural := 176;
  constant DRV_FILTER_B_BITS_HIGH : natural := 204;

  constant DRV_FILTER_C_MASK_LOW  : natural := 205;
  constant DRV_FILTER_C_MASK_HIGH : natural := 233;

  constant DRV_FILTER_C_CTRL_LOW  : natural := 234;
  constant DRV_FILTER_C_CTRL_HIGH : natural := 237;

  constant DRV_FILTER_C_BITS_LOW  : natural := 238;
  constant DRV_FILTER_C_BITS_HIGH : natural := 266;

  constant DRV_FILTER_RAN_CTRL_LOW  : natural := 267;
  constant DRV_FILTER_RAN_CTRL_HIGH : natural := 270;

  constant DRV_FILTER_RAN_LO_TH_LOW  : natural := 271;
  constant DRV_FILTER_RAN_LO_TH_HIGH : natural := 299;

  constant DRV_FILTER_RAN_HI_TH_LOW  : natural := 300;
  constant DRV_FILTER_RAN_HI_TH_HIGH : natural := 328;

  constant DRV_FILTERS_ENA_INDEX : natural := 329;

  --RX Buffer
  constant DRV_ERASE_RX_INDEX   : natural := 350;
  constant DRV_RTSOPT_INDEX     : natural := 351;
  constant DRV_READ_START_INDEX : natural := 352;
  constant DRV_CLR_OVR_INDEX    : natural := 353;

  --TXT Buffer
  constant DRV_ERASE_TXT2_INDEX : natural := 356;
  constant DRV_TXT1_WR          : natural := 357;

  --TX Buffer
  constant DRV_ERASE_TXT1_INDEX : natural := 366;
  constant DRV_TXT2_WR          : natural := 367;


  --Interrupt manager indices 
  constant DRV_INT_CLR_HIGH   : natural := 747;
  constant DRV_INT_CLR_LOW    : natural := 736;

  constant DRV_INT_ENA_SET_HIGH     : natural := 779;
  constant DRV_INT_ENA_SET_LOW      : natural := 768;

  constant DRV_INT_ENA_CLR_HIGH   : natural := 811;
  constant DRV_INT_ENA_CLR_LOW    : natural := 800;
  
  constant DRV_INT_MASK_SET_HIGH   : natural := 843;
  constant DRV_INT_MASK_SET_LOW    : natural := 832;
  
  constant DRV_INT_MASK_CLR_HIGH   : natural := 875;
  constant DRV_INT_MASK_CLR_LOW    : natural := 864;

  constant DRV_SAM_INDEX : natural := 372;
  
  
  ------------------------------------------------------------------------------
  -- CAN Core
  ------------------------------------------------------------------------------

  --Fault Confinement
  constant DRV_EWL_LOW  : natural := 400;
  constant DRV_EWL_HIGH : natural := 407;

  constant DRV_ERP_LOW  : natural := 408;
  constant DRV_ERP_HIGH : natural := 415;

  constant DRV_CTR_VAL_LOW  : natural := 416;
  constant DRV_CTR_VAL_HIGH : natural := 424;

  constant DRV_CTR_SEL_LOW  : natural := 425;
  constant DRV_CTR_SEL_HIGH : natural := 428;

  --Operation control FSM
  constant DRV_CAN_FD_ENA_INDEX    : natural := 460;
  constant DRV_RTR_PREF_INDEX      : natural := 461;
  constant DRV_BUS_MON_ENA_INDEX   : natural := 470;
  constant DRV_SELF_TEST_ENA_INDEX : natural := 471;

  constant DRV_RETR_LIM_ENA_INDEX : natural := 465;

  constant DRV_RETR_TH_LOW  : natural := 466;
  constant DRV_RETR_TH_HIGH : natural := 469;

  constant DRV_ABORT_TRAN_INDEX : natural := 472;

  constant DRV_SET_RX_CTR_INDEX : natural := 473;
  constant DRV_SET_TX_CTR_INDEX : natural := 474;

  constant DRV_SET_CTR_VAL_HIGH : natural := 506;
  constant DRV_SET_CTR_VAL_LOW  : natural := 475;

  constant DRV_ACK_FORB_INDEX        : natural := 507;
  constant DRV_INT_LOOBACK_ENA_INDEX : natural := 508;

  constant DRV_ENA_INDEX     : natural := 509;
  constant DRV_FD_TYPE_INDEX : natural := 510;

  ------------------------------------------------------------------------------
  -- Event logger
  ------------------------------------------------------------------------------
  constant DRV_TRIG_CONFIG_DATA_HIGH : natural := 551;
  constant DRV_TRIG_CONFIG_DATA_LOW  : natural := 520;

  constant DRV_TRIG_SOF_INDEX         : natural := 552;
  constant DRV_TRIG_ARB_LOST_INDEX    : natural := 553;
  constant DRV_TRIG_REC_VALID_INDEX   : natural := 554;
  constant DRV_TRIG_TRAN_VALID_INDEX  : natural := 555;
  constant DRV_TRIG_OVL_INDEX         : natural := 556;
  constant DRV_TRIG_ERROR_INDEX       : natural := 557;
  constant DRV_TRIG_BRS_INDEX         : natural := 558;
  constant DRV_TRIG_USER_WRITE_INDEX  : natural := 559;
  constant DRV_TRIG_ARB_START_INDEX   : natural := 560;
  constant DRV_TRIG_CONTR_START_INDEX : natural := 561;
  constant DRV_TRIG_DATA_START_INDEX  : natural := 562;
  constant DRV_TRIG_CRC_START_INDEX   : natural := 563;
  constant DRV_TRIG_ACK_REC_INDEX     : natural := 564;
  constant DRV_TRIG_ACK_N_REC_INDEX   : natural := 565;
  constant DRV_TRIG_EWL_REACHED_INDEX : natural := 566;
  constant DRV_TRIG_ERP_CHANGED_INDEX : natural := 567;
  constant DRV_TRIG_TRAN_START_INDEX  : natural := 568;
  constant DRV_TRIG_REC_START_INDEX   : natural := 569;

  constant DRV_CAP_SOF_INDEX         : natural := 580;
  constant DRV_CAP_ARB_LOST_INDEX    : natural := 581;
  constant DRV_CAP_REC_VALID_INDEX   : natural := 582;
  constant DRV_CAP_TRAN_VALID_INDEX  : natural := 583;
  constant DRV_CAP_OVL_INDEX         : natural := 584;
  constant DRV_CAP_ERROR_INDEX       : natural := 585;
  constant DRV_CAP_BRS_INDEX         : natural := 586;
  constant DRV_CAP_ARB_START_INDEX   : natural := 587;
  constant DRV_CAP_CONTR_START_INDEX : natural := 588;
  constant DRV_CAP_DATA_START_INDEX  : natural := 589;
  constant DRV_CAP_CRC_START_INDEX   : natural := 590;
  constant DRV_CAP_ACK_REC_INDEX     : natural := 591;
  constant DRV_CAP_ACK_N_REC_INDEX   : natural := 592;
  constant DRC_CAP_EWL_REACHED_INDEX : natural := 593;
  constant DRV_CAP_ERP_CHANGED_INDEX : natural := 594;
  constant DRV_CAP_TRAN_START_INDEX  : natural := 595;
  constant DRV_CAP_REC_START_INDEX   : natural := 596;
  constant DRV_CAP_SYNC_EDGE_INDEX   : natural := 597;
  constant DRV_CAP_STUFFED_INDEX     : natural := 598;
  constant DRV_CAP_DESTUFFED_INDEX   : natural := 599;
  constant DRV_CAP_OVR_INDEX         : natural := 600;

  constant DRV_LOG_CMD_STR_INDEX  : natural := 610;
  constant DRV_LOG_CMD_ABT_INDEX  : natural := 611;
  constant DRV_LOG_CMD_UP_INDEX   : natural := 612;
  constant DRV_LOG_CMD_DOWN_INDEX : natural := 613;

  ------------------------------------------------------------------------------
  -- RX, TX and TXT Buffer frame format signal indexes
  ------------------------------------------------------------------------------
  --Tx Message format (Format A)
  constant TX_FFW_HIGH : natural := 639;
  constant TX_FFW_LOW  : natural := 608;

  constant TX_IDW_HIGH : natural := 607;
  constant TX_IDW_LOW  : natural := 576;

  --16 Data words for up to 64 bytes of data
  constant TX_DATAW_HIGH : natural := 575;
  constant TX_DATA1W_LOW : natural := 544;
  constant TX_DATAW_LOW  : natural := 64;

  --Txt message format (Format B)
  constant TXT_FFW_HIGH : natural := 639;
  constant TXT_FFW_LOW  : natural := 608;

  constant TXT_IDW_HIGH : natural := 607;
  constant TXT_IDW_LOW  : natural := 576;
  
  constant TXT_TSLOW_HIGH : natural := 575;
  constant TXT_TSLOW_LOW  : natural := 544;

  constant TXT_TSUPP_HIGH : natural := 543;
  constant TXT_TSUPP_LOW  : natural := 512;

  constant TXT_DATAW_HIGH : natural := 511;
  constant TXT_DATAW_LOW  : natural := 0;


  ------------------------------------------------------------------------------
  -- Interrupt vector indices
  ------------------------------------------------------------------------------
  constant BUS_ERR_INT : natural := 7;
  constant ARB_LST_INT : natural := 6;
  constant ERR_PAS_INT : natural := 5;
  constant WAKE_INT    : natural := 4;
  constant DOV_INT     : natural := 3;
  constant ERR_WAR_INT : natural := 2;
  constant TX_INT      : natural := 1;
  constant RX_INT      : natural := 0;
  constant LOG_FIN_INT : natural := 8;
  constant RX_FULL_INT : natural := 9;
  constant BRS_INT     : natural := 10;


  ------------------------------------------------------------------------------
  -- Status bus Indices
  ------------------------------------------------------------------------------
  constant STAT_OP_STATE_LOW  : natural := 0;
  constant STAT_OP_STATE_HIGH : natural := 1;

  constant STAT_PC_STATE_LOW  : natural := 2;
  constant STAT_PC_STATE_HIGH : natural := 5;

  constant STAT_ARB_LOST_INDEX : natural := 6;

  constant STAT_SET_TRANSC_INDEX : natural := 7;
  constant STAT_SET_REC_INDEX    : natural := 8;
  constant STAT_IS_IDLE_INDEX    : natural := 9;

  constant STAT_SP_CONTROL_HIGH : natural := 11;
  constant STAT_SP_CONTROL_LOW  : natural := 10;

  constant STAT_SSP_RESET_INDEX       : natural := 12;
  constant STAT_TRV_DELAY_CALIB_INDEX : natural := 13;
  constant STAT_SYNC_CONTROL_HIGH     : natural := 15;
  constant STAT_SYNC_CONTROL_LOW      : natural := 14;

  constant STAT_DATA_TX_INDEX     : natural := 16;
  constant STAT_DATA_RX_INDEX     : natural := 17;
  constant STAT_BS_ENABLE_INDEX   : natural := 18;
  constant STAT_FIXED_STUFF_INDEX : natural := 19;
  constant STAT_DATA_HALT_INDEX   : natural := 20;
  constant STAT_BS_LENGTH_HIGH    : natural := 23;
  constant STAT_BS_LENGTH_LOW     : natural := 21;

  --Error indices
  constant STAT_STUFF_ERROR_INDEX      : natural := 24;
  constant STAT_DESTUFFED_INDEX        : natural := 25;
  constant STAT_BDS_ENA_INDEX          : natural := 26;
  constant STAT_STUFF_ERRROR_ENA_INDEX : natural := 27;
  constant STAT_FIXED_DESTUFF_INDEX    : natural := 28;
  constant STAT_BDS_LENGTH_HIGH        : natural := 31;
  constant STAT_BDS_LENGTH_LOW         : natural := 29;

  --Transcieve data
  constant STAT_TRAN_IDENT_HIGH : natural := 60;
  constant STAT_TRAN_IDENT_LOW  : natural := 32;

  constant STAT_TRAN_DLC_HIGH : natural := 64;
  constant STAT_TRAN_DLC_LOW  : natural := 61;

  constant STAT_TRAN_IS_RTR_INDEX     : natural := 65;
  constant STAT_TRAN_IDENT_TYPE_INDEX : natural := 66;
  constant STAT_TRAN_FRAME_TYPE_INDEX : natural := 67;
  constant STAT_TRAN_DATA_ACK_INDEX   : natural := 68;
  constant STAT_TRAN_BRS_INDEX        : natural := 69;
  constant STAT_FRAME_STORE_INDEX     : natural := 70;

  --Error counters and error state
  constant STAT_TX_COUNTER_HIGH         : natural := 79;
  constant STAT_TX_COUNTER_LOW          : natural := 71;
  constant STAT_RX_COUNTER_HIGH         : natural := 89;
  constant STAT_RX_COUNTER_LOW          : natural := 81;
  constant STAT_ERROR_COUNTER_NORM_HIGH : natural := 272;
  constant STAT_ERROR_COUNTER_NORM_LOW  : natural := 257;
  constant STAT_ERROR_COUNTER_FD_HIGH   : natural := 288;
  constant STAT_ERROR_COUNTER_FD_LOW    : natural := 273;

  constant STAT_ERC_LOW : natural := 100;
  constant STAT_ERC_HIGH : natural := 107;

  constant STAT_ERROR_STATE_HIGH : natural := 109;
  constant STAT_ERROR_STATE_LOW  : natural := 108;

  --Error signals
  constant STAT_FORM_ERROR_INDEX          : natural := 110;
  constant STAT_CRC_ERROR_INDEX           : natural := 111;
  constant STAT_ACK_ERROR_INDEX           : natural := 112;
  constant STAT_UNKNOWN_STATE_ERROR_INDEX : natural := 113;
  constant STAT_BIT_STUFF_ERROR_INDEX     : natural := 114;
  constant STAT_FIRST_BIT_AFTER_INDEX     : natural := 115;
  constant STAT_REC_VALID_INDEX           : natural := 116;
  constant STAT_TRAN_VALID_INDEX          : natural := 117;
  constant STAT_CONST7_INDEX              : natural := 118;
  constant STAT_CONST14_INDEX             : natural := 119;
  constant STAT_TRANSM_ERROR_INDEX        : natural := 120;

  --Recieved data interface
  constant STAT_REC_IDENT_HIGH       : natural := 149;
  constant STAT_REC_IDENT_LOW        : natural := 121;
  constant STAT_REC_DLC_HIGH         : natural := 153;
  constant STAT_REC_DLC_LOW          : natural := 150;
  constant STAT_REC_IS_RTR_INDEX     : natural := 154;
  constant STAT_REC_IDENT_TYPE_INDEX : natural := 155;
  constant STAT_REC_FRAME_TYPE_INDEX : natural := 156;
  constant STAT_REC_BRS_INDEX        : natural := 157;
  constant STAT_REC_CRC_HIGH         : natural := 178;
  constant STAT_REC_CRC_LOW          : natural := 158;
  constant STAT_REC_ESI_INDEX        : natural := 179;
  constant STAT_CRC_ENA_INDEX        : natural := 180;
  constant STAT_TRAN_TRIG            : natural := 181;
  constant STAT_REC_TRIG             : natural := 182;

  --Arbitration lost capture
  constant STAT_ALC_HIGH : natural := 187;
  constant STAT_ALC_LOW  : natural := 183;

  --Bus traffic registers
  constant STAT_RX_CTR_HIGH : natural := 219;
  constant STAT_RX_CTR_LOW  : natural := 188;

  constant STAT_TX_CTR_HIGH : natural := 251;
  constant STAT_TX_CTR_LOW  : natural := 220;

  constant STAT_ERP_CHANGED_INDEX : natural := 252;
  constant STAT_EWL_REACHED_INDEX : natural := 253;
  constant STAT_ERROR_VALID_INDEX : natural := 254;

  constant STAT_ACK_RECIEVED_OUT_INDEX : natural := 255;

  constant STAT_BIT_ERROR_VALID_INDEX : natural := 256;

  constant STAT_BS_CTR_HIGH : natural := 302;
  constant STAT_BS_CTR_LOW  : natural := 300;

  constant STAT_BD_CTR_HIGH : natural := 305;
  constant STAT_BD_CTR_LOW  : natural := 303;

  constant STAT_TS_HIGH : natural := 369;
  constant STAT_TS_LOW  : natural := 306;

  ------------------------------------------------------------------------------
  -- Memory Access
  ------------------------------------------------------------------------------
  --General Purpose register
  constant GPR_COMPONENT_TYPE : std_logic_vector(3 downto 0) := x"1";

  --OutPut Multiplexor component type
  constant OUTMUX_COMPONENT_TYPE : std_logic_vector(3 downto 0) := x"2";

  --FlexRay Node
  constant FLEXRAY_COMPONENT_TYPE : std_logic_vector(3 downto 0) := x"3";

  --CAN Node
  constant CAN_COMPONENT_TYPE : std_logic_vector(3 downto 0) := x"4";

  --LIN Node
  constant LIN_COMPONENT_TYPE : std_logic_vector(3 downto 0) := x"5";

  constant ACT_CSC : std_logic := '1';
  constant ACT_SRD : std_logic := '1';
  constant ACT_SWR : std_logic := '1';

  --Address ranges for component type and identifier
  constant COMP_TYPE_ADRESS_HIGHER : natural := 23;
  constant COMP_TYPE_ADRESS_LOWER  : natural := 20;
  constant ID_ADRESS_HIGHER        : natural := 19;
  constant ID_ADRESS_LOWER         : natural := 16;

  constant CAN_DEVICE_ID : std_logic_vector(31 downto 0) := x"0000CAFD";
  
end package;
