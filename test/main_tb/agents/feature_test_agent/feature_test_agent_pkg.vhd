--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2021-present Ondrej Ille
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- educational, research, evaluation, self-interest purposes. Using the
-- Component for commercial purposes is forbidden unless previously agreed with
-- Copyright holder.
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
-- -------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2015-2020 MIT License
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
--  Purpose:
--    Main test package for CTU CAN FD controller. Contains all test resources
--    for running CTU CAN FD test-bench and low-level access functions.
--
--    Note that in this library several types are (nearly) the same as types
--    in synthesizable code! This is done on purpose, to avoid using wrongly
--    defined values. These types are defined from documentation manually.
--
--------------------------------------------------------------------------------
-- Revision History:
--    27.5.2016   Created file
--    13.1.2017   Added formatting of identifier in CAN_send_frame,
--                CAN_read_frame to fit the native decimal interpretation
--                (the same way as in C driver)
--    27.11.2017  Added "reset_test" function fix. Implemented reset synchroniser
--                to avoid async reset in the core. As consequnce after the core
--                reset is released, the core has to wait at least TWO clock
--                cycles till the reset is synchronised and deasserted.
--    06.02.2018  Modified the library to work with generated constants from the
--                8 bit register map generated from IP-XACT.
--    09.02.2018  Added support fow RWCNT field in the SW_CAN_Frame.
--    15.02.2018  Added support for TXT Buffer commands in CAN Send frame
--                procedure.
--    23.02.2018  Corrected "CAN_generate_frame" function for proper placement
--                of BASE identifier to unsigned value.
--     28.4.2018  Converted TXT Buffer access functions to use generated macros.
--      1.5.2018  1. Added HAL layer types and functions.
--                2. Added Byte enable support to memory access functions.
--      7.6.2018  Added "CAN_insert_TX_frame" procedure.
--     18.6.2018  Added optimized clock_gen_proc, timestamp_gen_proc procedures.
--     15.9.2018  Added support for message filter manipulation!
--     27.9.2018  Added burst support for avalon access. Added option to read
--                frame from RX Buffer via burst partially!
--    19.11.2019  Added options to force transmitter delay and timestamp in
--                feature tests.
--     09.3.2021  Port to main test-bench, remove unit test stuff (kept in
--                original CanTestLib library). Use agents instead of direct
--                passing of memory buses!
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;
context ctu_can_fd_tb.rtl_context;

use ctu_can_fd_tb.mem_bus_agent_pkg.all;
use ctu_can_fd_tb.interrupt_agent_pkg.all;
use ctu_can_fd_tb.timestamp_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;
use ctu_can_fd_tb.test_probe_agent_pkg.all;


package feature_test_agent_pkg is

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Communication constants
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- Supported commands
    constant FEATURE_TEST_AGNT_FORCE_BUS                : integer := 0;
    constant FEATURE_TEST_AGNT_RELEASE_BUS              : integer := 1;
    constant FEATURE_TEST_AGNT_FORCE_CAN_RX             : integer := 2;
    constant FEATURE_TEST_AGNT_RELEASE_CAN_RX           : integer := 3;
    constant FEATURE_TEST_AGNT_SET_TRV_DELAY            : integer := 4;
    constant FEATURE_TEST_AGNT_CHECK_BUS_LEVEL          : integer := 5;
    constant FEATURE_TEST_AGNT_CHECK_CAN_TX             : integer := 6;
    constant FEATURE_TEST_AGNT_GET_CAN_TX               : integer := 7;
    constant FEATURE_TEST_AGNT_GET_CAN_RX               : integer := 8;
    
    -- Tag for messages
    constant FEATURE_TEST_AGENT_TAG : string := "Feature test Agent: ";


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Types
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    -- Core common types for register map. Implemented to create HAL like
    -- abstraction and allow easier modifications of register map without
    -- touching the test code!
    ----------------------------------------------------------------------------

    -- Controller modes
    type SW_mode is record
        reset                   :   boolean;
        bus_monitoring          :   boolean;
        test                    :   boolean;
        self_test               :   boolean;
        acceptance_filter       :   boolean;
        flexible_data_rate      :   boolean;
        rtr_pref                :   boolean;
        acknowledge_forbidden   :   boolean;
        internal_loopback       :   boolean;
        iso_fd_support          :   boolean;
        pex_support             :   boolean;
        fdrf                    :   boolean;
        restricted_operation    :   boolean;
        tx_buf_bus_off_failed   :   boolean;
    end record;
    
    constant SW_mode_rst_val : SW_mode := (false, false, false, false, false,
        true, false, false, false, true, false, false, false, true);

    -- Controller commands
    type SW_command is record
        release_rec_buffer      :   boolean;
        clear_data_overrun      :   boolean;
        err_ctrs_rst            :   boolean;
        rx_frame_ctr_rst        :   boolean;
        tx_frame_ctr_rst        :   boolean;
        clear_pexs_flag         :   boolean;
    end record;

    constant SW_command_rst_val : SW_command :=
        (false, false, false, false, false, false);

    -- Controller status
    type SW_status is record
        receive_buffer          :   boolean;
        data_overrun            :   boolean;
        tx_buffer_empty         :   boolean;
        error_transmission      :   boolean;
        receiver                :   boolean;
        transmitter             :   boolean;
        error_warning           :   boolean;
        bus_status              :   boolean;
        protocol_exception      :   boolean;
    end record;


    -- Interrupt sources
    type SW_interrupts is record
        receive_int             :   boolean;
        transmitt_int           :   boolean;
        error_warning_int       :   boolean;
        data_overrun_int        :   boolean;
        fcs_changed_int         :   boolean;
        arb_lost_int            :   boolean;
        bus_error_int           :   boolean;
        logger_finished_int     :   boolean;
        rx_buffer_full_int      :   boolean;
        bit_rate_shift_int      :   boolean;
        rx_buffer_not_empty_int :   boolean;
        tx_buffer_hw_cmd        :   boolean;
        overload_frame          :   boolean;
    end record;
    
    constant SW_interrupts_rst_val : SW_interrupts := (
        false, false, false, false, false, false, false, false,
        false, false, false, false, false);

    -- Fault confinement states
    type SW_fault_state is (
        fc_error_active,
        fc_error_passive,
        fc_bus_off
    );

    -- Fault confinement state thresholds
    type SW_fault_thresholds is record
        ewl                     :   natural range 0 to 255;
        erp                     :   natural range 0 to 255;
    end record;

    -- Error counters (Normal and Special)
    type SW_error_counters is record
        rx_counter              :   natural range 0 to 2 ** 9 - 1;
        tx_counter              :   natural range 0 to 2 ** 9 - 1;
        err_norm                :   natural range 0 to 2 ** 16 - 1;
        err_fd                  :   natural range 0 to 2 ** 16 - 1;
    end record;

    -- Traffic counters
    type SW_traffic_counters is record
        rx_frames               :   natural;
        tx_frames               :   natural;
    end record;

    -- RX Buffer info and status
    type SW_RX_Buffer_info is record
        rx_buff_size            :   natural range 0 to 2 ** 13 - 1;
        rx_mem_free             :   natural range 0 to 2 ** 13 - 1;
        rx_write_pointer        :   natural range 0 to 2 ** 13 - 1;
        rx_read_pointer         :   natural range 0 to 2 ** 13 - 1;
        rx_full                 :   boolean;
        rx_empty                :   boolean;
        rx_mof                  :   boolean;
        rx_frame_count          :   natural range 0 to 2 ** 11 - 1;
    end record;

    -- RX Buffer options
    type SW_RX_Buffer_options is record
        rx_time_stamp_options   :   boolean;
    end record;

    -- Error code capture
    type SW_error_type is (
        can_err_bit,
        can_err_form,
        can_err_ack,
        can_err_crc,
        can_err_stuff        
    );
    
    type SW_error_position is (
        err_pos_sof,
        err_pos_arbitration,
        err_pos_ctrl,
        err_pos_data,
        err_pos_crc,
        err_pos_ack,
        err_pos_eof,
        err_pos_err_frame,
        err_pos_overload_frame,
        err_pos_other
    );
    
    -- Error code capture data
    type SW_error_capture is record
        err_pos         : SW_error_position;
        err_type        : SW_error_type;
    end record;

    -- SSP (Secondary Sampling Point) configuration options
    type SSP_set_command_type is (
        ssp_meas_n_offset,
        ssp_no_ssp,
        ssp_offset
    );

    -- Protocol control Debug values
    type SW_PC_Debug is (
        pc_deb_none,
        pc_deb_sof,
        pc_deb_arbitration,
        pc_deb_control,
        pc_deb_data,
        pc_deb_stuff_count,
        pc_deb_crc,
        pc_deb_crc_delim,
        pc_deb_ack,
        pc_deb_ack_delim,
        pc_deb_eof,
        pc_deb_intermission,
        pc_deb_suspend,
        pc_deb_overload
    );

    -- TXT Buffer state (used in test access, not in synthesizable code)
    type SW_TXT_Buffer_state_type is (
        buf_not_exist,
        buf_empty,
        buf_ready,
        buf_tx_progress,
        buf_ab_progress,
        buf_aborted,
        buf_failed,
        buf_done
    );

    -- TXT Buffer commands (used in test access, not synthesizable code)
    type SW_TXT_Buffer_command_type is (
        buf_set_empty,
        buf_set_ready,
        buf_set_abort
    );
    
    -- TXT Buffer index type (assume highest available number of TXT buffers)
    subtype SW_TXT_index_type is natural range 1 to 8;

    ----------------------------------------------------------------------------
    -- Main Bus timing configuration type used in feature and sanity tests
    -- (using "naturals" instead of std_logic_vector)
    ----------------------------------------------------------------------------
    type bit_time_config_type is record
         tq_nbt                 :   natural;
         tq_dbt                 :   natural;
         prop_nbt               :   natural;
         ph1_nbt                :   natural;
         ph2_nbt                :   natural;
         sjw_nbt                :   natural;
         prop_dbt               :   natural;
         ph1_dbt                :   natural;
         ph2_dbt                :   natural;
         sjw_dbt                :   natural;
    end record;


    type SW_CAN_data_type is array (0 to 63) of std_logic_vector(7 downto 0);

    ----------------------------------------------------------------------------
    -- Software CAN Frame type. Used for generation, transmission, reception,
    -- comparison of CAN Frames.
    ----------------------------------------------------------------------------
    type SW_CAN_frame_type is record

        -- CAN Identifier. Decimal value. Note that the value differs for
        -- BASE and EXTENDED Identifiers!
        identifier              :   natural;

        -- Data payload
        data                    :   SW_CAN_data_type;

        -- Data length code as defined in CAN Standard
        dlc                     :   std_logic_vector(3 downto 0);

        -- Data length in bytes
        data_length             :   natural range 0 to 64;

        -- Identifier type (0 - BASE Format, 1 - Extended Format);
        ident_type              :   std_logic;

        -- Frame type (0 - Normal CAN, 1 - CAN FD)
        frame_format            :   std_logic;

        -- RTR Flag (0 - No RTR Frame, 1 - RTR Frame)
        rtr                     :   std_logic;

        -- Bit rate shift flag
        brs                     :   std_logic;

        -- ESI Flag (Error state indicator)
        esi                     :   std_logic;

        -- Timestamp (as defined in TIMESTAMP_U_W and TIMESTAMP_L_W)
        timestamp               :   std_logic_vector(63 downto 0);

        -- Receive word count field as stored in RX Buffer FRAME_FORM_W.
        -- Indicates number of 32 bit words which Frame ocupies in RX Buffer
        -- without FRAME_FORM_W.
        -- Note that this value is valid only for received frames and has
        -- no meaning in TXT Buffer.
        rwcnt                   :   natural;
    end record;


    type SW_CAN_mask_filter_type is (
        filter_A,
        filter_B,
        filter_C
    );


    type SW_CAN_mask_filter_config is record
        ID_value                :   natural;
        ID_mask                 :   natural;
        ident_type              :   std_logic;
        acc_CAN_2_0             :   boolean;
        acc_CAN_FD              :   boolean;
    end record;


    type SW_CAN_range_filter_config is record
        ID_th_low               :   natural;
        ID_th_high              :   natural;
        ident_type              :   std_logic;
        acc_CAN_2_0             :   boolean;
        acc_CAN_FD              :   boolean;
    end record;

    
    type t_feature_node is(
        DUT_NODE,
        TEST_NODE
    );
    
    type t_tgt_test_mem is(
        TST_TGT_RX_BUF,
        TST_TGT_TXT_BUF_1,
        TST_TGT_TXT_BUF_2,
        TST_TGT_TXT_BUF_3,
        TST_TGT_TXT_BUF_4,
        TST_TGT_TXT_BUF_5,
        TST_TGT_TXT_BUF_6,
        TST_TGT_TXT_BUF_7,
        TST_TGT_TXT_BUF_8
    );


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Functions declarations
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------


    ----------------------------------------------------------------------------
    -- Decode data length code from value as defined in CAN FD Standard to
    -- length of frame in bytes.
    --
    -- Arguments:
    --  dlc             Data length code as received or transmitted.
    --  length          Length of CAN Frame in bytes
    ----------------------------------------------------------------------------
    procedure decode_dlc(
        constant dlc            : in    std_logic_vector(3 downto 0);
        variable length         : out   natural
    );


    ----------------------------------------------------------------------------
    -- Decode Read word count value as present in FRAME_FORMAT_W of RX Buffer
    -- from DLC. Read word count value indicates how many 32-bit words will the
    -- buffer occupy in RX Buffer without Frame Format word.
    --
    -- Arguments:
    --  dlc             Data length code to decode as transmitted or received.
    --  rwcnt           Read word count value.
    ----------------------------------------------------------------------------
    procedure decode_dlc_rx_buff(
        constant dlc            : in    std_logic_vector(3 downto 0);
        variable rwcnt          : out   natural
    );


	----------------------------------------------------------------------------
    -- Decode length do DLC code.
    --
    -- Arguments:
    --  length          Data lenght in bytes.
    --  dlc				Variable where output Data lenght code  will be stored.
    ----------------------------------------------------------------------------
	procedure decode_length(
		constant length			: in	natural;
		variable dlc			: out	std_logic_vector(3 downto 0)
	);


    ----------------------------------------------------------------------------
    -- Decode number of 32-bit words CAN Frame will occupy in RX Buffer
    -- (together with Frame format word).
    --
    -- Arguments:
    --  dlc             Data length code to decode as transmitted or received.
    --  buff_space      Number of 32-bit words.
    ----------------------------------------------------------------------------
    procedure decode_dlc_buff(
        constant dlc            : in    std_logic_vector(3 downto 0);
        variable buff_space     : out   natural
    );


    ----------------------------------------------------------------------------
    -- Convert identifier from register format (as stored in IDENTIFIER_W of
    --  TXT Buffers and RX Buffer) to integer value as used by SW.
    --
    -- Arguments:
    --  identifier      Input identifier as stored in IDENTIFIER_W
    --  id_type         Type of identifier (BASE or EXTENDED)
    --  out             Identifier in integer format
    ----------------------------------------------------------------------------
    procedure id_hw_to_sw(
        constant id_in          : in    std_logic_vector(28 downto 0);
        constant id_type        : in    std_logic;
        variable id_out         : out   natural
    );


    ----------------------------------------------------------------------------
    -- Convert identifier from SW format to register format (as stored in
    --  IDENTIFIER_W of TXT Buffers and RX Buffer).
    --
    -- Arguments:
    --  identifier      Input identifier in integer format (as used by SW).
    --  id_type         Type of identifier (BASE or EXTENDED)
    --  out             Identifier in register format as stored in IDENTIFIER_W.
    ----------------------------------------------------------------------------
    procedure id_sw_to_hw(
        constant id_in          : in    natural;
        constant id_type        : in    std_logic;
        variable id_out         : out   std_logic_vector(28 downto 0)
    );

    ---------------------------------------------------------------------------
    -- Force bus level to given value. Applicable only in feature tests!
    --
    -- Arguments:
    --  bus_val     Value to be forced
    --  channel     Communication channel
    ---------------------------------------------------------------------------
    procedure force_bus_level(
        constant value                  : in    std_logic;
        signal   channel                : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Release bus level. Applicable only in feature tests.
    --
    -- Arguments:
    --  channel     Communication channel
    ---------------------------------------------------------------------------
    procedure release_bus_level(
        signal channel                  : inout t_com_channel              
    );

    
    ---------------------------------------------------------------------------
    -- Check bus level to be equal to a value.
    --
    -- Arguments:
    --  value       Expected value of bus
    --  channel     Communication channel
    ---------------------------------------------------------------------------
    procedure check_bus_level(
        constant value                    : in    std_logic;
        constant msg                      : in    string;
        signal   channel                  : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Force CAN RX of single controller to given value. This can be used when
    -- only RX value of single node shall be forced to different value
    -- 
    -- Applicable only in feature tests!
    --
    -- Arguments:
    --  value       Value to be forced
    --  node        Node on whose RX to force the value!
    --  channel     Communication channel
    ---------------------------------------------------------------------------
    procedure force_can_rx(
        constant value           : in    std_logic;
        constant node            : in    t_feature_node;
        signal   channel         : inout t_com_channel    
    );
    
    ---------------------------------------------------------------------------
    -- Release CAN_RX value. Applicable only in feature tests!
    --
    -- Arguments:
    --  channel     Communication channel
    ---------------------------------------------------------------------------
    procedure release_can_rx(
        signal   channel         : inout t_com_channel  
    );

    
    ---------------------------------------------------------------------------
    -- Checks value send on CAN_TX by a node.
    --
    -- Arguments:
    --  value       Expected value to be sent
    --  node        Node whose CAN_TX value ot check
    --  msg         Message to be printed
    --  channel     Communication channel
    ---------------------------------------------------------------------------
    procedure check_can_tx(
        constant value              : in    std_logic;
        constant node               : in    t_feature_node;
        constant msg                : in    string;
        signal   channel            : inout t_com_channel
    );
    
    ---------------------------------------------------------------------------
    -- Reads value send on CAN_TX by a node.
    --
    -- Arguments:
    --  node        Node whose CAN_TX value ot check
    --  channel     Communication channel
    --  value       Read value.
    ---------------------------------------------------------------------------
    procedure get_can_tx(
        constant node               : in    t_feature_node;
        variable value              : out   std_logic;
        signal   channel            : inout t_com_channel
    );
    
    ---------------------------------------------------------------------------
    -- Reads value received on CAN_RX by a node.
    --
    -- Arguments:
    --  node        Node whose CAN_TX value ot check
    --  channel     Communication channel
    --  value       Read value.
    ---------------------------------------------------------------------------
    procedure get_can_rx(
        constant node               : in    t_feature_node;
        variable value              : out   std_logic;
        signal   channel            : inout t_com_channel
    );

    ----------------------------------------------------------------------------
    -- Memory access routines
    ----------------------------------------------------------------------------


    ----------------------------------------------------------------------------
    -- Execute write access to CTU CAN FD Core (via memory bus agent). If size
    -- is not specified, 32 bit access is executed. If input data size is higher
    -- than 32 bits, burst access is executed.
    --
    -- Address bits meaning is following:
    --  [15:12]     Identifier (Index) of core. Allows to distinguish between
    --              up to 16 instances of CTU CAN FD Core.
    --  [11:0]      Register or Buffer offset within a the core.
    --
    -- Arguments:
    --  w_data          Data to write to CTU CAN FD Core.
    --  w_offset        Register or buffer offset (bits 11:0).
    --  node            Node which shall be accessed (Test node or DUT)
    --  stat_burst      If Burst access is executed, address should not be
    --                  incremented during the burst.
    --
    -- Note: Size of write access is given by size of write data !!
    --       It should be 8,16,32 or multiple of 32 for bursts 
    ----------------------------------------------------------------------------
    procedure CAN_write(
        constant  w_data        : in    std_logic_vector;
        constant  w_offset      : in    std_logic_vector(11 downto 0);
        constant  node          : in    t_feature_node;
        signal    channel       : inout t_com_channel;
        constant  stat_burst    : in    boolean := false
    );


    ----------------------------------------------------------------------------
    -- Execute read access from CTU CAN FD Core via memory bsu agent. If size is
    -- not specified, 32 bit access is executed. If input data size is bigger
    -- than 32 bits, burst access is executed.
    --
    -- Address bits meaning is following:
    --  [15:12]     Identifier (Index) of core. Allows to distinguish between
    --              up to 16 instances of CTU CAN FD Core.
    --  [11:0]      Register or Buffer offset within a the core.
    --
    -- Arguments:
    --  r_data          Variable in which Read data will be returned.
    --  r_offset        Register or buffer offset (bits 11:0).
    --  node            Node which shall be accessed (Test node or DUT)
    --  stat_burst      If Burst access is executed, address should not be
    --                  incremented during the burst.
    --
    -- Note: Size of write access is given by size of write data !!
    --       It should be 8,16,32 or multiple of 32 for bursts
    ----------------------------------------------------------------------------
    procedure CAN_read(
        variable  r_data        : out   std_logic_vector;
        constant  r_offset      : in    std_logic_vector(11 downto 0);
        constant  node          : in    t_feature_node;
        signal    channel       : inout t_com_channel;
        constant  stat_burst    : in    boolean := false
    );


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- CAN feauture TB configuration routines
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    
    ----------------------------------------------------------------------------
    -- Configure transmitter delay. Valid only in feature tests.
    --
    -- Arguments:
    --  tx_del          Delay to be set
    --  node            Node which shall be accessed (Test node or DUT).
    --  channel         Communication channel
    ----------------------------------------------------------------------------
    procedure ftr_tb_set_tran_delay(
        constant tx_del         : in    time;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );

    ----------------------------------------------------------------------------
    -- Configure timestamp in feature TB on inputs of DUT Node. Test node has
    -- timestamp tied.
    --
    -- Arguments:
    --  ts_value        Value to be forced to timestamp.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure ftr_tb_set_timestamp(
        constant ts_value       : in    std_logic_vector(63 downto 0);
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- CAN configuration routines
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    -- Configure Bus timing on CTU CAN FD Core.
    -- (duration of bit phases, synchronisation jump width, baud-rate prescaler)
    --
    -- Arguments:
    --  bus_timing      Bus timing structure that contains timing configuration
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_configure_timing(
        constant bus_timing     : in    bit_time_config_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read Bus timing configuration from CTU CAN FD Core.
    -- (duration of bit phases, synchronisation jump width, baud-rate prescaler)
    --
    -- Arguments:
    --  bus_timing      Bus timing structure that will be filled by timing
    --                  configuration.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_read_timing(
        signal   bus_timing     : out   bit_time_config_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );

    ----------------------------------------------------------------------------
    -- Read Bus timing configuration from CTU CAN FD Core.
    -- (duration of bit phases, synchronisation jump width, baud-rate prescaler)
    --
    -- Arguments:
    --  bus_timing      Bus timing structure that will be filled by timing
    --                  configuration.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_read_timing_v(
        variable bus_timing     : out   bit_time_config_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Print Bus timing configuration of CTU CAN FD Core.
    -- (duration of bit phases, synchronisation jump width, baud-rate prescaler)
    --
    -- Arguments:
    --  bus_timing      Bus timing structure that will be printed in simulator.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_print_timing(
        constant   bus_timing     : in   bit_time_config_type
    );


    ----------------------------------------------------------------------------
    -- Turn on/off CTU_CAN_FD Core.
    --
    -- Arguments:
    --  turn_on         Turns on the Core when "true". Turns off otherwise.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_turn_controller(
        constant turn_on        : in    boolean;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Enables/Disabled Retransmitt limiting in CTU CAN FD Core.
    --
    -- Arguments:
    --  enable          Enables retransmitt limiting when "true".
    --                  Disables rettransmitt limiting otherwise.
    --  limit           Limit for number of retransmissions.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_enable_retr_limit(
        constant enable         : in    boolean;
        constant limit          : in    natural range 0 to 15;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Configures mask filter of CTU CAN FD Core.
    --
    -- Arguments:
    --  filter          Identifier of Mask filter which should be configured.
    --  config          Configuration structure of CTU CAN FD mask filter.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_set_mask_filter(
        constant filter         : in    SW_CAN_mask_filter_type;
        constant config         : in    SW_CAN_mask_filter_config;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Configures mask filter of CTU CAN FD Core.
    --
    -- Arguments:
    --  config          Configuration structure of CTU CAN FD mask filter.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_set_range_filter(
        constant config         : in    SW_CAN_range_filter_config;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Generate random CAN FD Frame.
    --
    -- Arguments:
    --  frame           Output variable in which CAN FD Frame sill be generated.
    ----------------------------------------------------------------------------
    procedure CAN_generate_frame(
        variable frame          : inout SW_CAN_frame_type
    );


    ----------------------------------------------------------------------------
    -- Prints CAN Frame to simulator output
    --
    -- Arguments:
    --  frame           Frame to print
    --  severity        Severity level that should be used to print the frame.
    ----------------------------------------------------------------------------
    procedure CAN_print_frame(
        constant frame          : in    SW_CAN_frame_type
    );
    procedure CAN_print_frame_simple(
        constant frame          : in    SW_CAN_frame_type
    );


    ----------------------------------------------------------------------------
    -- Compare two CAN FD frames and decide if they are equal.
    --
    -- Arguments:
    --  frame_A         First frame in comparison
    --  frame_B         Second frame in comparison
    --  comp_ts         When "true" timestamps should be considered in
    --                  comparison. When "false" timestamps should not be
    --                  considered.
    --  outcome         Variable to store result of comparison into. When frames
    --                  are equal "true" is stored, "false" otherwise.
    ----------------------------------------------------------------------------
    procedure CAN_compare_frames(
        constant frame_A        : in    SW_CAN_frame_type;
        constant frame_B        : in    SW_CAN_frame_type;
        constant comp_ts        : in    boolean;
        variable outcome        : inout boolean
    );


    ----------------------------------------------------------------------------
    -- Inserts frame to TXT Buffer. Function does NOT check state of the
    -- buffer.
    --
    -- Arguments:
    --  frame           CAN FD Frame to send
    --  buf_nr          Number of TXT Buffer from which the frame should be
    --                  sent (1:4)
    --  node            Node which shall be accessed (Test node or DUT).
    --  outcome         Returns "true" if the frame was inserted properly,
    --                  "false" if TXT Buffer was in states : Ready,
    --                  TX in progress, Abort in progress
    ----------------------------------------------------------------------------
    procedure CAN_insert_TX_frame(
        constant frame          : in    SW_CAN_frame_type;
        constant buf_nr         : in    SW_TXT_index_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Check whether TXT Buffer is accessible (Empty, Aborted, TX Failed or Done)
    -- If yes, insert the frame to TXT Buffer and give "set_ready" command.
    -- The function does not wait until the frame is transmitted.
    --
    -- Arguments:
    --  frame           CAN FD Frame to send
    --  buf_nr          Number of TXT Buffer from which the frame should be
    --                  sent (1:4)
    --  node            Node which shall be accessed (Test node or DUT).
    --  outcome         Returns "true" if the frame was inserted properly,
    --                  "false" if TXT Buffer was in states : Ready,
    --                  TX in progress, Abort in progress
    ----------------------------------------------------------------------------
    procedure CAN_send_frame(
        constant frame          : in    SW_CAN_frame_type;
        constant buf_nr         : in    SW_TXT_index_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel;
        variable outcome        : out   boolean
    );


    ----------------------------------------------------------------------------
    -- Reads CAN Frame from RX Buffer FIFO.
    --
    -- Arguments:
    --  frame           Output variable where CAN FD Frame will be stored
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_read_frame(
        variable frame          : inout SW_CAN_frame_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Waits until frame starts (arbitration field),
    -- and then waits until frame finishes (Intermission). Note that this
    -- does not wait until Idle, because bus does not need to be Idle due to
    -- two immediately consecutive frames!
    --
    -- Procedure is polling on status of CTU CAN FD Core over Avalon bus!
    --
    -- Arguments:
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_wait_frame_sent(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Waits until CAN bus becomes idle (no frame in progress).
    --
    -- Procedure is polling on status of CTU CAN FD Core over Avalon bus!
    --
    -- Arguments:
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_wait_bus_idle(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Waits until CAN bus starts transmitting error frame.
    --
    -- Procedure is polling on status of CTU CAN FD Core over Avalon bus!
    --
    -- Arguments:
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_wait_error_frame(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Reads bit timing parameters and waits for length of several bit times.
    --
    -- Arguments:
    --  bits            Number of Bit times to wait for
    --  nominal         "true" if Nominal Bit time should be used, "false" if
    --                  Data Bit Time should be used.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_wait_n_bits(
        constant bits           : in    natural;
        constant nominal        : in    boolean;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Waits until transmission or reception is started by a Node.
    --
    -- Arguments:
    --  exit_trans      Exit when unit turns transceiver.
    --  exit_rec        Exit when unit turns receiver.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_wait_tx_rx_start(
        constant exit_trans     : in    boolean;
        constant exit_rec       : in    boolean;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );

    
    ----------------------------------------------------------------------------
    -- Wait until a Node is in Error Active state! Actively polls on Fault state
    -- register. Can be used after enabling CAN node to wait till integration
    -- field is over!
    --
    -- Arguments:
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_wait_bus_on(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Calculate length of CAN Frame in bits (stuff bits not included).
    --
    -- Arguments:
    --  frame           CAN Frame whose length should be calculated.
    --  bit_length      Variable in which the length of CAN Frame in bits is
    --                  stored.
    ----------------------------------------------------------------------------
    procedure CAN_calc_frame_length(
        constant frame          : in    SW_CAN_frame_type;
        variable bit_length     : inout natural
    );


    function CAN_add_unsigned(
        operator1               : in    std_logic_vector(11 downto 0);
        operator2               : in    std_logic_vector(11 downto 0)
    ) return std_logic_vector;


    ----------------------------------------------------------------------------
    -- Give command to TXT Buffer.
    --
    -- Arguments:
    --  cmd             Command to give to TXT Buffer.
    --  buf_index       Number of TXT Buffer which should receive the command.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure send_TXT_buf_cmd(
        constant cmd            : in    SW_TXT_Buffer_command_type;
        constant buf_n          : in    SW_TXT_index_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Give command to selected TXT Buffers in one bus access.
    --
    -- Arguments:
    --  cmd             Command to give to TXT Buffer.
    --  buf_vector      Bit vector with TXT Buffers which should receive 
    --                  the command (eg. "00001001" = command for buffers 1 and 4.)
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure send_TXT_buf_cmd(
        constant cmd            : in    SW_TXT_Buffer_command_type;
        constant buf_vector     : in    std_logic_vector(7 downto 0);  
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );

    ----------------------------------------------------------------------------
    -- Read state of TXT Buffer.
    --
    -- Arguments:
    --  buf_index       TXT Buffer number.
    --  retVal          Variable in which return state of the buffer will be
    --                  returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure get_tx_buf_state(
        constant buf_n          : in    SW_TXT_index_type;
        variable retVal         : out   SW_TXT_Buffer_state_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );

    ----------------------------------------------------------------------------
    -- Get number of txt buffers present in CTU CAN FD.
    --
    -- Arguments:
    --  num_buffers     Number of available TXT buffers.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure get_tx_buf_count(
        variable num_buffers    : out   natural range 1 to 8;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );
    
    
    ----------------------------------------------------------------------------
    -- Pick random TXT buffer (checks number of available buffers).
    --
    -- Arguments:
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure pick_random_txt_buffer(
        variable txt_buf        : out   SW_TXT_index_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read state of RX Buffer.
    --
    -- Arguments:
    --  retVal          Variable in which return state of the buffer will be
    --                  returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure get_rx_buf_state(
        variable retVal         : out   SW_RX_Buffer_info;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Set options of RX Buffer.
    --
    -- Arguments:
    --  options         Options to be applied on RX Buffer.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure set_rx_buf_options(
        constant options        : in    SW_RX_Buffer_options;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read version register and return the actual version of the core like so:
    --  MAJOR_VERSION * 10 + MINOR_VERSION.
    --
    -- Arguments:
    --  retVal          Variable in which return version will be returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure get_core_version(
        variable retVal         : out   natural;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Sets mode of CTU CAN FD Core.
    --
    -- Arguments:
    --  mode            Mode to set.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure set_core_mode(
        constant mode           : in    SW_mode;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Reads mode from CTU CAN FD Core.
    --
    -- Arguments:
    --  mode            Variable to which returned mode will be stored.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure get_core_mode(
        variable mode           : out   SW_mode;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Send command to execute SW reset
    --
    -- Arguments:
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure exec_SW_reset(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Send arbitrary command to the controller.
    --
    -- Arguments:
    --  command         Command to send to the controller.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure give_controller_command(
        constant command        : in    SW_command;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read status of CTU CAN FD controller.
    --
    -- Arguments:
    --  status          Variable in which status of the Core will be returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure get_controller_status(
        variable status         : out   SW_status;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read captured interrupt vector (status). "true" indicates interrupt
    -- occurred.
    --
    -- Arguments:
    --  interrupts      Variable in which Interrupt vector will be returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure read_int_status(
        variable interrupts     : out   SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Clear captured interrupt vector (status). "true" indicates interrupt
    -- should be cleared.
    --
    -- Arguments:
    --  interrupts      Interrupts which should be cleared.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure clear_int_status(
        constant interrupts     : in    SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read interrupt enable vector. "true" indicates interrupt
    -- is enabled for capturing.
    --
    -- Arguments:
    --  interrupts      Variable in which interrupt enable vector will be
    --                  returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure read_int_enable(
        variable interrupts     : out   SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Write interrupt enable vector (status). "true" indicates interrupt
    -- will be enabled for capturing, "false" indicates interrupt will be
    -- disabled for capturing.
    --
    -- Arguments:
    --  interrupts      Variable in which status of the Core will be returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure write_int_enable(
        constant interrupts     : in    SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read interrupt mask. "true" indicates interrupt is masked, thus it does
    -- not affect "int" output of CTU CAN FD Core.
    --
    -- Arguments:
    --  interrupts      Variable in which interrupt mask will be returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure read_int_mask(
        variable interrupts     : out   SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Write interrupt mask. "true" indicates interrupt is masked, thus it does
    -- not affect "int" output of CTU CAN FD Core.
    --
    -- Arguments:
    --  interrupts      Interrupt mask to write.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure write_int_mask(
        constant interrupts     : in    SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read fault confinement state of CTU CAN FD Core.
    --
    -- Arguments:
    --  fault_state     Variable in which fault confinement state will be
    --                  returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure get_fault_state(
        variable fault_state    : out   SW_fault_state;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Set fault confinement thresholds for Error warning limit and for
    -- Error passive.
    --
    -- Arguments:
    --  fault_th        Variable with fault confinement thresholds.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure set_fault_thresholds(
        constant fault_th       : in    SW_fault_thresholds;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Set fault confinement thresholds for Error warning limit and for
    -- Error passive.
    --
    -- Arguments:
    --  fault_th        Variable with fault confinement thresholds.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure get_fault_thresholds(
        variable fault_th       : out   SW_fault_thresholds;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read Error counters from CTU CAN FD Core.
    --
    -- Arguments:
    --  err_counters    Variable in which error counters will be returned.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure read_error_counters(
        variable err_counters   : out   SW_error_counters;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );
    
    ----------------------------------------------------------------------------
    -- Configure test memory access in CTU CAN FD Core.
    -- Note: To use this function, Test mode must be enabled (MODE[TSTM])
    --
    -- Arguments:
    --  enable          True - Enable test memory access.
    --                  False - Disable test memory access.
    --  node            Target node (DUT or Test node)
    --  channel         Communication channel
    ----------------------------------------------------------------------------
    procedure set_test_mem_access(
        constant enable         : in    boolean;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );

    ----------------------------------------------------------------------------
    -- Execute Write test access via Test registers to target memory
    -- Note: Test memory access must be enabled
    --
    -- Arguments:
    --  data            Write data
    --  address         Address to write into
    --  tgt_mem         Target memory
    --  node            Target node (DUT or Test node)
    --  channel         Communication channel
    ----------------------------------------------------------------------------
    procedure test_mem_write(
        constant data           : in    std_logic_vector(31 downto 0);
        constant address        : in    natural;
        constant tgt_mem        : in    t_tgt_test_mem;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );
    
    ----------------------------------------------------------------------------
    -- Execute Read test access via Test registers to target memory
    -- Note: Test memory access must be enabled
    --
    -- Arguments:
    --  data            Read data
    --  address         Address to write into
    --  node            Target node (DUT or Test node)
    --  channel         Communication channel
    ----------------------------------------------------------------------------
    procedure test_mem_read(
        variable data           : out   std_logic_vector(31 downto 0);
        constant address        : in    natural;
        constant tgt_mem        : in    t_tgt_test_mem;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ---------------------------------------------------------------------------
    -- Compare strings of possibly different length. The strings are considered
    -- equal if the longer one starts with the shorter one and the rest are
    -- spaces.
    --
    -- Arguments:
    --  a               First string to compare
    --  b               Second string to compare
    ---------------------------------------------------------------------------
    function str_equal(
        a : string;
        b : string
    ) return boolean;


    ---------------------------------------------------------------------------
    -- Pad string with spaces.
    ---------------------------------------------------------------------------
    impure function strtolen(
        n   : natural;
        src : string
    ) return string;


    ----------------------------------------------------------------------------
    -- Set Error counters from CTU CAN FD Core.
    --
    -- Arguments:
    --  err_counters    Variable from which error counters will be set.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure set_error_counters(
        constant err_counters   : in    SW_error_counters;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read arbitration lost capture register.
    --
    -- Arguments:
    --  alc             Bit index in which the arbitration was lost.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure read_alc(
        variable alc            : out   natural;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read traffic counters.
    --
    -- Arguments:
    --  ctr             Variable in which traffic counters will be stored
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure read_traffic_counters(
        variable ctr            : out   SW_traffic_counters;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read transceiver delay register.
    --
    -- Arguments:
    --  ctr             Variable in which traffic counters will be stored
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure read_trv_delay(
        variable trv_delay      : out   natural;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Read Timestamp from TIMESTAMP_LOW and TIMESTAMP_HIGH registers
    --
    -- Arguments:
    --  ts             	Variable in which timestamp will be stored
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_read_timestamp(
        variable ts		        : out   std_logic_vector(63 downto 0);
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    -- Configure SSP (Secondary Sampling Point) configuration: choose applicable
    -- SSP delaying source and set offest given by the user (if eventually used).
    --
    -- Arguments:
    --  ssp_source      Select required source of delaying.
    --  ssp_offset      Amount of clock cycles to wait for.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_configure_ssp(
        constant ssp_source     : in    SSP_set_command_type;
        constant ssp_offset_val : in    std_logic_vector(7 downto 0);
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );
    
    
    ----------------------------------------------------------------------------
    -- Configure priority of the TXT Buffers in TX Arbitrator. Higher priority 
    -- value signals that buffer is selected earlier for transmission. 
    -- 
    -- Arguments:
    --  buff_number     Select required buffer.
    --  priority        Value between 0 and 7, details in datasheet.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_configure_tx_priority(
        constant buff_number    : in    SW_TXT_index_type;
        constant priority       : in    natural range 0 to 7;   
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );    
    
    
    ----------------------------------------------------------------------------
    -- Read Error code capture register to determine position of last error. 
    -- 
    -- Arguments:
    --  err_capt        Information about last error on the bus
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_read_error_code_capture(
        variable err_capt       : inout SW_error_capture;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );

    
    ----------------------------------------------------------------------------
    -- Read Debug register to obtain Protocol Control Debug Information.
    --
    -- Arguments:
    --  pc_dbg          Output value.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_read_pc_debug_m(
        variable pc_dbg         : out   SW_PC_Debug;   
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );
    
    
    ----------------------------------------------------------------------------
    -- Poll on Debug register until Protocol control is in desired state.
    --
    -- Arguments:
    --  pc_dbg          State to poll on.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_wait_pc_state(
        constant pc_state       : in    SW_PC_Debug;   
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );
    
    
    ----------------------------------------------------------------------------
    -- Poll on Debug register until Protocol control is NOT in desired state.
    --
    -- Arguments:
    --  pc_dbg          State to poll on.
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_wait_not_pc_state(
        constant pc_state       : in    SW_PC_Debug;   
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );
    
    ----------------------------------------------------------------------------
    -- Read current retransmitt counter value.
    --
    -- Arguments:
    --  retr_ctr        Return value for retransmitt counter
    --  node            Node whose retransmitt counter to read
    --  channel         Communication channel to use
    ----------------------------------------------------------------------------
    procedure CAN_read_retr_ctr(
        variable retr_ctr       : out   natural;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );

    ----------------------------------------------------------------------------
    -- Wait until sample point.
    --
    -- Note: Blocks communication channel when waiting.
    --
    -- Arguments:
    --  node            Node whose sample point to wait on.
    --  skip_stuff_bits Whether stuff bits should be skipped or accounted.
    --  channel         Channel to use for communcation.
    ----------------------------------------------------------------------------
    procedure CAN_wait_sample_point(
        constant node               : in    t_feature_node;
        signal   channel            : inout t_com_channel;
        constant skip_stuff_bits    : in    boolean := true
    );

    ----------------------------------------------------------------------------
    -- Wait until start of bit (Sync Seg), (from Status Bus).
    --
    -- Arguments:
    --  pc_dbg           State to poll on.
    ----------------------------------------------------------------------------
    procedure CAN_wait_sync_seg(
        constant node               : in    t_feature_node;
        signal   channel            : inout t_com_channel
    );

    ----------------------------------------------------------------------------
    -- Initialize TXT Buffer memories
    --
    -- Arguments:
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_init_txtb_mems(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    );
    
    ----------------------------------------------------------------------------
    -- Generate random bit-rate.
    --
    -- Arguments:
    --  bt              Bit timing config
    --  node            Node which shall be accessed (Test node or DUT).
    ----------------------------------------------------------------------------
    procedure CAN_generate_random_bit_timing(
        variable bt         : inout   bit_time_config_type;
        signal   channel    : inout t_com_channel
    );


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Components declaration
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    component feature_test_agent is
    generic(
        -- Test details
        test_name               : string;
        test_type               : string;
        stand_alone_vip_mode    : boolean;
        
        -- DUT config
        cfg_sys_clk_period      : string;
        cfg_brp                 : natural;
        cfg_prop                : natural;
        cfg_ph_1                : natural;
        cfg_ph_2                : natural;
        cfg_sjw                 : natural;
        cfg_brp_fd              : natural;
        cfg_prop_fd             : natural;
        cfg_ph_1_fd             : natural;
        cfg_ph_2_fd             : natural;
        cfg_sjw_fd              : natural
    );
    port(
        -----------------------------------------------------------------------
        -- Test node connections
        -----------------------------------------------------------------------
        clk_sys         :   in  std_logic;
        res_n           :   in  std_logic;
        
        write_data      :   in  std_logic_vector(31 DOWNTO 0);
        read_data       :   out std_logic_vector(31 DOWNTO 0);
        adress          :   in  std_logic_vector(15 DOWNTO 0);
        scs             :   in  std_logic;
        srd             :   in  std_logic;
        swr             :   in  std_logic;
        sbe             :   in  std_logic_vector(3 DOWNTO 0);
        
        -- CAN bus from/to DUT
        dut_can_tx      :   in  std_logic;
        dut_can_rx      :   out std_logic;
        
        -- Test Nodes test probe output
        test_node_test_probe  : out t_ctu_can_fd_test_probe;
        test_node_scan_enable : in  std_logic
    );    
    end component;
   
end package;



--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Package implementation
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

package body feature_test_agent_pkg is


    function CAN_add_unsigned(
        operator1               : in    std_logic_vector(11 downto 0);
        operator2               : in    std_logic_vector(11 downto 0)
    ) return std_logic_vector is
    begin
        return std_logic_vector(unsigned(operator1) + unsigned(operator2));
    end function;


	procedure decode_length(
		constant length			: in	natural;
		variable dlc			: out	std_logic_vector(3 downto 0)
	) is
	begin
		case length is
        when 0 => dlc := "0000";
        when 1 => dlc := "0001";
        when 2 => dlc := "0010";
        when 3 => dlc := "0011";
        when 4 => dlc := "0100";
        when 5 => dlc := "0101";
        when 6 => dlc := "0110";
        when 7 => dlc := "0111";
        when 8 => dlc := "1000";
        when 12 => dlc := "1001";
        when 16 => dlc := "1010";
        when 20 => dlc := "1011";
        when 24 => dlc := "1100";
        when 32 => dlc := "1101";
        when 48 => dlc := "1110";
        when 64 => dlc := "1111";
        when others =>
			error_m("Invalid data length");
			dlc := "0000";
        end case;
	end procedure;


    procedure decode_dlc(
        constant dlc            : in    std_logic_vector(3 downto 0);
        variable length         : out   natural
    )is
    begin
        case dlc is
        when "0000" => length := 0;
        when "0001" => length := 1;
        when "0010" => length := 2;
        when "0011" => length := 3;
        when "0100" => length := 4;
        when "0101" => length := 5;
        when "0110" => length := 6;
        when "0111" => length := 7;
        when "1000" => length := 8;
        when "1001" => length := 12;
        when "1010" => length := 16;
        when "1011" => length := 20;
        when "1100" => length := 24;
        when "1101" => length := 32;
        when "1110" => length := 48;
        when "1111" => length := 64;
        when others => length := 0;
        end case;
    end procedure;


    procedure decode_dlc_rx_buff(
        constant dlc            : in    std_logic_vector(3 downto 0);
        variable rwcnt          : out   natural
    )is
    begin
        case dlc is
        when "0000" => rwcnt := 3;
        when "0001" => rwcnt := 4;
        when "0010" => rwcnt := 4;
        when "0011" => rwcnt := 4;
        when "0100" => rwcnt := 4;
        when "0101" => rwcnt := 5;
        when "0110" => rwcnt := 5;
        when "0111" => rwcnt := 5;
        when "1000" => rwcnt := 5;
        when "1001" => rwcnt := 6;
        when "1010" => rwcnt := 7;
        when "1011" => rwcnt := 8;
        when "1100" => rwcnt := 9;
        when "1101" => rwcnt := 11;
        when "1110" => rwcnt := 15;
        when "1111" => rwcnt := 19;
        when others => rwcnt := 0;
        end case;
    end procedure;


    procedure decode_dlc_buff(
        constant dlc            : in    std_logic_vector(3 downto 0);
        variable buff_space     : out   natural
    )is
    begin
        case dlc is
        when "0000" => buff_space := 0 + 4; --Zero bits
        when "0001" => buff_space := 1 + 4; --1 byte
        when "0010" => buff_space := 1 + 4; --2 bytes
        when "0011" => buff_space := 1 + 4; --3 bytes
        when "0100" => buff_space := 1 + 4; --4 bytes
        when "0101" => buff_space := 2 + 4; --5 bytes
        when "0110" => buff_space := 2 + 4; --6 bytes
        when "0111" => buff_space := 2 + 4; --7 bytes
        when "1000" => buff_space := 2 + 4; --8 bytes
        when "1001" => buff_space := 3 + 4; --12 bytes
        when "1010" => buff_space := 4 + 4; --16 bytes
        when "1011" => buff_space := 5 + 4; --20 bytes
        when "1100" => buff_space := 6 + 4; --24 bytes
        when "1101" => buff_space := 8 + 4; --32 bytes
        when "1110" => buff_space := 12 + 4; --48 bytes
        when "1111" => buff_space := 16 + 4; --64 bytes
        when others => buff_space := 0;
        end case;
    end procedure;


    procedure id_hw_to_sw(
        constant id_in          : in    std_logic_vector(28 downto 0);
        constant id_type        : in    std_logic;
        variable id_out         : out   natural
    )is
        variable tmp_vect       :       std_logic_vector(28 downto 0);
    begin
        if (id_type = EXTENDED) then
            tmp_vect := id_in(IDENTIFIER_BASE_H downto
                              IDENTIFIER_BASE_L) &
                        id_in(IDENTIFIER_EXT_H downto
                              IDENTIFIER_EXT_L);
            id_out   := to_integer(unsigned(tmp_vect));
        else
            tmp_vect   := "000000000000000000" &
                           id_in(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L);
            id_out     := to_integer(unsigned(tmp_vect));
        end if;
    end procedure;


    procedure id_sw_to_hw(
        constant id_in          : in    natural;
        constant id_type        : in    std_logic;
        variable id_out         : out   std_logic_vector(28 downto 0)
    )is
        variable id_vect        :       std_logic_vector(28 downto 0);
    begin
        if (id_type = EXTENDED) then
            check_m(id_in < 536870912,
                  "Extended Identifier: " & integer'image(id_in) &
                  " In range for Extended Identifier");
            id_vect := std_logic_vector(to_unsigned(id_in, 29));

            id_out(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) :=
                id_vect(28 downto 18);
            id_out(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) :=
                id_vect(17 downto 0);
        else
            check_m(id_in < 2048,
                  "Base Identifier: " & integer'image(id_in) &
                  " In range for Base Identifier");
            id_vect := "000000000000000000" &
                           std_logic_vector(to_unsigned(id_in, 11));
            id_out(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) :=
                id_vect(10 downto 0);
            id_out(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) := (OTHERS => '0');
        end if;
    end procedure;
   
    
    procedure force_bus_level(
        constant value                  : in    std_logic;
        signal   channel                : inout t_com_channel
    ) is
    begin
        info_m(FEATURE_TEST_AGENT_TAG &
             "Forcing bus level to: " & std_logic'image(value));
        com_channel_data.set_param(value);
        send(channel, C_FEATURE_TEST_AGENT_ID, FEATURE_TEST_AGNT_FORCE_BUS);
        debug_m("Bus level forced");
    end procedure;


    procedure release_bus_level(
        signal channel                  : inout t_com_channel              
    ) is
    begin
        info_m(FEATURE_TEST_AGENT_TAG & "Releasing bus level");
        send(channel, C_FEATURE_TEST_AGENT_ID, FEATURE_TEST_AGNT_RELEASE_BUS);
        debug_m("Bus level released");
    end procedure;


    procedure check_bus_level(
        constant value                    : in    std_logic;
        constant msg                      : in    string;
        signal   channel                  : inout t_com_channel
    ) is
    begin
        info_m(FEATURE_TEST_AGENT_TAG & msg);
        com_channel_data.set_param(value);
        send(channel, C_FEATURE_TEST_AGENT_ID, FEATURE_TEST_AGNT_CHECK_BUS_LEVEL);
        debug_m("Bus level checked");
    end procedure;


    procedure force_can_rx(
        constant value           : in    std_logic;
        constant node            : in    t_feature_node;
        signal   channel         : inout t_com_channel
    ) is
    begin
        info_m(FEATURE_TEST_AGENT_TAG &
             "Forcing CAN RX of: " & t_feature_node'image(node) &
             " to: " & std_logic'image(value));
        
        com_channel_data.set_param(value);
        if (node = DUT_NODE) then
            com_channel_data.set_param(0);
        else
            com_channel_data.set_param(1);
        end if;
        send(channel, C_FEATURE_TEST_AGENT_ID, FEATURE_TEST_AGNT_FORCE_CAN_RX);
        debug_m("CAN RX forced");
    end procedure;


    procedure release_can_rx(
        signal   channel         : inout t_com_channel
    ) is
    begin
        info_m(FEATURE_TEST_AGENT_TAG & "Releasing CAN RX");
        send(channel, C_FEATURE_TEST_AGENT_ID, FEATURE_TEST_AGNT_RELEASE_CAN_RX);        
        debug_m("CAN RX released");
    end procedure;


    procedure check_can_tx(
        constant value              : in    std_logic;
        constant node               : in    t_feature_node;
        constant msg                : in    string;
        signal   channel            : inout t_com_channel
    ) is
    begin
        info_m(FEATURE_TEST_AGENT_TAG & msg);
        if (node = DUT_NODE) then
            com_channel_data.set_param(0);
        else
            com_channel_data.set_param(1);
        end if;
        com_channel_data.set_param(value);
        send(channel, C_FEATURE_TEST_AGENT_ID, FEATURE_TEST_AGNT_CHECK_CAN_TX);
        debug_m("CAN TX Checked");
    end procedure;


    procedure get_can_tx(
        constant node               : in    t_feature_node;
        variable value              : out   std_logic;
        signal   channel            : inout t_com_channel
    ) is
    begin
        if (node = DUT_NODE) then
            com_channel_data.set_param(0);
        else
            com_channel_data.set_param(1);
        end if;
        com_channel_data.set_param(value);
        send(channel, C_FEATURE_TEST_AGENT_ID, FEATURE_TEST_AGNT_GET_CAN_TX);
        value := com_channel_data.get_param;
    end procedure;


    procedure get_can_rx(
        constant node               : in    t_feature_node;
        variable value              : out   std_logic;
        signal   channel            : inout t_com_channel
    ) is
    begin
        if (node = DUT_NODE) then
            com_channel_data.set_param(0);
        else
            com_channel_data.set_param(1);
        end if;
        send(channel, C_FEATURE_TEST_AGENT_ID, FEATURE_TEST_AGNT_GET_CAN_RX);
        value := com_channel_data.get_param;
    end procedure;


    procedure CAN_write(
        constant  w_data        : in    std_logic_vector;
        constant  w_offset      : in    std_logic_vector(11 downto 0);
        constant  node          : in    t_feature_node;
        signal    channel       : inout t_com_channel;
        constant  stat_burst    : in    boolean := false
    )is
    begin
       
        -- Set chip select 
        if (node = DUT_NODE) then
            mem_bus_agent_set_slave_index(channel, 0);
        elsif (node = TEST_NODE) then
            mem_bus_agent_set_slave_index(channel, 1);
        else
            error_m("Invalid slave node");
        end if;
        
        -- Memory bus agent should handle also bursts
        mem_bus_agent_write(
            channel => channel,
            address => to_integer(unsigned(w_offset)),
            write_data => w_data,
            blocking => true        
        );
    end procedure;


    procedure CAN_read(
        variable  r_data        : out   std_logic_vector;
        constant  r_offset      : in    std_logic_vector(11 downto 0);
        constant  node          : in    t_feature_node;
        signal    channel       : inout t_com_channel;
        constant  stat_burst    : in    boolean := false
    )is
    begin

        -- Set chip select
        if (node = DUT_NODE) then
            mem_bus_agent_set_slave_index(channel, 0);
        elsif (node = TEST_NODE) then
            mem_bus_agent_set_slave_index(channel, 1);
        else
            error_m("Invalid slave node");
        end if;

        mem_bus_agent_read(
            channel  => channel,
            address  => to_integer(unsigned(r_offset)),
            read_data => r_data,
            stat_burst => stat_burst
        );
    end procedure;
    
    
    procedure ftr_tb_set_tran_delay(
        constant tx_del         : in    time;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
    begin
        info_m(FEATURE_TEST_AGENT_TAG & " Setting transceiver delay");
        com_channel_data.set_param(tx_del);
        if (node = DUT_NODE) then
            com_channel_data.set_param(0);
        else
            com_channel_data.set_param(1);
        end if;
        send(channel, C_FEATURE_TEST_AGENT_ID, FEATURE_TEST_AGNT_SET_TRV_DELAY);
        debug_m(FEATURE_TEST_AGENT_TAG & " Transceiver delay set");
    end procedure;


    procedure ftr_tb_set_timestamp(
        constant ts_value       : in    std_logic_vector(63 downto 0);
        signal   channel        : inout t_com_channel
    )is
    begin
        timestamp_agent_timestamp_preset(channel, ts_value);
    end procedure;
    

    procedure CAN_configure_timing(
        constant bus_timing     : in    bit_time_config_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0) :=
                                        (OTHERS => '0');
    begin

        -- Bit timing register - Nominal
        data(BRP_H downto BRP_L) := std_logic_vector(to_unsigned(
                bus_timing.tq_nbt, BRP_H - BRP_L + 1));
        data(PROP_H downto PROP_L) := std_logic_vector(to_unsigned(
                bus_timing.prop_nbt, PROP_H - PROP_L + 1));
        data(PH1_H downto PH1_L) := std_logic_vector(to_unsigned(
                bus_timing.ph1_nbt, PH1_H - PH1_L + 1));
        data(PH2_H downto PH2_L) := std_logic_vector(to_unsigned(
                bus_timing.ph2_nbt, PH2_H - PH2_L + 1));
        data(SJW_H downto SJW_L) := std_logic_vector(to_unsigned(
                bus_timing.sjw_nbt, SJW_H - SJW_L + 1));
        CAN_write(data, BTR_ADR, node, channel);

        -- Bit timing register - Data
        data(BRP_FD_H downto BRP_FD_L) := std_logic_vector(to_unsigned(
                bus_timing.tq_dbt, BRP_FD_H - BRP_FD_L + 1));
        data(PROP_FD_H downto PROP_FD_L) := std_logic_vector(to_unsigned(
                bus_timing.prop_dbt, PROP_FD_H - PROP_FD_L + 1));
        data(PH1_FD_H downto PH1_FD_L) := std_logic_vector(to_unsigned(
                bus_timing.ph1_dbt, PH1_FD_H - PH1_FD_L + 1));
        data(PH2_FD_H downto PH2_FD_L) := std_logic_vector(to_unsigned(
                bus_timing.ph2_dbt, PH2_FD_H - PH2_FD_L + 1));
        data(SJW_FD_H downto SJW_FD_L) := std_logic_vector(to_unsigned(
                bus_timing.sjw_dbt, SJW_FD_H - SJW_FD_L + 1));
        CAN_write(data, BTR_FD_ADR, node, channel);
    end procedure;


    procedure CAN_read_timing(
        signal   bus_timing     : out   bit_time_config_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0);
    begin

        -- Bit timing register - Nominal
        CAN_read(data, BTR_ADR, node, channel);
        bus_timing.tq_nbt    <= to_integer(unsigned(data(BRP_H downto BRP_L)));
        bus_timing.prop_nbt  <= to_integer(unsigned(data(PROP_H downto PROP_L)));
        bus_timing.ph1_nbt   <= to_integer(unsigned(data(PH1_H downto PH1_L)));
        bus_timing.ph2_nbt   <= to_integer(unsigned(data(PH2_H downto PH2_L)));
        bus_timing.sjw_nbt   <= to_integer(unsigned(data(SJW_H downto SJW_L)));

        -- Bit timing register - Data
        CAN_read(data, BTR_FD_ADR, node, channel);
        bus_timing.tq_dbt    <= to_integer(unsigned(data(BRP_FD_H downto
                                                         BRP_FD_L)));
        bus_timing.prop_dbt  <= to_integer(unsigned(data(PROP_FD_H downto
                                                         PROP_FD_L)));
        bus_timing.ph1_dbt   <= to_integer(unsigned(data(PH1_FD_H downto
                                                         PH1_FD_L)));
        bus_timing.ph2_dbt   <= to_integer(unsigned(data(PH2_FD_H downto
                                                         PH2_FD_L)));
        bus_timing.sjw_dbt   <= to_integer(unsigned(data(SJW_FD_H downto
                                                         SJW_FD_L)));
    end procedure;


    procedure CAN_read_timing_v(
        variable bus_timing     : out   bit_time_config_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0);
    begin

        -- Bit timing register - Nominal
        CAN_read(data, BTR_ADR, node, channel);
        bus_timing.tq_nbt    := to_integer(unsigned(data(BRP_H downto BRP_L)));
        bus_timing.prop_nbt  := to_integer(unsigned(data(PROP_H downto PROP_L)));
        bus_timing.ph1_nbt   := to_integer(unsigned(data(PH1_H downto PH1_L)));
        bus_timing.ph2_nbt   := to_integer(unsigned(data(PH2_H downto PH2_L)));
        bus_timing.sjw_nbt   := to_integer(unsigned(data(SJW_H downto SJW_L)));

        -- Bit timing register - Data
        CAN_read(data, BTR_FD_ADR, node, channel);
        bus_timing.tq_dbt    := to_integer(unsigned(data(BRP_FD_H downto BRP_FD_L)));
        bus_timing.prop_dbt  := to_integer(unsigned(data(PROP_FD_H downto PROP_FD_L)));
        bus_timing.ph1_dbt   := to_integer(unsigned(data(PH1_FD_H downto PH1_FD_L)));
        bus_timing.ph2_dbt   := to_integer(unsigned(data(PH2_FD_H downto PH2_FD_L)));
        bus_timing.sjw_dbt   := to_integer(unsigned(data(SJW_FD_H downto SJW_FD_L)));
    end procedure;


    procedure CAN_print_timing(
        constant   bus_timing       : in    bit_time_config_type
    )is
        variable msg                :       line;
    begin
        info_m("Nominal Bit timing: " &
             "BRP:  " & integer'image(bus_timing.tq_nbt) & " " &
             "PROP: " & integer'image(bus_timing.prop_nbt) & " " &
             "PH1:  " & integer'image(bus_timing.ph1_nbt) & " " &
             "PH2:  " & integer'image(bus_timing.ph2_nbt) & " " &
             "SJW:  " & integer'image(bus_timing.sjw_nbt));

        info_m("Data Bit timing: " &
             "BRP:  " & integer'image(bus_timing.tq_dbt) &
             "PROP: " & integer'image(bus_timing.prop_dbt) &
             "PH1:  " & integer'image(bus_timing.ph1_dbt) &
             "PH2:  " & integer'image(bus_timing.ph2_dbt) &
             "SJW:  " & integer'image(bus_timing.sjw_dbt));
    end procedure;


    procedure CAN_turn_controller(
        constant turn_on        : in    boolean;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0) :=
                                        (OTHERS => '0');
        variable readback       :       std_logic_vector(31 downto 0);
    begin
        CAN_read(data, MODE_ADR, node, channel);
        if turn_on then
            data(ENA_IND) := CTU_CAN_ENABLED;
        else
            data(ENA_IND) := CTU_CAN_DISABLED;
        end if;
        CAN_write(data, MODE_ADR, node, channel);
        CAN_read(readback, MODE_ADR, node, channel);
        assert readback = data;
    end procedure;


    procedure CAN_enable_retr_limit(
        constant enable         : in    boolean;
        constant limit          : in    natural range 0 to 15;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data : std_logic_vector(15 downto 0) := (OTHERS => '0');
        variable low : natural := RTRTH_L mod 16;
        variable high : natural := RTRTH_H mod 16;
    begin
        CAN_read(data, SETTINGS_ADR, node, channel);
        if enable then
            data(RTRLE_IND mod 16) := '1';
        else
            data(RTRLE_IND mod 16) := '0';
        end if;
        data(high downto low) := std_logic_vector(to_unsigned(limit, 4));
        CAN_write(data, SETTINGS_ADR, node, channel);
    end procedure;


    procedure config_filter_frame_types(
        constant ident_type     : in    std_logic;
        constant acc_CAN_2_0    : in    boolean;
        constant acc_CAN_FD     : in    boolean;
        variable cfg            : inout std_logic_vector(3 downto 0)
    ) is
    begin
        cfg := (OTHERS => '0');
        if (ident_type = BASE) then
            if (acc_CAN_2_0) then
                cfg(0) := '1';
            else
                cfg(0) := '0';
            end if;
            if (acc_CAN_FD) then
                cfg(2) := '1';
            else
                cfg(2) := '0';
            end if;
        elsif (ident_type = EXTENDED) then
            if (acc_CAN_2_0) then
                cfg(1) := '1';
            else
                cfg(1) := '0';
            end if;
            if (acc_CAN_FD) then
                cfg(3) := '1';
            else
                cfg(3) := '0';
            end if;
        else
            error_m("Unsupported CAN frame type");
        end if;
    end procedure;


    procedure CAN_set_mask_filter(
        constant filter         : in    SW_CAN_mask_filter_type;
        constant config         : in    SW_CAN_mask_filter_config;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable mask_offset    :       std_logic_vector(11 downto 0);
        variable value_offset   :       std_logic_vector(11 downto 0);
        variable data           :       std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable ident_vect     :       std_logic_vector(28 downto 0) :=
                                            (OTHERS => '0');
        variable ctrl_offset    :       natural range 0 to 11 := 0;
        variable tmp            :       std_logic_vector(3 downto 0) :=
                                            (OTHERS => '0');
    begin

        if (filter = filter_A) then
            mask_offset  := FILTER_A_MASK_ADR;
            value_offset := FILTER_A_VAL_ADR;
            ctrl_offset  := FANB_IND;
        elsif (filter = filter_B) then
            mask_offset := FILTER_B_MASK_ADR;
            value_offset := FILTER_B_VAL_ADR;
            ctrl_offset  := FBNB_IND;
        elsif (filter = filter_C) then
            mask_offset := FILTER_C_MASK_ADR;
            value_offset := FILTER_C_VAL_ADR;
            ctrl_offset  := FCNB_IND;
        else
            error_m("Unsupported mask filter!");
        end if;

        -- Configure filter mask
        id_sw_to_hw(config.ID_mask, config.ident_type, ident_vect);
        data := "000" & ident_vect;
        CAN_write(data, mask_offset, node, channel);

        -- Configure filter value
        id_sw_to_hw(config.ID_value, config.ident_type, ident_vect);
        data := "000" & ident_vect;
        CAN_write(data, value_offset, node, channel);

        -- Configure Accepted frame types
        CAN_read(data, FILTER_CONTROL_ADR, node, channel);
        config_filter_frame_types(config.ident_type, config.acc_CAN_2_0,
                                  config.acc_CAN_FD, tmp);
        data(ctrl_offset + 3 downto ctrl_offset) := tmp;
        CAN_write(data, FILTER_CONTROL_ADR, node, channel);

    end procedure;


    procedure CAN_set_range_filter(
        constant config         : in    SW_CAN_range_filter_config;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable ident_lth_vect :       std_logic_vector(28 downto 0) :=
                                            (OTHERS => '0');
        variable ident_hth_vect :       std_logic_vector(28 downto 0) :=
                                            (OTHERS => '0');
        variable tmp            :       std_logic_vector(3 downto 0) :=
                                            (OTHERS => '0');
    begin

        -- Check High threshold aint lower than Low threshold
        if (config.ID_th_low > config.ID_th_high) then
            warning_m("High threshold of Range filter Lower than Low threshold!");
        end if;
        id_sw_to_hw(config.ID_th_high, config.ident_type, ident_hth_vect);

        -- Low threshold
        id_sw_to_hw(config.ID_th_low, config.ident_type, ident_lth_vect);
        data := "000" & ident_lth_vect;
        CAN_write(data, FILTER_RAN_LOW_ADR, node, channel);

        -- High threshold
        id_sw_to_hw(config.ID_th_high, config.ident_type, ident_hth_vect);
        data := "000" & ident_hth_vect;
        CAN_write(data, FILTER_RAN_HIGH_ADR, node, channel);

        -- Configure accepted Frame types
        CAN_read(data, FILTER_CONTROL_ADR, node, channel);
        config_filter_frame_types(config.ident_type, config.acc_CAN_2_0,
                                  config.acc_CAN_FD, tmp);
        data(FRFE_IND downto FRNB_IND) := tmp;
        CAN_write(data, FILTER_CONTROL_ADR, node, channel);

    end procedure;


    procedure CAN_generate_frame(
        variable frame          : inout SW_CAN_frame_type
    )is
        variable rand_value     : real := 0.0;
        variable aux            : std_logic_vector(28 downto 0);
        variable data_byte      : std_logic_vector(7 downto 0);
    begin

        rand_logic_v(frame.ident_type, 0.5);
        rand_logic_v(frame.frame_format, 0.5);
        rand_logic_v(frame.rtr, 0.5);
        rand_logic_v(frame.brs, 0.5);
        rand_logic_vect_v(frame.dlc, 0.3);

        rand_real_v(rand_value);

        ------------------------------------------------------------------------
        -- We generate only valid frame combinations to avoid problems...
        -- FD frames has no RTR frame, neither the RTR field!
        ------------------------------------------------------------------------
        if (frame.frame_format = FD_CAN) then
            frame.rtr := NO_RTR_FRAME;
        end if;

        ------------------------------------------------------------------------
        -- CAN 2.0 Frame has no BRS bit. Furthermore maximal data length
        -- of CAN 2.0 Frame is 8 bytes.
        ------------------------------------------------------------------------
        if (frame.frame_format = NORMAL_CAN) then
            frame.brs := BR_NO_SHIFT;

            -- Limit DLCs higher than 8 to max. 8!
            if (frame.dlc(3) = '1' and frame.dlc(2 downto 0) /= "000") then
                frame.dlc(3) := '0';
            end if;
        end if;

        -- If base identifier, the lowest bits of unsigned ID contain the
        -- basic value!
        aux := (OTHERS => '0');
        if (frame.ident_type = BASE) then
            rand_value          := rand_value * 2047.0;
            aux(10 downto 0)    := std_logic_vector(
                                   to_unsigned(integer(rand_value), 11));
        else
            rand_value          := rand_value * 536870911.0;
            aux(28 downto 0)    := std_logic_vector(
                                   to_unsigned(integer(rand_value), 29));
        end if;
        frame.identifier := to_integer(unsigned(aux));

        decode_dlc(frame.dlc, frame.data_length);
        frame.timestamp := (OTHERS => '0');

        if (frame.rtr = RTR_FRAME) then
            frame.data          := (OTHERS => (OTHERS => '0'));
            frame.dlc           := (OTHERS => '0');
            frame.data_length   := 0;
        end if;

        -- RWCNT is filled to have all information in the frame
        -- as is filled by the RX Buffer.
        decode_dlc_rx_buff(frame.dlc, frame.rwcnt);

        -- ESI is read only, but is is better to have initialized value in it!
        frame.esi := '0';

        -- Generate random data
        -- Unused bytes of data can be set to 0
        frame.data := (OTHERS => (OTHERS => '0'));
        if (frame.data_length > 0) then
            for i in 0 to frame.data_length - 1 loop
                rand_logic_vect_v(data_byte, 0.5);
                frame.data(i) := data_byte;
            end loop;
        end if;

    end procedure;


    procedure CAN_print_frame(
        constant frame          : in    SW_CAN_frame_type
    )is
        variable data_byte      :       std_logic_vector(7 downto 0);
        variable str_msg        :       string(1 to 400) := (OTHERS => ' ');
        variable str_len        :       natural := 0;
    begin

        info_m("*************************************************************");

        -- Identifier
        info_m("ID : 0x" & 
            to_hstring(std_logic_vector(to_unsigned(frame.identifier, 32))));

        -- Metadata
        info_m("DLC: " & to_hstring(frame.dlc) & ", Data length:" &
              to_string(frame.data_length));

        if (frame.rtr = RTR_FRAME) then
            info_m("RTR Frame");
        end if;

        if (frame.ident_type = BASE) then
            info_m("BASE identifier");
        else
            info_m("EXTENDED identifier");
        end if;

        if (frame.frame_format = NORMAL_CAN) then
            info_m("CAN 2.0 frame");
        else
            info_m("CAN FD frame");
        end if;

        info_m("RWCNT (read word count): " &
            to_string(std_logic_vector(to_unsigned(frame.rwcnt, 10))));

        -- Data words
        if (frame.rtr = NO_RTR_FRAME and frame.data_length > 0) then
            str_msg(1 to 6) := "Data: ";
            str_len := 6 + 5 * frame.data_length;
            for i in 0 to frame.data_length - 1 loop
                data_byte := frame.data(i);
                str_msg(7 + i * 5 to 11 + i * 5) :=
                    "0x" & to_hstring(frame.data(i)) & " ";
            end loop;
            info_m(str_msg(1 to str_len));
        end if;
        
        info_m("*************************************************************");
    end procedure;


    procedure CAN_print_frame_simple(
        constant frame          : in    SW_CAN_frame_type
    )is
        variable data_byte      :       std_logic_vector(7 downto 0);
        variable str_msg        :       string(1 to 512) := (OTHERS => ' ');
    begin

        str_msg(1 to 10) := "CAN Frame:";

        -- Identifier
        str_msg(11 to 18) := " ID : 0x";
        str_msg(19 to 26) :=
            to_hstring(std_logic_vector(to_unsigned(frame.identifier, 32)));

  
        info_m(str_msg);
    end procedure;


    procedure CAN_compare_frames(
        constant frame_A        : in    SW_CAN_frame_type;
        constant frame_B        : in    SW_CAN_frame_type;
        constant comp_ts        : in    boolean;
        variable outcome        : inout boolean
    )is
    begin
        outcome := true;

        if (frame_A.frame_format /= frame_B.frame_format) then
            info_m("Frame format (FDF) mismatch A: " &
                  std_logic'image(frame_A.frame_format) & " B: " &
                  std_logic'image(frame_B.frame_format));
            outcome := false;
        end if;

        if (frame_A.ident_type /= frame_B.ident_type) then
            info_m("Identifier type (IDE) mismatch A: " &
                  std_logic'image(frame_A.ident_type) & " B: " &
                  std_logic'image(frame_B.ident_type));
            outcome := false;
        end if;

        -- RTR should be te same only in normal CAN Frame. In FD Frame there is
        -- no RTR bit!
        if (frame_A.frame_format = NORMAL_CAN) then
            if (frame_A.rtr /= frame_B.rtr) then
                info_m("Remote transmission request (RTR) mismatch");
                outcome := false;
            end if;
        end if;

        -- BRS bit is compared only in FD frame
        if (frame_A.frame_format = FD_CAN) then
            if (frame_A.brs /= frame_B.brs) then
                info_m("Bit-rate shift (BRS) mismatch A: " &
                  std_logic'image(frame_A.brs) & " B: " &
                  std_logic'image(frame_B.brs));
                outcome := false;
            end if;
        end if;

        -- Received word count
        if (frame_A.rwcnt /= frame_B.rwcnt) then
            info_m("Read word count (RWCNT) mismatch A: " &
                  integer'image(frame_A.rwcnt) & " B: " &
                  integer'image(frame_B.rwcnt));
            outcome := false;
        end if;

        -- DLC comparison
        if (frame_A.dlc /= frame_B.dlc) then
            info_m("Data length code (DLC) mismatch A: " &
                  to_hstring(frame_A.dlc) & " B: " &
                  to_hstring(frame_B.dlc));
            outcome := false;
        end if;

        if (frame_A.identifier /= frame_B.identifier) then
            info_m("Identifier mismatch A: " & integer'image(frame_A.identifier) &
                 " B: " & integer'image(frame_B.identifier));
            outcome := false;
        end if;

        -- Compare data for NON-RTR Frames. To save time comparing frames whose
        -- metadata comparison failed, do it only for frames which are fine till
        -- now.
        if (outcome = true) then
            if ((frame_A.rtr = NO_RTR_FRAME or frame_A.frame_format = FD_CAN)
                and frame_A.data_length /= 0)
            then
                for i in 0 to (frame_A.data_length - 1) loop
                    if (frame_A.data(i) /= frame_B.data(i)) then
                        info_m("Data byte: " & integer'image(i) & " mismatch!");
                        outcome  := false;
                    end if;
                end loop;
            end if;
        end if;
    end procedure;


    procedure CAN_insert_TX_frame(
        constant frame          : in    SW_CAN_frame_type;
        constant buf_nr         : in    SW_TXT_index_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable w_data         :       std_logic_vector(31 downto 0) :=
                                        (OTHERS => '0');
        variable ident_vect     :       std_logic_vector(28 downto 0) :=
                                        (OTHERS => '0');
        variable length         :       natural;
        variable buf_offset     :       std_logic_vector(11 downto 0);
    begin
        -- Set Buffer address
        case buf_nr is
        when 1 => buf_offset := TXTB1_DATA_1_ADR;
        when 2 => buf_offset := TXTB2_DATA_1_ADR;
        when 3 => buf_offset := TXTB3_DATA_1_ADR;
        when 4 => buf_offset := TXTB4_DATA_1_ADR;
        when 5 => buf_offset := TXTB5_DATA_1_ADR;
        when 6 => buf_offset := TXTB6_DATA_1_ADR;
        when 7 => buf_offset := TXTB7_DATA_1_ADR;
        when 8 => buf_offset := TXTB8_DATA_1_ADR;     
        when others =>
            error_m("Unsupported TX buffer number");
        end case;

        -- Frame format word
        w_data                      := (OTHERS => '0');
        w_data(DLC_H downto DLC_L)  := frame.dlc;
        w_data(RTR_IND)             := frame.rtr;
        w_data(IDE_IND)             := frame.ident_type;
        w_data(FDF_IND)             := frame.frame_format;
        w_data(BRS_IND)             := frame.brs;
        w_data(ESI_RSV_IND)         := '0'; -- ESI is receive only
        CAN_write(w_data, buf_offset, node, channel);

        -- Identifier
        id_sw_to_hw(frame.identifier, frame.ident_type, ident_vect);
        w_data := "000" & ident_vect;
        CAN_write(w_data, CAN_add_unsigned(buf_offset, IDENTIFIER_W_ADR), node, channel);

        -- Timestamp
        w_data := frame.timestamp(31 downto 0);
        CAN_write(w_data, CAN_add_unsigned(buf_offset, TIMESTAMP_L_W_ADR), node, channel);
        w_data := frame.timestamp(63 downto 32);
        CAN_write(w_data, CAN_add_unsigned(buf_offset, TIMESTAMP_U_W_ADR), node, channel);

        -- Data words
        decode_dlc(frame.dlc, length);
        for i in 0 to (length - 1) / 4 loop
            w_data := frame.data((i * 4) + 3) &
                      frame.data((i * 4) + 2) &
                      frame.data((i * 4) + 1) &
                      frame.data((i * 4));
            CAN_write(w_data, std_logic_vector(unsigned(buf_offset) +
                              unsigned(DATA_1_4_W_ADR) + i * 4), node, channel);
        end loop;
    end procedure;


    procedure CAN_send_frame(
        constant frame          : in    SW_CAN_frame_type;
        constant buf_nr         : in    SW_TXT_index_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel;
        variable outcome        : out   boolean
    )is
        variable buf_state      :       SW_TXT_Buffer_state_type;
    begin
        outcome     := true;

        -- Read Status of TXT Buffer.
        get_tx_buf_state(buf_nr, buf_state, node, channel);

        -- If TXT Buffer was already locked -> Fail to insert and transmitt!
        if (buf_state = buf_tx_progress or
            buf_state = buf_ab_progress or
            buf_state = buf_ready)
        then
            error_m("Unable to send the frame, TXT buffer is READY, " &
                  "TX is in progress, or Abort is in progress");
            outcome     := false;
            return;
        end if;

        -- Insert frame to TXT Buffer
        CAN_insert_TX_frame(frame, buf_nr, node, channel);

        -- Give "Set ready" command to the buffer
        send_TXT_buf_cmd(buf_set_ready, buf_nr, node, channel);
    end procedure;





    procedure CAN_read_frame(
        variable frame          : inout SW_CAN_frame_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable r_data         :       std_logic_vector(31 downto 0) :=
                                        (OTHERS => '0');
        variable aux_vect       :       std_logic_vector(28 downto 0) :=
                                        (OTHERS => '0');
        variable burst_data     :       std_logic_vector(127 downto 0) :=
                                            (OTHERS => '0');
        variable frm_fmt_word   :       std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable ident_word     :       std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable ts_low_word    :       std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable ts_high_word   :       std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        constant burst_access   :       boolean := true;
    begin

        -- If Burst access is executed read first 4 words all at once!
        if (burst_access) then
            CAN_read(burst_data, RX_DATA_ADR, node, channel, true);
            frm_fmt_word := burst_data(31 downto 0);
            ident_word   := burst_data(63 downto 32);
            ts_low_word  := burst_data(95 downto 64);
            ts_high_word := burst_data(127 downto 96);
        else
            CAN_read(frm_fmt_word, RX_DATA_ADR, node, channel);
            CAN_read(ident_word, RX_DATA_ADR, node, channel);
            CAN_read(ts_low_word, RX_DATA_ADR, node, channel);
            CAN_read(ts_high_word, RX_DATA_ADR, node, channel);
        end if;

        -- Parse frame format word
        frame.dlc           := frm_fmt_word(DLC_H downto DLC_L);
        frame.rtr           := frm_fmt_word(RTR_IND);
        frame.ident_type    := frm_fmt_word(IDE_IND);
        frame.frame_format  := frm_fmt_word(FDF_IND);
        frame.brs           := frm_fmt_word(BRS_IND);
        frame.rwcnt         := to_integer(unsigned(
                               frm_fmt_word(RWCNT_H downto RWCNT_L)));
        decode_dlc(frame.dlc, frame.data_length);

        -- Parse ID
        aux_vect := ident_word(28 downto 0);
        id_hw_to_sw(aux_vect, frame.ident_type, frame.identifier);

        -- Timestamp
        frame.timestamp(31 downto 0)  := ts_low_word;
        frame.timestamp(63 downto 32) := ts_high_word;


        -- Now read data frames
        if ((frame.rtr = NO_RTR_FRAME or frame.frame_format = FD_CAN)
             and frame.data_length /= 0)
        then
            for i in 0 to (frame.data_length - 1) / 4 loop
                CAN_read(r_data, RX_DATA_ADR, node, channel);
                frame.data(i * 4)       := r_data(7 downto 0);
                frame.data((i * 4) + 1) := r_data(15 downto 8);
                frame.data((i * 4) + 2) := r_data(23 downto 16);
                frame.data((i * 4) + 3) := r_data(31 downto 24);
            end loop;
        end if;
    end procedure;


    procedure CAN_wait_frame_sent(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable r_data         :       std_logic_vector(31 downto 0) :=
                                        (OTHERS => '0');
    begin
        -- Wait until Base ID
        CAN_wait_pc_state(pc_deb_arbitration, node, channel);
        
        -- Wait until Intermission
        CAN_wait_pc_state(pc_deb_intermission, node, channel);
    end procedure;


    procedure CAN_wait_bus_idle(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable r_data         :       std_logic_vector(31 downto 0) :=
                                        (OTHERS => '0');
    begin
        info_m("Waiting till bus idle in node: " & t_feature_node'image(node));
        mem_bus_agent_disable_transaction_reporting(channel);
        
        CAN_read(r_data, STATUS_ADR, node, channel);
        while (r_data(IDLE_IND) = '0') loop
            CAN_read(r_data, STATUS_ADR, node, channel);
        end loop;
        
        info_m("Done");
        mem_bus_agent_enable_transaction_reporting(channel);
    end procedure;


    procedure CAN_wait_error_frame(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable r_data         :       std_logic_vector(31 downto 0) :=
                                        (OTHERS => '0');
    begin
        info_m("Waiting till error frame in node: " & t_feature_node'image(node));
        mem_bus_agent_disable_transaction_reporting(channel);
        
        -- Wait until unit starts to transmitt or recieve
        CAN_read(r_data, STATUS_ADR, node, channel);
        while (r_data(RXS_IND) = '0' and r_data(TXS_IND) = '0') loop
            CAN_read(r_data, STATUS_ADR, node, channel);
        end loop;

        -- Wait until error frame is not being transmitted
        CAN_read(r_data, STATUS_ADR, node, channel);
        while (r_data(EFT_IND) = '0') loop
            CAN_read(r_data, STATUS_ADR, node, channel);
        end loop;
        
        info_m("Done");
        mem_bus_agent_enable_transaction_reporting(channel);
    end procedure;


    procedure CAN_wait_n_bits(
        constant bits           : in    natural;
        constant nominal        : in    boolean;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable bus_timing     :       bit_time_config_type;
        variable wait_time      :       integer := 0;
    begin
        -- Read config of the node
        CAN_read_timing_v(bus_timing, node, channel);

        -- Calculate number of clock cycles to wait
        if (nominal) then
            wait_time := bus_timing.tq_nbt *
                            (bus_timing.prop_nbt + bus_timing.ph1_nbt +
                             bus_timing.ph2_nbt + 1);
        else
            wait_time := bus_timing.tq_dbt *
                            (bus_timing.prop_dbt + bus_timing.ph1_dbt +
                             bus_timing.ph2_dbt + 1);
        end if;

        -- Check Minimal Bit time
        check_m(wait_time > 6, "Calculated Bit Time shorter than minimal!");

        -- Count number of bits to wait
        -- Reading config took some time too, correct "wait_time" by 4 cycles
        wait_time := wait_time * bits;
        wait_time := wait_time - 4;

        -- Wait for calculated amount of clock cycles!
        for i in 0 to wait_time - 1 loop
            clk_agent_wait_cycle(channel);
        end loop;
    end procedure;


    procedure CAN_wait_tx_rx_start(
        constant exit_trans     : in    boolean;
        constant exit_rec       : in    boolean;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable r_data         :       std_logic_vector(31 downto 0);
    begin
        -- Wait until unit starts to transmitt or recieve
        info_m("Waiting till TX/RX of frame starts in node: " &
                t_feature_node'image(node));
        mem_bus_agent_disable_transaction_reporting(channel);
        
        while (true) loop
            CAN_read(r_data, STATUS_ADR, node, channel);
            if (exit_trans and r_data(TXS_IND) = '1') then
                exit;
            end if;
            if (exit_rec and r_data(RXS_IND) = '1') then
                exit;
            end if;
        end loop;
        
        mem_bus_agent_disable_transaction_reporting(channel);
        info_m("Done");
    end procedure;
    
    
    procedure CAN_wait_bus_on(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable fault_state    :       SW_fault_state;
    begin
        info_m("Waiting till bus is on in node: " & t_feature_node'image(node));
        mem_bus_agent_disable_transaction_reporting(channel);
        
        get_fault_state(fault_state, node, channel);
        while (fault_state /= fc_error_active) loop
            get_fault_state(fault_state, node, channel);
            wait for 200 ns;
        end loop;
        
        mem_bus_agent_enable_transaction_reporting(channel);
        info_m("Bus on in node: " & t_feature_node'image(node));
    end procedure;


    procedure CAN_calc_frame_length(
        constant frame          : in    SW_CAN_frame_type;
        variable bit_length     : inout natural
    )is
        variable aux            :       std_logic_vector(1 downto 0);
        variable data_length    :       natural;
    begin
        decode_dlc(frame.dlc, data_length);
        if (frame.rtr = RTR_FRAME and frame.frame_format = NORMAL_CAN) then
            data_length := 0;
        end if;

        -- Join the ident type and frame type
        aux:= frame.ident_type & frame.frame_format;

        -- Calculated identifer and control length
        case aux is
        when "00" =>
            bit_length := 18;
        when "01" =>
            bit_length := 23;
        when "10" =>
            bit_length := 39;
        when "11" =>
            bit_length := 41;
        when others =>
        end case;

        -- Add the data length field
        bit_length := bit_length + data_length;

        -- Add CRC
        if (data_length < 9) then
            bit_length := bit_length + 15;
        elsif (data_length < 17) then
            bit_length := bit_length + 17;
        else
            bit_length := bit_length + 21;
        end if;

        -- Add CRC delimiter, Acknowledge and Acknowledge delimiter
        bit_length := bit_length + 3;
    end procedure;


    procedure send_TXT_buf_cmd(
        constant cmd            : in    SW_TXT_Buffer_command_type;
        constant buf_n          : in    SW_TXT_index_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :         std_logic_vector(31 downto 0)
                                            := (OTHERS => '0');
    begin
        -- Set active command bit in TX_COMMAND register based on input command
        data(TXCE_IND) := '0';
        data(TXCR_IND) := '0';
        data(TXCA_IND) := '0';
        if (cmd = buf_set_empty) then
            data(TXCE_IND) := '1';
        elsif (cmd = buf_set_ready) then
            data(TXCR_IND) := '1';
        elsif (cmd = buf_set_abort) then
            data(TXCA_IND) := '1';
        end if;

        -- Set index of Buffer on which the command should be executed.
        data(buf_n + TXB1_IND - 1) := '1';

        -- Give the command
        CAN_write(data, TX_COMMAND_ADR, node, channel);
    end procedure;
    
    
    procedure send_TXT_buf_cmd(
        constant cmd            : in    SW_TXT_Buffer_command_type;
        constant buf_vector     : in    std_logic_vector(7 downto 0);
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :         std_logic_vector(31 downto 0)
                                            := (OTHERS => '0');
    begin
        -- Set active command bit in TX_COMMAND register based on input command
        data(TXCE_IND) := '0';
        data(TXCR_IND) := '0';
        data(TXCA_IND) := '0';
        if (cmd = buf_set_empty) then
            data(TXCE_IND) := '1';
        elsif (cmd = buf_set_ready) then
            data(TXCR_IND) := '1';
        elsif (cmd = buf_set_abort) then
            data(TXCA_IND) := '1';
        end if;
        
        -- Set index of Buffer on which the command should be executed.
        for i in 0 to 7 loop
        	if(buf_vector(i) = '1') then
        		data(i + TXB1_IND) := '1';
        	end if;
        end loop;  

        -- Give the command
        CAN_write(data, TX_COMMAND_ADR, node, channel);
    end procedure;
    

    procedure get_tx_buf_state(
        constant buf_n          : in    SW_TXT_index_type;
        variable retVal         : out   SW_TXT_Buffer_state_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0);
        variable b_state        :       std_logic_vector(3 downto 0);
        variable buf_index      :       natural range 0 to 7;
    begin
        CAN_read(data, TX_STATUS_ADR, node, channel);
        buf_index := buf_n - 1;
        b_state   := data((buf_index + 1) * 4 - 1 downto buf_index * 4);

        case b_state is
        when TXT_RDY  => retVal := buf_ready;
        when TXT_TRAN => retVal := buf_tx_progress;
        when TXT_ABTP => retVal := buf_ab_progress;
        when TXT_TOK  => retVal := buf_done;
        when TXT_ERR  => retVal := buf_failed;
        when TXT_ABT  => retVal := buf_aborted;
        when TXT_ETY  => retVal := buf_empty;
        when TXT_NOT_EXIST => retVal := buf_not_exist;
        when others =>
        error_m("Invalid TXT Buffer state: " &
              integer'image(to_integer(unsigned(b_state))));
        end case;

    end procedure;


    procedure get_tx_buf_count(
        variable num_buffers    : out   natural range 1 to 8;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(15 downto 0);
        variable low            :       integer;
        variable high           :       integer;
    begin
        CAN_read(data, TXTB_INFO_ADR, node, channel);
        low := TXT_BUFFER_COUNT_L mod 16;
        high := TXT_BUFFER_COUNT_H mod 16;
        info_m("Number of TXT buffers: " & integer'image(
                to_integer(unsigned(data(high downto low)))));
        num_buffers := to_integer(unsigned(data(high downto low)));
    end procedure;


    procedure pick_random_txt_buffer(
        variable txt_buf        : out   SW_TXT_index_type;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable num_buffers : natural range 1 to 8;
        variable tmp : natural;
    begin
        get_tx_buf_count(num_buffers, node, channel);
        rand_int_v(num_buffers, tmp);
        if (tmp = 0) then
            tmp := 1;
        end if;
        txt_buf := tmp;
    end procedure;


    procedure get_rx_buf_state(
        variable retVal         : out   SW_RX_Buffer_info;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        -- Read information about buffer memory first!
        CAN_read(data, RX_MEM_INFO_ADR, node, channel);
        retVal.rx_buff_size := to_integer(unsigned(
                                 data(RX_BUFF_SIZE_H downto RX_BUFF_SIZE_L)));
        retVal.rx_mem_free := to_integer(unsigned(
                                 data(RX_MEM_FREE_H downto RX_MEM_FREE_L)));

        -- Read memory pointers
        CAN_read(data, RX_POINTERS_ADR, node, channel);
        retVal.rx_write_pointer := to_integer(unsigned(
                                     data(RX_WPP_H downto RX_WPP_L)));
        retVal.rx_read_pointer  := to_integer(unsigned(
                                     data(RX_RPP_H downto RX_RPP_L)));

        -- Read memory status
        CAN_read(data, RX_STATUS_ADR, node, channel);
        retVal.rx_full          := false;
        retVal.rx_empty         := false;
        retVal.rx_mof           := false;

        if (data(RXF_IND) = '1') then
            retVal.rx_full      := true;
        end if;

        if (data(RXE_IND) = '1') then
            retVal.rx_empty     := true;
        end if;
        
        if (data(RXMOF_IND) = '1') then
            retVal.rx_mof       := true;
        end if;

        retVal.rx_frame_count   := to_integer(unsigned(
                                    data(RXFRC_H downto RXFRC_L)));
    end procedure;


    procedure set_rx_buf_options(
        constant options        : in    SW_RX_Buffer_options;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data : std_logic_vector(7 downto 0) := (OTHERS => '0');
    begin
        if (options.rx_time_stamp_options) then
            data(RTSOP_IND mod 8) := RTS_BEG;
        else
            data(RTSOP_IND mod 8) := RTS_END;
        end if;

        CAN_write(data, RX_SETTINGS_ADR, node, channel);
    end procedure;


    procedure get_core_version(
        variable retVal         : out   natural;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(15 downto 0);
        variable low_major      :       integer;
        variable high_major     :       integer;
        variable low_minor      :       integer;
        variable high_minor     :       integer;
    begin
        CAN_read(data, VERSION_ADR, node, channel);

        low_major := VER_MAJOR_L mod 16;
        high_major := VER_MAJOR_H mod 16;
        low_minor := VER_MINOR_L mod 16;
        high_minor := VER_MINOR_H mod 16;
        retVal := (10 * to_integer(unsigned(data(high_major downto low_major)))) +
                  to_integer(unsigned(data(high_minor downto low_minor)));
    end procedure;


    procedure set_core_mode(
        constant mode           : in    SW_mode;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data : std_logic_vector(15 downto 0);
    begin
        data := (OTHERS => '0');

        -- Following modes are stored in MODE register
        if (mode.reset) then
            data(RST_IND mod 16)       := '1';
        end if;

        if (mode.bus_monitoring) then
            data(BMM_IND mod 16)       := '1';
        end if;

        if (mode.self_test) then
            data(STM_IND mod 16)       := '1';
        end if;
        
        if (mode.test) then
            data(TSTM_IND mod 16)      := '1';
        end if;

        if (mode.acceptance_filter) then
            data(AFM_IND mod 16)       := '1';
        end if;

        if (mode.flexible_data_rate) then
            data(FDE_IND mod 16)       := '1';
        end if;

        if (mode.restricted_operation) then
            data(ROM_IND mod 16)       := '1';
        end if;

        if (mode.acknowledge_forbidden) then
            data(ACF_IND mod 16)       := '1';
        end if;

        CAN_write(data, MODE_ADR, node, channel);

        -- Following modes are stored in SETTINGS register
        CAN_read(data, SETTINGS_ADR, node, channel);

        if (mode.iso_fd_support) then
            data(NISOFD_IND mod 16)   := '0';
        else
            data(NISOFD_IND mod 16)   := '1';
        end if;

        if (mode.internal_loopback) then
            data(ILBP_IND mod 16)   := '1';
        else
            data(ILBP_IND mod 16)   := '0';
        end if;
        
        if (mode.pex_support) then
            data(PEX_IND mod 16) := '1';
        else
            data(PEX_IND mod 16) := '0';
        end if;
        
        if (mode.fdrf) then
            data(FDRF_IND mod 16) := '1';
        else
            data(FDRF_IND mod 16) := '0';
        end if;
        
        if (mode.tx_buf_bus_off_failed) then
            data(TBFBO_IND mod 16) := '1';
        else
            data(TBFBO_IND mod 16) := '0';
        end if;

        CAN_write(data, SETTINGS_ADR, node, channel);
    end procedure;


    procedure get_core_mode(
        variable mode           : out   SW_mode;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        CAN_read(data, MODE_ADR, node, channel);

        mode.reset                      := false;
        mode.bus_monitoring             := false;
        mode.self_test                  := false;
        mode.acceptance_filter          := false;
        mode.flexible_data_rate         := false;
        mode.rtr_pref                   := false;
        mode.acknowledge_forbidden      := false;
        mode.test                       := false;
        mode.fdrf                       := false;
        mode.restricted_operation       := false;

        if (data(RST_IND) = '1') then
            mode.reset                  := true;
        end if;

        if (data(BMM_IND) = '1') then
            mode.bus_monitoring         := true;
        end if;

        if (data(STM_IND) = '1') then
            mode.self_test              := true;
        end if;

        if (data(AFM_IND) = '1') then
            mode.acceptance_filter      := true;
        end if;

        if (data(FDE_IND) = '1') then
            mode.flexible_data_rate     := true;
        end if;

        if (data(ROM_IND) = '1') then
            mode.restricted_operation   := true;
        end if;

        if (data(ACF_IND) = '1') then
            mode.acknowledge_forbidden  := true;
        end if;
        
        if (data(TSTM_IND) = '1') then
            mode.test := true;
        end if;


        -- SETTINGs part of read data

        if (data(NISOFD_IND) = '0') then
            mode.iso_fd_support         := true;
        else
            mode.iso_fd_support         := false;
        end if;

        if (data(ILBP_IND) = '1') then
            mode.internal_loopback      := true;
        else
            mode.internal_loopback      := false;
        end if;
        
        if (data(PEX_IND) = '1') then
            mode.pex_support            := true;
        else
            mode.pex_support            := false;
        end if;
        
        if (data(FDRF_IND) = '1') then
            mode.fdrf                   := true;
        else
            mode.fdrf                   := false;
        end if;
        
        if (data(TBFBO_IND) = '1') then
            mode.tx_buf_bus_off_failed  := true;
        else
            mode.tx_buf_bus_off_failed  := false;
        end if;

    end procedure;


    procedure exec_SW_reset(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable mode           :       SW_mode;
    begin
        get_core_mode(mode, node, channel);

        -- Note that reset bit is self clearing, no need to write 0 afterwards!
        mode.reset := true;

        set_core_mode(mode, node, channel);
    end procedure;


    procedure give_controller_command(
        constant command        : in    SW_command;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        data := (OTHERS => '0');

        if (command.release_rec_buffer) then
            data(RRB_IND)        := '1';
        end if;

        if (command.clear_data_overrun) then
            data(CDO_IND)        := '1';
        end if;
        
        if (command.err_ctrs_rst) then
            data(ERCRST_IND)     := '1';
        end if;
        
        if (command.rx_frame_ctr_rst) then
            data(RXFCRST_IND)    := '1';
        end if;
        
        if (command.tx_frame_ctr_rst) then
            data(TXFCRST_IND)    := '1';
        end if;

        if (command.clear_pexs_flag) then
            data(CPEXS_IND)      := '1';
        end if;
        
        CAN_write(data, COMMAND_ADR, node, channel);
    end procedure;


    procedure get_controller_status(
        variable status         : out   SW_status;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        CAN_read(data, STATUS_ADR, node, channel);

        status.receive_buffer           := false;
        status.data_overrun             := false;
        status.tx_buffer_empty          := false;
        status.error_transmission       := false;
        status.receiver                 := false;
        status.transmitter              := false;
        status.error_warning            := false;
        status.bus_status               := false;

        if (data(RXNE_IND) = '1') then
            status.receive_buffer       := true;
        end if;

        if (data(DOR_IND) = '1') then
            status.data_overrun         := true;
        end if;

        if (data(TXNF_IND) = '1') then
            status.tx_buffer_empty      := true;
        end if;

        if (data(EFT_IND) = '1') then
            status.error_transmission   := true;
        end if;

        if (data(RXS_IND) = '1') then
            status.receiver             := true;
        end if;

        if (data(TXS_IND) = '1') then
            status.transmitter          := true;
        end if;

        if (data(EWL_IND) = '1') then
            status.error_warning        := true;
        end if;

        if (data(IDLE_IND) = '1') then
            status.bus_status           := true;
        end if;
        
        if (data(PEXS_IND) = '1') then
            status.protocol_exception   := true;
        end if;
    end procedure;


    procedure configure_retransmitt_limit(
        constant enable         : in    boolean;
        constant limit          : in    natural range 0 to 15;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data : std_logic_vector(15 downto 0);
        variable low : natural := RTRTH_L mod 16;
        variable high : natural := RTRTH_H mod 16;
    begin
        CAN_read(data, SETTINGS_ADR, node, channel);

        if (enable) then
            data(RTRLE_IND mod 16) := '1';
        else
            data(RTRLE_IND mod 16) := '0';
        end if;

        data(high downto low) :=
            std_logic_vector(to_unsigned(limit, RTRTH_H - RTRTH_L + 1));

        CAN_write(data, SETTINGS_ADR, node, channel);
    end procedure;


    procedure enable_controller(
        constant enable         : in    boolean;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data : std_logic_vector(15 downto 0);
    begin
        CAN_read(data, SETTINGS_ADR, node, channel);

        if (enable) then
            data(ENA_IND mod 16) := '1';
        else
            data(ENA_IND mod 16) := '0';
        end if;

        CAN_write(data, SETTINGS_ADR, node, channel);
    end procedure;


    function sw_int_to_int_reg(
        constant interrupts     :       SW_interrupts
    )return std_logic_vector is
        variable tmp            :       std_logic_vector(31 downto 0);
    begin
        tmp := (OTHERS => '0');

        if (interrupts.receive_int) then
            tmp(RXI_IND)        :=  '1';
        end if;

        if (interrupts.transmitt_int) then
            tmp(TXI_IND)        :=  '1';
        end if;

        if (interrupts.error_warning_int) then
            tmp(EWLI_IND)       :=  '1';
        end if;

        if (interrupts.data_overrun_int) then
            tmp(DOI_IND)        :=  '1';
        end if;

        if (interrupts.fcs_changed_int) then
            tmp(FCSI_IND)       :=  '1';
        end if;

        if (interrupts.arb_lost_int) then
            tmp(ALI_IND)        :=  '1';
        end if;

        if (interrupts.bus_error_int) then
            tmp(BEI_IND)        :=  '1';
        end if;

        if (interrupts.rx_buffer_full_int) then
            tmp(RXFI_IND)       :=  '1';
        end if;

        if (interrupts.bit_rate_shift_int) then
            tmp(BSI_IND)        :=  '1';
        end if;

        if (interrupts.rx_buffer_not_empty_int) then
            tmp(RBNEI_IND)      :=  '1';
        end if;

        if (interrupts.tx_buffer_hw_cmd) then
            tmp(TXBHCI_IND)     :=  '1';
        end if;
        
        if (interrupts.overload_frame) then
            tmp(OFI_IND)        := '1';
        end if;

        return tmp;
    end function;


    function int_reg_to_sw_int(
        constant int_reg        :       std_logic_vector(31 downto 0)
    )return SW_interrupts is
        variable tmp            :       SW_interrupts;
    begin
        tmp := (false, false, false, false, false, false,
                false, false, false, false, false, false, false);

        if (int_reg(RXI_IND) = '1') then
            tmp.receive_int              :=  true;
        end if;

        if (int_reg(TXI_IND) = '1') then
            tmp.transmitt_int            :=  true;
        end if;

        if (int_reg(EWLI_IND) = '1') then
            tmp.error_warning_int        :=  true;
        end if;

        if (int_reg(DOI_IND) = '1') then
            tmp.data_overrun_int         :=  true;
        end if;

        if (int_reg(FCSI_IND) = '1') then
            tmp.fcs_changed_int          :=  true;
        end if;

        if (int_reg(ALI_IND) = '1') then
            tmp.arb_lost_int             :=  true;
        end if;

        if (int_reg(BEI_IND) = '1') then
            tmp.bus_error_int            :=  true;
        end if;

        if (int_reg(RXFI_IND) = '1') then
            tmp.rx_buffer_full_int       :=  true;
        end if;

        if (int_reg(BSI_IND) = '1') then
            tmp.bit_rate_shift_int       :=  true;
        end if;

        if (int_reg(RBNEI_IND) = '1') then
            tmp.rx_buffer_not_empty_int  :=  true;
        end if;

        if (int_reg(TXBHCI_IND) = '1') then
            tmp.tx_buffer_hw_cmd         :=  true;
        end if;

        if (int_reg(OFI_IND) = '1') then
            tmp.overload_frame           := true;
        end if;

        return tmp;
    end function;


    procedure read_int_status(
        variable interrupts     : out   SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        CAN_read(data, INT_STAT_ADR, node, channel);
        interrupts := int_reg_to_sw_int(data);
    end procedure;


    procedure clear_int_status(
        constant interrupts     : in    SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        data := sw_int_to_int_reg(interrupts);
        CAN_write(data, INT_STAT_ADR, node, channel);
    end procedure;


    procedure read_int_enable(
        variable interrupts     : out   SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        CAN_read(data, INT_ENA_SET_ADR, node, channel);
        interrupts := int_reg_to_sw_int(data);
    end procedure;


    procedure write_int_enable(
        constant interrupts     : in    SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        -- Set interrupts which should be set to 1
        data := sw_int_to_int_reg(interrupts);
        CAN_write(data, INT_ENA_SET_ADR, node, channel);

        -- Clear interrupts which should be set to 0
        data := not data;
        CAN_write(data, INT_ENA_CLR_ADR, node, channel);
    end procedure;


    procedure read_int_mask(
        variable interrupts     : out   SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        CAN_read(data, INT_MASK_SET_ADR, node, channel);
        interrupts := int_reg_to_sw_int(data);
    end procedure;


    procedure write_int_mask(
        constant interrupts     : in    SW_interrupts;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        -- Set interrupts which should be set to 1
        data := sw_int_to_int_reg(interrupts);
        CAN_write(data, INT_MASK_SET_ADR, node, channel);

        -- Clear interrupts which should be set to 0
        data := not data;
        CAN_write(data, INT_MASK_CLR_ADR, node, channel);
    end procedure;



    procedure get_fault_state(
        variable fault_state    : out   SW_fault_state;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(15 downto 0);
    begin
        CAN_read(data, FAULT_STATE_ADR, node, channel);

        if (data(ERA_IND mod 16) = '1') then
            fault_state         := fc_error_active;
        elsif (data(ERP_IND mod 16) = '1') then
            fault_state         := fc_error_passive;
        elsif (data(BOF_IND mod 16) = '1') then
            fault_state         := fc_bus_off;
        end if;
    end procedure;


    procedure set_fault_thresholds(
        constant fault_th       : in    SW_fault_thresholds;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data : std_logic_vector(7 downto 0) := (OTHERS => '0');
    begin
        data((EW_LIMIT_H mod 8) downto (EW_LIMIT_L mod 8)) :=
            std_logic_vector(to_unsigned(fault_th.ewl, 8));
        CAN_write(data, EWL_ADR, node, channel);
        
        data((ERP_LIMIT_H mod 8) downto (ERP_LIMIT_L mod 8)) :=
            std_logic_vector(to_unsigned(fault_th.erp, 8));
        CAN_write(data, ERP_ADR, node, channel);
    end procedure;


    procedure get_fault_thresholds(
        variable fault_th       : out   SW_fault_thresholds;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data : std_logic_vector(7 downto 0) := (OTHERS => '0');
        variable low : integer;
        variable high : integer;
    begin
        CAN_read(data, EWL_ADR, node, channel);
        low := EW_LIMIT_L mod 8;
        high := EW_LIMIT_H mod 8;
        fault_th.ewl := to_integer(unsigned(data(high downto low)));

        CAN_read(data, ERP_ADR, node, channel);
        low := ERP_LIMIT_L mod 8;
        high := ERP_LIMIT_H mod 8;
        fault_th.erp := to_integer(unsigned(data(high downto low)));
    end procedure;


    procedure read_error_counters(
        variable err_counters   : out   SW_error_counters;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(15 downto 0);
        variable msg            :       line;
    begin
        -- Reading separately for possible future separation of REC and TEC!
        CAN_read(data, REC_ADR, node, channel);

        err_counters.rx_counter :=
                to_integer(unsigned(data((REC_VAL_H mod 16) downto 
                                         (REC_VAL_L mod 16) )));

        CAN_read(data, TEC_ADR, node, channel);
        err_counters.tx_counter :=
                to_integer(unsigned(data((TEC_VAL_H mod 16) downto 
                                         (TEC_VAL_L mod 16) )));

        CAN_read(data, ERR_NORM_ADR, node, channel);
        err_counters.err_norm :=
                to_integer(unsigned(data((ERR_NORM_VAL_H mod 16) downto
                                         (ERR_NORM_VAL_L mod 16) )));

        CAN_read(data, ERR_FD_ADR, node, channel);
        err_counters.err_fd :=
                to_integer(unsigned(data((ERR_FD_VAL_H mod 16) downto
                                         (ERR_FD_VAL_L mod 16) )));
    end procedure;


    procedure set_test_mem_access(
        constant enable         : in    boolean;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable mode : SW_mode;
        variable data : std_logic_vector(31 downto 0) := (OTHERS => '0');
    begin
        -- Check test mode is set, throw error otherwise
        get_core_mode(mode, node, channel);
        check_m(mode.test, "Test mode must be set when enabling Test memory access");

        -- Enable access
        if (enable) then
            data(TMAENA_IND) := '1';
        else
            data(TMAENA_IND) := '0';
        end if;
        CAN_write(data, TST_CONTROL_ADR, node, channel);
    end procedure;

    
    function tgt_test_mem_to_reg(
        constant tgt_mem        : in  t_tgt_test_mem
    ) return std_logic_vector is
    begin
        case tgt_mem is
        when TST_TGT_RX_BUF =>
            return TMTGT_RXBUF;
        when TST_TGT_TXT_BUF_1 =>
            return TMTGT_TXTBUF1;
        when TST_TGT_TXT_BUF_2 =>
            return TMTGT_TXTBUF2;
        when TST_TGT_TXT_BUF_3 =>
            return TMTGT_TXTBUF3;
        when TST_TGT_TXT_BUF_4 =>
            return TMTGT_TXTBUF4;
        when TST_TGT_TXT_BUF_5 =>
            return TMTGT_TXTBUF5;
        when TST_TGT_TXT_BUF_6 =>
            return TMTGT_TXTBUF6;
        when TST_TGT_TXT_BUF_7 =>
            return TMTGT_TXTBUF7;
        when TST_TGT_TXT_BUF_8 =>
            return TMTGT_TXTBUF8;
        end case;
    end function;


    procedure test_mem_write(
        constant data           : in    std_logic_vector(31 downto 0);
        constant address        : in    natural;
        constant tgt_mem        : in    t_tgt_test_mem;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data_i : std_logic_vector(31 downto 0) := (OTHERS => '0');
    begin
        -- Set address
        data_i(TST_ADDR_H downto TST_ADDR_L) :=
            std_logic_vector(to_unsigned(address, 16));
        data_i(TST_MTGT_H downto TST_MTGT_L) := tgt_test_mem_to_reg(tgt_mem);
        CAN_write(data_i, TST_DEST_ADR, node, channel);
        
        -- Set data
        data_i := data;
        CAN_write(data_i, TST_WDATA_ADR, node, channel);

        -- Execute write
        data_i := (OTHERS => '0');
        data_i(TMAENA_IND) := '1';
        data_i(TWRSTB_IND) := '1';
        CAN_write(data_i, TST_CONTROL_ADR, node, channel);        
    end procedure;


    procedure test_mem_read(
        variable data           : out   std_logic_vector(31 downto 0);
        constant address        : in    natural;
        constant tgt_mem        : in    t_tgt_test_mem;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data_i : std_logic_vector(31 downto 0) := (OTHERS => '0');
    begin
        -- Set address
        data_i(TST_ADDR_H downto TST_ADDR_L) :=
            std_logic_vector(to_unsigned(address, 16));
        data_i(TST_MTGT_H downto TST_MTGT_L) := tgt_test_mem_to_reg(tgt_mem);
        CAN_write(data_i, TST_DEST_ADR, node, channel);
        
        -- Wait for one clock cycle
        clk_agent_wait_cycle(channel);

        -- Read data
        CAN_read(data, TST_RDATA_ADR, node, channel);   
    end procedure;


    function str_equal(
        a : string;
        b : string
    ) return boolean is
        constant len : natural := MINIMUM(a'length, b'length);
        constant atail : string(a'left+len to a'right) := (others => ' ');
        constant btail : string(b'left+len to b'right) := (others => ' ');
    begin
        return     a(a'left to a'left+len-1) = b(b'left to b'left+len-1)
               and a(a'left+len to a'right) = atail
               and b(b'left+len to b'right) = btail;
    end function str_equal;


    impure function strtolen(
        n : natural;
        src : string
    ) return string is
        variable s : string(1 to n) := (others => ' ');
    begin
        check_m(src'length <= n, "String too long.");
        s(src'range) := src;
        return s;
    end function strtolen;


    procedure set_error_counters(
        constant err_counters   : in    SW_error_counters;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(15 downto 0);
        variable low            :       integer;
        variable high           :       integer;
    begin
        data := (OTHERS => '0');

        -- Indices for preset values
        low := CTPV_L mod 16;
        high := CTPV_H mod 16;
        
        -- TX Error counter
        data(high downto low) := std_logic_vector(to_unsigned(
                                    err_counters.tx_counter, 9));
        data(PTX_IND mod 16) := '1';
        CAN_write(data, CTR_PRES_ADR, node, channel);
        data(PTX_IND mod 16) := '0';

        -- RX Error counter
        data(high downto low) := std_logic_vector(to_unsigned(
                                    err_counters.rx_counter, 9));
        data(PRX_IND mod 16) := '1';
        CAN_write(data, CTR_PRES_ADR, node, channel);
        data(PRX_IND mod 16) := '0';

        -- Nominal bit rate counter
        data(high downto low) := std_logic_vector(to_unsigned(
                                    err_counters.err_norm, 9));
        data(ENORM_IND mod 16) := '1';
        CAN_write(data, CTR_PRES_ADR, node, channel);
        data(ENORM_IND mod 16) := '0';

        -- Data bit rate counter
        data(high downto low) := std_logic_vector(to_unsigned(
                                        err_counters.err_fd, 9));
        data(EFD_IND mod 16) := '1';
        CAN_write(data, CTR_PRES_ADR, node, channel);
        data(EFD_IND mod 16) := '0';
    end procedure;


    procedure read_alc(
        variable alc            : out   natural;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(7 downto 0);
        variable low            :       integer;
        variable high           :       integer;
    begin
        CAN_read(data, ALC_ADR, node, channel);

        low := ALC_ID_FIELD_L mod 8;
        high := ALC_ID_FIELD_H mod 8;
        case data(high downto low) is
        when ALC_BASE_ID =>
            alc := 11 - to_integer(unsigned(data((ALC_BIT_H mod 8) downto (ALC_BIT_L mod 8))));
        when ALC_EXTENSION =>
            alc := 31 - to_integer(unsigned(data((ALC_BIT_H mod 8) downto (ALC_BIT_L mod 8))));
        when ALC_SRR_RTR =>
            alc := 12;
        when ALC_IDE =>
            alc := 13;
        when ALC_RTR =>
            alc := 32;
        when ALC_RSVD =>
            alc := 0;
        when others =>
            error_m("Unsupported ALC type");
        end case;

    end procedure;


    procedure read_traffic_counters(
        variable ctr            : out   SW_traffic_counters;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        CAN_read(data, RX_FR_CTR_ADR, node, channel);
        ctr.rx_frames := to_integer(unsigned(data(
                            RX_FR_CTR_VAL_H downto RX_FR_CTR_VAL_L)));

        CAN_read(data, TX_FR_CTR_ADR, node, channel);
        ctr.tx_frames := to_integer(unsigned(data(
                            TX_FR_CTR_VAL_H downto TX_FR_CTR_VAL_L)));
    end procedure;


    procedure read_trv_delay(
        variable trv_delay      : out   natural;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable data           :       std_logic_vector(31 downto 0);
    begin
        CAN_read(data, TRV_DELAY_ADR, node, channel);
        trv_delay := to_integer(unsigned(data(
                            TRV_DELAY_VALUE_H downto TRV_DELAY_VALUE_L)));
    end procedure;



    procedure CAN_read_timestamp(
        variable ts	            : out   std_logic_vector(63 downto 0);
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable lower_word     :       std_logic_vector(31 downto 0);
        variable upper_word     :       std_logic_vector(31 downto 0);
    begin
        CAN_read(lower_word, TIMESTAMP_LOW_ADR, node, channel);
        CAN_read(upper_word, TIMESTAMP_HIGH_ADR, node, channel);

        ts := upper_word & lower_word;
    end procedure;


    procedure CAN_configure_ssp(
        constant ssp_source     : in    SSP_set_command_type;
        constant ssp_offset_val : in    std_logic_vector(7 downto 0);
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(15 downto 0) :=
                                            (OTHERS => '0');
        variable low            :       integer;
        variable high           :       integer;
    begin
        low := SSP_SRC_L mod 16;
        high := SSP_SRC_H mod 16;
        
        case ssp_source is
            when ssp_meas_n_offset =>
                data(high downto low) := SSP_SRC_MEAS_N_OFFSET;
            when ssp_no_ssp =>
               data(high downto low) := SSP_SRC_NO_SSP;
            when ssp_offset =>
                data(high downto low) := SSP_SRC_OFFSET;
            when others =>
                error_m("Unsupported SSP type.");
            end case;

        low := SSP_OFFSET_L mod 16;
        high := SSP_OFFSET_H mod 16;
        data(high downto low) := ssp_offset_val;

        CAN_write(data, SSP_CFG_ADR, node, channel);
    end procedure;
    
    
    procedure CAN_read_pc_debug_m(
        variable pc_dbg         : out   SW_PC_Debug;   
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
    begin
        CAN_read(data, DEBUG_REGISTER_ADR, node, channel);
    
        pc_dbg := pc_deb_none;
        if (data(PC_ARB_IND) = '1') then
            pc_dbg := pc_deb_arbitration;
        elsif (data(PC_CON_IND) = '1') then
            pc_dbg := pc_deb_control;
        elsif (data(PC_DAT_IND) = '1') then
            pc_dbg := pc_deb_data;
        elsif (data(PC_STC_IND) = '1') then
            pc_dbg := pc_deb_stuff_count;
        elsif (data(PC_CRC_IND) = '1') then
            pc_dbg := pc_deb_crc;
        elsif (data(PC_CRCD_IND) = '1') then
            pc_dbg := pc_deb_crc_delim;
        elsif (data(PC_ACK_IND) = '1') then
            pc_dbg := pc_deb_ack;
        elsif (data(PC_ACKD_IND) = '1') then
            pc_dbg := pc_deb_ack_delim;
        elsif (data(PC_EOF_IND) = '1') then
            pc_dbg := pc_deb_eof;
        elsif (data(PC_INT_IND) = '1') then
            pc_dbg := pc_deb_intermission;
        elsif (data(PC_SUSP_IND) = '1') then
            pc_dbg := pc_deb_suspend;
        elsif (data(PC_OVR_IND) = '1') then
            pc_dbg := pc_deb_overload;
        elsif (data(PC_SOF_IND) = '1') then
            pc_dbg := pc_deb_sof;
        end if;
    end procedure;


    procedure CAN_wait_pc_state(
        constant pc_state       : in    SW_PC_Debug;   
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable read_state     :       SW_PC_Debug;
    begin
        info_m("Waiting till node: " & t_feature_node'image(node) &
               " is in state: " & SW_PC_Debug'image(pc_state));
        mem_bus_agent_disable_transaction_reporting(channel);
        
        CAN_read_pc_debug_m(read_state, node, channel);
        while (read_state /= pc_state) loop
            clk_agent_wait_cycle(channel);
            CAN_read_pc_debug_m(read_state, node, channel);
        end loop;
        
        mem_bus_agent_enable_transaction_reporting(channel);
        info_m("Done");
    end procedure;
    
    
    procedure CAN_wait_not_pc_state(
        constant pc_state       : in    SW_PC_Debug;   
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    )is
        variable read_state     :       SW_PC_Debug;
    begin
        info_m("Waiting till node: " & t_feature_node'image(node) &
               " is NOT in state: " & SW_PC_Debug'image(pc_state));
        mem_bus_agent_disable_transaction_reporting(channel);
        
        CAN_read_pc_debug_m(read_state, node, channel);
        while (read_state = pc_state) loop
            clk_agent_wait_cycle(channel);
            CAN_read_pc_debug_m(read_state, node, channel);
        end loop;
        
        mem_bus_agent_enable_transaction_reporting(channel);
    end procedure;
    

    procedure CAN_configure_tx_priority(
        constant buff_number    : in    SW_TXT_index_type;
        constant priority       : in    natural range 0 to 7;   
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
   ) is
        variable data           :       std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable address        :       std_logic_vector(11 downto 0) :=
            (OTHERS => '0');
    begin   
        -- Read current register value to variable
        address := TX_PRIORITY_ADR;
        CAN_read(data, address, node, channel);
        info_m("Read 'TX_PRIORITY_ADR': 0x" & to_hstring(data) & ".");
        
       -- Select buffer and modify appropriate bits in the register
        case buff_number is
            when 1 =>
                data (2 downto 0) := std_logic_vector(to_unsigned(priority, 3));
            when 2 =>
                data (6 downto 4) := std_logic_vector(to_unsigned(priority, 3));
            when 3 =>
                data (10 downto 8) := std_logic_vector(to_unsigned(priority, 3));
            when 4 =>
                data (14 downto 12) := std_logic_vector(to_unsigned(priority, 3));
            when 5 =>
                data (18 downto 16) := std_logic_vector(to_unsigned(priority, 3));
            when 6 =>
                data (22 downto 20) := std_logic_vector(to_unsigned(priority, 3));
            when 7 =>
                data (26 downto 24) := std_logic_vector(to_unsigned(priority, 3));
            when 8 =>
                data (30 downto 28) := std_logic_vector(to_unsigned(priority, 3));
            when others =>
                error_m("Unsupported TX buffer number.");
            end case;

        -- Write back new value and exit procedure
        info_m("Write 'TX_PRIORITY_ADR': 0x" & to_hstring(data) & ".");
        address := TX_PRIORITY_ADR;
        CAN_write(data, address, node, channel);
    end procedure;


    procedure CAN_read_error_code_capture(
        variable err_capt       : inout SW_error_capture;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data           :       std_logic_vector(7 downto 0) :=
                                            (OTHERS => '0');
        variable low            :       integer;
        variable high           :       integer;
    begin
        CAN_read(data, ERR_CAPT_ADR, node, channel);
        
        low := ERR_TYPE_L mod 8;
        high := ERR_TYPE_H mod 8;
        case data(high downto low) is
        when ERC_BIT_ERR  => err_capt.err_type := can_err_bit;
        when ERC_CRC_ERR  => err_capt.err_type := can_err_crc;
        when ERC_FRM_ERR  => err_capt.err_type := can_err_form;
        when ERC_ACK_ERR  => err_capt.err_type := can_err_ack;
        when ERC_STUF_ERR => err_capt.err_type := can_err_stuff;
        when others =>
            error_m("Unknown Error type in Error code capture register!");
        end case;
        
        low := ERR_POS_L mod 8;
        high := ERR_POS_H mod 8;
        case data(high downto low) is
        when ERC_POS_SOF    => err_capt.err_pos := err_pos_sof;
        when ERC_POS_ARB    => err_capt.err_pos := err_pos_arbitration;
        when ERC_POS_CTRL   => err_capt.err_pos := err_pos_ctrl;
        when ERC_POS_DATA   => err_capt.err_pos := err_pos_data;
        when ERC_POS_CRC    => err_capt.err_pos := err_pos_crc;
        when ERC_POS_ACK    => err_capt.err_pos := err_pos_ack;
        when ERC_POS_EOF    => err_capt.err_pos := err_pos_eof;
        when ERC_POS_ERR    => err_capt.err_pos := err_pos_err_frame;
        when ERC_POS_OVRL   => err_capt.err_pos := err_pos_overload_frame;
        when ERC_POS_OTHER  => err_capt.err_pos := err_pos_other;
        when others =>
            error_m("Unknown Error position in Error code capture register!");
        end case;

    end procedure;
    
    
    procedure CAN_read_retr_ctr(
        variable retr_ctr       : out   natural;
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable data : std_logic_vector(7 downto 0);
    begin
        CAN_read(data, RETR_CTR_ADR, node, channel);
        retr_ctr := to_integer(unsigned(data));
    end procedure;
    
    
    procedure CAN_wait_sample_point(
        constant node               : in    t_feature_node;
        signal   channel            : inout t_com_channel;
        constant skip_stuff_bits    : in    boolean := true
    ) is
        variable node_i  : integer;
    begin
        if (node = DUT_NODE) then
            node_i := 0;
        else
            node_i := 1;
        end if;
        test_probe_agent_wait_sample(channel, node_i, skip_stuff_bits);
    end procedure;


    procedure CAN_wait_sync_seg(
        constant node               : in    t_feature_node;
        signal   channel            : inout t_com_channel
    ) is
        variable node_i  : integer;
    begin
        if (node = DUT_NODE) then
            node_i := 0;
        else
            node_i := 1;
        end if;
        test_probe_agent_wait_sync(channel, node_i);
    end procedure;


    procedure CAN_init_txtb_mems(
        constant node           : in    t_feature_node;
        signal   channel        : inout t_com_channel
    ) is
        variable address : std_logic_vector(11 downto 0);
        variable data    : std_logic_vector(31 downto 0):= (OTHERS => '0');
        variable num_bufs : integer;
    begin
        mem_bus_agent_disable_transaction_reporting(channel);
        get_tx_buf_count(num_bufs, node, channel);

        for i in 1 to num_bufs loop
            address := std_logic_vector(to_unsigned(
                        to_integer((unsigned(TXTB1_DATA_1_ADR)) * i), 12));
            for j in 0 to 19 loop
                CAN_write(data, address, node, channel);
                address := std_logic_vector(unsigned(address) + 4);
            end loop;
        end loop;
        mem_bus_agent_enable_transaction_reporting(channel);
    end procedure;

    procedure CAN_generate_random_bit_timing(
        variable bt         : inout   bit_time_config_type;
        signal   channel    : inout t_com_channel
    ) is
    begin
        -- Generate random Nominal bit rate!
        rand_int_v(127, bt.prop_nbt);
        rand_int_v(63, bt.ph1_nbt);
        rand_int_v(63, bt.ph2_nbt);
        -- Longer TQ is possible but test run-time is killing us!
        rand_int_v(32, bt.tq_nbt);
        rand_int_v(33, bt.sjw_nbt);

        -- Generate random Nominal bit rate!
        rand_int_v(63, bt.prop_dbt);
        rand_int_v(31, bt.ph1_dbt);
        rand_int_v(31, bt.ph2_dbt);        
        -- Constrain time quanta to something realistinc for data phase so
        -- that we don't have too long run times!
        rand_int_v(16, bt.tq_dbt);
        rand_int_v(33, bt.sjw_dbt);
        
        -- Constrain minimal BRP (0 is not allowed)!
        if (bt.tq_nbt = 0) then
            bt.tq_nbt := 1;
        end if;
        if (bt.tq_dbt = 0) then
            bt.tq_dbt := 1;
        end if;

        -- Make sure we have at least 10 cycles in nominal bit-rate
        -- (TSEG1=6 and TSEG2=4). This is what we have described as
        -- recommended minimum in datasheet!
        if (bt.tq_nbt * (bt.prop_nbt + bt.ph1_dbt + 1) < 6) then
            bt.prop_nbt := 3;
            bt.ph1_nbt := 3;
        end if;
        if (bt.tq_nbt * bt.ph2_nbt < 4) then
            bt.ph2_nbt := 4;
        end if;

        -- Make sure we have at least 5 clock cycles in data bit-rate
        -- (TSEG1 = 3 and TSEG2 = 2), this is absolute minimum!
        if (bt.tq_dbt * (bt.prop_dbt + bt.ph1_dbt + 1) < 3) then
            bt.prop_dbt := 1;
            bt.ph1_dbt := 1;
        end if;
        if (bt.tq_dbt * bt.ph2_dbt < 2) then
            bt.ph2_dbt := 2;
        end if;
        
        -- Make sure we have SJW at least 1
        if (bt.sjw_nbt = 0) then
            bt.sjw_dbt := 1;
        end if;
        if (bt.sjw_dbt = 0) then
            bt.sjw_dbt := 1;
        end if;
        

    end procedure;

end package body;
