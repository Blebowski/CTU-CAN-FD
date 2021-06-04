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
--    Main test library for CTU CAN FD controller. Contains all test resources
--    for running CANTest framework and low-level access functions.
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
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;
use ieee.std_logic_textio.all;
use STD.textio.all;

library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;
use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

library vunit_lib;
context vunit_lib.vunit_context;


package can_unit_test_pkg is

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Types
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    --Common test types
    ----------------------------------------------------------------------------

    -- Logger severity type (severities in increasing order)
    type log_lvl_type is (
        debug_l,
        info_l,
        warning_l,
        error_l
    );

    -- Behaviour of the test when error occurs
    type err_beh_type is (
        quit,
        go_on
    );

    -- Status of test
    type test_status_type is (
        waiting,
        running,
        passed,
        failed
    );

    ----------------------------------------------------------------------------
    -- Message filter types
    ----------------------------------------------------------------------------

    -- Frame information on input of message filter
    type mess_filter_input_type is record
        rec_ident_in            :   std_logic_vector(28 downto 0);
        ident_type              :   std_logic;
        frame_type              :   std_logic;
    end record;

    -- Driving bus values on the input off message filter
    type mess_filter_drv_type is record

        -- Filter A bit mask, control bits and bit value
        drv_filter_A_mask       :   std_logic_vector(28 downto 0);
        drv_filter_A_ctrl       :   std_logic_vector(3 downto 0);
        drv_filter_A_bits       :   std_logic_vector(28 downto 0);

        -- Filter B bit mask, control bits and bit value
        drv_filter_B_mask       :   std_logic_vector(28 downto 0);
        drv_filter_B_ctrl       :   std_logic_vector(3 downto 0);
        drv_filter_B_bits       :   std_logic_vector(28 downto 0);

        -- Filter C bit mask, control bits and bit value
        drv_filter_C_mask       :   std_logic_vector(28 downto 0);
        drv_filter_C_ctrl       :   std_logic_vector(3 downto 0);
        drv_filter_C_bits       :   std_logic_vector(28 downto 0);

        -- Range filter control bits, lower and upper thresholds
        drv_filter_ran_ctrl     :   std_logic_vector(3 downto 0);
        drv_filter_ran_lo_th    :   std_logic_vector(28 downto 0);
        drv_filter_ran_hi_th    :   std_logic_vector(28 downto 0);

        -- Filters are enabled
        drv_filters_ena         :   std_logic;

    end record;

    ----------------------------------------------------------------------------
    -- TXT Buffer types
    ----------------------------------------------------------------------------

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
    
    -- TXT Buffer index type
    subtype SW_TXT_index_type is natural range 1 to 8;


    ----------------------------------------------------------------------------
    -- Avalon memory interface type
    ----------------------------------------------------------------------------
    type Avalon_mem_type is record
        clk_sys                 :   std_logic;
        data_in                 :   std_logic_vector(31 downto 0);
        data_out                :   std_logic_vector(31 downto 0);
        address                 :   std_logic_vector(31 downto 0);
        scs                     :   std_logic;
        swr                     :   std_logic;
        srd                     :   std_logic;
        sbe                     :   std_logic_vector(3 downto 0);
    end record;


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


    ----------------------------------------------------------------------------
    -- Transceiver delay type
    ----------------------------------------------------------------------------
    type tran_delay_type is record
        tx_delay_sr             :   std_logic_vector(255 downto 0);
        rx_delay_sr             :   std_logic_vector(255 downto 0);
        tx_point                :   std_logic;
        rx_point                :   std_logic;
    end record;


    ----------------------------------------------------------------------------
    -- Test storage memory
    ----------------------------------------------------------------------------
    type test_mem_type is array (0 to 255) of std_logic_vector(31 downto 0);

    ----------------------------------------------------------------------------
    -- Clock params
    ----------------------------------------------------------------------------
    type generate_clock_precomputed_t is record
        low_time : time;
        high_time : time;
    end record;

    ----------------------------------------------------------------------------
    -- Bit sequence generator
    ----------------------------------------------------------------------------

    -- Longest possible CAN FD Frame is aroud 700 bits. If each bit has opposite
    -- polarity than previous one, this could use up to 700 entries. Have some
    -- reserve...
    constant BS_MAX_LENGTH      :   natural := 1024;

    type bs_durations_type is array (1 to BS_MAX_LENGTH) of natural;
    type bs_values_type is array (1 to BS_MAX_LENGTH) of std_logic;

    type bit_seq_type is record
        bit_durations           :   bs_durations_type;
        bit_values              :   bs_values_type;
        length                  :   natural;
    end record;


    ----------------------------------------------------------------------------
    -- Generate random CAN FD Frame.
    --
    -- Arguments:
    --  rand_ctr        Pointer to pool of random values.
    --  frame           Output variable in which CAN FD Frame sill be generated.
    ----------------------------------------------------------------------------
    procedure CAN_generate_frame(
        signal   rand_ctr       : inout natural range 0 to RAND_POOL_SIZE;
        variable frame          : inout SW_CAN_frame_type
    );

   

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Constants
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    constant f100_MHZ           :   natural := 10000;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- sanity test Stuff; must be in a package
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    constant NODE_COUNT : natural := 4;
    type bus_matrix_type is array(1 to NODE_COUNT, 1 to NODE_COUNT) of real;
    type anat_t is array (integer range <>) of natural;
    subtype anat_nc_t is anat_t (1 to NODE_COUNT);

    subtype epsilon_type is anat_nc_t;
    subtype trv_del_type is anat_nc_t;
    subtype timing_config_t is anat_t(1 to 10);


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Functions definitions
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------


    ----------------------------------------------------------------------------
    -- Function called at the end of the test which evaluates results of test.
    -- If Error threshold is bigger than number of errors which ocurred in the
    -- test, test status will be updated to "passed", if not, test status will
    -- be updated to "failed". After the evaluation test simulation is finished.
    --
    -- Arguments:
    --  error_th        Error threshold.
    --  errors          Number of errors which occurred in the test
    --  status          Status to be updated
    ----------------------------------------------------------------------------
    procedure evaluate_test(
        constant error_th         : in    natural;
        constant errors           : in    natural;
        signal   status           : out   test_status_type
    );


    ----------------------------------------------------------------------------
    -- Set highest loglevel that should be shown.
    -- Log levels are connected to CAN_test as input signals and driven by
    -- VUnit from configuration.
    --
    -- Log levels are bound to Vunit Logging library:
    --  debug_l   - All logs are shown (even pass and trace)
    --  info_l    - info(), warning(), error(), failure() are shown
    --  warning_l - warning(), error(), failure() are shown
    --  error_l   - error(), failure() are shown
    --
    -- Default VUnit logger is used.
    --
    -- Arguments:
    --  log_level        Severity level which is set in current test.
    ----------------------------------------------------------------------------
    procedure set_log_level(
          constant log_level      : in    log_lvl_type
    );


    ---------------------------------------------------------------------------
    -- Configure error behaviour (Quit / Go On) of Vunit.
    --
    -- Arguments:
    --  log_level        Severity level which is set in current test.
    ----------------------------------------------------------------------------
    procedure set_error_beh(
          constant error_beh      : in    err_beh_type
    );


    ----------------------------------------------------------------------------
    -- Generates clock signal for the test with custom period, duty cycle and
    -- clock jitter.
    --
    -- Arguments:
    --  par             Precomputed parameters.
    --  out_clk         Generated clock.
    ----------------------------------------------------------------------------
    procedure generate_clock(
        constant par            : in    generate_clock_precomputed_t;
        signal   out_clk        : out   std_logic
    );

    -- deprecated compatibility wrapper (lower performance)
    procedure generate_clock(
        constant period         : in    natural;
        constant duty           : in    natural;
        constant epsilon_ppm    : in    natural;
        signal   out_clk        : out   std_logic
    );

    ----------------------------------------------------------------------------
    -- Combinatorial procedure, which infinitely generates clock signal.
    --
    -- Arguments:
    --  period          Period of generated clock in picoseconds.
    --  duty            Duty cycle of generated clock in percents.
    --  epsilon_ppm     Clock uncertainty (jitter) which is always added to the
    --                  default clock period.
    ----------------------------------------------------------------------------
    procedure clock_gen_proc(
        constant period         : in    natural;
        constant duty           : in    natural;
        constant epsilon_ppm    : in    natural;
        signal   out_clk        : out   std_logic
    );


    ----------------------------------------------------------------------------
    -- Combinatorial procedure, which infinitely generates timestamp.
    --
    -- Arguments:
    --  clk             Input clock.
    --  timestamp       Output timestamp counter.
    ----------------------------------------------------------------------------
    procedure timestamp_gen_proc(
        signal     clk           : in    std_logic;
        signal     timestamp     : out   std_logic_vector(63 downto 0);
        signal     ts_preset     : in    std_logic;
        signal     ts_preset_val : in    std_logic_vector(63 downto 0)
    );


    ----------------------------------------------------------------------------
    -- Precompute constants for clock signal generation for the test with
    -- custom period, duty cycle and clock jitter.
    --
    -- Arguments:
    --  period          Period of generated clock in picoseconds.
    --  duty            Duty cycle of generated clock in percents.
    --  epsilon_ppm     Clock uncertainty (jitter) which is always added to the
    --                  default clock period.
    -- Returns: precomputed params
    ----------------------------------------------------------------------------
    function precompute_clock(
        constant period         : in    natural;
        constant duty           : in    natural;
        constant epsilon_ppm    : in    natural
    ) return generate_clock_precomputed_t;


    ----------------------------------------------------------------------------
    -- Asserts reset signal to restart the operated circuit. Then waits until
    -- run condition signal becomes "true" and sets test status to "running" and
    -- sets error counter to 0. Finally, deasserts reset signal.
    --
    -- Arguments:
    --  res_n           Reset signal (active low).
    --  status          Test status to assert "running".
    --  run             Run condition to wait until comes true.
    --  error_ctr       Error counter to erase.
    ----------------------------------------------------------------------------
    procedure reset_test(
        signal res_n            : out   std_logic;
        signal status           : out   test_status_type;
        signal run              : in    boolean;
        signal error_ctr        : out   natural
    );


    ----------------------------------------------------------------------------
    -- Process error ocurrence in the test. This consists of following steps:
    --  1. Increment error counter
    --  2. If Error behaviour is "quit", assert Immediate exit flag to "true".
    --     Otherwise assert Immediate exit flag to "false".
    --
    -- Arguments:
    --  error_ctr       Error counter to increment.
    --  error_beh       Error behaviour to evaluate.
    --  exit_imm        Immediate exit flag to assert.
    ----------------------------------------------------------------------------
    procedure process_error(
        signal   error_ctr      : inout natural;
        constant error_beh      : in    err_beh_type;
        signal   exit_imm       : out   boolean
    );


    ----------------------------------------------------------------------------
    -- Print basic testbench information.
    --
    -- Arguments:
    --  iterations      Number of iterations to execute in test.
    --  log_level       Severity level which is set in actual frame.
    --  error_beh       Error behaviour of test.
    --  error_tol       Error tolerance of test.
    ----------------------------------------------------------------------------
    procedure print_test_info(
        constant iterations     : in    natural;
        constant log_level      : in    log_lvl_type;
        constant error_beh      : in    err_beh_type;
        constant error_tol      : in    natural
    );


    ----------------------------------------------------------------------------
    -- Sets random counter based on the seed that was given. To differentiate
    -- individual counters, fixed offset can be added.
    --
    -- Arguments:
    --  seed            Random seed input.
    --  offset          Offset of counter
    --  rand_ctr        Random counter to set.
    ----------------------------------------------------------------------------
    procedure apply_rand_seed(
        constant seed            : in   natural;
        constant offset          : in   natural;
        signal   rand_ctr        : out  natural range 0 to RAND_POOL_SIZE
    );


    -- variation of above, only returns the seed value
    impure function apply_rand_seed(
        constant seed            : in   natural;
        constant offset          : in   natural
    ) return natural;


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


    ----------------------------------------------------------------------------
    -- Generate simple triggering signals.
    -- Two signals are generated, "sync" and "sample". No bit rate switch is
    -- implemented in this procedure. There is random amount of clock cycles
    -- between sync and sample, but not less than "min_diff" and not more than
    -- 10 clock cycles! "min_diff" is intended for determining how many clock
    -- cycles each circuit needs between bit transmittion and reception!
    --
    -- Arguments:
    --  rand_ctr        Pointer to random pool.
    --  sync            Synchronisation trigger (for transmission)
    --  sample          Sampling trigger (for reception)
    --  clk_sys         Clock signal
    --  min_diff        Minimal gap (in clock cycles) between sync and sample
    --                  signals.
    ----------------------------------------------------------------------------
    procedure generate_simple_trig(
        signal   rand_ctr       : inout natural range 0 to RAND_POOL_SIZE;
        signal   sync           : out   std_logic;
        signal   sample         : out   std_logic;
        signal   clk_sys        : in    std_logic;
        variable min_diff       : in    natural
    );


    procedure generate_trig(
        signal    sync          : out   std_logic;
        signal    sample        : out   std_logic;
        signal    clk_sys       : in    std_logic;
        signal    seg1          : in    natural;
        signal    seg2          : in    natural
    );


 	----------------------------------------------------------------------------
    -- Stores CAN Frame to auxiliarly test memory. Used for creating SW models
	-- of Buffers. Frame is stored in the same order as in TXT Buffer and RX
	-- Buffer:
	--		1. FRAME_FORMAT
	--		2. IDENTIFIER
	--		3. TIMESTAMP_L
	--		4. TIMESTAMP_U
	--		5-20. DATA
    --
    -- Arguments:
    --  frame         	Frame to store.
    --  memory			Memory to store the frame into
	--	pointer			Pointer to the memory index where frame shouldbe stored
    ----------------------------------------------------------------------------
    procedure store_frame_to_test_mem(
        constant frame          :  in       SW_CAN_frame_type;
        signal   memory         :  out      test_mem_type;
        signal   pointer        :  inout    natural
    );


	----------------------------------------------------------------------------
    -- Read frame from auxiliarly test memory. Used for creating SW models of
	-- Buffers. Frame is expected to be in the same format as IN TXT Buffer and
	-- RX Buffer:
	--		1. FRAME_FORMAT
	--		2. IDENTIFIER
	--		3. TIMESTAMP_L
	--		4. TIMESTAMP_U
	--		5-20. DATA
	--
    -- Arguments:
    --  frame           Output variable in which CAN FD Frame sill be generated.
    --  memory          Memory from which the CAN frame should be read.
    --  pointer         Pointer to memory where CAN Frame is starting.
    ----------------------------------------------------------------------------
    procedure read_frame_from_test_mem(
        variable frame          :  inout    SW_CAN_frame_type;
        constant memory         :  in       test_mem_type;
        variable pointer        :  inout    natural
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


    -- Compare strings of possibly different length.
    -- The strings are considered equal if the longer one starts with
    -- the shorter one and the rest are spaces.
    function str_equal(a : string; b : string) return boolean;

    -- Pad string with spaces.
    impure function strtolen(n : natural; src : string) return string;
    

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Component declarations
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    component CAN_test is
    generic (
        constant seed         :in   natural := 0;
        constant data_path    :in   string  := " "
    );
    port (
        signal run            :in   boolean;
        signal iterations     :in   natural;
        signal log_level      :in   log_lvl_type;
        signal error_beh      :in   err_beh_type;
        signal error_tol      :in   natural;
        signal status         :out  test_status_type;
        signal errors         :out  natural
    );
    end component;

	component bit_generator is
    generic (
        constant prescaler  :        natural := 1;
        constant invert     :        boolean := true
    );
    port (
        signal bit_seq      : in     bit_seq_type;
        signal clk_sys      : in     std_logic;

        -- Control signals
        signal run          : in     boolean := false;
        signal done         : buffer boolean := false;

        signal output       : out    std_logic
    );
	end component;

end package;




--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Library implementation
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

package body can_unit_test_pkg is
    procedure evaluate_test(
        constant error_th         : in    natural;
        constant errors           : in    natural;
        signal   status           : out   test_status_type
    ) is
    begin

        -- Evaluate the test result
        if (errors < error_th or errors = error_th) then
            status    <= passed;
            info("Test result: SUCCES");
            wait for 200 ns;
            --std.env.stop(0);
        else
            status    <= failed;
            error("Test result: FAILURE");
            wait for 200 ns;
            --std.env.stop(1);
        end if;

    end procedure;


    function precompute_clock(
        constant period         : in    natural;
        constant duty           : in    natural;
        constant epsilon_ppm    : in    natural
    ) return generate_clock_precomputed_t is
        variable real_period    :       real;
        variable rand_nr        :       real;
        variable high_per       :       real;
        variable low_per        :       real;
        variable res            :       generate_clock_precomputed_t;
    begin
        -- If clock has uncertainty then it is constantly added to the clock.
        -- This covers the worst case.
        real_period   := real(period) +
                         (real(period * epsilon_ppm)) / 1000000.0;
        high_per      := ((real(duty)) * real_period) / 100.0;
        low_per       := ((real(100-duty)) * real_period) / 100.0;
        res.high_time := integer(high_per * 1000.0) * 1 ns;
        res.high_time := res.high_time / 1000000;
        res.low_time  := integer(low_per * 1000.0) * 1 ns;
        res.low_time  := res.low_time / 1000000;
        return res;
    end function;


    procedure generate_clock(
        constant par            : in    generate_clock_precomputed_t;
        signal   out_clk        : out   std_logic
    ) is
    begin
        out_clk       <= '1';
        wait for par.high_time;
        out_clk       <= '0';
        wait for par.low_time;
    end procedure;


    procedure generate_clock(
        constant period         : in    natural;
        constant duty           : in    natural;
        constant epsilon_ppm    : in    natural;
        signal   out_clk        : out   std_logic
    ) is
        constant par : generate_clock_precomputed_t := precompute_clock(period, duty, epsilon_ppm);
    begin
        generate_clock(par, out_clk);
    end procedure;


    procedure clock_gen_proc(
        constant period         : in    natural;
        constant duty           : in    natural;
        constant epsilon_ppm    : in    natural;
        signal   out_clk        : out   std_logic
    ) is
        constant par : generate_clock_precomputed_t := precompute_clock(period, duty, epsilon_ppm);
    begin
        loop
            generate_clock(par, out_clk);
        end loop;
    end procedure;


    procedure timestamp_gen_proc(
        signal     clk           : in    std_logic;
        signal     timestamp     : out   std_logic_vector(63 downto 0);
        signal     ts_preset     : in    std_logic;
        signal     ts_preset_val : in    std_logic_vector(63 downto 0)
    ) is
        variable ts_lo    : natural := 0;
        variable tmp      : natural := 0;
        variable ts_hi    : natural := 0;
    begin
        loop
            -- falling edge, because on rising edge, the value must stay stable
            -- even after `wait for 0 ns`
            wait until falling_edge(clk);
            tmp := ts_lo + 1;
            if tmp < ts_lo then
                ts_hi := ts_hi + 1;
            end if;
            ts_lo := tmp;
            timestamp <= std_logic_vector(  to_unsigned(ts_hi, 32)
                                          & to_unsigned(ts_lo, 32));
                                          
            if (ts_preset = '1') then
                ts_lo := to_integer(unsigned(ts_preset_val(31 downto 0)));
                ts_hi := to_integer(unsigned(ts_preset_val(63 downto 32)));
                wait for 0 ns;
            end if;
        end loop;
    end procedure;


    function CAN_add_unsigned(
        operator1               : in    std_logic_vector(11 downto 0);
        operator2               : in    std_logic_vector(11 downto 0)
    ) return std_logic_vector is
    begin
        return std_logic_vector(unsigned(operator1) + unsigned(operator2));
    end function;


    procedure reset_test(
        signal res_n            : out   std_logic;
        signal status           : out   test_status_type;
        signal run              : in    boolean;
        signal error_ctr        : out   natural
        )is
    begin
        res_n     <= '0';
        status    <= waiting;

        wait for 0 ns;
        if not run then
            wait until run;
        end if;

        wait for 100 ns; -- some time in reset

        res_n     <= '1';
        status    <= running;
        error_ctr <= 0;
        wait for 250 ns;
    end procedure;


    procedure process_error(
        signal   error_ctr      : inout natural;
        constant error_beh      : in    err_beh_type;
        signal   exit_imm       : out   boolean
    )is
    begin
        error_ctr       <= error_ctr + 1;

        if (error_beh = quit) then
            exit_imm    <= true;
        else
            exit_imm    <= false;
        end if;

        wait for 0 ns;
    end procedure;


    procedure set_log_level(
         constant log_level      : in    log_lvl_type
    ) is
    begin
        show_all(display_handler);
        if log_level >= debug_l then
            null;
        end if;

        if log_level >= info_l then
            hide(display_handler, pass);
            hide(display_handler, trace);
            null;
        end if;

        if log_level >= warning_l then
            hide(display_handler, debug);
            hide(display_handler, info);
        end if;

        if log_level >= error_l then
            hide(display_handler, warning);
        end if;
    end procedure;


    procedure set_error_beh(
          constant error_beh      : in    err_beh_type
    ) is
    begin
        if (error_beh = quit) then
            set_stop_level(error);
        else
            set_stop_level(failure);
        end if;
    end procedure;


    procedure print_test_info(
        constant iterations     : in    natural;
        constant log_level      : in    log_lvl_type;
        constant error_beh      : in    err_beh_type;
        constant error_tol      : in    natural
    )is
    begin
        info("Test info:");
        info("Number of iterations: " & integer'image(iterations));

        info("Log level: " & log_lvl_type'image(log_level));
        set_log_level(log_level);

        if (error_beh = go_on) then
            info("When error is detected test runs on");
        else
            info("When error is detected test quits");
        end if;
        set_error_beh(error_beh);

        info("Error tolerance: " & integer'image(error_tol));
    end;


    impure function apply_rand_seed(
        constant seed            : in   natural;
        constant offset          : in   natural
    ) return natural is
        variable tmp             :      natural;
    begin
        tmp := seed + offset;
        info("Random initialized with seed " & natural'image(seed));
        return tmp mod RAND_POOL_SIZE;
    end function;


    procedure apply_rand_seed(
        constant seed            : in   natural;
        constant offset          : in   natural;
        signal   rand_ctr        : out  natural range 0 to RAND_POOL_SIZE
    )is
    begin
        rand_ctr    <= apply_rand_seed(seed, offset);
        wait for 0 ns;
    end procedure;


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
			error("Invalid data length");
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
            check(id_in < 536870912,
                  "Extended Identifier: " & integer'image(id_in) &
                  " Exceeds the maximal value!");
            id_vect := std_logic_vector(to_unsigned(id_in, 29));

            id_out(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) :=
                id_vect(28 downto 18);
            id_out(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) :=
                id_vect(17 downto 0);
        else
            check(id_in < 2048,
                  "Base Identifier: " & integer'image(id_in) &
                  " Exceeds the maximal value!");
            id_vect := "000000000000000000" &
                           std_logic_vector(to_unsigned(id_in, 11));
            id_out(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) :=
                id_vect(10 downto 0);
            id_out(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) := (OTHERS => '0');
        end if;
    end procedure;


    procedure generate_simple_trig(
        signal   rand_ctr       : inout natural range 0 to RAND_POOL_SIZE;
        signal   sync           : out   std_logic;
        signal   sample         : out   std_logic;
        signal   clk_sys        : in    std_logic;
        variable min_diff       : in    natural
    )is
        variable diff           :       real;
        variable round          :       natural;
    begin
        wait until rising_edge(clk_sys);
        sample      <=  '0';
        wait until rising_edge(clk_sys);
        wait until rising_edge(clk_sys);
        wait until rising_edge(clk_sys);
        sync        <=  '1';

        rand_real_v(rand_ctr, diff);
        diff := 10.0 * diff;
        if (integer(diff) < min_diff) then
          diff := real(min_diff);
        end if;
        round := integer(diff);

        for i in 0 to round loop
            wait until rising_edge(clk_sys);

            if (i = 0) then
                sync <= '0';
            end if;

            if (i = round) then
                sample <= '1';
            end if;
        end loop;
    end procedure;


    procedure generate_trig(
        signal    sync            : out     std_logic;
        signal    sample          : out     std_logic;
        signal    clk_sys         : in      std_logic;
        signal    seg1            : in      natural;
        signal    seg2            : in      natural
    )is
    begin
        wait until rising_edge(clk_sys);
        sync <= '1';
        wait until rising_edge(clk_sys);
        sync <= '0';

        for i in 1 to seg1 - 1 loop
            wait until rising_edge(clk_sys);
        end loop;

        sample <= '1';
        wait until rising_edge(clk_sys);
        sample <= '0';

        for i in 1 to seg2 - 1 loop
            wait until rising_edge(clk_sys);
        end loop;
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
            error("Unsupported CAN frame type");
        end if;
    end procedure;


    procedure CAN_print_frame(
        constant frame          : in    SW_CAN_frame_type
    )is
        variable data_byte      :       std_logic_vector(7 downto 0);
        variable str_msg        :       string(1 to 400) := (OTHERS => ' ');
        variable str_len        :       natural := 0;
    begin

        info("*************************************************************");

        -- Identifier
        info("ID : 0x" & 
            to_hstring(std_logic_vector(to_unsigned(frame.identifier, 32))));

        -- Metadata
        info("DLC: " & to_hstring(frame.dlc) & ", Data length:" &
              to_string(frame.data_length));

        if (frame.rtr = RTR_FRAME) then
            info("RTR Frame");
        end if;

        if (frame.ident_type = BASE) then
            info("BASE identifier");
        else
            info("EXTENDED identifier");
        end if;

        if (frame.frame_format = NORMAL_CAN) then
            info("CAN 2.0 frame");
        else
            info("CAN FD frame");
        end if;

        info("RWCNT (read word count): " &
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
            info(str_msg(1 to str_len));
        end if;
        
        info("*************************************************************");
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

  
        info(str_msg);
    end procedure;


    procedure store_frame_to_test_mem(
        constant frame          :  in       SW_CAN_frame_type;
        signal   memory         :  out      test_mem_type;
        signal   pointer        :  inout    natural
    )is
        variable ident_vect     :           std_logic_vector(28 downto 0);
    begin
        -- Frame format word
        memory(pointer) <= "0000000000000000" &
                            std_logic_vector(to_unsigned(frame.rwcnt, 5)) &
                            '0' & --We dont store ESI bit
                            frame.brs &
                            '1' &
                            frame.frame_format &
                            frame.ident_type &
                            frame.rtr &
                            '0' &
                            frame.dlc;

        -- Identifier
        if (frame.ident_type = BASE and frame.identifier > 2047) then
            error("Incorrect BASE Identifier length");

        elsif (frame.ident_type = EXTENDED and
               frame.identifier > 536870911)
        then
            error("Incorrect EXTENDED Identifier length");
        end if;

        ident_vect := std_logic_vector(to_unsigned(frame.identifier, 29));
        memory(pointer + 1)(31 downto 29) <= "000";

        -- Base Identifier
        if (frame.ident_type = BASE) then
            memory(pointer + 1)(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) <=
                                    ident_vect(10 downto 0);
            memory(pointer + 1)(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) <=
                                    (OTHERS => '0');
        -- Extended Identifier
        elsif (frame.ident_type = EXTENDED) then
            memory(pointer + 1)(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) <=
                                    ident_vect(17 downto 0);
            memory(pointer + 1)(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) <=
                                    ident_vect(28 downto 18);
        else
            error("Unsupported Identifier type");
        end if;

        -- Timestamp
        memory(pointer + 2) <= frame.timestamp(63 downto 32);
        memory(pointer + 3) <= frame.timestamp(31 downto 0);

        pointer           <= pointer + 4;
        wait for 0 ns;

        -- Data words
        if ((frame.rtr = '0' or frame.frame_format = '1') and
            (frame.data_length /= 0))
        then
            for i in 0 to ((frame.data_length - 1) / 4) loop
                memory(pointer + i) <= frame.data((i * 4) + 3) &
                                       frame.data((i * 4) + 2) &
                                       frame.data((i * 4) + 1) &
                                       frame.data((i * 4));
                wait for 0 ns;
            end loop;

            pointer <= pointer + ((frame.data_length - 1) / 4) + 1;
            wait for 0 ns;
        end if;
    end procedure;

    procedure CAN_generate_frame(
        signal   rand_ctr       : inout natural range 0 to RAND_POOL_SIZE;
        variable frame          : inout SW_CAN_frame_type
    )is
        variable rand_value     : real := 0.0;
        variable aux            : std_logic_vector(28 downto 0);
        variable data_byte      : std_logic_vector(7 downto 0);
    begin

        rand_logic_v(rand_ctr, frame.ident_type, 0.5);
        rand_logic_v(rand_ctr, frame.frame_format, 0.5);
        rand_logic_v(rand_ctr, frame.rtr, 0.5);
        rand_logic_v(rand_ctr, frame.brs, 0.5);
        rand_logic_vect_v(rand_ctr, frame.dlc, 0.3);

        rand_real_v(rand_ctr, rand_value);

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
                rand_logic_vect_v(rand_ctr, data_byte, 0.5);
                frame.data(i) := data_byte;
            end loop;
        end if;

    end procedure;

    

    procedure read_frame_from_test_mem(
        variable frame          :  inout    SW_CAN_frame_type;
        constant memory         :  in       test_mem_type;
        variable pointer        :  inout    natural
    ) is
        variable aux_vect       :           std_logic_vector(28 downto 0);
    begin
        -- Erase some unnecessary stuff
        frame.data          := (OTHERS => (OTHERS => '0'));
        frame.identifier    := 0;

        -- Frame format
        frame.dlc           := memory(pointer)(3 downto 0);
        frame.rtr           := memory(pointer)(5);
        frame.ident_type    := memory(pointer)(6);
        frame.frame_format  := memory(pointer)(7);
        frame.brs           := memory(pointer)(9);
        decode_dlc(frame.dlc, frame.data_length);
        frame.rwcnt         :=
          to_integer(unsigned(memory(pointer)(15 downto 11)));

        pointer             := pointer + 1;

        -- Identifier
        if (frame.ident_type = BASE) then
            aux_vect        := "000000000000000000" & memory(pointer)
                                (IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L);

        elsif (frame.ident_type = EXTENDED) then
            aux_vect        := memory(pointer)
                                (IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) &
                               memory(pointer)
                                (IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L);
        else
            error("Unsupported Identifier type");
        end if;

        frame.identifier  := to_integer(unsigned(aux_vect));
        pointer           := pointer + 1;

        -- Timestamp
        frame.timestamp     := memory(pointer) &
                               memory(pointer + 1);
        pointer             := pointer + 2;

        -- Data words
        if ((frame.rtr = '0' or frame.frame_format = '1') and
            frame.data_length /= 0)
        then
            for i in 0 to ((frame.data_length - 1) / 4) loop
                frame.data((i * 4) + 3) :=
                    memory(pointer)(31 downto 24);

                frame.data((i * 4) + 2) :=
                    memory(pointer)(23 downto 16);

                frame.data((i * 4) + 1) :=
                    memory(pointer)(15 downto 8);

                frame.data((i * 4)) :=
                    memory(pointer)(7 downto 0);

                pointer := pointer + 1;
            end loop;
        end if;
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


    function str_equal(a : string; b : string) return boolean is
        constant len : natural := MINIMUM(a'length, b'length);
        constant atail : string(a'left+len to a'right) := (others => ' ');
        constant btail : string(b'left+len to b'right) := (others => ' ');
    begin
        return     a(a'left to a'left+len-1) = b(b'left to b'left+len-1)
               and a(a'left+len to a'right) = atail
               and b(b'left+len to b'right) = btail;
    end function str_equal;

    impure function strtolen(n : natural; src : string) return string is
        variable s : string(1 to n) := (others => ' ');
    begin
        check(src'length <= n, "String too long.");
        s(src'range) := src;
        return s;
    end function strtolen;

end package body;


library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.can_unit_test_pkg.all;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

library vunit_lib;
context vunit_lib.vunit_context;


--------------------------------------------------------------------------------
-- Main test entity. Each test implements architecture of this entity.
--------------------------------------------------------------------------------
entity CAN_test is
    generic (
        constant seed           :in     natural := 0;

        -- Used only for "reference" test
        constant data_path      :in     string  :=
                    "test/reference/data_sets/log_500Kb_2Mb_80p_1K_samples_1"
    );
    port (

        -- Input trigger, test starts running when true
        signal run              :in     boolean;

        -- Number of iterations that test should do
        signal iterations       :in     natural;

        -- Logging level, severity which should be shown
        signal log_level        :in     log_lvl_type;

        -- Test behaviour when error occurs: Quit, or Go on
        signal error_beh        :in     err_beh_type;

        -- Error tolerance, error counter should not
        -- exceed this value in order for the test to pass
        signal error_tol        :in     natural;

    -- Status of the test
        signal status           :out    test_status_type;

        -- Amount of errors which appeared in the test
        signal errors           :out    natural
    );

    --Internal test signals
    signal error_ctr           :       natural := 0;
    signal loop_ctr            :       natural := 0;
    signal exit_imm            :       boolean := false;
    signal rand_ctr            :       natural range 0 to 3800 := 0;

end entity;
