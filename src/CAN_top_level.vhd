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
--  Enity encapsulating all functionality of CAN FD node.
--  Instances:
--      1x Memory registers
--      1x Interrupt manager
--      1x Prescaler (v3)
--      1x Bus synchronizes
--      1x Event Logger
--      1x Rx buffer
--      2x TXT buffer
--      1x Tx Arbitrator
--      1x Acceptance filters
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--    22.6.2016   1. Added rec_esi signal for error state propagation into
--                   RX buffer.
--                2. Added explicit architecture selection for each component
--                   (RTL)
--    24.8.2016   Added "use_logger" generic to the registers module.
--    28.11.2017  Added "rst_sync_comp" reset synchroniser.
--    30.11.2017  Changed TXT buffer to registers interface. The user is now
--                directly accessing the buffer by avalon access.
--    10.12.2017  Added "tx_time_sup" to enable/disable transmission at given
--                time and save some LUTs.
--    12.12.2017  Renamed "registers" entity to  "canfd_registers" to avoid
--                possible name conflicts.
--    20.12.2017  Removed obsolete "tran_data_in" signal.
--     10.2.2017  Removed "useFDsize" generic. When TX Buffer goes completely
--                to the Dual port RAM, there is no need to save memory
--                anymore.
--     15.2.2018  Added generic amount of TXT Buffers and support for TXT
--                buffer FSM, HW commands and SW commands.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;

entity CAN_top_level is
    generic(
        -- Whenever event logger should be synthetised
        constant use_logger     : boolean                := true;

        -- Receive Buffer size
        constant rx_buffer_size : natural range 32 to 4096 := 128;

        -- Whenever internal synchroniser chain should be used for incoming bus
        -- signals. Dont turn off unless external synchronisation chain is put on
        -- input of FPGA by synthetiser
        constant use_sync       : boolean                := true;

        -- ID (bits  19-16 of adress)
        constant ID             : natural range 0 to 15  := 1;

        -- Optional synthesis of received message filters
        -- By default the behaviour is as if all the filters are present
        constant sup_filtA      : boolean                := true;
        constant sup_filtB      : boolean                := true;
        constant sup_filtC      : boolean                := true;
        constant sup_range      : boolean                := true;
        constant logger_size    : natural range 0 to 512 := 8
    );
    port(
        --------------------------
        -- System clock and reset
        --------------------------
        signal clk_sys : in std_logic;
        signal res_n   : in std_logic;

        ---------------------
        -- Memory interface
        ---------------------
        signal data_in  : in  std_logic_vector(31 downto 0);
        signal data_out : out std_logic_vector(31 downto 0);
        signal adress   : in  std_logic_vector(23 downto 0);
        signal scs      : in  std_logic;    --Chip select
        signal srd      : in  std_logic;    --Serial read
        signal swr      : in  std_logic;    --Serial write
        signal sbe      : in  std_logic_vector(3 downto 0);
        --Note: This bus is Avalon compatible!

        --------------------
        -- Interrupt output
        --------------------
        signal int : out std_logic;

        -------------------
        -- CAN Bus output
        -------------------
        signal CAN_tx : out std_logic;
        signal CAN_rx : in  std_logic;

        ---------------------------
        -- Synchronisation signals
        ---------------------------
        --Time Quantum clocks possible to be used for synchronisation
        signal time_quanta_clk : out std_logic;

        -----------------------------------
        -- Internal signals for testbenches
        -----------------------------------
        -- synthesis translate_off
        signal drv_bus_o    : out std_logic_vector(1023 downto 0);
        signal stat_bus_o   : out std_logic_vector(511 downto 0);
        -- synthesis translate_on

        -------------------------------------------
        -- Timestamp value for time based messages
        -------------------------------------------
        signal timestamp : in std_logic_vector(63 downto 0)
    );
end entity CAN_top_level;

architecture rtl of CAN_top_level is

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    ---- Internal signals
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    -- Common control signals
    ----------------------------------------------------------------------------

    -- Overal reset (External+Reset by memory access)
    signal res_n_int  : std_logic;

    signal res_n_sync : std_logic;  -- Synchronised reset
    signal drv_bus    : std_logic_vector(1023 downto 0);
    signal stat_bus   : std_logic_vector(511 downto 0);

    --Interrupt signals
    signal int_vector :   std_logic_vector(INT_COUNT - 1 downto 0);
    signal int_ena    :   std_logic_vector(INT_COUNT - 1 downto 0);
    signal int_mask   :   std_logic_vector(INT_COUNT - 1 downto 0);

    ----------------------------------------------------------------------------
    -- Registers <--> RX Buffer Interface
    ----------------------------------------------------------------------------

    --Actually loaded data for reading
    signal rx_read_buff         : std_logic_vector(31 downto 0);

    --Actual size of synthetised message buffer (in 32 bit words)
    signal rx_buf_size          : std_logic_vector(12 downto 0);

    --Signal whenever buffer is full
    signal rx_full              : std_logic;

    --Signal whenever buffer is empty
    signal rx_empty             : std_logic;

    --Number of messaged stored in recieve buffer
    signal rx_message_count     : std_logic_vector(10 downto 0);

    --Number of free 32 bit wide ''windows''
    signal rx_mem_free          : std_logic_vector(12 downto 0);

    --Position of read pointer
    signal rx_read_pointer_pos  : std_logic_vector(11 downto 0);

    --Position of write pointer
    signal rx_write_pointer_pos : std_logic_vector(11 downto 0);

    --Message was discarded since Memory is full
    signal rx_message_disc      : std_logic;

    --Some data were discarded, register
    signal rx_data_overrun      : std_logic;

    -- Validated commands after Message filter
    signal rx_store_metadata_valid  : std_logic;
    signal rx_abort_valid           : std_logic;
    signal rx_store_data_valid      : std_logic;
    signal rec_message_store        : std_logic;

    ----------------------------------------------------------------------------
    -- Registers <--> TX Buffer, TXT Buffer
    ----------------------------------------------------------------------------

    --Data, Address and chip select into the RAM of TXT Buffer
    signal tran_data            : std_logic_vector(31 downto 0);
    signal tran_addr            : std_logic_vector(4 downto 0);
    signal tran_cs              : std_logic_vector(TXT_BUFFER_COUNT - 1 downto 0);

    -- Finite state machine types for TXT Buffer
    signal txtb_state           : txtb_state_type;

    -- Software commands + buffer indices that should be activated
    signal txt_sw_cmd           :  txt_sw_cmd_type;
    signal txt_buf_cmd_index    :  std_logic_vector(TXT_BUFFER_COUNT - 1 downto 0);
    signal txt_buf_prior        :  txtb_priorities_type;

    -- Indicates that TXT Buffer has changed and that Retrransmitt counter
    -- should be erased by Protocol control.
    signal txtb_changed         :  std_logic;

    ----------------------------------------------------------------------------
    -- Registers <--> event logger
    ----------------------------------------------------------------------------

    signal loger_act_data    : std_logic_vector(63 downto 0);
    signal log_write_pointer : std_logic_vector(7 downto 0);
    signal log_read_pointer  : std_logic_vector(7 downto 0);
    signal log_size          : std_logic_vector(7 downto 0);
    signal log_state_out     : logger_state_type;


    ----------------------------------------------------------------------------
    --TX Arbitrator <--> TX Buffer, TXT Buffer
    ----------------------------------------------------------------------------

    signal txt_hw_cmd_buf_index : natural range 0 to TXT_BUFFER_COUNT - 1;
    signal txt_buf_ready        : std_logic_vector(TXT_BUFFER_COUNT - 1 downto 0);

    -- Frames in TXT buffers on output - Data(addressed), Metadata (paralell)
    signal txt_word             : txtb_output_type;


    ----------------------------------------------------------------------------
    -- TX Arbitrator <--> CAN Core
    ----------------------------------------------------------------------------

    --TX Message data
    signal tran_data_out        : std_logic_vector(31 downto 0);

    --TX Data length code
    signal tran_dlc_out         : std_logic_vector(3 downto 0);

    --TX is remote frame
    signal tran_is_rtr          : std_logic;

    --TX Identifier type (0-Basic,1-Extended);
    signal tran_ident_type_out  : std_logic;

    --TX Frame type
    signal tran_frame_type_out  : std_logic;

    --Bit rate shift for CAN FD frames
    signal tran_brs_out         : std_logic;

    --Signal for CAN Core that frame on the output is valid and can be
    --stored for transmitting
    signal tran_frame_valid_out : std_logic;

    -- Hardware commands to TXT Buffer from Protocol control
    signal txt_hw_cmd           : txt_hw_cmd_type;

    -- TXT Buffer HW CMD Interrupt activated on TXT Buffer
    signal txt_hw_cmd_int       : std_logic_vector(TXT_BUFFER_COUNT - 1
                                                    downto 0);

    -- Hardware command index set by TX Arbitrator based on the current
    -- internal state
    signal txt_hw_cmd_index     : natural range 0 to TXT_BUFFER_COUNT - 1;

    --Pointer to TXT buffer memory (from TX Arbitrator)
    signal txt_buf_ptr          : natural range 0 to 19;

    -- Pointer to TXT Buffer memory (from CAN Core)
    signal txtb_core_pointer    : natural range 0 to 19;

    -- Transition of fault confinement state to bus-off
    signal bus_off_start        : std_logic;

    ----------------------------------------------------------------------------
    --RX Buffer <--> CAN Core
    ----------------------------------------------------------------------------

    --Message Identifier
    signal rec_ident_in      : std_logic_vector(28 downto 0);

    --Data length code
    signal rec_dlc_in        : std_logic_vector(3 downto 0);

    --Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_ident_type_in : std_logic;

    --Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_frame_type_in : std_logic;

    --Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_is_rtr        : std_logic;

    --Frame is received properly (can be committed to RX Buffer)
    signal rec_message_valid : std_logic;

    --Whenever frame was recieved with BIT Rate shift
    signal rec_brs           : std_logic;

    -- Received Error state indicator
    signal rec_esi           : std_logic;

    -- Signals start of frame for storing timestamp
    signal sof_pulse         : std_logic;

    -- Metadata can be stored to the RX Buffer
    signal rx_store_metadata : std_logic;

    -- Data word can be stored in RX Buffer
    signal rx_store_data     : std_logic;

    -- Data word to be stored
    signal rx_store_data_word : std_logic_vector(31 downto 0);

    -- Abort storing of RX Frame (in case of Error frame)
    signal rx_abort           : std_logic;


    ----------------------------------------------------------------------------
    -- RX Buffer <--> Message filters
    ----------------------------------------------------------------------------

    --Signal whenever identifier matches the filter identifiers
    signal out_ident_valid : std_logic;


    ----------------------------------------------------------------------------
    -- Interrupt manager <--> CAN Core
    ----------------------------------------------------------------------------

    --Valid Error appeared for interrupt
    signal error_valid           : std_logic;

    --Error pasive /Error acitve functionality changed
    signal error_passive_changed : std_logic;

    --Error warning limit reached
    signal error_warning_limit   : std_logic;

    --Arbitration was lost input
    signal arbitration_lost      : std_logic;

    --Wake up appeared
    signal wake_up_valid         : std_logic;

    --Message stored in CAN Core was sucessfully transmitted
    signal tx_finished           : std_logic;

    --Bit Rate Was Shifted
    signal br_shifted            : std_logic;

    --Event logging finsihed
    signal loger_finished : std_logic;


    ----------------------------------------------------------------------------
    -- Prescaler <--> CAN Core
    ----------------------------------------------------------------------------

    --Edge for synchronisation
    signal sync_edge      : std_logic;

    --Protocol control state
    signal OP_State       : oper_mode_type;

    --Time quantum clock - Nominal bit time
    signal clk_tq_nbt : std_logic;

    --Bit time - Nominal bit time
    signal clk_tq_dbt : std_logic;

    --Sample signal for nominal bit time
    signal sample_nbt       : std_logic;

    --Sample signal of data bit time
    signal sample_dbt       : std_logic;

    --Delay sample signals by 1 or 2 clock cycle
    signal sample_nbt_del_1 : std_logic;
    signal sample_dbt_del_1 : std_logic;
    signal sample_nbt_del_2 : std_logic;
    signal sample_dbt_del_2 : std_logic;

    -- Transmitt signals and delayed transmitt signals by 1 clock cycle
    signal sync_nbt       : std_logic;
    signal sync_dbt       : std_logic;
    signal sync_nbt_del_1 : std_logic;
    signal sync_dbt_del_1 : std_logic;

    signal sp_control   : std_logic_vector(1 downto 0);
    signal sync_control : std_logic_vector(1 downto 0);

    signal bt_FSM_out : bit_time_type;

    --Validated hard synchronisation edge to start Protocol control FSM
    signal hard_sync_edge_valid : std_logic;
    --Note: Sync edge from busSync.vhd cant be used! If it comes during sample
    -- nbt, sequence it causes errors! It needs to be strictly before or
    -- strictly after this sequence!!!


    ----------------------------------------------------------------------------
    -- Bus Synchroniser Interface
    ----------------------------------------------------------------------------

    --Transcieve data value
    signal data_tx           : std_logic;

    --Recieved data value
    signal data_rx           : std_logic;

    --Clear the Shift register at the  beginning of Data Phase!!!
    signal ssp_reset         : std_logic;

    --Calibration command for transciever delay compenstation (counter)
    signal trv_delay_calib   : std_logic;

    --Bit error with secondary sampling transciever!
    signal bit_Error_sec_sam : std_logic;

    --Secondary sample signal
    signal sample_sec       : std_logic;

    --Bit destuffing trigger for secondary sample point
    signal sample_sec_del_1 : std_logic;

    --Rec trig for secondary sample point
    signal sample_sec_del_2 : std_logic;

    -- Transceiver delay output
    signal trv_delay_out : std_logic_vector(15 downto 0);


    ----------------------------------------------------------------------------
    -- Defining explicit architectures for used entites
    ----------------------------------------------------------------------------
    for memory_registers_comp   : memory_registers use entity work.memory_registers(rtl);
    for rx_buffer_comp          : rx_buffer use entity work.rx_buffer(rtl);
    for tx_arbitrator_comp      : tx_arbitrator use entity work.tx_arbitrator(rtl);
    for frame_filters_comp      : frame_filters use entity work.frame_filters(rtl);
    for int_manager_comp        : int_manager use entity work.int_manager(rtl);
    for can_core_comp           : can_core use entity work.can_core(rtl);
    for prescaler_comp          : prescaler use entity work.prescaler(rtl);
    for bus_sampling_comp       : bus_sampling use entity work.bus_sampling(rtl);
    for rst_sync_comp           : rst_sync use entity work.rst_sync(rtl);

begin

    -- synthesis translate_off
    drv_bus_o   <= drv_bus;
    stat_bus_o  <= stat_bus;
    -- synthesis translate_on

    rst_sync_comp : rst_sync
    generic map(
        reset_polarity  => ACT_RESET
    )
    port map(
        clk             => clk_sys,
        arst            => res_n,
        rst             => res_n_sync
    );

    memory_registers_comp : memory_registers
    generic map(
        compType        => CAN_COMPONENT_TYPE,
        use_logger      => use_logger,
        sup_filtA       => sup_filtA,
        sup_filtB       => sup_filtB,
        sup_filtC       => sup_filtC,
        sup_range       => sup_range,
        sup_be          => true,
        buf_count       => TXT_BUFFER_COUNT,
        ID              => ID,
        DEVICE_ID       => CTU_CAN_FD_ID,
        VERSION_MINOR   => x"01",
        VERSION_MAJOR   => x"02"
    )
    port map(
        clk_sys              => clk_sys,
        res_n                => res_n_sync,
        res_out              => res_n_int,
        data_in              => data_in,
        data_out             => data_out,
        adress               => adress,
        scs                  => scs,
        srd                  => srd,
        swr                  => swr,
        sbe                  => sbe,
        drv_bus              => drv_bus,
        stat_bus             => stat_bus,
        rx_read_buff         => rx_read_buff,
        rx_buf_size          => rx_buf_size,
        rx_full              => rx_full,
        rx_empty             => rx_empty,
        rx_message_count     => rx_message_count,
        rx_mem_free          => rx_mem_free,
        rx_read_pointer_pos  => rx_read_pointer_pos,
        rx_write_pointer_pos => rx_write_pointer_pos,
        rx_data_overrun      => rx_data_overrun,
        tran_data            => tran_data,
        tran_addr            => tran_addr,
        txtb_cs              => tran_cs,
        txtb_state           => txtb_state,
        txt_sw_cmd           => txt_sw_cmd,
        txt_buf_cmd_index    => txt_buf_cmd_index,
        txt_buf_prior_out    => txt_buf_prior,
        int_vector           => int_vector,
        int_ena              => int_ena,
        int_mask             => int_mask,
        trv_delay_out        => trv_delay_out,
        loger_act_data       => loger_act_data,
        log_write_pointer    => log_write_pointer,
        log_read_pointer     => log_read_pointer,
        log_size             => log_size,
        log_state_out        => log_state_out
    );


    rx_buffer_comp : rx_buffer
    generic map(
        buff_size            => rx_buffer_size
    )
    port map(
        clk_sys              => clk_sys,
        res_n                => res_n_int,
        rec_ident_in         => rec_ident_in,
        rec_dlc_in           => rec_dlc_in,
        rec_ident_type_in    => rec_ident_type_in,
        rec_frame_type_in    => rec_frame_type_in,
        rec_is_rtr           => rec_is_rtr,
        rec_brs              => rec_brs,
        rec_esi              => rec_esi,

        store_metadata       => rx_store_metadata_valid,
        store_data           => rx_store_data_valid,
        store_data_word      => rx_store_data_word,
        rec_message_valid    => rec_message_store,
        rec_abort            => rx_abort_valid,

        sof_pulse            => sof_pulse,
        rx_buf_size          => rx_buf_size,
        rx_full              => rx_full,
        rx_empty             => rx_empty,
        rx_message_count     => rx_message_count,
        rx_mem_free          => rx_mem_free,
        rx_read_pointer_pos  => rx_read_pointer_pos,
        rx_write_pointer_pos => rx_write_pointer_pos,
        rx_data_overrun      => rx_data_overrun,
        timestamp            => timestamp,
        rx_read_buff         => rx_read_buff,
        drv_bus              => drv_bus
    );



    txt_buf_comp_gen : for i in 0 to TXT_BUFFER_COUNT - 1 generate
        txt_buffer_comp : txt_buffer
        generic map(
            buf_count             => TXT_BUFFER_COUNT,
            ID                    => i
        )
        port map(
            clk_sys               => clk_sys,
            res_n                 => res_n_int,
            tran_data             => tran_data,
            tran_addr             => tran_addr,
            tran_cs               => tran_cs(i),
            txt_sw_cmd            => txt_sw_cmd,
            txt_sw_buf_cmd_index  => txt_buf_cmd_index,
            txtb_state            => txtb_state(i),
            txt_hw_cmd            => txt_hw_cmd,
            txt_hw_cmd_int        => txt_hw_cmd_int(i),
            txt_hw_cmd_buf_index  => txt_hw_cmd_buf_index,
            bus_off_start         => bus_off_start,
            txt_word              => txt_word(i),
            txt_addr              => txt_buf_ptr,
            txt_buf_ready         => txt_buf_ready(i)
        );
    end generate;


    tx_arbitrator_comp : tx_arbitrator
    generic map(
        buf_count               => TXT_BUFFER_COUNT
    )
    port map(
        clk_sys                => clk_sys,
        res_n                  => res_n_int,
        txt_buf_in             => txt_word,
        txt_buf_ready          => txt_buf_ready,
        txtb_ptr               => txt_buf_ptr,
        tran_data_word_out     => tran_data_out,
        tran_dlc_out           => tran_dlc_out,
        tran_is_rtr            => tran_is_rtr,
        tran_ident_type_out    => tran_ident_type_out,
        tran_frame_type_out    => tran_frame_type_out,
        tran_brs_out           => tran_brs_out,
        tran_frame_valid_out   => tran_frame_valid_out,
        txt_hw_cmd             => txt_hw_cmd,
        txtb_changed           => txtb_changed,
        txt_hw_cmd_buf_index   => txt_hw_cmd_buf_index,
        txtb_core_pointer      => txtb_core_pointer,
        drv_bus                => drv_bus,
        txt_buf_prio           => txt_buf_prior,
        timestamp              => timestamp
    );

    frame_filters_comp : frame_filters
    generic map(
        sup_filtA       => sup_filtA,
        sup_filtB       => sup_filtB,
        sup_filtC       => sup_filtC,
        sup_range       => sup_range
    )
    port map(
        clk_sys         => clk_sys,
        res_n           => res_n_int,
        rec_ident_in    => rec_ident_in,
        ident_type      => rec_ident_type_in,
        frame_type      => rec_frame_type_in,

        ------------------------------------------------------------------------
        -- All commands on RX Buffer are generated by Protocol control when
        -- ID is already received (between CONTROL and EOF), thus ID
        -- comparison can be enabled always!
        ------------------------------------------------------------------------
        rec_ident_valid => '1',

        drv_bus         => drv_bus,
        out_ident_valid => out_ident_valid
    );

    ----------------------------------------------------------------------------
    -- Commands for storing Frame to RX Buffer are valid only when it passed
    -- message filter
    ----------------------------------------------------------------------------
    rx_store_metadata_valid <= rx_store_metadata and out_ident_valid;
    rx_abort_valid          <= rx_abort and out_ident_valid;
    rx_store_data_valid     <= rx_store_data and out_ident_valid;
    rec_message_store       <= rec_message_valid and out_ident_valid;

    int_manager_comp : int_manager
    generic map(
        int_count             => INT_COUNT
    )
    port map(
        clk_sys               => clk_sys,
        res_n                 => res_n_int,
        error_valid           => error_valid,
        error_passive_changed => error_passive_changed,
        error_warning_limit   => error_warning_limit,
        arbitration_lost      => arbitration_lost,
        tx_finished           => tx_finished,
        br_shifted            => br_shifted,
        rx_message_disc       => rx_data_overrun,
        rec_message_valid     => rec_message_valid,
        rx_full               => rx_full,
        rx_empty              => rx_empty,
        txt_hw_cmd_int        => txt_hw_cmd_int,
        loger_finished        => loger_finished,
        drv_bus               => drv_bus,
        int_out               => int,
        int_vector            => int_vector,
        int_ena               => int_ena,
        int_mask              => int_mask
    );

    can_core_comp : can_core
    port map(
        clk_sys               => clk_sys,
        res_n                 => res_n_int,
        drv_bus               => drv_bus,
        stat_bus              => stat_bus,
        tran_data_in          => tran_data_out,
        tran_dlc_in           => tran_dlc_out,
        tran_is_rtr_in        => tran_is_rtr,
        tran_ident_type_in    => tran_ident_type_out,
        tran_frame_type_in    => tran_frame_type_out,
        tran_brs_in           => tran_brs_out,
        tran_frame_valid_in   => tran_frame_valid_out,
        txt_hw_cmd            => txt_hw_cmd,
        txtb_changed          => txtb_changed,
        txt_buf_ptr           => txtb_core_pointer,
        rec_ident_out         => rec_ident_in,
        rec_dlc_out           => rec_dlc_in,
        rec_ident_type_out    => rec_ident_type_in,
        rec_frame_type_out    => rec_frame_type_in,
        rec_is_rtr_out        => rec_is_rtr,
        rec_brs_out           => rec_brs,
        rec_esi_out           => rec_esi,
        rec_message_valid_out => rec_message_valid,

        store_metadata        =>  rx_store_metadata,
        rec_abort             =>  rx_abort,
        store_data            =>  rx_store_data,
        store_data_word       =>  rx_store_data_word,

        arbitration_lost_out  => arbitration_lost,
        tx_finished           => tx_finished,
        br_shifted            => br_shifted,
        error_valid           => error_valid,
        error_passive_changed => error_passive_changed,
        error_warning_limit   => error_warning_limit,
        sample_nbt_del_2      => sample_nbt_del_2,
        sample_dbt_del_2      => sample_dbt_del_2,
        sample_nbt_del_1      => sample_nbt_del_1,
        sample_dbt_del_1      => sample_dbt_del_1,
        sync_nbt              => sync_nbt,
        sync_dbt              => sync_dbt,
        sync_nbt_del_1        => sync_nbt_del_1,
        sync_dbt_del_1        => sync_dbt_del_1,
        sample_sec            => sample_sec,
        sample_sec_del_1      => sample_sec_del_1,
        sample_sec_del_2      => sample_sec_del_2,
        sync_control          => sync_control,
        data_rx               => data_rx,
        data_tx               => data_tx,
        timestamp             => timestamp,
        sp_control            => sp_control,
        ssp_reset             => ssp_reset,
        trv_delay_calib       => trv_delay_calib,
        hard_sync_edge        => hard_sync_edge_valid,
        bit_Error_sec_sam     => bit_Error_sec_sam,
        bus_off_start         => bus_off_start,
        sof_pulse             => sof_pulse
    );

    prescaler_comp : prescaler
    port map(
        clk_sys              => clk_sys,
        res_n                => res_n_int,
        OP_State             => OP_State,
        sync_edge            => sync_edge,
        drv_bus              => drv_bus,
        clk_tq_nbt           => clk_tq_nbt,
        clk_tq_dbt           => clk_tq_dbt,
        sample_nbt           => sample_nbt,
        sample_dbt           => sample_dbt,
        bt_FSM_out           => bt_FSM_out,
        sample_nbt_del_1     => sample_nbt_del_1,
        sample_dbt_del_1     => sample_dbt_del_1,
        sample_nbt_del_2     => sample_nbt_del_2,
        sample_dbt_del_2     => sample_dbt_del_2,
        sync_nbt             => sync_nbt,
        sync_dbt             => sync_dbt,
        sync_nbt_del_1       => sync_nbt_del_1,
        sync_dbt_del_1       => sync_dbt_del_1,
        data_tx              => data_tx,
        hard_sync_edge_valid => hard_sync_edge_valid,
        sp_control           => sp_control,
        sync_control         => sync_control
    );

    bus_sampling_comp : bus_sampling
    generic map (
        use_Sync             => use_sync
    )
    port map(
        clk_sys              => clk_sys,
        res_n                => res_n_int,
        CAN_rx               => CAN_rx,
        CAN_tx               => CAN_tx,
        drv_bus              => drv_bus,
        sample_nbt           => sample_nbt,
        sample_dbt           => sample_dbt,
        sync_edge            => sync_edge,
        data_tx              => data_tx,
        data_rx              => data_rx,
        sp_control           => sp_control,
        ssp_reset            => ssp_reset,
        trv_delay_calib      => trv_delay_calib,

        --Note: Bit Error detection enabled always. bit_Error signal from this
        -- block used only for secondary sample point bit error detection!!
        bit_err_enable       => '1',

        sample_sec_out       => sample_sec,
        sample_sec_del_1_out => sample_sec_del_1,
        sample_sec_del_2_out => sample_sec_del_2,
        trv_delay_out        => trv_delay_out,
        bit_Error            => bit_Error_sec_sam
    );


    event_logger_gen_true : if (use_logger) generate
        event_logger_comp : event_logger
        generic map(
            memory_size         => logger_size
        )
        port map(
            clk_sys             => clk_sys,
            res_n               => res_n_int,

            drv_bus             => drv_bus,
            stat_bus            => stat_bus,
            sync_edge           => sync_edge,
            timestamp           => timestamp,

            loger_finished      => loger_finished,
            loger_act_data      => loger_act_data,
            log_write_pointer   => log_write_pointer,
            log_read_pointer    => log_read_pointer,
            log_size            => log_size,
            log_state_out       => log_state_out,
            bt_FSM              => bt_FSM_out,
            data_overrun        => rx_data_overrun
        );
    end generate event_logger_gen_true;

    event_logger_gen_false : if (not use_logger) generate
        loger_finished    <= '0';
        loger_act_data    <= (others => '0');
        log_write_pointer <= (others => '0');
        log_read_pointer  <= (others => '0');
        log_size          <= (others => '0');
		log_state_out     <= config;
    end generate event_logger_gen_false;

    --Bit time clock output propagation
    time_quanta_clk <= clk_tq_nbt when sp_control = NOMINAL_SAMPLE else
                       clk_tq_dbt;

    OP_State <= oper_mode_type'val(to_integer(unsigned(
                    stat_bus(STAT_OP_STATE_HIGH downto STAT_OP_STATE_LOW))));

end architecture;
