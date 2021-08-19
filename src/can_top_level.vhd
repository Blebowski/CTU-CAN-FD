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
-- Module:
--  CAN top level
--
-- Sub-modules:
--  1. Memory registers
--  2. Interrupt manager
--  3. Prescaler (v3)
--  4. Bus sampling
--  5. RX Buffer
--  6. TXT Buffers
--  7. TX Arbitrator
--  8. Frame filters
--  9. Reset synchroniser
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

use ctu_can_fd_rtl.can_registers_pkg.all;

entity can_top_level is
    generic(
        -- RX Buffer RAM size (32 bit words)
        rx_buffer_size      : natural range 32 to 4096 := 32;

        -- Number of supported TXT buffers
        txt_buffer_count    : natural range 2 to 8   := 2; 

        -- Synthesize Filter A
        sup_filtA           : boolean                := false;
        
        -- Synthesize Filter B
        sup_filtB           : boolean                := false;
        
        -- Synthesize Filter C
        sup_filtC           : boolean                := false;
        
        -- Synthesize Range Filter
        sup_range           : boolean                := false;
        
        -- Synthesize Test registers
        sup_test_registers  : boolean                := true;
        
        -- Insert Traffic counters
        sup_traffic_ctrs    : boolean                := false;
        
        -- Target technology (ASIC or FPGA)
        target_technology   : natural                := C_TECH_FPGA
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys     : in std_logic;
        
        -- Asynchronous reset
        res_n       : in std_logic;

        -- Synchronized reset
        res_n_out   : out std_logic;

        -----------------------------------------------------------------------
        -- DFT support
        -----------------------------------------------------------------------
        scan_enable : in std_logic;

        -----------------------------------------------------------------------
        -- Memory interface
        -----------------------------------------------------------------------
        -- Input data
        data_in     : in  std_logic_vector(31 downto 0);
        
        -- Output data
        data_out    : out std_logic_vector(31 downto 0);
        
        -- Address
        adress      : in  std_logic_vector(15 downto 0);
        
        -- Chip select
        scs         : in  std_logic;
        
        -- Read indication
        srd         : in  std_logic;
        
        -- Write indication
        swr         : in  std_logic;
        
        -- Byte enable
        sbe         : in  std_logic_vector(3 downto 0);
        
        -----------------------------------------------------------------------
        -- Interrupt Interface
        -----------------------------------------------------------------------
        -- Interrupt output
        int         : out std_logic;

        -----------------------------------------------------------------------
        -- CAN Bus Interface
        -----------------------------------------------------------------------
        -- TX signal to CAN bus
        can_tx      : out std_logic;
        
        -- RX signal from CAN bus
        can_rx      : in  std_logic;

        -----------------------------------------------------------------------
        -- Debug signals for testbench
        -----------------------------------------------------------------------
        test_probe  : out t_ctu_can_fd_test_probe;

        -----------------------------------------------------------------------
        -- Timestamp for time based transmission / reception
        -----------------------------------------------------------------------
        timestamp    : in std_logic_vector(63 downto 0)
    );
end entity can_top_level;

architecture rtl of can_top_level is

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    ---- Internal signals
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    -- Common signals
    ----------------------------------------------------------------------------
    -- Driving Bus
    signal drv_bus              :    std_logic_vector(1023 downto 0);
    
    -- Status Bus
    signal stat_bus             :    std_logic_vector(511 downto 0);
    
    -- Synchronised reset
    signal res_n_sync           :    std_logic;
    
    -- Internal reset (Synchronised reset + Soft Reset)
    signal res_n_i              :    std_logic;
    
    -- Sample control (Nominal, Data, Secondary)
    signal sp_control           :    std_logic_vector(1 downto 0);
    
    ----------------------------------------------------------------------------
    -- RX Buffer <-> Memory registers Interface
    ----------------------------------------------------------------------------
    -- Actual size of synthetised message buffer (in 32 bit words)
    signal rx_buf_size          :    std_logic_vector(12 downto 0);
    
    -- Signal whenever buffer is full (no free memory words)
    signal rx_full              :    std_logic;
    
    -- Signal whenever buffer is empty (no frame (message) is stored)
    signal rx_empty             :    std_logic;
    
    -- Number of frames stored in recieve buffer
    signal rx_frame_count       :    std_logic_vector(10 downto 0);
    
    -- Number of free 32 bit wide words
    signal rx_mem_free          :    std_logic_vector(12 downto 0);
    
    -- Position of read pointer
    signal rx_read_pointer      :    std_logic_vector(11 downto 0);
    
    -- Position of write pointer
    signal rx_write_pointer     :    std_logic_vector(11 downto 0);
    
    -- Overrun occurred, data were discarded!
    -- (This is a flag and persists until it is cleared by SW)! 
    signal rx_data_overrun      :    std_logic;
    
    -- Actually loaded data for reading from RX Buffer
    signal rx_read_buff         :    std_logic_vector(31 downto 0);

    -- RX buffer middle of frame
    signal rx_mof               :    std_logic;

    ----------------------------------------------------------------------------
    -- TXT Buffer <-> Memory registers Interface
    ----------------------------------------------------------------------------
        
    -- TXT Buffer RAM - Data input
    signal txtb_port_a_data     :    std_logic_vector(31 downto 0);
    
    -- TXT Buffer RAM - Address
    signal txtb_port_a_address  :    std_logic_vector(4 downto 0);
    
    -- TXT Buffer chip select
    signal txtb_port_a_cs       :    std_logic_vector(txt_buffer_count - 1 downto 0);
    
    -- TXT Buffer Port A byte enable
    signal txtb_port_a_be       :    std_logic_vector(3 downto 0);

    -- TXT Buffer status
    signal txtb_state           :    t_txt_bufs_state(txt_buffer_count - 1 downto 0);

    -- SW Commands to TXT Buffer
    signal txtb_sw_cmd          :    t_txtb_sw_cmd;
    
    -- Command Index (Index in logic 1 means command is valid for buffer)          
    signal txtb_sw_cmd_index    :    std_logic_vector(txt_buffer_count - 1 downto 0);
    
    -- TXT Buffer priorities
    signal txtb_prorities       :    t_txt_bufs_priorities(txt_buffer_count - 1 downto 0);
    
    -- TXT Buffer bus-off behavior
    signal txt_buf_failed_bof   :    std_logic;
    
    ------------------------------------------------------------------------
    -- Interrupt Manager <-> Memory registers Interface
    ------------------------------------------------------------------------
    -- Interrupt vector
    signal int_vector           :    std_logic_vector(C_INT_COUNT - 1 downto 0);
    
    -- Interrupt enable
    signal int_ena              :    std_logic_vector(C_INT_COUNT - 1 downto 0);
    
    -- Interrupt mask
    signal int_mask             :    std_logic_vector(C_INT_COUNT - 1 downto 0);
    
    ------------------------------------------------------------------------
    -- RX Buffer <-> CAN Core Interface
    ------------------------------------------------------------------------
    -- Frame Identifier
    signal rec_ident            :    std_logic_vector(28 downto 0);
    
    -- Data length code
    signal rec_dlc              :    std_logic_vector(3 downto 0);
    
    -- Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_ident_type       :    std_logic;
    
    -- Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_frame_type       :    std_logic;
    
    -- Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_is_rtr           :    std_logic;
    
    -- Whenever frame was recieved with BIT Rate shift 
    signal rec_brs              :    std_logic;

    -- Recieved error state indicator
    signal rec_esi              :    std_logic;
   
    -- Data word which should be stored when "store_data" is active!
    signal store_data_word      :    std_logic_vector(31 downto 0);

    -- Signals start of frame. If timestamp on RX frame should be captured
    -- in the beginning of the frame, this pulse captures the timestamp!
    signal sof_pulse            :    std_logic;

    ------------------------------------------------------------------------
    -- Frame filters <-> CAN Core Interface (Commands for RX Buffer)
    ------------------------------------------------------------------------
    -- After control field of CAN frame, metadata are valid and can be stored.
    -- This command starts the RX FSM for storing.
    signal store_metadata       :    std_logic;
    
    -- Signal that one word of data can be stored (TX_DATA_X_W). This signal
    -- is active when 4 bytes were received or data reception has finished 
    -- on 4 byte unaligned number of frames! (Thus allowing to store also
    -- data which are not 4 byte aligned!
    signal store_data           :    std_logic;

    -- Received frame valid (commit RX Frame)
    signal rec_valid            :    std_logic;
    
    -- Abort storing of RX Frame to RX Buffer.
    signal rec_abort            :    std_logic;
    
    -- Filtered version of RX Buffer commands
    signal store_metadata_f     :    std_logic;
    signal store_data_f         :    std_logic;
    signal rec_valid_f          :    std_logic;
    signal rec_abort_f          :    std_logic;
    
    ------------------------------------------------------------------------
    -- TXT Buffers <-> Interrrupt Manager Interface
    ------------------------------------------------------------------------
    -- TXT HW Commands Applied Interrupt
    signal txtb_hw_cmd_int      :    std_logic_vector(txt_buffer_count - 1 downto 0);

    ------------------------------------------------------------------------
    -- TXT Buffers <-> CAN Core Interface
    ------------------------------------------------------------------------    
    -- HW Commands 
    signal txtb_hw_cmd          :    t_txtb_hw_cmd;

    -- Unit just turned bus off.
    signal is_bus_off           :    std_logic;

    ------------------------------------------------------------------------
    -- TXT Buffers <-> TX Arbitrator
    ------------------------------------------------------------------------    
    -- Index of TXT Buffer for which HW commands is valid          
    signal txtb_hw_cmd_index   :   natural range 0 to txt_buffer_count - 1;
    
    -- TXT Buffers are available, can be selected by TX Arbitrator
    signal txtb_available      :   std_logic_vector(txt_buffer_count - 1 downto 0);
        
    -- Pointer to TXT Buffer
    signal txtb_ptr            :   natural range 0 to 19;
    
    -- TXT Buffer RAM data outputs
    signal txtb_port_b_data    :   t_txt_bufs_output(txt_buffer_count - 1 downto 0);
    
    -- TXT Buffer RAM address
    signal txtb_port_b_address :   natural range 0 to 19;
    
    -- Clock enable to TXT Buffer port B
    signal txtb_port_b_clk_en  :   std_logic;
    
    ------------------------------------------------------------------------
    -- CAN Core <-> TX Arbitrator
    ------------------------------------------------------------------------    
    -- TX Data length code
    signal tran_dlc            :   std_logic_vector(3 downto 0);
    
    -- TX Remote transmission request flag
    signal tran_is_rtr         :   std_logic;

    -- TX Identifier type (0-Basic,1-Extended);
    signal tran_ident_type     :   std_logic;
    
    -- TX Frame type (0-CAN 2.0, 1-CAN FD)
    signal tran_frame_type     :   std_logic;
    
    -- TX Frame Bit rate shift Flag 
    signal tran_brs            :   std_logic;
    
    -- TX Identifier
    signal tran_identifier     :   std_logic_vector(28 downto 0);
        
    -- Word from TXT Buffer RAM selected by TX Arbitrator
    signal tran_word           :   std_logic_vector(31 downto 0);
    
    -- Valid frame is selected from transmission on output of TX Arbitrator.
    -- CAN Core may lock TXT Buffer for transmission!
    signal tran_frame_valid    :   std_logic;
    
    -- Selected TXT Buffer index changed
    signal txtb_changed        :   std_logic;
    
    -- TXT Buffer clock enable
    signal txtb_clk_en         :   std_logic;
    
    ------------------------------------------------------------------------
    -- CAN Core <-> Interrupt manager
    ------------------------------------------------------------------------    
    -- Error appeared
    signal err_detected         :   std_logic;

    -- Fault confinement state functionality changed
    signal fcs_changed          :   std_logic;

    -- Error warning limit reached
    signal err_warning_limit    :   std_logic;

    -- Arbitration was lost input
    signal arbitration_lost     :   std_logic;

    -- Transmitted frame is valid
    signal tran_valid           :   std_logic;

    -- Bit Rate Was Shifted
    signal br_shifted           :   std_logic;
    
    -- Overload frame is being transmitted
    signal is_overload          :   std_logic;
    
    ------------------------------------------------------------------------
    -- CAN Core <-> Prescaler Interface
    ------------------------------------------------------------------------
    -- RX Triggers (Sample)  
    signal rx_triggers          :   std_logic_vector(C_SAMPLE_TRIGGER_COUNT - 1 downto 0);
    
    -- TX Trigger (Sync)
    signal tx_trigger           :   std_logic;
    
    -- Synchronisation control (No synchronisation, Hard Synchronisation,
    -- Resynchronisation
    signal sync_control         :   std_logic_vector(1 downto 0);
    
    -- No positive resynchronisation 
    signal no_pos_resync        :   std_logic;
    
    -- Enable Nominal Bit time counters.
    signal nbt_ctrs_en          :   std_logic;
        
    -- Enable Data Bit time counters.
    signal dbt_ctrs_en          :   std_logic;
    
    ------------------------------------------------------------------------
    -- Bus Sampling <-> Memory Registers Interface
    ------------------------------------------------------------------------
    -- Measured Transceiver delay 
    signal trv_delay            :   std_logic_vector(C_TRV_CTR_WIDTH - 1 downto 0);
    
    ------------------------------------------------------------------------
    -- Bus Sampling <-> CAN Core Interface
    ------------------------------------------------------------------------
    -- RX Data With Bit Stuffing
    signal rx_data_wbs          :   std_logic;
    
    -- TX Data With Bit Stuffing
    signal tx_data_wbs          :   std_logic;
    
    -- Secondary sample point reset
    signal ssp_reset            :   std_logic; 

    -- Enable measurement of Transmitter delay
    signal tran_delay_meas      :   std_logic;

    -- Bit Error detected 
    signal bit_err              :   std_logic;
        
    -- Secondary sample signal 
    signal sample_sec           :   std_logic;
    
    -- Reset Bit time measurement counter
    signal btmc_reset           :   std_logic;
    
    -- Start Measurement of data bit time (in TX Trigger)
    signal dbt_measure_start    :   std_logic;
    
    -- First SSP generated (in ESI bit)
    signal gen_first_ssp        :   std_logic;
    
    ------------------------------------------------------------------------
    -- Bus Sampling <-> Prescaler Interface
    ------------------------------------------------------------------------
    -- Synchronisation edge (aligned with time quanta)
    signal sync_edge            :   std_logic;
    
    ------------------------------------------------------------------------
    -- Bit time FSM outputs
    ------------------------------------------------------------------------
    -- Bit time FSM state
    signal bt_fsm               :   t_bit_time;
    
    -- Time quanta edge
    signal tq_edge              :   std_logic;

    ------------------------------------------------------------------------
    -- Memory testability
    ------------------------------------------------------------------------
    -- Test registers
    signal test_registers_out   :   test_registers_out_t;

    -- RX buffer RAM test output
    signal tst_rdata_rx_buf     :   std_logic_vector(31 downto 0);

    -- TXT Buffer outputs
    signal tst_rdata_txt_bufs   :   t_txt_bufs_output(txt_buffer_count - 1 downto 0);

begin

    -- Test probe for observation
    test_probe.rx_trigger_nbs <= stat_bus(STAT_REC_TRIG);
    test_probe.rx_trigger_wbs <= stat_bus(STAT_RX_TRIGGER);
    test_probe.tx_trigger <= stat_bus(STAT_TX_TRIGGER);

    ---------------------------------------------------------------------------
    -- Reset synchroniser
    ---------------------------------------------------------------------------
    rst_sync_inst : entity ctu_can_fd_rtl.rst_sync
    generic map(
        G_RESET_POLARITY  => '0'
    )
    port map(
        clk             => clk_sys,
        arst            => res_n,
        rst             => res_n_sync
    );
    res_n_out <= res_n_sync;

    ---------------------------------------------------------------------------
    -- Memory registers
    ---------------------------------------------------------------------------
    memory_registers_inst : entity ctu_can_fd_rtl.memory_registers
    generic map(
        G_SUP_FILTA             => sup_filtA,
        G_SUP_FILTB             => sup_filtB,
        G_SUP_FILTC             => sup_filtC,
        G_SUP_RANGE             => sup_range,
        G_SUP_TEST_REGISTERS    => sup_test_registers,
        G_SUP_TRAFFIC_CTRS      => sup_traffic_ctrs,
        G_TXT_BUFFER_COUNT      => txt_buffer_count, 
        G_INT_COUNT             => C_INT_COUNT,
        G_TRV_CTR_WIDTH         => C_TRV_CTR_WIDTH,
        G_DEVICE_ID             => C_CAN_DEVICE_ID,
        G_VERSION_MINOR         => C_CTU_CAN_FD_VERSION_MINOR,
        G_VERSION_MAJOR         => C_CTU_CAN_FD_VERSION_MAJOR,
        G_TECHNOLOGY            => target_technology
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n_sync,              -- IN
        res_out                 => res_n_i,                 -- OUT

        -- DFT support
        scan_enable             => scan_enable,             -- IN

        -- Memory Interface
        data_in                 => data_in,                 -- IN
        data_out                => data_out,                -- OUT
        adress                  => adress,                  -- IN
        scs                     => scs,                     -- IN
        srd                     => srd,                     -- IN
        swr                     => swr,                     -- IN
        sbe                     => sbe,                     -- IN
        timestamp               => timestamp,               -- IN
        
        -- Buses to/from rest of CTU CAN FD
        drv_bus                 => drv_bus,                 -- OUT
        stat_bus                => stat_bus,                -- IN

        -- Manufacturing testability
        test_registers_out      => test_registers_out,      -- OUT
        tst_rdata_rx_buf        => tst_rdata_rx_buf,        -- IN
        tst_rdata_txt_bufs      => tst_rdata_txt_bufs,      -- IN

        -- RX Buffer Interface
        rx_read_buff            => rx_read_buff,            -- OUT
        rx_buf_size             => rx_buf_size,             -- IN
        rx_full                 => rx_full,                 -- IN
        rx_empty                => rx_empty,                -- IN
        rx_frame_count          => rx_frame_count,          -- IN
        rx_mem_free             => rx_mem_free,             -- IN
        rx_read_pointer         => rx_read_pointer,         -- IN
        rx_write_pointer        => rx_write_pointer,        -- IN
        rx_data_overrun         => rx_data_overrun,         -- IN
        rx_mof                  => rx_mof,                  -- IN

        -- Interface to TXT Buffers
        txtb_port_a_data        => txtb_port_a_data,        -- OUT
        txtb_port_a_address     => txtb_port_a_address,     -- OUT
        txtb_port_a_cs          => txtb_port_a_cs,          -- OUT
        txtb_port_a_be          => txtb_port_a_be,          -- OUT
        txtb_state              => txtb_state,              -- IN
        txtb_sw_cmd             => txtb_sw_cmd,             -- OUT
        txtb_sw_cmd_index       => txtb_sw_cmd_index,       -- OUT
        txtb_prorities          => txtb_prorities,          -- OUT
        txt_buf_failed_bof      => txt_buf_failed_bof,      -- OUT
         
        -- Bus synchroniser interface
        trv_delay               => trv_delay,               -- IN

        -- Interrrupt Interface
        int_vector              => int_vector,              -- IN
        int_ena                 => int_ena,                 -- IN
        int_mask                => int_mask                 -- IN
    );

    ---------------------------------------------------------------------------
    -- RX Buffer
    ---------------------------------------------------------------------------
    rx_buffer_inst : entity ctu_can_fd_rtl.rx_buffer
    generic map(
        G_RX_BUFF_SIZE          => rx_buffer_size,
        G_TECHNOLOGY            => target_technology
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n_i,                 -- IN

        -- DFT support
        scan_enable             => scan_enable,             -- IN

        -- Metadata from CAN Core
        rec_ident               => rec_ident,               -- IN
        rec_dlc                 => rec_dlc,                 -- IN
        rec_ident_type          => rec_ident_type,          -- IN
        rec_frame_type          => rec_frame_type,          -- IN
        rec_is_rtr              => rec_is_rtr,              -- IN
        rec_brs                 => rec_brs,                 -- IN
        rec_esi                 => rec_esi,                 -- IN

        -- Control signals from CAN Core which control storing of CAN Frame.
        -- Filtered by Frame filters.
        store_metadata_f        => store_metadata_f,        -- IN
        store_data_f            => store_data_f,            -- IN
        store_data_word         => store_data_word,         -- IN
        rec_valid_f             => rec_valid_f,             -- IN
        rec_abort_f             => rec_abort_f,             -- IN
        sof_pulse               => sof_pulse,               -- IN

        -- Status signals of recieve buffer
        rx_buf_size             => rx_buf_size,             -- OUT
        rx_full                 => rx_full,                 -- OUT
        rx_empty                => rx_empty,                -- OUT
        rx_frame_count          => rx_frame_count,          -- OUT
        rx_mem_free             => rx_mem_free,             -- OUT
        rx_read_pointer         => rx_read_pointer,         -- OUT
        rx_write_pointer        => rx_write_pointer,        -- OUT
        rx_data_overrun         => rx_data_overrun,         -- OUT
        rx_mof                  => rx_mof,                  -- OUT
        
        -- External timestamp input
        timestamp               => timestamp,               -- IN

        -- Memory registers interface
        rx_read_buff            => rx_read_buff,            -- IN
        drv_bus                 => drv_bus,                 -- IN
        test_registers_out      => test_registers_out,      -- IN        
        tst_rdata_rx_buf        => tst_rdata_rx_buf         -- OUT
    );

    ---------------------------------------------------------------------------
    -- TXT Buffers
    ---------------------------------------------------------------------------
    txt_buf_comp_gen : for i in 0 to txt_buffer_count - 1 generate
    begin
        txt_buffer_inst : entity ctu_can_fd_rtl.txt_buffer
        generic map(
            G_TXT_BUFFER_COUNT  => txt_buffer_count,
            G_ID                => i,
            G_TECHNOLOGY        => target_technology
        )
        port map(
            clk_sys             => clk_sys,                         -- IN
            res_n               => res_n_i,                         -- IN

            -- DFT support
            scan_enable         => scan_enable,                     -- IN


            -- Memory Registers Interface
            txtb_port_a_data    => txtb_port_a_data,                -- IN
            txtb_port_a_address => txtb_port_a_address,             -- IN
            txtb_port_a_cs      => txtb_port_a_cs(i),               -- IN
            txtb_port_a_be      => txtb_port_a_be,                  -- IN
            txtb_sw_cmd         => txtb_sw_cmd,                     -- IN
            txtb_sw_cmd_index   => txtb_sw_cmd_index,               -- IN
            txtb_state          => txtb_state(i),                   -- OUT
            txt_buf_failed_bof  => txt_buf_failed_bof,              -- IN
            drv_rom_ena         => drv_bus(DRV_ROM_ENA_INDEX),      -- IN
            drv_bus_mon_ena     => drv_bus(DRV_BUS_MON_ENA_INDEX),  -- IN
    
            -- Memory testability
            test_registers_out  => test_registers_out,              -- IN
            tst_rdata_txt_buf   => tst_rdata_txt_bufs(i),           -- OUT
    
            -- Interrupt Manager Interface
            txtb_hw_cmd_int     => txtb_hw_cmd_int(i),              -- OUT
    
            -- CAN Core and TX Arbitrator Interface
            txtb_hw_cmd         => txtb_hw_cmd,                     -- IN
            txtb_hw_cmd_index   => txtb_hw_cmd_index,               -- IN
            txtb_port_b_data    => txtb_port_b_data(i),             -- OUT
            txtb_port_b_address => txtb_port_b_address,             -- IN
            txtb_port_b_clk_en  => txtb_port_b_clk_en,              -- IN
            is_bus_off          => is_bus_off,                      -- IN
            txtb_available      => txtb_available(i)                -- OUT
        );
    end generate;

    ---------------------------------------------------------------------------
    -- TX Arbitrator
    ---------------------------------------------------------------------------
    tx_arbitrator_inst : entity ctu_can_fd_rtl.tx_arbitrator
    generic map(
        G_TXT_BUFFER_COUNT      => txt_buffer_count
    )
    port map( 
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n_i,                 -- IN

        -- TXT Buffers interface
        txtb_port_b_data        => txtb_port_b_data,        -- IN
        txtb_available          => txtb_available,          -- IN
        txtb_port_b_address     => txtb_port_b_address,     -- OUT
        txtb_port_b_clk_en      => txtb_port_b_clk_en,      -- OUT

        -- CAN Core Interface
        tran_word               => tran_word,               -- OUT
        tran_dlc                => tran_dlc,                -- OUT
        tran_is_rtr             => tran_is_rtr,             -- OUT
        tran_ident_type         => tran_ident_type,         -- OUT
        tran_frame_type         => tran_frame_type,         -- OUT
        tran_brs                => tran_brs,                -- OUT
        tran_identifier         => tran_identifier,         -- OUT
        tran_frame_valid        => tran_frame_valid,        -- OUT
        txtb_hw_cmd             => txtb_hw_cmd,             -- IN
        txtb_changed            => txtb_changed,            -- OUT
        txtb_hw_cmd_index       => txtb_hw_cmd_index,       -- IN
        txtb_ptr                => txtb_ptr,                -- IN
        txtb_clk_en             => txtb_clk_en,             -- IN

        -- Memory registers interface
        txtb_prorities          => txtb_prorities,          -- IN
        drv_bus                 => drv_bus,                 -- IN
        timestamp               => timestamp                -- IN
    );

    ---------------------------------------------------------------------------
    -- Frame Filters
    ---------------------------------------------------------------------------
    frame_filters_inst : entity ctu_can_fd_rtl.frame_filters
    generic map(
        G_SUP_FILTA             => sup_filtA,
        G_SUP_FILTB             => sup_filtB,
        G_SUP_FILTC             => sup_filtC,
        G_SUP_RANGE             => sup_range
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n_i,                 -- IN

        -- Memory registers interface
        drv_bus                 => drv_bus,                 -- IN

        -- CAN Core interface
        rec_ident               => rec_ident,               -- IN
        rec_ident_type          => rec_ident_type,          -- IN
        rec_frame_type          => rec_frame_type,          -- IN
        rec_is_rtr              => rec_is_rtr,              -- IN
        store_metadata          => store_metadata,          -- IN
        store_data              => store_data,              -- IN
        rec_valid               => rec_valid,               -- IN
        rec_abort               => rec_abort,               -- IN

        -- Frame filters output
        ident_valid             => open,                    -- OUT
        store_metadata_f        => store_metadata_f,        -- OUT
        store_data_f            => store_data_f,            -- OUT
        rec_valid_f             => rec_valid_f,             -- OUT
        rec_abort_f             => rec_abort_f              -- OUT
    );

    ---------------------------------------------------------------------------
    -- Interrrupt Manager
    ---------------------------------------------------------------------------
    int_manager_inst : entity ctu_can_fd_rtl.int_manager
    generic map(
        G_INT_COUNT             => C_INT_COUNT,
        G_TXT_BUFFER_COUNT      => txt_buffer_count
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n_i,                 -- IN

        -- Interrupt sources
        err_detected            => err_detected,            -- IN
        fcs_changed             => fcs_changed,             -- IN
        err_warning_limit       => err_warning_limit,       -- IN
        arbitration_lost        => arbitration_lost,        -- IN
        tran_valid              => tran_valid,              -- IN
        br_shifted              => br_shifted,              -- IN
        rx_data_overrun         => rx_data_overrun,         -- IN
        rec_valid               => rec_valid,               -- IN
        rx_full                 => rx_full,                 -- IN
        rx_empty                => rx_empty,                -- IN
        txtb_hw_cmd_int         => txtb_hw_cmd_int,         -- IN
        is_overload             => is_overload,             -- IN

        -- Memory registers Interface
        drv_bus                 => drv_bus,                 -- IN
        int                     => int,                     -- OUT
        int_vector              => int_vector,              -- OUT
        int_mask                => int_mask,                -- OUT
        int_ena                 => int_ena                  -- OUT
    );

    ---------------------------------------------------------------------------
    -- CAN Core
    ---------------------------------------------------------------------------
    can_core_inst : entity ctu_can_fd_rtl.can_core
    generic map(
        G_SAMPLE_TRIGGER_COUNT  => C_SAMPLE_TRIGGER_COUNT,
        G_CTRL_CTR_WIDTH        => C_CTRL_CTR_WIDTH,
        G_RETR_LIM_CTR_WIDTH    => C_RETR_LIM_CTR_WIDTH,
        G_ERR_VALID_PIPELINE    => C_ERR_VALID_PIPELINE,
        G_CRC15_POL             => C_CRC15_POL,
        G_CRC17_POL             => C_CRC17_POL,
        G_CRC21_POL             => C_CRC21_POL
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n_i,                 -- IN
        
        -- DFT support
        scan_enable             => scan_enable,             -- IN
        
        -- Memory registers interface
        drv_bus                 => drv_bus,                 -- IN
        stat_bus                => stat_bus,                -- OUT

        -- Tx Arbitrator and TXT Buffers interface
        tran_word               => tran_word,               -- IN
        tran_dlc                => tran_dlc,                -- IN
        tran_is_rtr             => tran_is_rtr,             -- IN
        tran_ident_type         => tran_ident_type,         -- IN
        tran_frame_type         => tran_frame_type,         -- IN
        tran_brs                => tran_brs,                -- IN
        tran_identifier         => tran_identifier,         -- IN
        tran_frame_valid        => tran_frame_valid,        -- IN
        txtb_hw_cmd             => txtb_hw_cmd,             -- OUT
        txtb_changed            => txtb_changed,            -- IN
        txtb_ptr                => txtb_ptr,                -- OUT
        txtb_clk_en             => txtb_clk_en,             -- OUT
        is_bus_off              => is_bus_off,              -- OUT

        -- Recieve Buffer and Message Filter Interface
        rec_ident               => rec_ident,               -- OUT
        rec_dlc                 => rec_dlc,                 -- OUT
        rec_ident_type          => rec_ident_type,          -- OUT
        rec_frame_type          => rec_frame_type,          -- OUT
        rec_is_rtr              => rec_is_rtr,              -- OUT
        rec_brs                 => rec_brs,                 -- OUT
        rec_esi                 => rec_esi,                 -- OUT
        rec_valid               => rec_valid,               -- OUT
        store_metadata          => store_metadata,          -- OUT
        store_data              => store_data,              -- OUT
        store_data_word         => store_data_word,         -- OUT
        rec_abort               => rec_abort,               -- OUT
        sof_pulse               => sof_pulse,               -- OUT

        -- Interrupt Manager Interface 
        arbitration_lost        => arbitration_lost,        -- OUT
        tran_valid              => tran_valid,              -- OUT
        br_shifted              => br_shifted,              -- OUT
        err_detected            => err_detected,            -- OUT
        fcs_changed             => fcs_changed,             -- OUT
        err_warning_limit       => err_warning_limit,       -- OUT
        is_overload             => is_overload,             -- OUT

        -- Prescaler interface 
        rx_triggers             => rx_triggers,             -- IN
        tx_trigger              => tx_trigger,              -- IN
        sync_control            => sync_control,            -- OUT
        no_pos_resync           => no_pos_resync,           -- OUT
        sp_control              => sp_control,              -- OUT
        nbt_ctrs_en             => nbt_ctrs_en,             -- OUT
        dbt_ctrs_en             => dbt_ctrs_en,             -- OUT

        -- CAN Bus serial data stream
        rx_data_wbs             => rx_data_wbs,             -- IN
        tx_data_wbs             => tx_data_wbs,             -- OUT

        -- Others
        timestamp               => timestamp,               -- IN
        ssp_reset               => ssp_reset,               -- OUT
        tran_delay_meas         => tran_delay_meas,         -- OUT
        bit_err                 => bit_err,                 -- IN
        sample_sec              => sample_sec,              -- IN
        btmc_reset              => btmc_reset,              -- OUT
        dbt_measure_start       => dbt_measure_start,       -- OUT
        gen_first_ssp           => gen_first_ssp,           -- OUT
        sync_edge               => sync_edge                -- IN
    );
    
    
    ---------------------------------------------------------------------------
    -- Prescaler
    ---------------------------------------------------------------------------
    prescaler_inst : entity ctu_can_fd_rtl.prescaler
    generic map(
        G_TSEG1_NBT_WIDTH       => C_TSEG1_NBT_WIDTH,
        G_TSEG2_NBT_WIDTH       => C_TSEG2_NBT_WIDTH,
        G_BRP_NBT_WIDTH         => C_BRP_NBT_WIDTH,
        G_SJW_NBT_WIDTH         => C_SJW_NBT_WIDTH,
        G_TSEG1_DBT_WIDTH       => C_TSEG1_DBT_WIDTH,
        G_TSEG2_DBT_WIDTH       => C_TSEG2_DBT_WIDTH,
        G_BRP_DBT_WIDTH         => C_BRP_DBT_WIDTH,
        G_SJW_DBT_WIDTH         => C_SJW_DBT_WIDTH,
        G_SAMPLE_TRIGGER_COUNT  => C_SAMPLE_TRIGGER_COUNT
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n_i,                 -- IN
        
        -- Memory registers interface
        drv_bus                 => drv_bus,                 -- IN
        
        -- Control Interface
        sync_edge               => sync_edge,               -- IN
        sp_control              => sp_control,              -- IN
        sync_control            => sync_control,            -- IN
        no_pos_resync           => no_pos_resync,           -- IN
        nbt_ctrs_en             => nbt_ctrs_en,             -- IN
        dbt_ctrs_en             => dbt_ctrs_en,             -- IN
        
        -- Trigger signals
        rx_triggers             => rx_triggers,             -- OUT
        tx_trigger              => tx_trigger,              -- OUT
        
        -- Status outputs
        bt_fsm                  => bt_fsm,                  -- OUT
        tq_edge                 => tq_edge                  -- OUT
    );
  
 
    ---------------------------------------------------------------------------
    -- Bus Sampling
    ---------------------------------------------------------------------------
    bus_sampling_inst : entity ctu_can_fd_rtl.bus_sampling 
    generic map(
        G_SSP_DELAY_SAT_VAL     => C_SSP_DELAY_SAT_VAL,
        G_TX_CACHE_DEPTH        => C_TX_CACHE_DEPTH,
        G_TRV_CTR_WIDTH         => C_TRV_CTR_WIDTH,
        G_SSP_POS_WIDTH         => C_SSP_POS_WIDTH,
        G_USE_SSP_SATURATION    => C_USE_SSP_SATURATION,
        G_SSP_CTRS_WIDTH        => C_SSP_CTRS_WIDTH
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n_i,                 -- IN

        -- DFT support
        scan_enable             => scan_enable,             -- IN

        -- Physical layer interface
        can_rx                  => can_rx,                  -- IN
        can_tx                  => can_tx,                  -- OUT

        -- Memory registers interface
        drv_bus                 => drv_bus,                 -- IN
        trv_delay               => trv_delay,               -- OUT

        -- Prescaler interface
        rx_trigger              => rx_triggers(1),          -- IN
        tx_trigger              => tx_trigger,              -- IN
        sync_edge               => sync_edge,               -- OUT
        tq_edge                 => tq_edge,                 -- IN

        -- CAN Core Interface
        tx_data_wbs             => tx_data_wbs,             -- IN
        rx_data_wbs             => rx_data_wbs,             -- OUT
        sp_control              => sp_control,              -- IN
        ssp_reset               => ssp_reset,               -- IN
        tran_delay_meas         => tran_delay_meas,         -- IN
        sample_sec              => sample_sec,              -- OUT
        bit_err                 => bit_err,                 -- OUT
        btmc_reset              => btmc_reset,              -- IN
        dbt_measure_start       => dbt_measure_start,       -- IN
        gen_first_ssp           => gen_first_ssp            -- IN
    );
    
    -- <RELEASE_OFF>
    -----------------------------------------------------------------------
    -----------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------
    -----------------------------------------------------------------------
    
    -- psl default clock is rising_edge(clk_sys);

    -- psl no_tx_dominant_when_disabled_asrt : assert never
    --  (drv_bus(DRV_ENA_INDEX) = '0' and can_tx = DOMINANT)
    --  report "Dominant bit can't be transmitted when Node is disabled!";

    -- psl no_tx_dominant_when_bus_monitoring_asrt : assert never
    --  (drv_bus(DRV_BUS_MON_ENA_INDEX) = '1' and can_tx = DOMINANT)
    --  report "Dominant bit can't be transmitted in Bus monitoring mode!";
    
    -- Each TXT Buffer should have been unlocked already when unit is
    -- transmitting overload frame! This-way we can be sure that no we can
    -- block unlock command in Protocol control FSM in overload frames!
     
    txtb_asr_gen : for i in 0 to txt_buffer_count - 1 generate
    
    -- psl no_tx_buf_transmitting_in_overload_asrt : assert never
    --  (((txtb_state(i) = TXT_TRAN) or (txtb_state(i) = TXT_ABTP))) and
    --   (is_overload = '1')
    --   report "TXT Buffer should have been unlocked when node is in Overload frame!";
    
    end generate;

    -- Memory testability shall not be used when CTU CAN FD is enabled
    -- and operating!!
    -- psl no_mem_test_when_operating_asrt : assert never
    --  (drv_bus(DRV_ENA_INDEX) = '1' and test_registers_out.tst_control(TMAENA_IND) = '1')
    --  report "Memory testability shall not be enabled when CTU CAN FD is running!";

    -- <RELEASE_ON>
    

end architecture;