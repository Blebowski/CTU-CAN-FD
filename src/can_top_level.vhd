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
use ctu_can_fd_rtl.unary_ops_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

use ctu_can_fd_rtl.can_registers_pkg.all;

entity can_top_level is
    generic (
        -- RX Buffer RAM size (32 bit words)
        rx_buffer_size          : natural range 32 to 4096  := 32;

        -- Number of supported TXT buffers
        txt_buffer_count        : natural range 2 to 8      := C_TXT_BUFFER_COUNT;

        -- Synthesize Filter A
        sup_filtA               : boolean                   := false;

        -- Synthesize Filter B
        sup_filtB               : boolean                   := false;

        -- Synthesize Filter C
        sup_filtC               : boolean                   := false;

        -- Synthesize Range Filter
        sup_range               : boolean                   := false;

        -- Synthesize Test registers
        sup_test_registers      : boolean                   := true;

        -- Insert Traffic counters
        sup_traffic_ctrs        : boolean                   := false;

        -- Add parity bit to TXT Buffer and RX Buffer RAMs
        sup_parity              : boolean                   := false;

        -- Number of active timestamp bits
        active_timestamp_bits   : natural range 0 to 63     := 63;

        -- Reset TXT / RX Buffer RAMs
        reset_buffer_rams       : boolean                   := false;

        -- Target technology (ASIC or FPGA)
        target_technology       : natural                   := C_TECH_FPGA
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

    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    ---- Internal constants
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Width of RX Buffer pointers
    constant C_RX_BUFF_PTR_WIDTH        :   natural range 5 to 12 :=
        integer(ceil(log2(real(rx_buffer_size))));

    -- Width of RX Buffer frame counter
    -- Minimal sized frame is 4 words. At e.g. rx_buffer_size = 256 (8 bits for pointers),
    -- we get maximum 64 frames stored in RX Buffer (inclusive) -> 7 bits for frame counter.
    constant C_RX_BUF_FRAME_CNT_WIDTH   :   natural range 3 to 11 :=
        integer(ceil(log2(real(rx_buffer_size)))) - 1;

    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    ---- Internal signals
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------

    -----------------------------------------------------------------------------------------------
    -- Common signals
    -----------------------------------------------------------------------------------------------

    -- Synchronised reset
    signal res_n_sync                   :    std_logic;

    -- Core reset (Synchronised reset + Soft Reset + Active when disabled)
    signal res_core_n                   :    std_logic;

    -- Soft reset ((Synchronised reset + Soft Reset)
    signal res_soft_n                   :    std_logic;

    -- Sample control (Nominal, Data, Secondary)
    signal sp_control                   :    std_logic_vector(1 downto 0);

    -----------------------------------------------------------------------------------------------
    -- RX Buffer <-> Memory registers Interface
    -----------------------------------------------------------------------------------------------
    -- Signal whenever buffer is full (no free memory words)
    signal rx_full                      :    std_logic;

    -- Signal whenever buffer is empty (no frame (message) is stored)
    signal rx_empty                     :    std_logic;

    -- Number of frames stored in recieve buffer
    signal rx_frame_count               :    std_logic_vector(C_RX_BUF_FRAME_CNT_WIDTH - 1 downto 0);

    -- Number of free 32 bit wide words
    signal rx_mem_free                  :    std_logic_vector(C_RX_BUFF_PTR_WIDTH downto 0);

    -- Position of read pointer
    signal rx_read_pointer              :    std_logic_vector(C_RX_BUFF_PTR_WIDTH - 1 downto 0);

    -- Position of write pointer
    signal rx_write_pointer             :    std_logic_vector(C_RX_BUFF_PTR_WIDTH - 1 downto 0);

    -- Overrun occurred, data were discarded!
    -- (This is a flag and persists until it is cleared by SW)!
    signal rx_data_overrun              :    std_logic;

    -- RX buffer middle of frame
    signal rx_mof                       :    std_logic;

    -- RX Buffer parity error flag
    signal rx_parity_error              :    std_logic;

    -----------------------------------------------------------------------------------------------
    -- TXT Buffer <-> Memory registers Interface
    -----------------------------------------------------------------------------------------------

    -- TXT Buffer RAM - Data input
    signal txtb_port_a_data_in          :    std_logic_vector(31 downto 0);

    -- TXT Buffer RAM - Parity input
    signal txtb_port_a_parity           :    std_logic;

    -- TXT Buffer RAM - Address
    signal txtb_port_a_address          :    std_logic_vector(4 downto 0);

    -- TXT Buffer chip select
    signal txtb_port_a_cs               :    std_logic_vector(txt_buffer_count - 1 downto 0);

    -- TXT Buffer Port A byte enable
    signal txtb_port_a_be               :    std_logic_vector(3 downto 0);

    -- TXT Buffer status
    signal txtb_state                   :    t_txt_bufs_state(txt_buffer_count - 1 downto 0);

    -- TXT Buffer is operating in Backup buffer
    signal txtb_is_bb                   :    std_logic_vector(txt_buffer_count - 1 downto 0);

    -----------------------------------------------------------------------------------------------
    -- RX Buffer <-> CAN Core Interface
    -----------------------------------------------------------------------------------------------
    -- Frame Identifier
    signal rec_ident                    :    std_logic_vector(28 downto 0);

    -- Data length code
    signal rec_dlc                      :    std_logic_vector(3 downto 0);

    -- Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_ident_type               :    std_logic;

    -- Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_frame_type               :    std_logic;

    -- Received Loopback frame
    signal rec_lbpf                     :    std_logic;

    -- Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_is_rtr                   :    std_logic;

    -- Whenever frame was recieved with BIT Rate shift
    signal rec_brs                      :    std_logic;

    -- Recieved error state indicator
    signal rec_esi                      :    std_logic;

    -- Received Identifier is valid
    signal rec_ivld                     :    std_logic;

    -- Data word which should be stored when "store_data" is active!
    signal store_data_word              :    std_logic_vector(31 downto 0);

    -- Signals start of frame. If timestamp on RX frame should be captured
    -- in the beginning of the frame, this pulse captures the timestamp!
    signal sof_pulse                    :    std_logic;

    -----------------------------------------------------------------------------------------------
    -- Frame filters <-> CAN Core Interface (Commands for RX Buffer)
    -----------------------------------------------------------------------------------------------
    -- After control field of CAN frame, metadata are valid and can be stored.
    -- This command starts the RX FSM for storing.
    signal store_metadata               :    std_logic;

    -- Signal that one word of data can be stored (TX_DATA_X_W). This signal
    -- is active when 4 bytes were received or data reception has finished
    -- on 4 byte unaligned number of frames! (Thus allowing to store also
    -- data which are not 4 byte aligned!
    signal store_data                   :    std_logic;

    -- Received frame valid (commit RX Frame)
    signal rec_valid                    :    std_logic;

    -- Abort storing of RX Frame to RX Buffer.
    signal rec_abort                    :    std_logic;

    -- Filtered version of RX Buffer commands
    signal store_metadata_f             :    std_logic;
    signal store_data_f                 :    std_logic;
    signal rec_valid_f                  :    std_logic;
    signal rec_abort_f                  :    std_logic;

    -----------------------------------------------------------------------------------------------
    -- TXT Buffers <-> Interrrupt Manager Interface
    -----------------------------------------------------------------------------------------------
    -- TXT HW Commands Applied Interrupt
    signal txtb_hw_cmd_int              :    std_logic_vector(txt_buffer_count - 1 downto 0);

    -----------------------------------------------------------------------------------------------
    -- TXT Buffers <-> CAN Core Interface
    -----------------------------------------------------------------------------------------------
    -- HW Commands
    signal txtb_hw_cmd                  :    t_txtb_hw_cmd;

    -----------------------------------------------------------------------------------------------
    -- TXT Buffers <-> TX Arbitrator
    -----------------------------------------------------------------------------------------------
    -- Index of TXT Buffer for which HW commands is valid
    signal txtb_hw_cmd_cs               :   std_logic_vector(txt_buffer_count - 1 downto 0);

    -- TXT Buffers are available, can be selected by TX Arbitrator
    signal txtb_available               :   std_logic_vector(txt_buffer_count - 1 downto 0);

    -- TXT Buffer is in state for which its backup buffer can be used
    signal txtb_allow_bb                :   std_logic_vector(txt_buffer_count - 1 downto 0);

    -- Pointer to TXT Buffer
    signal txtb_ptr                     :   natural range 0 to 20;

    -- TXT Buffer RAM data outputs
    signal txtb_port_b_data_out         :   t_txt_bufs_output(txt_buffer_count - 1 downto 0);

    -- TXT Buffer RAM address
    signal txtb_port_b_address          :   std_logic_vector(4 downto 0);

    -- Clock enable to TXT Buffer port B
    signal txtb_port_b_clk_en           :   std_logic;

    -- Parity check valid
    signal txtb_parity_check_valid      :   std_logic_vector(txt_buffer_count - 1 downto 0);

    -- Parity mismatch
    signal txtb_parity_mismatch         :   std_logic_vector(txt_buffer_count - 1 downto 0);

    -- Parity error valid
    signal txtb_parity_error_valid      :   std_logic_vector(txt_buffer_count - 1 downto 0);

    -- TXT Buffer
    signal txtb_bb_parity_error         :   std_logic_vector(txt_buffer_count - 1 downto 0);

    -----------------------------------------------------------------------------------------------
    -- CAN Core <-> TX Arbitrator
    -----------------------------------------------------------------------------------------------
    -- TX Data length code
    signal tran_dlc                     :   std_logic_vector(3 downto 0);

    -- TX Remote transmission request flag
    signal tran_is_rtr                  :   std_logic;

    -- TX Identifier type (0-Basic,1-Extended);
    signal tran_ident_type              :   std_logic;

    -- TX Frame type (0-CAN 2.0, 1-CAN FD)
    signal tran_frame_type              :   std_logic;

    -- TX Frame Bit rate shift Flag
    signal tran_brs                     :   std_logic;

    -- TX Identifier
    signal tran_identifier              :   std_logic_vector(28 downto 0);

    -- TX Frame test word
    signal tran_frame_test              :   t_frame_test_w;

    -- Word from TXT Buffer RAM selected by TX Arbitrator
    signal tran_word                    :   std_logic_vector(31 downto 0);

    -- Valid frame is selected from transmission on output of TX Arbitrator.
    -- CAN Core may lock TXT Buffer for transmission!
    signal tran_frame_valid             :   std_logic;

    -- Parity error occured in TXT Buffer RAM during transmission of data words.
    signal tran_frame_parity_error      :   std_logic;

    -- Selected TXT Buffer index changed
    signal txtb_changed                 :   std_logic;

    -- TXT Buffer clock enable
    signal txtb_clk_en                  :   std_logic;

    -----------------------------------------------------------------------------------------------
    -- RX Buffer <-> TX Arbitrator
    -----------------------------------------------------------------------------------------------

    -- TXT Buffer index that is:
    --   - Currently validated (when no transmission is in progress)
    --   - Used for transmission (when transmission is in progress)
    signal curr_txtb_index              :   std_logic_vector(2 downto 0);

    -----------------------------------------------------------------------------------------------
    -- CAN Core <-> Interrupt manager
    -----------------------------------------------------------------------------------------------
    -- Error appeared
    signal err_detected                 :   std_logic;

    -- Fault confinement state functionality changed
    signal fcs_changed                  :   std_logic;

    -- Error warning limit reached
    signal err_warning_limit_pulse      :   std_logic;

    -- Arbitration was lost input
    signal arbitration_lost             :   std_logic;

    -- Transmitted frame is valid
    signal tran_valid                   :   std_logic;

    -- Bit Rate Was Shifted
    signal br_shifted                   :   std_logic;

    -----------------------------------------------------------------------------------------------
    -- CAN Core <-> Prescaler Interface
    -----------------------------------------------------------------------------------------------
    -- RX Triggers (Sample)
    signal rx_triggers                  :   std_logic_vector(C_SAMPLE_TRIGGER_COUNT - 1 downto 0);

    -- TX Trigger (Sync)
    signal tx_trigger                   :   std_logic;

    -- Synchronisation control (No synchronisation, Hard Synchronisation,
    -- Resynchronisation
    signal sync_control                 :   std_logic_vector(1 downto 0);

    -- No positive resynchronisation
    signal no_pos_resync                :   std_logic;

    -- Enable Nominal Bit time counters.
    signal nbt_ctrs_en                  :   std_logic;

    -- Enable Data Bit time counters.
    signal dbt_ctrs_en                  :   std_logic;

    -----------------------------------------------------------------------------------------------
    -- Bus Sampling <-> Memory Registers Interface
    -----------------------------------------------------------------------------------------------
    -- Measured Transceiver delay
    signal trv_delay                    :   std_logic_vector(C_TRV_CTR_WIDTH - 1 downto 0);

    -----------------------------------------------------------------------------------------------
    -- Bus Sampling <-> CAN Core Interface
    -----------------------------------------------------------------------------------------------
    -- RX Data With Bit Stuffing
    signal rx_data_wbs                  :   std_logic;

    -- TX Data With Bit Stuffing
    signal tx_data_wbs                  :   std_logic;

    -- Secondary sample point reset
    signal ssp_reset                    :   std_logic;

    -- Enable measurement of Transmitter delay
    signal tran_delay_meas              :   std_logic;

    -- Bit Error detected
    signal bit_err                      :   std_logic;

    -- Reset Bit time measurement counter
    signal btmc_reset                   :   std_logic;

    -- Start Measurement of data bit time (in TX Trigger)
    signal dbt_measure_start            :   std_logic;

    -- First SSP generated (in ESI bit)
    signal gen_first_ssp                :   std_logic;

    -----------------------------------------------------------------------------------------------
    -- Bus Sampling <-> Prescaler Interface
    -----------------------------------------------------------------------------------------------
    -- Synchronisation edge (aligned with time quanta)
    signal sync_edge                    :   std_logic;

    -----------------------------------------------------------------------------------------------
    -- Bit time FSM outputs
    -----------------------------------------------------------------------------------------------
    -- Time quanta edge
    signal tq_edge                      :   std_logic;

    -----------------------------------------------------------------------------------------------
    -- Memory registers
    -----------------------------------------------------------------------------------------------
    -- Configuration from Control registers to rest of the core
    signal mr_ctrl_out                      :   control_registers_out_t;

    -- Configuration from Test registers to rest of the core
    signal mr_tst_out                       :   test_registers_out_t;

    -- Status to registers from CAN core
    signal cc_stat                          :   t_can_core_stat;

    -- Debug record from Protocol control
    signal pc_dbg                           :   t_protocol_control_dbg;

    -- Interrupt values (actual interrupt status)
    signal mr_int_stat_rxi_o                :   std_logic;
    signal mr_int_stat_txi_o                :   std_logic;
    signal mr_int_stat_ewli_o               :   std_logic;
    signal mr_int_stat_doi_o                :   std_logic;
    signal mr_int_stat_fcsi_o               :   std_logic;
    signal mr_int_stat_ali_o                :   std_logic;
    signal mr_int_stat_bei_o                :   std_logic;
    signal mr_int_stat_ofi_o                :   std_logic;
    signal mr_int_stat_rxfi_o               :   std_logic;
    signal mr_int_stat_bsi_o                :   std_logic;
    signal mr_int_stat_rbnei_o              :   std_logic;
    signal mr_int_stat_txbhci_o             :   std_logic;

    -- Interrupt enable and mask
    signal mr_int_ena_set_int_ena_set_o     :   std_logic_vector(C_INT_COUNT - 1 downto 0);
    signal mr_int_mask_set_int_mask_set_o   :   std_logic_vector(C_INT_COUNT - 1 downto 0);


    -----------------------------------------------------------------------------------------------
    -- Memory testability Read data
    -----------------------------------------------------------------------------------------------
    -- RX buffer test data in
    signal mr_tst_rdata_tst_rdata_rxb       :   std_logic_vector(31 downto 0);

    -- TXT buffers test data input
    signal mr_tst_rdata_tst_rdata_txb       :   t_txt_bufs_output(txt_buffer_count - 1 downto 0);

    signal rxb_port_b_data_out              :   std_logic_vector(31 downto 0);

    signal pc_rx_trigger                    :   std_logic;

    signal mr_tx_command_txbi               :   std_logic_vector(txt_buffer_count - 1 downto 0);
    signal mr_tx_priority                   :   t_txt_bufs_priorities(txt_buffer_count - 1 downto 0);

begin

    -- Test probe for observation
    test_probe.rx_trigger_nbs <= pc_rx_trigger;
    test_probe.rx_trigger_wbs <= rx_triggers(0);
    test_probe.tx_trigger <= tx_trigger;

    -----------------------------------------------------------------------------------------------
    -- Reset synchroniser
    -----------------------------------------------------------------------------------------------
    rst_sync_inst : entity ctu_can_fd_rtl.rst_sync
    generic map (
        G_RESET_POLARITY  => '0'
    )
    port map (
        clk             => clk_sys,
        arst            => res_n,
        rst             => res_n_sync
    );
    res_n_out <= res_n_sync;

    -----------------------------------------------------------------------------------------------
    -- Memory registers
    -----------------------------------------------------------------------------------------------
    memory_registers_inst : entity ctu_can_fd_rtl.memory_registers
    generic map (
        G_SUP_FILTA                     => sup_filtA,
        G_SUP_FILTB                     => sup_filtB,
        G_SUP_FILTC                     => sup_filtC,
        G_SUP_RANGE                     => sup_range,
        G_SUP_TEST_REGISTERS            => sup_test_registers,
        G_SUP_TRAFFIC_CTRS              => sup_traffic_ctrs,
        G_SUP_PARITY                    => sup_parity,
        G_TXT_BUFFER_COUNT              => txt_buffer_count,
        G_RX_BUFF_SIZE                  => rx_buffer_size,
        G_RX_BUF_FRAME_CNT_WIDTH        => C_RX_BUF_FRAME_CNT_WIDTH,
        G_RX_BUFF_PTR_WIDTH             => C_RX_BUFF_PTR_WIDTH,
        G_INT_COUNT                     => C_INT_COUNT,
        G_TRV_CTR_WIDTH                 => C_TRV_CTR_WIDTH,
        G_TS_BITS                       => active_timestamp_bits,
        G_DEVICE_ID                     => C_CAN_DEVICE_ID,
        G_VERSION_MINOR                 => C_CTU_CAN_FD_VERSION_MINOR,
        G_VERSION_MAJOR                 => C_CTU_CAN_FD_VERSION_MAJOR,
        G_TECHNOLOGY                    => target_technology
    )
    port map (
        clk_sys                         => clk_sys,                         -- IN
        res_n                           => res_n_sync,                      -- IN

        -- Generated resets
        res_core_n                      => res_core_n,                      -- OUT
        res_soft_n                      => res_soft_n,                      -- OUT

        -- DFT support
        scan_enable                     => scan_enable,                     -- IN

        -- Memory Interface
        data_in                         => data_in,                         -- IN
        data_out                        => data_out,                        -- OUT
        adress                          => adress,                          -- IN
        scs                             => scs,                             -- IN
        srd                             => srd,                             -- IN
        swr                             => swr,                             -- IN
        sbe                             => sbe,                             -- IN
        timestamp                       => timestamp,                       -- IN

        -- Configuration and Status to/from rest of the core
        mr_ctrl_out                     => mr_ctrl_out,                     -- OUT
        mr_tst_out                      => mr_tst_out,                      -- OUT
        cc_stat                         => cc_stat,                         -- IN
        pc_dbg                          => pc_dbg,                          -- IN
        mr_tst_rdata_tst_rdata_rxb      => mr_tst_rdata_tst_rdata_rxb,      -- IN
        mr_tst_rdata_tst_rdata_txb      => mr_tst_rdata_tst_rdata_txb,      -- IN

        -- RX Buffer Interface
        rx_full                         => rx_full,                         -- IN
        rx_empty                        => rx_empty,                        -- IN
        rx_frame_count                  => rx_frame_count,                  -- IN
        rx_mem_free                     => rx_mem_free,                     -- IN
        rx_read_pointer                 => rx_read_pointer,                 -- IN
        rx_write_pointer                => rx_write_pointer,                -- IN
        rx_data_overrun                 => rx_data_overrun,                 -- IN
        rx_mof                          => rx_mof,                          -- IN
        rx_parity_error                 => rx_parity_error,                 -- IN
        rxb_port_b_data_out             => rxb_port_b_data_out,             -- IN

        -- Interface to TXT Buffers
        txtb_port_a_data_in             => txtb_port_a_data_in,             -- OUT
        txtb_port_a_address             => txtb_port_a_address,             -- OUT
        txtb_port_a_cs                  => txtb_port_a_cs,                  -- OUT
        txtb_port_a_be                  => txtb_port_a_be,                  -- OUT
        mr_tx_priority                  => mr_tx_priority,                  -- OUT
        mr_tx_command_txbi              => mr_tx_command_txbi,              -- OUT
        txtb_state                      => txtb_state,                      -- IN
        txtb_parity_error_valid         => txtb_parity_error_valid,         -- IN
        txtb_bb_parity_error            => txtb_bb_parity_error,            -- IN

        -- Bus synchroniser interface
        trv_delay                       => trv_delay,                       -- IN

        -- Interrrupt Manager Interface
        mr_int_stat_rxi_o               => mr_int_stat_rxi_o,               -- IN
        mr_int_stat_txi_o               => mr_int_stat_txi_o,               -- IN
        mr_int_stat_ewli_o              => mr_int_stat_ewli_o,              -- IN
        mr_int_stat_doi_o               => mr_int_stat_doi_o,               -- IN
        mr_int_stat_fcsi_o              => mr_int_stat_fcsi_o,              -- IN
        mr_int_stat_ali_o               => mr_int_stat_ali_o,               -- IN
        mr_int_stat_bei_o               => mr_int_stat_bei_o,               -- IN
        mr_int_stat_ofi_o               => mr_int_stat_ofi_o,               -- IN
        mr_int_stat_rxfi_o              => mr_int_stat_rxfi_o,              -- IN
        mr_int_stat_bsi_o               => mr_int_stat_bsi_o,               -- IN
        mr_int_stat_rbnei_o             => mr_int_stat_rbnei_o,             -- IN
        mr_int_stat_txbhci_o            => mr_int_stat_txbhci_o,            -- IN

        mr_int_ena_set_int_ena_set_o    => mr_int_ena_set_int_ena_set_o,    -- IN
        mr_int_mask_set_int_mask_set_o  => mr_int_mask_set_int_mask_set_o   -- IN
    );

    -----------------------------------------------------------------------------------------------
    -- RX Buffer
    -----------------------------------------------------------------------------------------------
    rx_buffer_inst : entity ctu_can_fd_rtl.rx_buffer
    generic map (
        G_RX_BUFF_SIZE                  => rx_buffer_size,
        G_RX_BUFF_PTR_WIDTH             => C_RX_BUFF_PTR_WIDTH,
        G_RX_BUF_FRAME_CNT_WIDTH        => C_RX_BUF_FRAME_CNT_WIDTH,
        G_SUP_PARITY                    => sup_parity,
        G_RESET_RX_BUF_RAM              => reset_buffer_rams,
        G_TECHNOLOGY                    => target_technology
    )
    port map (
        -- Clocks and Asynchronous reset
        clk_sys                         => clk_sys,                         -- IN
        res_n                           => res_core_n,                      -- IN

        -- DFT support
        scan_enable                     => scan_enable,                     -- IN

        -- Metadata from CAN Core
        rec_ident                       => rec_ident,                       -- IN
        rec_dlc                         => rec_dlc,                         -- IN
        rec_ident_type                  => rec_ident_type,                  -- IN
        rec_frame_type                  => rec_frame_type,                  -- IN
        rec_lbpf                        => rec_lbpf,                        -- IN
        rec_is_rtr                      => rec_is_rtr,                      -- IN
        rec_brs                         => rec_brs,                         -- IN
        rec_esi                         => rec_esi,                         -- IN
        rec_ivld                        => rec_ivld,                        -- IN

        -- Control signals from CAN Core which control storing of CAN Frame.
        -- Filtered by Frame filters.
        store_metadata_f                => store_metadata_f,                -- IN
        store_data_f                    => store_data_f,                    -- IN
        store_data_word                 => store_data_word,                 -- IN
        rec_valid_f                     => rec_valid_f,                     -- IN
        rec_abort_f                     => rec_abort_f,                     -- IN
        sof_pulse                       => sof_pulse,                       -- IN

        -- Status signals of recieve buffer
        rx_full                         => rx_full,                         -- OUT
        rx_empty                        => rx_empty,                        -- OUT
        rx_frame_count                  => rx_frame_count,                  -- OUT
        rx_mem_free                     => rx_mem_free,                     -- OUT
        rx_read_pointer                 => rx_read_pointer,                 -- OUT
        rx_write_pointer                => rx_write_pointer,                -- OUT
        rx_data_overrun                 => rx_data_overrun,                 -- OUT
        rx_mof                          => rx_mof,                          -- OUT
        rx_parity_error                 => rx_parity_error,                 -- OUT
        rxb_port_b_data_out             => rxb_port_b_data_out,             -- OUT

        -- External timestamp input
        timestamp                       => timestamp,                       -- IN

        -- TX Arbitrator interface
        curr_txtb_index                 => curr_txtb_index,                 -- IN

        -- Memory registers interface
        mr_mode_rxbam                   => mr_ctrl_out.mode_rxbam,          -- IN
        mr_command_cdo                  => mr_ctrl_out.command_cdo,         -- IN
        mr_command_crxpe                => mr_ctrl_out.command_crxpe,       -- IN
        mr_command_rrb                  => mr_ctrl_out.command_rrb,         -- IN
        mr_command_rxrpmv               => mr_ctrl_out.command_rxrpmv,      -- IN
        mr_rx_data_read                 => mr_ctrl_out.rx_data_read,        -- IN
        mr_rx_settings_rtsop            => mr_ctrl_out.rx_settings_rtsop,   -- IN
        mr_settings_pchke               => mr_ctrl_out.settings_pchke,      -- IN

        -- Memory testability
        mr_tst_control_tmaena           => mr_tst_out.tst_control_tmaena,   -- IN
        mr_tst_control_twrstb           => mr_tst_out.tst_control_twrstb,   -- IN
        mr_tst_dest_tst_addr            => mr_tst_out.tst_dest_tst_addr,    -- IN
        mr_tst_dest_tst_mtgt            => mr_tst_out.tst_dest_tst_mtgt,    -- IN
        mr_tst_wdata_tst_wdata          => mr_tst_out.tst_wdata_tst_wdata,  -- IN

        mr_tst_rdata_tst_rdata          => mr_tst_rdata_tst_rdata_rxb       -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- TXT Buffer port A parity encoding
    -----------------------------------------------------------------------------------------------
    txtb_parity_true_gen : if (sup_parity) generate
        txtb_port_a_parity_calculator_inst : entity ctu_can_fd_rtl.parity_calculator
        generic map (
            G_WIDTH         => 32,
            G_PARITY_TYPE   => C_PARITY_TYPE
        )
        port map(
            data_in         => txtb_port_a_data_in,
            parity          => txtb_port_a_parity
        );
    end generate;

    txtb_parity_false_gen : if (not sup_parity) generate
        txtb_port_a_parity <= '0';
    end generate;


    -----------------------------------------------------------------------------------------------
    -- TXT Buffers
    -----------------------------------------------------------------------------------------------

    txt_buf_comp_gen : for i in 0 to txt_buffer_count - 1 generate
    begin
        txt_buffer_inst : entity ctu_can_fd_rtl.txt_buffer
        generic map (
            G_TXT_BUFFER_COUNT          => txt_buffer_count,
            G_ID                        => i,
            G_TECHNOLOGY                => target_technology,
            G_SUP_PARITY                => sup_parity,
            G_RESET_TXT_BUF_RAM         => reset_buffer_rams
        )
        port map (
            -- Clock and Asynchronous reset
            clk_sys                     => clk_sys,                                     -- IN
            res_n                       => res_core_n,                                  -- IN

            -- DFT support
            scan_enable                 => scan_enable,                                 -- IN

            -- Memory Registers Interface
            mr_mode_bmm                 => mr_ctrl_out.mode_bmm,                        -- IN
            mr_mode_rom                 => mr_ctrl_out.mode_rom,                        -- IN
            mr_mode_txbbm               => mr_ctrl_out.mode_txbbm,                      -- IN
            mr_settings_tbfbo           => mr_ctrl_out.settings_tbfbo,                  -- IN
            mr_settings_pchke           => mr_ctrl_out.settings_pchke,                  -- IN
            mr_tx_command_txce          => mr_ctrl_out.tx_command_txce,                 -- IN
            mr_tx_command_txcr          => mr_ctrl_out.tx_command_txcr,                 -- IN
            mr_tx_command_txca          => mr_ctrl_out.tx_command_txca,                 -- IN
            mr_tx_command_txbi          => mr_tx_command_txbi(i),                       -- IN

            mr_tst_control_tmaena       => mr_tst_out.tst_control_tmaena,               -- IN
            mr_tst_control_twrstb       => mr_tst_out.tst_control_twrstb,               -- IN
            mr_tst_dest_tst_addr        => mr_tst_out.tst_dest_tst_addr(4 downto 0),    -- IN
            mr_tst_dest_tst_mtgt        => mr_tst_out.tst_dest_tst_mtgt,                -- IN
            mr_tst_wdata_tst_wdata      => mr_tst_out.tst_wdata_tst_wdata,              -- IN
            mr_tst_rdata_tst_rdata      => mr_tst_rdata_tst_rdata_txb(i),               -- OUT

            txtb_port_a_data_in         => txtb_port_a_data_in,                         -- IN
            txtb_port_a_parity          => txtb_port_a_parity,                          -- IN
            txtb_port_a_address         => txtb_port_a_address,                         -- IN
            txtb_port_a_cs              => txtb_port_a_cs(i),                           -- IN
            txtb_port_a_be              => txtb_port_a_be,                              -- IN

            txtb_state                  => txtb_state(i),                               -- OUT
            txtb_is_bb                  => txtb_is_bb(i),                               -- IN

            -- Interrupt Manager Interface
            txtb_hw_cmd_int             => txtb_hw_cmd_int(i),                          -- OUT

            -- CAN Core and TX Arbitrator Interface
            txtb_hw_cmd                 => txtb_hw_cmd,                                 -- IN
            txtb_hw_cmd_cs              => txtb_hw_cmd_cs(i),                           -- IN
            txtb_port_b_data_out        => txtb_port_b_data_out(i),                     -- OUT
            txtb_port_b_address         => txtb_port_b_address,                         -- IN
            txtb_port_b_clk_en          => txtb_port_b_clk_en,                          -- IN
            is_bus_off                  => cc_stat.is_bus_off,                          -- IN
            txtb_available              => txtb_available(i),                           -- OUT
            txtb_allow_bb               => txtb_allow_bb(i),                            -- OUT
            txtb_parity_check_valid     => txtb_parity_check_valid(i),                  -- IN
            txtb_parity_mismatch        => txtb_parity_mismatch(i),                     -- OUT
            txtb_parity_error_valid     => txtb_parity_error_valid(i),                  -- OUT
            txtb_bb_parity_error        => txtb_bb_parity_error(i)                      -- OUT
        );
    end generate;

    -----------------------------------------------------------------------------------------------
    -- TX Arbitrator
    -----------------------------------------------------------------------------------------------
    tx_arbitrator_inst : entity ctu_can_fd_rtl.tx_arbitrator
    generic map (
        G_TXT_BUFFER_COUNT              => txt_buffer_count
    )
    port map (
        -- Clock and Asynchronous reset
        clk_sys                         => clk_sys,                         -- IN
        res_n                           => res_core_n,                      -- IN

        -- TXT Buffers interface
        txtb_port_b_data_out            => txtb_port_b_data_out,            -- IN
        txtb_available                  => txtb_available,                  -- IN
        txtb_allow_bb                   => txtb_allow_bb,                   -- OUT
        txtb_port_b_address             => txtb_port_b_address,             -- OUT
        txtb_port_b_clk_en              => txtb_port_b_clk_en,              -- OUT
        txtb_parity_check_valid         => txtb_parity_check_valid,         -- OUT
        txtb_parity_mismatch            => txtb_parity_mismatch,            -- IN
        txtb_is_bb                      => txtb_is_bb,                      -- OUT

        -- CAN Core Interface
        tran_word                       => tran_word,                       -- OUT
        tran_dlc                        => tran_dlc,                        -- OUT
        tran_is_rtr                     => tran_is_rtr,                     -- OUT
        tran_ident_type                 => tran_ident_type,                 -- OUT
        tran_frame_type                 => tran_frame_type,                 -- OUT
        tran_brs                        => tran_brs,                        -- OUT
        tran_identifier                 => tran_identifier,                 -- OUT
        tran_frame_test                 => tran_frame_test,                 -- OUT
        tran_frame_valid                => tran_frame_valid,                -- OUT
        tran_frame_parity_error         => tran_frame_parity_error,         -- OUT
        txtb_hw_cmd                     => txtb_hw_cmd,                     -- IN
        txtb_changed                    => txtb_changed,                    -- OUT
        txtb_hw_cmd_cs                  => txtb_hw_cmd_cs,                  -- OUT
        txtb_ptr                        => txtb_ptr,                        -- IN
        txtb_clk_en                     => txtb_clk_en,                     -- IN

        -- RX Buffer interface
        curr_txtb_index                 => curr_txtb_index,                 -- OUT

        -- Memory registers interface
        mr_mode_tttm                    => mr_ctrl_out.mode_tttm,           -- IN
        mr_mode_txbbm                   => mr_ctrl_out.mode_txbbm,          -- IN
        mr_settings_pchke               => mr_ctrl_out.settings_pchke,      -- IN
        mr_tx_priority                  => mr_tx_priority,                  -- IN

        -- Timestamp
        timestamp                       => timestamp                        -- IN
    );

    -----------------------------------------------------------------------------------------------
    -- Frame Filters
    -----------------------------------------------------------------------------------------------
    frame_filters_inst : entity ctu_can_fd_rtl.frame_filters
    generic map(
        G_SUP_FILTA                     => sup_filtA,
        G_SUP_FILTB                     => sup_filtB,
        G_SUP_FILTC                     => sup_filtC,
        G_SUP_RANGE                     => sup_range
    )
    port map(
        -- Clock an Asynchronous reset
        clk_sys                         => clk_sys,                 -- IN
        res_n                           => res_core_n,              -- IN

        -- Memory registers interface
        mr_filter_control_fafe          => mr_ctrl_out.filter_control_fafe,  -- IN
        mr_filter_control_fafb          => mr_ctrl_out.filter_control_fafb,  -- IN
        mr_filter_control_fane          => mr_ctrl_out.filter_control_fane,  -- IN
        mr_filter_control_fanb          => mr_ctrl_out.filter_control_fanb,  -- IN

        mr_filter_control_fbfe          => mr_ctrl_out.filter_control_fbfe,  -- IN
        mr_filter_control_fbfb          => mr_ctrl_out.filter_control_fbfb,  -- IN
        mr_filter_control_fbne          => mr_ctrl_out.filter_control_fbne,  -- IN
        mr_filter_control_fbnb          => mr_ctrl_out.filter_control_fbnb,  -- IN

        mr_filter_control_fcfe          => mr_ctrl_out.filter_control_fcfe,  -- IN
        mr_filter_control_fcfb          => mr_ctrl_out.filter_control_fcfb,  -- IN
        mr_filter_control_fcne          => mr_ctrl_out.filter_control_fcne,  -- IN
        mr_filter_control_fcnb          => mr_ctrl_out.filter_control_fcnb,  -- IN

        mr_filter_control_frfe          => mr_ctrl_out.filter_control_frfe,  -- IN
        mr_filter_control_frfb          => mr_ctrl_out.filter_control_frfb,  -- IN
        mr_filter_control_frne          => mr_ctrl_out.filter_control_frne,  -- IN
        mr_filter_control_frnb          => mr_ctrl_out.filter_control_frnb,  -- IN

        mr_filter_a_mask_bit_mask_a_val     => mr_ctrl_out.filter_a_mask_bit_mask_a_val,     -- IN
        mr_filter_a_val_bit_val_a_val       => mr_ctrl_out.filter_a_val_bit_val_a_val,       -- IN
        mr_filter_b_mask_bit_mask_b_val     => mr_ctrl_out.filter_b_mask_bit_mask_b_val,     -- IN
        mr_filter_b_val_bit_val_b_val       => mr_ctrl_out.filter_b_val_bit_val_b_val,       -- IN
        mr_filter_c_mask_bit_mask_c_val     => mr_ctrl_out.filter_c_mask_bit_mask_c_val,     -- IN
        mr_filter_c_val_bit_val_c_val       => mr_ctrl_out.filter_c_val_bit_val_c_val,       -- IN
        mr_filter_ran_high_bit_ran_high_val => mr_ctrl_out.filter_ran_high_bit_ran_high_val, -- IN
        mr_filter_ran_low_bit_ran_low_val   => mr_ctrl_out.filter_ran_low_bit_ran_low_val,   -- IN

        mr_settings_fdrf                    => mr_ctrl_out.settings_fdrf,   -- IN
        mr_mode_afm                         => mr_ctrl_out.mode_afm,        -- IN

        -- CAN Core interface
        rec_ident                       => rec_ident,                       -- IN
        rec_ident_type                  => rec_ident_type,                  -- IN
        rec_frame_type                  => rec_frame_type,                  -- IN
        rec_is_rtr                      => rec_is_rtr,                      -- IN
        store_metadata                  => store_metadata,                  -- IN
        store_data                      => store_data,                      -- IN
        rec_valid                       => rec_valid,                       -- IN
        rec_abort                       => rec_abort,                       -- IN

        -- Frame filters output
        store_metadata_f                => store_metadata_f,                -- OUT
        store_data_f                    => store_data_f,                    -- OUT
        rec_valid_f                     => rec_valid_f,                     -- OUT
        rec_abort_f                     => rec_abort_f                      -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- Interrrupt Manager
    -----------------------------------------------------------------------------------------------
    int_manager_inst : entity ctu_can_fd_rtl.int_manager
    generic map(
        G_INT_COUNT                     => C_INT_COUNT,
        G_TXT_BUFFER_COUNT              => txt_buffer_count
    )
    port map(
        -- Clock and Asynchronous reset
        clk_sys                         => clk_sys,                                 -- IN

        -- WARNING:
        --      Interrupt manager MUST be reset by Soft reset, not by Core
        --      reset since it holds the actual values of Interrupt Enable
        --      and Interrupt Mask. These are considered as registers and
        --      they must be settable when the core is disabled!
        res_n                           => res_soft_n,                              -- IN

        -- Interrupt sources
        err_detected                    => err_detected,                            -- IN
        fcs_changed                     => fcs_changed,                             -- IN
        err_warning_limit_pulse         => err_warning_limit_pulse,                 -- IN
        arbitration_lost                => arbitration_lost,                        -- IN
        tran_valid                      => tran_valid,                              -- IN
        br_shifted                      => br_shifted,                              -- IN
        rx_data_overrun                 => rx_data_overrun,                         -- IN
        rec_valid                       => rec_valid,                               -- IN
        rx_full                         => rx_full,                                 -- IN
        rx_empty                        => rx_empty,                                -- IN
        txtb_hw_cmd_int                 => txtb_hw_cmd_int,                         -- IN
        is_overload                     => pc_dbg.is_overload,                      -- IN

        -- Memory registers Interface
        mr_int_ena_set_int_ena_set      => mr_ctrl_out.int_ena_set_int_ena_set,     -- IN
        mr_int_ena_clr_int_ena_clr      => mr_ctrl_out.int_ena_clr_int_ena_clr,     -- IN
        mr_int_mask_set_int_mask_set    => mr_ctrl_out.int_mask_set_int_mask_set,   -- IN
        mr_int_mask_clr_int_mask_clr    => mr_ctrl_out.int_mask_clr_int_mask_clr,   -- IN

        mr_int_stat_rxi_i               => mr_ctrl_out.int_stat_rxi,                -- IN
        mr_int_stat_txi_i               => mr_ctrl_out.int_stat_txi,                -- IN
        mr_int_stat_ewli_i              => mr_ctrl_out.int_stat_ewli,               -- IN
        mr_int_stat_doi_i               => mr_ctrl_out.int_stat_doi,                -- IN
        mr_int_stat_fcsi_i              => mr_ctrl_out.int_stat_fcsi,               -- IN
        mr_int_stat_ali_i               => mr_ctrl_out.int_stat_ali,                -- IN
        mr_int_stat_bei_i               => mr_ctrl_out.int_stat_bei,                -- IN
        mr_int_stat_ofi_i               => mr_ctrl_out.int_stat_ofi,                -- IN
        mr_int_stat_rxfi_i              => mr_ctrl_out.int_stat_rxfi,               -- IN
        mr_int_stat_bsi_i               => mr_ctrl_out.int_stat_bsi,                -- IN
        mr_int_stat_rbnei_i             => mr_ctrl_out.int_stat_rbnei,              -- IN
        mr_int_stat_txbhci_i            => mr_ctrl_out.int_stat_txbhci,             -- IN

        -- Reads from Interrupt registers
        mr_int_stat_rxi_o               => mr_int_stat_rxi_o,                       -- OUT
        mr_int_stat_txi_o               => mr_int_stat_txi_o,                       -- OUT
        mr_int_stat_ewli_o              => mr_int_stat_ewli_o,                      -- OUT
        mr_int_stat_doi_o               => mr_int_stat_doi_o,                       -- OUT
        mr_int_stat_fcsi_o              => mr_int_stat_fcsi_o,                      -- OUT
        mr_int_stat_ali_o               => mr_int_stat_ali_o,                       -- OUT
        mr_int_stat_bei_o               => mr_int_stat_bei_o,                       -- OUT
        mr_int_stat_ofi_o               => mr_int_stat_ofi_o,                       -- OUT
        mr_int_stat_rxfi_o              => mr_int_stat_rxfi_o,                      -- OUT
        mr_int_stat_bsi_o               => mr_int_stat_bsi_o,                       -- OUT
        mr_int_stat_rbnei_o             => mr_int_stat_rbnei_o,                     -- OUT
        mr_int_stat_txbhci_o            => mr_int_stat_txbhci_o,                    -- OUT

        mr_int_ena_set_int_ena_set_o    => mr_int_ena_set_int_ena_set_o,            -- OUT
        mr_int_mask_set_int_mask_set_o  => mr_int_mask_set_int_mask_set_o,          -- OUT

        -- Interrupt output
        int                             => int                                      -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- CAN Core
    -----------------------------------------------------------------------------------------------
    can_core_inst : entity ctu_can_fd_rtl.can_core
    generic map (
        G_SAMPLE_TRIGGER_COUNT          => C_SAMPLE_TRIGGER_COUNT,
        G_CTRL_CTR_WIDTH                => C_CTRL_CTR_WIDTH,
        G_RETR_LIM_CTR_WIDTH            => C_RETR_LIM_CTR_WIDTH,
        G_ERR_VALID_PIPELINE            => C_ERR_VALID_PIPELINE,
        G_CRC15_POL                     => C_CRC15_POL,
        G_CRC17_POL                     => C_CRC17_POL,
        G_CRC21_POL                     => C_CRC21_POL,
        G_SUP_TRAFFIC_CTRS              => sup_traffic_ctrs
    )
    port map (
        -- Clock and Asynchronous reset
        clk_sys                         => clk_sys,                                 -- IN
        res_n                           => res_core_n,                              -- IN

        -- DFT support
        scan_enable                     => scan_enable,                             -- IN

        -- Memory registers interface
        mr_mode_acf                     => mr_ctrl_out.mode_acf,                     -- IN
        mr_mode_stm                     => mr_ctrl_out.mode_stm,                     -- IN
        mr_mode_bmm                     => mr_ctrl_out.mode_bmm,                     -- IN
        mr_mode_fde                     => mr_ctrl_out.mode_fde,                     -- IN
        mr_mode_rom                     => mr_ctrl_out.mode_rom,                     -- IN
        mr_mode_tstm                    => mr_ctrl_out.mode_tstm,                    -- IN

        mr_settings_ena                 => mr_ctrl_out.settings_ena,                 -- IN
        mr_settings_nisofd              => mr_ctrl_out.settings_nisofd,              -- IN
        mr_settings_rtrth               => mr_ctrl_out.settings_rtrth,               -- IN
        mr_settings_rtrle               => mr_ctrl_out.settings_rtrle,               -- IN
        mr_settings_ilbp                => mr_ctrl_out.settings_ilbp,                -- IN
        mr_settings_pex                 => mr_ctrl_out.settings_pex,                 -- IN

        mr_command_ercrst               => mr_ctrl_out.command_ercrst,               -- IN
        mr_command_rxfcrst              => mr_ctrl_out.command_rxfcrst,              -- IN
        mr_command_txfcrst              => mr_ctrl_out.command_txfcrst,              -- IN
        mr_command_cpexs                => mr_ctrl_out.command_cpexs,                -- IN

        mr_ssp_cfg_ssp_src              => mr_ctrl_out.ssp_cfg_ssp_src,              -- IN

        mr_ewl_ew_limit                 => mr_ctrl_out.ewl_ew_limit,                 -- IN
        mr_erp_erp_limit                => mr_ctrl_out.erp_erp_limit,                -- IN

        mr_ctr_pres_ctpv                => mr_ctrl_out.ctr_pres_ctpv,                -- IN
        mr_ctr_pres_ptx                 => mr_ctrl_out.ctr_pres_ptx,                 -- IN
        mr_ctr_pres_prx                 => mr_ctrl_out.ctr_pres_prx,                 -- IN
        mr_ctr_pres_enorm               => mr_ctrl_out.ctr_pres_enorm,               -- IN
        mr_ctr_pres_efd                 => mr_ctrl_out.ctr_pres_efd,                 -- IN

        -- Status signals
        cc_stat                         => cc_stat,                                 -- OUT
        pc_dbg                          => pc_dbg,                                  -- OUT

        -- Tx Arbitrator and TXT Buffers interface
        tran_word                       => tran_word,                               -- IN
        tran_dlc                        => tran_dlc,                                -- IN
        tran_is_rtr                     => tran_is_rtr,                             -- IN
        tran_ident_type                 => tran_ident_type,                         -- IN
        tran_frame_type                 => tran_frame_type,                         -- IN
        tran_brs                        => tran_brs,                                -- IN
        tran_identifier                 => tran_identifier,                         -- IN
        tran_frame_test                 => tran_frame_test,                         -- IN
        tran_frame_valid                => tran_frame_valid,                        -- IN
        tran_frame_parity_error         => tran_frame_parity_error,                 -- IN
        txtb_hw_cmd                     => txtb_hw_cmd,                             -- OUT
        txtb_changed                    => txtb_changed,                            -- IN
        txtb_ptr                        => txtb_ptr,                                -- OUT
        txtb_clk_en                     => txtb_clk_en,                             -- OUT

        -- Recieve Buffer and Message Filter Interface
        rec_ident                       => rec_ident,                               -- OUT
        rec_dlc                         => rec_dlc,                                 -- OUT
        rec_ident_type                  => rec_ident_type,                          -- OUT
        rec_frame_type                  => rec_frame_type,                          -- OUT
        rec_lbpf                        => rec_lbpf,                                -- OUT
        rec_is_rtr                      => rec_is_rtr,                              -- OUT
        rec_brs                         => rec_brs,                                 -- OUT
        rec_esi                         => rec_esi,                                 -- OUT
        rec_valid                       => rec_valid,                               -- OUT
        store_metadata                  => store_metadata,                          -- OUT
        store_data                      => store_data,                              -- OUT
        store_data_word                 => store_data_word,                         -- OUT
        rec_abort                       => rec_abort,                               -- OUT
        sof_pulse                       => sof_pulse,                               -- OUT

        -- Interrupt Manager Interface
        arbitration_lost                => arbitration_lost,                        -- OUT
        tran_valid                      => tran_valid,                              -- OUT
        br_shifted                      => br_shifted,                              -- OUT
        err_detected                    => err_detected,                            -- OUT
        fcs_changed                     => fcs_changed,                             -- OUT
        err_warning_limit_pulse         => err_warning_limit_pulse,                 -- OUT

        -- Prescaler interface
        rx_triggers                     => rx_triggers,                             -- IN
        tx_trigger                      => tx_trigger,                              -- IN
        sync_control                    => sync_control,                            -- OUT
        no_pos_resync                   => no_pos_resync,                           -- OUT
        sp_control                      => sp_control,                              -- OUT
        nbt_ctrs_en                     => nbt_ctrs_en,                             -- OUT
        dbt_ctrs_en                     => dbt_ctrs_en,                             -- OUT

        -- CAN Bus serial data stream
        rx_data_wbs                     => rx_data_wbs,                             -- IN
        tx_data_wbs                     => tx_data_wbs,                             -- OUT

        -- Others
        ssp_reset                       => ssp_reset,                               -- OUT
        tran_delay_meas                 => tran_delay_meas,                         -- OUT
        bit_err                         => bit_err,                                 -- IN
        btmc_reset                      => btmc_reset,                              -- OUT
        dbt_measure_start               => dbt_measure_start,                       -- OUT
        gen_first_ssp                   => gen_first_ssp,                           -- OUT
        sync_edge                       => sync_edge,                               -- IN
        pc_rx_trigger                   => pc_rx_trigger                            -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- Prescaler
    -----------------------------------------------------------------------------------------------
    prescaler_inst : entity ctu_can_fd_rtl.prescaler
    generic map (
        G_TSEG1_NBT_WIDTH               => C_TSEG1_NBT_WIDTH,
        G_TSEG2_NBT_WIDTH               => C_TSEG2_NBT_WIDTH,
        G_BRP_NBT_WIDTH                 => C_BRP_NBT_WIDTH,
        G_SJW_NBT_WIDTH                 => C_SJW_NBT_WIDTH,
        G_TSEG1_DBT_WIDTH               => C_TSEG1_DBT_WIDTH,
        G_TSEG2_DBT_WIDTH               => C_TSEG2_DBT_WIDTH,
        G_BRP_DBT_WIDTH                 => C_BRP_DBT_WIDTH,
        G_SJW_DBT_WIDTH                 => C_SJW_DBT_WIDTH,
        G_SAMPLE_TRIGGER_COUNT          => C_SAMPLE_TRIGGER_COUNT
    )
    port map (
        -- Clock and Asynchronous reset
        clk_sys                         => clk_sys,                                 -- IN
        res_n                           => res_core_n,                              -- IN

        -- Memory registers interface
        mr_settings_ena                 => mr_ctrl_out.settings_ena,                -- IN

        mr_btr_prop                     => mr_ctrl_out.btr_prop,                    -- IN
        mr_btr_ph1                      => mr_ctrl_out.btr_ph1,                     -- IN
        mr_btr_ph2                      => mr_ctrl_out.btr_ph2,                     -- IN
        mr_btr_brp                      => mr_ctrl_out.btr_brp,                     -- IN
        mr_btr_sjw                      => mr_ctrl_out.btr_sjw,                     -- IN

        mr_btr_fd_prop_fd               => mr_ctrl_out.btr_fd_prop_fd,              -- IN
        mr_btr_fd_ph1_fd                => mr_ctrl_out.btr_fd_ph1_fd,               -- IN
        mr_btr_fd_ph2_fd                => mr_ctrl_out.btr_fd_ph2_fd,               -- IN
        mr_btr_fd_brp_fd                => mr_ctrl_out.btr_fd_brp_fd,               -- IN
        mr_btr_fd_sjw_fd                => mr_ctrl_out.btr_fd_sjw_fd,               -- IN

        -- Control Interface
        sync_edge                       => sync_edge,                               -- IN
        sp_control                      => sp_control,                              -- IN
        sync_control                    => sync_control,                            -- IN
        no_pos_resync                   => no_pos_resync,                           -- IN
        nbt_ctrs_en                     => nbt_ctrs_en,                             -- IN
        dbt_ctrs_en                     => dbt_ctrs_en,                             -- IN

        -- Trigger signals
        rx_triggers                     => rx_triggers,                             -- OUT
        tx_trigger                      => tx_trigger,                              -- OUT

        -- Status outputs
        tq_edge                         => tq_edge                                  -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- Bus Sampling
    -----------------------------------------------------------------------------------------------
    bus_sampling_inst : entity ctu_can_fd_rtl.bus_sampling
    generic map(
        G_SSP_DELAY_SAT_VAL             => C_SSP_DELAY_SAT_VAL,
        G_TX_CACHE_DEPTH                => C_TX_CACHE_DEPTH,
        G_TRV_CTR_WIDTH                 => C_TRV_CTR_WIDTH,
        G_SSP_POS_WIDTH                 => C_SSP_POS_WIDTH,
        G_SSP_CTRS_WIDTH                => C_SSP_CTRS_WIDTH
    )
    port map(
        -- Clock and Async reset
        clk_sys                         => clk_sys,                                 -- IN
        res_n                           => res_core_n,                              -- IN

        -- DFT support
        scan_enable                     => scan_enable,                             -- IN

        -- Physical layer interface
        can_rx                          => can_rx,                                  -- IN
        can_tx                          => can_tx,                                  -- OUT

        -- Memory registers interface
        mr_settings_ena                 => mr_ctrl_out.settings_ena,                -- IN
        mr_ssp_cfg_ssp_offset           => mr_ctrl_out.ssp_cfg_ssp_offset,          -- IN
        mr_ssp_cfg_ssp_src              => mr_ctrl_out.ssp_cfg_ssp_src,             -- IN

        trv_delay                       => trv_delay,                               -- OUT

        -- Prescaler interface
        rx_trigger                      => rx_triggers(1),                          -- IN
        tx_trigger                      => tx_trigger,                              -- IN
        sync_edge                       => sync_edge,                               -- OUT
        tq_edge                         => tq_edge,                                 -- IN

        -- CAN Core Interface
        tx_data_wbs                     => tx_data_wbs,                             -- IN
        rx_data_wbs                     => rx_data_wbs,                             -- OUT
        sp_control                      => sp_control,                              -- IN
        ssp_reset                       => ssp_reset,                               -- IN
        tran_delay_meas                 => tran_delay_meas,                         -- IN
        bit_err                         => bit_err,                                 -- OUT
        btmc_reset                      => btmc_reset,                              -- IN
        dbt_measure_start               => dbt_measure_start,                       -- IN
        gen_first_ssp                   => gen_first_ssp                            -- IN
    );

    -- <RELEASE_OFF>
    -- pragma translate_off
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl no_tx_dominant_when_disabled_asrt : assert never
    --  (mr_ctrl_out.settings_ena = '0' and can_tx = DOMINANT)
    --  report "Dominant bit can't be transmitted when Node is disabled!";

    -- psl no_tx_dominant_when_bus_monitoring_asrt : assert never
    --  (mr_ctrl_out.mode_bmm = '1' and can_tx = DOMINANT)
    --  report "Dominant bit can't be transmitted in Bus monitoring mode!";

    -- Each TXT Buffer should have been unlocked already when unit is
    -- transmitting overload frame! This-way we can be sure that no we can
    -- block unlock command in Protocol control FSM in overload frames!

    txtb_asr_gen : for i in 0 to txt_buffer_count - 1 generate
    begin

        process (txtb_state, pc_dbg.is_overload)
        begin
            if ((((txtb_state(i) = TXT_TRAN) or (txtb_state(i) = TXT_ABTP))) and
                  (pc_dbg.is_overload = '1')) then
                report "TXT Buffer should have been unlocked when node is in Overload frame!"
                severity error;
            end if;
        end process;

    end generate;

    -- pragma translate_on
    -- <RELEASE_ON>

end architecture;