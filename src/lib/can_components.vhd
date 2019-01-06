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
--  Package for components declarations to avoid writing component declarations
--  every time into architecture itself. Do not use comments on signals in this
--  file, comment the signal in the entity declaration!
--------------------------------------------------------------------------------
-- Revision History:
--    15.11.2017   Created file
--    27.11.2017   Added "rst_sync" asynchronous rest synchroniser circuit
--    29.11.2017   Removed "rec_data" between Protocol control and RX Buffer,
--                                 replaced with rec_dram_word and
--                 rec_dram_addr as part of resource optimization.
--    30.11.2017   Updated "txt_buffer" for direct access to buffer
--    29.9.2018    Added "inf_RAM_wrapper".
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use work.can_types.all;
use work.can_constants.all;
use work.can_registers_pkg.all;

package can_components is

    ------------------------------------------------------------------------------
    ------------------------------------------------------------------------------
    ---- CAN FD Core top level entity
    ------------------------------------------------------------------------------
    ------------------------------------------------------------------------------
    component can_top_level is
        generic(
            constant use_logger     : boolean               := true;
            constant rx_buffer_size : natural range 32 to 4096 := 128;
            constant use_sync       : boolean               := true;
            constant ID             : natural range 0 to 15 := 1;
            constant sup_filtA      : boolean               := true;
            constant sup_filtB      : boolean               := true;
            constant sup_filtC      : boolean               := true;
            constant sup_range      : boolean               := true;
            constant logger_size    : natural --range 0 to 512:=8
        );
        port(
            signal clk_sys         : in  std_logic;
            signal res_n           : in  std_logic;
            signal data_in         : in  std_logic_vector(31 downto 0);
            signal data_out        : out std_logic_vector(31 downto 0);
            signal adress          : in  std_logic_vector(23 downto 0);
            signal scs             : in  std_logic;
            signal srd             : in  std_logic;
            signal swr             : in  std_logic;
            signal sbe             : in  std_logic_vector(3 downto 0);
            signal int             : out std_logic;
            signal CAN_tx          : out std_logic;
            signal CAN_rx          : in  std_logic;
            signal time_quanta_clk : out std_logic;
            -- synthesis translate_off
            signal drv_bus_o    : out std_logic_vector(1023 downto 0);
            signal stat_bus_o   : out std_logic_vector(511 downto 0);
            -- synthesis translate_on
            signal timestamp       : in  std_logic_vector(63 downto 0)
      );
    end component;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    ---- CAN Top level components
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------


    ----------------------------------------------------------------------------
    -- Registers
    ----------------------------------------------------------------------------
    component memory_registers is
        generic(
            constant compType      : std_logic_vector(3 downto 0) := CAN_COMPONENT_TYPE;
            constant use_logger    : boolean                      := true;
            constant sup_filtA     : boolean                      := true;
            constant sup_filtB     : boolean                      := true;
            constant sup_filtC     : boolean                      := true;
            constant sup_range     : boolean                      := true;
            constant sup_be        : boolean                      := false;
            constant buf_count     : natural range 0 to 7         := 2;
            constant ID            : natural;
            constant DEVICE_ID     : std_logic_vector(15 downto 0);
            constant VERSION_MINOR : std_logic_vector(7 downto 0);
            constant VERSION_MAJOR : std_logic_vector(7 downto 0)
        );
        port(
            signal clk_sys              : in  std_logic;
            signal res_n                : in  std_logic;
            signal res_out              : out std_logic;
            signal data_in              : in  std_logic_vector(31 downto 0);
            signal data_out             : out std_logic_vector(31 downto 0);
            signal adress               : in  std_logic_vector(23 downto 0);
            signal scs                  : in  std_logic;
            signal srd                  : in  std_logic;
            signal swr                  : in  std_logic;
            signal sbe                  : in  std_logic_vector(3 downto 0);
            signal timestamp            : in  std_logic_vector(63 downto 0);
            signal drv_bus              : out std_logic_vector(1023 downto 0);
            signal stat_bus             : in  std_logic_vector(511 downto 0);
            signal rx_read_buff         : in  std_logic_vector(31 downto 0);
            signal rx_buf_size          : in  std_logic_vector(12 downto 0);
            signal rx_full              : in  std_logic;
            signal rx_empty             : in  std_logic;
            signal rx_message_count     : in  std_logic_vector(10 downto 0);
            signal rx_mem_free          : in  std_logic_vector(12 downto 0);
            signal rx_read_pointer_pos  : in  std_logic_vector(11 downto 0);
            signal rx_write_pointer_pos : in  std_logic_vector(11 downto 0);
            signal rx_data_overrun      : in  std_logic;
            signal tran_data            : out std_logic_vector(31 downto 0);
            signal tran_addr            : out std_logic_vector(4 downto 0);

            signal txtb_cs              : out std_logic_vector(
                                                buf_count - 1 downto 0);

            signal txtb_state           : in  txtb_state_type;
            signal txt_sw_cmd           : out txt_sw_cmd_type;

            signal txt_buf_cmd_index    : out std_logic_vector(
                                                buf_count - 1 downto 0);

            signal txt_buf_prior_out    : out txtb_priorities_type;
            signal trv_delay_out        : in  std_logic_vector(15 downto 0);

            signal int_vector           : in  std_logic_vector(
                                                INT_COUNT - 1 downto 0);

            signal int_ena              : in  std_logic_vector(
                                                INT_COUNT - 1 downto 0);

            signal int_mask             : in  std_logic_vector(
                                                INT_COUNT - 1 downto 0);

            signal loger_act_data       : in  std_logic_vector(63 downto 0);
            signal log_write_pointer    : in  std_logic_vector(7 downto 0);
            signal log_read_pointer     : in  std_logic_vector(7 downto 0);
            signal log_size             : in  std_logic_vector(7 downto 0);
            signal log_state_out        : in  logger_state_type
        );
    end component;

    ----------------------------------------------------------------------------
    -- Control registers sub-module
    ----------------------------------------------------------------------------
    component control_registers_reg_map is
        generic (
            constant DATA_WIDTH          : natural := 32;
            constant ADDRESS_WIDTH       : natural := 8;
            constant REGISTERED_READ     : boolean := true;
            constant CLEAR_READ_DATA     : boolean := true;
            constant RESET_POLARITY      : std_logic := '0';
            constant SUP_FILT_A          : boolean := true;
            constant SUP_RANGE           : boolean := true;
            constant SUP_FILT_C          : boolean := true;
            constant SUP_FILT_B          : boolean := true
        );
        port (
            signal clk_sys               :in std_logic;
            signal res_n                 :in std_logic;
            signal address               :in std_logic_vector(address_width - 1 downto 0);
            signal w_data                :in std_logic_vector(data_width - 1 downto 0);
            signal r_data                :out std_logic_vector(data_width - 1 downto 0);
            signal cs                    :in std_logic;
            signal read                  :in std_logic;
            signal write                 :in std_logic;
            signal be                    :in std_logic_vector(data_width / 8 - 1 downto 0);
            signal control_registers_out :out Control_registers_out_t;
            signal control_registers_in  :in Control_registers_in_t
        );
    end component control_registers_reg_map;


    ----------------------------------------------------------------------------
    -- Event logger registers sub-module
    ----------------------------------------------------------------------------
    component event_logger_reg_map is
        generic (
            constant DATA_WIDTH          : natural := 32;
            constant ADDRESS_WIDTH       : natural := 8;
            constant REGISTERED_READ     : boolean := true;
            constant CLEAR_READ_DATA     : boolean := true;
            constant RESET_POLARITY      : std_logic := '0'
        );
        port (
            signal clk_sys               :in std_logic;
            signal res_n                 :in std_logic;
            signal address               :in std_logic_vector(address_width - 1 downto 0);
            signal w_data                :in std_logic_vector(data_width - 1 downto 0);
            signal r_data                :out std_logic_vector(data_width - 1 downto 0);
            signal cs                    :in std_logic;
            signal read                  :in std_logic;
            signal write                 :in std_logic;
            signal be                    :in std_logic_vector(data_width / 8 - 1 downto 0);
            signal event_logger_out      :out Event_Logger_out_t;
            signal event_logger_in       :in Event_Logger_in_t
        );
    end component event_logger_reg_map;


    ----------------------------------------------------------------------------
    -- RX Buffer module
    ----------------------------------------------------------------------------
    component rx_buffer is
        generic(
            buff_size                   :       natural range 32 to 4096 := 32
        );
        port(
            signal clk_sys              :in     std_logic; --System clock
            signal res_n                :in     std_logic; --Async. reset
            signal rec_ident_in         :in     std_logic_vector(28 downto 0);
            signal rec_dlc_in           :in     std_logic_vector(3 downto 0);
            signal rec_ident_type_in    :in     std_logic;
            signal rec_frame_type_in    :in     std_logic;
            signal rec_is_rtr           :in     std_logic;
            signal rec_brs              :in     std_logic;
            signal rec_esi              :in     std_logic;
            signal store_metadata       :in     std_logic;
            signal store_data           :in     std_logic;
            signal store_data_word      :in     std_logic_vector(31 downto 0);
            signal rec_message_valid    :in     std_logic;
            signal rec_abort            :in     std_logic;
            signal sof_pulse            :in     std_logic;
            signal rx_buf_size          :out    std_logic_vector(12 downto 0);
            signal rx_full              :out    std_logic;
            signal rx_empty             :out    std_logic;
            signal rx_message_count     :out    std_logic_vector(10 downto 0);
            signal rx_mem_free          :out    std_logic_vector(12 downto 0);
            signal rx_read_pointer_pos  :out    std_logic_vector(11 downto 0);
            signal rx_write_pointer_pos :out    std_logic_vector(11 downto 0);
            signal rx_data_overrun      :out    std_logic;
            signal timestamp            :in     std_logic_vector(63 downto 0);
            signal rx_read_buff         :out    std_logic_vector(31 downto 0);
            signal drv_bus              :in     std_logic_vector(1023 downto 0)
        );
    end component;


    ----------------------------------------------------------------------------
    -- RX Buffer FSM
    ----------------------------------------------------------------------------    
    component rx_buffer_fsm is
        port(
            signal clk_sys              :in     std_logic; --System clock
            signal res_n                :in     std_logic; --Async. reset
            signal store_metadata       :in     std_logic;
            signal store_data           :in     std_logic;
            signal rec_message_valid    :in     std_logic;
            signal rec_abort            :in     std_logic;
            signal sof_pulse            :in     std_logic;
            signal drv_bus              :in     std_logic_vector(1023 downto 0);
            signal write_raw_intent     :out    std_logic;
            signal write_extra_ts       :out    std_logic;
            signal store_extra_ts_end   :out    std_logic;
            signal data_selector        :out    std_logic_vector(6 downto 0);
            signal store_extra_wr_ptr   :out    std_logic;
            signal inc_extra_wr_ptr     :out    std_logic;
            signal reset_overrun_flag   :out    std_logic
        );
    end component;


    ----------------------------------------------------------------------------
    -- RX Buffer Pointers
    ----------------------------------------------------------------------------    
    component rx_buffer_pointers is
    generic(
        buff_size                     :       natural range 32 to 4096 := 32
    );
    port(
        signal clk_sys                :in     std_logic; --System clock
        signal res_n                  :in     std_logic; --Async. reset
        signal rec_abort              :in     std_logic;
        signal commit_rx_frame        :in     std_logic;
        signal write_raw_OK           :in     std_logic;
        signal commit_overrun_abort   :in     std_logic;
        signal store_extra_wr_ptr     :in     std_logic;
        signal inc_extra_wr_ptr       :in     std_logic;
        signal read_increment         :in     std_logic;
        signal drv_bus                :in     std_logic_vector(1023 downto 0);
        signal read_pointer           :out    integer range 0 to buff_size - 1;
        signal read_pointer_inc_1     :out    integer range 0 to buff_size - 1;
        signal write_pointer          :out    integer range 0 to buff_size - 1;
        signal write_pointer_raw      :out    integer range 0 to buff_size - 1;
        signal write_pointer_extra_ts :out    integer range 0 to buff_size - 1;
        signal rx_mem_free_int        :out    integer range 0 to buff_size + 1
    );
    end component;


    ----------------------------------------------------------------------------
    -- Inferred RAM wrapper
    ----------------------------------------------------------------------------    
    component inf_ram_wrapper is
        generic(
            constant word_width           :     natural := 32;
            constant depth                :     natural := 32;
            constant address_width        :     natural := 8;
            constant reset_polarity       :     std_logic := '1';
            constant simulation_reset     :     boolean := true;
            constant sync_read            :     boolean := true
        );
        port(
            signal clk_sys                :in   std_logic;
            signal res_n                  :in   std_logic;
            signal addr_A                 :in   std_logic_vector(address_width -1
                                                downto 0);
            signal write                  :in   std_logic;
            signal data_in                :in   std_logic_vector(word_width - 1
                                                downto 0);
            signal addr_B                 :in   std_logic_vector(address_width - 1
                                                downto 0);
            signal data_out               :out  std_logic_vector(word_width - 1
                                                downto 0)
        );
    end component;


    ----------------------------------------------------------------------------
    -- TXT Buffer module
    ----------------------------------------------------------------------------
    component txt_buffer is
        generic(
            constant buf_count            :     natural range 1 to 8;
            constant ID                   :     natural := 1
        );
        port(
            signal clk_sys                :in   std_logic;
            signal res_n                  :in   std_logic; --Async reset
            signal tran_data              :in   std_logic_vector(31 downto 0);
            signal tran_addr              :in   std_logic_vector(4 downto 0);
            signal tran_cs                :in   std_logic;
            signal txt_sw_cmd             :in   txt_sw_cmd_type;
            signal txt_hw_cmd_int         :out  std_logic;
            signal txt_sw_buf_cmd_index   :in   std_logic_vector(
                                                    buf_count - 1 downto 0);

            signal txtb_state             :out  std_logic_vector(3 downto 0);
            signal txt_hw_cmd             :in   txt_hw_cmd_type;
            signal bus_off_start          :in   std_logic;
            signal txt_hw_cmd_buf_index   :in   natural range 0 to buf_count - 1;
            signal txt_word               :out  std_logic_vector(31 downto 0);
            signal txt_addr               :in   natural range 0 to 19;
            signal txt_buf_ready          :out  std_logic
        );
    end component;


    ----------------------------------------------------------------------------
    -- TXT Buffer FSM 
    ----------------------------------------------------------------------------
    component txt_buffer_fsm is
    generic(
        constant ID                   :     natural
    );
    port(
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;
        signal txt_sw_cmd             :in   txt_sw_cmd_type;
        signal sw_cbs                 :in   std_logic;
        signal txt_hw_cmd             :in   txt_hw_cmd_type;  
        signal hw_cbs                 :in   std_logic;
        signal bus_off_start          :in   std_logic;
        signal txtb_user_accessible   :out  std_logic;
        signal txtb_hw_cmd_int        :out  std_logic;
        signal txtb_state             :out  std_logic_vector(3 downto 0);
        signal txt_buf_ready          :out  std_logic
    );
    end component;

    ----------------------------------------------------------------------------
    -- TXT Arbitrator module
    ----------------------------------------------------------------------------
    component tx_arbitrator is
    generic(
        constant buf_count            :    natural range 1 to 8
    );
    port(
        signal clk_sys                :in  std_logic;
        signal res_n                  :in  std_logic;
        signal txt_buf_in             :in txtb_output_type;

        signal txt_buf_ready          :in std_logic_vector(buf_count - 1 downto 0);
        signal txtb_ptr               :out natural range 0 to 19;
        signal tran_data_word_out     :out std_logic_vector(31 downto 0);
        signal tran_dlc_out           :out std_logic_vector(3 downto 0);
        signal tran_is_rtr            :out std_logic;
        signal tran_ident_type_out    :out std_logic;
        signal tran_frame_type_out    :out std_logic;
        signal tran_brs_out           :out std_logic;
        signal tran_frame_valid_out   :out std_logic;
        signal txt_hw_cmd             :in txt_hw_cmd_type;
        signal txtb_changed           :out std_logic;
        signal txt_hw_cmd_buf_index   :out natural range 0 to buf_count - 1;
        signal txtb_core_pointer      :in natural range 0 to 19;
        signal drv_bus                :in std_logic_vector(1023 downto 0);
        signal txt_buf_prio           :in txtb_priorities_type;
        signal timestamp              :in std_logic_vector(63 downto 0)
    );
    end component;


    ----------------------------------------------------------------------------
    -- Priority decoder for TXT Buffer selection
    ----------------------------------------------------------------------------
    component priority_decoder is
    generic(
        constant buf_count          :   natural range 1 to 8
    );
    port(
        signal prio                 : in  txtb_priorities_type;
        signal prio_valid           : in  std_logic_vector(
                                            buf_count - 1 downto 0);
        signal output_valid         : out  std_logic;
        signal output_index         : out  natural range 0 to buf_count - 1
    );
    end component;


    ----------------------------------------------------------------------------
    -- TX Arbitrator FSM
    ----------------------------------------------------------------------------    
    component tx_arbitrator_fsm is
    port( 
        signal clk_sys                :in  std_logic;
        signal res_n                  :in  std_logic;

        signal select_buf_avail       :in  std_logic;
        signal select_index_changed   :in  std_logic;
        signal timestamp_valid        :in  std_logic;
        signal txt_hw_cmd             :in txt_hw_cmd_type;  

        signal load_ts_lw_addr        :out std_logic;
        signal load_ts_uw_addr        :out std_logic;
        signal load_ffmt_w_addr       :out std_logic;
        signal store_ts_l_w           :out std_logic;
        signal store_md_w             :out std_logic;
        signal tx_arb_locked          :out std_logic;
        signal store_last_txtb_index  :out std_logic;
        signal frame_valid_com_set    :out std_logic;
        signal frame_valid_com_clear  :out std_logic 
    );
    end component;


    ----------------------------------------------------------------------------
    -- Frame filters module
    ----------------------------------------------------------------------------
    component frame_filters is
        generic(
            constant sup_filtA : boolean := true;
            constant sup_filtB : boolean := true;
            constant sup_filtC : boolean := true;
            constant sup_range : boolean := true
        );
        port(
            signal clk_sys         : in  std_logic;
            signal res_n           : in  std_logic;
            signal rec_ident_in    : in  std_logic_vector(28 downto 0);
            signal ident_type      : in  std_logic;
            signal frame_type      : in  std_logic;
            signal rec_ident_valid : in  std_logic;
            signal drv_bus         : in  std_logic_vector(1023 downto 0);
            signal out_ident_valid : out std_logic
        );
    end component;


    ----------------------------------------------------------------------------
    -- Generic Bit Filter
    ----------------------------------------------------------------------------
    component bit_filter is
    generic(
        constant width              :   natural;
        constant is_present         :   boolean
    );
    port(
        signal filter_mask          : in  std_logic_vector(width - 1 downto 0);
        signal filter_value         : in  std_logic_vector(width - 1 downto 0);
        signal filter_input         : in  std_logic_vector(width - 1 downto 0);
        signal enable               : in  std_logic;
        signal valid                : out std_logic
    );
    end component;


    ----------------------------------------------------------------------------
    -- Range filter
    ----------------------------------------------------------------------------
    component range_filter is
    generic(
        constant width              :   natural;
        constant is_present         :   boolean        
    );
    port(
        signal filter_upp_th        : in    std_logic_vector(width - 1 downto 0);
        signal filter_low_th        : in    std_logic_vector(width - 1 downto 0);
        signal filter_input         : in    std_logic_vector(width - 1 downto 0);
        signal enable               : in    std_logic;
        signal valid                : out   std_logic
    );
    end component;


    ----------------------------------------------------------------------------
    -- Interrupt manager module
    ----------------------------------------------------------------------------
    component int_manager is
        generic(
            constant int_count          :     natural range 0 to 32 := 11
        );
        port(
            signal clk_sys                :in   std_logic;
            signal res_n                  :in   std_logic;
            signal error_valid            :in   std_logic;
            signal error_passive_changed  :in   std_logic;
            signal error_warning_limit    :in   std_logic;
            signal arbitration_lost       :in   std_logic;
            signal tx_finished            :in   std_logic;
            signal br_shifted             :in   std_logic;
            signal rx_message_disc        :in   std_logic;
            signal rec_message_valid      :in   std_logic;
            signal rx_full                :in   std_logic;
            signal rx_empty               :in   std_logic;
            signal txt_hw_cmd_int         :in   std_logic_vector(TXT_BUFFER_COUNT - 1
                                                                 downto 0);
            signal loger_finished         :in   std_logic;
            signal drv_bus                :in   std_logic_vector(1023 downto 0);
            signal int_out                :out  std_logic;

            signal int_vector             :out  std_logic_vector(
                                                    int_count - 1 downto 0);

            signal int_mask               :out  std_logic_vector(
                                                    int_count - 1 downto 0);

            signal int_ena                :out  std_logic_vector(
                                                    int_count - 1 downto 0)
        );
    end component;


    ----------------------------------------------------------------------------
    -- Single Interrupt module
    ----------------------------------------------------------------------------
    component int_module is
        generic(        
            constant reset_polarity        :    std_logic := '0';
            constant clear_priority        :    boolean := true
        );
        port(
            signal clk_sys                :in   std_logic; --System Clock
            signal res_n                  :in   std_logic; --Async Reset

            signal int_status_set         :in   std_logic;
            signal int_status_clear       :in   std_logic;

            signal int_mask_set           :in   std_logic;
            signal int_mask_clear         :in   std_logic;

            signal int_ena_set            :in   std_logic;
            signal int_ena_clear          :in   std_logic;

            signal int_status             :out  std_logic;
            signal int_mask               :out  std_logic;
            signal int_ena                :out  std_logic
        );  
    end component;


    ----------------------------------------------------------------------------
    -- CAN Core module
    ----------------------------------------------------------------------------
    component can_core is
        port(
            signal clk_sys               : in  std_logic;
            signal res_n                 : in  std_logic;
            signal drv_bus               : in  std_logic_vector(1023 downto 0);
            signal stat_bus              : out std_logic_vector(511 downto 0);
            signal tran_data_in          : in  std_logic_vector(31 downto 0);
            signal tran_dlc_in           : in  std_logic_vector(3 downto 0);
            signal tran_is_rtr_in        : in  std_logic;
            signal tran_ident_type_in    : in  std_logic;
            signal tran_frame_type_in    : in  std_logic;
            signal tran_brs_in           : in  std_logic;
            signal tran_frame_valid_in   : in  std_logic;
            signal txt_hw_cmd            : out txt_hw_cmd_type;
            signal txtb_changed          : in  std_logic;
            signal txt_buf_ptr           : out natural range 0 to 19;

            signal rec_ident_out         : out std_logic_vector(28 downto 0);
            signal rec_dlc_out           : out std_logic_vector(3 downto 0);
            signal rec_ident_type_out    : out std_logic;
            signal rec_frame_type_out    : out std_logic;
            signal rec_is_rtr_out        : out std_logic;
            signal rec_brs_out           : out std_logic;
            signal rec_esi_out           : out std_logic;
            signal rec_message_valid_out : out std_logic;
            signal store_metadata        : out std_logic;
            signal store_data            : out std_logic;
            signal store_data_word       : out std_logic_vector(31 downto 0);
            signal rec_abort             : out std_logic;

            signal arbitration_lost_out  : out std_logic;
            signal tx_finished           : out std_logic;
            signal br_shifted            : out std_logic;
            signal error_valid           : out std_logic;
            signal error_passive_changed : out std_logic;
            signal error_warning_limit   : out std_logic;
            signal sample_nbt_del_2      : in  std_logic;
            signal sample_dbt_del_2      : in  std_logic;
            signal sample_nbt_del_1      : in  std_logic;
            signal sample_dbt_del_1      : in  std_logic;
            signal sync_nbt              : in  std_logic;
            signal sync_dbt              : in  std_logic;
            signal sync_nbt_del_1        : in  std_logic;
            signal sync_dbt_del_1        : in  std_logic;
            signal sample_sec            : in  std_logic;
            signal sample_sec_del_1      : in  std_logic;
            signal sample_sec_del_2      : in  std_logic;
            signal sync_control          : out std_logic_vector(1 downto 0);
            signal data_rx               : in  std_logic;
            signal data_tx               : out std_logic;
            signal timestamp             : in  std_logic_vector(63 downto 0);
            signal sp_control            : out std_logic_vector(1 downto 0);
            signal ssp_reset             : out std_logic;
            signal trv_delay_calib       : out std_logic;
            signal bit_Error_sec_sam     : in  std_logic;
            signal hard_sync_edge        : in  std_logic;
            signal bus_off_start         : out std_logic;
            signal sof_pulse             : out std_logic
        );
    end component;


    ----------------------------------------------------------------------------
    -- Bus traffic counters
    ----------------------------------------------------------------------------
    component bus_traffic_counters is
    port(
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;
        signal clear_rx_ctr           :in   std_logic;
        signal clear_tx_ctr           :in   std_logic;
        signal inc_tx_ctr             :in   std_logic;
        signal inc_rx_ctr             :in   std_logic;
        signal tx_ctr                 :out  std_logic_vector(31 downto 0);
        signal rx_ctr                 :out  std_logic_vector(31 downto 0)
    );
    end component;

    ----------------------------------------------------------------------------
    -- Prescaler module
    ----------------------------------------------------------------------------
    component prescaler is
        port(
            signal clk_sys              : in  std_logic;
            signal res_n                : in  std_logic;
            signal sync_edge            : in  std_logic;
            signal OP_State             : in  oper_mode_type;
            signal drv_bus              : in  std_logic_vector(1023 downto 0);
            signal clk_tq_nbt           : out std_logic;
            signal clk_tq_dbt           : out std_logic;
            signal sample_nbt           : out std_logic;
            signal sample_dbt           : out std_logic;
            signal sample_nbt_del_1     : out std_logic;
            signal sample_dbt_del_1     : out std_logic;
            signal sample_nbt_del_2     : out std_logic;
            signal sample_dbt_del_2     : out std_logic;
            signal sync_nbt             : out std_logic;
            signal sync_dbt             : out std_logic;
            signal sync_nbt_del_1       : out std_logic;
            signal sync_dbt_del_1       : out std_logic;
            signal bt_FSM_out           : out bit_time_type;
            signal data_tx              : in  std_logic;
            signal hard_sync_edge_valid : out std_logic;
            signal sp_control           : in  std_logic_vector(1 downto 0);
            signal sync_control         : in  std_logic_vector(1 downto 0)
        );
    end component;


    ----------------------------------------------------------------------------
    -- Bus Sampling module
    ----------------------------------------------------------------------------
    component bus_sampling is
        generic (
            use_Sync : boolean
        );
        port(
            signal clk_sys              : in  std_logic;
            signal res_n                : in  std_logic;
            signal CAN_rx               : in  std_logic;
            signal CAN_tx               : out std_logic;
            signal drv_bus              : in  std_logic_vector(1023 downto 0);
            signal sample_nbt           : in  std_logic;
            signal sample_dbt           : in  std_logic;
            signal sync_edge            : out std_logic;
            signal data_tx              : in  std_logic;
            signal data_rx              : out std_logic;
            signal sp_control           : in  std_logic_vector(1 downto 0);
            signal ssp_reset            : in  std_logic;
            signal trv_delay_calib      : in  std_logic;
            signal bit_err_enable       : in  std_logic;
            signal sample_sec_out       : out std_logic;
            signal sample_sec_del_1_out : out std_logic;
            signal sample_sec_del_2_out : out std_logic;
            signal trv_delay_out        : out std_logic_vector(15 downto 0);
            signal bit_Error            : out std_logic
        );
    end component;


    ----------------------------------------------------------------------------
    -- Data edge detector
    ----------------------------------------------------------------------------
    component data_edge_detector is
        generic(
            constant reset_polarity         :     std_logic;
            constant tx_edge_pipeline       :     boolean := true;
            constant rx_edge_pipeline       :     boolean := true
        );
        port(
            signal clk_sys                  :in   std_logic;
            signal res_n                    :in   std_logic;
            signal tx_data                  :in   std_logic;
            signal rx_data                  :in   std_logic;
            signal prev_rx_sample           :in   std_logic;
            signal tx_edge                  :out  std_logic;
            signal rx_edge                  :out  std_logic
            );
    end component;

    ----------------------------------------------------------------------------
    -- Transceiver Delay measurement
    ----------------------------------------------------------------------------
    component trv_delay_measurement is
        generic(
            constant reset_polarity         :     std_logic;
            constant trv_ctr_width          :     natural := 7;
            constant use_ssp_saturation     :     boolean := true;
            constant ssp_saturation_lvl     :     natural
        );
        port(
            signal clk_sys                  :in   std_logic;
            signal res_n                    :in   std_logic;
            signal meas_start               :in   std_logic;
            signal meas_stop                :in   std_logic;
            signal meas_enable              :in   std_logic;
            signal ssp_offset               :in   std_logic_vector(trv_ctr_width - 1 downto 0);
            signal ssp_delay_select         :in   std_logic_vector(1 downto 0);
            signal trv_meas_progress        :out  std_logic;
            signal trv_delay_shadowed       :out  std_logic_vector(trv_ctr_width - 1 downto 0);
            signal ssp_delay_shadowed       :out  std_logic_vector(trv_ctr_width downto 0)
        );
    end component;

    ----------------------------------------------------------------------------
    -- Event Logger module
    ----------------------------------------------------------------------------
    component event_logger is
        generic(
            constant memory_size        :   natural := 16
        );
        port(
            signal clk_sys              : in  std_logic;
            signal res_n                : in  std_logic;
            signal drv_bus              : in  std_logic_vector(1023 downto 0);
            signal stat_bus             : in  std_logic_vector(511 downto 0);
            signal sync_edge            : in  std_logic;
            signal data_overrun         : in  std_logic;
            signal timestamp            : in  std_logic_vector(63 downto 0);
            signal bt_FSM               : in  bit_time_type;
            signal loger_finished       : out std_logic;
            signal loger_act_data       : out std_logic_vector(63 downto 0);
            signal log_write_pointer    : out std_logic_vector(7 downto 0);
            signal log_read_pointer     : out std_logic_vector(7 downto 0);
            signal log_size             : out std_logic_vector(7 downto 0);
            signal log_state_out        : out logger_state_type
        );
    end component;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    ---- CORE Top level components
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    -- CRC wrapper
    ----------------------------------------------------------------------------
    component crc_wrapper is
        generic(
            constant crc15_pol :     std_logic_vector(15 downto 0) := x"C599";
            constant crc17_pol :     std_logic_vector(19 downto 0) := x"3685B";
            constant crc21_pol :     std_logic_vector(23 downto 0) := x"302899"  
        );
        port(
            signal res_n            :in   std_logic;
            signal clk_sys          :in   std_logic;
            signal data_tx_nbs      :in   std_logic;
            signal data_tx_wbs      :in   std_logic;
            signal data_rx_wbs      :in   std_logic;
            signal data_rx_nbs      :in   std_logic;
            signal trig_tx_nbs      :in   std_logic;
            signal trig_tx_wbs      :in   std_logic;
            signal trig_rx_wbs      :in   std_logic;
            signal trig_rx_nbs      :in   std_logic;
            signal enable           :in   std_logic;
            signal drv_bus          :in   std_logic_vector(1023 downto 0);
            signal use_rx_crc       :in   std_logic;
            signal use_wbs_crc      :in   std_logic;
            signal crc15            :out  std_logic_vector(14 downto 0);
            signal crc17            :out  std_logic_vector(16 downto 0);
            signal crc21            :out  std_logic_vector(20 downto 0)
        );
    end component;


    ----------------------------------------------------------------------------
    -- CAN CRC module
    ----------------------------------------------------------------------------
    component can_crc is
        generic(
            constant crc15_pol : std_logic_vector(15 downto 0) := x"C599";
            constant crc17_pol : std_logic_vector(19 downto 0) := x"3685B";
            constant crc21_pol : std_logic_vector(23 downto 0) := x"302899"
        );
        port(
            signal data_in      : in  std_logic;
            signal clk_sys      : in  std_logic;
            signal trig         : in  std_logic;
            signal res_n        : in  std_logic;
            signal enable       : in  std_logic;
            signal drv_bus      : in  std_logic_vector(1023 downto 0);
            signal crc15        : out std_logic_vector(14 downto 0);
            signal crc17        : out std_logic_vector(16 downto 0);
            signal crc21        : out std_logic_vector(20 downto 0)
        );
    end component;


    ----------------------------------------------------------------------------
    -- Generic CRC calculation module
    ----------------------------------------------------------------------------
    component crc_calc is
    generic(
        constant crc_width      :     natural;
        constant reset_polarity :       std_logic := '0';
        constant polynomial     :     std_logic_vector
    );
    port(
        signal res_n            :in   std_logic;
        signal clk_sys          :in   std_logic;
        signal data_in          :in   std_logic;
        signal trig             :in   std_logic;
        signal enable           :in   std_logic; 
        signal init_vect        :in   std_logic_vector(crc_width - 1 downto 0);
        signal crc              :out  std_logic_vector(crc_width - 1 downto 0)
    );    
    end component;


    ----------------------------------------------------------------------------
    -- Bit Stuffing
    ----------------------------------------------------------------------------
    component bit_stuffing is
        port(
            signal clk_sys     : in  std_logic;
            signal res_n       : in  std_logic;
            signal tran_trig_1 : in  std_logic;
            signal enable      : in  std_logic;
            signal data_in     : in  std_logic;
            signal fixed_stuff : in  std_logic;
            signal data_halt   : out std_logic;
            signal length      : in  std_logic_vector(2 downto 0);
            signal bst_ctr     : out natural range 0 to 7;
            signal data_out    : out std_logic
        );
    end component;


    ----------------------------------------------------------------------------
    -- Bit Destuffing
    ----------------------------------------------------------------------------
    component bit_destuffing is
        port(
            signal clk_sys            : in  std_logic;
            signal res_n              : in  std_logic;
            signal data_in            : in  std_logic;
            signal trig_spl_1         : in  std_logic;
            signal stuff_Error        : out std_logic;
            signal data_out           : out std_logic;
            signal destuffed          : out std_logic;
            signal enable             : in  std_logic;
            signal stuff_Error_enable : in  std_logic;
            signal fixed_stuff        : in  std_logic;
            signal length             : in  std_logic_vector(2 downto 0);
            signal dst_ctr            : out natural range 0 to 7
        );
    end component;


    ----------------------------------------------------------------------------
    -- Operation control module
    ----------------------------------------------------------------------------
    component operation_control is
        port(
            signal clk_sys            : in  std_logic;
            signal res_n              : in  std_logic;
            signal drv_bus            : in  std_logic_vector(1023 downto 0);
            signal arbitration_lost   : in  std_logic;
            signal PC_State           : in  protocol_type;
            signal tran_data_valid_in : in  std_logic;
            signal set_transciever    : in  std_logic;
            signal set_reciever       : in  std_logic;
            signal is_idle            : in  std_logic;
            signal unknown_OP_state   : out std_logic;
            signal tran_trig          : in  std_logic;
            signal rec_trig           : in  std_logic;
            signal data_rx            : in  std_logic;
            signal OP_State           : out oper_mode_type
        );
    end component;


    ----------------------------------------------------------------------------
    -- Protocol Control module
    ----------------------------------------------------------------------------
    component protocol_control is
        port(
            signal clk_sys               : in  std_logic;
            signal res_n                 : in  std_logic;
            signal drv_bus               : in  std_logic_vector(1023 downto 0);
            signal int_loop_back_ena     : out std_logic;
            signal PC_State_out          : out protocol_type;
            signal alc                   : out std_logic_vector(7 downto 0);
            signal tran_data             : in  std_logic_vector(31 downto 0);
            signal tran_dlc              : in  std_logic_vector(3 downto 0);
            signal tran_is_rtr           : in  std_logic;
            signal tran_ident_type       : in  std_logic;
            signal tran_frame_type       : in  std_logic;
            signal tran_brs              : in  std_logic;
            signal txt_buf_ptr           : out natural range 0 to 19;
            signal tran_frame_valid_in   : in  std_logic;
            signal txt_hw_cmd            : out txt_hw_cmd_type;
            signal txtb_changed          : in  std_logic;
            signal br_shifted            : out std_logic;
            signal rec_ident             : out std_logic_vector(28 downto 0);
            signal rec_dlc               : out std_logic_vector(3 downto 0);
            signal rec_is_rtr            : out std_logic;
            signal rec_ident_type        : out std_logic;
            signal rec_frame_type        : out std_logic;
            signal rec_brs               : out std_logic;
            signal rec_crc               : out std_logic_vector(20 downto 0);
            signal rec_esi               : out std_logic;
            signal store_metadata        : out std_logic;
            signal rec_abort             : out std_logic;
            signal store_data            : out std_logic;
            signal store_data_word       : out std_logic_vector(31 downto 0);
            signal OP_state              : in  oper_mode_type;
            signal arbitration_lost      : out std_logic;
            signal is_idle               : out std_logic;
            signal set_transciever       : out std_logic;
            signal set_reciever          : out std_logic;
            signal ack_recieved_out      : out std_logic;
            signal error_state           : in  error_state_type;
            signal form_Error            : out std_logic;
            signal CRC_Error             : out std_logic;
            signal ack_Error             : out std_logic;
            signal bit_Error_valid       : in  std_logic;
            signal stuff_Error_valid     : in  std_logic;
            signal inc_one               : out std_logic;
            signal inc_eight             : out std_logic;
            signal dec_one               : out std_logic;
            signal tran_valid            : out std_logic;
            signal rec_valid             : out std_logic;
            signal tran_trig             : in  std_logic;
            signal rec_trig              : in  std_logic;
            signal data_tx               : out std_logic;
            signal stuff_enable          : out std_logic;
            signal fixed_stuff           : out std_logic;
            signal stuff_length          : out std_logic_vector(2 downto 0);
            signal data_rx               : in  std_logic;
            signal destuff_enable        : out std_logic;
            signal stuff_error_enable    : out std_logic;
            signal fixed_destuff         : out std_logic;
            signal destuff_length        : out std_logic_vector(2 downto 0);
            signal dst_ctr               : in  natural range 0 to 7;
            signal crc_enable            : out std_logic;
            signal unknown_OP_state      : in  std_logic;
            signal crc15                 : in  std_logic_vector(14 downto 0);
            signal crc17                 : in  std_logic_vector(16 downto 0);
            signal crc21                 : in  std_logic_vector(20 downto 0);
            signal sync_control          : out std_logic_vector(1 downto 0);
            signal sp_control            : out std_logic_vector(1 downto 0);
            signal ssp_reset             : out std_logic;
            signal trv_delay_calib       : out std_logic;
            signal hard_sync_edge        : in  std_logic;
            signal sof_pulse             : out std_logic
        );
    end component;


    ----------------------------------------------------------------------------
    -- Fault confinement
    ----------------------------------------------------------------------------
    component fault_confinement is
        port(
            signal clk_sys               : in  std_logic;
            signal res_n                 : in  std_logic;
            signal drv_bus               : in  std_logic_vector(1023 downto 0);
            signal stuff_Error           : in  std_logic;
            signal error_valid           : out std_logic;
            signal error_passive_changed : out std_logic;
            signal error_warning_limit   : out std_logic;
            signal OP_State              : in  oper_mode_type;
            signal data_rx               : in  std_logic;
            signal data_tx               : in  std_logic;
            signal rec_trig              : in  std_logic;
            signal tran_trig_1           : in  std_logic;
            signal PC_State              : in  protocol_type;
            signal sp_control            : in  std_logic_vector(1 downto 0);
            signal form_Error            : in  std_logic;
            signal CRC_Error             : in  std_logic;
            signal ack_Error             : in  std_logic;
            signal bit_Error_valid       : out std_logic;
            signal stuff_Error_valid     : out std_logic;
            signal inc_one               : in  std_logic;
            signal inc_eight             : in  std_logic;
            signal dec_one               : in  std_logic;
            signal enable                : in  std_logic;
            signal bit_Error_sec_sam     : in  std_logic;
            signal err_capt              : out std_logic_vector(7 downto 0);
            signal bit_Error_out         : out std_logic;
            signal bus_off_start         : out std_logic;
            signal tx_counter_out        : out std_logic_vector(8 downto 0);
            signal rx_counter_out        : out std_logic_vector(8 downto 0);
            signal err_counter_norm_out  : out std_logic_vector(15 downto 0);
            signal err_counter_fd_out    : out std_logic_vector(15 downto 0);
            signal error_state_out       : out error_state_type
        );
    end component;


    ----------------------------------------------------------------------------
    -- APB Interface
    ----------------------------------------------------------------------------
    component apb_ifc is
        generic (
            -- ID (bits  19-16 of reg_addr_o)
            ID : natural := 1
        );
        port (
            aclk             : in  std_logic;
            arstn            : in  std_logic;

            reg_data_in_o    : out std_logic_vector(31 downto 0);
            reg_data_out_i   : in  std_logic_vector(31 downto 0);
            reg_addr_o       : out std_logic_vector(23 downto 0);
            reg_be_o         : out std_logic_vector(3 downto 0);
            reg_rden_o       : out std_logic;
            reg_wren_o       : out std_logic;

            s_apb_paddr      : in  std_logic_vector(31 downto 0);
            s_apb_penable    : in  std_logic;
            s_apb_pprot      : in  std_logic_vector(2 downto 0);
            s_apb_prdata     : out std_logic_vector(31 downto 0);
            s_apb_pready     : out std_logic;
            s_apb_psel       : in  std_logic;
            s_apb_pslverr    : out std_logic;
            s_apb_pstrb      : in  std_logic_vector(3 downto 0);
            s_apb_pwdata     : in  std_logic_vector(31 downto 0);
            s_apb_pwrite     : in  std_logic
        );
    end component;

end package;
