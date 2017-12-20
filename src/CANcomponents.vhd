--------------------------------------------------------------------------------
-- 
-- CAN with Flexible Data-Rate IP Core 
-- 
-- Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisor: Jiri Novak <jnovak@fel.cvut.cz>
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
--------------------------------------------------------------------------------

library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use ieee.std_logic_unsigned.all;
use WORK.CANconstants.all;

package CANcomponents is

  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  ---- CAN FD Core top level entity
  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  component CAN_top_level is
    generic(
      constant use_logger     : boolean               := true;
      constant rx_buffer_size : natural               := 128;
      constant useFDSize      : boolean               := true;
      constant use_sync       : boolean               := true;
      constant ID             : natural range 0 to 15 := 1;
      constant sup_filtA      : boolean               := true;
      constant sup_filtB      : boolean               := true;
      constant sup_filtC      : boolean               := true;
      constant sup_range      : boolean               := true;
      constant tx_time_sup    : boolean               := true;
      constant sup_be         : boolean               := false;
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
      signal timestamp       : in  std_logic_vector(63 downto 0)
      );
  end component;


  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  ---- CAN Top level components
  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Registers
  ------------------------------------------------------------------------------
  component canfd_registers is
    generic(
      constant compType   : std_logic_vector(3 downto 0) := CAN_COMPONENT_TYPE;
      constant use_logger : boolean                      := true;
      constant sup_filtA  : boolean                      := true;
      constant sup_filtB  : boolean                      := true;
      constant sup_filtC  : boolean                      := true;
      constant sup_range  : boolean                      := true;
      constant sup_be     : boolean                      := false;
      constant ID         : natural
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
      signal drv_bus              : out std_logic_vector(1023 downto 0);
      signal stat_bus             : in  std_logic_vector(511 downto 0);
      signal rx_read_buff         : in  std_logic_vector(31 downto 0);
      signal rx_buf_size          : in  std_logic_vector(7 downto 0);
      signal rx_full              : in  std_logic;
      signal rx_empty             : in  std_logic;
      signal rx_message_count     : in  std_logic_vector(7 downto 0);
      signal rx_mem_free          : in  std_logic_vector(7 downto 0);
      signal rx_read_pointer_pos  : in  std_logic_vector(7 downto 0);
      signal rx_write_pointer_pos : in  std_logic_vector(7 downto 0);
      signal rx_message_disc      : in  std_logic;
      signal rx_data_overrun      : in  std_logic;
      signal tran_data            : out std_logic_vector(31 downto 0);
      signal tran_addr            : out std_logic_vector(4 downto 0);
      signal txt1_empty           : in  std_logic;
      signal txt1_disc            : in  std_logic;
      signal txt2_empty           : in  std_logic;
      signal txt2_disc            : in  std_logic;
      signal trv_delay_out        : in  std_logic_vector(15 downto 0);
      signal int_vector           : in  std_logic_vector(10 downto 0);
      signal loger_act_data       : in  std_logic_vector(63 downto 0);
      signal log_write_pointer    : in  std_logic_vector(7 downto 0);
      signal log_read_pointer     : in  std_logic_vector(7 downto 0);
      signal log_size             : in  std_logic_vector(7 downto 0);
      signal log_state_out        : in  logger_state_type
      );
  end component;

  ------------------------------------------------------------------------------
  -- RX Buffer module
  ------------------------------------------------------------------------------
  component rxBuffer is
    generic(
      buff_size : natural range 4 to 512
      );
    port(
      signal clk_sys              : in  std_logic;
      signal res_n                : in  std_logic;
      signal rec_ident_in         : in  std_logic_vector(28 downto 0);
      signal rec_dlc_in           : in  std_logic_vector(3 downto 0);
      signal rec_ident_type_in    : in  std_logic;
      signal rec_frame_type_in    : in  std_logic;
      signal rec_is_rtr           : in  std_logic;
      signal rec_message_valid    : in  std_logic;
      signal rec_brs              : in  std_logic;
      signal rec_esi              : in  std_logic;
      signal rec_message_ack      : out std_logic;
      signal rec_dram_word        : in  std_logic_vector(31 downto 0);
      signal rec_dram_addr        : out natural range 0 to 15;
      signal rx_buf_size          : out std_logic_vector(7 downto 0);
      signal rx_full              : out std_logic;
      signal rx_empty             : out std_logic;
      signal rx_message_count     : out std_logic_vector(7 downto 0);
      signal rx_mem_free          : out std_logic_vector(7 downto 0);
      signal rx_read_pointer_pos  : out std_logic_vector(7 downto 0);
      signal rx_write_pointer_pos : out std_logic_vector(7 downto 0);
      signal rx_message_disc      : out std_logic;
      signal rx_data_overrun      : out std_logic;
      signal timestamp            : in  std_logic_vector(63 downto 0);
      signal rx_read_buff         : out std_logic_vector(31 downto 0);
      signal drv_bus              : in  std_logic_vector(1023 downto 0)
      );
  end component;

  ------------------------------------------------------------------------------
  --TX Buffer  module
  ------------------------------------------------------------------------------
  component txBuffer is
    generic (
      buff_size : natural
      );
    port(
      signal clk_sys              : in  std_logic;
      signal res_n                : in  std_logic;
      signal drv_bus              : in  std_logic_vector(1023 downto 0);
      signal tran_data_in         : in  std_logic_vector(639 downto 0);
      signal tx_buffer_out        : out std_logic_vector(639 downto 0);
      signal tx_buffer_valid      : out std_logic;
      signal tx_buffer_ack        : in  std_logic;
      signal tx_buff_size         : out std_logic_vector(7 downto 0);
      signal tx_full              : out std_logic;
      signal tx_message_count     : out std_logic_vector(7 downto 0);
      signal tx_empty             : out std_logic;
      signal tx_mem_free          : out std_logic_vector (7 downto 0);
      signal tx_read_pointer_pos  : out std_logic_vector(7 downto 0);
      signal tx_write_pointer_pos : out std_logic_vector(7 downto 0);
      signal tx_message_disc      : out std_logic
      );
  end component;

  ------------------------------------------------------------------------------
  -- TXT Buffer module
  ------------------------------------------------------------------------------
  component txtBuffer is
    generic(
      constant ID        : natural := 1;
      constant useFDsize : boolean := false
      );
    port(
      signal clk_sys            : in  std_logic;
      signal res_n              : in  std_logic;
      signal drv_bus            : in  std_logic_vector(1023 downto 0);
      signal tran_data          : in  std_logic_vector(31 downto 0);
      signal tran_addr          : in  std_logic_vector(4 downto 0);
      signal txt_empty          : out std_logic;
      signal txt_data_ack       : in  std_logic;
      signal txt_data_word      : out std_logic_vector(31 downto 0);
      signal txt_data_addr      : in  natural range 0 to 15;
      signal txt_frame_info_out : out std_logic_vector(127 downto 0)
      );
  end component;

  ------------------------------------------------------------------------------
  -- TXT Arbitrator module
  ------------------------------------------------------------------------------
  component txArbitrator is
    generic(
      tx_time_sup : boolean := true
    );
    port(
      signal clk_sys              : in  std_logic;
      signal res_n                : in  std_logic;
      signal txt1buf_info_in      : in  std_logic_vector(639 downto 512);
      signal txt1buf_data_in      : in  std_logic_vector(31 downto 0);
      signal txt1_buffer_empty    : in  std_logic;
      signal txt1_buffer_ack      : out std_logic;
      signal txt2buf_info_in      : in  std_logic_vector(639 downto 512);
      signal txt2buf_data_in      : in  std_logic_vector(31 downto 0);
      signal txt2_buffer_empty    : in  std_logic;
      signal txt2_buffer_ack      : out std_logic;
      signal tran_data_word_out   : out std_logic_vector(31 downto 0);
      signal tran_ident_out       : out std_logic_vector(28 downto 0);
      signal tran_dlc_out         : out std_logic_vector(3 downto 0);
      signal tran_is_rtr          : out std_logic;
      signal tran_ident_type_out  : out std_logic;
      signal tran_frame_type_out  : out std_logic;
      signal tran_brs_out         : out std_logic;
      signal tran_frame_valid_out : out std_logic;
      signal tran_data_ack        : in  std_logic;
      signal tran_valid           : in  std_logic;
      signal drv_bus              : in  std_logic_vector(1023 downto 0);
      signal timestamp            : in  std_logic_vector(63 downto 0)
      );
  end component;

  ------------------------------------------------------------------------------
  -- Message filter module
  ------------------------------------------------------------------------------
  component messageFilter is
    generic
      (
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

  ------------------------------------------------------------------------------
  -- Interrupt manager module
  ------------------------------------------------------------------------------
  component intManager is
    generic(
      constant int_length : natural range 0 to 10 := 5
      );
    port(
      signal clk_sys               : in  std_logic;
      signal res_n                 : in  std_logic;
      signal error_valid           : in  std_logic;
      signal error_passive_changed : in  std_logic;
      signal error_warning_limit   : in  std_logic;
      signal arbitration_lost      : in  std_logic;
      signal wake_up_valid         : in  std_logic;
      signal tx_finished           : in  std_logic;
      signal br_shifted            : in  std_logic;
      signal rx_message_disc       : in  std_logic;
      signal rec_message_valid     : in  std_logic;
      signal rx_full               : in  std_logic;
      signal loger_finished        : in  std_logic;
      signal drv_bus               : in  std_logic_vector(1023 downto 0);
      signal int_out               : out std_logic;
      signal int_vector            : out std_logic_vector(10 downto 0)
      );
  end component;

  ------------------------------------------------------------------------------
  --CAN Core module --
  ------------------------------------------------------------------------------
  component core_top is
    port(
      signal clk_sys               : in  std_logic;
      signal res_n                 : in  std_logic;
      signal drv_bus               : in  std_logic_vector(1023 downto 0);
      signal stat_bus              : out std_logic_vector(511 downto 0);
      signal tran_data_in          : in  std_logic_vector(31 downto 0);
      signal tran_ident_in         : in  std_logic_vector(28 downto 0);
      signal tran_dlc_in           : in  std_logic_vector(3 downto 0);
      signal tran_is_rtr_in        : in  std_logic;
      signal tran_ident_type_in    : in  std_logic;
      signal tran_frame_type_in    : in  std_logic;
      signal tran_brs_in           : in  std_logic;
      signal tran_frame_valid_in   : in  std_logic;
      signal tran_data_ack_out     : out std_logic;
      signal txt_buf_ptr           : out natural range 0 to 15;
      signal rec_ident_out         : out std_logic_vector(28 downto 0);
      signal rec_dlc_out           : out std_logic_vector(3 downto 0);
      signal rec_ident_type_out    : out std_logic;
      signal rec_frame_type_out    : out std_logic;
      signal rec_is_rtr_out        : out std_logic;
      signal rec_brs_out           : out std_logic;
      signal rec_esi_out           : out std_logic;
      signal rec_message_valid_out : out std_logic;
      signal rec_message_ack_out   : in  std_logic;
      signal rec_dram_word_out     : out std_logic_vector(31 downto 0);
      signal rec_dram_addr_out     : in  natural range 0 to 15;
      signal arbitration_lost_out  : out std_logic;
      signal wake_up_valid         : out std_logic;
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
      signal hard_sync_edge        : in  std_logic
      );
  end component;

  ------------------------------------------------------------------------------
  -- Prescaler module
  ------------------------------------------------------------------------------
  component prescaler_v3 is
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

  ------------------------------------------------------------------------------
  -- Bus synchroniser module
  ------------------------------------------------------------------------------
  component busSync is
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

  ------------------------------------------------------------------------------
  -- CAN Logger module
  ------------------------------------------------------------------------------
  component CAN_logger is
    generic(
      constant memory_size : natural := 16
      );
    port(
      signal clk_sys           : in  std_logic;
      signal res_n             : in  std_logic;
      signal drv_bus           : in  std_logic_vector(1023 downto 0);
      signal stat_bus          : in  std_logic_vector(511 downto 0);
      signal sync_edge         : in  std_logic;
      signal data_overrun      : in  std_logic;
      signal timestamp         : in  std_logic_vector(63 downto 0);
      signal bt_FSM            : in  bit_time_type;
      signal loger_finished    : out std_logic;
      signal loger_act_data    : out std_logic_vector(63 downto 0);
      signal log_write_pointer : out std_logic_vector(7 downto 0);
      signal log_read_pointer  : out std_logic_vector(7 downto 0);
      signal log_size          : out std_logic_vector(7 downto 0);
      signal log_state_out     : out logger_state_type
      );
  end component;


  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  ---- CORE Top level components
  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- CRC module
  ------------------------------------------------------------------------------
  component canCRC is
    generic(
      constant crc15_pol : std_logic_vector(15 downto 0) := x"C599";
      constant crc17_pol : std_logic_vector(19 downto 0) := x"3685B";
      constant crc21_pol : std_logic_vector(23 downto 0) := x"302899"
      );
    port(
      signal data_in : in  std_logic;
      signal clk_sys : in  std_logic;
      signal trig    : in  std_logic;
      signal res_n   : in  std_logic;
      signal enable  : in  std_logic;
      signal drv_bus : in  std_logic_vector(1023 downto 0);
      signal crc15   : out std_logic_vector(14 downto 0);
      signal crc17   : out std_logic_vector(16 downto 0);
      signal crc21   : out std_logic_vector(20 downto 0)
      );
  end component;

  ------------------------------------------------------------------------------
  -- Transcieve buffer
  ------------------------------------------------------------------------------
  component tranBuffer is
    port(
      signal clk_sys            : in  std_logic;
      signal res_n              : in  std_logic;
      signal tran_ident_in      : in  std_logic_vector(28 downto 0);
      signal tran_dlc_in        : in  std_logic_vector(3 downto 0);
      signal tran_is_rtr_in     : in  std_logic;
      signal tran_ident_type_in : in  std_logic;
      signal tran_frame_type_in : in  std_logic;
      signal tran_brs_in        : in  std_logic;
      signal frame_store        : in  std_logic;
      signal tran_ident_base    : out std_logic_vector(10 downto 0);
      signal tran_ident_ext     : out std_logic_vector(17 downto 0);
      signal tran_dlc           : out std_logic_vector(3 downto 0);
      signal tran_is_rtr        : out std_logic;
      signal tran_ident_type    : out std_logic;
      signal tran_frame_type    : out std_logic;
      signal tran_brs           : out std_logic
      );
  end component;

  ------------------------------------------------------------------------------
  -- Bit Stuffing
  ------------------------------------------------------------------------------
  component bitStuffing_v2 is
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

  ------------------------------------------------------------------------------
  -- Bit Destuffing
  ------------------------------------------------------------------------------
  component bitDestuffing is
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

  ------------------------------------------------------------------------------
  -- Operation control FSM
  ------------------------------------------------------------------------------
  component operationControl is
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
      signal tran_trig          : in  std_logic;
      signal rec_trig           : in  std_logic;
      signal data_rx            :     std_logic;
      signal OP_State           : out oper_mode_type
      );
  end component;

  ------------------------------------------------------------------------------
  -- Protocol Control FSM
  ------------------------------------------------------------------------------
  component protocolControl is
    port(
      signal clk_sys               : in  std_logic;
      signal res_n                 : in  std_logic;
      signal drv_bus               : in  std_logic_vector(1023 downto 0);
      signal int_loop_back_ena     : out std_logic;
      signal PC_State_out          : out protocol_type;
      signal alc                   : out std_logic_vector(4 downto 0);
      signal tran_data             : in  std_logic_vector(31 downto 0);
      signal tran_ident_base       : in  std_logic_vector(10 downto 0);
      signal tran_ident_ext        : in  std_logic_vector(17 downto 0);
      signal tran_dlc              : in  std_logic_vector(3 downto 0);
      signal tran_is_rtr           : in  std_logic;
      signal tran_ident_type       : in  std_logic;
      signal tran_frame_type       : in  std_logic;
      signal tran_brs              : in  std_logic;
      signal txt_buf_ptr           : out natural range 0 to 15;
      signal frame_store           : out std_logic;
      signal tran_frame_valid_in   : in  std_logic;
      signal tran_data_ack         : out std_logic;
      signal br_shifted            : out std_logic;
      signal rec_ident             : out std_logic_vector(28 downto 0);
      signal rec_dlc               : out std_logic_vector(3 downto 0);
      signal rec_is_rtr            : out std_logic;
      signal rec_ident_type        : out std_logic;
      signal rec_frame_type        : out std_logic;
      signal rec_brs               : out std_logic;
      signal rec_crc               : out std_logic_vector(20 downto 0);
      signal rec_esi               : out std_logic;
      signal rec_dram_word         : out std_logic_vector(31 downto 0);
      signal rec_dram_addr         : in  natural range 0 to 15;
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
      signal unknown_state_Error   : out std_logic;
      signal bit_stuff_Error_valid : in  std_logic;
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
      signal crc15                 : in  std_logic_vector(14 downto 0);
      signal crc17                 : in  std_logic_vector(16 downto 0);
      signal crc21                 : in  std_logic_vector(20 downto 0);
      signal sync_control          : out std_logic_vector(1 downto 0);
      signal sp_control            : out std_logic_vector(1 downto 0);
      signal ssp_reset             : out std_logic;
      signal trv_delay_calib       : out std_logic;
      signal bit_err_enable        : out std_logic;
      signal hard_sync_edge        : in  std_logic
      );
  end component;

  ------------------------------------------------------------------------------
  -- Fault confinement
  ------------------------------------------------------------------------------
  component faultConf is
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
      signal unknown_state_Error   : in  std_logic;
      signal bit_stuff_Error_valid : out std_logic;
      signal inc_one               : in  std_logic;
      signal inc_eight             : in  std_logic;
      signal dec_one               : in  std_logic;
      signal enable                : in  std_logic;
      signal bit_Error_sec_sam     : in  std_logic;
      signal bit_Error_out         : out std_logic;
      signal tx_counter_out        : out std_logic_vector(8 downto 0);
      signal rx_counter_out        : out std_logic_vector(8 downto 0);
      signal err_counter_norm_out  : out std_logic_vector(15 downto 0);
      signal err_counter_fd_out    : out std_logic_vector(15 downto 0);
      signal error_state_out       : out error_state_type
      );
  end component;

  ------------------------------------------------------------------------------
  -- Asynchronous resset synchroniser 
  ------------------------------------------------------------------------------
  component rst_sync is
    port (
      signal clk    : in  std_logic;
      signal arst_n : in  std_logic;
      signal rst_n  : out std_logic
      );
  end component;



end package;
