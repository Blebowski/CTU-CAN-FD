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
--  Protocol control module. Implements CAN FD protocol. Consists of following
--  modules:
--      TOOD
--------------------------------------------------------------------------------
-- Revision History:
--
--    July 2015   Created file
--    24.3.2019   Stashed all previous notes due to re-implementation.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity protocol_control is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :     std_logic := '0';
        
        -- Control counter width
        G_CTRL_CTR_WIDTH        :     natural := 9;
        
        -- Retransmitt limit counter width
        G_RETR_LIM_CTR_WIDTH    :     natural := 4;
        
        -- Insert pipeline on "error_valid" 
        G_ERR_VALID_PIPELINE    :     boolean := true
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;
        
        -- Asynchronous reset
        res_n                   :in   std_logic;
        
        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- Driving bus
        drv_bus                 :in   std_logic_vector(1023 downto 0);
        
        -- Arbitration lost capture
        alc                     :out  std_logic_vector(7 downto 0);
        
        -- Error code capture
        erc_capture             :out  std_logic_vector(7 downto 0);
        
        -- Arbitration field is being transmitted
        is_arbitration          :out  std_logic;
        
        -- Control field is being transmitted
        is_control              :out  std_logic;

        -- Data field is being transmitted
        is_data                 :out  std_logic;

        -- Stuff Count field is being transmitted
        is_stuff_count          :out  std_logic;

        -- CRC field is being transmitted
        is_crc                  :out  std_logic;
        
        -- CRC Delimiter is being transmitted
        is_crc_delim            :out  std_logic;
        
        -- ACK field is being transmitted
        is_ack_field            :out  std_logic;
        
        -- ACK Delimiter is being transmitted
        is_ack_delim            :out  std_logic;
        
        -- End of Frame field is being transmitted
        is_eof                  :out  std_logic;
        
        -- Intermission is being transmitted
        is_intermission         :out  std_logic;
        
        -- Suspend transmission is being transmitted
        is_suspend              :out  std_logic;

        -- Error frame is being transmitted
        is_error                :out  std_logic;
        
        -- Overload frame is being transmitted
        is_overload             :out  std_logic;
        
        -----------------------------------------------------------------------
        -- TXT Buffers interface
        -----------------------------------------------------------------------
        -- TX Data word
        tran_word               :in   std_logic_vector(31 downto 0);
        
        -- TX Data length code
        tran_dlc                :in   std_logic_vector(3 downto 0);
        
        -- TX Remote transmission request flag
        tran_is_rtr             :in   std_logic;
        
        -- TX Identifier type (0-Basic, 1-Extended)
        tran_ident_type         :in   std_logic;
        
        -- TX Frame type (0-CAN 2.0, 1-CAN FD)
        tran_frame_type         :in   std_logic;
        
        -- TX Bit rate shift
        tran_brs                :in   std_logic; 
        
        -- Frame in TXT Buffer is valid any can be transmitted.
        tran_frame_valid        :in   std_logic;
        
        -- HW Commands for TX Arbitrator and TXT Buffers
        txtb_hw_cmd             :out  t_txtb_hw_cmd;
        
        -- Pointer to TXT buffer memory
        txtb_ptr                :out  natural range 0 to 19;
        
        -- Selected TXT Buffer index changed
        txtb_changed            :in   std_logic;
        
        -----------------------------------------------------------------------
        -- RX Buffer interface
        -----------------------------------------------------------------------
        -- RX CAN Identifier
        rec_ident               :out  std_logic_vector(28 downto 0);
        
        -- RX Data length code
        rec_dlc                 :out  std_logic_vector(3 downto 0);
        
        -- RX Remote transmission request flag
        rec_is_rtr              :out  std_logic;
        
        -- RX Recieved identifier type (0-BASE Format, 1-Extended Format);
        rec_ident_type          :out  std_logic;
        
        -- RX frame type (0-CAN 2.0, 1- CAN FD)
        rec_frame_type          :out  std_logic;
        
        -- RX Bit rate shift Flag
        rec_brs                 :out  std_logic;
        
        -- RX Error state indicator 
        rec_esi                 :out  std_logic;
        
        -- Store Metadata in RX Buffer
        store_metadata          :out  std_logic;
    
        -- Abort storing of frame in RX Buffer. Revert to last frame.
        rec_abort               :out  std_logic;
    
        -- Store data word to RX Buffer. 
        store_data              :out  std_logic;
        
        -- Data words to be stored to RX Buffer.
        store_data_word         :out  std_logic_vector(31 downto 0);
    
        -- Pulse in Start of Frame
        sof_pulse               :out  std_logic;
    
        -----------------------------------------------------------------------
        -- Operation control FSM Interface
        -----------------------------------------------------------------------
        -- Unit is transmitter
        is_transmitter          :in   std_logic;
        
        -- Unit is receiver
        is_receiver             :in   std_logic;
        
        -- Unit is idle
        is_idle                 :in  std_logic;
        
        -- Loss of arbitration -> Turn receiver!
        arbitration_lost        :out  std_logic;
        
        -- Set unit to be transmitter (in SOF)
        set_transmitter         :out  std_logic;
        
        -- Set unit to be receiver
        set_receiver            :out  std_logic;
        
        -- Set unit to be idle
        set_idle                :out  std_logic;
        
        -----------------------------------------------------------------------
        -- Fault confinement Interface
        -----------------------------------------------------------------------
        -- Unit is error active
        is_err_active           :in   std_logic;
        
        -- Unit is error passive
        is_err_passive          :in   std_logic;
        
        -- Unit is Bus-off
        is_bus_off              :in   std_logic;
        
        -- Error detected
        err_detected            :out  std_logic;
        
        -- Primary Error
        primary_error           :out  std_logic;
        
        -- Active Error or Overload flag is being tranmsmitted
        act_err_ovr_flag        :out  std_logic;

        -- Error delimiter too late
        err_delim_late          :out  std_logic;
        
        -- Set unit to be error active
        set_err_active          :out   std_logic;
        
        -- Error counters should remain unchanged
        err_ctrs_unchanged      :out   std_logic;
        
        -----------------------------------------------------------------------
        -- TX and RX Trigger signals to Sample and Transmitt Data
        -----------------------------------------------------------------------
        -- TX Trigger (in SYNC segment) 
        tx_trigger              :in   std_logic;
        
        -- RX Trigger (one clock cycle delayed after Sample point)
        rx_trigger              :in   std_logic;

        ------------------------------------------------------------------------
        -- CAN Bus serial data stream
        ------------------------------------------------------------------------
        -- TX Data
        tx_data_nbs             :out  std_logic;
        
        -- TX Data (post bit stuffing)
        tx_data_wbs             :in   std_logic;

        -- RX Data
        rx_data_nbs             :in   std_logic;

        ------------------------------------------------------------------------
        -- Bit Stuffing Interface
        ------------------------------------------------------------------------
        -- Bit Stuffing enabled
        stuff_enable            :out  std_logic;
        
        -- Bit De-stuffing enabled
        destuff_enable          :out  std_logic;

        -- Bit Stuffing type (0-Normal, 1-Fixed)
        fixed_stuff             :out  std_logic;

        -- Length of Bit Stuffing rule
        stuff_length            :out  std_logic_vector(2 downto 0);

        -- Enable detection of Stuff Error
        stuff_error_enable      :out  std_logic;

        -- Number of de-stuffed bits modulo 8
        dst_ctr                 :in   natural range 0 to 7;
        
        -- Number of stuffed bits modulo 8
        bst_ctr                 :in   natural range 0 to 7;
        
        -- Stuff Error
        stuff_error             :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Bus Sampling Interface
        ------------------------------------------------------------------------
        -- Bit Error detected
        bit_error               :in   std_logic;
        
        -----------------------------------------------------------------------
        -- CRC Interface
        -----------------------------------------------------------------------
        -- Enable CRC calculation
        crc_enable              :out  std_logic;
        
        -- CRC calculation - speculative enable
        crc_spec_enable         :out   std_logic;
        
        -- CRC Source to be used (CRC 15, CRC 17, CRC 21)
        crc_src                 :out  std_logic_vector(1 downto 0);

        -- Calculated CRC 15
        crc_15                  :in   std_logic_vector(14 downto 0);

        -- Calculated CRC 17
        crc_17                  :in   std_logic_vector(16 downto 0);
        
        -- Calculated CRC 21
        crc_21                  :in   std_logic_vector(20 downto 0);
        
        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control              :out  std_logic_vector(1 downto 0);
        
        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation
        sync_control            :out  std_logic_vector(1 downto 0); 
        
        -- No Resynchronisation due to positive phase error
        no_pos_resync           :out   std_logic;
        
        -- Clear the Shift register for secondary sampling point.
        ssp_reset               :out  std_logic;
        
        -- Enable measurement of Transciever delay
        trv_delay_calib         :out  std_logic;

        -- Transmitted frame is valid
        tran_valid              :out  std_logic;

        -- Received frame is valid
        rec_valid               :out  std_logic;

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- ACK received
        ack_received            :out  std_logic;

        -- Bit rate shifted
        br_shifted              :out  std_logic;
        
        -- Form Error has occurred
        form_error              :out  std_logic;

        -- ACK Error has occurred
        ack_error               :out  std_logic;
        
        -- CRC Error has occurred
        crc_error               :out  std_logic
    );
end entity;

architecture rtl of protocol_control is

  -----------------------------------------------------------------------------
  -- Driving bus aliases
  -----------------------------------------------------------------------------
  
  -- Whenever FD Frames are supported for reciever
  signal drv_can_fd_ena           :     std_logic;
  
  -- Bus Monitoring mode enabled
  signal drv_bus_mon_ena          :     std_logic;
  
  -- Retransmition limit enabled for errornous frames
  signal drv_retr_lim_ena         :     std_logic;
  
  -- Retransmittion treshold
  signal drv_retr_th              :     std_logic_vector(3 downto 0);
  
  -- Self Test Mode enabled
  signal drv_self_test_ena        :     std_logic;
  
  -- Forbidding acknowledge mode
  signal drv_ack_forb             :     std_logic;
  
  -- Enabling the whole controller
  signal drv_ena                  :     std_logic;
  
  -- Type of FD Format Frame (ISO, non-ISO)
  signal drv_fd_type              :     std_logic;
  
  -- Internal Loopback enabled
  signal drv_int_loopback_ena     :     std_logic;
  
  -- Bus off restart
  signal drv_bus_off_reset        :     std_logic;
  
  
  -----------------------------------------------------------------------------
  -- Internal signals
  -----------------------------------------------------------------------------
  -- TXT Buffer word (endianity swapped)
  signal tran_word_swapped        :     std_logic_vector(31 downto 0);
  
  -- Error frame request
  signal err_frm_req              :     std_logic;
  
  -- Load commands for TX Shift register
  signal tx_load_base_id          :     std_logic;
  signal tx_load_ext_id           :     std_logic;
  signal tx_load_dlc              :     std_logic;
  signal tx_load_data_word        :     std_logic;
  signal tx_load_stuff_count      :     std_logic;
  signal tx_load_crc              :     std_logic;
  
  -- TX Shift register enabled
  signal tx_shift_ena             :     std_logic;

  -- Transmit dominant bit
  signal tx_dominant              :     std_logic;
  
  -- Clear all registers in RX Shift register
  signal rx_clear                :      std_logic;
    
  -- Store commands for RX Shift register 
  signal rx_store_base_id        :     std_logic;
  signal rx_store_ext_id         :     std_logic;
  signal rx_store_ide            :     std_logic;
  signal rx_store_rtr            :     std_logic;
  signal rx_store_edl            :     std_logic;
  signal rx_store_dlc            :     std_logic;
  signal rx_store_esi            :     std_logic;
  signal rx_store_brs            :     std_logic;
  signal rx_store_stuff_count    :     std_logic;
    
  -- Clock Enable RX Shift register for each byte.
  signal rx_shift_ena            :     std_logic_vector(3 downto 0);
    
  -- Selector for inputs of each byte of shift register
  -- (0-Previous byte output, 1- RX Data input)
  signal rx_shift_in_sel         :     std_logic;
    
  -- RX value of Remote transmission request
  signal rec_is_rtr_i            :     std_logic;

  -- RX value of DLC (combinational), valid only in last bit of DLC
  signal rec_dlc_d               :     std_logic_vector(3 downto 0);
    
  -- RX value of DLC (captured)
  signal rec_dlc_q               :     std_logic_vector(3 downto 0);
    
  -- RX frame type (0-CAN 2.0, 1- CAN FD)
  signal rec_frame_type_i        :     std_logic;
  
  -- Preload control counter
  signal ctrl_ctr_pload        :      std_logic;
    
  -- Control counter preload value
  signal ctrl_ctr_pload_val    :      std_logic_vector(8 downto 0);
    
  -- Control counter is enabled
  signal ctrl_ctr_ena            :      std_logic;
    
  -- Control counter is zero
  signal ctrl_ctr_zero           :      std_logic;
    
  -- Control counter is equal to 1
  signal ctrl_ctr_one            :      std_logic;

  -- Control counter counted multiple of 8 bits
  signal ctrl_counted_byte       :      std_logic;

  -- Control counter byte index within a memory word
  signal ctrl_counted_byte_index :      std_logic_vector(1 downto 0);
    
  -- Control counter - TXT Buffer memory index
  signal ctrl_ctr_mem_index      :      std_logic_vector(4 downto 0);
  
  -- Complementary counter enable
  signal compl_ctr_ena           :      std_logic;
  
  -- Reintegration counter Clear (synchronous)
  signal reinteg_ctr_clr         :      std_logic;

  -- Enable counting (with RX Trigger)
  signal reinteg_ctr_enable      :      std_logic;
        
  -- Reintegration counter expired (reached 128)
  signal reinteg_ctr_expired     :      std_logic;
  
  -- Clear Retransmitt counter
  signal retr_ctr_clear          :      std_logic;

  -- Increment Retransmitt counter by 1
  signal retr_ctr_add            :      std_logic;

  -- Retransmitt limit was reached
  signal retr_limit_reached      :      std_logic;

  -- Form Error has occurred
  signal form_error_i            :      std_logic;

  -- ACK Error has occurred
  signal ack_error_i             :      std_logic;

  -- Perform CRC check
  signal crc_check               :      std_logic;
    
  -- Bit Error in arbitration field
  signal bit_error_arb           :      std_logic;
    
  -- Calculated CRC and Stuff count are matching received ones
  signal crc_match               :     std_logic;

  -- CRC error signalling
  signal crc_error_i             :     std_logic;

  -- Clear CRC Match flag
  signal crc_clear_match_flag    :      std_logic;

  -- CRC Source (CRC15, CRC17, CRC21)
  signal crc_src_i               :      std_logic_vector(1 downto 0);
    
  -- Error position field (for Error capture)
  signal err_pos                 :      std_logic_vector(4 downto 0);
    
  -- Arbitration field is being transmitted / received
  signal is_arbitration_i        :      std_logic;
  
  -- Bit error detection enabled
  signal bit_error_enable        :      std_logic;
  
  -- TX Data internal
  signal tx_data_nbs_i           :      std_logic;
  
  -- Received CRC (driven from RX Shift register)
  signal rx_crc                  :      std_logic_vector(20 downto 0);
  
  -- RX Stuff count (grey coded) + RX parity
  signal rx_stuff_count          :      std_logic_vector(3 downto 0);
  
  -- Stuff error enable (internal)
  signal stuff_error_enable_i    :      std_logic;
  
  -- Fixed Stuff (internal)
  signal fixed_stuff_i           :      std_logic;
  
begin
    
    ---------------------------------------------------------------------------
    -- Driving bus aliases
    ---------------------------------------------------------------------------
    drv_can_fd_ena        <=  drv_bus(DRV_CAN_FD_ENA_INDEX);
    drv_bus_mon_ena       <=  drv_bus(DRV_BUS_MON_ENA_INDEX);
    drv_retr_lim_ena      <=  drv_bus(DRV_RETR_LIM_ENA_INDEX);
    drv_retr_th           <=  drv_bus(DRV_RETR_TH_HIGH downto DRV_RETR_TH_LOW);
    drv_self_test_ena     <=  drv_bus(DRV_SELF_TEST_ENA_INDEX);
    drv_ack_forb          <=  drv_bus(DRV_ACK_FORB_INDEX);
    drv_ena               <=  drv_bus(DRV_ENA_INDEX);
    drv_fd_type           <=  drv_bus(DRV_FD_TYPE_INDEX);
    drv_int_loopback_ena  <=  drv_bus(DRV_INT_LOOBACK_ENA_INDEX);
    drv_bus_off_reset     <=  drv_bus(DRV_ERR_CTR_CLR);
  
    ---------------------------------------------------------------------------
    -- TX Data word endian swapper
    ---------------------------------------------------------------------------
    endian_swapper_tx_inst : endian_swapper 
    generic map(
        G_SWAP_BY_SIGNAL    => false,
        G_SWAP_GEN          => true,
        G_WORD_SIZE         => 4,   -- Number of Groups
        G_GROUP_SIZE        => 8    -- Group size (bits)
    )
    port map(
        input               => tran_word,           -- IN
        output              => tran_word_swapped,   -- OUT
        swap_in             => '0'                  -- IN
    );

    
    ---------------------------------------------------------------------------
    -- Protocol control FSM
    ---------------------------------------------------------------------------
    protocol_control_fsm_inst : protocol_control_fsm
    generic map(
        G_RESET_POLARITY        => G_RESET_POLARITY
    )
    port map(
        clk_sys                 => clk_sys,             -- IN
        res_n                   => res_n,               -- IN
        
        -- Signals which cause state change
        rx_trigger              => rx_trigger,          -- IN
        err_frm_req             => err_frm_req,         -- IN

        -- Memory registers interface
        drv_ena                 => drv_ena,             -- IN
        drv_fd_type             => drv_fd_type,         -- IN
        drv_bus_off_reset       => drv_bus_off_reset,   -- IN
        drv_ack_forb            => drv_ack_forb,        -- IN
        drv_self_test_ena       => drv_self_test_ena,   -- IN
        drv_bus_mon_ena         => drv_bus_mon_ena,     -- IN
        drv_retr_lim_ena        => drv_retr_lim_ena,    -- IN
        drv_int_loopback_ena    => drv_int_loopback_ena,-- IN
        drv_can_fd_ena          => drv_can_fd_ena,      -- IN
        is_control              => is_control,          -- OUT
        is_data                 => is_data,             -- OUT
        is_stuff_count          => is_stuff_count,      -- OUT
        is_crc                  => is_crc,              -- OUT
        is_crc_delim            => is_crc_delim,        -- OUT
        is_ack_field            => is_ack_field,        -- OUT
        is_ack_delim            => is_ack_delim,        -- OUT
        is_eof                  => is_eof,              -- OUT
        is_intermission         => is_intermission,     -- OUT
        is_suspend              => is_suspend,          -- OUT
        is_error                => is_error,            -- OUT
        is_overload             => is_overload,         -- OUT

        -- Data-path interface
        tx_data_wbs             => tx_data_wbs,         -- IN
        rx_data                 => rx_data_nbs,         -- IN

        -- RX Buffer interface
        store_metadata          => store_metadata,      -- OUT
        store_data              => store_data,          -- OUT
        rec_valid               => rec_valid,           -- OUT
        rec_abort               => rec_abort,           -- OUT
        sof_pulse               => sof_pulse,           -- OUT

        -- TXT Buffer, TX Arbitrator interface
        tran_frame_valid        => tran_frame_valid,    -- IN
        txtb_hw_cmd             => txtb_hw_cmd,         -- OUT
        txtb_ptr                => txtb_ptr,            -- OUT
        tran_dlc                => tran_dlc,            -- IN
        tran_is_rtr             => tran_is_rtr,         -- IN
        tran_frame_type         => tran_frame_type,     -- IN
        tran_ident_type         => tran_ident_type,     -- IN
        tran_brs                => tran_brs,            -- IN
                
        -- TX Shift register interface
        tx_load_base_id         => tx_load_base_id,     -- OUT
        tx_load_ext_id          => tx_load_ext_id,      -- OUT
        tx_load_dlc             => tx_load_dlc,         -- OUT
        tx_load_data_word       => tx_load_data_word,   -- OUT
        tx_load_stuff_count     => tx_load_stuff_count, -- OUT
        tx_load_crc             => tx_load_crc,         -- OUT
        tx_shift_ena            => tx_shift_ena,        -- OUT
        tx_dominant             => tx_dominant,         -- OUT
        
        -- RX Shift register interface
        rx_clear                => rx_clear,            -- OUT
        rx_store_base_id        => rx_store_base_id,    -- OUT
        rx_store_ext_id         => rx_store_ext_id,     -- OUT
        rx_store_ide            => rx_store_ide,        -- OUT
        rx_store_rtr            => rx_store_rtr,        -- OUT
        rx_store_edl            => rx_store_edl,        -- OUT
        rx_store_dlc            => rx_store_dlc,        -- OUT
        rx_store_esi            => rx_store_esi,        -- OUT
        rx_store_brs            => rx_store_brs,        -- OUT
        rx_store_stuff_count    => rx_store_stuff_count,-- OUT
        rx_shift_ena            => rx_shift_ena,        -- OUT
        rx_shift_in_sel         => rx_shift_in_sel,     -- OUT
        rec_is_rtr              => rec_is_rtr_i,        -- IN
        rec_dlc_d               => rec_dlc_d,           -- IN
        rec_dlc_q               => rec_dlc_q,           -- IN
        rec_frame_type          => rec_frame_type_i,    -- IN

        -- Control counter interface
        ctrl_ctr_pload          => ctrl_ctr_pload,          -- OUT
        ctrl_ctr_pload_val      => ctrl_ctr_pload_val,      -- OUT
        ctrl_ctr_ena            => ctrl_ctr_ena,            -- OUT
        ctrl_ctr_zero           => ctrl_ctr_zero,           -- IN
        ctrl_ctr_one            => ctrl_ctr_one,            -- IN
        ctrl_counted_byte       => ctrl_counted_byte,       -- IN
        ctrl_counted_byte_index => ctrl_counted_byte_index, -- IN
        ctrl_ctr_mem_index      => ctrl_ctr_mem_index,      -- IN
        compl_ctr_ena           => compl_ctr_ena,           -- OUT

        -- Reintegration counter interface
        reinteg_ctr_clr         => reinteg_ctr_clr,         -- OUT
        reinteg_ctr_enable      => reinteg_ctr_enable,      -- OUT
        reinteg_ctr_expired     => reinteg_ctr_expired,     -- IN

        -- Retransmitt counter interface
        retr_ctr_clear          => retr_ctr_clear,          -- OUT
        retr_ctr_add            => retr_ctr_add,            -- OUT
        retr_limit_reached      => retr_limit_reached,      -- IN

        -- Error detector interface
        form_error              => form_error_i,            -- OUT
        ack_error               => ack_error_i,             -- OUT
        crc_check               => crc_check,               -- OUT
        bit_error_arb           => bit_error_arb,           -- OUT
        crc_match               => crc_match,               -- IN
        crc_error               => crc_error_i,             -- OUT
        crc_clear_match_flag    => crc_clear_match_flag,    -- OUT
        crc_src                 => crc_src_i,               -- OUT
        err_pos                 => err_pos,                 -- OUT
        is_arbitration          => is_arbitration_i,        -- OUT
        
        -- Bit Stuffing/Destuffing control signals
        stuff_enable            => stuff_enable,            -- OUT
        destuff_enable          => destuff_enable,          -- OUT
        stuff_length            => stuff_length,            -- OUT
        fixed_stuff             => fixed_stuff_i,           -- OUT
        stuff_error_enable      => stuff_error_enable_i,    -- OUT
        
        -- Operation control interface
        is_transmitter          => is_transmitter,          -- IN
        is_receiver             => is_receiver,             -- IN
        is_idle                 => is_idle,                 -- IN
        arbitration_lost        => arbitration_lost,        -- OUT
        set_transmitter         => set_transmitter,         -- OUT
        set_receiver            => set_receiver,            -- OUT
        set_idle                => set_idle,                -- OUT

        -- Fault confinement interface
        primary_error           => primary_error,           -- OUT
        act_err_ovr_flag        => act_err_ovr_flag,        -- OUT
        set_err_active          => set_err_active,          -- OUT
        err_delim_late          => err_delim_late,          -- OUT
        is_err_active           => is_err_active,           -- IN
        is_err_passive          => is_err_passive,          -- IN
        is_bus_off              => is_bus_off,              -- IN

        -- Other control signals
        sp_control              => sp_control,              -- OUT
        sync_control            => sync_control,            -- OUT
        no_pos_resync           => no_pos_resync,           -- OUT
        ssp_reset               => ssp_reset,               -- OUT
        trv_delay_calib         => trv_delay_calib,         -- OUT
        tran_valid              => tran_valid,              -- OUT
        ack_received            => ack_received,            -- OUT
        crc_enable              => crc_enable,              -- OUT
        crc_spec_enable         => crc_spec_enable,         -- OUT
        bit_error_enable        => bit_error_enable,        -- OUT
        br_shifted              => br_shifted               -- OUT
    );

    ---------------------------------------------------------------------------
    -- Control counter
    ---------------------------------------------------------------------------
    control_counter_inst : control_counter
    generic map(
        G_RESET_POLARITY       => G_RESET_POLARITY,
        G_CTRL_CTR_WIDTH       => G_CTRL_CTR_WIDTH
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        -- Control signals
        rx_trigger              => rx_trigger,              -- IN
        ctrl_ctr_ena            => ctrl_ctr_ena,            -- IN
        ctrl_ctr_pload          => ctrl_ctr_pload,          -- IN
        ctrl_ctr_pload_val      => ctrl_ctr_pload_val,      -- IN
        compl_ctr_ena           => compl_ctr_ena,           -- IN

        -- Status signals
        ctrl_ctr_zero           => ctrl_ctr_zero,           -- OUT
        ctrl_ctr_one            => ctrl_ctr_one,            -- OUT
        ctrl_counted_byte       => ctrl_counted_byte,       -- OUT
        ctrl_counted_byte_index => ctrl_counted_byte_index, -- OUT
        ctrl_ctr_mem_index      => ctrl_ctr_mem_index       -- OUT
    );


    ---------------------------------------------------------------------------
    -- Reintegration counter
    ---------------------------------------------------------------------------
    reintegration_counter_inst : reintegration_counter
    generic map(
        G_RESET_POLARITY        => G_RESET_POLARITY
    )
    port map(
        clk_sys                 => clk_sys,             -- IN
        res_n                   => res_n,               -- IN

        -- Control signals
        reinteg_ctr_clr         => reinteg_ctr_clr,     -- IN
        reinteg_ctr_enable      => reinteg_ctr_enable,  -- IN
        rx_trigger              => rx_trigger,          -- IN

        -- Status signals
        reinteg_ctr_expired     => reinteg_ctr_expired  -- OUT
    );


    ---------------------------------------------------------------------------
    -- Retransmitt counter
    ---------------------------------------------------------------------------
    retransmitt_counter_inst : retransmitt_counter
    generic map(
        G_RESET_POLARITY        => G_RESET_POLARITY,
        G_RETR_LIM_CTR_WIDTH    => G_RETR_LIM_CTR_WIDTH
    )
    port map(
        clk_sys             => clk_sys,             -- IN
        res_n               => res_n,               -- IN

        -- Control signals
        txtb_changed        => txtb_changed,        -- IN
        retr_ctr_clear      => retr_ctr_clear,      -- IN
        retr_ctr_add        => retr_ctr_add,        -- IN
        retr_limit          => drv_retr_th,         -- IN

        -- Status signals
        retr_limit_reached  => retr_limit_reached   -- OUT
    );


    ---------------------------------------------------------------------------
    -- Error detector
    ---------------------------------------------------------------------------
    error_detector_inst : error_detector
    generic map(
        G_RESET_POLARITY        => G_RESET_POLARITY,
        G_ERR_VALID_PIPELINE    => G_ERR_VALID_PIPELINE
    )
    port map(
        clk_sys                 => clk_sys,             -- IN
        res_n                   => res_n,               -- IN

        -- Data-path interface
        tx_data                 => tx_data_nbs_i,       -- IN
        rx_data                 => rx_data_nbs,         -- IN
        
        -- Error sources
        bit_error               => bit_error,           -- IN
        bit_error_arb           => bit_error_arb,       -- IN
        stuff_error             => stuff_error,         -- IN
        form_error              => form_error_i,        -- IN
        ack_error               => ack_error_i,         -- IN
        crc_error               => crc_error_i,         -- IN
        
        -- CRC comparison data
        rx_crc                  => rx_crc,              -- IN
        crc_15                  => crc_15,              -- IN
        crc_17                  => crc_17,              -- IN
        crc_21                  => crc_21,              -- IN
        rx_stuff_count          => rx_stuff_count,      -- IN
        dst_ctr                 => dst_ctr,             -- IN

        -- Control signals
        bit_error_enable        => bit_error_enable,        -- IN
        stuff_error_enable      => stuff_error_enable_i,    -- IN
        fixed_stuff             => fixed_stuff_i,           -- IN
        err_pos                 => err_pos,                 -- IN
        crc_check               => crc_check,               -- IN
        crc_clear_match_flag    => crc_clear_match_flag,    -- IN
        crc_src                 => crc_src_i,               -- IN
        drv_fd_type             => drv_fd_type,             -- IN
        is_arbitration          => is_arbitration_i,        -- IN
        is_transmitter          => is_transmitter,          -- IN
        is_err_passive          => is_receiver,             -- IN

        -- Status output
        err_frm_req             => err_frm_req,         -- OUT
        err_detected            => err_detected,        -- OUT
        erc_capture             => erc_capture,         -- OUT
        crc_match               => crc_match,           -- OUT
        err_ctrs_unchanged      => err_ctrs_unchanged   -- OUT
    );

    
    ---------------------------------------------------------------------------
    -- TX Shift register
    ---------------------------------------------------------------------------
    tx_shift_reg_inst : tx_shift_reg
    port map(
        clk_sys                 => clk_sys,             -- IN
        res_n                   => res_n,               -- IN

        tx_trigger              => tx_trigger,          -- IN
        tx_data                 => tx_data_nbs_i,       -- OUT

        -- Protocol control FSM interface
        tx_load_base_id         => tx_load_base_id,     -- IN
        tx_load_ext_id          => tx_load_ext_id,      -- IN
        tx_load_dlc             => tx_load_dlc,         -- IN
        tx_load_data_word       => tx_load_data_word,   -- IN
        tx_load_stuff_count     => tx_load_stuff_count, -- IN
        tx_load_crc             => tx_load_crc,         -- IN
        tx_shift_ena            => tx_shift_ena,        -- IN
        tx_dominant             => tx_dominant,         -- IN
        crc_src                 => crc_src_i,           -- IN

        -- CAN CRC Interface
        crc_15                  => crc_15,              -- IN
        crc_17                  => crc_17,              -- IN
        crc_21                  => crc_21,              -- IN

        err_frm_req             => err_frm_req,             -- IN
        is_err_active           => is_err_active,           -- IN
        bst_ctr                 => bst_ctr,                 -- IN
        tran_word               => tran_word,               -- IN
        tran_word_swapped       => tran_word_swapped,       -- IN
        tran_dlc                => tran_dlc                 -- IN
    );


    ---------------------------------------------------------------------------
    -- RX Shift register
    ---------------------------------------------------------------------------
    rx_shift_reg_inst : rx_shift_reg
    generic map(
        G_RESET_POLARITY        => G_RESET_POLARITY
    )
    port map(
        clk_sys                 => clk_sys,             -- IN
        res_n                   => res_n,               -- IN
        rx_trigger              => rx_trigger,          -- IN

        -- Data-path interface
        rx_data                 => rx_data_nbs,         -- IN

        -- Protocol control FSM interface
        rx_clear                => rx_clear,                -- IN
        rx_shift_ena            => rx_shift_ena,            -- IN
        rx_shift_in_sel         => rx_shift_in_sel,         -- IN
        rx_store_base_id        => rx_store_base_id,        -- IN
        rx_store_ext_id         => rx_store_ext_id,         -- IN
        rx_store_ide            => rx_store_ide,            -- IN
        rx_store_rtr            => rx_store_rtr,            -- IN
        rx_store_edl            => rx_store_edl,            -- IN
        rx_store_dlc            => rx_store_dlc,            -- IN
        rx_store_esi            => rx_store_esi,            -- IN
        rx_store_brs            => rx_store_brs,            -- IN
        rx_store_stuff_count    => rx_store_stuff_count,    -- IN
        
        -- RX Buffer interface
        rec_ident               => rec_ident,           -- OUT
        rec_dlc_d               => rec_dlc_d,           -- OUT
        rec_dlc                 => rec_dlc_q,           -- OUT
        rec_is_rtr              => rec_is_rtr_i,        -- OUT
        rec_ident_type          => rec_ident_type,      -- OUT
        rec_frame_type          => rec_frame_type_i,    -- OUT
        rec_brs                 => rec_brs,             -- OUT
        rec_esi                 => rec_esi,             -- OUT
        store_data_word         => store_data_word,     -- OUT
        
        -- CRC information for CRC comparison
        rx_crc                  => rx_crc,              -- OUT
        rx_stuff_count          => rx_stuff_count       -- OUT
    );


    ---------------------------------------------------------------------------
    -- Internal signals propagation to output
    ---------------------------------------------------------------------------
    tx_data_nbs <= tx_data_nbs_i;
    rec_frame_type <= rec_frame_type_i;
    rec_is_rtr <= rec_is_rtr_i;
    rec_dlc <= rec_dlc_q;
    form_error <= form_error_i;
    ack_error <= ack_error_i;
    crc_error <= crc_error_i;
    is_arbitration <= is_arbitration_i;
    stuff_error_enable <= stuff_error_enable_i;
    fixed_stuff <= fixed_stuff_i;
    crc_src <= crc_src_i;
    
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);
        
    -- psl no_invalid_ack_err_asrt : assert never
    --  ((ack_error = '1' or crc_error = '1' or
    --    stuff_error = '1' or form_error_i = '1') and (is_error = '1'))
    -- report "ACK, Stuff, CRC Errors can't occur during Error or overload flag"
    --  severity error;
    
    -- psl sample_sec_proper_asrt : assert never
    --  (sp_control = SECONDARY_SAMPLE and is_transmitter = '0')
    --  report "Secondary sampling is allowed only for transmitter"
    --  severity error;


end architecture;