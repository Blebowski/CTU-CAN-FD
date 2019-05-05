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
--  Top level entity of CAN Core covering whole functionality of CAN FD Protocol
--  Instantiates: 1*protocol_control, 1*operation_control, 1*bitStuffing_v2,    
--  1*bitDestuffing, 2*CRC, 1* tranBuffer                                       
--  Logic for switching sampling and synchronisation signals implemented here   
--  stat_bus assignment implemented here!                                       
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015  Created file
--    26.4.2019  Stashed all previous notes due to re-implementation.
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
use work.endian_swap.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity can_core is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :    std_logic := '0';
        
        -- Number of signals in Sample trigger
        G_SAMPLE_TRIGGER_COUNT  :   natural range 2 to 8 := 2;
        
        -- Control counter width
        G_CTRL_CTR_WIDTH        :     natural := 9;
        
        -- Retransmitt limit counter width
        G_RETR_LIM_CTR_WIDTH    :     natural := 4;
        
        -- Insert pipeline on "error_valid" 
        G_ERR_VALID_PIPELINE    :     boolean := true;
        
        -- CRC 15 polynomial
        G_CRC15_POL             :     std_logic_vector(15 downto 0) := x"C599";
        
        -- CRC 17 polynomial
        G_CRC17_POL             :     std_logic_vector(19 downto 0) := x"3685B";
        
        -- CRC 15 polynomial
        G_CRC21_POL             :     std_logic_vector(23 downto 0) := x"302899"  
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous reset
        res_n                  :in   std_logic;
        
        ------------------------------------------------------------------------    
        -- Memory registers interface
        ------------------------------------------------------------------------
        -- Driving bus
        drv_bus                :in   std_logic_vector(1023 downto 0);

        -- Status bus
        stat_bus               :out  std_logic_vector(511 downto 0);

        ------------------------------------------------------------------------
        -- Tx Arbitrator and TXT Buffers interface
        ------------------------------------------------------------------------
        -- TX Data word
        tran_word              :in   std_logic_vector(31 downto 0);
        
        -- TX Data length code
        tran_dlc               :in   std_logic_vector(3 downto 0);
        
        -- TX Remote transmission request flag
        tran_is_rtr            :in   std_logic;

        -- TX Identifier type (0-Basic, 1-Extended)
        tran_ident_type        :in   std_logic;

        -- TX Frame type (0-CAN 2.0, 1-CAN FD)
        tran_frame_type        :in   std_logic;

        -- TX Bit Rate Shift
        tran_brs               :in   std_logic;

        -- Frame in TXT Buffer is valid any can be transmitted.
        tran_frame_valid       :in   std_logic; 

        -- HW Commands for TX Arbitrator and TXT Buffers
        txtb_hw_cmd            :out  t_txtb_hw_cmd;

        -- Selected TXT Buffer index changed
        txtb_changed           :in   std_logic;

        -- Pointer to TXT buffer memory
        txtb_ptr               :out  natural range 0 to 19;

        -- Transition to bus off has occurred
        is_bus_off             :out  std_logic;

        ------------------------------------------------------------------------
        -- Recieve Buffer and Message Filter Interface
        ------------------------------------------------------------------------
        -- RX CAN Identifier
        rec_ident              :out  std_logic_vector(28 downto 0);

        -- RX Data length code
        rec_dlc                :out  std_logic_vector(3 downto 0);

        -- RX Recieved identifier type (0-BASE Format, 1-Extended Format);
        rec_ident_type         :out  std_logic;

        -- RX frame type (0-CAN 2.0, 1- CAN FD) 
        rec_frame_type         :out  std_logic;

        -- RX Remote transmission request Flag
        rec_is_rtr             :out  std_logic;

        -- RX Bit Rate Shift bit
        rec_brs                :out  std_logic;

        -- RX Error state indicator
        rec_esi      	       :out  std_logic;

        -- RX Frame received succesfully, can be commited to RX Buffer.
        rec_valid              :out  std_logic; 

        -- Metadata are received OK, and can be stored in RX Buffer.
        store_metadata         :out  std_logic;

        -- Store data word to RX Buffer. 
        store_data             :out  std_logic;
        
        -- Data words to be stored to RX Buffer.
        store_data_word        :out  std_logic_vector(31 downto 0);

        -- Abort storing of frame in RX Buffer. Revert to last frame.
        rec_abort              :out  std_logic;
        
        -- Pulse in Start of Frame
        sof_pulse              :out  std_logic;

        ------------------------------------------------------------------------
        -- Interrupt Manager Interface 
        ------------------------------------------------------------------------
        -- Arbitration was lost
        arbitration_lost       :out  std_logic;

        -- Frame stored in CAN Core was sucessfully transmitted
        tran_valid             :out  std_logic; 

        -- Bit Rate Was Shifted
        br_shifted             :out  std_logic;

        -- Error is detected (Error frame will be transmitted)
        err_detected           :out  std_logic;

        -- Error passive state changed
        error_passive_changed  :out  std_logic;

        -- Error warning limit reached
        error_warning_limit    :out  std_logic;

        ------------------------------------------------------------------------
        -- Prescaler interface 
        ------------------------------------------------------------------------
        -- RX Triggers (in Sample Point)
        rx_triggers   :in   std_logic_vector(G_SAMPLE_TRIGGER_COUNT - 1 downto 0);
        
        -- TX Trigger
        tx_trigger    :in   std_logic;
        
        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation
        sync_control  :out  std_logic_vector(1 downto 0);
        
        -- No positive resynchronisation 
        no_pos_resync :out  std_logic;

        ------------------------------------------------------------------------
        -- CAN Bus serial data stream
        ------------------------------------------------------------------------
        -- RX Data from CAN Bus
        rx_data_wbs         :in   std_logic; 

        -- TX Data to CAN Bus
        tx_data_wbs         :out  std_logic; 

        ------------------------------------------------------------------------
        -- Others
        ------------------------------------------------------------------------
        timestamp           :in   std_logic_vector(63 downto 0);

        -- Sample control (Nominal, Data, Secondary)
        sp_control          :out  std_logic_vector(1 downto 0); 

        -- Secondary sample point reset
        ssp_reset           :out  std_logic; 

        -- Enable measurement of Transciever delay
        trv_delay_calib     :out  std_logic;

        -- Bit Error detected 
        bit_error           :in   std_logic;
        
        -- Secondary sample signal 
        sample_sec          :in   std_logic
    );
end entity;

architecture rtl of can_core is

    ----------------------------------------------------------------------------
    -- Driving bus aliases
    ----------------------------------------------------------------------------
    signal drv_clr_rx_ctr          :     std_logic;
    signal drv_clr_tx_ctr          :     std_logic;
    signal drv_bus_mon_ena         :     std_logic;
   
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Internal signals
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    -- TXT Buffer control
    signal tran_word_i        :   std_logic_vector(31 downto 0);
    signal tran_dlc_i               :   std_logic_vector(3 downto 0);
    signal tran_is_rtr_i            :   std_logic;
    signal tran_ident_type_i        :   std_logic;
    signal tran_frame_type_i        :   std_logic;
    signal tran_brs_i               :   std_logic; 
    signal txtb_hw_cmd_i             :   t_txtb_hw_cmd;
    
    -- Received frame
    signal rec_ident_i              :   std_logic_vector(28 downto 0);
    signal rec_dlc_i                :   std_logic_vector(3 downto 0);
    signal rec_ident_type_i         :   std_logic;
    signal rec_frame_type_i         :   std_logic;
    signal rec_is_rtr_i             :   std_logic;
    signal rec_brs_i                :   std_logic;
    signal rec_esi_i                :   std_logic;
    
    -- Arbitration lost capture
    signal alc                      :   std_logic_vector(7 downto 0);
    
    -- Error code capture
    signal erc_capture              :   std_logic_vector(7 downto 0);
    
    -- Operation control interface
    signal is_transmitter           :   std_logic;
    signal is_receiver              :   std_logic;
    signal is_idle                  :   std_logic;
    signal arbitration_lost_i       :   std_logic;
    signal set_transmitter          :   std_logic;
    signal set_receiver             :   std_logic;
    signal set_idle                 :   std_logic;
    
    -- Fault confinement Interface
    signal is_err_active           :    std_logic;
    signal is_err_passive          :    std_logic;
    signal is_bus_off_i            :    std_logic;
    signal err_detected_i          :    std_logic;
    signal primary_error           :    std_logic;
    signal act_err_ovr_flag        :    std_logic;
    signal err_delim_late          :    std_logic;
    signal set_err_active          :    std_logic;
    signal err_ctrs_unchanged      :    std_logic;

    -- Bit Stuffing Interface
    signal stuff_enable            :    std_logic;
    signal destuff_enable          :    std_logic;
    signal fixed_stuff             :    std_logic;
    signal stuff_length            :    std_logic_vector(2 downto 0);
    signal stuff_error_enable      :    std_logic;
    signal dst_ctr                 :    natural range 0 to 7;
    signal bst_ctr                 :    natural range 0 to 7;
    signal stuff_error             :    std_logic;
    
    -- CRC Interface
    signal crc_enable              :    std_logic;
    signal crc_spec_enable         :    std_logic;
    signal crc_src                 :    std_logic_vector(1 downto 0);
    signal crc_15                  :    std_logic_vector(14 downto 0);
    signal crc_17                  :    std_logic_vector(16 downto 0);
    signal crc_21                  :    std_logic_vector(20 downto 0);

    -- Protocol control - control outputs
    signal sp_control_i            :    std_logic_vector(1 downto 0);
    signal sync_control_i          :    std_logic_vector(1 downto 0); 
    signal ssp_reset_i             :    std_logic;
    signal trv_delay_calib_i       :    std_logic;
    signal tran_valid_i            :    std_logic;
    signal rec_valid_i             :    std_logic;
    signal ack_received_i          :    std_logic;
    signal br_shifted_i            :    std_logic;
    
    -- Fault confinement status signals
    signal error_passive_changed_i :    std_logic;
    signal error_warning_limit_i   :    std_logic;
    
    signal tx_err_ctr              :    std_logic_vector(8 downto 0);
    signal rx_err_ctr              :    std_logic_vector(8 downto 0);
    signal norm_err_ctr            :    std_logic_vector(15 downto 0);
    signal data_err_ctr            :    std_logic_vector(15 downto 0);
    
    -- Protocol control triggers
    signal pc_tx_trigger           :    std_logic;
    signal pc_rx_trigger           :    std_logic;
    
    -- Protocol control data inputs/outputs
    signal pc_tx_data_nbs          :    std_logic;
    signal pc_rx_data_nbs          :    std_logic;
    
    -- CRC Data inputs
    signal crc_data_tx_wbs         :    std_logic;
    signal crc_data_tx_nbs         :    std_logic;
    signal crc_data_rx_wbs         :    std_logic;
    signal crc_data_rx_nbs         :    std_logic;
    
    -- CRC Trigger inputs
    signal crc_trig_tx_wbs         :    std_logic;
    signal crc_trig_tx_nbs         :    std_logic;
    signal crc_trig_rx_wbs         :    std_logic;
    signal crc_trig_rx_nbs         :    std_logic;
    signal tx_trigger_q            :    std_logic;
    
    -- Bit stuffing signals
    signal bst_data_in             :    std_logic;
    signal bst_data_out            :    std_logic;
    signal bst_trigger             :    std_logic;
    signal data_halt               :    std_logic;
    
    -- Bit destuffing signals
    signal bds_data_in             :    std_logic;
    signal bds_data_out            :    std_logic;
    signal bds_trigger             :    std_logic;
    signal destuffed               :    std_logic;
    
    -- Bus traffic counters
    signal tx_ctr                  :    std_logic_vector(31 downto 0);
    signal rx_ctr                  :    std_logic_vector(31 downto 0);
    
    signal tx_data_wbs_i           :    std_logic;
    
    -- Looped back data for bus monitoring mode
    signal lpb_dominant            :    std_logic;
    
    -- Error indication
    signal form_error              :    std_logic;
    signal ack_error               :    std_logic;
    signal crc_error               :    std_logic;
    
    -- Protocol control debug  information
    signal is_arbitration          :     std_logic;
    signal is_control              :     std_logic;
    signal is_data                 :     std_logic;
    signal is_stuff_count          :     std_logic;
    signal is_crc                  :     std_logic;
    signal is_crc_delim            :     std_logic;
    signal is_ack_field            :     std_logic;
    signal is_ack_delim            :     std_logic;
    signal is_eof                  :     std_logic;
    signal is_error                :     std_logic;
    signal is_intermission         :     std_logic;
    signal is_suspend              :     std_logic;
    signal is_overload             :     std_logic;
    
begin
  
    ----------------------------------------------------------------------------
    -- Driving bus aliases
    ----------------------------------------------------------------------------
    drv_clr_rx_ctr        <=  drv_bus(DRV_CLR_RX_CTR_INDEX);
    drv_clr_tx_ctr        <=  drv_bus(DRV_CLR_TX_CTR_INDEX);
    drv_bus_mon_ena       <=  drv_bus(DRV_BUS_MON_ENA_INDEX);

    ----------------------------------------------------------------------------
    -- Protocol control
    ----------------------------------------------------------------------------
    protocol_control_inst : protocol_control
    generic map(
        G_RESET_POLARITY        => G_RESET_POLARITY,
        G_CTRL_CTR_WIDTH        => G_CTRL_CTR_WIDTH,
        G_RETR_LIM_CTR_WIDTH    => G_RETR_LIM_CTR_WIDTH,
        G_ERR_VALID_PIPELINE    => G_ERR_VALID_PIPELINE
    )
    port map(
        clk_sys                 => clk_sys,             -- IN
        res_n                   => res_n,               -- IN
        
        -- Memory registers interface
        drv_bus                 => drv_bus,             -- IN
        alc                     => alc,                 -- OUT
        erc_capture             => erc_capture,         -- OUT
        is_arbitration          => is_arbitration,      -- OUT
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
        
        -- TXT Buffers interface
        tran_word               => tran_word_i,         -- IN
        tran_dlc                => tran_dlc_i,          -- IN
        tran_is_rtr             => tran_is_rtr_i,       -- IN
        tran_ident_type         => tran_ident_type_i,   -- IN
        tran_frame_type         => tran_frame_type_i,   -- IN
        tran_brs                => tran_brs_i,          -- IN
        tran_frame_valid        => tran_frame_valid,    -- IN
        txtb_hw_cmd             => txtb_hw_cmd_i,       -- IN
        txtb_ptr                => txtb_ptr,            -- OUT
        txtb_changed            => txtb_changed,        -- OUT
        
        -- RX Buffer interface
        rec_ident               => rec_ident_i,         -- OUT
        rec_dlc                 => rec_dlc_i,           -- OUT
        rec_is_rtr              => rec_is_rtr_i,        -- OUT
        rec_ident_type          => rec_ident_type_i,    -- OUT
        rec_frame_type          => rec_frame_type_i,    -- OUT
        rec_brs                 => rec_brs_i,           -- OUT
        rec_esi                 => rec_esi_i,           -- OUT
        store_metadata          => store_metadata,      -- OUT
        rec_abort               => rec_abort,           -- OUT
        store_data              => store_data,          -- OUT
        store_data_word         => store_data_word,     -- OUT
        sof_pulse               => sof_pulse,           -- OUT
    
        -- Operation control FSM Interface
        is_transmitter          => is_transmitter,      -- IN
        is_receiver             => is_receiver,         -- IN
        is_idle                 => is_idle,             -- IN
        arbitration_lost        => arbitration_lost_i,  -- OUT
        set_transmitter         => set_transmitter,     -- OUT
        set_receiver            => set_receiver,        -- OUT
        set_idle                => set_idle,            -- OUT
        
        -- Fault confinement Interface
        is_err_active           => is_err_active,       -- IN
        is_err_passive          => is_err_passive,      -- IN
        is_bus_off              => is_bus_off_i,        -- IN
        err_detected            => err_detected_i,      -- OUT
        primary_error           => primary_error,       -- OUT
        act_err_ovr_flag        => act_err_ovr_flag,    -- OUT
        err_delim_late          => err_delim_late,      -- OUT
        set_err_active          => set_err_active,      -- OUT
        err_ctrs_unchanged      => err_ctrs_unchanged,  -- OUT
        
        -- TX and RX Trigger signals to Sample and Transmitt Data
        tx_trigger              => pc_tx_trigger,       -- IN
        rx_trigger              => pc_rx_trigger,       -- IN

        -- CAN Bus serial data stream
        tx_data_nbs             => pc_tx_data_nbs,      -- OUT
        rx_data_nbs             => pc_rx_data_nbs,      -- IN

        -- Bit Stuffing Interface
        stuff_enable            => stuff_enable,        -- OUT
        destuff_enable          => destuff_enable,      -- OUT
        fixed_stuff             => fixed_stuff,         -- OUT
        stuff_length            => stuff_length,        -- OUT
        stuff_error_enable      => stuff_error_enable,  -- OUT
        dst_ctr                 => dst_ctr,             -- IN
        bst_ctr                 => bst_ctr,             -- IN
        stuff_error             => stuff_error,         -- IN
        
        -- Bus Sampling Interface
        bit_error               => bit_error,           -- IN
        
        -- CRC Interface
        crc_enable              => crc_enable,          -- OUT
        crc_spec_enable         => crc_spec_enable,     -- OUT
        crc_src                 => crc_src,             -- OUT
        crc_15                  => crc_15,              -- IN
        crc_17                  => crc_17,              -- IN
        crc_21                  => crc_21,              -- IN
        
        -- Control signals
        sp_control              => sp_control_i,        -- OUT
        sync_control            => sync_control_i,      -- OUT
        no_pos_resync           => no_pos_resync,       -- OUT
        ssp_reset               => ssp_reset_i,         -- OUT
        trv_delay_calib         => trv_delay_calib_i,   -- OUT
        tran_valid              => tran_valid_i,        -- OUT
        rec_valid               => rec_valid_i,         -- OUT
        
        -- Status signals
        ack_received            => ack_received_i,      -- OUT
        br_shifted              => br_shifted_i,        -- OUT
        form_error              => form_error,          -- OUT
        ack_error               => ack_error,           -- OUT
        crc_error               => crc_error            -- OUT
    );


    ---------------------------------------------------------------------------
    -- Operation control FSM
    ---------------------------------------------------------------------------
    operation_control_inst : operation_control
    generic map(
        G_RESET_POLARITY     => G_RESET_POLARITY    
    )
    port map(
        clk_sys              => clk_sys,                -- IN
        res_n                => res_n,                  -- IN

        -- Memory registers Interface
        drv_bus              => drv_bus,                -- IN

        -- Protocol Control Interface
        arbitration_lost     => arbitration_lost_i,     -- IN
        set_transmitter      => set_transmitter,        -- IN
        set_receiver         => set_receiver,           -- IN
        set_idle             => set_idle,               -- IN
        is_transmitter       => is_transmitter,         -- OUT
        is_receiver          => is_receiver,            -- OUT
        is_idle              => is_idle                 -- OUT
    );
    

    ---------------------------------------------------------------------------
    -- Fault confinement
    ---------------------------------------------------------------------------
    fault_confinement_inst : fault_confinement
    generic map(
        G_RESET_POLARITY     => G_RESET_POLARITY
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        -- Memory registers interface
        drv_bus                 => drv_bus,                 -- IN
          
        -- Error signalling for interrupts
        error_passive_changed   => error_passive_changed_i, -- OUT
        error_warning_limit     => error_warning_limit_i,   -- OUT

        -- Operation control Interface
        is_transmitter          => is_transmitter,          -- IN
        is_receiver             => is_receiver,             -- IN
        
        -- Protocol control Interface
        sp_control              => sp_control_i,            -- IN
        set_err_active          => set_err_active,          -- IN
        err_detected            => err_detected_i,          -- IN
        err_ctrs_unchanged      => err_ctrs_unchanged,      -- IN
        primary_error           => primary_error,           -- IN
        act_err_ovr_flag        => act_err_ovr_flag,        -- IN
        err_delim_late          => err_delim_late,          -- IN
        tran_valid              => tran_valid_i,            -- IN
        rec_valid               => rec_valid_i,             -- IN

        -- Fault confinement State indication
        is_err_active           => is_err_active,           -- OUT
        is_err_passive          => is_err_passive,          -- OUT
        is_bus_off              => is_bus_off_i,            -- OUT

        -- Error counters
        tx_err_ctr              => tx_err_ctr,              -- OUT
        rx_err_ctr              => rx_err_ctr,              -- OUT
        norm_err_ctr            => norm_err_ctr,            -- OUT
        data_err_ctr            => data_err_ctr             -- OUT
    );


    ---------------------------------------------------------------------------
    -- CAN CRC
    ---------------------------------------------------------------------------
    can_crc_inst : can_crc
    generic map(
        G_RESET_POLARITY    => G_RESET_POLARITY,
        G_CRC15_POL         => G_CRC15_POL,
        G_CRC17_POL         => G_CRC17_POL,
        G_CRC21_POL         => G_CRC21_POL
    )
    port map(
        clk_sys          => clk_sys,                    -- IN
        res_n            => res_n,                      -- IN

        -- Memory registers interface
        drv_bus          => drv_bus,                    -- IN

        -- Data inputs for CRC calculation
        data_tx_wbs      => crc_data_tx_wbs,            -- IN
        data_tx_nbs      => crc_data_tx_nbs,            -- IN
        data_rx_wbs      => crc_data_rx_wbs,            -- IN
        data_rx_nbs      => crc_data_rx_nbs,            -- IN

        -- Trigger signals to process the data on each CRC input.
        trig_tx_wbs      => crc_trig_tx_wbs,            -- IN
        trig_tx_nbs      => crc_trig_tx_nbs,            -- IN
        trig_rx_wbs      => crc_trig_rx_wbs,            -- IN
        trig_rx_nbs      => crc_trig_rx_nbs,            -- IN

        -- Control signals
        crc_enable       => crc_enable,                 -- IN
        crc_spec_enable  => crc_spec_enable,            -- IN
        is_receiver      => is_receiver,                -- IN

        -- CRC Outputs
        crc_15           => crc_15,                     -- OUT
        crc_17           => crc_17,                     -- OUT
        crc_21           => crc_21                      -- OUT
    );


    ---------------------------------------------------------------------------
    -- Bit Stuffing
    ---------------------------------------------------------------------------
    bit_stuffing_inst : bit_stuffing
    generic map(
        G_RESET_POLARITY    => G_RESET_POLARITY
    )
    port map(
        clk_sys             => clk_sys,                 -- IN
        res_n               => res_n,                   -- IN

        -- Data-path
        data_in             => bst_data_in,             -- IN
        data_out            => bst_data_out,            -- OUT
        
        -- Control signals
        tx_trigger          => bst_trigger,             -- IN
        stuff_enable        => stuff_enable,            -- IN
        fixed_stuff         => fixed_stuff,             -- IN
        stuff_length        => stuff_length,            -- IN

        -- Status signals
        bst_ctr             => bst_ctr,                 -- OUT
        data_halt           => data_halt                -- OUT
    );


    ---------------------------------------------------------------------------
    -- Bit Destuffing
    ---------------------------------------------------------------------------
    bit_destuffing_inst : bit_destuffing
    generic map(
        G_RESET_POLARITY    => G_RESET_POLARITY
    )
    port map(
        clk_sys             => clk_sys,                 -- IN
        res_n               => res_n,                   -- IN

        -- Data-path
        data_in             => bds_data_in,             -- IN
        data_out            => bds_data_out,            -- OUT

        -- Control signals
        rx_trig             => bds_trigger,             -- IN
        destuff_enable      => destuff_enable,          -- IN
        stuff_error_enable  => stuff_error_enable,      -- IN
        fixed_stuff         => fixed_stuff,             -- IN
        destuff_length      => stuff_length,            -- IN
       
        -- Status Outpus
        stuff_error         => stuff_error,             -- OUT
        destuffed           => destuffed,               -- OUT
        dst_ctr             => dst_ctr                  -- OUT
    );


    ---------------------------------------------------------------------------
    -- Bus traffic counters
    ---------------------------------------------------------------------------
    bus_traffic_counters_inst : bus_traffic_counters
    generic map(
        G_RESET_POLARITY    => G_RESET_POLARITY
    )
    port map(
        clk_sys             => clk_sys,                 -- IN
        res_n               => res_n,                   -- IN

        -- Control signals
        clear_rx_ctr        => drv_clr_rx_ctr,          -- IN
        clear_tx_ctr        => drv_clr_tx_ctr,          -- IN
        inc_tx_ctr          => tran_valid_i,            -- IN
        inc_rx_ctr          => rec_valid_i,             -- IN

        -- Counter outputs
        tx_ctr              => tx_ctr,                  -- OUT
        rx_ctr              => rx_ctr                   -- OUT
    );
                         
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Trigger signals    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    ---------------------------------------------------------------------------
    -- Protocol control triggers:
    --  1. TX Trigger which shifts TX Shift register is enabled when
    --     stuff bit is not inserted!
    --  2. RX Trigger which shifts RX Shift register is enabled when
    --     stuff bit is not destuffed!
    ---------------------------------------------------------------------------
    pc_tx_trigger <= '1' when (tx_trigger = '1' and data_halt = '0')
                         else
                     '0';
     
    pc_rx_trigger <= '1' when (rx_triggers(0) = '1' and destuffed = '0')
                         else
                     '0';
                     
    ---------------------------------------------------------------------------
    -- Bit stuffing/destuffing triggers:
    --  1. Bit Stuffing - TX Trigger, stuff bit does not make any change here
    --     since also stuff bit must be processed by Bit Stuffing.
    --  2. Bit Destuffing - RX Trigger, one clock cycle in advance of TX
    --     Trigger for protocol control, since Bit stuffing is pipelined!
    --     Destuffed bits shall not block bit destuffing since these must also
    --     be processed by Bit destuffing.
    ---------------------------------------------------------------------------
    bst_trigger <= tx_trigger;    
    bds_trigger <= rx_triggers(1);
    
    ---------------------------------------------------------------------------
    -- CRC Triggers for CRC 15 (no bit stuffing):
    --  1. CRC RX NBS - Trigger for CRC15 from RX data without bit stuffing.
    --     The same trigger as for Protocol control reception in sample point.
    --     Trigger must be gated when bit was destuffed, because CRC15 for 
    --     CAN 2.0 frames shall not take stuff bits into account!
    --  2. CRC TX NBS - Trigger for CRC15 from TX data without bit stuffing.
    --     The same trigger as TX Trigger (inserts stuff bit). Must be gated
    --     when stuff bit is inserted!
    ---------------------------------------------------------------------------
    crc_trig_rx_nbs <= '1' when (rx_triggers(0) = '1' and destuffed = '0')
                           else
                       '0';

    crc_trig_tx_nbs <= '1' when (tx_trigger = '1' and data_halt = '0')
                           else
                       '0';

    ---------------------------------------------------------------------------
    -- CRC Trigger for CRC 17, 21 (with bit stuffing):
    --  1. CRC TX WBS - Trigger for CRC17, CRC21 from TX Data with bit stuffing.
    --     Trigger one clock cycle delayed from TX Trigger. Note that this
    --     trigger may be delayed since resynchronisation will never shorten
    --     phase 1 (between TX and RX triggers). This trigger must be gated
    --     for fixed stuff bits!!
    --  2. CRC RX WBS Trigger is the same trigger as the one used to process
    --     data by bit destuffing (one clock cycle in advance of Protocol 
    --     control sampling)! Fixed stuff bits must be left out!
    ---------------------------------------------------------------------------
    crc_trig_tx_wbs_reg : dff_arst
    generic map(
        reset_polarity     => G_RESET_POLARITY,
        rst_val            => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => tx_trigger,
        load               => '1',
        output             => tx_trigger_q
    );

    crc_trig_tx_wbs <= '0' when (fixed_stuff = '1' and data_halt = '1') else
                       '1' when (tx_trigger_q = '1') else
                       '0';

    ---------------------------------------------------------------------------
    -- We must gate fixed stuff bit for CRC from RX With Bit Stuffing. But we
    -- don't know if it is stuff bit, because this should be calculated at the
    -- same clock cycle as bit destuffing! So we must belay the information
    -- here! We sample the data (Bit Destuffing input) to avoid possible change,
    -- and calculate the CRC with rx_trigger(0) (the same trigger as sample
    -- point).
    ---------------------------------------------------------------------------
    crc_data_rx_wbs_reg : dff_arst
    generic map(
        reset_polarity     => G_RESET_POLARITY,
        rst_val            => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => bds_data_in,
        load               => rx_triggers(1),
        output             => crc_data_rx_wbs
    );
    
    crc_trig_rx_wbs <= '1' when (rx_triggers(0) = '1' and destuffed = '0') else
                       '0';
  
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Datapath connection    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
  
    ---------------------------------------------------------------------------
    -- Protocol control datapath connection:
    --  1. RX Data - Output of bit destuffing.
    --  2. TX Data - Input to bit stuffing.
    ---------------------------------------------------------------------------
    pc_rx_data_nbs <= bds_data_out;
    bst_data_in <= pc_tx_data_nbs;

    ---------------------------------------------------------------------------
    -- CRC 15 (No bit stuffing) data inputs:
    --  1. TX Data from Protocol control.
    --  2. RX Data after bit destuffing.
    ---------------------------------------------------------------------------
    crc_data_tx_nbs <= pc_tx_data_nbs;
    crc_data_rx_nbs <= bds_data_out;

    ---------------------------------------------------------------------------
    -- CRC 17,21 (With bit stuffing) data inputs:
    --  1. TX Data after Bit stuffing.
    --  2. RX Data before Bit destuffing.
    ---------------------------------------------------------------------------
    crc_data_tx_wbs <= bst_data_out;
    crc_data_rx_wbs <= bds_data_in;
    
    lpb_dominant <= rx_data_wbs and bst_data_out;
    
    ---------------------------------------------------------------------------
    -- Bit Stuffing data input:
    --  1. Bit Destuffing output for secondary sampling. This-way core will
    --     automatically receive what it transmitts without loop over
    --     Transceiver. Bit Error is detected by Bus sampling properly.
    --  2. Looped back dominant Bit for Bus monitoring Mode.
    --  3. Regular RX Data
    ---------------------------------------------------------------------------
    bds_data_in <= bst_data_out when (sp_control = SECONDARY_SAMPLE) else
                   lpb_dominant when (drv_bus_mon_ena = '1') else
                    rx_data_wbs;

    ---------------------------------------------------------------------------
    -- In Bus monitoring mode, transmitted data to the bus are only recessive.
    -- Otherwise transmitted data are stuffed data!
    ---------------------------------------------------------------------------
    tx_data_wbs_i <= RECESSIVE when (drv_bus_mon_ena = '1') else
                     bst_data_out;


    ----------------------------------------------------------------------------
    -- STATUS Bus Implementation
    ----------------------------------------------------------------------------
    stat_bus(511 downto 383) <= (OTHERS => '0');
    stat_bus(299 downto 297) <= (OTHERS => '0');
    stat_bus(187 downto 186) <= (OTHERS => '0');
    stat_bus(98 downto 90)   <= (OTHERS => '0');
    stat_bus(60 downto 32)   <= (OTHERS => '0');
    stat_bus(113)            <= '0';
    stat_bus(115)            <= '0';
    stat_bus(183)            <= '0';
    stat_bus(120 downto 118) <= (OTHERS => '0');
    stat_bus(178 downto 158) <= (OTHERS => '0');
    stat_bus(5 downto 2)     <= (OTHERS => '0');

    stat_bus(STAT_BR_SHIFTED) <=
        br_shifted_i;

    stat_bus(STAT_ERC_ERR_TYPE_HIGH downto STAT_ERC_ERR_POS_LOW) <=
        erc_capture;

    stat_bus(STAT_IS_TRANSMITTER_INDEX) <=
        is_transmitter;

    stat_bus(STAT_IS_RECEIVER_INDEX) <=
        is_receiver;
    
    stat_bus(STAT_PC_IS_ARBITRATION_INDEX) <=
        is_arbitration;
               
    stat_bus(STAT_PC_IS_CONTROL_INDEX) <=
        is_control;
    
    stat_bus(STAT_PC_IS_DATA_INDEX) <=
        is_data;
    
    stat_bus(STAT_PC_IS_STUFF_COUNT_INDEX) <=
        is_stuff_count;
        
    stat_bus(STAT_PC_IS_CRC_INDEX) <=
        is_crc;
    
    stat_bus(STAT_PC_IS_CRC_DELIM_INDEX) <=
        is_crc_delim;
    
    stat_bus(STAT_PC_IS_ACK_FIELD_INDEX) <=
        is_ack_field;
        
    stat_bus(STAT_PC_IS_ACK_DELIM_INDEX) <=
        is_ack_delim;
    
    stat_bus(STAT_PC_IS_EOF_INDEX) <=
        is_eof;
    
    stat_bus(STAT_PC_IS_INTERMISSION_INDEX) <=
        is_intermission;
    
    stat_bus(STAT_PC_IS_SUSPEND_INDEX) <=
        is_suspend;
        
    stat_bus(STAT_PC_IS_ERROR_INDEX) <=
        is_error;
    
    stat_bus(STAT_PC_IS_OVERLOAD_INDEX) <=
        is_overload;
    
    stat_bus(STAT_ARB_LOST_INDEX) <=
        arbitration_lost_i;
        
    stat_bus(STAT_SET_TRANSC_INDEX) <= 
        set_transmitter;
        
    stat_bus(STAT_SET_REC_INDEX) <=
        set_receiver;
        
    stat_bus(STAT_IS_IDLE_INDEX) <=
        is_idle;

    stat_bus(STAT_SP_CONTROL_HIGH downto STAT_SP_CONTROL_LOW) <=
        sp_control_i;

    stat_bus(STAT_SSP_RESET_INDEX) <=
        ssp_reset_i;

    stat_bus(STAT_TRV_DELAY_CALIB_INDEX) <=
        trv_delay_calib_i;

    stat_bus(STAT_SYNC_CONTROL_HIGH downto STAT_SYNC_CONTROL_LOW) <= 
        sync_control_i;

    stat_bus(STAT_DATA_TX_INDEX) <=
        tx_data_wbs_i;
        
    stat_bus(STAT_DATA_RX_INDEX) <=
        rx_data_wbs;

    stat_bus(STAT_BS_ENABLE_INDEX) <=
        stuff_enable;
        
    stat_bus(STAT_FIXED_STUFF_INDEX) <=
        fixed_stuff;
        
    stat_bus(STAT_DATA_HALT_INDEX) <=
        data_halt;
        
    stat_bus(STAT_BS_LENGTH_HIGH downto STAT_BS_LENGTH_LOW) <=
        stuff_length;

    stat_bus(STAT_STUFF_ERROR_INDEX) <=
        stuff_error;
        
    stat_bus(STAT_DESTUFFED_INDEX) <=
        destuffed;
        
    stat_bus(STAT_BDS_ENA_INDEX) <=
        destuff_enable;

    stat_bus(STAT_STUFF_ERRROR_ENA_INDEX) <= 
        stuff_error_enable;

    stat_bus(STAT_FIXED_DESTUFF_INDEX) <=
        fixed_stuff;
        
    stat_bus(STAT_BDS_LENGTH_HIGH downto STAT_BDS_LENGTH_LOW) <=
        stuff_length;
 
    stat_bus(STAT_TRAN_DLC_HIGH downto STAT_TRAN_DLC_LOW) <=
        tran_dlc;

    stat_bus(STAT_TRAN_IS_RTR_INDEX) <=
        tran_is_rtr;

    stat_bus(STAT_TRAN_IDENT_TYPE_INDEX) <=
        tran_ident_type;

    stat_bus(STAT_TRAN_FRAME_TYPE_INDEX) <=
        tran_frame_type;

    stat_bus(STAT_TRAN_DATA_ACK_INDEX) <=
        txtb_hw_cmd_i.lock;

    stat_bus(STAT_TRAN_BRS_INDEX) <=
        tran_brs;

    stat_bus(STAT_FRAME_STORE_INDEX) <=
        txtb_hw_cmd_i.lock;

    -- Error counters and state
    stat_bus(STAT_TX_COUNTER_HIGH downto STAT_TX_COUNTER_LOW) <=
        tx_err_ctr;

    stat_bus(STAT_RX_COUNTER_HIGH downto STAT_RX_COUNTER_LOW) <=
        rx_err_ctr;

    stat_bus(STAT_ERROR_COUNTER_NORM_HIGH downto STAT_ERROR_COUNTER_NORM_LOW) <=
        norm_err_ctr;

    stat_bus(STAT_ERROR_COUNTER_FD_HIGH downto STAT_ERROR_COUNTER_FD_LOW) <=
        data_err_ctr;

    stat_bus(STAT_IS_ERR_ACTIVE_INDEX) <=
        is_err_active;
    
    stat_bus(STAT_IS_ERR_PASSIVE_INDEX) <=
        is_err_passive;
    
    stat_bus(STAT_IS_BUS_OFF_INDEX) <=
        is_bus_off_i;
   
    stat_bus(STAT_FORM_ERROR_INDEX) <=
        form_error;
    
    stat_bus(STAT_CRC_ERROR_INDEX) <=
        crc_Error;
        
    stat_bus(STAT_ACK_ERROR_INDEX) <=
        ack_Error;
        
    stat_bus(STAT_BIT_STUFF_ERROR_INDEX) <=                 
        bit_error or stuff_error;

    stat_bus(STAT_REC_VALID_INDEX) <=
        rec_valid_i;
        
    stat_bus(STAT_TRAN_VALID_INDEX) <=
        tran_valid_i;
       
    -- Recieved data interface
    stat_bus(STAT_REC_IDENT_HIGH downto STAT_REC_IDENT_LOW) <=
        rec_ident_i;
        
    stat_bus(STAT_REC_DLC_HIGH downto STAT_REC_DLC_LOW) <=
        rec_dlc_i;
        
    stat_bus(STAT_REC_IS_RTR_INDEX) <=
        rec_is_rtr_i;
        
    stat_bus(STAT_REC_IDENT_TYPE_INDEX) <=
        rec_ident_type_i;
        
    stat_bus(STAT_REC_FRAME_TYPE_INDEX) <=
        rec_frame_type_i;
        
    stat_bus(STAT_REC_BRS_INDEX) <=
        rec_brs_i;
        
    stat_bus(STAT_REC_ESI_INDEX) <=
        rec_esi_i;

    stat_bus(STAT_CRC_ENA_INDEX) <=
        crc_enable;

    stat_bus(STAT_TRAN_TRIG) <=
        pc_tx_trigger;
        
    stat_bus(STAT_REC_TRIG) <=
        pc_rx_trigger;
    
    stat_bus(STAT_SAMPLE_INDEX) <=
        rx_triggers(0);

    stat_bus(STAT_SAMPLE_SEC) <=
        sample_sec;
    
    stat_bus(STAT_ALC_ID_FIELD_HIGH downto STAT_ALC_BIT_LOW) <=
        alc;

    stat_bus(STAT_RX_CTR_HIGH downto STAT_RX_CTR_LOW) <=
        rx_ctr;
        
    stat_bus(STAT_TX_CTR_HIGH downto STAT_TX_CTR_LOW) <=
        tx_ctr;

    stat_bus(STAT_ERP_CHANGED_INDEX) <=
        error_passive_changed_i;
        
    stat_bus(STAT_EWL_REACHED_INDEX) <=
        error_warning_limit_i;

    stat_bus(STAT_ERROR_VALID_INDEX) <=
        err_detected;
 
    stat_bus(STAT_ACK_RECIEVED_OUT_INDEX) <=
        ack_received_i;
        
    stat_bus(STAT_BIT_ERROR_VALID_INDEX) <=
        bit_error;

    stat_bus(STAT_BS_CTR_HIGH downto STAT_BS_CTR_LOW) <=
        std_logic_vector(to_unsigned(bst_ctr, bst_ctr'length));
          
    stat_bus(STAT_BD_CTR_HIGH downto STAT_BD_CTR_LOW) <=
        std_logic_vector(to_unsigned(dst_ctr, bst_ctr'length)); 

    stat_bus(STAT_TS_HIGH downto STAT_TS_LOW) <=
        timestamp;

    ---------------------------------------------------------------------------
    -- Internal signals to output propagation
    ---------------------------------------------------------------------------
    txtb_hw_cmd <= txtb_hw_cmd_i;
    rec_ident <= rec_ident_i; 
    rec_dlc <= rec_dlc_i;
    rec_ident_type <= rec_ident_type_i;
    rec_frame_type <= rec_frame_type_i;
    rec_is_rtr <= rec_is_rtr_i;
    rec_brs <= rec_brs_i;
    rec_esi <= rec_esi_i;
    rec_valid <= rec_valid_i;
    arbitration_lost <= arbitration_lost_i;
    tran_valid <= tran_valid_i;
    br_shifted <= br_shifted_i;
    err_detected <= err_detected_i;
    error_passive_changed <= error_passive_changed_i;
    error_warning_limit <= error_warning_limit_i;
    sync_control <= sync_control_i;
    tx_data_wbs <= tx_data_wbs_i;
    sp_control <= sp_control_i;
    ssp_reset <= ssp_reset_i;
    trv_delay_calib <= trv_delay_calib_i;
    is_bus_off <= is_bus_off_i;
 
end architecture;