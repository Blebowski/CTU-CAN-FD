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
-- Package:
--  CAN components.
--
-- Purpose:
--  Package for component declarations to avoid writing component declarations
--  every time into architecture itself.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.can_registers_pkg.all;

package can_components_pkg is

    component can_top_level is
        generic(
            -- RX Buffer RAM size (32 bit words)
            rx_buffer_size : natural range 32 to 4096 := 128;
    
            -- Number of supported TXT buffers
            txt_buffer_count    : natural range 2 to 8   := C_TXT_BUFFER_COUNT; 
        
            -- Synthesize Filter A
            sup_filtA      : boolean                := true;
            
            -- Synthesize Filter B
            sup_filtB      : boolean                := true;
            
            -- Synthesize Filter C
            sup_filtC      : boolean                := true;
            
            -- Synthesize Range Filter
            sup_range      : boolean                := true;
            
            -- Synthesize Test registers
            sup_test_registers  : boolean           := true;
            
            -- Support traffic counters
            sup_traffic_ctrs : boolean              := true;
            
            -- Target technology (ASIC or FPGA)
            target_technology   : natural           := C_TECH_ASIC
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
            -- Debug signals for testbenches
            -----------------------------------------------------------------------
            test_probe  : out t_ctu_can_fd_test_probe;
    
            -----------------------------------------------------------------------
            -- Timestamp for time based transmission / reception
            -----------------------------------------------------------------------
            timestamp    : in std_logic_vector(63 downto 0)
        );
    end component can_top_level;
    
    component ahb_ifc is
    port (
        -----------------------------------------------------------------------
        -- CTU CAN FD Interface
        -----------------------------------------------------------------------
        data_in          : out std_logic_vector(31 downto 0);
        data_out         : in  std_logic_vector(31 downto 0);
        adress           : out std_logic_vector(15 downto 0);
        sbe              : out std_logic_vector(3 downto 0);
        scs              : out std_logic;
        swr              : out std_logic;
        srd              : out std_logic;

        -----------------------------------------------------------------------
        -- AHB interface 
        -----------------------------------------------------------------------
        hresetn          : in std_logic;
        hclk             : in std_logic;
        haddr            : in std_logic_vector(31 downto 0);
        hwdata           : in std_logic_vector(31 downto 0);
        hsel             : in std_logic;
        hwrite           : in std_logic;
        hsize            : in std_logic_vector(2 downto 0);
        hburst           : in std_logic_vector(2 downto 0);
        hprot            : in std_logic_vector(3 downto 0);
        htrans           : in std_logic_vector(1 downto 0);
        hmastlock        : in std_logic;
        hready           : in std_logic;
        hreadyout        : out std_logic;
        hresp            : out std_logic;
        hrdata           : out std_logic_vector(31 downto 0)
    );
    end component;
   
    component bus_sampling is 
        generic(        
            -- Secondary sampling point Shift registers length
            G_SSP_DELAY_SAT_VAL     :     natural := 255;
    
            -- Depth of FIFO Cache for TX Data
            G_TX_CACHE_DEPTH        :     natural := 8;
            
            -- Width (number of bits) in transceiver delay measurement counter
            G_TRV_CTR_WIDTH         :     natural := 7;
            
            -- Width of SSP position
            G_SSP_POS_WIDTH          :     natural := 8;
            
            -- Optional usage of saturated value of ssp_delay 
            G_USE_SSP_SATURATION    :     boolean := true;
            
            -- Width of SSP generator counters (BTMC, SSPC)
            G_SSP_CTRS_WIDTH        :     natural := 14
        );  
        port(
            ------------------------------------------------------------------------
            -- Clock and Async reset
            ------------------------------------------------------------------------
            -- System clock
            clk_sys              :in   std_logic;
            
            -- Asynchronous reset
            res_n                :in   std_logic;

            ------------------------------------------------------------------------
            -- DFT support
            ------------------------------------------------------------------------
            scan_enable          :in   std_logic;
            
            ------------------------------------------------------------------------
            --  Physical layer interface
            ------------------------------------------------------------------------
            -- CAN serial stream output
            can_rx               :in   std_logic;
            
            -- CAN serial stream input
            can_tx               :out  std_logic;
    
            ------------------------------------------------------------------------
            -- Memorz registers interface
            ------------------------------------------------------------------------
            -- Driving bus
            drv_bus              :in   std_logic_vector(1023 downto 0);
            
            -- Measured Transceiver delay 
            trv_delay            :out  std_logic_vector(G_TRV_CTR_WIDTH - 1 downto 0);
              
            ------------------------------------------------------------------------
            -- Prescaler interface
            ------------------------------------------------------------------------
            -- RX Trigger
            rx_trigger           :in   std_logic;
            
            -- TX Trigger
            tx_trigger           :in   std_logic;
            
            -- Valid synchronisation edge appeared (Recessive to Dominant)
            sync_edge            :out  std_logic;
            
            -- Time quanta edge
            tq_edge              :in   std_logic;
    
            ------------------------------------------------------------------------
            -- CAN Core Interface
            ------------------------------------------------------------------------
            -- TX data
            tx_data_wbs          :in   std_logic;
    
            -- RX data
            rx_data_wbs          :out  std_logic;
    
            -- Sample control
            sp_control           :in   std_logic_vector(1 downto 0);
                
            -- Reset for Secondary Sampling point Shift register.
            ssp_reset            :in   std_logic;
    
            -- Enable measurement of transmitter delay
            tran_delay_meas      :in   std_logic; 
    
            -- Secondary sampling RX trigger
            sample_sec           :out  std_logic;
    
            -- Bit error detected
            bit_err              :out  std_logic;
            
            -- Reset Bit time measurement counter
            btmc_reset           :in   std_logic;
        
            -- Start Measurement of data bit time (in TX Trigger)
            dbt_measure_start    :in   std_logic;
        
            -- First SSP generated (in ESI bit)
            gen_first_ssp        :in   std_logic
        );
    end component bus_sampling;
   
    component bit_err_detector is
        port(
            ------------------------------------------------------------------------
            -- Clock and Async reset
            ------------------------------------------------------------------------
            -- System clock
            clk_sys                  :in   std_logic;
            
            -- Asynchronous reset
            res_n                    :in   std_logic;
            
            ------------------------------------------------------------------------
            -- Control signals
            ------------------------------------------------------------------------
            -- CTU CAN FD Core is enabled
            drv_ena                  :in   std_logic;
            
            -- Sample control
            sp_control               :in   std_logic_vector(1 downto 0);
            
            -- RX Trigger
            rx_trigger               :in   std_logic;
            
            -- RX Trigger - Secondary Sample
            sample_sec               :in   std_logic;
    
            -----------------------------------------------------------------------
            -- TX / RX Datapath
            -----------------------------------------------------------------------
            -- Actually transmitted data on CAN bus
            data_tx                  :in   std_logic;
            
            -- Delayed transmitted data (for detection in secondary sampling point)
            data_tx_delayed          :in   std_logic;
            
            -- RX Data (Synchronised)
            data_rx_synced           :in   std_logic;
    
            -----------------------------------------------------------------------
            -- Status outputs
            -----------------------------------------------------------------------
            -- Bit error detected
            bit_err                  : out std_logic
        );
    end component;
   
   
    component data_edge_detector is
        port(
            ------------------------------------------------------------------------
            -- Clock and Asynchronous reset
            ------------------------------------------------------------------------
            -- System clock
            clk_sys                  :in   std_logic;
            
            -- Asynchronous Reset
            res_n                    :in   std_logic;
    
            ------------------------------------------------------------------------
            -- Inputs
            ------------------------------------------------------------------------
            -- TX Data from CAN Core
            tx_data                  :in   std_logic;
            
            -- RX Data (from CAN Bus)
            rx_data                  :in   std_logic;
            
            -- RX Data value from previous Sample point.
            prev_rx_sample           :in   std_logic;
            
            -- Time quanta edge
            tq_edge                  :in   std_logic;
            
            ------------------------------------------------------------------------
            -- Outputs
            ------------------------------------------------------------------------
            -- Edge detected on TX Data
            tx_edge                  :out  std_logic;
    
            -- Edge detected on RX Data                                             
            rx_edge                  :out  std_logic;
            
            -- Synchronisation edge
            sync_edge                :out  std_logic
        );
    end component data_edge_detector;
   
   
    component sample_mux is
    port(
        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in   std_logic;
        
        -- Asynchronous reset
        res_n                :in   std_logic;       
        
        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- CTU CAN FD enabled
        drv_ena              :in   std_logic;
        
        -- Sample control (nominal, data, secondary)
        sp_control           :in   std_logic_vector(1 downto 0);
        
        -- RX Trigger
        rx_trigger           :in   std_logic;
        
        -- RX Trigger - Secondary Sampling
        sample_sec           :in   std_logic;

        -----------------------------------------------------------------------
        -- Datapath
        -----------------------------------------------------------------------
        -- RX Data (Synchronised)
        data_rx_synced       :in   std_logic;

        -- Sampled value of RX pin in Sample point (DFF output)
        prev_sample          :out  std_logic        
    );
    end component sample_mux;
   
  
    component ssp_generator is
    generic(
        -- Width of SSP generator counters (BTMC, SSPC)
        G_SSP_CTRS_WIDTH     :      natural := 14
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in   std_logic;

        -- Asynchronous reset
        res_n                :in   std_logic;       

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Reset Bit time measurement counter
        btmc_reset          :in    std_logic;

        -- Start Measurement of data bit time (in TX Trigger)
        dbt_measure_start   :in    std_logic;

        -- First SSP generated (in ESI bit)
        gen_first_ssp       :in    std_logic;

        -- SSP offset
        ssp_delay           :in    std_logic_vector(7 downto 0);

        -- SSP enable (SSP trigger gated when disabled)
        ssp_enable          :in    std_logic;

        -----------------------------------------------------------------------
        -- Trigger signals
        -----------------------------------------------------------------------
        -- TX Trigger
        tx_trigger          :in    std_logic;
        
        -- RX Trigger
        sample_sec          :out   std_logic
    );
    end component;
   
    component trv_delay_measurement is
    generic(
        -- Width (number of bits) in transceiver delay measurement counter
        G_TRV_CTR_WIDTH          :     natural := 7;

        -- Width of SSP position
        G_SSP_POS_WIDTH          :     natural := 8;

        -- Optional usage of saturated value of ssp_delay 
        G_USE_SSP_SATURATION     :     boolean := true;
        
        -- Saturation level for size of SSP_delay. This is to make sure that
        -- if there is smaller shift register for secondary sampling point we
        -- don't address outside of this register.
        G_SSP_SATURATION_LVL     :     natural
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys             :in   std_logic;
        
        -- Asynchronous reset        
        res_n               :in   std_logic;

        ------------------------------------------------------------------------
        -- DFT support
        ------------------------------------------------------------------------
        scan_enable         :in   std_logic;

        ------------------------------------------------------------------------
        -- Transceiver Delay measurement control
        ------------------------------------------------------------------------
        -- Start measurement (on TX Edge)
        edge_tx_valid       :in   std_logic;
        
        -- Stop measurement (on RX Edge)
        edge_rx_valid       :in   std_logic;
        
        -- Enable measurement of transmitter delay
        tran_delay_meas      :in   std_logic; 

        ------------------------------------------------------------------------
        -- Memory registers interface
        ------------------------------------------------------------------------
        -- Secondary sampling point offset
        ssp_offset          :in   std_logic_vector(G_SSP_POS_WIDTH - 1 downto 0);

        -- Source of secondary sampling point 
        -- (Measured, Offset, Measured and Offset)
        ssp_delay_select    :in   std_logic_vector(1 downto 0);

        ------------------------------------------------------------------------
        -- Status outputs
        ------------------------------------------------------------------------
        -- Shadowed value of Transceiver delay. Updated when measurement ends.
        trv_delay_shadowed  :out  std_logic_vector(G_TRV_CTR_WIDTH - 1 downto 0);
                                               
        -- Shadowed value of SSP configuration. Updated when measurement ends.
        ssp_delay_shadowed  :out  std_logic_vector(G_SSP_POS_WIDTH - 1 downto 0)
    );
    end component;
   
    component tx_data_cache is
    generic(
        -- Depth of FIFO (Number of bits that can be stored)
        G_TX_CACHE_DEPTH        :     natural range 4 to 32 := 8;
        
        -- FIFO reset value
        G_TX_CACHE_RST_VAL      :     std_logic := '0'
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys         :in   std_logic;
        
        -- Asynchronous reset
        res_n           :in   std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Store input data
        write           :in   std_logic;
        
        -- Read output data
        read            :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Data signals
        ------------------------------------------------------------------------
        -- Data inputs
        data_in         :in   std_logic;
        
        -- Data output
        data_out        :out  std_logic
    );
    end component tx_data_cache;
    
    
    component bit_destuffing is
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              : in std_logic;
        
        -- Asynchronous reset
        res_n                : in std_logic;

        ------------------------------------------------------------------------
        -- Data-path
        ------------------------------------------------------------------------
        -- Data input (from Bus Sampling)
        data_in              : in std_logic;
        
        -- Data output (to Protocol Control)
        data_out             : out std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- RX Trigger (in Sample point, from Prescaler).
        bds_trigger          : in std_logic;

        -- Bit Destuffing is enabled.
        destuff_enable       : in  std_logic;

        -- Bit destuffing type (0-Normal, 1-Fixed)    
        fixed_stuff          : in  std_logic;  

        -- Length of Bit De-Stuffing rule
        destuff_length       : in  std_logic_vector(2 downto 0);  
       
        ------------------------------------------------------------------------
        -- Status Outpus
        ------------------------------------------------------------------------
        -- Stuff error detected (more equal consecutive bits than length of
        -- stuff rule.
        stuff_err            : out std_logic;
        
        -- Data output is not valid, actual bit is stuff bit.
        destuffed            : out std_logic;
        
        -- Number of de-stuffed bits with normal bit stuffing method
        dst_ctr              : out std_logic_vector(2 downto 0)
    );
    end component bit_destuffing;
   
   
    component bit_stuffing is
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys             :in   std_logic;
        
        -- Asynchronous reset
        res_n               :in   std_logic;

        ------------------------------------------------------------------------
        -- Data-path
        ------------------------------------------------------------------------
        -- Data Input (from Protocol Control)
        data_in             :in   std_logic;
        
        -- Data Output (to CAN Bus)
        data_out            :out  std_logic;
        
        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- TX Trigger for Bit Stuffing (in SYNC segment) 
        bst_trigger         :in   std_logic; 
        
        -- Bit Stuffing enabled. If not, data are only passed to the output
        stuff_enable        :in   std_logic;

        -- Bit Stuffing type (0-Normal, 1-Fixed)
        fixed_stuff         :in   std_logic;    

        -- Length of Bit Stuffing rule
        stuff_length        :in   std_logic_vector(2 downto 0); 
        
        -- Frame transmission without SOF started
        tx_frame_no_sof     :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Status signals
        ------------------------------------------------------------------------
        -- Number of stuffed bits with Normal Bit stuffing
        bst_ctr             :out  std_logic_vector(2 downto 0); 
        
        -- Stuff bit is inserted, Protocol control operation to be halted for
        -- one bit time
        data_halt           :out  std_logic
    );
    end component bit_stuffing;


    component bus_traffic_counters is
    port(
        ------------------------------------------------------------------------
        -- System clock and Asynchronous Reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous Reset
        res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- DFT support
        ------------------------------------------------------------------------
        scan_enable            :in   std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Clear RX Traffic counter (Glitch free)
        clear_rx_ctr           :in   std_logic;
        
        -- Clear TX Traffic counter (Glitch free)
        clear_tx_ctr           :in   std_logic;

        -- Increment TX Traffic Counter
        inc_tx_ctr             :in   std_logic;
        
        -- Increment RX Traffic Counter
        inc_rx_ctr             :in   std_logic;

        ------------------------------------------------------------------------
        -- Counter outputs
        ------------------------------------------------------------------------
        -- TX Traffic counter
        tx_ctr                 :out  std_logic_vector(31 downto 0);
        
        -- RX Traffic counter
        rx_ctr                 :out  std_logic_vector(31 downto 0)
    );
    end component bus_traffic_counters;


    component can_crc is
    generic(
        -- CRC 15 polynomial
        G_CRC15_POL         :     std_logic_vector(15 downto 0) := x"C599";
        
        -- CRC 17 polynomial
        G_CRC17_POL         :     std_logic_vector(19 downto 0) := x"3685B";
        
        -- CRC 15 polynomial
        G_CRC21_POL         :     std_logic_vector(23 downto 0) := x"302899"  
    );
    port(
        ------------------------------------------------------------------------
        -- System clock and Asynchronous Reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys          :in   std_logic;

        -- Asynchronous reset
        res_n            :in   std_logic;

        ------------------------------------------------------------------------
        -- Memory registers interface
        ------------------------------------------------------------------------
        -- Driving bus
        drv_bus          :in   std_logic_vector(1023 downto 0);

        ------------------------------------------------------------------------
        -- Data inputs for CRC calculation
        ------------------------------------------------------------------------
        -- TX Data with Bit Stuffing
        data_tx_wbs      :in   std_logic;
        
        -- TX Data without Bit Stuffing
        data_tx_nbs      :in   std_logic;
        
        -- RX Data with Bit Stuffing
        data_rx_wbs      :in   std_logic;
        
        -- RX Data without Bit Stuffing
        data_rx_nbs      :in   std_logic;

        ------------------------------------------------------------------------
        -- Trigger signals to process the data on each CRC input.
        ------------------------------------------------------------------------
        -- Trigger for TX Data with Bit Stuffing
        trig_tx_wbs      :in   std_logic;
        
        -- Trigger for TX Data without Bit Stuffing
        trig_tx_nbs      :in   std_logic;
        
        -- Trigger for RX Data with Bit Stuffing
        trig_rx_wbs      :in   std_logic;
        
        -- Trigger for RX Data without Bit Stuffing
        trig_rx_nbs      :in   std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Enable for all CRC circuits.
        crc_enable       :in   std_logic;

        -- CRC calculation - speculative enable
        crc_spec_enable  :in   std_logic;

        -- Use RX Data for CRC calculation
        crc_calc_from_rx :in   std_logic;
        
        -- Load CRC Initialization vector
        load_init_vect   :in  std_logic;

        ------------------------------------------------------------------------
        -- CRC Outputs
        ------------------------------------------------------------------------
        -- Calculated CRC 15
        crc_15           :out  std_logic_vector(14 downto 0);

        -- Calculated CRC 17
        crc_17           :out  std_logic_vector(16 downto 0);
        
        -- Calculated CRC 21
        crc_21           :out  std_logic_vector(20 downto 0)
    );
    end component;


    component crc_calc is
    generic(
        -- Width of CRC sequence
        G_CRC_WIDTH         : natural;

        -- CRC Polynomial
        G_POLYNOMIAL        : std_logic_vector
    );
    port(
        ------------------------------------------------------------------------
        -- System clock and Asynchronous Reset
        ------------------------------------------------------------------------
        -- System clock input 
        clk_sys    :in   std_logic;

        -- Asynchronous reset
        res_n      :in   std_logic;

        ------------------------------------------------------------------------
        -- CRC Calculation control
        ------------------------------------------------------------------------
        -- Serial data input for CRC calculation
        data_in    :in   std_logic;

        -- Trigger to sample the input data
        trig       :in   std_logic;
 
        -- CRC calculation enabled
        enable     :in   std_logic; 

        -- Initialization vector for CRC calculation
        init_vect  :in   std_logic_vector(G_CRC_WIDTH - 1 downto 0);

        -- Load CRC Initialization vector
        load_init_vect   :in  std_logic;
        
        ------------------------------------------------------------------------
        -- CRC output
        ------------------------------------------------------------------------
        crc         :out  std_logic_vector(G_CRC_WIDTH - 1 downto 0)
    );    
    end component;


    component err_counters is
    port(
        -----------------------------------------------------------------------
        -- System clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock input 
        clk_sys                :in   std_logic;

        -- Asynchronous reset
        res_n                  :in   std_logic;

        -----------------------------------------------------------------------
        -- DFT support
        -----------------------------------------------------------------------
        scan_enable            :in   std_logic;

        -----------------------------------------------------------------------
        -- Control inputs
        -----------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control             :in   std_logic_vector(1 downto 0);

        -- Increment error counter by 1 
        inc_one                :in   std_logic;
        
        -- Increment error counter by 8
        inc_eight              :in   std_logic;
        
        -- Decrement error counter by 1
        dec_one                :in   std_logic;
        
        -- Set unit to error active (after re-integration). Erases error
        -- counters to 0!
        set_err_active         :in   std_logic;
        
        -- Preload TX Error counter
        tx_err_ctr_pload       :in   std_logic;
        
        -- Preload RX Error counter
        rx_err_ctr_pload       :in   std_logic;

        -- Preload value for Error counters
        drv_ctr_val            :in   std_logic_vector(8 downto 0);
        
        -- Unit is transmitter
        is_transmitter         :in   std_logic;

        -- Unit is receiver
        is_receiver            :in   std_logic;

        -----------------------------------------------------------------------
        -- Counter statuses
        -----------------------------------------------------------------------
        -- RX Error counter
        rx_err_ctr             :out  std_logic_vector(8 downto 0);
        
        -- TX Error counter
        tx_err_ctr             :out  std_logic_vector(8 downto 0);
        
        -- Nominal Bit Rate Error counter
        norm_err_ctr           :out  std_logic_vector(15 downto 0);
        
        -- Nominal Bit Rate Error counter
        data_err_ctr           :out  std_logic_vector(15 downto 0)
    );
    end component err_counters;


    component fault_confinement_fsm is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;
        
        -- Asynchronous reset
        res_n                   :in   std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Error warning limit
        ewl                     :in   std_logic_vector(8 downto 0);
        
        -- Error passive threshold
        erp                     :in   std_logic_vector(8 downto 0);

        -- Set unit to be error active
        set_err_active          :in   std_logic;
        
        -- Unit enabled
        drv_ena                 :in   std_logic;
       
        -----------------------------------------------------------------------
        -- Error counters
        -----------------------------------------------------------------------
        -- TX Error counter
        tx_err_ctr              :in   std_logic_vector(8 downto 0);
        
        -- RX Error counter
        rx_err_ctr              :in   std_logic_vector(8 downto 0);

        -----------------------------------------------------------------------
        -- Fault confinement State indication
        -----------------------------------------------------------------------
        -- Unit is error active
        is_err_active           :out  std_logic;
        
        -- Unit is error passive
        is_err_passive          :out  std_logic;
        
        -- Unit is Bus-off
        is_bus_off              :out  std_logic;

        -----------------------------------------------------------------------
        -- Status outputs
        -----------------------------------------------------------------------
        -- Error passive state changed
        fcs_changed             :out  std_logic;

        -- Error warning limit was reached
        err_warning_limit       :out  std_logic
    );
    end component fault_confinement_fsm;


    component fault_confinement_rules is
    port(
        -----------------------------------------------------------------------
        -- Operation control interface
        ------------------------------------------------------------------------
        -- Unit is transmitter
        is_transmitter          :in   std_logic;

        -- Unit is receiver
        is_receiver             :in   std_logic;

        -----------------------------------------------------------------------
        -- Protocol Control interface
        -----------------------------------------------------------------------
        -- Error is detected
        err_detected            :in   std_logic;
        
        -- Error counter should remain unchanged
        err_ctrs_unchanged      :in   std_logic;
        
        -- Primary Error
        primary_err           :in   std_logic;
        
        -- Active Error Flag or Overload flag is being tranmsmitted
        act_err_ovr_flag        :in   std_logic;
        
        -- Error delimiter too late
        err_delim_late          :in   std_logic;
        
        -- Transmission of frame valid
        tran_valid              :in   std_logic;
        
        -- Decrement receive Error counter
        decrement_rec           :in   std_logic;
        
        -- Bit Error in passive error flag after ACK error
        bit_err_after_ack_err   :in   std_logic;

        -----------------------------------------------------------------------
        -- Control registers interface
        -----------------------------------------------------------------------
        -- ROM mode enabled
        drv_rom_ena             :in   std_logic;

        -----------------------------------------------------------------------
        -- Output signals to error counters
        -----------------------------------------------------------------------
        -- Increment Error counter by 1
        inc_one                 :out  std_logic;

        -- Increment Error counter by 8
        inc_eight               :out  std_logic;
        
        -- Decrement Error counter by 1
        dec_one                 :out  std_logic
    );
    end component fault_confinement_rules;


    component fault_confinement is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;
        
        -- Asynchronous reset
        res_n                   :in   std_logic;

        -----------------------------------------------------------------------
        -- DFT support
        -----------------------------------------------------------------------
        scan_enable             :in   std_logic;

        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- Driving Bus
        drv_bus                 :in   std_logic_vector(1023 downto 0);
          
        -----------------------------------------------------------------------
        -- Error signalling for interrupts
        -----------------------------------------------------------------------
        -- Error passive state changed
        fcs_changed             :out  std_logic;

        -- Error warning limit was reached
        err_warning_limit       :out  std_logic;

        -----------------------------------------------------------------------
        -- Operation control Interface
        -----------------------------------------------------------------------
        -- Unit is transmitter
        is_transmitter          :in   std_logic;
        
        -- Unit is receiver
        is_receiver             :in   std_logic;
        
        -----------------------------------------------------------------------
        -- Protocol control Interface
        -----------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control              :in   std_logic_vector(1 downto 0);

        -- Set unit to error active (after re-integration). Erases eror
        -- counters to 0!
        set_err_active          :in   std_logic;
        
        -- Error is detected
        err_detected            :in   std_logic;
        
        -- Error counter should remain unchanged
        err_ctrs_unchanged      :in   std_logic;
        
        -- Primary Error
        primary_err           :in   std_logic;
        
        -- Active Error Flag or Overload flag is being tranmsmitted
        act_err_ovr_flag        :in   std_logic;
        
        -- Error delimiter too late
        err_delim_late          :in   std_logic;
        
        -- Transmission of frame valid
        tran_valid              :in   std_logic;
        
        -- Reception of frame valid
        rec_valid               :in   std_logic;
        
        -- Decrement receive Error counter
        decrement_rec           :in   std_logic;
        
        -- Bit Error in passive error flag after ACK error
        bit_err_after_ack_err   :in   std_logic;

        -----------------------------------------------------------------------
        -- Fault confinement State indication
        -----------------------------------------------------------------------
        -- Unit is error active
        is_err_active           :out   std_logic;
        
        -- Unit is error passive
        is_err_passive          :out   std_logic;
        
        -- Unit is Bus-off
        is_bus_off              :out   std_logic;

        -----------------------------------------------------------------------
        -- Error counters
        -----------------------------------------------------------------------
        -- TX Error counter
        tx_err_ctr              :out  std_logic_vector(8 downto 0);
        
        -- RX Error counter
        rx_err_ctr              :out  std_logic_vector(8 downto 0);
        
        -- Error counter in Nominal Bit-rate
        norm_err_ctr            :out  std_logic_vector(15 downto 0);
        
        -- Error counter in Data Bit-rate
        data_err_ctr            :out  std_logic_vector(15 downto 0)
    );
    end component;


    component operation_control is
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in   std_logic;
        
        -- Asynchronous reset
        res_n                :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Prescaler Interface
        ------------------------------------------------------------------------
        -- RX Trigger
        rx_trigger           :in   std_logic;

        ------------------------------------------------------------------------
        -- Fault confinement Interface
        ------------------------------------------------------------------------
        -- Unit is Bus-off
        is_bus_off           :in   std_logic;

        ------------------------------------------------------------------------
        -- Protocol Control Interface
        ------------------------------------------------------------------------
        -- Arbitration lost
        arbitration_lost     :in   std_logic;

        -- Set unit to be transmitter (in SOF)
        set_transmitter      :in   std_logic; 

        -- Set unit to be receiver
        set_receiver         :in   std_logic;

        -- Set unit to be idle
        set_idle             :in   std_logic;

        -- Status outputs
        is_transmitter       :out  std_logic;
        
        -- Unit is receiver
        is_receiver          :out  std_logic;
        
        -- Unit is idle
        is_idle              :out  std_logic
    );
    end component;

    component control_counter is
    generic(
        -- Width of control counter
        G_CTRL_CTR_WIDTH        :     natural := 9 
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys         :in   std_logic;

        -- Asynchronous reset
        res_n           :in   std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- RX Trigger (Decrements the counter)
        rx_trigger            :in   std_logic;

        -- Control counter counting is enabled
        ctrl_ctr_ena          :in   std_logic;

        -- Pre-load control counter
        ctrl_ctr_pload        :in   std_logic;
  
        -- Pre-load value for control counter
        ctrl_ctr_pload_val    :in   std_logic_vector(G_CTRL_CTR_WIDTH - 1 downto 0);
        
        -- Complementary counter enable
        compl_ctr_ena         :in   std_logic;

        -- Arbitration lost
        arbitration_lost      :in    std_logic;
        
        -- Arbitration lost
        alc_id_field          :in    std_logic_vector(2 downto 0);

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Control counter is equal to zero
        ctrl_ctr_zero           :out std_logic;

        -- Control counter is equal to one
        ctrl_ctr_one            :out std_logic;

        -- Control counter counted multiple of 8 bits
        ctrl_counted_byte       :out std_logic;
        
        -- Control counter byte index within a memory word
        ctrl_counted_byte_index :out std_logic_vector(1 downto 0);
        
        -- Index of memory word in TXT Buffer
        ctrl_ctr_mem_index      :out std_logic_vector(4 downto 0);
        
        -- Arbitration lost capture
        alc                     :out std_logic_vector(7 downto 0)
    );
    end component control_counter;

    component err_detector is
    generic(
        -- Pipeline should be inserted on Error signalling
        G_ERR_VALID_PIPELINE    :     boolean
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
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual TX Data
        tx_data                 :in   std_logic;
        
        -- Actual RX Data
        rx_data                 :in   std_logic;
        
        -----------------------------------------------------------------------
        -- Error sources
        -----------------------------------------------------------------------
        -- Bit error (from Bus sampling)
        bit_err                 :in   std_logic;
        
        -- Bit error in Arbitration field
        bit_err_arb             :in   std_logic;
        
        -- Stuff error
        stuff_err             :in   std_logic;
        
        -- Form Error
        form_err              :in   std_logic;
        
        -- ACK Error
        ack_err               :in   std_logic;

        -- CRC Error
        crc_err                 :in   std_logic;
        
        -----------------------------------------------------------------------
        -- CRC comparison data
        -----------------------------------------------------------------------
        -- Received CRC
        rx_crc                  :in   std_logic_vector(20 downto 0);
        
        -- Calculated CRC 15
        crc_15                  :in   std_logic_vector(14 downto 0);

        -- Calculated CRC 17
        crc_17                  :in   std_logic_vector(16 downto 0);
        
        -- Calculated CRC 21
        crc_21                  :in   std_logic_vector(20 downto 0);
        
        -- Received Stuff count (Gray coded) + Parity
        rx_stuff_count          :in   std_logic_vector(3 downto 0);
        
        -- Destuff counter mod 8
        dst_ctr                 :in   std_logic_vector(2 downto 0);

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Bit error enable
        bit_err_enable        :in   std_logic;

        -- Fixed Bit stuffing method
        fixed_stuff             :in   std_logic;

        -- Error position field (from Protocol control)
        err_pos                 :in   std_logic_vector(4 downto 0);

        -- Perform CRC Check
        crc_check               :in   std_logic;

        -- Clear CRC match flag
        crc_clear_match_flag    :in   std_logic;

        -- CRC Source (CRC15, CRC17, CRC21)
        crc_src                 :in   std_logic_vector(1 downto 0);

        -- FD Type (ISO FD, NON-ISO FD)
        drv_fd_type             :in   std_logic;

        -- Arbitration field is being transmitted / received
        is_arbitration          :in   std_logic;

        -- Unit is transmitter of frame
        is_transmitter          :in   std_logic;

        -- Unit is error passive
        is_err_passive          :in   std_logic;

        -----------------------------------------------------------------------
        -- Status output
        -----------------------------------------------------------------------
        -- Error frame request
        err_frm_req             :out  std_logic;

        -- Error detected (for Fault confinement)
        err_detected            :out  std_logic;

        -- Error code capture
        erc_capture             :out  std_logic_vector(7 downto 0);

        -- CRC match
        crc_match               :out  std_logic;

        -- Error counters should remain unchanged
        err_ctrs_unchanged      :out  std_logic
    );
    end component;

    component trigger_mux is
    generic(
        -- Number of Sample Triggers
        G_SAMPLE_TRIGGER_COUNT  :    natural := 2
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
        -- Input triggers
        ------------------------------------------------------------------------
        -- RX Triggers
        rx_triggers            :in   std_logic_vector(G_SAMPLE_TRIGGER_COUNT - 1 downto 0);

        -- TX Trigger
        tx_trigger             :in   std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Stuff bit is inserted, Protocol control operation to be halted for
        -- one bit time
        data_halt              :in   std_logic;
        
        -- Data output is not valid, actual bit is stuff bit.
        destuffed              :in   std_logic;
        
        -- Fixed bit stuffing method is used
        fixed_stuff            :in   std_logic;
        
        -- Bit Destuffing Data input
        bds_data_in            :in   std_logic;

        ------------------------------------------------------------------------
        -- Output triggers
        ------------------------------------------------------------------------
        -- Protocol control TX Trigger
        pc_tx_trigger          :out  std_logic;
        
        -- Protocol control RX Trigger
        pc_rx_trigger          :out  std_logic;
        
        -- Bit Stuffing Trigger
        bst_trigger            :out  std_logic;
        
        -- Bit De-Stuffing Trigger
        bds_trigger            :out  std_logic;
        
        -- CRC Trigger RX - No bit stuffing
        crc_trig_rx_nbs        :out  std_logic;
        
        -- CRC Trigger TX - No bit stuffing
        crc_trig_tx_nbs        :out  std_logic;
        
        -- CRC Trigger RX - With bit stuffing
        crc_trig_rx_wbs        :out  std_logic;
        
        -- CRC Trigger TX - With bit stuffing
        crc_trig_tx_wbs        :out  std_logic;
        
        ------------------------------------------------------------------------
        -- Status signals
        ------------------------------------------------------------------------
        -- CRC RX With Bit Stuffing - Data input
        crc_data_rx_wbs        :out  std_logic
    );
    end component;

    component protocol_control_fsm is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;
        
        -- Asynchronous reset
        res_n                   :in   std_logic;
        
        -----------------------------------------------------------------------
        -- Signals which cause state change
        -----------------------------------------------------------------------
        -- RX Trigger
        rx_trigger              :in   std_logic;

        -- Error frame request
        err_frm_req             :in   std_logic;

        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- CTU CAN FD is enabled
        drv_ena                 :in   std_logic;
        
        -- CAN FD type (ISO / NON-ISO)
        drv_fd_type             :in   std_logic;
        
        -- Command to start re-integration in Bus-off
        drv_bus_off_reset       :in   std_logic;
        
        -- Forbidding acknowledge mode
        drv_ack_forb            :in   std_logic;
        
        -- Self Test Mode enabled
        drv_self_test_ena       :in   std_logic;

        -- Bus Monitoring mode enabled
        drv_bus_mon_ena         :in   std_logic;
        
        -- Retransmition limit enabled for errornous frames
        drv_retr_lim_ena        :in   std_logic;
        
         -- Internal Loopback enabled
        drv_int_loopback_ena    :in   std_logic;
        
        -- Reception of CAN FD Frames is enabled
        drv_can_fd_ena          :in   std_logic;
        
        -- Secondary sampling point delay select
        drv_ssp_delay_select    :in   std_logic_vector(1 downto 0);
        
        -- Protocol exception handling
        drv_pex                 :in   std_logic;
        
        -- Protocol exception status clear
        drv_cpexs               :in   std_logic;

        -- ROM mode enabled
        drv_rom_ena             :in   std_logic;

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
        is_err_frm                :out  std_logic;
        
        -- Overload frame is being transmitted
        is_overload             :out  std_logic;
        
        -- Start of Frame
        is_sof                  :out  std_logic;
        
        -- Protocol exception status
        is_pexs                 :out  std_logic;
        
        -----------------------------------------------------------------------
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual TX Data (With Bit stuffing)
        tx_data_wbs             :in   std_logic;
        
        -- Actual RX Data
        rx_data_nbs             :in   std_logic;
        
        -----------------------------------------------------------------------
        -- RX Buffer interface
        -----------------------------------------------------------------------
        -- Command to store CAN frame metadata to RX Buffer
        store_metadata          :out  std_logic;

        -- Command to store word of CAN Data
        store_data              :out  std_logic;
        
        -- Received frame valid
        rec_valid               :out  std_logic;
        
        -- Command to abort storing of RX frame (due to Error frame)
        rec_abort               :out  std_logic;
        
        -- Start of Frame pulse
        sof_pulse               :out  std_logic;

        -----------------------------------------------------------------------
        -- TXT Buffer, TX Arbitrator interface
        -----------------------------------------------------------------------
        -- There is a valid frame for transmission
        tran_frame_valid        :in   std_logic;
        
        -- HW Commands to TXT Buffers
        txtb_hw_cmd             :out  t_txtb_hw_cmd;
        
        -- Pointer to TXT Buffer memory
        txtb_ptr                :out  natural range 0 to 19;
        
        -- Clock enable for TXT Buffer memory
        txtb_clk_en             :out  std_logic;
        
        -- TX Data length code
        tran_dlc                :in   std_logic_vector(3 downto 0);
        
        -- TX Remote transmission request flag
        tran_is_rtr             :in   std_logic;
        
        -- TX Frame type (0-CAN 2.0, 1-CAN FD)
        tran_frame_type         :in   std_logic;

        -- Identifier type (BASIC, EXTENDED)
        tran_ident_type         :in   std_logic;

        -- TX Bit rate shift
        tran_brs                :in   std_logic;
                
        -----------------------------------------------------------------------
        -- TX Shift register interface
        -----------------------------------------------------------------------
        -- Load Base Identifier to TX Shift register
        tx_load_base_id         :out  std_logic;

        -- Load extended Identifier to TX Shift register
        tx_load_ext_id          :out  std_logic;

        -- Load DLC
        tx_load_dlc             :out  std_logic;

        -- Load Data word to TX Shift register
        tx_load_data_word       :out  std_logic;
        
        -- Load Stuff count
        tx_load_stuff_count     :out  std_logic;

        -- Load CRC to TX Shift register
        tx_load_crc             :out  std_logic;

        -- Shift register enable (shifts with TX Trigger)
        tx_shift_ena            :out  std_logic;

        -- Force Dominant value instead of value from shift register
        tx_dominant             :out  std_logic;
        
        -----------------------------------------------------------------------
        -- RX Shift register interface
        -----------------------------------------------------------------------
        -- Clear all registers in RX Shift register
        rx_clear                :out  std_logic;
        
        -- Store Base Identifier 
        rx_store_base_id        :out  std_logic;
        
        -- Store Extended Identifier
        rx_store_ext_id         :out  std_logic;
        
        -- Store Identifier extension
        rx_store_ide            :out  std_logic;
        
        -- Store Remote transmission request
        rx_store_rtr            :out  std_logic;
        
        -- Store EDL (FDF) bit
        rx_store_edl            :out  std_logic;
        
        -- Store DLC
        rx_store_dlc            :out  std_logic;
        
        -- Store ESI
        rx_store_esi            :out  std_logic;
        
        -- Store BRS
        rx_store_brs            :out  std_logic;
        
        -- Store stuff count and Stuff Count parity
        rx_store_stuff_count    :out  std_logic;
        
        -- Clock Enable RX Shift register for each byte.
        rx_shift_ena            :out  std_logic_vector(3 downto 0);
        
        -- Selector for inputs of each byte of shift register
        -- (0-Previous byte output, 1- RX Data input)
        rx_shift_in_sel         :out  std_logic;
        
        -- RX value of Remote transmission request
        rec_is_rtr              :in   std_logic;

        -- RX value of DLC (combinational), valid only in last bit of DLC
        rec_dlc_d               :in   std_logic_vector(3 downto 0);
        
        -- RX value of DLC (captured)
        rec_dlc_q               :in   std_logic_vector(3 downto 0);
        
        -- RX frame type (0-CAN 2.0, 1- CAN FD)
        rec_frame_type          :in   std_logic;

        -----------------------------------------------------------------------
        -- Control counter interface
        -----------------------------------------------------------------------
        -- Preload control counter
        ctrl_ctr_pload          :out   std_logic;
        
        -- Control counter preload value
        ctrl_ctr_pload_val      :out   std_logic_vector(8 downto 0);
        
        -- Control counter is enabled
        ctrl_ctr_ena            :out   std_logic;
        
        -- Control counter is zero
        ctrl_ctr_zero           :in    std_logic;
        
        -- Control counter is equal to 1
        ctrl_ctr_one            :in    std_logic;

        -- Control counter counted multiple of 8 bits
        ctrl_counted_byte       :in    std_logic;

        -- Control counter byte index within a memory word
        ctrl_counted_byte_index :in    std_logic_vector(1 downto 0);
        
        -- Control counter - TXT Buffer memory index
        ctrl_ctr_mem_index      :in    std_logic_vector(4 downto 0);
        
        -- Complementary counter enable
        compl_ctr_ena           :out   std_logic;

        -- Arbitration lost capture ID field
        alc_id_field            :out   std_logic_vector(2 downto 0);

        -----------------------------------------------------------------------
        -- Reintegration counter interface
        -----------------------------------------------------------------------
        -- Reintegration counter Clear (synchronous)
        reinteg_ctr_clr         :out   std_logic;

        -- Enable counting (with RX Trigger)
        reinteg_ctr_enable      :out   std_logic;
        
        -- Reintegration counter expired (reached 128)
        reinteg_ctr_expired     :in    std_logic;

        -----------------------------------------------------------------------
        -- Retransmitt counter interface
        -----------------------------------------------------------------------
        -- Clear Retransmitt counter
        retr_ctr_clear          :out   std_logic;

        -- Increment Retransmitt counter by 1
        retr_ctr_add            :out   std_logic;

        -- Retransmitt limit was reached
        retr_limit_reached      :in    std_logic;

        -----------------------------------------------------------------------
        -- Error detector interface
        -----------------------------------------------------------------------
        -- Form Error has occurred
        form_err              :out   std_logic;

        -- ACK Error has occurred
        ack_err               :out   std_logic;

        -- Perform CRC check
        crc_check               :out   std_logic;
        
        -- Bit Error in arbitration field
        bit_err_arb           :out   std_logic;
        
        -- Calculated CRC and Stuff count are matching received ones
        crc_match               :in   std_logic;

        -- CRC error signalling
        crc_err                 :out  std_logic;

        -- Clear CRC Match flag
        crc_clear_match_flag    :out   std_logic;

        -- CRC Source (CRC15, CRC17, CRC21)
        crc_src                 :out   std_logic_vector(1 downto 0);
        
        -- Error position field (for Error capture)
        err_pos                 :out   std_logic_vector(4 downto 0);
        
        -----------------------------------------------------------------------
        -- Bit Stuffing/Destuffing control signals
        -----------------------------------------------------------------------
        -- Bit Stuffing is enabled
        stuff_enable            :out   std_logic;
        
        -- Bit De-stuffing is enabled
        destuff_enable          :out   std_logic;

        -- Length of Bit stuffing rule
        stuff_length            :out   std_logic_vector(2 downto 0);
        
        -- Fixed Bit stuffing method
        fixed_stuff             :out   std_logic;
        
        -- Frame transmission without SOF started
        tx_frame_no_sof         :out   std_logic;
        
        -----------------------------------------------------------------------
        -- Operation control interface
        -----------------------------------------------------------------------
        -- Unit is transmitter
        is_transmitter          :in   std_logic;
        
        -- Unit is receiver
        is_receiver             :in   std_logic;

        -- Loss of arbitration -> Turn receiver!
        arbitration_lost        :out  std_logic;

        -- Set unit to be transmitter (in SOF)
        set_transmitter         :out  std_logic;

        -- Set unit to be receiver
        set_receiver            :out  std_logic;

        -- Set unit to be idle
        set_idle                :out  std_logic;

        -----------------------------------------------------------------------
        -- Fault confinement interface
        -----------------------------------------------------------------------
        -- Primary Error
        primary_err           :out  std_logic;
        
        -- Active Error or Overload flag is being tranmsmitted
        act_err_ovr_flag        :out  std_logic;

        -- Set unit to be error active
        set_err_active          :out   std_logic;

        -- Error delimiter too late
        err_delim_late          :out  std_logic;

        -- Unit is error active
        is_err_active           :in   std_logic;
        
        -- Unit is error passive
        is_err_passive          :in   std_logic;
        
        -- Unit is Bus off
        is_bus_off              :in   std_logic;
        
        -- Decrement REC
        decrement_rec           :out  std_logic;
        
        -- Bit Error in passive error flag after ACK error
        bit_err_after_ack_err   :out  std_logic;

        -----------------------------------------------------------------------
        -- Other control signals
        -----------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control              :out   std_logic_vector(1 downto 0);
        
        -- Sample control (Registered)
        sp_control_q            :out   std_logic_vector(1 downto 0);
        
        -- Enable Nominal Bit time counters.
        nbt_ctrs_en             :out   std_logic;
        
        -- Enable Data Bit time counters.
        dbt_ctrs_en             :out   std_logic;

        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation)
        sync_control            :out   std_logic_vector(1 downto 0);

        -- Clear the Shift register for secondary sampling point.
        ssp_reset               :out   std_logic;

        -- Enable measurement of transmitter delay
        tran_delay_meas         :out   std_logic;

        -- Transmitted frame is valid
        tran_valid              :out   std_logic;

        -- CRC calculation enabled
        crc_enable              :out   std_logic;
        
        -- CRC calculation - speculative enable
        crc_spec_enable         :out   std_logic;
        
        -- Use RX Data for CRC calculation
        crc_calc_from_rx        :out   std_logic;

        -- Load CRC Initialization vector
        load_init_vect          :out   std_logic;

        -- Bit error enable
        bit_err_enable          :out   std_logic;

        -- Bit rate shifted
        br_shifted              :out   std_logic;
        
        -- Reset Bit time measurement counter
        btmc_reset              :out   std_logic;
    
        -- Start Measurement of data bit time (in TX Trigger)
        dbt_measure_start       :out  std_logic;
    
        -- First SSP generated (in ESI bit)
        gen_first_ssp           :out  std_logic;
        
        -- Synchronization edge
        sync_edge               :in   std_logic
    );
    end component;

    component protocol_control is
    generic(
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
        
        ------------------------------------------------------------------------
        -- DFT support
        ------------------------------------------------------------------------
        scan_enable             :in   std_logic;
        
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
        is_err_frm              :out  std_logic;
        
        -- Overload frame is being transmitted
        is_overload             :out  std_logic;
        
        -- Start of Frame
        is_sof                  :out  std_logic;
        
        -- Protocol exception status
        is_pexs                 :out  std_logic;
                
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

        -- TX Identifier
        tran_identifier         :in   std_logic_vector(28 downto 0);

        -- Frame in TXT Buffer is valid any can be transmitted.
        tran_frame_valid        :in   std_logic;
        
        -- HW Commands for TX Arbitrator and TXT Buffers
        txtb_hw_cmd             :out  t_txtb_hw_cmd;
        
        -- Pointer to TXT buffer memory
        txtb_ptr                :out  natural range 0 to 19;
        
        -- Clock enable for TXT Buffer memory
        txtb_clk_en             :out  std_logic;
        
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
        primary_err           :out  std_logic;
        
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
        
        -- Frame transmission without SOF started
        tx_frame_no_sof         :out  std_logic;

        -- Length of Bit Stuffing rule
        stuff_length            :out  std_logic_vector(2 downto 0);

        -- Number of de-stuffed bits modulo 8
        dst_ctr                 :in   std_logic_vector(2 downto 0);
        
        -- Number of stuffed bits modulo 8
        bst_ctr                 :in   std_logic_vector(2 downto 0);
        
        -- Stuff Error
        stuff_err               :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Bus Sampling Interface
        ------------------------------------------------------------------------
        -- Bit Error detected
        bit_err                 :in   std_logic;
        
        -- Reset Bit time measurement counter
        btmc_reset              :out   std_logic;
    
        -- Start Measurement of data bit time (in TX Trigger)
        dbt_measure_start       :out  std_logic;
    
        -- First SSP generated (in ESI bit)
        gen_first_ssp           :out  std_logic;
        
        -- Synchronization edge
        sync_edge               :in   std_logic;
        
        -----------------------------------------------------------------------
        -- CRC Interface
        -----------------------------------------------------------------------
        -- Enable CRC calculation
        crc_enable              :out  std_logic;
        
        -- CRC calculation - speculative enable
        crc_spec_enable         :out   std_logic;

        -- Use RX Data for CRC calculation
        crc_calc_from_rx        :out   std_logic;
        
        -- Load CRC Initialization vector
        load_init_vect          :out   std_logic;
        
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
                
        -- Sample control (Registered)
        sp_control_q            :out   std_logic_vector(1 downto 0);
        
        -- Enable Nominal Bit time counters.
        nbt_ctrs_en             :out   std_logic;
        
        -- Enable Data Bit time counters.
        dbt_ctrs_en             :out   std_logic;
        
        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation
        sync_control            :out  std_logic_vector(1 downto 0); 
        
        -- Clear the Shift register for secondary sampling point.
        ssp_reset               :out  std_logic;
        
        -- Enable measurement of transmitter delay
        tran_delay_meas         :out   std_logic; 

        -- Transmitted frame is valid
        tran_valid              :out  std_logic;

        -- Received frame is valid
        rec_valid               :out  std_logic;
        
        -- Decrement Receive Error counter
        decrement_rec           :out  std_logic;
        
        -- Bit Error in passive error flag after ACK error
        bit_err_after_ack_err   :out  std_logic;

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Bit rate shifted
        br_shifted              :out  std_logic;
        
        -- Form Error has occurred
        form_err                :out  std_logic;

        -- ACK Error has occurred
        ack_err                 :out  std_logic;
        
        -- CRC Error has occurred
        crc_err                 :out  std_logic;
        
        -- Status of retransmit counter (for observation purpose)
        retr_ctr                :out  std_logic_vector(G_RETR_LIM_CTR_WIDTH - 1 downto 0)
    );
    end component protocol_control;


    component reintegration_counter is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;

        -- Asynchronous reset
        res_n                   :in   std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Clear (synchronous)
        reinteg_ctr_clr         :in   std_logic;

        -- Enable counting (with RX Trigger)
        reinteg_ctr_enable      :in   std_logic;

        -- RX Trigger
        rx_trigger              :in   std_logic;

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Integration counter expired.
        reinteg_ctr_expired     :out  std_logic
    );
    end component;


    component retransmitt_counter is
    generic(
        -- Width of Retransmitt limit counter
        G_RETR_LIM_CTR_WIDTH    :     natural := 4 
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys        :in   std_logic;

        -- Asynchronous reset
        res_n          :in   std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Selected TXT Buffer changed in comparison to previous transmission
        txtb_changed   :in   std_logic;

        -- Clear the counter
        retr_ctr_clear :in   std_logic;
        
        -- Increment the counter by 1
        retr_ctr_add   :in   std_logic;
        
        -- Retransmitt limit
        retr_limit     :in   std_logic_vector(G_RETR_LIM_CTR_WIDTH - 1 downto 0);

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Retransmitt limit was reached
        retr_limit_reached  :out  std_logic;
        
        -- Status of retransmit counter (for observation purpose)
        retr_ctr            :out  std_logic_vector(G_RETR_LIM_CTR_WIDTH - 1 downto 0)
    );
    end component;


    component rx_shift_reg is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;

        -- Asynchronous reset
        res_n                   :in   std_logic;

        ------------------------------------------------------------------------
        -- DFT support
        ------------------------------------------------------------------------
        scan_enable             :in   std_logic;

        -----------------------------------------------------------------------
        -- Trigger signals
        -----------------------------------------------------------------------
        -- RX Trigger
        rx_trigger              :in   std_logic;

        -----------------------------------------------------------------------
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual RX Data
        rx_data_nbs             :in   std_logic;

        -----------------------------------------------------------------------
        -- Protocol control FSM interface
        -----------------------------------------------------------------------
        -- Clear all registers in Shift register (Glitch free)
        rx_clear                :in  std_logic;

        -- Clock Enable RX Shift register for each byte.
        rx_shift_ena            :in  std_logic_vector(3 downto 0);

        -- Selector for inputs of each byte of shift register
        -- (0-Previous byte output, 1- RX Data input)
        rx_shift_in_sel         :in  std_logic;

        -- Store Base Identifier 
        rx_store_base_id        :in  std_logic;

        -- Store Extended Identifier
        rx_store_ext_id         :in  std_logic;

        -- Store Identifier extension
        rx_store_ide            :in  std_logic;
        
        -- Store Remote transmission request
        rx_store_rtr            :in  std_logic;
        
        -- Store EDL (FDF) bit
        rx_store_edl            :in  std_logic;
        
        -- Store DLC
        rx_store_dlc            :in  std_logic;
        
        -- Store ESI
        rx_store_esi            :in  std_logic;
        
        -- Store BRS
        rx_store_brs            :in  std_logic;

        -- Store stuff count
        rx_store_stuff_count    :in  std_logic;
        
        -----------------------------------------------------------------------
        -- RX Buffer interface
        -----------------------------------------------------------------------
        -- RX CAN Identifier
        rec_ident               :out std_logic_vector(28 downto 0);
        
        -- RX Data length code (D input)
        rec_dlc_d               :out std_logic_vector(3 downto 0);
        
        -- RX Data length code
        rec_dlc                 :out std_logic_vector(3 downto 0);
        
        -- RX Remote transmission request flag
        rec_is_rtr              :out std_logic;
        
        -- RX Recieved identifier type (0-BASE Format, 1-Extended Format);
        rec_ident_type          :out std_logic;
        
        -- RX frame type (0-CAN 2.0, 1- CAN FD)
        rec_frame_type          :out std_logic;
        
        -- RX Bit rate shift Flag
        rec_brs                 :out std_logic;
        
        -- RX Error state indicator
        rec_esi                 :out std_logic;
        
        -- Data words to be stored to RX Buffer. Valid only when rx_trigger='1'
        -- in last bit of data word stored
        store_data_word         :out std_logic_vector(31 downto 0);
        
        -----------------------------------------------------------------------
        -- CRC information for CRC comparison
        -----------------------------------------------------------------------
        -- Received CRC
        rx_crc                  :out std_logic_vector(20 downto 0);
        
        -- Received Stuff count + Stuff Parity
        rx_stuff_count          :out std_logic_vector(3 downto 0)
    );
    end component;

    component tx_shift_reg is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;

        -- Asynchronous reset
        res_n                   :in   std_logic;

        -----------------------------------------------------------------------
        -- Trigger signals
        -----------------------------------------------------------------------
        -- RX Trigger
        tx_trigger              :in   std_logic;

        -----------------------------------------------------------------------
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual TX Data (no bit stuffing)
        tx_data_nbs             :out   std_logic;

        -----------------------------------------------------------------------
        -- Protocol control FSM interface
        -----------------------------------------------------------------------
        -- Load Base Identifier to TX Shift register
        tx_load_base_id         :in  std_logic;

        -- Load extended Identifier to TX Shift register
        tx_load_ext_id          :in  std_logic;

        -- Load DLC to TX Shift register
        tx_load_dlc             :in  std_logic;

        -- Load Data word to TX Shift register
        tx_load_data_word       :in  std_logic;

        -- Load Stuff count
        tx_load_stuff_count     :in  std_logic;
        
        -- Load CRC to TX Shift register
        tx_load_crc             :in  std_logic;
        
        -- Shift register enable (shifts with TX Trigger)
        tx_shift_ena            :in  std_logic;

        -- Force Dominant value instead of value from shift register
        tx_dominant             :in  std_logic;

        -- CRC Source (CRC15, CRC17, CRC21)
        crc_src                 :in  std_logic_vector(1 downto 0);

        -----------------------------------------------------------------------
        -- CAN CRC Interface
        -----------------------------------------------------------------------
        -- Calculated CRC 15
        crc_15                  :in   std_logic_vector(14 downto 0);

        -- Calculated CRC 17
        crc_17                  :in   std_logic_vector(16 downto 0);
        
        -- Calculated CRC 21
        crc_21                  :in   std_logic_vector(20 downto 0);

        -----------------------------------------------------------------------
        -- Error detector Interface
        -----------------------------------------------------------------------
        -- Error frame request
        err_frm_req             :in  std_logic;
        
        -----------------------------------------------------------------------
        -- Fault confinement Interface
        -----------------------------------------------------------------------
        -- Unit is error active
        is_err_active           :in  std_logic;
        
        -----------------------------------------------------------------------
        -- Bit Stuffing / Destuffing Interface
        -----------------------------------------------------------------------
        -- Stuff counter modulo 8
        bst_ctr                 :in  std_logic_vector(2 downto 0);

        -----------------------------------------------------------------------
        -- TXT Buffers interface
        -----------------------------------------------------------------------
        -- TX Identifier
        tran_identifier         :in   std_logic_vector(28 downto 0);
        
        -- TXT Buffer RAM word (byte endianity swapped)
        tran_word_swapped       :in   std_logic_vector(31 downto 0);

        -- TX Data length code
        tran_dlc                :in   std_logic_vector(3 downto 0)
    );
    end component;


    component can_core is
    generic(
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
        G_CRC21_POL             :     std_logic_vector(23 downto 0) := x"302899";

        -- Support traffic counters
        G_SUP_TRAFFIC_CTRS      :     boolean := true
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
        -- DFT support
        ------------------------------------------------------------------------
        scan_enable            :in   std_logic;
        
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

        -- TX Identifier
        tran_identifier        :in   std_logic_vector(28 downto 0);

        -- Frame in TXT Buffer is valid any can be transmitted.
        tran_frame_valid       :in   std_logic; 

        -- HW Commands for TX Arbitrator and TXT Buffers
        txtb_hw_cmd            :out  t_txtb_hw_cmd;

        -- Selected TXT Buffer index changed
        txtb_changed           :in   std_logic;

        -- Pointer to TXT buffer memory
        txtb_ptr               :out  natural range 0 to 19;
        
        -- Clock enable for TXT Buffer memory
        txtb_clk_en            :out  std_logic;

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
        rec_esi                :out  std_logic;

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

        -- Fault confinement state changed
        fcs_changed            :out  std_logic;

        -- Error warning limit reached
        err_warning_limit      :out  std_logic;
        
        -- Overload frame is being transmitted
        is_overload            :out  std_logic;

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

        -- Sample control (Nominal, Data, Secondary)
        sp_control    :out  std_logic_vector(1 downto 0); 
        
        -- Enable Nominal Bit time counters.
        nbt_ctrs_en   :out   std_logic;
        
        -- Enable Data Bit time counters.
        dbt_ctrs_en   :out   std_logic;


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

        -- Secondary sample point reset
        ssp_reset           :out  std_logic; 

        -- Enable measurement of Transmitter delay
        tran_delay_meas     :out  std_logic;

        -- Bit Error detected 
        bit_err             :in   std_logic;
        
        -- Secondary sample signal 
        sample_sec          :in   std_logic;
        
        -- Reset Bit time measurement counter
        btmc_reset          :out   std_logic;
    
        -- Start Measurement of data bit time (in TX Trigger)
        dbt_measure_start   :out  std_logic;
    
        -- First SSP generated (in ESI bit)
        gen_first_ssp       :out  std_logic;
        
        -- Synchronization edge
        sync_edge           :in   std_logic
    );
    end component;


    component bit_filter is
    generic(
        -- Filter width
        G_WIDTH              :   natural;

        -- Filter presence
        G_IS_PRESENT         :   boolean
    );
    port(
        -- Filter mask
        filter_mask          : in  std_logic_vector(G_WIDTH - 1 downto 0);

        -- Filter value
        filter_value         : in  std_logic_vector(G_WIDTH - 1 downto 0);

        -- Filter input
        filter_input         : in  std_logic_vector(G_WIDTH - 1 downto 0);

        -- Filter enable (output is stuck at zero when disabled)
        enable               : in  std_logic;

        -- '1' when Filter input passes the filter
        valid                : out std_logic
    );
    end component;
  
  
   component frame_filters is
    generic(
        -- Support filter A
        G_SUP_FILTA          : boolean := true;
        
        -- Support filter B
        G_SUP_FILTB          : boolean := true;
        
        -- Support filter C
        G_SUP_FILTC          : boolean := true;
        
        -- Support range filter
        G_SUP_RANGE          : boolean := true
    );
    port(
        ------------------------------------------------------------------------
        -- Clock an Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              : in std_logic;
        
        -- Asynchronous reset
        res_n                : in std_logic;

        ------------------------------------------------------------------------
        -- Memory registers interface
        ------------------------------------------------------------------------
        -- Driving Bus
        drv_bus              : in  std_logic_vector(1023 downto 0);

        ------------------------------------------------------------------------
        -- CAN Core interface
        ------------------------------------------------------------------------
        -- Receieved CAN ID
        rec_ident            : in  std_logic_vector(28 downto 0); 

        -- Received CAN ID type (0-Base Format, 1-Extended Format);
        rec_ident_type       : in  std_logic;

        -- Input frame type (0-CAN 2.0, 1- CAN FD) 
        rec_frame_type       : in  std_logic;

        -- Store Metadata in RX Buffer
        store_metadata       : in  std_logic;

        -- Command to store word of CAN Data
        store_data           : in  std_logic;
        
        -- Received frame valid
        rec_valid            : in  std_logic;
        
        -- Command to abort storing of RX frame (due to Error frame)
        rec_abort            : in  std_logic;
        
        -- RX Remote transmission request Flag
        rec_is_rtr           : in  std_logic;

        ------------------------------------------------------------------------
        -- Frame filters output
        ------------------------------------------------------------------------
        -- CAN ID passes the filters
        ident_valid          : out   std_logic;
        
        -- Store Metadata in RX Buffer - Filtered
        store_metadata_f     : out   std_logic;

        -- Command to store word of CAN Data - Filtered
        store_data_f         : out   std_logic;
        
        -- Received frame valid - Filtered
        rec_valid_f          : out   std_logic;
        
        -- Command to abort storing of RX frame (due to Error frame) - Filtered
        rec_abort_f          : out   std_logic
    );
    end component;


    component range_filter is
    generic(
        -- Filter width
        G_WIDTH              :   natural;

        -- Filter presence
        G_IS_PRESENT         :   boolean        
    );
    port(
        -- Upper threshold of a filter
        filter_upp_th      : in    std_logic_vector(G_WIDTH - 1 downto 0);

        -- Lower threshold of a filter
        filter_low_th      : in    std_logic_vector(G_WIDTH - 1 downto 0);

        -- Filter input
        filter_input       : in    std_logic_vector(G_WIDTH - 1 downto 0);

        -- Filter enable (output is stuck at zero when disabled)
        enable             : in    std_logic;

        -- Filter output
        valid              : out   std_logic
    );
    end component;


    component int_manager is
    generic(
        -- Number of supported interrupts
        G_INT_COUNT          : natural  := 11;
        
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT   : natural range 2 to 8 := C_TXT_BUFFER_COUNT
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;
        
        -- Asynchronous Reset
        res_n                   :in   std_logic;

        ------------------------------------------------------------------------
        -- Interrupt sources
        ------------------------------------------------------------------------
        -- Error appeared
        err_detected            :in   std_logic;

        -- Fault confinement state changed
        fcs_changed             :in   std_logic;

        -- Error warning limit reached
        err_warning_limit       :in   std_logic;

        -- Arbitration was lost input
        arbitration_lost        :in   std_logic;

        -- Transmitted frame is valid
        tran_valid              :in   std_logic;

        -- Bit Rate Was Shifted
        br_shifted              :in   std_logic;

        -- Rx Buffer data overrun
        rx_data_overrun         :in   std_logic;
        
        -- Received frame is valid
        rec_valid               :in   std_logic;
        
        -- RX Buffer is full
        rx_full          :in   std_logic;
        
        -- Recieve buffer is empty
        rx_empty         :in   std_logic;

        -- HW command on TXT Buffers interrupt
        txtb_hw_cmd_int  :in   std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);
        
        -- Overload frame is being transmitted
        is_overload      :in   std_logic;

        ------------------------------------------------------------------------
        -- Memory registers Interface
        ------------------------------------------------------------------------
        drv_bus          :in   std_logic_vector(1023 downto 0);

        -- Interrupt output
        int              :out  std_logic; 

        -- Interrupt vector
        int_vector       :out  std_logic_vector(G_INT_COUNT - 1 downto 0);

        -- Interrupt mask
        int_mask         :out  std_logic_vector(G_INT_COUNT - 1 downto 0);

        -- Interrupt enable
        int_ena          :out  std_logic_vector(G_INT_COUNT - 1 downto 0)
    );  
    end component int_manager;


    component int_module is
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System Clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous Reset
        res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Control control signals
        ------------------------------------------------------------------------
        -- Interrupt Status Set
        int_status_set         :in   std_logic;
        
        -- Interrupt Status Clear
        int_status_clear       :in   std_logic;

        -- Interrupt Mask Set
        int_mask_set           :in   std_logic;
        
        -- Interrupt Mask Clear
        int_mask_clear         :in   std_logic;

        -- Interrupt Enable Set
        int_ena_set            :in   std_logic;
        
        -- Interrupt Enable Clear
        int_ena_clear          :in   std_logic;

        ------------------------------------------------------------------------
        -- Interrupt output signals
        ------------------------------------------------------------------------
        -- Interrupt status (Interrupt vector)
        int_status             :out  std_logic;
        
        -- Interrupt mask
        int_mask               :out  std_logic;
        
        -- Interrupt enable
        int_ena                :out  std_logic
    );  
    end component;

    component memory_registers is
    generic(
        -- Support Filter A
        G_SUP_FILTA         : boolean                         := true;

        -- Support Filter B
        G_SUP_FILTB         : boolean                         := true;
        
        -- Support Filter C
        G_SUP_FILTC         : boolean                         := true;
        
        -- Support Range Filter
        G_SUP_RANGE         : boolean                         := true;

        -- Support Test registers
        G_SUP_TEST_REGISTERS: boolean                         := true;

        -- Support Traffic counters
        G_SUP_TRAFFIC_CTRS  : boolean                         := true;
        
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT  : natural range 2 to 8            := C_TXT_BUFFER_COUNT;

        -- Number of Interrupts
        G_INT_COUNT         : natural                         := 12;
        
        -- Width (number of bits) in transceiver delay measurement counter
        G_TRV_CTR_WIDTH     : natural                         := 7;

        -- DEVICE_ID (read from register)
        G_DEVICE_ID         : std_logic_vector(15 downto 0)   := x"CAFD";

        -- MINOR Design version
        G_VERSION_MINOR     : std_logic_vector(7 downto 0)    := x"01";

        -- MAJOR Design version
        G_VERSION_MAJOR     : std_logic_vector(7 downto 0)    := x"02";
        
        -- Technology type
        G_TECHNOLOGY        : natural                         := C_TECH_ASIC
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in   std_logic;
        
        -- Asynchronous reset        
        res_n                :in   std_logic;
        
        -- Reset output (input reset + Software Reset)
        res_out              :out  std_logic;

        ------------------------------------------------------------------------
        -- DFT support
        ------------------------------------------------------------------------
        scan_enable          :in   std_logic;

        ------------------------------------------------------------------------
        -- Memory Interface
        ------------------------------------------------------------------------
        -- Data input
        data_in              :in   std_logic_vector(31 downto 0);
        
        -- Data output
        data_out             :out  std_logic_vector(31 downto 0);
        
        -- Address
        adress               :in   std_logic_vector(15 downto 0);
        
        -- Chip Select
        scs                  :in   std_logic;
        
        -- Read
        srd                  :in   std_logic;
        
        -- Write
        swr                  :in   std_logic;
        
        -- Byte enable
        sbe                  :in   std_logic_vector(3 downto 0);
        
        -- Timestamp input
        timestamp            :in   std_logic_vector(63 downto 0);
        
        ------------------------------------------------------------------------
        -- Buses to/from rest of CTU CAN FD
        ------------------------------------------------------------------------
        -- Driving Bus
        drv_bus              :out  std_logic_vector(1023 downto 0);
        
        -- Status Bus
        stat_bus             :in   std_logic_vector(511 downto 0);
        
        ------------------------------------------------------------------------
        -- Manufacturing testability
        ------------------------------------------------------------------------
        -- Test registers outputs
        test_registers_out   :out  test_registers_out_t;
        
        -- RX buffer test data in
        tst_rdata_rx_buf     :in   std_logic_vector(31 downto 0);
        
        -- TXT buffers test data input
        tst_rdata_txt_bufs   :in   t_txt_bufs_output(G_TXT_BUFFER_COUNT - 1 downto 0);

        ------------------------------------------------------------------------
        -- RX Buffer Interface
        ------------------------------------------------------------------------
        -- RX Buffer data output
        rx_read_buff         :in   std_logic_vector(31 downto 0);

        -- Size of RX buffer (in words)
        rx_buf_size          :in   std_logic_vector(12 downto 0);

        -- RX Buffer is full
        rx_full              :in   std_logic;

        -- RX Buffer is empty
        rx_empty             :in   std_logic;

        -- Number of frames in RX buffer
        rx_frame_count       :in   std_logic_vector(10 downto 0);

        -- Number of free 32 bit words
        rx_mem_free          :in   std_logic_vector(12 downto 0);

        -- Position of read pointer
        rx_read_pointer      :in   std_logic_vector(11 downto 0);

        -- Position of write pointer
        rx_write_pointer     :in   std_logic_vector(11 downto 0);
            
        -- Data overrun Flag
        rx_data_overrun      :in   std_logic;
        
        -- Middle of frame indication
        rx_mof               :in   std_logic;

        ------------------------------------------------------------------------
        -- Interface to TXT Buffers
        ------------------------------------------------------------------------
        -- TXT Buffer RAM - Data input
        txtb_port_a_data     :out  std_logic_vector(31 downto 0);
        
        -- TXT Buffer RAM - Address
        txtb_port_a_address  :out  std_logic_vector(4 downto 0);
        
        -- TXT Buffer chip select
        txtb_port_a_cs       :out  std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        -- TXT Buffer status
        txtb_state           :in   t_txt_bufs_state(G_TXT_BUFFER_COUNT - 1 downto 0);

        -- SW Commands to TXT Buffer
        txtb_sw_cmd          :out  t_txtb_sw_cmd;
        
        -- SW Command Index (Index in logic 1 means command is valid for TXT Buffer)          
        txtb_sw_cmd_index    :out  std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);
        
        -- TXT Buffer priorities
        txtb_prorities       :out  t_txt_bufs_priorities(G_TXT_BUFFER_COUNT - 1 downto 0);
         
        -- TXT Buffer bus-off behavior
        txt_buf_failed_bof   :out  std_logic;
        
        ------------------------------------------------------------------------
        -- Bus synchroniser interface
        ------------------------------------------------------------------------
        -- Measured Transceiver Delay
        trv_delay            :in   std_logic_vector(G_TRV_CTR_WIDTH - 1 downto 0);
                    
        ------------------------------------------------------------------------
        -- Interrrupt Interface
        ------------------------------------------------------------------------
        -- Interrupt vector
        int_vector           :in   std_logic_vector(G_INT_COUNT - 1 downto 0);
        
        -- Interrupt enable
        int_ena              :in   std_logic_vector(G_INT_COUNT - 1 downto 0);
        
        -- Interrupt mask
        int_mask             :in   std_logic_vector(G_INT_COUNT - 1 downto 0)
    );  
    end component memory_registers;


    component bit_time_cfg_capture is
    generic (
        -- TSEG1 Width - Nominal Bit Time
        G_TSEG1_NBT_WIDTH  : natural := 8;
        
        -- TSEG2 Width - Nominal Bit Time
        G_TSEG2_NBT_WIDTH  : natural := 8;
        
        -- Baud rate prescaler Width - Nominal Bit Time
        G_BRP_NBT_WIDTH    : natural := 8;
        
        -- Synchronisation Jump width Width - Nominal Bit Time
        G_SJW_NBT_WIDTH    : natural := 5;
        
        -- TSEG1 Width - Data Bit Time
        G_TSEG1_DBT_WIDTH  : natural := 8;
        
        -- TSEG2 Width - Data Bit Time
        G_TSEG2_DBT_WIDTH  : natural := 8;
        
        -- Baud rate prescaler width - Data Bit Time
        G_BRP_DBT_WIDTH    : natural := 8;
        
        -- Synchronisation Jump Width width - Data Bit Time
        G_SJW_DBT_WIDTH    : natural := 5
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys     : in    std_logic;
        
        -- Asynchronous reset
        res_n       : in    std_logic;

        -----------------------------------------------------------------------
        -- Memory Registers interface
        -----------------------------------------------------------------------
        -- Driving Bus
        drv_bus     : in    std_logic_vector(1023 downto 0);

        -----------------------------------------------------------------------
        -- Output values
        -----------------------------------------------------------------------
        -- Time segment 1 - Nominal Bit Time
        tseg1_nbt   : out   std_logic_vector(G_TSEG1_NBT_WIDTH - 1 downto 0);
        
        -- Time segment 2 - Nominal Bit Time
        tseg2_nbt   : out   std_logic_vector(G_TSEG2_NBT_WIDTH - 1 downto 0);
        
        -- Baud Rate Prescaler - Nominal Bit Time
        brp_nbt     : out   std_logic_vector(G_BRP_NBT_WIDTH - 1 downto 0);
        
        -- Synchronisation Jump Width - Nominal Bit Time
        sjw_nbt     : out   std_logic_vector(G_SJW_NBT_WIDTH - 1 downto 0);
        
        -- Time segment 1 - Data Bit Time
        tseg1_dbt   : out   std_logic_vector(G_TSEG1_DBT_WIDTH - 1 downto 0);
        
        -- Time segment 2 - Data Bit Time
        tseg2_dbt   : out   std_logic_vector(G_TSEG2_DBT_WIDTH - 1 downto 0);
        
        -- Baud Rate Prescaler - Data Bit Time
        brp_dbt     : out   std_logic_vector(G_BRP_DBT_WIDTH - 1 downto 0);
        
        -- Synchronisation Jump Width - Data Bit Time
        sjw_dbt     : out   std_logic_vector(G_SJW_DBT_WIDTH - 1 downto 0);
        
        -- Signal to load the expected segment length by Bit time counters
        start_edge  : out   std_logic
    );
    end component bit_time_cfg_capture;


    component bit_time_counters is
    generic (
        -- Bit Time counter width
        G_BT_WIDTH        : natural := 8;
        
        -- Baud rate prescaler width
        G_BRP_WIDTH       : natural := 8
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys          : in    std_logic;
        
        -- Asynchrnous reset
        res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Baud rate Prescaler
        brp              : in    std_logic_vector(G_BRP_WIDTH - 1 downto 0);
        
        -- Time Quanta Counter reset (synchronous)
        tq_reset         : in    std_logic;
        
        -- Bit Time counter reset (synchronous)
        bt_reset         : in    std_logic;

        -- Counters enabled
        ctrs_en          : in    std_logic;

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Time Quanta edge
        tq_edge         : out   std_logic;
       
        -- Bit Time counter
        segm_counter      : out   std_logic_vector(G_BT_WIDTH - 1 downto 0)
    );
    end component;


    component bit_time_fsm is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys             : in    std_logic;
        
        -- Asynchronous reset
        res_n               : in    std_logic;

        -----------------------------------------------------------------------
        -- Control interface 
        -----------------------------------------------------------------------
        -- Segment end (either due to re-sync, or reaching expected length)
        segm_end            : in    std_logic;

        -- CTU CAN FD is enabled
        drv_ena             : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status signals 
        -----------------------------------------------------------------------
        -- Bit time FSM is in TSEG1
        is_tseg1            : out   std_logic;
        
        -- Bit time FSM is in TSEG2
        is_tseg2            : out   std_logic;
        
        -- Sample signal request (to sample point generator)
        rx_trig_req          : out   std_logic;
        
        -- Sync signal request
        tx_trig_req            : out   std_logic;
        
        -- Bit Tim FSM Output
        bt_fsm              : out   t_bit_time 
    );
    end component;


    component prescaler is
    generic(
        -- TSEG1 Width - Nominal Bit Time
        G_TSEG1_NBT_WIDTH       :   natural := 8;
        
        -- TSEG2 Width - Nominal Bit Time
        G_TSEG2_NBT_WIDTH       :   natural := 8;
        
        -- Baud rate prescaler Width - Nominal Bit Time
        G_BRP_NBT_WIDTH         :   natural := 8;
        
        -- Synchronisation Jump width Width - Nominal Bit Time
        G_SJW_NBT_WIDTH         :   natural := 5;
        
        -- TSEG1 Width - Data Bit Time
        G_TSEG1_DBT_WIDTH       :   natural := 8;
        
        -- TSEG2 Width - Data Bit Time
        G_TSEG2_DBT_WIDTH       :   natural := 8;
        
        -- Baud rate prescaler width - Data Bit Time
        G_BRP_DBT_WIDTH         :   natural := 8;
        
        -- Synchronisation Jump Width width - Data Bit Time
        G_SJW_DBT_WIDTH         :   natural := 5;
      
        -- Number of signals in Sample trigger
        G_SAMPLE_TRIGGER_COUNT  :   natural range 2 to 8 := 2
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys              :in std_logic;
        
        -- Asynchronous reset
        res_n                :in std_logic;
        
        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- Driving Bus
        drv_bus              :in std_logic_vector(1023 downto 0); 
        
        -----------------------------------------------------------------------
        -- Control Interface
        -----------------------------------------------------------------------
        -- Synchronisation edge (from Bus sampling)
        sync_edge            :in std_logic;
        
        -- Sample control (Nominal, Data, Secondary)
        sp_control           :in std_logic_vector(1 downto 0);
        
        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation
        sync_control         :in std_logic_vector(1 downto 0);
        
        -- No re-synchronisation should be executed due to positive phase
        -- error
        no_pos_resync        :in std_logic;

        -- Enable Nominal Bit time counters.
        nbt_ctrs_en          :in std_logic;
        
        -- Enable Data Bit time counters.
        dbt_ctrs_en          :in std_logic;
        
        -----------------------------------------------------------------------
        -- Trigger signals
        -----------------------------------------------------------------------
        -- RX Triggers
        rx_triggers     : out std_logic_vector(G_SAMPLE_TRIGGER_COUNT - 1 downto 0);
        
        -- TX Trigger
        tx_trigger      : out std_logic;
        
        -----------------------------------------------------------------------
        -- Status outputs
        -----------------------------------------------------------------------
        
        -- Bit Time FSM state
        bt_fsm          : out t_bit_time;
        
        -- Time quanta edge
        tq_edge         : out std_logic
    );
    end component;


    component bit_segment_meter is
    generic (
        -- SJW width
        G_SJW_WIDTH               :       natural := 4;
        
        -- TSEG1 width
        G_TSEG1_WIDTH             :       natural := 8;
        
        -- TSEG2 width
        G_TSEG2_WIDTH             :       natural := 8;
        
        -- Bit counter width
        G_BT_WIDTH                :       natural := 8
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys          : in    std_logic;
        
        -- Asynchronous reset
        res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Control interface
        -----------------------------------------------------------------------
        -- There is a valid re-synchronisation edge.
        resync_edge_valid    : in    std_logic;

        -----------------------------------------------------------------------
        -- Bit Time FSM interface
        -----------------------------------------------------------------------        
        -- Bit time is in SYNC, PROP or PH1
        is_tseg1         : in    std_logic;
        
        -- Bit time is in PH2
        is_tseg2         : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Bit Time config capture interface
        -----------------------------------------------------------------------
        -- Time segment 1 (SYNC + PROP + PH1)
        tseg_1       : in    std_logic_vector(G_TSEG1_WIDTH - 1 downto 0);
        
        -- Time segment 2 (PH2)
        tseg_2       : in    std_logic_vector(G_TSEG2_WIDTH - 1 downto 0);
        
        -- Synchronisation Jump Width
        sjw          : in    std_logic_vector(G_SJW_WIDTH - 1 downto 0);
        
        -- Circuit operation has started -> load expected segment length reg.
        start_edge   : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Bit Time counter interface
        -----------------------------------------------------------------------
        -- Bit time counter
        segm_counter   : in    std_logic_vector(G_BT_WIDTH - 1 downto 0);

        -----------------------------------------------------------------------
        -- End of segment detector
        -----------------------------------------------------------------------
        -- End of segment (either TSEG1 or TSEG2)
        segm_end         : in    std_logic;
        
        -- Hard synchronisation valid
        h_sync_valid     : in    std_logic;

        -----------------------------------------------------------------------
        -- Output interface (signalling end of segment)
        -----------------------------------------------------------------------
        -- End of segment request
        exit_segm_req    : out   std_logic
    );
    end component;


    component segment_end_detector is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys            : in    std_logic;
        
        -- Asynchronous reset
        res_n              : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Control interface
        -----------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control         : in    std_logic_vector(1 downto 0);
        
        -- Hard synchronisation edge is valid
        h_sync_edge_valid  : in    std_logic;
        
        -- Segment end request (Nominal)
        exit_segm_req_nbt  : in    std_logic;
        
        -- Segment end request (Data)
        exit_segm_req_dbt  : in    std_logic;

        -- Bit time FSM is in TSEG1
        is_tseg1           : in    std_logic;
        
        -- Bit time FSM is in TSEG2
        is_tseg2           : in    std_logic;

        -- Nominal Time quanta is active
        tq_edge_nbt        : in    std_logic;
        
        -- Data Time quanta is active
        tq_edge_dbt        : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Segment end
        segm_end           : out   std_logic;
        
        -- Hard Synchronisation is valid
        h_sync_valid       : out   std_logic;
        
        -- Clear Bit time counters
        bt_ctr_clear       : out   std_logic
    );
    end component segment_end_detector;


    component synchronisation_checker is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys          : in    std_logic;
        
        -- Asynchronous Reset
        res_n            : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Control interface
        -----------------------------------------------------------------------
        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation
        sync_control     : in    std_logic_vector(1 downto 0);
        
        -- Synchronisation edge (from Bus sampling)
        sync_edge        : in    std_logic;
        
        -- No re-synchronisation should be executed due to positive phase
        -- error
        no_pos_resync    : in    std_logic;
        
        -- End of segment
        segment_end      : in    std_logic;
        
        -- Bit time FSM is in TSEG1
        is_tseg1         : in    std_logic;
        
        -- Bit time FSM is in TSEG2
        is_tseg2         : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status
        -----------------------------------------------------------------------
        -- Resynchronisation edge is valid
        resync_edge_valid    : out std_logic;
        
        -- Hard synchronisation edge is valid
        h_sync_edge_valid    : out std_logic
    );
    end component;


    component trigger_generator is
    generic (
        -- Number of signals in Sample trigger
        G_SAMPLE_TRIGGER_COUNT    : natural range 2 to 8 := 3
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys          : in    std_logic;
        
        -- Asynchronous reset
        res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signal
        -----------------------------------------------------------------------
        -- Sample point Request (RX Trigger request)
        rx_trig_req       : in    std_logic;
        
        -- Sync Trigger Request (TX Trigger request)
        tx_trig_req       : in    std_logic;

        -----------------------------------------------------------------------
        -- Trigger outputs
        -----------------------------------------------------------------------
        -- RX Triggers (Two in two following clock cycles)
        rx_triggers     : out std_logic_vector(G_SAMPLE_TRIGGER_COUNT - 1 downto 0);
        
        -- TX Trigger
        tx_trigger      : out std_logic
    );
    end component;


    component rx_buffer_fsm is
    port(
        ------------------------------------------------------------------------
        -- Clocks and Asynchronouts reset 
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in     std_logic;
        
        -- Asynchronous reset
        res_n                :in     std_logic;

        ------------------------------------------------------------------------
        -- Control signals from CAN Core (Filtered by Frame filters)
        ------------------------------------------------------------------------
        -- Start Storing of Metadata to RX Buffer (first 4 words of frame)
        store_metadata_f     :in     std_logic;
       
        -- Store Data word to RX Buffer
        store_data_f         :in     std_logic;

        -- Received frame valid
        rec_valid_f          :in     std_logic;
        
        -- Abort storing of RX Frame to RX Buffer.
        rec_abort_f          :in     std_logic;

        -----------------------------------------------------------------------
        -- FSM outputs
        -----------------------------------------------------------------------
        -- Intent to write to RX Buffer RAM
        write_raw_intent     :out    std_logic;

        -- Write Timestamp to RX Buffer RAM memory
        write_ts             :out    std_logic;

        -- Storing of Timestamp from end of frame has ended.
        stored_ts            :out    std_logic;

        -- Data selector for selection of memory word to be stored in RX Buffer 
        -- RAM (one hot coded)
        data_selector        :out    std_logic_vector(4 downto 0);

        -- Load timestamp write pointer from regular write pointer
        store_ts_wr_ptr      :out    std_logic;

        -- Increment timestamp write pointer by 1
        inc_ts_wr_ptr        :out    std_logic;

        -- Reset internal overrun flag
        reset_overrun_flag   :out    std_logic
    );
    end component;


    component rx_buffer_pointers is
    generic(
        -- RX Buffer size
        G_RX_BUFF_SIZE        :       natural range 32 to 4096 := 32
    );
    port(
        ------------------------------------------------------------------------
        -- Clocks and Asynchronous reset 
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in     std_logic;
        
        -- RX Buffer Reset (External + Release receive Buffer)
        rx_buf_res_n_q_scan  :in     std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Abort storing of frame in RX Buffer. Revert to last frame. Raw RX
        -- pointer will be reverted to internal RX pointers.
        rec_abort_f          :in     std_logic;

        -- Commit RX Frame to RX Buffer. Raw pointer will be stored internal
        -- RX pointer.
        commit_rx_frame      :in     std_logic;

        -- RX Buffer RAM is being written and there is enough space available.
        write_raw_OK         :in     std_logic;

        -- RX Frame is not commited, write pointer raw should be reverted to
        -- last stored write_pointer value.
        commit_overrun_abort :in     std_logic;

        -- RX Buffer FSM signals to store write pointer to timestamp write pointer
        store_ts_wr_ptr      :in     std_logic;

        -- RX Buffer FSM signals to increment timestamp write pointer
        inc_ts_wr_ptr        :in     std_logic;

        -- RX Buffer RAM is being read by SW
        read_increment       :in     std_logic;

        -----------------------------------------------------------------------
        -- Status outputs
        -----------------------------------------------------------------------
        -- Read Pointer (access from SW)
        read_pointer           :out     std_logic_vector(11 downto 0);

        -- Read pointer incremented by 1 (combinationally)
        read_pointer_inc_1     :out     std_logic_vector(11 downto 0);

        -- Write pointer (committed, available to SW, after frame was stored)
        write_pointer          :out     std_logic_vector(11 downto 0);

        -- Write pointer RAW. Changing during frame, as frame is continously stored
        -- to the buffer. When frame is sucesfully received, it is updated to
        -- write pointer!
        write_pointer_raw      :out     std_logic_vector(11 downto 0);

        -- Timestamp write pointer
        write_pointer_ts       :out     std_logic_vector(11 downto 0);

        -- Number of free memory words available for user
        rx_mem_free_i          :out     std_logic_vector(12 downto 0)
    );
    end component;

    component rx_buffer_ram is
    generic(
        -- RX Buffer size
        G_RX_BUFF_SIZE        :       natural range 32 to 4096 := 32
    );
    port(
        ------------------------------------------------------------------------
        -- Clocks and Asynchronous reset 
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in     std_logic;
        
        -- Asynchronous reset
        res_n                :in     std_logic;

        ------------------------------------------------------------------------
        -- Memory testability
        ------------------------------------------------------------------------
        -- Test registers
        test_registers_out   :in     test_registers_out_t;

        -- Test output
        tst_rdata_rx_buf     :out    std_logic_vector(31 downto 0);

        ------------------------------------------------------------------------
        -- Port A - Write (from CAN Core)
        ------------------------------------------------------------------------
        -- Address
        port_a_address       :in     std_logic_vector(11 downto 0);
        
        -- Data
        port_a_data_in       :in     std_logic_vector(31 downto 0);
        
        -- Write signal
        port_a_write         :in     std_logic;

        -----------------------------------------------------------------------
        -- Port B - Read (from Memory registers)
        -----------------------------------------------------------------------
        -- Address
        port_b_address       :in     std_logic_vector(11 downto 0);
        
        -- Data
        port_b_data_out      :out    std_logic_vector(31 downto 0)
    );
    end component;


    component rx_buffer is
    generic(
        -- RX Buffer size
        G_RX_BUFF_SIZE              :       natural range 32 to 4096 := 32;
        
        -- Technology type
        G_TECHNOLOGY                :       natural := C_TECH_ASIC
    );
    port(
        ------------------------------------------------------------------------
        -- Clocks and Asynchronous reset 
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in     std_logic;
        
        -- Async. reset
        res_n                :in     std_logic;

        ------------------------------------------------------------------------
        -- DFT support
        ------------------------------------------------------------------------
        scan_enable          :in     std_logic;

        ------------------------------------------------------------------------
        -- Metadata from CAN Core
        ------------------------------------------------------------------------
        -- Frame Identifier
        rec_ident            :in     std_logic_vector(28 downto 0);
        
        -- Data length code
        rec_dlc              :in     std_logic_vector(3 downto 0);
        
        -- Recieved identifier type (0-BASE Format, 1-Extended Format);
        rec_ident_type       :in     std_logic;
        
        -- Recieved frame type (0-Normal CAN, 1- CAN FD)
        rec_frame_type       :in     std_logic;
        
        -- Recieved frame is RTR Frame(0-No, 1-Yes)
        rec_is_rtr           :in     std_logic;
        
        -- Whenever frame was recieved with BIT Rate shift 
        rec_brs              :in     std_logic;

        -- Recieved error state indicator
        rec_esi              :in     std_logic;

        ------------------------------------------------------------------------
        -- Control signals from CAN Core which control storing of CAN Frame.
        -- (Filtered by Frame Filters)
        ------------------------------------------------------------------------
        -- After control field of CAN frame, metadata are valid and can be stored.
        -- This command starts the RX FSM for storing.
        store_metadata_f     :in     std_logic;
       
        -- Signal that one word of data can be stored (TX_DATA_X_W). This signal
        -- is active when 4 bytes were received or data reception has finished 
        -- on 4 byte unaligned number of frames! (Thus allowing to store also
        -- data which are not 4 byte aligned!
        store_data_f         :in     std_logic;

        -- Data word which should be stored when "store_data" is active!
        store_data_word      :in     std_logic_vector(31 downto 0);

        -- Received frame valid (commit RX Frame)
        rec_valid_f          :in     std_logic;
        
        -- Abort storing of RX Frame to RX Buffer.
        rec_abort_f          :in     std_logic;

        -- Signals start of frame. If timestamp on RX frame should be captured
        -- in the beginning of the frame, this pulse captures the timestamp!
        sof_pulse            :in     std_logic;

        -----------------------------------------------------------------------
        -- Status signals of RX buffer
        -----------------------------------------------------------------------
        -- Actual size of synthetised message buffer (in 32 bit words)
        rx_buf_size          :out    std_logic_vector(12 downto 0);
        
        -- Signal whenever buffer is full (no free memory words)
        rx_full              :out    std_logic;
        
        -- Signal whenever buffer is empty (no frame (message) is stored)
        rx_empty             :out    std_logic;
        
        -- Number of frames (messages) stored in recieve buffer
        rx_frame_count       :out    std_logic_vector(10 downto 0);
        
        -- Number of free 32 bit wide words
        rx_mem_free          :out    std_logic_vector(12 downto 0);
        
        -- Position of read pointer
        rx_read_pointer      :out    std_logic_vector(11 downto 0);
        
        -- Position of write pointer
        rx_write_pointer     :out    std_logic_vector(11 downto 0);
        
        -- Overrun occurred, data were discarded!
        -- (This is a flag and persists until it is cleared by SW)! 
        rx_data_overrun      :out    std_logic;
        
        -- Middle of frame indication
        rx_mof               :out    std_logic;
        
        -- External timestamp input
        timestamp            :in     std_logic_vector(63 downto 0);

        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- Actually loaded data for reading
        rx_read_buff         :out    std_logic_vector(31 downto 0);
        
        -- Driving bus from registers
        drv_bus              :in     std_logic_vector(1023 downto 0);
        
        -- Test registers
        test_registers_out   :in     test_registers_out_t;
        
        -- RX buffer RAM test output
        tst_rdata_rx_buf     :out    std_logic_vector(31 downto 0)
    );
    end component;

    component priority_decoder is
    generic(
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT     : natural range 2 to 8
    );
    port( 
        ------------------------------------------------------------------------
        -- TXT Buffer information
        ------------------------------------------------------------------------
        -- TXT Buffer priority
        prio             : in  t_txt_bufs_priorities(G_TXT_BUFFER_COUNT - 1 downto 0);
        
        -- TXT Buffer is valid for selection
        prio_valid       : in  std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        ------------------------------------------------------------------------
        -- Output interface
        ------------------------------------------------------------------------
        -- Whether selected buffer is valid 
        -- (at least one of the buffers must be non-empty and allowed)
        output_valid     : out  std_logic;

        -- Index of highest priority buffer which is non-empty and allowed
        -- for transmission
        output_index     : out  natural range 0 to G_TXT_BUFFER_COUNT - 1
    );
    end component;


    component tx_arbitrator_fsm is
    port( 
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                :in  std_logic;
        
        -- Asynchronous reset
        res_n                  :in  std_logic;
        
        -----------------------------------------------------------------------
        -- Priority decoder interface
        -----------------------------------------------------------------------
        -- TXT Buffer is valid and selected for transmission
        select_buf_avail       :in  std_logic;
        
        -- Priority decoder output has changed. TXT Arbitrator FSM has to restart
        -- selection process.
        select_index_changed   :in  std_logic;
        
        -----------------------------------------------------------------------
        -- Timestamp comparison interface
        -----------------------------------------------------------------------
        timestamp_valid        :in  std_logic;
        
        -----------------------------------------------------------------------
        -- CAN Core Interface
        -----------------------------------------------------------------------
        -- HW Commands from CAN Core for manipulation with TXT Buffers 
        txtb_hw_cmd             :in t_txtb_hw_cmd;  
        
        ---------------------------------------------------------------------------
        -- TX Arbitrator FSM outputs
        ---------------------------------------------------------------------------
        -- Load Timestamp lower word to metadata pointer
        load_ts_lw_addr        :out std_logic;
        
        -- Load Timestamp upper word to metadata pointer
        load_ts_uw_addr        :out std_logic;
        
        -- Load Frame format word to metadata pointer
        load_ffmt_w_addr       :out std_logic;

        -- Load identifier word to metadata pointer
        load_ident_w_addr      :out std_logic;
        
        -- Clock enable for TXT Buffer RAM
        txtb_meta_clk_en       :out std_logic;

        -- Store timestamp lower word
        store_ts_l_w           :out std_logic;
        
        -- Store metadata (Frame format word) on the output of TX Arbitrator
        store_md_w             :out std_logic;
        
        -- Store identifier (Identifier word) on the output of TX Arbitrator
        store_ident_w          :out std_logic;

        -- Store metadata (Frame format word) to double buffer registers.
        buffer_md_w            :out std_logic; 
        
        -- Signals that TX Arbitrator is locked (CAN Core is transmitting from TXT
        -- Buffer)
        tx_arb_locked          :out std_logic;
        
        -- Store last locked TXT Buffer index
        store_last_txtb_index  :out std_logic;
        
        -- Set valid selected buffer on TX Arbitrator output.
        frame_valid_com_set    :out std_logic;    
        
        -- Clear valid selected buffer on TX Arbitrator output.
        frame_valid_com_clear  :out std_logic 
    );
    end component;


    component tx_arbitrator is
    generic(
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT      : natural range 2 to 8
    );
    port( 
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                :in  std_logic;
        
        -- Asynchronous reset        
        res_n                  :in  std_logic;

        -----------------------------------------------------------------------
        -- TXT Buffers interface
        -----------------------------------------------------------------------
        -- Data words from TXT Buffers RAM memories
        txtb_port_b_data        :in t_txt_bufs_output(G_TXT_BUFFER_COUNT - 1 downto 0);
        
        -- TXT Buffers are available, can be selected by TX Arbitrator
        txtb_available          :in std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);
        
        -- Pointer to TXT Buffer
        txtb_port_b_address     :out natural range 0 to 19;
        
        -- Clock enable to TXT Buffer port B
        txtb_port_b_clk_en      :out std_logic;

        -----------------------------------------------------------------------
        -- CAN Core Interface
        -----------------------------------------------------------------------
        -- TXT Buffer memory word
        tran_word               :out std_logic_vector(31 downto 0);

        -- TX Data length code
        tran_dlc                :out std_logic_vector(3 downto 0);
    
        -- TX Remote transmission request flag
        tran_is_rtr             :out std_logic;

        -- TX Identifier type (0-Basic,1-Extended);
        tran_ident_type         :out std_logic;
    
        -- TX Frame type (0-CAN 2.0, 1-CAN FD)
        tran_frame_type         :out std_logic;
    
        -- TX Frame Bit rate shift Flag 
        tran_brs                :out std_logic;
        
        -- TX Identifier
        tran_identifier         :out std_logic_vector(28 downto 0);
        
        -- There is valid frame selected, can be locked for transmission
        tran_frame_valid        :out std_logic;

        -- HW Commands from CAN Core for manipulation with TXT Buffers 
        txtb_hw_cmd             :in t_txtb_hw_cmd;

        -- Selected TXT Buffer changed in comparison to previous transmission
        txtb_changed            :out std_logic;

        -- Index of the TXT Buffer for which the actual HW command is valid
        txtb_hw_cmd_index       :out natural range 0 to G_TXT_BUFFER_COUNT - 1;

        -- Pointer to TXT Buffer given by CAN Core. Used for reading data words
        txtb_ptr                :in  natural range 0 to 19;

        -- TXT Buffer clock enable (from Protocol control)
        txtb_clk_en             :in  std_logic;

        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- Priorities of TXT Buffers
        txtb_prorities          :in t_txt_bufs_priorities(G_TXT_BUFFER_COUNT - 1 downto 0);
    
        -- TimeStamp value
        timestamp               :in std_logic_vector(63 downto 0)
    );
    end component;


    component txt_buffer_fsm is
    generic(
        -- TXT Buffer ID
        G_ID                   :     natural
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
        -- SW commands
        txtb_sw_cmd             :in   t_txtb_sw_cmd;
        
        -- SW buffer select
        sw_cbs                  :in   std_logic;
        
        -- TXT Buffer bus-off behavior
        txt_buf_failed_bof      :in   std_logic;
        
        -- Restricted operation mode
        drv_rom_ena            :in   std_logic;

        -- Bus monitoring mode
        drv_bus_mon_ena        :in   std_logic;

        ------------------------------------------------------------------------   
        -- CAN Core interface
        ------------------------------------------------------------------------
        -- HW Commands
        txtb_hw_cmd             :in   t_txtb_hw_cmd;  
        
        -- HW Buffer select
        hw_cbs                 :in   std_logic;
    
        -- Unit is Bus off
        is_bus_off             :in   std_logic;

        ------------------------------------------------------------------------
        -- Status signals
        ------------------------------------------------------------------------
        -- Buffer accessible from SW
        txtb_user_accessible   :out  std_logic;

        -- HW Command applied on TXT Buffer.
        txtb_hw_cmd_int        :out  std_logic;

        -- Buffer status (FSM state) encoded for reading by SW.
        txtb_state             :out  std_logic_vector(3 downto 0);

        -- TXT Buffer is available to be locked by CAN Core for transmission
        txtb_available         :out  std_logic;
        
        -- UnMask content of TXT Buffer RAM
        txtb_unmask_data_ram   :out  std_logic
    );             
    end component;

    component txt_buffer_ram is
    generic(
        -- TXT buffer ID
        G_ID                   :     natural
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
        -- Memory Testability
        ------------------------------------------------------------------------
        -- Test registers
        test_registers_out     :in   test_registers_out_t;
        
        -- TXT buffer RAM test output
        tst_rdata_txt_buf      :out  std_logic_vector(31 downto 0);

        ------------------------------------------------------------------------
        -- Port A - Write (from Memory registers)
        ------------------------------------------------------------------------
        -- Address
        port_a_address       :in     std_logic_vector(4 downto 0);
        
        -- Data
        port_a_data_in       :in     std_logic_vector(31 downto 0);
        
        -- Write signal
        port_a_write         :in     std_logic;

        -----------------------------------------------------------------------
        -- Port B - Read (from CAN Core)
        -----------------------------------------------------------------------
        -- Address
        port_b_address       :in     std_logic_vector(4 downto 0);
        
        -- Data
        port_b_data_out      :out    std_logic_vector(31 downto 0)
    );
    end component;

    component txt_buffer is
    generic(
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT     :     natural range 2 to 8;
        
        -- TXT Buffer ID
        G_ID                   :     natural := 1;
        
        -- Technology type
        G_TECHNOLOGY           :     natural := C_TECH_ASIC
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous reset
        res_n                  :in   std_logic;

        -----------------------------------------------------------------------
        -- DFT support
        -----------------------------------------------------------------------
        scan_enable            :in   std_logic;

        ------------------------------------------------------------------------
        -- Memory Registers Interface
        ------------------------------------------------------------------------
        -- Data to be written to TXT Buffer RAM
        txtb_port_a_data       :in   std_logic_vector(31 downto 0);
        
        -- Address in TXT Buffer RAM
        txtb_port_a_address    :in   std_logic_vector(4 downto 0);

        -- TXT Buffer RAM chip select
        txtb_port_a_cs         :in   std_logic;

        -- SW commands
        txtb_sw_cmd            :in   t_txtb_sw_cmd;
        
        -- TXT Buffer index for which SW command is valid
        txtb_sw_cmd_index      :in   std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        -- Buffer State (encoded for Memory registers)
        txtb_state             :out  std_logic_vector(3 downto 0);

        -- TXT buffer bus off behavior
        txt_buf_failed_bof     :in   std_logic;

        -- Restricted operation mode
        drv_rom_ena            :in   std_logic;

        -- Bus monitoring mode
        drv_bus_mon_ena        :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Memory Testability
        ------------------------------------------------------------------------
        -- Test registers
        test_registers_out     :in   test_registers_out_t;
        
        -- TXT buffer RAM test output
        tst_rdata_txt_buf      :out  std_logic_vector(31 downto 0);
        
        ------------------------------------------------------------------------   
        -- Interrupt Manager Interface
        ------------------------------------------------------------------------
        -- HW Command applied
        txtb_hw_cmd_int         :out  std_logic;

        ------------------------------------------------------------------------
        -- CAN Core and TX Arbitrator Interface
        ------------------------------------------------------------------------
        -- HW Commands 
        txtb_hw_cmd            :in   t_txtb_hw_cmd;
        
        -- Index of TXT Buffer for which HW commands is valid          
        txtb_hw_cmd_index      :in   natural range 0 to G_TXT_BUFFER_COUNT - 1;

        -- TXT Buffer RAM data output
        txtb_port_b_data       :out  std_logic_vector(31 downto 0);
        
        -- TXT Buffer RAM address
        txtb_port_b_address    :in   natural range 0 to 19;
        
        -- Clock enable to TXT Buffer port B
        txtb_port_b_clk_en     :in   std_logic;

        -- Unit just turned bus off.
        is_bus_off             :in   std_logic;

        -- TXT Buffer is available to be locked by CAN Core for transmission
        txtb_available         :out  std_logic
    );
    end component;

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
        constant SUP_FILT_B          : boolean := true;
        constant SUP_TRAFFIC_CTRS    : boolean := true
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
        signal lock_1                :in std_logic;
        signal lock_2                :in std_logic;
        signal control_registers_out :out Control_registers_out_t;
        signal control_registers_in  :in Control_registers_in_t
    );
    end component control_registers_reg_map;

    component test_registers_reg_map is
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
        signal lock_1                :in std_logic;
        signal lock_2                :in std_logic;
        signal test_registers_out    :out Test_registers_out_t;
        signal test_registers_in     :in Test_registers_in_t
    );
    end component test_registers_reg_map;

    component CTU_CAN_FD_v1_0 is
    generic(
        use_logger       : boolean                := true;
        rx_buffer_size   : natural range 4 to 512 := 128;
        use_sync         : boolean                := true;
        sup_filtA        : boolean                := true;
        sup_filtB        : boolean                := true;
        sup_filtC        : boolean                := true;
        sup_range        : boolean                := true;
        logger_size      : natural range 0 to 512 := 8
    );
    port(
        aclk             : in  std_logic;
        arstn            : in  std_logic;

        irq              : out std_logic;
        CAN_tx           : out std_logic;
        CAN_rx           : in  std_logic;
        timestamp        : in std_logic_vector(63 downto 0);

        -- Ports of APB4
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
    end component CTU_CAN_FD_v1_0;


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
            
            reg_data_in_o    : out std_logic_vector(31 downto 0);
            reg_data_out_i   : in  std_logic_vector(31 downto 0);
            reg_addr_o       : out std_logic_vector(15 downto 0);
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
