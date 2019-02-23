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
--  Provides interrupt on int_out output. Interrupt sources are configurable
--  from drv_bus. Interrupt vector provides sources of last interrupts. It is 
--  erased from driving bus by  drv_int_vect_erase
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--    4.6.2016    Interrupt active, interrupt length added to keep interrupt ac-
--                tive for dedicated amount of clock cycles! Each interrupt 
--                which comes in between is stored into interrupt mask but not
--                fired as  separate interrupt!!! 
--    6.6.2016    Added edge detection to interrupt sources! This is to be sure
--                that one long active interrupt source will fire only one in-
--                terrupt and not fire interrupts consecutively! THen it could
--                happend that interrupt handler is interrupted by another in-
--                terrupt from the same source signal representing same event...
--                Fast CPU might get cycled in many interrupt handler calls. 
--    27.6.2016   Added bug fix of RX Buffer full interrupt
--    07.3.2018   Reimplemented to support masking, separate set, and clear on
--                interrupt enable and interrupt mask. Interrupts changed to
--                be level based instead of edge based with fixed duration. This
--                is more fitting for SocketCAN implementation.
--    12.3.2018   Implemented RX Buffer not empty and TX Buffer HW command INT.
--    30.8.2018   Moved HW command detection logic to TXT Buffer from here.
--                Thus TXT Buffer can properly filter commands, to avoid
--                overflow of interrupts! Replaced "txt_hw_cmd" with 
--                "txt_hw_cmd_int" signal.
--   11.12.2018   Separated interrupt logic to dedicated sub-module. Added
--                option to have configurable set/clear of interrupt by
--                generic option...
--    23.2.2019   Added PSL functional coverage.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

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

entity int_manager is
    generic(
        --Length in clock cycles how long will interrupt stay active
        constant int_count            :natural range 0 to 32 := 11
    );
    port(
        ------------------------------------------------------------------------
        -- System Clock and reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic; --System Clock
        signal res_n                  :in   std_logic; --Async Reset

        ------------------------------------------------------------------------
        -- Interrupt sources
        ------------------------------------------------------------------------
        -- Valid Error appeared for interrupt
        signal error_valid            :in   std_logic;

        -- Error pasive /Error acitve functionality changed
        signal error_passive_changed  :in   std_logic;

        -- Error warning limit reached
        signal error_warning_limit    :in   std_logic;

        -- Arbitration was lost input
        signal arbitration_lost       :in   std_logic;

        -- Message stored in CAN Core was sucessfully transmitted
        signal tx_finished            :in   std_logic;

        -- Bit Rate Was Shifted
        signal br_shifted             :in   std_logic;

        -- Rx Buffer
        signal rx_message_disc        :in   std_logic; --Income frame was discarded
        signal rec_message_valid      :in   std_logic; --Message recieved!
        -- Note : use the "out_ident_valid" signal of messageFilters. 
        -- Therefore only interrupt is started for signals which pass income 
        -- filters

        signal rx_full                :in   std_logic;
        -- RX Buffer is full (the last income message filled the remaining space)
        -- NOTE! rec_message_valid will be in logic one for two clock cycles

        -- Recieve buffer is empty
        signal rx_empty               :in   std_logic;

        -- HW command on TXT Buffers interrupt
        signal txt_hw_cmd_int         :in   std_logic_vector(TXT_BUFFER_COUNT - 1
                                                             downto 0);

        -- Event logger
        signal loger_finished         :in   std_logic;  --Event logging finsihed

        ------------------------------------------------------------------------
        -- Driving registers Interface
        ------------------------------------------------------------------------
        signal drv_bus                :in   std_logic_vector(1023 downto 0);

        -- Interrupt output
        signal int_out                :out  std_logic; 

        -- Interrupt vector (Interrupt register of SJA1000)
        signal int_vector             :out  std_logic_vector(
                                                int_count - 1 downto 0);

        signal int_mask               :out  std_logic_vector(
                                                int_count - 1 downto 0);

        signal int_ena                :out  std_logic_vector(
                                                int_count - 1 downto 0)
    );  
end entity;

architecture rtl of int_manager is
  
    ----------------------------------------------------------------------------
    -- Driving bus aliases 
    ----------------------------------------------------------------------------
    signal drv_int_vect_clr       :     std_logic_vector(int_count - 1 downto 0);

    signal drv_int_ena_set        :     std_logic_vector(int_count - 1 downto 0);

    signal drv_int_ena_clr        :     std_logic_vector(int_count - 1 downto 0);

    signal drv_int_mask_set       :     std_logic_vector(int_count - 1 downto 0);

    signal drv_int_mask_clr       :     std_logic_vector(int_count - 1 downto 0);

    ----------------------------------------------------------------------------
    -- Internal registers and signals
    ----------------------------------------------------------------------------

    signal int_ena_i              :     std_logic_vector(int_count - 1 downto 0);

    signal int_mask_i             :     std_logic_vector(int_count - 1 downto 0);

    signal int_vect_i             :     std_logic_vector(int_count - 1 downto 0);

    signal int_input_active       :     std_logic_vector(int_count - 1 downto 0);

    constant zero_mask            :     std_logic_vector(int_count - 1 downto 0)
                                                := (OTHERS => '0');
                                                
    ----------------------------------------------------------------------------
    -- Reset over set priority assignment
    ----------------------------------------------------------------------------
    type int_s_r_priority_type is array(0 to int_count - 1) of boolean;
    
    constant int_clear_priority     :     int_s_r_priority_type :=
        (false,  -- RXI_IND
         false,  -- TXI_IND
         false,  -- EWLI_IND
         true,   -- DOI_IND
         false,  -- EPI_IND
         false,  -- ALI_IND
         false,  -- BEI_IND
         false,  -- LFI_IND
         false,  -- RXFI_IND
         false,  -- BSI_IND
         false,  -- RBNEI_IND
         false   -- TXBHCI_IND
        );

begin
  
    -- Driving bus aliases
    drv_int_vect_clr  <= drv_bus(DRV_INT_CLR_HIGH downto DRV_INT_CLR_LOW);
    drv_int_ena_set   <= drv_bus(DRV_INT_ENA_SET_HIGH downto DRV_INT_ENA_SET_LOW);
    drv_int_ena_clr   <= drv_bus(DRV_INT_ENA_CLR_HIGH downto DRV_INT_ENA_CLR_LOW);
    drv_int_mask_set  <= drv_bus(DRV_INT_MASK_SET_HIGH downto DRV_INT_MASK_SET_LOW);
    drv_int_mask_clr  <= drv_bus(DRV_INT_MASK_CLR_HIGH downto DRV_INT_MASK_CLR_LOW);


    ---------------------------------------------------------------------------      
    -- Register to output propagation
    ---------------------------------------------------------------------------
    int_vector                          <= int_vect_i;
    int_mask                            <= int_mask_i;
    int_ena                             <= int_ena_i;


    ---------------------------------------------------------------------------      
    -- Driving Interrupt output when there is at least one active interrupt
    -- enabled.
    ---------------------------------------------------------------------------
    int_out  <= '0' when (int_vect_i and int_ena_i) = zero_mask else
                '1';

    ---------------------------------------------------------------------------
    -- Interrupt register masking and enabling
    ---------------------------------------------------------------------------
    int_input_active(RXI_IND)       <= rec_message_valid;
    int_input_active(TXI_IND)       <= tx_finished;
    int_input_active(EWLI_IND)      <= error_warning_limit;
    int_input_active(DOI_IND)       <= rx_message_disc;
    int_input_active(EPI_IND)       <= error_passive_changed;
    int_input_active(ALI_IND)       <= arbitration_lost;
    int_input_active(BEI_IND)       <= error_valid;
    int_input_active(LFI_IND)       <= loger_finished;
    int_input_active(RXFI_IND)      <= rx_full;
    int_input_active(BSI_IND)       <= br_shifted;
    int_input_active(RBNEI_IND)     <= not rx_empty;
    int_input_active(TXBHCI_IND)    <= or_reduce(txt_hw_cmd_int);


    ---------------------------------------------------------------------------
    -- Interrupt module instances
    ---------------------------------------------------------------------------
    int_module_gen : for i in 0 to int_count - 1 generate
        
        int_module_comp : int_module
        generic map(        
            reset_polarity         => ACT_RESET,
            clear_priority         => int_clear_priority(i)
        )
        port map(
            clk_sys                => clk_sys,
            res_n                  => res_n,

            int_status_set         => int_input_active(i),
            int_status_clear       => drv_int_vect_clr(i),

            int_mask_set           => drv_int_mask_set(i),
            int_mask_clear         => drv_int_mask_clr(i),

            int_ena_set            => drv_int_ena_set(i),
            int_ena_clear          => drv_int_ena_clr(i),

            int_status             => int_vect_i(i),
            int_mask               => int_mask_i(i),
            int_ena                => int_ena_i(i)
        );
    end generate int_module_gen;


    ---------------------------------------------------------------------------
    -- Functional coverage
    ---------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl rxi_set_cov : cover
    --  {int_vect_i(RXI_IND) = '0';int_vect_i(RXI_IND) = '1'};

    -- psl rxi_enable_cov : cover
    --  (int_vect_i(RXI_IND) = '1' and int_ena(RXI_IND) = '1');


    -- psl txi_set_cov : cover
    --  {int_vect_i(TXI_IND) = '0';int_vect_i(TXI_IND) = '1'};

    -- psl txi_enable_cov : cover
    --  (int_vect_i(TXI_IND) = '1' and int_ena(TXI_IND) = '1');


    -- psl ewli_int_set_cov : cover
    --  {int_vect_i(EWLI_IND) = '0';int_vect_i(EWLI_IND) = '1'};

    -- psl ewli_enable_cov : cover
    --  (int_vect_i(EWLI_IND) = '1' and int_ena(EWLI_IND) = '1');


    -- psl doi_int_set_cov : cover
    --  {int_vect_i(DOI_IND) = '0';int_vect_i(DOI_IND) = '1'};

    -- psl doi_enable_cov : cover
    --  (int_vect_i(DOI_IND) = '1' and int_ena(DOI_IND) = '1');


    -- psl epi_int_set_cov : cover
    --  {int_vect_i(EPI_IND) = '0';int_vect_i(EPI_IND) = '1'};

    -- psl epi_enable_cov : cover
    --  (int_vect_i(EPI_IND) = '1' and int_ena(EPI_IND) = '1');


    -- psl ali_int_set_cov : cover
    --  {int_vect_i(ALI_IND) = '0';int_vect_i(ALI_IND) = '1'};

    -- psl ali_enable_cov : cover
    --  (int_vect_i(ALI_IND) = '1' and int_ena(ALI_IND) = '1');


    -- psl beu_int_set_cov : cover
    --  {int_vect_i(BEI_IND) = '0';int_vect_i(BEI_IND) = '1'};

    -- psl bei_enable_cov : cover
    --  (int_vect_i(BEI_IND) = '1' and int_ena(BEI_IND) = '1');


    -- psl lfi_int_set_cov : cover
    --  {int_vect_i(LFI_IND) = '0';int_vect_i(LFI_IND) = '1'};

    -- psl lfi_enable_cov : cover
    --  (int_vect_i(LFI_IND) = '1' and int_ena(LFI_IND) = '1');


    -- psl rxfi_int_set_cov : cover
    --  {int_vect_i(RXFI_IND) = '0';int_vect_i(RXFI_IND) = '1'};

    -- psl rxfi_enable_cov : cover
    --  (int_vect_i(RXFI_IND) = '1' and int_ena(RXFI_IND) = '1');


    -- psl bsi_int_set_cov : cover
    --  {int_vect_i(BSI_IND) = '0';int_vect_i(BSI_IND) = '1'};

    -- psl bsi_enable_cov : cover
    --  (int_vect_i(BSI_IND) = '1' and int_ena(BSI_IND) = '1');


    -- psl rbnei_int_set_cov : cover
    --  {int_vect_i(RBNEI_IND) = '0';int_vect_i(RBNEI_IND) = '1'};

    -- psl rbnei_enable_cov : cover
    --  (int_vect_i(RBNEI_IND) = '1' and int_ena(RBNEI_IND) = '1');


    -- psl txbhci_int_set_cov : cover
    --  {int_vect_i(TXBHCI_IND) = '0';int_vect_i(TXBHCI_IND) = '1'};

    -- psl txbhci_enable_cov : cover
    --  (int_vect_i(TXBHCI_IND) = '1' and int_ena(TXBHCI_IND) = '1');

end architecture;
