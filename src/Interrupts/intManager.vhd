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
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE WORK.CANconstants.ALL;
use work.CAN_FD_register_map.all;
use work.reduce_lib.all;
use work.CANcomponents.all;

entity intManager is
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
  
end entity;

architecture rtl of intManager is
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
    int_input_active(BEI_IND)       <= error_valid;
    int_input_active(ALI_IND)       <= arbitration_lost;
    int_input_active(EPI_IND)       <= error_passive_changed;
    int_input_active(DOI_IND)       <= rx_message_disc;
    int_input_active(EWLI_IND)      <= error_warning_limit;
    int_input_active(TXI_IND)       <= tx_finished;
    int_input_active(RXI_IND)       <= rec_message_valid;
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
            clear_priority         => false
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

end architecture;
