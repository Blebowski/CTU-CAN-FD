--------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy 
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is 
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in 
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
-- IN THE SOFTWARE.
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
--
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
USE WORK.CANconstants.ALL;

entity intManager is
  GENERIC(
		--Length in clock cycles how long will interrupt stay active
    constant int_length:natural range 0 to 10:=5
    );
  PORT(
    --------------------------
    --System Clock and reset--
    --------------------------
    signal clk_sys                :in   std_logic; --System Clock
    signal res_n                  :in   std_logic; --Async Reset
    
    ---------------------
    --Interrupt sources -
    ---------------------
    --Valid Error appeared for interrupt
    signal error_valid            :in   std_logic;
    
    --Error pasive /Error acitve functionality changed
    signal error_passive_changed  :in   std_logic;
    
    --Error warning limit reached
    signal error_warning_limit    :in   std_logic;
    
    --Arbitration was lost input
    signal arbitration_lost       :in   std_logic;
    
    --Wake up appeared
    signal wake_up_valid          :in   std_logic;
    
    --Message stored in CAN Core was sucessfully transmitted
    signal tx_finished            :in   std_logic;
    
    --Bit Rate Was Shifted
    signal br_shifted             :in   std_logic;
    
    --Rx Buffer
    signal rx_message_disc        :in   std_logic; --Income frame was discarded
    signal rec_message_valid      :in   std_logic; --Message recieved!
    --Note : use the "out_ident_valid" signal of messageFilters. Therefore only
    --interrupt is started for signals which pass income filters
    
    signal rx_full                :in   std_logic;
    --RX Buffer is full (the last income message filled the remaining space)
    --NOTE! rec_message_valid will be in logic one for two clock cycles
    
    --Event logger
    signal loger_finished         :in   std_logic;  --Event logging finsihed
    
    -------------------------------
    --Driving registers Interface--
    -------------------------------
    signal drv_bus                :in   std_logic_vector(1023 downto 0);
    signal int_out                :out  std_logic; --Interrupt output
    
    --Interrupt vector (Interrupt register of SJA1000)
    signal int_vector             :out  std_logic_vector(10 downto 0) 
    
  );
  -----------------------
  --Driving bus aliases--
  -----------------------
  
  --Bus Error interrupt enable
  signal drv_bus_err_int_ena      :     std_logic;
  
  --Arbitrarion lost interrupt enable
  signal drv_arb_lst_int_ena      :     std_logic;
  
  --Error state changed interrupt enable
  signal drv_err_pas_int_ena      :     std_logic;
  
  --Wake up interrupt enable
  signal drv_wake_int_ena         :     std_logic;
  
  --Data OverRun interrupt enable
  signal drv_dov_int_ena          :     std_logic;
  
  --Error warning limit reached
  signal drv_err_war_int_ena      :     std_logic;
  
  --Frame sucessfully transcieved
  signal drv_tx_int_ena           :     std_logic;
  
  --Frame sucessfully recieved
  signal drv_rx_int_ena           :     std_logic;
  
  --Event logging finished interrupt enable
  signal drv_log_fin_int_ena      :     std_logic;
  
  --Recieve buffer full interrupt enable
  signal drv_rx_full_int_ena      :     std_logic;
  
  --Bit Rate Shift interrupt enable  
  signal drv_brs_int_ena          :     std_logic;

	--Logic 1 erases interrupt vector
  signal drv_int_vect_erase       :     std_logic;
  
  -- Erase previous vector value
  signal drv_int_vect_erase_prev  :     std_logic;
  
  ----------------------------------
  --Internal registers and signals--
  ----------------------------------
  constant int_vector_length      :     natural:=11;
  
  --Interrupt vector register
  signal int_vector_reg           : 		std_logic_vector(int_vector_length-1 
																												 downto 0);
  
  --Interrupt register Vector mask 
  signal int_mask                 :     std_logic_vector(int_vector_length-1
																												 downto 0);
  
  --Registered value of interrupt
  signal int_out_reg              :     std_logic; 
  constant zero_mask              :     std_logic_vector(int_vector_length-1
																												 downto 0):=(OTHERS=>'0');
  
  signal interrupt_active         :     std_logic;
  signal interrupt_counter        :     natural;
  
  -------------------------------------------------
  --Registers for edge detection on source signals
  -------------------------------------------------
  
  --Valid Error appeared for interrupt
  signal error_valid_r              :   std_logic;
  
  --Error pasive /Error acitve functionality changed
  signal error_passive_changed_r    :   std_logic;
  
  --Error warning limit reached
  signal error_warning_limit_r      :   std_logic;
  
  --Arbitration was lost input
  signal arbitration_lost_r         :   std_logic;
  
  --Wake up appeared
  signal wake_up_valid_r            :   std_logic;
  
  --Message stored in CAN Core was sucessfully transmitted
  signal tx_finished_r              :   std_logic;
  
  --Bit Rate Was Shifted
  signal br_shifted_r               :   std_logic;
  
  --Income message was discarded
  signal rx_message_disc_r          :   std_logic;
  
  --Message recieved!
  signal rec_message_valid_r        :   std_logic;
  signal rx_full_r                  :   std_logic;
  
  --Event logging finsihed
  signal loger_finished_r           :   std_logic;
  
  
end entity;

architecture rtl of intManager is
begin
  
  --Driving bus aliases
  drv_bus_err_int_ena         <=  drv_bus(DRV_BUS_ERR_INT_ENA_INDEX);
  drv_arb_lst_int_ena         <=  drv_bus(DRV_ARB_LST_INT_ENA_INDEX);
  drv_err_pas_int_ena         <=  drv_bus(DRV_ERR_PAS_INT_ENA_INDEX);
  drv_wake_int_ena            <=  drv_bus(DRV_WAKE_INT_ENA_INDEX);
  drv_dov_int_ena             <=  drv_bus(DRV_DOV_INT_ENA_INDEX);
  drv_err_war_int_ena         <=  drv_bus(DRV_ERR_WAR_INT_ENA_INDEX);
  drv_tx_int_ena              <=  drv_bus(DRV_TX_INT_ENA_INDEX);
  drv_rx_int_ena              <=  drv_bus(DRV_RX_INT_ENA_INDEX);
  drv_log_fin_int_ena         <=  drv_bus(DRV_LOG_FIN_INT_ENA_INDEX);
  drv_rx_full_int_ena         <=  drv_bus(DRV_RX_FULL_INT_ENA_INDEX);
  drv_brs_int_ena             <=  drv_bus(DRV_BRS_INT_ENA_INDEX);
  drv_int_vect_erase          <=  drv_bus(DRV_INT_VECT_ERASE_INDEX);
  
  --Register to output propagation
  int_vector                  <=  int_vector_reg;
  int_out                     <=  int_out_reg;
  
  --Interrupt register masking and enabling
  int_mask(BUS_ERR_INT)       <=  drv_bus_err_int_ena   and 
																	error_valid           and 
																	(not error_valid_r);
  int_mask(ARB_LST_INT)       <=  drv_arb_lst_int_ena   and 
																	arbitration_lost      and 
																	(not arbitration_lost_r);
  int_mask(ERR_PAS_INT)       <=  drv_err_pas_int_ena   and 
																	error_passive_changed and 
																	(not error_passive_changed_r);
  int_mask(WAKE_INT)          <=  drv_wake_int_ena      and 
																	wake_up_valid         and 
																	(not wake_up_valid_r);
  int_mask(DOV_INT)           <=  drv_dov_int_ena       and 
																	rx_message_disc       and 
																	(not rx_message_disc_r);
  int_mask(ERR_WAR_INT)       <=  drv_err_war_int_ena   and 
																	error_warning_limit   and 
																	(not error_warning_limit_r);
  int_mask(TX_INT)            <=  drv_tx_int_ena        and 
																	tx_finished           and 
																	(not tx_finished_r);
  int_mask(RX_INT)            <=  drv_rx_int_ena        and 
																	rec_message_valid     and 
																	(not rec_message_valid_r);
  int_mask(LOG_FIN_INT)       <=  drv_log_fin_int_ena   and 
																	loger_finished        and 
																	(not loger_finished_r);
  
  int_mask(RX_FULL_INT)       <=  drv_rx_full_int_ena   and 
																	rx_full and (not rx_full_r);
  
  --Note: also rec_message_valid has to be compared otherwise interrupt 
  --would start always when the buffer is full 
  int_mask(BRS_INT)           <=  drv_brs_int_ena       and br_shifted; 
  
  int_out_reg                 <= '1' when interrupt_active = '1' else '0';
  
  ----------------------------------------------------
  --Edge detection process
  ----------------------------------------------------
  edge_det:process(res_n,clk_sys)
  begin
    if(res_n=ACT_RESET)then
      error_valid_r               <= '0'; 
      error_passive_changed_r     <= '0'; 
      error_warning_limit_r       <= '0'; 
      arbitration_lost_r          <= '0'; 
      wake_up_valid_r             <= '0'; 
      tx_finished_r               <= '0'; 
      br_shifted_r                <= '0'; 
      rx_message_disc_r           <= '0'; 
      rec_message_valid_r         <= '0'; 
      rx_full_r                   <= '0'; 
      loger_finished_r            <= '0'; 
    elsif rising_edge(clk_sys)then
      error_valid_r               <= error_valid; 
      error_passive_changed_r     <= error_passive_changed; 
      error_warning_limit_r       <= error_warning_limit; 
      arbitration_lost_r          <= arbitration_lost; 
      wake_up_valid_r             <= wake_up_valid; 
      tx_finished_r               <= tx_finished; 
      br_shifted_r                <= br_shifted; 
      rx_message_disc_r           <= rx_message_disc; 
      rec_message_valid_r         <= rec_message_valid; 
      rx_full_r                   <= rx_full; 
      loger_finished_r            <= loger_finished; 
    end if;
  end process;
  
  
  ----------------------------------------------------
  --Main interrupt process
  ----------------------------------------------------
  int_proc:process(res_n,clk_sys)
  begin
  if(res_n=ACT_RESET)then
    int_vector_reg            <=  (OTHERS=>'0');
    drv_int_vect_erase_prev   <=  '0';
    interrupt_active          <=  '0';
    interrupt_counter         <=  0;
  elsif rising_edge(clk_sys)then
    drv_int_vect_erase_prev   <=  drv_int_vect_erase;
    interrupt_active          <=  interrupt_active;
    interrupt_counter         <=  interrupt_counter;
    
    --Interrupt vector handling (falling edge of reading detection)
    if(drv_int_vect_erase='0' and drv_int_vect_erase_prev='1' )then
      int_vector_reg          <=  (OTHERS=>'0');
    else
      --Storing actual interrupt mask
      int_vector_reg          <=  int_vector_reg or int_mask; 
    end if;
    
    --Interrupt output
    if(int_mask /= zero_mask and interrupt_active='0')then
      interrupt_active        <= '1';
    end if;
    
    --Keep the interrupt active for int_length clock cycles
    if(interrupt_active='1')then
      if(interrupt_counter=int_length)then
        interrupt_active<='0';
        interrupt_counter<=0;
      else
        interrupt_counter<=interrupt_counter+1;
      end if;
    end if; 
    
  end if;  
  end process;
  
end architecture;
