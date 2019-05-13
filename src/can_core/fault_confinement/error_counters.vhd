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
--  Error counters
--
--------------------------------------------------------------------------------
-- Revision History:
--   26.03.2019   Created file
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

entity error_counters is
    generic(
        -- Reset polarity
        G_RESET_POLARITY       :     std_logic := '0'
    );
    port(
        -----------------------------------------------------------------------
        -- System clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock input 
        clk_sys                :in   std_logic;

        -- Asynchronous reset
        res_n                  :in   std_logic;

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
        
        -- Reset error counters (asynchronously)
        reset_err_counters     :in   std_logic;
        
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
end entity;

architecture rtl of error_counters is

    -- Error counter registers
    signal tx_err_ctr_d : unsigned(8 downto 0);
    signal rx_err_ctr_d : unsigned(8 downto 0);
    signal tx_err_ctr_q : unsigned(8 downto 0);
    signal rx_err_ctr_q : unsigned(8 downto 0);

    -- Clock enables for error counter registers
    signal tx_err_ctr_ce : std_logic;
    signal rx_err_ctr_ce : std_logic;

    -- Selected error counter
    signal err_ctr_selected : unsigned(8 downto 0);

    -- Modify TX/RX Error counter
    signal modif_tx_ctr   : std_logic;
    signal modif_rx_ctr   : std_logic;

    -- Error counters increment
    signal err_ctr_inc      : unsigned(8 downto 0);
    
    -- TX/RX Error counter decremented value
    signal tx_err_ctr_dec   : unsigned(8 downto 0);
    signal rx_err_ctr_dec   : unsigned(8 downto 0);
    
    -- Error counters for nominal bit errors, data bit errors
    signal nom_err_ctr_d     : unsigned(15 downto 0);
    signal data_err_ctr_d     : unsigned(15 downto 0);
    signal nom_err_ctr_q     : unsigned(15 downto 0);
    signal data_err_ctr_q     : unsigned(15 downto 0);
    
    -- Selected value of counter
    signal nom_dat_sel_ctr   : unsigned(15 downto 0);
    
    -- Combinationally added value
    signal nom_dat_sel_ctr_add : unsigned(15 downto 0);
    
    -- Clock enables for error counter registers
    signal nom_err_ctr_ce : std_logic;
    signal data_err_ctr_ce : std_logic;
    
begin

    modif_tx_ctr <= '1' when (inc_eight = '1' or dec_one = '1') else
                    '0';

   -- Counters are modified either +1. +8 or -1
   modif_rx_ctr <= '1' when (inc_one = '1' or inc_eight = '1' or dec_one = '1')
                       else
                   '0';

   -- Increment by 1 or 8
   err_ctr_inc <= "000000001" when (inc_one = '1') else
                  "000001000";

   ----------------------------------------------------------------------------
   -- TX Error counter, next value calculation
   ----------------------------------------------------------------------------
   tx_err_ctr_dec <=  (tx_err_ctr_q - 1) when (tx_err_ctr_q > 0) else
                      tx_err_ctr_q;

   -- Next value for error counter inctement when any of "inc" commands is
   -- valid. Decrement otherwise!
   tx_err_ctr_d <=                 
             unsigned(drv_ctr_val) when (tx_err_ctr_pload = '1') else
                  tx_err_ctr_q + 8 when (inc_eight = '1')
                                   else
                    tx_err_ctr_dec;

   -- Clock enable: Tick error counter register when unit is transmitter and
   -- one of commands is valid!
   tx_err_ctr_ce <= '1' when (modif_tx_ctr = '1' and is_transmitter = '1') else
                    '1' when (tx_err_ctr_pload = '1') else
                    '0';

   ----------------------------------------------------------------------------
   -- TX Error counter register
   ----------------------------------------------------------------------------
   tx_err_ctr_reg_proc : process(clk_sys, res_n, reset_err_counters)
   begin
       if (res_n = G_RESET_POLARITY or reset_err_counters = '1') then
           tx_err_ctr_q <= (OTHERS => '0');
       elsif (rising_edge(clk_sys)) then
           if (tx_err_ctr_ce = '1') then
               tx_err_ctr_q <= tx_err_ctr_d;
           end if;
       end if;
   end process;

   ----------------------------------------------------------------------------
   -- RX Error counter, next value calculation
   ----------------------------------------------------------------------------
   -- Set to 120 when counter is more than 127, decrement otherwise!
   rx_err_ctr_dec <= to_unsigned(120, 9) when (rx_err_ctr_q > 127) else
                      (rx_err_ctr_q - 1) when (tx_err_ctr_q > 0) else
                            rx_err_ctr_q;

   -- Next value for error counter inctement when any of "inc" commands is
   -- valid. Decrement otherwise!
   rx_err_ctr_d <=
             unsigned(drv_ctr_val) when (rx_err_ctr_pload = '1') else
        rx_err_ctr_q + err_ctr_inc when (inc_one = '1' or inc_eight = '1') else
                    rx_err_ctr_dec;
      
   -- Clock enable: Tick error counter register when unit is transmitter and
   -- one of commands is valid!
   rx_err_ctr_ce <= '1' when (modif_rx_ctr = '1' and is_receiver = '1') else
                    '1' when (rx_err_ctr_pload = '1') else
                    '0';

   ----------------------------------------------------------------------------
   -- RX Error counter register
   ----------------------------------------------------------------------------
   rx_err_ctr_reg_proc : process(clk_sys, res_n, reset_err_counters)
   begin
       if (res_n = G_RESET_POLARITY or reset_err_counters = '1') then
           rx_err_ctr_q <= (OTHERS => '0');
       elsif (rising_edge(clk_sys)) then
           if (rx_err_ctr_ce = '1') then
               rx_err_ctr_q <= rx_err_ctr_q;
           end if;
       end if;
   end process;


   ----------------------------------------------------------------------------
   ----------------------------------------------------------------------------
   -- Error counters for Errors in Nominal, Data Bit rate
   ----------------------------------------------------------------------------
   ----------------------------------------------------------------------------

   -- Selection of counter to be incremented
   nom_dat_sel_ctr <= nom_err_ctr_q when (sp_control = NOMINAL_SAMPLE) else
                      data_err_ctr_q; 

   nom_dat_sel_ctr_add <= nom_dat_sel_ctr + 1;

   nom_err_ctr_d <= nom_dat_sel_ctr_add;
   data_err_ctr_d <= nom_dat_sel_ctr_add;

   -- Clock enables for counters, increment only
   nom_err_ctr_ce <= '1' when (inc_one = '1' or inc_eight = '1') and
                              (sp_control = NOMINAL_SAMPLE)
                         else
                     '0';
   
   data_err_ctr_ce <= '1' when (inc_one = '1' or inc_eight = '1') and
                               (sp_control = DATA_SAMPLE or 
                                sp_control = SECONDARY_SAMPLE)
                          else
                      '0';
   
   ----------------------------------------------------------------------------
   -- Nominal / Data Bit rate error counters registers
   ----------------------------------------------------------------------------
   nom_err_ctr_proc : process(clk_sys, res_n, reset_err_counters)
   begin
       if (res_n = G_RESET_POLARITY or reset_err_counters = '1') then
           nom_err_ctr_q <= (OTHERS => '0');
       elsif (rising_edge(clk_sys)) then
           if (nom_err_ctr_ce = '1') then
               nom_err_ctr_q <= nom_err_ctr_d;
           end if;
       end if;
   end process;

   dat_err_ctr_proc : process(clk_sys, res_n, reset_err_counters)
   begin
       if (res_n = G_RESET_POLARITY or reset_err_counters = '1') then
           data_err_ctr_q <= (OTHERS => '0');
       elsif (rising_edge(clk_sys)) then
           if (data_err_ctr_ce = '1') then
               data_err_ctr_q <= data_err_ctr_d;
           end if;
       end if;
   end process;

   ----------------------------------------------------------------------------
   -- Internal signals to output propagation
   ----------------------------------------------------------------------------
   rx_err_ctr  <= std_logic_vector(rx_err_ctr_q);
   tx_err_ctr  <= std_logic_vector(tx_err_ctr_q);

   norm_err_ctr <= std_logic_vector(nom_err_ctr_q);
   data_err_ctr <= std_logic_vector(data_err_ctr_q);

   ----------------------------------------------------------------------------
   -- Assertions
   ----------------------------------------------------------------------------

   -- psl default clock is rising_edge(clk_sys);
   --
   -- psl no_simul_inc_dec_asrt : assert never 
   --  (inc_one = '1' and inc_eight = '1') or
   --  (inc_one = '1' and dec_one = '1') or
   --  (dec_one = '1' and inc_eight = '1')
   -- report "Can't manipulate Error counters by multiple commands at once"
   -- severity error;

   -- psl no_simul_transm_rec_asrt : assert never
   --   (is_transmitter = '1' and is_receiver = '1')
   -- report "Unit can't be transmitter and receiver at once"
   -- severity error;

   -- psl rx_ctr_never_mt_262 : assert never
   --  (rx_err_ctr_q > 262)
   --  report "RX Error counter is bigger than 262, node should be Bus off!"
   --  severity error;

   -- psl tx_ctr_never_mt_262 : assert never
   --  (tx_err_ctr_q > 262)
   --  report "TX Error counter is bigger than 262, node should be Bus off!"
   --  severity error;

end architecture;