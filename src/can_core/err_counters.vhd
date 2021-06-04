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
--  Error counters.
--
-- Purpose:
--  Error counters for fault confinement. Following counters are implemented:
--    RX Error counter - Counts errors of reciver.
--    TX Error counter - Counts errors of transmitter.
--    Nominal Error counter - Counts errors in Nominal Bit rate.
--    Data Error counter - Counts errors in Data Bit rate.
--  Only RX Error counter and TX Error counter are used for Fault confinement.
--  Nominal and Data Error counter are used to distuiguish relative error rate
--  of both bit-rates and are always incremented by 1 when Error frame is
--  transmitted. All counters can be modified from Driving Bus.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity err_counters is
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
        scan_enable            : in  std_logic;

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
end entity;

architecture rtl of err_counters is

    -- Error counter registers
    signal tx_err_ctr_d     : unsigned(8 downto 0);
    signal rx_err_ctr_d     : unsigned(8 downto 0);
    signal tx_err_ctr_q     : unsigned(8 downto 0);
    signal rx_err_ctr_q     : unsigned(8 downto 0);
    
    signal rx_err_ctr_inc   : unsigned(8 downto 0);
    signal rx_err_ctr_sat   : unsigned(8 downto 0);

    -- Clock enables for error counter registers
    signal tx_err_ctr_ce    : std_logic;
    signal rx_err_ctr_ce    : std_logic;

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
    signal data_err_ctr_d    : unsigned(15 downto 0);
    signal nom_err_ctr_q     : unsigned(15 downto 0);
    signal data_err_ctr_q    : unsigned(15 downto 0);
    
    -- Selected value of counter
    signal nom_dat_sel_ctr   : unsigned(15 downto 0);
    
    -- Combinationally added value
    signal nom_dat_sel_ctr_add : unsigned(15 downto 0);
    
    -- Clock enables for error counter registers
    signal nom_err_ctr_ce  : std_logic;
    signal data_err_ctr_ce : std_logic;
    
    signal res_err_ctrs_d       : std_logic;
    signal res_err_ctrs_q       : std_logic;
    signal res_err_ctrs_q_scan  : std_logic;

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

    ---------------------------------------------------------------------------
    -- Registering counter reset (to avoid glitches)
    ---------------------------------------------------------------------------
    res_err_ctrs_d <= '0' when (res_n = '0' or set_err_active = '1')
                          else
                      '1';

    rst_reg_inst : entity ctu_can_fd_rtl.dff_arst
    generic map(
        G_RESET_POLARITY   => '0',
        
        -- Reset to the same value as is polarity of reset so that other DFFs
        -- which are reset by output of this one will be reset too!
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,                -- IN
        clk                => clk_sys,              -- IN
        input              => res_err_ctrs_d,       -- IN

        output             => res_err_ctrs_q        -- OUT
    );
    
    ---------------------------------------------------------------------------
    -- Gate reset in scan mode
    ---------------------------------------------------------------------------
    mux2_res_tst_inst : entity ctu_can_fd_rtl.mux2
    port map(
        a                  => res_err_ctrs_q, 
        b                  => '1',
        sel                => scan_enable,

        -- Output
        z                  => res_err_ctrs_q_scan
    );

   ----------------------------------------------------------------------------
   -- TX Error counter register
   ----------------------------------------------------------------------------
   tx_err_ctr_reg_proc : process(clk_sys, res_err_ctrs_q_scan)
   begin
       if (res_err_ctrs_q_scan = '0') then
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
                      (rx_err_ctr_q - 1) when (rx_err_ctr_q > 0) else
                            rx_err_ctr_q;


   -- Inrement RX counter
   rx_err_ctr_inc <= rx_err_ctr_q + err_ctr_inc;
   
   -- Saturate RX counter when overflow would occur according to 12.1.4.3
   -- of CAN FD ISO spec
   rx_err_ctr_sat <= (OTHERS => '1') when (rx_err_ctr_inc < rx_err_ctr_q) else
                      rx_err_ctr_inc;

   ----------------------------------------------------------------------------
   -- Next value for error counter increment when any of "inc" commands is
   -- valid. Decrement otherwise!
   ----------------------------------------------------------------------------
   rx_err_ctr_d <=
             unsigned(drv_ctr_val) when (rx_err_ctr_pload = '1') else
                    rx_err_ctr_sat when (inc_one = '1' or inc_eight = '1') else
                    rx_err_ctr_dec;
      
   -- Clock enable: Tick error counter register when unit is transmitter and
   -- one of commands is valid!
   rx_err_ctr_ce <= '1' when (modif_rx_ctr = '1' and is_receiver = '1') else
                    '1' when (rx_err_ctr_pload = '1') else
                    '0';

   ----------------------------------------------------------------------------
   -- RX Error counter register
   ----------------------------------------------------------------------------
   rx_err_ctr_reg_proc : process(clk_sys, res_err_ctrs_q_scan)
   begin
       if (res_err_ctrs_q_scan = '0') then
           rx_err_ctr_q <= (OTHERS => '0');
       elsif (rising_edge(clk_sys)) then
           if (rx_err_ctr_ce = '1') then
               rx_err_ctr_q <= rx_err_ctr_d;
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
   nom_err_ctr_proc : process(clk_sys, res_err_ctrs_q_scan)
   begin
       if (res_err_ctrs_q_scan = '0') then
           nom_err_ctr_q <= (OTHERS => '0');
       elsif (rising_edge(clk_sys)) then
           if (nom_err_ctr_ce = '1') then
               nom_err_ctr_q <= nom_err_ctr_d;
           end if;
       end if;
   end process;

   dat_err_ctr_proc : process(clk_sys, res_err_ctrs_q_scan)
   begin
       if (res_err_ctrs_q_scan = '0') then
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

   -- <RELEASE_OFF>
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
   
   -- psl err_ctrs_inc_one_cov : cover
   --   {inc_one = '1'};
   
   -- psl err_ctrs_inc_eight_cov : cover
   --   {inc_eight = '1'};
   
   -- psl err_ctrs_dec_one_cov : cover
   --   {dec_one = '1'};

   -- psl err_ctrs_rec_saturation : cover
   --   {(rx_err_ctr_inc < rx_err_ctr_q) and rx_err_ctr_ce = '1'};

   -- <RELEASE_ON>

end architecture;