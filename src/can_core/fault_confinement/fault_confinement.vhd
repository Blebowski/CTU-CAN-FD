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
--  Circuit handling Fault Confinement. Error counters increment is handled 
--  by signals inc_one, inc_eight, dec_one. RX TX counters for fault confinement
--  are availiable. Two more counters are availiable to distinguish between 
--  errors in Data phase and normal phase. All counters are pressetable from
--  driving bus. Treshold for signalling error warning limit and transition
--  to error_pssive are also parameters given by driving bus. Default values
--  are compliant with CAN FD standard.
--------------------------------------------------------------------------------
-- Revision History:
--    June 2015  Created file
--    19.6.2016  Modified counters for error couting in both FD and NORMAL mode.
--               Counters extended to 16 bits wide, to match the format in the 
--               registers!
--    27.6.2016  Bug fix. Changed error warning limit reached detection to greater
--               than and equal instead of only equal.
--    30.6.2016  Bug fix. Added equal or greater to fault confinement error 
--               passive state. According to CAN spec. error counter value equal
--               or greater than 128 is error passive, not only greater than!
--    05.1.2018  Added "erc_capt_r" register for last error capture.
--    08.5.2018  Added pragmas for one-hot decoding on increment, decrement
--               error counters.
--    12.7.2018  Added counters for erasing error counters upon reception of
--               128 consecutive 11 recessive bits as protocol compliant way
--               to transfer from Bus-off to Error active!
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

entity fault_confinement is 
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous reset
        res_n                  :in   std_logic;

        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- Driving Bus
        drv_bus                :in   std_logic_vector(1023 downto 0);
          
        -----------------------------------------------------------------------
        -- Error signalling for interrupts
        -----------------------------------------------------------------------
        -- Error passive state changed
        error_passive_changed  :out  std_logic;

        -- Error warning limit was reached
        error_warning_limit    :out  std_logic; 

        -----------------------------------------------------------------------
        -- Operation control Interface
        -----------------------------------------------------------------------
        -- Unit is transceiver
        is_transceiver         :in   std_logic;
        
        -- Unit is receiver
        is_receiver            :in   std_logic;
        
        -----------------------------------------------------------------------
        -- Protocol control Interface
        -----------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control             :in   std_logic_vector(1 downto 0);

        -- Increment error counter by 1 
        inc_one                :in   std_logic;
        
        -- Increment error counter by 8
        inc_eight              :in   std_logic;
        
        -- Decrement error counter by 1
        dec_one                :in   std_logic;

        -----------------------------------------------------------------------
        -- Fault confinement State indication
        -----------------------------------------------------------------------
        -- Unit is error active
        is_err_active           :in   std_logic;
        
        -- Unit is error passive
        is_err_passive          :in   std_logic;
        
        -- Unit is Bus-off
        is_bus_off              :in   std_logic;

        -----------------------------------------------------------------------
        -- Error counters
        -----------------------------------------------------------------------
        -- TX Error counter
        tx_counter_out          :out  std_logic_vector(8 downto 0);
        
        -- RX Error counter
        rx_counter_out          :out  std_logic_vector(8 downto 0);
        
        -- Error counter in Nominal Bit-rate
        err_counter_norm_out    :out  std_logic_vector(15 downto 0);
        
        -- Error counter in Data Bit-rate
        err_counter_fd_out      :out  std_logic_vector(15 downto 0)
    );
end entity;


architecture rtl of fault_confinement is
 
    ---------------------------------------------------------------------------
    -- Driving bus aliases
    ---------------------------------------------------------------------------
    signal drv_ewl                   :     std_logic_vector(7 downto 0);
    signal drv_erp                   :     std_logic_vector(7 downto 0);
    signal drv_ctr_val               :     std_logic_vector(8 downto 0);
    signal drv_ctr_sel               :     std_logic_vector(3 downto 0);
    signal drv_clr_err_ctrs          :     std_logic;

begin
  
    ---------------------------------------------------------------------------
    -- Driving bus aliases
    ---------------------------------------------------------------------------
    drv_ewl             <=  drv_bus(DRV_EWL_HIGH downto DRV_EWL_LOW);
    drv_erp             <=  drv_bus(DRV_ERP_HIGH downto DRV_ERP_LOW);
    drv_ctr_val         <=  drv_bus(DRV_CTR_VAL_HIGH downto DRV_CTR_VAL_LOW);
    drv_ctr_sel         <=  drv_bus(DRV_CTR_SEL_HIGH downto DRV_CTR_SEL_LOW);
    drv_clr_err_ctrs    <=  drv_bus(DRV_ERR_CTR_CLR);

    ---------------------------------------------------------------------------
    -- Error counters to output propagation
    ---------------------------------------------------------------------------
    tx_counter_out          <=  std_logic_vector(to_unsigned(tx_counter, 9));
    rx_counter_out          <=  std_logic_vector(to_unsigned(rx_counter, 9));

    err_counter_norm_out    <=  std_logic_vector(to_unsigned(
                                    err_counter_norm, 16));
    err_counter_fd_out      <=  std_logic_vector(to_unsigned(
                                    err_counter_fd, 16));

end architecture;