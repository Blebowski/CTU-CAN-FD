--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Project advisors and co-authors:
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--  Feature test for setting error counters from user and its appropriate fault
--  confinement state manipulation!
--
--------------------------------------------------------------------------------
-- Revision History:
--
--    30.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.CAN_FD_register_map.all;

package fault_confinement_feature is

    procedure fault_confinement_feature_exec(
        variable    outcome         : inout boolean;
        signal      rand_ctr        : inout natural range 0 to RAND_POOL_SIZE;
        signal      mem_bus_1       : inout Avalon_mem_type;
        signal      mem_bus_2       : inout Avalon_mem_type;
        signal      bus_level       : in    std_logic;
        signal      drv_bus_1       : in    std_logic_vector(1023 downto 0);
        signal      drv_bus_2       : in    std_logic_vector(1023 downto 0);
        signal      stat_bus_1      : in    std_logic_vector(511 downto 0);
        signal      stat_bus_2      : in    std_logic_vector(511 downto 0)
    );

end package;


package body fault_confinement_feature is

    procedure fault_confinement_feature_exec(
        variable    outcome         : inout boolean;
        signal      rand_ctr        : inout natural range 0 to RAND_POOL_SIZE;
        signal      mem_bus_1       : inout Avalon_mem_type;
        signal      mem_bus_2       : inout Avalon_mem_type;
        signal      bus_level       : in    std_logic;
        signal      drv_bus_1       : in    std_logic_vector(1023 downto 0);
        signal      drv_bus_2       : in    std_logic_vector(1023 downto 0);
        signal      stat_bus_1      : in    std_logic_vector(511 downto 0);
        signal      stat_bus_2      : in    std_logic_vector(511 downto 0)
    )is
        variable r_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ctr_1              :       natural;
        variable ctr_2              :       natural;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable rand_val           :       real;
        variable th_1               :       natural := 0;
        variable rxc                :       natural := 0;
        variable txc                :       natural := 0;

        variable err_counters       :       SW_error_counters;
        variable fault_th           :       SW_fault_thresholds := (0, 0);
        variable fault_th_2         :       SW_fault_thresholds := (0, 0);
        variable fault_state        :       SW_fault_state;
    begin
        outcome := true;

        ------------------------------------------------------------------------
        -- Generate random setting of ERP treshold and RX counters to preset
        ------------------------------------------------------------------------
        rand_real_v(rand_ctr, rand_val);
        fault_th.erp := integer(rand_val * 254.0);

        rand_real_v(rand_ctr, rand_val);
        err_counters.rx_counter := integer(rand_val * 257.0);

        rand_real_v(rand_ctr,rand_val);
        err_counters.tx_counter := integer(rand_val * 257.0);


        ------------------------------------------------------------------------
        -- Set the counter and tresholds
        ------------------------------------------------------------------------
        set_error_counters(err_counters, ID_1, mem_bus_1);
        set_fault_thresholds(fault_th, ID_1, mem_bus_1);


        ------------------------------------------------------------------------
        -- Read counters back
        ------------------------------------------------------------------------
        get_fault_thresholds(fault_th_2, ID_1, mem_bus_1);

        if (fault_th.ewl /= fault_th_2.ewl) then
            outcome := false;
        end if;

        if (fault_th.erp /= fault_th_2.erp) then
            outcome := false;
        end if;

        ------------------------------------------------------------------------
        -- Read fault confinement state
        ------------------------------------------------------------------------
        get_fault_state(fault_state, ID_1, mem_bus_1);

        if (err_counters.tx_counter > 255 or
            err_counters.rx_counter > 255)
        then
            if (fault_state /= fc_bus_off) then
                outcome := false;
            end if;
        elsif (err_counters.tx_counter < fault_th.ewl and
               err_counters.rx_counter < fault_th.ewl)
        then
            if (fault_state /= fc_error_active) then
                outcome := false;
            end if;
        else
          if (fault_state /= fc_error_passive) then
                outcome := false;
            end if;
        end if;

    end procedure;

end package body;
