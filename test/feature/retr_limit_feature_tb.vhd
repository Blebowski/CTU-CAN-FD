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
--  Feature test for retransmitt limitation
--                                      
--------------------------------------------------------------------------------
-- Revision History:
--    30.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--    12.06.2018  Modified to use CAN Test lib instead of direct register
--                access functions.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

package retr_limit_feature is
  
    procedure retr_limit_feature_exec(
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


package body retr_limit_feature is
  
    procedure retr_limit_feature_exec(
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
        variable retr_th            :       natural;
        variable mode_backup        :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');

        variable mode               :       SW_mode := (false, false, false,
                                                false, true, false, false,
                                                false, false, false);
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable buf_state          :       SW_TXT_Buffer_state_type;
    begin
        outcome := true;

        ------------------------------------------------------------------------
        -- Set both nodes to forbid acknowledge
        ------------------------------------------------------------------------
        mode.acknowledge_forbidden := true;
        set_core_mode(mode, ID_2, mem_bus_2);
        set_core_mode(mode, ID_1, mem_bus_1);
        mode.acknowledge_forbidden := false;

        ------------------------------------------------------------------------
        -- Erase error counters node 1
        ------------------------------------------------------------------------
        set_error_counters(err_counters, ID_1, mem_bus_1);

        ------------------------------------------------------------------------
        -- Set Node 1 retransmitt limit
        ------------------------------------------------------------------------
        rand_int_v(rand_ctr, 15, retr_th);
        report "Retransmitt threshold: " & Integer'image(retr_th);
        CAN_enable_retr_limit(true, retr_th, ID_1, mem_bus_1);

        ------------------------------------------------------------------------
        -- Generate and send frame by Node 1
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);    
        CAN_frame.rtr := RTR_FRAME;
        CAN_frame.frame_format := NORMAL_CAN; 
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus_1, frame_sent);

        ------------------------------------------------------------------------
        -- Wait number of retransmissions. After each one, TXT Buffer should
        -- be back in ready. After last one, it should be in failed.
        ------------------------------------------------------------------------
        for i in 0 to retr_th loop
            CAN_wait_frame_sent(ID_1, mem_bus_1);
            get_tx_buf_state(1, buf_state, ID_1, mem_bus_1);
            if (i /= retr_th) then
                if (buf_state /= buf_ready) then
                    report "Buffer not ready";
                    outcome := false;
                    exit;
                end if;
            else
                if (buf_state /= buf_failed) then
                    report "Buffer not failed";
                    outcome := false;
                end if;
            end if;
        end loop;

        ------------------------------------------------------------------------
        -- Read TX Counter, it should be equal to 8 times number of retransmitts
        -- plus one original transmittion does not count as retransmittion.
        ------------------------------------------------------------------------
        read_error_counters(err_counters, ID_1, mem_bus_1);
        if (err_counters.tx_counter /= 8 * (retr_th + 1)) then
            report "Counters exp: " & Integer'Image(err_counters.tx_counter) &
                   " coutners real: " & Integer'image(8 * (retr_th + 1));
            outcome := false;
        end if;

        ------------------------------------------------------------------------
        -- Set node  2 to allow acknowledge again
        ------------------------------------------------------------------------
        set_core_mode(mode, ID_2, mem_bus_2);
        set_core_mode(mode, ID_1, mem_bus_1);

        wait for 40000 ns;
  end procedure;
  
end package body;
