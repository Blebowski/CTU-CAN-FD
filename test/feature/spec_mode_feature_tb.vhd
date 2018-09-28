--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
-- 
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <jerabma7@fel.cvut.cz>
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
--  Special modes feature testbench. Verifies behaviour of Self-test mode,
--  Acknowledge forbidden mode and Listen only mode.
--
--  Test sequence:
--      1. Part 1:
--          1.1 Set STM in Node 1, STM and ACF in Node 2.
--          1.2 Read traffic counters in both nodes.
--          1.3 Start Transmission by Node 1.
--          1.4 Wait until "delim_ack" protocol state of Node 2.
--          1.5 Monitor that all three bits of this state (CRC Delim, ACK,
--              ACK Delim) are recessive on bus (no ACK is sent).
--          1.6 Wait till end of frame. Read traffic counters again.
--          1.7 Check that TX counter was incremented in Node 1 and RX counter
--              was incremented in Node 2.
--      2. Part 2:
--          1.1 Set STM in Node 1, LOM in Node 2.
--          1.2 Read traffic counters in both nodes.
--          1.3 Start Transmission by Node 1.
--          1.4 Wait until "delim_ack" protocol state of Node 2.
--          1.5 Monitor that all three bits of this state (CRC Delim, ACK,
--              ACK Delim) are recessive on bus (no ACK is sent).
--          1.6 Wait till end of frame. Read traffic counters again.
--          1.7 Check that TX counter was incremented in Node 1 and RX counter
--              was incremented in Node 2.
--
--------------------------------------------------------------------------------
-- Revision History:
--    24.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--     12.6.2018  Modified to use CAN Test lib functions instead of direct
--                register access.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

use work.CAN_FD_register_map.all;

package spec_mode_feature is
    procedure spec_mode_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body spec_mode_feature is
    procedure spec_mode_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable mode               :       SW_mode := (false, false, false,
                                                false, true, false, false,
                                                false, false, false);
        variable ctr_1_1            :       SW_traffic_counters;
        variable ctr_1_2            :       SW_traffic_counters;
        variable ctr_2_1            :       SW_traffic_counters;
        variable ctr_2_2            :       SW_traffic_counters;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Part 1
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Set STM in node 1 and STM, ACF in node 2
        ------------------------------------------------------------------------
        mode.self_test := true;
        set_core_mode(mode, ID_1, mem_bus(1));

        mode.acknowledge_forbidden := true;
        set_core_mode(mode, ID_2, mem_bus(2));

        mode.self_test := false;
        mode.acknowledge_forbidden := false;

        ------------------------------------------------------------------------
        -- Check the TX RX counters
        ------------------------------------------------------------------------
        read_traffic_counters(ctr_1_1, ID_1, mem_bus(1));
        read_traffic_counters(ctr_1_2, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Send frame by node 1
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        ------------------------------------------------------------------------
        -- Wait until one of the nodes is in ack field plus one more clock
        -- cycles since after CRC we are in ack_delim immediately, thus bus
        -- level can still be last bit of CRC which can be dominant!
        ------------------------------------------------------------------------
        while (protocol_type'VAL(to_integer(unsigned(
               iout(2).stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))
               /= delim_ack)
        loop
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;
        if (bus_level = DOMINANT) then
            wait until rising_edge(bus_level);
        end if;

        ------------------------------------------------------------------------
        -- Now monitor the bus level to see if it is recessive during whole
        -- acknowledge field. Monitor always on reciever! IN FD transciever
        -- workaround is used for state switching in TX trigger just slightly
        -- delayed!!!
        ------------------------------------------------------------------------
        while (protocol_type'VAL(to_integer(unsigned(
               iout(2).stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))
               = delim_ack)
        loop
            wait until rising_edge(mem_bus(1).clk_sys);
            if (bus_level = DOMINANT) then
                -- LCOV_EXCL_START
                o.outcome := false;
                report "ACK detected when there should be no ACK!"
                    severity error;
                -- LCOV_EXCL_STOP
            end if;
        end loop;

        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Check the TX RX counters
        ------------------------------------------------------------------------
        read_traffic_counters(ctr_2_1, ID_1, mem_bus(1));
        read_traffic_counters(ctr_2_2, ID_2, mem_bus(2));

        if (ctr_1_1.tx_frames + 1 /= ctr_2_1.tx_frames) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "TX Frames counter incremented unexpectedly!"
                severity error;
            -- LCOV_EXCL_STOP
        end if;

        if (ctr_1_2.rx_frames + 1 /= ctr_2_2.rx_frames) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "RX Frames counter incremented unexpectedly!"
                severity error;
            -- LCOV_EXCL_STOP
        end if;


        ------------------------------------------------------------------------
        -- Part 2
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Set STM in node 1 and LOM mode in Node 2. Thisway node 1 does not
        -- expect acknowledge and node 2 reroutes the acknowledge to itself
        -- internally so it gets the acknowledge from itself but it is not on
        -- the bus!
        ------------------------------------------------------------------------
        mode.self_test := true;
        set_core_mode(mode, ID_1, mem_bus(1));
        mode.self_test := false;

        mode.listen_only := true;
        set_core_mode(mode, ID_2, mem_bus(2));
        mode.listen_only := false;

        ------------------------------------------------------------------------
        -- Send frame by node 1
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        ------------------------------------------------------------------------
        -- Wait until node 2 is in ack field Since bus is delayed we have to
        -- wait until the first rising edge on income data!
        ------------------------------------------------------------------------
        while (protocol_type'VAL(to_integer(unsigned(
                iout(2).stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))
                /= delim_ack)
        loop
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;

        if (bus_level = DOMINANT) then
            wait until rising_edge(bus_level);
        end if;

        ------------------------------------------------------------------------
        -- Now monitor the bus level to see if it is recessive during whole
        -- acknowledge field.
        ------------------------------------------------------------------------
        while (protocol_type'VAL(to_integer(unsigned(
                iout(2).stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))
                = delim_ack)
        loop
            wait until rising_edge(mem_bus(1).clk_sys);
            if (bus_level = DOMINANT) then
                o.outcome := false;
            end if;
        end loop;

        CAN_wait_bus_idle(ID_1,mem_bus(1));

        ------------------------------------------------------------------------
        -- Check the TX RX counters
        ------------------------------------------------------------------------
        read_traffic_counters(ctr_2_1, ID_1, mem_bus(1));
        read_traffic_counters(ctr_2_2, ID_2, mem_bus(2));

        if (ctr_1_1.tx_frames + 2 /= ctr_2_1.tx_frames) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "TX Frames counter incremented unexpectedly!"
                severity error;
            -- LCOV_EXCL_STOP
        end if;

        if (ctr_1_2.rx_frames + 2 /= ctr_2_2.rx_frames) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "RX Frames counter incremented unexpectedly!"
                severity error;
            -- LCOV_EXCL_STOP
        end if;

  end procedure;

end package body;
