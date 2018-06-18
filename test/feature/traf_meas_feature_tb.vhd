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
--  Traffic measurment feature test implementation.
--
--  Test sequence:
--  	1. Generate random number N from 0 to 5
--  	2. Measure TX counter of node 1 and RX counter of node 2
--  	3. Send N random frames
--  	4. Measure TX counter of node 1 and RX counter of node 2 again
--  	5. Compare if both counters were increased by N
--
--------------------------------------------------------------------------------
-- Revision History:
--
--    24.6.2016   Created file
--    11.6.2018   Modified to use CAN Test lib instead of direct register
--                acccess.
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

package traf_meas_feature is
    procedure traf_meas_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );

end package;


package body traf_meas_feature is
    procedure traf_meas_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :       natural := 1;
        variable ID_2               :       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable rand_value         :       natural;

        variable ctr_1_1            :       SW_traffic_counters;
        variable ctr_1_2            :       SW_traffic_counters;
        variable ctr_2_1            :       SW_traffic_counters;
        variable ctr_2_2            :       SW_traffic_counters;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Check the TX RX counters
        ------------------------------------------------------------------------
        read_traffic_counters(ctr_1_1, ID_1, mem_bus(1));
        read_traffic_counters(ctr_1_2, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Generate the CAN frames to send
        ------------------------------------------------------------------------
        rand_int_v(rand_ctr, 5, rand_value);
        for i in 0 to rand_value - 1 loop
            CAN_generate_frame(rand_ctr, CAN_frame);
            CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
            CAN_wait_frame_sent(ID_1, mem_bus(1));
        end loop;

        ------------------------------------------------------------------------
        -- Check the TX RX counters
        ------------------------------------------------------------------------
        read_traffic_counters(ctr_2_1, ID_1, mem_bus(1));
        read_traffic_counters(ctr_2_2, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Check That TX counters were increased accordingly
        ------------------------------------------------------------------------
        if (ctr_1_1.tx_frames + rand_value /= ctr_2_1.tx_frames) then
            o.outcome := false;
        end if;

        ------------------------------------------------------------------------
        -- Check That RX counters were increased accordingly
        ------------------------------------------------------------------------
        if (ctr_1_2.rx_frames + rand_value /= ctr_2_2.rx_frames) then
            o.outcome := false;
        end if;
    end procedure;

end package body;
