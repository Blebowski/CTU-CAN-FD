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

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

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
                                                false, true, false,
                                                false, false, false);
        variable ctr_1_1            :       SW_traffic_counters;
        variable ctr_1_2            :       SW_traffic_counters;
        variable ctr_2_1            :       SW_traffic_counters;
        variable ctr_2_2            :       SW_traffic_counters;
        variable pc_state           :       SW_PC_Debug;
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
        -- Wait until Node 2 is in CRC Delimiter. Transmitted bit might still
        -- be DOMINANT since last bit of CRC might be dominant. TX Bit is
        -- updated in SYNC segment. Wait for rising edge on bus_level if so!
        ------------------------------------------------------------------------
        CAN_wait_pc_state(pc_deb_crc_delim, ID_2, mem_bus(2));
        if (bus_level = DOMINANT) then
            wait until rising_edge(bus_level);
        end if;

        ------------------------------------------------------------------------
        -- Now monitor the bus level to see if it is recessive during whole
        -- CRC Delimim, ACK and ACK Delim. Monitor always on reciever! IN FD
        -- transciever workaround is used for state switching in TX trigger 
        -- just slightly delayed!!!
        ------------------------------------------------------------------------
        CAN_read_pc_debug(pc_state, ID_2, mem_bus(2));
        while (pc_state = pc_deb_crc_delim or pc_state = pc_deb_ack or 
               pc_state = pc_deb_ack_delim)
        loop
            check(bus_level = RECESSIVE,
                  "CRC Delim, ACK and ACK Delim should be recessive now!");
            CAN_read_pc_debug(pc_state, ID_2, mem_bus(2));
        end loop;

        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Check the TX RX counters
        ------------------------------------------------------------------------
        read_traffic_counters(ctr_2_1, ID_1, mem_bus(1));
        read_traffic_counters(ctr_2_2, ID_2, mem_bus(2));

        check(ctr_1_1.tx_frames + 1 = ctr_2_1.tx_frames,
              "TX Frames counter incremented unexpectedly!");

        check(ctr_1_2.rx_frames + 1 = ctr_2_2.rx_frames,
              "RX Frames counter incremented unexpectedly!");

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
        -- Wait until node 2 is in CRC Delimiter field since bus is delayed we
        -- have to wait until the first rising edge on income data!
        ------------------------------------------------------------------------
        CAN_wait_pc_state(pc_deb_crc_delim, ID_2, mem_bus(2));
        if (bus_level = DOMINANT) then
            wait until rising_edge(bus_level);
        end if;

        ------------------------------------------------------------------------
        -- Now monitor the bus level to see if it is recessive during whole
        -- CRC Delim, ACK and ACK Delim field.
        ------------------------------------------------------------------------
        while (pc_state = pc_deb_crc_delim or pc_state = pc_deb_ack or 
               pc_state = pc_deb_ack_delim)
        loop
            check(bus_level = RECESSIVE,
                  "CRC Delim, ACK and ACK Delim should be recessive now!");
            CAN_read_pc_debug(pc_state, ID_2, mem_bus(2));
        end loop;
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Check the TX RX counters
        ------------------------------------------------------------------------
        read_traffic_counters(ctr_2_1, ID_1, mem_bus(1));
        read_traffic_counters(ctr_2_2, ID_2, mem_bus(2));

        check(ctr_1_1.tx_frames + 2 = ctr_2_1.tx_frames,
              "TX Frames counter incremented unexpectedly!");

        check(ctr_1_2.rx_frames + 2 = ctr_2_2.rx_frames,
              "RX Frames counter incremented unexpectedly!");

  end procedure;

end package body;