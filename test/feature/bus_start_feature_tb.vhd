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
--  Bus Start feature test. Verifies if unit is capable of joining fully,
--  loaded bus.
--
--  Test sequence:
--      1. Disable both Nodes.
--      2. Generate CAN Frame with ID 10, and insert it to all 4 TXT buffers of
--         Node 1.
--      3. Generate ACK for Node 1 by forcing bus level to be dominant during
--         ACK Slot. This is necessary for 11 consecutive recessive bits:
--          ACK Delim + 7 EOF + 3 Interframe
--      4. Enable Node 1, wait for some time until its integration ends.
--      5. Now, Node 1 is transmitting. Generate CAN Frame with lower ID than
--         10 (e.g. 9) and insert it to Node 2.
--      6. Enable Node 2. Node 2 should finish its integration during 11 bits
--         after frame from Node 1, start transmitting next frame, and win
--         arbitration since it has lower ID. Note that it is guaranteed that
--         if the unit does not join the integration, next frame will be tran-
--         smitted by Node 1, since it has 4 frames available! Thus it is
--         guaranteed that there will be minimal necessary gap for integration
--         which must be enough for Node 2!
--      7. Wait until Node 1 transmitts its second frame!
--      
--------------------------------------------------------------------------------
-- Revision History:
--    30.8.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.can_constants.all;
USE work.CANtestLib.All;
use work.CAN_FD_frame_format.all;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

use work.CAN_FD_register_map.all;

package bus_start_feature is
    procedure bus_start_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body bus_start_feature is
    procedure bus_start_feature_exec(
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
        variable rx_state           :       SW_RX_Buffer_info;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Disable both controllers
        ------------------------------------------------------------------------
        CAN_turn_controller(false, ID_1, mem_bus(1));
        CAN_turn_controller(false, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Generate frame, hardcode ID to 514, Identifier type to BASE
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.ident_type := BASE;
        CAN_frame.identifier := 514;

        ------------------------------------------------------------------------
        -- Insert frame to 2 TXT Buffers of Node 1
        ------------------------------------------------------------------------
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_send_frame(CAN_frame, 2, ID_1, mem_bus(1), frame_sent);

        ------------------------------------------------------------------------
        -- Turn on Node 1, Wait until it starts transmitting
        ------------------------------------------------------------------------
        report "Turning ON Node 1.";
        CAN_turn_controller(true, ID_1, mem_bus(1));
        while (protocol_type'VAL(to_integer(unsigned(
                iout(1).stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))
                /= sof)
        loop
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;

        ------------------------------------------------------------------------
        -- Modify frame to have ID 513, and insert it to Node 2 for transmission.
        -- Enable Node 2, so that it may start integration.
        ------------------------------------------------------------------------
        report "Turning ON Node 2.";
        wait_rand_cycles(rand_ctr, mem_bus(1).clk_sys, 10, 11);
        CAN_frame.identifier := 513;
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_turn_controller(true, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Generate ACK for Node 1, so that there will be for sure only 11
        -- consecutive recessive bits.
        ------------------------------------------------------------------------
        while (protocol_type'VAL(to_integer(unsigned(
                iout(1).stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))
                /= delim_ack)
        loop
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;

        -- Assuming default timing configuration -> 200 cycles per Bit
        -- Wait through CRC Delimiter + little bit
        report "Started delim_ack";
        wait_rand_cycles(rand_ctr, mem_bus(1).clk_sys, 209, 210);
        report "Forcing ACK";
        so.bl_inject <= DOMINANT;
        so.bl_force  <= true;
        wait_rand_cycles(rand_ctr, mem_bus(1).clk_sys, 200, 201);
        so.bl_force <= false;
        so.bl_inject <= RECESSIVE;

        ------------------------------------------------------------------------
        -- Wait until bus is idle.
        ------------------------------------------------------------------------
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        report "Bus is idle";

        ------------------------------------------------------------------------
        -- Now wait until another frame is transmitted. This one should be
        -- arbitrated, and Node 2 should have won.
        ------------------------------------------------------------------------
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        report "Second frame was sent!";

        ------------------------------------------------------------------------
        -- Read status of RX Buffer in Node 1! It should have sent ACK and
        -- Received frame from Node 2 with ID = 513!
        -- If not, Node 2 did not manage to get out of Integration during
        -- minimal necessary time, and test fails!
        --
        -- Note that IDs were selected so that there would be RECESSIVE bit
        -- in second bit of ID! This is to avoid error frame in case when
        -- Node 2 locks after SOF bit (according to standard it can skip SOF,
        -- if it detects DOMINANT during Interrmission), and thus does not
        -- send Stuff bit as Node 1. Due to this, CAN FD standard does not
        -- increment error counters upon stuff error in Arbitration!
        -- After Error Frame, both nodes have frame available and both start
        -- transmission of SOF, so this error will not cause deadlock!
        -- 
        -- CTU CAN FD tries to be smarter, not skip SOF, but move Protocol
        -- Control to SOF upon hard sync edge though! This thing is verified
        -- in special feature test!
        ------------------------------------------------------------------------
        get_rx_buf_state(rx_state, ID_1, mem_bus(1));
        report "Read RX Buffer state";

        if (rx_state.rx_empty) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "RX Buffer is empty, but Frame should be received!"
                severity error;
            -- LCOV_EXCL_STOP
        end if;

        CAN_frame.identifier := 0;
        CAN_read_frame(CAN_frame, ID_1, mem_bus(1));
        if (CAN_frame.identifier /= 513) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "Wrong Identifier received by Node 1. Expected: 513 , Real: " &
                    integer'image(CAN_frame.identifier) severity error;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Now wait until Node 1 transmitts frame which lost the arbitration
        -- so that we leave test environmnet withou pending frames in TXT
        -- Buffers.
        ------------------------------------------------------------------------
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        report "Last frame was sent!";

  end procedure;

end package body;
