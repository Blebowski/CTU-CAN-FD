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
--  Test of Suspend transmission field of Interframe space.
--
--  Test sequence:
--      1. Set Node 1 to be error passive by configuring TX Error counter
--         to be 128. Set Node 2 to be error active by configuring both
--         Error counters to be 0.
--      2. Read out error state of both nodes. Check that Node 1 is error 
--         passive, and Node 2 is error active.
--      Loop (Forever):
--          3. Insert frame to two TXT Buffers of Node 1. Insert frame to Node 2.
--          4. Wait until Node 1 starts transmission.
--          5. Insert frame to Node 2.
--          6. Wait until frame is transmitted by Node 1 (end of EOF).
--          7. Wait for 3 Bit times, to compensate for Intermission field!
--          8. Wait for N Bit times (suspend field).
--          9. Give command to Node 2 to start Transmission.
--         10. Check that Node 1 did become receiver, not transmitter although
--             it has frame for transmission available. It can become transmi-
--             tter only if we waited more than 7 bits! Break out of loop in
--             this case and check that N=8!
--
--  Just found out that Gedit supports insertion of emojis. Lets see how
--   this is gonna work... 🙄🤔😳🤤
--
--------------------------------------------------------------------------------
-- Revision History:
--    2.9.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
use work.CAN_FD_frame_format.all;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

use work.CAN_FD_register_map.all;

package suspend_transmission_feature is
    procedure suspend_transmission_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body suspend_transmission_feature is
    procedure suspend_transmission_feature_exec(
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
        variable error_ctrs         :       SW_error_counters :=
                                            (0, 0, 0, 0);
        variable fault_state        :       SW_fault_state := fc_error_active;
        variable n                  :       integer := 0;
        variable status             :       SW_status;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Force Node 1 to error passive and Check it
        ------------------------------------------------------------------------
        error_ctrs.rx_counter := 150;
        set_error_counters(error_ctrs, ID_1, mem_bus(1));        
        wait for 30 ns;
        get_fault_state(fault_state, ID_1, mem_bus(1));
        if (fault_state /= fc_error_passive) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "Node 1 not Error passive as expected!"
                severity error;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Force Node 2 to error Active and Check it!
        ------------------------------------------------------------------------
        error_ctrs.rx_counter := 0;
        set_error_counters(error_ctrs, ID_2, mem_bus(2));        
        wait for 30 ns;
        get_fault_state(fault_state, ID_2, mem_bus(2));
        if (fault_state /= fc_error_active) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "Node 2 not Error Active as expected!"
                severity error;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Generate frame, hardcode ID to 514, Identifier type to BASE
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.ident_type := BASE;
        CAN_frame.identifier := 514;

        while (true) loop

            --------------------------------------------------------------------
            -- Send two frames from Node 1. First will start transmission.
            -- Then Add the same frame to Node 2. 
            --------------------------------------------------------------------
            CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
            CAN_send_frame(CAN_frame, 2, ID_1, mem_bus(1), frame_sent);
            report "Inserted 2 frames to Node 1";

            wait for 500 ns;
            CAN_insert_TX_frame(CAN_frame, 1, ID_2, mem_bus(2));
            report "Inserted 1 frame to Node 2";

            --------------------------------------------------------------------
            -- Wait until the end of EOF field
            --------------------------------------------------------------------        
            while (protocol_type'VAL(to_integer(unsigned(
                    iout(1).stat_bus(STAT_PC_STATE_HIGH downto
                                        STAT_PC_STATE_LOW))))
                    /= eof)
            loop
                wait until rising_edge(mem_bus(1).clk_sys);
            end loop;

            while (protocol_type'VAL(to_integer(unsigned(
                    iout(1).stat_bus(STAT_PC_STATE_HIGH downto
                                        STAT_PC_STATE_LOW))))
                    = eof)
            loop
                wait until rising_edge(mem_bus(1).clk_sys);
            end loop;
            report "End of EOF field";
            wait for 20 ns;

            -- Wait for N + 3 Bit times. 3 is length of intermission,
            -- remaining N bits are prolongging Suspend field!
            CAN_wait_n_bits(n + 3, true, ID_1, mem_bus(1));
            report "Waited " & integer'image(n) & " bits";

            -- Give command to the frame in Node 2 for transmission!
            send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));

            -- Wait until frame transmission starts on Node 1!
            while (protocol_type'VAL(to_integer(unsigned(
                    iout(1).stat_bus(STAT_PC_STATE_HIGH downto
                                        STAT_PC_STATE_LOW))))
                    /= sof)
            loop
                wait until rising_edge(mem_bus(1).clk_sys);
            end loop;
            report "Frame started";
            wait for 20 ns;

            -- Check Operational State of Node 1!
            get_controller_status(status, ID_1, mem_bus(1));
            if (status.receiver) then
                if (n < 8) then
                    report "Unit turned receiver on bit: " & integer'image(n);
                else
                    -- LCOV_EXCL_START
                    o.outcome := false;
                    report "Unit turned receiver after 8 bit Suspend"
                        severity error;
                    -- LCOV_EXCL_STOP
                end if;
            elsif (status.transmitter) then
                if (n < 8) then
                    -- LCOV_EXCL_START
                    o.outcome := false;
                    report "Suspend transmission shorter than 8 bits!"
                        severity error;
                    -- LCOV_EXCL_STOP
                else
                    report "Suspend transmission equal to 8 bits!";
                    exit;
                end if;
            else
                -- LCOV_EXCL_START
                o.outcome := false;
                report "Unit in SOF, but not Transceiver nor Receiver!"
                    severity error;
                -- LCOV_EXCL_STOP
            end if;
            n := n + 1;

            -- Wait until unit frame is finished (Now should be by Node 2)!
            CAN_wait_bus_idle(ID_1, mem_bus(1));

            -- Node 1 still has one frame for transmission (the one which was
            -- suspended)
            CAN_wait_frame_sent(ID_1, mem_bus(1));

        end loop;


        ------------------------------------------------------------------------
        -- Now wait until Node 1 transmitts frame which lost the arbitration
        -- so that we leave test environmnet withou pending frames in TXT
        -- Buffers.
        ------------------------------------------------------------------------
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        report "Last frame was sent!";

  end procedure;

end package body;
