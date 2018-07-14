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
--  Feature test for RTR preferred behaviour of CTU CAN FD. RTR preffered
--  behaviour determines whether inserted DLC will be sent as part of RTR frame
--  or all zeroes will be sent in RTR Frame.
--
--  Test sequence is like so:
--      1. Part 1:
--          1.1 Generate frame with arbitrary DLC and force RTR flag
--          1.2 Configure RTR behaviour to send all zeroes.
--          1.3 Send frame by Node 1.
--          1.4 Wait until Node 2 receives the frame. Read frame from Node 2
--          1.5 Check if received DLC is 0.
--      2. Part 2:
--          2.1 Configure RTR behaviour to send the original value
--          2.2 Send frame by Node 1.
--          2.3 Wait until Node 2 receives the frame. Read frame from Node 2
--          2.4 Check if received DLC is matching transmitted DLC.
--
--------------------------------------------------------------------------------
-- Revision History:
--
--    23.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--     12.6.2018  Updated to use CAN Test library functions instead of direct
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
use work.CAN_FD_frame_format.all;

package rtr_pref_feature is
    procedure rtr_pref_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body rtr_pref_feature is
    procedure rtr_pref_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :        natural := 1;
        variable ID_2               :        natural := 2;
        variable tx_frame           :        SW_CAN_frame_type;
        variable rx_frame           :        SW_CAN_frame_type;
        variable frame_sent         :        boolean := false;
        variable command            :        SW_command := (false, false, false);
        variable status             :        SW_status;
        variable mode               :        SW_mode := (false, false, false,
                                                false, false, false, false,
                                                false, false, false);
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Part 1
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Generate CAN frame with arbitrary DLC and force RTR after.
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, tx_frame);
        tx_frame.rtr := RTR_FRAME;
        tx_frame.frame_format := NORMAL_CAN;

        ------------------------------------------------------------------------
        -- Set the RTR preferred behaviour to send the DLC all zeroes..
        ------------------------------------------------------------------------
        mode.rtr_pref := true;
        set_core_mode(mode, ID_2, mem_bus(2));
        set_core_mode(mode, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Restart the content of the Node 2 RX Buffer
        ------------------------------------------------------------------------
        command.release_rec_buffer := true;
        give_controller_command(command, ID_2, mem_bus(2));
        command.release_rec_buffer := false;

        ------------------------------------------------------------------------
        -- Insert the frame for transmittion and wait until it was sent
        ------------------------------------------------------------------------
        CAN_send_frame(tx_frame, 1, ID_1, mem_bus(1), frame_sent);
        if (not frame_sent) then
            o.outcome := false;
        end if;
        CAN_wait_frame_sent(ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Check that recieved DLC is zero
        ------------------------------------------------------------------------
        CAN_read_frame(rx_frame, ID_2, mem_bus(2));
        if (rx_frame.dlc /= x"0") then
            o.outcome := false;
        end if;


        ------------------------------------------------------------------------
        -- Part 2
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Set the RTR preferred behaviour to send the original DLC
        ------------------------------------------------------------------------
        mode.rtr_pref := false;
        set_core_mode(mode, ID_2, mem_bus(2));
        set_core_mode(mode, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Restart the content of the Node 2 RX Buffer
        ------------------------------------------------------------------------
        command.release_rec_buffer := true;
        give_controller_command(command, ID_2, mem_bus(2));
        command.release_rec_buffer := false;

        ------------------------------------------------------------------------
        -- Insert the frame for transmittion and wait until recieved
        ------------------------------------------------------------------------
        CAN_send_frame(tx_frame, 1, ID_1, mem_bus(1), frame_sent);
        if (not frame_sent) then
            o.outcome := false;
        end if;
        CAN_wait_frame_sent(ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Check that recieved DLC is matching transmitted DLC
        ------------------------------------------------------------------------
        CAN_read_frame(rx_frame, ID_2, mem_bus(2));
        if (rx_frame.dlc /= tx_frame.dlc) then
            o.outcome := false;
        end if;

  end procedure;

end package body;
