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
--  Forbidding FD frames feature test implementation.
--
-- Test sequence:
--      1. Configure Node 1 to forbid CAN FD reception.
--      2. Read RX error counter on Node 1.
--      3. Generate CAN FD frame and send it by Node 2.
--      4. Wait till end of error frame and check that receive errror counter
--         was increased.
--      5. Send CAN 2.0 frame from Node 2
--      6. Frame is transmitted, wait till the end, check that receive error
--         counter was decreased (frame was transmitted OK).
--      7. Allow CAN FD reception in Node 1. Insert CAN FD Frame to Node 2.
--      8. Wait till end of transmission, read error counters and verify it
--         was decreased.
--      9. If error counters are over 70, restart them so that nodes won't
--         turn error passive.
--
--------------------------------------------------------------------------------
-- Revision History:
--    21.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.can_constants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;


package forbid_fd_feature is
    procedure forbid_fd_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body forbid_fd_feature is

    procedure forbid_fd_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ID_1               :       natural := 1;
        variable ID_2               :       natural := 2;

        variable mode               :       SW_mode := (false, false, false,
                                             false, false, false, false, false,
                                             false, false);
        variable err_counters_1     :       SW_error_counters;
        variable err_counters_2     :       SW_error_counters;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- First disable the FD support of both Nodes. This is done to make
        -- sure that both nodes have the same ISO type set.
        ------------------------------------------------------------------------
        mode.flexible_data_rate := true;
        set_core_mode(mode, ID_2, mem_bus(2));
        mode.flexible_data_rate := false;
        set_core_mode(mode, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Read RX Error counter node 1
        ------------------------------------------------------------------------
        read_error_counters(err_counters_1, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Send FD frame by node 2 and wait for error frame...
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.frame_format := FD_CAN;
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_error_transmitted(ID_2, mem_bus(2));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Read RX Error counter node 1 again
        ------------------------------------------------------------------------
        read_error_counters(err_counters_2, ID_1, mem_bus(1));

        -- Counter should be increased
        if ((err_counters_1.rx_counter + 1 + 8) /= err_counters_2.rx_counter) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "RX Error counter not incremented as expected!" severity error;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Now send the same frame, but not the FD type. Wait until bus is idle
        ------------------------------------------------------------------------
        CAN_frame.frame_format := NORMAL_CAN;
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Read RX Error counter node 1 again
        ------------------------------------------------------------------------
        read_error_counters(err_counters_2, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Counter should be decreased by one now due to sucesfull reception.
        -- But it should be increased by 8 since it is the first node that
        -- detected the error!
        ------------------------------------------------------------------------
        if ((err_counters_1.rx_counter + 8) /= err_counters_2.rx_counter) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "RX Error counter not incremented as expected!" severity error;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Now enable the FD support of Node 1
        ------------------------------------------------------------------------
        mode.flexible_data_rate := true;
        set_core_mode(mode, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Now again send the same frame but FD type now unit should accept
        -- the frame OK!
        ------------------------------------------------------------------------
        CAN_frame.frame_format := FD_CAN;
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Read RX Error counter node 1 again
        ------------------------------------------------------------------------
        read_error_counters(err_counters_2, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Counter should be less than the value read now or both should be
        -- zeroes when counter cannnot already be lowered...
        ------------------------------------------------------------------------
        if ((err_counters_1.rx_counter + 7) /= err_counters_2.rx_counter) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "RX Error counter not decremented as expected!" severity error;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Since counter is incremented more than decremented, after many
        -- iterations UNIT will go to error_passive and then bus_off. To forbid
        -- this we clear error counters
        ------------------------------------------------------------------------
        if (err_counters_2.rx_counter > 70) then
            report "Resetting error counters";
            err_counters_2.rx_counter := 0;
            err_counters_2.tx_counter := 0;
            set_error_counters(err_counters_2, ID_1, mem_bus(1));
            set_error_counters(err_counters_2, ID_2, mem_bus(2));
        end if;
  end procedure;

end package body;
