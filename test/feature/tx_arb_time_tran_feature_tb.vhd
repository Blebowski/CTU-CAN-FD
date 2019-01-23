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
--  TX arbitration and time transmittion feature test
--
--  Test sequence:
--    1. Part 1:
--      1.1 Measure timestamp from status bus
--      1.2 Insert frame to be transmitted from actual time further by random
--          interval.
--      1.3 Wait until frame is started to be transmitted
--      1.4 Check whether difference between actual timestamp and time when
--          frame should have been transmitted is less than 150. Note that
--          timestamp is in feature environment increased every clock cycle.
--          One bit time in default configuration has 130 clock cycles. Thus if
--          we insert the frame in begining of bit time in takes nearly whole
--          bit time until its transmittion is started.
--      1.5 Repeat steps 1-4 but use Buffer 2 for transmittion.
--
--------------------------------------------------------------------------------
-- Revision History:
--    23.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--    13.06.2018  1. Used CAN Test lib instead of register access functions.
--                2. Removed transmission from multiple buffers, since buffers
--                   are now compared with priority. This will be covered in
--                   separate test.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package tx_arb_time_tran_feature is
    procedure tx_arb_time_tran_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body tx_arb_time_tran_feature is
    procedure tx_arb_time_tran_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        constant ID_1           	:       natural := 1;
        constant ID_2           	:       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable CAN_frame_2        :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable act_ts             :       std_logic_vector(63 downto 0);
        variable rand_value         :       real := 0.0;
        variable rand_value_2       :       real := 0.0;
        variable aux1               :       natural;
        variable aux2               :       natural;
        variable status             :       SW_Status;
    begin

        o.outcome := true;

        ------------------------------------------------------------------------
        -- Wait until unit for sure comes out of integration.
        ------------------------------------------------------------------------
        wait_rand_cycles(rand_ctr, mem_bus(1).clk_sys, 1600, 1601);

        ------------------------------------------------------------------------
        -- Part 1
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Measure timestamp and generate frame
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_generate_frame(rand_ctr, CAN_frame_2);
        act_ts := iout(1).stat_bus(STAT_TS_HIGH downto STAT_TS_LOW);

        ------------------------------------------------------------------------
        -- Add random value
        ------------------------------------------------------------------------
        rand_real_v(rand_ctr, rand_value);

        -- Here we assume this test will  use only lowest 32 bits!
        CAN_frame.timestamp(63 downto 32) := act_ts(63 downto 32);
        CAN_frame.timestamp(31 downto 0)  :=
            std_logic_vector(unsigned(act_ts(31 downto 0)) + 30 +
                             integer(rand_value * 10000.0));

        ------------------------------------------------------------------------
        -- Send frame and check when TX started
        ------------------------------------------------------------------------
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        loop
            get_controller_status(status, ID_1, mem_bus(1));
            if (status.transmitter) then
                exit;
            end if;
        end loop;

        aux1 := to_integer(unsigned(
                    iout(1).stat_bus(STAT_TS_HIGH - 32 downto STAT_TS_LOW)));
        aux2 := to_integer(unsigned(CAN_frame.timestamp(31 downto 0)));

        ------------------------------------------------------------------------
        -- We tolerate up to 190 clock cycles between actual timestamp and
        -- transmitt time. Default time settings have 140 clock cycles per Bit
        -- Time. There is up to 40 clock cycles of storing CAN frame. 6 clock
        -- cycles are delay of TX Arbitrator! This gives possible delay
        -- of 186 clock cycles. Let's take 190 to have some reserve!
        ------------------------------------------------------------------------
        check(aux1 - aux2 <= 190, "Frame not sent at time when expected!");
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Do  the same with buffer 2
        ------------------------------------------------------------------------
        act_ts := iout(1).stat_bus(STAT_TS_HIGH downto STAT_TS_LOW);

        ------------------------------------------------------------------------
        -- Add random value
        ------------------------------------------------------------------------
        rand_real_v(rand_ctr, rand_value);
        --Here we assume this test will  use only lowest 32 bits!
        CAN_frame.timestamp(63 downto 32) := act_ts(63 downto 32);
        CAN_frame.timestamp(31 downto 0) :=
            std_logic_vector(unsigned(act_ts(31 downto 0)) + 30 +
                                integer(rand_value * 10000.0));

        ------------------------------------------------------------------------
        -- Send frame and check when TX started
        ------------------------------------------------------------------------
        CAN_send_frame(CAN_frame, 2, ID_1, mem_bus(1), frame_sent);
        loop
            get_controller_status(status, ID_1, mem_bus(1));
            if (status.transmitter) then
                exit;
            end if;
        end loop;

        aux1 := to_integer(unsigned(
                    iout(1).stat_bus(STAT_TS_HIGH - 32 downto STAT_TS_LOW)));
        aux2 := to_integer(unsigned(CAN_frame.timestamp(31 downto 0)));

        ------------------------------------------------------------------------
        -- We tolerate up to 150 clock cycles between actual timestamp and
        -- transmitt time. This fits to the default setting of up to 130 clock
        -- cycles per bit time!
        ------------------------------------------------------------------------
        check(aux1 - aux2 <= 150, "Frame not sent at time when expected!");
        CAN_wait_bus_idle(ID_1, mem_bus(1));

  end procedure;

end package body;