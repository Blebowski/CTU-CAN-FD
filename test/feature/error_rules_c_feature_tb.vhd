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
-- @TestInfoStart
--
-- @Purpose:
--  Fault confinement rules - rule C - feature test.
--
-- @Verifies:
--  @1. When a transmitter sends an error flag, the transmit error counter shall
--      be incremented by 8.
--      Exception 1:
--        If the transmitter is error-passive and detects an ACK error because of
--        not detecting a dominant ACK and does not detect a dominant bit while
--        sending its passive error flag.
--      Exception 2:
--        If the transmitter sends an error flag because a stuff error occurred
--        during arbitration, whereby the stuff bit should have been recessive,
--        and has been sent recessive but is monitored to be dominant.
--  @2. ERR_CAPT[ERR_POS] = Arbitration and Stuff Error!
--
-- @Test sequence:
--  @1. Set Node 2 to ACK forbidden and test modes. Set Node 1 to One-shot mode.
--      Send CAN frame by Node 1. Wait until CAN frame is sent and check that TX
--      Error counter of Node 1 has incremented by 8. Check that RX Error counter
--      is not incremented in Node 1.
--  @2. Set Node 1 to Error Passive (via CTR_PRES). Send CAN frame and wait until
--      it is sent. Check that TX Error counter was not incremented. Check that
--      RX Error counter was not incremented. Reset Error counters.
--  @3. Generate CAN frame with Recessive Stuff bit in CAN ID. Send such frame
--      by Node 1 and force that Stuff bit to Dominant. Check that Error frame
--      was transmitted. Check that Stuff Error occured during arbitraton. Check
--      that TEC was not incremented in Node 1! Check that REC was incremented
--      by 1 in Node 2.
--  @4. Generate CAN frame with Dominant Stuff bit in CAN ID. Send such frame by
--      Node 1 and force that Stuff bit to Recessive. Check that Error frame was
--      transmitted. Check that Stuff Error (or bit error) occured during
--      arbitration. Check that TX Error counter was incremented by 8 in Node 1.
--      Check that REC was incremented by 1 in Node 2!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    26.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package error_rules_c_feature is
    procedure error_rules_c_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;

package body error_rules_c_feature is
    procedure error_rules_c_feature_exec(
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
        
        variable status             :       SW_status;

        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters_1     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_3     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_4     :       SW_error_counters := (0, 0, 0, 0);

        variable id_vect            :       std_logic_vector(28 downto 0) :=
                                                (OTHERS => '0');
        variable err_capt           :       SW_error_capture;
        
    begin

        -----------------------------------------------------------------------
        -- @1. Set Node 2 to ACK forbidden and test modes. Set Node 1 to One-shot
        --    mode. Send CAN frame by Node 1. Wait until CAN frame is sent and
        --    check that TX Error counter of Node 1 has incremented by 8. Check
        --    that RX Error counter is not incremented in Node 1.
        -----------------------------------------------------------------------
        info("Step 1");
        
        read_error_counters(err_counters_2, ID_1, mem_bus(1));

        mode_2.acknowledge_forbidden := true;
        mode_2.test := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));

        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_error_frame(ID_1, mem_bus(1));
        
        read_error_counters(err_counters_2, ID_1, mem_bus(1));
        
        check(err_counters_2.tx_counter = err_counters_1.tx_counter + 8,
            "TX Error counter inctemented by 8 in transmitter!");

        check(err_counters_2.rx_counter = err_counters_1.rx_counter,
            "RX Error counter unchanged in transmitter!");
            
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        
        -----------------------------------------------------------------------
        -- @2. Set Node 1 to Error Passive (via CTR_PRES). Send CAN frame and
        --    wait until it is sent. Check that TX Error counter was not
        --    incremented. Check that RX Error counter was not incremented.
        --    Reset Error counters.
        -----------------------------------------------------------------------
        info("Step 2");
        
        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));

        err_counters_1.tx_counter := 140;
        set_error_counters(err_counters_1, ID_1, mem_bus(1));
        read_error_counters(err_counters_2, ID_1, mem_bus(1));
        
        CAN_generate_frame(rand_ctr, CAN_frame);
        -- This is so that fixed ID can be set in further steps!
        CAN_frame.ident_type := EXTENDED;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_error_frame(ID_1, mem_bus(1));

        wait for 20 ns;
        read_error_counters(err_counters_3, ID_1, mem_bus(1));
        
        check(err_counters_3.tx_counter = err_counters_2.tx_counter,
            "TX Error counter not incremented as result of ACK Error!");

        check(err_counters_3.rx_counter = err_counters_2.rx_counter,
            "RX Error counter unchanged in transmitter!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        
        err_counters_1.tx_counter := 0;
        err_counters_1.rx_counter := 0;
        set_error_counters(err_counters_1, ID_1, mem_bus(1));

        -----------------------------------------------------------------------
        -- @3. Generate CAN frame with Recessive Stuff bit in CAN ID. Send such
        --    frame by Node 1 and force that Stuff bit to Dominant. Check that
        --    Error frame was transmitted. Check that Stuff Error occured 
        --    during arbitraton. Check that TEC was not incremented in Node 1!
        --    Check that REC was incremented by 1 in Node 2.
        -----------------------------------------------------------------------
        info("Step 3");

        read_error_counters(err_counters_1, ID_1, mem_bus(1));

        id_vect := id_vect(10 downto 0) & "000000000000000000";
        CAN_frame.identifier := to_integer(unsigned(id_vect));
        
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));
        
        -- Wait for 5 dominant bits (SOF + 4 bits of ID)
        for i in 0 to 4 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;

        -- Force Dominant to beat Recessive Stuff bit!
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns;
        release_bus_level(so.bl_force);

        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission,
            "Error frame is being transmitted by Node 1!");

        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_pos = err_pos_arbitration, "Error in arbitration!");
        check(err_capt.err_type = can_err_stuff, "Stuff Error type!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        read_error_counters(err_counters_2, ID_1, mem_bus(1));

        check(err_counters_1.tx_counter = err_counters_2.tx_counter,
            "TX Error counter not incremented due to Stuff error in arbitration!");

        check(err_counters_1.rx_counter = err_counters_2.rx_counter,
            "RX Error counter not incremented when transmitter!");

        -----------------------------------------------------------------------
        -- @4. Generate CAN frame with Dominant Stuff bit in CAN ID. Send such
        --    frame by Node 1 and force that Stuff bit to Recessive. Check that
        --    Error frame was transmitted. Check that Stuff Error (or bit error)
        --    occured during arbitration. Check that TX Error counter was 
        --    incremented by 8 in Node 1. Check that REC was incremented by 1 in
        --    Node 2!
        -----------------------------------------------------------------------
        info("Step 4");

        read_error_counters(err_counters_1, ID_1, mem_bus(1));

        -- Dominant Stuff bit will be inserted after 5 bits!
        id_vect := "11111000000000000000000000000";
        CAN_frame.identifier := to_integer(unsigned(id_vect));
        
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));

        -- Wait for 6 bits (SOF + 5 Recessive bits of ID)
        for i in 0 to 5 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;
        
        -- Force Recessive to beat Dominant Stuff bit!
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns;
        release_bus_level(so.bl_force);

        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission,
            "Error frame is being transmitted by Node 1!");

        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_pos = err_pos_arbitration, "Error in arbitration!");
        check(err_capt.err_type = can_err_stuff, "Stuff Error type!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        read_error_counters(err_counters_2, ID_1, mem_bus(1));

        check(err_counters_1.tx_counter + 8 = err_counters_2.tx_counter,
            "TX Error counter not incremented due to Sending Dominant Stuff bit"&
            " and monitoring recessive!");

        check(err_counters_1.rx_counter = err_counters_2.rx_counter,
            "RX Error counter not incremented when transmitter!");

    end procedure;

end package body;