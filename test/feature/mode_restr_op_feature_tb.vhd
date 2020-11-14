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
--  Restricted operation mode - feature test.
--
-- @Verifies:
--  @1. When error is detected in ROM mode, Integration is entered and REC, or
--      TEC are not modified!
--  @2. TXT Buffer after issuing Set Ready command in ROM mode, will end up
--      in TX Failed and no frame will be transmitted!
--  @3. When Overload condition is detected by CTU CAN FD in ROM mode
--      (last bit of EOF, first two bits of Intermission), it enters bus
--      integration state.
--  @4. Node is able to receive next frame right after detecting Error in
--      restricted operation mode.
--  @5. Node in restricted operation mode sends ACK bit dominant!
--
--
-- @Test sequence:
--  @1. Configure ROM mode in Node 1. Insert frame to TXT buffer and issue
--      Set ready command. Verify that TXT buffer ends up in TX failed.
--      Check that unit is not transmitter. Repeat several times in a row.
--  @2. Send frame by Node 2 with stuff bit. Force this stuff bit to opposite
--      value on CAN RX of Node 1. Check that Node 1 goes to integrating. Check
--      that REC/TEC of Node 1 are not changed.
--  @3. Node 2 will retransmitt frame from previous step. Wait until ACK field
--      in Node 1. Check that Node 1 is transmitting dominant bit.
--  @4. Wait until EOF field of Node 1. Wait randomly for 7,8,9 bits to offset
--      to last bit of EOF or first/second bit of Intermission. Force CAN bus
--      to dominant. Check that Node 1 will end in integration state (and
--      not in overload state).
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    10.11.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package mode_restr_op_feature is
    procedure mode_restr_op_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body mode_restr_op_feature is
    procedure mode_restr_op_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     :       SW_error_counters := (0, 0, 0, 0);

        variable fault_th           :       SW_fault_thresholds;
        variable fault_th_2         :       SW_fault_thresholds;
        
        variable txt_buf_state      :       SW_TXT_Buffer_state_type;
        variable status             :       SW_status;
        variable bit_waits          :       natural;
    begin

        ------------------------------------------------------------------------
        --  @1. Configure ROM mode in Node 1. Insert frame to TXT buffer and
        --      issue Set ready command. Verify that TXT buffer ends up in TX
        --      failed. Check that unit is not transmitter. Repeat several times
        --      in a row.
        ------------------------------------------------------------------------
        info("Step 1: Configure ROM mode in Node 1. Try to send frame in ROM!");
        mode_1.restricted_operation := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        -- To make sure we have only one ACK bit
        CAN_TX_frame.frame_format := NORMAL_CAN;
        CAN_send_frame(CAN_TX_frame, 1, ID_1, mem_bus(1), frame_sent);
        wait for 20 ns;
        
        for i in 0 to 20 loop
            get_controller_status(status, ID_1, mem_bus(1));
            check_false(status.transmitter, "Node does not transmitt in ROM!");
            check_false(status.receiver, "Node turned receiver in ROM -> WTF?");
            check(status.bus_status, "Node remains idle");
            get_tx_buf_state(1, txt_buf_state, ID_1, mem_bus(1));
            check(txt_buf_state = buf_failed, "TXT buffer went to failed in ROM!");
            wait for 100 ns;
        end loop;

        -----------------------------------------------------------------------
        -- @2. Send frame by Node 2 with stuff bit. Force this stuff bit to
        --     opposite value on CAN RX of Node 1. Check that Node 1 goes to
        --     integrating. Check that REC/TEC of Node 1 are not changed.
        --
        --     Note: By default for feature tests, device is in One shot mode,
        --           so two frames are issued to invoke transmission back-to
        --           back by Node 2.
        -----------------------------------------------------------------------
        info("Step 2: Send frame by Node 2!");
        CAN_TX_frame.identifier := 0;
        CAN_TX_frame.ident_type := BASE;
        CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_send_frame(CAN_TX_frame, 2, ID_2, mem_bus(2), frame_sent);
        -- Following ends just after sample point of SOF!
        CAN_wait_tx_rx_start(false, true, ID_1, mem_bus(1));
        
        -- 5th bit of Base ID should be recessive stuff bit.
        for i in 0 to 3 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns;

        CAN_wait_sample_point(iout(1).stat_bus, false);        
        wait for 10 ns;
        release_bus_level(so.bl_force);

        get_controller_status(status, ID_1, mem_bus(1));
        check(status.bus_status, "Node became idle after Error in ROM mod!");
        read_error_counters(err_counters, ID_1, mem_bus(1));
        check(err_counters.rx_counter = 0, "REC not incremented in ROM!");
        check(err_counters.tx_counter = 0, "TEC not incremented in ROM!");

        -----------------------------------------------------------------------
        -- @3. Node 2 will retransmitt frame from previous step. Wait until
        --     ACK field in Node 1. Check that Node 1 is transmitting dominant
        --     bit.
        -----------------------------------------------------------------------
        info("Step 3: Wait till ACK of retrasnmitted frame.");
        CAN_wait_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        CAN_wait_sync_seg(iout(1).stat_bus);
        wait for 10 ns;
        check(iout(1).can_tx = DOMINANT, "Send Dominant ACK after Error in ROM!");
        CAN_wait_sample_point(iout(1).stat_bus, false);

        -----------------------------------------------------------------------
        -- @4. Wait until EOF field of Node 1. Wait randomly for 7,8,9 bits to
        --     offset to last bit of EOF or first/second bit of Intermission.
        --     Force CAN bus to dominant. Check that Node 1 will end in
        --     integration state (and not in overload state).
        -----------------------------------------------------------------------
        info("Step 4: Overload condition reaction by CTU CAN FD!");
        
        -- 0 - 2
        rand_int_v(rand_ctr, 2, bit_waits);
        bit_waits := bit_waits + 7;
        
        -- This should offset to last bit of EOF, first or second bit of interm.
        for i in 0 to bit_waits - 1 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;
        
        CAN_wait_sync_seg(iout(1).stat_bus);
        
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 40 ns;
        release_bus_level(so.bl_force);
        
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.bus_status, "Node became idle after Error in ROM mod!");
        read_error_counters(err_counters, ID_1, mem_bus(1));
        check(err_counters.rx_counter = 0, "REC not incremented in ROM!");
        check(err_counters.tx_counter = 0, "TEC not incremented in ROM!");
        
        wait for 30000 ns;

  end procedure;

end package body;