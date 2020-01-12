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
--  Interrupt RX feature test.
--
-- @Verifies:
--  @1. RX Interrupt is set when frame is received OK in EOF field.
--  @2. RX Interrupt causes INT to go high when it is enabled.
--  @3. RX Interrupt causes INT to go low when it is disabled.
--  @4. RX Interrupt is cleared by write to INT_STATUS register.
--  @5. RX Interrupt is not set when Error frame occurs.
--  @6. RX Interrupt is not set when it is masked.
--  @7. RX Interrupt enable is manipulated properly by INT_ENA_SET and
--      INT_ENA_CLEAR.
--  @8. RX Interrupt mask is manipulated properly by INT_MASk_SET and
--      INT_MASK_CLEAR.
--  @9. RX Interrupt is not set when frame was transmitted.

-- @Test sequence:
--  @1. Unmask and enable RX Interrupt, disable and mask all other interrupts on
--      Node 1.
--  @2. Set Retransmitt limit to 0 on Node 2 (One shot-mode). Enable Retransmitt
--      limitations on Node 2. Send frame by Node 2.
--  @3. Monitor Node 1 frame, check that in the beginning of EOF, Interrupt is
--      Not set, after EOF it is set.
--  @4. Check that INT pin is high. Disable RX Interrupt and check that INT pin
--      goes low. Enable Interrupt and check it goes high again.
--  @5. Clear RX Interrupt, check it is cleared and INT pin goes low.
--  @6. Send Frame by Node 2. Force bus level during ACK to recessive, check that
--      error frame is transmitted by both nodes. Wait till bus is idle, check
--      that no RX Interrupt was set, INT pin is low.
--  @7. Mask RX Interrupt. Send frame by Node 2. Check that after frame RX
--      Interrupt was not set. Check that INT pin is low.
--  @8. Unmask RX Interrupt. Send frame by Node 2. Check that after frame RX
--      Interrupt was set. Check INT pin is high.
--  @9. Disable RX Interrupt, check that RX interrupt was disabled.
-- @10. Enable RX Interrupt, check that RX interrupt was enabled.
-- @11. Mask RX Interrupt, check RX Interrupt is Masked.
-- @12. Un-Mask RX Interrupt, check RX Interrupt is Un-masked.
-- @13. Send frame by Node 1. Check that after frame RX Interrupt is not set and
--      Interrupt pin remains low.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    22.6.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package int_rx_feature is
    procedure int_rx_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;

package body int_rx_feature is
    procedure int_rx_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable CAN_frame          :     SW_CAN_frame_type;
        variable CAN_frame_rx       :     SW_CAN_frame_type;
        variable frame_sent         :     boolean := false;
        variable frames_equal       :     boolean := false;
        variable ID_1           	:     natural := 1;
        variable ID_2           	:     natural := 2;

        variable int_mask           :     SW_interrupts := SW_interrupts_rst_val;
        variable int_ena            :     SW_interrupts := SW_interrupts_rst_val;
        variable int_stat           :     SW_interrupts := SW_interrupts_rst_val;
        variable pc_dbg             :     SW_PC_Debug;  
    begin

        -----------------------------------------------------------------------
        -- @1. Unmask and enable RX Interrupt, disable and mask all other 
        --    interrupts on Node 1.
        -----------------------------------------------------------------------
        info("Step 1: Setting RX Interrupt");
        int_mask.receive_int := false;
        int_ena.receive_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        write_int_enable(int_ena, ID_1, mem_bus(1));
        
        -----------------------------------------------------------------------
        --  @2. Set Retransmitt limit to 0 on Node 2 (One shot-mode). Enable 
        --     Retransmitt limitations on Node 2. Send frame by Node 2.
        -----------------------------------------------------------------------
        info("Step 2: Sending frame");
        CAN_enable_retr_limit(true, 0, ID_2, mem_bus(2));
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        
        -----------------------------------------------------------------------
        --  @3. Monitor Node 1 frame, check that in the beginning of EOF, 
        --     Interrupt is Not set, after EOF it is set.
        -----------------------------------------------------------------------  
        info("Step 3: Check RX Interrupt is set in EOF!"); 
        CAN_wait_pc_state(pc_deb_eof, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.receive_int,
            "RX Interrupt not set in beginning of EOF");
        CAN_wait_not_pc_state(pc_deb_eof, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        
        check(int_stat.receive_int, "RX Interrupt set at the end of EOF");
        
        -- Wait till bus is idle, read-out received frame so that there is
        -- nothing in RX Buffer of Node 1.
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_read_frame(CAN_frame_rx, ID_1, mem_bus(1));
        CAN_compare_frames(CAN_frame, CAN_frame_rx, false, frames_equal);
        
        check(frames_equal, "TX, RX frames should be equal!");
        
        -----------------------------------------------------------------------
        --  @4. Check that INT pin is high. Disable RX Interrupt and check that
        --     INT pin goes low. Enable Interrupt and check it goes high again.
        -----------------------------------------------------------------------
        info("Step 4: Check INT pin toggles");
        check(iout(1).irq = '1', "INT pin should be high!");
        int_ena.receive_int := false;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        
        check(iout(1).irq = '0', "INT pin should be low!");
        
        int_ena.receive_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        
        check(iout(1).irq = '1', "INT pin should be high!");

        -----------------------------------------------------------------------
        --  @5. Clear RX Interrupt, check it is cleared and INT pin goes low.
        -----------------------------------------------------------------------
        info("Step 4: Clear RX Interrupt, Check INT pin toggles");
        int_stat.receive_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        
        check_false(int_mask.receive_int, "RX Interrupt status should be 0!");
        check(iout(1).irq = '0', "INT pin should be low!");
        
        -----------------------------------------------------------------------
        --  @6. Send Frame by Node 2. Force bus level during ACK to recessive, 
        --     check that error frame is transmitted by both nodes. Wait till
        --     bus is idle, check.
        -----------------------------------------------------------------------
        info("Step 6: Check RX Interrupt is not set upon Error Frame!");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_not_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        -- TODO: Check error frame is being transmitted!
        release_bus_level(so.bl_force);
        
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        read_int_status(int_stat, ID_1, mem_bus(1));
        
        check_false(int_stat.receive_int, "RX Interrupt status should be 0!");
        check(iout(1).irq = '0', "INT pin should be low!");

        -----------------------------------------------------------------------
        --  @7. Mask RX Interrupt. Send frame by Node 2. Check that after frame
        --     RX Interrupt was not set. Check that INT pin is low.
        -----------------------------------------------------------------------
        info("Step 7: Check Masked RX Interrupt is not captured");
        int_mask.receive_int := true;
        int_ena.receive_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_2, mem_bus(2));
        CAN_read_frame(CAN_frame_rx, ID_1, mem_bus(1));
        CAN_compare_frames(CAN_frame, CAN_frame_rx, false, frames_equal);
        check(frames_equal, "TX, RX frames should be equal!");
        read_int_status(int_stat, ID_1, mem_bus(1));
        
        check_false(int_stat.receive_int, "RX Interrupt status should be 0!");
        check(iout(1).irq = '0', "INT pin should be low!");

        -----------------------------------------------------------------------
        --  @8. Unmask RX Interrupt. Send frame by Node 2. Check that after 
        --     frame RX Interrupt was set. Check INT pin is high.
        -----------------------------------------------------------------------
        info("Step 8: Check Un-Masked RX Interrupt is captured");
        int_mask.receive_int := false;
        int_ena.receive_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_2, mem_bus(2));
        CAN_read_frame(CAN_frame_rx, ID_1, mem_bus(1));
        CAN_compare_frames(CAN_frame, CAN_frame_rx, false, frames_equal);
        check(frames_equal, "TX, RX frames should be equal!");
        read_int_status(int_stat, ID_1, mem_bus(1));
        
        check(int_stat.receive_int, "RX Interrupt status should be 1!");
        check(iout(1).irq = '1', "INT pin should be high");
        
        -----------------------------------------------------------------------
        --  @9. Disable RX Interrupt, check that RX interrupt was disabled.
        -----------------------------------------------------------------------
        info("Step 9: Check RX Interrupt Enable Set");
        int_ena.receive_int := false;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        int_ena.receive_int := true;
        read_int_enable(int_ena, ID_1, mem_bus(1));
        check_false(int_ena.receive_int, "RX Interrupt should be disabled!");
        
        -----------------------------------------------------------------------
        -- @10. Enable RX Interrupt, check that RX interrupt was enabled.
        -----------------------------------------------------------------------
        info("Step 10: Check RX Interrupt Enable Clear");
        int_ena.receive_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        int_ena.receive_int := false;
        read_int_enable(int_ena, ID_1, mem_bus(1));
        
        check(int_ena.receive_int, "RX Interrupt should be enabled!");        
        
        -----------------------------------------------------------------------
        -- @11. Mask RX Interrupt, check RX Interrupt is Masked.
        -----------------------------------------------------------------------
        info("Step 11: Check RX Interrupt Mask Set");
        int_mask.receive_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        int_mask.receive_int := false;
        read_int_mask(int_mask, ID_1, mem_bus(1));
        
        check(int_ena.receive_int, "RX Interrupt should be masked!");        
        
        -----------------------------------------------------------------------
        -- @12. Un-Mask RX Interrupt, check RX Interrupt is Un-masked.
        -----------------------------------------------------------------------
        info("Step 12: Check RX Interrupt Mask Clear");
        int_mask.receive_int := false;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        int_mask.receive_int := true;
        read_int_mask(int_mask, ID_1, mem_bus(1));
        
        check_false(int_mask.receive_int, "RX Interrupt should be unmasked!");
        
        -----------------------------------------------------------------------
        -- @13. Send frame by Node 1. Check that after frame RX Interrupt is 
        --     not set and INT pin remains low.
        -----------------------------------------------------------------------
        info("Step 13: Check TX does not cause RX Interrupt to be captured!");
        int_stat.receive_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        int_mask.receive_int := false;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        int_ena.receive_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        CAN_read_frame(CAN_frame_rx, ID_2, mem_bus(2));
        CAN_compare_frames(CAN_frame, CAN_frame_rx, false, frames_equal);
        read_int_status(int_stat, ID_1, mem_bus(1));
        
        check(frames_equal, "TX, RX frames should be equal!");
        check_false(int_stat.receive_int, "RX Interrupt should not be set after TX!");
        check(iout(1).irq = '0', "INT pins should not be high after TX!");
        
        info("Finished RX interrupt test");
        wait for 1000 ns;

    end procedure;
end package body;