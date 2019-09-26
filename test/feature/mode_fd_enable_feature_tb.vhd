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
--  Flexible data-rate enable feature test!
--
-- Verifies:
--  1. CAN FD frame (EDL bit recessive) is received OK when Flexible data-rate
--     mode is enabled (as default).
--  2. Receiving CAN FD frame results in Error frame when Flexible data-rate
--     mode is disabled.
--  3. Transmitting CAN FD frame when Flexible data-rate mode is disabled
--     results in transmission of Error frame.
--
-- Test sequence:
--  1. Send CAN FD frame by Node 2. Wait till frame is sent. Read it from Node 1
--     and compare it with send frame.
--  2. Disable Flexible data-rate mode in Node 1. Send CAN frame by Node 2.
--     Wait till Control field of Node 1. Set both nodes to One-shot mode.
--  3. Wait till Node 1 is not in Control field. Check that it is transmitting
--     error frame. Read Error code capture and check that it shows Form Error
--     during Control field. Wait till the frame is transmitted.
--  4. Set Node 2 to Acknowledge forbidden mode. Transmitt frame by Node 1.
--     Wait till it is sent, read Error code capture and check it is NOT equal
--     to Form error (this is just to achieve change in Error code capture).  
--  5. Send frame by Node 1. Wait till it is in Control field.
--  6. Wait until Node 1 is not in control field. Check that Error frame is
--     being transmitted. Read Error code capture and check that it shows Form 
--     Error during Control field. Wait till the frame is transmitted.
--------------------------------------------------------------------------------
-- Revision History:
--    22.9.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package mode_fd_enable_feature is
    procedure mode_fd_enable_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body mode_fd_enable_feature is
    procedure mode_fd_enable_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ctr_1              :       natural;
        variable ctr_2              :       natural;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable rand_val           :       real;
        variable retr_th            :       natural;
        variable mode_backup        :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable txt_buf_state      :       SW_TXT_Buffer_state_type;
        variable rx_buf_state       :       SW_RX_Buffer_info;
        variable status             :       SW_status;
        variable frames_equal       :       boolean := false;
        variable err_capt           :       SW_error_capture;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- 1. Send CAN FD frame by Node 2. Wait till frame is sent. Read it from
        --    Node 1 and compare it with send frame.
        ------------------------------------------------------------------------
        info("Step 1: Sending CAN FD frame when FD mode enabled!");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        CAN_read_frame(CAN_RX_frame, ID_1, mem_bus(1));
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        check(frames_equal, "TX - RX frames matching!");

        ------------------------------------------------------------------------
        -- 2. Disable Flexible data-rate mode in Node 1. Send CAN frame by 
        --    Node 2. Wait till Control field of Node 1. Set both nodes to
        --    One-shot mode.
        ------------------------------------------------------------------------
        info("Step 2: Disable FD mode, send frame!");
        mode_1.flexible_data_rate := false;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        CAN_enable_retr_limit(true, 0, ID_2, mem_bus(2));
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        
        CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_pc_state(pc_deb_control, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- 3. Wait till Node 1 is not in Control field. Check that it is 
        --    transmitting error frame. Read Error code capture and check that
        --    it shows Form Error during Control field. Wait till the frame is
        --    transmitted.
        ------------------------------------------------------------------------
        info("Step 3: Check error frame is transmitted, Form error occurs!");
        CAN_wait_not_pc_state(pc_deb_control, ID_1, mem_bus(1));
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission,
            "Error frame transmitted as response to CAN FD frame!");
        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_type = can_err_form,
            "Error type: " & SW_error_type'image(err_capt.err_type));
        check(err_capt.err_pos = err_pos_ctrl,
            "Error in :" & SW_error_position'image(err_capt.err_pos));
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- 4. Set Node 2 to Acknowledge forbidden mode. Transmitt frame by 
        --    Node 1. Wait till it is sent, read Error code capture and check it
        --    is NOT equal to Form error (this is just to achieve change in Error
        --    code capture).
        ------------------------------------------------------------------------
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        CAN_TX_frame.frame_format := NORMAL_CAN;
        CAN_send_frame(CAN_TX_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check_false(err_capt.err_type = can_err_form, "Error type changed!");
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- 5. Send frame by Node 1. Wait till it is in Control field.
        ------------------------------------------------------------------------
        info("Step 4: Send frame by node with FD disabled");
        CAN_send_frame(CAN_TX_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_control, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- 6. Wait until Node 1 is not in control field. Check that Error frame
        --    is being transmitted. Read Error code capture and check that it
        --    shows Form Error during Control field. Wait till the frame is
        --    transmitted.
        ------------------------------------------------------------------------
        info("Step 5: Check node transmitts error frame on ist own FD frame!");
        CAN_wait_not_pc_state(pc_deb_control, ID_1, mem_bus(1));
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission,
            "Error frame transmitted when attempting to send CAN FD frame!");

        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_type = can_err_form,
            "Error type: " & SW_error_type'image(err_capt.err_type));
        check(err_capt.err_pos = err_pos_ctrl,
            "Error in :" & SW_error_position'image(err_capt.err_pos));
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        wait for 1000 ns;
        
  end procedure;

end package body;