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
--  Frame filters mode feature test.  
--
-- @Verifies:
--  @1. When in Frame filters mode, RX frame which does not pass frame filters
--      will not be received!
--  @2. When in Frame filters mode, RX frame which does pass frame filters will
--      be received.
--  @3. When not in frame filters mode, RX frame which does not pass frame filters
--      will be received.
--
-- @Test sequence:
--  @1. Configure frame filters mode in Node 1. Configure filter A to receive only
--      can frames with Odd base IDs (lowest bit = 1). Configure all other filters
--      not to accept any frames (disable all frame/identifier type combinations).
--  @2. Generate random CAN frame and send by Node 2. Wait till frame is received.
--      If frame should be received (LSB of ID=1), check it is received. If it
--      should not be received, check it is not received!
--  @3. Disable frame filters mode in Node 1. Send CAN frame with LSB of ID=0,
--      (should not be received according to frame filters settings).
--  @4. Wait till frame passes and check it is received in Node 2!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    22.9.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package mode_frame_filters_feature is
    procedure mode_frame_filters_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body mode_frame_filters_feature is
    procedure mode_frame_filters_feature_exec(
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
        
        variable rx_buf_state       :       SW_RX_Buffer_info;
        variable frames_equal       :       boolean := false;
        variable filt_A_cfg         :       SW_CAN_mask_filter_config;
        variable filt_B_C_cfg       :       SW_CAN_mask_filter_config;
        variable filter_range_cfg   :       SW_CAN_range_filter_config;
    begin

        ------------------------------------------------------------------------
        -- @1. Configure frame filters mode in Node 1. Configure filter A to 
        --    receive only can frames with Odd base IDs (lowest bit = 1). 
        --    Configure all other filters not to accept any frames (disable all
        --    frame/identifier type combinations).
        ------------------------------------------------------------------------
        info("Step 1: Configure frame filters");
        mode_1.acceptance_filter := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));

        -- Filter A (accept BASE IDs with LSB equal to 1)
        filt_A_cfg.ID_value := 1;
        filt_A_cfg.ID_mask := 1;
        filt_A_cfg.ident_type := BASE;
        filt_A_cfg.acc_CAN_2_0 := true;
        filt_A_cfg.acc_CAN_FD := true;
        CAN_set_mask_filter(filter_A, filt_A_cfg, ID_1, mem_bus(1));
        
        -- Filter B and C (dont accept any frame type)
        filt_B_C_cfg.ID_value := 1;
        filt_B_C_cfg.ID_mask := 1;
        filt_B_C_cfg.ident_type := BASE;
        filt_B_C_cfg.acc_CAN_2_0 := false;
        filt_B_C_cfg.acc_CAN_FD := false;
        CAN_set_mask_filter(filter_B, filt_B_C_cfg, ID_1, mem_bus(1));
        CAN_set_mask_filter(filter_C, filt_B_C_cfg, ID_1, mem_bus(1));

        -- Range filters (dont accept any frame type)
        filter_range_cfg.ID_th_low := 0;
        filter_range_cfg.ID_th_high := 0;
        filter_range_cfg.ident_type := BASE; 
        filter_range_cfg.acc_CAN_2_0 := false;
        filter_range_cfg.acc_CAN_FD := false;
        CAN_set_range_filter(filter_range_cfg, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @2. Generate random CAN frame and send by Node 2. Wait till frame is 
        --    received. If frame should be received (LSB of ID=1), check it is
        --    received. If it should not be received, check it is not received!
        ------------------------------------------------------------------------
        info("Step 2: Check frame filters mode operation");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_TX_frame.ident_type := BASE;
        CAN_TX_frame.identifier := CAN_TX_frame.identifier mod 2048;
        CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        get_rx_buf_state(rx_buf_state, ID_1, mem_bus(1));
        
        -- Frame should be received
        if (CAN_TX_frame.identifier mod 2 = 1) then
            check(rx_buf_state.rx_frame_count = 1, "Frame not filtered out!"); 
            CAN_read_frame(CAN_RX_frame, ID_1, mem_bus(1));
            CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
            
        -- Frame should not be received
        else
            check(rx_buf_state.rx_frame_count = 0, "Frame filtered out!");
        end if;
                
        ------------------------------------------------------------------------
        -- @3. Disable frame filters mode in Node 1. Send CAN frame with LSB of
        --    ID=0,(should not be received according to frame filters settings).
        ------------------------------------------------------------------------
        info("Step 3: Check frame filters mode disabled!");
        mode_1.acceptance_filter := false;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        CAN_TX_frame.identifier := 10; -- Filter A set to filter it out!
        CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        get_rx_buf_state(rx_buf_state, ID_1, mem_bus(1));
        
        check(rx_buf_state.rx_frame_count = 1,
            "Frame not filtered out when frame filters mode disabled!"); 
        CAN_read_frame(CAN_RX_frame, ID_1, mem_bus(1));
            CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);

        wait for 1000 ns;
        
  end procedure;

end package body;