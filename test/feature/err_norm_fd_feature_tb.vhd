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
--  ERR_NORM (Nominal bit-rate) error counter feature test.
--
-- @Verifies:
--  @1. ERR_NORM is incremented by 1 when error occurs for trasmitter/receiver
--      during nominal bit-rate.
--  @2. ERR_FD is not increented by 1 when error occurs for transmitter/receiver
--      during nominal bit-rate.
--  @3. ERR_NORM is not incremented by 1 when error occurs for trasmitter/receiver
--      during data bit-rate.
--  @4. ERR_FD is incremented by 1 when error occurs for trasmitter/receiver
--      during data bit-rate.
--
-- @Test sequence:
--  @1. Generate random frame where bit rate is not switched. Insert the frame
--      to Node 1. Wait until Node 1 starts transmission. Wait for random time
--      until Node 1 transmits Dominant bit. Force the bus-level Recessive for one
--      bit time! This should invoke bit error in Node 1. Wait until bus is idle.
--      Check that ERR_NORM in Node 1 and 2 incremented by 1. Check that ERR_FD
--      in Node 1 and 2 remained the same!
--  @2. Generate random frame where bit rate shall be switched. Wait until data
--      portion of that frame. Wait until Recessive bit is transmitted. Force
--      bus Dominant for 1 bit time! Wait until bus is idle. Check that ERR_FD
--      incremented in Node 1 and Node 2 by 1. Check that ERR_NORM remained the
--      same in Node 1 and Node 2.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    16.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package err_norm_fd_feature is
    procedure err_norm_fd_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body err_norm_fd_feature is
    procedure err_norm_fd_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;

        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        variable err_counters_1_1   :     SW_error_counters;
        variable err_counters_1_2   :     SW_error_counters;

        variable err_counters_2_1   :     SW_error_counters;
        variable err_counters_2_2   :     SW_error_counters;
        
        variable frame_sent         :     boolean;

    begin

        -----------------------------------------------------------------------
        -- @1. Generate random frame where bit rate is not switched. Insert the
        --    frame to Node 1. Wait until Node 1 starts transmission. Wait for
        --    random time until Node 1 transmits Dominant bit. Force the bus-
        --    level Recessive for one bit time! This should invoke bit error in
        --    Node 1. Wait until bus is idle. Check that ERR_NORM in Node 1 and
        --    2 incremented by 1. Check that ERR_FD in Node 1 and 2 remained
        --    the same!
        -----------------------------------------------------------------------
        info("Step 1");

        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));

        read_error_counters(err_counters_1_1, ID_1, mem_bus(1));
        read_error_counters(err_counters_1_2, ID_2, mem_bus(2));
        
        CAN_generate_frame(rand_ctr, frame_1);
        if (frame_1.frame_format = FD_CAN) then
            frame_1.brs := BR_NO_SHIFT;
        end if;
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));

        -- Should be enough to cover many parts of CAN frame but go beyond
        -- frame!
        wait_rand_cycles(rand_ctr, mem_bus(1).clk_sys, 0, 4500);
        
        wait until iout(1).CAN_tx = DOMINANT;
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns; -- To be sure that opposite bit is sampled!
        release_bus_level(so.bl_force);

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        read_error_counters(err_counters_2_1, ID_1, mem_bus(1));
        read_error_counters(err_counters_2_2, ID_2, mem_bus(2));
        
        check(err_counters_1_1.err_norm + 1 = err_counters_2_1.err_norm,
                "ERR_NORM incremented by 1 in transmitter!");
        check(err_counters_1_2.err_norm + 1 = err_counters_2_2.err_norm,
                "ERR_NORM incremented by 1 in receiver!");

        check(err_counters_1_1.err_fd = err_counters_2_1.err_fd,
                "ERR_FD not incremented by 1 in transmitter!");
        check(err_counters_1_2.err_fd = err_counters_2_2.err_fd,
                "ERR_FD not incremented by 1 in receiver!");

        -----------------------------------------------------------------------
        -- @2. Generate random frame where bit rate shall be switched. Wait
        --    until data portion of that frame. Wait until Recessive bit is
        --    transmitted. Force bus Dominant for 1 bit time! Wait until bus is
        --    idle. Check that ERR_FD incremented in Node 1 and Node 2 by 1.
        --    Check that ERR_NORM remained the same in Node 1 and Node 2.
        -----------------------------------------------------------------------
        info("Step 2");

        read_error_counters(err_counters_1_1, ID_1, mem_bus(1));
        read_error_counters(err_counters_1_2, ID_2, mem_bus(2));

        CAN_generate_frame(rand_ctr, frame_1);
        frame_1.frame_format := FD_CAN;
        frame_1.brs := BR_SHIFT;
        
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));

        CAN_wait_pc_state(pc_deb_control, ID_1, mem_bus(1));
        CAN_wait_not_pc_state(pc_deb_control, ID_1, mem_bus(1));
        
        -- Now we should be in Data bit rate! This is either CRC or data field!
        wait_rand_cycles(rand_ctr, mem_bus(1).clk_sys, 0, 150);

        wait until iout(1).CAN_tx = RECESSIVE;
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);

        -- Wait for some time, unit should sample the forced dominant bus
        -- value. This can be later due to SSP, so we must wait for sufficient
        -- time of SSP! If bit-rate is fast enough, we might wait till Error
        -- frame. We don't mind since node is error active and value that
        -- we forced will be DOMINANT which is the same as during Error flag.
        -- Sowe release the bus during the error flag!
        -- Here we assume that Error flag will last less than 2500 ns. With
        -- max 1 us of Nominal bit-rate we should be fine!
        wait for 2550 ns; -- SSP could be up to 255!
        release_bus_level(so.bl_force);

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        read_error_counters(err_counters_2_1, ID_1, mem_bus(1));
        read_error_counters(err_counters_2_2, ID_2, mem_bus(2));

        check((err_counters_1_1.err_fd + 1 = err_counters_2_1.err_fd),
                "ERR_FD incremented by 1 in transmitter!");
        check(err_counters_1_2.err_fd + 1 = err_counters_2_2.err_fd,
                "ERR_FD incremented by 1 in receiver!");

        check(err_counters_1_1.err_norm = err_counters_2_1.err_norm,
                "ERR_NORM not incremented by 1 in transmitter!");
        check(err_counters_1_2.err_norm = err_counters_2_2.err_norm,
                "ERR_NORM not incremented by 1 in receiver!");

        wait for 1000 ns;

  end procedure;

end package body;
