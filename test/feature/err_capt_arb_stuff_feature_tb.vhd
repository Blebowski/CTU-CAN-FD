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
--  ERR_CAPT[ERR_POS] = ERC_POS_ARB, stuff error feature test. 
--
-- @Verifies:
--  @1. Detection of stuff error in Arbitration field. Value of ERR_CAPT[ERR_POS]
--      when stuff error should have been detected in arbitration field.
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Generate CAN frame, force ID to contain first 4 bits dominant. Send such
--      frame by Node 1. Wait until arbitration field in Node 1. Wait for 4
--      bits and after that force bit dominant (stuff bit should be recessive).
--      Wait until Sample point and check that Error frame is being transmitted
--      by Node 1. Check that ERR_CAPT signals Stuff Error during arbitration.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    12.01.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package err_capt_arb_stuff_feature is
    procedure err_capt_arb_stuff_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body err_capt_arb_stuff_feature is
    procedure err_capt_arb_stuff_feature_exec(
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

        -- Node status
        variable stat_1             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;    

        variable frame_sent         :     boolean;
        
        variable err_capt           :     SW_error_capture;

    begin

        -----------------------------------------------------------------------
        -- @1. Check that ERR_CAPT contains no error (post reset).
        -----------------------------------------------------------------------
        info("Step 1");
        
        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_pos = err_pos_other, "Reset of ERR_CAPT!");
        
        -----------------------------------------------------------------------        
        -- @2. Generate CAN frame, force ID to contain first 4 bits dominant.
        --     Send such frame by Node 1. Wait until arbitration field in Node
        --     1. Wait for 4 bits and after that force bit dominant (stuff bit
        --     should be recessive). Wait until Sample point and check that
        --     Error frame is being transmitted by Node 1. Check that ERR_CAPT
        --     signals Stuff Error during arbitration.
        -----------------------------------------------------------------------
        info("Step 2");
        
        CAN_generate_frame(rand_ctr, frame_1);
        frame_1.ident_type := BASE;
        frame_1.identifier := 1; -- First 4 bits should be dominant!
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));
        CAN_wait_pc_state(pc_deb_arbitration, ID_1, mem_bus(1));

        for i in 1 to 4 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;
        wait for 20 ns;
        
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns;

        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.error_transmission, "Error frame transmitted by Node 1!");
        release_bus_level(so.bl_force);

        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_type = can_err_stuff, "Stuff error detected!");
        check(err_capt.err_pos = err_pos_arbitration, "Error detected in Arbitration!");
                
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        -- The other node shoul have also detected stuff error in arbitration (due to
        -- that Error frame), check it!
        CAN_read_error_code_capture(err_capt, ID_2, mem_bus(2));
        check(err_capt.err_type = can_err_stuff, "Stuff error detected!");
        check(err_capt.err_pos = err_pos_arbitration, "Error detected in Arbitration!");

        wait for 100 ns;

  end procedure;

end package body;
