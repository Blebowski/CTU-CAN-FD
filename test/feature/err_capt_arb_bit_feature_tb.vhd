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
--  ERR_CAPT[ERR_POS] = ERC_POS_ARB, bit error feature test. 
--
-- @Verifies:
--  @1. Detection of bit error in Arbitration field. Value of ERR_CAPT[ERR_POS]
--      when bit error should have been detected in arbitration field.
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Generate CAN frame and send it by Node 1. Wait until transmission starts
--      and wait until arbitration field. Wait for random amount of time until
--      Dominant bit is sent! Force bus low and wait until sample point. Check
--      that Error frame is being transmitted. Check value of ERR_CAPT.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    03.02.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package err_capt_arb_bit_feature is
    procedure err_capt_arb_bit_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body err_capt_arb_bit_feature is
    procedure err_capt_arb_bit_feature_exec(
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

        variable id_vect            :     std_logic_vector(28 downto 0);
        variable wait_time          :     natural;
        
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
        --  @2. Generate CAN frame and send it by Node 1. Wait until transmission
        --     starts and wait until arbitration field. Wait for random amount
        --     of time until Dominant bit is sent! Force bus low and wait until
        --     sample point. Check that Error frame is being transmitted. Check
        --     value of ERR_CAPT.
        -----------------------------------------------------------------------
        info("Step 2");
        
        CAN_generate_frame(rand_ctr, frame_1);
        frame_1.ident_type := EXTENDED;
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));
        
        CAN_wait_pc_state(pc_deb_arbitration, ID_1, mem_bus(1));
        
        -- Wait time is adjusted so that we are sure that we will still be in
        -- arbitration field (of base or extended). After 26 bits, if there are
        -- all dominant till end of frame, we are sure at least one stuff bit
        -- will be there!
        rand_int_v(rand_ctr, 25, wait_time);
        info ("Waiting for:" & integer'image(wait_time) & " bits!");

        for i in 1 to wait_time loop
            CAN_wait_sync_seg(iout(1).stat_bus);
            info("Wait sync");
            wait for 20 ns;
        end loop;
        info("Waiting finished!");

        while (iout(1).can_tx = RECESSIVE) loop
            CAN_wait_sync_seg(iout(1).stat_bus);
            wait for 20 ns;
        end loop;
        
        -- Force bus for one bit time
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns; -- To be sure that opposite bit is sampled!
        release_bus_level(so.bl_force);
        
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check (stat_1.error_transmission, "Error frame is being transmitted!");
        
        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        
        -- If Dominant stuff bit is sent and recessive is monitored, then this
        -- can be detected as Stuff Error, not as bit Error!
        check(err_capt.err_type = can_err_bit or err_capt.err_type = can_err_stuff,
                "Bit or Stuff error detected!");
        check(err_capt.err_pos = err_pos_arbitration, "Error detected in Arbitration!");
        
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        wait for 100 ns;

  end procedure;

end package body;
