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
--  Jiri Novak <jnovak@fel.cvut.cz>
--  Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
--  Start of transmission from Intermission 
--
-- @Verifies:
--  @1. Transmission is started when Node detects Dominant bit during third
--      bit of intermission and it has frame for transmission available.
--  @2. Reception is started when Node detectes Dominant bit during third bit
--      of Intermission and it has no frame for transmission available.
--
-- @Test sequence:
--  @1. Insert CAN frame for transmission into Node 2. Wait until transmission
--      will be started. Insert CAN frame to Node 1 during transmission of frame
--      from Node 2 and wait until Intermission.
--  @2. Wait for two sample points and force the bus dominant during third bit
--      of intermission for Node 1. Wait until sample point, and check that Node
--      1 started transmitting. Check that Node 1 is in "arbitration" phase.
--      Check that Node 1 is NOT in SOF. Wait until frame is sent, and check
--      it is properly receieved by Node 2 (Node 2 should have turned receiver).
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    22.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package tx_from_intermission_feature is
    procedure tx_from_intermission_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body tx_from_intermission_feature is
    procedure tx_from_intermission_feature_exec(
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
        variable frame_2            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        variable frame_sent         :     boolean;
        variable frame_equal        :     boolean;
    begin

        -----------------------------------------------------------------------
        -- @1. Insert CAN frame for transmission into Node 2. Wait until
        --    transmission will be started. Insert CAN frame to Node 1 during 
        --    transmission of frame from Node 2 and wait until Intermission.
        -----------------------------------------------------------------------
        info("Step 1");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_send_frame(frame_1, 1, ID_2, mem_bus(2), frame_sent);

        CAN_wait_tx_rx_start(true, false, ID_2, mem_bus(2));
        wait for 5000 ns; -- To be sure Node 1 started reception!

        CAN_generate_frame(rand_ctr, frame_2);
        CAN_send_frame(frame_2, 1, ID_1, mem_bus(1), frame_sent);

        -- We must wait for Intermission of Node 1! Only that way we are sure
        -- we properly measure two bits of its intermission!
        CAN_wait_pc_state(pc_deb_intermission, ID_1, mem_bus(1));

        -----------------------------------------------------------------------
        -- @2. Wait for two sample points and force the bus dominant during
        --    third bit of intermission for Node 1. Wait until sample point,
        --    and check that Node 1 started transmitting. Check that Node 1 is
        --    in "arbitration" phase. Check that Node 1 is NOT in SOF. Wait 
        --    until frame is sent, and check it is properly receieved by Node 2
        --    (Node 2 should have turned receiver).
        -----------------------------------------------------------------------
        info("Step 2");
        CAN_wait_sample_point(iout(1).stat_bus, false);
        CAN_wait_sample_point(iout(1).stat_bus, false);

        -- This is needed to be sure that Node 2 also reached second sample
        -- point of intermission. Otherwise, it would interpret this as
        -- overload condition, and it would not turn reciever!
        wait for 100 ns;

        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 15 ns; -- To be sure sample point was processed!
        release_bus_level(so.bl_force);
        
        -- Now Node 1 thinks that third bit of its intermission was dominant.
        -- It should start transmitting with first bit of Base ID. Node 2 should
        -- also interpret this as SOF and start receiving.
        -- Hopefully, Node 2 clock will not be too slow so that it will still
        -- catch this as second bit of Intermission and Interpret this as
        -- Overload frame ...

        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));     
        check(pc_dbg = pc_deb_arbitration, "Node 2 in arbitration");
        check_false(pc_dbg = pc_deb_sof, "Node 2 NOT in SOF!");

        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.transmitter, "Node 1 transmitter!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(2));

        CAN_read_frame(frame_rx, ID_2, mem_bus(2));
        CAN_compare_frames(frame_rx, frame_2, false, frame_equal);

        check(frame_equal, "TX/RX frame match");
        
        wait for 100 ns;

  end procedure;

end package body;
