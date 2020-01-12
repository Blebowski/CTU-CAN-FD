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
--  Stuff bit on last bit of Data field.
--
-- @Verifies:
--  @1. When stuff bit is inserted after last bit of Data field, no extra stuff
--      bit will be inserted during first bit of Stuff count (with fixed
--      stuffing). Wait until frame is sent. Read frame and check it is received
--      succesfully!
--
-- @Test sequence:
--  @1. Generate CAN frame which has last 5 bits of data field dominant! Send
--      it by Node 1. Wait until frame is sent and read it from Node 2. Check
--      that frames are equal.
--      Note: The check is only visual so far...
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--   02.12.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package stuff_in_data_feature is
    procedure stuff_in_data_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body stuff_in_data_feature is
    procedure stuff_in_data_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        constant ID_1               :        natural := 1;
        constant ID_2               :        natural := 2;
        variable CAN_frame          :        SW_CAN_frame_type;
        variable CAN_frame_2        :        SW_CAN_frame_type  := 
                    (0, (OTHERS => (OTHERS => '0')), "0000", 0, '0', '0',
                     '0', '0', '0', (OTHERS => '0'), 0);
        variable frame_sent         :        boolean;
        variable frames_equal       :        boolean;
    begin

        ----------------------------------------------------------------------
        -- @1. Generate CAN frame which has last 5 bits of data field dominant!
        --    Send it by Node 1. Wait until frame is sent and read it from
        --    Node 2. Check that frames are equal. Note: The actual check is
        --    done by assertion!
        ----------------------------------------------------------------------

        info("Step 1");
        CAN_generate_frame(rand_ctr, CAN_frame);
        
        -- Use only DLC of 1, as data byte 1 is set so that stuff bit is
        -- inserted at last bit of data field!
        CAN_frame.dlc := "0001";
        CAN_frame.data(0) := x"21";
        CAN_frame.rtr := NO_RTR_FRAME;
        CAN_frame.frame_format := FD_CAN;
        decode_dlc(CAN_frame.dlc, CAN_frame.data_length);
        decode_dlc_rx_buff(CAN_frame.dlc, CAN_frame.rwcnt);

        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(2));

        CAN_read_frame(CAN_frame_2, ID_2, mem_bus(2));
        CAN_compare_frames(CAN_frame, CAN_frame_2, false, frames_equal);
        check(frames_equal, "Frame received OK!");

  end procedure;

end package body;