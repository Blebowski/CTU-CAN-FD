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
--  ERR_CAPT CRC error feature test. 
--
-- @Verifies:
--  @1. Detection of CRC error when calculated CRC is not equal to received
--      CRC. Value of ERR_CAPT when CRC error should have been detected in
--      ACK field.
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Generate CAN frame and send it by Node 1. Wait until CRC field in Node 2
--      and wait for random number of bits. Force CAN_RX of Node 2 for duration
--      of 1 bit to opposite value (to mess up received CRC of Node 2). Wait
--      until ACK bit in Node 1. Force bus low (as if other node is ACKing the
--      frame) and wait until sample point. Wait until ACK delimiter of Node 2.
--      Wait until Node 2 is NOT in ACK delimiter and check that it transmitts
--      Error frame. Check that ERR_CAPT contains CRC Error.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    12.01.2020   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package err_capt_crc_err_feature is
    procedure err_capt_crc_err_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body err_capt_crc_err_feature is
    procedure err_capt_crc_err_feature_exec(
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
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;    

        variable frame_sent         :     boolean;
        
        variable err_capt           :     SW_error_capture;
        variable mode_2             :     SW_mode := SW_mode_rst_val;
        variable wait_time          :     natural;
    begin

        -----------------------------------------------------------------------
        -- @1. Check that ERR_CAPT contains no error (post reset).
        -----------------------------------------------------------------------
        info("Step 1");
        
        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_pos = err_pos_other, "Reset of ERR_CAPT!");
        
        -----------------------------------------------------------------------        
        -- @2. Generate CAN frame and send it by Node 1. Wait until CRC field
        --     in Node 2 and wait for random number of bits. Force CAN_RX of
        --     Node 2 for duration of 1 bit to opposite value (to mess up
        --     received CRC of Node 2). Wait until ACK bit in Node 1. Force bus
        --     low (as if other node is ACKing the frame) and wait until sample
        --     point. Wait until ACK delimiter of Node 2. Wait until Node 2 is
        --     NOT in ACK delimiter and check that it transmitts Error frame.
        --     Check that ERR_CAPT contains CRC Error.
        -----------------------------------------------------------------------
        info("Step 2");
        
        CAN_generate_frame(rand_ctr, frame_1);
        frame_1.frame_format := NORMAL_CAN; --Use CAN 2.0 to have sinle bit ACK
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
        
        CAN_wait_pc_state(pc_deb_crc, ID_2, mem_bus(2));
        rand_int_v(rand_ctr, 14, wait_time);

        info("waiting for:" & integer'image(wait_time) & " bits!");
        for i in 1 to wait_time loop
            CAN_wait_sync_seg(iout(2).stat_bus);
        end loop;
        wait for 100 ns; -- To be sure we are ssuficiently far in the bit!

        -- Force can_rx of Node 2 to oposite value!
        -- TODO: There is a drawback here that test might fail when we flip
        --       stuff bit! If we do so, then error frame will come sooner!
        force_can_rx(not iout(2).can_rx, ID_2, so.crx_force, so.crx_inject,
                        so.crx_index);
        CAN_wait_sample_point(iout(2).stat_bus);
        release_can_rx(so.crx_force);

        -- Generate ACK for Node 1. We have to wait till both Nodes are in ACK so
        -- that we don't accidentaly generate it for Node 2 while it is still in
        -- in CRC Delimiter, that would be form error!
        CAN_wait_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        CAN_wait_pc_state(pc_deb_ack, ID_2, mem_bus(2));
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        check(iout(1).can_tx = RECESSIVE, "Send Recessive ACK upon CRC error [1]!");
        CAN_wait_sample_point(iout(1).stat_bus);
        check(iout(1).can_tx = RECESSIVE, "Send Recessive ACK upon CRC error [2]!");
        wait for 20 ns;
        release_bus_level(so.bl_force);

        -- Wait till End of ACK delimiter of Node 2. Now Node 1 should have
        -- received ACK from TB, therefore it should not be transmitting
        -- error frame from ACK delimiter! But Node 2 should not have sent ACK
        -- and should have mismatching CRC, therefore after ACK delimiter,
        -- it should send Error frame!
        CAN_wait_pc_state(pc_deb_ack_delim, ID_2, mem_bus(2));
        CAN_wait_not_pc_state(pc_deb_ack_delim, ID_2, mem_bus(2));
        wait for 20 ns;

        -- Now state has changed, we should be in Error frame because ACK was
        -- recessive
        get_controller_status(stat_2, ID_2, mem_bus(2));
        check(stat_2.error_transmission, "Error frame transmitted by Node 2!");

        CAN_read_error_code_capture(err_capt, ID_2, mem_bus(2));
        check(err_capt.err_type = can_err_crc, "CRC error detected!");
        check(err_capt.err_pos = err_pos_ack,
            "Error detected in CRC Delim/ACK/ACK Delim field!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        wait for 100 ns;

  end procedure;

end package body;
