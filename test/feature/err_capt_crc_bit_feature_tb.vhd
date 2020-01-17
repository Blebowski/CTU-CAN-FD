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
--  ERR_CAPT[ERR_POS] = ERC_POS_CRC feature test - bit error.
--
-- @Verifies:
--  @1. Detection of bit error in CRC field.
--  @2. Value of ERR_CAPT when bit error is detected in CRC field.
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Generate CAN frame, send it by Node 1. Wait until CRC field. Wait for
--      random duration of CRC field. Force bus to opposite value as transmitted
--      bit wait until sample point. Check that error frame is being transmitted.
--      Check that ERR_CAPT signals bit error in CRC field!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    03.02.2020   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package err_capt_crc_bit_feature is
    procedure err_capt_crc_bit_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body err_capt_crc_bit_feature is
    procedure err_capt_crc_bit_feature_exec(
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

        variable frame_sent         :     boolean;
        
        variable err_capt           :     SW_error_capture;
        variable tmp                :     natural;
        variable crc_len            :     natural;

    begin

        -----------------------------------------------------------------------
        -- @1. Check that ERR_CAPT contains no error (post reset).
        -----------------------------------------------------------------------
        info("Step 1");

        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_pos = err_pos_other, "Reset of ERR_CAPT!");

        -----------------------------------------------------------------------
        -- @2. Generate CAN frame, send it by Node 1. Wait until data field.
        --    Wait for random duration of data field. Force bus to opposite
        --    value as transmitted bit wait until sample point. Check that
        --    error frame is being transmitted. Check that ERR_CAPT signals bit
        --    error in data field!
        -----------------------------------------------------------------------
        info("Step 2");

        CAN_generate_frame(rand_ctr, frame_1);
        frame_1.rtr := NO_RTR_FRAME;
        
        -- Don't sample by SSP!
        frame_1.brs := BR_NO_SHIFT;
        
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));
        
        if (frame_1.frame_format = FD_CAN) then
            CAN_wait_pc_state(pc_deb_stuff_count, ID_1, mem_bus(1));
        else
            CAN_wait_pc_state(pc_deb_crc, ID_1, mem_bus(1));
        end if;

        -- Wait for random number of bits
        if (frame_1.frame_format = FD_CAN) then
            if (frame_1.data_length > 16) then
                crc_len := 24; -- CRC21 + Stuff count + Parity - 1
            else
                crc_len := 20; -- CRC17 + Stuff count + Parity - 1
            end if;
        else
            crc_len := 15; -- CRC 15 (CAN 2.0 frames have no Stuff count)!
        end if;

        -- Wait for Random amount of bits, but not longer than CRC field!
        rand_int_v(rand_ctr, crc_len - 1, tmp);
        info("Waiting for: " & integer'image(tmp) & " bits!");
        for i in 1 to tmp loop
            CAN_wait_sample_point(iout(1).stat_bus, true);
        end loop;

        CAN_wait_sync_seg(iout(1).stat_bus);
        wait for 20 ns;

        force_bus_level(not iout(1).can_tx, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns; -- To be sure that opposite bit is sampled!
        release_bus_level(so.bl_force);
        
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check (stat_1.error_transmission, "Error frame is being transmitted!");
        
        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        
        -----------------------------------------------------------------------
        -- It might happend that we corrupt a bit which is just stuff bit (due
        -- to randomization). If this is a CAN FD frame, this will be considered
        -- as form error (stuff error during fixed bit stuffing shall be reported
        -- as form error). In such case, form error has higher priority than
        -- stuff error, so form error is reported. Tolerate this as this is not
        -- a bug!
        -----------------------------------------------------------------------
        if (frame_1.frame_format = FD_CAN) then
            check(err_capt.err_type = can_err_bit or
                  err_capt.err_type = can_err_form, "Bit or Form error detected!");
        else
            check(err_capt.err_type = can_err_bit, "Bit error detected!");
        end if;
            
        check(err_capt.err_pos = err_pos_crc, "Error detected in CRC field!");
        
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        wait for 100 ns;

  end procedure;

end package body;
