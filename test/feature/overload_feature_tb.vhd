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
-- Purpose:
--  Overload frame feature test 
--
-- Verifies:
--  1. Overload frame is transmitted as a result of sampling dominant during
--     first and second bit of intermission.
--  2. Overload frame is transmitted as a result of sampling dominant during
--     last bit of end of frame by receiver.
--  3. Error frame is transmitted as a result of sampling dominant bit during
--     last bit of end of frame by transmitter.
--
-- Test sequence:
--  1. Send frame by Node 1. Wait until first bit of intermission in Node 1 and
--     force bus level Dominant. Check that Node 1 transmitts Overload frame.
--     Wait until the end of overload frame in Node 1.
--  2. Check that we are in "Intermission field now". Wait until second bit of
--     Intermission and force bus low. Wait until sample point and check that
--     Node has transmitted Overload frame. Wait until the end of Overload frame.
--     Wait until bus is idle for both Nodes.
--  3. Send frame by Node 1. Wait until last bit of End of Frame field of Node 1.
--     Force bus low. Check that Node 1 reacts with Error frame. Check that
--     Node 2 reacts with Overload frame.
--------------------------------------------------------------------------------
-- Revision History:
--    22.11.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package overload_feature is
    procedure overload_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body overload_feature is
    procedure overload_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable rand_value         :       real;
        variable alc                :       natural;

        -- Some unit lost the arbitration...
        -- 0 - initial , 1-Node 1 turned rec, 2 - Node 2 turned rec
        variable unit_rec           :     natural := 0;

        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                               (OTHERS => '0');
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_2            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        
        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;        
        variable frame_sent         :     boolean;

        variable id_vect            :     std_logic_vector(28 downto 0);
        variable command            :     SW_command := SW_command_rst_val;
        
        variable num_frames         :     integer;
        variable mode_1             :     SW_mode;
        
        variable frame_equal        :     boolean;
        variable status             :     SW_status;

    begin
        
        -----------------------------------------------------------------------
        -- 1. Send frame by Node 1. Wait until first bit of intermission in
        --    Node 1 and force bus level Dominant. Check that Node 1 transmitts
        --    Overload frame. Wait until the end of overload frame in Node 1.
        -----------------------------------------------------------------------
        info("Step 1");
        
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
        
        CAN_wait_pc_state(pc_deb_intermission, ID_1, mem_bus(1));
        wait for 15 ns;

        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 15 ns; -- To be sure sample point was processed!
        release_bus_level(so.bl_force);

        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));

        -- Now we check for whole duration of overload flag! Check that
        -- dominant bit is transmitted!
        for i in 0 to 5 loop
            info("Overload flag index: " & integer'image(i));
            check(pc_dbg = pc_deb_overload, "Overload frame transmitted!");
            CAN_wait_sample_point(iout(1).stat_bus, false);
            check(iout(1).can_tx = DOMINANT, "Dominant Overload flag transmitted!");
        end loop;

        CAN_wait_not_pc_state(pc_deb_overload, ID_1, mem_bus(1)); 

        -----------------------------------------------------------------------
        -- 2. Check that we are in "Intermission field now". Wait until second
        --    bit of Intermission and force bus low. Wait until sample point
        --    and check that Node has transmitted Overload frame. Wait until
        --    the end of Overload frame. Wait until bus is idle for both Nodes.
        -----------------------------------------------------------------------
        info("Step 2");
        
        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        check(pc_dbg = pc_deb_intermission, "Intermission after Overload frame");

        CAN_wait_sample_point(iout(1).stat_bus, false);

        -- Now we are beyond sample point in first bit of intermission!
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 15 ns; -- To be sure sample point was processed!
        release_bus_level(so.bl_force);

        -- Now we check for whole duration of overload flag! Check that
        -- dominant bit is transmitted!
        for i in 0 to 5 loop
            info("Overload flag index: " & integer'image(i));
            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            check(pc_dbg = pc_deb_overload, "Overload frame transmitted!");
            CAN_wait_sample_point(iout(1).stat_bus, false);
            check(iout(1).can_tx = DOMINANT, "Dominant Overload flag transmitted!");
        end loop;

        CAN_wait_not_pc_state(pc_deb_overload, ID_1, mem_bus(1)); 
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(2));

        -----------------------------------------------------------------------
        -- 3. Send frame by Node 1. Wait until last bit of End of Frame field
        --    of Node 1. Force bus low. Check that Node 1 reacts with Error
        --    frame. Check that Node 2 reacts with Overload frame.
        -----------------------------------------------------------------------
        info("Step 3");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);

        CAN_wait_pc_state(pc_deb_eof, ID_1, mem_bus(1));
        for i in 0 to 5 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;
        
        -- This is to cover possibility that sample point of one bit before end
        -- of EOF in Node 2 did not pass yet! We want to be also in last bit
        -- of EOF of Node 2, because only then we get Overload frame!
        wait for 400 ns;

        -- Now we should be in one bit before the end of EOF!
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 25 ns; -- To be sure sample point was processed!
        release_bus_level(so.bl_force);

        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission,
            "Transmitter sends error frame due to dominant bit in last bit of EOF!");

        -- Node 2 is transmitting overload as a result of last bit of EOF
        -- dominant during EOF! 
        CAN_wait_sample_point(iout(1).stat_bus, false);
        CAN_read_pc_debug(pc_dbg, ID_2, mem_bus(2));
        check(pc_dbg = pc_deb_overload,
            "Receiver sends overload frame due to dominant bit in last bit of EOF!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));

  end procedure;

end package body;