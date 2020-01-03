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
--  ERR_CAPT[ERR_POS] = ERC_POS_CTRL, form error feature test. 
--
-- Verifies:
--  1. Detection of form error in control field on r0 bit in CAN 2.0 base frame,
--     on r0 bit in CAN FD base frame, on r0/r1 bits in CAN 2.0 extended frame
--     and r0 in CAN FD extended frame!
--  2. Value of ERR_CAPT[ERR_POS] when form error shall be detected in control
--     field of CAN frame!
--
-- Test sequence:
--  1. Check that ERR_CAPT contains no error (post reset).
--  2. Generate CAN frame (CAN 2.0 Base only, CAN FD Base only, CAN 2.0 Extended,
--     CAN FD extended), send it by Node 1. Wait until Arbitration field and wait
--     for 13 (Base ID, RTR, IDE) or 14 (Base ID, RTR, IDE, EDL) or 32 bits
--     (Base ID, SRR, IDE, Ext ID, RTR) or 33 (Base ID, SRR, IDE, Ext ID, RTR, 
--     r1) or 33 (Base ID, SRR, IDE, Ext ID, RTR, EDL) bits based on frame type.
--     Force bus Recessive (reserved bits are dominant) and wait until sample
--     point. Check that Node is transmitting error frame. Check that ERR_CAPT
--     signals Form Error in Control field. Reset the node, Wait until integration
--     is over and check that ERR_CAPT is at its reset value (this is to check
--     that next loops will truly set ERR_CAPT). Repeat with each frame type!
--------------------------------------------------------------------------------
-- Revision History:
--    03.02.2020   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package err_capt_ctrl_form_feature is
    procedure err_capt_ctrl_form_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body err_capt_ctrl_form_feature is
    procedure err_capt_ctrl_form_feature_exec(
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

        variable id_vect            :     std_logic_vector(28 downto 0);
        variable wait_time          :     natural;
        
        variable err_counters_1_1   :     SW_error_counters;
        variable err_counters_1_2   :     SW_error_counters;

        variable err_counters_2_1   :     SW_error_counters;
        variable err_counters_2_2   :     SW_error_counters;
        
        variable frame_sent         :     boolean;
        
        variable err_capt           :     SW_error_capture;

    begin

        -----------------------------------------------------------------------
        -- 1. Check that ERR_CAPT contains no error (post reset).
        -----------------------------------------------------------------------
        info("Step 1");
        
        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_pos = err_pos_other, "Reset of ERR_CAPT!");
        
        -----------------------------------------------------------------------
        -- 2. Generate CAN frame (CAN 2.0 Base only, CAN FD Base only, CAN 2.0
        --    Extended, CAN FD extended), send it by Node 1. Wait until
        --    Arbitration field and wait for 13 (Base ID, RTR, IDE) or 14 (Base
        --    ID, RTR, IDE, EDL) or 34 bits (Base ID, SRR, IDE, Ext ID, RTR) or
        --    35 (Base ID, SRR, IDE, Ext ID, RTR, r1) or 35 (Base ID, SRR, IDE,
        --    Ext ID, RTR, EDL) bits based on frame type. Force bus Recessive
        --    (reserved bits are dominant) and wait until sample point. Check
        --    that Node is transmitting error frame. Check that ERR_CAPT signals
        --    Form Error in Control field. Reset the node, Wait until integration
        --    is over and check that ERR_CAPT is at its reset value (this is to
        ---   check that next loops will truly set ERR_CAPT). Repeat with each
        --    frame type!
        -----------------------------------------------------------------------
        for i in 1 to 5 loop
            info ("Inner Loop: " & integer'image(i));
            CAN_generate_frame(rand_ctr, frame_1);
            
            -- ID is not important in this TC. Avoid overflows of high generated
            -- IDs on Base IDs!
            frame_1.identifier := 10;
            -- This is to avoid failing assertions on simultaneous RTR and EDL
            -- flag (if r0 is corrupted by TC to be recessive!). RTR flag is
            -- not important in this TC, therefore we can afford to fixate it!
            frame_1.RTR := NO_RTR_FRAME;
            
            case i is
            when 1 =>
                frame_1.frame_format := NORMAL_CAN;
                frame_1.ident_type := BASE;
                wait_time := 13; -- Till r0
            when 2 =>
                frame_1.frame_format := FD_CAN;
                frame_1.ident_type := BASE;
                wait_time := 14; -- Till r0
            when 3 =>
                frame_1.frame_format := NORMAL_CAN;
                frame_1.ident_type := EXTENDED;
                wait_time := 32; -- Till r1
            when 4 =>
                frame_1.frame_format := NORMAL_CAN;
                frame_1.ident_type := EXTENDED;
                wait_time := 33; -- Till r0
            when 5 =>
                frame_1.frame_format := FD_CAN;
                frame_1.ident_type := EXTENDED;
                wait_time := 33; -- Till r0
            end case;
            
            CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
            CAN_wait_pc_state(pc_deb_arbitration, ID_1, mem_bus(1));
            
            info("Waiting for: " & integer'image(wait_time) & " bits!");
            for j in 1 to wait_time loop
                CAN_wait_sample_point(iout(1).stat_bus, true);
            end loop;
            
            -- Force bus for one bit time
            force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
            CAN_wait_sample_point(iout(1).stat_bus, false);
            wait for 20 ns; -- To be sure that opposite bit is sampled!
            release_bus_level(so.bl_force);
            
            -- Check errors
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check (stat_1.error_transmission,
                    "Error frame is being transmitted!");
        
            CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
            check(err_capt.err_type = can_err_form, "Form error detected!");
            check(err_capt.err_pos = err_pos_ctrl,
                    "Error detected in Control field!");
            wait for 100 ns; -- For debug only to see waves properly!

            -- Reset the node
            exec_SW_reset(ID_1, mem_bus(1));
            CAN_turn_controller(true, ID_1, mem_bus(1));
            CAN_wait_bus_on(ID_1, mem_bus(1));
            CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
            check(err_capt.err_pos = err_pos_other, "Reset value other");
        end loop;

        wait for 100 ns;

  end procedure;

end package body;
