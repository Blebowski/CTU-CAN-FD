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
--  ERR_CAPT[ERR_POS] = ERC_POS_CTRL, bit error feature test. 
--
-- @Verifies:
--  @1. Detection of bit error in IDE bit of frame with Base identifier!
--  @2. Detection of bit error in EDL bit of CAN FD frame with Base identifier
--      and with Extended Identifier!
--  @3. Detection of bit error in ESI/BRS and DLC bit fields.
--  @4. Value of ERR_CAPT[ERR_POS] when bit error shall be detected in control
--      field of CAN frame!
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Generate CAN frame (frame with Base ID only, CAN FD frames with Base and
--      extended identifier, CAN FD frame with Base identifier), send it by
--      Node 1. Wait until Arbitration field and until sample point of one bit
--      before bit error shall be detected. Force bus to opposite value as shall
--      be transmitted and wait until sample point. Check that Node is transmitting
--      error frame. Check that ERR_CAPT signals Bit Error in Control field.
--      Reset the node, Wait until integration is over and check that ERR_CAPT
--      is at its reset value (this is to check that next loops will truly set
--      ERR_CAPT). Repeat with each frame type!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    03.02.2020   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package err_capt_ctrl_bit_feature is
    procedure err_capt_ctrl_bit_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body err_capt_ctrl_bit_feature is
    procedure err_capt_ctrl_bit_feature_exec(
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
  
        variable wait_time          :     natural;

        variable frame_sent         :     boolean;
        
        variable err_capt           :     SW_error_capture;
        variable tmp                :     natural;
        
        variable force_value        :     std_logic := '0';
    begin

        -- Other controller is not need in this test. Disable it not to have
        -- failing assertions due to force bit errors!
        CAN_turn_controller(false, ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- @1. Check that ERR_CAPT contains no error (post reset).
        -----------------------------------------------------------------------
        info("Step 1");
        
        CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
        check(err_capt.err_pos = err_pos_other, "Reset of ERR_CAPT!");
        
        -----------------------------------------------------------------------
        -- @2. Generate CAN frame (frame with Base ID only, CAN FD frames with
        --    Base and extended identifier, CAN FD frame with Base identifier),
        --    send it by Node 1. Wait until Arbitration field and until sample
        --    point of one bit before bit error shall be detected. Force bus to
        --    opposite value as shall be transmitted and wait until sample point.
        --    Check that Node is transmitting error frame. Check that ERR_CAPT
        --    signals Bit Error in Control field. Reset the node, Wait until
        --    integration is over and check that ERR_CAPT is at its reset value
        --    (this is to check that next loops will truly set ERR_CAPT).
        --    Repeat with each frame type!
        -----------------------------------------------------------------------
        for i in 1 to 4 loop
            info ("Inner Loop: " & integer'image(i));
            CAN_generate_frame(rand_ctr, frame_1);

            -- Detect patterns in which stuff bit might be placed. In such 
            -- case, avoid it. Because if we stuff the same value of bit as
            -- we are trying to force, error frame will not be sent (obviously,
            -- bus has equal value as is sent) and test will fail!
            if (frame_1.dlc(3) = frame_1.dlc(2)) then
                
                -- It is enough to break first two equal bits!
                frame_1.dlc(3) := not frame_1.dlc(2);
                decode_dlc(frame_1.dlc, frame_1.data_length);
            end if;

            -- ID is not important in this TC. Avoid overflows of high generated
            -- IDs on Base IDs!
            frame_1.identifier := 10;
            -- This is to avoid failing assertions on simultaneous RTR and EDL
            -- flag (if r0 is corrupted by TC to be recessive!). RTR flag is
            -- not important in this TC, therefore we can afford to fixate it!
            frame_1.RTR := NO_RTR_FRAME;
            
            case i is
            when 1 =>
                frame_1.ident_type := BASE;
                wait_time := 12; -- Till IDE
                force_value := RECESSIVE;
            when 2 =>
                frame_1.frame_format := FD_CAN;
                frame_1.ident_type := BASE;
                wait_time := 13; -- Till EDL
                force_value := DOMINANT;
            when 3 =>
                frame_1.frame_format := FD_CAN;
                frame_1.ident_type := EXTENDED;
                wait_time := 32; -- Till EDL
                force_value := DOMINANT;
            when 4 =>
                frame_1.frame_format := FD_CAN;
                frame_1.ident_type := BASE;
                wait_time := 15; -- Till r0
                
                -- Extend wait time to random BRS,ESI or DLC
                rand_int_v(rand_ctr, 5, tmp);
                wait_time := wait_time + tmp;
                
                -- Force value:
                -- BRS -> Opposite of BRS
                -- ESI -> Recessive (we are error active so we transmit dominabt)
                -- DLC -> opposite of n-th bit of DLC!
                case tmp is
                when 0 =>
                    force_value := not frame_1.brs;
                    info("Forcing BRS to dominant!");
                when 1 =>
                    force_value := RECESSIVE;
                    info("Forcing ESI to recessive!");
                when 2 =>
                    force_value := not frame_1.dlc(3);
                    info("Forcing DLC(3)");
                when 3 =>
                    force_value := not frame_1.dlc(2);
                    info("Forcing DLC(2)");
                when 4 =>
                    force_value := not frame_1.dlc(1);
                    info("Forcing DLC(1)");
                when 5 =>
                    force_value := not frame_1.dlc(0);
                    info("Forcing DLC(0)");
                when others =>
                    error("Invalid generated number!");
                end case;
            end case;
            
            CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
            CAN_wait_pc_state(pc_deb_arbitration, ID_1, mem_bus(1));
            
            info("Waiting for: " & integer'image(wait_time) & " bits!");
            for j in 1 to wait_time loop
                CAN_wait_sample_point(iout(1).stat_bus, true);
            end loop;
            
            -- Force bus for one bit time
            force_bus_level(force_value, so.bl_force, so.bl_inject);
            CAN_wait_sample_point(iout(1).stat_bus, false);
            wait for 20 ns; -- To be sure that opposite bit is sampled!
            release_bus_level(so.bl_force);
            
            -- Check errors
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check (stat_1.error_transmission,
                    "Error frame is being transmitted!");
        
            CAN_read_error_code_capture(err_capt, ID_1, mem_bus(1));
            check(err_capt.err_type = can_err_bit, "Bit error detected!");
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
