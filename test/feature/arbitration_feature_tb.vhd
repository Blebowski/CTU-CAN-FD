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
--  Arbitration test generates random identifier, identifier type, frame_type,
--  RTR frame into both CAN Nodes of feature_test environment. Then it inserts
--  both frames to be transmitted immediately. Then it waits whether one of the
--  unit turns to be reciever or an error appears! It checks out which unit lost
--  the arbitration and compares with expected loser. Situations like Identifier
--  collision, Victory of Base vs extended, victory of non-RTR vs RTR frame are
--  covered in this testbench!
--
--  Test sequence:
--      1. Generate two random CAN frames.
--      2. Correct frame metadata and identifiers, to emulate arbitration of
--         BASE vs EXTENDED IDs, RTR vs. NON-RTR etc or observe collision.
--      3. Determine expected winner.
--      4. Modify data of each frame to have different values for possible
--         collision.
--      5. Insert frames for transmission and give "set_ready" command.
--      6. Wait until units start transmission.
--      7. Wait until one of the units turn receiver or collision appears.
--      8. Compare expected o.outcome with actual o.outcome.
--      9. Wait until bus is idle.
--
--------------------------------------------------------------------------------
-- Revision History:
--
--    20.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--    11.06.2018  Modified to work with HAL functions from CAN Test lib instead
--                of direct register access.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.can_constants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

package arbitration_feature is
    procedure arbitration_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body Arbitration_feature is
    procedure arbitration_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable rand_value         :       real;
        variable alc                :       natural;

        -- 0-Node 1; 1-Node 2; 2-Collision
        variable exp_winner         :     natural := 0;

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

        -- Node status
        variable stat_1             :     SW_status;
        variable stat_2             :     SW_status;

        -- Temporary variables for IDs recalculated to decimal value with
        -- identifier type taken into account
        variable ident_1            :     natural;
        variable ident_2            :     natural;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Forbid retransmitt limiting!
        ------------------------------------------------------------------------
         CAN_enable_retr_limit(false, 0, ID_1, mem_bus(1));
         CAN_enable_retr_limit(false, 0, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Generate Two random CAN Frames.
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_generate_frame(rand_ctr, frame_2);

        ------------------------------------------------------------------------
        -- Emulate special cases on the bus!
        ------------------------------------------------------------------------
        rand_real_v(rand_ctr, rand_value);

        ------------------------------------------------------------------------
        -- Colision -> ID, RTR, and ID type matching
        ------------------------------------------------------------------------
        if (rand_value < 0.1) then
            frame_1 := frame_2;

        ------------------------------------------------------------------------
        -- ID Match, RTR Match, oposite type of identifier (BASE and EXTENDED)
        ------------------------------------------------------------------------
        elsif (rand_value < 0.2) then
            frame_1.identifier := frame_2.identifier;
            frame_1.rtr := frame_2.rtr;
            frame_1.ident_type := not frame_2.ident_type;

        ------------------------------------------------------------------------
        -- ID match, Type of ID match, RTR different
        ------------------------------------------------------------------------
        elsif (rand_value < 0.3) then
            frame_1.identifier := frame_2.identifier;
            frame_1.ident_type := frame_2.ident_type;
            frame_1.rtr := not frame_2.rtr;
        end if;

        ------------------------------------------------------------------------
        -- Recalc ID to decimal value with Ident type
        ------------------------------------------------------------------------
        if (frame_1.ident_type = EXTENDED) then
            ident_1 := frame_1.identifier;
        else
            ident_1 := to_integer(unsigned(std_logic_vector'(
                        std_logic_vector(to_unsigned(frame_1.identifier, 11))
                        & "000000000000000000")));
        end if;

        if (frame_2.ident_type = EXTENDED) then
            ident_2 := frame_2.identifier;
        else
            ident_2 := to_integer(unsigned(std_logic_vector'(
                        std_logic_vector(to_unsigned(frame_2.identifier, 11))
                        & "000000000000000000")));
        end if;

        ------------------------------------------------------------------------
        -- Evaluate who should win the arbitration:
        --   1. Matching ID -> Decide based on ID type and RTR
        --   2. NON Matching ID -> Lower ID should win!
        ------------------------------------------------------------------------
        if (ident_1 = ident_2) then

            -- ID Type, ID, RTR the same -> collision!
            if (frame_1.rtr = frame_2.rtr and
                frame_1.ident_type = frame_2.ident_type)
            then
                exp_winner := 2;
                report "Expecting collision";

            -- CAN 2.0 and CAN FD frames with matching ID will cause collision!
            elsif (frame_1.frame_format /= frame_2.frame_format) then
                exp_winner := 2;
                report "Expecting collision";

            -- Same RTR, but different ident type, IDENT type selects winner!
            elsif (frame_1.ident_type = BASE and
                   frame_2.ident_type = EXTENDED)
            then
                report "Testing victory of BASE against EXTENDED";
                exp_winner := 0;

            elsif (frame_1.ident_type = EXTENDED and
                   frame_2.ident_type = BASE)
            then
                report "Testing victory of BASE against EXTENDED";
                exp_winner := 1;

            -- Same identifiers, different RTRs, RTR always selects winner!
            elsif (frame_1.rtr = NO_RTR_FRAME and
                   frame_2.rtr = RTR_FRAME)
            then
                report "Testing victory of non RTR against RTR";
                exp_winner := 0;

            elsif (frame_1.rtr = RTR_FRAME and
                   frame_2.rtr = NO_RTR_FRAME)
            then
                report "Testing victory of non RTR against RTR";
                exp_winner := 1;
            end if;

        ------------------------------------------------------------------------
        -- Frame 2 should win
        ------------------------------------------------------------------------
        elsif (ident_1 > ident_2) then
            exp_winner := 1;

        ------------------------------------------------------------------------
        -- Frame 1 should win
        ------------------------------------------------------------------------
        elsif (ident_2 > ident_1) then
            exp_winner := 0;
        end if;

        ------------------------------------------------------------------------
        -- Diferentiate data words in both frames, so that collision will
        -- occur, when it is supposed to!
        ------------------------------------------------------------------------
        frame_1.data(0) := x"AA";
        frame_2.data(0) := x"55";

        ------------------------------------------------------------------------
        -- Insert both frames to transmitt.
        ------------------------------------------------------------------------
        CAN_insert_TX_frame(frame_1, 1, ID_1, mem_bus(1));
        CAN_insert_TX_frame(frame_2, 1, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Give "set_ready" command to TXT Buffers in both CAN Nodes!
        ------------------------------------------------------------------------
        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Now we have to wait until both units starts to transmitt!!!
        ------------------------------------------------------------------------
        loop
            get_controller_status(stat_1, ID_1, mem_bus(1));
            get_controller_status(stat_2, ID_2, mem_bus(2));
            if (stat_1.transmitter and stat_2.transmitter) then
                exit;
            end if;
        end loop;

        ------------------------------------------------------------------------
        -- Loop as long as one of the units turns to be reciever, or error
        -- appears.
        ------------------------------------------------------------------------
        while (unit_rec = 0) loop

            get_controller_status(stat_1, ID_1, mem_bus(1));
            get_controller_status(stat_2, ID_2, mem_bus(2));

            -- Unit 1 turned reciever
            if (stat_1.receiver) then
                unit_rec := 1;
            end if;

            -- Unit 2 turned receiver
            if (stat_2.receiver) then
                unit_rec := 2;
            end if;

            -- Error frame transmitted by unit 1
            if (stat_1.error_transmission) then
                unit_rec := 3;
            end if;

            -- Error frame transmitted by unit 2
            if (stat_2.error_transmission) then
                unit_rec := 3;
            end if;
        end loop;

        ------------------------------------------------------------------------
        -- Check whether expected winner is the unit which lost the arbitration
        ------------------------------------------------------------------------
        if (unit_rec = 1 and exp_winner = 0) or
           (unit_rec = 2 and exp_winner = 1)
        then
            -- LCOV_EXCL_START
            report "Wrong unit lost arbitration. Expected: " &
                integer'image(exp_winner) & " Real: " & integer'image(unit_rec)
            severity error;

            report "Frame 1:";
            CAN_print_frame(frame_1, info_l);
            report "Frame 2:";
            CAN_print_frame(frame_2, info_l);

            o.outcome := false;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Send abort transmission to both frames so that no unit will
        -- attempt to retransmitt.
        ------------------------------------------------------------------------
        send_TXT_buf_cmd(buf_set_abort, 1, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_abort, 1, ID_2, mem_bus(2));

        CAN_wait_frame_sent(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Check what is the value in the ALC register
        ------------------------------------------------------------------------
        if (unit_rec = 1) then
            read_alc(alc, ID_1, mem_bus(1));
        elsif (unit_rec = 2) then
            read_alc(alc, ID_2, mem_bus(2));
        end if;

        ------------------------------------------------------------------------
        -- If error frame is transmitted and collision not have appeared
        ------------------------------------------------------------------------
        if (unit_rec = 3 and exp_winner /= 2) then
            -- LCOV_EXCL_START
            report "Collision should have appeared" severity error;

            report "Frame 1:";
            CAN_print_frame(frame_1, info_l);
            report "Frame 2:";
            CAN_print_frame(frame_2, info_l);

            o.outcome := false;
            -- LCOV_EXCL_STOP
        end if;

        wait for 100000 ns;
  end procedure;

end package body;
