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
--  This test verifies if two consecutive write (without gap cycle), first
--  clearing overrun flag, second clearing overrun interrupt, might malfunction
--  due to two clock cycle delay on clearing overrun (which is caused by
--  pipeline architecture).
--  
--  Test sequence:
--      1. Configure Node 1 to capture data overrun interrupts.
--      2. Release Receie Buffer of Node 1.
--      3. Generate frame and start transmission by Node 2.
--      4. Upon the end of transmission, check if Data overrun flag is set
--         on Node 1. If no, return to point 3.
--      4. Read data overrun interrupt status and make sure it is set.
--      5. Clear data overrun flag.
--      6. Clear interrupt status of Data Overrun. Make sure these two conse-
--         cutive write cycles have only one clock cycle gap (no waits in
--         between)!
--      7. Check if overrun interrupt was cleared by reading interrupt status
--         again!
--------------------------------------------------------------------------------
-- Revision History:
--
--    9.9.2018     Created file 
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

use work.CAN_FD_register_map.all;

package data_overrun_clear_feature is
    procedure data_overrun_clear_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
	);
end package;


package body data_overrun_clear_feature is
    procedure data_overrun_clear_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :        std_logic_vector(31 downto 0) :=
                                                 (OTHERS => '0');
        variable CAN_frame          :        SW_CAN_frame_type;
        variable frame_sent         :        boolean := false;
        variable ID_1               :        natural := 1;
        variable ID_2               :        natural := 2;

        variable interrupts         :        SW_interrupts :=
                                                (false, false, false,
                                                 false, false, false,
                                                 false, false, false,
                                                 false, false, false);
        variable command            :        SW_command := (false, false, false);
        variable status             :        SW_status;

    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Set Node 1 to capture Data overrun interrupts, unmask it too!
        ------------------------------------------------------------------------
        interrupts.data_overrun_int := true;
        write_int_enable(interrupts, ID_1, mem_bus(1));
        interrupts.data_overrun_int := false;
        write_int_mask(interrupts, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Release RX Buffer of Node 1
        ------------------------------------------------------------------------
        command.release_rec_buffer := true;
        give_controller_command(command, ID_1, mem_bus(1));
        command.release_rec_buffer := false;

        ------------------------------------------------------------------------
        -- Generate CAN frame and send it until data Overrun occurs on Node 1
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        while (true) loop
            CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
            CAN_wait_frame_sent(ID_1, mem_bus(1));
            get_controller_status(status, ID_1, mem_bus(1));
            if (status.data_overrun) then
                report "Overrun Flag Detected!";                
                exit;
            end if;
        end loop;
        
        ------------------------------------------------------------------------
        -- Check Interrupt Status on Data Overrun!
        ------------------------------------------------------------------------
        read_int_status(interrupts, ID_1, mem_bus(1));
        if (not interrupts.data_overrun_int) then
            report "Data overrun Interrupt not captured as expected!"
                severity error;
        end if;

        ------------------------------------------------------------------------
        -- Clear data overrun flag and clear data overrun interrupt!
        ------------------------------------------------------------------------
        command.clear_data_overrun := true;
        give_controller_command(command, ID_1, mem_bus(1));
        interrupts.data_overrun_int := true;
        clear_int_status(interrupts, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Read interrupt status and check data overrun was cleared!
        ------------------------------------------------------------------------  
        read_int_status(interrupts, ID_1, mem_bus(1));
        if (interrupts.data_overrun_int) then
            report "Data overrun Interrupt was not cleared!" severity error;
        end if;

    end procedure;

end package body;
