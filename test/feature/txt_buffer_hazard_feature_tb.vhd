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
--  This test will verify that TX Buffer datapath is hazard free.
--
--  Test sequence:  
--      1. Insert frame to TXT Buffer
--      2. Mark the buffer as ready.
--      3. Immediately send set_abort command.
--      4. Readout status of the buffer.
--      5. If the buffer is "aborted", check that no transmission is in progress
--         (e.g. via STATUS), throw an error if not.
--      6. If the buffer is "abort in progress" check that transmission is 
--         in progress and wait till its end. Throw an error if not.
--      7. If buffer is in any other state, throw an error.  
--
--------------------------------------------------------------------------------
-- Revision History:
--      17.1.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package txt_buffer_hazard_feature is

    procedure txt_buffer_hazard_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body txt_buffer_hazard_feature is

    procedure txt_buffer_hazard_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable w_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable frame_length       :       natural;
        variable rand_value         :       real;
        variable wt                 :       time;
        variable bus_config         :       bit_time_config_type;
        variable still_tx           :       boolean := false;
        variable PC_State           :       protocol_type;
        variable err_counters       :       SW_error_counters;
        variable command            :       SW_command := (false, false, false);
        variable status             :       SW_status;
	    variable txt_buf_state	    :	    SW_TXT_Buffer_state_type;	
  begin
        o.outcome := true;

    -- Wait till Integration phase is over
    for i in 0 to 3000 loop
        wait until rising_edge(mem_bus(1).clk_sys);
    end loop;

    -- Generate CAN frame
    CAN_generate_frame(rand_ctr, CAN_frame);

    -- Insert the frame for transmittion
    CAN_insert_TX_frame(CAN_frame, 1, ID_1, mem_bus(1));

    -- Repeat test several times
    for i in 1 to 150 loop

        -- Give "Set ready" command to the buffer
        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));

        -- Wait for some clock cycles before sending abort command.
        for j in 0 to i loop
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;

        -- Give "Set abort" command to the buffer
        send_TXT_buf_cmd(buf_set_abort, 1, ID_1, mem_bus(1));

        -- Wait for some clock cycles before reading buffer and controller state
        for i in 0 to 20 loop
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;

        -- Read "TXT Buffer state"
        get_tx_buf_state(1, txt_buf_state, ID_1, mem_bus(1));

        -- Read status of CTU CAN FD controller.
        get_controller_status(status, ID_1, mem_bus(1));

        -- Is controller transmitting?
        if (status.transmitter) then
            -- Is also TXT buffer in transmitting state? Or if set_abort command
            -- comes just at the time when the core LOCKs the buffer, it goes to 
            -- "Abort in Progress".
            if (txt_buf_state = buf_tx_progress) or 
                (txt_buf_state = buf_ab_progress) then
                -- Everything is ok
                info("Transmitting: Both TXT buffer and controller " & 
                     "in consistence state." & " [" & to_string(i) & "]");
         
                -- Wait until bus is idle 
                ------------------------------------------------------------------
                CAN_wait_bus_idle(ID_1, mem_bus(1));

                -- Is the unit now in idle since it is after transmittion already?
                get_controller_status(status, ID_1, mem_bus(1));
                check(status.bus_status, "Unit is not Idle!");
            else
                -- Inconsistency happened
                check_failed("Inconsistence: CAN controller transmitter=TRUE, TXT " &
                             "buffer=" & to_string(txt_buf_state) &
                             " [" & to_string(i) & "]");
            end if;
        else 
            -- Is also TXT buffer in aborted state?
            if (txt_buf_state = buf_aborted) then
                -- Everything is ok
                info("Both TXT buffer and controller in aborted state" &
                     " [" & to_string(i) & "]");		 
            else
                -- Inconsistency happened
               check_failed("Inconsistence: CAN controller transmitter=FALSE, TXT " &
                            "buffer=" & to_string(txt_buf_state) & 
                            " [" & to_string(i) & "]");
            end if;
        end if;
	end loop;
  end procedure;
end package body;
