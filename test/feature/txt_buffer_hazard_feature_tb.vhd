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
--  Feature test for immediate abortion of CAN Transmission.
--
--  Test sequence:
--      1. Generate CAN Frame, and start transmission by Node 1.
--      2. Wait until transmission is started, plus some random time.
--      3. Read protocol control state, and if unit is still in the frame,
--         send "abort" command via COMMAND register.
--      4. Wait 5 clock cycles and check if unit stopped being transmitter,
--         if not report an error.
--      5. Wait until the other unit transmitts error frame which is caused
--         by sudden disapearing of transmitter during frame.
--      6. Clear error counters.
--
--------------------------------------------------------------------------------
-- Revision History:
--     22.6.2016  Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--     12.6.2018  Modified test to use HAL like functions from CAN Test lib
--                instead of raw register access functions.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package abort_transmittion_feature is

    procedure abort_transmittion_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );

end package;


package body abort_transmittion_feature is

    procedure abort_transmittion_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    )is
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






	for i in 1 to 2 loop

		------------------------------------------------------------------------
		-- Generate CAN frame
		------------------------------------------------------------------------
		CAN_generate_frame(rand_ctr, CAN_frame);

		------------------------------------------------------------------------
		-- Insert the frame for transmittion
		------------------------------------------------------------------------
		CAN_insert_TX_frame(CAN_frame, 1, ID_1, mem_bus(1));


  		-- Give "Set ready" command to the buffer
       		 send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));

		------------------------------------------------------------------------
		-- Wait until unit turns transmitter
		------------------------------------------------------------------------
		loop
		    get_controller_status(status, ID_1, mem_bus(1));
		    if (status.transmitter) then
		        exit;
		    end if;
		end loop;

		------------------------------------------------------------------------
		-- Now wait random time up to 250 000 clock cycles!
		-- But at least 200 clock cycles! We want to avoid situation that unit
		-- aborts immediately after the frame was commited and SOF was not
		-- yet sent!

		-- TODO: rewrite with non random wait (more precisely defined time steps)
		------------------------------------------------------------------------
		--wait_rand_cycles(std_logic_vector(i*unsigned(rand_ctr)), mem_bus(1).clk_sys, 200, 250000);
	
		for k in 10 to (50*i) loop
			wait until rising_edge(mem_bus(1).clk_sys);
		end loop;


		-- Give "Set abort" command to the buffer
       		 send_TXT_buf_cmd(buf_set_abort, 1, ID_1, mem_bus(1));

		
		-- Now wait for few clock cycles until Node aborts the transmittion
		for i in 0 to 5 loop
			wait until rising_edge(mem_bus(1).clk_sys);
		end loop;


		-- Read "TXT Buffer state"
		get_tx_buf_state(1, txt_buf_state, ID_1, mem_bus(1));

		



		-- Read status of CTU CAN FD controller.
		get_controller_status(status, ID_1, mem_bus(1));


		-- Is controller transmitting?
		if (status.transmitter) then
			-- Is also TXT buffer in transmittin state?
			if (txt_buf_state = buf_tx_progress) then
				-- Everything is ok
				report "Both TXT buffer and controller in tx progress state.";

				-- Wait until bus is idle 
				--------------------------------------------------------------------
				CAN_wait_bus_idle(ID_1, mem_bus(1));

				-- Check that unit is now idle since it is after transmittion already
				get_controller_status(status, ID_1, mem_bus(1));
				if (not status.bus_status) then
					-- LCOV_EXCL_START
					report "Unit is not Idle!" severity error;
					o.outcome := false;
					-- LCOV_EXCL_STOP
				end if;
			else
				-- Inconsistency happened 
				report "status: ";
		 		report "Inconsistence: CAN controller transmitter=TRUE, TXT buffer=" & to_string(txt_buf_state) & " [" & to_string(i) & "]" severity error;
				
				o.outcome := false;
			end if;
		else 
			-- Is also TXT buffer in aborted state?
			if (txt_buf_state = buf_aborted) then
				-- Everything is ok
				report "Both TXT buffer and controller in aborted state.";		
			else
				-- Inconsistency happened
		 		report "Inconsistence: CAN controller transmitter=FALSE, TXT buffer=" & to_string(txt_buf_state) & " [" & to_string(i) & "]" severity error;
				o.outcome := false;
			end if;
		end if;
	end loop;
  end procedure;

end package body;
