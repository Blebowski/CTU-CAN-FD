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
--  
--
--  Test sequence:  
--      1. 
--      2. 
--
--------------------------------------------------------------------------------
-- Revision History:
--      24.3.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package txt_buffer_priority_feature is

    procedure txt_buffer_priority_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body txt_buffer_priority_feature is

    procedure txt_buffer_priority_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
    	type CAN_frame_array_type is array (1 to 4) of SW_CAN_frame_type; 
    	type priority_array_type is array (1 to 4) of natural range 0 to 8;          
    	
        variable data             	:       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable address			:		std_logic_vector(11 downto 0) :=
												(OTHERS => '0');
        variable ID_1           	:       natural := 1;		-- Transmiter
        variable ID_2           	:       natural := 2;		-- Receiver          
        variable CAN_frame_array_tx	:		CAN_frame_array_type;   
        variable CAN_frame_array_rx	:		CAN_frame_array_type;                
        variable priority_array		:		priority_array_type;
        variable priority_value_tmp :       natural range 0 to 8;	
        variable priority_value_max :       natural range 0 to 8;
        variable priority_index_max	:  		natural range 0 to 8;       
        variable frame_equal        :       boolean := false;

  begin
  	o.outcome := true;


    
    -- TODO Generate priorities
    priority_array(1) := 3;		-- Priority of TXT Buffer 1
    priority_array(2) := 2;		-- Priority of TXT Buffer 2
    priority_array(3) := 3;		-- Priority of TXT Buffer 3
    priority_array(4) := 5;		-- Priority of TXT Buffer 4
    
    --for i in 1 to 4 loop
    ----	priority_array_bckp(i) := priority_array(i);
    --end loop; 
    
   -- Convert priorities to vector
    data := std_logic_vector(to_unsigned(0, 16)) &
    		std_logic_vector(to_unsigned(priority_array(4), 4)) &	-- Priority of TXT Buffer 4
		    std_logic_vector(to_unsigned(priority_array(3), 4)) &	-- Priority of TXT Buffer 3
		    std_logic_vector(to_unsigned(priority_array(2), 4)) &	-- Priority of TXT Buffer 2
		    std_logic_vector(to_unsigned(priority_array(1), 4));	-- Priority of TXT Buffer 1
		    
    -- Set tx buffer priorities
 	address := TX_PRIORITY_ADR;
    CAN_write(data, address, ID_1, mem_bus(1), BIT_16);
   

	
    -- Sorting buffer priorities - descending. First generate frame to buffer 
    -- with the highest priority. In each iteration find unfulfilled buffer with 
    -- the highest available priority and insert new frame to it .
	for j in 1 to 4 loop
	    priority_value_tmp := 0;	-- Init value for each iteration.
	    priority_value_max := 0;	-- Init value for each iteration.
		    
		-- Find unused buffer with the highest priority (used buffer = priority 8)
	    for i in 1 to 4 loop
	    	-- Use only ">" in this statement because documenntion says
	    	-- "If two buffers have equal priorities, the one with lower index is selected"
		    if((priority_array(i) > priority_value_max) AND 
		    	(priority_array(i) /= 8)) then
				priority_value_max := priority_array(i);	-- The highest priority	found
				priority_index_max := i;		-- Index of buffer with the highest priority				
			end if;    	
		end loop; 
		
		-- Print buffer number with its priority value.		
		info("[" & Integer'image(j) & "] Priority " & 
				Integer'image(priority_array(priority_index_max)) & " in buffer " & 
				Integer'image(priority_index_max) & ".");
				
		-- Mark buffer as used
		priority_array(priority_index_max) := 8;	
	
    	-- Generate CAN frame		-
		CAN_generate_frame(rand_ctr, CAN_frame_array_tx(j));
		CAN_print_frame_simple(CAN_frame_array_tx(j));

		-- Insert the frame for transmittion
    	CAN_insert_TX_frame(CAN_frame_array_tx(j), priority_index_max, ID_1, mem_bus(1));
 	end loop;   
 	info("Note: higher priority value -> higher priority");
    
  	
  	-- Atomic set ready command
  	send_TXT_buf_cmd(buf_set_ready, "1111", ID_1, mem_bus(1));
  	info("Set ready command done.");
  	
    ------------------------------------------------------------------------
    -- Now we have to wait until units starts to transmitt!!!
    ------------------------------------------------------------------------
    --loop
    --    get_controller_status(stat_1, ID_1, mem_bus(1));
    --   if (stat_1.transmitter) then
    --       exit;
	--	end if;
    -- end loop;
    CAN_wait_frame_sent(ID_1, mem_bus(1));
	CAN_wait_frame_sent(ID_1, mem_bus(1));
    CAN_wait_frame_sent(ID_1, mem_bus(1));
    CAN_wait_frame_sent(ID_1, mem_bus(1));
    
	
	-- Read all frames on the second node
	for i in 1 to 4 loop
		info("Read Frame " & Integer'image(i) & ":");
		CAN_read_frame(CAN_frame_array_rx(i), ID_2, mem_bus(2));
	end loop; 
        
   	-- Generate some debug logs.
 	info("TX frames:");
    for i in 1 to 4 loop
    	CAN_print_frame_simple(CAN_frame_array_tx(i));
    end loop; 
    
 	info("RX frames:");
    for i in 1 to 4 loop
    	CAN_print_frame_simple(CAN_frame_array_rx(i));
    end loop; 
    	        
    	        
    -- Eval results    
  	for i in 1 to 4 loop
		CAN_compare_frames(CAN_frame_array_tx(i), CAN_frame_array_rx(i), false, frame_equal);
		
		if(frame_equal = false) then
			info("FRAMES NOT EQUAL:");
			CAN_print_frame(CAN_frame_array_rx(i));
			CAN_print_frame(CAN_frame_array_tx(i));			
			o.outcome := false;
			exit;
		end if;

		-- check_false(frame_equal = true, "FRAMES ARE DIFFERENT!");
	end loop; 
          

   
  end procedure;
end package body;
