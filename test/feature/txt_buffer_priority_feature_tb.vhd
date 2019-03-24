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
        variable data             	:       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable address			:		std_logic_vector(11 downto 0) :=
												(OTHERS => '0');
                                                

        variable ID_1           	:       natural := 1;		-- Transmiter
        variable ID_2           	:       natural := 2;		-- Receiver
        variable CAN_frame          :       SW_CAN_frame_type;
        
        type CAN_frame_array_type is array (1 to 4) of SW_CAN_frame_type;       
        variable CAN_frame_array_tx	:		CAN_frame_array_type;   
        variable CAN_frame_array_rx	:		CAN_frame_array_type;   
        
        type priority_array_type is array (1 to 4) of natural range 0 to 8;       
        variable priority_array		:		priority_array_type;
         variable priority_array_bckp		:		priority_array_type;
        variable priority_value_tmp       :       natural range 0 to 8;
        variable priority_value_max       :       natural range 0 to 8;
        variable priority_index_max			:  	natural range 0 to 8;
        
        variable frame_equal         :       boolean := false;
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
	    
	            -- Node status
        variable stat_1             :     SW_status;
        variable stat_2             :     SW_status;
        
                -- Some unit lost the arbitration...
        -- 0 - initial , 1-Node 1 turned rec, 2 - Node 2 turned rec
        variable unit_rec           :     natural := 0;
  begin
        o.outcome := true;

    -- Wait till Integration phase is over
 --   for i in 0 to 3000 loop
 ---       wait until rising_edge(mem_bus(1).clk_sys);
 --   end loop;
    
    -- TODO Generate priorities
    priority_array(1) := 6;		-- Priority of TXT Buffer 1
    priority_array(2) := 1;		-- Priority of TXT Buffer 2
    priority_array(3) := 2;		-- Priority of TXT Buffer 3
    priority_array(4) := 3;		-- Priority of TXT Buffer 4
    
    for i in 1 to 4 loop
    	priority_array_bckp(i) := priority_array(i);
    end loop; 
    
   -- Convert priorities to vector
    data := std_logic_vector(to_unsigned(0, 16)) &
    		std_logic_vector(to_unsigned(priority_array(4), 4)) &	-- Priority of TXT Buffer 4
		    std_logic_vector(to_unsigned(priority_array(3), 4)) &	-- Priority of TXT Buffer 3
		    std_logic_vector(to_unsigned(priority_array(2), 4)) &	-- Priority of TXT Buffer 2
		    std_logic_vector(to_unsigned(priority_array(1), 4));	-- Priority of TXT Buffer 1
		    
    -- Set tx buffer priorities
 	--data := std_logic_vector(to_unsigned(17185, data'length)); -- 17185: B4 1st, B3 2nd, B2 3rd, B1 last.
 	address := TX_PRIORITY_ADR;
    CAN_write(data, address, ID_1, mem_bus(1), BIT_16);
    
    
       -- seradit odeslane ramce podle priorit
    --- najit nejvyssi prioritu
    --- ramec dat na prvni pozici
    --- najit druhou nejvyssi prioritu, dal ramec hned za to
    

	
    
  	-- prvni pozice v poli = frame s nejvyssi prioritou
  	-- druhy frame, druha nejvyssi priorita
	for j in 1 to 4 loop
	    priority_value_tmp := 0;
	    priority_value_max := 0;
	    for i in 1 to 4 loop
		    if((priority_array_bckp(i) > priority_value_max) AND (priority_array_bckp(i) /= 8)) then
				priority_value_max := priority_array_bckp(i);
				priority_index_max := i;
				
				
			end if;    	
		end loop; 
		
		-- Nejvyssi prioritu ma buffer cislo 'priority_index_max'
		priority_array_bckp(priority_index_max) := 8;
		info("Priority indexy: " & Integer'image(priority_index_max) & ":");
	
	
		-- Frame s nejvyssi prioritou je v poli prvni.
		-- At nize dam frame do bufferu s jakymkoliv cislem, prvni frame v poli musi dorazit prvni.
    	-- Generate CAN frame
		--CAN_generate_frame(rand_ctr, CAN_frame);
		CAN_generate_frame(rand_ctr, CAN_frame_array_tx(j));

		-- Frame s nejvyssi prioritou dam do bufferu cislo 'priority_index_max'.
		-- Insert the frame for transmittion
    	-- CAN_insert_TX_frame(CAN_frame, i, ID_1, mem_bus(1));
    	CAN_insert_TX_frame(CAN_frame_array_tx(j), priority_index_max, ID_1, mem_bus(1));
 	end loop;   
    

    -- TODO atomic access
  	for i in 1 to 4 loop
  		-- Give "Set ready" command to the buffer
  		send_TXT_buf_cmd(buf_set_ready, i, ID_1, mem_bus(1));
  	end loop;  
  	
  	-- Atomic set ready command
  	--send_TXT_buf_cmd(buf_set_ready, "1111", ID_1, mem_bus(1));
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
    
  info("DEBUG 2");
  
        ------------------------------------------------------------------------
    -- Loop as long as one of the units turns to be reciever, or error
    -- appears.
    ------------------------------------------------------------------------
  --  while (unit_rec = 0) loop
    --    get_controller_status(stat_1, ID_1, mem_bus(1));

        -- Unit 1 turned reciever
    --    if (stat_1.receiver) then
    --        unit_rec := 1;
    --    end if;

        -- Error frame transmitted by unit 1
   --     if (stat_1.error_transmission) then
   --        unit_rec := 3;
  --      end if;

--    end loop;
    
   info("DEBUG 3");   
   
   
    --CAN_wait_bus_idle(ID_1, mem_bus(1));
	
	
	for i in 1 to 4 loop
		--info("Read Frame 1:");
		info("Read Frame " & Integer'image(i) & ":");
 
		CAN_read_frame(CAN_frame_array_rx(i), ID_2, mem_bus(2));
	end loop; 
        
        
        
    
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
		--CAN_compare_frames(CAN_frame_array_rx(i), CAN_frame_array_tx(i), false, frame_equal);
		
		--info("TX frame number  " & Integer'image(i) & ":");
		---CAN_print_frame_simple(CAN_frame_array_tx(i));
		--info("RX frame number  " & Integer'image(i) & ":");
		--CAN_print_frame_simple(CAN_frame_array_rx(i));
		--check_false(frame_equal,"FRAMES ARE DIFFERENT!");
	end loop; 
          
        
        
     --  CAN_compare_frames();
        

    -- Repeat test several times
    --for i in 1 to 150 loop

        -- Give "Set ready" command to the buffer
       -- send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));

        -- Wait for some clock cycles before sending abort command.
        --for j in 0 to i loop
       --     wait until rising_edge(mem_bus(1).clk_sys);
       -- end loop;

        -- Give "Set abort" command to the buffer
      --  send_TXT_buf_cmd(buf_set_abort, 1, ID_1, mem_bus(1));

        -- Wait for some clock cycles before reading buffer and controller state
       -- for i in 0 to 20 loop
       --     wait until rising_edge(mem_bus(1).clk_sys);
       -- end loop;

        -- Read "TXT Buffer state"
     --   get_tx_buf_state(1, txt_buf_state, ID_1, mem_bus(1));

        -- Read status of CTU CAN FD controller.
      --  get_controller_status(status, ID_1, mem_bus(1));

        -- Is controller transmitting?
      --  if (status.transmitter) then
            -- Is also TXT buffer in transmitting state? Or if set_abort command
            -- comes just at the time when the core LOCKs the buffer, it goes to 
            -- "Abort in Progress".
      --      if (txt_buf_state = buf_tx_progress) or 
         --       (txt_buf_state = buf_ab_progress) then
                -- Everything is ok
       --         info("Transmitting: Both TXT buffer and controller " & 
           --          "in consistence state." & " [" & to_string(i) & "]");
         
                -- Wait until bus is idle 
                ------------------------------------------------------------------
           --     CAN_wait_bus_idle(ID_1, mem_bus(1));

                -- Is the unit now in idle since it is after transmittion already?
           --     get_controller_status(status, ID_1, mem_bus(1));
           --     check(status.bus_status, "Unit is not Idle!");
          --  else
                -- Inconsistency happened
          --      check_failed("Inconsistence: CAN controller transmitter=TRUE, TXT " &
           --                  "buffer=" & to_string(txt_buf_state) &
           --                  " [" & to_string(i) & "]");
          --  end if;
     --   else 
            -- Is also TXT buffer in aborted state?
     --       if (txt_buf_state = buf_aborted) then
                -- Everything is ok
      --          info("Both TXT buffer and controller in aborted state" &
      --               " [" & to_string(i) & "]");		 
       --     else
                -- Inconsistency happened
       --        check_failed("Inconsistence: CAN controller transmitter=FALSE, TXT " &
       --                     "buffer=" & to_string(txt_buf_state) & 
       --                     " [" & to_string(i) & "]");
       --     end if;
      --  end if;
	--end loop;
  end procedure;
end package body;
