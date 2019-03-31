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
--  The aim here is to test whether TX arbitrator selects a frame from TXT
--  Buffer with the highest priority and sends it, in case of more TXT Buffers
--  in "READY" state.
--
--  Test sequence:  
--      1. Generate and set random priorities to TX buffers.
--      2. Sort the priorities by size (highest priority first)
--      3. Generate CAN frame for each TX buffer
--      4. Give set ready command to all buffers at the same time
--      5. Wait until all frames are received on the second node
--      6. Check if all frames were received in correct order.
--
--------------------------------------------------------------------------------
-- Revision History:
--      24.3.2019   Created file
--      31.3.2019   Test completed.
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

        variable data               :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable address            :       std_logic_vector(11 downto 0) :=
                                                (OTHERS => '0');
        variable ID_1               :       natural := 1;   -- Transmiter
        variable ID_2               :       natural := 2;   -- Receiver          
        variable CAN_frame_array_tx :       CAN_frame_array_type;   
        variable CAN_frame_array_rx :       CAN_frame_array_type;                
        variable priority_array     :       priority_array_type;
        variable priority_value_tmp :       natural range 0 to 8;	
        variable priority_value_max :       natural range 0 to 8;
        variable priority_index_max :       natural range 0 to 8;       
        variable frame_equal        :       boolean := false;
        variable tmp_int            :       natural := 0;

    begin
        o.outcome := true;
            
        --  Generate random priorities and write it to the Priority register
        for j in 1 to 4 loop
            rand_int_v(rand_ctr, 7, tmp_int);   -- Get random number
            priority_array(j) := tmp_int;     -- Priority of TXT Buffer N
            wait for 0 ns;  -- Wait to 'refresh' random generator
            
            -- Write generated priority to the register
            CAN_configure_tx_priority(j, priority_array(j), ID_1, mem_bus(1)); 
        end loop;


        -- Sorting buffer priorities - descending. First generate frame to buffer 
        -- with the highest priority. In each iteration find unfulfilled buffer with 
        -- the highest available priority and insert new frame to it .
        for j in 1 to 4 loop
            priority_value_tmp := 0;    -- Init value for each iteration.
            priority_value_max := 0;    -- Init value for each iteration.

            -- Find unused buffer with the highest priority (used buffer = priority 8)
            for i in 1 to 4 loop
                -- Use only ">" in this statement because documenntion says
                -- "If two buffers have equal priorities, the one with lower index is selected"
                if((priority_array(i) > priority_value_max) AND
                    (priority_array(i) /= 8)) then
                    priority_value_max := priority_array(i);    -- The highest priority	found
                    priority_index_max := i;    -- Index of buffer with the highest priority
                end if;
            end loop; 
    
            -- Print buffer number with its priority value.
            info("[" & Integer'image(j) & "] Priority " & 
                    Integer'image(priority_array(priority_index_max)) & " in buffer " & 
                    Integer'image(priority_index_max) & ".");
    
            -- Mark buffer as used
            priority_array(priority_index_max) := 8;
    
            -- Generate CAN frame
            CAN_generate_frame(rand_ctr, CAN_frame_array_tx(j));
            CAN_print_frame_simple(CAN_frame_array_tx(j));
    
            -- Insert the frame for transmittion
            CAN_insert_TX_frame(CAN_frame_array_tx(j), priority_index_max, ID_1, mem_bus(1));
        end loop;   
        info("Note: higher priority value -> higher priority");
        
        
        -- Atomic set ready command
        send_TXT_buf_cmd(buf_set_ready, "1111", ID_1, mem_bus(1));
        info("Set ready command done.");
    
        -- Wait until all four frames are sent
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
        info("TX frames (highest priority first)");
        for i in 1 to 4 loop
            CAN_print_frame_simple(CAN_frame_array_tx(i));
        end loop; 
        
        info("RX frames (in order of receive):");
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
        info("Test successful.");
    end procedure;
end package body;
