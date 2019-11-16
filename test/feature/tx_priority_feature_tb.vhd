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
--  TX Priority feature test.
--
-- Verifies:
--  1. TX_PRIORITY register gives priority of TXT Buffers in selection for
--     transmission.
--  2. TXT Buffer with higher priority is selected for transmission.
--  3. When TXT Buffer of higher priority is not Ready, it will not be selected
--     for transmission.
--  4. When two TXT Buffers have equal priority, TXT Buffer with lower priority
--     is selected for transmission.
--
-- Test sequence:
--  1. Generate random priorities of TXT Buffers and configure them in Node 1.
--     Generate random mask of TXT Buffers which will actually be used for
--     transmission. Generate random frames for transmission and insert them
--     to Node 1.
--  2. Send atomic command to TXT Buffer 1 which will set all TX frames to
--     Ready. Wait until expected amount of frames is received.
--  3. Check that frames are received in expected order.
--
--------------------------------------------------------------------------------
-- Revision History:
--      24.3.2019   Created file
--      31.3.2019   Test completed.
--     16.11.2019   Test re-written.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package tx_priority_feature is

    procedure tx_priority_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body tx_priority_feature is

    procedure tx_priority_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        type CAN_frame_array_type is array (1 to 4) of SW_CAN_frame_type; 

        variable data               :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable address            :       std_logic_vector(11 downto 0) :=
                                                (OTHERS => '0');

        type t_txt_buf_priority_pair is record
            priority        :       natural range 0 to 7;
            index           :       natural range 1 to 4;
            buffer_used     :       boolean;
        end record;

        type t_txt_bufs_array is array (1 to 4) of t_txt_buf_priority_pair;

        variable txt_buf_priorities        :       t_txt_bufs_array;      
        variable tmp_buff                  :       t_txt_buf_priority_pair;

        variable ID_1               :       natural := 1;   -- Transmiter
        variable ID_2               :       natural := 2;   -- Receiver          
        variable CAN_frame_array_tx :       CAN_frame_array_type;   
        
        variable CAN_frame_rx       :       SW_CAN_frame_type;
        
        variable max_priority_val   :       natural range 0 to 7;	
        variable max_priority_index :       natural range 0 to 7;

        variable priority_index_max :       natural range 0 to 8;       
        variable frame_equal        :       boolean := false;
        variable tmp_int            :       natural := 0;

        variable txt_buf_mask       :       std_logic_vector(3 downto 0) := "0000";
        variable buffers_used       :       natural := 0;

        variable found              :       boolean;

    begin
        o.outcome := true;

        -----------------------------------------------------------------------
        --  1. Generate random priorities of TXT Buffers and configure them in
        --     Node 1. Generate random mask of TXT Buffers which will actually
        --     be used for transmission. Generate random frames for transmi-
        --     ssion and insert them to Node 1.
        -----------------------------------------------------------------------

        --  Generate random priorities and write it to TX_PRIORITY
        for j in 1 to 4 loop
            rand_int_v(rand_ctr, 7, tmp_int);
            txt_buf_priorities(j).priority := tmp_int;
            txt_buf_priorities(j).index := j;
            
            -- Generate whether buffer will be used!
            rand_int_v(rand_ctr, 4, tmp_int);
            if (tmp_int < 4) then
                txt_buf_priorities(j).buffer_used := true;
            else
                txt_buf_priorities(j).buffer_used := false;
            end if;

            -- Write generated priority TX_PRIORITY
            CAN_configure_tx_priority(j, txt_buf_priorities(j).priority, ID_1, mem_bus(1));            
        end loop;

        -- Count number of used TXT Buffers!
        for i in 1 to 4 loop
            if (txt_buf_priorities(i).buffer_used) then
                buffers_used := buffers_used + 1;
            end if;
        end loop;

        if (buffers_used = 0) then
            buffers_used := buffers_used + 1;
            txt_buf_priorities(1).buffer_used := true;
        end if;

        -- Generate random CAN frames (generate all, some will be skipped)
        for i in 1 to 4 loop
            CAN_generate_frame(rand_ctr, CAN_frame_array_tx(i));
        end loop;

        -- Sort TXT Buffers based on priorities
        for i in 1 to 4 loop
            max_priority_val := 0;
            max_priority_index := 0;

            for j in i to 4 loop
                if (txt_buf_priorities(j).priority > max_priority_val) then
                    max_priority_val := txt_buf_priorities(j).priority;
                    max_priority_index := j;
                end if;
            end loop;

            -- Swap TXT Buffers with highest priority one!
            if (max_priority_val > 0) then
                tmp_buff := txt_buf_priorities(i);
                txt_buf_priorities(i) := txt_buf_priorities(max_priority_index);
                txt_buf_priorities(max_priority_index) := tmp_buff;
            end if;
        end loop;

        -- Do second sorting which orders buffers with the same priority according
        -- to lower buffer index first. This could be done together with first
        -- search, but lets keep it simple stupid.
        for i in 1 to 3 loop
            for j in i+1 to 4 loop
                if (txt_buf_priorities(i).priority = txt_buf_priorities(j).priority
                    and txt_buf_priorities(i).index > txt_buf_priorities(j).index)
                then
                    tmp_buff := txt_buf_priorities(i);
                    txt_buf_priorities(i) := txt_buf_priorities(j);
                    txt_buf_priorities(j) := tmp_buff;      
                end if;
            end loop;
        end loop;

        -- Insert CAN frames to TXT Buffers. Put first to highest priority buffer
        -- Now TXT Buffers are sorted!
        for i in 1 to 4 loop
            CAN_insert_TX_frame(CAN_frame_array_tx(i), txt_buf_priorities(i).index,
                                ID_1, mem_bus(1));
        end loop;
        
        info("Number of used buffers: " & integer'image(buffers_used));
        info("TXT Buffer configuration (highest priority first):");
        for i in 1 to 4 loop
            info("Buffer index: " & integer'image(txt_buf_priorities(i).index) &
                 " Priority: " & integer'image(txt_buf_priorities(i).priority) &
                 " Used: " & boolean'image(txt_buf_priorities(i).buffer_used) &
                 " CAN ID: " & to_hstring(std_logic_vector(to_unsigned(CAN_frame_array_tx(i).identifier, 32))));
        
            if (txt_buf_priorities(i).buffer_used) then         
                txt_buf_mask(txt_buf_priorities(i).index - 1) := '1';
            end if;
        end loop;

        -----------------------------------------------------------------------
        -- 2. Send atomic command to TXT Buffer 1 which will set all TX frames
        --    to Ready. Wait until expected amount of frames is received.
        -----------------------------------------------------------------------
        info("TXT Buffer mask: " & to_hstring(txt_buf_mask));
        
        send_TXT_buf_cmd(buf_set_ready, txt_buf_mask, ID_1, mem_bus(1));

        for i in 1 to buffers_used loop
            CAN_wait_frame_sent(ID_1, mem_bus(1));
        end loop;

        -----------------------------------------------------------------------
        -- 3. Check that frames are received in expected order.
        -----------------------------------------------------------------------
        for i in 1 to 4 loop
            
            -- Skip Buffer if it should not have been used
            if (not txt_buf_priorities(i).buffer_used) then
                next;
            end if;
            
            info("Reading out frame number: " & integer'image(i));

            CAN_read_frame(CAN_frame_rx, ID_2, mem_bus(2));
            CAN_compare_frames(CAN_frame_rx, CAN_frame_array_tx(i), false, frame_equal);
        
            if(frame_equal = false) then
                info("FRAMES NOT EQUAL:");
                info("Received frame:");
                CAN_print_frame_simple(CAN_frame_rx);
                info("Expected frame:");
                CAN_print_frame_simple(CAN_frame_array_tx(i));         
                error("Error!");
                exit;
            end if;

        end loop;

    end procedure;
end package body;
