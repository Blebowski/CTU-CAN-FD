--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2021-present Ondrej Ille
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- educational, research, evaluation, self-interest purposes. Using the
-- Component for commercial purposes is forbidden unless previously agreed with
-- Copyright holder.
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
-- -------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2015-2020 MIT License
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
--  TX Priority feature test.
--
-- @Verifies:
--  @1. TX_PRIORITY register gives priority of TXT Buffers in selection for
--      transmission.
--  @2. TXT Buffer with higher priority is selected for transmission.
--  @3. When TXT Buffer of higher priority is not Ready, it will not be selected
--      for transmission.
--  @4. When two TXT Buffers have equal priority, TXT Buffer with lower priority
--      is selected for transmission.
--
-- @Test sequence:
--  @1. Generate random priorities of TXT Buffers and configure them in DUT.
--      Generate random mask of TXT Buffers which will actually be used for
--      transmission. Generate random frames for transmission and insert them
--      to DUT.
--  @2. Send atomic command to TXT Buffer 1 which will set all TX frames to
--      Ready. Wait until expected amount of frames is received.
--  @3. Check that frames are received in expected order.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--      24.3.2019   Created file
--      31.3.2019   Test completed.
--     16.11.2019   Test re-written.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package tx_priority_ftest is

    procedure tx_priority_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body tx_priority_ftest is

    procedure tx_priority_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        type CAN_frame_array_type is array (1 to 8) of SW_CAN_frame_type; 

        type t_txt_buf_priority_pair is record
            priority        :       natural range 0 to 7;
            index           :       natural range 1 to 8;
            buffer_used     :       boolean;
        end record;

        type t_txt_bufs_array is array (1 to 8) of t_txt_buf_priority_pair;

        variable txt_buf_priorities        :       t_txt_bufs_array;      
        variable tmp_buff                  :       t_txt_buf_priority_pair;

        variable CAN_frame_array_tx :       CAN_frame_array_type;   
        
        variable CAN_frame_rx       :       SW_CAN_frame_type;
        
        variable max_priority_val   :       natural range 0 to 7;	
        variable max_priority_index :       natural range 0 to 8;

        variable frame_equal        :       boolean := false;
        variable tmp_int            :       natural := 0;

        variable txt_buf_mask       :       std_logic_vector(7 downto 0) := "00000000";
        variable buffers_used       :       natural := 0;
        
        variable num_txt_bufs       :       natural;
    begin

        -----------------------------------------------------------------------
        -- @1. Generate random priorities of TXT Buffers and configure them in
        --     DUT. Generate random mask of TXT Buffers which will actually
        --     be used for transmission. Generate random frames for transmi-
        --     ssion and insert them to DUT.
        -----------------------------------------------------------------------
        info_m("Step 1");

        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);

        --  Generate random priorities and write it to TX_PRIORITY
        for j in 1 to num_txt_bufs loop
            rand_int_v(7, tmp_int);
            txt_buf_priorities(j).priority := tmp_int;
            txt_buf_priorities(j).index := j;
            
            -- Generate whether buffer will be used!
            rand_int_v(4, tmp_int);
            if (tmp_int < 4) then
                txt_buf_priorities(j).buffer_used := true;
            else
                txt_buf_priorities(j).buffer_used := false;
            end if;

            -- Write generated priority TX_PRIORITY
            CAN_configure_tx_priority(j, txt_buf_priorities(j).priority,
                DUT_NODE, chn);
        end loop;

        -- Count number of used TXT Buffers!
        for i in 1 to num_txt_bufs loop
            if (txt_buf_priorities(i).buffer_used) then
                buffers_used := buffers_used + 1;
            end if;
        end loop;

        if (buffers_used = 0) then
            buffers_used := buffers_used + 1;
            txt_buf_priorities(1).buffer_used := true;
        end if;

        -- Generate random CAN frames (generate all, some will be skipped)
        for i in 1 to num_txt_bufs loop
            CAN_generate_frame(CAN_frame_array_tx(i));
        end loop;

        -- Sort TXT Buffers based on priorities
        for i in 1 to num_txt_bufs loop
            max_priority_val := 0;
            max_priority_index := 0;

            for j in i to num_txt_bufs loop
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
        for i in 1 to num_txt_bufs - 1 loop
            for j in i+1 to num_txt_bufs loop
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
        for i in 1 to num_txt_bufs loop
            CAN_insert_TX_frame(CAN_frame_array_tx(i), txt_buf_priorities(i).index,
                                DUT_NODE, chn);
        end loop;
        
        info_m("Number of used buffers: " & integer'image(buffers_used));
        info_m("TXT Buffer configuration (highest priority first):");
        for i in 1 to num_txt_bufs loop
            info_m("Buffer index: " & integer'image(txt_buf_priorities(i).index) &
                 " Priority: " & integer'image(txt_buf_priorities(i).priority) &
                 " Used: " & boolean'image(txt_buf_priorities(i).buffer_used) &
                 " CAN ID: " & to_hstring(std_logic_vector(to_unsigned(CAN_frame_array_tx(i).identifier, 32))));
        
            if (txt_buf_priorities(i).buffer_used) then         
                txt_buf_mask(txt_buf_priorities(i).index - 1) := '1';
            end if;
        end loop;

        -----------------------------------------------------------------------
        -- @2. Send atomic command to TXT Buffer 1 which will set all TX frames
        --    to Ready. Wait until expected amount of frames is received.
        -----------------------------------------------------------------------
        info_m("Step 2");

        info_m("TXT Buffer mask: " & to_hstring(txt_buf_mask));
        
        send_TXT_buf_cmd(buf_set_ready, txt_buf_mask, DUT_NODE, chn);

        for i in 1 to buffers_used loop
            CAN_wait_frame_sent(DUT_NODE, chn);
        end loop;

        -----------------------------------------------------------------------
        -- @3. Check that frames are received in expected order.
        -----------------------------------------------------------------------
        info_m("Step 3");

        for i in 1 to num_txt_bufs loop
            
            -- Skip Buffer if it should not have been used
            if (not txt_buf_priorities(i).buffer_used) then
                next;
            end if;
            
            info_m("Reading out frame number: " & integer'image(i));

            CAN_read_frame(CAN_frame_rx, TEST_NODE, chn);
            CAN_compare_frames(CAN_frame_rx, CAN_frame_array_tx(i), false, frame_equal);
        
            if(frame_equal = false) then
                info_m("FRAMES NOT EQUAL:");
                info_m("Received frame:");
                CAN_print_frame_simple(CAN_frame_rx);
                info_m("Expected frame:");
                CAN_print_frame_simple(CAN_frame_array_tx(i));         
                error_m("Error!");
                exit;
            end if;

        end loop;

    end procedure;
end package body;
