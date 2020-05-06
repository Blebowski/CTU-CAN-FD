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
--  Jiri Novak <jnovak@fel.cvut.cz>
--  Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
--  STATUS[RXNE] feature test.
--
-- @Verifies:
--  @1. When no frame is stored in RX Buffer, STATUS[RXNE] is not set.
--  @2. When one or more frames is stored in RX Buffer, STATUS[RXNE] is set.
--  @3. STATUS[RXNE] is not set when last word of last frame in RX Buffer is
--      read.
--
-- @Test sequence:
--  @1. Read STATUS[RXNE] of Node 1 and check it is not set. Send random amount
--      of CAN frames by Node 2 and wait until they are received. Check that
--      after each one, STATUS[RXNE] is set.
--  @2. Read out frame by frame and check that STATUS[RXNE] is still set. Read
--      all frames but last one.
--  @3. Read out last frame word by word and check that STATUS[RXNE] is still
--      set and STATUS[RXNE] is not set after reading out last word.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    31.10.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package status_rxne_feature is
    procedure status_rxne_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body status_rxne_feature is
    procedure status_rxne_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;

        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;

        variable num_frames         :     integer;
    begin

        -----------------------------------------------------------------------
        --  @1. Read STATUS[RXNE] of Node 1 and check it is not set. Send
        --     random amount of CAN frames by Node 2 and wait until they are
        --     received. Check that after each one, STATUS[RXNE] is set.
        -----------------------------------------------------------------------
        info("Step 1");
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check_false(stat_1.receive_buffer, "RX Buffer empty");
        
        rand_int_v(rand_ctr, 6, num_frames);
        num_frames := num_frames + 1;
        
        CAN_generate_frame(rand_ctr, frame_1);
        frame_1.rtr := RTR_FRAME;
        frame_1.frame_format := NORMAL_CAN;
        CAN_insert_TX_frame(frame_1, 1, ID_2, mem_bus(2));
        
        for i in 0 to num_frames - 1 loop
            send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));
            CAN_wait_frame_sent(ID_2, mem_bus(2));
            
            CAN_wait_bus_idle(ID_1, mem_bus(1));
            CAN_wait_bus_idle(ID_2, mem_bus(2));
            
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check(stat_1.receive_buffer, "RX Buffer not empty");
        end loop;

        -----------------------------------------------------------------------
        --  @2. Read out frame by frame and check that STATUS[RXNE] is still set.
        --     Read all frames but last one.
        -----------------------------------------------------------------------
        info("Step 2");
        for i in 0 to num_frames - 2 loop
            CAN_read_frame(frame_rx, ID_1, mem_bus(1));
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check(stat_1.receive_buffer, "RX Buffer not empty");
        end loop;
        
        -----------------------------------------------------------------------
        --  @3. Read out last frame word by word and check that STATUS[RXNE] is
        --     still set and STATUS[RXNE] is not set after reading out last
        --     word.
        -----------------------------------------------------------------------
        for i in 0 to 3 loop -- RTR frame has 4 words in RX Buffer
            CAN_read(r_data, RX_DATA_ADR, ID_1, mem_bus(1));
            get_controller_status(stat_1, ID_1, mem_bus(1));
            
            if (i = 3) then
                check_false(stat_1.receive_buffer,
                    "STATUS[RXNE] not set after last word was read out!");
            else
                check(stat_1.receive_buffer,
                    "STATUS[RXNE] set before last word was read out!");
            end if;
        end loop;
        
        wait for 100 ns;

  end procedure;

end package body;
