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
--  No Start of Frame feature test - frame transmission!
--
-- Verifies:
--  1. When a dominant bit is sampled in Bus idle and a frame is available for
--     transmission, its transmission is started without transmitting SOF bit.
--  2. When CTU CAN FD joins transmission without transmitting SOF bit, it
--     accounts SOF bit as transmitted dominant bit in number of equal conse-
--     cutive bits.
--
-- Test sequence:
--  1. Configure both Nodes to one-shot mode.
--  2. Insert CAN frames which have first 5 bits of identifier equal to zero to
--     both nodes. Check both nodes are Idle. Wait till Sample point in Node 1.
--  3. Send Set ready command to both nodes. Wait until Node 1 is not in Bus
--     idle state. Check it is transmitting Base Identifier (NOT SOF)!
--  4. Wait until sample point 5 times (5th bit of idetifier) in Node 1. Check
--     Node 1 is transmitting Recessive bit (Stuff bit).
--  5. Wait until frame is over. Check frame was received OK, read it from 
--     receiving node and verify it was received OK!
--------------------------------------------------------------------------------
-- Revision History:
--    07.10.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package no_sof_tx_feature is
    procedure no_sof_tx_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body no_sof_tx_feature is
    procedure no_sof_tx_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable rand_value         :       real;
        variable alc                :       natural;

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
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        
        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;
        
        constant id_template        :     std_logic_vector(10 downto 0) :=
                "01010101010";
        variable id_var             :     std_logic_vector(10 downto 0) :=
                 (OTHERS => '0');
                 
        variable pc_state           :     SW_PC_Debug;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- 1. Configure both Nodes to one-shot mode.
        ------------------------------------------------------------------------
        info("Step 1: Configure one -shot mode");
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        CAN_enable_retr_limit(true, 0, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- 2. Insert CAN frames which have first 5 bits of identifier equal to
        --    zero to both nodes. Check both nodes are Idle. Wait till Sample
        --    point in Node 2.
        ------------------------------------------------------------------------
        info("Step 2: Insert CAN frames!");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_generate_frame(rand_ctr, frame_2);
        frame_1.ident_type := BASE;
        frame_2.ident_type := BASE;
        frame_1.identifier := 1;
        frame_2.identifier := 2;
        -- Use FD can frames, they contain stuff count!!
        frame_1.frame_format := FD_CAN;
        frame_2.frame_format := FD_CAN;
        CAN_insert_TX_frame(frame_1, 1, ID_1, mem_bus(1));
        CAN_insert_TX_frame(frame_2, 1, ID_2, mem_bus(2));
        CAN_wait_sample_point(iout(2).stat_bus);
        
        ------------------------------------------------------------------------
        -- 3. Send Set ready command to both nodes. Wait until Node 1 is not in
        --    Bus idle state. Check it is transmitting Base Identifier (NOT SOF)!
        ------------------------------------------------------------------------
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));
        CAN_wait_sample_point(iout(2).stat_bus);
        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        
        -- Wait until bus is not idle by Node 1!
        get_controller_status(stat_1, ID_1, mem_bus(1));
        while (stat_1.bus_status) loop
            get_controller_status(stat_1, ID_1, mem_bus(1));
        end loop;

        CAN_read_pc_debug(pc_state, ID_1, mem_bus(1));
        check(pc_state = pc_deb_arbitration, "Node 1 did not transmitt SOF!");
        wait for 20 ns;

        ------------------------------------------------------------------------
        -- 4. Wait until sample point 5 times (5th bit of idetifier) in Node 1.
        --    Check Node 1 is transmitting Recessive bit (Stuff bit).
        ------------------------------------------------------------------------
        for i in 0 to 4 loop
            CAN_wait_sample_point(iout(1).stat_bus, skip_stuff_bits => false);
        end loop;
        check(iout(1).can_tx = RECESSIVE, "Stuff bit inserted!");

        ------------------------------------------------------------------------
        -- 5. Wait until frame is over. Check frame was received OK, read it 
        --    from receiving node and verify it was received OK!
        ------------------------------------------------------------------------
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        get_tx_buf_state(1, txt_buf_state, ID_1, mem_bus(1));
        check(txt_buf_state = buf_done, "Frame transmitted OK!");
        
        get_rx_buf_state(rx_buf_info, ID_2, mem_bus(2));
        check(rx_buf_info.rx_frame_count = 1, "Frame received OK!");

        CAN_read_frame(frame_rx, ID_2, mem_bus(2));
        CAN_compare_frames(frame_rx, frame_1, false, frames_equal);
        check(frames_equal, "TX vs. RX frames match!");

    wait for 1000 ns;
  end procedure;

end package body;
