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
--  Arbitration lost capture - RTR bit after Extended Identifier.
--
-- @Verifies:
--  @1. CAN RTR frame with extended identifier loses arbitration against
--      CAN data frame with the same identifier. 
--  @3. Arbitration lost capture position on RTR bit after Extended identifier.
--
-- @Test sequence:
--  @1. Configure both Nodes to one-shot mode.
--  @2. Generate two CAN frames with both Extended matching Identifiers.
--      Frame 1 is RTR frame, frame 2 is data frame.
--  @3. Wait till sample point in Node 1. Send Frame 1 by Node 1 and Frame 2 by
--      Node 2.
--  @4. Wait till arbitration field in Node 1. Wait till sample point 31 times
--      (11 Base ID + RTR/SRR + IDE + ID Extension + RTR). Check Node 1 is
--      transmitting Recessive, Check Node 2 is transmitting Dominant. Check
--      Node 1 lost arbitration. Check Node 2 is still transmitter. Read ALC
--      from Node 1 and check it.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    02.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package alc_rtr_ext_id_feature is
    procedure alc_rtr_ext_id_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body alc_rtr_ext_id_feature is
    procedure alc_rtr_ext_id_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable alc                :       natural;

        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;

        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_2            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        variable stat_2             :     SW_status;

        variable id_vect            :     std_logic_vector(28 downto 0);
    begin

        -----------------------------------------------------------------------
        -- @1. Configure both Nodes to one-shot mode.
        -----------------------------------------------------------------------
        info("Step 1: Configure one -shot mode");
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        CAN_enable_retr_limit(true, 0, ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- @2. Generate two CAN frames with both Extended matching Identifiers.
        --     Frame 1 is RTR frame, frame 2 is data frame.
        -----------------------------------------------------------------------
        info("Step 2: Generate CAN frames with matching IDs!");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_generate_frame(rand_ctr, frame_2);
        
        frame_1.ident_type := EXTENDED;
        frame_2.ident_type := EXTENDED;
        frame_1.rtr := RTR_FRAME;
        frame_2.rtr := NO_RTR_FRAME;
        frame_1.frame_format := NORMAL_CAN;
        frame_2.frame_format := NORMAL_CAN;
        frame_2.identifier := frame_1.identifier;

        ------------------------------------------------------------------------
        -- @3. Wait till sample point in Node 1. Send Frame 1 by Node 1 and 
        --    Frame 2 by Node 2.
        ------------------------------------------------------------------------
        info("Step 3: Send frames");
        CAN_insert_TX_frame(frame_1, 1, ID_1, mem_bus(1));
        CAN_insert_TX_frame(frame_2, 1, ID_2, mem_bus(2));
        CAN_wait_sample_point(iout(1).stat_bus);

        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- @4. Wait till arbitration field in Node 1. Wait till sample point 31
        --     times (11 Base ID + RTR/SRR + IDE + ID Extension + RTR). Check
        --     Node 1 is transmitting Recessive, Check Node 2 is transmitting
        --     Dominant. Check Node 1 lost arbitration. Check Node 2 is still
        --     transmitter. Read ALC from Node 1 and check it.
        -----------------------------------------------------------------------
        info("Step 4: Check arbitration lost on RTR after Extended ID");
        CAN_wait_pc_state(pc_deb_arbitration, ID_1, mem_bus(1));
        for i in 0 to 31 loop
            CAN_wait_sample_point(iout(1).stat_bus);
        end loop;
        check(iout(2).can_tx = DOMINANT, "Dominant RTR transmitted!");
        check(iout(1).can_tx = RECESSIVE, "Recessive RTR transmitted!");
        wait for 20 ns; -- To account for trigger processing
        
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.receiver, "Node 1 lost arbitration!");
        get_controller_status(stat_2, ID_2, mem_bus(2));
        check(stat_2.transmitter, "Node 2 transmitter!");
        
        read_alc(alc, ID_1, mem_bus(1));
        check(alc = 32, "Arbitration lost at correct bit by Node 1!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        wait for 100 ns;

  end procedure;

end package body;
