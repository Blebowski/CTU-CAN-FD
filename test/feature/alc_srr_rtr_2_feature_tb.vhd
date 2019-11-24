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
--  Arbitration lost capture - SRR/RTR bit feature test 2.
--
-- Verifies:
--  1. CAN FD frame wins over CAN 2.0 RTR frame with the same identifier.
--  2. CAN FD frame wins over CAN frame with Extended identifier with the same
--     base identifier.
--  3. Arbitration lost capture position on SRR/RTR bit after Base identifier.
--
-- Test sequence:
--  1. Configure both Nodes to one-shot mode.
--  2. Generate two CAN frames: Frame 1 - CAN FD Base frame, Frame 2 - CAN 2.0
--     RTR frame with Base ID. Base identifier of both CAN frames is matching!
--  3. Wait till sample point in Node 1. Send Frame 1 by Node 1 and Frame 2 by
--     Node 2.
--  4. Wait till arbitration field in Node 1. Wait till sample point 12 times
--     (11 Base ID + RTR/SRR). Check Node 1 is transmitting dominant, Check
--     Node 2 is transmitting recessive. Check Node 2 lost arbitration. Check
--     Node 1 is still transmitter. Read ALC from Node 2 and check it.
--  5. Generate two CAN Frames: Frame 1 - CAN FD frame with Base ID. Frame 2 -
--     Frame with Extended Identifier.
--  6. Wait till sample point in Node 1. Send Frame 1 by Node 1 and Frame 2 by
--     Node 2.
--  7. Wait till arbitration field in Node 2. Wait till sample point 12 times
--     (11 Base ID + RTR/SRR). Check Node 2 is transmitting recessive, Check
--     Node 1 is transmitting dominant. Check Node 2 lost arbitration. Check
--     Node 1 is still transmitter. Read ALC from Node 2 and check it.
--------------------------------------------------------------------------------
-- Revision History:
--    02.10.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package alc_srr_rtr_2_feature is
    procedure alc_srr_rtr_2_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body alc_srr_rtr_2_feature is
    procedure alc_srr_rtr_2_feature_exec(
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

        variable id_vect            :     std_logic_vector(28 downto 0);
    begin
        o.outcome := true;

        -----------------------------------------------------------------------
        -- 1. Configure both Nodes to one-shot mode.
        -----------------------------------------------------------------------
        info("Step 1: Configure one -shot mode");
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        CAN_enable_retr_limit(true, 0, ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- 2. Generate two CAN frames: Frame 1 - CAN FD Base frame, Frame 2 - 
        --    CAN 2.0 RTR frame with Base ID. Base identifier of both CAN 
        --    frames is matching!
        -----------------------------------------------------------------------
        info("Step 2: Generate CAN frames with matching IDs!");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_generate_frame(rand_ctr, frame_2);
        
        frame_1.ident_type := BASE;
        frame_2.ident_type := BASE;
        frame_2.rtr := RTR_FRAME;
        frame_1.frame_format := FD_CAN;
        frame_2.frame_format := NORMAL_CAN;
        frame_2.identifier := (frame_2.identifier mod 2**11);
        frame_1.identifier := frame_2.identifier;

        ------------------------------------------------------------------------
        -- 3. Wait till sample point in Node 1. Send Frame 1 by Node 1 and 
        --    Frame 2 by Node 2.
        ------------------------------------------------------------------------
        info("Step 3: Send frames");
        CAN_insert_TX_frame(frame_1, 1, ID_1, mem_bus(1));
        CAN_insert_TX_frame(frame_2, 1, ID_2, mem_bus(2));
        CAN_wait_sample_point(iout(1).stat_bus);

        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- 4. Wait till arbitration field in Node 1. Wait till sample point 12
        --    times (11 Base ID + RTR/SRR). Check Node 1 is transmitting
        --    dominant, Check Node 2 is transmitting recessive. Check Node 2 
        --    lost arbitration. Check Node 1 is still transmitter. Read ALC
        --    from Node 2 and check it.
        -----------------------------------------------------------------------
        info("Step 4: Check arbitration lost on SRR/RTR");
        CAN_wait_pc_state(pc_deb_arbitration, ID_2, mem_bus(2));
        for i in 0 to 11 loop
            CAN_wait_sample_point(iout(2).stat_bus);
        end loop;
        check(iout(1).can_tx = DOMINANT, "Dominant SRR transmitted!");
        check(iout(2).can_tx = RECESSIVE, "Recessive RTR transmitted!");
        wait for 20 ns; -- To account for trigger processing
        
        get_controller_status(stat_2, ID_2, mem_bus(2));
        check(stat_2.receiver, "Node 2 lost arbitration!");
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.transmitter, "Node 1 transmitter!");
        
        read_alc(alc, ID_2, mem_bus(2));
        check(alc = 12, "Arbitration lost at correct bit by Node 1!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- 5. Generate two CAN Frames: Frame 1 - CAN FD frame with Base ID. 
        --    Frame 2 - Frame with Extended Identifier.
        -----------------------------------------------------------------------
        info("Step 5: Generate frames");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_generate_frame(rand_ctr, frame_2);

        frame_1.identifier := (frame_1.identifier mod 2**11);
        frame_1.frame_format := FD_CAN;
        frame_1.ident_type := BASE;
        
        id_vect := std_logic_vector(to_unsigned(frame_1.identifier, 29));

        -- Shift base ID up for extended id to match Base ID of Node 2!
        id_vect := id_vect(10 downto 0) & "000000000000000000";
        frame_2.identifier := to_integer(unsigned(id_vect));
        frame_2.ident_type := EXTENDED;

        -----------------------------------------------------------------------
        -- 6. Wait till sample point in Node 1. Send Frame 1 by Node 1 and 
        --    Frame 2 by Node 2.
        -----------------------------------------------------------------------
        info("Step 6: Send frames");
        CAN_insert_TX_frame(frame_1, 1, ID_1, mem_bus(1));
        CAN_insert_TX_frame(frame_2, 1, ID_2, mem_bus(2));
        CAN_wait_sample_point(iout(1).stat_bus);
        
        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));
        
        -----------------------------------------------------------------------
        -- 7. Wait till arbitration field in Node 2. Wait till sample point 12
        --    times (11 Base ID + RTR/SRR). Check Node 2 is transmitting 
        --    recessive, Check Node 1 is transmitting dominant. Check Node 2
        --    lost arbitration. Check Node 1 is still transmitter. Read ALC 
        --    from Node 2 and check it.
        -----------------------------------------------------------------------
        info("Step 7: Check arbitration lost on SRR/RTR");
        CAN_wait_pc_state(pc_deb_arbitration, ID_2, mem_bus(2));
        for i in 0 to 11 loop
            CAN_wait_sample_point(iout(2).stat_bus);
        end loop;
        check(iout(1).can_tx = DOMINANT, "Dominant RTR transmitted!");
        check(iout(2).can_tx = RECESSIVE, "Recessive RTR transmitted!");
        
        -- Wait for up to one bit time since triggers can be shifted! 
        wait for 1000 ns;
        
        get_controller_status(stat_2, ID_2, mem_bus(2));
        check(stat_2.receiver, "Node 2 lost arbitration!");
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.transmitter, "Node 1 transmitter!");
        
        read_alc(alc, ID_2, mem_bus(2));
        check(alc = 12, "Arbitration lost at correct bit by Node 2!");
        
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

    wait for 1000 ns;
  end procedure;

end package body;
