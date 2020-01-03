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
--  Arbitration lost capture - Base ID feature test.
--
-- Verifies:
--  1. Arbitration within base identifier. Node loses arbitration on a bit where
--     it send recessive and samples dominant.
--  2. Arbitration lost capture position within Base identifier.
--
-- Test sequence:
--  1. Configure both Nodes to one-shot mode.
--  2. Loop by N between 1 and 11: 
--    2.1 Generate two CAN frames. Both with base ID only. Both IDs are the
--        same, appart from N-th bit. On N-th bit Node 1 will have Dominant
--        Node 2 Recessive.
--    2.2 Wait till sample point on Node 1. Send frame 1 by Node 1 and frame 2 
--        by Node 2 right one after another.
--    2.3 Wait till Arbitration field in Node 2. This is right after sample
--        point of Node 2 in SOF or Intermission (if there is no SOF). Check
--        that Node 2 is Transmitter.
--    2.4 Wait N-times till sample point in Node 2. After every wait before N
--        is reached, check Node 2 is still transmitter. After N waits we are
--        right after Sample point where Node 2 should have lost arbitration.
--        Check Node 2 is receiver. Read content of ALC, check arbitration was
--        lost at correct position.
--    2.5 Wait till the CRC delimiter in Node 2, and monitor that Node 2 is 
--        transmitting recessive value.
--    2.6 Wait till bus is idle! Check frame was sucessfully transmitted in
--        Node 1. Check it was succesfully received in Node 2!
--------------------------------------------------------------------------------
-- Revision History:
--    20.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--    11.06.2018  Modified to work with HAL functions from CAN Test lib instead
--                of direct register access.
--     28.9.2019  Re-write TC to cover ALC related functionality!
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package alc_base_id_feature is
    procedure alc_base_id_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body alc_base_id_feature is
    procedure alc_base_id_feature_exec(
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
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        
        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;
        
        constant id_template        :     std_logic_vector(10 downto 0) :=
                "01010101010";
        variable id_var             :     std_logic_vector(10 downto 0) :=
                 (OTHERS => '0');
    begin

        ------------------------------------------------------------------------
        -- 1. Configure both Nodes to one-shot mode.
        ------------------------------------------------------------------------
        info("Step 1: Configure one -shot mode");
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        CAN_enable_retr_limit(true, 0, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        --  2. Loop by N between 1 and 11: 
        ------------------------------------------------------------------------
        info("Step 2: Loop over each bit of Base ID!");
        for N in 1 to 11 loop
            info("-----------------------------------------------------------");
            info("Step 2: Bit " & integer'image(N));
            info("-----------------------------------------------------------");
            
            --------------------------------------------------------------------
            -- 2.1 Generate two CAN frames. Both with base ID only. Both IDs are
            --     the same, appart from N-th bit. On N-th bit Node 1 will have
            --     Dominant Node 2 Recessive.
            --------------------------------------------------------------------
            info("Step 2.1: Generate frames!");
            CAN_generate_frame(rand_ctr, frame_1);
            CAN_generate_frame(rand_ctr, frame_2);
            frame_1.ident_type := BASE;        
            frame_2.ident_type := BASE;
            
            -- Node 1 - Should win -> N-th bit Dominant.
            id_var := id_template;
            id_var(11 - N) := DOMINANT;
            frame_1.identifier := to_integer(unsigned(id_var));

            -- Node 2 - Should loose -> N-th bit Recessive.
            id_var := id_template;
            id_var(11 - N) := RECESSIVE;
            frame_2.identifier := to_integer(unsigned(id_var));
            
            CAN_insert_TX_frame(frame_1, 1, ID_1, mem_bus(1));
            CAN_insert_TX_frame(frame_2, 1, ID_2, mem_bus(2));

            --------------------------------------------------------------------
            -- 2.2 Wait till sample point on Node 1. Send frame 1 by Node 1 and
            --     frame 2 by Node 2 right one after another.
            --------------------------------------------------------------------
            info("Step 2.2: Send frames!");
            CAN_wait_sample_point(iout(1).stat_bus);
            send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
            send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));
            
            --------------------------------------------------------------------
            -- 2.3 Wait till Arbitration field in Node 2. This is right after
            --     sample point of Node 2 in SOF or Intermission (if there is no
            --     SOF). Check that Node 2 is Transmitter.
            --------------------------------------------------------------------
            info("Step 2.2: Wait till arbitration!");
            CAN_wait_pc_state(pc_deb_arbitration, ID_2, mem_bus(2));
            get_controller_status(stat_2, ID_2, mem_bus(2));
            check(stat_2.transmitter, "Node 2 transmitting!");
    
            -------------------------------------------------------------------
            -- 2.4 Wait N-times till sample point in Node 2. After every wait 
            --     before N is reached, check Node 2 is still transmitter.
            --     After N waits we are right after Sample point where Node 2
            --     should have lost arbitration. Check Node 2 is receiver.
            --     Read content of ALC, check arbitration was lost at correct
            --     position.
            -------------------------------------------------------------------
            info("Step 2.4: Wait till N-th bit!");
            for K in 1 to N loop
                info ("Loop: " & integer'image(K));
                CAN_wait_sample_point(iout(2).stat_bus);
                wait for 20 ns; -- Wait until RX trigger is processed!
                
                -- Arbitration should have been lost!
                if (K = N) then
                    get_controller_status(stat_2, ID_2, mem_bus(2));
                    check(stat_2.receiver, "Node 2 receiver!");
                    check_false(stat_2.transmitter, "Node 2 not transmitter!");

                    read_alc(alc, ID_2, mem_bus(2));
                    check(alc = N, "Arbitration lost at correct bit by Node 2!");
                    
                    read_alc(alc, ID_1, mem_bus(1));
                    check(alc = 0, "Arbitration not lost by Node 1!");
        
                    check(iout(2).can_tx = RECESSIVE, "Recessive transmitted!");
        
                -- Arbitration should not have been lost yet!
                else
                    get_controller_status(stat_2, ID_2, mem_bus(2));
                    check(stat_2.transmitter, "Node 2 transmitter!");
                    check_false(stat_2.receiver, "Node 2 not receiver!");
                    
                    if (K mod 2 = 0) then
                        check(iout(2).can_tx = RECESSIVE, "Recessive transmitted!");
                    else
                        check(iout(2).can_tx = DOMINANT, "Dominant transmitted!");
                    end if;
                end if;

            end loop;
            
        -----------------------------------------------------------------------
        -- 2.5 Wait till the CRC delimiter in Node 2, and monitor that Node 2
        --     is transmitting recessive value.
        -----------------------------------------------------------------------
        info("Step 2.5: Wait till end of frame!");
        CAN_read_pc_debug(pc_dbg, ID_2, mem_bus(2));
        while (pc_dbg /= pc_deb_crc_delim) loop
            CAN_read_pc_debug(pc_dbg, ID_2, mem_bus(2));
            check(iout(2).can_tx = RECESSIVE, "Recessive transmitted!");
            -- To make checks more sparse not to consume simulation time!
            wait for 100 ns;    
        end loop;
        
        -----------------------------------------------------------------------
        -- 2.7 Wait till bus is idle! Check frame was sucessfully transmitted
        --     in Node 1. Check it was succesfully received in Node 2!
        -----------------------------------------------------------------------
        info("Step 2.7: Wait till bus is idle!");
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        get_tx_buf_state(1, txt_buf_state, ID_1, mem_bus(1));
        check(txt_buf_state = buf_done, "Frame transmitted OK!");
        
        get_rx_buf_state(rx_buf_info, ID_2, mem_bus(2));
        check(rx_buf_info.rx_frame_count = 1, "Frame received OK!");
        
        CAN_read_frame(frame_rx, ID_2, mem_bus(2));
        CAN_compare_frames(frame_rx, frame_1, false, frames_equal);
        check(frames_equal, "TX vs. RX frames match!");

    end loop;

    wait for 1000 ns;
  end procedure;

end package body;
