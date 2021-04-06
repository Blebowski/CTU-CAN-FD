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
--  Arbitration lost capture - ID Extension feature test.
--
-- @Verifies:
--  @1. Arbitration within identifier extension. Node loses arbitration on a bit
--      where it send recessive and samples dominant.
--  @2. Arbitration lost capture position within Identifier extension
--
-- @Test sequence:
--  @1. Configure both Nodes to one-shot mode.
--  @2. Loop by N between 1 and 18: 
--   @2.1 Generate two CAN frames. Both with Extended ID. Both IDs have Base ID
--        the same. N-th bit of ID Extension differs. On N-th bit DUT will
--        have Dominant, Test node Recessive.
--   @2.2 Wait till sample point on Test Node. Send frame 1 by Test Node and 
--        frame 2 by DUT right one after another.
--   @2.3 Wait till Arbitration field in DUT. This is right after sample
--        point of DUT in SOF or Intermission (if there is no SOF). Check
--        that DUT is Transmitter.
--   @2.4 Wait 11+1+1 (Base ID, RTR/SRR, IDE) times until sample point in DUT.
--   @2.5 Wait N-times till sample point in Test node. After every wait before N
--        is reached, check DUT is still transmitter. After N waits we are
--        right after Sample point where DUT should have lost arbitration.
--        Check DUT is receiver. Read content of ALC, check arbitration was
--        lost at correct position.
--   @2.6 Wait till the CRC delimiter in DUT, and monitor that DUT is 
--        transmitting recessive value.
--   @2.7 Wait till bus is idle! Check frame was sucessfully transmitted in
--        Test Node. Check it was succesfully received in DUT!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    05.10.2019  Created file.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package alc_id_extension_ftest is
    procedure alc_id_extension_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body alc_id_extension_ftest is
    procedure alc_id_extension_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable alc                :       natural;

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
        
        constant id_template        :     std_logic_vector(28 downto 0) :=
                "10101001010010101010101010101";
        variable id_var             :     std_logic_vector(28 downto 0) :=
                 (OTHERS => '0');
    begin

        ------------------------------------------------------------------------
        -- @1. Configure both Nodes to one-shot mode.
        ------------------------------------------------------------------------
        info_m("Step 1: Configure one -shot mode");
        CAN_enable_retr_limit(true, 0, TEST_NODE, chn);
        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);

        ------------------------------------------------------------------------
        --  @2. Loop by N between 1 and 18: 
        ------------------------------------------------------------------------
        info_m("Step 2: Loop over each bit of ID Extension!");
        for N in 1 to 18 loop
            info_m("-----------------------------------------------------------");
            info_m("Step 2: Bit " & integer'image(N));
            info_m("-----------------------------------------------------------");
            
            --------------------------------------------------------------------
            -- @2.1 Generate two CAN frames. Both with Extended ID. Both IDs have
            --     Base ID the same. N-th bit of ID Extension differs. On N-th
            --     bit Test Node will have Dominant, DUT Recessive.
            --------------------------------------------------------------------
            info_m("Step 2.1: Generate frames!");
            CAN_generate_frame(frame_1);
            CAN_generate_frame(frame_2);
            frame_1.ident_type := EXTENDED;        
            frame_2.ident_type := EXTENDED;
            
            frame_1.rtr := NO_RTR_FRAME;
            frame_2.rtr := NO_RTR_FRAME;
            
            -- Use short data length to avoid long test times!
            frame_1.data_length := 1;
            
            -- Test Node - Should win -> N-th bit Dominant.
            id_var := id_template;
            id_var(18 - N) := DOMINANT;
            frame_1.identifier := to_integer(unsigned(id_var));

            -- DUT - Should loose -> N-th bit Recessive.
            id_var := id_template;
            id_var(18 - N) := RECESSIVE;
            frame_2.identifier := to_integer(unsigned(id_var));
            
            CAN_insert_TX_frame(frame_1, 1, TEST_NODE, chn);
            CAN_insert_TX_frame(frame_2, 1, DUT_NODE, chn);

            --------------------------------------------------------------------
            -- @2.2 Wait till sample point on DUT. Send frame 1 by Test Node and
            --     frame 2 by DUT right one after another.
            --------------------------------------------------------------------
            info_m("Step 2.2: Send frames!");
            CAN_wait_sample_point(TEST_NODE, chn);
            send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
            send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);
            
            --------------------------------------------------------------------
            -- @2.3 Wait till Arbitration field in DUT. This is right after
            --     sample point of DUT in SOF or Intermission (if there is no
            --     SOF). Check that DUT is Transmitter.
            --------------------------------------------------------------------
            info_m("Step 2.3: Wait till arbitration!");
            CAN_wait_pc_state(pc_deb_arbitration, DUT_NODE, chn);
            get_controller_status(stat_2, DUT_NODE, chn);
            check_m(stat_2.transmitter, "DUT transmitting!");
    
            -------------------------------------------------------------------
            -- @2.4 Wait 11+1+1 (Base ID, RTR/SRR, IDE) times until sample 
            --     point in Test node.
            -------------------------------------------------------------------
            info_m("Step 2.4: Wait till Identifier Extension!");
            for i in 1 to 13 loop
                CAN_wait_sample_point(DUT_NODE, chn); 
            end loop;
    
            -------------------------------------------------------------------
            -- @2.5 Wait N-times till sample point in DUT. After every wait 
            --     before N is reached, check DUTis still transmitter.
            --     After N waits we are right after Sample point where DUT
            --     should have lost arbitration. Check DUT is receiver.
            --     Read content of ALC, check arbitration was lost at correct
            --     position.
            -------------------------------------------------------------------
            info_m("Step 2.5: Wait till N-th bit!");
            for K in 1 to N loop
                info_m("Loop: " & integer'image(K));
                CAN_wait_sample_point(DUT_NODE, chn);
                wait for 20 ns; -- Wait until RX trigger is processed!
                
                -- Arbitration should have been lost!
                if (K = N) then
                    get_controller_status(stat_2, DUT_NODE, chn);
                    check_m(stat_2.receiver, "DUT node receiver!");
                    check_false_m(stat_2.transmitter, "Test node not transmitter!");

                    read_alc(alc, DUT_NODE, chn);
                    check_m(alc = N + 13, "Arbitration lost at correct bit by DUT!");
                    
                    read_alc(alc, TEST_NODE, chn);
                    check_m(alc = 0, "Arbitration not lost by Test node!");
        
                    check_can_tx(RECESSIVE, DUT_NODE, "Recessive transmitted!", chn);
        
                -- Arbitration should not have been lost yet!
                else
                    get_controller_status(stat_2, DUT_NODE, chn);
                    check_m(stat_2.transmitter, "DUT transmitter!");
                    check_false_m(stat_2.receiver, "DUT not receiver!");
                    
                    if (K mod 2 = 0) then
                        check_can_tx(RECESSIVE, DUT_NODE, "Recessive transmitted!", chn);
                    else
                        check_can_tx(DOMINANT, DUT_NODE, "Dominant transmitted!", chn);
                    end if;
                end if;

            end loop;
            
        -----------------------------------------------------------------------
        -- @2.6 Wait till the CRC delimiter in DUT, and monitor that DUT
        --     is transmitting recessive value.
        -----------------------------------------------------------------------
        info_m("Step 2.5: Wait till end of frame!");
        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        mem_bus_agent_disable_transaction_reporting(chn);
        while (pc_dbg /= pc_deb_crc_delim) loop
            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            check_can_tx(RECESSIVE, DUT_NODE, "Recessive transmitted!", chn);
            -- To make checks more sparse not to consume simulation time!
            wait for 100 ns;
        end loop;
        mem_bus_agent_enable_transaction_reporting(chn);
        
        -----------------------------------------------------------------------
        -- @2.7 Wait till bus is idle! Check frame was sucessfully transmitted
        --     in Test Node. Check it was succesfully received in DUT!
        -----------------------------------------------------------------------
        info_m("Step 2.7: Wait till bus is idle!");
        CAN_wait_bus_idle(TEST_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        get_tx_buf_state(1, txt_buf_state, TEST_NODE, chn);
        check_m(txt_buf_state = buf_done, "Frame transmitted OK!");
        
        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_frame_count = 1, "Frame received OK!");
        
        CAN_read_frame(frame_rx, DUT_NODE, chn);
        CAN_compare_frames(frame_rx, frame_1, false, frames_equal);
        check_m(frames_equal, "TX vs. RX frames match!");

    end loop;

  end procedure;

end package body;
