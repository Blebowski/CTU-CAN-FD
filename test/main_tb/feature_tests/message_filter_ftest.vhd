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
--  Bus Start feature test. Verifies if unit is capable of joining fully,
--  loaded bus.
--
-- @Test sequence:
--      Part 1 (Message filter disabled):
--          1. Disable message filters in DUT.
--          2. Set all filters in DUT to not to pass any frame/identifier.
--          3. Send CAN frame by Test node.
--          4. Verify it was received by DUT and can be read out of RX Buffer.
--          5. Enable message filters in DUT.
--          6. Send CAN frame by Test node.
--          7. Verify that frame is not received, since filters are enabled and
--             configured not to pass any frame!
--      Part 2 (Mask filter):
--          1. Enable mask filter in DUT.
--          2. Configure random mask filter value, mask and accepted frame types.
--          3. Send CAN frames by Test node, one for each combination of frame type
--             / Identifier type. Identifier of the frame should be set so that
--             it passes the filter.
--          4. Check that frame is received if frame format is matching.
--          5. Send CAN frames by Test node, which are matching the CAN frame
--             type / Identifier type and not matching bit filter value.
--          6. Verify that frames are not received in DUT.
--          7. Repeat part 2, for remaining bit filters from A, B, C.
--      Part 3 (Range filter - Base frame):
--          1. Enable Range filter in DUT. Set frame acceptance for Base
--             Identifier.
--          2. Send CAN frame whose ID decimal value is lower than Low threshold
--             by Test node.
--          3. Verify that CAN frame is not received.
--          4. Send CAN frame with ID decimal value equal to Low threshold by
--             Test node.
--          5. Verify that frame was received by DUT.
--          6. Send CAN frame whose decimal value of ID is between low threshold
--             and high threshold.
--          7. Verify that frame is received by Test node.
--          8. Send CAN frame whose decimal value of ID is equal to High 
--             threshold of range filter.
--          9. Verify that frame was received by DUT.
--         10. Send frame which is higher than high threshold by Test node.
--             Verify that frame is not received by CAN DUT.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    15.9.2018   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package message_filter_ftest is
    procedure message_filter_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body message_filter_ftest is
    procedure message_filter_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable mode               :       SW_mode := SW_mode_rst_val;
        variable rx_state           :       SW_RX_Buffer_info;
        variable mask_filt_config   :       SW_CAN_mask_filter_config :=
                                                (0, 0, '0', false, false);
        variable range_filt_config  :       SW_CAN_range_filter_config := 
                                                (0, 0, '0', false, false);
        variable command            :       SW_command := SW_command_rst_val;
        variable tmp_int            :       natural := 0;
        variable tmp_log_vect       :       std_logic_vector(28 downto 0);
        variable tmp_log            :       std_logic := '0';
        variable should_pass        :       boolean := false;
        variable mask_filter        :       SW_CAN_mask_filter_type;
        variable l_th               :       natural := 0;
        variable h_th               :       natural := 0;
    begin
        CAN_generate_frame(CAN_frame);
        CAN_frame.brs := '0';

        ------------------------------------------------------------------------
        -- Part 1 (Message filters disabled)
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Disable filter usage in DUT. Set configuration of all filters
        -- not to pass any frame (to make sure that frame does not pass filter
        -- by chance).
        ------------------------------------------------------------------------
        CAN_set_mask_filter(filter_A, mask_filt_config, DUT_NODE, chn);
        CAN_set_mask_filter(filter_B, mask_filt_config, DUT_NODE, chn);
        CAN_set_mask_filter(filter_C, mask_filt_config, DUT_NODE, chn);
        CAN_set_range_filter(range_filt_config, DUT_NODE, chn);

        mode.acceptance_filter := false;
        set_core_mode(mode, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- Send frame by Test node. Check that frame was received!
        ------------------------------------------------------------------------
        command.release_rec_buffer := true;
        give_controller_command(command, DUT_NODE, chn);
        
        CAN_send_frame(CAN_frame, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        get_rx_buf_state(rx_state, DUT_NODE, chn);
        check_false_m(rx_state.rx_empty,
            "Frame is not received when Message filters are disabled!");

        ------------------------------------------------------------------------
        -- Enable message filters. Send frame and check that no frame is
        -- received! Now fitlers are enabled, but all filters are set not to
        -- pass any frame / identifier type! Thus any frame should not match
        -- any filter and frame should be dropped!
        ------------------------------------------------------------------------
        command.release_rec_buffer := true;
        give_controller_command(command, DUT_NODE, chn);
        
        mode.acceptance_filter := true;
        set_core_mode(mode, DUT_NODE, chn);

        CAN_send_frame(CAN_frame, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        get_rx_buf_state(rx_state, DUT_NODE, chn);
        check_m(rx_state.rx_empty,
            "Frame passed Message filters when all filters are disabled!");


        ------------------------------------------------------------------------
        -- Part 2 (Mask filters)
        ------------------------------------------------------------------------
        for i in 1 to 3 loop
            case i is 
                when 1 => mask_filter := filter_A;
                when 2 => mask_filter := filter_B;
                when 3 => mask_filter := filter_C;
            end case;
            CAN_frame.ident_type := BASE;
            CAN_frame.identifier := CAN_frame.identifier mod 2048;

            --------------------------------------------------------------------
            -- First reset value of each filter so that each next iteraion does
            -- not keep the settings of previous filter!
            --------------------------------------------------------------------
            mask_filt_config.acc_CAN_2_0 := false;
            mask_filt_config.acc_CAN_FD := false;
            mask_filt_config.ID_mask := 0;
            mask_filt_config.ID_value := 0;
            CAN_set_mask_filter(filter_A, mask_filt_config, DUT_NODE, chn);
            CAN_set_mask_filter(filter_B, mask_filt_config, DUT_NODE, chn);
            CAN_set_mask_filter(filter_C, mask_filt_config, DUT_NODE, chn);

            --------------------------------------------------------------------
            -- Configure several settings for one mask filter. Bit Matching
            -- will be done always on one bit only to avoid long sequences.
            -- Proper masking (of all bits) is verified in Message filter
            -- unit test. In each scenario, it is evaluated whether frame should
            -- be received (PASS) or not (FAIL).
            -- Following scenarios are considered:
            --  @1. Accept only FD, Bit mask match, Frame type match -> PASS
            --  @2. Accept only FD, Bit mask match, Frame type not -> FAIL
            --  @3. Accept only FD, Bit mask mismatch, Frame type match -> FAIL
            --  @4. Accept only 2.0, Bit mask match, Frame type match -> PASS
            --  @5. Accept only 2.0, Bit mask match, Frame type not -> FAIL
            --  @6. Accept only 2.0, Bit mask mismatch, Frame type match -> FAIL
            --  @7. Accept both, Bit mask match, Frame type CAN 2.0 -> PASS
            --  @8. Accept both, Bit mask match, Frame type FD -> PASS
            --  @9. Accept both, Bit mask mismatch, Frame type any -> FAIL
            --------------------------------------------------------------------
            for j in 1 to 9 loop
                mask_filt_config.acc_CAN_2_0 := false;
                mask_filt_config.acc_CAN_FD := false;
                mask_filt_config.ident_type := CAN_frame.ident_type;

                info_m("Starting scenario: " & integer'image(i) & "."); 

                -- Set accepted frame type based on scenario:
                if (j < 4) then
                    mask_filt_config.acc_CAN_FD := true;
                elsif (j < 7) then
                    mask_filt_config.acc_CAN_2_0 := true;
                else
                    mask_filt_config.acc_CAN_FD := true;
                    mask_filt_config.acc_CAN_2_0 := true;
                end if;

                -- Set expected frame types
                if ((j = 1) or (j = 3) or (j = 5) or (j = 8)) then
                    CAN_frame.frame_format := FD_CAN;
                else
                    CAN_frame.frame_format := NORMAL_CAN;
                end if;

                -- Generate random mask with one bit set, use only
                tmp_log_vect := (OTHERS => '0');
                if (CAN_frame.ident_type = BASE) then
                    rand_int_v(10, tmp_int);
                else
                    rand_int_v(28, tmp_int);
                end if;
                tmp_log_vect(tmp_int) := '1';
                mask_filt_config.ID_mask :=
                    to_integer(unsigned(tmp_log_vect));
                info_m("Filter mask: " & integer'image(mask_filt_config.ID_mask));

                -- Generate bit value which will either match or not match
                -- this mask depending on scenario.
                tmp_log_vect := (OTHERS => '0');
                rand_logic_v(tmp_log, 0.5);
                tmp_log_vect(tmp_int) := tmp_log;
                mask_filt_config.ID_value := 
                        to_integer(unsigned(tmp_log_vect));
                info_m("Filter value: " & integer'image(mask_filt_config.ID_value));

                -- Set equal for scenarios wher Bit mask is match, set
                -- opposite where it is not equal!
                id_sw_to_hw(CAN_frame.identifier, CAN_frame.ident_type,
                            tmp_log_vect);
                if (CAN_frame.ident_type = BASE) then
                    tmp_int := tmp_int + 18;
                end if;

                if ((j = 1) or (j = 4) or (j = 7) or (j = 8)) then
                    tmp_log_vect(tmp_int) := tmp_log;
                else
                    tmp_log_vect(tmp_int) := not tmp_log;
                end if;
                id_hw_to_sw(tmp_log_vect, CAN_frame.ident_type,
                            CAN_frame.identifier);
                info_m("CAN ID: " & integer'image(CAN_frame.identifier));

                -- Calculate whether frame should pass or not
                should_pass := false;
                if ((j = 1) or (j = 4) or (j = 7) or (j = 8)) then
                    should_pass := true;
                end if;

                -- Set filter settings
                CAN_set_mask_filter(mask_filter, mask_filt_config, DUT_NODE, chn);

                -- Send Frame by Test node and check if frame is received as
                -- expected or not! Flush RX Buffer first!
                command.release_rec_buffer := true;
                give_controller_command(command, DUT_NODE, chn);
                CAN_send_frame(CAN_frame, 1, TEST_NODE, chn, frame_sent);
                CAN_wait_frame_sent(DUT_NODE, chn);

                wait for 100 ns;
                get_rx_buf_state(rx_state, DUT_NODE, chn);

                -- Check!
               check_false_m((rx_state.rx_empty = true) and (should_pass = true),
                    "Frame should have passed but did NOT!");
                    
               check_false_m((rx_state.rx_empty = false) and (should_pass = false),
                    "Frame should NOT have passed but did!");
            end loop;
        end loop;

        ------------------------------------------------------------------------
        -- Part 3 (Range filters)
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Disable mask fitlters
        ------------------------------------------------------------------------        
        mask_filt_config.acc_CAN_2_0 := false;
        mask_filt_config.acc_CAN_FD := false;
        mask_filt_config.ID_mask := 0;
        mask_filt_config.ID_value := 0;
        CAN_set_mask_filter(filter_A, mask_filt_config, DUT_NODE, chn);
        CAN_set_mask_filter(filter_B, mask_filt_config, DUT_NODE, chn);
        CAN_set_mask_filter(filter_C, mask_filt_config, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- Generate random thresholds for identifier!
        ------------------------------------------------------------------------
        if (CAN_frame.ident_type = BASE) then
            rand_int_v((2 ** 6), l_th);
            rand_int_v((2 ** 11) - 1, h_th);
        else
            rand_int_v((2 ** 14), l_th);
            rand_int_v((2 ** 28) - 1, h_th);
        end if;

        -- Make sure upper threshold is not lower then low threshold!
        -- Avoid 0, max ident. so that we can test all cases!
        l_th := l_th + 1;
        if (l_th >= h_th) then
            h_th := l_th + 5;
        end if;
        h_th := h_th - 1;

        ------------------------------------------------------------------------
        -- Configure Range filter
        ------------------------------------------------------------------------
        range_filt_config.acc_CAN_2_0 := true;
        range_filt_config.acc_CAN_FD := true;
        range_filt_config.ID_th_high := h_th;
        range_filt_config.ID_th_low := l_th;
        CAN_set_range_filter(range_filt_config, DUT_NODE, chn);
        CAN_generate_frame(CAN_frame);
        CAN_frame.ident_type := BASE;
        CAN_frame.rtr := RTR_FRAME;
 
        info_m("ID value: " & integer'image(CAN_frame.identifier));
        info_m("Low threshold: " & integer'image(l_th));
        info_m("High threshold: " & integer'image(h_th));

        ------------------------------------------------------------------------
        -- Execute test of range fitlers. Following scenarios are tested:
        --  @1. CAN ID Lower than Low TH -> FAIL.
        --  @2. CAN ID Equal to Low TH -> PASS
        --  @3. CAN ID between Low and High TH -> PASS 
        --  @4. CAN ID equal to High TH -> PASS
        --  @5. CAN ID higher than High TH -> FAIL
        ------------------------------------------------------------------------
        for i in 1 to 5 loop
            
            case i is
                when 1 => CAN_frame.identifier := l_th - 1;
                when 2 => CAN_frame.identifier := l_th;
                when 3 => CAN_frame.identifier := ((h_th - l_th) / 2) + l_th;
                when 4 => CAN_frame.identifier := h_th;
                when 5 => CAN_frame.identifier := h_th + 1;
            end case;

            if ((i = 2) or (i = 3) or (i = 4)) then
                should_pass := true;
            else
                should_pass := false;
            end if;

            command.release_rec_buffer := true;
            give_controller_command(command, DUT_NODE, chn);

            CAN_send_frame(CAN_frame, 1, TEST_NODE, chn, frame_sent);
            CAN_wait_frame_sent(DUT_NODE, chn);
            get_rx_buf_state(rx_state, DUT_NODE, chn);

            if ((rx_state.rx_empty and should_pass) or
                ((not rx_state.rx_empty) and (not should_pass)))
            then
                -- LCOV_EXCL_START
                case i is
                    when 1 =>
                        error_m("Frame with ID lower than Low threshold passed," &
                              "but should NOT!");
                    when 2 =>
                        error_m("Frame with ID equal to Low threshold didnt pass," &
                              "but should!");
                    when 3 =>                        
                        error_m("Frame with ID betwen Low and High threshold did" &
                              "not pass, but should!");
                    when 4 =>                        
                        error_m("Frame with ID equal to Hig threshold did not," &
                              "pass, but should!");
                    when 5 =>
                        error_m("Frame with ID higher than High threshold did," &
                              "pass, but should NOT!");
                end case;
                -- LCOV_EXCL_STOP
            end if;
        end loop;

  end procedure;

end package body;