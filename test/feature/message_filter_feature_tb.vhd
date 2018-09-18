--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Project advisors and co-authors:
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--  Bus Start feature test. Verifies if unit is capable of joining fully,
--  loaded bus.
--
--  Test sequence:
--      Part 1 (Message filter disabled):
--          1. Disable message filters in Node 1.
--          2. Set all filters in Node 1 to not to pass any frame/identifier.
--          3. Send CAN frame by Node 2.
--          4. Verify it was received by Node 1 and can be read out of RX Buffer.
--          5. Enable message filters in Node 1.
--          6. Send CAN frame by Node 2.
--          7. Verify that frame is not received, since filters are enabled and
--             configured not to pass any frame!
--      Part 2 (Mask filter):
--          1. Enable mask filter in Node 1.
--          2. Configure random mask filter value, mask and accepted frame types.
--          3. Send CAN frames by Node 2, one for each combination of frame type
--             / Identifier type. Identifier of the frame should be set so that
--             it passes the filter.
--          4. Check that frame is received if frame format is matching.
--          5. Send CAN frames by Node 2, which are matching the CAN frame
--             type / Identifier type and not matching bit filter value.
--          6. Verify that frames are not received in Node 1.
--          7. Repeat part 2, for remaining bit filters from A, B, C.
--      Part 3 (Range filter - Base frame):
--          1. Enable Range filter in Node 1. Set frame acceptance for Base
--             Identifier.
--          2. Send CAN frame whose ID decimal value is lower than Low threshold
--             by Node 2.
--          3. Verify that CAN frame is not received.
--          4. Send CAN frame with ID decimal value equal to Low threshold by
--             Node 2.
--          5. Verify that frame was received by Node 1.
--          6. Send CAN frame whose decimal value of ID is between low threshold
--             and high threshold.
--          7. Verify that frame is received by Node 2.
--          8. Send CAN frame whose decimal value of ID is equal to High 
--             threshold of range filter.
--          9. Verify that frame was received by Node 1.
--         10. Send frame which is higher than high threshold by Node 2.
--             Verify that frame is not received by CAN Node 1.
--------------------------------------------------------------------------------
-- Revision History:
--    15.9.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
use work.CAN_FD_frame_format.all;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

use work.CAN_FD_register_map.all;

package message_filter_feature is
    procedure message_filter_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body message_filter_feature is
    procedure message_filter_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable mode               :       SW_mode := (false, false, false,
                                                false, true, false, false,
                                                false, false, true);
        variable rx_state           :       SW_RX_Buffer_info;
        variable mask_filt_config   :       SW_CAN_mask_filter_config :=
                                                (0, 0, '0', false, false);
        variable range_filt_config  :       SW_CAN_range_filter_config := 
                                                (0, 0, '0', false, false);
        variable command            :       SW_command := (false, false, false);
        variable tmp_int            :       natural := 0;
        variable tmp_log_vect       :       std_logic_vector(28 downto 0);
        variable tmp_log            :       std_logic := '0';
        variable should_pass        :       boolean := false;
        variable mask_filter        :       SW_CAN_mask_filter_type;
    begin
        o.outcome := true;
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.brs := '0';
        
        wait_rand_cycles(rand_ctr, mem_bus(1).clk_sys, 1600, 1601);

        ------------------------------------------------------------------------
        -- Part 1 (Message filters disabled)
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Disable filter usage in Node 1. Set configuration of all filters
        -- not to pass any frame (to make sure that frame does not pass filter
        -- by chance).
        ------------------------------------------------------------------------
        CAN_set_mask_filter(filter_A, mask_filt_config, ID_1, mem_bus(1));
        CAN_set_mask_filter(filter_B, mask_filt_config, ID_1, mem_bus(1));
        CAN_set_mask_filter(filter_C, mask_filt_config, ID_1, mem_bus(1));
        CAN_set_range_filter(range_filt_config, ID_1, mem_bus(1));
        mode.acceptance_filter := false;
        set_core_mode(mode, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Send frame by Node 2. Check that frame was received!
        ------------------------------------------------------------------------
        command.release_rec_buffer := true;
        give_controller_command(command, ID_1, mem_bus(1));
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        get_rx_buf_state(rx_state, ID_1, mem_bus(1));
        if (rx_state.rx_empty) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "Frame is not received when Message filters are disabled!"
                severity error;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Enable message filters. Send frame and check that no frame is
        -- received! Now fitlers are enabled, but all filters are set not to
        -- pass any frame / identifier type! Thus any frame should not match
        -- any filter and frame should be dropped!
        ------------------------------------------------------------------------
        command.release_rec_buffer := true;
        give_controller_command(command, ID_1, mem_bus(1));
        mode.acceptance_filter := true;
        set_core_mode(mode, ID_1, mem_bus(1));

        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        get_rx_buf_state(rx_state, ID_1, mem_bus(1));
        if (not rx_state.rx_empty) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "Frame passed Message filters when all filters are disabled!"
                severity error;
            -- LCOV_EXCL_STOP
        end if;


        ------------------------------------------------------------------------
        -- Part 2 (Mask filters)
        ------------------------------------------------------------------------
        for i in 1 to 3 loop
            case i is 
                when 1 => mask_filter := filter_A;
                when 2 => mask_filter := filter_B;
                when 3 => mask_filter := filter_C;
            end case;

            --------------------------------------------------------------------
            -- First reset value of each filter so that each next iteraion does
            -- not keep the settings of previous filter!
            --------------------------------------------------------------------
            mask_filt_config.acc_CAN_2_0 := false;
            mask_filt_config.acc_CAN_FD := false;
            mask_filt_config.ID_mask := 0;
            mask_filt_config.ID_value := 0;
            CAN_set_mask_filter(filter_A, mask_filt_config, ID_1, mem_bus(1));
            CAN_set_mask_filter(filter_B, mask_filt_config, ID_1, mem_bus(1));
            CAN_set_mask_filter(filter_C, mask_filt_config, ID_1, mem_bus(1));

            --------------------------------------------------------------------
            -- Configure several settings for one mask filter. Bit Matching
            -- will be done always on one bit only to avoid long sequences.
            -- Proper masking (of all bits) is verified in Message filter
            -- unit test. In each scenario, it is evaluated whether frame should
            -- be received (PASS) or not (FAIL).
            -- Following scenarios are considered:
            --  1. Accept only FD, Bit mask match, Frame type match -> PASS
            --  2. Accept only FD, Bit mask match, Frame type not -> FAIL
            --  3. Accept only FD, Bit mask mismatch, Frame type match -> FAIL
            --  4. Accept only 2.0, Bit mask match, Frame type match -> PASS
            --  5. Accept only 2.0, Bit mask match, Frame type not -> FAIL
            --  6. Accept only 2.0, Bit mask mismatch, Frame type match -> FAIL
            --  7. Accept both, Bit mask match, Frame type CAN 2.0 -> PASS
            --  8. Accept both, Bit mask match, Frame type FD -> PASS
            --  9. Accept both, Bit mask mismatch, Frame type any -> FAIL
            --------------------------------------------------------------------
            for j in 1 to 9 loop
                mask_filt_config.acc_CAN_2_0 := false;
                mask_filt_config.acc_CAN_FD := false;

                report "Starting scenario: " & integer'image(i) & "." &
                        integer'image(j); 

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
                    rand_int_v(rand_ctr, 10, tmp_int);
                else
                    rand_int_v(rand_ctr, 28, tmp_int);
                end if;
                tmp_log_vect(tmp_int) := '1';
                mask_filt_config.ID_mask :=
                    to_integer(unsigned(tmp_log_vect));
                report "Filter mask: " &
                         integer'image(mask_filt_config.ID_mask);

                -- Generate bit value which will either match or not match
                -- this mask depending on scenario.
                tmp_log_vect := (OTHERS => '0');
                rand_logic_v(rand_ctr, tmp_log, 0.5);
                tmp_log_vect(tmp_int) := tmp_log;
                mask_filt_config.ID_value := 
                        to_integer(unsigned(tmp_log_vect));
                tmp_log_vect := std_logic_vector(
                                    to_unsigned(CAN_frame.identifier, 29));
                report "Filter value: " &
                        integer'image(mask_filt_config.ID_value);

                -- Set equal for scenarios wher Bit mask is match, set
                -- opposite where it is not equal!
                if ((j = 1) or (j = 4) or (j = 7) or (j = 8)) then
                    tmp_log_vect(tmp_int) := tmp_log;
                else
                    tmp_log_vect(tmp_int) := not tmp_log;
                end if;
                CAN_frame.identifier := to_integer(unsigned(tmp_log_vect));
                report "CAN ID: " &
                        integer'image(CAN_frame.identifier);

                -- Calculate whether frame should pass or not
                should_pass := false;
                if ((j = 1) or (j = 4) or (j = 7) or (j = 8)) then
                    should_pass := true;
                end if;

                -- Set filter settings
                CAN_set_mask_filter(mask_filter, mask_filt_config, ID_1, mem_bus(1));

                -- Send Frame by Node 2 and check if frame is received as
                -- expected or not! Flush RX Buffer first!
                command.release_rec_buffer := true;
                give_controller_command(command, ID_1, mem_bus(1));
                CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
                CAN_wait_frame_sent(ID_1, mem_bus(1));

                wait for 100 ns;
                get_rx_buf_state(rx_state, ID_1, mem_bus(1));

                -- Check!
                if ((rx_state.rx_empty = true) and (should_pass = true)) then
                    report "Frame should have passed but did NOT!" severity
                        error;
                end if;
                if ((rx_state.rx_empty = false) and (should_pass = false)) then
                    report "Frame should NOT have passed but did!" severity
                        error;
                end if;
            end loop;
        end loop;

  end procedure;

end package body;
