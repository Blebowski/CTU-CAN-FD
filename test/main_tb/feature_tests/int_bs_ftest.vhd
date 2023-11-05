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
--  Interrupt RX Buffer Full feature test.
--
-- @Verifies:
--  @1. When INT_MASK[BS] = 0, INT_STAT[BS] is set when RX Buffer is full.
--  @2. When INT_MASK[BS] = 1, INT_STAT[BS] is set when RX Buffer is full.
--  @3. When INT_ENA[BS]  = 1, INT_STAT[BS] = 1 causes interrupt output to
--      go high.
--  @4. When INT_ENA[BS]  = 0, INT_STAT[BS] = 1 does not cause interrupt
--      output to go high.
--  @7. RXF Interrupt enable is manipulated properly by INT_ENA_SET and
--      INT_ENA_CLEAR.
--  @8. RXF Interrupt mask is manipulated properly by INT_MASk_SET and
--      INT_MASK_CLEAR.

-- @Test sequence:
--  @1. Loop through all configurations of INT_MASK[BS] and INT_EN[BS].
--      @2.1 Configure interrupt.
--      @2.2 Send frame with bit-rate shift by DUT Node. Wait until it is
--           sent.
--      @2.3 Check that when Interrupt is masked it is not captured. Check that
--           when Interrupt is Enabled, it causes interrupt to propagate to
--           output
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    5.11.2023   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.interrupt_agent_pkg.all;

package int_bs_ftest is
    procedure int_bs_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;

package body int_bs_ftest is
    procedure int_bs_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :     SW_CAN_frame_type;
        variable CAN_frame_rx       :     SW_CAN_frame_type;
        variable frame_sent         :     boolean := false;
        variable frames_equal       :     boolean := false;

        variable int_mask           :     SW_interrupts := SW_interrupts_rst_val;
        variable int_ena            :     SW_interrupts := SW_interrupts_rst_val;
        variable int_stat           :     SW_interrupts := SW_interrupts_rst_val;
        variable pc_dbg             :     SW_PC_Debug;
        variable rxb_state          :     SW_RX_Buffer_info;

    begin

        -----------------------------------------------------------------------
        -- @1. Loop through all configurations of INT_MASK[BS] and INT_EN[BS].
        -----------------------------------------------------------------------
        for shall_transmitt in boolean'left to boolean'right loop
            for do_shift in boolean'left to boolean'right loop
                for mask in boolean'left to boolean'right loop
                    for ena in boolean'left to boolean'right loop

                        ---------------------------------------------------------------
                        -- @2.1 Configure interrupt.
                        ---------------------------------------------------------------
                        info_m("Step 2.1 with:" &
                                    "MASK: "  & boolean'image(mask) &
                                    "ENABLE:" & boolean'image(ena));


                        int_mask.bit_rate_shift_int := mask;
                        int_ena.bit_rate_shift_int := ena;

                        write_int_mask(int_mask, DUT_NODE, chn);
                        write_int_enable(int_ena, DUT_NODE, chn);

                        ---------------------------------------------------------------
                        -- @2.2 Send frame with bit-rate shift by DUT Node. Wait until
                        --      it is sent.
                        ---------------------------------------------------------------
                        info_m("Step 2.2");

                        CAN_generate_frame(CAN_frame);
                        CAN_frame.data_length := 1;
                        CAN_frame.frame_format := FD_CAN;
                        if (do_shift) then
                            CAN_frame.brs := BR_SHIFT;
                        else
                            CAN_frame.brs := BR_NO_SHIFT;
                        end if;
                        decode_length(CAN_frame.data_length, CAN_frame.dlc);

                        if (shall_transmitt) then
                            CAN_send_frame(CAN_Frame, 1, DUT_NODE, chn, frame_sent);
                            CAN_wait_frame_sent(DUT_NODE, chn);
                        else
                            CAN_send_frame(CAN_Frame, 1, TEST_NODE, chn, frame_sent);
                            CAN_wait_frame_sent(TEST_NODE, chn);
                        end if;

                        ---------------------------------------------------------------
                        -- @2.3 Check that when Interrupt is masked it is not captured.
                        --      Check that when Interrupt is Enabled, it causes interrupt
                        --      to propagate to output.
                        ---------------------------------------------------------------
                        read_int_status(int_stat, DUT_NODE, chn);
                        wait for 20 ns;

                        check_m(int_stat.bit_rate_shift_int = ((not mask) and do_shift),
                            "INT_MASK[BS] != INT_STAT[BS]" &
                            "INT_MASK[BS] : " & boolean'image(int_mask.bit_rate_shift_int) &
                            "INT_STAT[BS] : " & boolean'image(int_stat.bit_rate_shift_int));

                        if (do_shift and (not mask) and ena) then
                            wait for 20 ns;
                            interrupt_agent_check_asserted(chn);

                            -- Disable and check output is low
                            int_ena.bit_rate_shift_int := false;
                            write_int_enable(int_ena, DUT_NODE, chn);
                            wait for 20 ns;
                            interrupt_agent_check_not_asserted(chn);

                            -- Enable and check output is high again
                            int_ena.bit_rate_shift_int := true;
                            write_int_enable(int_ena, DUT_NODE, chn);
                            wait for 20 ns;
                            interrupt_agent_check_asserted(chn);

                            -- Mask, and clear and check it was cleared
                            int_mask.bit_rate_shift_int := true;
                            write_int_mask(int_mask, DUT_NODE, chn);
                            clear_int_status(int_stat, DUT_NODE, chn);
                            wait for 20 ns;
                            read_int_status(int_stat, DUT_NODE, chn);

                            check_false_m(int_stat.bit_rate_shift_int,
                                        "RX Buffer Full Interrupt cleared!");
                            interrupt_agent_check_not_asserted(chn);

                        else
                            interrupt_agent_check_not_asserted(chn);
                        end if;

                    end loop;
                end loop;
            end loop;
        end loop;

    end procedure;
end package body;