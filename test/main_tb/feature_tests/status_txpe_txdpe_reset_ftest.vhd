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
--  STATUS[TXPE] and STATUS[TXDPE] clear feature test.
--
-- @Verifies:
--  @1. STATUS[TXPE] and STATUS[TXDPE] are cleared by Soft reset!
--
-- @Test sequence:
--  @1. Set DUT to Test mode and to TXT Buffer backup mode.
--  @2. Insert the CAN frame for transmission into a TXT Buffer 1.
--      Insert the same frame into TXT Buffer 2 (backup buffer).
--  @3. Generate random bit-flip in FRAME_FORMAT_W word in the TXT Buffer
--      1 and TXT Buffer 2 memory via test interface.
--  @4. Send Set Ready command to this TXT Buffers 1 and 2. Wait for some time,
--      and check that when SETTINGS[PCHKE] = 1, TXT Buffers 1 and 2 ended up in
--      "Parity Error" state.
--  @5. Check STATUS[TXPE] = 1 and STATUS[TXDPE] = 1.
--  @6. Issue soft-reset to the DUT Node, and check STATUS[TXPE] = 0
--      and STATUS[TXDPE] = 0.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    29.8.2024   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package status_txpe_txdpe_reset_ftest is
    procedure status_txpe_txdpe_reset_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body status_txpe_txdpe_reset_ftest is
    procedure status_txpe_txdpe_reset_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable frame_1            :     SW_CAN_frame_type;
        variable stat_1             :     SW_status;
        variable mode_1             :     SW_mode := SW_mode_rst_val;

        variable r_data             :     std_logic_vector(31 downto 0);
        variable corrupt_bit_index  :     integer;

        variable tst_mem            :     t_tgt_test_mem;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Set DUT to Test mode and to TXT Buffer backup mode. Enable Parity check.
        --     Generate random CAN frame.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode_1.test := true;
        mode_1.tx_buf_backup := true;
        mode_1.parity_check := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        CAN_generate_frame(frame_1);

        -------------------------------------------------------------------------------------------
        -- @2. Insert the CAN frame for transmission into a TXT Buffer 1.
        --     Insert the same frame into TXT Buffer 2 (backup buffer).
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        CAN_insert_TX_frame(frame_1, 1, DUT_NODE, chn);
        CAN_insert_TX_frame(frame_1, 2, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @3. Generate random bit-flip in FRAME_FORMAT_W word in the TXT Buffer 1
        --     and TXT Buffer 2 memory via test interface.
        -------------------------------------------------------------------------------------------
        info_m("Step 3");

        -- Enable test access
        set_test_mem_access(true, DUT_NODE, chn);

        -- Read, flip, and write back for TXT Buffer 1 and 2
        for i in 1 to 2 loop
            tst_mem := txt_buf_to_test_mem_tgt(i);
            test_mem_read(r_data, 0, tst_mem, DUT_NODE, chn);
            rand_int_v(31, corrupt_bit_index);
            r_data(corrupt_bit_index) := not r_data(corrupt_bit_index);
            test_mem_write(r_data, 0, tst_mem, DUT_NODE, chn);
        end loop;

        -- Disable test mem access
        set_test_mem_access(false, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @4. Send Set Ready command to this TXT Buffers 1 and 2. Wait for some time,
        --     and check that when SETTINGS[PCHKE] = 1, TXT Buffers 1 and 2 ended up in
        --     "Parity Error" state.
        -------------------------------------------------------------------------------------------
        info_m("Step 4");

        send_TXT_buf_cmd(buf_set_ready, "00000011", DUT_NODE, chn);
        wait for 10 us;

        -------------------------------------------------------------------------------------------
        -- @5. Check STATUS[TXPE] = 1 and STATUS[TXDPE] = 1.
        -------------------------------------------------------------------------------------------
        info_m("Step 5");

        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.tx_parity_error, "STATUS[TXPE] = 1");
        check_m(stat_1.tx_double_parity_error, "STATUS[TXDPE] = 1");

        -------------------------------------------------------------------------------------------
        -- @6. Issue soft-reset to the DUT Node, and check STATUS[TXPE] = 0
        --      and STATUS[TXDPE] = 0.
        -------------------------------------------------------------------------------------------
        info_m("Step 6");

        exec_SW_reset(DUT_NODE, chn);
        wait for 20 ns;

        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.tx_parity_error, "STATUS[TXPE] = 0");
        check_false_m(stat_1.tx_double_parity_error, "STATUS[TXDPE] = 0");

    end procedure;

end package body;
