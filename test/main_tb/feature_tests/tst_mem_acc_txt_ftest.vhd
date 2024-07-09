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
--  Test access to TXT buffer RAMs.
--
-- @Verifies:
--  @1. TXT buffer RAM can be written and read via Test registers.
--  @2. Write to one TXT buffer RAM does not affect write to other TXT buffer
--      RAMs.
--  @3. TXT buffer RAM cannot be written via Test registers when MODE[TSTM]=0.
--
-- @Test sequence:
--  @1. Read number of TXT buffers in DUT. Disable DUT (since memory testability
--      shall be used when DUT is disabled).
--  @2. Iterate for each: TXT Buffer, MODE[TSTM], TST_CONTROL[TMAEN] and value
--      written to the memory.
--      @2.1 Take content and write it to TXT Buffer RAM.
--      @2.2 Read whole TXT buffer RAM back and compare read values with
--           written ones if test mode was enabled. Otherwise, check its all
--           zeroes. Read TXT buffer RAM of other buffers, and check there are
--           all zeroes.
--      @2.3 Write all zeroes to the TXT Buffer.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    2.5.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package tst_mem_acc_txt_ftest is
    procedure tst_mem_acc_txt_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body tst_mem_acc_txt_ftest is

    procedure tst_mem_acc_txt_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable r_data  : std_logic_vector(31 downto 0) := (OTHERS => '0');

        type t_test_mem is
            array (0 to 20) of std_logic_vector(31 downto 0);
        variable w_content      : t_test_mem := (OTHERS => (OTHERS => '0'));

        type t_test_values is
            array (0 to 4) of std_logic_vector(31 downto 0);
        variable w_values       : t_test_values := (
            x"AAAAAAAA",
            x"55555555",
            x"C3C3C3C3",
            x"3C3C3C3C",
            x"00000000"          -- Random
        );

        variable mode           : SW_mode    := SW_mode_rst_val;
        variable num_txt_bufs   : natural;
        variable tgt_mtm        : t_tgt_test_mem;
        variable rnd            : integer;
    begin

        -----------------------------------------------------------------------
        -- @1. Read number of TXT buffers in DUT. Disable DUT (since memory
        --     testability shall be used when DUT is disabled).
        -----------------------------------------------------------------------
        info_m("Step 1");

        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);
        info_m("Number of TXT buffers: " & integer'image(num_txt_bufs));

        -- First we must disable DUT
        CAN_turn_controller(false, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Iterate for each: TXT Buffer, MODE[TSTM], TST_CONTROL[TMAEN] and
        --     value written to the memory.
        -----------------------------------------------------------------------
        info_m("Step 2");
        for used_txt_buf in 1 to num_txt_bufs loop
            for mode_tstm_ena in boolean'left to boolean'right loop
                for mode_tma_ena in boolean'left to boolean'right loop
                    for i in t_test_values'low to t_test_values'high loop

                        info_m("Step 2 with "        &
                                "TXT Buffer index: " & integer'image(used_txt_buf) &
                                "MODE[TSTM]: "       & boolean'image(mode_tstm_ena) &
                                "Test value:"        & to_hstring(w_values(i)));

                        -- Configure test mode
                        mode.test := mode_tstm_ena;
                        set_core_mode(mode, DUT_NODE, chn);

                        set_test_mem_access(true, DUT_NODE, chn);
                        wait for 50 ns;

                        -- Fill test data
                        for j in 0 to 20 loop
                            if (i = t_test_values'high) then
                                rand_logic_vect_v(w_content(j), 0.5);
                            else
                                w_content(j) := w_values(i);
                            end if;
                        end loop;
                        wait for 10 ns;

                        case used_txt_buf is
                        when 1 => tgt_mtm := TST_TGT_TXT_BUF_1;
                        when 2 => tgt_mtm := TST_TGT_TXT_BUF_2;
                        when 3 => tgt_mtm := TST_TGT_TXT_BUF_3;
                        when 4 => tgt_mtm := TST_TGT_TXT_BUF_4;
                        when 5 => tgt_mtm := TST_TGT_TXT_BUF_5;
                        when 6 => tgt_mtm := TST_TGT_TXT_BUF_6;
                        when 7 => tgt_mtm := TST_TGT_TXT_BUF_7;
                        when 8 => tgt_mtm := TST_TGT_TXT_BUF_8;
                        when others =>
                            error_m("Invalid TXT buffer");
                        end case;

                        -----------------------------------------------------------
                        -- @2.1 Take content and write it to TXT Buffer RAM.
                        -----------------------------------------------------------
                        for addr in 0 to 20 loop
                            test_mem_write(w_content(addr), addr, tgt_mtm, DUT_NODE, chn, mode_tma_ena);
                        end loop;

                        wait for 100 ns;

                        -----------------------------------------------------------------------
                        -- @2.2 Read whole TXT buffer RAM back and compare read values with
                        --      written ones. Read TXT buffer RAM of other buffers, and check
                        --      there are all zeroes.
                        -----------------------------------------------------------------------
                        info_m("Step 2.2");

                        for buf_ind in 1 to 8 loop

                            case buf_ind is
                            when 1 => tgt_mtm := TST_TGT_TXT_BUF_1;
                            when 2 => tgt_mtm := TST_TGT_TXT_BUF_2;
                            when 3 => tgt_mtm := TST_TGT_TXT_BUF_3;
                            when 4 => tgt_mtm := TST_TGT_TXT_BUF_4;
                            when 5 => tgt_mtm := TST_TGT_TXT_BUF_5;
                            when 6 => tgt_mtm := TST_TGT_TXT_BUF_6;
                            when 7 => tgt_mtm := TST_TGT_TXT_BUF_7;
                            when 8 => tgt_mtm := TST_TGT_TXT_BUF_8;
                            end case;

                            for addr in 0 to 19 loop
                                test_mem_read(r_data, addr, tgt_mtm, DUT_NODE, chn);

                                -- Only check in test mode!
                                if (buf_ind = used_txt_buf and mode_tstm_ena and mode_tma_ena) then
                                    check_m(r_data = w_content(addr),
                                        "TXT Buffer "            & integer'image(buf_ind) &
                                        " RAM data at address: " & integer'image(addr) &
                                        " Expected: 0x"          & to_hstring(w_content(addr)) &
                                        " Read: 0x"              & to_hstring(r_data));
                                else
                                    check_m(r_data = x"00000000",
                                        "TXT Buffer "            & integer'image(buf_ind) &
                                        " RAM data at address: " & integer'image(addr) &
                                        " Expected: 0x00000000"  &
                                        " Read: 0x"              & to_hstring(r_data));
                                end if;
                            end loop;
                        end loop;

                        -----------------------------------------------------------
                        -- @2.3 Write all zeroes to the TXT Buffer.
                        -----------------------------------------------------------
                        info_m("Step 2.2");

                        case used_txt_buf is
                        when 1 => tgt_mtm := TST_TGT_TXT_BUF_1;
                        when 2 => tgt_mtm := TST_TGT_TXT_BUF_2;
                        when 3 => tgt_mtm := TST_TGT_TXT_BUF_3;
                        when 4 => tgt_mtm := TST_TGT_TXT_BUF_4;
                        when 5 => tgt_mtm := TST_TGT_TXT_BUF_5;
                        when 6 => tgt_mtm := TST_TGT_TXT_BUF_6;
                        when 7 => tgt_mtm := TST_TGT_TXT_BUF_7;
                        when 8 => tgt_mtm := TST_TGT_TXT_BUF_8;
                        when others =>
                            error_m("Invalid TXT buffer");
                        end case;

                        for addr in 0 to 19 loop
                            test_mem_write(x"00000000", addr, tgt_mtm, DUT_NODE, chn, mode_tma_ena);
                        end loop;

                        wait for 100 ns;
                        wait for 1 us;

                    end loop;
                end loop;
            end loop;
        end loop;

  end procedure;

end package body;
