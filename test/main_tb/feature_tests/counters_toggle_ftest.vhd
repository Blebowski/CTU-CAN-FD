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
--  Test to achieve full toggle coverage on ERR_NORM, ERR_FD, RX_FR_CTR and
--  TX_FR_CTR register.
--
-- @Verifies:
--  @1. Full width of ERR_NORM, ERR_FD, RX_FR_CTR and TX_FR_CTR can be accessed
--      from register map.
--
-- @Test sequence:
--  @1. Read expected value of ERR_NORM, ERR_FD, RX_FR_CTR and TX_FR_CTR
--      from DUT and check it matches value forced to DUT. Value forced to
--      DUT obtained from TB scratchpad (placed there by TB).
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    18.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package counters_toggle_ftest is
    procedure counters_toggle_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body counters_toggle_ftest is

    procedure counters_toggle_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable r_data : std_logic_vector(31 downto 0) := (OTHERS => '0');
    begin

        -----------------------------------------------------------------------
        -- @1. Read expected value of ERR_NORM, ERR_FD, RX_FR_CTR and
        --     TX_FR_CTR from DUT and check it matches value forced to DUT.
        --     Value forced to DUT obtained from TB scratchpad
        --     (placed there by TB).
        -----------------------------------------------------------------------
        info_m("Step 1: Check ERR_NORM, ERR_FD, RX_FR_CTR and TX_FR_CTR");

        CAN_read(r_data, ERR_NORM_ADR, DUT_NODE, chn);
        check_m(r_data(ERR_NORM_VAL_H downto ERR_NORM_VAL_L) = force_values.get_err_norm,
                "ERR_NORM is OK");
        check_m(r_data(ERR_FD_VAL_H downto ERR_FD_VAL_L) = force_values.get_err_fd,
                "ERR_FD is OK");

        CAN_read(r_data, RX_FR_CTR_ADR, DUT_NODE, chn);
        check_m(r_data = force_values.get_rx_counter, "RX_FR_CTR is OK");

        CAN_read(r_data, RX_FR_CTR_ADR, DUT_NODE, chn);
        check_m(r_data = force_values.get_rx_counter, "RX_FR_CTR is OK");

  end procedure;

end package body;
