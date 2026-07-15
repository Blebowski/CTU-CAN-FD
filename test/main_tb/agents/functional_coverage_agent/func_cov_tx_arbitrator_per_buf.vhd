--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2021-2023 Ondrej Ille
-- Copyright (C) 2023-     Logic Design Services Ltd.s
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- non-commercial purposes. Using the Component for commercial purposes is
-- forbidden unless previously agreed with Copyright holder.
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
--  @Purpose:
--    Functional coverage for TX Arbitrator
--
--------------------------------------------------------------------------------
-- Revision History:
--    1.6.2025   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;
context ctu_can_fd_tb.rtl_context;

use ctu_can_fd_tb.clk_gen_agent_pkg.all;
use ctu_can_fd_tb.tb_shared_vars_pkg.all;

entity func_cov_tx_arbitrator_per_buf is
    generic (
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT          :     natural range 1 to 8;

        -- Index of current TXT Buffer
        G_TXT_BUF_INDEX : natural

    );
    port (
        -- DUT clock
        clk    :   in  std_logic
    );
end entity;

architecture tb of func_cov_tx_arbitrator_per_buf is

    alias curr_txtb_index_i is
        << signal .tb_top_ctu_can_fd.i_dut.i_tx_arbitrator.curr_txtb_index_i : natural range 0 to G_TXT_BUFFER_COUNT - 1 >>;

    alias txtb_hw_cmd is
        << signal .tb_top_ctu_can_fd.i_dut.i_tx_arbitrator.txtb_hw_cmd : t_txtb_hw_cmd >>;

    alias txtb_hw_cmd_unlock is
        << signal .tb_top_ctu_can_fd.i_dut.i_tx_arbitrator.txtb_hw_cmd_unlock : std_logic >>;

    alias txtb_available is
        << signal .tb_top_ctu_can_fd.i_dut.i_tx_arbitrator.txtb_available : std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0) >>;

    alias select_buf_index is
        << signal .tb_top_ctu_can_fd.i_dut.i_tx_arbitrator.select_buf_index : natural range 0 to G_TXT_BUFFER_COUNT - 1 >>;

begin

    -- psl default clock is rising_edge(clk);

    -- psl txt_lock_buf_cov : cover
    --    {curr_txtb_index_i = G_TXT_BUF_INDEX and txtb_hw_cmd.lock = '1'};

    -- psl txt_unlock_buf_cov : cover
    --    {curr_txtb_index_i = 0 and txtb_hw_cmd_unlock = '1'};

    -- Change of buffer from available to not available but not due to lock
    -- (e.g. set abort).

    -- psl buf_ready_to_not_ready_cov : cover
    --    {txtb_available(G_TXT_BUF_INDEX) = '1' and select_buf_index = G_TXT_BUF_INDEX and
    --     txtb_hw_cmd.lock = '0'; txtb_available(G_TXT_BUF_INDEX) = '0'}
    --    report "Buffer became non-ready but not due to lock command";

end architecture;