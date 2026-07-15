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
--    Functional coverage for a single TXT Buffer
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

entity func_cov_txt_buffer_even is
    generic (
        G_TXT_BUFFER_INDEX : natural
    );
    port (
        -- DUT clock
        clk    :   in  std_logic
    );
end entity;

architecture tb of func_cov_txt_buffer_even is

    -----------------------------------------------------------------------------------------------
    -- Aliases to "txt_buffer" top
    -----------------------------------------------------------------------------------------------
    alias mr_tx_command_txcr is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.mr_tx_command_txcr : std_logic >>;

    alias mr_tx_command_txce is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.mr_tx_command_txce : std_logic >>;

    alias mr_tx_command_txca is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.mr_tx_command_txca : std_logic >>;

    alias mr_tx_command_txbi is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.mr_tx_command_txbi : std_logic >>;

    alias txtb_hw_cmd is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.txtb_hw_cmd : t_txtb_hw_cmd >>;

    alias txtb_hw_cmd_cs is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.txtb_hw_cmd_cs : std_logic >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "txt_buffer_fsm"
    -----------------------------------------------------------------------------------------------
    alias curr_state is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.i_txt_buffer_fsm.curr_state : t_txt_buf_state >>;

    alias next_state is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.i_txt_buffer_fsm.next_state : t_txt_buf_state >>;

    alias txtb_parity_error_valid is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.i_txt_buffer_fsm.txtb_parity_error_valid : std_logic >>;

    alias abort_applied is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.abort_applied : std_logic >>;

    alias txt_fsm_ce is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.i_txt_buffer_fsm.txt_fsm_ce : std_logic >>;

    alias txtb_hw_cmd_i is
        << signal .tb_top_ctu_can_fd.dut.g_txt_buf_comp(G_TXT_BUFFER_INDEX).g_txt_buf_even.i_txt_buffer_even.i_txt_buffer_fsm.txtb_hw_cmd : t_txtb_hw_cmd >>;

begin

    -- psl default clock is rising_edge(clk);

    -------------------------------------------------------------------------------------------
    -- Each SW command active
    -------------------------------------------------------------------------------------------

    -- psl txtb_set_ready_cov : cover {mr_tx_command_txcr = '1' and mr_tx_command_txbi = '1'};
    -- psl txtb_set_empty_cov : cover {mr_tx_command_txce = '1' and mr_tx_command_txbi = '1'};
    -- psl txtb_set_abort_cov : cover {mr_tx_command_txca = '1' and mr_tx_command_txbi = '1'};

    -------------------------------------------------------------------------------------------
    -- HW Commands
    -------------------------------------------------------------------------------------------

    -- psl txtb_hw_lock     : cover {txtb_hw_cmd.lock = '1'     and txtb_hw_cmd_cs = '1'};
    -- psl txtb_hw_valid    : cover {txtb_hw_cmd.valid = '1'    and txtb_hw_cmd_cs = '1'};
    -- psl txtb_hw_err      : cover {txtb_hw_cmd.err = '1'      and txtb_hw_cmd_cs = '1'};
    -- psl txtb_hw_arbl     : cover {txtb_hw_cmd.arbl = '1'     and txtb_hw_cmd_cs = '1'};
    -- psl txtb_hw_failed   : cover {txtb_hw_cmd.failed = '1'   and txtb_hw_cmd_cs = '1'};

    -------------------------------------------------------------------------------------------
    -- Parity error during each possible state
    -------------------------------------------------------------------------------------------

    -- psl txtb_perr_txt_ready_cov : cover
    --      {curr_state = s_txt_ready and txtb_parity_error_valid = '1'};

    -- psl txtb_perr_txt_tx_prog_cov : cover
    --      {curr_state = s_txt_tx_prog and txtb_parity_error_valid = '1'};

    -- psl txtb_perr_txt_ab_prog_cov : cover
    --      {curr_state = s_txt_ab_prog and txtb_parity_error_valid = '1'};

    -------------------------------------------------------------------------------------------
    -- Simultaneous HW and SW Commands
    -------------------------------------------------------------------------------------------
    --
    -- psl txtb_hw_sw_cmd_txt_ready_hazard_cov : cover
    --  {txtb_hw_cmd.lock = '1' and txtb_hw_cmd_cs = '1' and abort_applied = '1' and
    --   curr_state = s_txt_ready};
    --
    -- psl txtb_hw_sw_cmd_txt_tx_prog_hazard_cov : cover
    --  {((txtb_hw_cmd_i.valid = '1' or txtb_hw_cmd_i.err = '1' or
    --     txtb_hw_cmd_i.arbl = '1' or txtb_hw_cmd_i.failed = '1') and
    --    abort_applied = '1' and curr_state = s_txt_tx_prog)};

    -------------------------------------------------------------------------------------------
    -- Corner-case transitions of FSM
    -------------------------------------------------------------------------------------------
    --
    -- psl txtb_ready_to_abt_in_progress_cov : cover
    --  {curr_state = s_txt_ready and next_state = s_txt_ab_prog and txt_fsm_ce = '1'};
    --
    -- psl txtb_abt_in_progress_to_parity_error_cov : cover
    --  {curr_state = s_txt_ab_prog and next_state = s_txt_parity_err and txt_fsm_ce = '1'};
    --
    -- psl txtb_tx_in_progress_to_aborted_cov : cover
    --  {curr_state = s_txt_tx_prog and next_state = s_txt_aborted and txt_fsm_ce = '1'};

end architecture;