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

entity func_cov_tx_arbitrator is
    generic (
        G_TXT_BUFFER_COUNT : natural
    );
    port (
        -- DUT clock
        clk    :   in  std_logic
    );
end entity;

architecture tb of func_cov_tx_arbitrator is

    -----------------------------------------------------------------------------------------------
    -- Aliases to "txarb_top" top
    -----------------------------------------------------------------------------------------------

    alias txtb_hw_cmd is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.txtb_hw_cmd : t_txtb_hw_cmd >>;

    alias tran_frame_valid is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.tran_frame_valid : std_logic >>;

    alias mr_mode_txbbm is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.mr_mode_txbbm : std_logic >>;

    alias mr_mode_tttm is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.mr_mode_tttm : std_logic >>;

    alias select_buf_avail is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.select_buf_avail : std_logic >>;

    alias select_buf_index is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.select_buf_index : natural range 0 to G_TXT_BUFFER_COUNT - 1 >>;

    alias txtb_available is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.txtb_available : std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0) >>;

    alias txtb_changed is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.txtb_changed : std_logic >>;

    alias select_index_changed is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.select_index_changed : std_logic >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "txarb_fsm" top
    -----------------------------------------------------------------------------------------------

    alias curr_state is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.i_txarb_fsm.curr_state : t_tx_arb_state >>;

    alias txtb_hw_cmd_lock is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.i_txarb_fsm.txtb_hw_cmd_lock : std_logic >>;

    alias parity_error_vld is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.i_txarb_fsm.parity_error_vld : std_logic >>;

    alias fsm_wait_state_q is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.i_txarb_fsm.fsm_wait_state_q : std_logic >>;

    alias timestamp_valid is
        << signal .tb_top_ctu_can_fd.i_dut.i_txarb_top.i_txarb_fsm.timestamp_valid : std_logic >>;

begin

    -- psl default clock is rising_edge(clk);

    -----------------------------------------------------------------------------------------------
    -- Lock and unlock commands
    -----------------------------------------------------------------------------------------------

    g_each_buf : for i in 0 to G_TXT_BUFFER_COUNT - 1 generate
    begin
        i_func_cov_tx_arbitrator_per_buf : entity ctu_can_fd_tb.func_cov_tx_arbitrator_per_buf
        generic map (
            G_TXT_BUFFER_COUNT  => G_TXT_BUFFER_COUNT,
            G_TXT_BUF_INDEX     => i
        )
        port map (
            clk                 => clk
        );
    end generate g_each_buf;

    -----------------------------------------------------------------------------------------------
    -- Modes
    -----------------------------------------------------------------------------------------------

    -- Note: We use gating by tran_frame_valid to avoid falsly covered scenarios,
    --       where reset value has the mode disabled!
    --
    -- psl txtb_ttm_ena_cov : cover
    --    {mr_mode_tttm = '1' and tran_frame_valid = '1'};
    -- psl txtb_ttm_dis_cov : cover
    --    {mr_mode_tttm = '0' and tran_frame_valid = '1'};

    -- psl txtb_txbbm_ena_cov : cover
    --    {mr_mode_txbbm = '1' and tran_frame_valid = '1'};
    -- psl txtb_txbbm_dis_cov : cover
    --    {mr_mode_txbbm = '0' and tran_frame_valid = '1'};

    -----------------------------------------------------------------------------------------------
    -- Selected TXT Buffer change corner-cases
    -----------------------------------------------------------------------------------------------

    -- psl txt_buf_change_cov : cover
    --    {txtb_changed = '1' and txtb_hw_cmd.lock = '1'}
    --    report "TX Buffer changed between two frames";
    --
    -- psl txt_buf_sim_chng_and_lock_cov : cover
    --    {select_index_changed = '1' and txtb_hw_cmd.lock = '1'};

    -----------------------------------------------------------------------------------------------
    -- Lock commands in various parts of TXT Buffer validation
    -----------------------------------------------------------------------------------------------

    -- psl txtb_lock_arb_sel_low_cov : cover
    --  {curr_state = s_arb_sel_low_ts and txtb_hw_cmd_lock = '1'};
    --
    -- psl txtb_lock_arb_sel_hi_cov : cover
    --  {curr_state = s_arb_sel_upp_ts and txtb_hw_cmd_lock = '1'};
    --
    -- psl txtb_lock_arb_sel_ftw_cov : cover
    --  {curr_state = s_arb_sel_ftw and txtb_hw_cmd_lock = '1'};
    --
    -- psl txtb_lock_arb_sel_ffw_cov : cover
    --  {curr_state = s_arb_sel_ffw and txtb_hw_cmd_lock = '1'};
    --
    -- psl txtb_lock_arb_sel_idw_cov : cover
    --  {curr_state = s_arb_sel_idw and txtb_hw_cmd_lock = '1'};
    --
    -- psl txtb_lock_arb_sel_validated_cov : cover
    --  {curr_state = s_arb_validated and txtb_hw_cmd_lock = '1'};

    -----------------------------------------------------------------------------------------------
    -- TXT Buffer becoming suddenly unavailable during TXT Buffer validation
    -----------------------------------------------------------------------------------------------

    -- psl txtb_not_available_arb_sel_low_cov : cover
    --  {curr_state = s_arb_sel_low_ts and select_buf_avail = '0'};
    --
    -- psl txtb_not_available_arb_sel_upp_cov : cover
    --  {curr_state = s_arb_sel_upp_ts and select_buf_avail = '0'};
    --
    -- psl txtb_not_available_arb_sel_ffw_cov : cover
    --  {curr_state = s_arb_sel_ffw and select_buf_avail = '0'};
    --
    -- psl txtb_not_available_arb_sel_ftw_cov : cover
    --  {curr_state = s_arb_sel_ftw and select_buf_avail = '0'};
    --
    -- psl txtb_not_available_arb_sel_idw_cov : cover
    --  {curr_state = s_arb_sel_idw and select_buf_avail = '0'};
    --
    -- psl txtb_not_available_arb_validated_cov : cover
    --  {curr_state = s_arb_validated and select_buf_avail = '0'};

    -----------------------------------------------------------------------------------------------
    -- Parity errors
    -----------------------------------------------------------------------------------------------

    -- psl txtb_ffw_parity_error_cov : cover
    --  {curr_state = s_arb_sel_ffw and parity_error_vld = '1'};

    -- psl txtb_idw_parity_error_cov : cover
    --  {curr_state = s_arb_sel_idw and parity_error_vld = '1'};

    -- psl txtb_lts_parity_error_cov : cover
    --  {curr_state = s_arb_sel_low_ts and parity_error_vld = '1'};

    -- psl txtb_uts_parity_error_cov : cover
    --  {curr_state = s_arb_sel_upp_ts and parity_error_vld = '1'};

    -----------------------------------------------------------------------------------------------
    -- Waiting till timestamp will be ready (transmission at given time)
    -----------------------------------------------------------------------------------------------

    -- psl txt_buf_wait_till_timestamp_cov : cover
    --    {curr_state = s_arb_sel_upp_ts and fsm_wait_state_q = '0' and
    --     timestamp_valid = '0'};

end architecture;