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
--    Functional coverage for CAN Core
--
--------------------------------------------------------------------------------
-- Revision History:
--    27.4.2025   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;
context ctu_can_fd_tb.rtl_context;

use ctu_can_fd_tb.clk_gen_agent_pkg.all;
use ctu_can_fd_tb.tb_shared_vars_pkg.all;

entity func_cov_can_core is
    port (
        -- DUT clock
        clk    :   in  std_logic
    );
end entity;

architecture tb of func_cov_can_core is

    -----------------------------------------------------------------------------------------------
    -- Aliases to "mac_top" top
    -----------------------------------------------------------------------------------------------

    alias tran_ident_type is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.tran_ident_type : std_logic >>;

    alias tran_frame_type is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.tran_frame_type : std_logic >>;

    alias tran_is_rtr is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.tran_is_rtr : std_logic >>;

    alias tran_frame_valid is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.tran_frame_valid : std_logic >>;

    alias rec_ident_type is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.rec_ident_type : std_logic >>;

    alias rec_frame_type is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.rec_frame_type : std_logic >>;

    alias rec_is_rtr is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.rec_is_rtr : std_logic >>;

    alias store_metadata is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.store_metadata : std_logic >>;

    alias tran_valid is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.tran_valid : std_logic >>;

    alias rec_valid is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.rec_valid : std_logic >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "mac_bit_stuffing" and "mac_bit_destuffing"
    -----------------------------------------------------------------------------------------------
    alias bds_trigger is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_bit_destuffing.bds_trigger : std_logic >>;

    alias non_fix_to_fix_chng is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_bit_destuffing.non_fix_to_fix_chng : std_logic >>;

    alias stuff_err_q is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_bit_destuffing.stuff_err_q : std_logic >>;

    alias stuff_lvl_reached is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_bit_destuffing.stuff_lvl_reached : std_logic >>;

    alias fixed_stuff is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_bit_destuffing.fixed_stuff : std_logic >>;


    -----------------------------------------------------------------------------------------------
    -- Aliases to "mac_fc_err_counters"
    -----------------------------------------------------------------------------------------------

    alias inc_one is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_err_counters.inc_one : std_logic >>;

    alias inc_eight is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_err_counters.inc_eight : std_logic >>;

    alias dec_one is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_err_counters.dec_one : std_logic >>;

    alias rx_err_ctr_inc is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_err_counters.rx_err_ctr_inc : unsigned(8 downto 0) >>;

    alias rx_err_ctr_q is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_err_counters.rx_err_ctr_q : unsigned(8 downto 0) >>;

    alias rx_err_ctr_ce is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_err_counters.rx_err_ctr_ce : std_logic >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "mac_pc_err_detector"
    -----------------------------------------------------------------------------------------------

    alias bit_err is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_err_detector.bit_err : std_logic >>;

    alias bit_err_arb is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_err_detector.bit_err_arb : std_logic >>;

    alias stuff_err is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_err_detector.stuff_err : std_logic >>;

    alias form_err is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_err_detector.form_err : std_logic >>;

    alias ack_err is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_err_detector.ack_err : std_logic >>;

    alias crc_err is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_err_detector.crc_err : std_logic >>;

    alias tran_frame_parity_error is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_err_detector.tran_frame_parity_error : std_logic >>;

    alias err_capt_err_type_q is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_err_detector.err_capt_err_type_q : std_logic_vector(2 downto 0) >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "mac_fc_rules"
    -----------------------------------------------------------------------------------------------

    alias primary_err is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_rules.primary_err : std_logic >>;

    alias err_ctrs_unchanged is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_rules.err_ctrs_unchanged : std_logic >>;

    alias is_receiver is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_rules.is_receiver : std_logic >>;

    alias is_transmitter is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_rules.is_transmitter : std_logic >>;

    alias act_err_ovr_flag is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_rules.act_err_ovr_flag : std_logic >>;

    alias err_detected is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_rules.err_detected : std_logic >>;

    alias err_delim_late is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_rules.err_delim_late : std_logic >>;

    alias bit_err_after_ack_err is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_rules.bit_err_after_ack_err : std_logic >>;

    alias decrement_rec is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_fc_top.i_mac_fc_rules.decrement_rec : std_logic >>;


    -----------------------------------------------------------------------------------------------
    -- Aliases to "mac_pc_fsm"
    -----------------------------------------------------------------------------------------------

    alias curr_state is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.curr_state : t_protocol_control_state >>;

    alias next_state is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.next_state : t_protocol_control_state >>;

    alias err_frm_req is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.err_frm_req : std_logic >>;

    alias pex_on_fdf_enable is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.pex_on_fdf_enable : std_logic >>;

    alias pex_on_res_enable is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.pex_on_res_enable : std_logic >>;

    alias mr_status_pexs is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.mr_status_pexs : std_logic >>;

    alias pexs_set is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.pexs_set : std_logic >>;

    alias arbitration_lost_i is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.arbitration_lost_i : std_logic >>;

    alias mr_settings_pex is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.mr_settings_pex : std_logic >>;

    alias mr_mode_fde is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.mr_mode_fde : std_logic >>;

    alias mr_settings_nisofd is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_fsm.mr_settings_nisofd : std_logic >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "mac_pc_reintegration_counter"
    -----------------------------------------------------------------------------------------------
    alias reinteg_ctr_clr is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_reintegration_counter.reinteg_ctr_clr : std_logic >>;

    alias reinteg_ctr_expired is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_reintegration_counter.reinteg_ctr_expired : std_logic >>;

    alias reinteg_ctr_enable is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_reintegration_counter.reinteg_ctr_enable : std_logic >>;

    alias rx_trigger is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_reintegration_counter.rx_trigger : std_logic >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "mac_pc_rx_shift_reg"
    -----------------------------------------------------------------------------------------------

    alias rx_clear is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_clear : std_logic >>;

    alias rx_shift_in_sel is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_shift_in_sel : std_logic >>;

    alias rx_shift_ena is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_shift_ena : std_logic_vector(3 downto 0) >>;

    alias rx_store_base_id is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_store_base_id : std_logic >>;

    alias rx_store_ext_id is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_store_ext_id : std_logic >>;

    alias rx_store_ide is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_store_ide : std_logic >>;

    alias rx_store_rtr is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_store_rtr : std_logic >>;

    alias rx_store_edl is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_store_edl : std_logic >>;

    alias rx_store_dlc is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_store_dlc : std_logic >>;

    alias rx_store_esi is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_store_esi : std_logic >>;

    alias rx_store_brs is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_store_brs : std_logic >>;

    alias rx_store_stuff_count is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_rx_shift_reg.rx_store_stuff_count : std_logic >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "mac_pc_tx_shift_reg"
    -----------------------------------------------------------------------------------------------

    alias tx_load_base_id is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_tx_shift_reg.tx_load_base_id : std_logic >>;

    alias tx_load_ext_id is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_tx_shift_reg.tx_load_ext_id : std_logic >>;

    alias tx_load_dlc is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_tx_shift_reg.tx_load_dlc : std_logic >>;

    alias tx_load_data_word is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_tx_shift_reg.tx_load_data_word : std_logic >>;

    alias tx_load_stuff_count is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_tx_shift_reg.tx_load_stuff_count : std_logic >>;

    alias tx_load_crc is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_tx_shift_reg.tx_load_crc : std_logic >>;

    alias tran_frame_test is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_tx_shift_reg.tran_frame_test : t_frame_test_w >>;

    alias mr_mode_tstm is
        << signal .tb_top_ctu_can_fd.i_dut.i_mac_top.i_mac_pc_top.i_mac_pc_tx_shift_reg.mr_mode_tstm : std_logic >>;

begin

    -- psl default clock is rising_edge(clk);

    -----------------------------------------------------------------------------------------------
    -- Transmitted and received frame
    -----------------------------------------------------------------------------------------------

    -- psl traffic_ctrs_tx_inc_cov : cover
    --  {tran_valid = '1'};

    -- psl traffic_ctrs_rx_inc_cov : cover
    --  {rec_valid = '1'};

    -----------------------------------------------------------------------------------------------
    -- Transmitted FDF/IDE/RTR combinations
    -----------------------------------------------------------------------------------------------

    -- psl tx_base_id_can_2_0_cov : cover
    --  {tran_ident_type  = BASE        and
    --   tran_frame_type  = NORMAL_CAN  and
    --   tran_is_rtr      = '0'         and
    --   tran_frame_valid = '1'};

    -- psl tx_extended_id_can_2_0_cov : cover
    --  {tran_ident_type  = EXTENDED    and
    --   tran_frame_type  = NORMAL_CAN  and
    --   tran_is_rtr      = '0'         and
    --   tran_frame_valid = '1'};

    -- psl tx_base_id_can_fd_cov : cover
    --  {tran_ident_type  = BASE        and
    --   tran_frame_type  = FD_CAN      and
    --   tran_is_rtr      = '0'         and
    --   tran_frame_valid = '1'};

    -- psl tx_extended_id_can_fd_cov : cover
    --  {tran_ident_type  = EXTENDED    and
    --   tran_frame_type  = FD_CAN      and
    --   tran_is_rtr      = '0'         and
    --   tran_frame_valid = '1'};

    -- psl tx_base_id_can_2_0_rtr_cov : cover
    --  {tran_ident_type  = BASE        and
    --   tran_frame_type  = NORMAL_CAN  and
    --   tran_is_rtr      = '1'         and
    --   tran_frame_valid = '1'};

    -- psl tx_extended_id_can_2_0_rtr_cov : cover
    --  {tran_ident_type  = EXTENDED    and
    --   tran_frame_type  = NORMAL_CAN  and
    --   tran_is_rtr      = '1'         and
    --   tran_frame_valid = '1'};

    -- psl tx_base_id_can_fd_rtr_cov : cover
    --  {tran_ident_type  = BASE        and
    --   tran_frame_type  = FD_CAN      and
    --   tran_is_rtr      = '1'         and
    --   tran_frame_valid = '1'};

    -- psl tx_extended_id_can_fd_rtr_cov : cover
    --  {tran_ident_type  = EXTENDED    and
    --   tran_frame_type  = FD_CAN      and
    --   tran_is_rtr      = '1'         and
    --   tran_frame_valid = '1'};


    -----------------------------------------------------------------------------------------------
    -- Received frame FDF/IDE/RTR combinations
    -----------------------------------------------------------------------------------------------

    -- psl rx_base_id_can_2_0_cov : cover
    --  {rec_ident_type   = BASE        and
    --   rec_frame_type   = NORMAL_CAN  and
    --   rec_is_rtr       = '0'         and
    --   store_metadata   = '1'};

    -- psl rx_extended_id_can_2_0_cov : cover
    --  {rec_ident_type  = EXTENDED    and
    --   rec_frame_type  = NORMAL_CAN  and
    --   rec_is_rtr      = '0'         and
    --   store_metadata  = '1'};

    -- psl rx_base_id_can_fd_cov : cover
    --  {rec_ident_type  = BASE        and
    --   rec_frame_type  = FD_CAN      and
    --   rec_is_rtr      = '0'         and
    --   store_metadata  = '1'};

    -- psl rx_extended_id_can_fd_cov : cover
    --  {rec_ident_type  = EXTENDED    and
    --   rec_frame_type  = FD_CAN      and
    --   rec_is_rtr      = '0'         and
    --   store_metadata  = '1'};

    -- psl rx_base_id_can_2_0_rtr_cov : cover
    --  {rec_ident_type  = BASE        and
    --   rec_frame_type  = NORMAL_CAN  and
    --   rec_is_rtr      = '1'         and
    --   store_metadata  = '1'};

    -- psl rx_extended_id_can_2_0_rtr_cov : cover
    --  {rec_ident_type  = EXTENDED    and
    --   rec_frame_type  = NORMAL_CAN  and
    --   rec_is_rtr      = '1'         and
    --   store_metadata  = '1'};

    -----------------------------------------------------------------------------------------------
    -- Bit stuffing and destuffing
    -----------------------------------------------------------------------------------------------

    -- psl bds_non_fix_to_fixed_change_cov : cover
    --  {bds_trigger = '1' and non_fix_to_fix_chng = '1'};

    -- psl bds_stuff_err_detect_cov : cover
    --  {stuff_err_q = '1'};

    -- psl bds_stuff_lvl_reached_regular_cov : cover
    --  {stuff_lvl_reached = '1' and fixed_stuff = '0'};

    -- psl bds_stuff_lvl_reached_fixed_cov : cover
    --  {stuff_lvl_reached = '1' and fixed_stuff = '1'};


    -----------------------------------------------------------------------------------------------
    -- Error counters
    -----------------------------------------------------------------------------------------------

    -- psl err_ctrs_inc_one_cov : cover
    --   {inc_one = '1'};

    -- psl err_ctrs_inc_eight_cov : cover
    --   {inc_eight = '1'};

    -- psl err_ctrs_dec_one_cov : cover
    --   {dec_one = '1'};

    -- psl err_ctrs_rec_saturation : cover
    --   {(rx_err_ctr_inc < rx_err_ctr_q) and rx_err_ctr_ce = '1'};


    -----------------------------------------------------------------------------------------------
    -- Error detector
    -----------------------------------------------------------------------------------------------

    -- Types of error being signalled

    -- psl err_detect_bit_err_cov : cover
    --  {bit_err = '1'};

    -- psl err_detect_bit_err_arb_cov : cover
    --  {bit_err_arb = '1'};

    -- psl err_detect_stuff_err_cov : cover
    --  {stuff_err = '1'};

    -- psl err_detect_form_err_cov : cover
    --  {form_err = '1'};

    -- psl err_detect_ack_err_cov : cover
    --  {ack_err = '1'};

    -- psl err_detect_crc_err_cov : cover
    --  {crc_err = '1'};

    -- psl err_detect_parity_err_cov : cover
    --  {tran_frame_parity_error = '1'};

    -- Types of error being captured

    -- psl err_capt_q_form_err_cov : cover
    --  {err_capt_err_type_q = ERC_FRM_ERR};

    -- psl err_capt_q_bit_err_cov : cover
    --  {err_capt_err_type_q = ERC_BIT_ERR};

    -- psl err_capt_q_crc_err_cov : cover
    --  {err_capt_err_type_q = ERC_CRC_ERR};

    -- psl err_capt_q_ack_err_cov : cover
    --  {err_capt_err_type_q = ERC_ACK_ERR};

    -- psl err_capt_q_stuff_err_cov : cover
    --  {err_capt_err_type_q = ERC_STUF_ERR};

    -- psl err_capt_q_prt_err_cov : cover
    --  {err_capt_err_type_q = ERC_PRT_ERR};


    -----------------------------------------------------------------------------------------------
    -- Fault confinement rules
    -----------------------------------------------------------------------------------------------

    -- psl err_ctr_inc_eight_A : cover
    --  {primary_err = '1' and is_receiver = '1'};

    -- psl err_ctr_inc_eight_B : cover
    --  {(act_err_ovr_flag = '1' and err_detected = '1') and
    --    (not(primary_err = '1' and is_receiver = '1'))};

    -- psl err_ctr_inc_eight_C : cover
    --  {(is_transmitter = '1' and err_detected = '1' and err_ctrs_unchanged = '0') and
    --    (not(act_err_ovr_flag = '1' and err_detected = '1')) and
    --    (not(primary_err = '1' and is_receiver = '1'))};

    -- psl err_ctr_inc_eight_D : cover
    --  { (err_delim_late = '1' or bit_err_after_ack_err = '1') and
    --    (not(is_transmitter = '1' and err_detected = '1' and err_ctrs_unchanged = '0')) and
    --    (not(act_err_ovr_flag = '1' and err_detected = '1')) and
    --    (not(primary_err = '1' and is_receiver = '1'))};

    -- psl err_ctr_dec_one_A : cover
    --  {decrement_rec = '1' and tran_valid = '0'};

    -- psl err_ctr_dec_one_B : cover
    --  {decrement_rec = '0' and tran_valid = '1'};


    -----------------------------------------------------------------------------------------------
    -- Protocol control
    -----------------------------------------------------------------------------------------------

    -- Error frame request in various parts of CAN frame!

    -- Note: SOF must be actually previous cycle, since at time when error
    --       frame request arrives, it is already next state!
    --
    -- psl err_frm_req_in_sof_cov : cover
    --  {curr_state = s_pc_sof; err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_base_id_in_base_cov : cover
    --  {curr_state = s_pc_base_id and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ext_id_in_ext_id_cov : cover
    --  {curr_state = s_pc_ext_id and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ext_id_in_rtr_srr_r1_cov : cover
    --  {curr_state = s_pc_rtr_srr_r1 and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ext_id_in_ide_cov : cover
    --  {curr_state = s_pc_ide and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_rtr_r1_cov : cover
    --  {curr_state = s_pc_rtr_r1 and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_edl_r1_cov : cover
    --  {curr_state = s_pc_edl_r1 and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_r0_ext_cov : cover
    --  {curr_state = s_pc_r0_ext and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_r0_fd_cov : cover
    --  {curr_state = s_pc_r0_fd and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_edl_r0_cov : cover
    --  {curr_state = s_pc_edl_r0 and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_esi_cov : cover
    --  {curr_state = s_pc_esi and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_dlc_cov : cover
    --  {curr_state = s_pc_dlc and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_data_cov : cover
    --  {curr_state = s_pc_data and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_stuff_count_cov : cover
    --  {curr_state = s_pc_stuff_count and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_crc_cov : cover
    --  {curr_state = s_pc_crc and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_crc_delim_cov : cover
    --  {curr_state = s_pc_crc_delim and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ack_cov : cover
    --  {curr_state = s_pc_ack and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_eof_cov : cover
    --  {curr_state = s_pc_eof and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_act_err_flag_cov : cover
    --  {curr_state = s_pc_act_err_flag and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ovr_flag_cov : cover
    --  {curr_state = s_pc_ovr_flag and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_ovr_delim_cov : cover
    --  {curr_state = s_pc_ovr_delim and err_frm_req = '1'};

    -- psl err_frm_req_in_s_pc_err_delim_cov : cover
    --  {curr_state = s_pc_err_delim and err_frm_req = '1'};


    -- Overload frame requests

    -- psl ovr_from_eof_cov : cover
    --  {curr_state = s_pc_eof and next_state = s_pc_ovr_flag};

    -- psl ovr_from_intermission_cov : cover
    --  {curr_state = s_pc_intermission and next_state = s_pc_ovr_flag};

    -- psl ovr_from_err_delim : cover
    --  {curr_state = s_pc_err_delim and next_state = s_pc_ovr_flag};

    -- psl ovr_from_ovr_delim_cov : cover
    --  {curr_state = s_pc_ovr_delim and next_state = s_pc_ovr_flag};


    -- Protocol exception

    -- psl pex_on_fdf_enable_cov : cover
    --  {pex_on_fdf_enable = '1' and mr_status_pexs = '1'};

    -- psl pex_on_res_enable_cov : cover
    --  {pex_on_res_enable = '1' and mr_status_pexs = '1'};

    -- psl pex_in_s_pc_r0_fd_cov : cover
    --  {curr_state = s_pc_r0_fd and pexs_set = '1'};

    -- psl pex_in_s_pc_edl_r1_cov : cover
    --  {curr_state = s_pc_edl_r1 and pexs_set = '1'};

    -- CAN frame being transmitted / received with various DUT compliance configurations.

    -- psl classical_can_cov : cover
    --   {curr_state = s_pc_base_id and mr_settings_pex = '0' and mr_mode_fde = '0'};

    -- psl fd_tolerant_can_cov : cover
    --   {curr_state = s_pc_base_id and mr_settings_pex = '1' and mr_mode_fde = '0'};

    -- psl fd_enabled_can_cov : cover
    --   {curr_state = s_pc_base_id and mr_settings_pex = '0' and mr_mode_fde = '1'};

    -- psl fd_enabled_with_pex : cover
    --   {curr_state = s_pc_base_id and mr_settings_pex = '1' and mr_mode_fde = '1'};

    -- ISO and Non-ISO variants of the protocol

    -- psl iso_fd_cov : cover
    --   {curr_state = s_pc_base_id and mr_settings_nisofd = '0'};

    -- psl non_iso_fd_cov : cover
    --   {curr_state = s_pc_base_id and mr_settings_nisofd = '1'};

    -- Arbitration lost

    -- psl arb_lost_base_id_cov : cover
    --  {curr_state = s_pc_base_id and arbitration_lost_i = '1'};

    -- psl arb_lost_rtr_srr_r1_cov : cover
    --  {curr_state = s_pc_rtr_srr_r1 and arbitration_lost_i = '1'};

    -- psl arb_lost_ide_cov : cover
    --  {curr_state = s_pc_ide and arbitration_lost_i = '1'};

    -- psl arb_lost_ext_id_cov : cover
    --  {curr_state = s_pc_ext_id and arbitration_lost_i = '1'};

    -- psl arb_lost_rtr_r1_cov : cover
    --  {curr_state = s_pc_rtr_r1 and arbitration_lost_i = '1'};


    -----------------------------------------------------------------------------------------------
    -- Reintegration counter
    -----------------------------------------------------------------------------------------------

    -- psl reinteg_ctr_clr_cov : cover
    --  {reinteg_ctr_clr = '1'};

    -- psl reinteg_ctr_expired_cov : cover
    --  {reinteg_ctr_expired = '1'};

    -- psl reinteg_ctr_ce_A : cover
    --  {(reinteg_ctr_clr = '1') and (not(reinteg_ctr_enable = '1' and rx_trigger = '1'))};

    -- psl reinteg_ctr_ce_B : cover
    --  {(not(reinteg_ctr_clr = '1')) and (reinteg_ctr_enable = '1' and rx_trigger = '1')};


    -----------------------------------------------------------------------------------------------
    -- RX Shift register
    -----------------------------------------------------------------------------------------------

    -- psl rx_shift_reg_clear_cov : cover
    --  {rx_clear = '1'};

    -- In linear mode, all bytes are shifting at once!
    -- psl rx_shift_reg_linear_mode_cov : cover
    --  {rx_shift_in_sel = '0' and rx_shift_ena = "1111"};

    -- psl rx_shift_reg_byte_mode_byte_1_cov : cover
    --  {rx_shift_in_sel = '1' and rx_shift_ena = "0001"};

    -- psl rx_shift_reg_byte_mode_byte_2_cov : cover
    --  {rx_shift_in_sel = '1' and rx_shift_ena = "0010"};

    -- psl rx_shift_reg_byte_mode_byte_3_cov : cover
    --  {rx_shift_in_sel = '1' and rx_shift_ena = "0100"};

    -- psl rx_shift_reg_byte_mode_byte_4_cov : cover
    --  {rx_shift_in_sel = '1' and rx_shift_ena = "1000"};

    -- psl rx_shift_reg_store_base_id_cov : cover
    --  {rx_store_base_id = '1'};

    -- psl rx_shift_reg_store_ext_id_cov : cover
    --  {rx_store_ext_id = '1'};

    -- psl rx_shift_reg_store_ide_cov : cover
    --  {rx_store_ide = '1'};

    -- psl rx_shift_reg_store_rtr_cov : cover
    --  {rx_store_rtr = '1'};

    -- psl rx_shift_reg_store_edl_cov : cover
    --  {rx_store_edl = '1'};

    -- psl rx_shift_reg_store_dlc_cov : cover
    --  {rx_store_dlc = '1'};

    -- psl rx_shift_reg_store_esi_cov : cover
    --  {rx_store_esi = '1'};

    -- psl rx_shift_reg_store_brs_cov : cover
    --  {rx_store_brs = '1'};

    -- psl rx_shift_reg_store_stuff_count_cov : cover
    --  {rx_store_stuff_count = '1'};


    -----------------------------------------------------------------------------------------------
    -- TX Shift register
    -----------------------------------------------------------------------------------------------

    -- psl tx_shift_reg_load_base_id_cov : cover
    --  {tx_load_base_id = '1'};

    -- psl tx_shift_reg_load_extended_id_cov : cover
    --  {tx_load_ext_id = '1'};

    -- psl tx_shift_reg_load_dlc_cov : cover
    --  {tx_load_dlc = '1'};

    -- psl tx_shift_reg_load_data_word_cov : cover
    --  {tx_load_data_word = '1'};

    -- psl tx_shift_reg_load_stuff_count_cov : cover
    --  {tx_load_stuff_count = '1'};

    -- psl tx_shift_reg_load_crc_cov : cover
    --  {tx_load_crc = '1'};

    -- psl tx_shift_flip_fstc_cov : cover
    --  {tran_frame_test.fstc = '1' and mr_mode_tstm = '1'};

    -- psl tx_shift_flip_fcrc_cov : cover
    --  {tran_frame_test.fcrc = '1' and mr_mode_tstm = '1'};

    -- psl tx_shift_flip_sdlc_cov : cover
    --  {tran_frame_test.sdlc = '1' and mr_mode_tstm = '1'};

    -- psl tx_shift_flip_fstc_disable_cov : cover
    --  {tran_frame_test.fstc = '1' and mr_mode_tstm = '0'};

    -- psl tx_shift_flip_fcrc_disable_cov : cover
    --  {tran_frame_test.fcrc = '1' and mr_mode_tstm = '0'};

    -- psl tx_shift_flip_sdlc_disable_cov : cover
    --  {tran_frame_test.sdlc = '1' and mr_mode_tstm = '0'};


end architecture;