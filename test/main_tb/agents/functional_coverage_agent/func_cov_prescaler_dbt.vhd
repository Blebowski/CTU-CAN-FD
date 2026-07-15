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
--    Functional coverage for Prescaler
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

entity func_cov_prescaler_dbt is
    port (
        -- DUT clock
        clk    :   in  std_logic
    );
end entity;

architecture tb of func_cov_prescaler_dbt is

    alias is_tseg1 is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_nbt.is_tseg1 : std_logic >>;

    alias is_tseg2 is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_nbt.is_tseg2 : std_logic >>;

    alias exp_seg_length_ce is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_dbt.exp_seg_length_ce : std_logic >>;

    alias use_basic_segm_length is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_dbt.use_basic_segm_length : std_logic >>;

    alias phase_err is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_dbt.phase_err : unsigned(6 downto 0) >>;

    alias sjw is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_dbt.sjw : std_logic_vector(4 downto 0) >>;

    alias exit_segm_req is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_dbt.exit_segm_req : std_logic >>;

    alias exit_ph2_immediate is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_dbt.exit_ph2_immediate : std_logic >>;

    alias exit_segm_regular_tseg1 is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_dbt.exit_segm_regular_tseg1 : std_logic >>;

    alias exit_segm_regular_tseg2 is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.i_bit_segment_meter_dbt.exit_segm_regular_tseg2 : std_logic >>;

    alias resync_edge_valid is
        << signal .tb_top_ctu_can_fd.dut.i_prescaler.resync_edge_valid : std_logic >>;
begin

    -- psl default clock is rising_edge(clk);

    -- Positive resynchronization with E < SJW
    -- psl dbt_pos_resync_e_less_than_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and use_basic_segm_length = '0' and is_tseg1 = '1'
    --   and resync_edge_valid = '1' and
    --   (to_integer(unsigned(phase_err)) < to_integer(unsigned(sjw)))};

    -- Positive resynchronization with E > SJW
    -- psl dbt_pos_resync_e_more_than_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and use_basic_segm_length = '0' and is_tseg1 = '1'
    --   and resync_edge_valid = '1' and
    --   (to_integer(unsigned(phase_err)) > to_integer(unsigned(sjw)))};

    -- Positive resynchronization with E = SJW
    -- psl dbt_pos_resync_e_equal_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and use_basic_segm_length = '0' and is_tseg1 = '1'
    --   and resync_edge_valid = '1' and
    --   (to_integer(unsigned(phase_err)) = to_integer(unsigned(sjw)))};

    -- Negative resynchronization with E < SJW
    -- psl dbt_neg_resync_e_less_than_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and resync_edge_valid = '1' and is_tseg2 = '1' and
    --   (to_integer(unsigned(phase_err)) < to_integer(unsigned(sjw)))};

    -- Negative resynchronization with E > SJW
    -- psl dbt_neg_resync_e_more_than_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and resync_edge_valid = '1' and is_tseg2 = '1' and
    --   (to_integer(unsigned(phase_err)) > to_integer(unsigned(sjw)))};

    -- Negative resynchronization with E = SJW
    -- psl dbt_neg_resync_e_equal_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and resync_edge_valid = '1' and is_tseg2 = '1' and
    --   (to_integer(unsigned(phase_err)) = to_integer(unsigned(sjw)))};

    -- psl dbt_exit_segm_immediate_cov : cover
    --  {exit_segm_req = '1' and exit_ph2_immediate = '1'};

    -- psl dbt_exit_segm_regular_tseg1_cov : cover
    --  {exit_segm_req = '1' and exit_segm_regular_tseg1 = '1' and exit_segm_regular_tseg2 = '0'};

    -- psl dbt_exit_segm_regular_tseg2_cov : cover
    --  {exit_segm_req = '1' and exit_segm_regular_tseg1 = '0' and exit_segm_regular_tseg2 = '1'};

end architecture;