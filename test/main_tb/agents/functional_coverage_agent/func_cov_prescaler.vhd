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

entity func_cov_prescaler is
    port (
        -- DUT clock
        clk    :   in  std_logic
    );
end entity;

architecture tb of func_cov_prescaler is

    -----------------------------------------------------------------------------------------------
    -- Aliases to "btl_top" top
    -----------------------------------------------------------------------------------------------

    alias bit_rate_q is
        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.bit_rate_q : t_bit_rate >>;

--    alias tseg1_nbt is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.tseg1_nbt : std_logic_vector(7 downto 0) >>;
--
--    alias tseg2_nbt is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.tseg2_nbt : std_logic_vector(5 downto 0) >>;
--
--    alias tseg1_dbt is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.tseg1_dbt : std_logic_vector(6 downto 0) >>;
--
--    alias tseg2_dbt is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.tseg2_dbt : std_logic_vector(4 downto 0) >>;
--
--    alias brp_nbt is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.brp_nbt : std_logic_vector(7 downto 0) >>;
--
--    alias brp_dbt is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.brp_dbt : std_logic_vector(7 downto 0) >>;

    -- alias resync_edge_valid is
    --     << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.resync_edge_valid : std_logic >>;

    alias is_tseg1 is
        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.is_tseg1 : std_logic >>;

    alias is_tseg2 is
        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.is_tseg2 : std_logic >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "btl_synchronisation_checker" top
    -----------------------------------------------------------------------------------------------
--    alias sync_flag is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.i_btl_synchronisation_checker.sync_flag : std_logic >>;
--
--    alias h_sync_edge is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.i_btl_synchronisation_checker.h_sync_edge : std_logic >>;
--
--    alias resync_edge is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.i_btl_synchronisation_checker.resync_edge : std_logic >>;
--
--    alias h_sync_edge_valid is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.i_btl_synchronisation_checker.h_sync_edge_valid : std_logic >>;
--
    -----------------------------------------------------------------------------------------------
    -- Aliases to "btl_trigger_generator" top
    -----------------------------------------------------------------------------------------------

--    alias rx_trig_req_q is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.i_btl_trigger_generator.rx_trig_req_q : std_logic >>;
--
--    alias tx_trig_req is
--        << signal .tb_top_ctu_can_fd.i_dut.i_btl_top.i_btl_trigger_generator.tx_trig_req : std_logic >>;

begin

    -- psl default clock is rising_edge(clk);

    -----------------------------------------------------------------------------------------------
    -- Types of bit-rate / sample point
    -----------------------------------------------------------------------------------------------

    -- psl nominal_sample_cov : cover
    --  {bit_rate_q = BIT_RATE_NOMINAL};

    -- psl data_sample_cov : cover
    --  {bit_rate_q = BIT_RATE_FD};


    -----------------------------------------------------------------------------------------------
    -- Minimal / Maximal bit rates
    -----------------------------------------------------------------------------------------------

    -- TODO: Update these assertions accordingly!

    -- p sl minimal_bit_time_nbt_cov : cover
    --  {to_integer(unsigned(tseg1_nbt)) = 5 and to_integer(unsigned(tseg2_nbt)) = 3
    --   and to_integer(unsigned(brp_nbt)) = 1};

    -- p sl minimal_bit_time_dbt_cov : cover
    --  {to_integer(unsigned(tseg1_dbt)) = 3 and to_integer(unsigned(tseg2_dbt)) = 2
    --   and to_integer(unsigned(brp_dbt)) = 1};

    -- p sl maximal_bit_time_nbt_cov : cover
    --  {to_integer(unsigned(tseg1_nbt)) = 191 and to_integer(unsigned(tseg2_nbt)) = 63};

    -- p sl maximal_bit_time_dbt_cov : cover
    --  {to_integer(unsigned(tseg1_dbt)) = 95 and to_integer(unsigned(tseg2_dbt)) = 31};


    -----------------------------------------------------------------------------------------------
    -- Various other bit rates
    -----------------------------------------------------------------------------------------------

    -- TODO: Update these assertions!
    -- p sl brp_bin_1_1_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 1 and to_integer(unsigned(brp_dbt)) = 1};

    -- p sl brp_bin_2_1_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 2 and to_integer(unsigned(brp_dbt)) = 1};

    -- p sl brp_bin_3_1_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 3 and to_integer(unsigned(brp_dbt)) = 1};

    -- p sl brp_bin_4_1_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 4 and to_integer(unsigned(brp_dbt)) = 1};

    -- p sl brp_bin_4_3_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 4 and to_integer(unsigned(brp_dbt)) = 3};


    -----------------------------------------------------------------------------------------------
    -- Maximal prescaler
    -----------------------------------------------------------------------------------------------

    -- TODO: Update these assertions!
    -- p sl brp_nbt_max_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 255};

    -- p sl brp_dbt_max_cov : cover
    --  {to_integer(unsigned(brp_dbt)) = 255};


    -----------------------------------------------------------------------------------------------
    -- Resynchronization
    -----------------------------------------------------------------------------------------------

    -- TODO: Update these assertions!

    -- Positive resynchronization in Nominal bit rate
    -- p sl pos_resync_in_nominal_bit_rate_cov : cover
    --  {resync_edge_valid = '1' and is_tseg1 = '1' and bit_rate = BIT_RATE_NOMINAL};

    -- Negative resynchronization in Nominal bit rate!
    -- p sl neg_resync_in_nominal_bit_rate_cov : cover
    --  {resync_edge_valid = '1' and is_tseg2 = '1' and bit_rate = BIT_RATE_NOMINAL};

    -- Negative resynchronization in Data bit rate
    -- p sl neg_resync_in_data_bit_rate_cov : cover
    --  {resync_edge_valid = '1' and is_tseg2 = '1' and bit_rate = BIT_RATE_DATA};

    -- Positive resynchronization in Data bit rate
    -- p sl pos_resync_in_data_bit_rate_cov : cover
    --  {resync_edge_valid = '1' and is_tseg1 = '1' and bit_rate = BIT_RATE_DATA};


    -----------------------------------------------------------------------------------------------
    -- Synchronization
    -----------------------------------------------------------------------------------------------

    -- TODO: Upate these assertions
    -- p sl h_sync_ignored_due_to_previous_sync_cov : cover
    --  {sync_flag = '1' and h_sync_edge = '1'};

    -- p sl re_sync_ignored_due_to_previous_sync_cov : cover
    --  {sync_flag = '1' and resync_edge = '1'};

    -- p sl h_sync_in_tseg_1_cov : cover
    --  {h_sync_edge_valid = '1' and is_tseg1 = '1'};

    -- p sl h_sync_in_tseg_2_cov : cover
    --  {h_sync_edge_valid = '1' and is_tseg2 = '1'};

    -- Hard synchronization in TSEG1
    -- p sl re_sync_in_tseg_1_cov : cover
    --  {resync_edge_valid = '1' and is_tseg1 = '1'};

    -- p sl re_sync_in_tseg_2_cov : cover
    --  {resync_edge_valid = '1' and is_tseg2 = '1'};


    -----------------------------------------------------------------------------------------------
    -- Trigger generation
    -----------------------------------------------------------------------------------------------

    -- TODO: Update
    -- p sl tx_trigger_throttling_cov : cover
    --  {rx_trig_req_q = '1' and tx_trig_req = '1'}
    --  report "TX trigger throtlled!";

end architecture;