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
--    Functional coverage for RX Buffer
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

entity func_cov_rx_buffer is
    generic (
        -- RX Buffer size
        G_RX_BUFF_SIZE              :     natural range 32 to 4096
    );
    port (
        -- DUT clock
        clk    :   in  std_logic
    );
end entity;

architecture tb of func_cov_rx_buffer is

    -----------------------------------------------------------------------------------------------
    -- Aliases to "rx_buffer" top
    -----------------------------------------------------------------------------------------------

    alias data_overrun_i is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.data_overrun_i : std_logic >>;

    alias data_overrun_flg is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.data_overrun_flg : std_logic >>;

    alias read_increment is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.read_increment : std_logic >>;

    alias read_counter_q is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.read_counter_q : unsigned(4 downto 0) >>;

    alias commit_rx_frame is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.commit_rx_frame : std_logic >>;

    alias write_raw_intent is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.write_raw_intent : std_logic >>;

    alias mr_rx_settings_rtsop is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.mr_rx_settings_rtsop : std_logic >>;

    alias rec_is_rtr is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.rec_is_rtr : std_logic >>;

    alias rec_dlc is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.rec_dlc : std_logic_vector(3 downto 0) >>;

    alias rx_parity_error is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.rx_parity_error : std_logic >>;

    alias mr_command_crxpe is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.mr_command_crxpe : std_logic >>;

    -----------------------------------------------------------------------------------------------
    -- Aliases to "rx_buffer_pointers" top
    -----------------------------------------------------------------------------------------------
    alias C_FREE_MEM_WIDTH is
        << constant .tb_top_ctu_can_fd.dut.i_rx_buffer.i_rx_buffer_pointers.C_FREE_MEM_WIDTH : natural >>;

    alias rx_mem_free_raw is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.i_rx_buffer_pointers.rx_mem_free_raw : unsigned(C_FREE_MEM_WIDTH-1 downto 0) >>;

    alias rx_mem_free_i is
        << signal .tb_top_ctu_can_fd.dut.i_rx_buffer.i_rx_buffer_pointers.rx_mem_free_i : std_logic_vector(C_FREE_MEM_WIDTH-1 downto 0) >>;


begin

    -- psl default clock is rising_edge(clk);

    -----------------------------------------------------------------------------------------------
    -- Corner-cases
    -----------------------------------------------------------------------------------------------
    --
    -- psl rx_buf_overrun_flags_cov :
    --      cover {data_overrun_i = '1' and data_overrun_flg = '1'};
    --
    -- psl rx_buf_commit_and_read_cov :
    --      cover {read_increment = '1' and read_counter_q = "00001" and commit_rx_frame = '1'}
    --      report "RX Buffer Commit and Frame read finish - Simultaneous!";
    --
    -- psl rx_buf_write_and_read_cov :
    --      cover {write_raw_intent = '1' and read_increment = '1'};
    --
    -- psl rx_buf_read_after_write_cov :
    --      cover {write_raw_intent = '1'; read_increment = '1'};
    --
    -- psl rx_buf_write_after_read_cov :
    --      cover {read_increment = '1'; write_raw_intent = '1'};
    --
    -- psl rx_buf_sof_timestamp :
    --      cover {mr_rx_settings_rtsop = RTS_BEG and commit_rx_frame = '1'};
    --
    -- psl rx_buf_eof_timestamp :
    --      cover {mr_rx_settings_rtsop = RTS_END and commit_rx_frame = '1'};
    --
    -- psl rx_buf_burst_read_short_cov :
    --      cover {(read_increment = '1')[*4]};
    --
    -- psl rx_buf_burst_read_max_cov :
    --      cover {(read_increment = '1')[*16]};
    -- Note: SW reads the frame like so: Read metadata one by one and then 16 data words.
    --       Therefore highest burst achievable is 16 with current TB!
    --

    -----------------------------------------------------------------------------------------------
    -- Received frame types
    -----------------------------------------------------------------------------------------------

    -- psl rx_buf_store_rtr_cov :
    --      cover {rec_is_rtr = '1' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_empty_frame_cov :
    --      cover {rec_dlc = "0000" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_1_byte_frame_cov :
    --      cover {rec_dlc = "0001" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_2_byte_frame_cov :
    --      cover {rec_dlc = "0010" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_3_byte_frame_cov :
    --      cover {rec_dlc = "0011" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_4_byte_frame_cov :
    --      cover {rec_dlc = "0100" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_5_byte_frame_cov :
    --      cover {rec_dlc = "0101" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_8_byte_frame_cov :
    --      cover {rec_dlc = "1000" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_64_byte_frame_cov :
    --      cover {rec_dlc = "1111" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- In the cycle where mr_command_crxpe is active, the rx_parity_error is cleared.
    -- Since memory bus is driven in between falling edges, and functional coverage
    -- is sampled with delayed clock, we can't detect case where rx_parity_error = '1'
    -- and mr_command_crxpe = '1' simultaneously. Thus we detect falling edge due to
    -- clear!
    -- psl rx_parity_err_clr_cov :
    --      cover {rx_parity_error = '1'; rx_parity_error = '0' and mr_command_crxpe = '1'};


    -----------------------------------------------------------------------------------------------
    -- Pointers
    -----------------------------------------------------------------------------------------------

    -- psl rx_no_raw_mem_free_cov :
    --      cover {to_integer(unsigned(rx_mem_free_raw)) = 0};
    --
    -- psl rx_all_raw_mem_free_cov :
    --      cover {to_integer(unsigned(rx_mem_free_raw)) = G_RX_BUFF_SIZE};
    --
    -- psl rx_no_int_mem_free_cov :
    --      cover {to_integer(unsigned(rx_mem_free_i)) = 0};
    --
    -- psl rx_all_int_mem_free_cov :
    --      cover {to_integer(unsigned(rx_mem_free_i)) = G_RX_BUFF_SIZE};
    --


end architecture;