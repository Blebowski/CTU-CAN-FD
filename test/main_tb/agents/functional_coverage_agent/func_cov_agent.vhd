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
--    Functional coverage agent
--
--    Functional coverage agent implements functional coverage (PSL assertions)
--    for CTU CAN FD. The internal signals of the DUT are probed by External
--    Names.
--
--------------------------------------------------------------------------------
-- Revision History:
--    27.4.2025   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.clk_gen_agent_pkg.all;
use ctu_can_fd_tb.tb_shared_vars_pkg.all;

entity func_cov_agent is
    generic (
        -- RX Buffer size
        G_RX_BUFF_SIZE              :     natural range 32 to 4096;

        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT          :     natural range 1 to 8
    );
    port (
        -- DUT clock
        clk    :   in  std_logic
    );
end entity;

architecture tb of func_cov_agent is

    signal clk_delayed : std_logic;

begin

    -- Delay the clock so that we always sample stable signals and
    -- avoid possible delta-races. 1 ps should be "good enogh" that
    -- no signals
    clk_delayed <= clk after 1 ps;

    i_func_cov_can_core : entity ctu_can_fd_tb.func_cov_can_core
    port map (
        clk => clk_delayed
    );

    i_func_cov_prescaler : entity ctu_can_fd_tb.func_cov_prescaler
    port map (
        clk => clk_delayed
    );

    i_func_cov_bus_sampling : entity ctu_can_fd_tb.func_cov_bus_sampling
    port map (
        clk => clk_delayed
    );

    i_func_cov_rx_buffer : entity ctu_can_fd_tb.func_cov_rx_buffer
    generic map (
        G_RX_BUFF_SIZE => G_RX_BUFF_SIZE
    )
    port map (
        clk => clk_delayed
    );

    i_func_cov_tx_arbitrator : entity ctu_can_fd_tb.func_cov_tx_arbitrator
    generic map (
        G_TXT_BUFFER_COUNT => G_TXT_BUFFER_COUNT
    )
    port map (
        clk => clk_delayed
    );

    g_each_buf : for i in 0 to G_TXT_BUFFER_COUNT - 1 generate
    begin
        g_txt_buf_even : if ((i mod 2) = 0) generate
            i_func_cov_txt_buffer_even : entity ctu_can_fd_tb.func_cov_txt_buffer_even
            generic map (
                G_TXT_BUFFER_INDEX => i
            )
            port map (
                clk => clk_delayed
            );
        end generate;

        g_txt_buf_odd : if ((i mod 2) = 1) generate
            i_func_cov_txt_buffer_odd : entity ctu_can_fd_tb.func_cov_txt_buffer_odd
            generic map (
                G_TXT_BUFFER_INDEX => i
            )
            port map (
                clk => clk_delayed
            );
        end generate;

    end generate;

end architecture;