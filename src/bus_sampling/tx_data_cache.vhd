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
-- Module:
--  TX Data Cache.
--
-- Purpose:
--  Stores TX Data into FIFO buffer in time of regular sample point and read
--  at the time of secondary sample point. Output data are used for bit
--  error detection.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity tx_data_cache is
    generic (
        -- Depth of FIFO (Number of bits that can be stored)
        G_TX_CACHE_DEPTH        :     natural range 4 to 32;

        -- Size of TX Data cache pointer
        G_TX_CACHE_PTR_WIDTH    :     natural;

        -- FIFO reset value
        G_TX_CACHE_RST_VAL      :     std_logic
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys         :in   std_logic;
        res_n           :in   std_logic;

        -------------------------------------------------------------------------------------------
        -- Control signals
        -------------------------------------------------------------------------------------------
        -- Store input data
        write           :in   std_logic;

        -- Read output data
        read            :in   std_logic;

        -------------------------------------------------------------------------------------------
        -- Data signals
        -------------------------------------------------------------------------------------------
        -- Data inputs
        data_in         :in   std_logic;

        -- Data output
        data_out        :out  std_logic
    );
end entity;

architecture rtl of tx_data_cache is

    -- Cache Memory (FIFO in DFFs)
    signal tx_cache_mem         : std_logic_vector(G_TX_CACHE_DEPTH - 1 downto 0);

    -- Write Pointer
    signal write_pointer_q      : unsigned(G_TX_CACHE_PTR_WIDTH - 1 downto 0);
    signal write_pointer_d      : unsigned(G_TX_CACHE_PTR_WIDTH - 1 downto 0);

    -- Read pointer
    signal read_pointer_q       : unsigned(G_TX_CACHE_PTR_WIDTH - 1 downto 0);
    signal read_pointer_d       : unsigned(G_TX_CACHE_PTR_WIDTH - 1 downto 0);

begin

    -------------------------------------------------------------------------------------------
    -- Combinationally incrementing write and read pointers
    -------------------------------------------------------------------------------------------
    write_pointer_d <= write_pointer_q + 1;
    read_pointer_d <= read_pointer_q + 1;


    -------------------------------------------------------------------------------------------
    -- Incrementing the pointers upon read or write.
    -------------------------------------------------------------------------------------------
    p_write_ptr : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            write_pointer_q        <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            if (write = '1') then
                write_pointer_q    <= write_pointer_d;
            end if;
        end if;
    end process;


    p_read_ptr : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            read_pointer_q         <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            if (read = '1') then
                read_pointer_q     <= read_pointer_d;
            end if;
        end if;
    end process;


    -------------------------------------------------------------------------------------------
    -- Storing data to FIFO.
    -------------------------------------------------------------------------------------------
    p_tx_cache_mem : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tx_cache_mem <= (others => G_TX_CACHE_RST_VAL);
        elsif (rising_edge(clk_sys)) then
            if (write = '1') then
                tx_cache_mem(to_integer(write_pointer_q(2 downto 0))) <= data_in;
            end if;
        end if;
    end process;


    -------------------------------------------------------------------------------------------
    -- Reading data from FIFO combinationally.
    -- We need to have the data available right away, not pipelined!
    -------------------------------------------------------------------------------------------
    data_out <= tx_cache_mem(to_integer(read_pointer_q(2 downto 0)));

    -------------------------------------------------------------------------------------------
    -- Assertions on input signals
    -------------------------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- psl no_fifo_overflow_asrt : assert never
    --  ((read_pointer_q(3) /= write_pointer_q(3)) and
    --   (read_pointer_q(2 downto 0) = write_pointer_q(2 downto 0)) and
    --   (write = '1'))
    -- report "TX Cache is full, there should be less than 8 bits on the fly!";

    -- psl no_empty_read : assert never
    --  (read = '1' and write_pointer_q = read_pointer_q)
    -- report "Read from empty TX CACHE";

end architecture;