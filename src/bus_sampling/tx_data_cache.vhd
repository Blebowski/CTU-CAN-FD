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
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity tx_data_cache is
    generic(
        -- Depth of FIFO (Number of bits that can be stored)
        G_TX_CACHE_DEPTH        :     natural range 4 to 32 := 8;
        
        -- FIFO reset value
        G_TX_CACHE_RST_VAL      :     std_logic := '0'
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys         :in   std_logic;
        
        -- Asynchronous reset
        res_n           :in   std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Store input data
        write           :in   std_logic;
        
        -- Read output data
        read            :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Data signals
        ------------------------------------------------------------------------
        -- Data inputs
        data_in         :in   std_logic;
        
        -- Data output
        data_out        :out  std_logic
    );
end entity;

architecture rtl of tx_data_cache is

    -- Cache Memory (FIFO in DFFs)
    signal tx_cache_mem         : std_logic_vector(G_TX_CACHE_DEPTH - 1 downto 0);
    
    ---------------------------------------------------------------------------
    -- Access pointers
    ---------------------------------------------------------------------------
    -- Write Pointer
    signal write_pointer_q      : natural range 0 to G_TX_CACHE_DEPTH - 1;
    signal write_pointer_d      : natural range 0 to G_TX_CACHE_DEPTH - 1;

    -- Read pointer
    signal read_pointer_q       : natural range 0 to G_TX_CACHE_DEPTH - 1;
    signal read_pointer_d       : natural range 0 to G_TX_CACHE_DEPTH - 1; 

begin
    
    ----------------------------------------------------------------------------
    -- Combinationally incrementing write and read pointers
    ----------------------------------------------------------------------------
    write_pointer_d <= (write_pointer_q + 1) mod G_TX_CACHE_DEPTH;
    read_pointer_d <= (read_pointer_q + 1) mod G_TX_CACHE_DEPTH;

    
    ----------------------------------------------------------------------------
    -- Incrementing the pointers upon read or write.
    ----------------------------------------------------------------------------
    write_ptr_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            write_pointer_q        <= 0;
        elsif (rising_edge(clk_sys)) then
            if (write = '1') then
                write_pointer_q    <= write_pointer_d;
            end if;
        end if;
    end process;


    read_ptr_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            read_pointer_q         <= 0;
        elsif (rising_edge(clk_sys)) then
            if (read = '1') then
                read_pointer_q     <= read_pointer_d;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Storing data to FIFO.
    ----------------------------------------------------------------------------
    tx_cache_mem_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tx_cache_mem <= (OTHERS => G_TX_CACHE_RST_VAL);
        elsif (rising_edge(clk_sys)) then
            if (write = '1') then
                tx_cache_mem(write_pointer_q) <= data_in;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Reading data from FIFO combinationally.
    -- We need to have the data available right away, not pipelined!
    ----------------------------------------------------------------------------
    data_out <= tx_cache_mem(read_pointer_q);

    -- <RELEASE_OFF>
    ----------------------------------------------------------------------------
    -- Assertions on input signals
    ----------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- Here be stricter to make the check easier! Allow at least one bit free
    -- in the FIFO!
    -- psl no_fifo_overflow_asrt : assert never
    --  ((read_pointer_q - 1 = write_pointer_q) and (write = '1'))
    -- report "TX Cache is full, there should be less than 4 bits on the fly!"
    -- severity error;
    
    -- psl no_empty_read : assert never
    --  (read = '1' and write_pointer_q = read_pointer_q)
    -- report "Read from empty TX CACHE"
    -- severity error;
    --
    -- Note: When read pointer is equal to write pointer, FIFO is for sure
    -- empty, because we detect error when it is almost full. So we never get
    -- to situation that read pointer equals write pointer when FIFO is full!

    -- psl tx_data_cache_one_bit_on_fly_cov : cover
    --  {write_pointer_q = read_pointer_q + 1};

    -- psl tx_data_cache_two_bits_on_fly_cov : cover
    --  {write_pointer_q = read_pointer_q + 2};

    -- psl tx_data_cache_three_bits_on_fly_cov : cover
    --  {write_pointer_q = read_pointer_q + 3};

    -- psl tx_data_cache_four_bits_on_fly_cov : cover
    --  {write_pointer_q = read_pointer_q + 4};

    -- <RELEASE_ON>

end architecture;