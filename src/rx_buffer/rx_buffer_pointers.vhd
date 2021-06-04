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
--  RX Buffer pointers.
--
-- Purpose:
--  Pointers to RX Buffer RAM in RX Buffer and free memory calculation.
--  Following pointers are implemented:
--    1. Read pointer
--    2. Write pointer raw
--    3. Write pointer (regular, commited)
--    4. Write pointer for storing timestamp.
--  Counters for free memory:
--    1. RX mem free internal for control of storing and overrun
--    2. RX mem free available to user.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity rx_buffer_pointers is
    generic(
        -- RX Buffer size
        G_RX_BUFF_SIZE        :       natural range 32 to 4096 := 32
    );
    port(
        ------------------------------------------------------------------------
        -- Clocks and Asynchronous reset 
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in     std_logic;
        
        -- RX Buffer Reset (External + Release receive Buffer)
        rx_buf_res_n_q_scan  :in     std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Abort storing of frame in RX Buffer. Revert to last frame. Raw RX
        -- pointer will be reverted to internal RX pointers.
        rec_abort_f          :in     std_logic;

        -- Commit RX Frame to RX Buffer. Raw pointer will be stored internal
        -- RX pointer.
        commit_rx_frame      :in     std_logic;

        -- RX Buffer RAM is being written and there is enough space available.
        write_raw_OK         :in     std_logic;

        -- RX Frame is not commited, write pointer raw should be reverted to
        -- last stored write_pointer value.
        commit_overrun_abort :in     std_logic;

        -- RX Buffer FSM signals to store regular write pointer to timestamp 
        -- write pointer
        store_ts_wr_ptr      :in     std_logic;

        -- RX Buffer FSM signals to increment timestamp write pointer
        inc_ts_wr_ptr        :in     std_logic;

        -- RX Buffer RAM is being read by SW
        read_increment       :in     std_logic;

        -----------------------------------------------------------------------
        -- Status outputs
        -----------------------------------------------------------------------
        -- Read Pointer (access from SW)
        read_pointer           :out  std_logic_vector(11 downto 0);

        -- Read pointer incremented by 1 (combinationally)
        read_pointer_inc_1     :out  std_logic_vector(11 downto 0);

        -- Write pointer (committed, available to SW, after frame was stored)
        write_pointer          :out  std_logic_vector(11 downto 0);

        -- Write pointer RAW. Changing during frame, as frame is continously stored
        -- to the buffer. When frame is sucesfully received, it is updated to
        -- write pointer!
        write_pointer_raw      :out  std_logic_vector(11 downto 0);

        -- Timestamp write pointer
        write_pointer_ts       :out  std_logic_vector(11 downto 0);

        -- Number of free memory words available for user
        rx_mem_free_i          :out  std_logic_vector(12 downto 0)
    );
end entity;

architecture rtl of rx_buffer_pointers is

    ----------------------------------------------------------------------------
    -- Memory pointers
    ----------------------------------------------------------------------------

    function Log2( input:integer )
    return integer is
        variable temp,log:integer;
    begin
        temp := input;
        log := 0;
        while (temp > 1) loop
            temp := temp / 2;
            log := log + 1;
        end loop;
        return log;
    end function log2;
    
    -- Width of memory pointer
    constant C_PTR_WIDTH         : natural := log2(G_RX_BUFF_SIZE);

    -- Width of free memory
    constant C_FREE_MEM_WIDTH    : natural := C_PTR_WIDTH + 1;

    signal read_pointer_i        :       unsigned(C_PTR_WIDTH - 1 downto 0);
    signal read_pointer_inc_1_i  :       unsigned(C_PTR_WIDTH - 1 downto 0);
    signal write_pointer_i       :       unsigned(C_PTR_WIDTH - 1 downto 0);
    
    signal write_pointer_raw_i   :       unsigned(C_PTR_WIDTH - 1 downto 0);
    signal write_pointer_raw_d   :       unsigned(C_PTR_WIDTH - 1 downto 0);
    signal write_pointer_raw_ce  :       std_logic;
    
    signal write_pointer_ts_i    :    unsigned(C_PTR_WIDTH - 1 downto 0);
    signal write_pointer_ts_d    :    unsigned(C_PTR_WIDTH - 1 downto 0);
    signal write_pointer_ts_ce   :    std_logic;
    
    signal rx_mem_free_i_i     :       unsigned(C_FREE_MEM_WIDTH - 1 downto 0);

    ----------------------------------------------------------------------------
    -- Memory free status signals
    ----------------------------------------------------------------------------
    
    -- Raw value of number of free memory words.
    signal rx_mem_free_raw          :  unsigned(C_FREE_MEM_WIDTH - 1 downto 0);

    -- Number of free memory words calculated during frame storing before commit
    -- combinationally incremented by 1.
    signal rx_mem_free_raw_inc_1    :  unsigned(C_FREE_MEM_WIDTH - 1 downto 0);

    -- Number of free memory words calculated during frame storing before commit
    -- combinationally decremented by 1.
    signal rx_mem_free_raw_dec_1    :  unsigned(C_FREE_MEM_WIDTH - 1 downto 0);

    -- Number of free memory words available to SW, combinationally icnremented
    -- by 1.
    signal rx_mem_free_i_inc_1      :  unsigned(C_FREE_MEM_WIDTH - 1 downto 0);

begin
    read_pointer            <= std_logic_vector(resize(read_pointer_i, 12));
    read_pointer_inc_1      <= std_logic_vector(resize(read_pointer_inc_1_i, 12));
    write_pointer           <= std_logic_vector(resize(write_pointer_i, 12));
    write_pointer_raw       <= std_logic_vector(resize(write_pointer_raw_i, 12));
    write_pointer_ts        <= std_logic_vector(resize(write_pointer_ts_i, 12));
    rx_mem_free_i           <= std_logic_vector(resize(rx_mem_free_i_i, 13));


    ----------------------------------------------------------------------------
    -- Read pointer, incremented during read from RX Buffer FIFO.
    -- Moving to next word by reading (if there is sth to read).
    ----------------------------------------------------------------------------
    read_pointer_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            read_pointer_i         <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (read_increment = '1') then
                read_pointer_i    <= read_pointer_inc_1_i;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Write pointers available to the user manipulation. Loading 
    -- "write_pointer_raw_int" to  "write_pointer_int" when frame is committed.
    ----------------------------------------------------------------------------
    write_pointer_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            write_pointer_i       <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (commit_rx_frame = '1') then
                write_pointer_i   <= write_pointer_raw_i;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Write pointers raw manipulation:
    --  1. Increment when word is written to memory.
    --  2. Reset when "rec_abort_f" is active (Error frame) or frame finished
    --     and overrun occurred meanwhile. Reset to value of last commited write
    --     pointer.
    ----------------------------------------------------------------------------    
    write_pointer_raw_d <= write_pointer_raw_i + 1 when (write_raw_OK = '1')
                                                   else
                                   write_pointer_i;
    
    write_pointer_raw_ce <= '1' when (write_raw_OK = '1') else
                            '1' when (rec_abort_f = '1') else
                            '1' when (commit_overrun_abort = '1') else
                            '0';
    
    write_pointer_raw_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
           write_pointer_raw_i   <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (write_pointer_raw_ce = '1') then
                write_pointer_raw_i <= write_pointer_raw_d;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Timestamp write pointer.
    ----------------------------------------------------------------------------
    write_pointer_ts_d <= write_pointer_raw_i when (store_ts_wr_ptr = '1') else
                       write_pointer_ts_i + 1;

    -- Tick only when it should be incremented or stored
    write_pointer_ts_ce <= '1' when (store_ts_wr_ptr = '1') else
                           '1' when (inc_ts_wr_ptr = '1') else
                           '0';

    timestamp_write_ptr_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            write_pointer_ts_i  <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (write_pointer_ts_ce = '1') then
                write_pointer_ts_i <= write_pointer_ts_d;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Calculating amount of free memory.
    ----------------------------------------------------------------------------
    mem_free_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            rx_mem_free_i_i <= to_unsigned(G_RX_BUFF_SIZE, C_FREE_MEM_WIDTH);
            rx_mem_free_raw <= to_unsigned(G_RX_BUFF_SIZE, C_FREE_MEM_WIDTH);

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Calculate free memory internally (raw)
            --------------------------------------------------------------------
            if (read_increment = '1') then

                -- Read of memory word, and abort at the same time. Revert last
                -- commited value of read pointer incremented by 1.
                if (rec_abort_f = '1' or commit_overrun_abort = '1') then
                    rx_mem_free_raw <= rx_mem_free_i_inc_1;

                -- Read of memory word and no write of memory word. Load raw
                -- value incremented by 1.
                elsif (write_raw_OK = '0') then
                    rx_mem_free_raw <= rx_mem_free_raw_inc_1;

                -- Implicit else: Read increment and write of a word at the same
                -- time, rx_mem_free_raw remains unchanged.
                end if;

            else

                -- Abort, or abort was previously flaged -> Revert last commited
                -- value.
                if (rec_abort_f = '1' or commit_overrun_abort = '1') then
                    rx_mem_free_raw <= rx_mem_free_i_i;

                -- No read, write only, decrement by 1.
                elsif (write_raw_OK = '1') then
                    rx_mem_free_raw <= rx_mem_free_raw_dec_1;
                end if;
            end if;

            --------------------------------------------------------------------
            -- Calculate free memory for user:
            --      1. Increment when user reads the frame.
            --      2. Load RAW value when comitt occurs
            --------------------------------------------------------------------
            if (read_increment = '1') then
                if (commit_rx_frame = '1') then        
                    rx_mem_free_i_i <= rx_mem_free_raw_inc_1;
                else
                    rx_mem_free_i_i <= rx_mem_free_i_inc_1;
                end if;
    
            elsif (commit_rx_frame = '1') then
                rx_mem_free_i_i     <= rx_mem_free_raw;
            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Calculating incremented value of free memory combinationally
    ----------------------------------------------------------------------------
    mem_free_arith_proc : process(rx_mem_free_i_i, rx_mem_free_raw)
    begin
        rx_mem_free_i_inc_1     <= rx_mem_free_i_i + 1;
        rx_mem_free_raw_inc_1   <= rx_mem_free_raw + 1;
        rx_mem_free_raw_dec_1   <= rx_mem_free_raw - 1;
    end process;

    ----------------------------------------------------------------------------
    -- Calculation of Incremented Read Pointer combinationally. This is used
    -- for two things:
    --  1. Actual Increment of Read pointer during read of RX_DATA.
    --  2. Adressing RX Buffer RAM read side by incremented value to avoid one
    --     clock cycle delay on "read_pointer_int" and thus allow bursts on read
    --     from RX_DATA register!
    ----------------------------------------------------------------------------
    read_pointer_inc_proc : process(read_pointer_i)
    begin
        read_pointer_inc_1_i <= read_pointer_i + 1;
    end process;
    
    -- <RELEASE_OFF>
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Functional coverage
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);
    --
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
    -- psl rx_write_ptr_higher_than_read_ptr_cov : 
    --      cover {write_pointer_i > read_pointer_i};
    --
    -- psl rx_read_ptr_higher_than_write_ptr_cov : 
    --      cover {read_pointer_i > write_pointer_i};    

    -- <RELEASE_ON>
end architecture;