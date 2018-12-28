--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
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
-- Purpose:
--  Pointers to RX Buffer RAM in RX Buffer and free memory calculation.
--  Following pointers are implemented:
--    1. Read pointer
--    2. Write pointer raw
--    3. Write pointer (regular, commited)
--    4. Write pointer for storing extra timestamp from end of frame.
--  Counters for free memory:
--    1. RX mem free internal for control of storing and overrun
--    2. RX mem free available to user.
--------------------------------------------------------------------------------
-- Revision History:
--    15.12.2018    Created file
--------------------------------------------------------------------------------

Library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CAN_FD_frame_format.all;
use work.CAN_FD_register_map.all;
use work.can_components.all;

entity rx_buffer_pointers is
    generic(

        -- Only 2^k are allowed as buff_size. Memory adressing is in modular 
        -- arithmetic, synthesis of modulo by number other than 2^k might not
        -- play nicely (will consume lot of LUTs)!!
        buff_size                   :       natural range 32 to 4096 := 32
    );
    port(
        ------------------------------------------------------------------------
        -- Clocks and reset 
        ------------------------------------------------------------------------
        signal clk_sys              :in     std_logic; --System clock
        signal res_n                :in     std_logic; --Async. reset

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
               
        -- If error frame occurred, CAN Core activates this signal.
        -- "write_pointer_raw" will be restarted to last committed value in
        -- "write_pointer".
        signal rec_abort            :in     std_logic;

        -- Frame is commited to RX Buffer FSM, raw write pointer should be moved
        -- to regular write pointer.
        signal commit_rx_frame      :in     std_logic;

        -- RX Buffer RAM is being written and there is enough space available.
        signal write_raw_OK         :in     std_logic;

        -- RX Frame is not commited, write pointer raw should be reverted to
        -- last stored write_pointer value.
        signal commit_overrun_abort :in     std_logic;

        -- RX Buffer FSM signals to store write pointer to extra write pointer
        signal store_extra_wr_ptr   :in     std_logic;

        -- RX Buffer FSM signals to increment extra write pointer
        signal inc_extra_wr_ptr     :in     std_logic;

        -- RX Buffer RAM is being read by SW
        signal read_increment       :in     std_logic;


        ------------------------------------
        -- User registers interface
        ------------------------------------
        
        -- Driving bus from registers
        signal drv_bus              :in     std_logic_vector(1023 downto 0);

        ------------------------------------
        -- Pointer outputs
        ------------------------------------
        -- Read Pointer (access from SW)
        signal read_pointer         :out     natural range 0 to buff_size - 1;

        -- Read pointer incremented by 1 (combinationally)
        signal read_pointer_inc_1   :out     natural range 0 to buff_size - 1;

        -- Write pointer (committed, available to SW, after frame was stored)
        signal write_pointer        :out     natural range 0 to buff_size - 1;

        -- Write pointer RAW. Changing during frame, as frame is continously stored
        -- to the buffer. When frame is sucesfully received, it is updated to
        -- write pointer!
        signal write_pointer_raw    :out     natural range 0 to buff_size - 1;

        -- Extra write pointer which is used for storing timestamp at the end of
        -- data frame!
        signal write_pointer_extra_ts :out   natural range 0 to buff_size - 1;

        ------------------------------------
        -- Mem. free outputs
        ------------------------------------
        -- Number of free memory words available for user
        signal rx_mem_free_int      :out    natural range 0 to buff_size

    );
end entity;


architecture rtl of rx_buffer_pointers is

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Driving bus signal aliases
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    -- Erase command from driving registers. Resets FIFO pointers!
    signal drv_erase_rx             :       std_logic;

    -- Raw value of number of free memory words.
    signal rx_mem_free_raw          :       natural range 0 to buff_size;

    -- Number of free memory words calculated during frame storing before commit
    -- combinationally incremented by 1.
    signal rx_mem_free_raw_inc_1    :       natural range 0 to buff_size + 1;

    -- Number of free memory words calculated during frame storing before commit
    -- combinationally decremented by 1.
    signal rx_mem_free_raw_dec_1    :       natural range -1 to buff_size;

    -- Number of free memory words available to SW, combinationally icnremented
    -- by 1.
    signal rx_mem_free_int_inc_1    :       natural range 0 to buff_size + 1;

begin

    ----------------------------------------------------------------------------
    -- Driving bus aliases
    ----------------------------------------------------------------------------
    drv_erase_rx          <= drv_bus(DRV_ERASE_RX_INDEX);


    ----------------------------------------------------------------------------
    -- Read pointer, incremented during read from RX Buffer FIFO.
    ----------------------------------------------------------------------------
    read_pointer_proc : process(clk_sys, res_n, drv_erase_rx)
    begin
        if (res_n = ACT_RESET or drv_erase_rx = '1') then
            read_pointer            <= 0;

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Moving to next word by reading (if there is sth to read).
            --------------------------------------------------------------------
            if (read_increment = '1') then
                read_pointer        <= read_pointer_inc_1;
            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Write pointers manipulation. Following pointers are handled:
    --   1. Write pointer raw, which is incremented upon each write to RX buffer
    --      RAM.
    --   2. Write pointer which is available to user.
    ----------------------------------------------------------------------------
    write_pointer_proc : process(clk_sys, res_n, drv_erase_rx)
    begin
        if (res_n = ACT_RESET or drv_erase_rx = '1') then
            write_pointer           <= 0;
            write_pointer_raw       <= 0;

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Loading "write_pointer_raw" to  "write_pointer" when frame is
            -- committed.
            --------------------------------------------------------------------
            if (commit_rx_frame = '1') then
                write_pointer       <= write_pointer_raw;
            end if;

            --------------------------------------------------------------------
            -- Updating "write_pointer_raw":
            --      1. Increment when word is written to memory.
            --      2. Reset when "rec_abort" is active (Error frame) or 
            --         frame finished and overrun occurred meanwhile. Reset to
            --         value of last commited write pointer.
            --------------------------------------------------------------------
            if (write_raw_OK = '1') then
                write_pointer_raw   <= (write_pointer_raw + 1) mod buff_size;
                        
            elsif (rec_abort = '1' or commit_overrun_abort = '1') then
                write_pointer_raw   <= write_pointer;

            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Extra write pointer for storing value of timestamp from end of frame.
    ----------------------------------------------------------------------------
    extra_write_ptr_proc : process(clk_sys, res_n, drv_erase_rx)
    begin
        if (res_n = ACT_RESET or drv_erase_rx = '1') then
            write_pointer_extra_ts  <= 0;

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Setting extra write pointer for write of timestamp from end of
            -- frame...
            -- First timestamp word is offset by 3 from committed write pointer.
            -- Second one is offset by 4.
            --------------------------------------------------------------------
            if (store_extra_wr_ptr = '1') then
                write_pointer_extra_ts    <= write_pointer;

            elsif (inc_extra_wr_ptr = '1') then
                write_pointer_extra_ts    <= (write_pointer_extra_ts + 1) mod
                                              buff_size;

            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Calculating amount of free memory.
    ----------------------------------------------------------------------------
    mem_free_proc : process(clk_sys, res_n, drv_erase_rx)
    begin
        if (res_n = ACT_RESET or drv_erase_rx = '1') then
            rx_mem_free_int         <= buff_size;
            rx_mem_free_raw         <= buff_size;

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Calculate free memory internally (raw)
            --------------------------------------------------------------------
            if (read_increment = '1') then

                -- Read of memory word, and abort at the same time. Revert last
                -- commited value of read pointer incremented by 1.
                if (rec_abort = '1' or commit_overrun_abort = '1') then
                    rx_mem_free_raw <= rx_mem_free_int_inc_1;

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
                if (rec_abort = '1' or commit_overrun_abort = '1') then
                    rx_mem_free_raw <= rx_mem_free_int;

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
                    rx_mem_free_int     <= rx_mem_free_raw_inc_1;
                else
                    rx_mem_free_int     <= rx_mem_free_int_inc_1;
                end if;
    
            elsif (commit_rx_frame = '1') then
                rx_mem_free_int         <= rx_mem_free_raw;
            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Calculating incremented value of free memory combinationally
    ----------------------------------------------------------------------------
    rx_mem_free_int_inc_1   <= rx_mem_free_int + 1;
    rx_mem_free_raw_inc_1   <= rx_mem_free_raw + 1;
    rx_mem_free_raw_dec_1   <= rx_mem_free_raw - 1;


    ----------------------------------------------------------------------------
    -- Calculation of Incremented Read Pointer combinationally. This is used
    -- for two things:
    --  1. Actual Increment of Read pointer during read of RX_DATA.
    --  2. Adressing RX Buffer RAM read side by incremented value to avoid one
    --     clock cycle delay on "read_pointer" and thus allow bursts on read
    --     from RX_DATA register!
    ----------------------------------------------------------------------------
    read_pointer_inc_1 <= (read_pointer + 1) mod buff_size;


end architecture;
