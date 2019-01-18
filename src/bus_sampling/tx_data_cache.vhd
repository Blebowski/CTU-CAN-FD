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
--  TX Data Cache (FIFO-like). Stores TX Data into FIFO buffer. TX Data are
--  stored in time of regular sample point and read at the time of delayed
--  sample point. Read data are used for bit error detection.
--------------------------------------------------------------------------------
--    16.01.2019  Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.endian_swap.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity tx_data_cache is
    generic(
        -- Reset polarity
        constant reset_polarity         :     std_logic;
        
        -- Depth of FIFO (Number of bits that can be stored)
        constant tx_cache_depth         :     natural range 4 to 32 := 8;
        
        -- FIFO reset value
        constant tx_cache_res_val       :     std_logic
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        signal clk_sys                  :in   std_logic;
        signal res_n                    :in   std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Store input data
        signal write                    :in   std_logic;
        
        -- Read output data
        signal read                     :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Data signals
        ------------------------------------------------------------------------
        signal data_in                  :in   std_logic;
        signal data_out                 :out  std_logic
        );
end entity;


architecture rtl of tx_data_cache is


    -- Cache Memory (FIFO in DFFs)
    signal tx_cache_mem         : std_logic_vector(tx_cache_depth - 1 downto 0);
    
    ---------------------------------------------------------------------------
    -- Access pointers
    ---------------------------------------------------------------------------
    -- Write Pointer
    signal write_pointer        : natural range 0 to tx_cache_depth - 1;
    signal write_pointer_nxt    : natural range 0 to tx_cache_depth - 1;

    -- Read pointer
    signal read_pointer         : natural range 0 to tx_cache_depth - 1;
    signal read_pointer_nxt     : natural range 0 to tx_cache_depth - 1; 

begin
    
    ----------------------------------------------------------------------------
    -- Combinationally incrementing write and read pointers
    ----------------------------------------------------------------------------
    write_pointer_nxt <= (write_pointer + 1) mod tx_cache_depth;
    read_pointer_nxt <= (read_pointer + 1) mod tx_cache_depth;

    
    ----------------------------------------------------------------------------
    -- Incrementing the pointers upon read or write.
    ----------------------------------------------------------------------------
    write_ptr_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            write_pointer        <= 0;
        elsif (rising_edge(clk_sys)) then
            if (write = '1') then
                write_pointer    <= write_pointer_nxt;
            end if;
        end if;
    end process;


    read_ptr_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            read_pointer         <= 0;
        elsif (rising_edge(clk_sys)) then
            if (read = '1') then
                read_pointer     <= read_pointer_nxt;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Storing data to FIFO.
    ----------------------------------------------------------------------------
    tx_cache_mem_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            tx_cache_mem <= (OTHERS => tx_cache_res_val);
        elsif (rising_edge(clk_sys)) then
            if (write = '1') then
                tx_cache_mem(write_pointer) <= data_in;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Reading data from FIFO combinationally.
    -- We need to have the data available right away, not pipelined!
    ----------------------------------------------------------------------------
    data_out <= tx_cache_mem(read_pointer);


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Assertions on input signals
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    
    ----------------------------------------------------------------------------
    -- Monitor overflow of both pointers (as if highest bit), and check that:
    --  1. There is no read when FIFO is empty
    --  2. It never happends that FIFO is full and write occurs, this is
    --     failure, since design does not have this control and would corrupt
    --     previous data. This is not expected to happend since 8 bits in FIFO
    --     means de-facto 8 bits on the fly on CAN Bus. This is crazy and no
    --     one will ever use it! Just in case something like this happends be
    --     sure that it is caught in simulation...
    ----------------------------------------------------------------------------
    -- pragma translate_off
    assert_proc : process
        variable write_ptr_higher : boolean := true;
        variable cache_full       : boolean := false;
        variable cache_empty      : boolean := false;
    begin
        wait until rising_edge(clk_sys);
        
        -- Write overflows -> Write is now under read
        if (write = '1' and write_pointer = tx_cache_depth - 1) then
            write_ptr_higher := false;
        end if;
        
        -- Read overflows -> Read is now under write
        if (read = '1' and read_pointer = tx_cache_depth - 1) then
            write_ptr_higher := true;
        end if;

        -- Find out if cache is full or empty!
        if (read_pointer = write_pointer) then
            if (write_ptr_higher) then
                cache_full := true;
            else
                cache_empty := true;
            end if;
        end if;
        
        -- Check that if FIFO is empty no read occurs
        -- if (cache_empty and read = '1') then
        --     report "Should not read from empty TX Data cache -> BUG!"
        --     severity failure; 
        -- end if;

        -- Check that if FIFO is full no write occurs
        -- if (cache_full and read = '1') then
        --     report "Should not write to full TX Data cache -> BUG!"
        --     severity failure; 
        -- end if;
    end process;
    -- pragma translate_on
    
    
end architecture;
