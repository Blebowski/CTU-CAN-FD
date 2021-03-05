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
--  @Purpose:
--    Memory bus agent. Configurable over Vunit Communication library.
--
--    More on Vunit and its communication library can be found at:
---     https://vunit.github.io/documentation.html
--      https://vunit.github.io/com/user_guide.html
--
--    Memory bus agent executes accesses on simple RAM-like interface. It
--    contains FIFO which buffers the accesses. Following types of accesses are
--    supported:
--      1. Non-blocking write access
--      2. Blocking write access
--      3. Read access (always blocking)
--
--    This means that it is impossible to post several Read accesses and execute
--    them at once, because read always has to return value, therefore its
--    transaction must elapse.
--
--    Memory bus agent supports two modes:
--      normal mode
--      X-mode
--    In X-mode control signals are driven to X everywhere apart from setup +
--    hold within rising edge of clock and read data are sampled with data
--    output delay.
--
--    Memory Bus agent can be started or stopped via Vunit Communication Library.
--    When Memory Bus agent is started, it executes transactions from its
--    internal FIFO. When it is stopped, transcations are buffered in FIFO and
--    executed once Memory bus agent is started.
--
--    For correct operation in X-mode, clock period must be configured in
--    Memory bus agent. Clock signal is provided externally, but for proper
--    measurement of setup and hold times, its period must be provided. 
--
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--    04.2.2021   Adjusted to work without Vunits COM library.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use ieee.math_real.uniform;
use ieee.math_real.floor;

Library ctu_can_fd_tb;
use ctu_can_fd_tb.tb_communication_pkg.ALL;
use ctu_can_fd_tb.tb_report_pkg.ALL;

use ctu_can_fd_tb.mem_bus_agent_pkg.all;

entity mem_bus_agent is
    generic(
        G_ACCESS_FIFO_DEPTH  : natural := 32
    );
    port (
        -- Clock
        clk             : in    std_logic;

        -- Memory bus interface
        scs             : out   std_logic := '0';
        swr             : out   std_logic := 'X';
        srd             : out   std_logic := 'X';
        sbe             : out   std_logic_vector(3 downto 0) := "XXXX";
        write_data      : out   std_logic_vector(31 downto 0) := (others => '0');
        read_data       : in    std_logic_vector(31 downto 0);
        address         : out   std_logic_vector(15 downto 0) := (others => '0')
    );
end entity;

architecture tb of mem_bus_agent is

    type t_mem_bus_access_array is array (0 to G_ACCESS_FIFO_DEPTH - 1)
        of t_mem_bus_access_item;

    signal mem_bus_access_fifo  :   t_mem_bus_access_array;
    signal fifo_wp              :   integer := 0;
    signal fifo_rp              :   integer := 0;

    --------------------------------------------------------------------------
    -- Configuration parameters
    ---------------------------------------------------------------------------
    signal mem_bus_agent_ena    :   boolean := false;
    signal is_x_mode            :   boolean := true;
    
    -- Only single setup for all input signals
    signal x_mode_setup         :   time := 2 ns;
    signal x_mode_hold          :   time := 1 ns;
    
    -- Output signal is output data only!
    signal data_out_delay       :   time := 3 ns;

    signal period               :   time := 10 ns;

    signal read_data_i          :   std_logic_vector(31 downto 0);
    
begin
    
    --------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable cmd            : integer;
        variable reply_code     : integer;
        variable transaction    : t_mem_bus_access_item;
        variable tmp            : std_logic_vector(127 downto 0);
    begin
        receive_start(default_channel, C_MEM_BUS_AGENT_ID);
        
        -- Command is sent as message type
        cmd := com_channel_data.get_msg_code;
        reply_code := C_REPLY_CODE_OK;

        case cmd is
        when MEM_BUS_AGNT_CMD_START =>
            mem_bus_agent_ena <= true;
        when MEM_BUS_AGNT_CMD_STOP =>
            mem_bus_agent_ena <= false;

        -- For non-blocking writes only post to FIFO, don't wait!
        when MEM_BUS_AGNT_CMD_WRITE_NON_BLOCKING =>
            transaction.write := true;
            transaction.address := com_channel_data.get_param;
            tmp := com_channel_data.get_param;
            transaction.byte_enable := tmp(3 DOWNTO 0);
            transaction.write_data := tmp(35 DOWNTO 4);

            mem_bus_access_fifo(fifo_wp) <= transaction;
            wait for 0 ns;
            fifo_wp <= (fifo_wp + 1) mod G_ACCESS_FIFO_DEPTH;

        -- For blocking writes wait until FIFO is emptied. This will wait also if
        -- previous non-blocking transactions were pushed!
        when MEM_BUS_AGNT_CMD_WRITE_BLOCKING =>
            transaction.write := true;
            transaction.address := com_channel_data.get_param;
            tmp := com_channel_data.get_param;
            transaction.byte_enable := tmp(3 DOWNTO 0);
            transaction.write_data := tmp(35 DOWNTO 4);

            mem_bus_access_fifo(fifo_wp) <= transaction;
            wait for 0 ns;
            fifo_wp <= (fifo_wp + 1) mod G_ACCESS_FIFO_DEPTH;
            wait for 0 ns;
            
            wait until (fifo_wp = fifo_rp);

        -- Reads are always blocking
        when MEM_BUS_AGNT_CMD_READ =>
            transaction.write := false;
            transaction.address := com_channel_data.get_param;
            tmp := com_channel_data.get_param;
            transaction.byte_enable := tmp(3 DOWNTO 0);

            mem_bus_access_fifo(fifo_wp) <= transaction;
            wait for 0 ns;
            fifo_wp <= (fifo_wp + 1) mod G_ACCESS_FIFO_DEPTH;
            wait for 0 ns;

            wait until (fifo_wp = fifo_rp);
            --info("Read data when pushing response: " & to_hstring(read_data_i));
            com_channel_data.set_param(read_data_i);

        when MEM_BUS_AGNT_CMD_X_MODE_START =>
            is_x_mode <= true;

        when MEM_BUS_AGNT_CMD_X_MODE_STOP =>
            is_x_mode <= false;

        when MEM_BUS_AGNT_CMD_SET_X_MODE_SETUP =>
            x_mode_setup <= com_channel_data.get_param;

        when MEM_BUS_AGNT_CMD_SET_X_MODE_HOLD =>
            x_mode_hold <= com_channel_data.get_param;

        when MEM_BUS_AGNT_CMD_SET_PERIOD =>
            period <= com_channel_data.get_param;

        when MEM_BUS_AGNT_CMD_SET_OUTPUT_DELAY =>
            data_out_delay <= com_channel_data.get_param;

        when MEM_BUS_AGNT_CMD_WAIT_DONE =>
            if (fifo_wp /= fifo_rp) then
                wait until fifo_wp = fifo_rp;
            end if;

        when others =>
            info ("Invalid message type: " & integer'image(cmd));
            reply_code := C_REPLY_CODE_ERR;
        end case;

        receive_finish(default_channel, reply_code);
    end process;

    
    ---------------------------------------------------------------------------
    -- Memory bus access process
    ---------------------------------------------------------------------------
    mem_bus_access_proc : process
        variable curr_access   : t_mem_bus_access_item;

        procedure drive_access(
            variable mem_access    : inout t_mem_bus_access_item;
            signal   read_data_i   : out   std_logic_vector(31 downto 0)
        ) is
            variable post_re_time, post_re_time_2   : time;
        begin
            -- Chip select can't have Xs, otherwise we get dummy transactions!
            wait until falling_edge(clk);
            scs <= '1';

            if (is_x_mode) then
                swr <= 'X';
                srd <= 'X';
                write_data <= (OTHERS => 'X');
                sbe <= "XXXX";
                address <= (OTHERS => 'X');
                wait for (period / 2) - x_mode_setup;
            end if;

            if (mem_access.write) then
                swr <= '1';
                srd <= '0';
                write_data <= mem_access.write_data;
            else
                swr <= '0';
                srd <= '1';
            end if;

            address <= std_logic_vector(to_unsigned(mem_access.address, address'length));
            sbe <= mem_access.byte_enable;

            wait until rising_edge(clk);

            -- If hold time is larger than data output time, first sample output data,
            -- otherwise first drive X due to end of hold!
            if (x_mode_hold > data_out_delay) then
                post_re_time := data_out_delay;
                post_re_time_2 := x_mode_hold - post_re_time;
            else
                post_re_time := x_mode_hold;
                post_re_time_2 := data_out_delay - post_re_time;
            end if;

            if (is_x_mode) then
                wait for post_re_time;

                if (post_re_time = x_mode_hold) then
                    scs <= '0';
                    swr <= 'X';
                    srd <= 'X';
                    write_data <= (OTHERS => 'X');
                    sbe <= "XXXX";
                    address <= (OTHERS => 'X');
                else
                    read_data_i <= read_data;
                    wait for 0 ns;
                end if;
                
                wait for post_re_time_2;
                
                if (post_re_time = x_mode_hold) then
                    read_data_i <= read_data;
                else
                    scs <= '0';
                    swr <= 'X';
                    srd <= 'X';
                    write_data <= (OTHERS => 'X');
                    sbe <= "XXXX";
                    address <= (OTHERS => 'X');
                end if;

            -- Sample data without waiting, it should be available immediately!
            else
                wait for 1 ps; -- To avoid possible delta cycles
                read_data_i <= read_data;
                wait for (period / 2) - 2 ps; -- This will end up just before next falling edge!
                swr <= '0';
                srd <= '0';
                scs <= '0';
                address <= (OTHERS => '0');
                write_data <= (OTHERS => '0');
                sbe <= (OTHERS => '0');
            end if;
        end procedure;

    begin
        if (mem_bus_agent_ena) then
            while (true) loop
                if (not mem_bus_agent_ena) then
                    exit;            
                end if;

                -- There is something in FIFO -> do memory access
                if (fifo_rp /= fifo_wp) then
                    curr_access := mem_bus_access_fifo(fifo_rp);
                    drive_access(curr_access, read_data_i);

                    fifo_rp <= (fifo_rp + 1) mod G_ACCESS_FIFO_DEPTH;
                    wait for 0 ns;
                else
                    wait until fifo_rp /= fifo_wp or mem_bus_agent_ena=false;
                end if;
            end loop;
        else
            wait until mem_bus_agent_ena;
        end if;
    end process;
    
end architecture;