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
--  @Purpose:
--    Memory bus agent. Configurable over Vunit Communication library.
--    TODO: Further documentation!
--
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use ieee.math_real.uniform;
use ieee.math_real.floor;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

Library work;
use work.mem_bus_agent_pkg.all;

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

begin
    
    --------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable msg, ack_msg   : msg_t := new_msg;
        variable cmd            : integer;
        variable transaction    : t_mem_bus_access_item;
    begin
        receive(net, actor_mem_bus_agent, msg);
        
        -- Command is sent as message type
        cmd := message_type(msg).p_code;
        ack_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_REPLY_OK));

        case cmd is
        when MEM_BUS_AGNT_CMD_START =>
            mem_bus_agent_ena <= true;
        when MEM_BUS_AGNT_CMD_STOP =>
            mem_bus_agent_ena <= false;

        -- For non-blocking writes only post to FIFO, don't wait!
        when MEM_BUS_AGNT_CMD_WRITE_NON_BLOCKING =>
            transaction.write := true;
            transaction.address := pop(msg);
            transaction.byte_enable := pop(msg);
            transaction.write_data := pop(msg);

            mem_bus_access_fifo(fifo_wp) <= transaction;
            wait for 0 ns;
            fifo_wp <= (fifo_wp + 1) mod G_ACCESS_FIFO_DEPTH;

        -- For blocking writes wait until FIFO is emptied. This will wait also if
        -- previous non-blocking transactions were pushed!
        when MEM_BUS_AGNT_CMD_WRITE_BLOCKING =>
            transaction.write := true;
            transaction.address := pop(msg);
            transaction.byte_enable := pop(msg);
            if (transaction.write) then
                transaction.write_data := pop(msg);
            end if;

            mem_bus_access_fifo(fifo_wp) <= transaction;
            wait for 0 ns;
            fifo_wp <= (fifo_wp + 1) mod G_ACCESS_FIFO_DEPTH;
            wait for 0 ns;
            
            wait until (fifo_wp = fifo_rp);

        -- Reads are always blocking
        when MEM_BUS_AGNT_CMD_READ =>
            transaction.write := false;
            transaction.address := pop(msg);
            transaction.byte_enable := pop(msg);

            mem_bus_access_fifo(fifo_wp) <= transaction;
            wait for 0 ns;
            fifo_wp <= (fifo_wp + 1) mod G_ACCESS_FIFO_DEPTH;
            wait for 0 ns;

            wait until (fifo_wp = fifo_rp);
            push(ack_msg, mem_bus_access_fifo(fifo_wp).read_data);

        when MEM_BUS_AGNT_CMD_X_MODE_START =>
            is_x_mode <= true;
        when MEM_BUS_AGNT_CMD_X_MODE_STOP =>
            is_x_mode <= false;

        when MEM_BUS_AGNT_CMD_SET_X_MODE_SETUP =>
            x_mode_setup <= pop(msg);

        when MEM_BUS_AGNT_CMD_SET_X_MODE_HOLD =>
            x_mode_hold <= pop(msg);

        when MEM_BUS_AGNT_CMD_SET_PERIOD =>
            period <= pop(msg);

        when MEM_BUS_AGNT_CMD_SET_OUTPUT_DELAY =>
            data_out_delay <= pop(msg);

        when MEM_BUS_AGNT_CMD_WAIT_DONE =>
            if (fifo_wp /= fifo_rp) then
                wait until fifo_wp = fifo_rp;
            end if;

        when others =>
            info ("Invalid message type: " & integer'image(cmd));
            ack_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_REPLY_ERR));
        end case;

        reply(net, msg, ack_msg);
    end process;

    
    ---------------------------------------------------------------------------
    -- Memory bus access process
    ---------------------------------------------------------------------------
    mem_bus_access_proc : process
        variable curr_access : t_mem_bus_access_item;
        
        procedure drive_access(
            variable mem_access    : inout t_mem_bus_access_item
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
                    mem_access.read_data := read_data;
                end if;
                
                wait for post_re_time_2;
                
                if (post_re_time = x_mode_hold) then
                    mem_access.read_data := read_data;
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
                mem_access.read_data := read_data;
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
                    drive_access(curr_access);
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