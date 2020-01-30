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
--    CAN Bus agent. Configurable over Vunit Communication library.
--    TODO: Further documentation!
--  
--------------------------------------------------------------------------------
-- Revision History:
--    25.1.2020   Created file
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
use work.can_agent_pkg.all;

entity can_agent is
    generic(
        G_DRIVER_FIFO_DEPTH : natural := 128;
        G_MONITOR_FIFO_DEPTH : natural := 128
    );
    port (
        -- CAN bus connections
        can_tx   :   in     std_logic;
        can_rx   :   out    std_logic := '1'
    );
end entity;

architecture tb of can_agent is

    type t_driver_fifo_mem is array (0 to G_DRIVER_FIFO_DEPTH - 1) of
        t_can_driver_entry;

    type t_monitor_fifo_mem is array (0 to G_MONITOR_FIFO_DEPTH - 1) of
        t_can_monitor_entry;

    -- Driver signals
    signal driver_mem               :   t_driver_fifo_mem;
    signal driver_wp                :   natural := 0;
    signal driver_rp                :   natural := 0;
    signal driver_ena               :   boolean := false;
    signal driving_in_progress      :   boolean := false;

    -- Monitor signals
    signal monitor_mem              :   t_monitor_fifo_mem;
    signal monitor_wp               :   natural := 0;
    signal monitor_rp               :   natural := 0;
    signal monitor_ena              :   boolean := false;
    signal monitoring_in_progress   :   boolean := false;

    ---------------------------------------------------------------------------
    -- Parameters configured over communication library
    ---------------------------------------------------------------------------
    signal driver_wait_timeout      :   time := 1 ms;

    signal driven_item              :   t_can_driver_entry := (
        value           => 'Z',
        drive_time      => 0 ns,
        print_msg       => false,
        msg             => C_EMPTY_STRING
    );

begin

    ---------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable msg, ack_msg : msg_t := new_msg;
        variable cmd : integer;
        variable push_item : t_can_driver_entry;
        variable tmp : boolean := false;
        
        procedure driver_fifo_push is
        begin
            driver_mem(driver_wp) <= push_item;
            driver_wp <= (driver_wp + 1) mod G_DRIVER_FIFO_DEPTH;
            wait for 0 ns;
        end procedure;

    begin
        receive(net, actor_can_agent, msg);
        debug(CAN_AGENT_TAG & "Received message on actor_can_agent");

        -- Command is sent as message type
        cmd := message_type(msg).p_code;
        ack_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_REPLY_OK));

        case cmd is
        when CAN_AGNT_CMD_DRIVER_START =>
            driver_ena <= true;

        when CAN_AGNT_CMD_DRIVER_STOP =>
            driver_ena <= false;

        when CAN_AGNT_CMD_DRIVER_FLUSH =>
            -- Just move to position of acutally driven item (Read pointer).
            -- This will efectively stash all written items and only current
            -- driven item will be finished!
            driver_wp <= driver_rp;

        when CAN_AGNT_CMD_DRIVER_GET_PROGRESS =>
            push(ack_msg, driving_in_progress);

        when CAN_AGNT_CMD_DRIVER_GET_DRIVEN_VAL =>
            push(ack_msg, can_rx);

        when CAN_AGNT_CMD_DRIVER_PUSH_ITEM =>
            push_item.value := pop(msg);
            push_item.drive_time := pop(msg);
            push_item.print_msg := pop(msg);
            if (push_item.print_msg) then
                push_item.msg := pop(msg);
            end if;

            -- Check for overflow, keep one item empty so that we don't have to
            -- handle pointers
            if ((driver_wp + 1) mod G_DRIVER_FIFO_DEPTH = driver_rp) then
                ack_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_REPLY_ERR));
                error(CAN_AGENT_TAG & "Driver FIFO overflow! -> Skipping");
            else
                driver_fifo_push;             
            end if;

        when CAN_AGNT_CMD_DRIVER_SET_WAIT_TIMEOUT =>
            driver_wait_timeout <= pop(msg);

        when CAN_AGNT_CMD_DRIVER_WAIT_FINISH =>
            wait for 0 ns;
            if (driving_in_progress) then
                wait until (driving_in_progress = false) for driver_wait_timeout;
            end if;

        when CAN_AGNT_CMD_DRIVER_DRIVE_SINGLE_ITEM =>
            if (not driver_ena) then
                driver_ena <= true;
                tmp := true;
            end if;

            push_item.value := pop(msg);
            push_item.drive_time := pop(msg);
            push_item.print_msg := pop(msg);
            if (push_item.print_msg) then
                push_item.msg := pop(msg);
            end if;

            if (driver_wp /= driver_rp) then
                warning(CAN_AGENT_TAG &
                        "Driver FIFO not empty, other items will be driven first...");
            end if;

            if ((driver_wp + 1) mod G_DRIVER_FIFO_DEPTH = driver_rp) then
                ack_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_REPLY_ERR));
                error(CAN_AGENT_TAG & "Driver FIFO overflow -> Skipping");
            else
                driver_fifo_push;
                wait until (driver_wp = driver_rp) for driver_wait_timeout;

                if (driver_wp /= driver_rp) then
                    warning(CAN_AGENT_TAG & " Drive_single_item timeout!");
                end if;
            end if;

            -- If driver was previously disabled, put it back into the same state!
            if (tmp) then
                driver_ena <= false;
            end if;

        when CAN_AGNT_CMD_DRIVER_DRIVE_ALL_ITEMS =>
            if (not driver_ena) then
                driver_ena <= true;
                tmp := true;
            end if;

            if (driver_wp = driver_rp) then
                warning(CAN_AGENT_TAG & "Driver FIFO empty, no items will be driven");
            else
                wait until (driver_wp = driver_rp) for driver_wait_timeout;
            end if;

            if (driver_wp /= driver_rp) then
                warning(CAN_AGENT_TAG & " Drive_all_items timeout!");
            end if;

            -- If driver was previously disabled, put it back into the same state!
            if (tmp) then
                driver_ena <= false;
            end if;

        when others =>
            warning (CAN_AGENT_TAG & "Invalid message type: " & integer'image(cmd));
            ack_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_REPLY_ERR));
        end case;

        reply(net, msg, ack_msg);
    end process;
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Driver process (reading from Driver FIFO)
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    driver_proc : process
    begin
        if (driver_ena) then
            while (true) loop
                if (not driver_ena) then
                    exit;            
                end if;

                -- There is something in FIFO -> drive it!
                if (driver_rp /= driver_wp) then
                    driving_in_progress <= true;
                    driven_item <= driver_mem(driver_rp);
                    wait for 0 ns;

                    debug(CAN_AGENT_TAG &
                          "Driving: " & std_logic'image(driven_item.value) &
                          " for time: " & time'image(driven_item.drive_time));
                    if (driven_item.print_msg) then
                        info(driven_item.msg);
                    end if;

                    can_rx <= driven_item.value;
                    wait for driven_item.drive_time;
                    driver_rp <= (driver_rp + 1) mod G_DRIVER_FIFO_DEPTH;
                    wait for 0 ns;
                else
                    driving_in_progress <= false;
                    wait until driver_rp /= driver_wp or driver_ena=false;
                end if;
            end loop;
        else
            wait until driver_ena;
        end if;
    end process;
    
    -- TODO: Implement monitor!!!
    
end architecture;