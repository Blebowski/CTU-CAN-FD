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
    signal monitor_in_progress      :   boolean := false;
    signal monitor_state            :   t_can_monitor_state := mon_disabled;
    
    ---------------------------------------------------------------------------
    -- Parameters configured over communication library
    ---------------------------------------------------------------------------
    
    -- Driver
    signal driver_wait_timeout      :   time := 1 ms;
    signal driven_item              :   t_can_driver_entry := (
        value           => 'Z',
        drive_time      => 0 ns,
        print_msg       => false,
        msg             => C_EMPTY_STRING
    );
    
    -- Monitor
    signal monitor_wait_timeout     :   time := 1 ms;
    signal monitored_item           :   t_can_monitor_entry := (
        value           => 'Z',
        monitor_time    => 0 ns,
        print_msg       => false,
        msg             => C_EMPTY_STRING,
        check_severity  => warning
    );

    signal monitor_trigger          :   t_can_monitor_trigger := trig_immediately;
    signal monitor_sample_rate      :   time := 1 ns;
    signal monitor_trig_wait_time   :   time := 100 ns;
    
    -- Debug signal only, shows where can_rx is sampled!
    signal monitor_sample           :   std_logic := '0';

    -- Debug signal only, shows mismatch (by an X)
    signal monitor_mismatch         :   std_logic := '0';

    -- Mismatch counter
    signal mon_mismatch_ctr         :   integer := 0;

begin

    ---------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable msg, ack_msg : msg_t := new_msg;
        variable cmd : integer;
        variable push_item : t_can_driver_entry;
        variable push_mon_item : t_can_monitor_entry;
        variable tmp : boolean := false;
        variable tmp_int : integer := 0;

        procedure driver_fifo_push is
        begin
            driver_mem(driver_wp) <= push_item;
            driver_wp <= (driver_wp + 1) mod G_DRIVER_FIFO_DEPTH;
            wait for 0 ns;
        end procedure;

        procedure monitor_fifo_push is
        begin
            monitor_mem(monitor_wp) <= push_mon_item;
            monitor_wp <= (monitor_wp + 1) mod G_MONITOR_FIFO_DEPTH;
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
            else
                push_item.msg := (OTHERS => ' ');
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
            else
                push_item.msg := (OTHERS => ' ');
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

        when CAN_AGNT_CMD_MONITOR_START =>
            monitor_ena <= true;

        when CAN_AGNT_CMD_MONITOR_STOP =>
            monitor_ena <= false;

        when CAN_AGNT_CMD_MONITOR_FLUSH =>
            -- Just move to position of acutally driven item (Read pointer).
            -- This will efectively stash all written items and only current
            -- driven item will be finished!
            monitor_wp <= monitor_rp;

        when CAN_AGNT_CMD_MONITOR_GET_STATE =>
            push(ack_msg, t_can_monitor_state'pos(monitor_state));

        when CAN_AGNT_CMD_MONITOR_GET_MONITORED_VAL =>
            push(ack_msg, monitored_item.value);

        when CAN_AGNT_CMD_MONITOR_PUSH_ITEM =>
            push_mon_item.value := pop(msg);
            push_mon_item.monitor_time := pop(msg);
            tmp_int := pop(msg);
            push_mon_item.check_severity := log_level_t'val(tmp_int);
            push_mon_item.print_msg := pop(msg);
            if (push_mon_item.print_msg) then
                push_mon_item.msg := pop(msg);
            else
                push_mon_item.msg := (OTHERS => ' ');
            end if;
            
            monitor_fifo_push;

        when CAN_AGNT_CMD_MONITOR_SET_WAIT_TIMEOUT =>
            monitor_wait_timeout <= pop(msg);
            
        when CAN_AGNT_CMD_MONITOR_WAIT_FINISH =>
            wait for 0 ns;
            if (monitor_in_progress) then
                wait until (monitor_in_progress = false) for monitor_wait_timeout;
            end if;

        when CAN_AGNT_CMD_MONITOR_MONITOR_SINGLE_ITEM =>
            if (not monitor_ena) then
                monitor_ena <= true;
                tmp := true;
            end if;
            
            push_mon_item.value := pop(msg);
            push_mon_item.monitor_time := pop(msg);
            tmp_int := pop(msg);
            push_mon_item.check_severity := log_level_t'val(tmp_int);
            push_mon_item.print_msg := pop(msg);
            if (push_mon_item.print_msg) then
                push_mon_item.msg := pop(msg);
            else
                push_mon_item.msg := (OTHERS => ' ');
            end if;

            if (monitor_wp /= monitor_rp) then
                warning(CAN_AGENT_TAG &
                        "Monitor FIFO not empty, other items will be monitored first...");
            end if;
            
            if ((monitor_wp + 1) mod G_MONITOR_FIFO_DEPTH = monitor_rp) then
                ack_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_REPLY_ERR));
                error(CAN_AGENT_TAG & "Monitor FIFO overflow -> Skipping");
            else
                monitor_fifo_push;
                wait until (monitor_wp = monitor_rp) for monitor_wait_timeout;

                if (monitor_wp /= monitor_rp) then
                    warning(CAN_AGENT_TAG & " Monitor_single_item timeout!");
                end if;
            end if;

            -- If monitor was previously disabled, put it back into the same state!
            if (tmp) then
                monitor_ena <= false;
            end if;

        when CAN_AGNT_CMD_MONITOR_MONITOR_ALL_ITEMS =>    
            if (not monitor_ena) then
                monitor_ena <= true;
                tmp := true;
            end if;

            if (monitor_wp = monitor_rp) then
                warning(CAN_AGENT_TAG & "Monitor FIFO empty, no items will be driven");
            else
                wait until (monitor_wp = monitor_rp) for monitor_wait_timeout;
            end if;

            if (monitor_wp /= monitor_rp) then
                warning(CAN_AGENT_TAG & " Monitor_all_items timeout!");
            end if;

            -- If monitor was previously disabled, put it back into the same state!
            if (tmp) then
                monitor_ena <= false;
            end if;

        when CAN_AGNT_CMD_MONITOR_SET_TRIGGER =>
            tmp_int := pop(msg);
            monitor_trigger <= t_can_monitor_trigger'val(tmp_int);

        when CAN_AGNT_CMD_MONITOR_GET_TRIGGER =>
            push(ack_msg, t_can_monitor_trigger'pos(monitor_trigger));

        when CAN_AGNT_CMD_MONITOR_SET_SAMPLE_RATE =>
            monitor_sample_rate <= pop(msg);

        when CAN_AGNT_CMD_MONITOR_GET_SAMPLE_RATE =>
            push(ack_msg, monitor_sample_rate);

        when CAN_AGNT_CMD_MONITOR_CHECK_RESULT =>
            check(mon_mismatch_ctr = 0, CAN_AGENT_TAG & "Mismatches in monitor!");

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
                    driving_in_progress <= false;
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
            driving_in_progress <= false;
            wait until driver_ena;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Monitor process (FSM)
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    monitor_proc : process
        variable mon_count          : integer := 0;
    begin
        case monitor_state is
        when mon_disabled =>
            if (monitor_ena) then
                if (monitor_trigger = trig_immediately) then
                    monitor_state <= mon_running;
                else
                    monitor_state <= mon_waiting_for_trigger;
                end if;
                wait for 0 ns;
            else
                wait until monitor_ena;
            end if;

        when mon_waiting_for_trigger =>
            if (monitor_ena = false)then
                monitor_state <= mon_disabled;
                wait for 0 ns;
            else
                case monitor_trigger is
                when trig_immediately =>
                when trig_can_rx_rising_edge =>
                    wait until rising_edge(can_rx) or (monitor_ena = false);
                when trig_can_rx_falling_edge =>
                    wait until falling_edge(can_rx) or (monitor_ena = false);
                when trig_can_tx_rising_edge =>
                    wait until rising_edge(can_tx) or (monitor_ena = false);
                when trig_can_tx_falling_edge =>
                    wait until falling_edge(can_tx) or (monitor_ena = false);

                when trig_time_elapsed =>
                    wait until (monitor_ena = false) for monitor_trig_wait_time;

                -- Note: This triggers also when driver is started but its
                --       FIFO is empty so it does not drive anything!
                when trig_driver_start =>
                    wait until (driver_ena = true) or (monitor_ena = false);
                    
                when trig_driver_stop =>
                    wait until (driver_ena = false) or (monitor_ena = false);
                end case;

                if (monitor_ena = false) then
                    monitor_state <= mon_disabled;
                else
                    monitor_state <= mon_running;
                end if;
                wait for 0 ns;
            end if;

        when mon_running =>
            mon_mismatch_ctr <= 0;
            while (true) loop
                if (monitor_ena = false) then
                    monitor_state <= mon_disabled;
                    monitor_in_progress <= false;
                    wait for 0 ns;
                    exit;
                end if;

                if (monitor_wp = monitor_rp) then
                    
                    if (mon_mismatch_ctr = 0) then
                        monitor_state <= mon_passed;
                    else
                        monitor_state <= mon_failed;
                    end if;
                    monitor_in_progress <= false;
                    monitor_mismatch <= '0';
                    wait for 0 ns;
                    exit;
                end if;

                monitor_in_progress <= true;

                -- Read value from FIFO and start monitoring it!
                monitored_item <= monitor_mem(monitor_rp);
                wait for 0 ns;
                
                debug(CAN_AGENT_TAG & 
                      "Monitoring: " & std_logic'image(monitored_item.value) &
                      " for time: " & time'image(monitored_item.monitor_time));
                
                if (monitored_item.print_msg) then
                    info(monitored_item.msg);
                end if;
                
                mon_count := monitored_item.monitor_time / monitor_sample_rate;
                debug("Number of samples: " & integer'image(mon_count));
                
                -- So far do not support when sample rate is not multiple of monitor
                -- time! This would require extra handling of waiting time after last
                -- sample!
                if (mon_count * monitor_sample_rate /= monitored_item.monitor_time) then
                    error("Monitor time must be multiple of sample rate! " &
                          "Monitor time: " & time'image(monitored_item.monitor_time) &
                          " Sample rate: " & time'image(monitor_sample_rate));
                end if;
                
                -- Do not check on the last iteration because value might be changing
                -- there already (if sample rate is multiple of monitor time, then we
                -- are exactly at the end of monitor_time).
                for i in 1 to mon_count - 1 loop
                    wait for (monitor_sample_rate - 1 ps);
                    monitor_sample <= '1';
                    wait for 1 ps;
                    monitor_sample <= '0';
                    wait for 0 ns;

                    if (can_tx /= monitored_item.value) then
                        monitor_mismatch <= 'X';
                        mon_mismatch_ctr <= mon_mismatch_ctr + 1;

                        log(CAN_AGENT_TAG &
                              "Monitor mismatch! Expected: " &
                              std_logic'image(monitored_item.value) & 
                              " Monitored: " & std_logic'image(can_tx),
                            monitored_item.check_severity);
                    else
                        monitor_mismatch <= '0';
                    end if;
                end loop;
                
                -- We must wait one sample rate longer because we monitored only
                -- -1 samples!
                wait for monitor_sample_rate;

                monitor_rp <= (monitor_rp + 1) mod G_MONITOR_FIFO_DEPTH;
                wait for 0 ns;
            end loop;

        when mon_passed =>
            info ("*****************************************************");
            info (" Monitor sequence PASSED");
            info ("*****************************************************");
            monitor_state <= mon_disabled;
            wait for 0 ns;

        when mon_failed =>
            info ("*****************************************************");
            info (" Monitor sequence FAILED");
            info ("*****************************************************");
            monitor_state <= mon_disabled;
            wait for 0 ns;
            
        end case;
    end process;

end architecture;