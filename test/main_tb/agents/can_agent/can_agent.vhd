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
--    CAN Bus agent. Configurable over Vunit Communication library.
--
--    More on Vunit and its communication library can be found at:
---     https://vunit.github.io/documentation.html
--      https://vunit.github.io/com/user_guide.html
--
--    CAN Bus agent is connected to can_tx and can_rx signals of DUT. It drives
--    can_rx via driver and it monitors_can_tx via monitor.
--
--    Driver contains FIFO of items to be driven. Each item contains value and
--    a time for which to drive the value. When driving of 1 item finishes it
--    is immediately followed by next one until driver FIFO is empty. Driver
--    can be started by a command over Communication library (this is intended
--    for sending frame by CAN agent), or simultaneously with Monitor (this is
--    intended for receiving frame by CAN Agent from DUT).
--
--    Monitor contains a FIFO of items to be monitored. Each item contains
--    value, time and sample period for which to monitor the value. When
--    Monitoring of 1 item finishes, it is immediately followed by next one
--    until monitor FIFO is empty. Monitor does not check for the value
--    constantly, but it samples can_tx output with configurable sampling
--    period. If monitored value is not equal to expected value when it is
--    sampled, internal mismatch counter is incremented and warning is printed.
--
--    Monitor can be in one of five states: "Idle", "Waiting for Trigger",
--    "Running", "Passed" and "Failed". When it is "Idle", no monitoring is
--    progress. When it is started via communication library, it moves to
--    "Waiting for Trigger". In this state it waits until trigger event occurs
--    (Type of trigger event is also configurable over Communication library)
--    and moves to "Running". When it is running, it monitors items from Monitor
--    FIFO one after another. After all items were monitored, monitor transfers to
--    "Passed" when no mismatch between can_tx and monitored values occured
--    during monitoring. Otherwise it transfers to failed.
--
--    Trigger functionality of monitor is a way how to synchronize its operation
--    with driver. Monitor can be configured to start at the same time as
--    driver starts.
--
--    When Monitor operation starts its internal error counter is erased. When
--    its ends, succesfull monitoring (no errors) can be checked via
--    Communication library.
--
-------------------------------------------------------------------------------
--    MONITORING of an ITEM (examples):
--
--    Lets suppose that monitor is about to monitor logic 1 for 100 ns with
--    sample period of 20 ns. Following figure shows where can_tx will be
--    checked:
--
--               _______________________________________________________
--  can_tx  _____|                                                      |____
--
--  Check:                 O           O           O           O
--               |         |           |           |           |        |
--              0ns       20ns        40ns        60 ns       80ns     100ns
--
--  Note that monitor will NOT execute check of can_tx less than sample_time
--  after start of monitoring item, and less than sample time before the end of
--  monitoring of an item! This behaviour is intentional!
--
--  The intention is, to set sample time to value of Time Quanta. This allows
--  checking with granularity of time quanta.
--
--  When DUT is configured e.g with BRP = 4, and Driver is not synchronized
--  with DUT, then shift between Driver and DUT can up to 1 TQ! Therefore, the
--  value of edge coming from DUT can be floating in range between 0 ns and
--  1 TQ! (DUT does not operate on lower granularity than time quanta).
--
-------------------------------------------------------------------------------
--    API for work with CAN agent is implemented in "can_agent_pkg" package. 
--
--------------------------------------------------------------------------------
-- Revision History:
--    25.1.2020   Created file
--    04.2.2021   Adjusted to work without Vunits COM library.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.can_agent_pkg.all;


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
    signal driver_wait_for_monitor  :   boolean := false;

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
    signal driver_wait_timeout      :   time := 3 ms;
    signal driven_item              :   t_can_driver_entry := (
        value           => 'Z',
        drive_time      => 0 ns,
        print_msg       => false,
        msg             => C_EMPTY_STRING
    );
    
    -- Monitor
    signal monitor_wait_timeout     :   time := 3 ms;
    signal monitored_item           :   t_can_monitor_entry := (
        value           => 'Z',
        monitor_time    => 0 ns,
        sample_rate     => 0 ns,
        print_msg       => false,
        msg             => C_EMPTY_STRING
    );

    signal monitor_trigger          :   t_can_monitor_trigger := trig_immediately;
    signal monitor_trig_wait_time   :   time := 100 ns;
    
    -- Debug signal only, shows where can_rx is sampled!
    signal monitor_sample           :   std_logic := '0';

    -- Debug signal only, shows mismatch (by an X)
    signal monitor_mismatch         :   std_logic := '0';

    -- Mismatch counter
    signal mon_mismatch_ctr         :   integer := 0;

    -- Input delay
    signal mon_input_delay          :   time := 0 ns;

    -- can_tx to can_rx feedback
    signal tx_to_rx_feedback_enable :   boolean := false;

    -- Internal value of can_rx
    signal can_rx_i                 :   std_logic := '1';

begin

    ---------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable cmd            : integer;
        variable reply_code     : integer;
        variable push_item      : t_can_driver_entry;
        variable push_mon_item  : t_can_monitor_entry;
        variable tmp            : boolean := false;
        variable tmp_int        : integer := 0;

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
        receive_start(default_channel, C_CAN_AGENT_ID);
        
        -- Command is sent as message type
        cmd := com_channel_data.get_msg_code;
        reply_code := C_REPLY_CODE_OK;

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
            com_channel_data.set_param(driving_in_progress);

        when CAN_AGNT_CMD_DRIVER_GET_DRIVEN_VAL =>
            com_channel_data.set_param(can_rx);

        when CAN_AGNT_CMD_DRIVER_PUSH_ITEM =>
            push_item.value := com_channel_data.get_param;
            push_item.drive_time := com_channel_data.get_param;
            push_item.print_msg := com_channel_data.get_param;
            if (push_item.print_msg) then
                push_item.msg := com_channel_data.get_param;
            else
                push_item.msg := (OTHERS => ' ');
            end if;

            -- Check for overflow, keep one item empty so that we don't have to
            -- handle pointers
            if ((driver_wp + 1) mod G_DRIVER_FIFO_DEPTH = driver_rp) then
                reply_code := C_REPLY_CODE_ERR;
                error_m(CAN_AGENT_TAG & "Driver FIFO overflow! -> Skipping");
            else
                driver_fifo_push;             
            end if;

        when CAN_AGNT_CMD_DRIVER_SET_WAIT_TIMEOUT =>
            driver_wait_timeout <= com_channel_data.get_param;

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

            push_item.value := com_channel_data.get_param;
            push_item.drive_time := com_channel_data.get_param;
            push_item.print_msg := com_channel_data.get_param;
            if (push_item.print_msg) then
                push_item.msg := com_channel_data.get_param;
            else
                push_item.msg := (OTHERS => ' ');
            end if;

            if (driver_wp /= driver_rp) then
                warning_m(CAN_AGENT_TAG &
                        "Driver FIFO not empty, other items will be driven first...");
            end if;

            if ((driver_wp + 1) mod G_DRIVER_FIFO_DEPTH = driver_rp) then
                reply_code := C_REPLY_CODE_ERR;
                error_m(CAN_AGENT_TAG & "Driver FIFO overflow -> Skipping");
            else
                driver_fifo_push;
                wait until (driver_wp = driver_rp) for driver_wait_timeout;

                if (driver_wp /= driver_rp) then
                    warning_m(CAN_AGENT_TAG & " Drive_single_item timeout!");
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
                warning_m(CAN_AGENT_TAG & "Driver FIFO empty, no items will be driven");
            else
                wait until (driver_wp = driver_rp) for driver_wait_timeout;
            end if;

            if (driver_wp /= driver_rp) then
                warning_m(CAN_AGENT_TAG & " Drive_all_items timeout!");
            end if;

            -- If driver was previously disabled, put it back into the same state!
            if (tmp) then
                driver_ena <= false;
            end if;

        when CAN_AGNT_CMD_SET_WAIT_FOR_MONITOR =>
            driver_wait_for_monitor <= com_channel_data.get_param;

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
            com_channel_data.set_param(t_can_monitor_state'pos(monitor_state));

        when CAN_AGNT_CMD_MONITOR_GET_MONITORED_VAL =>
            com_channel_data.set_param(monitored_item.value);

        when CAN_AGNT_CMD_MONITOR_PUSH_ITEM =>
            push_mon_item.value := com_channel_data.get_param;
            push_mon_item.monitor_time := com_channel_data.get_param;
            push_mon_item.print_msg := com_channel_data.get_param;
            if (push_mon_item.print_msg) then
                push_mon_item.msg := com_channel_data.get_param;
            else
                push_mon_item.msg := (OTHERS => ' ');
            end if;
            push_mon_item.sample_rate := com_channel_data.get_param2;
            
            monitor_fifo_push;

        when CAN_AGNT_CMD_MONITOR_SET_WAIT_TIMEOUT =>
            monitor_wait_timeout <= com_channel_data.get_param;

        when CAN_AGNT_CMD_MONITOR_WAIT_FINISH =>
            wait for 0 ns;
            wait until (monitor_in_progress = false) for monitor_wait_timeout;

        when CAN_AGNT_CMD_MONITOR_MONITOR_SINGLE_ITEM =>
            if (not monitor_ena) then
                monitor_ena <= true;
                tmp := true;
            end if;
            
            push_mon_item.value := com_channel_data.get_param;
            push_mon_item.monitor_time := com_channel_data.get_param;
            push_mon_item.print_msg := com_channel_data.get_param;
            if (push_mon_item.print_msg) then
                push_mon_item.msg := com_channel_data.get_param;
            else
                push_mon_item.msg := (OTHERS => ' ');
            end if;
            push_mon_item.sample_rate := com_channel_data.get_param2;

            if (monitor_wp /= monitor_rp) then
                warning_m(CAN_AGENT_TAG &
                        "Monitor FIFO not empty, other items will be monitored first...");
            end if;
            
            if ((monitor_wp + 1) mod G_MONITOR_FIFO_DEPTH = monitor_rp) then
                reply_code := C_REPLY_CODE_ERR;
                error_m(CAN_AGENT_TAG & "Monitor FIFO overflow -> Skipping");
            else
                monitor_fifo_push;
                wait until (monitor_wp = monitor_rp) for monitor_wait_timeout;

                if (monitor_wp /= monitor_rp) then
                    warning_m(CAN_AGENT_TAG & " Monitor_single_item timeout!");
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
                warning_m(CAN_AGENT_TAG & "Monitor FIFO empty, no items will be driven");
            else
                wait until (monitor_wp = monitor_rp) for monitor_wait_timeout;
            end if;

            if (monitor_wp /= monitor_rp) then
                warning_m(CAN_AGENT_TAG & " Monitor_all_items timeout!");
            end if;

            -- If monitor was previously disabled, put it back into the same state!
            if (tmp) then
                monitor_ena <= false;
            end if;

        when CAN_AGNT_CMD_MONITOR_SET_TRIGGER =>
            tmp_int := com_channel_data.get_param;
            monitor_trigger <= t_can_monitor_trigger'val(tmp_int);

        when CAN_AGNT_CMD_MONITOR_GET_TRIGGER =>
            com_channel_data.set_param(t_can_monitor_trigger'pos(monitor_trigger));

        when CAN_AGNT_CMD_MONITOR_CHECK_RESULT =>
            check_m(mon_mismatch_ctr = 0, CAN_AGENT_TAG & "Mismatches in monitor!");

        when CAN_AGNT_CMD_MONITOR_SET_INPUT_DELAY =>
            mon_input_delay <= com_channel_data.get_param;

        when CAN_AGNT_CMD_TX_RX_FEEDBACK_ENABLE =>
            tx_to_rx_feedback_enable <= true;
            wait for 0 ns;
        
        when CAN_AGNT_CMD_TX_RX_FEEDBACK_DISABLE =>
            tx_to_rx_feedback_enable <= false;
            wait for 0 ns;

        when others =>
            warning_m(CAN_AGENT_TAG & "Invalid message type: " & integer'image(cmd));
            reply_code := C_REPLY_CODE_ERR;
        end case;

        receive_finish(default_channel, reply_code);
    end process;
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Driver process (reading from Driver FIFO)
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    driver_proc : process
    begin
        if (driver_ena) then
            if (driver_wait_for_monitor = true) then
                wait until (monitor_state = mon_running);
            end if;
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

                    --debug_m(CAN_AGENT_TAG &
                    --      "Driving: " & std_logic'image(driven_item.value) &
                    --      " for time: " & time'image(driven_item.drive_time));
                    --if (driven_item.print_msg) then
                    --    info_m("Driving item: " & driven_item.msg);
                    --end if;

                    can_rx_i <= driven_item.value;
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
        variable monitored_time     : time := 0 fs;
        
        -----------------------------------------------------------------------
        -- Comparison procedure for monitor. Simple "=" operator is not enough
        -- since GHDL might improprely handle don't care values. Also, compare
        -- is not symmetrical (can_tx is now allowed to have "don't care"
        -----------------------------------------------------------------------
        impure function monitor_compare return boolean is
        begin
            if (can_tx = 'X' or
                can_tx = 'U' or
                can_tx = '-' or
                can_tx = 'W' or
                can_tx = 'Z')
            then
                return false;
            end if;
            
            if (monitored_item.value = '-' and (can_tx = '1' or can_rx = '0'))
            then
                return true;
            end if;

            if (can_tx = '0' and monitored_item.value = '0') then
                return true;
            elsif (can_tx = '1' and monitored_item.value = '1') then
                return true;
            end if;
            
            return false;
        end function;
        
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
                    -- Wait additional monitor delay
                    wait for mon_input_delay;                    

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
                
                --debug_m(CAN_AGENT_TAG & 
                --      "Monitoring: " & std_logic'image(monitored_item.value) &
                --      " for time: " & time'image(monitored_item.monitor_time));
                
                --if (monitored_item.print_msg) then
                --    info_m("Monitoring item: " & monitored_item.msg);
                --end if;
                
                mon_count := monitored_item.monitor_time / monitored_item.sample_rate;
                monitored_time := 0 ns;
                debug_m("Number of samples: " & integer'image(mon_count));

                ------------------------------------------------------------------------
                -- Monitor until we are not closer than sample rate from the end. This
                -- will truncate the last sample so that we dont compare too close to
                -- the end!
                ------------------------------------------------------------------------
                while (monitored_time < monitored_item.monitor_time - monitored_item.sample_rate) loop
                    wait for (monitored_item.sample_rate - 1 ps);
                    monitor_sample <= '1';
                    wait for 1 ps;
                    monitor_sample <= '0';
                    wait for 0 ns;

                    if (monitor_compare = false) then
                        monitor_mismatch <= 'X';
                        mon_mismatch_ctr <= mon_mismatch_ctr + 1;

                        warning_m(CAN_AGENT_TAG &
                                "Monitor mismatch! Expected: " &
                                std_logic'image(monitored_item.value) & 
                                " Monitored: " & std_logic'image(can_tx));
                    else
                        monitor_mismatch <= '0';
                    end if;
                    monitored_time := monitored_time + monitored_item.sample_rate;
                end loop;

                -- We must wait the rest so that monitor item lasts proper time!
                wait for monitored_item.monitor_time - monitored_time;

                monitor_mismatch <= '0';

                monitor_rp <= (monitor_rp + 1) mod G_MONITOR_FIFO_DEPTH;
                wait for 0 ns;
            end loop;

        when mon_passed =>
            info_m("*****************************************************");
            info_m (" Monitor sequence PASSED");
            info_m ("*****************************************************");
            monitor_state <= mon_disabled;
            wait for 0 ns;

        when mon_failed =>
            info_m ("*****************************************************");
            info_m (" Monitor sequence FAILED");
            info_m ("*****************************************************");
            monitor_state <= mon_disabled;
            wait for 0 ns;
            
        end case;
    end process;

    -----------------------------------------------------------------------------
    -- CAN tx to CAN rx feedback. This corresponds to CAN bus realization!
    -----------------------------------------------------------------------------
    can_rx <= can_rx_i when (tx_to_rx_feedback_enable = false) else
              can_rx_i and can_tx;

end architecture;