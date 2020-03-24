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
--    Test controller agent. Controls CAN FD compliance TB over Vunit
--    communication library.
--
--    More on Vunit and its communication library can be found at:
--     https://vunit.github.io/documentation.html
--     https://vunit.github.io/com/user_guide.html
--  
--------------------------------------------------------------------------------
-- Revision History:
--    31.1.2020   Created file
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
use work.test_controller_agent_pkg.all;
use work.clk_gen_agent_pkg.all;
use work.rst_gen_agent_pkg.all;
use work.mem_bus_agent_pkg.all;
use work.can_agent_pkg.all;
use work.can_compliance_tb_pkg.all;

entity test_controller_agent is
    generic (
        -- This is Vunit runner config, don't name it runner_cfg so that
        -- Vunit will not scan it automatically as standalone test!
        cfg             : string;
        
        -- Test configuration
        cfg_clock_period   : time := 10 ns;
        cfg_brp            : natural := 4;  
        cfg_prop           : natural := 0;
        cfg_ph_1           : natural := 1;
        cfg_ph_2           : natural := 1;
        cfg_sjw            : natural := 2;
        cfg_brp_fd         : natural := 1;
        cfg_prop_fd        : natural := 3;
        cfg_ph_1_fd        : natural := 1;
        cfg_ph_2_fd        : natural := 2;
        cfg_sjw_fd         : natural := 2
    );
    port(
        -- VPI communication interface
        vpi_clk         : out   std_logic;
        vpi_req         : in    std_logic;
        vpi_ack         : out   std_logic := '0';
        vpi_cmd         : in    std_logic_vector(7 downto 0);
        vpi_dest        : in    std_logic_vector(7 downto 0);
        vpi_data_in     : in    std_logic_vector(63 downto 0);
        vpi_str_buf_in  : in    std_logic_vector(511 downto 0);
        vpi_data_out    : out   std_logic_vector(63 downto 0);
    
        -- VPI test control interface
        vpi_control_req     : out   std_logic := '0';
        vpi_control_gnt     : in    std_logic
    );
end entity;

architecture tb of test_controller_agent is

    signal test_end     : std_logic := '0';
    signal test_result  : std_logic := '0';

    ---------------------------------------------------------------------------
    -- 
    --
    -- @param net Network on which Reset agent listens (use "net").
    -- @param vpi_data_out VPI data output in testbench top.    
    ---------------------------------------------------------------------------
    procedure vpi_process_rst_agnt(
        signal      net             : inout network_t;
        signal      vpi_data_out    : out   std_logic_vector
    ) is
        variable data   :  std_logic;
    begin
        case vpi_cmd is
        when VPI_RST_AGNT_CMD_ASSERT =>
            rst_agent_assert(net);
        when VPI_RST_AGNT_CMD_DEASSERT =>
            rst_agent_deassert(net);
        when VPI_RST_AGNT_CMD_POLARITY_SET =>
            rst_agent_polarity_set(net, vpi_data_in(0));
        when VPI_RST_AGNT_CMD_POLARITY_GET =>
            rst_agent_polarity_get(net, data);
            vpi_data_out(0) <= data;
            wait for 0 ns;
        when others =>
            error("VPI: Unknown Reset agent command with code: 0x" & to_hstring(vpi_cmd));
        end case;
    end procedure;


    ---------------------------------------------------------------------------
    -- 
    --
    -- @param net Network on which Reset agent listens (use "net").
    -- @param vpi_data_out VPI data output in testbench top.    
    ---------------------------------------------------------------------------
    procedure vpi_process_clk_agent(
        signal net                  : inout network_t;
        signal vpi_data_out         : out   std_logic_vector
    ) is
        variable period : time;
        variable jitter : time;
        variable duty : integer range 0 to 100;
        variable vpi_data_out_i : std_logic_vector(63 downto 0);
    begin
        case vpi_cmd is
        when VPI_CLK_AGNT_CMD_START =>
            clk_gen_agent_start(net);
        when VPI_CLK_AGNT_CMD_STOP =>
            clk_gen_agent_stop(net);
        when VPI_CLK_AGNT_CMD_PERIOD_SET =>
            logic_vector_to_time(vpi_data_in, period);
            clk_agent_set_period(net, period);
        when VPI_CLK_AGNT_CMD_PERIOD_GET =>
            clk_agent_get_period(net, period);
            time_to_logic_vector(period, vpi_data_out_i);
            vpi_data_out <= vpi_data_out_i;
        when VPI_CLK_AGNT_CMD_JITTER_SET =>
            logic_vector_to_time(vpi_data_in, jitter);
            clk_agent_set_jitter(net, jitter);
        when VPI_CLK_AGNT_CMD_JITTER_GET =>
            clk_agent_get_jitter(net, jitter);
            time_to_logic_vector(jitter, vpi_data_out_i);
            vpi_data_out <= vpi_data_out_i;
        when VPI_CLK_AGNT_CMD_DUTY_SET =>
            duty := to_integer(unsigned(vpi_data_in));
            clk_agent_set_duty(net, duty);
        when VPI_CLK_AGNT_CMD_DUTY_GET =>
            clk_agent_get_duty(net, duty);
            vpi_data_out <= std_logic_vector(to_unsigned(duty, 64));
        when others =>
            error("VPI: Unknown Clock generator agent command with code: 0x" & to_hstring(vpi_cmd));
        end case;
    end procedure;


    ---------------------------------------------------------------------------
    -- 
    --
    -- @param net Network on which Reset agent listens (use "net").
    -- @param vpi_data_out VPI data output in testbench top.    
    ---------------------------------------------------------------------------
    procedure vpi_process_mem_bus_agent(
        signal net                  : inout network_t;
        signal vpi_data_out         : out   std_logic_vector
    ) is
        variable address    :  integer;
        variable data_8     :  std_logic_vector(7 downto 0);
        variable data_16    :  std_logic_vector(15 downto 0);
        variable data_32    :  std_logic_vector(31 downto 0);
        variable blocking   :  boolean;
        variable setup          :  time;
        variable hold           :  time;
        variable period         :  time;
        variable output_delay   :  time;
    begin
        case vpi_cmd is
        when VPI_MEM_BUS_AGNT_START =>
            mem_bus_agent_start(net);
        when VPI_MEM_BUS_AGNT_STOP =>
            mem_bus_agent_stop(net);
        when VPI_MEM_BUS_AGNT_WRITE =>
            data_8 := vpi_data_in(7 downto 0);
            data_16 := vpi_data_in(15 downto 0);
            data_32 := vpi_data_in(31 downto 0);
            address := to_integer(unsigned(vpi_data_in(47 downto 32)));
            
            -- Blocking tag enconded in bit 50
            if (vpi_data_in(50) = '1') then
                blocking := true;
            else
                blocking := false;
            end if;

            -- Access size encoded in these two bits!
            if (vpi_data_in(49 downto 48) = "00") then
                mem_bus_agent_write(net, address, data_8, blocking);
            elsif (vpi_data_in(49 downto 48) = "01") then
                mem_bus_agent_write(net, address, data_16, blocking);
            elsif (vpi_data_in(49 downto 48) = "10") then                
                mem_bus_agent_write(net, address, data_32, blocking);
            else
                error("VPI: Invalid memory bus agent write access size: " &
                        to_hstring(vpi_data_in(49 downto 48)));
            end if;

        when VPI_MEM_BUS_AGNT_READ =>
            address := to_integer(unsigned(vpi_data_in(47 downto 32)));

            vpi_data_out(vpi_data_out'length - 1 downto 0) <= (OTHERS => '0');
            if (vpi_data_in(49 downto 48) = "00") then
                mem_bus_agent_read(net, address, data_8);
                vpi_data_out(7 downto 0) <= data_8;
            elsif (vpi_data_in(49 downto 48) = "01") then
                mem_bus_agent_read(net, address, data_16);
                vpi_data_out(15 downto 0) <= data_16;
            elsif (vpi_data_in(49 downto 48) = "10") then
                mem_bus_agent_read(net, address, data_32);
                vpi_data_out(31 downto 0) <= data_32;
            else
                error("VPI: Invalid memory bus agent read access size: " &
                        to_hstring(vpi_data_in(49 downto 48)));
            end if;

        when VPI_MEM_BUS_AGNT_X_MODE_START =>
            mem_bus_agent_x_mode_start(net);

        when VPI_MEM_BUS_AGNT_X_MODE_STOP =>
            mem_bus_agent_x_mode_start(net);

        when VPI_MEM_BUS_AGNT_SET_X_MODE_SETUP =>
            logic_vector_to_time(vpi_data_in, setup);
            mem_bus_agent_set_x_mode_setup(net, setup);

        when VPI_MEM_BUS_AGNT_SET_X_MODE_HOLD =>
            logic_vector_to_time(vpi_data_in, hold);
            mem_bus_agent_set_x_mode_hold(net, hold);

        when VPI_MEM_BUS_AGNT_SET_PERIOD =>
            logic_vector_to_time(vpi_data_in, period);
            mem_bus_agent_set_period(net, period);

        when VPI_MEM_BUS_AGNT_SET_OUTPUT_DELAY =>
            logic_vector_to_time(vpi_data_in, output_delay);
            mem_bus_agent_set_output_delay(net, output_delay);

        when VPI_MEM_BUS_AGNT_WAIT_DONE =>
            mem_bus_agent_wait_done(net);

        when others =>
            error("VPI: Unknown Memory bus agent command with code: 0x" & to_hstring(vpi_cmd));
        end case;
    end procedure;


    ---------------------------------------------------------------------------
    -- 
    --
    -- @param net Network on which Reset agent listens (use "net").
    -- @param vpi_data_out VPI data output in testbench top.    
    ---------------------------------------------------------------------------
    procedure vpi_process_can_agent(
        signal net                  : inout network_t;
        signal vpi_data_out         : out   std_logic_vector
    ) is
        variable progress       : boolean;
        variable driven_val     : std_logic;
        variable monitored_val  : std_logic;
        variable driver_item    : t_can_driver_entry;
        variable monitor_item   : t_can_monitor_entry;
        variable timeout        : time;
        variable monitor_state  : t_can_monitor_state;
        variable trigger        : t_can_monitor_trigger;
        variable result         : std_logic;
        variable sample_rate    : time;
        variable vpi_data_out_i : std_logic_vector(63 downto 0);
    begin
        case vpi_cmd is
            
        when VPI_CAN_AGNT_DRIVER_START =>
            can_agent_driver_start(net);

        when VPI_CAN_AGNT_DRIVER_STOP =>
            can_agent_driver_stop(net);

        when VPI_CAN_AGNT_DRIVER_FLUSH =>
            can_agent_driver_flush(net);

        when VPI_CAN_AGNT_DRIVER_GET_PROGRESS =>
            can_agent_driver_get_progress(net, progress);
            if (progress) then
                vpi_data_out(0) <= '1';
            else
                vpi_data_out(0) <= '0';
            end if;

        when VPI_CAN_AGNT_DRIVER_GET_DRIVEN_VAL =>
            can_agent_driver_get_driven_val(net, driven_val);
            vpi_data_out(0) <= driven_val;

        when VPI_CAN_AGNT_DRIVER_PUSH_ITEM =>
            
            -- Time conversion from 64 bits truly uses only 62 bits, stuff
            -- remaining information for driven item into remaining two
            -- bits so that we don't need to declare next signal via VPI.
            driver_item.value := vpi_data_in(63);
            if (vpi_data_in(62) = '1') then
                driver_item.print_msg := true;
                logic_vector_to_str(vpi_str_buf_in, driver_item.msg);
            else
                driver_item.print_msg := false;
            end if;
            logic_vector_to_time(vpi_data_in, driver_item.drive_time);
            
            can_agent_driver_push_item(net, driver_item);

        when VPI_CAN_AGNT_DRIVER_SET_WAIT_TIMEOUT =>
            logic_vector_to_time(vpi_data_in, timeout);
            can_agent_driver_set_wait_timeout(net, timeout);

        when VPI_CAN_AGNT_DRIVER_WAIT_FINISH =>
            can_agent_driver_wait_finish(net);
            
        when VPI_CAN_AGNT_DRIVER_DRIVE_SINGLE_ITEM =>
            
            -- Time conversion from 64 bits truly uses only 62 bits, stuff
            -- remaining information for driven item into remaining two
            -- bits so that we don't need to declare next signal via VPI.
            driver_item.value := vpi_data_in(63);
            if (vpi_data_in(62) = '1') then
                driver_item.print_msg := true;
                logic_vector_to_str(vpi_str_buf_in, driver_item.msg);
            else
                driver_item.print_msg := false;
            end if;
            logic_vector_to_time(vpi_data_in, driver_item.drive_time);
            can_agent_driver_drive_single_item(net, driver_item);

        when VPI_CAN_AGNT_DRIVER_DRIVE_ALL_ITEM =>
            can_agent_driver_drive_all_items(net);

        when VPI_CAN_AGNT_MONITOR_START =>
            can_agent_monitor_start(net);

        when VPI_CAN_AGNT_MONITOR_STOP =>
            can_agent_monitor_stop(net);

        when VPI_CAN_AGNT_MONITOR_FLUSH =>
            can_agent_monitor_flush(net);

        when VPI_CAN_AGNT_MONITOR_GET_STATE =>
            can_agent_monitor_get_state(net, monitor_state);
            case monitor_state is
            when mon_disabled =>
                vpi_data_out(2 downto 0) <= "000";
            when mon_waiting_for_trigger =>
                vpi_data_out(2 downto 0) <= "001";
            when mon_running =>
                vpi_data_out(2 downto 0) <= "010";
            when mon_passed =>
                vpi_data_out(2 downto 0) <= "011";
            when mon_failed =>
                vpi_data_out(2 downto 0) <= "100";
            end case;

        when VPI_CAN_AGNT_MONITOR_GET_MONITORED_VAL =>
            can_agent_monitor_get_monitored_val(net, monitored_val);
            vpi_data_out(0) <= monitored_val;

        when VPI_CAN_AGNT_MONITOR_PUSH_ITEM =>
            monitor_item.value := vpi_data_in(63);
            if (vpi_data_in(62) = '1') then
                monitor_item.print_msg := true;
                logic_vector_to_str(vpi_str_buf_in, monitor_item.msg);
            else
                monitor_item.print_msg := false;
            end if;

            -- Time conversion from 64 bits truly uses only 62 bits, stuff
            -- remaining information for driven item into remaining two
            -- bits so that we don't need to declare next signal via VPI.
            logic_vector_to_time(vpi_data_in, monitor_item.monitor_time);
            
            can_agent_monitor_push_item(net, monitor_item);

        when VPI_CAN_AGNT_MONITOR_SET_WAIT_TIMEOUT =>
            logic_vector_to_time(vpi_data_in, timeout);
            can_agent_monitor_set_wait_timeout(net, timeout);
            
        when VPI_CAN_AGNT_MONITOR_WAIT_FINISH =>
            can_agent_monitor_wait_finish(net);

        when VPI_CAN_AGNT_MONITOR_MONITOR_SINGLE_ITEM =>
            monitor_item.value := vpi_data_in(63);
            if (vpi_data_in(62) = '1') then
                monitor_item.print_msg := true;
                logic_vector_to_str(vpi_str_buf_in, monitor_item.msg);
            else
                monitor_item.print_msg := false;
            end if;

            -- Time conversion from 64 bits truly uses only 62 bits, stuff
            -- remaining information for driven item into remaining two
            -- bits so that we don't need to declare next signal via VPI.
            logic_vector_to_time(vpi_data_in, monitor_item.monitor_time);
            
            can_agent_monitor_single_item(net, monitor_item);

        when VPI_CAN_AGNT_MONITOR_MONITOR_ALL_ITEMS =>
            can_agent_monitor_all_items(net);
    
        when VPI_CAN_AGNT_MONITOR_SET_TRIGGER =>
            case vpi_data_in(2 downto 0) is
            when "000" =>
                trigger := trig_immediately;
            when "001" =>
                trigger := trig_can_rx_rising_edge;
            when "010" =>
                trigger := trig_can_rx_falling_edge;
            when "011" =>
                trigger := trig_can_tx_rising_edge;
            when "100" =>
                trigger := trig_can_tx_falling_edge;
            when "101" =>
                trigger := trig_time_elapsed;
            when "110" =>
                trigger := trig_driver_start;
            when "111" =>
                trigger := trig_driver_stop;
            when others =>
                error("Invalid monitor trigger type: " &
                        to_hstring(vpi_data_in(2 downto 0)));
            end case;
            can_agent_monitor_set_trigger(net, trigger);

        when VPI_CAN_AGNT_MONITOR_GET_TRIGGER =>
            can_agent_monitor_get_trigger(net, trigger);
            case trigger is
            when trig_immediately =>
                vpi_data_out(2 downto 0) <= "000";
            when trig_can_rx_rising_edge =>
                vpi_data_out(2 downto 0) <= "001";
            when trig_can_rx_falling_edge =>
                vpi_data_out(2 downto 0) <= "010";
            when trig_can_tx_rising_edge =>
                vpi_data_out(2 downto 0) <= "011";
            when trig_can_tx_falling_edge =>
                vpi_data_out(2 downto 0) <= "100";
            when trig_time_elapsed =>
                vpi_data_out(2 downto 0) <= "101";
            when trig_driver_start =>
                vpi_data_out(2 downto 0) <= "110";
            when trig_driver_stop =>
                vpi_data_out(2 downto 0) <= "111";
            end case;

        when VPI_CAN_AGNT_MONITOR_SET_SAMPLE_RATE =>
            logic_vector_to_time(vpi_data_in, sample_rate);
            can_agent_monitor_set_sample_rate(net, sample_rate);

        when VPI_CAN_AGNT_MONITOR_GET_SAMPLE_RATE =>
            can_agent_monitor_get_sample_rate(net, sample_rate);
            time_to_logic_vector(sample_rate, vpi_data_out_i);
            vpi_data_out <= vpi_data_out_i;

        when VPI_CAN_AGNT_MONITOR_CHECK_RESULT =>
            can_agent_monitor_check_result(net);

        when others =>
            error("VPI: Unknown CAN agent command with code: 0x" & to_hstring(vpi_cmd));
        end case;
    end procedure;


    ---------------------------------------------------------------------------
    -- 
    --
    -- @param net Network on which Reset agent listens (use "net").
    -- @param vpi_data_out VPI data output in testbench top.
    -- @param test_end Signal to signalize that test has ended.
    -- @param test_result Signal to signalize test result (1-passed, 0-failed).
    ---------------------------------------------------------------------------
    procedure vpi_process_test_agent(
        signal net                  : inout network_t;
        signal vpi_data_out         : out   std_logic_vector;
        signal test_end             : out   std_logic;
        signal test_result          : out   std_logic
    ) is
        variable param_name         : string(1 to 64);
        variable vpi_data_out_i     : std_logic_vector(63 downto 0) := (OTHERS => '0');
    begin
        case vpi_cmd is
        when VPI_TEST_AGNT_TEST_END =>
            test_end <= '1';
            test_result <= vpi_data_in(0);

        when VPI_TEST_AGNT_GET_CFG =>
            -- VPI message string encodes desired configuration parameter
            logic_vector_to_str(vpi_str_buf_in, param_name);
            vpi_data_out(vpi_data_out'length - 1 downto 0) <= (OTHERS => '0');
            
            info("SW test queries parameter: " & param_name(1 to 64) );

            if (param_name(45 to 64) = "CFG_DUT_CLOCK_PERIOD") then
               time_to_logic_vector(cfg_clock_period, vpi_data_out_i);
               vpi_data_out <= vpi_data_out_i;

            elsif (param_name(54 to 64) = "CFG_DUT_BRP") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_brp, vpi_data_out'length));

            elsif (param_name(53 to 64) = "CFG_DUT_PROP") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_prop, vpi_data_out'length));

            elsif (param_name(54 to 64) = "CFG_DUT_PH1") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_ph_1, vpi_data_out'length));

            elsif (param_name(54 to 64) = "CFG_DUT_PH2") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_ph_2, vpi_data_out'length));

            elsif (param_name(54 to 64) = "CFG_DUT_SJW") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_sjw, vpi_data_out'length));

            elsif (param_name(51 to 64) = "CFG_DUT_BRP_FD") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_brp_fd, vpi_data_out'length));

            elsif (param_name(50 to 64) = "CFG_DUT_PROP_FD") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_prop_fd, vpi_data_out'length));

            elsif (param_name(51 to 64) = "CFG_DUT_PH1_FD") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_ph_1_fd, vpi_data_out'length));

            elsif (param_name(51 to 64) = "CFG_DUT_PH2_FD") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_ph_2_fd, vpi_data_out'length));

            elsif (param_name(51 to 64) = "CFG_DUT_SJW_FD") then
               vpi_data_out <= std_logic_vector(to_unsigned(cfg_sjw_fd, vpi_data_out'length));

            else
               error("Unsupported configuration parameter name: " & param_name);
            end if;

        when others =>
            error("VPI: Unknown test agent command with code: 0x" & to_hstring(vpi_cmd));
        end case;
    end procedure;

begin
    
    ---------------------------------------------------------------------------
    -- Main test process
    ---------------------------------------------------------------------------
    main_test_proc : process
    begin
        test_runner_setup(runner, cfg);

        wait for 10 ns;

        -----------------------------------------------------------------------
        -- Relinquish control to SW part of TB. SW part of TB controls
        -- everything, reset, clock, memory access, CAN agent (driver, monitor)
        -----------------------------------------------------------------------
        info("Requesting TB control from SW");
        vpi_control_req <= '1';
        wait for 1 ns;

        if (vpi_control_gnt /= '1') then
            wait until vpi_control_gnt <= '1' for 10 ns;
        end if;

        wait for 0 ns;
        check(vpi_control_gnt = '1', "SW part of TB took over simulation control!");
        wait for 0 ns;

        info("Waiting till SW part of test ends!");
        wait until (test_end = '1');
        info("SW part of TB signals test has ended");

        wait for 100 ns;
        if (test_result = '1') then
            test_runner_cleanup(runner, false);
        else
            test_runner_cleanup(runner, true);
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Generate VPI clock.
    -- Callbacks are executed on this clock and requests from SW are colllected
    -- by these callbacks!
    ---------------------------------------------------------------------------
    vpi_clk_gen_proc : process
    begin
        vpi_clk <= '1';
        wait for 1 ns;
        vpi_clk <= '0';
        wait for 1 ns;
    end process;

    ---------------------------------------------------------------------------
    -- Listen on VPI commands and send them to individual agents!
    ---------------------------------------------------------------------------
    vpi_listener_process : process
    begin
        
        -----------------------------------------------------------------------
        -- Poll on for vpi_req = '1'
        -----------------------------------------------------------------------
        wait until (vpi_req = '1');
        wait for 1 ps;
        
        -----------------------------------------------------------------------
        -- Process command (and get answer in case of read)
        -----------------------------------------------------------------------
        case vpi_dest is
        when VPI_DEST_RES_GEN_AGENT =>
            vpi_process_rst_agnt(net, vpi_data_out);
        when VPI_DEST_CLK_GEN_AGENT =>
            vpi_process_clk_agent(net, vpi_data_out);
        when VPI_DEST_MEM_BUS_AGENT =>
            vpi_process_mem_bus_agent(net, vpi_data_out);
        when VPI_DEST_CAN_AGENT =>
            vpi_process_can_agent(net, vpi_data_out);
        when VPI_DEST_TEST_CONTROLLER_AGENT =>
            vpi_process_test_agent(net, vpi_data_out, test_end, test_result);
        when OTHERS =>
            error("Unknown VPI destination: " & to_hstring(vpi_dest));
        end case;

        wait for 1 ps;

        -----------------------------------------------------------------------
        -- Issue vpi_ack = '1'
        -----------------------------------------------------------------------
        vpi_ack <= '1';
        wait for 1 ps;

        -----------------------------------------------------------------------
        -- Poll on for vpi_reg = '0'
        -----------------------------------------------------------------------
        wait until (vpi_req = '0');
        wait for 1 ps;
        vpi_ack <= '0';
        wait for 1 ps;

    end process;

end architecture;