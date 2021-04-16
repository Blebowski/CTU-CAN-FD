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
--    Test controller agent - Controls rest of CTU CAN FD VIP  
--
--------------------------------------------------------------------------------
-- Revision History:
--    12.3.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.test_controller_agent_pkg.all;
use ctu_can_fd_tb.timestamp_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;
use ctu_can_fd_tb.interrupt_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;
use ctu_can_fd_tb.test_probe_agent_pkg.all;
use ctu_can_fd_tb.reset_agent_pkg.all;
use ctu_can_fd_tb.can_agent_pkg.all;


entity test_controller_agent is
    generic(
        -- Test configuration
        test_name               : string;
        test_type               : string;
        stand_alone_vip_mode    : boolean;
        seed                    : natural;
        
        -- DUT configuration
        -- DUT Clock period
        cfg_sys_clk_period      : string;

        -- Bit timing cofnig used in; compliance tests
        cfg_brp                 : natural;
        cfg_prop                : natural;
        cfg_ph_1                : natural;
        cfg_ph_2                : natural;
        cfg_sjw                 : natural;
        cfg_brp_fd              : natural;
        cfg_prop_fd             : natural;
        cfg_ph_1_fd             : natural;
        cfg_ph_2_fd             : natural;
        cfg_sjw_fd              : natural
    );
    port (
        -- VIP test control / status signals
        test_start              : in  std_logic;
        test_done               : out std_logic := '0';
        test_success            : out std_logic := '0';
        
        -- PLI interface for communication with compliance test library
        pli_clk                 : out std_logic;
        pli_req                 : in  std_logic;
        pli_ack                 : out std_logic := '0';
        pli_cmd                 : in  std_logic_vector(7 downto 0);
        pli_dest                : in  std_logic_vector(7 downto 0);
        pli_data_in             : in  std_logic_vector(63 downto 0);
        pli_data_in_2           : in  std_logic_vector(63 downto 0);
        pli_str_buf_in          : in  std_logic_vector(511 downto 0);
        pli_data_out            : out std_logic_vector(63 downto 0);

        -- PLI interface for giving test control to compliance test library
        pli_control_req         : out std_logic := '0';
        pli_control_gnt         : in  std_logic
    );
end entity;

architecture tb of test_controller_agent is

    signal seed_applied : boolean := false;

begin
    
    ---------------------------------------------------------------------------
    -- Connect configuration internally to shared signal so that packages
    -- can read it (avoid generic packages since not all simulators support
    -- them).
    ---------------------------------------------------------------------------
    cfg_brp_i <= cfg_brp;
    cfg_prop_i <= cfg_prop;
    cfg_ph_1_i <= cfg_ph_1;
    cfg_ph_2_i <= cfg_ph_2;
    cfg_sjw_i <= cfg_sjw;
    cfg_brp_fd_i <= cfg_brp_fd;
    cfg_prop_fd_i <= cfg_prop_fd;
    cfg_ph_1_fd_i <= cfg_ph_1_fd;
    cfg_ph_2_fd_i <= cfg_ph_2_fd;
    cfg_sjw_fd_i <= cfg_sjw_fd;
    cfg_sys_clk_period_i <= time'value(cfg_sys_clk_period);
    cfg_seed_i <= seed;
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Main test process
    --
    -- Gives commands to other processes / agents like so:
    --  Feature tests:
    --   Invoke Feature test agent which runs the test and gives back result
    --
    --  Reference tests:
    --   Invoke Reference test agent which runs the test and gives back result
    --
    -- Compliance tests:
    --   Pass control to compliance_test_proc which invokes compliance test
    --   library (test control over PLI).
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    main_test_proc : process
        variable test_success_i : std_logic := '0';
    begin
        wait for 1 ns;
        wait until test_start = '1';
        
        -- Apply random seed, but only if it was not applied before.
        -- If there are multiple iterations, we want different random data
        -- to be used for each one!
        if (seed_applied = false) then
            apply_rand_seed(seed);
            seed_applied <= true;
        end if;
        
        -----------------------------------------------------------------------
        -- Configure System clock,
        --  - Period based on generic
        -----------------------------------------------------------------------
        if (stand_alone_vip_mode) then
            info_m("Configuring Clock agent");
            clk_agent_set_period(default_channel, time'value(cfg_sys_clk_period));
            clk_agent_set_duty(default_channel, 50);
            clk_agent_set_jitter(default_channel, 0 ns);
            clk_gen_agent_start(default_channel);
        end if;
        
        -----------------------------------------------------------------------
        -- Configure Timestamp generation
        --  - Step 1
        --  - Each clock cycle
        -----------------------------------------------------------------------
        info_m("Configuring Timestamp agent"); 
        timestamp_agent_set_step(default_channel, 1);
        timestamp_agent_set_prescaler(default_channel, 1);
        timestamp_agent_timestamp_preset(default_channel, x"0000000000000000");
        timestamp_agent_start(default_channel);

        -----------------------------------------------------------------------
        -- Configure Reset agent and Exectue reset
        --  - Polarity 0
        -----------------------------------------------------------------------
        info_m("Configuring Reset agent, executing reset");
        rst_agent_polarity_set(default_channel, '0');
        rst_agent_assert(default_channel);
        wait for 10 ns;
        rst_agent_deassert(default_channel);
        wait for 100 ns; -- For reset to internally de-assert!

        -----------------------------------------------------------------------
        -- Configure Memory bus agent
        -----------------------------------------------------------------------
        info_m("Configuring Memory bus agent");
        mem_bus_agent_x_mode_start(default_channel);
        mem_bus_agent_set_x_mode_setup(default_channel, 2 ns);
        mem_bus_agent_set_x_mode_hold(default_channel, 2 ns);
        mem_bus_agent_set_output_delay(default_channel, 4 ns);
        mem_bus_agent_start(default_channel);

        -----------------------------------------------------------------------
        -- Configure CAN agent
        --
        -- Present in compliance tests and reference tests only!
        -----------------------------------------------------------------------
        if (test_type = "compliance" or test_type = "reference") then
            info_m("Configuring CAN Agent");
            can_agent_monitor_flush(default_channel);
            can_agent_driver_flush(default_channel);
            can_agent_monitor_stop(default_channel);
            can_agent_driver_stop(default_channel);    
            can_agent_monitor_set_input_delay(default_channel, 20 ns);
        end if;

        if (test_type = "compliance") then
            compliance_start <= '1';
            wait until compliance_done = '1';
            test_success_i := pli_test_result;

        elsif (test_type = "feature") then
            feature_start <= '1';
            wait until feature_done = '1';
            test_success_i := feature_result;

        elsif (test_type = "reference") then
            reference_start <= '1';
            wait until reference_done = '1';
            test_success_i := reference_result;

        else 
            error_m("Unknown test type!");
        end if;
        
        -- Stop clock agent (not to generate any further simulation events)
        clk_gen_agent_stop(default_channel);
        
        compliance_start <= '0';
        feature_start <= '0';
        reference_start <= '0';
        wait for 5 ns;

        test_done <= '1';
        test_success <= test_success_i;
        
        info_m("******************************************");
        if (test_success_i = '1') then
            info_m("CTU CAN FD VIP: Test PASSED");
        else
            info_m("CTU CAN FD VIP: Test FAILED");
        end if;
        info_m("******************************************");

        wait until test_start = '0';
        test_done <= '0';

    end process;


    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Compliance test specific part
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    compliance_tests_gen : if (test_type = "compliance") generate
        
        ---------------------------------------------------------------------------
        -- Compliance test handling process
        --
        -- Passes control to Compliance test library linked to simulator.
        -- Communication with this library is done via PLI interface.
        ---------------------------------------------------------------------------
        compliance_test_proc : process
        begin
            wait until compliance_start = '1';
            
            -----------------------------------------------------------------------
            -- Give control over the TB to compliance test library which runs the
            -- test and operates on other agents.
            -----------------------------------------------------------------------
            info_m("Requesting TB control from Compliance test library via PLI...");
            pli_control_req <= '1';
            wait for 1 ns;

            if (pli_control_gnt /= '1') then
                wait until pli_control_gnt = '1' for 10 ns;
            end if;
    
            wait for 0 ns;
            check_m(pli_control_gnt = '1',
                    "Compliance test library took over simulation control!");
            wait for 0 ns;
    
            info_m("Waiting till Compliance test library is done running test...");
            wait until (pli_test_end = '1');
            compliance_done <= '1';
            info_m("Compliance test library signals test has ended");
    
            wait for 50 ns;
            pli_control_req <= '0';
            compliance_done <= '0';
            wait for 50 ns;
        end process;
        
        -----------------------------------------------------------------------
        -- Listen on PLI commands and send them to individual agents!
        -----------------------------------------------------------------------
        pli_listener_process : process
        begin
            
            -------------------------------------------------------------------
            -- Poll on for pli_req = '1'
            -------------------------------------------------------------------
            wait until (pli_req = '1');
            wait for 1 ps;
            
            -------------------------------------------------------------------
            -- Process command (and get answer in case of read)
            -------------------------------------------------------------------
            case pli_dest is
            when PLI_DEST_RES_GEN_AGENT =>
                pli_process_rst_agnt(pli_cmd, pli_data_out, pli_data_in,
                                     default_channel);

            when PLI_DEST_CLK_GEN_AGENT =>
                pli_process_clk_agent(pli_cmd, pli_data_out, pli_data_in,
                                      default_channel);

            when PLI_DEST_MEM_BUS_AGENT =>
                pli_process_mem_bus_agent(pli_cmd, pli_data_out, pli_data_in,
                                          default_channel);

            when PLI_DEST_CAN_AGENT =>
                pli_process_can_agent(pli_cmd, pli_data_out, pli_data_in,
                    pli_data_in_2, pli_str_buf_in, default_channel);

            when PLI_DEST_TEST_CONTROLLER_AGENT =>
                pli_process_test_agent(pli_cmd, pli_data_out, pli_data_in,
                    pli_str_buf_in, pli_test_end, pli_test_result);

            when OTHERS =>
                error_m("Unknown agent destination: " & to_hstring(pli_dest));
            end case;
    
            wait for 1 ps;
    
            -------------------------------------------------------------------
            -- Issue pli_ack = '1'
            -------------------------------------------------------------------
            pli_ack <= '1';
            wait for 1 ps;
    
            -------------------------------------------------------------------
            -- Finish the PLI handshake
            -------------------------------------------------------------------
            wait until (pli_req = '0');
            wait for 1 ps;
            pli_ack <= '0';
            wait for 1 ps;

        end process;


        -----------------------------------------------------------------------
        -- PLI clock generation
        --
        -- Creatse clock for synchronous communication over PLI interface.
        -- Although compliance test library executes test in different context,
        -- it needs to synchronize with simulator context. To do this, 
        -- compliance test library passes all messages to TB via shared memory,
        -- which is read synchronously with PLI callbacks!
        -----------------------------------------------------------------------
        pli_clk_gen_proc : process
        begin
            pli_clk <= '1';
            wait for 1 ns;
            pli_clk <= '0';
            wait for 1 ns;
        end process;

    end generate;
    
    
end architecture;
