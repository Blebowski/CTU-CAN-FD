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
--    CTU CAN FD VIP - Main verification component, encapsulates all test
--    functionality.
--  
--------------------------------------------------------------------------------
-- Revision History:
--    26.1.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;
context ctu_can_fd_tb.tb_agents_context;
context ctu_can_fd_tb.rtl_context;


entity ctu_can_fd_vip is
    generic(
        -- Test details
        test_name               : string;
        test_type               : string;
        stand_alone_vip_mode    : boolean;

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
        cfg_sjw_fd              : natural;
        
        -- Seed
        seed                    : natural := 0;
        
        -- Reference test iterations
        reference_iterations    : natural range 1 to 1000 := 1000
    );
    port(
        -- Test control
        test_start          : in  std_logic;
        test_done           : out std_logic := '0';
        test_success        : out std_logic := '0';
         
        -- DUT interface
        clk_sys             : inout std_logic;
        res_n               : inout std_logic;
        
        -- DFT support
        scan_enable         : out   std_logic;
        
        write_data          : out   std_logic_vector(31 DOWNTO 0);
        read_data           : in    std_logic_vector(31 DOWNTO 0);
        adress              : out   std_logic_vector(15 DOWNTO 0);
        scs                 : out   std_logic;
        srd                 : out   std_logic;
        swr                 : out   std_logic;
        sbe                 : out   std_logic_vector(3 DOWNTO 0);

        int                 : in    std_logic;

        can_tx              : in    std_logic;
        can_rx              : out   std_logic;          

        test_probe          : in    t_ctu_can_fd_test_probe;
        timestamp           : out   std_logic_vector(63 DOWNTO 0) 
    );
end entity;


architecture behav of ctu_can_fd_vip is
    
    signal clk_sys_i            : std_logic;
    signal clk_sys_clock_agent  : std_logic;

    -- RX signals on CAN bus, as driven by various agents.
    -- TX signal is unique (driven by DUT)
    signal can_rx_compliance_agent  : std_logic := '1';
    signal can_rx_feature_agent     : std_logic := '1';
    
    ---------------------------------------------------------------------------
    -- Internal chip selects:
    --  0 - DUT - out of VIP
    --  1 - Test node of feature tests
    ---------------------------------------------------------------------------
    signal scs_i                    : std_logic_vector(1 downto 0);
    signal scs_i_reg                : std_logic_vector(1 downto 0);
    
    signal read_data_test_node      : std_logic_vector(31 downto 0);
    signal read_data_muxed          : std_logic_vector(31 downto 0);
    
    signal res_n_i                  : std_logic;
    
    
    -- PLI interface for communication with compliance test library
    signal pli_clk                 : std_logic;
    signal pli_req                 : std_logic;
    signal pli_ack                 : std_logic := '0';
    signal pli_cmd                 : std_logic_vector(7 downto 0);
    signal pli_dest                : std_logic_vector(7 downto 0);
    signal pli_data_in             : std_logic_vector(63 downto 0);
    signal pli_data_in_2           : std_logic_vector(63 downto 0);
    signal pli_str_buf_in          : std_logic_vector(511 downto 0);
    signal pli_data_out            : std_logic_vector(63 downto 0);

    -- PLI interface for giving test control to compliance test library
    signal pli_control_req         : std_logic := '0';
    signal pli_control_gnt         : std_logic;

    signal pli_test_name_array     : std_logic_vector((test_name'length * 8) - 1 downto 0)
        := (OTHERS => '0');

    -- Test probe of Test node in feature tests
    signal test_node_test_probe    : t_ctu_can_fd_test_probe;

    -- DFT control for test node
    signal test_node_scan_enable   : std_logic;

begin
    
    ---------------------------------------------------------------------------
    -- Reset agent - Asserts reset
    ---------------------------------------------------------------------------
    reset_agent_inst : reset_agent
    port map (
        reset   => res_n_i
    );
    
    ---------------------------------------------------------------------------
    -- Clock agent - Generates clock
    ---------------------------------------------------------------------------
    clk_gen_agent_inst : clk_gen_agent
    port map(
        clock_out  => clk_sys_clock_agent,
        clock_in   => clk_sys_i
    );

    ---------------------------------------------------------------------------
    -- Memory bus agent - Executes memory accesses
    ---------------------------------------------------------------------------
    mem_bus_agent_inst : mem_bus_agent
    generic map(
        G_ACCESS_FIFO_DEPTH => 32,
        G_NUM_SLAVES => 2
    )
    port map(
        clk             => clk_sys_i,
        scs             => scs_i,
        swr             => swr,
        srd             => srd,
        sbe             => sbe,
        write_data      => write_data,
        read_data       => read_data_muxed,
        address         => adress
    );

    ---------------------------------------------------------------------------
    -- Interrupt agent - Checks interrupt pin
    ---------------------------------------------------------------------------
    interrupt_agent_inst : interrupt_agent
    port map(
        int   => int
    );
    
    ---------------------------------------------------------------------------
    -- Timestamp agent - generates timestamp for DUT
    ---------------------------------------------------------------------------
    timestamp_agent_inst : timestamp_agent
    port map(
        clk_sys         => clk_sys_i,
        timestamp       => timestamp
    );
    
    ---------------------------------------------------------------------------
    -- Test probe agent - allows peeking signals brought to test-probe.
    ---------------------------------------------------------------------------
    test_probe_agent_inst : test_probe_agent
    port map(
        dut_test_probe       => test_probe,
        test_node_test_probe => test_node_test_probe,
        
        dut_scan_enable      => scan_enable,
        test_node_scan_enable=> test_node_scan_enable
    );
    
    ---------------------------------------------------------------------------
    -- Test controller agent - controls simulation
    ---------------------------------------------------------------------------
    test_controller_agent_inst : test_controller_agent
    generic map(
        test_name               => test_name,
        test_type               => test_type,
        stand_alone_vip_mode    => stand_alone_vip_mode,
        seed                    => seed,
        
        -- DUT configuration
        cfg_sys_clk_period      => cfg_sys_clk_period,
        cfg_brp                 => cfg_brp,
        cfg_prop                => cfg_prop,
        cfg_ph_1                => cfg_ph_1,
        cfg_ph_2                => cfg_ph_2,
        cfg_sjw                 => cfg_sjw,
        cfg_brp_fd              => cfg_brp_fd,
        cfg_prop_fd             => cfg_prop_fd,
        cfg_ph_1_fd             => cfg_ph_1_fd,
        cfg_ph_2_fd             => cfg_ph_2_fd,
        cfg_sjw_fd              => cfg_sjw_fd
    )
    port map(
        -- Test control interface (to VIP top)
        test_start              => test_start,
        test_done               => test_done,
        test_success            => test_success,
        
        -- PLI communication interface
        pli_clk                 => pli_clk,
        pli_req                 => pli_req,
        pli_ack                 => pli_ack,
        pli_cmd                 => pli_cmd,
        pli_dest                => pli_dest,
        pli_data_in             => pli_data_in,
        pli_data_in_2           => pli_data_in_2,
        pli_str_buf_in          => pli_str_buf_in,
        pli_data_out            => pli_data_out,

        -- PLI interface for giving test control to compliance test library
        pli_control_req         => pli_control_req,
        pli_control_gnt         => pli_control_gnt
    );

    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Test type specific agents
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Compliance tests agent (CAN agent of ISO compliance library).
    --
    -- Used by compliance tests and reference tests.
    ---------------------------------------------------------------------------
    can_agent_gen : if (test_type = "compliance" or test_type = "reference") generate
        compliance_agent_inst : can_agent
        generic map(
            G_DRIVER_FIFO_DEPTH     => 2048,
            G_MONITOR_FIFO_DEPTH    => 2048
        )
        port map(
            can_tx   => can_tx,
            can_rx   => can_rx_compliance_agent
        );
    end generate;
    
    ---------------------------------------------------------------------------
    -- Feature test agent. Used only by feature tests.
    ---------------------------------------------------------------------------
    feature_test_agent_gen : if (test_type = "feature") generate
        feature_test_agent_inst : feature_test_agent
        generic map(
            -- Test details
            test_name           => test_name,
            test_type           => test_type,
            stand_alone_vip_mode => stand_alone_vip_mode,
        
            -- DUT configuration
            cfg_sys_clk_period  => cfg_sys_clk_period,
           
            cfg_brp             => cfg_brp,
            cfg_prop            => cfg_prop,
            cfg_ph_1            => cfg_ph_1,
            cfg_ph_2            => cfg_ph_2,
            cfg_sjw             => cfg_sjw,
            cfg_brp_fd          => cfg_brp_fd,
            cfg_prop_fd         => cfg_prop_fd,
            cfg_ph_1_fd         => cfg_ph_1_fd,
            cfg_ph_2_fd         => cfg_ph_2_fd,
            cfg_sjw_fd          => cfg_sjw_fd
        )
        port map (
            -- Test node connections
            clk_sys             => clk_sys_i,
            res_n               => res_n_i,
            
            write_data          => write_data,
            read_data           => read_data_test_node,
            adress              => adress,
            scs                 => scs_i(1),
            srd                 => srd,
            swr                 => swr,
            sbe                 => sbe,
            
            -- CAN bus from/to DUT
            dut_can_tx          => can_tx,
            dut_can_rx          => can_rx_feature_agent,
            
            test_node_test_probe  => test_node_test_probe,
            test_node_scan_enable => test_node_scan_enable
        );
    end generate;
    
    ---------------------------------------------------------------------------
    -- Reference test agent
    ---------------------------------------------------------------------------
    reference_test_agent_inst : reference_test_agent
    generic map(
        test_name            => test_name,
        test_type            => test_type,
        stand_alone_vip_mode => stand_alone_vip_mode,
        reference_iterations => reference_iterations
    );


    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Other common stuff
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    ---------------------------------------------------------------------------
    -- In stand-alone mode clock agent drives clock, in non-stand-alone, it
    -- receives clock from outside and clock agent is disconnected!
    ---------------------------------------------------------------------------
    stand_alone_mode_gen_true : if (stand_alone_vip_mode) generate
        clk_sys_i <= clk_sys_clock_agent;
        clk_sys <= clk_sys_clock_agent;
    end generate;

    stand_alone_mode_gen_false : if (not stand_alone_vip_mode) generate
        clk_sys_i <= clk_sys;
    end generate;

    res_n <= res_n_i;

    ---------------------------------------------------------------------------
    -- CAN bus connection
    -- 
    -- Realize wired-AND by multiple agents. If agents are not active, they
    -- drive high, therfore they dont affect the bus!
    ---------------------------------------------------------------------------
    can_rx <= can_rx_compliance_agent AND can_rx_feature_agent;
    
    ---------------------------------------------------------------------------
    -- DUT Memory bus routing
    ---------------------------------------------------------------------------
    scs <= scs_i(0);
    
    scs_reg_proc : process(ALL)
    begin
        if (rising_edge(clk_sys_i)) then
            scs_i_reg <= scs_i;
        end if;
    end process;
    
    read_data_muxed <= read_data when (scs_i_reg(0) = '1') else
                       read_data_test_node when (scs_i_reg(1) = '1') else
                       (OTHERS => '0');
    
    ---------------------------------------------------------------------------
    -- Write test name from generic to PLI interface signal
    ---------------------------------------------------------------------------
    test_proc : process
    begin
        pli_str_to_logic_vector(test_name, pli_test_name_array);
        wait;
    end process;
    
    ---------------------------------------------------------------------------
    -- Checks
    ---------------------------------------------------------------------------
    assert test_type = "feature" or test_type = "compliance" or test_type = "reference"
    report "Unsupported test type: " & test_type & ", choose one of: feature, compliance, reference"
    severity failure; 

end architecture;