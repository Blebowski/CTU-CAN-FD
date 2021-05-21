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
--    Feature test agent. Executes feature tests.
--
--    Feature test agent is started by test controller agent, when test_type
--    is configured to "feature". Feature test agent resets DUT, configures it,
--    enables it, runs test sequence.
--
--------------------------------------------------------------------------------
-- Revision History:
--    11.3.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_agents_context;

entity feature_test_agent is
    generic(
        -- Test details
        test_name               : string;
        test_type               : string;
        stand_alone_vip_mode    : boolean;

        -- DUT config
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
    port(
        -----------------------------------------------------------------------
        -- Test node connections
        -----------------------------------------------------------------------
        clk_sys         :   in  std_logic;
        res_n           :   in  std_logic;
        
        write_data      :   in  std_logic_vector(31 DOWNTO 0);
        read_data       :   out std_logic_vector(31 DOWNTO 0);
        adress          :   in  std_logic_vector(15 DOWNTO 0);
        scs             :   in  std_logic;
        srd             :   in  std_logic;
        swr             :   in  std_logic;
        sbe             :   in  std_logic_vector(3 DOWNTO 0);
        
        -- CAN bus from/to DUT
        dut_can_tx      :   in  std_logic;
        dut_can_rx      :   out std_logic;
        
        -- Test Nodes test probe output
        test_node_test_probe  : out t_ctu_can_fd_test_probe;
        test_node_scan_enable : in  std_logic
    );
end entity;


architecture tb of feature_test_agent is
    
    signal bus_level    :   std_logic;
    
    -- Test node signals
    signal test_node_can_tx : std_logic;
    signal test_node_can_rx : std_logic;

    -- Delayed CAN bus signals
    signal dut_can_tx_delayed : std_logic;
    signal test_node_can_tx_delayed : std_logic;

    -- Forcing bus level value (ANDed bus level)
    signal force_bus_level_i        : boolean := false;
    signal force_bus_level_value    : std_logic;

    -- Forcing CAN RX of only single node
    signal force_can_rx_dut         : boolean := false;
    signal force_can_rx_test_node   : boolean := false;
    signal force_can_rx_value       : std_logic;

    -- Transceiver delays (on can_tx signal)
    signal can_tx_delay_dut         : time := 1 ns;
    signal can_tx_delay_test_node   : time := 1 ns;

begin
    
    ---------------------------------------------------------------------------
    -- Test node
    ---------------------------------------------------------------------------
    test_node_inst : entity ctu_can_fd_rtl.can_top_level
    generic map(
        -- Keep config hard-coded, it is enough that DUT is configurable!
        rx_buffer_size      => 256, -- Size to receive 8 frames is needed
        txt_buffer_count    => 4,
        sup_filtA           => false,
        sup_filtB           => false,
        sup_filtC           => false,
        sup_range           => false,
        sup_traffic_ctrs    => false,
        target_technology   => C_TECH_ASIC
    )
    port map(
        -- Clock and Asynchronous reset
        clk_sys     => clk_sys,
        res_n       => res_n,
        
        -- DFT support
        scan_enable => test_node_scan_enable,

        -- Memory interface
        data_in     => write_data,
        data_out    => read_data,
        adress      => adress,
        scs         => scs,
        srd         => srd,
        swr         => swr,
        sbe         => sbe,

        -- Interrupt Interface - not needed for test node
        int         => open,

        -- CAN Bus Interface
        can_tx      => test_node_can_tx,
        can_rx      => test_node_can_rx,

        -- Test probe, timestamp, not needed for test node!
        test_probe  => test_node_test_probe,
        timestamp   => (OTHERS => '1')
    );

    ---------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable cmd : integer;
        variable reply_code : integer;
        variable tmp : integer;
        variable tmp_logic : std_logic;
    begin
        receive_start(default_channel, C_FEATURE_TEST_AGENT_ID);

        -- Command is sent as message type
        cmd := com_channel_data.get_msg_code;
        reply_code := C_REPLY_CODE_OK;
         
        case cmd is
        when FEATURE_TEST_AGNT_FORCE_BUS =>
            force_bus_level_value <= com_channel_data.get_param;
            force_bus_level_i <= true;

        when FEATURE_TEST_AGNT_RELEASE_BUS =>
            force_bus_level_i <= false;
            
        when FEATURE_TEST_AGNT_FORCE_CAN_RX =>
            tmp := com_channel_data.get_param;
            force_can_rx_value <= com_channel_data.get_param;
            if (tmp = 0) then
                force_can_rx_dut <= true;
            else
                force_can_rx_test_node <= true;
            end if;
            
        when FEATURE_TEST_AGNT_RELEASE_CAN_RX =>
            force_can_rx_dut <= false;
            force_can_rx_test_node <= false;

        when FEATURE_TEST_AGNT_SET_TRV_DELAY =>
            tmp := com_channel_data.get_param;
            if (tmp = 0) then
                can_tx_delay_dut <= com_channel_data.get_param;
            else
                can_tx_delay_test_node <= com_channel_data.get_param;
            end if;

        when FEATURE_TEST_AGNT_CHECK_BUS_LEVEL =>
            tmp_logic := com_channel_data.get_param; 
            check_m(tmp_logic = bus_level, FEATURE_TEST_AGENT_TAG &
                    "Bus level value shoul be:" & std_logic'image(tmp_logic));

        when FEATURE_TEST_AGNT_CHECK_CAN_TX =>
            tmp := com_channel_data.get_param;
            tmp_logic := com_channel_data.get_param;
            if (tmp = 0) then
                check_m(tmp_logic = dut_can_tx, "DUT CAN TX");
            else
                check_m(tmp_logic = test_node_can_tx, "Test node CAN TX");
            end if;

        when FEATURE_TEST_AGNT_GET_CAN_TX =>
            tmp := com_channel_data.get_param;
            if (tmp = 0) then
                com_channel_data.set_param(dut_can_tx);
            else
                com_channel_data.set_param(test_node_can_tx);
            end if;

        when FEATURE_TEST_AGNT_GET_CAN_RX =>
            tmp := com_channel_data.get_param;
            if (tmp = 0) then
                com_channel_data.set_param(dut_can_rx);
            else
                com_channel_data.set_param(test_node_can_rx);
            end if;

        when others =>
            info_m("Invalid message type: " & integer'image(cmd));
            reply_code := C_REPLY_CODE_ERR;

        end case;
        receive_finish(default_channel, reply_code);
    end process;

    ---------------------------------------------------------------------------
    -- Signal delaying 
    ---------------------------------------------------------------------------
    i_tx_delay_dut : entity ctu_can_fd_tb.signal_delayer
    generic map (
        NSAMPLES    => 32
    )
    port map (
        input       => dut_can_tx,
        delay       => can_tx_delay_dut,
        delayed     => dut_can_tx_delayed
    );
    
    i_tx_delay_test_node : entity ctu_can_fd_tb.signal_delayer
    generic map (
        NSAMPLES    => 32
    )
    port map (
        input       => test_node_can_tx,
        delay       => can_tx_delay_test_node,
        delayed     => test_node_can_tx_delayed
    );
    
    ---------------------------------------------------------------------------
    -- Bus level and RX signal of each node
    ---------------------------------------------------------------------------
    bus_level <= force_bus_level_value when force_bus_level_i else
                 dut_can_tx_delayed and test_node_can_tx_delayed;

    dut_can_rx <= force_can_rx_value when force_can_rx_dut else
                  bus_level;

    test_node_can_rx <= force_can_rx_value when force_can_rx_test_node else
                        bus_level;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Test control process
    --
    -- Waits on start request from Test controller agent and runs a test.
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    test_process : process        
        variable bus_timing     :    bit_time_config_type :=(
            tq_nbt      => cfg_brp,
            tq_dbt      => cfg_brp_fd,
            prop_nbt    => cfg_prop,
            ph1_nbt     => cfg_ph_1,
            ph2_nbt     => cfg_ph_2,
            sjw_nbt     => cfg_sjw,
            prop_dbt    => cfg_prop_fd,
            ph1_dbt     => cfg_ph_1_fd,
            ph2_dbt     => cfg_ph_2_fd,
            sjw_dbt     => cfg_sjw_fd
         );
    begin
        wait until feature_start = '1';

        -- Pre-set test to be "passed", any error will make it fail
        ctu_vip_test_result.set_result(true);
        
        -- Initialize TXT Buffer memories
        info_m("***************************************************************");
        info_m("Clearing TXT Buffer memories!");
        info_m("***************************************************************");
        info_m("DUT node:");
        CAN_init_txtb_mems(DUT_NODE, default_channel);
        info_m("Test node:");
        CAN_init_txtb_mems(TEST_NODE, default_channel);
        
        -- Configure bit timing
        CAN_configure_timing(bus_timing, DUT_NODE, default_channel);
        CAN_configure_timing(bus_timing, TEST_NODE, default_channel);

        -- Set default retransmitt limit to 0 (Failed frames are not retransmited)
        CAN_enable_retr_limit(true, 0, DUT_NODE, default_channel);
        CAN_enable_retr_limit(true, 0, TEST_NODE, default_channel);

        -- Enable CAN controllers
        CAN_turn_controller(true, DUT_NODE, default_channel);
        CAN_turn_controller(true, TEST_NODE, default_channel);
        info_m("Controllers are ON");
        
        -- Wait till integration is over in both nodes
        CAN_wait_bus_on(DUT_NODE, default_channel);
        CAN_wait_bus_on(TEST_NODE, default_channel);
        info_m("Bus integration finished");

        -- Execute feature test
        exec_feature_test(test_name, default_channel);

        -- Signal test is done.
        feature_result <= ctu_vip_test_result.get_result;
        wait for 0 ns;
        feature_done <= '1';
        wait until feature_start = '0';
        feature_done <= '0';
        wait for 0 ns;

    end process;

end architecture;