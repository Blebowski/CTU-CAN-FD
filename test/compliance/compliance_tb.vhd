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
--    CAN compliance testbench. Top level entity
--  
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

Library work;

-- Testbench libraries
use work.clk_gen_agent_pkg.all;
use work.rst_gen_agent_pkg.all;
use work.mem_bus_agent_pkg.all;
use work.can_agent_pkg.all;

-- Design libraries
use work.can_components.all;

entity can_compliance_tb is
    generic (
        runner_cfg : string := runner_cfg_default
    );
end entity;

architecture tb of can_compliance_tb is
    
    signal dut_clk  : std_logic;
    signal dut_rst  : std_logic;

    -- CAN bus signals
    signal dut_can_tx   : std_logic;
    signal dut_can_rx   : std_logic;

    -- DUT memory bus (RAM-like memory bus)
    signal dut_address      : std_logic_vector(15 downto 0);
    signal dut_write_data   : std_logic_vector(31 downto 0);
    signal dut_read_data    : std_logic_vector(31 downto 0);
    signal dut_scs          : std_logic;
    signal dut_swr          : std_logic;
    signal dut_srd          : std_logic;
    signal dut_sbe          : std_logic_vector(3 downto 0);

    signal dut_timestamp    : std_logic_vector(63 downto 0) := (OTHERS => '0');

    signal test_signal      : std_logic := '0';

begin

    ---------------------------------------------------------------------------
    -- Clock generator agent
    ---------------------------------------------------------------------------
    clk_gen_agent_inst : clk_gen_agent
    port map(
        clock   => dut_clk
    );

    ---------------------------------------------------------------------------
    -- Reset generator agent
    ---------------------------------------------------------------------------
    rst_gen_agent_inst : rst_gen_agent
    port map(
        reset   => dut_rst
    );

    ---------------------------------------------------------------------------
    -- CAN Agent (Driver for CAN RX, Monitor for CAN TX) 
    ---------------------------------------------------------------------------
    can_agent_inst : can_agent
    generic map(
        G_DRIVER_FIFO_DEPTH  => 1024,
        G_MONITOR_FIFO_DEPTH => 1024
    )
    port map(
        -- CAN bus connections
        can_tx   => dut_can_tx,
        can_rx   => dut_can_rx
    );

    ---------------------------------------------------------------------------
    -- Memory bus agent (for RAM-like interface)
    ---------------------------------------------------------------------------
    mem_bus_agent_inst : mem_bus_agent
    generic map(
        G_ACCESS_FIFO_DEPTH  => 32
    )
    port map(
        clk             => dut_clk,

        scs             => dut_scs,
        swr             => dut_swr,
        srd             => dut_srd,
        sbe             => dut_sbe,
        write_data      => dut_write_data,
        read_data       => dut_read_data,
        address         => dut_address
    );

    ---------------------------------------------------------------------------
    -- DUT (Use RAM-like memory bus)
    ---------------------------------------------------------------------------
    dut : can_top_level
    generic map(
        rx_buffer_size      => 64,
        sup_filtA           => false,
        sup_filtB           => false,
        sup_filtC           => false,
        sup_range           => false,
        sup_traffic_ctrs    => true
    )
    port map(
        -- Clock and Asynchronous reset
        clk_sys     => dut_clk,
        res_n       => dut_rst,

        -- Memory interface
        data_in     => dut_write_data,
        data_out    => dut_read_data,
        adress      => dut_address,
        scs         => dut_scs,
        srd         => dut_srd,
        swr         => dut_swr,
        sbe         => dut_sbe,

        -- Interrupt Interface
        int         => open,

        -- CAN Bus Interface
        can_tx      => dut_can_tx,
        can_rx      => dut_can_rx,

        -- Internal signals for testbenches
        drv_bus_o   => open,
        stat_bus_o  => open,

        -- Timestamp for time based transmission / reception
        timestamp   => dut_timestamp
    );


    -- This is an example process only!
    test_proc : process
        variable read_data : std_logic_vector(31 downto 0);
    begin
        test_runner_setup(runner, runner_cfg);
        
        wait for 100 ns;
        
        polarity_set_rst_agent(net, '0');
        assert_rst_agent(net);
        
        set_period_clk_agent(net, 10 ns);
        start_clk_gen_agent(net);
        
        wait for 100 ns;
        set_period_clk_agent(net, 20 ns);
        deassert_rst_agent(net);

        wait for 100 ns;
        set_period_clk_agent(net, 5 ns);
        
        wait for 200 ns;
        set_period_clk_agent(net, 10 ns);
        set_duty_clk_agent(net, 50);
        wait for 100 ns;

        test_signal <= '1';

        --mem_bus_agent_write_non_blocking(net, 4, x"AAAAAAAA", "1111");
        --mem_bus_agent_write_non_blocking(net, 8, x"BBBBBBBB", "1111");
        wait for 10 ns;
        mem_bus_agent_start(net);

        mem_bus_agent_read(net, 0, read_data, "1111");
        
        wait for 1000 ns;
        mem_bus_agent_stop(net);

        wait for 1000 ns;

        can_agent_monitor_set_trigger(net, trig_driver_start);
        can_agent_monitor_push_value(net, '0', 20 ns, "SOF");
        can_agent_monitor_push_value(net, '1', 20 ns, "ID[0]");
        can_agent_monitor_push_value(net, '0', 20 ns, "ID[1]");
        can_agent_monitor_push_value(net, '1', 20 ns, "ID[2]");
        can_agent_monitor_start(net);
        
        -- Test CAN agent driver here!
        can_agent_driver_push_value(net, '0', 20 ns, "A");
        can_agent_driver_push_value(net, '1', 20 ns, "B");
        can_agent_driver_push_value(net, '0', 20 ns, "C");
        can_agent_driver_push_value(net, '1', 20 ns, "D");
        can_agent_driver_start(net);

        --wait for 1000 ns;
        can_agent_monitor_wait_finish(net);
        can_agent_monitor_stop(net);
        can_agent_driver_stop(net);

        wait for 1000 ns;
        
        can_agent_monitor_check_result(net);


        stop_clk_gen_agent(net);

        test_runner_cleanup(runner, false);

        wait;
    end process;


end architecture;