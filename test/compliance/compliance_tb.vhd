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
use work.test_controller_agent_pkg.all;

use work.can_compliance_tb_pkg.all;

-- Design libraries
use work.can_components.all;

entity can_compliance_tb is
    generic (
        runner_cfg     : string := runner_cfg_default;
        vpi_test_name  : string;
        
        -- Test configuration
        cfg_clock_period   : string := "10 ns";
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

    ---------------------------------------------------------------------------
    -- This is an ugly hack to pass String over VPI since GHDL does not provide
    -- string type signals on vpiScan for nets!
    --------------------------------------------------------------------------- 
    signal vpi_test_name_array : std_logic_vector((vpi_test_name'length * 8) - 1 downto 0)
        := (OTHERS => '0');

    -- Top level VPI for communication with SW part of TB
    signal vpi_clk          : std_logic := '0';
    signal vpi_req          : std_logic := '0';
    signal vpi_ack          : std_logic := '0';
    signal vpi_cmd          : std_logic_vector(7 downto 0) := (OTHERS => '0');
    signal vpi_dest         : std_logic_vector(7 downto 0) := (OTHERS => '0');
    signal vpi_data_in      : std_logic_vector(63 downto 0) := (OTHERS => '0');
    signal vpi_data_in_2    : std_logic_vector(63 downto 0) := (OTHERS => '0');
    signal vpi_str_buf_in   : std_logic_vector(511 downto 0) := (OTHERS => '0');
    signal vpi_data_out     : std_logic_vector(63 downto 0) := (OTHERS => '0');

    -- VPI test control interface
    signal vpi_control_req      : std_logic := '0';
    signal vpi_control_gnt      : std_logic := '0';

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

    ---------------------------------------------------------------------------
    -- Test controller agent (communication with SW part of TB)
    ---------------------------------------------------------------------------
    test_controller_agent_inst : test_controller_agent
    generic map(
        cfg => runner_cfg,
        
        cfg_clock_period => time'value(cfg_clock_period),
        cfg_brp          => cfg_brp,
        cfg_prop         => cfg_prop,
        cfg_ph_1         => cfg_ph_1,
        cfg_ph_2         => cfg_ph_2,
        cfg_sjw          => cfg_sjw,
        cfg_brp_fd       => cfg_brp_fd,
        cfg_prop_fd      => cfg_prop_fd,
        cfg_ph_1_fd      => cfg_ph_1_fd,
        cfg_ph_2_fd      => cfg_ph_2_fd,
        cfg_sjw_fd       => cfg_sjw_fd
    )
    port map(
        -- VPI communication interface
        vpi_clk         => vpi_clk,
        vpi_req         => vpi_req,
        vpi_ack         => vpi_ack,
        vpi_cmd         => vpi_cmd,
        vpi_dest        => vpi_dest,
        vpi_data_in     => vpi_data_in,
        vpi_data_in_2   => vpi_data_in_2,
        vpi_str_buf_in  => vpi_str_buf_in,
        vpi_data_out    => vpi_data_out,

        -- VPI test control interface
        vpi_control_req     => vpi_control_req,
        vpi_control_gnt     => vpi_control_gnt
    );

    ---------------------------------------------------------------------------
    -- Write test name from generic to signal (TODO: Test config!)
    ---------------------------------------------------------------------------
    test_proc : process
    begin
        str_to_logic_vector(vpi_test_name, vpi_test_name_array);
        wait;
    end process;


end architecture;
