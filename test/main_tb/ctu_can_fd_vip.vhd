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
--    CTU CAN FD VIP
--  
--------------------------------------------------------------------------------
-- Revision History:
--    26.1.2021   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

-- Testbench libry
Library ctu_can_fd_tb;

-- Common stuff
use ctu_can_fd_tb.tb_communication_pkg.all;
use ctu_can_fd_tb.tb_report_pkg.all;

-- Agents
use ctu_can_fd_tb.reset_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;
use ctu_can_fd_tb.can_agent_pkg.all;

-- Design libraries
Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_components.all;
use ctu_can_fd_rtl.can_types.all;


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
        cfg_sjw_fd              : natural
    );
    port(
        -- Test control
        test_start          : in  std_logic;
        test_done           : out std_logic := '0';
        test_success        : out std_logic := '0';
         
        -- DUT interface
        clk_sys             : inout std_logic;
        res_n               : out   std_logic;
        
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
    signal can_rx_compliance_agent  : std_logic;
    signal can_rx_feature_agent     : std_logic;
    
begin
    
    ---------------------------------------------------------------------------
    -- Reset agent
    ---------------------------------------------------------------------------
    reset_agent_inst : reset_agent
    port map (
        reset   => res_n
    );
    
    ---------------------------------------------------------------------------
    -- Clock agent
    ---------------------------------------------------------------------------
    clk_gen_agent_inst : clk_gen_agent
    port map(
        clock  => clk_sys_clock_agent
    );

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

    ---------------------------------------------------------------------------
    -- Memory bus agent
    ---------------------------------------------------------------------------
    mem_bus_agent_inst : mem_bus_agent
    generic map(
        G_ACCESS_FIFO_DEPTH => 32
    )
    port map(
        clk             => clk_sys_i,
        scs             => scs,
        swr             => swr,
        srd             => srd,
        sbe             => sbe,
        write_data      => write_data,
        read_data       => read_data,
        address         => adress
    );
    
    ---------------------------------------------------------------------------
    -- Compliance tests agent (CAN agent of ISO compliance library)
    ---------------------------------------------------------------------------
    compliance_agent_inst : can_agent
    generic map(
        G_DRIVER_FIFO_DEPTH     => 2048,
        G_MONITOR_FIFO_DEPTH    => 2048
    )
    port map(
        can_tx   => can_tx,
        can_rx   => can_rx_compliance_agent
    );

    ---------------------------------------------------------------------------
    -- CAN bus connection
    -- 
    -- Realize wired-AND by multiple agents. If agents are not active, they
    -- drive high, therfore they dont affect the bus!
    ---------------------------------------------------------------------------
    can_rx <= can_rx_compliance_agent AND can_rx_feature_agent;


    ---------------------------------------------------------------------------
    -- Test control - so far dummy
    ---------------------------------------------------------------------------
    test_proc : process
    begin
        -- Start hand-shake
        wait until test_start = '1';
     
                
        info("Asserting reset");            
        rst_agent_assert(default_channel);
        info("Asserted reset");


        wait for 10 ns;
        rst_agent_deassert(default_channel);
        wait for 10 ns;
     
        clk_gen_agent_start(default_channel);
        wait for 1000 ns;
        clk_gen_agent_stop(default_channel);
        wait for 100 ns;
     
        test_success <= '1';
        wait for 0 ns;
        test_done <= '1';
        
     
        
        -- Finish hand-shake
        wait until test_start = '0';
        test_done <= '0';
    end process;

    


    
    ---------------------------------------------------------------------------
    -- Checks
    ---------------------------------------------------------------------------
    assert test_type = "feature" or test_type = "compliance" or test_type = "reference"
    report "Unsupported test type: " & test_type & ", choose one of: feature, compliance, reference"
    severity failure; 

end architecture;