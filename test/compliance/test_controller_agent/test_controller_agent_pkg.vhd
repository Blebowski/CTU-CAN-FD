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
--    Package with declarations for test controller agent.
--------------------------------------------------------------------------------
-- Revision History:
--    31.1.2020   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package test_controller_agent_pkg is

    component test_controller_agent is
    generic (
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
        cfg_sjw_fd         : natural := 2;
        
        seed               : natural := 0
    );
    port(
        -- VPI communication interface
        vpi_clk         : out   std_logic;
        vpi_req         : in    std_logic;
        vpi_ack         : out   std_logic;
        vpi_cmd         : in    std_logic_vector(7 downto 0);
        vpi_dest        : in    std_logic_vector(7 downto 0);
        vpi_data_in     : in    std_logic_vector(63 downto 0);
        vpi_data_in_2   : in    std_logic_vector(63 downto 0);
        vpi_str_buf_in  : in    std_logic_vector(511 downto 0);
        vpi_data_out    : out   std_logic_vector(63 downto 0);

        -- VPI test control interface
        vpi_control_req     : out   std_logic;
        vpi_control_gnt     : in    std_logic
    );
    end component;

    -- VPI command destinations
    constant VPI_DEST_TEST_CONTROLLER_AGENT : std_logic_vector(7 downto 0) := x"00";
    constant VPI_DEST_CLK_GEN_AGENT         : std_logic_vector(7 downto 0) := x"01";
    constant VPI_DEST_RES_GEN_AGENT         : std_logic_vector(7 downto 0) := x"02";
    constant VPI_DEST_MEM_BUS_AGENT         : std_logic_vector(7 downto 0) := x"03";
    constant VPI_DEST_CAN_AGENT             : std_logic_vector(7 downto 0) := x"04";

    -- VPI commands for Reset agent
    constant VPI_RST_AGNT_CMD_ASSERT        : std_logic_vector(7 downto 0) := x"01";
    constant VPI_RST_AGNT_CMD_DEASSERT      : std_logic_vector(7 downto 0) := x"02";
    constant VPI_RST_AGNT_CMD_POLARITY_SET  : std_logic_vector(7 downto 0) := x"03";
    constant VPI_RST_AGNT_CMD_POLARITY_GET  : std_logic_vector(7 downto 0) := x"04";

    -- VPI commands for Clock generator
    constant VPI_CLK_AGNT_CMD_START        : std_logic_vector(7 downto 0) := x"01";
    constant VPI_CLK_AGNT_CMD_STOP         : std_logic_vector(7 downto 0) := x"02";
    constant VPI_CLK_AGNT_CMD_PERIOD_SET   : std_logic_vector(7 downto 0) := x"03";
    constant VPI_CLK_AGNT_CMD_PERIOD_GET   : std_logic_vector(7 downto 0) := x"04";
    constant VPI_CLK_AGNT_CMD_JITTER_SET   : std_logic_vector(7 downto 0) := x"05";
    constant VPI_CLK_AGNT_CMD_JITTER_GET   : std_logic_vector(7 downto 0) := x"06";
    constant VPI_CLK_AGNT_CMD_DUTY_SET     : std_logic_vector(7 downto 0) := x"07";
    constant VPI_CLK_AGNT_CMD_DUTY_GET     : std_logic_vector(7 downto 0) := x"08";

    -- VPI commands for Memory bus agent
    constant VPI_MEM_BUS_AGNT_START                 : std_logic_vector(7 downto 0) := x"01";
    constant VPI_MEM_BUS_AGNT_STOP                  : std_logic_vector(7 downto 0) := x"02";
    constant VPI_MEM_BUS_AGNT_WRITE                 : std_logic_vector(7 downto 0) := x"03";
    constant VPI_MEM_BUS_AGNT_READ                  : std_logic_vector(7 downto 0) := x"04";
    constant VPI_MEM_BUS_AGNT_X_MODE_START          : std_logic_vector(7 downto 0) := x"05";
    constant VPI_MEM_BUS_AGNT_X_MODE_STOP           : std_logic_vector(7 downto 0) := x"06";
    constant VPI_MEM_BUS_AGNT_SET_X_MODE_SETUP      : std_logic_vector(7 downto 0) := x"07";
    constant VPI_MEM_BUS_AGNT_SET_X_MODE_HOLD       : std_logic_vector(7 downto 0) := x"08";   
    constant VPI_MEM_BUS_AGNT_SET_PERIOD            : std_logic_vector(7 downto 0) := x"09";   
    constant VPI_MEM_BUS_AGNT_SET_OUTPUT_DELAY      : std_logic_vector(7 downto 0) := x"0A";
    constant VPI_MEM_BUS_AGNT_WAIT_DONE             : std_logic_vector(7 downto 0) := x"0B";

    -- VPI commands for CAN Agent
    constant VPI_CAN_AGNT_DRIVER_START                  : std_logic_vector(7 downto 0) := x"01";
    constant VPI_CAN_AGNT_DRIVER_STOP                   : std_logic_vector(7 downto 0) := x"02";
    constant VPI_CAN_AGNT_DRIVER_FLUSH                  : std_logic_vector(7 downto 0) := x"03";
    constant VPI_CAN_AGNT_DRIVER_GET_PROGRESS           : std_logic_vector(7 downto 0) := x"04";
    constant VPI_CAN_AGNT_DRIVER_GET_DRIVEN_VAL         : std_logic_vector(7 downto 0) := x"05";
    constant VPI_CAN_AGNT_DRIVER_PUSH_ITEM              : std_logic_vector(7 downto 0) := x"06";
    constant VPI_CAN_AGNT_DRIVER_SET_WAIT_TIMEOUT       : std_logic_vector(7 downto 0) := x"07";
    constant VPI_CAN_AGNT_DRIVER_WAIT_FINISH            : std_logic_vector(7 downto 0) := x"08";
    constant VPI_CAN_AGNT_DRIVER_DRIVE_SINGLE_ITEM      : std_logic_vector(7 downto 0) := x"09";
    constant VPI_CAN_AGNT_DRIVER_DRIVE_ALL_ITEM         : std_logic_vector(7 downto 0) := x"0A";

    constant VPI_CAN_AGNT_MONITOR_START                 : std_logic_vector(7 downto 0) := x"0B";
    constant VPI_CAN_AGNT_MONITOR_STOP                  : std_logic_vector(7 downto 0) := x"0C";
    constant VPI_CAN_AGNT_MONITOR_FLUSH                 : std_logic_vector(7 downto 0) := x"0D";
    constant VPI_CAN_AGNT_MONITOR_GET_STATE             : std_logic_vector(7 downto 0) := x"0E";
    constant VPI_CAN_AGNT_MONITOR_GET_MONITORED_VAL     : std_logic_vector(7 downto 0) := x"0F";
    constant VPI_CAN_AGNT_MONITOR_PUSH_ITEM             : std_logic_vector(7 downto 0) := x"10";
    constant VPI_CAN_AGNT_MONITOR_SET_WAIT_TIMEOUT      : std_logic_vector(7 downto 0) := x"11";
    constant VPI_CAN_AGNT_MONITOR_WAIT_FINISH           : std_logic_vector(7 downto 0) := x"12";
    constant VPI_CAN_AGNT_MONITOR_MONITOR_SINGLE_ITEM   : std_logic_vector(7 downto 0) := x"13";
    constant VPI_CAN_AGNT_MONITOR_MONITOR_ALL_ITEMS     : std_logic_vector(7 downto 0) := x"14";

    constant VPI_CAN_AGNT_MONITOR_SET_TRIGGER           : std_logic_vector(7 downto 0) := x"15";
    constant VPI_CAN_AGNT_MONITOR_GET_TRIGGER           : std_logic_vector(7 downto 0) := x"16";

    constant VPI_CAN_AGNT_MONITOR_CHECK_RESULT          : std_logic_vector(7 downto 0) := x"19";
    
    constant VPI_CAN_AGNT_MONITOR_SET_INPUT_DELAY       : std_logic_vector(7 downto 0) := x"1A";

    constant VPI_CAN_AGNT_TX_RX_FEEDBACK_ENABLE         : std_logic_vector(7 downto 0) := x"1B";
    constant VPI_CAN_AGNT_TX_RX_FEEDBACK_DISABLE        : std_logic_vector(7 downto 0) := x"1C";
    
    constant VPI_CAN_AGNT_DRIVER_SET_WAIT_FOR_MONITOR   : std_logic_vector(7 downto 0) := x"1D";

    -- VPI commands for Test controller agent
    constant VPI_TEST_AGNT_TEST_END                     : std_logic_vector(7 downto 0) := x"01";
    constant VPI_TEST_AGNT_GET_CFG                      : std_logic_vector(7 downto 0) := x"02";
    constant VPI_TEST_AGNT_GET_SEED                     : std_logic_vector(7 downto 0) := x"03";

end package;


library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package body test_controller_agent_pkg is

end package body;
