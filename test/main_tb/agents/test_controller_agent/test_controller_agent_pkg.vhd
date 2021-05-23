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
--    Package with API of test controller agent.
--------------------------------------------------------------------------------
-- Revision History:
--    31.1.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.can_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;
use ctu_can_fd_tb.interrupt_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;
use ctu_can_fd_tb.reset_agent_pkg.all;


package test_controller_agent_pkg is

    component test_controller_agent is
    generic(
        -- Test configuration
        test_name               : string;
        test_type               : string;
        stand_alone_vip_mode    : boolean;
        seed                    : natural;
        
        -- DUT configuration
        cfg_sys_clk_period      : string;

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
        -- VPI top test control / status signals
        test_start          : in  std_logic;
        test_done           : out std_logic := '0';
        test_success        : out std_logic := '0';
        
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
    end component;

    -- PLI command destinations
    constant PLI_DEST_TEST_CONTROLLER_AGENT : std_logic_vector(7 downto 0) := x"00";
    constant PLI_DEST_CLK_GEN_AGENT         : std_logic_vector(7 downto 0) := x"01";
    constant PLI_DEST_RES_GEN_AGENT         : std_logic_vector(7 downto 0) := x"02";
    constant PLI_DEST_MEM_BUS_AGENT         : std_logic_vector(7 downto 0) := x"03";
    constant PLI_DEST_CAN_AGENT             : std_logic_vector(7 downto 0) := x"04";

    -- PLI commands for Reset agent
    constant PLI_RST_AGNT_CMD_ASSERT        : std_logic_vector(7 downto 0) := x"01";
    constant PLI_RST_AGNT_CMD_DEASSERT      : std_logic_vector(7 downto 0) := x"02";
    constant PLI_RST_AGNT_CMD_POLARITY_SET  : std_logic_vector(7 downto 0) := x"03";
    constant PLI_RST_AGNT_CMD_POLARITY_GET  : std_logic_vector(7 downto 0) := x"04";

    -- PLI commands for Clock generator
    constant PLI_CLK_AGNT_CMD_START        : std_logic_vector(7 downto 0) := x"01";
    constant PLI_CLK_AGNT_CMD_STOP         : std_logic_vector(7 downto 0) := x"02";
    constant PLI_CLK_AGNT_CMD_PERIOD_SET   : std_logic_vector(7 downto 0) := x"03";
    constant PLI_CLK_AGNT_CMD_PERIOD_GET   : std_logic_vector(7 downto 0) := x"04";
    constant PLI_CLK_AGNT_CMD_JITTER_SET   : std_logic_vector(7 downto 0) := x"05";
    constant PLI_CLK_AGNT_CMD_JITTER_GET   : std_logic_vector(7 downto 0) := x"06";
    constant PLI_CLK_AGNT_CMD_DUTY_SET     : std_logic_vector(7 downto 0) := x"07";
    constant PLI_CLK_AGNT_CMD_DUTY_GET     : std_logic_vector(7 downto 0) := x"08";

    -- PLI commands for Memory bus agent
    constant PLI_MEM_BUS_AGNT_START                 : std_logic_vector(7 downto 0) := x"01";
    constant PLI_MEM_BUS_AGNT_STOP                  : std_logic_vector(7 downto 0) := x"02";
    constant PLI_MEM_BUS_AGNT_WRITE                 : std_logic_vector(7 downto 0) := x"03";
    constant PLI_MEM_BUS_AGNT_READ                  : std_logic_vector(7 downto 0) := x"04";
    constant PLI_MEM_BUS_AGNT_X_MODE_START          : std_logic_vector(7 downto 0) := x"05";
    constant PLI_MEM_BUS_AGNT_X_MODE_STOP           : std_logic_vector(7 downto 0) := x"06";
    constant PLI_MEM_BUS_AGNT_SET_X_MODE_SETUP      : std_logic_vector(7 downto 0) := x"07";
    constant PLI_MEM_BUS_AGNT_SET_X_MODE_HOLD       : std_logic_vector(7 downto 0) := x"08";   
    constant PLI_MEM_BUS_AGNT_SET_OUTPUT_DELAY      : std_logic_vector(7 downto 0) := x"0A";
    constant PLI_MEM_BUS_AGNT_WAIT_DONE             : std_logic_vector(7 downto 0) := x"0B";

    -- PLI commands for CAN Agent
    constant PLI_CAN_AGNT_DRIVER_START                  : std_logic_vector(7 downto 0) := x"01";
    constant PLI_CAN_AGNT_DRIVER_STOP                   : std_logic_vector(7 downto 0) := x"02";
    constant PLI_CAN_AGNT_DRIVER_FLUSH                  : std_logic_vector(7 downto 0) := x"03";
    constant PLI_CAN_AGNT_DRIVER_GET_PROGRESS           : std_logic_vector(7 downto 0) := x"04";
    constant PLI_CAN_AGNT_DRIVER_GET_DRIVEN_VAL         : std_logic_vector(7 downto 0) := x"05";
    constant PLI_CAN_AGNT_DRIVER_PUSH_ITEM              : std_logic_vector(7 downto 0) := x"06";
    constant PLI_CAN_AGNT_DRIVER_SET_WAIT_TIMEOUT       : std_logic_vector(7 downto 0) := x"07";
    constant PLI_CAN_AGNT_DRIVER_WAIT_FINISH            : std_logic_vector(7 downto 0) := x"08";
    constant PLI_CAN_AGNT_DRIVER_DRIVE_SINGLE_ITEM      : std_logic_vector(7 downto 0) := x"09";
    constant PLI_CAN_AGNT_DRIVER_DRIVE_ALL_ITEM         : std_logic_vector(7 downto 0) := x"0A";

    constant PLI_CAN_AGNT_MONITOR_START                 : std_logic_vector(7 downto 0) := x"0B";
    constant PLI_CAN_AGNT_MONITOR_STOP                  : std_logic_vector(7 downto 0) := x"0C";
    constant PLI_CAN_AGNT_MONITOR_FLUSH                 : std_logic_vector(7 downto 0) := x"0D";
    constant PLI_CAN_AGNT_MONITOR_GET_STATE             : std_logic_vector(7 downto 0) := x"0E";
    constant PLI_CAN_AGNT_MONITOR_GET_MONITORED_VAL     : std_logic_vector(7 downto 0) := x"0F";
    constant PLI_CAN_AGNT_MONITOR_PUSH_ITEM             : std_logic_vector(7 downto 0) := x"10";
    constant PLI_CAN_AGNT_MONITOR_SET_WAIT_TIMEOUT      : std_logic_vector(7 downto 0) := x"11";
    constant PLI_CAN_AGNT_MONITOR_WAIT_FINISH           : std_logic_vector(7 downto 0) := x"12";
    constant PLI_CAN_AGNT_MONITOR_MONITOR_SINGLE_ITEM   : std_logic_vector(7 downto 0) := x"13";
    constant PLI_CAN_AGNT_MONITOR_MONITOR_ALL_ITEMS     : std_logic_vector(7 downto 0) := x"14";

    constant PLI_CAN_AGNT_MONITOR_SET_TRIGGER           : std_logic_vector(7 downto 0) := x"15";
    constant PLI_CAN_AGNT_MONITOR_GET_TRIGGER           : std_logic_vector(7 downto 0) := x"16";

    constant PLI_CAN_AGNT_MONITOR_CHECK_RESULT          : std_logic_vector(7 downto 0) := x"19";
    
    constant PLI_CAN_AGNT_MONITOR_SET_INPUT_DELAY       : std_logic_vector(7 downto 0) := x"1A";

    constant PLI_CAN_AGNT_TX_RX_FEEDBACK_ENABLE         : std_logic_vector(7 downto 0) := x"1B";
    constant PLI_CAN_AGNT_TX_RX_FEEDBACK_DISABLE        : std_logic_vector(7 downto 0) := x"1C";
    
    constant PLI_CAN_AGNT_DRIVER_SET_WAIT_FOR_MONITOR   : std_logic_vector(7 downto 0) := x"1D";

    -- PLI commands for Test controller agent
    constant PLI_TEST_AGNT_TEST_END                     : std_logic_vector(7 downto 0) := x"01";
    constant PLI_TEST_AGNT_GET_CFG                      : std_logic_vector(7 downto 0) := x"02";
    constant PLI_TEST_AGNT_GET_SEED                     : std_logic_vector(7 downto 0) := x"03";


    ---------------------------------------------------------------------------
    -- Common signals
    -- Need to be accessed by procedures!
    ---------------------------------------------------------------------------
    
    -- Interface between test controller agent and
    signal feature_start    : std_logic := '0';
    signal feature_done     : std_logic := '0';
    signal feature_result   : std_logic := '0';
    
    -- Interface between main test controller process and compliance test
    -- process (within test controller agent)
    signal compliance_start : std_logic := '0';
    signal compliance_done  : std_logic := '0';
    
    -- Interface between test controller agent and compliance library
    signal pli_test_end     : std_logic := '0';
    signal pli_test_result  : std_logic := '0';
    
    -- Interface between test controller agent and reference test agent
    signal reference_start  : std_logic := '0';
    signal reference_done   : std_logic := '0';
    signal reference_result : std_logic := '0';

    -- Bit timing cofnig used in; compliance tests
    signal cfg_sys_clk_period_i     : time;
    signal cfg_brp_i                : natural;
    signal cfg_prop_i               : natural;
    signal cfg_ph_1_i               : natural;
    signal cfg_ph_2_i               : natural;
    signal cfg_sjw_i                : natural;
    signal cfg_brp_fd_i             : natural;
    signal cfg_prop_fd_i            : natural;
    signal cfg_ph_1_fd_i            : natural;
    signal cfg_ph_2_fd_i            : natural;
    signal cfg_sjw_fd_i             : natural;
    signal cfg_seed_i               : natural;


    ---------------------------------------------------------------------------
    -- Process command for Reset agent
    --
    -- @param pli_cmd      PLI command signal
    -- @param pli_data_out PLI data out signal
    -- @param pli_data_in  PLI data input
    -- @param channel      Communication channel    
    ---------------------------------------------------------------------------
    procedure pli_process_rst_agnt(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      channel         : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Process command for Clock agent.
    --
    -- @param pli_cmd      PLI command signal
    -- @param pli_data_out PLI data out signal
    -- @param pli_data_in  PLI data input
    -- @param channel      Communication channel
    ---------------------------------------------------------------------------
    procedure pli_process_clk_agent(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      channel         : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Process command for memory bus agent.
    --
    -- @param pli_cmd      PLI command signal
    -- @param pli_data_out PLI data out signal
    -- @param pli_data_in  PLI data input
    -- @param channel      Communication channel  
    ---------------------------------------------------------------------------
    procedure pli_process_mem_bus_agent(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      channel         : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Process command for CAN Agent.
    --
    -- @param pli_cmd           PLI command signal
    -- @param pli_data_out      PLI data out signal
    -- @param pli_data_in       PLI data input
    -- @param pli_data_in       PLI data input 2
    -- @param pli_str_buf_in    PLI string buffer input
    -- @param channel           Communication channel  
    ---------------------------------------------------------------------------
    procedure pli_process_can_agent(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      pli_data_in_2   : in    std_logic_vector;
        signal      pli_str_buf_in  : in    std_logic_vector(511 downto 0);
        signal      channel         : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Process test agent 
    --
    -- @param pli_cmd           PLI command signal
    -- @param pli_data_out      PLI data out signal
    -- @param pli_data_in       PLI data input
    -- @param pli_str_buf_in    PLI string buffer input
    -- @param channel           Communication channel
    -- @param test_end          Signal to signalize that test has ended.
    -- @param test_result       Signal to signalize test result
    --                          (1-passed, 0-failed).
    ---------------------------------------------------------------------------
    procedure pli_process_test_agent(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      pli_str_buf_in  : in    std_logic_vector(511 downto 0);
        signal      test_end        : out   std_logic;
        signal      test_result     : out   std_logic
    );

end package;


package body test_controller_agent_pkg is

    
    procedure pli_process_rst_agnt(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      channel         : inout t_com_channel
    ) is
        variable data   :  std_logic;
    begin
        case pli_cmd is
        when PLI_RST_AGNT_CMD_ASSERT =>
            rst_agent_assert(channel);
        when PLI_RST_AGNT_CMD_DEASSERT =>
            rst_agent_deassert(channel);
        when PLI_RST_AGNT_CMD_POLARITY_SET =>
            rst_agent_polarity_set(channel, pli_data_in(0));
        when PLI_RST_AGNT_CMD_POLARITY_GET =>
            rst_agent_polarity_get(channel, data);
            pli_data_out(0) <= data;
            wait for 0 ns;
        when others =>
            error_m("VPI: Unknown Reset agent command with code: 0x" & to_hstring(pli_cmd));
        end case;
    end procedure;


    procedure pli_process_clk_agent(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      channel         : inout t_com_channel
    ) is
        variable period : time;
        variable jitter : time;
        variable duty : integer range 0 to 100;
        variable pli_data_out_i : std_logic_vector(63 downto 0);
    begin
        case pli_cmd is
        when PLI_CLK_AGNT_CMD_START =>
            clk_gen_agent_start(channel);
        when PLI_CLK_AGNT_CMD_STOP =>
            clk_gen_agent_stop(channel);
        when PLI_CLK_AGNT_CMD_PERIOD_SET =>
            pli_logic_vector_to_time(pli_data_in, period);
            clk_agent_set_period(channel, period);
        when PLI_CLK_AGNT_CMD_PERIOD_GET =>
            clk_agent_get_period(channel, period);
            pli_time_to_logic_vector(period, pli_data_out_i);
            pli_data_out <= pli_data_out_i;
        when PLI_CLK_AGNT_CMD_JITTER_SET =>
            pli_logic_vector_to_time(pli_data_in, jitter);
            clk_agent_set_jitter(channel, jitter);
        when PLI_CLK_AGNT_CMD_JITTER_GET =>
            clk_agent_get_jitter(channel, jitter);
            pli_time_to_logic_vector(jitter, pli_data_out_i);
            pli_data_out <= pli_data_out_i;
        when PLI_CLK_AGNT_CMD_DUTY_SET =>
            duty := to_integer(unsigned(pli_data_in));
            clk_agent_set_duty(channel, duty);
        when PLI_CLK_AGNT_CMD_DUTY_GET =>
            clk_agent_get_duty(channel, duty);
            pli_data_out <= std_logic_vector(to_unsigned(duty, 64));
        when others =>
            error_m("VPI: Unknown Clock generator agent command with code: 0x" & to_hstring(pli_cmd));
        end case;
    end procedure;
    
    
    procedure pli_process_mem_bus_agent(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      channel         : inout t_com_channel
    ) is
        variable address        :  integer;
        variable data_8         :  std_logic_vector(7 downto 0);
        variable data_16        :  std_logic_vector(15 downto 0);
        variable data_32        :  std_logic_vector(31 downto 0);
        variable blocking       :  boolean;
        variable setup          :  time;
        variable hold           :  time;
        variable period         :  time;
        variable output_delay   :  time;
    begin
        case pli_cmd is
        when PLI_MEM_BUS_AGNT_START =>
            mem_bus_agent_start(channel);
        when PLI_MEM_BUS_AGNT_STOP =>
            mem_bus_agent_stop(channel);
        when PLI_MEM_BUS_AGNT_WRITE =>
            data_8 := pli_data_in(7 downto 0);
            data_16 := pli_data_in(15 downto 0);
            data_32 := pli_data_in(31 downto 0);
            address := to_integer(unsigned(pli_data_in(47 downto 32)));
            
            -- Blocking tag enconded in bit 50
            if (pli_data_in(50) = '1') then
                blocking := true;
            else
                blocking := false;
            end if;

            -- Access size encoded in these two bits!
            if (pli_data_in(49 downto 48) = "00") then
                mem_bus_agent_write(channel, address, data_8, blocking);
            elsif (pli_data_in(49 downto 48) = "01") then
                mem_bus_agent_write(channel, address, data_16, blocking);
            elsif (pli_data_in(49 downto 48) = "10") then                
                mem_bus_agent_write(channel, address, data_32, blocking);
            else
                error_m("VPI: Invalid memory bus agent write access size: " &
                        to_hstring(pli_data_in(49 downto 48)));
            end if;

        when PLI_MEM_BUS_AGNT_READ =>
            address := to_integer(unsigned(pli_data_in(47 downto 32)));

            pli_data_out(pli_data_out'length - 1 downto 0) <= (OTHERS => '0');
            if (pli_data_in(49 downto 48) = "00") then
                mem_bus_agent_read(channel, address, data_8);
                pli_data_out(7 downto 0) <= data_8;
            elsif (pli_data_in(49 downto 48) = "01") then
                mem_bus_agent_read(channel, address, data_16);
                pli_data_out(15 downto 0) <= data_16;
            elsif (pli_data_in(49 downto 48) = "10") then
                mem_bus_agent_read(channel, address, data_32);
                pli_data_out(31 downto 0) <= data_32;
            else
                error_m("VPI: Invalid memory bus agent read access size: " &
                        to_hstring(pli_data_in(49 downto 48)));
            end if;

        when PLI_MEM_BUS_AGNT_X_MODE_START =>
            mem_bus_agent_x_mode_start(channel);

        when PLI_MEM_BUS_AGNT_X_MODE_STOP =>
            mem_bus_agent_x_mode_start(channel);

        when PLI_MEM_BUS_AGNT_SET_X_MODE_SETUP =>
            pli_logic_vector_to_time(pli_data_in, setup);
            mem_bus_agent_set_x_mode_setup(channel, setup);

        when PLI_MEM_BUS_AGNT_SET_X_MODE_HOLD =>
            pli_logic_vector_to_time(pli_data_in, hold);
            mem_bus_agent_set_x_mode_hold(channel, hold);

        when PLI_MEM_BUS_AGNT_SET_OUTPUT_DELAY =>
            pli_logic_vector_to_time(pli_data_in, output_delay);
            mem_bus_agent_set_output_delay(channel, output_delay);

        when PLI_MEM_BUS_AGNT_WAIT_DONE =>
            mem_bus_agent_wait_done(channel);

        when others =>
            error_m("VPI: Unknown Memory bus agent command with code: 0x" & to_hstring(pli_cmd));
        end case;
    end procedure;
    
    
    procedure pli_process_can_agent(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      pli_data_in_2   : in    std_logic_vector;
        signal      pli_str_buf_in  : in    std_logic_vector(511 downto 0);
        signal      channel         : inout t_com_channel
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
        variable pli_data_out_i : std_logic_vector(63 downto 0);
        variable input_delay    : time;
    begin
        case pli_cmd is
            
        when PLI_CAN_AGNT_DRIVER_START =>
            can_agent_driver_start(channel);

        when PLI_CAN_AGNT_DRIVER_STOP =>
            can_agent_driver_stop(channel);

        when PLI_CAN_AGNT_DRIVER_FLUSH =>
            can_agent_driver_flush(channel);

        when PLI_CAN_AGNT_DRIVER_GET_PROGRESS =>
            can_agent_driver_get_progress(channel, progress);
            if (progress) then
                pli_data_out(0) <= '1';
            else
                pli_data_out(0) <= '0';
            end if;

        when PLI_CAN_AGNT_DRIVER_GET_DRIVEN_VAL =>
            can_agent_driver_get_driven_val(channel, driven_val);
            pli_data_out(0) <= driven_val;

        when PLI_CAN_AGNT_DRIVER_PUSH_ITEM =>
            
            -- Time conversion from 64 bits truly uses only 62 bits, stuff
            -- remaining information for driven item into remaining two
            -- bits so that we don't need to declare next signal via VPI.
            driver_item.value := pli_data_in(63);
            if (pli_data_in(62) = '1') then
                driver_item.print_msg := true;
                pli_logic_vector_to_str(pli_str_buf_in, driver_item.msg);
            else
                driver_item.print_msg := false;
            end if;
            pli_logic_vector_to_time(pli_data_in, driver_item.drive_time);
            
            can_agent_driver_push_item(channel, driver_item);

        when PLI_CAN_AGNT_DRIVER_SET_WAIT_TIMEOUT =>
            pli_logic_vector_to_time(pli_data_in, timeout);
            can_agent_driver_set_wait_timeout(channel, timeout);

        when PLI_CAN_AGNT_DRIVER_WAIT_FINISH =>
            can_agent_driver_wait_finish(channel);
            
        when PLI_CAN_AGNT_DRIVER_DRIVE_SINGLE_ITEM =>
            
            -- Time conversion from 64 bits truly uses only 62 bits, stuff
            -- remaining information for driven item into remaining two
            -- bits so that we don't need to declare next signal via VPI.
            driver_item.value := pli_data_in(63);
            if (pli_data_in(62) = '1') then
                driver_item.print_msg := true;
                pli_logic_vector_to_str(pli_str_buf_in, driver_item.msg);
            else
                driver_item.print_msg := false;
            end if;
            pli_logic_vector_to_time(pli_data_in, driver_item.drive_time);
            can_agent_driver_drive_single_item(channel, driver_item);

        when PLI_CAN_AGNT_DRIVER_DRIVE_ALL_ITEM =>
            can_agent_driver_drive_all_items(channel);

        when PLI_CAN_AGNT_DRIVER_SET_WAIT_FOR_MONITOR =>
            if (pli_data_in(0) = '1') then
                can_agent_driver_set_wait_for_monitor(channel, true);
            else
                can_agent_driver_set_wait_for_monitor(channel, false);
            end if;

        when PLI_CAN_AGNT_MONITOR_START =>
            can_agent_monitor_start(channel);

        when PLI_CAN_AGNT_MONITOR_STOP =>
            can_agent_monitor_stop(channel);

        when PLI_CAN_AGNT_MONITOR_FLUSH =>
            can_agent_monitor_flush(channel);

        when PLI_CAN_AGNT_MONITOR_GET_STATE =>
            can_agent_monitor_get_state(channel, monitor_state);
            case monitor_state is
            when mon_disabled =>
                pli_data_out(2 downto 0) <= "000";
            when mon_waiting_for_trigger =>
                pli_data_out(2 downto 0) <= "001";
            when mon_running =>
                pli_data_out(2 downto 0) <= "010";
            when mon_passed =>
                pli_data_out(2 downto 0) <= "011";
            when mon_failed =>
                pli_data_out(2 downto 0) <= "100";
            end case;

        when PLI_CAN_AGNT_MONITOR_GET_MONITORED_VAL =>
            can_agent_monitor_get_monitored_val(channel, monitored_val);
            pli_data_out(0) <= monitored_val;

        when PLI_CAN_AGNT_MONITOR_PUSH_ITEM =>
            monitor_item.value := pli_data_in(63);
            if (pli_data_in(62) = '1') then
                monitor_item.print_msg := true;
                pli_logic_vector_to_str(pli_str_buf_in, monitor_item.msg);
            else
                monitor_item.print_msg := false;
            end if;

            -- Time conversion from 64 bits truly uses only 62 bits, stuff
            -- remaining information for driven item into remaining two
            -- bits so that we don't need to declare next signal via VPI.
            pli_logic_vector_to_time(pli_data_in, monitor_item.monitor_time);
            pli_logic_vector_to_time(pli_data_in_2, monitor_item.sample_rate);

            can_agent_monitor_push_item(channel, monitor_item);

        when PLI_CAN_AGNT_MONITOR_SET_WAIT_TIMEOUT =>
            pli_logic_vector_to_time(pli_data_in, timeout);
            can_agent_monitor_set_wait_timeout(channel, timeout);
            
        when PLI_CAN_AGNT_MONITOR_WAIT_FINISH =>
            can_agent_monitor_wait_finish(channel);

        when PLI_CAN_AGNT_MONITOR_MONITOR_SINGLE_ITEM =>
            monitor_item.value := pli_data_in(63);
            if (pli_data_in(62) = '1') then
                monitor_item.print_msg := true;
                pli_logic_vector_to_str(pli_str_buf_in, monitor_item.msg);
            else
                monitor_item.print_msg := false;
            end if;

            -- Time conversion from 64 bits truly uses only 62 bits, stuff
            -- remaining information for driven item into remaining two
            -- bits so that we don't need to declare next signal via VPI.
            pli_logic_vector_to_time(pli_data_in, monitor_item.monitor_time);
            pli_logic_vector_to_time(pli_data_in_2, monitor_item.sample_rate);

            can_agent_monitor_single_item(channel, monitor_item);

        when PLI_CAN_AGNT_MONITOR_MONITOR_ALL_ITEMS =>
            can_agent_monitor_all_items(channel);
    
        when PLI_CAN_AGNT_MONITOR_SET_TRIGGER =>
            case pli_data_in(2 downto 0) is
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
                error_m("Invalid monitor trigger type: " &
                        to_hstring(pli_data_in(2 downto 0)));
            end case;
            can_agent_monitor_set_trigger(channel, trigger);

        when PLI_CAN_AGNT_MONITOR_GET_TRIGGER =>
            can_agent_monitor_get_trigger(channel, trigger);
            case trigger is
            when trig_immediately =>
                pli_data_out(2 downto 0) <= "000";
            when trig_can_rx_rising_edge =>
                pli_data_out(2 downto 0) <= "001";
            when trig_can_rx_falling_edge =>
                pli_data_out(2 downto 0) <= "010";
            when trig_can_tx_rising_edge =>
                pli_data_out(2 downto 0) <= "011";
            when trig_can_tx_falling_edge =>
                pli_data_out(2 downto 0) <= "100";
            when trig_time_elapsed =>
                pli_data_out(2 downto 0) <= "101";
            when trig_driver_start =>
                pli_data_out(2 downto 0) <= "110";
            when trig_driver_stop =>
                pli_data_out(2 downto 0) <= "111";
            end case;

        when PLI_CAN_AGNT_MONITOR_CHECK_RESULT =>
            can_agent_monitor_check_result(channel);

        when PLI_CAN_AGNT_MONITOR_SET_INPUT_DELAY =>
            pli_logic_vector_to_time(pli_data_in, input_delay);
            can_agent_monitor_set_input_delay(channel, input_delay);
            
        when PLI_CAN_AGNT_TX_RX_FEEDBACK_ENABLE =>
            can_agent_configure_tx_to_rx_feedback(channel, true);
            
        when PLI_CAN_AGNT_TX_RX_FEEDBACK_DISABLE =>
            can_agent_configure_tx_to_rx_feedback(channel, false);
            
        when others =>
            error_m("VPI: Unknown CAN agent command with code: 0x" & to_hstring(pli_cmd));
        end case;
    end procedure;
    
    
    procedure pli_process_test_agent(
        signal      pli_cmd         : in    std_logic_vector(7 downto 0);
        signal      pli_data_out    : out   std_logic_vector;
        signal      pli_data_in     : in    std_logic_vector;
        signal      pli_str_buf_in  : in    std_logic_vector(511 downto 0);
        signal      test_end        : out   std_logic;
        signal      test_result     : out   std_logic
    ) is
        variable param_name     : string(1 to 64);
        variable pli_data_out_i : std_logic_vector(63 downto 0) := (OTHERS => '0');
    begin
        case pli_cmd is
        when PLI_TEST_AGNT_TEST_END =>
            test_end    <= '1';
            test_result <= pli_data_in(0);

        when PLI_TEST_AGNT_GET_CFG =>
            -- VPI message string encodes desired configuration parameter
            pli_logic_vector_to_str(pli_str_buf_in, param_name);
            pli_data_out(pli_data_out'length - 1 downto 0) <= (OTHERS => '0');

            -- Replace '\0' by ' ' to avoid invalid characters in generated XML. This then
            -- prevents conversion of Junit XML to e.g. HTML !!
            for i in 1 to 64 loop
                if (param_name(i) = character'val(0)) then
                    param_name(i) := ' ';
                end if;
            end loop;
            info_m("SW test queries parameter: " & param_name(1 to 64));

            if (param_name(45 to 64) = "CFG_DUT_CLOCK_PERIOD") then
                pli_time_to_logic_vector(cfg_sys_clk_period_i, pli_data_out_i);
                pli_data_out <= pli_data_out_i;

            elsif (param_name(54 to 64) = "CFG_DUT_BRP") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_brp_i, pli_data_out'length));

            elsif (param_name(53 to 64) = "CFG_DUT_PROP") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_prop_i, pli_data_out'length));

            elsif (param_name(54 to 64) = "CFG_DUT_PH1") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_ph_1_i, pli_data_out'length));

            elsif (param_name(54 to 64) = "CFG_DUT_PH2") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_ph_2_i, pli_data_out'length));

            elsif (param_name(54 to 64) = "CFG_DUT_SJW") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_sjw_i, pli_data_out'length));

            elsif (param_name(51 to 64) = "CFG_DUT_BRP_FD") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_brp_fd_i, pli_data_out'length));

            elsif (param_name(50 to 64) = "CFG_DUT_PROP_FD") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_prop_fd_i, pli_data_out'length));

            elsif (param_name(51 to 64) = "CFG_DUT_PH1_FD") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_ph_1_fd_i, pli_data_out'length));

            elsif (param_name(51 to 64) = "CFG_DUT_PH2_FD") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_ph_2_fd_i, pli_data_out'length));

            elsif (param_name(51 to 64) = "CFG_DUT_SJW_FD") then
                pli_data_out <= std_logic_vector(to_unsigned(cfg_sjw_fd_i, pli_data_out'length));

            else
                error_m("Unsupported configuration parameter name: " & param_name);
            end if;

        when PLI_TEST_AGNT_GET_SEED =>
            pli_data_out <= std_logic_vector(to_unsigned(cfg_seed_i, pli_data_out'length));

        when others =>
            error_m("VPI: Unknown test agent command with code: 0x" & to_hstring(pli_cmd));
        end case;
    end procedure;

end package body;
