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
--    Test probe agent - Allows checking internal signals of CTU CAN FD which
--      were brought to "test_probe" top.
--
--    TODO: Once GHDL supports external names, we can remove the test-probe
--          from CTU CAN FD Top, and mirror on internal signal value!  
--
--------------------------------------------------------------------------------
-- Revision History:
--    27.3.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.test_probe_agent_pkg.all;


entity test_probe_agent is
    port (
        -- VIP test control / status signals
        dut_test_probe          : in t_ctu_can_fd_test_probe;
        test_node_test_probe    : in t_ctu_can_fd_test_probe ;
        
        -- DFT support of VIP
        dut_scan_enable         : out std_logic;
        test_node_scan_enable   : out std_logic
    );
end entity;

architecture tb of test_probe_agent is
    
    signal dut_scan_enable_i         : std_logic := '0';
    signal test_node_scan_enable_i   : std_logic := '0';
    
begin

    ---------------------------------------------------------------------------
    -- Comunication receiver process
    ---------------------------------------------------------------------------
    receiver_proc : process
        variable cmd : integer;
        variable reply_code : integer;
        
        variable node : integer;
        
        variable tmp_bool : boolean;
        variable tmp_logic : std_logic;

        -----------------------------------------------------------------------
        -- This implemenation suffers from lower performance, than simple
        -- "wait until", however, it does not exit when signal it waits on
        -- changes only for delta-cycle since it is output of combinatorial
        -- logic!
        -----------------------------------------------------------------------
        procedure wait_signal_delta_glitch_free(
            signal sig      : in std_logic
        ) is
        begin        
            while true loop
                wait until sig = '1';
                wait for 1 ps;
                if (sig = '1') then
                    exit;
                end if;
            end loop;
        end procedure;
    begin
        receive_start(default_channel, C_TEST_PROBE_AGENT_ID);

        -- Command is sent as message type
        cmd := com_channel_data.get_msg_code;
        reply_code := C_REPLY_CODE_OK;
         
        -- Read node on whose test probe to wait
        node := com_channel_data.get_param;
        
        case cmd is
        when TEST_PROBE_AGNT_WAIT_SAMPLE_NO_STUFF =>
            if (node = 0) then
                wait_signal_delta_glitch_free(dut_test_probe.rx_trigger_nbs);
            else
                wait_signal_delta_glitch_free(test_node_test_probe.rx_trigger_nbs);
            end if;
                  
        when TEST_PROBE_AGNT_WAIT_SAMPLE_STUFF =>
            if (node = 0) then
                wait_signal_delta_glitch_free(dut_test_probe.rx_trigger_wbs);
            else
                wait_signal_delta_glitch_free(test_node_test_probe.rx_trigger_wbs);
            end if;
            
        when TEST_PROBE_AGNT_WAIT_SYNC =>
            if (node = 0) then
                wait_signal_delta_glitch_free(dut_test_probe.tx_trigger);
            else
                wait_signal_delta_glitch_free(test_node_test_probe.tx_trigger);
            end if;

        when TEST_PROBE_AGNT_SCAN_CONFIGURE =>
            tmp_bool := com_channel_data.get_param;
            
            if tmp_bool then
                tmp_logic := '1';
            else
                tmp_logic := '0';
            end if;

            if (node = 0) then
                dut_scan_enable_i <= tmp_logic;
            else
                test_node_scan_enable_i <= tmp_logic;
            end if;

        when others =>
            info_m("Invalid message type: " & integer'image(cmd));
            reply_code := C_REPLY_CODE_ERR;

        end case;
        receive_finish(default_channel, reply_code);
    end process;
    
    dut_scan_enable <= dut_scan_enable_i;
    test_node_scan_enable <= test_node_scan_enable_i;
    
end architecture;
