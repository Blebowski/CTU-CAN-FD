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
--    Test controller agent.
--    TODO: Further documentation!
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

entity test_controller_agent is
    generic (
        runner_cfg : string := runner_cfg_default
    );
    port(
        -- VPI communication interface
        vpi_req         : in    std_logic;
        vpi_ack         : out   std_logic;
        vpi_cmd         : in    integer;
        vpi_dest        : in    integer;
        vpi_data_in     : in    std_logic_vector(31 downto 0);
        vpi_data_out    : out   std_logic_vector(31 downto 0);
    
        -- VPI test control interface
        vpi_control_req : out   std_logic;
        vpi_control_gnt : in    std_logic;
        vpi_test_end    : in    std_logic;
        vpi_test_result : in    boolean
    );
end entity;

architecture tb of test_controller_agent is

    procedure vpi_process_rst_agnt is
        variable data   :  std_logic;
    begin
        case vpi_cmd is
        when VPI_RST_AGNT_CMD_ASSERT =>
            assert_rst_agent(net);
        when VPI_RST_AGNT_CMD_DEASSERT =>
            deassert_rst_agent(net);
        when VPI_RST_AGNT_CMD_POLARITY_SET =>
            polarity_set_rst_agent(net, vpi_data_in(0));
        when VPI_RST_AGNT_CMD_POLARITY_GET =>
            data := polarity_get_rst_agent(net);
            vpi_data_out(0) <= data;
            wait for 0 ns;
        when others =>
            error("TODO");
        end case;
    end procedure;

begin
    
    ---------------------------------------------------------------------------
    -- Main test process
    ---------------------------------------------------------------------------
    main_test_proc : process
    begin
        test_runner_setup(runner, runner_cfg);
        
        -----------------------------------------------------------------------
        -- Relinquish control to SW part of TB. SW part of TB controls
        -- everything, reset, clock, memory access, CAN agent (driver, monitor)
        -----------------------------------------------------------------------
        vpi_control_req <= '1';
        wait until vpi_control_gnt <= '1' for 10 ns;

        if (vpi_control_gnt /= 1) then
            error("SW part of TB did not take over control!");
        end if;
        
        -----------------------------------------------------------------------
        -- Wait until SW part of TB signals to us we are done!
        -- (No need to add timeout, VUnit takes care of this)
        -----------------------------------------------------------------------
        wait until vpi_test_end = '1';

        test_runner_cleanup(runner, vpi_test_result);
    end process;
    
    ---------------------------------------------------------------------------
    -- Listen on VPI commands and send them to individual agents!
    ---------------------------------------------------------------------------
    vpi_listener_process : process
    begin
        wait until (vpi_req = '1');
        
        case vpi_dest is
        when VPI_DEST_RES_GEN_AGENT =>
            vpi_process_rst_agnt;
        end case;

        vpi_ack <= '1';
        wait until (vpi_req = '0');
        vpi_ack <= '0';
        wait for 0 ns;
    end process;

end architecture;