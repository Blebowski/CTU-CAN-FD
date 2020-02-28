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
--    Test controller agent. Controls CAN FD compliance TB over Vunit
--    communication library.
--
--    More on Vunit and its communication library can be found at:
--     https://vunit.github.io/documentation.html
--     https://vunit.github.io/com/user_guide.html
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
use work.test_controller_agent_pkg.all;
use work.clk_gen_agent_pkg.all;
use work.rst_gen_agent_pkg.all;
use work.mem_bus_agent_pkg.all;
use work.can_agent_pkg.all;

entity test_controller_agent is
    generic (
        -- This is Vunit runner config, don't name it runner_cfg so that
        -- Vunit will not scan it automatically as standalone test!
        cfg             : string
    );
    port(
        -- VPI communication interface
        vpi_req         : in    std_logic;
        vpi_ack         : out   std_logic := '0';
        vpi_cmd         : in    std_logic_vector(7 downto 0);
        vpi_dest        : in    std_logic_vector(7 downto 0);
        vpi_data_in     : in    std_logic_vector(31 downto 0);
        vpi_data_out    : out   std_logic_vector(31 downto 0);
    
        -- VPI Mutext lock/unlock interface
        vpi_mutex_lock      : out   std_logic;
        vpi_mutex_unlock    : out   std_logic;
        
        -- VPI test control interface
        vpi_control_req     : out   std_logic := '0';
        vpi_control_gnt     : in    std_logic;
        vpi_test_end        : in    std_logic;
        vpi_test_result     : in    boolean
    );
end entity;

architecture tb of test_controller_agent is

    procedure vpi_process_rst_agnt(
        signal      net             : inout network_t;
        signal      vpi_data_out    : out   std_logic_vector
    ) is
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
            polarity_get_rst_agent(net, data);
            vpi_data_out(0) <= data;
            wait for 0 ns;
        when others =>
            error("Unknown command!");
        end case;
    end procedure;

begin
    
    ---------------------------------------------------------------------------
    -- Main test process
    ---------------------------------------------------------------------------
    main_test_proc : process
    begin
        test_runner_setup(runner, cfg);

        wait for 10 ns;

        -----------------------------------------------------------------------
        -- Relinquish control to SW part of TB. SW part of TB controls
        -- everything, reset, clock, memory access, CAN agent (driver, monitor)
        -----------------------------------------------------------------------
        info("Requesting TB control from SW");
        vpi_control_req <= '1';
        wait for 1 ns;

        if (vpi_control_gnt /= '1') then
            wait until vpi_control_gnt <= '1' for 10 ns;
        end if;

        wait for 0 ns;
        check(vpi_control_gnt = '1', "SW part of TB took over simulation control!");
        wait for 0 ns;

        info("Waiting till SW part of test ends!");
        
        wait for 5000 ns;

        info("SW part of TB signals test has ended");
        test_runner_cleanup(runner, true);
        
        --wait until vpi_test_end = '1';
        
    end process;


    ---------------------------------------------------------------------------
    -- Listen on VPI commands and send them to individual agents!
    ---------------------------------------------------------------------------
    vpi_listener_process : process
        
        -----------------------------------------------------------------------
        -- Lock / Unlock of mutex in VPI .so library via interface with
        -- callback.
        -- Due to non-zero wait after assigning lock/unlock, this process
        -- should schedule away in simulator till next simulation time 1 ps
        -- larger. At that point simulator should treat it as another non-delta
        -- cycle and should execute VPI callback which will lock the mutex and
        -- return only after it is locked. So when any of these two functions
        -- exits it should hold the mutex for VPI handshake interface and
        -- running C++ test thread should not mess with values on vpi_*
        -- signals!
        --
        -- Further-more memory barriers should not be needed since mutex is
        -- used.
        --
        -- Note that whole handshake interface will have lot of overhead because
        -- following sequence must be executed to send just single transaction
        -- from C++ thread to simulation:
        --  1. CPP  : Set data, destination to be sent
        --  2. CPP  : Lock mutex
        --  3. CPP  : Set vpi_req = '1'
        --  4. CPP  : Unlock mutex
        --  5. GHDL : Lock mutex
        --  6. GHDL : Read vpi_req = '1'
        --  7. GHDL : Process the transaction, issue vpi_ack = '1'
        --  8. GHDL : Unlock mutex
        --  9. CPP  : Lock mutex
        -- 10. CPP  : Read vpi_ack = '1'
        -- 11. CPP  : Issue vpi_ack = '0'
        -- 12. CPP  : Unlock mutex
        -- 13. GHDL : Lock mutes
        -- 14. GHDL : Read vpi_req = '0'
        -- 15. GHDL : Issue vpi_ack = '0'
        -- 16. GHDL : Unlock mutex
        -- 17. CPP  : Lock mutex
        -- 18. CPP  : Read vpi_ack ='0'
        -- 19. CPP  : Unlock mutex
        --
        -- It is assumed that this form of communication will be e.g. one time
        -- setup, sending data to monitor/driver, or sending memory transaction
        -- to memory bus agent (for reading/writing data from/to DUT). This
        -- should not be used for polling as simulation will be very un-efective
        -- in this case!
        --
        -- To synchronize TB and SW operation, rather SW mechanisms should be
        -- used: e.g. callback on driver/monitor finished!
        -----------------------------------------------------------------------
        procedure lock_handshake_mutex is
        begin
            vpi_mutex_lock <= not vpi_mutex_lock;
            wait for 1 ps;
        end procedure;
        
        procedure unlock_handshake_mutex is
        begin
            vpi_mutex_unlock <= not vpi_mutex_unlock;
            wait for 1 ps;
        end procedure;

    begin
        while (true) loop
            lock_handshake_mutex;
            if (vpi_req = '1') then
                exit;
            end if;
            unlock_handshake_mutex;
            wait for 1 ns;
        end loop;
        wait for 1 ps;
        
        case vpi_dest is
        when VPI_DEST_RES_GEN_AGENT =>
            vpi_process_rst_agnt(net, vpi_data_out);
        when OTHERS =>
            error("Unknown VPI command: " & to_hstring(vpi_dest));
        end case;

        vpi_ack <= '1';
        unlock_handshake_mutex;
        wait for 1 ps;

        while (true) loop
            lock_handshake_mutex;
            if (vpi_req = '0') then
                exit;
            end if;
            wait for 1 ns;
            unlock_handshake_mutex;
        end loop;
        wait for 1 ps;
        
        vpi_ack <= '0';
        unlock_handshake_mutex;
        wait for 1 ps;

    end process;

end architecture;