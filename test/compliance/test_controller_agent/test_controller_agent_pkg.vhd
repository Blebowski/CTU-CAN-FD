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

    -- VPI command destinations
    constant VPI_DEST_TEST_CONTROLLER_AGENT : integer := 0;
    constant VPI_DEST_CLK_GEN_AGENT         : integer := 1;
    constant VPI_DEST_RES_GEN_AGENT         : integer := 2;
    constant VPI_DEST_MEM_BUS_AGENT         : integer := 3;
    constant VPI_DEST_CAN_AGENT             : integer := 4;

    -- VPI commands for Reset agent
    constant VPI_RST_AGNT_CMD_ASSERT        : integer := 0;
    constant VPI_RST_AGNT_CMD_DEASSERT      : integer := 1;
    constant VPI_RST_AGNT_CMD_POLARITY_SET  : integer := 2;
    constant VPI_RST_AGNT_CMD_POLARITY_GET  : integer := 3;

    -- VPI commands for Clock generator
    -- TODO:

    -- VPI commands for Memory bus agent
    -- TODO:
    
    -- VPI commands for CAN Agent
    -- TODO:

end package;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package body test_controller_agent_pkg is

    

end package body;
