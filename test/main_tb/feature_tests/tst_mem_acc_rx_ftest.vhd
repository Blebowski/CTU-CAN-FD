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
-- @TestInfoStart
--
-- @Purpose:
--  Test access to RX buffer RAM. 
--
-- @Verifies:
--  @1. RX buffer RAM can be written and read via Test registers.
--
-- @Test sequence:
--  @1. Read size of RX buffer in DUT. Enable Test Mode and Test access in DUT.
--      Disable DUT (since memory testability shall be used when DUT is disabled)
--  @2. Generate random content for whole RAM.
--  @3. Write whole RX buffer RAM.
--  @4. Read whole RX buffer RAM back and compare read values with written ones.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    2.5.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package tst_mem_acc_rx_ftest is
    procedure tst_mem_acc_rx_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body tst_mem_acc_rx_ftest is

    procedure tst_mem_acc_rx_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable r_data  : std_logic_vector(31 downto 0) := (OTHERS => '0');
        variable rx_info : SW_RX_Buffer_info;
        
        type t_test_mem is
            array (0 to 4095) of std_logic_vector(31 downto 0);
        variable w_content : t_test_mem := (OTHERS => (OTHERS => '0'));
        variable mode      : SW_mode    := SW_mode_rst_val;
    begin

        -----------------------------------------------------------------------
        -- @1. Read size of RX buffer in DUT. Enable Test Mode and Test access
        --     in DUT. Disable DUT (since memory testability shall be used
        --     when DUT is disabled).
        -----------------------------------------------------------------------
        info_m("Step 1");
        get_rx_buf_state(rx_info, DUT_NODE, chn);
        mode.test := true;
        set_core_mode(mode, DUT_NODE, chn);
        
        -- First we must disable DUT
        CAN_turn_controller(false, DUT_NODE, chn);
        set_test_mem_access(true, DUT_NODE, chn);
        
        -----------------------------------------------------------------------
        -- @2. Generate random content for whole RAM.
        -----------------------------------------------------------------------
        info_m("Step 2");
        for i in 0 to rx_info.rx_mem_free - 1 loop
            rand_logic_vect_v(w_content(i), 0.5);
            wait for 1 ps; -- To avoid timeout due to max delta cycles
        end loop;
        
        -----------------------------------------------------------------------
        -- @3. Write whole RX buffer RAM.
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        -- To reduce log size for biggest RX buffer size.
        mem_bus_agent_disable_transaction_reporting(chn);
        
        for i in 0 to rx_info.rx_mem_free - 1 loop
            test_mem_write(w_content(i), i, TST_TGT_RX_BUF, DUT_NODE, chn);
        end loop;

        -----------------------------------------------------------------------
        -- @4. Read whole RX buffer RAM back and compare read values with
        --     written ones.
        -----------------------------------------------------------------------
        for i in 0 to rx_info.rx_mem_free - 1 loop
            test_mem_read(r_data, i, TST_TGT_RX_BUF, DUT_NODE, chn);
            check_m(r_data = w_content(i), "RX RAM data at address: " &
                integer'image(i) & " Expected: 0x" & to_hstring(w_content(i)) &
                " Read: 0x" & to_hstring(r_data));
        end loop;
        
        mem_bus_agent_enable_transaction_reporting(chn);

  end procedure;

end package body;
