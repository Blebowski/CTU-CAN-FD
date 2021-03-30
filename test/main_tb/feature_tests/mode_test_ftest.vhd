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
--  Test mode - feature test.
--
-- @Verifies:
--  @1. EWL register is writable in test mode, not writable when not in test
--      mode.
--  @2. ERP register is writable in test mode, not writable when not in test
--      mode.
--  @3. CTR_PRES register is writable in test mode, not writable when not in
--      test mode.
--
-- @Test sequence:
--  @1. Configure test mode in DUT. Check that EWL and ERP are equal to its
--      reset values!
--  @2. Generate random values and store them to EWL, ERP. Read back and check
--      that both were properly written!
--  @3. Disable test mode in DUT. Generate random values and store them to
--      EWL and ERP. Read back and check that values were NOT written!
--  @4. Enable test mode in DUT. Read each error counter and check it is 0!
--      Set each error counter to random value via CTR_PRES. Read back and check
--      that value was written!
--  @5. Disable test mode in DUT. Set each error counter to random values and
--      check that these values were NOT written (counters equal to previous
--      values).
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    18.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package mode_test_ftest is
    procedure mode_test_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body mode_test_ftest is
    procedure mode_test_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     :       SW_error_counters := (0, 0, 0, 0);

        variable fault_th           :       SW_fault_thresholds;
        variable fault_th_2         :       SW_fault_thresholds;
    begin

        ------------------------------------------------------------------------
        -- @1. Configure test mode in DUT. Check that EWL and ERP are equal to
        --     its reset values!
        ------------------------------------------------------------------------
        info_m("Step 1: Configure test mode, check EWL/ERP!");
        
        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        get_fault_thresholds(fault_th, DUT_NODE, chn);
        check_m(fault_th.ewl = to_integer(unsigned(EW_LIMIT_RSTVAL)), 
            "EWL equal to reset value!");
        check_m(fault_th.erp = to_integer(unsigned(ERP_LIMIT_RSTVAL)), 
            "ERP equal to reset value!");           

        ------------------------------------------------------------------------
        -- @2. Generate random values and store them to EWL, ERP. Read back and
        --     check that both were properly written!
        ------------------------------------------------------------------------
        info_m("Step 2: Store random values to EWL/ERP!");
        rand_int_v(255, fault_th.ewl);
        rand_int_v(255, fault_th.erp); 
        set_fault_thresholds(fault_th, DUT_NODE, chn);
        get_fault_thresholds(fault_th_2, DUT_NODE, chn);
        
        check_m(fault_th.ewl = fault_th_2.ewl, "EWL written in test mode!");
        check_m(fault_th.erp = fault_th_2.erp, "ERP written in test mode!");
        
        ------------------------------------------------------------------------
        -- @3. Disable test mode in DUT. Generate random values and store 
        --     them to EWL and ERP. Read back and check that values were NOT
        --     written!
        ------------------------------------------------------------------------
        info_m("Step 3: Disable test mode, check EWL/ERP not written!"); 
        
        mode_1.test := false;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        rand_int_v(255, fault_th_2.ewl);
        rand_int_v(255, fault_th_2.erp);
        set_fault_thresholds(fault_th_2, DUT_NODE, chn);
        get_fault_thresholds(fault_th_2, DUT_NODE, chn);
        
        check_m(fault_th.ewl = fault_th_2.ewl,
            "EWL not written when not in test mode!");
        check_m(fault_th.erp = fault_th_2.erp,
            "ERP not written when not in test mode!");
        
        ------------------------------------------------------------------------
        -- @4. Enable test mode in DUT. Read each error counter and check it
        --     is 0! Set each error counter to random value via CTR_PRES. Read
        --     back and check that value was written!
        ------------------------------------------------------------------------
        info_m("Step 4: Enable test mode, check CTR_PRES written!");
        
        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        read_error_counters(err_counters, DUT_NODE, chn);
        check_m(err_counters.rx_counter = 0, "REC = 0");
        check_m(err_counters.tx_counter = 0, "TEC = 0");
        check_m(err_counters.err_norm = 0, "Errors in Nominal bit rate = 0");
        check_m(err_counters.err_fd = 0, "Errors in Data bit rate = 0");
        
        rand_int_v(511, err_counters.rx_counter);
        rand_int_v(511, err_counters.tx_counter);

        set_error_counters(err_counters, DUT_NODE, chn);
        read_error_counters(err_counters_2, DUT_NODE, chn);
        
        check_m(err_counters.rx_counter = err_counters_2.rx_counter,
            "RX Error counter set!");
        check_m(err_counters.tx_counter = err_counters_2.tx_counter,
            "TX Error counter set!");
        -- Do not test Nominal/Data bit error rate counters here since these
        -- are clear only! These are tested in separate (CTR_PRES) test!
        
        ------------------------------------------------------------------------
        -- @5. Disable test mode in DUT. Set each error counter to random 
        --     values and check that these values were NOT written (counters 
        --     equal to previous values).
        ------------------------------------------------------------------------
        info_m("Step 5: Disable test mode, check CTR_PRES not written!");
        
        mode_1.test := false;
        set_core_mode(mode_1, DUT_NODE, chn);

        rand_int_v(511, err_counters_2.rx_counter);
        rand_int_v(511, err_counters_2.tx_counter);
        
        set_error_counters(err_counters_2, DUT_NODE, chn);
        read_error_counters(err_counters_2, DUT_NODE, chn);
        
        check_m(err_counters.rx_counter = err_counters_2.rx_counter,
            "RX Error counter set!");
        check_m(err_counters.tx_counter = err_counters_2.tx_counter,
            "TX Error counter set!");
        
  end procedure;

end package body;