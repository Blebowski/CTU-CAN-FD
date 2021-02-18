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
--  RX Buffer status MOF (middle of frame) feature test implementation.
--
-- @Verifies:
--  RX_STATUS[MOF] bit functionality.
--
-- @Test sequence:
--  @1. Repeat 10 loops of:
--   @1.1 Check that RX_STATUS[MOF] is 0 in Node 1 (no frame stored in RX Buffer).
--        Send frame by Node 2, wait till it is send, and check RX_STATUS[MOF] is
--        still zero.
--   @1.2 Read one word from RX_DATA, and check that RX_STATUS[MOF] is set. Read
--        rest of the frame, word by word and check that RX_STATUS[MOF] is set.
--        Check that after last read, it is not set anymore.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    26.2.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package rx_status_mof_feature is
    procedure rx_status_mof_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body rx_status_mof_feature is
    procedure rx_status_mof_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1           	:       natural range 0 to 15 := 1;
        variable ID_2           	:       natural range 0 to 15 := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable send_more          :       boolean := true;
        variable in_RX_buf          :       natural range 0 to 1023;
        variable frame_sent         :       boolean := false;
        variable number_frms_sent   :       natural range 0 to 1023;

        variable buf_info           :       SW_RX_Buffer_info;
        variable command            :       SW_command := SW_command_rst_val;
        variable status             :       SW_status;
        
        variable read_data          :       std_logic_vector(31 downto 0);
        variable rwcnt              :       natural;   
    begin

        for i in 1 to 10 loop
            info ("Loop nr:" & integer'image(i));
       
            -------------------------------------------------------------------
            -- @1.1 Check that RX_STATUS[MOF] is 0 in Node 1 (no frame stored
            --      in RX Buffer). Send frame by Node 2, wait till it is send,
            --      and check RX_STATUS[MOF] is still zero.
            -------------------------------------------------------------------
            info("Step 1.1");
            get_rx_buf_state(buf_info, ID_1, mem_bus(1));
            check(buf_info.rx_mof = false, "RX_STATUS[MOF] not set");
            
            CAN_generate_frame(rand_ctr, CAN_frame);
            CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
            CAN_wait_frame_sent(ID_1, mem_bus(1));
            
            get_rx_buf_state(buf_info, ID_1, mem_bus(1));
            check(buf_info.rx_mof = false, "RX_STATUS[MOF] not set");
            
            -------------------------------------------------------------------
            -- @1.2 Read one word from RX_DATA, and check that RX_STATUS[MOF]
            --      is set. Read rest of the frame, word by word and check that
            --      RX_STATUS[MOF] is set. Check that after last read, it is
            --      not set anymore.
            -------------------------------------------------------------------
            info("Step 1.2");
            
            CAN_read(read_data, RX_DATA_ADR, ID_1, mem_bus(1));
            get_rx_buf_state(buf_info, ID_1, mem_bus(1));
            check(buf_info.rx_mof = true,
                  "RX_STATUS[MOF] is set after first word!");
            
            rwcnt := to_integer(unsigned(read_data(RWCNT_H downto RWCNT_L)));
            for j in 1 to rwcnt - 1 loop
                CAN_read(read_data, RX_DATA_ADR, ID_1, mem_bus(1));
                get_rx_buf_state(buf_info, ID_1, mem_bus(1));
                check(buf_info.rx_mof = true, "RX_STATUS[MOF] is set after word " &
                      integer'image(j));
            end loop;
            
            CAN_read(read_data, RX_DATA_ADR, ID_1, mem_bus(1));
            get_rx_buf_state(buf_info, ID_1, mem_bus(1));
            check(buf_info.rx_mof = false,
                  "RX_STATUS[MOF] is not set after last word");

        end loop;

    end procedure;

end package body;