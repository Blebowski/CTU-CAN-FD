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
--  Single bus node test
--
-- @Verifies:
--  @1. Node which is single on bus (other node is not transmitting, acking,
--      sending error frames), will turn error passive and not bus-off when
--      trying to transmitt a frame!
--
-- @Test sequence:
--  @1. Disable Node 2, disable retransmitt limit in Node 1.
--  @2. Transmitt frame by Node 1.
--  @3. Wait until error frame starts and check that error counter is incremented
--      by 8. Repeat until node turns error passive!
--  @4. Wait for several times that node transmitts a frame and check that after
--      each, TX Error counter is not incremented.
--
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    18.7.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package single_bus_node_feature is
    procedure single_bus_node_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body single_bus_node_feature is
    procedure single_bus_node_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable alc                :       natural;

        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;

        variable fault_state_1      :     SW_fault_state;
        
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_2            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        variable mode_1             :     SW_mode;

        -- Node status
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        
        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;
        
        variable err_counters       :     SW_error_counters;
        
        constant id_template        :     std_logic_vector(10 downto 0) :=
                "01010101010";
        variable id_var             :     std_logic_vector(10 downto 0) :=
                 (OTHERS => '0');
        variable retr_index         :     natural := 0;
        variable tec_at_err_passive :     natural;
        variable frame_sent         :     boolean := false;
    begin

        ------------------------------------------------------------------------
        -- @1. Disable Node 2, disable retransmitt limit in Node 1.
        ------------------------------------------------------------------------
        info("Step 1: Disabling Node 2");
        CAN_turn_controller(false, ID_2, mem_bus(2));        
        CAN_enable_retr_limit(false, 0, ID_1, mem_bus(1));
        
        --get_core_mode(mode_1, ID_1, mem_bus(1));
        --mode_1.self_test := true;
        --set_core_mode(mode_1, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- @2. Transmitt frame by Node 1.
        ------------------------------------------------------------------------
        info("Step 2: Transmit frame by Node 1");
        CAN_generate_frame(rand_ctr, frame_1);
        frame_1.rtr := NO_RTR_FRAME;
        frame_1.frame_format := NORMAL_CAN;
        frame_1.dlc := "0001";
        frame_1.data_length := 1;
        frame_1.identifier := 256;
        frame_1.data(0) := x"AA";      

        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);

        ------------------------------------------------------------------------
        -- @3.  Wait until error frame starts and check that error counter is
        --      incremented by 8. Repeat until node turns error passive!
        ------------------------------------------------------------------------
        info("Step 3: Looping till error passive");
        get_fault_state(fault_state_1, ID_1, mem_bus(1));
        while (fault_state_1 = fc_error_active) loop
            CAN_wait_error_frame(ID_1, mem_bus(1));
        
            -- Wait till error frame is for sure over
            for i in 0 to 13 loop
                CAN_wait_sample_point(iout(1).stat_bus);
            end loop;

            read_error_counters(err_counters, ID_1, mem_bus(1));
            retr_index := retr_index + 1;
            check(err_counters.tx_counter = retr_index * 8);
                        
            get_fault_state(fault_state_1, ID_1, mem_bus(1));
        end loop;

        ------------------------------------------------------------------------
        -- @4. Wait for several times that node transmitts a frame and check
        --     that after each, TX Error counter is not incremented.
        ------------------------------------------------------------------------
        info("Step 4: Checking TEC stays!");
        read_error_counters(err_counters, ID_1, mem_bus(1));
        tec_at_err_passive := err_counters.tx_counter;
        
        for i in 0 to 10 loop
            CAN_wait_error_frame(ID_1, mem_bus(1));
        
            -- Wait till error frame is for sure over
            for j in 0 to 13 loop
                CAN_wait_sample_point(iout(1).stat_bus);
            end loop;

            read_error_counters(err_counters, ID_1, mem_bus(1));
            check(err_counters.tx_counter = tec_at_err_passive);
        end loop;


    wait for 1000 ns;
  end procedure;

end package body;
