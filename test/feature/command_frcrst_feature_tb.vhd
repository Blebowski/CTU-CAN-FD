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
-- @TestInfoStart
--
-- @Purpose:
--  TX/RX frame counters clear command.
--
-- @Verifies:
--  @1. TX Frame counter is cleared by COMMAND[TXRFCRST].
--  @2. TX Frame counter is not cleared by COMMAND[TXFRCRST].
--  @3. RX Frame counter is cleared by COMMAND[RXRFCRST].
--  @4. RX Frame counter is not cleared by COMMAND[RXFRCRST].
--
-- @Test sequence:
--  @1. Generate and send frame by Node 2. Check that TX frame counter of Node 2
--      is not zero. Issue COMMAND[RXFRCRST] and check it is still not 0. Issue
--      COMMAND[TXFRCRST] and check it is 0 now.
--  @2. Check that RX Frame counter of Node 1 is not zero. Issue COMMAND[TXFRCRST]
--      and check it is still not 0. Issue COMMAND[RXFRCRST] and RX Frame counter
--      in Node 1 is 0.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    25.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package command_frcrst_feature is
    procedure command_frcrst_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body command_frcrst_feature is
    procedure command_frcrst_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;

        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        variable command            :     SW_command := SW_command_rst_val;
        
        variable traff_ctrs_1       :     SW_traffic_counters;
        variable traff_ctrs_2       :     SW_traffic_counters;

    begin

        -----------------------------------------------------------------------
        -- @1. Generate and send frame by Node 2. Check that TX frame counter of
        --    Node 2 is not zero. Issue COMMAND[RXFRCRST] and check it is still
        --    not 0. Issue COMMAND[TXFRCRST] and check it is 0 now.
        -----------------------------------------------------------------------
        info("Step 1");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_insert_TX_frame(frame_1, 1, ID_2, mem_bus(2));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));

        CAN_wait_frame_sent(ID_1, mem_bus(1));

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        read_traffic_counters(traff_ctrs_2, ID_2, mem_bus(2));
        check(traff_ctrs_2.tx_frames /= 0, "TX frame counter not 0!");

        command.rx_frame_ctr_rst := true;
        give_controller_command(command, ID_2, mem_bus(2));

        read_traffic_counters(traff_ctrs_2, ID_2, mem_bus(2));
        check(traff_ctrs_2.tx_frames /= 0, "TX frame counter not 0 again!");

        command.tx_frame_ctr_rst := true;
        command.rx_frame_ctr_rst := false;
        give_controller_command(command, ID_2, mem_bus(2));
        
        read_traffic_counters(traff_ctrs_2, ID_2, mem_bus(2));
        check(traff_ctrs_2.tx_frames = 0, "TX frame counter erased!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- @2. Check that RX Frame counter of Node 1 is not zero. Issue
        --    COMMAND[TXFRCRST] and check it is still not 0. Issue 
        --    COMMAND[RXFRCRST] and RX Frame counter in Node 1 is 0. 
        -----------------------------------------------------------------------
        info("Step 2");
        
        read_traffic_counters(traff_ctrs_1, ID_1, mem_bus(1));
        info("Node 1");
        info("RX frames: " & integer'image(traff_ctrs_1.rx_frames));
        info("TX frames: " & integer'image(traff_ctrs_1.tx_frames));
        check(traff_ctrs_1.rx_frames /= 0, "RX frame counter not 0!");
        
        command.tx_frame_ctr_rst := true;
        command.rx_frame_ctr_rst := false;
        give_controller_command(command, ID_1, mem_bus(1));
        
        read_traffic_counters(traff_ctrs_1, ID_1, mem_bus(1));
        check(traff_ctrs_1.rx_frames /= 0, "RX frame counter not 0 again!");
        
        command.tx_frame_ctr_rst := false;
        command.rx_frame_ctr_rst := true;
        give_controller_command(command, ID_1, mem_bus(1));
        
        read_traffic_counters(traff_ctrs_1, ID_1, mem_bus(1));
        check(traff_ctrs_1.rx_frames = 0, "RX frame counter erased!");

        wait for 100 ns;

  end procedure;

end package body;