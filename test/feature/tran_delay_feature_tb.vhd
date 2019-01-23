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
-- Purpose:
--  Transceiver delay compensation feature test.
--
--  Test sequence is like so:
--    1. Generate random CAN FD Frame with BRS.
--    2. Send frame on the bus.
--    3. Wait unil frame is sent.
--    4. Read value of transceiver delay and check if it is matching constant
--       value set in environment.
--
--------------------------------------------------------------------------------
-- Revision History:
--
--    28.6.2016   Created file
--    12.6.2018   Changed to use CAN test lib instead of direct register access.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package tran_delay_feature is
    procedure tran_delay_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );

end package;


package body tran_delay_feature is
    procedure tran_delay_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable w_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable delay              :       natural;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Generate CAN frame
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.rtr := NO_RTR_FRAME;
        CAN_frame.frame_format := FD_CAN;
        CAN_frame.brs := BR_SHIFT;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Read the transciever delay compensation register
        ------------------------------------------------------------------------
        read_trv_delay(delay, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Check if delay is matching environment transceiver delay...
        -- Note, that 2 DFFs sync. chain must be taken into account
        -- Feature TB has delay hardcoded to 20!
        ------------------------------------------------------------------------
        check(delay = 22, "Transceiver delay not measured as expected: " &
				integer'image(delay));

  end procedure;

end package body;