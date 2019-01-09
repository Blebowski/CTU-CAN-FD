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
--  Feature test for retransmitt limitation
--
--------------------------------------------------------------------------------
-- Revision History:
--    30.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--    12.06.2018  Modified to use CAN Test lib instead of direct register
--                access functions.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package retr_limit_feature is
    procedure retr_limit_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body retr_limit_feature is
    procedure retr_limit_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ctr_1              :       natural;
        variable ctr_2              :       natural;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable rand_val           :       real;
        variable retr_th            :       natural;
        variable mode_backup        :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');

        variable mode               :       SW_mode := (false, false, false,
                                                false, true, false, false,
                                                false, false, false);
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable buf_state          :       SW_TXT_Buffer_state_type;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Set both nodes to forbid acknowledge
        ------------------------------------------------------------------------
        mode.acknowledge_forbidden := true;
        set_core_mode(mode, ID_2, mem_bus(2));
        set_core_mode(mode, ID_1, mem_bus(1));
        mode.acknowledge_forbidden := false;

        ------------------------------------------------------------------------
        -- Erase error counters node 1
        ------------------------------------------------------------------------
        set_error_counters(err_counters, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Set Node 1 retransmitt limit
        ------------------------------------------------------------------------
        rand_int_v(rand_ctr, 15, retr_th);
        info("Retransmitt threshold: " & Integer'image(retr_th));
        CAN_enable_retr_limit(true, retr_th, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Generate and send frame by Node 1
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.rtr := RTR_FRAME;
        CAN_frame.frame_format := NORMAL_CAN;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        ------------------------------------------------------------------------
        -- Wait number of retransmissions. After each one, TXT Buffer should
        -- be back in ready. After last one, it should be in failed.
        ------------------------------------------------------------------------
        for i in 0 to retr_th loop
            CAN_wait_frame_sent(ID_1, mem_bus(1));
            get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
            if (i /= retr_th) then
                if (buf_state /= buf_ready) then
                    -- LCOV_EXCL_START
                    error("TXT Buffer not ready");
                    o.outcome := false;
                    exit;
                    -- LCOV_EXCL_STOP
                end if;
            else
                if (buf_state /= buf_failed) then
                    -- LCOV_EXCL_START
                    error("TXT Buffer not failed");
                    o.outcome := false;
                    -- LCOV_EXCL_STOP
                end if;
            end if;
        end loop;

        ------------------------------------------------------------------------
        -- Read TX Counter, it should be equal to 8 times number of retransmitts
        -- plus one original transmittion does not count as retransmittion.
        ------------------------------------------------------------------------
        read_error_counters(err_counters, ID_1, mem_bus(1));
        if (err_counters.tx_counter /= 8 * (retr_th + 1)) then
            -- LCOV_EXCL_START
            error("Counters exp: " & Integer'Image(err_counters.tx_counter) &
                  " counters real: " & Integer'image(8 * (retr_th + 1)));
            o.outcome := false;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Set node  2 to allow acknowledge again
        ------------------------------------------------------------------------
        set_core_mode(mode, ID_2, mem_bus(2));
        set_core_mode(mode, ID_1, mem_bus(1));

        wait for 40000 ns;
  end procedure;

end package body;
