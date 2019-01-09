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
--  Feature test for Timestamp options on RX frame in RX Buffer!
--
--------------------------------------------------------------------------------
-- Revision History:
--
--    29.6.2018     Created file 
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package timestamp_options_feature is
    procedure timestamp_options_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
	);
end package;


package body timestamp_options_feature is
    procedure timestamp_options_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :        std_logic_vector(31 downto 0) :=
                                                 (OTHERS => '0');
        variable CAN_frame          :        SW_CAN_frame_type;
        variable frame_sent         :        boolean := false;
        variable options            :        SW_RX_Buffer_options;
        variable ts_beg             :        std_logic_vector(31 downto 0);
        variable ts_end             :        std_logic_vector(31 downto 0);
        variable ID_1               :        natural := 1;
        variable ID_2               :        natural := 2;
        variable diff               :        integer;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- If this is the only test, wait until controller will come out
        -- of integration phase!
        ------------------------------------------------------------------------
        for i in 0 to 1700 loop
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;

        ------------------------------------------------------------------------
        -- Configure timestamp options to the begining of CAN Frame.
        ------------------------------------------------------------------------
        options.rx_time_stamp_options := true;
        set_rx_buf_options(options, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Generate CAN Frame and start transmission, capture actual timestamp
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        ts_beg := iout(2).stat_bus(STAT_TS_LOW + 31 downto STAT_TS_LOW);

        ------------------------------------------------------------------------
        -- Wait until frame was send, read by node 2 and check that timestamp
        -- is very close to captured one from beginning...
        ------------------------------------------------------------------------      
        CAN_wait_frame_sent(ID_2, mem_bus(2));
        CAN_read_frame(CAN_frame, ID_2, mem_bus(2));
        diff := to_integer(unsigned(CAN_frame.timestamp(31 downto 0))) -
                to_integer(unsigned(ts_beg));

        ------------------------------------------------------------------------
        -- Bit time in default config is 160 clock cycles. Give some reserve.
        ------------------------------------------------------------------------  
        if (diff > 200) then
            -- LCOV_EXCL_START
            error("Timestamp difference is too big from SOF! " & 
                    integer'image(diff));
            o.outcome := false;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Configure timestamp options to the end of CAN Frame.
        ------------------------------------------------------------------------
        options.rx_time_stamp_options := false;
        set_rx_buf_options(options, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Generate CAN Frame and start transmission
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        ------------------------------------------------------------------------
        -- Wait until frame was send, read by node 2 and check that timestamp
        -- is very close to captured one from beginning...
        ------------------------------------------------------------------------      
        CAN_wait_frame_sent(ID_2, mem_bus(2));
        ts_end := iout(2).stat_bus(STAT_TS_LOW + 31 downto STAT_TS_LOW);
        wait until rising_edge(mem_bus(2).clk_sys);
        wait until rising_edge(mem_bus(2).clk_sys);
        wait until rising_edge(mem_bus(2).clk_sys);

        CAN_read_frame(CAN_frame, ID_2, mem_bus(2));
        diff := to_integer(unsigned(ts_end)) -
                to_integer(unsigned(CAN_frame.timestamp(31 downto 0)));

        ------------------------------------------------------------------------
        -- Timestamp is taken in EOF. CAN_wait_frame_sent is exited after
        -- intermission, when controller is in idle! Thus there are 3
        -- extra bits of difference in timestamp!
        ------------------------------------------------------------------------
        if (diff > 600) then
            -- LCOV_EXCL_START
            error("Timestamp difference is too big from EOF!" &
                   integer'image(diff));
            o.outcome := false;
            -- LCOV_EXCL_STOP
        end if;

    end procedure;

end package body;
