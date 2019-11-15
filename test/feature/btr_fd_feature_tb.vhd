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
--  BTR FD (Bit timing register - flexible data rate) feature test.
--
-- Verifies:
--  1. BTR FD register properly configures PROP, PH1, PH2 registers.
--  2. Transmission/reception at random bit-rate.
--
-- Test sequence:
--   1. Disable both Nodes. Generate random bit-rate and configure it sa Data
--      bit-rate! Nominal bit-rate remains the default one which was set by
--      testbench.
--   2. Enable both Nodes and send CAN FD frame where bit-rate is shifted by
--      Node 1. Wait until data field in Node 1 and measure duration till next
--      sample point! Transmitter during Data phase shall no re-synchronize
--      therefore, bit time will always have nominal length!
--   3. Wait until frame is sent and check it is correctly received by Node 2.
--------------------------------------------------------------------------------
-- Revision History:
--   11.11.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package btr_fd_feature is
    procedure btr_fd_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body btr_fd_feature is
    procedure btr_fd_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable CAN_frame_1        :       SW_CAN_frame_type;
        variable CAN_frame_2        :       SW_CAN_frame_type;
        variable CAN_frame_3        :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable mode               :       SW_mode := SW_mode_rst_val;
        variable rx_state           :       SW_RX_Buffer_info;
        
        variable fault_state_1      :       SW_fault_state;
        variable fault_state_2      :       SW_fault_state;
        
        variable read_state         :       SW_PC_Debug;
        variable status             :       SW_status;
        
        variable bus_timing         :       bit_time_config_type;

        variable clock_per_bit      :       natural := 0;

        variable clock_meas         :       natural := 0;
        variable frames_equal       :       boolean;
    begin
        o.outcome := true;

        -----------------------------------------------------------------------
        -- 1. Disable both Nodes. Generate random bit-rate and configure it sa 
        --    Data bit-rate! Nominal bit-rate remains the default one which was
        --    set by testbench.
        -----------------------------------------------------------------------
        info("Step 1");
        CAN_turn_controller(false, ID_1, mem_bus(1));
        CAN_turn_controller(false, ID_2, mem_bus(2));

        -- Read timing so that NBT values are kept!
        CAN_read_timing_v(bus_timing, ID_1, mem_bus(1));

        -- Generate random Nominal bit rate!
        rand_int_v(rand_ctr, 63, bus_timing.prop_dbt);
        rand_int_v(rand_ctr, 31, bus_timing.ph1_dbt);
        rand_int_v(rand_ctr, 31, bus_timing.ph2_dbt);
        
        -- Constrain time quanta to something realistinc for data phase so
        -- that we don't have too long run times!
        rand_int_v(rand_ctr, 16, bus_timing.tq_dbt);
        rand_int_v(rand_ctr, 33, bus_timing.sjw_dbt);

        -- SJW should be at least one because clocks differ by some value so
        -- there should be chance to compensate
        if (bus_timing.sjw_dbt = 0) then
            bus_timing.sjw_dbt := 1;
        end if;

        -- Constrain minimal BRP (0 is not allowed)!
        if (bus_timing.tq_dbt = 0) then
            bus_timing.tq_dbt := 1;
        end if;

        -- Pre-calculate expected number of clock cycles
        clock_per_bit := (1 + bus_timing.prop_dbt + bus_timing.ph1_dbt +
                          bus_timing.ph2_dbt) * bus_timing.tq_dbt;

        -- It has no sense to test bit times where there is less than 5 clocks per bit!
        -- lets constrain it to something reasonable.
        while (clock_per_bit < 6) loop

            if (bus_timing.prop_dbt < 127) then
                bus_timing.prop_dbt := bus_timing.prop_dbt + 1;
            end if;

            if (bus_timing.ph1_dbt < 63) then
                bus_timing.ph1_dbt := bus_timing.ph1_dbt + 1;
            end if;

            if (bus_timing.ph2_dbt < 63) then
                bus_timing.ph2_dbt := bus_timing.ph2_dbt + 1;
            end if;

            clock_per_bit := (1 + bus_timing.prop_dbt + bus_timing.ph1_dbt +
                              bus_timing.ph2_dbt) * bus_timing.tq_dbt;
        end loop;

        -- Constrain minimal duration of PH2 to be 2 clock cycles!
        if (bus_timing.ph2_dbt * bus_timing.prop_dbt < 2) then
            bus_timing.ph2_dbt := 2;
        end if;

        -- Constrain minimal duration of TSEG1 to be 2 clock cycles!
        if ((bus_timing.prop_dbt + bus_timing.ph1_dbt + 1) * bus_timing.tq_dbt < 2) then
            bus_timing.prop_dbt := 1;
        end if;

        -- Pre-calculate expected number of clock cycles after all corrections!
        clock_per_bit := (1 + bus_timing.prop_dbt + bus_timing.ph1_dbt +
                          bus_timing.ph2_dbt) * bus_timing.tq_dbt;

        CAN_configure_timing(bus_timing, ID_1, mem_bus(1));
        CAN_configure_timing(bus_timing, ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- 2. Enable both Nodes and send CAN FD frame where bit-rate is shifted
        --    by Node 1. Wait until data field in Node 1 and measure duration
        --    till next sample point! Transmitter during Data phase shall no
        --    re-synchronize therefore, bit time will always have nominal length!
        -----------------------------------------------------------------------
        info("Step 2");
        CAN_turn_controller(true, ID_1, mem_bus(1));
        CAN_turn_controller(true, ID_2, mem_bus(2));

        CAN_wait_bus_on(ID_1, mem_bus(1));
        CAN_wait_bus_on(ID_2, mem_bus(2));

        info("CAN bus nominal bit-rate:");
        info("PROP: " & integer'image(bus_timing.prop_nbt));
        info("PH1: " & integer'image(bus_timing.ph1_nbt));
        info("PH2: " & integer'image(bus_timing.ph2_nbt));
        info("SJW: " & integer'image(bus_timing.sjw_nbt));

        info("CAN bus Data bit-rate:");
        info("PROP: " & integer'image(bus_timing.prop_dbt));
        info("PH1: " & integer'image(bus_timing.ph1_dbt));
        info("PH2: " & integer'image(bus_timing.ph2_dbt));
        info("SJW: " & integer'image(bus_timing.sjw_dbt));

        CAN_generate_frame(rand_ctr, CAN_frame_1);
        CAN_frame_1.brs := BR_SHIFT;
        CAN_frame_1.frame_format := FD_CAN;
    
        -- Force DLC length to 1 byte only not to have long test run time!
        CAN_frame_1.dlc := "0001";
        decode_dlc(CAN_frame_1.dlc, CAN_frame_1.data_length);
        CAN_frame_1.data(0) := x"AA";
        CAN_frame_1.data(1) := x"BB";
        CAN_frame_1.data(2) := x"CC";
        CAN_frame_1.data(3) := x"DD";
    
        CAN_send_frame(CAN_frame_1, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_data, ID_1, mem_bus(1));

        CAN_wait_sample_point(iout(1).stat_bus, false);
        
        wait for 11 ns;

        -- Measure duration till next Sample point!
        while (iout(1).stat_bus(STAT_RX_TRIGGER) = '0') loop
            clock_meas := clock_meas + 1;
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;

        check(clock_per_bit = clock_meas,
            " Expected clock per bit: " & integer'image(clock_per_bit) &
            " Measured clock per bit: " & integer'image(clock_meas));        

        -----------------------------------------------------------------------
        -- 3. Wait until frame is sent and check it is correctly received by
        --    Node 2.
        -----------------------------------------------------------------------
        info("Step 3");
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        CAN_read_frame(CAN_frame_2, ID_2, mem_bus(2));

        CAN_compare_frames(CAN_frame_1, CAN_frame_2, false, frames_equal);
        check(frames_equal, "TX/RX frame equal!");

  end procedure;

end package body;