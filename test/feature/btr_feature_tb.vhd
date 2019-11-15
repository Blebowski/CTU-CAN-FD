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
--  BTR (Bit timing register) feature test.
--
-- Verifies:
--  1. BTR register properly configures PROP, PH1, PH2 registers.
--  2. Transmission/reception at random bit-rate.
--
-- Test sequence:
--   1. Disable both Nodes. Generate random bit-rate and configure it sa Nominal
--      bit-rate! Enable both nodes and wait till both nodes are on.
--   2. Wait until sample point in Node 1 and measure number of clock cycles
--      till next sample point. Check that it corresponds to pre-computed value!
--   4. Send frame by Node 1 and wait until it is sent. Read frame from Node 2
--      and check they are matching. Do vice-versa from Node 2 to Node 1!
--------------------------------------------------------------------------------
-- Revision History:
--   11.11.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package btr_feature is
    procedure btr_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body btr_feature is
    procedure btr_feature_exec(
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

    begin
        o.outcome := true;

        -----------------------------------------------------------------------
        -- 1. Disable both Nodes. Generate random bit-rate and configure it sa 
        --    Nominal bit-rate! Enable both nodes and wait till both nodes are 
        --    on.
        -----------------------------------------------------------------------
        info("Step 1");
        CAN_turn_controller(false, ID_1, mem_bus(1));
        CAN_turn_controller(false, ID_2, mem_bus(2));

        -- Generate random Nominal bit rate!
        rand_int_v(rand_ctr, 127, bus_timing.prop_nbt);
        rand_int_v(rand_ctr, 63, bus_timing.ph1_nbt);
        rand_int_v(rand_ctr, 63, bus_timing.ph2_nbt);
        rand_int_v(rand_ctr, 63, bus_timing.tq_nbt);
        rand_int_v(rand_ctr, 63, bus_timing.sjw_nbt);

        -----------------------------------------------------------------------
        -- Leave Data bit rate unconfigured! This will result in XXXs in register
        -- map, and XXXs in prescaler (we will see about warnings), but in this
        -- test we will NOT use data bit-rate, therefore all logic operating
        -- on XXXs should be ignored! So it will be a nice experiment that
        -- even crazy things in DBT does not ruin NBT operation...
        -- This test will never have chance to pass at any "X pesimistic"
        -- simulation. But who has money for X pessimism simulator? :(
        -----------------------------------------------------------------------

        -- SJW should be at least one because clocks differ by some value so
        -- there should be chance to compensate
        if (bus_timing.sjw_nbt = 0) then
            bus_timing.sjw_nbt := 1;
        end if;

        -- Constrain minimal BRP (0 is not allowed)!
        if (bus_timing.tq_nbt = 0) then
            bus_timing.tq_nbt := 1;
        end if;

        -- Pre-calculate expected number of clock cycles
        clock_per_bit := (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt +
                          bus_timing.ph2_nbt) * bus_timing.tq_nbt;

        -- It has no sense to test frequencies above 5 MHz for Nominal Bit-rate
        -- lets constrain it to something reasonable.
        while (clock_per_bit < 20) loop

            if (bus_timing.prop_nbt < 127) then
                bus_timing.prop_nbt := bus_timing.prop_nbt + 1;
            end if;

            if (bus_timing.ph1_nbt < 63) then
                bus_timing.ph1_nbt := bus_timing.ph1_nbt + 1;
            end if;

            if (bus_timing.ph2_nbt < 63) then
                bus_timing.ph2_nbt := bus_timing.ph2_nbt + 1;
            end if;

            clock_per_bit := (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt +
                              bus_timing.ph2_nbt) * bus_timing.tq_nbt;
        end loop;

        -- Constrain minimal duration of PH2 to be 2 clock cycles!
        if (bus_timing.ph2_nbt * bus_timing.prop_nbt < 2) then
            bus_timing.ph2_nbt := 2;
        end if;

        -- Constrain minimal duration of TSEG1 to be 2 clock cycles!
        if ((bus_timing.prop_nbt + bus_timing.ph1_nbt + 1) * bus_timing.tq_nbt < 2) then
            bus_timing.prop_nbt := 1;
        end if;

        -- Pre-calculate expected number of clock cycles after all corrections!
        clock_per_bit := (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt +
                          bus_timing.ph2_nbt) * bus_timing.tq_nbt;

        CAN_configure_timing(bus_timing, ID_1, mem_bus(1));

        CAN_turn_controller(true, ID_1, mem_bus(1));
        CAN_turn_controller(true, ID_2, mem_bus(2));

        CAN_wait_bus_on(ID_1, mem_bus(1));
        CAN_wait_bus_on(ID_2, mem_bus(2));

        info("CAN bus nominal bit-rate:");
        info("PROP: " & integer'image(bus_timing.prop_nbt));
        info("PH1: " & integer'image(bus_timing.ph1_nbt));
        info("PH2: " & integer'image(bus_timing.ph2_nbt));
        info("SJW: " & integer'image(bus_timing.sjw_nbt));

        -----------------------------------------------------------------------
        -- 2. Wait until sample point in Node 1 and measure number of clock
        --    cycles till next sample point. Check that it corresponds to
        --    pre-computed value!
        -----------------------------------------------------------------------
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 11 ns;

        while (iout(1).stat_bus(STAT_RX_TRIGGER) = '0') loop
            clock_meas := clock_meas + 1;
            wait until rising_edge(mem_bus(1).clk_sys);
        end loop;

        check(clock_per_bit = clock_meas,
            " Expected clock per bit: " & integer'image(clock_per_bit) &
            " Measured clock per bit: " & integer'image(clock_meas));

        -----------------------------------------------------------------------
        -- 4. Send frame by Node 1 and wait until it is sent. Read frame from
        --    Node 2 and check they are matching. Do vice-versa from Node 2 to
        --    Node 1!
        -----------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame_1);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_2, mem_bus(2));
        CAN_read_frame(CAN_frame_2, ID_2, mem_bus(2));

        CAN_compare_frames(CAN_frame_1, CAN_frame_2, frames_equal);
        check(frames_equal, "TX/RX frame equal!");

  end procedure;

end package body;