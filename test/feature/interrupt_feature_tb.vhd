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
--  Interrupt generation feature test
--
--  Test sequence is like so:
--    1. Part 1:
--          1.1 Set TX Interrupt on Node 2, RX interrupt on Node 1
--          1.2 Send CAN Frame by Node 2
--          1.3 Observe that interrupt occurred
--    2. Part 2:
--          2.1 Set Interrupt on Error frame on both nodes.
--          2.2 Send frames with collision and detect that collision ocurred.
--    3. Part 3:
--          3.1 Set Interrupt on RX Buffer full and Data overrun.
--          3.2 Send frames with 4 words and wait till interrupt ocurrs.
--          3.3 Check if interrupts were generated.
--    4. Part 4:
--          4.1 Set Bit rate shift interrupt on both nodes.
--          4.2 Send CAN FD Frame with BRS bit set
--          4.3 Observe interrupts and check that Interrupt is captured in
--              interrupt vector
--    5. Part 5:
--          5.1 Set Arbitration lost interrupt
--          5.2 Insert frames with mismatching IDs to each node
--          5.3 Observe Arbitration lost interrupt on Node with higher value
--              of identifier.
--------------------------------------------------------------------------------
-- Revision History:
--
--    27.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--     11.6.2018  Modified to use HAL access functions from CAN Test lib instead
--                of direct register access.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package interrupt_feature is
    procedure interrupt_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body interrupt_feature is
    procedure interrupt_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable CAN_frame          :     SW_CAN_frame_type;
        variable frame_sent         :     boolean := false;
        variable size_of_buf        :     natural;
        variable ctr_1              :     natural;
        variable ctr_2              :     natural;
        variable ID_1           	:     natural := 1;
        variable ID_2           	:     natural := 2;
        variable vect_1             :     std_logic_vector(31 downto 0);
        variable vect_2             :     std_logic_vector(31 downto 0);
        variable mode_prev          :     std_logic_vector(31 downto 0);
        variable mode_prev_2        :     std_logic_vector(31 downto 0);

        variable int_mask           :     SW_interrupts := (false, false, false,
                                            false, false, false, false, false,
                                            false, false, false, false);
        variable int_ena            :     SW_interrupts := (false, false, false,
                                            false, false, false, false, false,
                                            false, false, false, false);
        variable int_stat           :     SW_interrupts := (false, false, false,
                                            false, false, false, false, false,
                                            false, false, false, false);
        variable command            :     SW_command := (false, false, false);
        variable buf_info           :     SW_RX_Buffer_info;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Unmask all Interrupts. Should be by default, but rather do it...
        ------------------------------------------------------------------------
        write_int_mask(int_mask, ID_1, mem_bus(1));
        write_int_mask(int_mask, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Part 1
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Recieve INT node 1, TX int node 2
        ------------------------------------------------------------------------
        info("Starting TX RX interrupt");
        int_ena.receive_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        int_ena.receive_int := false;

        int_ena.transmitt_int := true;
        write_int_enable(int_ena, ID_2, mem_bus(2));
        int_ena.transmitt_int := false;

        ------------------------------------------------------------------------
        -- Send by node 2
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);

        wait until rising_edge(iout(1).irq) or rising_edge(iout(2).irq);
        wait until rising_edge(iout(1).irq) or rising_edge(iout(2).irq);

        CAN_wait_frame_sent(ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Check that interrupt was generated
        ------------------------------------------------------------------------
        read_int_status(int_stat, ID_1, mem_bus(1));
        if (not int_stat.receive_int) then
            -- LCOV_EXCL_START
            o.outcome := false;
            error("RX Interrupt not present!");
            -- LCOV_EXCL_STOP
        end if;
        clear_int_status(int_stat, ID_1, mem_bus(1));

        read_int_status(int_stat, ID_2, mem_bus(2));
        if (not int_stat.transmitt_int) then
            o.outcome := false;
        end if;
        clear_int_status(int_stat, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- Part 2
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Error Interrupt both nodes
        ------------------------------------------------------------------------
        info("Starting Error interrupt");
        int_ena.bus_error_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        write_int_enable(int_ena, ID_2, mem_bus(2));
        int_ena.bus_error_int := false;

        ------------------------------------------------------------------------
        -- Send conflicting frames
        ------------------------------------------------------------------------
        CAN_frame.data(0) := x"AB";
        CAN_frame.rtr := NO_RTR_FRAME;
        CAN_insert_TX_frame(CAN_frame, 1, ID_2, mem_bus(2));
        CAN_frame.data(0) := x"CD";
        CAN_insert_TX_frame(CAN_frame, 1, ID_1, mem_bus(1));

        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));

        wait until rising_edge(iout(1).irq) or rising_edge(iout(2).irq);
        wait until rising_edge(iout(1).irq) or rising_edge(iout(2).irq);

        ------------------------------------------------------------------------
        -- Detect interrupt error flag
        ------------------------------------------------------------------------
        read_int_status(int_stat, ID_1, mem_bus(1));
        if (not int_stat.bus_error_int) then
            -- LCOV_EXCL_START
            o.outcome := false;
            error("Bus error Interrput not present (Node 1)");
            -- LCOV_EXCL_STOP
        end if;
        clear_int_status(int_stat, ID_1, mem_bus(1));

        read_int_status(int_stat, ID_2, mem_bus(2));
        if (not int_stat.bus_error_int) then
            -- LCOV_EXCL_START
            error("Bus error Interrupt no present (Node 2)");
            o.outcome := false;
            -- LCOV_EXCL_STOP
        end if;
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        clear_int_status(int_stat, ID_2, mem_bus(2));
        wait for 15000 ns;


        ------------------------------------------------------------------------
        -- Part 3
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Data overrun interrupt and recieve buffer full interrupt node 2
        ------------------------------------------------------------------------
        info("Starting Data overrun, recieve buffer interrupt");
        int_ena.data_overrun_int := true;
        int_ena.rx_buffer_full_int := true;
        write_int_enable(int_ena, ID_2, mem_bus(2));
        int_ena.data_overrun_int := false;
        int_ena.rx_buffer_full_int := false;

        -- Give release receive buffer command
        command.release_rec_buffer := true;
        give_controller_command(command, ID_2, mem_bus(2));
        command.release_rec_buffer := false;

        ------------------------------------------------------------------------
        -- Size of buffer 2
        -- Note that size of RTR is 4. Each synthesizable size of buffer is
        -- multiple of 4!
        ------------------------------------------------------------------------
        get_rx_buf_state(buf_info, ID_2, mem_bus(2));

        --report "Buffer size: " & Integer'image(buf_info.rx_buff_size);

        -- Send RTR frames till we fill the buffer
        CAN_frame.rtr := RTR_FRAME;
        CAN_frame.frame_format := NORMAL_CAN;
        for i in 0 to (buf_info.rx_buff_size / 4) + 1 loop
            CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

            CAN_wait_frame_sent(ID_1, mem_bus(1));

            -- On last frame RX Buffer should be full. Check if interrupt was
            -- fired and clear it!
            if (i = (buf_info.rx_buff_size / 4)) then
                if (iout(2).irq = '0') then
                    -- LCOV_EXCL_START
                    error("RX Buffer Full interrupt is not active!");
                    o.outcome := false;
                    -- LCOV_EXCL_STOP
                else
                    read_int_status(int_stat, ID_2, mem_bus(2));
                    clear_int_status(int_stat, ID_2, mem_bus(2));
                end if;
            end if;
        end loop;


        ------------------------------------------------------------------------
        -- Detect the data overrun interrupt flag and recieve buffer full flag
        ------------------------------------------------------------------------
        read_int_status(int_stat, ID_2, mem_bus(2));

        if (not int_stat.rx_buffer_full_int) then
            -- LCOV_EXCL_START
            o.outcome := false;
            error("RX Buffer Full Interrupt not present!");
            -- LCOV_EXCL_STOP
        end if;

        if (not int_stat.data_overrun_int) then
            -- LCOV_EXCL_START
            o.outcome := false;
            error("Data overrun Interrupt not present!");
            -- LCOV_EXCL_STOP
        end if;
        clear_int_status(int_stat, ID_2, mem_bus(2));
        wait for 30000 ns;


        ------------------------------------------------------------------------
        -- Part 4
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Bit rate shift interrupt on both nodes
        ------------------------------------------------------------------------
        info("Starting Bit rate shift interrupt");

        int_ena.bit_rate_shift_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        write_int_enable(int_ena, ID_2, mem_bus(2));
        int_ena.bit_rate_shift_int := false;

        CAN_frame.frame_format := FD_CAN;
        CAN_frame.rtr := NO_RTR_FRAME;
        CAN_frame.brs := BR_SHIFT;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        ------------------------------------------------------------------------
        -- Wait on bit rate shift
        ------------------------------------------------------------------------
        wait until rising_edge(iout(2).irq);

        ------------------------------------------------------------------------
        -- Detect the Bit rate shift interrupt flag
        ------------------------------------------------------------------------
        read_int_status(int_stat, ID_2, mem_bus(2));
        if (not int_stat.bit_rate_shift_int) then
            -- LCOV_EXCL_START
            o.outcome := false;
            error("Bit Rate shift interrupt not present");
            -- LCOV_EXCL_STOP
        end if;
        CAN_wait_frame_sent(ID_2,mem_bus(2));
        clear_int_status(int_stat, ID_2, mem_bus(2));

        read_int_status(int_stat, ID_1, mem_bus(1));
        clear_int_status(int_stat, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Part 5
        ------------------------------------------------------------------------
        ------------------------------------------------------------------------
        -- Arbitration lost interrupt in node 1
        ------------------------------------------------------------------------
        info("Starting arbitration lost int");
        int_ena.arb_lost_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Send frames by both nodes with IDs fabricated so that Node 1 loses.
        ------------------------------------------------------------------------
        CAN_frame.frame_format := NORMAL_CAN;
        CAN_frame.rtr := NO_RTR_FRAME;
        CAN_frame.brs := BR_NO_SHIFT;
        CAN_frame.ident_type := EXTENDED;
        CAN_frame.identifier := 5;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_frame.identifier := 4;
        CAN_send_frame(CAN_frame, 2, ID_2, mem_bus(2), frame_sent);
        wait until rising_edge(iout(1).irq);

        read_int_status(int_stat, ID_1, mem_bus(1));
        if (not int_stat.arb_lost_int) then
            -- LCOV_EXCL_START
            o.outcome := false;
            error("Arbitration Lost Interrupt not present!");
            -- LCOV_EXCL_STOP
        end if;
        clear_int_status(int_stat, ID_1, mem_bus(1));

        -- Send abort command on node that lost arbitration so that it does
        -- not try to transmitt again.
        send_TXT_buf_cmd(buf_set_abort, 1, ID_1, mem_bus(1));

        -- Wait till transmission is done
        CAN_wait_frame_sent(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Clear all interrupts in both nodes
        ------------------------------------------------------------------------
        read_int_status(int_stat, ID_1, mem_bus(1));
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_2, mem_bus(2));
        clear_int_status(int_stat, ID_2, mem_bus(2));

        info("Finished interrupt test");
        wait for 300000 ns;

    end procedure;

end package body;
