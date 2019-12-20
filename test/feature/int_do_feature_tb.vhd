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
--  Interrupt DOI feature test.
--
-- Verifies:
--  1. DO Interrupt is set when there is an attempt to write to full RX FIFO.
--  2. DO Interrupt is not set when there is a write to RX FIFO which just
--     fills the last words in the FIFO.
--  3. DO Interrupt is not set when it is masked.
--  4. DO Interrupt causes INT to go high when it is enabled.
--  5. DO Interrupt causes INT to go low when it is disabled.
--  6. DO Interrupt is cleared by write to INT_STATUS register.
--  7. DO Interrupt enable is manipulated properly by INT_ENA_SET and
--     INT_ENA_CLEAR.
--  8. DO Interrupt mask is manipulated properly by INT_MASK_SET and
--     INT_MASK_CLEAR.
--
-- Test sequence:
--  1. Unmask and enable DO Interrupt, disable and mask all other interrupts on
--     Node 1.
--  2. Read RX Buffer size of Node 1 and send number of RTR frames by Node 2 
--     which will just fill RX Buffer of Node (no overrun yet). Check DO
--     Interrupt is not set and INT pin is low after each frame.
--  3. Send one more frame which should cause Data overrun by Node 2.
--     Check that DO Interrupt is set. Check that INT pin is high.
--  4. Disable DO Interrupt and check INT pin goes low. Enable DO Interrupt
--     and check INT pin goes high.
--  5. Clear DO Interrupt and check it is still set (Data overrun flag is still
--     set).
--  6. Clear Data overrun flag, clear DO Interrupt. Check DO Interrupt is
--     cleared. Check INT pin goes low.
--  7. Mask DO Interrupt. Send one frame by Node 2. Check that DO Interrupt
--     is not set. Check that INT pin is low. Check that Data overrun flag (DOR)
--     is set (overrun really occurred).
--  8. Disable DO Interrupt and check it was disabled. Enable DO Interrupt and
--     check it was enabled.
--  9. Mask DO Interrupt and check it was masked. Un-mask DO Interrupt and
--     check it was un-masked.
--------------------------------------------------------------------------------
-- Revision History:
--    1.7.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package int_do_feature is
    procedure int_do_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body int_do_feature is
    procedure int_do_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable CAN_frame          :     SW_CAN_frame_type;
        variable CAN_frame_rx       :     SW_CAN_frame_type;
        variable frame_sent         :     boolean := false;
        variable frames_equal       :     boolean := false;
        variable size_of_buf        :     natural;
        variable ctr_1              :     natural;
        variable ctr_2              :     natural;
        variable ID_1           	:     natural := 1;
        variable ID_2           	:     natural := 2;
        variable vect_1             :     std_logic_vector(31 downto 0);
        variable vect_2             :     std_logic_vector(31 downto 0);
        variable mode_prev          :     std_logic_vector(31 downto 0);
        variable mode_prev_2        :     std_logic_vector(31 downto 0);

        variable int_mask           :     SW_interrupts := SW_interrupts_rst_val;
        variable int_ena            :     SW_interrupts := SW_interrupts_rst_val;
        variable int_stat           :     SW_interrupts := SW_interrupts_rst_val;
        variable command            :     SW_command := SW_command_rst_val;
        variable mode               :     SW_mode := SW_mode_rst_val;
        variable buf_info           :     SW_RX_Buffer_info;
        variable pc_dbg             :     SW_PC_Debug;
        variable err_ctrs           :     SW_error_counters;
        variable status             :     SW_status;
    begin


        -----------------------------------------------------------------------
        -- 1. Unmask and enable DO Interrupt, disable and mask all other 
        --    interrupts on Node 1.
        -----------------------------------------------------------------------
        info("Step 1: Setting DO Interrupt");
        int_mask.data_overrun_int := false;
        int_ena.data_overrun_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        write_int_enable(int_ena, ID_1, mem_bus(1));

        -----------------------------------------------------------------------
        -- 2. Read RX Buffer size of Node 1 and send number of RTR frames by 
        --    Node 2 which will just fill RX Buffer of Node 1.
        -----------------------------------------------------------------------
        info("Step 2: Filling RX Buffer FIFO");
        get_rx_buf_state(buf_info, ID_1, mem_bus(1));
        info("Buffer size: " & Integer'image(buf_info.rx_buff_size));
        
        -- Send RTR frames till we fill the buffer
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.rtr := RTR_FRAME;
        CAN_frame.frame_format := NORMAL_CAN;
        for i in 0 to (buf_info.rx_buff_size / 4) - 1 loop
            CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
            CAN_wait_frame_sent(ID_2, mem_bus(2));

            read_int_status(int_stat, ID_1, mem_bus(1));
            check_false(int_stat.data_overrun_int,
                "DO Interrupt not set when filling FIFO!");
            check(iout(1).irq = '0', "INT pin should be low!");
        end loop;
        
        -----------------------------------------------------------------------
        -- 3. Send one more frame which should cause Data overrun by Node 2.
        --    Check that DO Interrupt is set. Check that INT pin is high.
        -----------------------------------------------------------------------
        info("Step 3: Overruning RX Buffer FIFO");   
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_2, mem_bus(2));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check(int_stat.data_overrun_int, "DO Interrupt set after filling FIFO!");
        check(iout(1).irq = '1', "INT pin should be high!");

        -----------------------------------------------------------------------
        -- 4. Disable DO Interrupt and check INT pin goes low. Enable DO
        --    Interrupt and check INT pin goes high.
        -----------------------------------------------------------------------
        info("Step 4: Check DO Interrupt toggles INT pin");
        int_ena.data_overrun_int := false;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        check(iout(1).irq = '0', "INT pin should be low!");
        int_ena.data_overrun_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        check(iout(1).irq = '1', "INT pin should be high!");

        -----------------------------------------------------------------------
        -- 5. Clear DO Interrupt and check it is still set (Data Overrun flag 
        --    is still set).
        -----------------------------------------------------------------------
        info("Step 5: Clear DO Interrupt - DOR Flag Set.");
        int_stat.data_overrun_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        int_stat.data_overrun_int := false;
        read_int_status(int_stat, ID_1, mem_bus(1));
        check(int_stat.data_overrun_int, "DO Interrupt still set after clear!");
        check(iout(1).irq = '1', "INT pin should be high!");     
        
        -----------------------------------------------------------------------
        -- 6. Clear Data overrun flag, clear DO Interrupt. Check DO Interrupt
        --    is cleared. Check INT pin goes low.
        -----------------------------------------------------------------------
        info("Step 6: Clear DO Interrupt - DOR Flag Cleared.");
        command.clear_data_overrun := true;
        give_controller_command(command, ID_1, mem_bus(1));
        command.clear_data_overrun := false;
        int_stat.data_overrun_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.data_overrun_int, "DO Interrupt cleared!");
        check(iout(1).irq = '0', "INT pin should be low!");     
        
        -----------------------------------------------------------------------
        -- 7. Mask DO Interrupt. Send one frame by Node 2. Check that DO 
        --    Interrupt is not set. Check that INT pin is low. Check that 
        --    Data overrun flag (DOR) is set (overrun really occurred).
        -----------------------------------------------------------------------
        info("Step 7: DO Interrupt not set when masked!");
        int_mask.data_overrun_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_frame_sent(ID_2, mem_bus(2));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.data_overrun_int, "DO Interrupt not set when masked!");
        check(iout(1).irq = '0', "INT pin should be low!");
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.data_overrun, "DOR flag not set!");
        
        -- Clear DOR flag again.
        command.clear_data_overrun := true;
        give_controller_command(command, ID_1, mem_bus(1));
        command.clear_data_overrun := false;
        
        -----------------------------------------------------------------------
        -- 8. Disable DO Interrupt and check it was disabled. Enable DO 
        --    Interrupt and check it was enabled.
        -----------------------------------------------------------------------
        info("Step 8: Check DO Interrupt enable works OK!");
        int_ena.data_overrun_int := false;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        int_ena.data_overrun_int := true;
        read_int_enable(int_ena, ID_1, mem_bus(1));
        check_false(int_ena.data_overrun_int, "DO Interrupt enabled!");

        int_ena.data_overrun_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        int_ena.data_overrun_int := false;
        read_int_enable(int_ena, ID_1, mem_bus(1));
        check(int_ena.data_overrun_int, "DO Interrupt disabled!");

        -----------------------------------------------------------------------
        -- 9. Mask DO Interrupt and check it was masked. Un-mask DO Interrupt 
        --    and check it was un-masked.
        -----------------------------------------------------------------------
        info("Step 9: Check DO Interrupt mask works OK!");
        int_mask.data_overrun_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        int_mask.data_overrun_int := false;
        read_int_mask(int_mask, ID_1, mem_bus(1));
        check(int_mask.data_overrun_int, "DO Interrupt masked!");

        int_mask.data_overrun_int := false;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        int_mask.data_overrun_int := true;
        read_int_mask(int_mask, ID_1, mem_bus(1));
        check_false(int_mask.data_overrun_int, "DO Interrupt masked!");

        info("Finished DO interrupt test");
        wait for 1000 ns;

    end procedure;
end package body;