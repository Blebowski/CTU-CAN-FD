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
--  RX Buffer empty - read feature test
--
-- Verifies:
--  1. Verifies that when reading from empty RX Buffer, RX pointer is not
--     incremented!
--
-- Test sequence:
--  1. Read pointers from RX Buffer, check pointers are 0 (DUT is post reset).
--  2. Try to read CAN frame from RX Buffer. This should generate at least 4
--     reads from RX DATA register.
--  3. Read pointers from RX Buffer, check pointers are still 0.
--
--------------------------------------------------------------------------------
-- Revision History:
--    18.10.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package rx_buf_empty_read_feature is
    procedure rx_buf_empty_read_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body rx_buf_empty_read_feature is
    procedure rx_buf_empty_read_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable rand_value         :       real;
        variable alc                :       natural;

        -- Some unit lost the arbitration...
        -- 0 - initial , 1-Node 1 turned rec, 2 - Node 2 turned rec
        variable unit_rec           :     natural := 0;

        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                               (OTHERS => '0');
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_2            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        
        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;        

        variable id_vect            :     std_logic_vector(28 downto 0);
    begin
        o.outcome := true;

        -----------------------------------------------------------------------
        -- 1. Read pointers from RX Buffer, check pointers are 0 (DUT is post 
        --    reset).
        -----------------------------------------------------------------------
        info("Step 1: Reading RX buffer pointers for first time");
        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        check(rx_buf_info.rx_read_pointer = 0, "Read pointer 0!");
        check(rx_buf_info.rx_write_pointer = 0, "Write pointer 0!");
        
        -----------------------------------------------------------------------
        -- 2. Try to read CAN frame from RX Buffer. This should generate at 
        --    least 4 reads from RX DATA register.
        -----------------------------------------------------------------------
        info("Step 2: Try to read frame from empty RX Buffer!");
        CAN_read_frame(frame_rx, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- 3. Read pointers from RX Buffer, check pointers are still 0.
        ------------------------------------------------------------------------
        info("Step 3: Read RX Buffer pointers again!");
        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        check(rx_buf_info.rx_read_pointer = 0, "Read pointer 0!");
        check(rx_buf_info.rx_write_pointer = 0, "Write pointer 0!");

        wait for 1000 ns;

  end procedure;

end package body;
