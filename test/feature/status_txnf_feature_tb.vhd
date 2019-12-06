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
--  Jiri Novak <jnovak@fel.cvut.cz>
--  Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
--  STATUS[TXNF] feature test.
--
-- Verifies:
--  1. When no TXT Buffer is in Empty state, STATUS[TXNF] is not set.
--  2. When at least on TXT Buffer is in Empty state STATUS[TXNF] is set.
--
-- Test sequence:
--  1. Set LOM mode in Node 1. Check that STATUS[TXNF] is set (all TXT Buffers
--     should be empty).
--  2. Issue Set ready and Set Abort consecutively to all TXT Buffers. Check
--     that STATUS[TXNF] is set before last buffer. Check that after last buffer
--     STATUS[TXNF] is not set.
--  3. Check that all TXT Buffers are Aborted now. Move always single buffer to
--     empty and check that STATUS[TXNF] is set. Move this buffer to Ready again
--     and check that STATUS[TXNF] is not set. Repeat with each TXT Buffer.
--     
--------------------------------------------------------------------------------
-- Revision History:
--    31.10.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package status_txnf_feature is
    procedure status_txnf_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body status_txnf_feature is
    procedure status_txnf_feature_exec(
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
        variable command            :     SW_command := SW_command_rst_val;
        
        variable num_frames         :     integer;
        variable mode_1             :     SW_mode;
    begin

        -----------------------------------------------------------------------
        -- 1. Set LOM mode in Node 1. Check that STATUS[TXNF] is set (all TXT
        --    Buffers should be empty).
        -----------------------------------------------------------------------
        info("Step 1");
        mode_1.listen_only := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));

        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.tx_buffer_empty, "STATUS[TXNF] set!");

        -----------------------------------------------------------------------
        -- 2. Issue Set ready and Set Abort consecutively to all TXT Buffers.
        --    Check that STATUS[TXNF] is set before last buffer. Check that
        --    after last buffer STATUS[TXNF] is not set.
        -----------------------------------------------------------------------
        for i in 1 to C_TXT_BUFFER_COUNT loop
            send_TXT_buf_cmd(buf_set_ready, i, ID_1, mem_bus(1));
            send_TXT_buf_cmd(buf_set_abort, i, ID_1, mem_bus(1));

            get_controller_status(stat_1, ID_1, mem_bus(1));

            if (i = C_TXT_BUFFER_COUNT) then
                check_false(stat_1.tx_buffer_empty,
                    "STATUS[TXNF] not set after last TXT Buffer");
            else
                check(stat_1.tx_buffer_empty,
                    "STATUS[TXNF] set before last TXT Buffer");
            end if;
        end loop;

        -----------------------------------------------------------------------
        -- 3. Check that all TXT Buffers are Aborted now. Move always single
        --    buffer to empty and check that STATUS[TXNF] is set. Move this
        --    buffer to Ready again and check that STATUS[TXNF] is not set.
        --    Repeat with each TXT Buffer.
        -----------------------------------------------------------------------
        info("Step 3");
        for i in 1 to C_TXT_BUFFER_COUNT loop
            get_tx_buf_state(i, txt_buf_state, ID_1, mem_bus(1));
            check(txt_buf_state = buf_aborted, "TXT Buffer: " &
                integer'image(i) & " aborted!");
        end loop;

        for i in 1 to C_TXT_BUFFER_COUNT loop
            send_TXT_buf_cmd(buf_set_empty, i, ID_1, mem_bus(1));
            
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check(stat_1.tx_buffer_empty, "STATUS[TXNF] set!");
            
            send_TXT_buf_cmd(buf_set_ready, i, ID_1, mem_bus(1));
            
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check_false(stat_1.tx_buffer_empty, "STATUS[TXNF] not set!");
            
            get_tx_buf_state(i, txt_buf_state, ID_1, mem_bus(1));
            check(txt_buf_state = buf_ready, "TXT Buffer: " &
                integer'image(i) & " ready!");
        end loop;
        
        wait for 100 ns;

  end procedure;

end package body;
