--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2021-present Ondrej Ille
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- educational, research, evaluation, self-interest purposes. Using the
-- Component for commercial purposes is forbidden unless previously agreed with
-- Copyright holder.
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
-- -------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2015-2020 MIT License
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
--  ERR_CAPT[ERR_POS] = ERC_POS_ARB, bit error feature test. 
--
-- @Verifies:
--  @1. Detection of bit error in Arbitration field. Value of ERR_CAPT[ERR_POS]
--      when bit error should have been detected in arbitration field.
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Generate CAN frame and send it by DUT. Wait until transmission starts
--      and wait until arbitration field. Wait for random amount of time until
--      Dominant bit is sent! Force bus low and wait until sample point. Check
--      that Error frame is being transmitted. Check value of ERR_CAPT.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    03.02.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package err_capt_arb_bit_ftest is
    procedure err_capt_arb_bit_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body err_capt_arb_bit_ftest is
    procedure err_capt_arb_bit_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is        
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;    

        variable id_vect            :     std_logic_vector(28 downto 0);
        variable wait_time          :     natural;
        
        variable frame_sent         :     boolean;
        
        variable err_capt           :     SW_error_capture;
        variable can_tx_val         :     std_logic;
    begin

        -----------------------------------------------------------------------
        -- @1. Check that ERR_CAPT contains no error (post reset).
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        check_m(err_capt.err_pos = err_pos_other, "Reset of ERR_CAPT!");
        
        -----------------------------------------------------------------------        
        --  @2. Generate CAN frame and send it by DUT. Wait until transmission
        --      starts and wait until arbitration field. Wait for random amount
        --      of time until Dominant bit is sent! Force bus low and wait until
        --      sample point. Check that Error frame is being transmitted. Check
        --      value of ERR_CAPT.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        CAN_generate_frame(frame_1);
        frame_1.ident_type := EXTENDED;
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        
        CAN_wait_pc_state(pc_deb_arbitration, DUT_NODE, chn);
        
        -- Wait time is adjusted so that we are sure that we will still be in
        -- arbitration field (of base or extended). After 26 bits, if there are
        -- all dominant till end of frame, we are sure at least one stuff bit
        -- will be there!
        rand_int_v(24, wait_time);
        -- We need at least 1 bit to wait, otherwise we try to corrupt still in SOF!
        wait_time := wait_time + 1;
        info_m("Waiting for:" & integer'image(wait_time) & " bits!");

        for i in 1 to wait_time loop
            CAN_wait_sync_seg(DUT_NODE, chn);
            info_m("Wait sync");
            wait for 20 ns;
        end loop;
        info_m("Waiting finished!");

        get_can_tx(DUT_NODE, can_tx_val, chn);
        while (can_tx_val = RECESSIVE) loop
            CAN_wait_sync_seg(DUT_NODE, chn);
            wait for 20 ns;
            get_can_tx(DUT_NODE, can_tx_val, chn);
        end loop;
        
        -- Force bus for one bit time
        force_bus_level(RECESSIVE, chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns; -- To be sure that opposite bit is sampled!
        release_bus_level(chn);
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.error_transmission, "Error frame is being transmitted!");
        
        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        --error_m("HOVNO");
        -- If Dominant stuff bit is sent and recessive is monitored, then this
        -- can be detected as Stuff Error, not as bit Error!
        check_m(err_capt.err_type = can_err_bit or err_capt.err_type = can_err_stuff,
                "Bit or Stuff error detected!");
        check_m(err_capt.err_pos = err_pos_arbitration, "Error detected in Arbitration!");
        
        CAN_wait_bus_idle(DUT_NODE, chn);

  end procedure;

end package body;
