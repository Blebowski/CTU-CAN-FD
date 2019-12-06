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

package ssp_offset_feature is
    procedure ssp_offset_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body ssp_offset_feature is
    procedure ssp_offset_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable data               :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable address            :       std_logic_vector(11 downto 0) :=
                                                (OTHERS => '0');
                                                
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable retr_th            :       natural;

        variable mode               :       SW_mode := (false, false, false,
                                                false, true, false, false,
                                                false, false, false);
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable buf_state          :       SW_TXT_Buffer_state_type;
        
        variable ssp_source         :       SSP_set_command_type;   
        variable ssp_offset         :       std_logic_vector(6 downto 0);   
    begin

        ------------------------------------------------------------------------
        -- Set both nodes to forbid acknowledge
        ------------------------------------------------------------------------
       -- mode.acknowledge_forbidden := true;
       -- set_core_mode(mode, ID_1, mem_bus(1));
       -- set_core_mode(mode, ID_2, mem_bus(2));
       -- mode.acknowledge_forbidden := false;

        ------------------------------------------------------------------------
        -- Configure SSP_CFG on Node 1
        ------------------------------------------------------------------------ 
        ssp_source := ssp_measured;
        ssp_offset := (OTHERS => '0');
        CAN_configure_ssp(ssp_source, ssp_offset, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- Generate and send FD frame by Node 1
        ------------------------------------------------------------------------
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.frame_format := NORMAL_CAN;   --FD_CAN;
        --CAN_frame.rtr := NO_RTR_FRAME;
       -- CAN_frame.brs := BR_NO_SHIFT;   --BR_SHIFT;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        ------------------------------------------------------------------------
        -- Waits until reception is started by a Node 2.
        ------------------------------------------------------------------------
        CAN_wait_tx_rx_start(false, true, ID_2, mem_bus(2));
        
        -- počkat si na BRS bit
        -- vynulovat si čítač a přičítat clockly hodiny
        -- počkat až dostanu signal od SSP
        -- přečíst čítat
        -- vyhodnotit, jestli čítač načítal, co jsem nastavil
        
        
        -- Wait till transmission is done
        CAN_wait_frame_sent(ID_1, mem_bus(1));

        CAN_wait_bus_idle(ID_2, mem_bus(2));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        ------------------------------------------------------------------------
        -- Wait number of retransmissions. After each one, TXT Buffer should
        -- be back in ready. After last one, it should be in failed.
        ------------------------------------------------------------------------
        --for i in 0 to retr_th loop
           -- CAN_wait_frame_sent(ID_1, mem_bus(1));
            --get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
           -- if (i /= retr_th) then
            --    check(buf_state = buf_ready, "TXT Buffer not ready");
         --   else
           --     check(buf_state = buf_failed, "TXT Buffer not failed");
           -- end if;
       -- end loop;

        ------------------------------------------------------------------------
        -- Read TX Counter, it should be equal to 8 times number of retransmitts
        -- plus one original transmittion does not count as retransmittion.
        ------------------------------------------------------------------------
        read_error_counters(err_counters, ID_1, mem_bus(1));
        check(err_counters.tx_counter = 8 * (retr_th + 1),
            "Counters exp: " & Integer'Image(err_counters.tx_counter) &
            " counters real: " & Integer'image(8 * (retr_th + 1)));

 
  end procedure;

end package body;