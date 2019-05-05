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
--  Package with component declarations for models
--------------------------------------------------------------------------------
-- Revision History:
--    05.03.2019  Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

-- Name of work library is by default set to "lib" in GHDL.
Library lib;
use lib.id_transfer.all;
use lib.can_constants.all;
use lib.can_components.all;
use lib.can_types.all;
use lib.cmn_lib.all;
use lib.drv_stat_pkg.all;
use lib.reduce_lib.all;

use lib.CAN_FD_register_map.all;
use lib.CAN_FD_frame_format.all;

Library work;
use work.CANtestLib.All;

package models_pkg is

   component prescaler_model is
    generic(
      -- Reset polarity
      reset_polarity        :   std_logic := '0';
      
      -- Clock period
      clock_period          :   time := 10 ns
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and async reset
        -----------------------------------------------------------------------
        signal clk_sys              :in std_logic;
        signal res_n                :in std_logic;
    
        -----------------------------------------------------------------------
        -- Bus synch Interface
        -----------------------------------------------------------------------
        signal sync_edge            :in std_logic;
        signal OP_State             :in t_operation_control_state;
        
        -- Driving Bus
        signal drv_bus              :in std_logic_vector(1023 downto 0); 
        
        -- Bit time FSM output
        signal bt_FSM               :out t_bit_time;
    
        -- What is actual node transmitting on the bus
        signal data_tx              :in   std_logic;
    
        -----------------------------------------------------------------------
        -- Bit time and Synchronisation config
        -----------------------------------------------------------------------
        signal sp_control           :in std_logic_vector(1 downto 0);
        signal sync_control         :in std_logic_vector(1 downto 0)
    );
    end component;

end package;