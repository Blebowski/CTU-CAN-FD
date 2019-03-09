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
use lib.endian_swap.all;
use lib.reduce_lib.all;

use lib.CAN_FD_register_map.all;
use lib.CAN_FD_frame_format.all;

Library work;
use work.CANtestLib.All;

package models_pkg is

    component prescaler_model is
    generic(
      reset_polarity        :   std_logic := '0';
      ipt_length            :   natural := 3;
      sync_trigger_count    :   natural range 2 to 8 := 2;
      sample_trigger_count  :   natural range 2 to 8 := 3;
      clock_period          :   time := 10 ns
    );
    port(
        signal clk_sys              :in std_logic;  --System clock
        signal res_n                :in std_logic;   --Async reset
        signal sync_edge            :in std_logic;        --Edge for synchronisation
        signal OP_State             :in oper_mode_type;   --Protocol control state
        signal drv_bus              :in std_logic_vector(1023 downto 0); 
        signal clk_tq_nbt           :out std_logic;
        signal clk_tq_dbt           :out std_logic;    
        signal sample_nbt   :out std_logic_vector(sample_trigger_count - 1 downto 0); 
        signal sample_dbt   :out std_logic_vector(sample_trigger_count - 1 downto 0);
        signal sync_nbt     :out std_logic_vector(sync_trigger_count - 1 downto 0);
        signal sync_dbt     :out std_logic_vector(sync_trigger_count - 1 downto 0);
        signal bt_FSM_out           :out bit_time_type;
        signal data_tx              :in   std_logic;
        signal hard_sync_edge_valid :out std_logic;     
        signal sp_control           :in std_logic_vector(1 downto 0);
        signal sync_control         :in std_logic_vector(1 downto 0)
  );
  end component;

end package;