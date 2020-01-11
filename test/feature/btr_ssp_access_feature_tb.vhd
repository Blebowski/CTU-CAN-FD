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
-- @TestInfoStart
--
-- @Purpose:
--  BTR, BTR_FD and SSP_CFG register access feature test.
--
-- @Verifies:
--  @1. When node is disabled SETTINGS[ENA]='0', BTR, BTR_FD and SSP_CFG registers
--      are not writable. When node is enabled, they are writable!
--
-- @Test sequence:
--  @1. Read values in BTR, BTR_FD and SSP_CFG registers. Try to write them,
--      read them back and check that value has not changed! Node 1 is enabled!
--  @2. Disable Node 1 and try to write BTR, BTR_FD and SSP_CFG registers. Read
--      them back and check that value was written!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--   06.12.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package btr_ssp_access_feature is
    procedure btr_ssp_access_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body btr_ssp_access_feature is
    procedure btr_ssp_access_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        constant ID_1               :        natural := 1;
        constant ID_2               :        natural := 2;

        variable btr                :        std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable btr_fd             :        std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable ssp_cfg            :        std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable btr_2              :        std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable btr_fd_2           :        std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable ssp_cfg_2          :        std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable rand_value         :        std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
    begin

        ----------------------------------------------------------------------
        -- @1. Read values in BTR, BTR_FD and SSP_CFG registers. Try to write 
        --    them, read them back and check that value has not changed!
        --    Node 1 is enabled!
        ----------------------------------------------------------------------
        info("Step 1");

        CAN_read(btr, BTR_ADR, ID_1, mem_bus(1), BIT_32);
        CAN_read(btr_fd, BTR_FD_ADR, ID_1, mem_bus(1), BIT_32);
        CAN_read(ssp_cfg, SSP_CFG_ADR, ID_1, mem_bus(1), BIT_16);

        rand_logic_vect_v(rand_ctr, rand_value, 0.5);
        CAN_write(rand_value, BTR_ADR, ID_1, mem_bus(1), BIT_32);
        rand_logic_vect_v(rand_ctr, rand_value, 0.5);
        CAN_write(rand_value, BTR_FD_ADR, ID_1, mem_bus(1), BIT_32);
        rand_logic_vect_v(rand_ctr, rand_value, 0.5);
        CAN_write(rand_value, SSP_CFG_ADR, ID_1, mem_bus(1), BIT_16);
        
        CAN_read(btr_2, BTR_ADR, ID_1, mem_bus(1), BIT_32);
        CAN_read(btr_fd_2, BTR_FD_ADR, ID_1, mem_bus(1), BIT_32);
        CAN_read(ssp_cfg_2, SSP_CFG_ADR, ID_1, mem_bus(1), BIT_16);

        check(btr = btr_2, "BTR register not written!");
        check(btr_fd = btr_fd_2, "BTR FD register not written!");
        check(ssp_cfg = ssp_cfg_2, "SSP_CFG register not written!");

        ----------------------------------------------------------------------
        -- @2. Disable Node 1 and try to write BTR, BTR_FD and SSP_CFG
        --    registers. Read them back and check that value was written!
        ----------------------------------------------------------------------
        info("Step 2");
        
        CAN_turn_controller(false, ID_1, mem_bus(1));
        CAN_turn_controller(false, ID_2, mem_bus(2));
        
        rand_logic_vect_v(rand_ctr, rand_value, 0.5);
        CAN_write(rand_value, BTR_ADR, ID_1, mem_bus(1), BIT_32);
        CAN_read(btr, BTR_ADR, ID_1, mem_bus(1), BIT_32);
        check(btr = rand_value, "BTR register written!");

        rand_logic_vect_v(rand_ctr, rand_value, 0.5);
        rand_value(18) := '0';
        rand_value(12) := '0';
        rand_value(6) := '0'; -- These bits are not implemented!
        CAN_write(rand_value, BTR_FD_ADR, ID_1, mem_bus(1), BIT_32);
        CAN_read(btr, BTR_FD_ADR, ID_1, mem_bus(1), BIT_32);
        check(btr = rand_value, "BTR FD register written!");

        rand_logic_vect_v(rand_ctr, rand_value, 0.5);
        rand_value(15 downto 0) := (OTHERS => '0');
        rand_value(31 downto 26) := (OTHERS => '0');
        CAN_write(rand_value, SSP_CFG_ADR, ID_1, mem_bus(1), BIT_16);
        CAN_read(btr, SSP_CFG_ADR, ID_1, mem_bus(1), BIT_16);
        btr(15 downto 0) := (OTHERS => '0');  -- @16 LSBs are other register!
        btr(31 downto 26) := (OTHERS => '0'); -- These are not implemented!
        check(btr = rand_value, "SSP CFG register written!");

  end procedure;
  
end package body;