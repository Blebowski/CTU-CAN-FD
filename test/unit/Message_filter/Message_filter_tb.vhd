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
--  Unit test for the frame filters circuit.
--
-- @Verifies:
--  @1. Bit filter functionality (Filter value and filter mask).
--  @2. Range filter functionality (Low and High thresholds).
--
-- @Test sequence:
--  @1. Generate random bit values, bit masks for bit filters and low-high
--      thresholds for range filter.
--  @2. Generate random CAN ID and frame and identifier type on input of Frame
--      filters.
--  @3. Calculate whether frame shall pass filters (SW model).
--  @4. Check whether output of frame filters equals to output of SW model.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    30.5.2016   Created file
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;
use ieee.std_logic_textio.all;
use STD.textio.all;

library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer.all;
use ctu_can_fd_rtl.can_constants.all;
use ctu_can_fd_rtl.can_components.all;
use ctu_can_fd_rtl.can_types.all;
use ctu_can_fd_rtl.cmn_lib.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.reduce_lib.all;
use ctu_can_fd_rtl.can_config.all;
use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.can_unit_test_pkg.all;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

library vunit_lib;
context vunit_lib.vunit_context;

architecture mess_filt_unit_test of CAN_test is

    -- Clock and reset
    signal clk_sys            : std_logic;
    signal res_n              : std_logic;

    -- Received identifier
    signal rec_ident_in       : std_logic_vector(28 downto 0);

    -- Received identifier type
    -- (0-BASE Format, 1-Extended Format);
    signal ident_type         : std_logic;

    -- Input frame type (0-Normal CAN, 1- CAN FD)
    signal frame_type         : std_logic;

    signal drv_bus            : std_logic_vector(1023 downto 0);
    signal out_ident_valid    : std_logic;

    -- Internal testbench signals
    signal frame_info         : mess_filter_input_type :=
                        ((OTHERS => '0'), '0', '0');

    signal drv_settings       : mess_filter_drv_type   :=
                        ((OTHERS => '0'), (OTHERS => '0'),(OTHERS => '0'),
                         (OTHERS => '0'), (OTHERS => '0'),(OTHERS => '0'),
                         (OTHERS => '0'), (OTHERS => '0'),(OTHERS => '0'),
                         (OTHERS => '0'), (OTHERS => '0'),(OTHERS => '0'), '0');

    procedure generate_input(
        signal rand_ctr        :inout natural range 0 to RAND_POOL_SIZE;
        signal frame_info      :out   mess_filter_input_type
    )is
    begin
        rand_logic_vect_s (rand_ctr,  frame_info.rec_ident_in     ,0.5);
        rand_logic_s    (rand_ctr,  frame_info.ident_type       ,0.5);
        rand_logic_s    (rand_ctr,  frame_info.frame_type       ,0.5);
    end procedure;


    procedure generate_setting(
        signal rand_ctr        :inout   natural range 0 to RAND_POOL_SIZE;
        signal drv_settings    :inout   mess_filter_drv_type
    )is
    begin
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_A_bits, 0.50);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_A_mask, 0.15);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_A_ctrl, 0.50);

        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_B_bits, 0.50);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_B_mask, 0.15);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_B_ctrl, 0.50);

        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_C_bits, 0.50);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_C_mask, 0.15);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_C_ctrl, 0.50);

        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_ran_hi_th, 0.60);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_ran_lo_th, 0.40);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_ran_ctrl,  0.50);

        rand_logic_s       (rand_ctr, drv_settings.drv_filters_ena, 0.9);
    end procedure;


    impure function validate(
        signal drv_settings   :in     mess_filter_drv_type;
        signal filt_res       :in     std_logic;
        signal frame_info     :in     mess_filter_input_type)
    return boolean is
        variable join         :       std_logic_vector(1 downto 0);
        variable ctrl         :       std_logic_vector(3 downto 0);
        variable A_type       :       boolean;
        variable B_type       :       boolean;
        variable C_type       :       boolean;
        variable ran_type     :       boolean;
        variable A_vals       :       boolean;
        variable B_vals       :       boolean;
        variable C_vals       :       boolean;
        variable ran_vals     :       boolean;
        variable ident_dec    :       integer;
        variable ran_low_dec  :       integer;
        variable ran_high_dec :       integer;
        variable frame_conc   :       std_logic_vector(28 downto 0);
        variable low_conc     :       std_logic_vector(28 downto 0);
        variable high_conc    :       std_logic_vector(28 downto 0);
        variable inv_type     :       boolean;
    begin

        -- Filters disabled -> Output is considered always valid
        if (drv_settings.drv_filters_ena = '0') then
          return true;
        end if;

        join := frame_info.frame_type & frame_info.ident_type;
        case join is
            when "00" => ctrl := "0001" ; --CAN BASIC
            when "01" => ctrl := "0010" ; --CAN Extended
            when "10" => ctrl := "0100" ; --CAN FD Basic
            when "11" => ctrl := "1000" ; --CAN Fd Extended
            when others => ctrl := "0000" ;
        end case;

        -- Calculate the values of matching frames
        A_type :=  not ((ctrl and drv_settings.drv_filter_A_ctrl) = "0000");
        B_type :=  not ((ctrl and drv_settings.drv_filter_B_ctrl) = "0000");
        C_type :=  not ((ctrl and drv_settings.drv_filter_C_ctrl) = "0000");
        ran_type :=  not ((ctrl and drv_settings.drv_filter_ran_ctrl) = "0000");

        A_vals := ((frame_info.rec_ident_in and
                  drv_settings.drv_filter_A_mask)
                 =
                 (drv_settings.drv_filter_A_bits and
                  drv_settings.drv_filter_A_mask));

        B_vals := ((frame_info.rec_ident_in and
                  drv_settings.drv_filter_B_mask)
                 =
                 (drv_settings.drv_filter_B_bits and
                  drv_settings.drv_filter_B_mask));

        C_vals :=  ((frame_info.rec_ident_in and
                   drv_settings.drv_filter_C_mask)
                  =
                  (drv_settings.drv_filter_C_bits and
                   drv_settings.drv_filter_C_mask));

        frame_conc := frame_info.rec_ident_in(28 downto 18) &
                    frame_info.rec_ident_in(17 downto 0);
        ident_dec  := to_integer(unsigned(frame_conc));

        -- Note that here identifier parts are not swapped since driving bus
        -- value is already decimal value!
        low_conc   := drv_settings.drv_filter_ran_lo_th(28 downto 18) &
                    drv_settings.drv_filter_ran_lo_th(17 downto 0);
        ident_dec  := to_integer(unsigned(low_conc));

        high_conc  := drv_settings.drv_filter_ran_hi_th(28 downto 18) &
                    drv_settings.drv_filter_ran_hi_th(17 downto 0);
        ident_dec  := to_integer(unsigned(high_conc));

        ran_vals   := ((frame_conc < high_conc) or (frame_conc = high_conc)) and
                    ((frame_conc > low_conc) or  (frame_conc = low_conc));


        --------------------------------------------
        -- Invalid frame type was not filtered out
        --------------------------------------------
        check((A_type = true)    or
              (B_type = true)    or
              (C_type = true)    or
              (ran_type = true)  or
              (filt_res = '0'),
              "No filter should have valid frame type, but output is valid!");

        -------------------------------------------
        -- Valid or invalid frames on input
        -------------------------------------------
        if(((A_type and  A_vals) or
           (B_type and B_vals) or
           (C_type and C_vals) or
           (ran_type and ran_vals))
          and
           (drv_settings.drv_filters_ena = '1')
        ) then
        
            check(filt_res = '1', "Valid frame did not pass filters!");
            if (filt_res = '1') then   --Is detected
                return true;
            else
                return false;
            end if;

        else
            
            check(filt_res = '0', "Invalid frame passed filters!");
            if (filt_res = '0') then -- Is not detected
                return true;
            else
                return false;
            end if;

        end if;
    end function;


begin

    ----------------------------------------------------------------------------
    -- Instance of frame filters
    ----------------------------------------------------------------------------
    frame_filters_comp : frame_filters
    PORT map(
        clk_sys            =>  clk_sys,
        res_n              =>  res_n,
        rec_ident          =>  rec_ident_in,
        rec_ident_type     =>  ident_type,
        rec_frame_type     =>  frame_type,
        rec_is_rtr         =>  '0',
        drv_bus            =>  drv_bus,
        ident_valid        =>  out_ident_valid,
        
        -- Don't test command filtering as this is trivial. Testing is done on
        -- out ident valid only!
        store_metadata     =>  '0',
        store_data         =>  '0',
        rec_valid          =>  '0',
        rec_abort          =>  '0'
    );


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);

    -- Propagate driving bus to driving bus signals
    drv_bus(DRV_FILTER_A_MASK_HIGH downto DRV_FILTER_A_MASK_LOW) <=
        drv_settings.drv_filter_A_mask;

    drv_bus(DRV_FILTER_A_CTRL_HIGH downto DRV_FILTER_A_CTRL_LOW) <=
        drv_settings.drv_filter_A_ctrl;

    drv_bus(DRV_FILTER_A_BITS_HIGH downto DRV_FILTER_A_BITS_LOW) <=
        drv_settings.drv_filter_A_bits;

    drv_bus(DRV_FILTER_B_MASK_HIGH downto DRV_FILTER_B_MASK_LOW) <=
        drv_settings.drv_filter_B_mask;

    drv_bus(DRV_FILTER_B_CTRL_HIGH downto DRV_FILTER_B_CTRL_LOW) <=
        drv_settings.drv_filter_B_ctrl;

    drv_bus(DRV_FILTER_B_BITS_HIGH downto DRV_FILTER_B_BITS_LOW) <=
        drv_settings.drv_filter_B_bits;
    drv_bus(DRV_FILTER_C_MASK_HIGH downto DRV_FILTER_C_MASK_LOW) <=
        drv_settings.drv_filter_C_mask;

    drv_bus(DRV_FILTER_C_CTRL_HIGH downto DRV_FILTER_C_CTRL_LOW) <=
        drv_settings.drv_filter_C_ctrl;

    drv_bus(DRV_FILTER_C_BITS_HIGH downto DRV_FILTER_C_BITS_LOW) <=
        drv_settings.drv_filter_C_bits;

    drv_bus(DRV_FILTER_RAN_CTRL_HIGH downto DRV_FILTER_RAN_CTRL_LOW) <=
        drv_settings.drv_filter_ran_ctrl;

    drv_bus(DRV_FILTER_RAN_LO_TH_HIGH downto DRV_FILTER_RAN_LO_TH_LOW) <=
        drv_settings.drv_filter_ran_lo_th;

    drv_bus(DRV_FILTER_RAN_HI_TH_HIGH downto DRV_FILTER_RAN_HI_TH_LOW) <=
        drv_settings.drv_filter_ran_hi_th;

    drv_bus(DRV_FILTERS_ENA_INDEX) <=
        drv_settings.drv_filters_ena;

    drv_bus(DRV_FILTER_DROP_RF_INDEX) <= '0';
    
    -- Connect input generator to the circuit
    rec_ident_in       <=  frame_info.rec_ident_in;
    ident_type         <=  frame_info.ident_type;
    frame_type         <=  frame_info.frame_type;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
    begin
        info("Restarting Message filter test!");
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 0, rand_ctr);
        info("Restarted Message filter test");
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        info("Starting message filter main loop");

        while (loop_ctr < iterations  or exit_imm)
        loop
            info("Starting loop nr " & integer'image(loop_ctr));

            generate_input    (rand_ctr, frame_info);
            generate_setting  (rand_ctr, drv_settings);

            wait for 10 ns;

            if (validate(drv_settings, out_ident_valid, frame_info) = false)
            then
                process_error(error_ctr, error_beh, exit_imm);
            end if;

            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

end architecture;
