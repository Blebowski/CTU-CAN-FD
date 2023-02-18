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
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;
use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.can_unit_test_pkg.all;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

library vunit_lib;
context vunit_lib.vunit_context;

architecture message_filter_unit_test of CAN_test is

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

    signal store_metadata_f   : std_logic;

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
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_a_bits, 0.50);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_a_mask, 0.15);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_a_ctrl, 0.50);

        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_b_bits, 0.50);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_b_mask, 0.15);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_b_ctrl, 0.50);

        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_c_bits, 0.50);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_c_mask, 0.15);
        rand_logic_vect_s  (rand_ctr, drv_settings.drv_filter_c_ctrl, 0.50);

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
        variable a_type       :       boolean;
        variable b_type       :       boolean;
        variable c_type       :       boolean;
        variable ran_type     :       boolean;
        variable a_vals       :       boolean;
        variable b_vals       :       boolean;
        variable c_vals       :       boolean;
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
        a_type :=  not ((ctrl and drv_settings.drv_filter_a_ctrl) = "0000");
        b_type :=  not ((ctrl and drv_settings.drv_filter_b_ctrl) = "0000");
        c_type :=  not ((ctrl and drv_settings.drv_filter_c_ctrl) = "0000");
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
    frame_filters_comp : entity ctu_can_fd_rtl.frame_filters
    PORT map(
        clk_sys                             => clk_sys,
        res_n                               => res_n,
        rec_ident                           => rec_ident_in,
        rec_ident_type                      => ident_type,
        rec_frame_type                      => frame_type,
        rec_is_rtr                          => '0',

        mr_filter_control_fafe              => drv_settings.drv_filter_a_ctrl(3),
        mr_filter_control_fafb              => drv_settings.drv_filter_a_ctrl(2),
        mr_filter_control_fane              => drv_settings.drv_filter_a_ctrl(1),
        mr_filter_control_fanb              => drv_settings.drv_filter_a_ctrl(0),

        mr_filter_control_fbfe              => drv_settings.drv_filter_b_ctrl(3),
        mr_filter_control_fbfb              => drv_settings.drv_filter_b_ctrl(2),
        mr_filter_control_fbne              => drv_settings.drv_filter_b_ctrl(1),
        mr_filter_control_fbnb              => drv_settings.drv_filter_b_ctrl(0),

        mr_filter_control_fcfe              => drv_settings.drv_filter_c_ctrl(3),
        mr_filter_control_fcfb              => drv_settings.drv_filter_c_ctrl(2),
        mr_filter_control_fcne              => drv_settings.drv_filter_c_ctrl(1),
        mr_filter_control_fcnb              => drv_settings.drv_filter_c_ctrl(0),

        mr_filter_control_frfe              => drv_settings.drv_filter_ran_ctrl(3),
        mr_filter_control_frfb              => drv_settings.drv_filter_ran_ctrl(2),
        mr_filter_control_frne              => drv_settings.drv_filter_ran_ctrl(1),
        mr_filter_control_frnb              => drv_settings.drv_filter_ran_ctrl(0),

        mr_filter_a_mask_bit_mask_a_val     => drv_settings.drv_filter_a_mask,
        mr_filter_a_val_bit_val_a_val       => drv_settings.drv_filter_a_bits,
        mr_filter_b_mask_bit_mask_b_val     => drv_settings.drv_filter_b_mask,
        mr_filter_b_val_bit_val_b_val       => drv_settings.drv_filter_b_bits,
        mr_filter_c_mask_bit_mask_c_val     => drv_settings.drv_filter_c_mask,
        mr_filter_c_val_bit_val_c_val       => drv_settings.drv_filter_c_bits,
        mr_filter_ran_high_bit_ran_high_val => drv_settings.drv_filter_ran_hi_th,
        mr_filter_ran_low_bit_ran_low_val   => drv_settings.drv_filter_ran_lo_th,
        mr_settings_fdrf                    => '0',
        mr_mode_afm                         => drv_settings.drv_filters_ena,

        -- Don't test command filtering as this is trivial. Testing is done on
        -- out ident valid only!
        store_metadata                      => '1',
        store_metadata_f                    => store_metadata_f,

        store_data                          => '0',
        rec_valid                           => '0',
        rec_abort                           => '0'
    );


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0, out_clk => clk_sys);

    -- Connect input generator to the circuit
    rec_ident_in       <= frame_info.rec_ident_in;
    ident_type         <= frame_info.ident_type;
    frame_type         <= frame_info.frame_type;


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

            if (validate(drv_settings, store_metadata_f, frame_info) = false)
            then
                process_error(error_ctr, error_beh, exit_imm);
            end if;

            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

end architecture;
