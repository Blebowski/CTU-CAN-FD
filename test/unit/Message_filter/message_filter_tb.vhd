--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Project advisors and co-authors:
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--  Unit test for the message filter circuit
--------------------------------------------------------------------------------
-- Revision History:
--    30.5.2016   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.ID_transfer.all;

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

    -- Identifier valid (active log 1)
    signal rec_ident_valid    : std_logic;

    signal drv_bus            : std_logic_vector(1023 downto 0);
    signal out_ident_valid    : std_logic;

    -- Internal testbench signals
    signal frame_info         : mess_filter_input_type :=
                        ((OTHERS => '0'), '0', '0', '0');

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
        rand_logic_s    (rand_ctr,  frame_info.rec_ident_valid  ,0.9);
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


    function validate(
        signal drv_settings   :in     mess_filter_drv_type;
        signal filt_res       :in     std_logic;
        signal log_level      :in     log_lvl_type;
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
    begin

        -- Filters disabled but result is positive -> error
        if (drv_settings.drv_filters_ena = '0') then

          if (frame_info.rec_ident_valid = filt_res) then
              return true;
          else
              log("Filters disabled but result positive", error_l, log_level);
              return false;
          end if;
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
        if( (A_type = false)    and
          (B_type = false)    and
          (C_type = false)    and
          (ran_type = false)  and
          (filt_res = '1')
        )then
            log("Invalid frame type was not filtered out", error_l, log_level);
            return false;
        end if;

        -------------------------------------------
        -- Valid or invalid frames on input
        -------------------------------------------
        if( ((A_type and  A_vals) or
           (B_type and B_vals) or
           (C_type and C_vals) or
           (ran_type and ran_vals))
          and
           (drv_settings.drv_filters_ena='1')
          and
           (frame_info.rec_ident_valid='1')
        ) then

            if (filt_res = '1') then   --Is detected
                return true;

            elsif (filt_res = '0') then -- Is not detected
                -- LCOV_EXCL_START
                log("Valid frame not detected", error_l, log_level);
                return false;
                -- LCOV_EXCL_STOP
            else
                -- LCOV_EXCL_START
                log("Filter res undefined", error_l, log_level);
                return false;
                -- LCOV_EXCL_STOP
            end if;

        else

            if (filt_res = '1') then   --Is detected
                -- LCOV_EXCL_START
                log("Invalid frame but frame detected", error_l, log_level);
                return false;
                -- LCOV_EXCL_STOP

            elsif (filt_res = '0') then -- Is not detected
                return true;

            else
                -- LCOV_EXCL_START
                log("Filter res undefined", error_l, log_level);
                return false;
                -- LCOV_EXCL_STOP
            end if;

        end if;
    end function;


begin

    ----------------------------------------------------------------------------
    -- Instance of the message filter
    ----------------------------------------------------------------------------
    messageFilter_comp : messageFilter
    PORT map(
        clk_sys            =>  clk_sys,
        res_n              =>  res_n,
        rec_ident_in       =>  rec_ident_in,
        ident_type         =>  ident_type,
        frame_type         =>  frame_type,
        rec_ident_valid    =>  rec_ident_valid,
        drv_bus            =>  drv_bus,
        out_ident_valid    =>  out_ident_valid
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


    -- Connect input generator to the circuit
    rec_ident_in       <=  frame_info.rec_ident_in;
    ident_type         <=  frame_info.ident_type;
    frame_type         <=  frame_info.frame_type;
    rec_ident_valid    <=  frame_info.rec_ident_valid;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
    begin
        log("Restarting Message filter test!", info_l, log_level);
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 0, rand_ctr);
        log("Restarted Message filter test", info_l, log_level);
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        log("Starting message filter main loop", info_l, log_level);

        while (loop_ctr < iterations  or exit_imm)
        loop
            log("Starting loop nr " & integer'image(loop_ctr),
                info_l, log_level);

            generate_input    (rand_ctr, frame_info);
            generate_setting  (rand_ctr, drv_settings);

            wait for 10 ns;

            if (validate(drv_settings, out_ident_valid, log_level, frame_info)
                = false)
            then
                process_error(error_ctr, error_beh, exit_imm);
            end if;

            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

end architecture;
