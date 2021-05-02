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
--  Unit test for the TX Buffer circuit
--
-- @Verifies:
--  @1. TODO
--
-- @Test sequence:
--  @1. TODO
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    14.6.2016   Created file
--    15.4.2018   Modified testbench to support new FSM in TX Buffer. Added
--                Random data checking and state transition checks.
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Test implementation
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

use ctu_can_fd_rtl.can_registers_pkg.all;

library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

library vunit_lib;
context vunit_lib.vunit_context;


architecture tx_buffer_unit_test of CAN_test is

    -- Clocking and reset
    signal clk_sys                :   std_logic:='0';
    signal res_n                  :   std_logic:='0';

    -------------------------------
    --Driving Registers Interface--
    -------------------------------

    -- Data and address for SW access into the RAM of TXT Buffer
    signal txtb_port_a_data              :     std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
    signal txtb_port_a_address              :     std_logic_vector(4 downto 0) :=
                                            (OTHERS => '0');
    signal txtb_port_a_cs                :     std_logic := '0';

    -- SW commands from user registers
    signal txtb_sw_cmd             :     t_txtb_sw_cmd := ('0','0','0');
    signal txtb_sw_cmd_index       :     std_logic_vector(3 downto 0) :=
                                        (OTHERS => '1');
    ------------------
    --Status signals--
    ------------------
    signal txtb_state             :     std_logic_vector(3 downto 0);

    ------------------------------------
    --CAN Core and TX Arbiter Interface-
    ------------------------------------

    -- Commands from the CAN Core for manipulation of the CAN
    signal txtb_hw_cmd            :     t_txtb_hw_cmd :=
                                          ('0', '0', '0', '0', '0', '0');

    signal txtb_hw_cmd_int         :     std_logic;
    signal txtb_hw_cmd_index   :     natural range 0 to 3 := 0;

    -- Buffer output and pointer to the RAM memory
    signal txtb_port_b_data               :     std_logic_vector(31 downto 0);
    signal txtb_port_b_address               :     natural range 0 to 19 := 0;

    -- Signals to the TX Arbitrator that it can be selected for transmission
    -- (used as input to priority decoder)
    signal txtb_available      :     std_logic;

    -- Signals that immediate transition to Bus-off state occurred!
    signal is_bus_off          :     std_logic := '0';

    signal test_registers_out       :    test_registers_out_t :=
        ((OTHERS => '0'), (OTHERS => '0'), (OTHERS => '0'));

    ------------------------------------
    -- Internal testbench signals
    ------------------------------------
    type shadow_memory_type is array (0 to 19) of std_logic_vector(31 downto 0);
    signal shadow_mem             :     shadow_memory_type
            := (OTHERS => (OTHERS => 'U'));

    -- Random generator counters
    signal rand_gen_ctr           :     natural range 0 to RAND_POOL_SIZE;
    signal rand_read_ctr          :     natural range 0 to RAND_POOL_SIZE;
    signal rand_com_gen_ctr       :     natural range 0 to RAND_POOL_SIZE;

    -- Error counters
    signal data_coh_err_ctr       :     natural;
    signal state_coh_error_ctr    :     natural;

    -- Immediate exits
    signal exit_imm_1             :     boolean;
    signal exit_imm_2             :     boolean;

    signal txtb_exp_state         :     std_logic_vector(3 downto 0);

    procedure calc_exp_state(
        signal sw_cmd             : in  t_txtb_sw_cmd;
        signal hw_cmd             : in  t_txtb_hw_cmd;
        signal act_state          : in  std_logic_vector(3 downto 0);
        signal exp_state          : out std_logic_vector(3 downto 0)
    ) is
    begin

        -- By default, the state does not change. Only after command!
        exp_state   <= act_state;

        case act_state is
        when TXT_ETY =>
            if (sw_cmd.set_rdy = '1') then
                exp_state   <= TXT_RDY;
            end if;

        when TXT_RDY =>
            if (hw_cmd.lock = '1') then
                if (sw_cmd.set_abt = '1') then
                    exp_state   <= TXT_ABTP;
                else
                    exp_state   <= TXT_TRAN;
                end if;
            elsif (sw_cmd.set_abt = '1') then
                exp_state       <= TXT_ABT;
            end if;

        when TXT_TRAN =>
            if (sw_cmd.set_abt = '1') then
                exp_state   <= TXT_ABTP;
            end if;

            if (hw_cmd.unlock = '1') then
                if (hw_cmd.valid = '1') then
                    exp_state   <= TXT_TOK;
                elsif (hw_cmd.err = '1' or hw_cmd.arbl = '1') then
                    if (sw_cmd.set_abt = '1') then
                        exp_state   <= TXT_ABT;
                    else
                        exp_state   <= TXT_RDY;
                    end if;
                elsif (hw_cmd.failed = '1') then
                    exp_state   <= TXT_ERR;
                end if;
            end if;

        when TXT_ABTP =>
            if (hw_cmd.unlock = '1') then
                if (hw_cmd.valid = '1') then
                    exp_state   <= TXT_TOK;
                elsif (hw_cmd.err = '1' or hw_cmd.arbl = '1') then
                    exp_state   <= TXT_ABT;
                elsif (hw_cmd.failed = '1') then
                    exp_state   <= TXT_ERR;
                end if;
            end if;

        when TXT_TOK =>
            if (sw_cmd.set_ety = '1') then
                exp_state   <= TXT_ETY;
            elsif (sw_cmd.set_rdy = '1') then
                exp_state   <= TXT_RDY;
            end if;

        when TXT_ABT =>
            if (sw_cmd.set_ety = '1') then
                exp_state   <= TXT_ETY;
            elsif (sw_cmd.set_rdy = '1') then
                exp_state   <= TXT_RDY;
            end if;

        when TXT_ERR =>
            if (sw_cmd.set_ety = '1') then
                exp_state   <= TXT_ETY;
            elsif (sw_cmd.set_rdy = '1') then
                exp_state   <= TXT_RDY;
            end if;
        when others =>
        end case;
    end procedure;

begin

    ----------------------------------------------------------------------------
    -- DUT - Create only one buffer instance
    ----------------------------------------------------------------------------
    txt_buffer_comp : txt_buffer
    generic map(
        G_TXT_BUFFER_COUNT      => 4,
        G_ID                    => 0,
        G_TECHNOLOGY            => C_TECH_ASIC
    )
    port map(
        clk_sys                 => clk_sys,
        res_n                   => res_n,
        txtb_port_a_data        => txtb_port_a_data,
        txtb_port_a_address     => txtb_port_a_address,
        txtb_port_a_cs          => txtb_port_a_cs,
        txtb_sw_cmd             => txtb_sw_cmd,
        txtb_sw_cmd_index       => txtb_sw_cmd_index,
        txtb_state              => txtb_state,
        txtb_hw_cmd             => txtb_hw_cmd,
        txtb_hw_cmd_int         => txtb_hw_cmd_int,
        txtb_hw_cmd_index       => txtb_hw_cmd_index,
        drv_rom_ena             => '0',
        drv_bus_mon_ena         => '0',
        txt_buf_failed_bof      => '1',
        is_bus_off              => is_bus_off,
        txtb_port_b_data        => txtb_port_b_data,
        txtb_port_b_address     => txtb_port_b_address,
        txtb_port_b_clk_en      => '1',
        txtb_available          => txtb_available,
        test_registers_out      => test_registers_out,
        tst_rdata_txt_buf       => open
    );


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);


    ----------------------------------------------------------------------------
    -- Data generation - stored by user writes
    ----------------------------------------------------------------------------
    data_gen_proc : process
        variable buf_fsm : std_logic_vector(3 downto 0);
    begin
        txtb_port_a_cs      <= '0';
        while res_n = C_RESET_POLARITY loop
            wait until rising_edge(clk_sys);
            apply_rand_seed(seed, 3, rand_gen_ctr);
        end loop;

        -- Generate random address and data and attempt to store it
        -- to the buffer.
        wait until rising_edge(clk_sys);
        rand_logic_vect_s(rand_gen_ctr, txtb_port_a_data, 0.5);
        rand_logic_vect_s(rand_gen_ctr, txtb_port_a_address, 0.5);
        if (to_integer(unsigned(txtb_port_a_address)) > 19) then
            txtb_port_a_address  <= "00000";
        end if;
        txtb_port_a_cs <=  '1';
        wait for 0 ns;

        wait until rising_edge(clk_sys);
        txtb_port_a_cs <= '0';
        buf_fsm := txtb_state;
        wait until rising_edge(clk_sys);
        -- Data should be stored only if the buffer is accessible by user,
        -- when it is not ready, neither transmission is in progress.
        -- Store it in the shadow buffer!
        if (buf_fsm /= TXT_RDY and
            buf_fsm /= TXT_TRAN and
            buf_fsm /= TXT_ABTP)
        then
            shadow_mem(to_integer(unsigned(txtb_port_a_address))) <= txtb_port_a_data;
        end if;

        txtb_port_a_cs <=  '0';
        wait until rising_edge(clk_sys);
    end process;


    ----------------------------------------------------------------------------
    -- Reading the data like as If from CAN Core
    ----------------------------------------------------------------------------
    data_read_proc : process
        variable tmp   : std_logic_vector(4 downto 0);
        constant C_DATA_ZEROES : std_logic_vector(31 downto 0) := (OTHERS => '0');
    begin
        while res_n = C_RESET_POLARITY loop
            wait until rising_edge(clk_sys);
            apply_rand_seed(seed, 2, rand_read_ctr);
        end loop;

        data_coh_err_ctr <= 0;
        wait until falling_edge(clk_sys);
        -- Read data from random address in the buffer
        rand_logic_vect_v(rand_read_ctr, tmp, 0.5);
        if (to_integer(unsigned(tmp)) > 19) then
            tmp    := "00000";
        end if;

        txtb_port_b_address <= to_integer(unsigned(tmp));

        wait until falling_edge(clk_sys) and txtb_port_a_cs = '0';

        -- At any point the data should be matching the data in
        -- the shadow buffer
        if (txtb_state = TXT_RDY or txtb_state = TXT_TRAN or txtb_state = TXT_ABTP) then           
            check(txtb_port_b_data = shadow_mem(txtb_port_b_address), "Data coherency error!");
        else
            check(txtb_port_b_data = C_DATA_ZEROES, "Data not masked out!");
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Sending random commands to the buffer from SW and HW
    ----------------------------------------------------------------------------
    commands_proc : process
        variable tmp_real : real;
    begin

        while res_n = C_RESET_POLARITY loop
            wait until rising_edge(clk_sys);
            apply_rand_seed(seed, 1, rand_com_gen_ctr);
        end loop;

        wait until falling_edge(clk_sys);

        -- Generate HW commands
        rand_logic_s(rand_com_gen_ctr, txtb_hw_cmd.lock, 0.2);
        rand_logic_s(rand_com_gen_ctr, txtb_hw_cmd.unlock, 0.2);

        if (txtb_state /= TXT_RDY) then
            txtb_hw_cmd.lock   <= '0';
        end if;

        if (txtb_state /= TXT_TRAN and txtb_state /= TXT_ABTP) then
            txtb_hw_cmd.unlock <= '0';
        end if;
        wait for 0 ns;

        if (txtb_hw_cmd.unlock = '1') then
            rand_real_v(rand_com_gen_ctr, tmp_real);

            if (tmp_real < 0.3) then
                 txtb_hw_cmd.valid  <= '1';
            elsif (tmp_real < 0.6) then
                 txtb_hw_cmd.arbl   <= '1';
            elsif (tmp_real < 0.8) then
                 txtb_hw_cmd.err    <= '1';
            else
                 txtb_hw_cmd.failed <='1';
            end if;

        end if;

        -- Generate SW commands
        rand_logic_s(rand_com_gen_ctr, txtb_sw_cmd.set_rdy, 0.2);
        rand_logic_s(rand_com_gen_ctr, txtb_sw_cmd.set_ety, 0.2);
        rand_logic_s(rand_com_gen_ctr, txtb_sw_cmd.set_abt, 0.2);
        wait for 0 ns;

        -- Calculate the expected state
        calc_exp_state(txtb_sw_cmd, txtb_hw_cmd, txtb_state, txtb_exp_state);

        wait until rising_edge(clk_sys);
        wait until falling_edge(clk_sys);
        -- Check whether the state ended up as expected
        check(txtb_state = txtb_exp_state,
              "State not updated as expected! Actual: " &
	               to_hstring(txtb_state) & " Expected: " &
                   to_hstring(txtb_exp_state));

        -- Set all the commands to be inactive
        txtb_hw_cmd.valid   <= '0';
        txtb_hw_cmd.err     <= '0';
        txtb_hw_cmd.arbl    <= '0';
        txtb_hw_cmd.failed  <= '0';
        txtb_hw_cmd.lock    <= '0';
        txtb_hw_cmd.unlock  <= '0';
        txtb_sw_cmd.set_rdy <= '0';
        txtb_sw_cmd.set_ety <= '0';
        txtb_sw_cmd.set_abt <= '0';

    end process;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
        variable rand_nr    : real;
        variable rand_time  : time;
    begin
        info("Restarting TXT Buffer test!");
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 0, rand_ctr);
        info("Restarted TXT Buffer test");
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        info("Starting TXT Buffer main loop");

        while (loop_ctr < iterations  or  exit_imm)
        loop
            info("Starting loop nr " & integer'image(loop_ctr));
            
            wait until falling_edge(clk_sys);
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);

            -- Just add the errors from two separate processes
            error_ctr   <= state_coh_error_ctr + data_coh_err_ctr;

            loop_ctr    <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

    errors <= error_ctr;

end architecture;