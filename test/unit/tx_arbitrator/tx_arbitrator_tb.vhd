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
--  Unit test for TX Arbitrator circuit
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
--    30.5.2016   Created file
--    23.4.2018   Updated test to cover TX Arbitrator with continous timestamp
--                load.
--    5.9.2018    Relaxed timing conditions. Allowed one clock cycle mismatch
--                between DUT and model. 
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
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;
use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.can_unit_test_pkg.all;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

library vunit_lib;
context vunit_lib.vunit_context;

architecture tx_arbitrator_unit_test of CAN_test is

    ------------------------
    -- DUT signals
    ------------------------
    signal clk_sys                :  std_logic;
    signal res_n                  :  std_logic := '0';
    signal txtb_port_b_data       :  t_txt_bufs_output(C_TXT_BUFFER_COUNT - 1 downto 0) :=
                                        (OTHERS => (OTHERS => '0'));

    signal txtb_available         :  std_logic_vector(C_TXT_BUFFER_COUNT - 1 downto 0)
                                        := (OTHERS => '0');

    signal txtb_port_b_address               :  natural range 0 to 19;
    signal tran_word     :  std_logic_vector(31 downto 0);
    signal tran_dlc           :  std_logic_vector(3 downto 0);
    signal tran_is_rtr            :  std_logic;
    signal tran_ident_type    :  std_logic;
    signal tran_frame_type    :  std_logic;
    signal tran_brs           :  std_logic;
    signal tran_frame_valid   :  std_logic;
    signal txtb_hw_cmd            :  t_txtb_hw_cmd :=
                                        ('0','0','0','0','0','0');
    signal txtb_changed           :  std_logic;
    signal txtb_hw_cmd_index      :  natural range 0 to C_TXT_BUFFER_COUNT - 1;
    signal txtb_ptr      :  natural range 0 to 19 := 0;
    signal txtb_prorities         :  t_txt_bufs_priorities(C_TXT_BUFFER_COUNT - 1 downto 0) :=
                                        (OTHERS => (OTHERS => '0'));
    signal timestamp              :  std_logic_vector(63 downto 0) :=
                                        (OTHERS => '0');

    ----------------------------------------------------------------------------
    -- Internal TB signals
    ----------------------------------------------------------------------------

    -- Memories as if connected to TXT Buffers
    type txtb_test_mem_type is array (0 to 19) of std_logic_vector(31 downto 0);
    type txtb_multi_test_type is array (0 to C_TXT_BUFFER_COUNT - 1) of
         txtb_test_mem_type;
    signal shadow_mem             : txtb_multi_test_type :=
                                    (OTHERS => (OTHERS => (OTHERS => '0')));

    -- Random pool pointers
    signal rand_ctr_1             : natural range 0 to RAND_POOL_SIZE;
    signal rand_ctr_2             : natural range 0 to RAND_POOL_SIZE;
    signal rand_ctr_3             : natural range 0 to RAND_POOL_SIZE;
    signal rand_ctr_4             : natural range 0 to RAND_POOL_SIZE;

    -- Highest priority buffer which is ready
    signal high_prio_buf_index    : natural range 0 to C_TXT_BUFFER_COUNT - 1;
    signal high_prio_buf_index_d  : natural range 0 to C_TXT_BUFFER_COUNT - 1;
    signal high_prio_valid        : boolean;
    signal high_prio_valid_d      : boolean;

     -- Modeled outputs
     signal mod_dlc_out           :  std_logic_vector(3 downto 0) := "0000";
     signal mod_is_rtr            :  std_logic := '0';
     signal mod_ident_type_out    :  std_logic := '0';
     signal mod_frame_type_out    :  std_logic := '0';
     signal mod_brs_out           :  std_logic := '0';

     -- Committed output of the frame_valid
     signal mod_frame_com         :  std_logic := '0';

     signal mod_buf_index         :  natural range 0 to C_TXT_BUFFER_COUNT - 1 :=
                                     0;
     signal mod_frame_valid_out   :  std_logic := '0';

    -- Model is locked (as if transmission in progress)
    signal mod_locked             :  boolean := false;

    -- Last locked buffer to detect functionality of txtb_changed
    signal last_locked_index      : natural range 0 to C_TXT_BUFFER_COUNT - 1 :=
                                    0;

    -- Error counters
    signal cmp_err_ctr            :  natural := 0;

    -- Delay propagation of metadata to the output!
    signal del_counter            :  natural;

    -- Signals indicating mismatch between Model and DUT
    signal frame_valid_mism       :  boolean;
    signal metadata_mism          :  boolean;
    signal hw_cmd_buf_index_mism  :  boolean;
    signal sel_buf_mism           :  boolean;
    signal mism_ctr               :  natural := 0;

    signal ts_preset        : std_logic_vector(2 downto 1) := "00";
    signal ts_preset_val    : std_logic_vector(63 downto 0) := (OTHERS => '0');

    ----------------------------------------------------------------------------
    -- Compare function for two 64 bit std logic vectors
    ----------------------------------------------------------------------------
    function less_than(
        signal   a       : in std_logic_vector(63 downto 0);
        signal   b       : in std_logic_vector(63 downto 0)
    ) return boolean is
    begin
        if (unsigned(a(63 downto 32)) < unsigned(b(63 downto 32))) or
          ((a(63 downto 32) = b(63 downto 32)) and
          (unsigned(a(31 downto 0)) < unsigned(b(31 downto 0))))
        then
            return true;
        else
            return false;
        end if;
     end function;


    ----------------------------------------------------------------------------
    -- Setting TXT Buffer priorities as from user registers
    ----------------------------------------------------------------------------
    procedure set_priorities(
        signal rand_ptr               :inout natural range 0 to RAND_POOL_SIZE;
        signal txtb_prorities           :out   t_txt_bufs_priorities
    )is
        variable tmp                  : std_logic_vector(2 downto 0);
    begin
        for i in 0 to C_TXT_BUFFER_COUNT - 1 loop
            rand_logic_vect_v(rand_ptr, tmp, 0.5);
            txtb_prorities(i)      <= tmp;
        end loop;
    end procedure;


begin

    ----------------------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------------------
    tx_arbitrator_comp : entity ctu_can_fd_rtl.tx_arbitrator
    generic map(
        G_TXT_BUFFER_COUNT     => C_TXT_BUFFER_COUNT
    )
    port map(
        clk_sys                => clk_sys,
        res_n                  => res_n,
        txtb_port_b_data       => txtb_port_b_data,
        txtb_available         => txtb_available,
        txtb_port_b_address    => txtb_port_b_address,
        txtb_port_b_clk_en     => open,
        tran_word              => tran_word,
        tran_dlc               => tran_dlc,
        tran_is_rtr            => tran_is_rtr,
        tran_ident_type        => tran_ident_type,
        tran_frame_type        => tran_frame_type,
        tran_brs               => tran_brs,
        tran_frame_valid       => tran_frame_valid,
        txtb_hw_cmd            => txtb_hw_cmd,
        txtb_changed           => txtb_changed,
        txtb_hw_cmd_index      => txtb_hw_cmd_index,
        txtb_ptr               => txtb_ptr,
        txtb_clk_en            => '1',
        drv_bus                => (others => '0'),
        txtb_prorities         => txtb_prorities,
        timestamp              => timestamp
    );


    ----------------------------------------------------------------------------
    -- Emulate Ready state of the buffers!
    ----------------------------------------------------------------------------
    buf_em_proc : process
        variable wait_time_r    : real;
        variable wait_time      : time;
        variable buf_index      : real;
        variable ready          : std_logic;
    begin

        -- Wait up to 500 ns
        rand_real_v(rand_ctr_1, wait_time_r);
        wait_time_r := wait_time_r * 500.0;
        wait_time   := wait_time_r * 1 ns;
        wait for wait_time;
        wait until rising_edge(clk_sys);

        if (res_n = '0') then
            apply_rand_seed(seed, 3, rand_ctr_1);
        end if;

        -- Choose random TXT Buffer
        rand_real_v(rand_ctr_1, buf_index);
        buf_index := buf_index * real(C_TXT_BUFFER_COUNT - 1);

        ------------------------------------------------------------------------
        -- Check whether the buffer is not locked by the Core, in this case one
        -- can't change the buffer state not to be ready. SW could only send
        -- command to abort transmission.
        ------------------------------------------------------------------------
        if ((mod_locked = false or
             mod_buf_index /= integer(buf_index))) and
            (txtb_hw_cmd.lock = '0')
        then
            -- Choose whether buffer will be set to ready or not
            rand_logic_v(rand_ctr_1, ready, 0.1);

            -- Set it to the buffer
            txtb_available(integer(buf_index)) <= ready;
        end if;

    end process;


    ----------------------------------------------------------------------------
    -- Emulate Access to buffers when not ready
    ----------------------------------------------------------------------------
    buf_acc_proc : process
        variable tmp            : std_logic_vector(31 downto 0);
        variable extra_time     : std_logic_vector(7 downto 0);
        variable time_to_tx     : std_logic_vector(31 downto 0);
        variable buf_index      : real;
    begin

        if (res_n = '0') then
            apply_rand_seed(seed, 2, rand_ctr_4);
        end if;

        -- Choose random TXT Buffer
        rand_real_v(rand_ctr_4, buf_index);
        buf_index := buf_index * real(C_TXT_BUFFER_COUNT - 1);

        -- Wait till it can be accessed
        while txtb_available(integer(buf_index)) = '1' loop
            wait until rising_edge(clk_sys);
        end loop;

        -- Fill the buffer with random data
        for i in 0 to 19 loop
            rand_logic_vect_v(rand_ctr_4, tmp, 0.5);
            shadow_mem(integer(buf_index))(i) <= tmp;
        end loop;

        ------------------------------------------------------------------------
        -- Make sure that timestamp words in the buffer have some normal value,
        -- otherwise no buffer would ever get on output...
        -- Set the time to transmit to actual timestamp + some extra time
        ------------------------------------------------------------------------
        shadow_mem(integer(buf_index))(3) <= timestamp(63 downto 32);
        rand_logic_vect_v(rand_ctr_4, extra_time, 0.3);
        time_to_tx  := std_logic_vector(to_unsigned(
                        to_integer(unsigned(timestamp(31 downto 0))) +
                        to_integer(unsigned(extra_time)), 32));
        shadow_mem(integer(buf_index))(2) <= time_to_tx;

        ------------------------------------------------------------------------
        -- Wait some extra time not to have too many data changes in buffer and
        -- avoid confusion during test bring-up!
        ------------------------------------------------------------------------
        wait for 5000 ns;

    end process;


    ------------------------------------------------------------------------------
    -- Connect TX Arbitrator to the shadow memories which emulate TXT Buffers
    -- Note that data must be returned one clock cycle later exactly as TXT
    -- Buffer does it!
    ------------------------------------------------------------------------------
    buf_access_emu_proc : process (res_n, clk_sys)
    begin
        if (res_n = '0') then
             txtb_port_b_data <= (OTHERS => (OTHERS => 'U'));
        elsif (rising_edge(clk_sys)) then
             for i in 0 to C_TXT_BUFFER_COUNT - 1 loop
                 txtb_port_b_data(i) <= shadow_mem(i)(txtb_port_b_address);
             end loop;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Model TX Arbitrator. Choose Highest priority "ready" TXT Buffer.
    -- This should model the "priorityDecoder" entity inside TX Arbitrator.
    ----------------------------------------------------------------------------
    prio_dec_model_proc : process(txtb_available, txtb_prorities)
        variable tmp_index    : natural;
        variable tmp_prio     : natural;
        variable txt_valid    : boolean;
    begin


        ------------------------------------------------------------------------
        -- By default we have to assume the highest buffer
        -- This is default returned index by priority decoder when there is no
        -- valid frame!
        ------------------------------------------------------------------------
        tmp_index           := C_TXT_BUFFER_COUNT - 1;
        tmp_prio            := 0;
        txt_valid           := false;

        -- Choose highest priority TXT buffer
        for i in C_TXT_BUFFER_COUNT - 1 downto 0 loop
            if ((to_integer(unsigned(txtb_prorities(i))) >= tmp_prio) and
                txtb_available(i) = '1')
            then
                tmp_index   := i;
                tmp_prio    := to_integer(unsigned(txtb_prorities(i)));
                txt_valid   := true;
            end if;
        end loop;

        -- Update the buffer
        high_prio_buf_index <= tmp_index;
        high_prio_valid     <= txt_valid;
    end process;


    ----------------------------------------------------------------------------
    -- Model delay of combinationally selected value. This must be done, to make
    -- sure that If TX Arbitrator selection is in progress, selection process
    -- will restart if selected buffer index has changed!
    ----------------------------------------------------------------------------
    prio_dec_delay_proc : process(clk_sys)
    begin
        if (rising_edge(clk_sys)) then
            high_prio_buf_index_d <= high_prio_buf_index;
            high_prio_valid_d     <= high_prio_valid;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Emulate selection process! Based on chosen highest priority buffer which
    -- is ready, update outputs with 6 clock cycle delay!
    -- This can be done only when TX Arbitrator is not locked for transmission!
    -- If buffer index changes, restart the selection process!
    ----------------------------------------------------------------------------
    tx_time_proc : process
        variable update     : boolean       := false;
        variable tmp_timest : std_logic_vector(63 downto 0) := (OTHERS => '0');
    begin

        update 		:= false;
        tmp_timest 	:= shadow_mem(integer(high_prio_buf_index))(3) &
                       shadow_mem(integer(high_prio_buf_index))(2);

        -- Model is locked
        if (mod_locked = true) then
            wait until rising_edge(clk_sys);
            del_counter   <= 1;

        -- Buffer became invalid
        elsif (high_prio_valid = false) then
            mod_frame_com <= '0';
            del_counter   <= 1;
            wait until rising_edge(clk_sys);

        -- Selected buffer has changed
        elsif (high_prio_buf_index /= high_prio_buf_index_d) then
            del_counter   <= 1;
            wait until rising_edge(clk_sys);

        -- Timestamp not yet elapsed! As if waiting now only two more clock
        -- cycles for selection of frame format word!
        elsif (to_integer(unsigned(tmp_timest)) >=
               to_integer(unsigned(timestamp)))
        then
            del_counter   <= 3;
            wait until rising_edge(clk_sys);

        -- HW Command lock came on previously loaded Frame / Buffer
        elsif (txtb_hw_cmd.lock = '1') then
            del_counter   <= 0;
            wait until rising_edge(clk_sys);

        -- Count the delay
        else
            if (del_counter < 6) then
                del_counter <= del_counter + 1;
                update 	    := false;
            else
                update      := true;
            end if;

            if (update = true) then
                -- Propagate metadata to the output
                mod_buf_index        <= high_prio_buf_index;
                mod_dlc_out          <= shadow_mem(high_prio_buf_index)(0)
                                                   (DLC_H downto DLC_L);
                mod_is_rtr           <= shadow_mem(high_prio_buf_index)(0)
                                                   (RTR_IND);
                mod_ident_type_out   <= shadow_mem(high_prio_buf_index)(0)
                                                   (IDE_IND);
                mod_frame_type_out   <= shadow_mem(high_prio_buf_index)(0)
                                                   (FDF_IND);
                mod_brs_out          <= shadow_mem(high_prio_buf_index)(0)
                                                   (BRS_IND);
                mod_frame_com        <= '1';
            end if;

            wait until rising_edge(clk_sys);
        end if;

    end process;

    mod_frame_valid_out <= '1' when ((mod_frame_com = '1' and
                                    high_prio_valid = true) or
                                    (mod_locked = true))
                             else
                         '0';


    ----------------------------------------------------------------------------
    -- Model LOCK and UNLOCK commands as if coming from CAN Core.
    ----------------------------------------------------------------------------
    cmd_mod_proc : process
        variable wait_time      : time;
        variable wait_time_r    : real;
    begin

        -- Wait till test start
        while res_n = '0' loop
            wait until rising_edge(clk_sys);
            apply_rand_seed(seed, 1, rand_ctr_3);
        end loop;

        -- Lock command can be generated only when the output of DUT is valid
        while tran_frame_valid = '0' loop
            wait until rising_edge(clk_sys);
        end loop;

        ------------------------------------------------------------------------
        -- One clock cycle must elapse, to model synchronous response from
        -- CAN Core!
        ------------------------------------------------------------------------
        wait until rising_edge(clk_sys);

        -- Frame could have been invalidated by SW since then
        if (tran_frame_valid = '1') then
            -- Lock the Buffer
            txtb_hw_cmd.lock     <= '1';
            mod_locked          <= true;
            wait until rising_edge(clk_sys);
            txtb_hw_cmd.lock     <= '0';

            last_locked_index   <= mod_buf_index;

            --------------------------------------------------------------------
            -- Wait some time (as if Frame transmission was in progress). Have
            -- realistic waiting time (at least 1000 ns) to cover for SOF bit.
            --------------------------------------------------------------------
            rand_real_v(rand_ctr_3, wait_time_r);
            wait_time_r         := wait_time_r * 10000.0;
            wait_time_r         := wait_time_r + 1000.0;
            wait_time           := wait_time_r * 1 ns;
            wait for wait_time;
            wait until rising_edge(clk_sys);

            -- Unlock the Buffer
            txtb_hw_cmd.unlock   <= '1';
            wait until rising_edge(clk_sys);
            mod_locked          <= false;
            txtb_hw_cmd.unlock   <= '0';

            --------------------------------------------------------------------
            -- Before the next possible LOCK command, buffers must be evaluated.
            -- This takes up to 3 states of TX Arbitrator FSM.
            -- We can be less strict here, since unlock occurs at the end of the
            -- frame (either Normal or Error).
            -- After that there is at least 3 bits of interframe space. Assuming
            -- 10 clock cycles per bit, this gives at least 30 clock cycles!
            --------------------------------------------------------------------
            for i in 1 to 30 loop
                wait until rising_edge(clk_sys);
            end loop;
        end if;

    end process;


    ----------------------------------------------------------------------------
    -- Compare DUT outputs with model outputs
    ----------------------------------------------------------------------------
    frame_valid_mism <= true when (mod_frame_valid_out /= tran_frame_valid)
                             else
                        false;

    metadata_mism <= true when 
                             ((mod_dlc_out            /= tran_dlc) or
                              (mod_is_rtr             /= tran_is_rtr) or
                              (mod_ident_type_out     /= tran_ident_type) or
                              (mod_frame_type_out     /= tran_frame_type))
                          else
                     false;

    hw_cmd_buf_index_mism <= true when (txtb_hw_cmd_index /= mod_buf_index)
                             else
                        false;

    sel_buf_mism <= true when (last_locked_index /= mod_buf_index and
                               txtb_changed = '0' and
                               txtb_hw_cmd.lock = '1')
                         else
                    false;


    ----------------------------------------------------------------------------
    -- Allow only one clock cycle mismatch!
    ----------------------------------------------------------------------------
    cons_check_proc : process
    begin
        if (frame_valid_mism or metadata_mism or sel_buf_mism or
            hw_cmd_buf_index_mism)
        then
            mism_ctr <= mism_ctr + 1;
        else
            mism_ctr <= 0;
        end if;

        wait for 0 ns;

        check(mism_ctr < 2,
              "Mismatch between DUT and Model for more than 2 clock cycles!");

        wait until falling_edge(clk_sys);
    end process;


    ----------------------------------------------------------------------------
    -- Clock and timestamp generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);
    timestamp_gen_proc(clk_sys, timestamp, ts_preset(1), ts_preset_val);

    errors <= error_ctr;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
        variable outcome : boolean;
    begin
        info("Restarting TX Arbitrator test!");
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 0, rand_ctr_2);
        info("Restarted TX Arbitrator test");
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        info("Starting main loop");

        while (loop_ctr < iterations  or  exit_imm)
        loop
            info("Starting loop nr " & integer'image(loop_ctr));

            -- Configure TXT Buffer priorities!
            set_priorities(rand_ctr_2, txtb_prorities);

            wait for 5000 ns;

            error_ctr <= cmp_err_ctr;

            wait until rising_edge(clk_sys);

            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
  end process;

end architecture;
