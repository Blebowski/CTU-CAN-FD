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
--  Sanity test simulates real bus operation with 4 CTU CAN FD Cores connected
--  on CAN Bus. Real bus topology with signal delay is implemented via
--  "signalDelayer". Random bit noise is inserted to the bus simulation.
--  Each node is clocked with different clock, to cover clock jitter and clock
--  uncertainty of real bus controllers!
--
--------------------------------------------------------------------------------
-- Revision History:
--
--    7.7.2016   Created file
--    23.9.2017  Added bugfix for proper identifier correction. Identifier
--               layout change from 13.1.2017 caused that identifier correction
--               did not avoid collisions due to assumption of old identifier
--               layout.
--    09.2.2018  Added support fow RWCNT field in the SW_CAN_Frame. RWCNT is
--               stored also to TX Memory. Thus when read on RX side, it should
--               match the calculated value on TX.
--    17.2.2018  1. Modified to support prioritized version of TXT Buffers.
--               2. Added TXT frame counter
--------------------------------------------------------------------------------


Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;

use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity sanity_test is
  port (

    -- Input trigger, test starts running when true
    signal run              :in     boolean;

    -- Number of iterations that test should do
    signal iterations       :in     natural;

    -- Logging level, severity which should be shown
    signal log_level        :in     log_lvl_type;

    -- Test behaviour when error occurs: Quit, or Go on
    signal error_beh        :in     err_beh_type;

    -- Error tolerance, error counter should not exceed this value in order
    -- for the test to pass
    signal error_tol        :in     natural;

    -- Status of the test
    signal status           :out    test_status_type;

    -- Amount of errors which appeared in the test
    signal errors           :out    natural ;
    --TODO: Error log results

    ----------------------------------------------
    -- Test configuration
    ----------------------------------------------

    -- Uncertainties of each clock (in ppm)
    signal epsilon_v        :       epsilon_type;

    -- Transceiver delay parameters
    signal trv_del_v        :       trv_del_type;

    -- Configuration of bus topology
    signal bus_matrix       :       bus_matrix_type;

    -- Number of iterations in Gauss distribution calculation
    signal iter_am          :       natural := 40;

    -- Noise pulse width mean in nanaoseconds
    signal nw_mean          :       real;

    -- Noise pulse width variance
    signal nw_var           :       real;

    -- Gap between two noise pulses mean in nanoseconds
    signal ng_mean          :       real;

    -- Gap variance
    signal ng_var           :       real;

    signal topology         :       string (1 to 50);

    signal timing_config    :       bit_time_config_type
  );
end entity;

architecture behavioral of sanity_test is
    -- Internal test signals
    signal error_ctr        :       natural := 0;
    signal loop_ctr         :       natural := 0;
    signal exit_imm         :       boolean := false;
    signal rand_ctr         :       natural range 0 to 3800 := 0;

    ----------------------------------------------
    ----------------------------------------------
    -- Internal signals
    ----------------------------------------------
    ----------------------------------------------

    -- Bus topology is simulated via delay from each node to each other node ->
    -- Bus matrix is used to store the delays!
    type delay_matrix_type is array(1 to NODE_COUNT, 1 to NODE_COUNT) of time;
    signal delay_matrix : delay_matrix_type;

    -- Memory interfaces
    type mem_bus_arr_type is array (1 to NODE_COUNT) of Avalon_mem_type;
    signal mb_arr : mem_bus_arr_type;

    -- Auxiliarly memory signals used due to no support for connecting
    -- record types to components!
    type mem_vect_arr_type is
        array (1 to NODE_COUNT) of std_logic_vector(31 downto 0);

    type mem_contr_arr_type is
        array (1 to NODE_COUNT) of std_logic;

    type mem_be_arr_type is
        array (1 to NODE_COUNT) of std_logic_vector(3 downto 0);

    type mem_addr_arr_type is
        array (1 to NODE_COUNT) of std_logic_vector(23 downto 0);

    signal mem_aux_data_in      : mem_vect_arr_type :=
                                    (OTHERS => (OTHERS => '0'));

    signal mem_aux_data_out     : mem_vect_arr_type :=
                                    (OTHERS => (OTHERS => '0'));

    signal mem_aux_address      : mem_addr_arr_type :=
                                    (OTHERS => (OTHERS => '0'));

    signal mem_aux_scs          : mem_contr_arr_type :=
                                    (OTHERS => '0');

    signal mem_aux_swr          : mem_contr_arr_type :=
                                    (OTHERS => '0');

    signal mem_aux_srd          : mem_contr_arr_type :=
                                    (OTHERS => '0');

    -- By default all accesses are 32 bit!
    signal mem_aux_sbe          : mem_be_arr_type :=
                                    (OTHERS => (OTHERS => '1'));

    signal mem_aux_clk          : mem_contr_arr_type :=
                                    (OTHERS => '0');

    signal res_n_v              : std_logic_vector(1 to NODE_COUNT) :=
                                    (OTHERS => '0');

    signal int_v                : std_logic_vector(1 to NODE_COUNT) :=
                                    (OTHERS => '0');

    signal CAN_tx_v             : std_logic_vector(1 to NODE_COUNT);
    signal CAN_rx_v             : std_logic_vector(1 to NODE_COUNT) :=
                                    (OTHERS => RECESSIVE);

    signal time_quanta_clk_v    : std_logic_vector(1 to NODE_COUNT) :=
                                    (OTHERS => '0');

    -- Timestamp signals
    type timestamp_arr_type is
        array (1 to NODE_COUNT) of std_logic_vector(63 downto 0);

    signal timestamp_v          : timestamp_arr_type :=
                                    (OTHERS => (OTHERS =>'0'));

    -- Transceiver delay shift registers
    type trv_del_shift_reg is array (1 to NODE_COUNT) of tran_delay_type;

    signal transciever          : trv_del_shift_reg :=
        (OTHERS => ((OTHERS => RECESSIVE), (OTHERS => RECESSIVE), '1', '1'));

    -- Bus realisation signals
    type bus_delayed_type is
        array (1 to NODE_COUNT, 1 to NODE_COUNT) of std_logic;

    signal bus_delayed          : bus_delayed_type :=
                                    (OTHERS => (OTHERS => RECESSIVE));
    signal bus_clk              : std_logic := '0';

    -- Noise registers
    signal noise_reg            : std_logic_vector (1 to NODE_COUNT) :=
                                    (OTHERS => RECESSIVE);

    signal noise_force          : std_logic_vector (1 to NODE_COUNT) :=
                                    (OTHERS => '0');

    -- Traffic storage memories
    type storage_mem_array      is
        array (1 to NODE_COUNT) of test_mem_type;

    signal tx_mems              : storage_mem_array :=
                                    (OTHERS => (OTHERS => (OTHERS => '0')));

    signal rx_mems              : storage_mem_array :=
                                    (OTHERS => (OTHERS => (OTHERS => '0')));

    type storage_pointer_array is
        array (1 to NODE_COUNT) of natural;

    signal tx_mem_pointers      : storage_pointer_array := (OTHERS => 0);
    signal rx_mem_pointers      : storage_pointer_array := (OTHERS => 0);

    signal tx_mem_pointers_done : boolean;

    type mem_status_array_type is
        array (1 to NODE_COUNT) of boolean;

    -- Indicator whether traffic storage memories are full
    signal tx_full              : mem_status_array_type := (OTHERS => false);

    -- Control signals
    type control_bool_array_type is array (1 to NODE_COUNT) of boolean;
    signal do_wait              : control_bool_array_type := (OTHERS => true);
    signal do_config            : control_bool_array_type := (OTHERS => false);
    signal do_traffic           : control_bool_array_type := (OTHERS => false);
    signal do_erase_mems        : control_bool_array_type := (OTHERS => false);
    signal do_read_errors       : control_bool_array_type := (OTHERS => false);
    signal config_done          : control_bool_array_type := (OTHERS => false);
    signal do_restart_mem_if    : control_bool_array_type := (OTHERS => true);
    signal do_noise             : boolean := false;

    signal erp_detected         : control_bool_array_type := (OTHERS => false);
    signal test_desc            : string (1 to 50) :=
        "                                                  ";

    -- Random counters for number generation
    type rand_ctr_array_type is
        array (1 to NODE_COUNT) of natural range 0 to RAND_POOL_SIZE;

    signal rand_ctr_gen         : rand_ctr_array_type;
    signal rand_ident_ctr       : natural range 0 to RAND_POOL_SIZE := 0;

    -- Auxiliarly signal for identifier correction to have similiar identifiers
    -- and interesting arbitration!
    signal common_ident         : natural;

    -- Overall frame counter for the whole test
    type frame_counter_array_type
        is array (1 to NODE_COUNT) of natural;

    -- Frame counters for each node
    signal frame_counters       : frame_counter_array_type;

    -- Frame counter for all frames on the bus
    signal overal_frame_counter : natural;

    ----------------------------------------------
    ----------------------------------------------
    -- Test procedures and functions
    ----------------------------------------------
    ----------------------------------------------


  ------------------------------------------------------------------------------
  -- Check if all frames from TX memory are found in RX memory
  ------------------------------------------------------------------------------
    procedure check_memories(
        signal tx_mems          :  in       storage_mem_array;
        signal rx_mems          :  in       storage_mem_array;
        variable outcome        :  inout    boolean
    )is
        variable tx_r_ptr       :           natural := 0;
        variable rx_r_ptr       :           natural := 0;
        variable TX_frame       :           SW_CAN_frame_type;
        variable RX_frame       :           SW_CAN_frame_type;
        variable comp_out       :           boolean;
        variable detected       :           boolean := false;
        variable node_error     :           boolean := false;
        variable tmp_mem        :           test_mem_type;
    begin
        outcome     := true;
        tx_r_ptr    := 0;
        rx_r_ptr    := 0;

        -- Now we check that content of each TX memory is located in each RX
        -- memory except the one with the same index (node does not recieve its
        -- own frames)!
        for i in 1 to NODE_COUNT loop

            node_error := false;
            tx_r_ptr := 0;

            for j in 1 to NODE_COUNT loop

                if (i /= j) then
                    rx_r_ptr := 0;
                    -- Detect frame based on TBF bit which is always 1. If
                    -- this bit is set we assume frame is there. We have to
                    -- keep policy of erasing the memory properly then!
                    while (tx_mems(i)(tx_r_ptr)(8) = '1') loop

                        -- Read frame from TX Mem
                        tmp_mem := tx_mems(i);
                        read_frame_from_test_mem(TX_frame, tmp_mem, tx_r_ptr);

                        -- Now browse trough the RX mem j and check if we find
                        -- his frame. Set "comp_out" if frame was found.
                        rx_r_ptr := 0;
                        comp_out := false;
                        detected := false;

                        while (rx_mems(j)(rx_r_ptr)(8) = '1' and
                               comp_out = false)
                        loop
                            tmp_mem := rx_mems(j);
                            read_frame_from_test_mem(RX_frame, tmp_mem, rx_r_ptr);
                            CAN_compare_frames(TX_frame, RX_frame, false, comp_out);
                            if (comp_out) then
                                detected := true;
                            end if;
                        end loop;

                        if (detected = false) then
                            outcome:= false;
                            node_error := true;
                        end if;
                    end loop;
                end if;
            end loop;

            -- Print contents of test memories for each node if something
            -- went wrong!
            if (node_error) then
                tx_r_ptr := 0;
                log("TX Memory Node " & integer'image(i) & ":",
                    error_l, log_level);
                while (tx_mems(i)(tx_r_ptr)(8) = '1') loop
                    tmp_mem := tx_mems(i);
                    read_frame_from_test_mem(TX_frame, tmp_mem, tx_r_ptr);
                    CAN_print_frame(TX_frame, error_l);
                end loop;

                log("RX Memory Node " & integer'image(i) & ":",
                    error_l, log_level);
                rx_r_ptr := 0;
                while (rx_mems(i)(rx_r_ptr)(8) = '1') loop
                    tmp_mem := rx_mems(i);
                    read_frame_from_test_mem(RX_frame, tmp_mem, rx_r_ptr);
                    CAN_print_frame(RX_frame, error_l);
                end loop;
            end if;

        end loop;
    end procedure;


    ----------------------------------------------------------------------------
    -- To avoid collisions, we have to make sure that we have different IDs.
    -- So we set last 3 bits to unsigned value of the controller index.
    -- Additionally we modify the identifiers to have first N bits matching
    -- (N is random), so that arbitration is lost in random bit. With absolutely
    -- random identifiers arbitration is usually lost very soon (first 3 bits).
    ----------------------------------------------------------------------------
    procedure correct_identifiers(
        signal   rand_ctr       : inout     natural range 0 to RAND_POOL_SIZE;
        variable frame          : inout     SW_CAN_frame_type;
        constant index          : in        natural;
        signal   com_id         : in        natural
    )is
        variable aux_vect       :           std_logic_vector(28 downto 0);
        variable aux_common     :           std_logic_vector(28 downto 0);
        variable rand_val       :           real;
        variable rand_index     :           natural;
    begin
        ------------------------------------------------------------------------
        -- Correct first bits to have interesting arbitration...
        -- Correct the last bits to avoid collisions
        ------------------------------------------------------------------------
        rand_real_v(rand_ctr, rand_val);
        aux_common := std_logic_vector(to_unsigned(com_id, 29));

        if (frame.ident_type = EXTENDED) then
            rand_index := integer(25.0 * rand_val);
            aux_vect   := std_logic_vector(to_unsigned(frame.identifier, 29));
            aux_vect (28 downto 28 - rand_index) :=
                        aux_common(28 downto 28 - rand_index);
        else
            rand_index := integer(7.0 * rand_val);
            aux_vect   := "000000000000000000" &
                        std_logic_vector(to_unsigned(frame.identifier, 11));
            aux_vect (10 downto 10 - rand_index) :=
                        aux_common(28 downto 28 - rand_index);
        end if;
        aux_vect (2 downto 0)   := std_logic_vector(to_unsigned(index, 3));
        frame.identifier        := to_integer(unsigned(aux_vect));
    end procedure;


    procedure restart_mem_bus(
        signal mem_bus          : out       Avalon_mem_type
    )is
    begin
        mem_bus.scs         <= '0';
        mem_bus.swr         <= '0';
        mem_bus.srd         <= '0';
        mem_bus.address     <= (OTHERS => '0');
        mem_bus.data_in     <= (OTHERS => '0');
        mem_bus.data_out    <= (OTHERS => 'Z');
        mem_bus.clk_sys     <= 'Z';
    end procedure;

    function bus_matrix_to_delay(signal bm : in real) return time is
    begin
        return 10.0 * bm * 500 ps;
    end function;

    type tr_del_t is array (1 to NODE_COUNT) of time;
    signal tr_del : tr_del_t;
    signal tr_tx_and_rx : std_logic_vector(1 to NODE_COUNT);
begin

    ----------------------------------------------------------------------------
    -- CAN Nodes instances
    ----------------------------------------------------------------------------
    comp_gen : for i in 1 to NODE_COUNT generate
        node_1_comp : CAN_top_level
        generic map(
             use_logger       => true,
             rx_buffer_size   => 64,
             use_sync         => true,
             ID               => i,
             logger_size      => 16
        )
        port map(
             res_n            => res_n_v(i),
             clk_sys          => mem_aux_clk(i),
             data_in          => mem_aux_data_in(i),
             data_out         => mem_aux_data_out(i),
             adress           => mem_aux_address(i),
          	 scs              => mem_aux_scs(i),
             srd              => mem_aux_srd(i),
             swr              => mem_aux_swr(i),
             sbe              => mem_aux_sbe(i),
             int              => int_v(i),
             CAN_tx           => CAN_tx_v(i),
             CAN_rx           => CAN_rx_v(i),
             time_quanta_clk  => time_quanta_clk_v(i),
             timestamp        => timestamp_v(i)
        );

        mb_arr(i).clk_sys       <=  mem_aux_clk(i);
        mem_aux_data_in(i)      <=  mb_arr(i).data_in;
        mem_aux_address(i)      <=  mb_arr(i).address;
        mem_aux_scs(i)          <=  mb_arr(i).scs;
        mem_aux_swr(i)          <=  mb_arr(i).swr;
        mem_aux_srd(i)          <=  mb_arr(i).srd;
        mb_arr(i).data_out      <=  mem_aux_data_out(i);
    end generate comp_gen;



    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_generic : for i in 1 to NODE_COUNT generate
        clock_gen_1 : process
            constant period   : natural := f100_Mhz;
            constant duty     : natural := 50;
            variable epsilon  : natural;
        begin
          epsilon := epsilon_v(i);
          generate_clock(period, duty, epsilon, mem_aux_clk(i));
          timestamp_v(i) <= std_logic_vector(unsigned(timestamp_v(i)) + 1);
        end process;
    end generate clock_generic;


    ----------------------------------------------------------------------------
    -- Realisation of transciever delay
    ----------------------------------------------------------------------------
    tr_del_gen : for i in 1 to NODE_COUNT generate
        tr_del(i) <= (trv_del_v(i) / 2 - 2) * f100_mhz * 1 ps
                      when trv_del_v(i)/2 > 1 else
                           trv_del_v(i)/2 * f100_mhz * 1 ps;
        trv_del_gen_tx_delayer : entity work.signal_delayer
            generic map (
                NSAMPLES => 16
            )
            port map (
                input   => CAN_tx_v(i),
                delay   => tr_del(i),
                delayed => transciever(i).tx_point
            );
        tr_tx_and_rx(i) <= (transciever(i).tx_point AND transciever(i).rx_point);
        trv_del_gen_rx_delayer : entity work.signal_delayer
            generic map (
                NSAMPLES => 16
            )
            port map (
                input   => tr_tx_and_rx(i),
                delay   => tr_del(i),
                delayed => CAN_rx_v(i)
            );
    end generate tr_del_gen;


    ----------------------------------------------------------------------------
    -- Bus clock generation
    --
    -- 500 ps equals approximately 10 cm of conductor! This is coarse estimate,
    -- however purpose of this simulation is not to examine exact propagation
    -- delays via different physical channels! This simple estimate is enough.
    ----------------------------------------------------------------------------
    bus_clk_proc : process
    begin
        bus_clk <= '0';
        wait for 250 ps;
        bus_clk <= '1';
        wait for 250 ps;
    end process;


    ----------------------------------------------------------------------------
    -- Realisation of delay on CAN Bus via signal delayer
    ----------------------------------------------------------------------------
    bus_gen_delay_tx : for i in 1 to NODE_COUNT generate
        bus_gen_delay_tx2 : for j in 1 to NODE_COUNT generate

            delay_matrix(j, i) <= bus_matrix_to_delay(bus_matrix(j, i));

            i_txdelay : entity work.signal_delayer
                generic map (
                    NSAMPLES    => 16
                )
                port map (
                    input       => transciever(i).tx_point,
                    delay       => delay_matrix(j,i),
                    delayed     => bus_delayed(i,j)
                );

        end generate bus_gen_delay_tx2;
    end generate bus_gen_delay_tx;

    ----------------------------------------------------------------------------
    -- Realisation of CAN bus with delayed signals
    ----------------------------------------------------------------------------
    bus_gen : for i in 1 to NODE_COUNT generate
        bus_gen_proc : process
            variable rx_lvl : std_logic := RECESSIVE;
            variable index  : natural   := 0;
        begin
            wait until rising_edge(bus_clk);
            rx_lvl := RECESSIVE;

            -- In one recieving node iterate trough all nodes
            -- and AND all delayed signals.
            for j in 1 to NODE_COUNT loop
                rx_lvl := rx_lvl AND bus_delayed(j, i);
            end loop;

            -- Now add the Generated noise to the recieving node
            if (noise_force(i) = '1') then
                rx_lvl := noise_reg(i);
            end if;

            transciever(i).rx_point <= rx_lvl;
        end process;
    end generate bus_gen;

    ----------------------------------------------------------------------------
    -- Noise generator
    ----------------------------------------------------------------------------
    noise_gen_proc:process
        variable noise_time : real := 0.0;
        variable noise_gap  : real := 0.0;
        variable aux        : std_logic_vector(3 downto 0);
        variable gaus_par   : rand_distribution_par_type;
        variable exp_par    : rand_distribution_par_type;
    begin
        if (do_noise) then

            -- Generate noise pulse with gauss distribution
            gaus_par(GAUSS_iterations)  := real(iter_am);
            gaus_par(GAUSS_mean)        := nw_mean;
            gaus_par(GAUSS_variance)    := nw_var;
            rand_real_distr_v(rand_ctr, noise_time, GAUSS, gaus_par);

            -- Generate noise pulse gap with exponential distribution
            exp_par(EXPONENTIAL_mean)   := ng_mean;
            rand_real_distr_v(rand_ctr, noise_gap, EXPONENTIAL, exp_par);

            noise_force <= (OTHERS => '0');
            wait for integer(noise_gap) * 1 ns;

            --Generate noise polarity and info whether noise
            -- should be forced to the node
            -- We cant put noise to all. That would be global
            -- error at all times
            rand_logic_vect_v(rand_ctr ,aux,0.5);
            noise_reg   <= aux;
            rand_logic_vect_v(rand_ctr ,aux,0.5);
            noise_force <= aux;

            wait for integer(noise_time) * 1 ns;

        else
            wait for 10 ns;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Common identifier generation we have to set some common base for
    -- identifiers to have interesting arbitration...
    ----------------------------------------------------------------------------
    ident_gen_proc : process
        variable rand_value : real := 0.0;
    begin
        for i in 0 to 100 loop
          wait until rising_edge(mb_arr(1).clk_sys);
        end loop;
        rand_real_v(rand_ident_ctr, rand_value);
        common_ident <= integer(rand_value * 536870911.0);
    end process;


    ----------------------------------------------------------------------------
    -- Calculation of frame counter
    ----------------------------------------------------------------------------
    overal_frame_counter <= (frame_counters(1) + frame_counters(2) +
                             frame_counters(3) + frame_counters(4)) / 3;


    ------------------------------------------------------------------------------
    -- Node access - Traffic generation and error states
    ------------------------------------------------------------------------------
    access_gen : for i in 1 to NODE_COUNT generate

        node_access_proc:process
            variable rand_real      :       real;
            variable TX_frame       :       SW_CAN_frame_type;
            variable RX_frame       :       SW_CAN_frame_type;
            variable r_data         :       std_logic_vector(31 downto 0);
            variable n_index        :       natural := i;
            variable frame_sent     :       boolean := false;
            variable used_txtb      :       natural := 1;
            variable txtb_state     :       SW_TXT_Buffer_state_type;
            variable rx_buf_state   :       SW_RX_Buffer_info;
            variable fault_state    :       SW_fault_state;
        begin
            if (do_restart_mem_if(i)) then
                restart_mem_bus(mb_arr(i));
                wait for 10 ns;

            elsif (do_wait(i)) then
                wait until rising_edge(mb_arr(i).clk_sys);

            elsif (do_erase_mems(i)) then
                tx_mems(i)          <= (OTHERS => (OTHERS => '0'));
                rx_mems(i)          <= (OTHERS => (OTHERS => '0'));
                tx_mem_pointers(i)  <= 0;
                rx_mem_pointers(i)  <= 0;
                erp_detected(i)     <= false;
                wait for 10 ns;
            else
                if (do_config(i)) then

                    config_done (i) <= false;

                    -- Perform the configuration of the node
                    CAN_turn_controller(true, n_index, mb_arr(i));
                    CAN_configure_timing(timing_config, n_index, mb_arr(i));

                    -- Signal back to main process config finished
                    config_done(i)  <= true;
                    wait for 100 ns;

                else
                    if (do_traffic(i)) then

                        -- Check if first TXT Buffer is Empty or previous
                        -- transmission was Done...
                        get_tx_buf_state(used_txtb, txtb_state, n_index,
                                         mb_arr(i));

                        -- Send frame if yes...
                        if (txtb_state = buf_empty or txtb_state = buf_done) then

                            -- Generate and transmitt frames until TX memory
                            -- of given node is full.
                            if (not tx_full(i)) then

                                -- To avoid having the same frames from all nodes
                                -- we modify "rand_counter" by index of node
                                if (rand_ctr_gen(i) + i > RAND_POOL_SIZE) then
                                    rand_ctr_gen(i) <= 0;
                                else
                                    rand_ctr_gen(i) <= rand_ctr_gen(i) + i;
                                end if;
                                wait for 0 ns;
                                CAN_generate_frame(rand_ctr_gen(i), TX_frame);

                                -- Now we have absolutely RANDOM identifiers
                                -- To avoid collisions we correct them to have
                                -- unique endings...
                                correct_identifiers(rand_ctr_gen(i),
                                                    TX_frame, i, common_ident);

                                -- Finally insert the frame for transmission
                                -- and also to test memories!
                                CAN_send_frame(TX_frame, 1, n_index, mb_arr(i),
                                               frame_sent);

                                store_frame_to_test_mem(TX_frame, tx_mems(i),
                                                        tx_mem_pointers(i));
                            end if;
                        end if;

                        -- Check if RX buffer is non-empty and read frames if not!
                        get_rx_buf_state(rx_buf_state, n_index, mb_arr(i));
                        while (rx_buf_state.rx_empty = false) loop
                            CAN_read_frame(RX_frame, n_index, mb_arr(i));
                            store_frame_to_test_mem(RX_frame, rx_mems(i),
                                                    rx_mem_pointers(i));

                            get_rx_buf_state(rx_buf_state, n_index, mb_arr(i));

                            -- Count the received frames
                            frame_counters(i) <= frame_counters(i) + 1;
                        end loop;

                    end if;

                    -- Check if unit is not error passive. If node is error
                    -- passive data consitency will be corrupted!!!
                    -- That should never happend !!!
                    if (do_read_errors(n_index)) then
                        get_fault_state(fault_state, n_index, mb_arr(i));
                        if (fault_state /= fc_error_active) then
                            erp_detected(i) <= true;
                        end if;
                    end if;
                end if;
            end if;
        end process;

        ----------------------------------------------
        -- Memory statuses
        ----------------------------------------------
        -- Note that we keep reserve of up to 60 words, thats
        -- exactly how much we need to fit 3 times longest frame...
        tx_full(i) <= true when
                           tx_mem_pointers(i) + 245 >= tx_mems(i)'length
                           else
                      false;

    end generate access_gen;


    tx_mem_pointers_done <= true when
                                    ((tx_mem_pointers(1) +
                                      tx_mem_pointers(2) +
                                      tx_mem_pointers(3)) =
                                     rx_mem_pointers(4))
                                    and
                                    ((tx_mem_pointers(2) +
                                      tx_mem_pointers(3) +
                                      tx_mem_pointers(4)) =
                                     rx_mem_pointers(1))
                                    and
                                    ((tx_mem_pointers(3) +
                                      tx_mem_pointers(4) +
                                      tx_mem_pointers(1)) =
                                     rx_mem_pointers(2))
                                    and
                                    ((tx_mem_pointers(1) +
                                      tx_mem_pointers(2) +
                                      tx_mem_pointers(4)) =
                                     rx_mem_pointers(3))
                                else
                            false;


    ----------------------------------------------------------------------------
    -- Main test process
    ----------------------------------------------------------------------------
    test_proc : process
        variable outcome    : boolean  := true;
        variable step_done  : boolean  := false;
    begin
        print_test_info(iterations, log_level, error_beh, error_tol);
        -- TODO: print sanity test params

        wait for 5 ns;
        do_restart_mem_if <= (OTHERS => false);
        wait for 5 ns;
        reset_test(res_n_v(1), status, run, error_ctr);
        reset_test(res_n_v(2), status, run, error_ctr);
        reset_test(res_n_v(3), status, run, error_ctr);
        reset_test(res_n_v(4), status, run, error_ctr);
        loop_ctr <= 0;

        -- Deactivate the wait signal
        do_wait <= (OTHERS => false);

        log("Applying Configuration...", info_l, log_level);
        for i in 1 to NODE_COUNT loop
            do_config(i) <= true;
        end loop;

        wait until config_done = (true, true, true, true);
        for i in 1 to NODE_COUNT loop
            do_config(i)      <= false;
            do_read_errors(i) <= true;
        end loop;
        log("Configuration applied", info_l, log_level);

        ------------------------------------------------------------------------
        -- Main loop of the test
        ------------------------------------------------------------------------
        while (loop_ctr < iterations  or  exit_imm) loop
            log("Starting loop nr " & integer'image(loop_ctr),
                info_l, log_level);

            -- Here is a special case when previous step failed.
            -- We need to reset and reconfigure node
            if (erp_detected(1) or erp_detected(2) or
                erp_detected(3) or erp_detected(4))
            then
                res_n_v <= (OTHERS => '0');
                wait for 50 ns;
                res_n_v <= (OTHERS => '1');
                wait for 10 ns;

                do_config <= (OTHERS => true);
                wait for 1000 ns;
                do_config <= (OTHERS => false);
                wait for 10 ns;
            end if;

            -- Erase the test memories
            for i in 1 to NODE_COUNT loop
                do_erase_mems(i) <= true;
            end loop;
            wait for 1000 ns;

            for i in 1 to NODE_COUNT loop
                do_erase_mems(i) <= false;
            end loop;

            -- Start the traffic generation
            for i in 1 to NODE_COUNT loop
                do_traffic(i)     <= true;
                do_noise          <= true;
            end loop;

            wait for 2000 ns;

            -- Units stop transmiting when they reach tx_full! We wait until
            -- everything what was transmitted is also recieved!
            step_done :=false;
            while (step_done = false) loop
                if (tx_mem_pointers_done) then
                  step_done:= true;
                end if;

                -- If any of the nodes turned error passive
                if (erp_detected(1) or erp_detected(2) or
                    erp_detected(3) or erp_detected(4))
                then
                    step_done:= true;
                    report "Some unit turned error passive -> " &
                           "Most probably traffic consitency check will fail!"
                    severity error;
                end if;
                wait for 100 ns;
            end loop;

            -- Stop the traffic generation
            for i in 1 to NODE_COUNT loop
                do_traffic(i)     <= false;
                do_noise          <= false;
            end loop;

            wait for 200 ns;

            -- Now evaluate the memories content...
            check_memories(tx_mems, rx_mems, outcome);

            if (outcome = false) then
                log("Traffic consitency check error!", error_l, log_level);
                process_error(error_ctr, error_beh, exit_imm);
            end if;

            wait for 1000 ns;
            loop_ctr <= loop_ctr + 1;
            wait for 0 ns;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

  errors <= error_ctr;

end architecture;

architecture sanity_test of CAN_test is
    signal epsilon_v    : epsilon_type := (0,150,300,450);
    signal trv_del_v    : trv_del_type := (5,5,5,5);

    signal bus_matrix   : bus_matrix_type := (  ( 0.0,10.0,20.0,30.0),
                                                (10.0, 0.0,10.0,20.0),
                                                (20.0,10.0, 0.0,10.0),
                                                (30.0,20.0,10.0, 0.0));
    ---------------------
    -- Noise parameters
    ---------------------
    signal iter_am     : natural := 40;

    -- Noise pulse width mean in nanaoseconds
    signal nw_mean     : real := 15.0;

    -- Noise pulse width variance
    signal nw_var      : real := 5.0;

    -- Gap between two noise pulses mean in nanoseconds
    signal ng_mean     : real := 10000.0;

    -- Gap variance
    signal ng_var      : real :=  6000.0;
    signal topology    : string (1 to 50) :=
        "                                                  ";

    -- With 100 MHz this is 1Mbit(nom)/2Mbit(Data)
    signal timing_config : bit_time_config_type := (5,5,9,5,5,3,3,3,3,2);
begin
    i_st: entity work.sanity_test
    port map(
        run             => run,
        iterations      => iterations,
        log_level       => log_level,
        error_beh       => error_beh,
        error_tol       => error_tol,
        status          => status,
        errors          => errors,
        epsilon_v       => epsilon_v,
        trv_del_v       => trv_del_v,
        bus_matrix      => bus_matrix,
        iter_am         => iter_am,
        nw_mean         => nw_mean,
        nw_var          => nw_var,
        ng_mean         => ng_mean,
        ng_var          => ng_var,
        topology        => topology,
        timing_config   => timing_config
    );
end architecture;
