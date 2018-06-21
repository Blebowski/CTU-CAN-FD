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
--  Main environment for reference test.
--
--  TODO: Description.
--
--------------------------------------------------------------------------------
-- Revision History:
--    20.6.2018   Created file
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

use STD.textio.all;
use IEEE.std_logic_textio.all;
use work.CAN_FD_frame_format.all;


entity CAN_reference_test is
    generic (
        constant seed         :in   natural := 0;
        constant config_path  :in   string := 
			"/DOKUMENTY/Skola/CVUT-FEL/CAN_FD_IP_Core/test/reference" &
            "/configs/log_500Kb_2Mb_80p_1K_samples_1"
    );
    port (
        signal run            :in   boolean := true;
        signal iterations     :in   natural := 50;
        signal log_level      :in   log_lvl_type := info_l;
        signal error_beh      :in   err_beh_type := go_on;
        signal error_tol      :in   natural := 0;
        signal status         :out  test_status_type;
        signal errors         :out  natural
    );
end entity;


architecture CAN_reference_test of CAN_reference_test is

    ----------------------------------------------------------------------------
    -- CAN Core interface
    ----------------------------------------------------------------------------    
    signal clk_sys          : std_logic := '0';
    signal res_n            : std_logic := ACT_RESET;
    signal int              : std_logic;
    signal CAN_tx           : std_logic;
    signal CAN_rx           : std_logic := RECESSIVE;
    signal time_quanta_clk  : std_logic;
    signal timestamp        : std_logic_vector(63 downto 0) := (OTHERS => '0');
    signal data_in          : std_logic_vector(31 downto 0) := (OTHERS => '0');
    signal data_out         : std_logic_vector(31 downto 0);
    signal adress           : std_logic_vector(23 downto 0) := (OTHERS => '0');
    signal scs              : std_logic := '0';
    signal srd              : std_logic := '0';
    signal swr              : std_logic := '0';
    signal sbe              : std_logic_vector(3 downto 0) := "0000";

    signal drv_bus          : std_logic_vector(1023 downto 0);
    signal stat_bus         : std_logic_vector(511 downto 0);

    signal mem_bus          : Avalon_mem_type := ('0', (OTHERS => '0'), 
                                 (OTHERS => '0'), (OTHERS => '0'), '0', '0', '0',
                                (OTHERS => '0'));

	----------------------------------------------------------------------------
	-- 500 Kb/s Nominal, 2 Mb/s Data, 80 % sample point (for Nominal and Data)
	----------------------------------------------------------------------------
	signal timing_config 	: bit_time_config_type := 
			(2, 1, 40, 39, 20, 10, 20, 14, 15, 10);

    ----------------------------------------------------------------------------
    -- Bit generator interface
    ----------------------------------------------------------------------------    
    signal bit_gen_run      : boolean := false;
    signal bit_gen_done     : boolean := false;
    signal bit_gen_out      : std_logic := RECESSIVE;

    signal bit_sequence     : bit_seq_type := ((OTHERS => 1), (OTHERS => '0'), 1);

    ----------------------------------------------------------------------------
    -- Config file with generated bit sequences
    ----------------------------------------------------------------------------
    file config_file        : text;
   
    signal error_ctr        : natural := 0;
    signal rand_ctr         : natural range 0 to RAND_POOL_SIZE := 0;
    signal loop_ctr         : natural := 0;
    signal exit_imm         : boolean := false;


    procedure read_dummy_chars(
        variable entry              :  inout     line;
        constant num_chars          :  in        natural
    ) is
        variable char               :            character;
    begin
        for i in 1 to num_chars loop
            read(entry, char);
        end loop;
    end procedure;


    procedure read_bit_sequence(
        file     input              :       text;         
        variable frame              : inout SW_CAN_frame_type;
        signal   bit_sequence       : inout bit_seq_type;
        signal   rand_ctr           : inout integer range 0 to RAND_POOL_SIZE
    ) is
        variable entry              :       line;
        variable iter               :       integer;
        
        -- Strings for file parsing
        variable id_type            :       string(1 to 8);
        variable flag               :       string(1 to 3);

        variable tmp_int            :       integer;
        variable tmp_logic          :       std_logic;

        variable not_end            :       boolean;
        variable rand_val           :       integer;
    begin
        readline(input, entry);

	    -- Read CAN Identifier
		read(entry, flag);
        read_dummy_chars(entry, 1);
        if (flag /= "CAN") then
            report "Invalid input config format" severity error;
        end if;

        -- Frame type
        read(entry, flag);
        read_dummy_chars(entry, 1);
        if (flag = "2.0") then
            frame.frame_format := NORMAL_CAN;
        elsif (flag = "FD ") then
            frame.frame_format := FD_CAN;
        else
            report "Invalid CAN Frame format in Reference test input" severity
                error;
        end if;

        -- Identifier type
        read(entry, id_type);        
        read_dummy_chars(entry, 1);
        if (id_type = "BASE    ") then
            frame.ident_type := BASE;
        elsif (id_type = "EXTENDED") then
            frame.ident_type := EXTENDED;
        else
            report "Invalid CAN Identifier type in Reference test input" 
                severity error;
        end if;

        -- RTR Flag
        read(entry, flag);
        read_dummy_chars(entry, 1);
        if (flag = "RTR") then
            frame.rtr := RTR_FRAME;
        else
            frame.rtr := NO_RTR_FRAME;
        end if;

        -- BRS Flag
        read(entry, flag);
        read_dummy_chars(entry, 1);
        if (flag = "BRS") then
            frame.brs := BR_SHIFT;
        else
            frame.brs := BR_NO_SHIFT;
        end if;

        -- Read "Data length:  X ID:      YZ"
        read_dummy_chars(entry, 13);
        read(entry, frame.data_length);
        read_dummy_chars(entry, 5);
        read(entry, frame.identifier);

        decode_length(frame.data_length, frame.dlc);
        decode_dlc_rx_buff(frame.dlc, frame.rwcnt);

        -- Read "Data:" template
        read_dummy_chars(entry, 6);

        -- Read Data bytes
        for i in 0 to 63 loop
            hread(entry, frame.data(i));
        end loop;

        -- Bit sequence
        iter := 1;
        read_dummy_chars(entry, 14);
        while (true) loop

            -- Read number of consecutive equal values
            read(entry, tmp_int, not_end);
            if (not not_end) then
                exit;
            end if;
            bit_sequence.bit_durations(iter) <= tmp_int;
            read_dummy_chars(entry, 1);

            -- Read Value
            read(entry, tmp_logic);
            bit_sequence.bit_values(iter) <= tmp_logic;
            read_dummy_chars(entry, 1);

            iter := iter + 1;
        end loop;

        bit_sequence.length <= iter - 1;
        wait for 0 ns;

        ------------------------------------------------------------------------
        -- Correct Last and first elements of bit sequence. First and last
        -- parts are recorded before and after trigger and leave the bus
        -- idle for un-necesarilly long!
        -- Correct them to have +- 13 bit times in first sequence and about 7
        -- bit times in the last sequence.
        -- Add some random time (+- 1 bit time) to make sure that HARD 
        -- synchronisation is fully tested!
        -- 13 bit times are chosen so that controller has time to come out of
        -- integrating phase in the first iteration!
        ------------------------------------------------------------------------ 
        rand_int_v(rand_ctr, 200, rand_val);
        bit_sequence.bit_durations(1) <= 200 * 13 + rand_val;

        rand_int_v(rand_ctr, 200, rand_val);
        bit_sequence.bit_durations(bit_sequence.length) <= 200 * 7 + rand_val;        

        wait for 0 ns;

    end procedure;

	procedure restart_mem_bus(
        signal mem_bus : out  Avalon_mem_type
    ) is begin
        mem_bus.scs         <= '0';
        mem_bus.swr         <= '0';
        mem_bus.srd         <= '0';
        mem_bus.address     <= (OTHERS =>'0');
        mem_bus.data_in     <= (OTHERS =>'0');
        mem_bus.clk_sys     <= 'Z';
        mem_bus.data_out    <= (OTHERS =>'Z');
        mem_bus.sbe         <= x"F";
    end procedure;

begin

	errors <= error_ctr;

    ----------------------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------------------
    CAN_inst : CAN_top_level
        generic map(
            use_logger        => true,
            rx_buffer_size    => 64,
            use_sync          => true,
            ID                => 0,
            logger_size       => 16
        )
        port map(
            clk_sys           => clk_sys,
            res_n             => res_n,
            data_in           => data_in,
            data_out          => data_out,
            adress            => adress,
            scs               => scs,
            srd               => srd,
            swr               => swr,
            sbe               => sbe,
            int               => int,
            CAN_tx            => CAN_tx,
            CAN_rx            => CAN_rx,
            time_quanta_clk   => time_quanta_clk,
            timestamp         => timestamp,
            drv_bus_o         => drv_bus,
            stat_bus_o        => stat_bus
        );


    sbe <= mem_bus.sbe;
    scs <= mem_bus.scs;
    srd <= mem_bus.srd;
    swr <= mem_bus.swr;
    adress <= mem_bus.address;
    data_in <= mem_bus.data_in;

    mem_bus.data_out <= data_out;
    mem_bus.clk_sys <= clk_sys;

	-- CAN Bus
	CAN_rx <= bit_gen_out and CAN_tx;

    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clk_gen_proc : clock_gen_proc(period => f100_Mhz, duty => 50, 
                                   epsilon_ppm => 0, out_clk => clk_sys);
    tsgen_proc : timestamp_gen_proc(clk_sys, timestamp);

    ----------------------------------------------------------------------------
    -- Bit generator
    ----------------------------------------------------------------------------
    bit_gen_comp : bit_generator
    generic map (
        prescaler       => 1,
        invert          => true
    )
    port map(
        bit_seq         => bit_sequence,
        clk_sys         => clk_sys,
        run             => bit_gen_run,
        done            => bit_gen_done,
        output          => bit_gen_out
    );


    ----------------------------------------------------------------------------
    -- Main test process
    ----------------------------------------------------------------------------
    test_proc : process
        variable TX_frame         : SW_CAN_frame_type;
        variable RX_frame         : SW_CAN_frame_type;
        variable result           : boolean;
    begin
        log("Restarting Reference test!", info_l, log_level);
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 0, rand_ctr);
		restart_mem_bus(mem_bus);

        log("Restarted Reference test", info_l, log_level);
        print_test_info(iterations, log_level, error_beh, error_tol);

		log("Configuring Bit rate, enabling controller", info_l, log_level);		
		CAN_configure_timing(timing_config, 0, mem_bus);
		CAN_turn_controller(true, 0, mem_bus);
        
        log("Opening test config file", info_l, log_level);
        file_open(config_file, config_path, read_mode);

        while (loop_ctr < iterations or exit_imm)
        loop
            log("Starting loop nr " & integer'image(loop_ctr), info_l,
                log_level);

            -- Read one frame and its bit sequence from input file
            read_bit_sequence(config_file, TX_frame, bit_sequence, rand_ctr);

            -- Start bit generator and wait until sequence is transmitted
            bit_gen_run <= true;
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            wait until bit_gen_done = true;

            -- Read CAN Frame from the device.
            CAN_read_frame(RX_frame, 0, mem_bus);

            -- Compare Transmitted frame from Kvaser and received frame by
            -- CTU CAN FD
            CAN_compare_frames(TX_frame, RX_frame, false, result);
 
            if (not result) then
                log("TX, RX frames mismatch!", error_l, log_level);
                log("TX Frame:", error_l, log_level);
                CAN_print_frame(TX_frame, log_level);
                log("RX Frame:", error_l, log_level);
                CAN_print_frame(RX_frame, log_level);
                process_error(error_ctr, error_beh, exit_imm);
            end if;

            loop_ctr <= loop_ctr + 1;
        end loop;
        file_close(config_file);

        evaluate_test(error_tol, error_ctr, status);
    end process;


end architecture;


