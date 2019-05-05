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
--  Unit test for the Interrupt manager.
--  Random interrupt source signals are generated in the testbench. Periodically
--  random setting of interrupt generator is used. Then test waits and evaluates
--  whether interrupt prediction (int_test_ctr) matches the actual number of
--  interrupts measured on the int_out rising and falling edges. Also interrupt
--  vector is read and compared with modeled interrupt vector.
--------------------------------------------------------------------------------
-- Revision History:
--    6.6.2016   Created file
--   19.4.2018   Modified testbench to be compliant with separation of interrupt
--               set, Interrupt clear and interrupt mask separation.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

architecture int_man_unit_test of CAN_test is

    -- System Clock and reset
    signal clk_sys                :   std_logic := '0';
    signal res_n                  :   std_logic := '0';

    ----------------------------------------------
    -- Interrupt inputs
    ----------------------------------------------

    -- Valid Error appeared for interrupt
    signal err_detected            :   std_logic := '0';

    -- Error pasive /Error acitve functionality changed
    signal error_passive_changed  :   std_logic := '0';

    -- Error warning limit reached
    signal error_warning_limit    :   std_logic := '0';

    -- Arbitration was lost input
    signal arbitration_lost       :   std_logic := '0';

    -- Message stored in CAN Core was sucessfully transmitted
    signal tran_valid            :   std_logic := '0';

    -- Bit Rate Was Shifted
    signal br_shifted             :   std_logic := '0';

    -- Income message was discarded
    signal rx_data_overrun        :   std_logic := '0';

    -- Message recieved!
    signal rec_valid      :   std_logic := '0';

    -- RX Buffer full
    signal rx_full                :   std_logic := '0';

    -- Event logging finsihed
    signal loger_finished         :   std_logic := '0';

    -- RX Buffer not empty
    signal rx_empty               :   std_logic := '1';

    -- HW command on TX Buffer
    signal txtb_hw_cmd_int   :   std_logic_vector(C_TXT_BUFFER_COUNT - 1 downto 0);

    ----------------------------------------------
    -- Status signals
    ----------------------------------------------
    signal int_ena                :   std_logic_vector(C_INT_COUNT - 1 downto 0);
    signal int_vector             :   std_logic_vector(C_INT_COUNT - 1 downto 0);
    signal int_mask               :   std_logic_vector(C_INT_COUNT - 1 downto 0);

    signal int                :   std_logic;

    ----------------------------------------------
    -- Internal testbench signals
    ----------------------------------------------
    signal drv_bus                :   std_logic_vector(1023 downto 0) :=
                                            (OTHERS => '0');

    signal rand_ctr_1             :   natural range 0 to RAND_POOL_SIZE;

    signal error_ctr_2            :   natural;

    -- Driving signals from memory registers!
    signal drv_int_clear          :   std_logic_vector(C_INT_COUNT - 1 downto 0)
                                          := (OTHERS =>'0');
    signal drv_int_ena_set        :   std_logic_vector(C_INT_COUNT - 1 downto 0)
                                          := (OTHERS =>'0');
    signal drv_int_ena_clear      :   std_logic_vector(C_INT_COUNT - 1 downto 0)
                                          := (OTHERS =>'0');
    signal drv_int_mask_set       :   std_logic_vector(C_INT_COUNT - 1 downto 0)
                                          := (OTHERS =>'0');
    signal drv_int_mask_clear     :   std_logic_vector(C_INT_COUNT - 1 downto 0)
                                          := (OTHERS =>'0');

    -- Expected status, mask and enable (similar as in DUT)
    signal int_ena_exp            :   std_logic_vector(C_INT_COUNT - 1 downto 0)
                                          := (OTHERS =>'0');
    signal int_status_exp         :   std_logic_vector(C_INT_COUNT - 1 downto 0)
                                          := (OTHERS =>'0');
    signal int_mask_exp           :   std_logic_vector(C_INT_COUNT - 1 downto 0)
                                          := (OTHERS =>'0');

    -- Joined input vector
    signal int_input              :   std_logic_vector(C_INT_COUNT - 1 downto 0)
                                          := (OTHERS =>'0');


    ----------------------------------------------------------------------------
    -- Generates random interrupt sources
    ----------------------------------------------------------------------------
    procedure generate_sources(
        signal rand_ctr               :inout   natural range 0 to RAND_POOL_SIZE;

        -- Valid Error appeared for interrupt
        signal err_detected            :inout   std_logic;

        -- Error pasive /Error acitve functionality changed
        signal error_passive_changed  :inout   std_logic;

        -- Error warning limit reached
        signal error_warning_limit    :inout   std_logic;

        -- Arbitration was lost input
        signal arbitration_lost       :inout   std_logic;

        -- Message stored in CAN Core was sucessfully transmitted
        signal tran_valid            :inout   std_logic;

        -- Bit Rate Was Shifted
        signal br_shifted             :inout   std_logic;

        -- Income message was discarded
        signal rx_data_overrun        :inout   std_logic;

        -- Message recieved!
        signal rec_valid      :inout   std_logic;

        -- RX Buffer full
        signal rx_full                :inout   std_logic;

        -- Event loggging finished
        signal loger_finished         :inout   std_logic;

        -- RX Buffer empty
        signal rx_empty               :inout   std_logic;

        -- TXT HW Command
        signal txtb_hw_cmd_int         :inout   std_logic_vector(C_TXT_BUFFER_COUNT - 1
                                                                downto 0)
    )is
        variable tmp                  :        std_logic;
    begin
        if (err_detected = '1') then
            rand_logic_s(rand_ctr, err_detected, 0.85);
        else
            rand_logic_s(rand_ctr, err_detected, 0.1);
        end if;

        if (error_passive_changed = '1') then
            rand_logic_s(rand_ctr, error_passive_changed, 0.85);
        else
            rand_logic_s(rand_ctr, error_passive_changed, 0.05);
        end if;

        if (error_warning_limit = '1') then
            rand_logic_s(rand_ctr, error_warning_limit, 0.85);
        else
            rand_logic_s(rand_ctr, error_warning_limit, 0.05);
        end if;

        if (arbitration_lost = '1') then
            rand_logic_s(rand_ctr, arbitration_lost, 0.95);
        else
            rand_logic_s(rand_ctr, arbitration_lost, 0.05);
        end if;

        if (tran_valid = '1') then
            rand_logic_s(rand_ctr, tran_valid, 0.95);
        else
            rand_logic_s(rand_ctr, tran_valid, 0.05);
        end if;

        if (br_shifted = '1') then
            rand_logic_s(rand_ctr, br_shifted, 0.95);
        else
            rand_logic_s(rand_ctr, br_shifted, 0.05);
        end if;

        if (rx_data_overrun = '1') then
            rand_logic_s(rand_ctr, rx_data_overrun, 0.95);
        else
            rand_logic_s(rand_ctr, rx_data_overrun, 0.05);
        end if;

        if (rec_valid = '1') then
            rand_logic_s(rand_ctr, rec_valid, 0.95);
        else
            rand_logic_s(rand_ctr, rec_valid, 0.05);
        end if;

        if (rx_full = '1') then
            rand_logic_s(rand_ctr, rx_full, 0.95);
        else
            rand_logic_s(rand_ctr, rx_full, 0.05);
        end if;

        if (loger_finished = '1') then
            rand_logic_s(rand_ctr, loger_finished, 0.95);
        else
            rand_logic_s(rand_ctr, loger_finished, 0.05);
        end if;

        if (rx_empty = '0') then
            rand_logic_s(rand_ctr, rx_empty, 0.95);
        else
            rand_logic_s(rand_ctr, rx_empty, 0.05);
        end if;

        for i in 0 to C_TXT_BUFFER_COUNT - 1 loop
            if (txtb_hw_cmd_int(i) = '1') then
                rand_logic_v(rand_ctr, tmp, 0.95);
            else
                rand_logic_v(rand_ctr, tmp, 0.05);
            end if;
            txtb_hw_cmd_int(i) <= tmp;
        end loop;

    end procedure;


    ----------------------------------------------------------------------------
    -- Generates interrupt commands as if comming on driving bus from memory
    -- registers!
    ----------------------------------------------------------------------------
    procedure generate_commands(
        signal drv_int_clear       :out std_logic_vector(C_INT_COUNT - 1 downto 0);
        signal drv_int_ena_set     :out std_logic_vector(C_INT_COUNT - 1 downto 0);
        signal drv_int_ena_clear   :out std_logic_vector(C_INT_COUNT - 1 downto 0);
        signal drv_int_mask_set    :out std_logic_vector(C_INT_COUNT - 1 downto 0);
        signal drv_int_mask_clear  :out std_logic_vector(C_INT_COUNT - 1 downto 0);
        signal rand_ctr            :inout natural range 0 to RAND_POOL_SIZE
    ) is
        variable tmp             : real;
    begin
        rand_real_v(rand_ctr, tmp);

        -- Erase the commands by default!
        drv_int_clear            <= (OTHERS => '0');
        drv_int_ena_set          <= (OTHERS => '0');
        drv_int_ena_clear        <= (OTHERS => '0');
        drv_int_mask_set         <= (OTHERS => '0');
        drv_int_mask_clear       <= (OTHERS => '0');

        -- Only one command is generated at any time, since commands are
        -- coming from different registers!
        if (tmp < 0.2) then
            rand_logic_vect_s(rand_ctr, drv_int_clear, 0.4);

        elsif (tmp < 0.4) then
            rand_logic_vect_s(rand_ctr, drv_int_ena_set, 0.2);

        elsif (tmp < 0.6) then
            rand_logic_vect_s(rand_ctr, drv_int_ena_clear, 0.4);

        elsif (tmp < 0.8) then
            rand_logic_vect_s(rand_ctr, drv_int_mask_set, 0.2);

        else
            rand_logic_vect_s(rand_ctr, drv_int_mask_clear, 0.4);
        end if;

        wait for 0 ns;

    end procedure;

begin

    ----------------------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------------------
    int_manager_comp : int_manager
    GENERIC map(
        G_INT_COUNT           => C_INT_COUNT
    )
    PORT map(
        clk_sys               =>   clk_sys,
        res_n                 =>   res_n,
        err_detected           =>   err_detected,
        error_passive_changed =>   error_passive_changed,
        error_warning_limit   =>   error_warning_limit,
        arbitration_lost      =>   arbitration_lost,
        tran_valid           =>   tran_valid ,
        br_shifted            =>   br_shifted,
        rx_empty              =>   rx_empty,
        txtb_hw_cmd_int        =>   txtb_hw_cmd_int,
        rx_data_overrun       =>   rx_data_overrun ,
        rec_valid     =>   rec_valid ,
        rx_full               =>   rx_full,
        loger_finished        =>   loger_finished,
        drv_bus               =>   drv_bus ,
        int               =>   int,
        int_vector            =>   int_vector,
        int_mask              =>   int_mask,
        int_ena               =>   int_ena
    );

    -- Joining interrupt inputs to interrupt status
    int_input(BEI_IND)            <=  err_detected;
    int_input(ALI_IND)            <=  arbitration_lost;
    int_input(EPI_IND)            <=  error_passive_changed;
    int_input(DOI_IND)            <=  rx_data_overrun;
    int_input(EWLI_IND)           <=  error_warning_limit;
    int_input(TXI_IND)            <=  tran_valid;
    int_input(RXI_IND)            <=  rec_valid;
    int_input(LFI_IND)            <=  loger_finished;
    int_input(RXFI_IND)           <=  rx_full;
    int_input(BSI_IND)            <=  br_shifted;
    int_input(RBNEI_IND)          <=  not rx_empty;
    int_input(TXBHCI_IND)         <=  or_reduce(txtb_hw_cmd_int);


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);


    ----------------------------------------------------------------------------
    -- Generating random sources
    ----------------------------------------------------------------------------
    src_gen : process
    begin
        wait for 195 ns;
        if (res_n = C_RESET_POLARITY) then
            apply_rand_seed(seed, 1, rand_ctr_1);
        end if;

        while true loop
            wait until falling_edge(clk_sys);
            generate_sources(rand_ctr_1, err_detected, error_passive_changed ,
                           error_warning_limit , arbitration_lost, tran_valid,
                           br_shifted, rx_data_overrun , rec_valid ,
                           rx_full , loger_finished, rx_empty, txtb_hw_cmd_int);
        end loop;
    end process;


    ----------------------------------------------------------------------------
    -- Connection to Driving bus
    ----------------------------------------------------------------------------
    drv_bus(DRV_INT_CLR_HIGH downto DRV_INT_CLR_LOW)
            <= drv_int_clear;

    drv_bus(DRV_INT_ENA_SET_HIGH downto DRV_INT_ENA_SET_LOW)
            <= drv_int_ena_set;

    drv_bus(DRV_INT_ENA_CLR_HIGH downto DRV_INT_ENA_CLR_LOW)
            <= drv_int_ena_clear;

    drv_bus(DRV_INT_MASK_SET_HIGH downto DRV_INT_MASK_SET_LOW)
            <= drv_int_mask_set;

    drv_bus(DRV_INT_MASK_CLR_HIGH downto DRV_INT_MASK_CLR_LOW)
            <= drv_int_mask_clear;


    ----------------------------------------------------------------------------
    -- Calculate expected outputs
    ----------------------------------------------------------------------------
    int_emu_proc : process
        variable int_output     : boolean;
        variable outcome        : boolean;
        variable exp_output     : boolean;
        constant zeroes         : std_logic_vector(C_INT_COUNT - 1 downto 0) :=
                                      (OTHERS => '0');
    begin

        while (run = false) loop
            wait until rising_edge(clk_sys);
        end loop;

        wait until rising_edge(clk_sys);

        outcome := true;

        for i in 0 to C_INT_COUNT - 1 loop

            -- Interrupt enable
            if (drv_int_ena_set(i) = '1') then
                int_ena_exp(i) <= '1';
            elsif (drv_int_ena_clear(i) = '1') then
                int_ena_exp(i) <= '0';
            end if;

            -- Interrupt mask
            if (drv_int_mask_set(i) = '1') then
                int_mask_exp(i) <= '1';
            elsif (drv_int_mask_clear(i) = '1') then
                int_mask_exp(i) <= '0';
            end if;

            -------------------------------------------------------------------
            -- Interrupt clear and capturing!
            -- Clear priority only for DOI, for others set priority.
            -------------------------------------------------------------------
            if (i = DOI_IND) then
                if (drv_int_clear(i) = '1') then
                    int_status_exp(i) <= '0';
                elsif (int_input(i) = '1' and int_mask_exp(i) = '0') then
                    int_status_exp(i) <= '1';
                end if;
            else
                if (int_input(i) = '1' and int_mask_exp(i) = '0') then
                    int_status_exp(i) <= '1';
                elsif (drv_int_clear(i) = '1') then
                    int_status_exp(i) <= '0';
                end if;
            end if;
            
        end loop;

        -- Calculating expected interrupt output
        if ((int_vector AND int_ena_exp) = zeroes) then
            exp_output      := false;
        else
            exp_output      := true;
        end if;

        -- Checking the expected and real outputs
        check(int_ena = int_ena_exp, "Interrupt enable mismatch");
        check(int_mask = int_mask_exp, "Interrupt mask mismatch");
        check(int_vector = int_status_exp, "Interrupt vector mismatch");
        
        if (int = '0') then
            int_output := false;
        elsif (int = '1') then
            int_output := true;
        else
            error("Interrupt value undefined!");
        end if;
        check(exp_output = int_output, "Interrupt output mismatch");

    end process;

    -- Error propagation to the output
    errors <= error_ctr;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
        variable outcome : boolean := false;
    begin
        info("Restarting Interrupt test!");
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 1, rand_ctr);
        info("Restarted Interrupt test");
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        info("Starting Interrupt main loop");

        while (loop_ctr < iterations  or  exit_imm)
        loop
              info("Starting loop nr " & integer'image(loop_ctr));

              wait until falling_edge(clk_sys);

              -- Generate commands as coming from user registers
              generate_commands(drv_int_clear, drv_int_ena_set,
                                drv_int_ena_clear, drv_int_mask_set,
                                drv_int_mask_clear, rand_ctr);
              wait for 50 ns;

              -- Errors are evaluated in separate process.
              error_ctr <= error_ctr_2;

              loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

end architecture;
