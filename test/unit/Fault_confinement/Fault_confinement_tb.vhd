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
--  Unit test for the Fault confinement circuit.
--------------------------------------------------------------------------------
-- Revision History:
--    8.5.2018   Created file
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

use work.ID_transfer.all;

architecture Fault_Confinement_unit_test of CAN_test is
  
    ----------------------------------------------------------------------------
    -- DUT Interface
    ----------------------------------------------------------------------------

    -- Clock and reset
    signal clk_sys                :    std_logic := '0';
    signal res_n                  :    std_logic := '0';

    --Driving registers interface-
    signal drv_bus                :    std_logic_vector(1023 downto 0) :=
                                            (OTHERS => '0');

    --Stuffing Error from bit destuffing
    signal stuff_Error            :    std_logic := '0';
      
    -- At least one error appeared
    signal error_valid            :    std_logic;

    -- Error passive state changed
    signal error_passive_changed  :    std_logic;

    --Error warning limit was reached
    signal error_warning_limit    :    std_logic; 

    signal OP_State               :    oper_mode_type := transciever;

    -- Recieved data. Valid with the same signal as rec_trig in CAN Core
    signal data_rx                :    std_logic := RECESSIVE;

    -- Transcieved data by CAN Core. Valid with one clk_sys delay from 
    -- tran_trig! The same trigger signal as Bit-Stuffing!
    signal data_tx                :    std_logic := RECESSIVE; 
    signal rec_trig               :    std_logic := '0';

    -- Transcieve data trigger one clk_sys delayed behind the tran_trig
    signal tran_trig_1            :    std_logic := '0';

    signal PC_State               :     protocol_type := interframe;
    signal sp_control             :     std_logic_vector(1 downto 0) :=
                                            NOMINAL_SAMPLE;

    -- Form Error from PC State
    signal form_Error             :     std_logic := '0';

    -- CRC Error from PC State
    signal CRC_Error              :     std_logic := '0';

    -- Acknowledge Error from PC
    signal ack_Error              :     std_logic := '0';

    -- Some of the state machines, or signals 
    -- reached unknown state!! Shouldnt happend!!
    signal unknown_state_Error    :     std_logic := '0';

    -- Error signal for PC control FSM from fault
    -- confinement unit (Bit error or Stuff Error appeared)
    signal bit_Error_valid        :    std_logic;
    signal stuff_Error_valid      :    std_logic;
     
    signal bit_Error_out          :    std_logic;

    -- Interface for increment and decrementing error counters
    signal inc_one                :     std_logic := '0';
    signal inc_eight              :     std_logic := '0';
    signal dec_one                :     std_logic := '0';

    -- Enable for error counting
    signal enable                 :     std_logic := '0';

    -- Bit Error detected with secondary sampling point at busSync.vhd
    signal bit_Error_sec_sam      :     std_logic := '0';
    signal err_capt               :     std_logic_vector(7 downto 0);

    -- Error counters
    signal tx_counter_out         :     std_logic_vector(8 downto 0);
    signal rx_counter_out         :     std_logic_vector(8 downto 0);
    signal err_counter_norm_out   :     std_logic_vector(15 downto 0);
    signal err_counter_fd_out     :     std_logic_vector(15 downto 0);

    -- Fault confinement status
    signal error_state_out        :     error_state_type;

    
    ----------------------------------------------------------------------------
    -- Internal testbench signals
    ----------------------------------------------------------------------------

    -- Random pool pointers
    signal rand_ctr_1             :     natural range 0 to RAND_POOL_SIZE;

    -- Error counters
    signal err_ctr_1              :     natural := 0;
    signal err_ctr_2              :     natural := 0;

    -- Driving bus aliases
    signal drv_ewl                :     std_logic_vector(7 downto 0) :=
                                            x"3F";

    signal drv_erp                :     std_logic_vector(7 downto 0) :=
                                            x"7F";

    signal drv_ctr_val            :     std_logic_vector(8 downto 0) :=
                                            (OTHERS => '0');

    signal drv_ctr_sel            :     std_logic_vector(3 downto 0) :=
                                            (OTHERS => '0');

    -- Models for RX, TX Error counters
    signal rx_err_model           :     integer := 0;
    signal tx_err_model           :     integer := 0;
    signal norm_err_model         :     integer := 0;
    signal fd_err_model           :     integer := 0;

    -- Modeled state
    signal fc_model               :     error_state_type;
    
begin

    ----------------------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------------------
    faultConf_comp : faultConf 
    port map(
        clk_sys                => clk_sys,
        res_n                  => res_n,
        drv_bus                => drv_bus,
        stuff_Error            => stuff_Error,
        error_valid            => error_valid,
        error_passive_changed  => error_passive_changed,
        error_warning_limit    => error_warning_limit,
        OP_State               => OP_State,
        data_rx                => data_rx,
        data_tx                => data_tx,
        rec_trig               => rec_trig,
        tran_trig_1            => tran_trig_1,
        PC_State               => PC_State,
        sp_control             => sp_control,
        form_Error             => form_Error,
        CRC_Error              => CRC_Error,
        ack_Error              => ack_Error,
        unknown_state_Error    => unknown_state_Error,
        bit_Error_valid        => bit_Error_valid,
        stuff_Error_valid      => stuff_Error_valid,
        bit_Error_out          => bit_Error_out,
        inc_one                => inc_one,
        inc_eight              => inc_eight,
        dec_one                => dec_one,
        enable                 => enable,
        bit_Error_sec_sam      => bit_Error_sec_sam,
        err_capt               => err_capt,
        tx_counter_out         => tx_counter_out,
        rx_counter_out         => rx_counter_out,
        err_counter_norm_out   => err_counter_norm_out,
        err_counter_fd_out     => err_counter_fd_out,
        error_state_out        => error_state_out
    );


    ----------------------------------------------------------------------------
    -- Connection of driving bus aliases
    ----------------------------------------------------------------------------
    drv_bus(DRV_EWL_HIGH downto DRV_EWL_LOW)            <= drv_ewl;
    drv_bus(DRV_ERP_HIGH downto DRV_ERP_LOW)            <= drv_erp;
    drv_bus(DRV_CTR_VAL_HIGH downto DRV_CTR_VAL_LOW)    <= drv_ctr_val;
    drv_bus(DRV_CTR_SEL_HIGH downto DRV_CTR_SEL_LOW)    <= drv_ctr_sel;


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen : process
        variable period   :natural := f100_Mhz;
        variable duty     :natural := 50;
        variable epsilon  :natural := 0;
    begin
        generate_clock(period, duty, epsilon, clk_sys);
    end process;  

  
    ----------------------------------------------------------------------------
    -- Generation of Increase, Decrease and Preset signals on error counters.
    -- Checking whether counter was increased, decreased properly!
    ----------------------------------------------------------------------------
    inc_dec_gen_proc : process
        variable rand_nr        : real;
        variable tmp            : std_logic;
        variable exp_increment  : integer := 0;
        variable SW_preset      : boolean;
    begin

        while (res_n = ACT_RESET) loop
            wait until rising_edge(clk_sys);
            apply_rand_seed(seed, 0, rand_ctr);
        end loop;

        wait until rising_edge(clk_sys);        

        -- Generate random command for error counters
        rand_real_v(rand_ctr_1, rand_nr);

        -- Set all signals to 0
        inc_eight   <= '0';
        inc_one     <= '0';
        dec_one     <= '0';
        drv_ctr_sel <= (OTHERS => '0');
        drv_ctr_val <= (OTHERS => '0');
        SW_preset   := false;
     
        -- If counters are bigger than would ever occur in real core, erase
        -- them!
        if (rx_err_model > 264 or tx_err_model > 264) then
            drv_ctr_sel     <= "1111";
            drv_ctr_val     <= (OTHERS => '0');
            SW_preset       := true;

        -- Increase by 8
        elsif (rand_nr < 0.1) then
            inc_eight   <= '1';
            exp_increment := 8;

        -- Increase by 1
        elsif (rand_nr < 0.5) then
            inc_one     <= '1';
            exp_increment := 1;

        -- Decrease by 1
        elsif (rand_nr < 0.9) then
            dec_one     <= '1';
            exp_increment := -1;

        -- Preset from SW! Highest bit has very little probability.
        -- This represents the fact that SW would rarely set the error counter
        -- to values more than 255!
        else
            rand_logic_vect_s(rand_ctr_1, drv_ctr_sel, 0.3);
            rand_logic_vect_s(rand_ctr_1, drv_ctr_val, 0.5);
            rand_logic_v(rand_ctr_1, tmp, 0.2);
            drv_ctr_val(drv_ctr_val'length - 1) <= tmp;
            SW_preset := true;
            exp_increment := 0;
        end if;

        wait until rising_edge(clk_sys);

        -- Preset from SW is mutually exclusive with HW set in the test. In real
        -- DUT SW preset wins!
        if (SW_preset) then
            if (drv_ctr_sel(0) = '1') then
                tx_err_model    <= to_integer(unsigned(drv_ctr_val));
            end if;
            if (drv_ctr_sel(1) = '1') then
                rx_err_model    <= to_integer(unsigned(drv_ctr_val));
            end if;
            if (drv_ctr_sel(2) = '1') then
                norm_err_model  <= to_integer(unsigned(drv_ctr_val));
            end if;
            if (drv_ctr_sel(3) = '1') then
                fd_err_model    <= to_integer(unsigned(drv_ctr_val));
            end if;
        
        -- Calculate expected value for error counters
        else
            if (OP_State = transciever) then
                tx_err_model <= tx_err_model + exp_increment;
                wait for 0 ns;
                if (tx_err_model < 0) then
                    tx_err_model <= 0;
                end if;
            elsif (OP_State = reciever) then
                rx_err_model <= rx_err_model + exp_increment;
                wait for 0 ns;
                if (rx_err_model < 0) then
                    rx_err_model <= 0;
                end if;
            end if;

            -- FD and NORMAL Error counters increment by one only !!
            if (exp_increment > 0) then
                if (sp_control = NOMINAL_SAMPLE) then
                    norm_err_model  <= norm_err_model + 1;
                elsif (sp_control = DATA_SAMPLE or 
                       sp_control = SECONDARY_SAMPLE)
                then
                    fd_err_model    <= fd_err_model + 1;
                end if;
            end if;
        end if;

        inc_eight   <= '0';
        inc_one     <= '0';
        dec_one     <= '0';
        drv_ctr_sel <= (OTHERS => '0');
        drv_ctr_val <= (OTHERS => '0');

        wait until falling_edge(clk_sys);

        if ( (tx_err_model /= to_integer(unsigned(tx_counter_out))) or
             (rx_err_model /= to_integer(unsigned(rx_counter_out))) or
             (norm_err_model /= to_integer(unsigned(err_counter_norm_out))) or
             (fd_err_model /= to_integer(unsigned(err_counter_fd_out))))
        then
            log("Error counters not as expected", error_l, log_level);
            err_ctr_1 <= err_ctr_1 + 1;
        end if;

    end process;


    ----------------------------------------------------------------------------
    -- Checking the Fault conf. state
    ----------------------------------------------------------------------------
    fc_check_state_proc : process
    begin

        wait until rising_edge(clk_sys);

        if (tx_err_model > 255 or
            rx_err_model > 255)
        then
            fc_model    <= bus_off;
        elsif (tx_err_model > to_integer(unsigned(drv_erp)) or
               rx_err_model > to_integer(unsigned(drv_erp)))
        then
            fc_model    <= error_passive;
        else
            fc_model    <= error_active;
        end if;

        wait until rising_edge(clk_sys);

        if (fc_model /= error_state_out) then
            log("Fault confinement state differs!", error_l, log_level);
            err_ctr_2 <= err_ctr_2 + 1;
        end if;

    end process;

    errors <= error_ctr;    


    ----------------------------------------------------------------------------
    -- Main test process 
    ----------------------------------------------------------------------------
    test_proc : process
    begin
        log("Restarting Fault confinement test!", info_l, log_level);
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        log("Restarted Fault confinement test", info_l, log_level);
        print_test_info(iterations, log_level, error_beh, error_tol);

        while (loop_ctr < iterations or exit_imm)
        loop
            log("Starting loop nr " & integer'image(loop_ctr), info_l,
                log_level);
            
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);

            error_ctr <= err_ctr_1 + err_ctr_2;
            
            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;
  
  
end architecture;



--------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
--------------------------------------------------------------------------------

architecture Fault_confinement_unit_test_wrapper of CAN_test_wrapper is
  
    --Select architecture of the test
    for test_comp : CAN_test use entity work.CAN_test(Fault_confinement_unit_test);

    -- Input trigger, test starts running when true 
    signal run              :   boolean;

    -- Status of the test       
    signal status_int       :   test_status_type;

    -- Amount of errors which appeared in the test
    signal errors           :   natural;   

begin
  
    -- In this test wrapper generics are directly connected to the signals
    -- of test entity
    test_comp : CAN_test
    port map(
        run              =>  run,
        iterations       =>  iterations , 
        log_level        =>  log_level,
        error_beh        =>  error_beh,
        error_tol        =>  error_tol,                                                     
        status           =>  status_int,
        errors           =>  errors
    );

    status              <= status_int;

    ------------------------------------
    -- Starts the test and lets it run
    ------------------------------------
    test : process
    begin
        run               <= true;
        wait for 1 ns;

        --Wait until the only test finishes and then propagate the results
        wait until (status_int = passed or status_int = failed);  

        wait for 100 ns;
        run               <= false;
    
    end process;
  
  
end;
