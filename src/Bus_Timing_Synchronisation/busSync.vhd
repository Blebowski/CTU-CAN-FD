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
--  1. Implements Generic Synchronisation chain for incoming data
--  2. Detects appropriate edge for synchronisation!!
--  3. Measure Transciever delay compensation on command
--  4. Sample bus values in sample point of Bit time:
--    a) By normal sampling, at position of sample point (Transciever,reciever)
--    b) In secondary sample point for Transciever of CAN FD Data Phase
--  5. Detect bit Error by comparing transmitted values and Sampled values!
--
--Note: this bit error detection used in the end only for data phase transciever 
--      of CAN FD Phase! In other cases bit error is detected inside CAN-Core by
--      comparing transmitted and recieved bit.
--------------------------------------------------------------------------------
--    July 2015   Created file
--    19.12.2015  Added tripple sampling mode. Furthermore sampling is disabled
--                when whole controller is disabled
--    15.6.2016   1.edge_tx_valid signal now provides edge detection only from 
--                  RECESSIVE to DOMINANT values! In CAN FD standard EDL and r0
--                  bits are used for TRD measurment! When busSync is configured
--                  to start on TX edge and finish on RX edge.
--                2.Changed trv_delay size to cover mostly 127 clock cycles!
--                  Changed shift register sizes to 130 to avoid missing secon-
--                  dary sample signals!!
--                3.Fixed tripple sampling mode selection from three. Added 
--                  missing signals
--                4. trv_to_restart signal added to make it impossible to res-
--                   tart trv delay measurment without rising edge on trv_delay
--                   calib signal. This removes a bug when trv_delay_calib
--                   is forgottern active and circuit keeps measuring on every 
--                   edge...
--    27.6.2016   Bug fix. Transciever delay measurment conditions switched. Now
--                starting edge is the most prioritized logic. Thus transciever
--                delay counter is always erased with TX edge!
--                'elsif(edge_tx_valid='1')' swapped with 'if(trv_running='1')'.
--                Thisway if more TX edge would come before first RX edge is
--                sampled, shorter time would be measured! Note that this is OK
--                since in CAN spec. EDL bit is in nominal bit time and no other
--                edge can be transmitted before it is recieved (condition of 
--                original CAN)
--    20.4.2018   Added register to "trv_delay_out". Transceiver delay is
--                updated on the output only once the measurement has finished.
--                Thus reading TRV_DELAY register would always result in tran-
--                sceiver delay of last CAN FD Frame. This is done to avoid
--                reading wrong value from TRV_DELAY register during measurment.
--   10.12.2018   Re-factored, added generic Shift register instances. Added
--                generic synchronisation chain module.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE WORK.CANconstants.ALL;
use work.CAN_FD_register_map.all;
use work.cmn_lib.all;

entity busSync is 
    GENERIC (

        ------------------------------------------------------------------------
        -- Synchronisation chain should be used for sampled data from the bus.
        -- Turn off only when Synthetizer puts synchronisation chain automati-
        -- cally on the output pins! Otherwise metastability issues will occur!
        ------------------------------------------------------------------------
        constant use_Sync               :     boolean := false 
    );  
    PORT(
        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        signal clk_sys                  :in   std_logic;
        signal res_n                    :in   std_logic;

        ------------------------------------------------------------------------
        --  Physical layer interface
        ------------------------------------------------------------------------
        signal CAN_rx                   :in   std_logic;
        signal CAN_tx                   :out  std_logic;

        ------------------------------------------------------------------------
        -- Driving registers interface
        ------------------------------------------------------------------------
        signal drv_bus                  :in   std_logic_vector(1023 downto 0);
          
        ------------------------------------------------------------------------
        -- Prescaler interface
        ------------------------------------------------------------------------

        -- Sample command for nominal bit time
        signal sample_nbt               :in   std_logic;

        -- Sample command for data bit tim
        signal sample_dbt               :in   std_logic;

        -- Synchronisation edge appeared
        signal sync_edge                :out  std_logic;


        ------------------------------------------------------------------------
        -- CAN Core Interface
        ------------------------------------------------------------------------

        -- Transcieve data value
        signal data_tx                  :in   std_logic;

        -- Recieved data value
        signal data_rx                  :out  std_logic;

        -- Sample point control
        --  00 : sample_nbt used for sampling (Nominal bit time sampling,
        --        Transciever and Reciever)
        --  01 : sample_dbt used for sampling (Data bit time sampling, only
        --        Reciever)
        --  10 : Sampling with transciever delay compensation (Data bit time,
        --        transciever)
        signal sp_control               :in   std_logic_vector(1 downto 0);
            
        -- Clear the Shift register at the  beginning of Data Phase!!!
        signal ssp_reset                :in   std_logic;

        -- Calibration command for transciever delay compenstation (counter)
        signal trv_delay_calib          :in   std_logic; 

        -- Bit Error detection enable (Ex. disabled when recieving data)
        signal bit_err_enable           :in   std_logic; 

        -- Secondary sample signal and delayed signals by 1 and 2 clock cycles
        signal sample_sec_out           :out  std_logic;
        signal sample_sec_del_1_out     :out  std_logic;
        signal sample_sec_del_2_out     :out  std_logic;

        signal trv_delay_out            :out  std_logic_vector(15 downto 0);

        ------------------------------------------------------------------------
        -- Error Handler Interface
        ------------------------------------------------------------------------
        signal bit_Error                :out  std_logic 
        -- Bit Error appeared (monitored value different than transcieved value)
        -- Used only for bit error with secondary sample point
        
    );
end entity;


architecture rtl of busSync is


    -----------------------------------------------------------------------------
    -- Internal constants declaration.
    -----------------------------------------------------------------------------

    -- Shift registers length
    constant SSP_SHIFT_LENGTH        :     natural := 130;
    constant TX_DATA_SHIFT_LENGTH    :     natural := 130;

    -- Reset value for secondar sampling point shift registers
    constant SSP_SHIFT_RST_VAL       :     std_logic_vector(SSP_SHIFT_LENGTH - 1
                                                            downto 0) :=
                                                (OTHERS => '0');

    constant TX_DATA_SHIFT_RST_VAL   :     std_logic_vector(TX_DATA_SHIFT_LENGTH - 1
                                                            downto 0) :=
                                                (OTHERS => RECESSIVE);


    -----------------------------------------------------------------------------
    -- Driving bus aliases
    -----------------------------------------------------------------------------

    -- Tripple sampling (as SJA1000)
    signal drv_sam                   :     std_logic;

    -- Enable of the whole driver
    signal drv_ena                   :     std_logic;


    -----------------------------------------------------------------------------
    -- Internal registers and signals
    -----------------------------------------------------------------------------

    -- CAN RX synchronisation chain output
    signal CAN_rx_synced             :     std_logic;

    -- Internal received CAN Data. Selected between Raw input value and
    -- synchronised value by signal synchroniser.
    signal CAN_rx_i                  :     std_logic;

    -- Majority value from all three sampled values in tripple sampling shift 
    -- register
    signal CAN_rx_trs_majority       :     std_logic; 

    -- CAN RX Data selected between normally sampled data (CAN_rx_i) and
    -- Tripple sampled data!
    signal data_rx_nbt               :     std_logic;

    -- Bus sampling and edge detection, Previously sampled value on CAN bus
    signal prev_Sample               :     std_logic;

    -- Secondary sampling signal (sampling with transciever delay compensation)
    signal sample_sec                :     std_logic;
    signal sample_sec_comb           :     std_logic;

    -- Secondary sample signal 1 and 2 clk_sys cycles delayed
    signal sample_sec_del_1          :     std_logic;
    signal sample_sec_del_2          :     std_logic;

    -- Shift Register for storing the TX data for secondary sample point
    signal tx_data_shift             :     std_logic_vector
                                            (TX_DATA_SHIFT_LENGTH - 1 downto 0);

    -- Delayed TX Data from TX Data shift register at position of secondary
    -- sampling point.
    signal tx_data_delayed           :      std_logic;

    -- Shift Register for generating secondary sampling signal
    signal sample_sec_shift          :     std_logic_vector
                                            (SSP_SHIFT_LENGTH - 1 downto 0); 

    -- Register for edge detection
    signal edge_rx_det               :     std_logic;

    -- Appropriate edge appeared at recieved data
    signal edge_rx_valid             :     std_logic;

    -- Edge Appeared at transcieved data
    signal edge_tx_det               :     std_logic;

    -- Edge appeared at transcieved data
    signal edge_tx_valid             :     std_logic;

    -- Tripple sampling shift register
    signal trs_reg                   :     std_logic_vector(2 downto 0);

    --Bit Error register
    signal bit_Error_reg             :     std_logic;

    --Note: Bit Error is set up at sample point for whole bit 
    -- time until next sample point!!!!!

    --Transciever delay value (in clk_sys clock periods)
    --Length of this vector corresponds to maximal measurable length of TRD!!!
    --It is 256 clock cycles - assuming 100 Mhz clk sys, more than 2,5 us...
    signal trv_delay                 :     std_logic_vector(6 downto 0); 

    --Delay compensation measuring is running (between tx edge and rx edge)
    signal trv_meas_running          :     std_logic;    
    signal trv_meas_to_restart       :     std_logic;

begin
    
    ---------------------------------------------------------------------------
    -- Driving bus alias
    ---------------------------------------------------------------------------
    drv_sam               <= drv_bus(DRV_SAM_INDEX);
    drv_ena               <= drv_bus(DRV_ENA_INDEX);


    ----------------------------------------------------------------------------
    -- Synchronisation chain for input signal
    ----------------------------------------------------------------------------
    can_rx_sig_sync_comp : sig_sync
    port map(
        clk     => clk_sys,
        async   => CAN_rx,
        sync    => CAN_rx_synced
    );


    ---------------------------------------------------------------------------
    -- Synchronisation-chain selection
    ---------------------------------------------------------------------------
    sync_chain_true : if (use_Sync = true) generate
        CAN_rx_i <= CAN_rx_synced;
    end generate;

    sync_chain_false : if (use_Sync = false) generate
        CAN_rx_i <= CAN_rx;
    end generate;


    ----------------------------------------------------------------------------
    -- Transceiver delay propagation to output.
    -- Transceiver delay is propagated only once the measurement is finished!
    -- If not, reading TRV_DELAY register during the measurement would return
    -- immediate value in the counter which would not correspond to transceiver
    -- delay measured during last FD frame!! 
    ----------------------------------------------------------------------------
    trv_delay_out_proc : process (clk_sys, res_n)
    begin
        if (res_n = ACT_RESET) then
            trv_delay_out       <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then

            -- Do not propagate during the measurement!
            if (trv_meas_running = '0') then
                trv_delay_out   <= "000000000" & trv_delay;
            end if;

        end if;
    end process;


    ---------------------------------------------------------------------------
    -- Tripple sampling majority selection
    ---------------------------------------------------------------------------
    trs_maj_dec_comp : majority_decoder_3
    port map(
        input   => trs_reg,
        output  => CAN_rx_trs_majority
    );


    ---------------------------------------------------------------------------
    -- Selection of RX Data sampled in Nominal Bit-rate between Normally
    -- sampled and tripple sampled data.
    ---------------------------------------------------------------------------
    data_rx_nbt <= CAN_rx_trs_majority when (drv_sam = TSM_ENABLE) else
                   CAN_rx_i;


    ----------------------------------------------------------------------------
    -- RX Data Edge detection (for one clk_sys period)
    -- Note: Sucessful samplng of previous bit is necessary 
    --      (prev_Sample register)
    ----------------------------------------------------------------------------
    rx_edge_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            edge_rx_det         <= RECESSIVE;
            edge_rx_valid       <= '0';
            
        elsif rising_edge(clk_sys) then

            -- Rx data
            edge_rx_det           <= CAN_rx_i;
            
            --Rx data edge appeared from recessive to dominant
            if ((not edge_rx_det = CAN_rx_i) and 
                (not prev_Sample = CAN_rx_i) and (prev_Sample = RECESSIVE))
            then
                edge_rx_valid     <= '1';
            else
                edge_rx_valid     <= '0';
            end if;

        end if;
    end process rx_edge_proc;


    ----------------------------------------------------------------------------
    -- TX Data Edge detection (for one clk_sys period)
    ----------------------------------------------------------------------------
    tx_edge_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            edge_tx_det         <= RECESSIVE;
            edge_tx_valid       <= '0';

        elsif rising_edge(clk_sys) then

            -- Tx data
            edge_tx_det           <= data_tx;

            --Tx data edge appeared from RECESSIVE to dominant
            if ((not (edge_tx_det = data_tx)) and (edge_tx_det = RECESSIVE)) then 
                edge_tx_valid     <= '1';
            else
                edge_tx_valid     <= '0';
            end if;

        end if;
    end process tx_edge_proc;

   
    ----------------------------------------------------------------------------
    -- Control for transceiver delay measurement.
    ----------------------------------------------------------------------------
    trv_delay_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then    
            trv_meas_running       <= '0';
            trv_meas_to_restart    <= '0';

        elsif (rising_edge(clk_sys)) then
           
            -- Delay measurement enabled
            if (trv_delay_calib = '1') then

                -- Stop delay measurement upon receiving the edge on RX Data.
                if (edge_rx_valid = '1') then
                    trv_meas_running     <= '0';

                -- Start measurement on edge on TX Data.
                elsif (edge_tx_valid = '1' and trv_meas_to_restart = '0') then
                    trv_meas_running     <= '1';
                    trv_meas_to_restart  <= '1';
                end if;

            else
                trv_meas_to_restart      <= '0';
            end if;

        end if;
    end process trv_delay_proc;


    ----------------------------------------------------------------------------
    -- Transceiver delay measurement
    ----------------------------------------------------------------------------
    trv_delay_meas_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            trv_delay         <= (OTHERS => '0');
            
        elsif (rising_edge(clk_sys)) then

            -- Delay measurement enabled
            if (trv_delay_calib = '1') then
                
                -- Restart upon edge on TX Data
                if (edge_tx_valid = '1') then
                    trv_delay       <= (OTHERS => '0');

                -- Increment each clock cycle
                elsif (trv_meas_running = '1') then
                    trv_delay       <= std_logic_vector(unsigned(trv_delay) + 1);
                end if;

            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Shift register for secondary sampling point. Normal sample point trigger
    -- is shifted into shift register to generate delayed sampling point.
    ----------------------------------------------------------------------------
    ssp_shift_reg_comp : shift_reg
    generic map(
        reset_polarity     => ACT_RESET,
        reset_value        => SSP_SHIFT_RST_VAL,
        width              => SSP_SHIFT_LENGTH,
        shift_down         => false
    )
    port map(
        clk                => clk_sys,
        res_n              => res_n,

        input              => sample_dbt,
        preload            => ssp_reset,
        preload_val        => SSP_SHIFT_RST_VAL,
        enable             => '1',

        reg_stat           => sample_sec_shift,
        output             => open
    );

    ----------------------------------------------------------------------------
    -- Shift register for TX data. Stored by shift register to be compared
    -- with sampled RX Data in Secondary sampling point to detect bit error.
    ----------------------------------------------------------------------------
    tx_data_shift_reg_comp : shift_reg
    generic map(
        reset_polarity     => ACT_RESET,
        reset_value        => TX_DATA_SHIFT_RST_VAL,
        width              => TX_DATA_SHIFT_LENGTH,
        shift_down         => false
    )
    port map(
        clk                => clk_sys,
        res_n              => res_n,


        input              => data_tx,
        preload            => ssp_reset,
        preload_val        => TX_DATA_SHIFT_RST_VAL,
        enable             => '1',

        reg_stat           => tx_data_shift,
        output             => open
    );


    ----------------------------------------------------------------------------
    -- Secondary sampling point address decoder. Secondary sampling point
    -- is taken from SSP Shift register at position of transceiver delay.
    ----------------------------------------------------------------------------
    sample_sec_comb <= sample_sec_shift(to_integer(unsigned(trv_delay)));

    ----------------------------------------------------------------------------
    -- Delayed TX data address decoder. At the time of secondary sampling point,
    -- TX data from TX Data shift register at position of transceiver delay are
    -- taken for bit error detection!
    ----------------------------------------------------------------------------
    tx_data_delayed <= tx_data_shift(to_integer(unsigned(trv_delay)));


    ----------------------------------------------------------------------------
    -- Registering secondary sampling point
    ----------------------------------------------------------------------------
    ssp_gen_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            sample_sec          <= '0';
            
        elsif rising_edge(clk_sys) then  
            if (ssp_reset = '1') then
                sample_sec        <= '0';
            else
                sample_sec        <= sample_sec_comb;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Delay process for secondary sampling singal
    ----------------------------------------------------------------------------
    del_samsec_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            sample_sec_del_1    <= '0';  
            sample_sec_del_2    <= '0';    
        elsif (rising_edge(clk_sys)) then
            sample_sec_del_1    <= sample_sec;
            sample_sec_del_2    <= sample_sec_del_1;  
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Separate process for tripple sampling with simple shift register. 
    -- Sampling is continous into shift register of lenth 3. If this option is 
    -- desired then majority out of whole shift register is selected as sampled
    -- value!
    ----------------------------------------------------------------------------
    trs_shift_reg_comp : shift_reg
    generic map(
        reset_polarity     => ACT_RESET,
        reset_value        => "000",
        width              => 3,
        shift_down         => false
    )
    port map(
        clk                => clk_sys,
        res_n              => res_n,
        
        input              => CAN_rx_i,
        preload            => '0',
        preload_val        => "000",
        enable             => '1',

        reg_stat           => trs_reg,
        output             => open
    );


    ----------------------------------------------------------------------------
    -- Sampling of bus value
    ----------------------------------------------------------------------------
    sample_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            prev_Sample           <=  RECESSIVE;

        elsif rising_edge(clk_sys) then

            if (drv_ena = ENABLED) then
                case sp_control is

                ----------------------------------------------------------------
                -- Sampling with Nominal Bit Time 
                -- (normal CAN, transceiver, receiver)
                --
                -- Tripple sampling option selects the majority from last 
                -- three sampled values
                ----------------------------------------------------------------
                when NOMINAL_SAMPLE =>
                    if (sample_nbt = '1') then
                        prev_Sample <= data_rx_nbt;
                    end if;

                ----------------------------------------------------------------
                -- Sampling with Data Bit Time (CAN FD, reciever).
                ----------------------------------------------------------------
                when DATA_SAMPLE =>
                    if (sample_dbt = '1') then  
                        prev_Sample <= CAN_rx_i;
                    end if;

                ----------------------------------------------------------------
                -- Sampling with Secondary sampling point.
                -- (CAN FD, transciever)
                ----------------------------------------------------------------
                when SECONDARY_SAMPLE =>
                    if (sample_sec = '1') then
                        prev_Sample <= CAN_rx_i;
                    end if; 

                 when others =>
                    prev_Sample <= prev_Sample;
                end case;

            end if;
        end if;
    end process sample_proc;

    
    ----------------------------------------------------------------------------
    -- Bit Error detection process
    ----------------------------------------------------------------------------
    bit_err_detect_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            bit_Error_reg         <=  '0';

        elsif rising_edge(clk_sys) then

            if (drv_ena = ENABLED and bit_err_enable = '1') then
                case sp_control is

                ----------------------------------------------------------------
                -- Sampling with nominal bit time 
                -- (normal CAN, transciever, reciever)
                ----------------------------------------------------------------
                when NOMINAL_SAMPLE =>
                    if (sample_nbt = '1') then

                        -- If TX data are equal to RX Data -> No problem.
                        if (data_rx_nbt = data_tx) then 
                            bit_Error_reg <= '0';
                        else
                            bit_Error_reg <= '1';
                        end if;

                    end if;

                ----------------------------------------------------------------
                -- Sampling with data bit time (CAN FD, reciever)
                ----------------------------------------------------------------
                when DATA_SAMPLE =>
                    if (sample_dbt = '1') then 

                        --Bit Error detection when sampling
                        if (CAN_rx_i = data_tx) then 
                            bit_Error_reg <= '0';
                        else
                            bit_Error_reg <= '1';
                        end if; 
                    end if;

                ----------------------------------------------------------------
                -- Sampling with transciever delay compensation
                -- (CAN FD, transciever)
                ----------------------------------------------------------------
                when SECONDARY_SAMPLE =>
                    if (sample_sec = '1') then

                        -- Bit Error comparison differs in this case, not actual
                        -- transmitted bit is compared, but delayed bit is
                        -- compared (in ssp_shift register)
                        if (CAN_rx_i = tx_data_delayed) then
                            bit_Error_reg <= '0';
                        else
                            bit_Error_reg <= '1';
                        end if;
                    end if; 
                    
                 when others =>

                end case;

            -- If whole Core is disabled, or Bit Error detection is disabled,
            -- hold permanently in zero!
            else
                bit_Error_reg <= '0';
            end if;

        end if;  
    end process bit_err_detect_proc;


    -- Propagating sampled data to CAN Core
    data_rx            <= prev_Sample;

    --Output data propagation
    CAN_tx             <= data_tx;

    -- As synchroniation edge, valid edge on RX Data is selected!
    sync_edge          <= edge_rx_valid;

    -- Registers to output propagation
    sample_sec_out        <=  sample_sec;
    sample_sec_del_1_out  <=  sample_sec_del_1;
    sample_sec_del_2_out  <=  sample_sec_del_2;

    -- Bit Error propagation
    bit_Error             <=  bit_Error_reg;

end architecture;
