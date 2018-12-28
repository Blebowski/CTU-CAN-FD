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
--  Simple bit stuffing circuit with HandShake protocol. When bit is stuffed 
--  transciever (CAN Core) has to stop transcieving for one bit time. data_halt 
--  output is set to logic 1 when bit is stuffed.
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Second version of bit Stuffing circuit. Enables configurable stuff length. 
-- Operation starts when enable='1'. Valid data already has to be on data_in 
-- then. Operates with triggering signal tran_trig_1 Fixed Stuffing method can 
-- be used by setting logic on fixed_stuff input. In fixed stuff inverse bit is 
-- inserted after every stuff_length bits, even if their polarity is not equal!                     
--------------------------------------------------------------------------------
-- Revision History:
--
--    June 2015  Created file
--    July 2015  Created second version of the bitstuffing circuit
--    19.5.2016  same_bits counter erased when edge detection on fixed_stuff 
--               detected. Avoids inserting stuff bit in CRC field after less 
--               than stuff_count bits when last bits of data field were equal!
--    6.6.2016   Added fixed stuff bit at the transition from non fixed stuff to
--               fixed stuff! Thisway bit stuffing also covers the one fixed 
--               stuff bit in the beginning of CRC phase!! Added bit stuffing 
--               counter to count the dynamic stuff bits in ISO FD.
--    13.6.2016  Added mod 8 into same_bits counter increase
--    12.1.2017  Changed priority of fixed bit-stuffing processing. Fixed bit 
--               stuffing should always have higher priority than non-fixed 
--               bit-stuffing and thus be before in the If-elsif condition!
--               This is due to possible conflic of normal and fixed bit stuffing
--               in the start of FD CRC. Fixed bit-stuff should win!
--   22.12.2018  Re-worked bit-stuffing to hae the logic separated to several
--               processes.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;

entity bit_stuffing is 
    port(

        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        signal clk_sys        :in   std_logic;
        signal res_n          :in   std_logic;

        ------------------------------------------------------------------------
        -- Prescaler interface - sampling
        ------------------------------------------------------------------------
        --Trigger signal for propagating the data 
        --(one clk_sys delayed behind beginning of bit time) 
        signal tran_trig_1    :in   std_logic; 

        ------------------------------------------------------------------------
        --CAN Core interface
        ------------------------------------------------------------------------

        -- Enabling the operation of the circuit
        signal enable         :in   std_logic; 

        -- Data Input sampled
        signal data_in        :in   std_logic;

        -- If fixed bit stuffing should be used (CRC of CAN FD)
        signal fixed_stuff    :in   std_logic;    

        -- Logic 1 signals stuffed bit for CAN Core. CAN Core has to halt the 
        -- data sending for one bit-time 
        signal data_halt      :out  std_logic;    

        -- Length of Bit Stuffing
        signal length         :in   std_logic_vector(2 downto 0); 

        -- Bit stuffing counter
        signal bst_ctr        :out  natural range 0 to 7;

        ------------------------------------------------------------------------
        --Bus Synchroniser interface
        ------------------------------------------------------------------------
        signal data_out       :out  std_logic --Data output
        --Note: Data are sent into bus synchroniser but can be also
        --      fed back to CAN Core for CRC calculation in CAN FD Phase!
        
    );
end entity;


architecture rtl of bit_stuffing is

    ---------------------------------------------------------------------------
    -- Counter with number of equal consequent bits
    ---------------------------------------------------------------------------
    signal same_bits            :     natural range 0 to 7; 
    signal same_bits_add        :     natural range 0 to 7;
    signal same_bits_nxt        :     natural range 0 to 7;


    -- Value of previously transcieved bit
    signal prev_bit             :     std_logic;

    -- Halt for CAN Core             
    signal halt_reg             :     std_logic;
    signal halt_reg_nxt         :     std_logic;

    ---------------------------------------------------------------------------
    -- Registered value of fixed stuffing.
    ---------------------------------------------------------------------------
    signal fixed_reg            :     std_logic;
    signal fixed_reg_nxt        :     std_logic;

    ---------------------------------------------------------------------------
    -- Counter with regularly stuffed bits
    ---------------------------------------------------------------------------
    signal stuff_ctr            :     natural range 0 to 7;
    signal stuff_ctr_add        :     natural range 0 to 7;
    signal stuff_ctr_nxt        :     natural range 0 to 7;

    ---------------------------------------------------------------------------
    -- Registered value of enable input
    ---------------------------------------------------------------------------
    signal enable_prev          :     std_logic;


    ---------------------------------------------------------------------------
    -- Combinational signals
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Bit stuffing method has changed from non-fixed to fixed bit stuffing.
    -- No need to assume change from fixed to non-fixed since after coding
    -- CRC of CAN FD, there are no further stuff bits with non-fixed bit
    -- stuffing!
    ---------------------------------------------------------------------------
    signal non_fix_to_fix_chng  :     std_logic;

    ---------------------------------------------------------------------------
    -- Signals that stuff count has reached number of same consecutive bits
    -- and that stuff bit should be inserted!
    ---------------------------------------------------------------------------
    signal stuff_lvl_reached    :     std_logic;

    ---------------------------------------------------------------------------
    -- Counter of equal consecutive bits should be re-set to 1 in next
    -- processed bit
    ---------------------------------------------------------------------------
    signal same_bits_rst_trig   :     std_logic;

    ---------------------------------------------------------------------------
    -- Counter of equal consecutive bits should be re-set to 1 in next clock
    -- cycle.
    ---------------------------------------------------------------------------
    signal same_bits_rst        :     std_logic;

    ---------------------------------------------------------------------------
    -- Condition for insertion of stuff bit
    ---------------------------------------------------------------------------
    signal insert_stuff_bit     :     std_logic;

    ---------------------------------------------------------------------------
    -- Calculation of next data output value when circuit is enabled
    ---------------------------------------------------------------------------
    signal data_out_nxt_ena     :     std_logic;

    ---------------------------------------------------------------------------
    -- Next data output value (both when enabled and disabled)
    ---------------------------------------------------------------------------
    signal data_out_nxt         :     std_logic;

    ---------------------------------------------------------------------------
    -- Clock enable for output data register
    ---------------------------------------------------------------------------
    signal data_out_load        :     std_logic;

begin

    ---------------------------------------------------------------------------
    -- Registering previous value of enable input to detect 0->1 transition.
    ---------------------------------------------------------------------------
    dff_ena_reg : dff_arst
    generic map(
        reset_polarity     => ACT_RESET,
        rst_val            => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => enable,
        load               => '1',
        output             => enable_prev
    );

    ---------------------------------------------------------------------------
    -- Detection of change on fixed stuff settings upon mismatch between
    -- actual and registered value of fixed stuff settings from previous bit.
    ---------------------------------------------------------------------------
    non_fix_to_fix_chng    <= '1' when (fixed_stuff = '1' and fixed_reg = '0')
                                  else
                              '0';

    ---------------------------------------------------------------------------
    -- Calculation of next value in fixed stuff register:
    --  1. Re-started upon 0->1 transition on "enable"
    --  2. Store "fixed_stuff" configuration when data are processed
    ---------------------------------------------------------------------------    
    fixed_reg_nxt <= '0'         when (enable_prev = '0') else
                     fixed_stuff when (tran_trig_1 = '1') else
                     fixed_reg;

    ---------------------------------------------------------------------------
    -- Registering previous value of fixed bit stuffing to detect first
    -- fixed stuff bit and insert stuff bit in the beginning of CRC for CAN FD
    -- automatically!
    ---------------------------------------------------------------------------
    dff_fixed_stuff_reg : dff_arst
    generic map(
        reset_polarity     => ACT_RESET,
        rst_val            => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => fixed_reg_nxt,
        load               => enable,
        output             => fixed_reg
    );

    ---------------------------------------------------------------------------    
    -- Combinationally incremented stuff counter by 1.
    ---------------------------------------------------------------------------
    stuff_ctr_add <= (stuff_ctr + 1) mod 8;


    ---------------------------------------------------------------------------
    -- Calculation of next combinational value for counter of stuffed bits:
    --  1. Erase when restarted bit stuffing.
    --  2. Upon insertion of non-fixed stuff bit increment.
    --  3. Keep previous value otherwise.
    ---------------------------------------------------------------------------
    stuff_ctr_nxt <= 0             when (enable_prev = '0') else
                     stuff_ctr_add when (tran_trig_1 = '1' and
                                         stuff_lvl_reached = '1' and
                                         fixed_stuff = '0') else
                     stuff_ctr;

    ---------------------------------------------------------------------------
    -- Counter of stuffed bits (for CRC of ISO FD).
    ---------------------------------------------------------------------------    
    stuff_ctr_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            stuff_ctr           <=  0;

        elsif rising_edge(clk_sys) then
            if (enable = '1') then
                stuff_ctr       <= stuff_ctr_nxt;
            end if;  
        end if;
    end process;


    ---------------------------------------------------------------------------
    -- Reset counter of equal consecutive bits to 1 with bit processing when:
    --  1. Processing first bit of fixed bit stuffing
    --  2. Stuff level was reached -> Stuff bit will be inserted
    --  3. Processed bit differs from previous bit (data out register) and 
    --     regular stuffing is used.
    ---------------------------------------------------------------------------
    same_bits_rst_trig <= '1' when (non_fix_to_fix_chng = '1') or
                                   (stuff_lvl_reached = '1') or
                                   (data_in /= data_out and fixed_stuff = '0')
                              else
                          '0';


    ---------------------------------------------------------------------------
    -- Reset counter of equal consecutive bits:
    --  1. Upon start of bit-stuffing
    --  2. When processing bit and should be restarted by dedicated signal.    
    ---------------------------------------------------------------------------
    same_bits_rst <= '1' when (enable_prev = '0') or
                              (tran_trig_1 = '1' and same_bits_rst_trig = '1')
                         else
                     '0';


    ---------------------------------------------------------------------------
    -- Combinationally incremented value of equal consecutive bits of equal
    -- value.
    ---------------------------------------------------------------------------
    same_bits_add <= (same_bits + 1) mod 8;


    ---------------------------------------------------------------------------
    -- Next value for counter of equal consecutive bits:
    --  1. Reset
    --  2. Increment if not reset when processing bit.
    --  3. Keep original value otherwise.
    ---------------------------------------------------------------------------
    same_bits_nxt <= 1             when (same_bits_rst = '1') else
                     same_bits_add when (tran_trig_1 = '1') else
                     same_bits;


    ---------------------------------------------------------------------------
    -- Number of stuff bits is reached when:
    --  1. Normal bit stuffing, number of same bits is equal to stuff rule
    --     length. Stuff bit is already included in counting next consecutive
    --     bits of equal value (recursive behaviour of bit-stuffing).
    --  2. Fixed bit stuffing, number of same bits is equal to one more than
    --     rule length, since stuff bit is not included then!
    ---------------------------------------------------------------------------
    stuff_lvl_reached <= '1' when (same_bits = unsigned(length) and fixed_stuff = '0') or
                                  (same_bits = unsigned(length) + 1 and fixed_stuff = '1')
                             else
                         '0';


    ---------------------------------------------------------------------------
    -- Counter of equal consecutive bits on input
    ---------------------------------------------------------------------------
    same_bits_ctr_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            same_bits           <=  1;

        elsif rising_edge(clk_sys) then
            if (enable = '1') then
                same_bits       <= same_bits_nxt;
            else
                same_bits       <= 1;
            end if;
        end if;
    end process;


    ---------------------------------------------------------------------------
    -- Stuff bit should be inserted:
    --  1. Upon change of non-fixed to fixed bit stuffing
    --  2. Number of equal consecutive bits has reached length of stuff rule.
    ---------------------------------------------------------------------------
    insert_stuff_bit <= '1' when (non_fix_to_fix_chng = '1' or 
                                  stuff_lvl_reached = '1')
                            else
                        '0';       

    ---------------------------------------------------------------------------
    -- Calculation of output data value:
    --  1. Recessive bit after restart
    --  2. Negation of previous value when stuff bit is inserted.
    --  3. Pipe the input data upon trigger without stufffing
    --  4. Keep previous value otherwise
    ---------------------------------------------------------------------------
    data_out_nxt_ena <= RECESSIVE      when (enable_prev = '0') else
                        (not data_out) when (tran_trig_1 = '1' and insert_stuff_bit = '1') else
                        data_in        when (tran_trig_1 = '1') else
                        data_out;

    data_out_nxt <= data_out_nxt_ena when (enable = '1') else
                    data_in          when (tran_trig_1 = '1') else
                    data_out;

    data_out_load <= '1' when (enable = '1' or tran_trig_1 = '1') else
                     '0';

    ---------------------------------------------------------------------------
    -- Output data register. Stuffed data are stored to this register in
    -- trigger, or input data are piped directly to this register when
    -- enable = '0'.
    ---------------------------------------------------------------------------
    dff_data_out_reg : dff_arst
    generic map(
        reset_polarity     => ACT_RESET,
        rst_val            => RECESSIVE
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => data_out_nxt,
        load               => data_out_load,
        output             => data_out
    );


    ---------------------------------------------------------------------------
    -- Halt register signals to CAN Core that it should wait for one clock
    -- cycle because stuff-bit was inserted.
    --
    -- Next value for halt register:
    --  1. Erase upon start of bit-stuffing, or when circuit is disabled.
    --  2. Signal halt when stuff bit is inserted.
    --  3. Erase when bit is processed, but stuff bit is not inserted.
    ---------------------------------------------------------------------------
    halt_reg_nxt <= '0' when (enable_prev = '0' or enable = '0') else
                    '1' when (tran_trig_1 = '1' and insert_stuff_bit = '1') else
                    '0' when (tran_trig_1 = '1') else
                    halt_reg;

    ---------------------------------------------------------------------------
    -- Halt register instance
    ---------------------------------------------------------------------------
    dff_halt_reg : dff_arst
    generic map(
        reset_polarity     => ACT_RESET,
        rst_val            => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => halt_reg_nxt,
        load               => '1',
        output             => halt_reg
    );


    ---------------------------------------------------------------------------
    -- Propagating internal signals to output 
    ---------------------------------------------------------------------------
    bst_ctr               <= stuff_ctr;
    data_halt             <= halt_reg; --Propagating halt value BACK to CAN Core
  
end architecture;
