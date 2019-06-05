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

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity bit_stuffing is
    generic(
        -- Reset polarity
        G_RESET_POLARITY     :     std_logic := '0'
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys             :in   std_logic;
        
        -- Asynchronous reset
        res_n               :in   std_logic;

        ------------------------------------------------------------------------
        -- Data-path
        ------------------------------------------------------------------------
        -- Data Input (from Protocol Control)
        data_in             :in   std_logic;
        
        -- Data Output (to CAN Bus)
        data_out            :out  std_logic;
        
        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Bit Stuffing Trigger (in SYNC segment) 
        bst_trigger         :in   std_logic; 
        
        -- Bit Stuffing enabled. If not, data are only passed to the output
        stuff_enable        :in   std_logic;

        -- Bit Stuffing type (0-Normal, 1-Fixed)
        fixed_stuff         :in   std_logic;    

        -- Length of Bit Stuffing rule
        stuff_length        :in   std_logic_vector(2 downto 0); 

        ------------------------------------------------------------------------
        -- Status signals
        ------------------------------------------------------------------------
        -- Number of stuffed bits with Normal Bit stuffing
        bst_ctr             :out  natural range 0 to 7;
        
        -- Stuff bit is inserted, Protocol control operation to be halted for
        -- one bit time
        data_halt           :out  std_logic
    );
end entity;

architecture rtl of bit_stuffing is

    signal data_out_i        :     std_logic;

    ---------------------------------------------------------------------------
    -- Counter with number of equal consequent bits
    ---------------------------------------------------------------------------
    signal same_bits_q          :     natural range 0 to 7; 
    signal same_bits_add        :     natural range 0 to 7;
    signal same_bits_d          :     natural range 0 to 7;

    -- Halt for CAN Core             
    signal data_halt_q          :     std_logic;
    signal data_halt_d          :     std_logic;

    ---------------------------------------------------------------------------
    -- Registered value of fixed stuffing.
    ---------------------------------------------------------------------------
    signal fixed_reg_q          :     std_logic;
    signal fixed_reg_d          :     std_logic;

    ---------------------------------------------------------------------------
    -- Counter with regularly stuffed bits
    ---------------------------------------------------------------------------
    signal bst_ctr_q          :     natural range 0 to 7;
    signal bst_ctr_add        :     natural range 0 to 7;
    signal bst_ctr_d          :     natural range 0 to 7;

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
    signal data_out_d_ena     :     std_logic;

    ---------------------------------------------------------------------------
    -- Next data output value (both when enabled and disabled)
    ---------------------------------------------------------------------------
    signal data_out_d         :     std_logic;

    ---------------------------------------------------------------------------
    -- Clock enable for output data register
    ---------------------------------------------------------------------------
    signal data_out_ce          :     std_logic;

begin

    data_out <= data_out_i;

    ---------------------------------------------------------------------------
    -- Registering previous value of enable input to detect 0->1 transition.
    ---------------------------------------------------------------------------
    dff_ena_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => stuff_enable,
        ce                 => '1',
        output             => enable_prev
    );

    ---------------------------------------------------------------------------
    -- Detection of change on fixed stuff settings upon mismatch between
    -- actual and registered value of fixed stuff settings from previous bit.
    ---------------------------------------------------------------------------
    non_fix_to_fix_chng    <= '1' when (fixed_stuff = '1' and fixed_reg_q = '0')
                                  else
                              '0';

    ---------------------------------------------------------------------------
    -- Calculation of next value in fixed stuff register:
    --  1. Re-started upon 0->1 transition on "enable"
    --  2. Store "fixed_stuff" configuration when data are processed
    ---------------------------------------------------------------------------    
    fixed_reg_d <= '0'         when (enable_prev = '0') else
                   fixed_stuff when (bst_trigger = '1') else
                   fixed_reg_q;

    ---------------------------------------------------------------------------
    -- Registering previous value of fixed bit stuffing to detect first
    -- fixed stuff bit and insert stuff bit in the beginning of CRC for CAN FD
    -- automatically!
    ---------------------------------------------------------------------------
    dff_fixed_stuff_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => fixed_reg_d,
        ce                 => stuff_enable,
        output             => fixed_reg_q
    );

    ---------------------------------------------------------------------------    
    -- Combinationally incremented stuff counter by 1.
    ---------------------------------------------------------------------------
    bst_ctr_add <= (bst_ctr_q + 1) mod 8;


    ---------------------------------------------------------------------------
    -- Calculation of next combinational value for counter of stuffed bits:
    --  1. Erase when restarted bit stuffing.
    --  2. Upon insertion of non-fixed stuff bit increment.
    --  3. Keep previous value otherwise.
    ---------------------------------------------------------------------------
    bst_ctr_d <=  0           when (enable_prev = '0') else
                  bst_ctr_add when (bst_trigger = '1' and
                                    stuff_lvl_reached = '1' and
                                    fixed_stuff = '0') else
                    bst_ctr_q;

    ---------------------------------------------------------------------------
    -- Counter of stuffed bits (for CRC of ISO FD).
    ---------------------------------------------------------------------------    
    stuff_ctr_proc : process(res_n, clk_sys)
    begin
        if (res_n = G_RESET_POLARITY) then
            bst_ctr_q           <=  0;

        elsif rising_edge(clk_sys) then
            if (stuff_enable = '1') then
                bst_ctr_q       <= bst_ctr_d;
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
                                   (data_in /= data_out_i and fixed_stuff = '0')
                              else
                          '0';


    ---------------------------------------------------------------------------
    -- Reset counter of equal consecutive bits:
    --  1. Upon start of bit-stuffing
    --  2. When processing bit and should be restarted by dedicated signal.    
    ---------------------------------------------------------------------------
    same_bits_rst <= '1' when (enable_prev = '0') or
                              (bst_trigger = '1' and same_bits_rst_trig = '1')
                         else
                     '0';


    ---------------------------------------------------------------------------
    -- Combinationally incremented value of equal consecutive bits of equal
    -- value.
    ---------------------------------------------------------------------------
    same_bits_add <= (same_bits_q + 1) mod 8;


    ---------------------------------------------------------------------------
    -- Next value for counter of equal consecutive bits:
    --  1. Reset
    --  2. Increment if not reset when processing bit.
    --  3. Keep original value otherwise.
    ---------------------------------------------------------------------------
    same_bits_d <= 1             when (same_bits_rst = '1') else
                     same_bits_add when (bst_trigger = '1') else
                     same_bits_q;


    ---------------------------------------------------------------------------
    -- Number of stuff bits is reached when:
    --  1. Normal bit stuffing, number of same bits is equal to stuff rule
    --     length. Stuff bit is already included in counting next consecutive
    --     bits of equal value (recursive behaviour of bit-stuffing).
    --  2. Fixed bit stuffing, number of same bits is equal to one more than
    --     rule length, since stuff bit is not included then!
    ---------------------------------------------------------------------------
    stuff_lvl_reached <= '1' when (same_bits_q = unsigned(stuff_length) and fixed_stuff = '0') or
                                  (same_bits_q = unsigned(stuff_length) + 1 and fixed_stuff = '1')
                             else
                         '0';


    ---------------------------------------------------------------------------
    -- Counter of equal consecutive bits on input
    ---------------------------------------------------------------------------
    same_bits_ctr_proc : process(res_n, clk_sys)
    begin
        if (res_n = G_RESET_POLARITY) then
            same_bits_q           <=  1;

        elsif rising_edge(clk_sys) then
            if (stuff_enable = '1') then
                same_bits_q       <= same_bits_d;
            else
                same_bits_q       <= 1;
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
    data_out_d_ena <= (not data_out_i) when (bst_trigger = '1' and insert_stuff_bit = '1') else
                              data_in  when (bst_trigger = '1') else
                           data_out_i;

    data_out_d <= data_out_d_ena when (stuff_enable = '1') else
                         data_in when (bst_trigger = '1') else
                      data_out_i;

    data_out_ce <= '1' when (stuff_enable = '1' or bst_trigger = '1') else
                     '0';

    ---------------------------------------------------------------------------
    -- Output data register. Stuffed data are stored to this register in
    -- trigger, or input data are piped directly to this register when
    -- enable = '0'.
    ---------------------------------------------------------------------------
    dff_data_out_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => RECESSIVE
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => data_out_d,
        ce                 => data_out_ce,
        output             => data_out_i
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
    data_halt_d <= '0' when (enable_prev = '0' or stuff_enable = '0') else
                   '1' when (bst_trigger = '1' and insert_stuff_bit = '1') else
                   '0' when (bst_trigger = '1') else
                   data_halt_q;

    ---------------------------------------------------------------------------
    -- Halt register instance
    ---------------------------------------------------------------------------
    dff_halt_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => data_halt_d,
        ce                 => '1',
        output             => data_halt_q
    );


    ---------------------------------------------------------------------------
    -- Propagating internal signals to output 
    ---------------------------------------------------------------------------
    bst_ctr               <= bst_ctr_q;
    data_halt             <= data_halt_q;
  
end architecture;