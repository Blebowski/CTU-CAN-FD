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
--Purpose:
--  Bit destuffing circuit. Data sampled always with valid trig_spl_1 signal. 
--  Length of bitStuffing controlled via stuff_length input. Stuff error signa-
--  lises Error when the stuff rule is not valid (stuff_lenght+1) consecutive   
--  bits of the same polarity. Signal destuffed  indicates that current output
--  bit is not valid data bit, but is destuffed bit taken out from input data
--  stream!                                                                  
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--    19.5.2016   1. Added Stuff bit counter to cover ISO FD extra field!
--                2. Edge detection 0->1 added at fixed_stuff input. Once edge 
--                   is detected same_bits counter is erased! This prevents the 
--                   error of inserting stuff bit sooner than fixed length when 
--                   last bit of data field have equal value!
--    6.6.2016    Added fixed stuff bit at the transition from non fixed stuff 
--                to fixed stuff! Thisway bit stuffing also covers the one fixed
--                stuff bit in the beginning of CRC phase!! Added bit stuffing 
--                counter to count the dynamic stuff bits in ISO FD.
--   13.6.2016    1.Added mod 8 into same_bits counter increase
--                2.Added keeping previous value of dst_counter when circuit is 
--                  disabled instead of erasing! This way ciruit is compatible
--                  with bit stuffing!
--                3.Added warning when bit stuffing rule is set to 0 or 1 which
--                  is invalid setting!
--    12.1.2017  Changed priority of fixed bit-destuffing processing. Fixed bit 
--               destuffing should always have higher priority than non-fixed 
--               bit-destuffing and thus be before in the If-elsif condition!
--               This is due to possible conflic of normal and fixed bit destu-
--               ffing in the start of FD CRC. Fixed bit-destuff should win!
--    23.5.2018  Bug-fix of stuff error detection. Stuff error on special
--               stuff-bit in first bit of CRC also must be detected.
--   28.12.2018  Re-worked bit-destuffing to hae the logic separated to several
--               processes.
--------------------------------------------------------------------------------

library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use WORK.can_constants.all;
use work.cmn_lib.all;

entity bit_destuffing is
    port(
        ------------------------------------------------------------------------
        -- Clock And Reset
        ------------------------------------------------------------------------
        signal clk_sys : in std_logic;
        signal res_n   : in std_logic;

        ------------------------------------------------------------------------
        -- Bus Sampling Interface
        ------------------------------------------------------------------------
        signal data_in : in std_logic;

        ------------------------------------------------------------------------
        -- Prescaler interface
        ------------------------------------------------------------------------
        -- Triggering signal with one clk_sys delay behind the used 
        -- sampling signal
        signal trig_spl_1 : in std_logic;

        ------------------------------------------------------------------------
        --Error Signalling
        ------------------------------------------------------------------------

        -- Stuff error detected when stuff rule is 
        signal stuff_Error  : out std_logic;

        ------------------------------------------------------------------------
        --CAN Core interface
        ------------------------------------------------------------------------

        -- Data output for CAN Core
        signal data_out           : out std_logic;

        -- Signal that data on output are not valid but it is a stuff bit
        signal destuffed          : out std_logic;  

        -- Enable of the circuit
        signal enable             : in  std_logic;

        -- Enable stuff Error logging
        signal stuff_Error_enable : in  std_logic;

        -- Whenever fixed bit Destuffing method is used    
        signal fixed_stuff        : in  std_logic;  

        -- Length of bit stuffing rule
        signal length             : in  std_logic_vector(2 downto 0);  

        -- Number of destuffed bits with regular bit stuffing method
        signal dst_ctr            : out natural range 0 to 7  
    );
end entity;

architecture rtl of bit_destuffing is

    -- Stuff bit should be discarded
    signal discard_stuff_bit       : std_logic;

    -- Change from non-fixed to fixed bit stuffing occured
    signal non_fix_to_fix_chng     : std_logic;

    -- Number of equal consecutive bits on input is equal to length of stuff
    -- rule
    signal stuff_lvl_reached       : std_logic;

    -- Stuff rule is violated -> Stuff error.
    signal stuff_rule_violate      : std_logic;

    -- Previous value of enable - register
    signal enable_prev             : std_logic;

    ---------------------------------------------------------------------------
    -- Previous value of fixed stuff - register 
    ---------------------------------------------------------------------------
    signal fixed_prev              : std_logic;
    signal fixed_prev_nxt          : std_logic;

    ---------------------------------------------------------------------------
    -- Counter with number of equal consecutive bits on input
    ---------------------------------------------------------------------------
    signal same_bits               : natural range 0 to 7;
    signal same_bits_add           : natural range 0 to 7;
    signal same_bits_nxt           : natural range 0 to 7;
    signal same_bits_erase         : std_logic;

    ---------------------------------------------------------------------------
    -- Register with flag that bit was destuffed from serial stream
    ---------------------------------------------------------------------------
    signal destuffed_reg           : std_logic;
    signal destuffed_reg_nxt       : std_logic;

    ---------------------------------------------------------------------------
    -- Register with error flag signalling stuff error
    ---------------------------------------------------------------------------
    signal error_reg               : std_logic;
    signal error_reg_nxt           : std_logic;

    ---------------------------------------------------------------------------
    -- ISO CAN FD destuff bit counter
    -- Counter of destuffed bits by non-fixed bit stuffing.
    ---------------------------------------------------------------------------
    signal dst_bit_ctr             : natural range 0 to 7;
    signal dst_bit_ctr_nxt         : natural range 0 to 7;
    signal dst_bit_ctr_add         : natural range 0 to 7;

    ---------------------------------------------------------------------------
    -- Value of previous processed bit.
    ---------------------------------------------------------------------------
    signal prev_val                : std_logic;
    signal prev_val_nxt            : std_logic;

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
    non_fix_to_fix_chng    <= '1' when (fixed_stuff = '1' and fixed_prev = '0')
                                  else
                              '0';

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
    -- Stuff bit should be discarded:
    --  1. Upon change of non-fixed to fixed bit stuffing
    --  2. Number of equal consecutive bits has reached length of stuff rule.
    ---------------------------------------------------------------------------
    discard_stuff_bit <= '1' when (non_fix_to_fix_chng = '1' or 
                                   stuff_lvl_reached = '1')
                             else
                         '0';


    ---------------------------------------------------------------------------
    -- Calculation of next value in fixed stuff register:
    --  1. Re-started upon 0->1 transition on "enable"
    --  2. Store "fixed_stuff" configuration when data are processed
    ---------------------------------------------------------------------------    
    fixed_prev_nxt <= '0'         when (enable_prev = '0') else
                      fixed_stuff when (trig_spl_1 = '1') else
                      fixed_prev;


    ---------------------------------------------------------------------------
    -- Stuff rules is violated under following conditions:
    --  1. Actually processed bit should be discarded.
    --  2. Previously processed bit is equal to actual bit on input
    --     (N+1 bit is not different)
    --  3. Stuff error detection is enabled.
    ---------------------------------------------------------------------------
    stuff_rule_violate <= '1' when (discard_stuff_bit = '1' and
                                    prev_val = data_in and
                                    stuff_Error_enable = '1')
                              else
                          '0';


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

        input              => fixed_prev_nxt,
        load               => enable,
        output             => fixed_prev
    );


    ----------------------------------------------------------------------------
    -- Combinationally incremented valued of counter with number of destuffed
    -- bits.
    ---------------------------------------------------------------------------
    dst_bit_ctr_add <= (dst_bit_ctr + 1) mod 8;


    ----------------------------------------------------------------------------
    -- Counter with de-stuffed bits, next value:
    --  1. Erase upon edge on enable
    --  2. Increment when non-fixed stuff bit is inserted
    --  3. Keep otherwise
    ---------------------------------------------------------------------------
    dst_bit_ctr_nxt <= 0                when (enable_prev = '0') else
                       dst_bit_ctr_add  when (trig_spl_1 = '1' and 
                                              stuff_lvl_reached = '1' and
                                              fixed_stuff = '0') else
                       dst_bit_ctr;


    ---------------------------------------------------------------------------
    -- Counter with number of de-stuffed bits - register assignment
    ---------------------------------------------------------------------------
    dst_bit_ctr_proc : process(clk_sys, res_n)
    begin
        if (res_n = ACT_RESET) then
            dst_bit_ctr         <= 0;

        elsif (rising_edge(clk_sys)) then
            if (enable = '1') then
                dst_bit_ctr     <= dst_bit_ctr_nxt;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Counter of equal consecutive bits should be erased:
    --  1. Circuit disabled, or just enabled (edge on enable)
    --  2. Stuff bit is just discarded.
    --  3. Bit is processed by non-fixed bit stuffing, but it differs from
    --     previous processed bit.
    ---------------------------------------------------------------------------
    same_bits_erase <= '1' when (enable = '0' or enable_prev = '0') else
                       '1' when (trig_spl_1 = '1' and discard_stuff_bit = '1') else
                       '1' when (trig_spl_1 = '1' and 
                                 data_in /= prev_val and 
                                 fixed_stuff = '0') else
                       '0';

    ----------------------------------------------------------------------------
    -- Combinationally incremented value of counter of equal consecutive
    -- bits by 1.
    ---------------------------------------------------------------------------    
    same_bits_add   <= (same_bits + 1) mod 8;


    ----------------------------------------------------------------------------
    -- Next value for counter of equal consecutive bits:
    --  1. Erase counter when signalled.
    --  2. Increment upon processing of bit.
    --  3. Keep its value otherwise.
    ---------------------------------------------------------------------------
    same_bits_nxt   <= 1             when (same_bits_erase = '1') else
                       same_bits_add when (trig_spl_1 = '1') else
                       same_bits;


    ----------------------------------------------------------------------------
    -- Counter of equal consecutive bits - register assignment.
    ---------------------------------------------------------------------------
    same_bits_ctr_proc : process(clk_sys, res_n)
    begin
        if (res_n = ACT_RESET) then
            same_bits <= 1;

        elsif (rising_edge(clk_sys)) then
            same_bits <= same_bits_nxt;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Destuffed flag - next value:
    --  1. Erase when circuit is disabled.
    --  2. Set when bit is processed and destuffed.
    --  3. Erase when bit is processed but should not be discarded.
    --  4. Keep value otherwise.
    ---------------------------------------------------------------------------
    destuffed_reg_nxt   <= '0' when (enable = '0') else
                           '1' when (trig_spl_1 = '1' and
                                     discard_stuff_bit = '1') else
                           '0' when (trig_spl_1 = '1') else
                           destuffed_reg;


    ---------------------------------------------------------------------------
    -- Destuffed flag - register assignment
    ---------------------------------------------------------------------------
    dff_destuffed_flag_reg : dff_arst
    generic map(
        reset_polarity     => ACT_RESET,
        rst_val            => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => destuffed_reg_nxt,
        load               => '1',
        output             => destuffed_reg
    );


    ---------------------------------------------------------------------------
    -- Error register next value:
    --  1. Set when bit should be processed and stuff rule is violated.
    --  2. Cleared otherwise
    ---------------------------------------------------------------------------
    error_reg_nxt <= '1' when (trig_spl_1 = '1' and stuff_rule_violate = '1') else
                     '0';


    ---------------------------------------------------------------------------
    -- Error register - register assignment
    ---------------------------------------------------------------------------
    dff_error_reg : dff_arst
    generic map(
        reset_polarity     => ACT_RESET,
        rst_val            => '0'
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => error_reg_nxt,
        load               => '1',
        output             => error_reg
    );


    ----------------------------------------------------------------------------
    -- Previously processed value - next value:
    --  1. Set to RECESSIVE upon edge on enable
    --  2. Set to RECESSIVE when non-fixed bit stuffing changes to fixed
    --     bit stuffing. TODO: IS THIS OK???
    ---------------------------------------------------------------------------
    prev_val_nxt <= RECESSIVE when (enable = '1' and enable_prev = '0') else
                    RECESSIVE when (trig_spl_1 = '1' and non_fix_to_fix_chng = '1') else
                    data_in   when (trig_spl_1 = '1') else
                    prev_val;


    ---------------------------------------------------------------------------
    -- Previously processed value - register assignment
    ---------------------------------------------------------------------------
    dff_prev_val_reg : dff_arst
    generic map(
        reset_polarity     => ACT_RESET,
        rst_val            => RECESSIVE
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => prev_val_nxt,
        load               => '1',
        output             => prev_val
    );


    ---------------------------------------------------------------------------
    -- Propagation to output
    ---------------------------------------------------------------------------

    -- Data output is fed directly from input, only destuffed bits are marked
    -- via "destuffed".
    data_out    <= data_in;

    destuffed   <= destuffed_reg;
    stuff_Error <= error_reg;
    dst_ctr     <= dst_bit_ctr;


    ----------------------------------------------------------------------------
    -- Assertions on input settings
    ----------------------------------------------------------------------------
    input_length_assert_proc : process(clk_sys)
    begin
        if (rising_edge(clk_sys)) then
            if ((length = "000" or length = "001") and (enable = '1')) then
                -- LCOV_EXCL_START
                report "0 and 1 bit stuffing length is invalid!" severity warning;
                -- LCOV_EXCL_STOP
            end if;
        end if;
    end process;

end architecture;

