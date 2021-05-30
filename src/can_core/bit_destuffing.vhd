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
-- Module:
--  Bit Destuffing.
-- 
-- Purpose:
--  Performs bit destuffing from received data stream. Operates in Destuff
--  pipeline stage with Bit Destuffing trigger. Length of Bit Destuffing rule
--  is controlled by Protocol control FSM. Implements regular bit stuffing,
--  as well as Fixed bit stuffing. Upon transition from non-fixed to fixed bit
--  stuffing, inserts an extra stuff bit. 
--  Detects stuff error if n+1-th bit is not opposite of previous processed bit. 
--  Indicates that bit was destuffed when n bits of equal polarity are processed.
--  Processing of input data always take one clock cycle (output is DFF). When
--  circuit is disabled, it only propagates data from input to output with Bit
--  Destuffing Trigger and does not detect Stuff Error, neither performs Bit
--  destuffing. Counts number of destuffed bits modulo 8.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity bit_destuffing is
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              : in std_logic;
        
        -- Asynchronous reset
        res_n                : in std_logic;

        ------------------------------------------------------------------------
        -- Data-path
        ------------------------------------------------------------------------
        -- Data input (from Bus Sampling)
        data_in              : in std_logic;
        
        -- Data output (to Protocol Control)
        data_out             : out std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Bit Destuffing Trigger (in Sample point, from Prescaler).
        bds_trigger          : in std_logic;

        -- Bit Destuffing is enabled.
        destuff_enable       : in  std_logic;

        -- Bit destuffing type (0-Normal, 1-Fixed)    
        fixed_stuff          : in  std_logic;  

        -- Length of Bit De-Stuffing rule
        destuff_length       : in  std_logic_vector(2 downto 0);  
       
        ------------------------------------------------------------------------
        -- Status Outpus
        ------------------------------------------------------------------------
        -- Stuff error detected (more equal consecutive bits than length of
        -- stuff rule.
        stuff_err            : out std_logic;
        
        -- Data output is not valid, actual bit is stuff bit.
        destuffed            : out std_logic;
        
        -- Number of de-stuffed bits with normal bit stuffing method
        dst_ctr              : out std_logic_vector(2 downto 0)
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
    signal fixed_prev_q            : std_logic;
    signal fixed_prev_d            : std_logic;

    ---------------------------------------------------------------------------
    -- Counter with number of equal consecutive bits on input
    ---------------------------------------------------------------------------
    signal same_bits_d             : unsigned(2 downto 0);
    signal same_bits_q             : unsigned(2 downto 0);
    signal same_bits_add           : unsigned(2 downto 0);
    signal same_bits_erase         : std_logic;

    ---------------------------------------------------------------------------
    -- Register with flag that bit was destuffed from serial stream
    ---------------------------------------------------------------------------
    signal destuffed_q             : std_logic;
    signal destuffed_d             : std_logic;

    ---------------------------------------------------------------------------
    -- Register with error flag signalling stuff error
    ---------------------------------------------------------------------------
    signal stuff_err_q             : std_logic;
    signal stuff_err_d             : std_logic;

    ---------------------------------------------------------------------------
    -- ISO CAN FD destuff bit counter
    -- Counter of destuffed bits by non-fixed bit stuffing.
    ---------------------------------------------------------------------------
    signal dst_ctr_q           : unsigned(2 downto 0);
    signal dst_ctr_d           : unsigned(2 downto 0);
    signal dst_ctr_add         : unsigned(2 downto 0);

    ---------------------------------------------------------------------------
    -- Value of previous processed bit.
    ---------------------------------------------------------------------------
    signal prev_val_q              : std_logic;
    signal prev_val_d              : std_logic;

begin

    ---------------------------------------------------------------------------
    -- Registering previous value of enable input to detect 0->1 transition.
    ---------------------------------------------------------------------------
    dff_ena_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => '0',
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,            -- IN
        clk                => clk_sys,          -- IN
        input              => destuff_enable,   -- IN
        
        output             => enable_prev       -- OUT
    );

    ---------------------------------------------------------------------------
    -- Detection of change on fixed stuff settings upon mismatch between
    -- actual and registered value of fixed stuff settings from previous bit.
    ---------------------------------------------------------------------------
    non_fix_to_fix_chng    <= '1' when (fixed_stuff = '1' and fixed_prev_q = '0')
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
    stuff_lvl_reached <= '1' when (same_bits_q = unsigned(destuff_length) and fixed_stuff = '0') or
                                  (same_bits_q = unsigned(destuff_length) + 1 and fixed_stuff = '1')
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
    fixed_prev_d <= '0'         when (enable_prev = '0') else
                    fixed_stuff when (bds_trigger = '1') else
                    fixed_prev_q;


    ---------------------------------------------------------------------------
    -- Stuff rules is violated under following conditions:
    --  1. Actually processed bit should be discarded.
    --  2. Previously processed bit is equal to actual bit on input
    --     (N+1 bit is not different)
    --  3. Stuff error detection is enabled.
    ---------------------------------------------------------------------------
    stuff_rule_violate <= '1' when (discard_stuff_bit = '1' and
                                    prev_val_q = data_in and
                                    destuff_enable = '1')
                              else
                          '0';


    ---------------------------------------------------------------------------
    -- Registering previous value of fixed bit stuffing to detect first
    -- fixed stuff bit and insert stuff bit in the beginning of CRC for CAN FD
    -- automatically!
    ---------------------------------------------------------------------------
    dff_fixed_stuff_reg : dff_arst_ce
    generic map(
        G_RESET_POLARITY   => '0',
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,            -- IN
        clk                => clk_sys,          -- IN
        input              => fixed_prev_d,     -- IN
        ce                 => destuff_enable,   -- IN
        
        output             => fixed_prev_q      -- OUT
    );


    ----------------------------------------------------------------------------
    -- Combinationally incremented valued of counter with number of destuffed
    -- bits.
    ---------------------------------------------------------------------------
    dst_ctr_add <= (dst_ctr_q + 1) mod 8;


    ----------------------------------------------------------------------------
    -- Counter with de-stuffed bits, next value:
    --  1. Erase upon edge on enable
    --  2. Increment when non-fixed stuff bit is inserted
    --  3. Keep otherwise
    ---------------------------------------------------------------------------
    dst_ctr_d <=       "000"  when (enable_prev = '0') else
                 dst_ctr_add  when (bds_trigger = '1' and 
                                    stuff_lvl_reached = '1' and
                                    fixed_stuff = '0') else
                  dst_ctr_q;


    ---------------------------------------------------------------------------
    -- Counter with number of de-stuffed bits - register assignment
    ---------------------------------------------------------------------------
    dst_ctr_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            dst_ctr_q         <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (destuff_enable = '1') then
                dst_ctr_q     <= dst_ctr_d;
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
    same_bits_erase <= '1' when (destuff_enable = '0' or enable_prev = '0') else
                       '1' when (bds_trigger = '1' and discard_stuff_bit = '1') else
                       '1' when (bds_trigger = '1' and 
                                 data_in /= prev_val_q and 
                                 fixed_stuff = '0') else
                       '0';

    ----------------------------------------------------------------------------
    -- Combinationally incremented value of counter of equal consecutive
    -- bits by 1.
    ---------------------------------------------------------------------------    
    same_bits_add   <= (same_bits_q + 1) mod 8;


    ----------------------------------------------------------------------------
    -- Next value for counter of equal consecutive bits:
    --  1. Erase counter when signalled.
    --  2. Increment upon processing of bit.
    --  3. Keep its value otherwise.
    ---------------------------------------------------------------------------
    same_bits_d   <=         "001" when (same_bits_erase = '1') else
                     same_bits_add when (bds_trigger = '1') else
                     same_bits_q;


    ----------------------------------------------------------------------------
    -- Counter of equal consecutive bits - register assignment.
    ---------------------------------------------------------------------------
    same_bits_ctr_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            same_bits_q <= "001";
        elsif (rising_edge(clk_sys)) then
            same_bits_q <= same_bits_d;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Destuffed flag - next value:
    --  1. Erase when circuit is disabled.
    --  2. Set when bit is processed and destuffed.
    --  3. Erase when bit is processed but should not be discarded.
    --  4. Keep value otherwise.
    ---------------------------------------------------------------------------
    destuffed_d   <= '0' when (destuff_enable = '0') else
                     '1' when (bds_trigger = '1' and
                               discard_stuff_bit = '1') else
                     '0' when (bds_trigger = '1') else
                     destuffed_q;


    ---------------------------------------------------------------------------
    -- Destuffed flag - register assignment
    ---------------------------------------------------------------------------
    dff_destuffed_flag_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => '0',
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,            -- IN
        clk                => clk_sys,          -- IN
        input              => destuffed_d,      -- IN
        
        output             => destuffed_q       -- OUT
    );


    ---------------------------------------------------------------------------
    -- Error register next value:
    --  1. Set when bit should be processed and stuff rule is violated.
    --  2. Cleared otherwise
    ---------------------------------------------------------------------------
    stuff_err_d <= '1' when (bds_trigger = '1' and stuff_rule_violate = '1') else
                   '0';


    ---------------------------------------------------------------------------
    -- Error register - register assignment
    ---------------------------------------------------------------------------
    dff_err_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => '0',
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,            -- IN
        clk                => clk_sys,          -- IN
        input              => stuff_err_d,      -- IN
        
        output             => stuff_err_q       -- OUT
    );


    ----------------------------------------------------------------------------
    -- Previously processed value - next value:
    --  1. Set to RECESSIVE upon edge on enable
    --  2. Set to RECESSIVE when non-fixed bit stuffing changes to fixed
    --     bit stuffing.
    ---------------------------------------------------------------------------
    prev_val_d <= RECESSIVE when (bds_trigger = '1' and non_fix_to_fix_chng = '1') else
                  data_in   when (bds_trigger = '1') else
                  prev_val_q;


    ---------------------------------------------------------------------------
    -- Previously processed value - register assignment
    ---------------------------------------------------------------------------
    dff_prev_val_reg : dff_arst
    generic map(
        G_RESET_POLARITY   => '0',
        G_RST_VAL          => RECESSIVE
    )
    port map(
        arst               => res_n,            -- IN
        clk                => clk_sys,          -- IN
        input              => prev_val_d,       -- IN
        
        output             => prev_val_q        -- OUT
    );


    ---------------------------------------------------------------------------
    -- Sampling of data value to output during operation. One clock cycle
    -- of delay is inserted so that next pipeline stage always processes the
    -- same data!
    ---------------------------------------------------------------------------
    dff_data_out_val_reg : dff_arst_ce
    generic map(
        G_RESET_POLARITY   => '0',
        G_RST_VAL          => RECESSIVE
    )
    port map(
        arst               => res_n,
        clk                => clk_sys,

        input              => data_in,
        ce                 => bds_trigger,
        output             => data_out
    );

    ---------------------------------------------------------------------------
    -- Propagation to output
    ---------------------------------------------------------------------------

    destuffed   <= destuffed_q;
    stuff_err   <= stuff_err_q;
    dst_ctr     <= std_logic_vector(dst_ctr_q);


    -- <RELEASE_OFF>
    ----------------------------------------------------------------------------
    -- Assertions on input settings
    ----------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl valid_stuff_length_setting_asrt : assert never
    --   ((destuff_length = "000" or destuff_length = "001") and (destuff_enable = '1'))
    -- report "0 and 1 bit stuffing length is invalid!" severity error;

    -- psl bds_non_fix_to_fixed_change_cov : cover
    --  {bds_trigger = '1' and non_fix_to_fix_chng = '1'};

    -- psl bds_stuff_err_detect_cov : cover
    --  {stuff_err_q = '1'};

    -- psl bds_stuff_lvl_reached_regular_cov : cover
    --  {stuff_lvl_reached = '1' and fixed_stuff = '0'};

    -- psl bds_stuff_lvl_reached_fixed_cov : cover
    --  {stuff_lvl_reached = '1' and fixed_stuff = '1'};

    -- <RELEASE_ON>

end architecture;
