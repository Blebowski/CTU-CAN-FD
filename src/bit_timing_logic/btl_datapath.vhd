--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2021-2023 Ondrej Ille
-- Copyright (C) 2023-     Logic Design Services Ltd.s
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- non-commercial purposes. Using the Component for commercial purposes is
-- forbidden unless previously agreed with Copyright holder.
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
--  Bit Timing Logic - Datapath
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity btl_datapath is
    generic (
        G_TSEG1_WIDTH       :     natural;
        G_TSEG2_WIDTH       :     natural;
        G_BRP_WIDTH         :     natural;
        G_SJW_WIDTH         :     natural
    );
    port(
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        rst_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_btr_prop             : in  std_logic_vector(6 downto 0);
        mr_btr_ph1              : in  std_logic_vector(5 downto 0);
        mr_btr_ph2              : in  std_logic_vector(5 downto 0);
        mr_btr_brp              : in  std_logic_vector(7 downto 0);
        mr_btr_sjw              : in  std_logic_vector(4 downto 0);

        mr_btr_fd_prop_fd       : in  std_logic_vector(5 downto 0);
        mr_btr_fd_ph1_fd        : in  std_logic_vector(4 downto 0);
        mr_btr_fd_ph2_fd        : in  std_logic_vector(4 downto 0);
        mr_btr_fd_brp_fd        : in  std_logic_vector(7 downto 0);
        mr_btr_fd_sjw_fd        : in  std_logic_vector(4 downto 0);

        -------------------------------------------------------------------------------------------
        -- Control Interface from PC FSM
        -------------------------------------------------------------------------------------------
        sync_edge               : in  std_logic;
        bit_rate_d              : in  t_bit_rate;
        bit_rate_q              : in  t_bit_rate;
        sync_control            : in  std_logic_vector(1 downto 0);
        no_pos_resync           : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Interface to BT FSM
        -------------------------------------------------------------------------------------------
        is_tseg1                : in  std_logic;
        is_tseg2                : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Trigger signals
        -------------------------------------------------------------------------------------------
        rx_trigger              : out std_logic;
        tx_trigger              : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Status outputs
        -------------------------------------------------------------------------------------------
        tq_edge                 : out std_logic
    );
end entity;

architecture rtl of btl_datapath is

    -----------------------------------------------------------------------------------------------
    -- State information
    -----------------------------------------------------------------------------------------------

    -- Time Quanta Counter
    signal tq_cnt_nxt               : unsigned(G_BRP_WIDTH - 1 downto 0);
    signal tq_cnt_d                 : unsigned(G_BRP_WIDTH - 1 downto 0);
    signal tq_cnt_q                 : unsigned(G_BRP_WIDTH - 1 downto 0);
    signal tq_cnt_pload             : std_logic;

    constant C_TQ_COUNTER_ZERO      : unsigned(G_BRP_WIDTH - 1 downto 0) := (others => '0');

    signal tq_edge_i                : std_logic;

    -- Segment Counter
    -- Should cover largest segment that can be counted in the given bit-rate also with synchronisation!
    --  Nominal:  SYNC (1) + PROP_NBT (127) + PH1_NBT (63) + SJW_NBT (31) = 222 -> Fits into 8 bits
    --  Data:     SYNC (1) + PROP_DBT (63)  + PH1_DBT (31) + SJW_DBT (31) = 126 -> Fits into 7 bits
    constant C_BT_WIDTH             : natural := 8;

    signal segm_cnt_d               : unsigned(C_BT_WIDTH - 1 downto 0);
    signal segm_cnt_q               : unsigned(C_BT_WIDTH - 1 downto 0);
    signal segm_cnt_tseg1           : unsigned(C_BT_WIDTH - 1 downto 0);
    signal segm_cnt_tseg2           : unsigned(C_BT_WIDTH - 1 downto 0);

    -- Synchronization flag
    signal sync_flag_q              : std_logic;
    signal sync_flag_set            : std_logic;
    signal sync_flag_clr            : std_logic;

    -- Phase error counter
    signal phase_err_d              : unsigned(C_BT_WIDTH - 1 downto 0);
    signal phase_err_q              : unsigned(C_BT_WIDTH - 1 downto 0);
    signal phase_err_sat            : unsigned(C_BT_WIDTH - 1 downto 0);
    signal phase_err_lt_sjw         : std_logic;
    signal phase_err_mt_sjw_by_one  : std_logic;
    signal sjw_mt_zero              : std_logic;

    -----------------------------------------------------------------------------------------------
    -- Other signals
    -----------------------------------------------------------------------------------------------

    signal tseg1_prop               : unsigned(G_TSEG1_WIDTH - 1 downto 0);
    signal tseg1_ph1                : unsigned(G_TSEG1_WIDTH - 1 downto 0);
    signal tseg1_minus_1            : unsigned(G_TSEG1_WIDTH - 1 downto 0);
    signal tseg2_ph2                : unsigned(G_TSEG2_WIDTH - 1 downto 0);
    signal tseg2_minus_1            : unsigned(G_TSEG2_WIDTH - 1 downto 0);
    signal sjw                      : unsigned(G_SJW_WIDTH - 1 downto 0);

    signal tx_trigger_i             : std_logic;
    signal rx_trigger_i             : std_logic;
    signal rx_trigger_q             : std_logic;
    signal rx_trigger_set           : std_logic;

    signal h_sync_edge              : std_logic;
    signal resync_edge              : std_logic;
    signal h_sync_edge_valid        : std_logic;
    signal immediate_tseg2_end      : std_logic;
    signal preload_shorter_tseg1    : std_logic;
    signal pos_resync_edge_valid    : std_logic;
    signal neg_resync_edge_valid    : std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- Time quanta counter
    -----------------------------------------------------------------------------------------------
    tq_edge_i <= '1' when (tq_cnt_q = C_TQ_COUNTER_ZERO) else
                 '0';

    tq_cnt_pload <= '1' when (tq_edge_i = '1' or tx_trigger_i = '1' or rx_trigger_i = '1') else
                    '0';

    tq_cnt_nxt <=
              unsigned(mr_btr_brp) when (tq_cnt_pload = '1' and bit_rate_d = BIT_RATE_NOMINAL) else
        unsigned(mr_btr_fd_brp_fd) when (tq_cnt_pload = '1' and bit_rate_d = BIT_RATE_FD) else
                         tq_cnt_q;

    -- This could be move and merged with tq_cnt_nxt if we BRP was from zero!
    tq_cnt_d <= tq_cnt_nxt - 1;

    p_tq_counter : process(clk_sys, rst_n)
    begin
        if (rst_n = '0') then
            tq_cnt_q <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            tq_cnt_q <= tq_cnt_d;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Segment counter
    -----------------------------------------------------------------------------------------------
    -- TODO: This logic could be written better with less additions and subtractions!
    segm_cnt_tseg1 <=              tseg1_minus_1 - 1 when (h_sync_edge_valid = '1' or
                                                           preload_shorter_tseg1 = '1') else
                      segm_cnt_q + phase_err_sat - 1 when (tq_edge = '1' and
                                                           pos_resync_edge_valid = '1') else
                      tseg1_minus_1;

    segm_cnt_tseg2 <= segm_cnt_q - phase_err_sat - 1 when (neg_resync_edge_valid = '1') else
                      resize(tseg2_minus_1, C_BT_WIDTH);

    segm_cnt_d <= segm_cnt_tseg1 when (tx_trigger_i = '1' or pos_resync_edge_valid = '1' or
                                       h_sync_edge_valid = '1' or immediate_tseg2_end = '1') else
                  segm_cnt_tseg2 when (rx_trigger_i = '1' or neg_resync_edge_valid = '1') else
                  segm_cnt_q - 1;

    p_segm_cnt : process(clk_sys, rst_n)
    begin
        if (rst_n = '0') then
            segm_cnt_q <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            if (tq_edge_i = '1') then
                segm_cnt_q <= segm_cnt_d;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Phase error counting
    -----------------------------------------------------------------------------------------------
    phase_err_d <= to_unsigned(1, C_BT_WIDTH) when (h_sync_edge_valid = '1' or
                                                    preload_shorter_tseg1 = '1') else
                   to_unsigned(0, C_BT_WIDTH) when (tx_trigger = '1') else
                resize(tseg2_ph2, C_BT_WIDTH) when (rx_trigger = '1') else
                              phase_err_q - 1 when (is_tseg2 = '1') else
                              phase_err_q + 1;

    p_phase_err_reg : process(clk_sys, rst_n)
    begin
        if (rst_n = '0') then
            phase_err_q <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            if (tq_edge_i = '1') then
                phase_err_q <= phase_err_d;
            end if;
        end if;
    end process;

    -- SJW needs to be based on "old" bit-rate since it does not affect TSEG2 length in the
    -- moment of bit-rate shift! Contrarily, if late edge arrives exactly at the end of TSEG1,
    -- (when RX trigger should be normally active), positive resynchronization occurs. At that
    -- time we gate RX trigger if SJW > 0 (since we prolong TSEG1). Such SJW choice must use
    -- the "old" bit-rate, not the "new to be switched to"!
    sjw <= unsigned(mr_btr_sjw) when (bit_rate_q = BIT_RATE_NOMINAL) else
           unsigned(mr_btr_fd_sjw_fd);

    sjw_mt_zero <= '1' when (sjw > 0) else
                   '0';

    phase_err_lt_sjw <= '1' when (phase_err_q <= sjw) else
                        '0';

    phase_err_mt_sjw_by_one <= '1' when (phase_err_q = sjw + 1) else
                               '0';

    phase_err_sat <= phase_err_q when (phase_err_lt_sjw = '1') else
                     resize(sjw, C_BT_WIDTH);

    -------------------------------------------------------------------------------------------
    -- Time segments length computations
    -- TSEG1 can be muxed based on "old" value of bit-rate since it does not change at the
    -- end of bit (TODO: This will be different for CAN XL )!
    -- TSEG2 must be muxed based on "new" value (active at sample point) since at the moment
    -- of BRS, segm_cnt must be preloaded to length in "new" bit-rate.
    -------------------------------------------------------------------------------------------
    tseg1_prop <=
        resize(unsigned(mr_btr_prop), G_TSEG1_WIDTH) when (bit_rate_q = BIT_RATE_NOMINAL) else
        resize(unsigned(mr_btr_fd_prop_fd), G_TSEG1_WIDTH);

    tseg1_ph1 <=
        resize(unsigned(mr_btr_ph1), G_TSEG1_WIDTH) when (bit_rate_q = BIT_RATE_NOMINAL) else
        resize(unsigned(mr_btr_fd_ph1_fd), G_TSEG1_WIDTH);

    tseg1_minus_1 <= tseg1_prop + tseg1_ph1;

    tseg2_ph2 <= resize(unsigned(mr_btr_ph2), G_TSEG2_WIDTH) when (bit_rate_d = BIT_RATE_NOMINAL) else
                 resize(unsigned(mr_btr_fd_ph2_fd), G_TSEG2_WIDTH);

    -- The subtract could be removed if PH2 from zero!
    tseg2_minus_1 <= resize(unsigned(tseg2_ph2) - 1, G_TSEG2_WIDTH);

    -----------------------------------------------------------------------------------------------
    -- Synchronization logic
    -----------------------------------------------------------------------------------------------
    resync_edge <= '1' when (sync_edge = '1' and sync_control = RE_SYNC) else
                   '0';

    h_sync_edge <= '1' when (sync_edge = '1' and sync_control = HARD_SYNC) else
                   '0';

    sync_flag_set <= '1' when (resync_edge = '1' or h_sync_edge = '1') else
                     '0';

    sync_flag_clr <= '1' when (rx_trigger_i = '1') else
                     '0';

    p_sync_flag : process(rst_n, clk_sys)
    begin
        if (rst_n = '0') then
            sync_flag_q <= '0';
        elsif (rising_edge(clk_sys)) then
            -- Set should have priority here for the case of H_SYNC right at the end
            -- of TSEG1. Then rx_trigger_i = 1 at the same time as hard-sync occurs,
            -- and TSEG1 restarts, but more sync edges until next TSEG1 end should
            -- be ignored!
            if (sync_flag_set = '1') then
                sync_flag_q <= '1';
            elsif (sync_flag_clr = '1') then
                sync_flag_q <= '0';
            end if;
        end if;
    end process;

    h_sync_edge_valid <= '0' when (sync_flag_q = '1') else
                         '0' when (no_pos_resync = '1' and is_tseg1 = '1') else
                         '1' when (h_sync_edge = '1') else
                         '0';

    immediate_tseg2_end <= '1' when (is_tseg2 = '1' and resync_edge = '1' and
                                     (phase_err_lt_sjw = '1' or phase_err_mt_sjw_by_one = '1')) else
                           '0';

    preload_shorter_tseg1 <= '1' when (is_tseg2 = '1' and resync_edge = '1' and
                                       phase_err_lt_sjw = '1') else
                             '0';

    pos_resync_edge_valid <= '0' when (no_pos_resync = '1' or sync_flag_q = '1') else
                             '1' when (is_tseg1 = '1' and resync_edge = '1') else
                             '0';

    neg_resync_edge_valid <= '0' when (sync_flag_q = '1') else
                             '1' when (is_tseg2 = '1' and resync_edge = '1') else
                             '0';

    -------------------------------------------------------------------------------------------
    -- TX / RX trigger generation
    -------------------------------------------------------------------------------------------

    -- TX Trigger can't be registered as it may need to occur due to immediate hard
    -- synchronization edge or negative resynchronization shorter than SJW.
    -- This is OK, since TX trigger does not have big combo fanout.
    tx_trigger_i <= '0' when (is_tseg2 = '0' or tq_edge = '0') else
                    '1' when (h_sync_edge_valid = '1' or segm_cnt_q = 0 or
                              immediate_tseg2_end = '1') else
                    '0';

    -- To be set when:
    --  1. TSEG1 and we are about to enter last Time Quanta of TSEG2
    --  2. TSEG2 and follow-up time quanta will be only 1!
    rx_trigger_set <= '1' when (is_tseg1 = '1' and segm_cnt_d = 0 and tq_edge_i = '1' and
                                rx_trigger_q = '0') else
                      '1' when (is_tseg2 = '1' and tseg1_minus_1 = x"00" and tx_trigger_i = '1') else
                      '0';

    -- RX Trigger can be registered since it only occurs after planned segment length counted
    -- to zero! Also has heavy combo fanout in PC FSM!
    p_rx_trigger : process (clk_sys, rst_n)
    begin
        if (rst_n = '0') then
            rx_trigger_q <= '0';
        elsif (rising_edge(clk_sys)) then
            if (rx_trigger_set = '1') then
                rx_trigger_q <= '1';
            elsif (tq_edge_i= '1') then
                rx_trigger_q <= '0';
            end if;
        end if;
    end process;

    -------------------------------------------------------------------------------------------
    -- Internal signals to output propagation
    -------------------------------------------------------------------------------------------
    tq_edge <= tq_edge_i;
    tx_trigger <= tx_trigger_i;

    -- We must gate RX trigger in case hard-sync or positive resync occured.
    -- For hard-sync, the edge may arrive anytime, even in sample point, we must reset the bit
    -- then, not process it.
    -- For positive resync, the edge may arrive right at the sample point, we must not process
    -- the data (end TSEG1, generate RX Trigger), but prolong the TSEG1 instead!
    rx_trigger_i <= '0' when (h_sync_edge_valid = '1' or
                              (pos_resync_edge_valid = '1' and sjw_mt_zero = '1')) else
                    '1' when (rx_trigger_q = '1' and tq_edge = '1') else
                    '0';
    rx_trigger <= rx_trigger_i;

    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl min_lenght_ph1_nbt_asrt : assert always
    --  {rst_n = '1'} |=>
    --      {to_integer(unsigned(mr_btr_ph1) + unsigned(mr_btr_prop) + 1) * to_integer(unsigned(mr_btr_brp)) > 2}
    --  report "Lenght of TSEG1(NBT) must be more than 2 clock cycles!";

    -- psl min_lenght_ph2_nbt_asrt : assert always
    --  {rst_n = '1'} |=>
    --      {to_integer(unsigned(mr_btr_ph2)) * to_integer(unsigned(mr_btr_brp)) >= 2}
    --  report "Lenght of TSEG2(NBT) must be more than 1 clock cycle!";

    -- psl min_lenght_ph1_dbt_asrt : assert always
    --  {rst_n = '1'} |=>
    --      {to_integer(unsigned(mr_btr_fd_ph1_fd) + unsigned(mr_btr_fd_prop_fd) + 1) * to_integer(unsigned(mr_btr_fd_brp_fd)) > 2}
    --  report "Lenght of TSEG1(DBT) must be more than 2 clock cycles!";

    -- psl min_lenght_ph2_dbt_asrt : assert always
    --  {rst_n = '1'} |=>
    --       {to_integer(unsigned(mr_btr_fd_ph2_fd)) * to_integer(unsigned(mr_btr_fd_brp_fd)) >= 2}
    --  report "Lenght of TSEG2(DBT) must be more than 1 clock cycle!";

end architecture rtl;