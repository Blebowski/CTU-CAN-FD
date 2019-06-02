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
--  Error detector.
--------------------------------------------------------------------------------
-- Revision History:
--    29.3.2019   Created file
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

entity err_detector is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :     std_logic := '0';
        
        -- Pipeline should be inserted on Error signalling
        G_ERR_VALID_PIPELINE    :     boolean
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;

        -- Asynchronous reset
        res_n                   :in   std_logic;

        -----------------------------------------------------------------------
        -- Data-path interface
        -----------------------------------------------------------------------
        -- Actual TX Data
        tx_data                 :in   std_logic;
        
        -- Actual RX Data
        rx_data                 :in   std_logic;
        
        -----------------------------------------------------------------------
        -- Error sources
        -----------------------------------------------------------------------
        -- Bit error (from Bus sampling)
        bit_err                 :in   std_logic;
        
        -- Bit error in Arbitration field
        bit_err_arb             :in   std_logic;
        
        -- Stuff error
        stuff_err               :in   std_logic;
        
        -- Form Error
        form_err                :in   std_logic;
        
        -- ACK Error
        ack_err                 :in   std_logic;

        -- CRC Error
        crc_err                 :in   std_logic;
        
        -----------------------------------------------------------------------
        -- CRC comparison data
        -----------------------------------------------------------------------
        -- Received CRC
        rx_crc                  :in   std_logic_vector(20 downto 0);
        
        -- Calculated CRC 15
        crc_15                  :in   std_logic_vector(14 downto 0);

        -- Calculated CRC 17
        crc_17                  :in   std_logic_vector(16 downto 0);
        
        -- Calculated CRC 21
        crc_21                  :in   std_logic_vector(20 downto 0);
        
        -- Received Stuff count (Gray coded) + Parity
        rx_stuff_count          :in   std_logic_vector(3 downto 0);
        
        -- Destuff counter mod 8
        dst_ctr                 :in   natural range 0 to 7;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Bit error enable
        bit_err_enable          :in   std_logic;

        -- Stuff error enable
        stuff_err_enable        :in   std_logic;

        -- Fixed Bit stuffing method
        fixed_stuff             :in   std_logic;

        -- Error position field (from Protocol control)
        err_pos                 :in   std_logic_vector(4 downto 0);

        -- Perform CRC Check
        crc_check               :in   std_logic;

        -- Clear CRC match flag
        crc_clear_match_flag    :in   std_logic;

        -- CRC Source (CRC15, CRC17, CRC21)
        crc_src                 :in   std_logic_vector(1 downto 0);

        -- FD Type (ISO FD, NON-ISO FD)
        drv_fd_type             :in   std_logic;

        -- Arbitration field is being transmitted / received
        is_arbitration          :in   std_logic;

        -- Unit is transmitter of frame
        is_transmitter          :in   std_logic;

        -- Unit is error passive
        is_err_passive          :in   std_logic;

        -----------------------------------------------------------------------
        -- Status output
        -----------------------------------------------------------------------
        -- Error frame request
        err_frm_req             :out  std_logic;

        -- Error detected (for Fault confinement)
        err_detected            :out  std_logic;

        -- Error code capture
        erc_capture             :out  std_logic_vector(7 downto 0);

        -- CRC match
        crc_match               :out  std_logic;

        -- Error counters should remain unchanged
        err_ctrs_unchanged      :out  std_logic
    );
end entity;

architecture rtl of err_detector is

    -- Internal Error valid
    signal err_frm_req_i  : std_logic;

    -- Error capture register
    signal err_type_d     : std_logic_vector(2 downto 0);
    signal err_type_q     : std_logic_vector(2 downto 0);
    signal err_pos_q      : std_logic_vector(4 downto 0);
    
    -- Internal form error
    signal form_err_i     : std_logic;

    -- CRC Match detection
    signal crc_match_c    : std_logic;
    signal crc_match_d    : std_logic;
    signal crc_match_q    : std_logic;
    
    -- De-Stuff counter grey coded
    signal dst_ctr_grey   : std_logic_vector(2 downto 0);
    signal dst_parity     : std_logic;
    
    -- Stuff counter should be checked
    signal stuff_count_check : std_logic;
    
    -- CRC Check results
    signal crc_15_ok       : std_logic;
    signal crc_17_ok       : std_logic;
    signal crc_21_ok       : std_logic;
    signal stuff_count_ok  : std_logic;

    -- Destuff counter converted to vector
    signal dst_ctr_vect    : std_logic_vector(2 downto 0);
    
    -- Aliases for received CRC (for easier debugging)
    signal rx_crc_15       : std_logic_vector(14 downto 0);
    signal rx_crc_17       : std_logic_vector(16 downto 0);
    signal rx_crc_21       : std_logic_vector(20 downto 0);

begin

    ---------------------------------------------------------------------------
    -- Error frame request. Invoked by each Error type which should cause
    -- Error frame in the following bit!
    ---------------------------------------------------------------------------

    -- Error frame request for any type of error which causes transition to
    -- Error frame in the next bit.
    err_frm_req_i <= '1' when (bit_err = '1' and bit_err_enable = '1') else
                     '1' when (stuff_err = '1' and stuff_err_enable = '1') else
                     '1' when (form_err = '1' or ack_err = '1') else
                     '1' when (crc_err = '1') else
                     '1' when (bit_err_arb = '1') else
                     '0';

    -- Fixed stuff error shall be reported as Form Error!
    form_err_i <= '1' when (form_err = '1') else
                  '1' when (stuff_err = '1' and stuff_err_enable = '1' and
                            fixed_stuff = '1') else
                  '0';

    err_pipeline_true_gen : if (G_ERR_VALID_PIPELINE) generate
    begin
        err_valid_reg_proc : process(res_n, clk_sys)
        begin
            if (res_n = G_RESET_POLARITY) then
                err_frm_req <= '0';
            elsif (rising_edge(clk_sys)) then
                err_frm_req <= err_frm_req_i;
            end if;
        end process;
    end generate err_pipeline_true_gen;

    err_pipeline_false_gen : if (not G_ERR_VALID_PIPELINE) generate
    begin
        err_frm_req <= err_frm_req_i;
    end generate err_pipeline_false_gen;


    ---------------------------------------------------------------------------
    -- De-Stuff counter grey-coding + parity
    ---------------------------------------------------------------------------
    with dst_ctr select dst_ctr_grey <= 
        "000" when 0,
        "001" when 1,
        "011" when 2,
        "010" when 3,
        "110" when 4,
        "111" when 5,
        "101" when 6,
        "100" when 7,
        "000" when others;

    dst_ctr_vect <= std_logic_vector(to_unsigned(dst_ctr, 3));
    dst_parity <= dst_ctr_grey(0) xor dst_ctr_grey(1) xor dst_ctr_grey(2);

    ---------------------------------------------------------------------------
    -- CRC Check
    ---------------------------------------------------------------------------
    -- Check stuff counters for ISO FD and FD Frames only!
    stuff_count_check <= '1' when (drv_fd_type = ISO_FD) and
                                  (crc_src = C_CRC17_SRC or crc_src = C_CRC21_SRC)
                             else
                         '0';

    -- CRC aliases from RX Shift register
    rx_crc_15 <= rx_crc(14 downto 0);
    rx_crc_17 <= rx_crc(16 downto 0);
    rx_crc_21 <= rx_crc(20 downto 0);
    
    -- CRC 15 bits check
    crc_15_ok <= '1' when (rx_crc_15 = crc_15) else
                 '0';

    -- CRC 17 check
    crc_17_ok <= '1' when (rx_crc_17 = crc_17) else
                 '0';
                 
    -- CRC 21 check
    crc_21_ok <= '1' when (rx_crc_21 = crc_21) else
                 '0';

    -- Stuff counter OK, including parity!
    stuff_count_ok <= '1' when (rx_stuff_count = dst_ctr_grey & dst_parity)
                          else
                      '0';

    -- CRC Match
    crc_match_c <= '0' when (crc_15_ok = '0' and crc_src = C_CRC15_SRC) or
                            (crc_17_ok = '0' and crc_src = C_CRC17_SRC) or
                            (crc_21_ok = '0' and crc_src = C_CRC21_SRC) or
                            (stuff_count_ok = '0' and stuff_count_check = '1')
                       else
                   '1';

    crc_match_d <= '0' when (crc_clear_match_flag = '1') else
                   crc_match_c when (crc_check = '1') else
                   crc_match_q;
    
    crc_err_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            crc_match_q <= '0';
        elsif (rising_edge(clk_sys)) then
            crc_match_q <= crc_match_d;
        end if;
    end process;
                      
    --------------------------------------------------------------------------
    -- Error counters should remain unchanged according to 12.1.4.2 in 
    -- ISO11898-1:2015 in following cases:
    --  1. Error passive transmitter detects ACK error.
    --  2. Transmitter detects stuff error in Arbitration when bit should
    --     have been recessive, but was transmitted dominant!
    --------------------------------------------------------------------------
    err_ctrs_unchanged <= '1' when (ack_err = '1' and is_err_passive = '1')
                              else
                          '1' when (stuff_err = '1' and
                                    stuff_err_enable = '1' and
                                    is_arbitration = '1' and
                                    rx_data = DOMINANT and
                                    tx_data = RECESSIVE)
                              else
                          '0';


    -- Error is detected when error frame is requested
    err_detected <= err_frm_req_i;

    --------------------------------------------------------------------------
    -- Error code, next value
    ---------------------------------------------------------------------------
    err_type_d <= "000" when (bit_err = '1' and bit_err_enable = '1') else
                  "001" when (crc_err = '1') else
                  "010" when (form_err_i = '1') else
                  "011" when (ack_err = '1') else
                  "100" when (stuff_err = '1' and stuff_err_enable = '1') else
                  err_type_q;
                  
    ---------------------------------------------------------------------------
    -- Error type register
    ---------------------------------------------------------------------------
    err_type_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            err_type_q <= "000";
            err_pos_q <= "11111";
        elsif (rising_edge(clk_sys)) then
            if (err_frm_req_i = '1' or crc_err = '1') then
                err_type_q <= err_type_d;
                err_pos_q  <= err_pos;
            end if;
        end if;
    end process;

    -- Internal signal to output propagation
    erc_capture <= err_pos_q & err_type_q;
    crc_match <= crc_match_q;

    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

end architecture;