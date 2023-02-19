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
--  Frame filters for CAN RX frame.
--
-- Purpose:
--  Filters out commands for RX Buffer based on value of received CAN Identifier.
--  Filter identifier type, frame type are controlled by Driving Bus.
--  11 bit and 29 bit filters can be compared. If 13 bit filters are compared,
--  then MSB 18 bits in Received Identifier has to be zeros. Also mask for the
--  filter in case of 16-bit filter HAS to have 16 uppest bits equal to zero!
--  Filters  A,B,C and Range are present. If input identifier matches at least one
--  it is considered as valid. Frame type (CAN Basic, CAN Extended, CAN FD Basic)
--  are also selectable for filtering. Filters can be optionally left out from
--  synthesis or disabled in runtime. If filters are disabled, no frame is
--  filtered out.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity frame_filters is
    generic (
        -- Support filter A
        G_SUP_FILTA                         : boolean := true;

        -- Support filter B
        G_SUP_FILTB                         : boolean := true;

        -- Support filter C
        G_SUP_FILTC                         : boolean := true;

        -- Support range filter
        G_SUP_RANGE                         : boolean := true
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock an Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                             : in  std_logic;
        res_n                               : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_filter_control_fafe              : in  std_logic;
        mr_filter_control_fafb              : in  std_logic;
        mr_filter_control_fane              : in  std_logic;
        mr_filter_control_fanb              : in  std_logic;

        mr_filter_control_fbfe              : in  std_logic;
        mr_filter_control_fbfb              : in  std_logic;
        mr_filter_control_fbne              : in  std_logic;
        mr_filter_control_fbnb              : in  std_logic;

        mr_filter_control_fcfe              : in  std_logic;
        mr_filter_control_fcfb              : in  std_logic;
        mr_filter_control_fcne              : in  std_logic;
        mr_filter_control_fcnb              : in  std_logic;

        mr_filter_control_frfe              : in  std_logic;
        mr_filter_control_frfb              : in  std_logic;
        mr_filter_control_frne              : in  std_logic;
        mr_filter_control_frnb              : in  std_logic;

        mr_filter_a_mask_bit_mask_a_val     : in  std_logic_vector(28 downto 0);
        mr_filter_a_val_bit_val_a_val       : in  std_logic_vector(28 downto 0);
        mr_filter_b_mask_bit_mask_b_val     : in  std_logic_vector(28 downto 0);
        mr_filter_b_val_bit_val_b_val       : in  std_logic_vector(28 downto 0);
        mr_filter_c_mask_bit_mask_c_val     : in  std_logic_vector(28 downto 0);
        mr_filter_c_val_bit_val_c_val       : in  std_logic_vector(28 downto 0);
        mr_filter_ran_high_bit_ran_high_val : in  std_logic_vector(28 downto 0);
        mr_filter_ran_low_bit_ran_low_val   : in  std_logic_vector(28 downto 0);

        mr_settings_fdrf                    : in  std_logic;
        mr_mode_afm                         : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- CAN Core interface
        -------------------------------------------------------------------------------------------
        -- Receieved CAN ID
        rec_ident                           : in  std_logic_vector(28 downto 0);

        -- Received CAN ID type (0-Base Format, 1-Extended Format);
        rec_ident_type                      : in  std_logic;

        -- Input frame type (0-CAN 2.0, 1- CAN FD)
        rec_frame_type                      : in  std_logic;

        -- RX Remote transmission request Flag
        rec_is_rtr                          : in  std_logic;

        -- Store Metadata in RX Buffer
        store_metadata                      : in  std_logic;

        -- Command to store word of CAN Data
        store_data                          : in  std_logic;

        -- Received frame valid
        rec_valid                           : in  std_logic;

        -- Command to abort storing of RX frame (due to Error frame)
        rec_abort                           : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Frame filters output
        -------------------------------------------------------------------------------------------
        -- Store Metadata in RX Buffer - Filtered
        store_metadata_f                    : out std_logic;

        -- Command to store word of CAN Data - Filtered
        store_data_f                        : out std_logic;

        -- Received frame valid - Filtered
        rec_valid_f                         : out std_logic;

        -- Command to abort storing of RX frame (due to Error frame) - Filtered
        rec_abort_f                         : out std_logic
    );
end entity;

architecture rtl of frame_filters is

    -- Outputs of individual filters are valid
    signal int_filter_A_valid       :       std_logic;
    signal int_filter_B_valid       :       std_logic;
    signal int_filter_C_valid       :       std_logic;
    signal int_filter_ran_valid     :       std_logic;

    -- Masks for each filter with allowed frame types
    signal mask_filter_a            :       std_logic_vector(3 downto 0);
    signal mask_filter_b            :       std_logic_vector(3 downto 0);
    signal mask_filter_c            :       std_logic_vector(3 downto 0);
    signal mask_filter_range        :       std_logic_vector(3 downto 0);

    -- Frame type on input to be compared with driving signal
    signal int_data_type            :       std_logic_vector(3 downto 0);

    -- Concat of types of data on input
    signal int_data_ctrl            :       std_logic_vector(1 downto 0);

    -- Enable signal for filters
    signal filter_a_enable          :       std_logic;
    signal filter_b_enable          :       std_logic;
    signal filter_c_enable          :       std_logic;
    signal filter_range_enable      :       std_logic;

    signal filter_result            :       std_logic;

    -- Valid output value
    signal ident_valid_d            :       std_logic;
    signal ident_valid_q            :       std_logic;

    -- Indication that frame should be dropped
    signal drop_rtr_frame           :       std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- Decoding Filter enables based on accepted frame types by each filter
    -----------------------------------------------------------------------------------------------

    -- Input frame type internal signal
    int_data_ctrl <= rec_frame_type & rec_ident_type;

    -- Decoder frame_type&ident_type to one-hot
    with int_data_ctrl select int_data_type <=
        "0001" when "00",       --CAN Basic
        "0010" when "01",       --CAN Extended
        "0100" when "10",       --CAN FD Basic
        "1000" when others;     --CAN Fd Extended

    -- Filter is enabled when at least one Frame type/Identifier type is matching
    -- the configured value
    mask_filter_a <= mr_filter_control_fafe & mr_filter_control_fafb &
                     mr_filter_control_fane & mr_filter_control_fanb;

    mask_filter_b <= mr_filter_control_fbfe & mr_filter_control_fbfb &
                     mr_filter_control_fbne & mr_filter_control_fbnb;

    mask_filter_c <= mr_filter_control_fcfe & mr_filter_control_fcfb &
                     mr_filter_control_fcne & mr_filter_control_fcnb;

    mask_filter_range <= mr_filter_control_frfe & mr_filter_control_frfb &
                         mr_filter_control_frne & mr_filter_control_frnb;


    filter_a_enable <= '1' when ((mask_filter_a and int_data_type) /= x"0")
                           else
                       '0';

    filter_b_enable <= '1' when ((mask_filter_b and int_data_type) /= x"0")
                           else
                       '0';

    filter_c_enable <= '1' when ((mask_filter_c and int_data_type) /= x"0")
                           else
                       '0';

    filter_range_enable <= '1' when ((mask_filter_range and int_data_type) /= x"0")
                               else
                           '0';

    -----------------------------------------------------------------------------------------------
    -- Filter instances
    -----------------------------------------------------------------------------------------------
    bit_filter_a_inst : entity ctu_can_fd_rtl.bit_filter
    generic map (
        G_WIDTH         => 29,
        G_IS_PRESENT    => G_SUP_FILTA
    )
    port map (
        filter_mask     => mr_filter_a_mask_bit_mask_a_val,         -- IN
        filter_value    => mr_filter_a_val_bit_val_a_val,           -- IN
        filter_input    => rec_ident,                               -- IN
        enable          => filter_a_enable,                         -- IN

        valid           => int_filter_a_valid                       -- OUT
    );

    bit_filter_b_inst : entity ctu_can_fd_rtl.bit_filter
    generic map (
        G_WIDTH         => 29,
        G_IS_PRESENT    => G_SUP_FILTB
    )
    port map (
        filter_mask     => mr_filter_b_mask_bit_mask_b_val,         -- IN
        filter_value    => mr_filter_b_val_bit_val_b_val,           -- IN
        filter_input    => rec_ident,                               -- IN
        enable          => filter_b_enable,                         -- IN

        valid           => int_filter_b_valid                       -- OUT
    );

    bit_filter_c_inst : entity ctu_can_fd_rtl.bit_filter
    generic map (
        G_WIDTH         => 29,
        G_IS_PRESENT    => G_SUP_FILTC
    )
    port map (
        filter_mask     => mr_filter_c_mask_bit_mask_c_val,         -- IN
        filter_value    => mr_filter_c_val_bit_val_c_val,           -- IN
        filter_input    => rec_ident,                               -- IN
        enable          => filter_c_enable,                         -- IN

        valid           => int_filter_c_valid                       -- OUT
    );

    range_filter_inst : entity ctu_can_fd_rtl.range_filter
    generic map (
        G_WIDTH         => 29,
        G_IS_PRESENT    => G_SUP_RANGE
    )
    port map (
        filter_upp_th   => mr_filter_ran_high_bit_ran_high_val,     -- IN
        filter_low_th   => mr_filter_ran_low_bit_ran_low_val,       -- IN
        filter_input    => rec_ident,                               -- IN
        enable          => filter_range_enable,                     -- IN

        valid           => int_filter_ran_valid                     -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- If no filter is supported then Identifier is always valid, regardless of SETTINGS[ENA] ! If
    -- Core is not synthesized, turning filters on should not affect the acceptance! Everyhting
    -- should be affected!
    -----------------------------------------------------------------------------------------------
    filt_sup_gen_false : if (G_SUP_FILTA = false and G_SUP_FILTB = false and
                             G_SUP_FILTC = false and G_SUP_RANGE = false) generate
        ident_valid_d <= '1';
        filter_result <= '0';
        drop_rtr_frame <= '0';
    end generate;


    filt_sup_gen_true : if (G_SUP_FILTA = true or G_SUP_FILTB = true or
                            G_SUP_FILTC = true or G_SUP_RANGE = true) generate

        drop_rtr_frame <= '1' when (mr_settings_fdrf = DROP_RF_ENABLED
                                    and rec_is_rtr = RTR_FRAME)
                              else
                          '0';

        filter_result <= '0' when (drop_rtr_frame = '1') else
                         '1' when (int_filter_a_valid = '1' or
                                   int_filter_b_valid = '1' or
                                   int_filter_c_valid = '1' or
                                   int_filter_ran_valid = '1')
                             else
                         '0';

        ident_valid_d <=  filter_result when (mr_mode_afm = '1')
                                        else
                                    '1';
    end generate;


    -----------------------------------------------------------------------------------------------
    -- To avoid long combinational paths, valid filter output is pipelined. This is OK since
    -- received frame is valid on input for many clock cycles!
    -----------------------------------------------------------------------------------------------
    valid_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            ident_valid_q <= '0';
        elsif rising_edge(clk_sys) then
            ident_valid_q   <= ident_valid_d;
        end if;
    end process valid_reg_proc;

    -----------------------------------------------------------------------------------------------
    -- Filtering RX Buffer commands
    -----------------------------------------------------------------------------------------------
    store_metadata_f <= '1' when (store_metadata = '1' and ident_valid_q = '1')
                            else
                        '0';

    store_data_f <= '1' when (store_data = '1' and ident_valid_q = '1')
                        else
                    '0';

    rec_valid_f <= '1' when (rec_valid = '1' and ident_valid_q = '1')
                       else
                   '0';

    rec_abort_f <= '1' when (rec_abort = '1' and ident_valid_q = '1')
                       else
                   '0';

end architecture;
