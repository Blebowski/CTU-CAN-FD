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
use ctu_can_fd_rtl.id_transfer.all;
use ctu_can_fd_rtl.can_constants.all;
use ctu_can_fd_rtl.can_components.all;
use ctu_can_fd_rtl.can_types.all;
use ctu_can_fd_rtl.cmn_lib.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.reduce_lib.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity frame_filters is
    generic(
        -- Reset polarity
        G_RESET_POLARITY     : std_logic := '0';
        
        -- Support filter A
        G_SUP_FILTA          : boolean := true;
        
        -- Support filter B
        G_SUP_FILTB          : boolean := true;
        
        -- Support filter C
        G_SUP_FILTC          : boolean := true;
        
        -- Support range filter
        G_SUP_RANGE          : boolean := true
    );
    port(
        ------------------------------------------------------------------------
        -- Clock an Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              : in std_logic;
        
        -- Asynchronous reset
        res_n                : in std_logic;

        ------------------------------------------------------------------------
        -- Memory registers interface
        ------------------------------------------------------------------------
        -- Driving Bus
        drv_bus              : in  std_logic_vector(1023 downto 0);

        ------------------------------------------------------------------------
        -- CAN Core interface
        ------------------------------------------------------------------------
        -- Receieved CAN ID
        rec_ident            : in  std_logic_vector(28 downto 0); 

        -- Received CAN ID type (0-Base Format, 1-Extended Format);
        rec_ident_type       : in  std_logic;

        -- Input frame type (0-CAN 2.0, 1- CAN FD) 
        rec_frame_type       : in  std_logic;
        
        -- RX Remote transmission request Flag
        rec_is_rtr           : in  std_logic;

        -- Store Metadata in RX Buffer
        store_metadata       : in  std_logic;

        -- Command to store word of CAN Data
        store_data           : in  std_logic;
        
        -- Received frame valid
        rec_valid            : in  std_logic;
        
        -- Command to abort storing of RX frame (due to Error frame)
        rec_abort            : in  std_logic;

        ------------------------------------------------------------------------
        -- Frame filters output
        ------------------------------------------------------------------------
        -- CAN ID passes the filters
        ident_valid          : out   std_logic;
        
        -- Store Metadata in RX Buffer - Filtered
        store_metadata_f     : out   std_logic;

        -- Command to store word of CAN Data - Filtered
        store_data_f         : out   std_logic;
        
        -- Received frame valid - Filtered
        rec_valid_f          : out   std_logic;
        
        -- Command to abort storing of RX frame (due to Error frame) - Filtered
        rec_abort_f          : out   std_logic
    );
end entity;
  
architecture rtl of frame_filters is

    ----------------------------------------------------------------------------
    -- INTERNAL SIGNAL aliases
    ----------------------------------------------------------------------------
    -- Filter A bit mask
    signal drv_filter_A_mask        :       std_logic_vector(28 downto 0);

    -- Filter A control bits
    signal drv_filter_A_ctrl        :       std_logic_vector(3 downto 0);

    -- Filter A bits
    signal drv_filter_A_bits        :       std_logic_vector(28 downto 0);

    -- Output of filter A is valid (internal signal)
    signal int_filter_A_valid       :       std_logic;


    -- Filter B bit mask
    signal drv_filter_B_mask        :       std_logic_vector(28 downto 0);

    -- Filter B control bits
    signal drv_filter_B_ctrl        :       std_logic_vector(3 downto 0);

    -- Filter B bits
    signal drv_filter_B_bits        :       std_logic_vector(28 downto 0);

    -- Output of filter B is valid (internal signal)
    signal int_filter_B_valid       :       std_logic;


    -- Filter C bit mask
    signal drv_filter_C_mask        :       std_logic_vector(28 downto 0);

    -- Filter C control bits
    signal drv_filter_C_ctrl        :       std_logic_vector(3 downto 0);

    -- Filter C control bits
    signal drv_filter_C_bits        :       std_logic_vector(28 downto 0);

    -- Output of filter C is valid (internal signal)
    signal int_filter_C_valid       :       std_logic;

    -- Range filter control bits
    signal drv_filter_ran_ctrl      :       std_logic_vector(3 downto 0);

    -- Lower range filter trehsold
    signal drv_filter_ran_lo_th     :       std_logic_vector(28 downto 0);

    -- Upper range filter trehsold
    signal drv_filter_ran_hi_th     :       std_logic_vector(28 downto 0);

    -- Output of range filter is valid
    signal int_filter_ran_valid     :       std_logic;


    -- Enable the message filters
    signal drv_filters_ena          :       std_logic;

    -- Frame type on input to be compared with driving signal
    signal int_data_type            :       std_logic_vector(3 downto 0);

    -- Concat of types of data on input
    signal int_data_ctrl            :       std_logic_vector(1 downto 0);

    -- Decimal values of identifiers
    signal id_1_dec                 :       natural;
    signal id_2_dec                 :       natural;

    -- Enable signal for filters
    signal filter_A_enable          :       std_logic;
    signal filter_B_enable          :       std_logic;
    signal filter_C_enable          :       std_logic;
    signal filter_range_enable      :       std_logic; 

    signal filter_result            :       std_logic;

    -- Valid output value
    signal ident_valid_d            :       std_logic;  
    signal ident_valid_q            :       std_logic;  
 
    signal drv_drop_remote_frames   :       std_logic;
 
    signal drop_rtr_frame           :       std_logic;
    
begin

    ---------------------------------------------------------------------------
    -- Driving signal aliases
    ---------------------------------------------------------------------------
    drv_filter_A_mask           <= drv_bus(DRV_FILTER_A_MASK_HIGH downto
                                           DRV_FILTER_A_MASK_LOW);
    drv_filter_A_ctrl           <= drv_bus(DRV_FILTER_A_CTRL_HIGH downto
                                           DRV_FILTER_A_CTRL_LOW);
    drv_filter_A_bits           <= drv_bus(DRV_FILTER_A_BITS_HIGH downto
                                           DRV_FILTER_A_BITS_LOW);
    drv_filter_B_mask           <= drv_bus(DRV_FILTER_B_MASK_HIGH downto
                                           DRV_FILTER_B_MASK_LOW);
    drv_filter_B_ctrl           <= drv_bus(DRV_FILTER_B_CTRL_HIGH downto
                                           DRV_FILTER_B_CTRL_LOW);
    drv_filter_B_bits           <= drv_bus(DRV_FILTER_B_BITS_HIGH downto
                                           DRV_FILTER_B_BITS_LOW);
    drv_filter_C_mask           <= drv_bus(DRV_FILTER_C_MASK_HIGH downto
                                           DRV_FILTER_C_MASK_LOW);
    drv_filter_C_ctrl           <= drv_bus(DRV_FILTER_C_CTRL_HIGH downto
                                           DRV_FILTER_C_CTRL_LOW);
    drv_filter_C_bits           <= drv_bus(DRV_FILTER_C_BITS_HIGH downto
                                           DRV_FILTER_C_BITS_LOW);
    drv_filter_ran_ctrl         <= drv_bus(DRV_FILTER_RAN_CTRL_HIGH downto
                                           DRV_FILTER_RAN_CTRL_LOW);
    drv_filter_ran_lo_th        <= drv_bus(DRV_FILTER_RAN_LO_TH_HIGH downto
                                           DRV_FILTER_RAN_LO_TH_LOW);
    drv_filter_ran_hi_th        <= drv_bus(DRV_FILTER_RAN_HI_TH_HIGH downto
                                           DRV_FILTER_RAN_HI_TH_LOW);
    drv_filters_ena             <= drv_bus(DRV_FILTERS_ENA_INDEX);
    drv_drop_remote_frames      <= drv_bus(DRV_FILTER_DROP_RF_INDEX);

    ---------------------------------------------------------------------------
    -- Decoding Filter enables based on accepted frame types by each filter
    ---------------------------------------------------------------------------

    -- Input frame type internal signal
    int_data_ctrl               <= rec_frame_type & rec_ident_type;
    
    -- Decoder frame_type&ident_type to one-hot 
    with int_data_ctrl select int_data_type <=
        "0001" when "00", --CAN Basic
        "0010" when "01", --CAN Extended
        "0100" when "10", --CAN FD Basic
        "1000" when "11", --CAN Fd Extended
        "0000" when others;

    -- Filter is enabled when at least one Frame type/Identifier type is matching
    -- the configured value
    filter_A_enable <= '1' when ((drv_filter_A_ctrl and int_data_type) /= x"0")
                           else
                       '0';
    filter_B_enable <= '1' when ((drv_filter_B_ctrl and int_data_type) /= x"0")
                           else
                       '0';
    filter_C_enable <= '1' when ((drv_filter_C_ctrl and int_data_type) /= x"0")
                           else
                       '0';
    filter_range_enable <= '1' when ((drv_filter_ran_ctrl and int_data_type) /= x"0")
                           else
                           '0';

    ---------------------------------------------------------------------------
    -- Filter instances
    ---------------------------------------------------------------------------
    bit_filter_A_inst : bit_filter
    generic map(
        G_WIDTH           => 29,
        G_IS_PRESENT      => G_SUP_FILTA
    )
    port map(
        filter_mask     => drv_filter_A_mask,       -- IN
        filter_value    => drv_filter_A_bits,       -- IN
        filter_input    => rec_ident,               -- IN
        enable          => filter_A_enable,         -- IN
        
        valid           => int_filter_A_valid       -- OUT
    );

    bit_filter_B_inst : bit_filter
    generic map(
        G_WIDTH           => 29,
        G_IS_PRESENT      => G_SUP_FILTB
    )
    port map(
        filter_mask     => drv_filter_B_mask,       -- IN
        filter_value    => drv_filter_B_bits,       -- IN
        filter_input    => rec_ident,               -- IN
        enable          => filter_B_enable,         -- IN
        
        valid           => int_filter_B_valid       -- OUT
    );

    bit_filter_C_inst : bit_filter
    generic map(
        G_WIDTH           => 29,
        G_IS_PRESENT      => G_SUP_FILTC
    )
    port map(
        filter_mask     => drv_filter_C_mask,       -- IN
        filter_value    => drv_filter_C_bits,       -- IN
        filter_input    => rec_ident,               -- IN
        enable          => filter_C_enable,         -- IN
        
        valid           => int_filter_C_valid       -- OUT
    );
   
                 
    range_filter_inst : range_filter
    generic map(
        G_WIDTH           => 29,
        G_IS_PRESENT      => G_SUP_RANGE
    )
    port map(
        filter_upp_th   => drv_filter_ran_hi_th,    -- IN
        filter_low_th   => drv_filter_ran_lo_th,    -- IN
        filter_input    => rec_ident,               -- IN
        enable          => filter_range_enable,     -- IN
        
        valid           => int_filter_ran_valid     -- OUT
    );

 
    ---------------------------------------------------------------------------
    -- If no filter is supported then Identifier is always valid, regardless
    -- of 'drv_filters_ena'! If Core is not synthesized,  turning filters on
    -- should not affect the acceptance! Everyhting should be affected!
    ---------------------------------------------------------------------------
    filt_sup_gen_false : if (G_SUP_FILTA = false and G_SUP_FILTB = false and
                             G_SUP_FILTC = false and G_SUP_RANGE = false) generate
        ident_valid_d <= '1';
    end generate;


    filt_sup_gen_true : if (G_SUP_FILTA = true or G_SUP_FILTB = true or
                            G_SUP_FILTC = true or G_SUP_RANGE = true) generate

        drop_rtr_frame <= '1' when (drv_drop_remote_frames = DROP_RF_ENABLED
                                    and rec_is_rtr = RTR_FRAME)
                              else
                          '0';
        
        filter_result <= '0' when (drop_rtr_frame = '1') else
                         '1' when (int_filter_A_valid = '1' or
                                   int_filter_B_valid = '1' or
                                   int_filter_C_valid = '1' or
                                   int_filter_ran_valid = '1')
                             else
                         '0';

        ident_valid_d <=  filter_result when (drv_filters_ena = '1')
                                        else
                                    '1';
    end generate;


    ----------------------------------------------------------------------------
    -- To avoid long combinational paths, valid filter output is pipelined. 
    -- This is OK since received frame is valid on input for many clock cycles!
    ----------------------------------------------------------------------------
    valid_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = G_RESET_POLARITY) then
            ident_valid_q <= '0';
        elsif rising_edge(clk_sys) then
            ident_valid_q   <= ident_valid_d;
        end if;
    end process valid_reg_proc;
    
    ---------------------------------------------------------------------------
    -- Filtering RX Buffer commands
    ---------------------------------------------------------------------------
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
                   
    ident_valid <= ident_valid_q;

end architecture;
