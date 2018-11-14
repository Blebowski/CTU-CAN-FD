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
--  Message filter for recieved data. Combinational circuit with valid data re-
--  gister at output of circuit. Filter identifier type, frame type are contro-
--  lled by drv_bus from registers. 13 bit and 29 bit filters can be compared.
--  If 13 bit filters are compared then MSB 16 bits in rec_ident_in has to be 
--  zeros. Also mask for the filter in case of 13-bit filter HAS to have 16 
--  uppest bits equal to zero! It is set by drv_bus signals from control regis-
--  ters. Filters  A,B,C,D are present. If input identifier matches at least one
--  it is considered as valid. Frame type (CAN Basic, CAN Extended, CAN FD Basic)
--  are also selectable for filtering.
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--    17.1.2016   Added ID change from register value to decimal value for range
--                filter comparison
--    1.6.2016    Fixed wrong enable decoding from driving bus signals! Filters
--                were disabled but
--                frame was anyway propagated to the output!
--------------------------------------------------------------------------------

Library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.ID_transfer.all;
use work.CANcomponents.ALL;
use work.CAN_FD_frame_format.all;

entity messageFilter is
    generic(
        -- Optional synthesis of received message filters
        -- By default the behaviour is as if all the filters are present
        constant sup_filtA          : boolean := true;
        constant sup_filtB          : boolean := true;
        constant sup_filtC          : boolean := true;
        constant sup_range          : boolean := true
    );
    port(
        ------------------------------------------------------------------------
        -- Clock an reset signals
        ------------------------------------------------------------------------
        signal clk_sys              : in std_logic; 	--System clock
        signal res_n                : in std_logic;  --Async reset


        ------------------------------------------------------------------------
        -- Driving signals from CAN Core
        ------------------------------------------------------------------------

        -- Receieved identifier
        signal rec_ident_in         : in  std_logic_vector(28 downto 0); 

        -- Input message identifier type
        -- (0-BASE Format, 1-Extended Format);
        signal ident_type           : in  std_logic;

        -- Input frame type (0-Normal CAN, 1- CAN FD) 
        signal frame_type           : in  std_logic;

        -- Identifier valid (active log 1)
        signal rec_ident_valid      : in  std_logic;

        -- Driving bus from registers
        signal drv_bus              : in  std_logic_vector(1023 downto 0);

        ------------------------------------------------------------------------
        -- Signal whenever identifier matches the filter identifiers
        ------------------------------------------------------------------------
        signal out_ident_valid      : out   std_logic
    );

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

    -- At least one filter output is valid
    signal min_one_filt_valid       :       std_logic;

    ----------------------------------------------------------------------------
    -- REGISTERS
    ----------------------------------------------------------------------------

    -- Register for valid output value
    signal valid_reg                :       std_logic;  
 
end entity;
  
architecture rtl of messageFilter is
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
    

    ---------------------------------------------------------------------------
    -- Decoding Filter enables based on accepted frame types by each filter
    ---------------------------------------------------------------------------

    -- Input frame type internal signal
    int_data_ctrl               <= frame_type & ident_type;
    
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
    filt_A_comp : bitFilter
    generic map(
        width           => 29,
        is_present      => sup_filtA
    )
    port map(
        filter_mask     => drv_filter_A_mask,
        filter_value    => drv_filter_A_bits,
        filter_input    => rec_ident_in,
        enable          => filter_A_enable,
        valid           => int_filter_A_valid
    );

    filt_B_comp : bitFilter
    generic map(
        width           => 29,
        is_present      => sup_filtB
    )
    port map(
        filter_mask     => drv_filter_B_mask,
        filter_value    => drv_filter_B_bits,
        filter_input    => rec_ident_in,
        enable          => filter_B_enable,
        valid           => int_filter_B_valid
    );

    filt_C_comp : bitFilter
    generic map(
        width           => 29,
        is_present      => sup_filtC
    )
    port map(
        filter_mask     => drv_filter_C_mask,
        filter_value    => drv_filter_C_bits,
        filter_input    => rec_ident_in,
        enable          => filter_C_enable,
        valid           => int_filter_C_valid
    );
   
                 
    filt_range_comp : rangeFilter
    generic map(
        width           => 29,
        is_present      => sup_range
    )
    port map(
        filter_upp_th   => drv_filter_ran_hi_th,
        filter_low_th   => drv_filter_ran_lo_th,
        filter_input    => rec_ident_in,
        enable          => filter_range_enable,
        valid           => int_filter_ran_valid
    );

 
    ---------------------------------------------------------------------------
    -- If no filter is supported then output should be determined just by
    -- input, regardless of 'drv_filters_ena'! If Core is not synthesized, 
    -- turning filters on should not affect the acceptance! Everyhting should
    -- be affected!
    ---------------------------------------------------------------------------
    filt_sup_gen_false : if (sup_filtA = false and sup_filtB = false and
                             sup_filtC = false and sup_range = false) generate
        valid_reg           <= rec_ident_valid;
    end generate;


    filt_sup_gen_true : if (sup_filtA = true or sup_filtB = true or
                            sup_filtC = true or sup_range = true) generate

        min_one_filt_valid <= int_filter_A_valid   OR
                              int_filter_B_valid   OR
                              int_filter_C_valid   OR
                              int_filter_ran_valid;

        -- If received message is valid and at least one of the filters is  
        -- matching the message passed the filter.
        valid_reg             <=  (rec_ident_valid AND min_one_filt_valid)
                                  when (drv_filters_ena = '1')
                                  else rec_ident_valid;
    end generate;


    ----------------------------------------------------------------------------
    -- To avoid long combinational paths, valid filter output is pipelined. 
    -- This is OK since received frame is valid on input for many clock cycles!
    ----------------------------------------------------------------------------
    valid_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            out_ident_valid   <= '0';
        elsif rising_edge(clk_sys) then
            out_ident_valid   <= valid_reg;
        end if;
    end process valid_reg_proc;

  
end architecture;
