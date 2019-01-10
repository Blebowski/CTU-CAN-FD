--------------------------------------------------------------------------------
-- 
-- Register map generation tool
--
-- Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this SW component and associated documentation files (the "Component"),
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
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Purpose:
--   Generic data multiplexor.
--
--   Simplified diagram:
--
--
--              data_selector
--                    |
--                    v
--             ---------------
--             |        ____ |
--             |       /     |
--             |      /      |<--- Maximal Index
--             | ____/       |
--             |             |
--             ---------------
--                    |
--                    | (saturated index)
--                    V
--                |-------|                                Output register
--            --->|*      |                                  (optional) 
--             .  |  *    |                                   
--             .  |    *  |  sel_data  |-----|               |--------| 
--   data_in   .  |      *|----------->|     |  masked_data  |        | data_out
--             .  |    *  |            | AND |-------------->| D    Q |-------->
--             .  |  *    |      ----->|     |               |        |
--            --->|*      |      |     |-----|               |        |
--                |-------|      |                clk_sys ---|>       |
--                               |                           |        |   
--     data_mask                 |                           |--------|
--   ----------------------------|
--
--
--  Input data are concatenated to one long std_logic_vector to avoid
--  definition of extra array type.
--
--------------------------------------------------------------------------------
-- Revision History:
--     3.11.2018   Created file
--     8.01.2019   Added data saturation upon address overflow.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

entity data_mux is
    generic(

        -- Width of data output
        constant data_out_width        :     natural := 32;

        -- Width of data input. Must be divisible by "data_out_width"
        constant data_in_width         :     natural := 256;

        -- Width of selector signal
        -- (Number of data inputs given as  2**sel_width - 1)
        constant sel_width             :     natural := 8;

        -- Choose betweed registered / non-registered output
        constant registered_out        :     boolean := false;

        -- Reset polarity
        constant reset_polarity        :     std_logic := '0'
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Data selector (unsigned)
        ------------------------------------------------------------------------
        signal data_selector          :in   std_logic_vector(sel_width - 1 downto 0);

        ------------------------------------------------------------------------
        -- Data inputs
        ------------------------------------------------------------------------
        signal data_in                :in   std_logic_vector(data_in_width - 1 downto 0);

        ------------------------------------------------------------------------
        -- Masking signals for data outputs, each bit can be masked out.
        -- Serves for implementation of byte enables.
        -- data_mask(i) = '1' -> i-th bit is propagated to the output
        -- data_mask(i) = '0' -> i-th bit is not propagated to the output.
        ------------------------------------------------------------------------
        signal data_mask_n            :in  std_logic_vector(data_out_width - 1 downto 0);
        
        ------------------------------------------------------------------------
        -- Enables data propagation to the output.
        ------------------------------------------------------------------------
        signal enable                 :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Output, one-hot coded. In logic 1 for each valid address
        ------------------------------------------------------------------------
        signal data_out               :out  std_logic_vector(data_out_width - 1 downto 0)
    );

end entity data_mux;


architecture rtl of data_mux is

    -- Data output from data mux (before masking)
    signal sel_data                    :    std_logic_vector(data_out_width - 1 downto 0);

    -- Data after saturation. Saturated data return all zeroes when address overflow 
    -- and read beyond last address of register block occurs.
    signal saturated_data             :    std_logic_vector(data_out_width - 1 downto 0);

    -- Data output from data mux (after masking and saturation)
    signal masked_data                :    std_logic_vector(data_out_width - 1 downto 0);

    -- Internal data select converted to natural to avoid ugly code in
    -- selection
    signal index                      :    natural;
    
    -- Index saturation
    constant INDEX_MAX                :    natural := (data_in_width / data_out_width - 1);

    -- Saturated value of internal index.
    signal index_sat                  :    natural range 0 to INDEX_MAX;
    
begin

    ---------------------------------------------------------------------------
    -- Conversion to integer
    ---------------------------------------------------------------------------
    index <= to_integer(unsigned(data_selector));


    ---------------------------------------------------------------------------
    -- Data selector saturation, we need to saturate data selector in case
    -- we don't have 2^N inputs. Address conversion of n bit vector to index
    -- of less than 2^N - 1 would result in overflow in simulator. Using 
    -- modulo is not effective, modulo by non 2^N number results in extra
    -- shitty logic...
    ---------------------------------------------------------------------------
    index_sat <= index when (index <= INDEX_MAX) else
                 INDEX_MAX;
    

    ---------------------------------------------------------------------------
    -- Data mux -> Row given by index, column given by generic
    ---------------------------------------------------------------------------
    data_mux_gen : for i in 0 to data_out_width - 1 generate
        sel_data(i) <= data_in(index_sat * data_out_width + i);
    end generate data_mux_gen;


    ---------------------------------------------------------------------------
    -- Data saturation
    ---------------------------------------------------------------------------
    data_saturation_gen : for i in 0 to data_out_width - 1 generate
        saturated_data(i) <= sel_data(i) when (index <= INDEX_MAX)
                                         else
                             '0';
    end generate data_saturation_gen;


    ---------------------------------------------------------------------------
    -- Data masking
    ---------------------------------------------------------------------------
    data_mask_gen : for i in 0 to data_out_width - 1 generate
        masked_data(i) <= saturated_data(i) and data_mask_n(i);
    end generate data_mask_gen;


    ---------------------------------------------------------------------------
    -- Registering / Not-registering output
    ---------------------------------------------------------------------------
    data_mux_reg_true_gen : if (registered_out) generate
        data_mux_reg_proc : process(res_n, clk_sys)
        begin
            if (res_n = reset_polarity) then
                data_out <= (OTHERS => '0');

            elsif (rising_edge(clk_sys)) then
                if (enable = '1') then
                    data_out <= masked_data;
                end if;
            end if;
        end process;
    end generate data_mux_reg_true_gen;

    data_mux_reg_false_gen : if (not registered_out) generate
        data_out <= masked_data when (enable = '1') else
                    (OTHERS => '0');
    end generate data_mux_reg_false_gen;

end architecture;
