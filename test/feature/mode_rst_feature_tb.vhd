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
-- @TestInfoStart
--
-- @Purpose:
--  Soft reset feature test.
--
-- @Verifies:
--  @1. Reset value of all memory registers.
--  @2. MODE[RST] will reset all the memory registers.
--
-- @Test sequence:
--  @1. Write all RW registers to random value. Check they were written.
--  @2. Execute SW reset via MODE[RST].
--  @3. Read all registers and check they return their reset value.
--  @4. Write all RW registers to random value. Check they were written.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    28.10.2019   Created file
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Here we don't use "synt" context on purpose so that we don't include
-- CAN_FD_frame_format because there are duplicit definitions of "t_memory_reg"
-- TODO: This will be fixed later on!
--------------------------------------------------------------------------------
 
Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library lib;
use lib.id_transfer.all;
use lib.can_constants.all;
use lib.can_components.all;
use lib.can_types.all;
use lib.cmn_lib.all;
use lib.drv_stat_pkg.all;
use lib.reduce_lib.all;
use lib.can_config.all;
use lib.tb_reg_map_defs_pkg.All;
use lib.can_fd_tb_register_map.All;

use lib.CAN_FD_register_map.all;

context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package mode_rst_feature is
    procedure mode_rst_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
    
    procedure mask_reg_val(
        reg                         : in     t_memory_reg;
        value                       : inout  std_logic_vector(31 downto 0)
    );
end package;


package body mode_rst_feature is

    ---------------------------------------------------------------------------
    -- Mask register value based on its size and address to align it to 32 bit.
    ---------------------------------------------------------------------------
    procedure mask_reg_val(
        reg                         : in     t_memory_reg;
        value                       : inout  std_logic_vector(31 downto 0)
    )is
    begin
        if (reg.size /= 16 and reg.size /= 8 and reg.size /= 32) then
            error("Unsupported register size: " & integer'image(reg.size));
        end if;

        if (reg.size = 8) then
            case reg.address(1 downto 0) is
            when "00" => value := "000000000000000000000000" & value(7 downto 0);
            when "01" => value := "0000000000000000" & value(15 downto 8) & "00000000";
            when "10" => value := "00000000" & value(23 downto 16) & "0000000000000000";
            when "11" => value := value(31 downto 24) & "000000000000000000000000";
            when others =>
                error("Address undefined: " & to_string(unsigned(reg.address)));
            end case;
        end if;
        
        if (reg.size = 16) then
            case (reg.address(1)) is
            when '0' => value := "0000000000000000" & value(15 downto 0);
            when '1' => value := value(31 downto 16) & "0000000000000000";
            when others =>
                error("Address undefined: " & to_string(unsigned(reg.address)));
            end case;
        end if;
        
        -- Mask un-implemented bits!
        for i in 0 to 31 loop
            if (reg.is_implem(i) = '0') then
                value(i) := '0';
            end if;
        end loop; 
    
    end procedure;


    procedure mode_rst_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                               (OTHERS => '0');
        
        variable reg_model          :     t_Control_registers_list;
        
        variable rand_data          :     std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable reg_rst_val        :     std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');

        variable access_size        :     aval_access_size := BIT_32;

    begin

        -----------------------------------------------------------------------
        -- @1. Write all RW registers to random value. Check they were written.
        -----------------------------------------------------------------------
        info("Step 1");
        for i in 0 to Control_registers_list'length - 1 loop
            if (Control_registers_list(i).reg_type = reg_read_write) then

                rand_logic_vect_v(rand_ctr, rand_data, 0.5);
                mask_reg_val(Control_registers_list(i), rand_data);
                
                -- Don't issue Soft reset under our own hands! Keep test mode!
                if (Control_registers_list(i).address = MODE_ADR) then
                    rand_data(RST_IND) := '0';
                    rand_data(TSTM_IND) := '1';
                end if;
                
                -- Keep what was written in models reset value attribute!    
                reg_model(i).reset_val := rand_data;

                if (Control_registers_list(i).size = 8) then 
                    access_size := BIT_8;
                elsif (Control_registers_list(i).size = 16) then
                    access_size := BIT_16;
                elsif (Control_registers_list(i).size = 32) then
                    access_size := BIT_32;
                end if;

                -- Write masked random value
                CAN_write(rand_data, Control_registers_list(i).address, ID_1,
                            mem_bus(1), access_size);
        
                -- Read the value back and compare
                CAN_read(r_data, Control_registers_list(i).address, ID_1,
                            mem_bus(1), access_size);

                mask_reg_val(Control_registers_list(i), r_data);

                check(r_data = rand_data, "Address: 0x" &
                    to_hstring(unsigned(Control_registers_list(i).address)) & 
                    " RW register written! Expected data: 0x" &
                    to_hstring(unsigned(rand_data)) & " Read data: 0x" &
                    to_hstring(unsigned(r_data)));
            end if;
        end loop;

        -----------------------------------------------------------------------
        -- @2. Execute SW reset via MODE[RST].
        -----------------------------------------------------------------------
        info("Step 2");
        exec_SW_reset(ID_1, mem_bus(1));

        -----------------------------------------------------------------------
        -- @3. Read all registers and check they return their reset value.
        -----------------------------------------------------------------------
        info("Step 3");
        for i in 0 to Control_registers_list'length - 1 loop

            if (Control_registers_list(i).size = 8) then 
                access_size := BIT_8;
            elsif (Control_registers_list(i).size = 16) then
                access_size := BIT_16;
            elsif (Control_registers_list(i).size = 32) then
                access_size := BIT_32;
            end if;

            CAN_read(r_data, Control_registers_list(i).address, ID_1,
                        mem_bus(1), access_size);
            mask_reg_val(Control_registers_list(i), r_data);
            
            reg_rst_val := Control_registers_list(i).reset_val;
            mask_reg_val(Control_registers_list(i), reg_rst_val);

            -- RX_MEM_INFO register is generic dependant -> Do it manually!
            if (Control_registers_list(i).address = x"060")then
                reg_rst_val := (OTHERS => '0');
                reg_rst_val(6) := '1';  -- RX Buffer size = 32
                reg_rst_val(22) := '1'; -- RX Mem free = 32
            end if;

            -- Timestamp High/Low registers reflect current value of external
            -- input -> skip them!
            if (Control_registers_list(i).address = x"094" or
                Control_registers_list(i).address = x"098")
            then
                next;
            end if;

            check(r_data = reg_rst_val, "Address: 0x" &
                to_hstring(unsigned(Control_registers_list(i).address)) & 
                " Reset value! Expected reset value: 0x" &
                    to_hstring(unsigned(reg_rst_val)) & " Read data: 0x" &
                    to_hstring(unsigned(r_data)));
        end loop;

        wait for 100 ns;

  end procedure;

end package body;