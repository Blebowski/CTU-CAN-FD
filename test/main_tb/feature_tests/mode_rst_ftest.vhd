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
--  @3. Read all Control registers and check they return their reset value.
--  @4. Check if Test registers are present and test all read write registers
--      in test registers
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    28.10.2019   Created file
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Here we don't use "synt" context on purpose so that we don't include
-- CAN_FD_frame_format because there are duplicit definitions of "t_memory_reg"
--------------------------------------------------------------------------------
 
Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;
use ctu_can_fd_rtl.CAN_FD_register_map.all;

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package mode_rst_ftest is
    procedure mode_rst_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
    
    procedure mask_reg_val(
        reg                         : in     t_memory_reg;
        value                       : inout  std_logic_vector
    );
    
    procedure get_reg_rst_val(
        reg                         : in     t_memory_reg;
        result                      : inout  std_logic_vector
    );
end package;


package body mode_rst_ftest is

    ---------------------------------------------------------------------------
    -- Mask register value based on its size and address to align it to 32 bit.
    ---------------------------------------------------------------------------
    procedure mask_reg_val(
        reg                         : in     t_memory_reg;
        value                       : inout  std_logic_vector
    ) is
    begin
        if (reg.size /= 16 and reg.size /= 8 and reg.size /= 32) then
            error_m("Unsupported register size: " & integer'image(reg.size));
        end if;

        check_m(value'length = reg.size, "Valid register size");
        for i in 0 to reg.size - 1 loop
            if (reg.is_implem(i) = '0') then
                value(i) := '0';
            end if;
        end loop;
    end procedure;


    procedure get_reg_rst_val(
        reg                         : in     t_memory_reg;
        result                      : inout  std_logic_vector
    ) is
    begin
        if (reg.size = 8) then
            case reg.address(1 downto 0) is
            when "00" => result := reg.reset_val(7 downto 0);
            when "01" => result := reg.reset_val(15 downto 8);
            when "10" => result := reg.reset_val(23 downto 16);
            when "11" => result := reg.reset_val(31 downto 24);
            when others =>
                error_m("Address undefined: " & to_string(unsigned(reg.address)));
            end case;
        end if;
        
        if (reg.size = 16) then
            case (reg.address(1)) is
            when '0' => result := reg.reset_val(15 downto 0);
            when '1' => result := reg.reset_val(31 downto 16);
            when others =>
                error_m("Address undefined: " & to_string(unsigned(reg.address)));
            end case;
        end if;
        
        if (reg.size = 32) then
            result := reg.reset_val;
        end if;
    end procedure;
    
    
    procedure test_rw_reg(
                 reg       : in    t_memory_reg;
        variable rand_data : inout std_logic_vector;
        variable read_data : inout std_logic_vector;
        signal channel     : inout t_com_channel
    ) is
    begin
        mask_reg_val(reg, rand_data);
        info_m ("Testing RW register at address: " & to_hstring(reg.address) &
                " size: " & integer'image(reg.size));
        if (reg.address = MODE_ADR) then
            rand_data(0) := '0';
        end if;
        CAN_write(rand_data, reg.address, DUT_NODE, channel);
        CAN_read(read_data, reg.address, DUT_NODE, channel);
        mask_reg_val(reg, read_data);
        check_m(read_data = rand_data, "Address: 0x" &
                to_hstring(unsigned(reg.address)) & 
                " RW register written! Expected data: 0x" &
                to_hstring(unsigned(rand_data)) & " Read data: 0x" &
                to_hstring(unsigned(read_data)));
    end procedure;


    procedure mode_rst_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable r_data_32      : std_logic_vector(31 downto 0) := (OTHERS => '0');
        variable r_data_16      : std_logic_vector(15 downto 0) := (OTHERS => '0');
        variable r_data_8       : std_logic_vector(7 downto 0) := (OTHERS => '0');
        
        variable reg_model      : t_Control_registers_list;
        
        variable rand_data_8    : std_logic_vector(7 downto 0) := (OTHERS => '0');
        variable rand_data_16   : std_logic_vector(15 downto 0) := (OTHERS => '0');
        variable rand_data_32   : std_logic_vector(31 downto 0) := (OTHERS => '0');
        
        variable reg_rst_val_32 : std_logic_vector(31 downto 0) := (OTHERS => '0');
        variable reg_rst_val_16 : std_logic_vector(15 downto 0) := (OTHERS => '0');
        variable reg_rst_val_8  : std_logic_vector(7 downto 0) := (OTHERS => '0');

        variable num_txt_bufs   : natural;
        
        variable test_regs_present : boolean;
        
        variable mode           : SW_mode := SW_mode_rst_val;
    begin

        -----------------------------------------------------------------------
        -- @1. Write all RW registers to random value. Check they were written.
        -----------------------------------------------------------------------
        info_m("Step 1");
        for i in 0 to Control_registers_list'length - 1 loop
            if (Control_registers_list(i).reg_type = reg_read_write) then

                rand_logic_vect_v(rand_data_8, 0.5);
                rand_logic_vect_v(rand_data_16, 0.5);
                rand_logic_vect_v(rand_data_32, 0.5);

                ---------------------------------------------------------------
                -- 8 bit register size
                ---------------------------------------------------------------
                if (Control_registers_list(i).size = 8) then
                    test_rw_reg(Control_registers_list(i),
                                rand_data_8, r_data_8, chn);

                ---------------------------------------------------------------
                -- 16 bit register size
                ---------------------------------------------------------------
                elsif (Control_registers_list(i).size = 16) then
                    test_rw_reg(Control_registers_list(i),
                                rand_data_16, r_data_16, chn);

                ---------------------------------------------------------------
                -- 32 bit register size
                ---------------------------------------------------------------                    
                elsif (Control_registers_list(i).size = 32) then
                    test_rw_reg(Control_registers_list(i),
                                rand_data_32, r_data_32, chn);

                else
                    error_m("Unsupported register size: " &
                                integer'image(Control_registers_list(i).size));
                end if;

            end if;
        end loop;

        -----------------------------------------------------------------------
        -- @2. Execute SW reset via MODE[RST].
        -----------------------------------------------------------------------
        info_m("Step 2");
        exec_SW_reset(DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @3. Read all Control registers and check they return their reset
        --     value.
        -----------------------------------------------------------------------
        info_m("Step 3 - Control registers");
        for i in 0 to Control_registers_list'length - 1 loop

            if (Control_registers_list(i).size = 8) then
                CAN_read(r_data_8, Control_registers_list(i).address,
                         DUT_NODE, chn);
                mask_reg_val(Control_registers_list(i), r_data_8);
                
                get_reg_rst_val(Control_registers_list(i), reg_rst_val_8);
                mask_reg_val(Control_registers_list(i), reg_rst_val_8);

                check_m(r_data_8 = reg_rst_val_8, "Address: 0x" &
                to_hstring(unsigned(Control_registers_list(i).address)) & 
                " Reset value! Expected reset value: 0x" &
                    to_hstring(unsigned(reg_rst_val_8)) & " Read data: 0x" &
                    to_hstring(unsigned(r_data_8)));
                    
            elsif (Control_registers_list(i).size = 16) then
                CAN_read(r_data_16, Control_registers_list(i).address,
                         DUT_NODE, chn);
                mask_reg_val(Control_registers_list(i), r_data_16);
                
                get_reg_rst_val(Control_registers_list(i), reg_rst_val_16);
                mask_reg_val(Control_registers_list(i), reg_rst_val_16);
                
                check_m(r_data_16 = reg_rst_val_16, "Address: 0x" &
                to_hstring(unsigned(Control_registers_list(i).address)) & 
                " Reset value! Expected reset value: 0x" &
                    to_hstring(unsigned(reg_rst_val_16)) & " Read data: 0x" &
                    to_hstring(unsigned(r_data_16)));

            elsif (Control_registers_list(i).size = 32) then
                CAN_read(r_data_32, Control_registers_list(i).address,
                         DUT_NODE, chn);
                mask_reg_val(Control_registers_list(i), r_data_32);

                get_reg_rst_val(Control_registers_list(i), reg_rst_val_32);
                mask_reg_val(Control_registers_list(i), reg_rst_val_32);

                -- RX_MEM_INFO register, all bits generic dependant -> Skip!
                if (Control_registers_list(i).address = RX_MEM_INFO_ADR) then
                    next;
                end if;
    
                -- TXTB_INFO is generic dependant -> Get number of TXT Buffers
                if (Control_registers_list(i).address = TX_STATUS_ADR) then
                    get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);
                    reg_rst_val_32 := (OTHERS => '0');
                    
                    -- Each buffer should be "EMPTY"
                    for bi in 0 to num_txt_bufs - 1 loop
                        reg_rst_val_32((bi + 1) * 4 - 1 downto bi * 4) := "1000";
                    end loop;
                end if;

                -- Timestamp High/Low registers reflect current value of external
                -- input -> skip them!
                if (Control_registers_list(i).address = TIMESTAMP_LOW_ADR or
                    Control_registers_list(i).address = TIMESTAMP_HIGH_ADR)
                then
                    next;
                end if;
                
                -- STATUS register -> Mask STCNT, STRGS bits
                if (Control_registers_list(i).address = STATUS_ADR) then
                    r_data_32(STCNT_IND) := '0';
                    r_data_32(STRGS_IND) := '0';
                end if;
                
                check_m(r_data_32 = reg_rst_val_32, "Address: 0x" &
                to_hstring(unsigned(Control_registers_list(i).address)) & 
                " Reset value! Expected reset value: 0x" &
                    to_hstring(unsigned(reg_rst_val_32)) & " Read data: 0x" &
                    to_hstring(unsigned(r_data_32)));
            else
                error_m("Unsupported register size: " &
                            integer'image(Control_registers_list(i).size));
            end if;

        end loop;
        
        -----------------------------------------------------------------------
        -- @4. Check if Test registers are present and test all read write 
        --     registers in test registers.
        -----------------------------------------------------------------------
        info_m("Step 4 - Test Test registers");
        
        CAN_check_test_registers(test_regs_present, DUT_NODE, chn);

        if (test_regs_present) then
            
            -- Enable Test mode since test registers are accessible only in
            -- Test mode
            mode.test := true;
            set_core_mode(mode, DUT_NODE, chn);
            
            for i in 0 to Test_registers_list'length - 1 loop
                if (Test_registers_list(i).reg_type = reg_read_write) then
                    rand_logic_vect_v(rand_data_8, 0.5);
                    rand_logic_vect_v(rand_data_16, 0.5);
                    rand_logic_vect_v(rand_data_32, 0.5);
                    
                    -- Do not write Write strobe. It is auto-clear, so it cant
                    -- be read back!
                    if (Test_registers_list(i).address = TST_CONTROL_ADR) then
                        rand_data_32(TWRSTB_IND) := '0';
                    end if;
                
                    -----------------------------------------------------------
                    -- 8 bit register size
                    -----------------------------------------------------------
                    if (Test_registers_list(i).size = 8) then
                        test_rw_reg(Test_registers_list(i),
                                    rand_data_8, r_data_8, chn);
    
                    -----------------------------------------------------------
                    -- 16 bit register size
                    -----------------------------------------------------------
                    elsif (Test_registers_list(i).size = 16) then
                        test_rw_reg(Test_registers_list(i),
                                    rand_data_16, r_data_16, chn);
    
                    -----------------------------------------------------------
                    -- 32 bit register size
                    -----------------------------------------------------------                    
                    elsif (Test_registers_list(i).size = 32) then
                        test_rw_reg(Test_registers_list(i),
                                    rand_data_32, r_data_32, chn);
    
                    else
                        error_m("Unsupported register size: " &
                                    integer'image(Test_registers_list(i).size));
                    end if;
                end if;
            end loop;
        end if;
        
  end procedure;

end package body;