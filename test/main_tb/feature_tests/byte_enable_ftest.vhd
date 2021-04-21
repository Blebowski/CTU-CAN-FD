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
--  Byte enable feature test.
--
-- @Verifies:
--  @1. 8/16/32 bit read access.
--  @2. 8/16/32 bit write access.
--
-- @Test sequence:
--  @1. Disable DUT (so that BTR, BTR_FD are writable). Read 32-bit register
--      via 32-bit access. Check correct value.
--  @2. Read 32-bit register via 2 16 bit accesses. Check correct values.
--  @3. Read 32-bit register via 4 8 bit accesses. Check correct values.
--  @4. Erase 32-bit R/W register. Write pattern to a single byte. Read 32-bit
--      check that pattern was written to single byte only and rest are zeroes.
--      Repeat with each byte!
--  @5. Erase 32-bit R/W register. Write pattern to lower half word. Read 32-bit,
--      check that pattern was written to half word only and rest are zeroes.
--      Repeat with upper half word!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    12.7.2018     Created file
--    28.9.2019     Accustomized to common feature test format. 
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package byte_enable_ftest is
    procedure byte_enable_ftest_exec(
        signal      chn             : inout  t_com_channel
	);
end package;

package body byte_enable_ftest is

    procedure byte_enable_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable data_32  : std_logic_vector(31 downto 0) := (OTHERS => '0');
        variable data_16  : std_logic_vector(15 downto 0) := (OTHERS => '0');
        variable data_8   : std_logic_vector(7 downto 0) := (OTHERS => '0');

        variable exp_8    : std_logic_vector(7 downto 0);
        variable exp_16   : std_logic_vector(15 downto 0);
        variable address  : std_logic_vector(11 downto 0) := (OTHERS => '0');
        variable ID       : natural := 1;
    begin

        ------------------------------------------------------------------------
        -- @1. Disable DUT (so that BTR, BTR_FD are writable). Read 32-bit 
        --     register via 32-bit access. Check correct value.
        ------------------------------------------------------------------------
        info_m("Step 1: 32-bit read");

        CAN_turn_controller(false, DUT_NODE, chn);
        address := YOLO_REG_ADR;
        CAN_read(data_32, address, DUT_NODE, chn);
        check_m(data_32 = x"DEADBEEF", "32 bit read error");

        ------------------------------------------------------------------------
        -- @2. Read 32-bit register via 2 16 bit accesses. Check correct values.
        ------------------------------------------------------------------------
        info_m("Step 2: 16-bit read");

        for i in 0 to 1 loop        
            data_16 := (OTHERS => '0');
            address := std_logic_vector(to_unsigned(
		            (to_integer(unsigned(YOLO_REG_ADR)) + i * 2), 12));
            CAN_read(data_16, address, DUT_NODE, chn);

            exp_16 := YOLO_VAL_RSTVAL(16 * i + 15 downto 16 * i);
            if (i = 0) then
                check_m(data_16 = exp_16, "16 bit access - lower half word");
            else
                check_m(data_16 = exp_16, "16 bit access - upper half word");
            end if;
        end loop;

        ------------------------------------------------------------------------
        -- @3. Read 32-bit register via 4 8 bit accesses. Check correct values.
        ------------------------------------------------------------------------
        info_m("Step 3: 8-bit read");

        for i in 0 to 3 loop
            data_8 := (OTHERS => '0');
            address := std_logic_vector(to_unsigned(
		            (to_integer(unsigned(YOLO_REG_ADR)) + i), 12));
            CAN_read(data_8, address, DUT_NODE, chn);

            -- Checking if valid 1 byte matches register value
            exp_8 := YOLO_VAL_RSTVAL(8 * i + 7 downto 8 * i);
            check_m (exp_8 = data_8, "8 bit access, byte: " & integer'image(i));
        end loop;

        ------------------------------------------------------------------------
        -- @4. Erase 32-bit R/W register. Write pattern to a single byte. Read 
        --     32-bit check that pattern was written to single byte only and 
        --     rest are zeroes. Repeat with each byte!
        ------------------------------------------------------------------------
        info_m("Step 4: 8-bit write");

        for i in 0 to 3 loop
            address := BTR_FD_ADR;
            data_32 := (OTHERS => '0');
            CAN_write(data_32, address, DUT_NODE, chn);

            address := std_logic_vector(to_unsigned(
		            (to_integer(unsigned(BTR_FD_ADR)) + i), 12));
            data_8 := x"AA";
            CAN_write(data_8, address, DUT_NODE, chn);
            
            CAN_read(data_32, BTR_FD_ADR, DUT_NODE, chn);

            -- Checking if one written byte was written OK!
            check_m(data_32(8 * i + 7 downto 8 * i) = x"AA",
                    "Write access - 8 bit (valid byte), Index :" & integer'image(i));

            -- Checking if other bytes are 0
            case i is
            when 0 =>
                check_m(data_32(31 downto 8) = x"000000",
                       "Write access - 8 bit (Empty byte 0)");
            when 1 =>
                check_m(data_32(31 downto 16) = x"0000" and
                        data_32(7 downto 0) = x"00",
                        "Write access - 8 bit (Empty byte 1)");
            when 2 =>
                check_m(data_32(31 downto 24) = x"00" and
                        data_32(15 downto 0) = x"0000",
                        "Write access - 8 bit (Empty byte 2)");

            when 3 =>
                check_m(data_32(23 downto 0) = x"000000",
                        "Write access - 8 bit (Empty byte 3)");

            when others =>
                error_m("Invalid byte index"); -- LCOV_EXCL_LINE
            end case;
        end loop;
        
        ------------------------------------------------------------------------
        --  @5. Erase 32-bit R/W register. Write pattern to lower half word. Read
        --      32-bit, check that pattern was written to half word only and rest
        --      are zeroes. Repeat with upper half word!
        ------------------------------------------------------------------------
        info_m("Step 4: 16-bit write");

        for i in 0 to 1 loop
            address := BTR_FD_ADR;
            data_32 := (OTHERS => '0');
            CAN_write(data_32, address, DUT_NODE, chn);

            address := std_logic_vector(to_unsigned(
                        (to_integer(unsigned(BTR_FD_ADR)) + (i * 2)), 12));
            data_16 := x"AAAA";
            CAN_write(data_16, address, DUT_NODE, chn);
            
            data_32 := (OTHERS => '0');
            CAN_read(data_32, BTR_FD_ADR, DUT_NODE, chn);
            
            -- Checking if one written byte was written OK!
            check_m(data_32(16 * i + 15 downto 16 * i) = x"AAAA",
                  "Write access - 16 bit (valid half word), Index :" & integer'image(i));

            -- Checking if other half word is 0
            case i is
            when 0 =>
                check_m(data_32(31 downto 16) = x"0000",
                        "Write access - 16 bit (Empty half word 0)");
            when 1 =>
                check_m(data_32(15 downto 0) = x"0000",
                        "Write access - 16 bit (Empty half word 1)");

            when others =>
                error_m("Invalid half word index"); -- LCOV_EXCL_LINE
            end case;
        end loop;       

    end procedure;

end package body;