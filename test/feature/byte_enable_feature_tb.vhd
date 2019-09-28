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
--  Byte enable feature test.
--
-- Verifies:
--  1. 8/16/32 bit read access.
--  2. 8/16/32 bit write access.
--
-- Test sequence:
--  1. Disable Node 1 (so that BTR, BTR_FD are writable). Read 32-bit register
--     via 32-bit access. Check correct value.
--  2. Read 32-bit register via 2 16 bit accesses. Check correct values.
--  3. Read 32-bit register via 4 8 bit accesses. Check correct values.
--  4. Erase 32-bit R/W register. Write pattern to a single byte. Read 32-bit
--     check that pattern was written to single byte only and rest are zeroes.
--     Repeat with each byte!
--  5. Erase 32-bit R/W register. Write pattern to lower half word. Read 32-bit,
--     check that pattern was written to half word only and rest are zeroes.
--     Repeat with upper half word!
--------------------------------------------------------------------------------
-- Revision History:
--    12.7.2018     Created file
--    28.9.2019     Accustomized to common feature test format. 
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package byte_enable_feature is
    procedure byte_enable_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
	);
end package;


package body byte_enable_feature is

    procedure byte_enable_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable data               :        std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable address			:		 std_logic_vector(11 downto 0) :=
                                                (OTHERS => '0');
        variable ID                 :        natural := 1;
        variable errmsg             :        line;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- 1. Disable Node 1 (so that BTR, BTR_FD are writable). Read 32-bit 
        --    register via 32-bit access. Check correct value.
        ------------------------------------------------------------------------
        info("Step 1: 32-bit read");
        CAN_turn_controller(false, ID, mem_bus(1));
        address := YOLO_REG_ADR;
        CAN_read(data, address, ID, mem_bus(1), BIT_32);
        check(data = x"DEADBEEF", "32 bit read error");

        ------------------------------------------------------------------------
        -- 2. Read 32-bit register via 2 16 bit accesses. Check correct values.
        ------------------------------------------------------------------------
        info("Step 2: 16-bit read");
        for i in 0 to 1 loop        
            data := (OTHERS => '0');
            address := std_logic_vector(to_unsigned(
		            (to_integer(unsigned(YOLO_REG_ADR)) + i * 2), 12));
            CAN_read(data, address, ID, mem_bus(1), BIT_16);

            -- Checking if valid 2 bytes match register value
            check(data(16 * i + 15 downto 16 * i) = 
	              YOLO_VAL_RSTVAL(16 * i + 15 downto 16 * i),
	              "Read error - 16 bit access (valid byte), Index:");

            -- Checking invalid 2 bytes are 0
            check(data(16 * (1 - i) + 15 downto 16 * (1 - i)) = x"0000",
                 "Read error -16 bit access (empty byte), Index :");
        end loop;

        ------------------------------------------------------------------------
        -- 3. Read 32-bit register via 4 8 bit accesses. Check correct values.
        ------------------------------------------------------------------------
        info("Step 3: 8-bit read");
        for i in 0 to 3 loop        
            data := (OTHERS => '0');
            address := std_logic_vector(to_unsigned(
		            (to_integer(unsigned(YOLO_REG_ADR)) + i), 12));
            CAN_read(data, address, ID, mem_bus(1), BIT_8);

            -- Checking if valid 1 byte matches register value
            check(data(8 * i + 7 downto 8 * i) = 
	              YOLO_VAL_RSTVAL(8 * i + 7 downto 8 * i),
	              "Read error - 8 bit access (valid byte), Index :");

            -- Checking if other bytes are 0
            case i is
            when 0 =>
                check(data(31 downto 8) = x"000000",
                      "Read error - 8 bit (Empty byte 0)");
            when 1 =>
                check(data(31 downto 16) = x"0000" and
                      data(7 downto 0) = x"00",
                      "Read error - 8 bit (Empty byte 1)");
            when 2 =>
                check(data(31 downto 24) = x"00" and
                      data(15 downto 0) = x"0000",
                      "Read error - 8 bit (Empty byte 2)");
            when 3 =>
                check(data(23 downto 0) = x"000000",
                "Read error - 8 bit (Empty byte 3)");
                
            when others =>
                error("Invalid byte index"); -- LCOV_EXCL_LINE
            end case;
        end loop;

        ------------------------------------------------------------------------
        -- 4. Erase 32-bit R/W register. Write pattern to a single byte. Read 
        --    32-bit check that pattern was written to single byte only and 
        --    rest are zeroes. Repeat with each byte!
        ------------------------------------------------------------------------
        info("Step 4: 8-bit write");
        for i in 0 to 3 loop
            address := BTR_FD_ADR;
            data := (OTHERS => '0');
            CAN_write(data, address, ID, mem_bus(1));

            address := std_logic_vector(to_unsigned(
		            (to_integer(unsigned(BTR_FD_ADR)) + i), 12));
            data(8 * i + 7 downto 8 * i) := x"AA";
            CAN_write(data, address, ID, mem_bus(1), BIT_8);
            data := (OTHERS => '0');

            CAN_read(data, BTR_FD_ADR, ID, mem_bus(1), BIT_32);
            
            -- Checking if one written byte was written OK!
            check(data(8 * i + 7 downto 8 * i) = x"AA",
                  "Write access - 8 bit (valid byte), Index :" & integer'image(i));

            -- Checking if other bytes are 0
            case i is
            when 0 =>
                check(data(31 downto 8) = x"000000",
                     "Write access - 8 bit (Empty byte 0)");
            when 1 =>
                check(data(31 downto 16) = x"0000" and
                      data(7 downto 0) = x"00",
                      "Write access - 8 bit (Empty byte 1)");
            when 2 =>
                check(data(31 downto 24) = x"00" and
                      data(15 downto 0) = x"0000",
                      "Write access - 8 bit (Empty byte 2)");

            when 3 =>
                check(data(23 downto 0) = x"000000",
                      "Write access - 8 bit (Empty byte 3)");
                      
            when others =>
                error("Invalid byte index"); -- LCOV_EXCL_LINE
            end case;
        end loop;
        
        ------------------------------------------------------------------------
        --  5. Erase 32-bit R/W register. Write pattern to lower half word. Read
        --     32-bit, check that pattern was written to half word only and rest
        --     are zeroes. Repeat with upper half word!
        ------------------------------------------------------------------------
        info("Step 4: 16-bit write");
        for i in 0 to 1 loop
            address := BTR_FD_ADR;
            data := (OTHERS => '0');
            CAN_write(data, address, ID, mem_bus(1));

            address := std_logic_vector(to_unsigned(
                        (to_integer(unsigned(BTR_FD_ADR)) + (i * 2)), 12));
            data(16 * i + 15 downto 16 * i) := x"AAAA";
            CAN_write(data, address, ID, mem_bus(1), BIT_16);
            data := (OTHERS => '0');

            CAN_read(data, BTR_FD_ADR, ID, mem_bus(1), BIT_32);
            
            -- Checking if one written byte was written OK!
            check(data(16 * i + 15 downto 16 * i) = x"AAAA",
                  "Write access - 16 bit (valid half word), Index :" & integer'image(i));

            -- Checking if other half word is 0
            case i is
            when 0 =>
                check(data(31 downto 16) = x"0000",
                     "Write access - 16 bit (Empty half word 0)");
            when 1 =>
                check(data(15 downto 0) = x"0000",
                      "Write access - 16 bit (Empty half word 1)");

            when others =>
                error("Invalid half word index"); -- LCOV_EXCL_LINE
            end case;
        end loop;       

    end procedure;

end package body;