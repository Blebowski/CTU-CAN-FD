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
--  Purpose:
--    Library with "reduce" functions to support generic OR between elements
--    of std_logic_vector. Unary logic operators are not well supported by
--    synthesis tools, thus this workaround is used!
--
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

package reduce_lib is

    ----------------------------------------------------------------------------
    -- Performs OR operation between all elements of vector
    --
    -- Arguments:
    --  period          Period of generated clock in picoseconds.
    -- Returns: OR of all elements of vector
    ----------------------------------------------------------------------------
    function or_reduce(
        constant input         : in    std_logic_vector
    ) return std_logic;

    ----------------------------------------------------------------------------
    -- Performs AND operation between all elements of vector
    --
    -- Arguments:
    --  period          Period of generated clock in picoseconds.
    -- Returns: AND of all elements of vector
    ----------------------------------------------------------------------------
    function and_reduce(
        constant input         : in    std_logic_vector
    ) return std_logic;

end package;




--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Library implementation
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

package body reduce_lib is


    function or_reduce(
        constant input         : in    std_logic_vector
    ) return std_logic is
        variable tmp           :       std_logic := '0';
    begin
        for i in input'range loop
            tmp := tmp or input(i);
        end loop;
        return tmp;
    end function;

    function and_reduce(
        constant input         : in    std_logic_vector
    ) return std_logic is
        variable tmp           :       std_logic := '1';
    begin
        for i in input'range loop
            tmp := tmp and input(i);
        end loop;
        return tmp;
    end function;

end package body;
