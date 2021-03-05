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
--  @Purpose:
--    Package with reporting routines.
--
--------------------------------------------------------------------------------
-- Revision History:
--    28.2.2021   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

package tb_report_pkg is

    type t_log_verbosity is (
        verbosity_debug,
        verbosity_info,
        verbosity_warning,
        verbosity_error
    );
    
    signal global_verbosity : t_log_verbosity := verbosity_info;
    
    procedure debug(
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string
    );
    
    procedure info(
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string
    );

    procedure warning(
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string
    );
    
    procedure error(
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string
    );

    procedure check(
                 cond        : in boolean;
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string  
    );
    
    procedure check_false(
                 cond        : in boolean;
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string  
    );

end package;

package body tb_report_pkg is
    
    procedure info(
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string
    ) is
    begin
        if (global_verbosity = verbosity_debug or
            global_verbosity = verbosity_info)
        then
            report "INFO:" & msg severity note;
        end if;
    end procedure;

    
    procedure warning(
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string
    ) is
    begin
        if (global_verbosity = verbosity_debug or
            global_verbosity = verbosity_info or
            global_verbosity = verbosity_warning)
        then
            report "WARNING: " & msg severity warning;
        end if;    
    end procedure;


    procedure error(
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string
    ) is
    begin
        if ((global_verbosity = verbosity_debug or
            global_verbosity = verbosity_info or
            global_verbosity = verbosity_warning or
            global_verbosity = verbosity_error))
        then
            report "ERROR: " & msg severity error;
        end if;
    end procedure;
    
    procedure debug(
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string
    ) is
    begin
        if (global_verbosity = verbosity_debug) then
            report "DEBUG: " & msg severity note;
        end if;
    end procedure;


    procedure check(
                 cond        : in boolean;
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string    
    ) is
    begin
        if (cond) then
            report "PASS: " & msg;
        else
            report "FAIL: " & msg severity error;
        end if;
    end procedure;
    
    
    procedure check_false(
                 cond        : in boolean;
                 msg         : in string;
        constant line_num    : in natural;
        constant file_name   : in string  
    ) is
    begin
        if (not cond) then
            report "PASS: " & msg;
        else
            report "FAIL: " & msg severity error;
        end if;
    end procedure;
    

end package body;
