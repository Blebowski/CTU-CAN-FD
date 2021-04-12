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

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;

-- Only place where Vunit is used. All functions are wrapped so that TB can run
-- also without Vunit!
library vunit_lib;
context vunit_lib.vunit_context;


package tb_report_pkg is

    type t_log_verbosity is (
        verbosity_debug,
        verbosity_info,
        verbosity_warning,
        verbosity_error
    );
    
    signal global_verbosity : t_log_verbosity := verbosity_info;
    
    procedure set_log_verbosity(
        constant value                : in  t_log_verbosity;
        signal verbosity            : out t_log_verbosity
    );
    
    procedure debug_m(
                 msg         : in string
    );
    
    procedure info_m(
        msg         : in string
    );

    procedure warning_m(
                 msg         : in string
    );
    
    procedure error_m(
                 msg         : in string
    );

    procedure check_m(
                 cond        : in boolean;
                 msg         : in string
    );
    
    procedure check_false_m(
                 cond        : in boolean;
                 msg         : in string
    );

end package;

package body tb_report_pkg is


    procedure set_log_verbosity(
        constant value              : in  t_log_verbosity;
        signal verbosity            : out t_log_verbosity
    ) is
    begin
        show_all(display_handler);
        if value >= verbosity_debug then
            null;
        end if;

        if value >= verbosity_info then
            hide(display_handler, debug);
            null;
        end if;

        if value >= verbosity_warning then
            hide(display_handler, debug);
            hide(display_handler, info);
            hide(display_handler, pass);
            hide(display_handler, trace);
            
        end if;

        if value >= verbosity_error then
            hide(display_handler, debug);
            hide(display_handler, info);
            hide(display_handler, pass);
            hide(display_handler, trace);
            hide(display_handler, warning);
        end if;
        verbosity <= value;
    end procedure;

    
    procedure info_m(
        msg         : in string
    ) is
    begin
        info(msg);
    end procedure;

    
    procedure warning_m(
                 msg         : in string
    ) is
    begin
        warning(msg); 
    end procedure;


    procedure error_m(
                 msg         : in string
    ) is
    begin
        error(msg);
    end procedure;
    
    procedure debug_m(
                 msg         : in string
    ) is
    begin
        debug(msg);
    end procedure;


    procedure check_m(
                 cond        : in boolean;
                 msg         : in string
    ) is
    begin
        check(cond, msg);
    end procedure;
    
    
    procedure check_false_m(
                 cond        : in boolean;
                 msg         : in string
    ) is
    begin
        check_false(cond, msg);
    end procedure;
    

end package body;
