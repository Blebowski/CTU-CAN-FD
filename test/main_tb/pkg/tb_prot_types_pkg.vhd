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
--    Package with protected types
--
--------------------------------------------------------------------------------
-- Revision History:
--    15.2.2022   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;

package tb_prot_types_pkg is

    -- Redefined on purpose to avoid circular dependency
    constant C_NUM_AGENTS               : natural := 10;

    -----------------------------------------------------------------------
    -- Communication channel data
    --
    -- Shared data structure used as buffer for passing data over
    -- communication channel. It is filled by sender always before issuing
    -- event on communication channel. Since VHDL should hold mutex over
    -- shared variable, it should be safe way to pass data between
    -- processes!
    -----------------------------------------------------------------------
    type t_com_channel_data is protected

        -------------------------------------------------------------------
        --
        -------------------------------------------------------------------
        procedure set_dest_and_msg_code(
            dest        : in natural range 0 to C_NUM_AGENTS;
            msg_code    : in integer
        );

        -------------------------------------------------------------------
        --
        -------------------------------------------------------------------
        impure function get_dest return integer;

        -------------------------------------------------------------------
        --
        -------------------------------------------------------------------
        impure function get_msg_code return integer;

        -------------------------------------------------------------------
        --
        -------------------------------------------------------------------
        procedure set_reply_code(
            reply_code  : in natural
        );

        -------------------------------------------------------------------
        --
        -------------------------------------------------------------------
        impure function get_reply_code return natural;

        -------------------------------------------------------------------
        --
        -------------------------------------------------------------------
        procedure set_param(param : in  std_logic_vector);
        procedure set_param(param : in  std_logic);
        procedure set_param(param : in  time);
        procedure set_param2(param : in  time);
        procedure set_param(param : in  integer);
        procedure set_param(param : in  boolean);
        procedure set_param(param : in  string);


        -------------------------------------------------------------------
        --
        -------------------------------------------------------------------
        impure function get_param return std_logic;
        impure function get_param return std_logic_vector;
        impure function get_param return time;
        impure function get_param2 return time;
        impure function get_param return integer;
        impure function get_param return boolean;
        impure function get_param return string;

    end protected;

    -----------------------------------------------------------------------
    --
    -----------------------------------------------------------------------
    type t_ctu_test_result is protected
        procedure set_result(result : boolean);
        impure function get_result return boolean;
        impure function get_result return std_logic;
    end protected;

    -----------------------------------------------------------------------
    --
    -----------------------------------------------------------------------
    type t_prot_boolean is protected
        procedure set(new_val : boolean);
        impure function get return boolean;
    end protected;

end package;


package body tb_prot_types_pkg is

    type t_com_channel_data is protected body
        variable dest_i             : natural range 0 to C_NUM_AGENTS;
        variable msg_code_i         : integer;

        variable reply_code_i       : integer;

        variable par_logic_vect     : std_logic_vector(127 downto 0);
        variable par_logic          : std_logic;
        variable par_time           : time;
        variable par_time_2         : time;
        variable par_int            : integer;
        variable par_bool           : boolean;
        variable par_string         : string(1 to 100);

        procedure set_dest_and_msg_code(
            dest        : in natural range 0 to C_NUM_AGENTS;
            msg_code    : in integer
        ) is
        begin
            dest_i := dest;
            msg_code_i := msg_code;
        end procedure;


        procedure set_reply_code(
            reply_code  : in natural
        ) is
        begin
            reply_code_i := reply_code;
        end procedure;


        impure function get_reply_code return natural is
        begin
            return reply_code_i;
        end function;


        impure function get_dest return integer is
        begin
            return dest_i;
        end function;


        impure function get_msg_code return integer is
        begin
            return msg_code_i;
        end function;

        procedure set_param(
            param       : in  std_logic
        ) is
        begin
            par_logic := param;
        end procedure;


        procedure set_param(
            param       : in  std_logic_vector
        ) is
        begin
            if (param'length <= 128) then
                par_logic_vect(param'length - 1 downto 0) := param;
            else
                -- Remove to avoid circular dependency
                --warning_m(COM_PKG_TAG & " Truncating passed parameter!!");
                par_logic_vect := param(127 downto 0);
            end if;
        end procedure;


        procedure set_param(
            param       : in  time
        ) is
        begin
            par_time := param;
        end procedure;


        procedure set_param2(
            param       : in  time
        ) is
        begin
            par_time_2 := param;
        end procedure;


        procedure set_param(
            param       : in  integer
        ) is
        begin
            par_int := param;
        end procedure;


        procedure set_param(
            param : in  boolean
        ) is
        begin
            par_bool := param;
        end procedure;


        procedure set_param(
            param : in  string
        ) is
        begin
            par_string := param;
        end procedure;


        impure function get_param return std_logic
        is
        begin
            return par_logic;
        end function;


        impure function get_param return std_logic_vector
        is
        begin
            return par_logic_vect;
        end function;


        impure function get_param return time
        is
        begin
            return par_time;
        end function;


        impure function get_param2 return time
        is
        begin
            return par_time_2;
        end function;


        impure function get_param return integer
        is
        begin
            return par_int;
        end function;

        impure function get_param return string
        is
        begin
            return par_string;
        end function;

        impure function get_param return boolean
        is
        begin
            return par_bool;
        end function;

    end protected body;


    type t_ctu_test_result is protected body

        variable result_i : boolean;

        procedure set_result(result : boolean) is
        begin
            result_i := result;
        end procedure;

        impure function get_result return boolean is
        begin
            return result_i;
        end function;

        impure function get_result return std_logic is
        begin
            if result_i then
                return '1';
            else
                return '0';
            end if;
        end function;

    end protected body;

    -----------------------------------------------------------------------
    --
    -----------------------------------------------------------------------
    type t_prot_boolean is protected body
        variable val : boolean;

        procedure set(new_val : boolean) is
        begin
            val := new_val;
        end procedure;

        impure function get return boolean is
        begin
            return val;
        end function;

    end protected body;

end package body;