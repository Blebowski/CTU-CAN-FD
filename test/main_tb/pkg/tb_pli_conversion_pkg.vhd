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
--    Package for CTU CAN FD compliance testbench. Contains common definitions
--    for whole CTU CAN FD compliance testbench.    
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;


package tb_pli_conversion_pkg is

    ---------------------------------------------------------------------------
    --
    ---------------------------------------------------------------------------
    procedure pli_str_to_logic_vector(
               input       : in   string;
        signal output      : out  std_logic_vector
    );
    
    
    ---------------------------------------------------------------------------
    --
    ---------------------------------------------------------------------------
    procedure pli_logic_vector_to_str(
                input       : in   std_logic_vector;
       variable output      : out  string
    );


    ---------------------------------------------------------------------------
    --
    ---------------------------------------------------------------------------
    procedure pli_time_to_logic_vector(
                 input       : in  time;
        variable output      : out std_logic_vector(63 downto 0)   
    );


    ---------------------------------------------------------------------------
    --
    ---------------------------------------------------------------------------
    procedure pli_logic_vector_to_time(
                 input       : in  std_logic_vector(63 downto 0);
        variable output      : out time
    );

end package;


package body tb_pli_conversion_pkg is


    procedure pli_str_to_logic_vector(
               input       : in   string;
        signal output      : out  std_logic_vector
    ) is
        variable cropped_length : integer := input'length;
    begin
        -- By default null everywhere, no string character
        output(output'length - 1 downto 0) <= (OTHERS => '0');
        
        -- Crop if string is longer
        if (output'length < cropped_length * 8) then
            cropped_length := output'length / 8;
        end if;
        
        -- Convert as if ASCII
        for i in 0 to cropped_length - 1 loop
            output(i * 8 + 7 downto i * 8) <= std_logic_vector(to_unsigned(
                    character'pos(input(input'length - i)), 8));
        end loop;
    end procedure;


    procedure pli_logic_vector_to_str(
                input       : in   std_logic_vector;
       variable output      : out  string
    ) is
        variable cropped_length : integer := input'length / 8;
    begin
        -- By default null everywhere, no string character
        output(1 to output'high) := (OTHERS => ' ');
        
        -- Crop if vector is longer than output string
        if (cropped_length > output'length) then
            cropped_length := output'length;
        end if;
        
        -- Convert as if ASCII
        for i in 0 to cropped_length - 1 loop
            output(cropped_length - i) := character'val(to_integer(unsigned(
                            input(i * 8 + 7 downto i * 8))));
        end loop;    
    end procedure;
    


    procedure pli_time_to_logic_vector(
                 input       : in  time;
        variable output      : out std_logic_vector(63 downto 0)   
    ) is
        variable low       : integer;
        variable high      : integer;
    begin 
        high := input / (integer'high * 1 fs);
        
        -- If input is higher than integer'high, it will overflow automatically
        --  performing needed modulo operation.
        low := input / 1 fs;

        output(30 downto 0) := std_logic_vector(to_unsigned(low, 31));
        output(61 downto 31) := std_logic_vector(to_unsigned(high, 31));
        
        -- Note: Positive integer is up to 2^31 - 1, convert to two integers
        --       and crop highest two bits. This will effectively overflow
        --       all time values above 2^62 * 1 fs, but we don't care!
    end procedure;


    procedure pli_logic_vector_to_time(
                 input       : in  std_logic_vector(63 downto 0);
        variable output      : out time
    ) is
        variable low       : integer;
        variable high      : integer;
    begin
        low := to_integer(unsigned(input(30 downto 0)));
        high := to_integer(unsigned(input(61 downto 31)));

        output := (low * 1 fs) + (high * (integer'high * 1 fs + 1 fs)); 
    end;

end package body;
