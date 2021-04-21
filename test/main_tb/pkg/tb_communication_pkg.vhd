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
--    Package with CTU CAN FD TB communication routines between agents.
--
--------------------------------------------------------------------------------
-- Revision History:
--    28.2.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;

use ctu_can_fd_tb.tb_report_pkg.all;


package tb_communication_pkg is

    constant C_NUM_AGENTS               : natural := 10;
    
    constant C_RESET_AGENT_ID           : natural := 0;
    constant C_CLOCK_AGENT_ID           : natural := 1; 
    constant C_MEM_BUS_AGENT_ID         : natural := 2;
    constant C_CAN_AGENT_ID             : natural := 3;
    constant C_FEATURE_TEST_AGENT_ID    : natural := 4;
    constant C_INTERRUPT_AGENT_ID       : natural := 5;
    constant C_TIMESTAMP_AGENT_ID       : natural := 6;
    constant C_TEST_PROBE_AGENT_ID      : natural := 7;

    constant COM_PKG_TAG : string := "Communication PKG: ";

    -----------------------------------------------------------------------
    -- Communcation channel
    -- Channel signals that there is message hanging in the channel data
    -- 
    -- Communication principle is similar to Vunits COM library!  
    -----------------------------------------------------------------------
    subtype t_com_channel is std_logic;
    
    constant C_COM_CHANNEL_ACTIVE : t_com_channel := '1';
    constant C_COM_CHANNEL_INACTIVE : t_com_channel := 'Z';

    signal default_channel : t_com_channel := C_COM_CHANNEL_INACTIVE;

    -- Reply codes
    constant C_REPLY_CODE_OK : natural := 0;
    constant C_REPLY_CODE_ERR : natural := 1;

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
    
    shared variable com_channel_data : t_com_channel_data;
    
    
    -----------------------------------------------------------------------
    -- Sends request on a channel to an agent
    --
    -- @param channel   Channel on which to send the request
    -- @param dest      Target agent
    -- @param msg_code  Message code to send
    -----------------------------------------------------------------------
    procedure send(
        signal   channel    : inout t_com_channel;
        constant dest       : in    integer range 0 to C_NUM_AGENTS;
        constant msg_code   : in    integer
    );


    -----------------------------------------------------------------------
    -- Start receiving (use by agent). Exits when message is sent on
    -- the channel to agent "dest".
    --
    -- @param channel   Channel on which to send the request
    -- @param dest      Agents destination (unique per agent)
    -----------------------------------------------------------------------
    procedure receive_start(
        signal   channel     : inout  t_com_channel;
        constant dest        : in     integer range 0 to C_NUM_AGENTS
    );
    

    -----------------------------------------------------------------------
    -- Finishes receiving (use by agent). Sets reply code which is then
    -- checked by send.
    --
    -- @param channel       Channel on which to send the request
    -- @param reply_code    Reply code to set.
    -----------------------------------------------------------------------
    procedure receive_finish(
        signal   channel     : inout  t_com_channel;
        constant reply_code  : in     natural
    );

end package;


package body tb_communication_pkg is


    type t_com_channel_data is protected
    body
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
                warning_m(COM_PKG_TAG & " Truncating passed parameter!!");
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


    procedure notify(
        signal channel      : inout t_com_channel
    ) is
    begin
        if channel /= C_COM_CHANNEL_ACTIVE then
            channel <= C_COM_CHANNEL_ACTIVE;
            wait until channel = C_COM_CHANNEL_ACTIVE;
            channel <= C_COM_CHANNEL_INACTIVE;
            wait until channel = C_COM_CHANNEL_INACTIVE;
        else
            error_m(COM_PKG_TAG & "Attempting to notify over active channel!");
        end if;
    end procedure;


    procedure send(
        signal   channel    : inout t_com_channel;
        constant dest       : in    integer range 0 to C_NUM_AGENTS;
        constant msg_code   : in    integer
    ) is
    begin
        com_channel_data.set_dest_and_msg_code(dest, msg_code);
        wait for 0 ns;

        -- Send over the channel
        notify(channel);
        
        -- Wait for response back. Agents should satisfy that only one agent
        -- will process sent message (thanks to dest), and therefore we
        -- are guaranteed to get ACK only from one agent back.
        wait until channel = C_COM_CHANNEL_ACTIVE;
        
        -- Check reply code
        if com_channel_data.get_reply_code /= C_REPLY_CODE_OK then
            error_m(COM_PKG_TAG & "Reply code error from " & integer'image(dest));
        end if;
        
        wait until channel = C_COM_CHANNEL_INACTIVE;
        
    end procedure;


    procedure receive_start(
        signal   channel     : inout  t_com_channel;
        constant dest        : in     integer range 0 to C_NUM_AGENTS
    ) is
    begin
        -- Poll till there is request on the channel
        while true loop
            wait until channel = C_COM_CHANNEL_ACTIVE;
            if (com_channel_data.get_dest = dest) then
                exit;
            end if;
        end loop;
    end procedure;


    procedure receive_finish(
        signal   channel     : inout  t_com_channel;
        constant reply_code  : in     natural
    ) is
    begin
        com_channel_data.set_reply_code(reply_code);
        wait for 0 ns;
        notify(channel);
    end procedure;
    

end package body;
