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
--  @Purpose:
--    Package with declarations for CAN bus agent accessible over Vunit
--    communication library!
--------------------------------------------------------------------------------
-- Revision History:
--    26.1.2020   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package can_agent_pkg is

    ---------------------------------------------------------------------------
    -- CAN agent component    
    ---------------------------------------------------------------------------
    component can_agent is
    generic(
        G_DRIVER_FIFO_DEPTH : natural := 128;
        G_MONITOR_FIFO_DEPTH : natural := 128
    );
    port (
        -- CAN bus connections
        can_tx   :   in     std_logic;
        can_rx   :   out    std_logic := '1'
    );
    end component;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- CAN agent API    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    constant C_MAX_MSG_LENGTH : natural := 100;    
    constant C_EMPTY_STRING : string(1 to C_MAX_MSG_LENGTH) := (OTHERS => ' ');
    
    -- Driver FIFO entry
    type t_can_driver_entry is record
        value           :   std_logic;          -- Value to be driven
        drive_time      :   time;               -- Time for which to drive
        print_msg       :   boolean;            -- Whether message should be printed
                                                -- when driving of value starts!
        msg             :   string(1 to C_MAX_MSG_LENGTH);   -- Message which to print;
    end record;

    -- Monitor FIFO entry
    type t_can_monitor_entry is record
        value           :   std_logic;          -- Value to be checked
        monitor_time    :   time;               -- Time for which to monitor
        msg             :   string(1 to C_MAX_MSG_LENGTH);   -- Message which to print
    end record;
    
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure can_agent_driver_start(
        signal      net         : inout network_t
    ); 

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure can_agent_driver_stop(
        signal      net         : inout network_t
    ); 

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure can_agent_driver_flush(
        signal      net         : inout network_t
    ); 

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure can_agent_driver_get_progress(
        signal      net         : inout network_t;
        variable    progress    : out   boolean
    );

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure can_agent_driver_get_driven_val(
        signal      net         : inout network_t;
        variable    driven_val  : out   std_logic
    );
    
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure can_agent_driver_push_item(
        signal      net         : inout network_t;
        variable    item        : in    t_can_driver_entry
    );

    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure can_agent_driver_set_wait_timeout(
        signal      net         : inout network_t;
        variable    timeout     : in    time
    );
    
    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure can_agent_driver_wait_finish(
        signal      net         : inout network_t
    );

    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure can_agent_driver_push_value(
        signal      net         : inout network_t;
        constant    value       : in    std_logic;
        constant    time        : in    time
    );
    
    
    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure can_agent_driver_push_value(
        signal      net         : inout network_t;
        constant    value       : in    std_logic;
        constant    time        : in    time;
        constant    msg         : in    string(1 to C_MAX_MSG_LENGTH)
    );
    
    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure can_agent_driver_drive_single_item(
        signal      net         : inout network_t;
        variable    item        : in    t_can_driver_entry
    );
    
    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure can_agent_driver_drive_all_items(
        signal      net         : inout network_t
    );

    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure can_agent_driver_drive_value(
        signal      net         : inout network_t;
        constant    value       : in    std_logic;
        constant    time        : in    time
    );
    
    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure can_agent_driver_drive_value(
        signal      net         : inout network_t;
        constant    value       : in    std_logic;
        constant    time        : in    time;
        constant    msg         : in    string(1 to C_MAX_MSG_LENGTH)
    );

    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Private declarations 
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- Supported commands for clock agent (sent as message types)
    constant CAN_AGNT_CMD_DRIVER_START              : integer := 0;
    constant CAN_AGNT_CMD_DRIVER_STOP               : integer := 1;
    constant CAN_AGNT_CMD_DRIVER_FLUSH              : integer := 2;
    constant CAN_AGNT_CMD_DRIVER_GET_PROGRESS       : integer := 3;
    constant CAN_AGNT_CMD_DRIVER_GET_DRIVEN_VAL     : integer := 4;
    constant CAN_AGNT_CMD_DRIVER_PUSH_ITEM          : integer := 5;
    constant CAN_AGNT_CMD_DRIVER_SET_WAIT_TIMEOUT   : integer := 6;
    constant CAN_AGNT_CMD_DRIVER_WAIT_FINISH        : integer := 7;
    constant CAN_AGNT_CMD_DRIVER_DRIVE_SINGLE_ITEM  : integer := 8;
    constant CAN_AGNT_CMD_DRIVER_DRIVE_ALL_ITEMS    : integer := 9;
   
    constant CAN_AGNT_CMD_REPLY_OK                  : integer := 256;
    constant CAN_AGNT_CMD_REPLY_ERR                 : integer := 257;
    
    -- CAN agent actor
    constant actor_can_agent : actor_t := new_actor("actor_can_agent");

    -- Tag for messages
    constant CAN_AGENT_TAG : string := "CAN Agent: ";

end package;


library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package body can_agent_pkg is

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Clock generator agent API implementation
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    procedure can_agent_driver_start(
        signal      net         : inout network_t
    ) is
        constant can_gen_rec : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(CAN_AGENT_TAG  & "Starting driver");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_START));
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
              CAN_AGENT_TAG & " Driver started");
    end procedure;
    
    
    procedure can_agent_driver_stop(
        signal      net         : inout network_t
    ) is
        constant can_gen_rec : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(CAN_AGENT_TAG & "Stopping driver");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_STOP));
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg);
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
            CAN_AGENT_TAG & "Driver stopped");
    end procedure;


    procedure can_agent_driver_flush(
        signal      net         : inout network_t
    ) is
        constant can_gen_rec : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(CAN_AGENT_TAG & "Flushing driver FIFO");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_FLUSH));
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
              CAN_AGENT_TAG & "CAN agent driver FIFO flushed");
    end procedure;


    procedure can_agent_driver_get_progress(
        signal      net         : inout network_t;
        variable    progress    : out   boolean
    ) is
        constant can_gen_rec : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(CAN_AGENT_TAG & "Getting driver progress");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_GET_PROGRESS));
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        progress := pop(reply_msg);
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
              CAN_AGENT_TAG & "Driver progress got");
    end procedure;


    procedure can_agent_driver_get_driven_val(
        signal      net         : inout network_t;
        variable    driven_val  : out   std_logic
    ) is
        constant can_gen_rec : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(CAN_AGENT_TAG & "Getting driven value");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_GET_DRIVEN_VAL));
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        driven_val := pop(reply_msg);
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
              CAN_AGENT_TAG & "Driven value got");
    end procedure;


    procedure can_agent_driver_push_item(
        signal      net         : inout network_t;
        variable    item        : in    t_can_driver_entry
    ) is
        constant can_gen_rec : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(CAN_AGENT_TAG & "Pushing item into driver FIFO");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_PUSH_ITEM));
        push(req_msg, item.value);
        push(req_msg, item.drive_time);
        push(req_msg, item.print_msg);
        if (item.print_msg) then
            push(req_msg, item.msg);
        end if;
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
              CAN_AGENT_TAG & "Item pushed into driver FIFO");
    end procedure;


    procedure can_agent_driver_set_wait_timeout(
        signal      net         : inout network_t;
        variable    timeout     : in    time
    ) is
        constant can_gen_rec : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(CAN_AGENT_TAG & "Setting wait timeout for driver");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_SET_WAIT_TIMEOUT));
        push(req_msg, timeout);
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
              CAN_AGENT_TAG & "Wait timeout for driver set");
    end procedure;


    procedure can_agent_driver_wait_finish(
        signal      net         : inout network_t
    ) is
        constant can_gen_rec : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(CAN_AGENT_TAG & "Waiting for driver to finish");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_WAIT_FINISH));
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
              CAN_AGENT_TAG & "Driver finished");
    end procedure;


    procedure can_agent_driver_push_value(
        signal      net         : inout network_t;
        constant    value       : in    std_logic;
        constant    time        : in    time
    ) is
        variable item           : t_can_driver_entry;
    begin
        item.value := value;
        item.print_msg := false;
        item.drive_time := time;
        can_agent_driver_push_item(net, item);
    end procedure;


    procedure can_agent_driver_push_value(
        signal      net         : inout network_t;
        constant    value       : in    std_logic;
        constant    time        : in    time;
        constant    msg         : in    string(1 to C_MAX_MSG_LENGTH)
    ) is
        variable item           : t_can_driver_entry;
    begin
        item.value := value;
        item.print_msg := true;
        item.drive_time := time;
        item.msg := msg;
        can_agent_driver_push_item(net, item);
    end procedure;


    procedure can_agent_driver_drive_single_item(
        signal      net         : inout network_t;
        variable    item        : in    t_can_driver_entry
    ) is
        constant can_gen_rec            : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg     : msg_t;
    begin
        info(CAN_AGENT_TAG & "Driving single item");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_DRIVE_SINGLE_ITEM));
        push(req_msg, item.value);
        push(req_msg, item.drive_time);
        push(req_msg, item.print_msg);
        if (item.print_msg) then
            push(req_msg, item.msg);
        end if;
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
              CAN_AGENT_TAG & "Single item driven");
    end procedure;


    procedure can_agent_driver_drive_all_items(
        signal      net         : inout network_t
    ) is
        constant can_gen_rec            : actor_t := find("actor_can_agent");
        variable req_msg, reply_msg     : msg_t;
    begin    
        info(CAN_AGENT_TAG & "Driving all items in driver FIFO");
        req_msg := new_msg(msg_type => (p_code => CAN_AGNT_CMD_DRIVER_DRIVE_ALL_ITEMS));
        send(net         => net,
             receiver    => can_gen_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = CAN_AGNT_CMD_REPLY_OK,
              CAN_AGENT_TAG & "All items driven from driver FIFO driven");
    end procedure;


    procedure can_agent_driver_drive_value(
        signal      net         : inout network_t;
        constant    value       : in    std_logic;
        constant    time        : in    time
    ) is
        variable item                   : t_can_driver_entry;
    begin
        item.value := value;
        item.drive_time := time;
        item.print_msg := false;
        can_agent_driver_drive_single_item(net, item);
    end procedure;


    procedure can_agent_driver_drive_value(
        signal      net         : inout network_t;
        constant    value       : in    std_logic;
        constant    time        : in    time;
        constant    msg         : in    string(1 to C_MAX_MSG_LENGTH)
    ) is
        variable item                   : t_can_driver_entry;
    begin
        item.value := value;
        item.drive_time := time;
        item.print_msg := true;
        item.msg := msg;
        can_agent_driver_drive_single_item(net, item);
    end procedure;

    
end package body;