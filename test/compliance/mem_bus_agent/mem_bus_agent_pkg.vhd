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
--    Package with declarations for Memory bus agent accessible over Vunit
--    communication library!
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package mem_bus_agent_pkg is

    ---------------------------------------------------------------------------
    -- Memory bus agent component    
    ---------------------------------------------------------------------------
    component mem_bus_agent is
    generic(
        G_ACCESS_FIFO_DEPTH  : natural := 32
    );
    port (
        clk             : in    std_logic;
        scs             : out   std_logic := '0';
        swr             : out   std_logic := 'X';
        srd             : out   std_logic := 'X';
        sbe             : out   std_logic_vector(3 downto 0) := "XXXX";
        write_data      : out   std_logic_vector(31 downto 0) := (others => '0');
        read_data       : in    std_logic_vector(31 downto 0);
        address         : out   std_logic_vector(15 downto 0) := (others => '0')
    );
    end component;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Memory bus agent API    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    type t_mem_bus_access_item is record
        write           :   boolean;
        address         :   natural range 0 to 2 ** 16 - 1;
        byte_enable     :   std_logic_vector(3 downto 0);
        write_data      :   std_logic_vector(31 downto 0);
        read_data       :   std_logic_vector(31 downto 0);
    end record;

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_start(
        signal      net         : inout network_t
    );
    
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_stop(
        signal      net         : inout network_t
    );
    
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_write_non_blocking(
        signal      net         : inout network_t;
                    address     : in    integer;
                    write_data  : in    std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    );

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_write_blocking(
        signal      net         : inout network_t;
                    address     : in    integer;
                    write_data  : in    std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    );

    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_x_mode_start(
        signal      net         : inout network_t
    );

    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_x_mode_stop(
        signal      net         : inout network_t
    );

    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_set_x_mode_setup(
        signal      net         : inout network_t;
                    setup       : in    time
    );

    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_set_x_mode_hold(
        signal      net         : inout network_t;
                    hold        : in    time
    );

    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_set_period(
        signal      net        : inout network_t;
                    period     : in    time
    );

    ---------------------------------------------------------------------------
    -- TODO!
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_set_output_delay(
        signal      net        : inout network_t;
                    out_delay  : in    time
    );

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_read(
        signal      net         : inout network_t;
                    address     : in    integer;
        variable    read_data   : out   std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    );

    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_write(
        signal      net         : inout network_t;
                    address     : in    integer;
                    write_data  : in    std_logic_vector;
                    blocking    : in    boolean := true
    );
    
    ---------------------------------------------------------------------------
    -- TODO!    
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_read(
        signal      net         : inout network_t;
                    address     : in    integer;
        variable    read_data   : out   std_logic_vector
    );
   
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Private declarations 
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- Supported commands for clock agent (sent as message types)
    constant MEM_BUS_AGNT_CMD_START                 : integer := 0;
    constant MEM_BUS_AGNT_CMD_STOP                  : integer := 1;
    constant MEM_BUS_AGNT_CMD_WRITE_NON_BLOCKING    : integer := 2;
    constant MEM_BUS_AGNT_CMD_WRITE_BLOCKING        : integer := 3;
    constant MEM_BUS_AGNT_CMD_READ                  : integer := 4;

    constant MEM_BUS_AGNT_CMD_X_MODE_START          : integer := 5;
    constant MEM_BUS_AGNT_CMD_X_MODE_STOP           : integer := 6;
    constant MEM_BUS_AGNT_CMD_SET_X_MODE_SETUP      : integer := 7;
    constant MEM_BUS_AGNT_CMD_SET_X_MODE_HOLD       : integer := 8;

    constant MEM_BUS_AGNT_CMD_SET_PERIOD            : integer := 9;
    constant MEM_BUS_AGNT_CMD_SET_OUTPUT_DELAY      : integer := 10;
    constant MEM_BUS_AGNT_CMD_WAIT_DONE             : integer := 11;

    constant MEM_BUS_AGNT_CMD_REPLY_OK   : integer := 256;
    constant MEM_BUS_AGNT_CMD_REPLY_ERR  : integer := 257;

    -- Reset gen actor
    constant actor_mem_bus_agent : actor_t := new_actor("actor_mem_bus_agent");

    -- Tag for messages
    constant MEM_BUS_AGENT_TAG : string := "Memory Bus Agent: ";

end package;


library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

package body mem_bus_agent_pkg is
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Memory bus agent API
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
   
    procedure mem_bus_agent_start(
        signal      net         : inout network_t
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(MEM_BUS_AGENT_TAG & "Starting");
        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_START));
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "Started");
    end procedure;


    procedure mem_bus_agent_stop(
        signal      net         : inout network_t
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(MEM_BUS_AGENT_TAG & "Stopping");
        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_STOP));
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "Stopped");
    end procedure;


    procedure mem_bus_agent_write_non_blocking(
        signal      net         : inout network_t;
                    address     : in    integer;
                    write_data  : in    std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    )  is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        debug(MEM_BUS_AGENT_TAG & "Posting non-blocking write, Address: 0x" &
               to_hstring(std_logic_vector(to_unsigned(address, 16))) &
              " " & to_hstring(write_data));

        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_WRITE_NON_BLOCKING));
        push(req_msg, address);
        push(req_msg, byte_enable);
        push(req_msg, write_data);
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);

        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "Mem bus agent non-blocking write posted");
    end procedure;


    procedure mem_bus_agent_write_blocking(
        signal      net         : inout network_t;
                    address     : in    integer;
                    write_data  : in    std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    )  is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        debug(MEM_BUS_AGENT_TAG & "Blocking write, Address: 0x" &
               to_hstring(std_logic_vector(to_unsigned(address, 16))) &
              " " & to_hstring(write_data));

        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_WRITE_BLOCKING));
        push(req_msg, address);
        push(req_msg, byte_enable);
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);

        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "Blocking write succesfull");
    end procedure;


    procedure mem_bus_agent_read(
        signal      net         : inout network_t;
                    address     : in    integer;
        variable    read_data   : out   std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        debug(MEM_BUS_AGENT_TAG & "Read, Address: 0x" &
               to_hstring(std_logic_vector(to_unsigned(address, 16))));

        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_READ));
        push(req_msg, address);
        push(req_msg, byte_enable);
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);

        receive_reply(net, req_msg, reply_msg);
        read_data := pop(reply_msg);
        wait for 0 ns;
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "Read done, read data: 0x" & to_hstring(read_data));
    end procedure;


    procedure mem_bus_agent_x_mode_start(
        signal      net         : inout network_t
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(MEM_BUS_AGENT_TAG & "Enabling X mode");
        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_X_MODE_START));
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "X mode enabled");
    end procedure;


    procedure mem_bus_agent_x_mode_stop(
        signal      net         : inout network_t
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(MEM_BUS_AGENT_TAG & "Disabling X mode");
        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_X_MODE_STOP));
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "X mode disabled");
    end procedure;


    procedure mem_bus_agent_set_x_mode_setup(
        signal      net         : inout network_t;
                    setup       : in    time
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(MEM_BUS_AGENT_TAG & "Setting X mode setup to: " & time'image(setup));
        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_SET_X_MODE_SETUP));
        push(req_msg, setup);
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "X mode setup configured");
    end procedure;


    procedure mem_bus_agent_set_x_mode_hold(
        signal      net         : inout network_t;
                    hold       : in    time
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(MEM_BUS_AGENT_TAG & "Setting X mode hold to: " & time'image(hold));
        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_SET_X_MODE_HOLD));
        push(req_msg, hold);
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "X mode hold configured");
    end procedure;


    procedure mem_bus_agent_set_period(
        signal      net        : inout network_t;
                    period     : in    time
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(MEM_BUS_AGENT_TAG & "Setting clock period to: " & time'image(period));
        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_SET_PERIOD));
        push(req_msg, period);
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "clock period configured");
    end procedure;


    procedure mem_bus_agent_set_output_delay(
        signal      net        : inout network_t;
                    out_delay  : in    time
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(MEM_BUS_AGENT_TAG & "Setting data out output delay " & time'image(out_delay));
        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_SET_OUTPUT_DELAY));
        push(req_msg, out_delay);
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "data out output delay set");
    end procedure;


    procedure mem_bus_agent_wait_done(
        signal      net        : inout network_t
    ) is
        constant mem_bus_agnt_rec : actor_t := find("actor_mem_bus_agent");
        variable req_msg, reply_msg  : msg_t;
    begin
        info(MEM_BUS_AGENT_TAG & "Waiting till all accesses are executed");
        req_msg := new_msg(msg_type => (p_code => MEM_BUS_AGNT_CMD_WAIT_DONE));
        send(net         => net,
             receiver    => mem_bus_agnt_rec,
             msg         => req_msg);
        receive_reply(net, req_msg, reply_msg); 
        check(message_type(reply_msg).p_code = MEM_BUS_AGNT_CMD_REPLY_OK,
              MEM_BUS_AGENT_TAG & "All accesses are executed!");
    end procedure;


    ---------------------------------------------------------------------------
    -- 8, 16 and 32 bits accesses are supported. Also bursts of multiples of
    -- 32 bits. Unaligned accesses are not supported (e.g. writing 4 byte buffer
    -- to address 0x2).
    ---------------------------------------------------------------------------
    function check_access_size(
        address     : in    integer;
        size        : in    natural
    ) return boolean is
    begin
        if (size /= 8 and size /= 16 and (size mod 32 /= 0)) then
            return false;
        end if;

        -- Half-word unaligned access
        if (size = 16 and (address mod 2) /= 0) then
            return false;
        end if;

        -- Word unaligned access
        if (size mod 32 = 0) and (address mod 32 /= 0) then
            return false;
        end if;

        return true;
    end function;


    procedure convert_be_and_write_data(
                 address     : in    natural;
                 data_in     : in    std_logic_vector;
        variable be          : out   std_logic_vector(3 downto 0);
        variable write_data  : out   std_logic_vector(31 downto 0)
    ) is
    begin
        write_data := (OTHERS => '0');
        case data_in'length is
        when 8 =>
            case (address mod 4) is
            when 0 =>
                be := "0001";
                write_data(7 downto 0) := data_in;
            when 1 =>
                be := "0010";
                write_data(15 downto 8) := data_in;
            when 2 =>
                be := "0100";
                write_data(23 downto 16) := data_in;
            when 3 =>
                be := "1000";
                write_data(31 downto 24) := data_in;
            end case;
        when 16 =>
            case (address mod 2) is
            when 0 =>
                be := "0011";
                write_data(15 downto 0) := data_in;
            when 1 =>
                be := "1100";
                write_data(31 downto 16) := data_in;
            end case;
        when others =>
            be := "1111";
            write_data(31 downto 0) := data_in;
        end case;
    end procedure;


    procedure convert_be_and_read_data(
                 address     : in    natural;
                 data_in     : in    std_logic_vector(31 downto 0);
        variable be          : out   std_logic_vector(3 downto 0);
        variable read_data   : out   std_logic_vector
    )is
    begin
        for i in 0 to read_data'length - 1 loop
            read_data(i) := '0';
        end loop;

        case data_in'length is
        when 8 =>
            case (address mod 4) is
            when 0 =>
                be := "0001";
                read_data := data_in(7 downto 0);
            when 1 =>
                be := "0010";
                read_data := data_in(15 downto 8);
            when 2 =>
                be := "0100";
                read_data := data_in(23 downto 16);
            when 3 =>
                be := "1000";
                read_data := data_in(31 downto 24);
            end case;
        when 16 =>
            case (address mod 2) is
            when 0 =>
                be := "0011";
                read_data := data_in(15 downto 0);
            when 1 =>
                be := "1100";
               read_data := data_in(31 downto 16);
            end case;
        when others =>
            be := "1111";
            read_data := data_in(31 downto 0);
        end case;
    end procedure;


    procedure mem_bus_agent_write(
        signal      net         : inout network_t;
                    address     : in    integer;
                    write_data  : in    std_logic_vector;
                    blocking    : in    boolean := true
    ) is
        variable addr_aligned   :       integer;
        variable addr_loop      :       integer;
        variable be             :       std_logic_vector(3 downto 0);
        variable data_32        :       std_logic_vector(31 downto 0);
        variable loop_count     :       integer;
    begin
        if (not check_access_size(address, write_data'length)) then
            return;
        end if;

        if (write_data'length = 8 or write_data'length = 16 or write_data'length = 32) then
            addr_aligned := address - (address mod 4);
            convert_be_and_write_data(address, write_data, be, data_32);
            if (blocking) then
                mem_bus_agent_write_blocking(net, addr_aligned, data_32, be);
            else
                mem_bus_agent_write_non_blocking(net, addr_aligned, data_32, be);
            end if;
        else
            loop_count := write_data'length / 32;
            addr_loop := address;

            for i in 0 to loop_count-1 loop
                data_32 := write_data(i*32+31 downto i*32);
                if (blocking) then
                    mem_bus_agent_write_blocking(net, addr_loop, data_32, be);
                else
                    mem_bus_agent_write_non_blocking(net, addr_loop, data_32, be);
                end if;
                addr_loop := addr_loop + 4;
            end loop;
        end if;
    end procedure;
    

    procedure mem_bus_agent_read(
        signal      net         : inout network_t;
                    address     : in    integer;
        variable    read_data   : out   std_logic_vector
    ) is
        variable addr_aligned   :       integer;
        variable addr_loop      :       integer;
        variable be             :       std_logic_vector(3 downto 0);
        variable data_32        :       std_logic_vector(31 downto 0);
        variable loop_count     :       integer;
    begin
        if (not check_access_size(address, read_data'length)) then
            return;
        end if;

        if (read_data'length = 8 or read_data'length = 16 or read_data'length = 32) then
            addr_aligned := address - (address mod 4);
            mem_bus_agent_read(net, addr_aligned, data_32, be);
            convert_be_and_read_data(address, data_32, be, read_data);
        else
            loop_count := read_data'length / 32;
            addr_loop := address;

            for i in 0 to loop_count-1 loop
                mem_bus_agent_read(net, addr_aligned, data_32, be);
                read_data(i*32+31 downto i*32) := data_32;
                addr_loop := addr_loop + 4;
            end loop;
        end if;
    end procedure;

end package body;