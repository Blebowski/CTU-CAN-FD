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
--    Package with API for Memory bus agent.
--
--------------------------------------------------------------------------------
-- Revision History:
--    19.1.2020   Created file
--    04.2.2021   Adjusted to work without Vunits COM library.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;


package mem_bus_agent_pkg is

    ---------------------------------------------------------------------------
    -- Memory bus agent component    
    ---------------------------------------------------------------------------
    component mem_bus_agent is
    generic(
        G_ACCESS_FIFO_DEPTH  : natural := 32;
        G_NUM_SLAVES         : natural := 2
    );
    port (
        clk             : in    std_logic;
        scs             : out   std_logic_vector(G_NUM_SLAVES - 1 downto 0) := (OTHERS => '0');
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
    -- Start memory bus agent.
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_start(
        signal channel  : inout t_com_channel
    );
    
    ---------------------------------------------------------------------------
    -- Stop memory bus agent.
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_stop(
        signal channel  : inout t_com_channel
    );
    
    ---------------------------------------------------------------------------
    -- Push write transaction to Memory bus agent FIFO. Function returns
    -- immediately after transaction is inserted and does not wait until
    -- transaction is executed.
    --
    -- This function is "raw" in such sense that user has to fill byte_enable
    -- and according data by himself.
    --
    -- @param channel   Channel on which to send the request
    -- @param address       Memory bus address
    -- @param write_data    Data to write
    -- @param byte_enable   Byte enable value.
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_write_non_blocking(
        signal      channel     : inout t_com_channel;
                    address     : in    integer;
                    write_data  : in    std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    );

    ---------------------------------------------------------------------------
    -- Push write transaction to Memory bus agent FIFO and wait till this
    -- memory transaction is executed. Memory bus agent must be enabled before
    -- calling this function.
    --
    -- @param channel   Channel on which to send the request
    -- @param address       Memory bus address
    -- @param write_data    Data to write
    -- @param byte_enable   Byte enable signals.
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_write_blocking(
        signal      channel     : inout t_com_channel;
                    address     : in    integer;
                    write_data  : in    std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    );

    ---------------------------------------------------------------------------
    -- Start X-mode in Memory bus agent. In X-mode, setup, hold and data out
    -- delay parameters are considered and signals of memory transaction are
    -- driven to X everywhere apart from "Setup + hold" window. Also, read
    -- data on memory bus are sampled with "data out" delay from point where
    -- they should be exposed output of DUT.
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_x_mode_start(
        signal      channel     : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Stop X-mode in Memory bus agent.
    --
    -- @param channel   Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_x_mode_stop(
        signal      channel     : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Configure Setup time for X-mode of Memory bus agent.
    --
    -- @param channel       Channel on which to send the request
    -- @param setup         Setup time to be configured.
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_set_x_mode_setup(
        signal      channel     : inout t_com_channel;
                    setup       : in    time
    );

    ---------------------------------------------------------------------------
    -- Configure Hold time for X-mode of Memory bus agent.
    --
    -- @param channel       Channel on which to send the request
    -- @param hold          Hold time to be configured.
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_set_x_mode_hold(
        signal      channel     : inout t_com_channel;
                    hold        : in    time
    );


    ---------------------------------------------------------------------------
    -- Configure data out delay of DUT. During read transactions data are
    -- sampled from DUT with data out delay from rising edge of clock.
    --
    -- @param channel       Channel on which to send the request
    -- @param out_delay     Output delay to use for sampling.
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_set_output_delay(
        signal      channel     : inout t_com_channel;
                    out_delay   : in    time
    );

    
    ---------------------------------------------------------------------------
    -- Wait till all transactions executed by memory bus agent are over.
    --
    -- @param channel       Channel on which to send the request
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_wait_done(
        signal      channel     : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Execute read transaction by Memory bus agent. This function returns
    -- after read transaction was executed.
    --
    -- @param channel       Channel on which to send the request
    -- @param address       Memory bus address.
    -- @param read_data     Variable in which output data will be read.
    -- @param byte_enable   Byte enable value.
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_read(
        signal      channel     : inout t_com_channel;
                    address     : in    integer;
        variable    read_data   : inout std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    );

    ---------------------------------------------------------------------------
    -- Insert Write transaction to Memory bus agent FIFO.
    --
    -- This function is "wrapper" and provides higher level abstraction for
    -- memory bus access.
    --
    -- Size of Memory access is determined by size of "write_data" input
    -- parameter. Following values are supported:
    --  8       - Byte access       -> Any address
    --  16      - Half-word access  -> Address must be half-word aligned.
    --  32      - Word access       -> Address must be word aligned.
    --  32 * N  - N cell burst      -> Address must be word aligned.
    --
    -- If address is not aligned properly, unaligned access error is reported
    -- and transaction is ignored.
    --
    -- In case of N-cell burst, this burst is executed in N consecutive clock
    -- cycles. Data are interpreted LSB first, e.g:
    --
    --  Input data vector:      127 downto 0
    --
    --  First burst cell:       31 downto 0
    --  Second burst cell:      63 downto 32
    --  Third burst cell:       95 downto 64
    --  Fourth burst cell:      127 downto 96    
    --
    -- @param channel       Channel on which to send the request
    -- @param address       Memory bus address.
    -- @param write_data    Data to be written.
    -- @param blocking      True if function shall return only after write
    --                      was executed (blocking write), false if function
    --                      shall return immediately (non-blocking write).
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_write(
        signal      channel     : inout t_com_channel;
                    address     : in    integer;
                    write_data  : in    std_logic_vector;
                    blocking    : in    boolean := true
    );
    
    ---------------------------------------------------------------------------
    -- Insert Read transaction to Memory bus agent FIFO.
    --
    -- This function is "wrapper" and provides higher level abstraction for
    -- memory bus access.
    --
    -- Size of Memory access is determined by size of "read_data" input
    -- parameter. Following values are supported:
    --  8       - Byte access       -> Any address
    --  16      - Half-word access  -> Address must be half-word aligned.
    --  32      - Word access       -> Address must be word aligned.
    --  32 * N  - N cell burst      -> Address must be word aligned.
    --
    -- If address is not aligned properly, unaligned access error is reported
    -- and transaction is ignored.
    --
    -- In case of N-cell burst, this burst is executed in N consecutive clock
    -- cycles. Data are interpreted LSB first, e.g:
    --
    --  Output data vector:      127 downto 0
    --
    --  First burst cell:       31 downto 0
    --  Second burst cell:      63 downto 32
    --  Third burst cell:       95 downto 64
    --  Fourth burst cell:      127 downto 96    
    --
    -- @param channel       Channel on which to send the request
    -- @param address       Memory bus address.
    -- @param read_data     Variable in which output data are read.
    -- @param stat_burst    If true, address is not incremented during burst
    --                      accesses. Usefull for readout of FIFOs
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_read(
        signal      channel     : inout t_com_channel;
                    address     : in    integer;
        variable    read_data   : inout std_logic_vector;
        constant    stat_burst  : in    boolean := false
    );
   
    
    ---------------------------------------------------------------------------
    -- Changes slave node to which the transaction will be routed. Slave nodes
    -- are distuiguished by chip select.
    --
    -- @param channel       Channel on which to send the request
    --
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_set_slave_index(
        signal      channel     : inout t_com_channel;
                    node        : in    natural
    );
    
    ---------------------------------------------------------------------------
    -- Enable transaction reporting (reports each memory access to console)
    --
    -- @param channel       Channel on which to send the request
    --
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_enable_transaction_reporting(
        signal      channel     : inout t_com_channel
    );

    ---------------------------------------------------------------------------
    -- Disable transaction reporting (reports each memory access to console)
    --
    -- @param channel       Channel on which to send the request
    --
    ---------------------------------------------------------------------------
    procedure mem_bus_agent_disable_transaction_reporting(
        signal      channel     : inout t_com_channel
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

    constant MEM_BUS_AGNT_CMD_SET_OUTPUT_DELAY      : integer := 10;
    constant MEM_BUS_AGNT_CMD_WAIT_DONE             : integer := 11;
    
    constant MEM_BUS_AGNT_CMD_SET_SLAVE_INDEX       : integer := 12;
    
    constant MEM_BUS_AGNT_CMD_ENABLE_TRANS_REPORT   : integer := 13;
    constant MEM_BUS_AGNT_CMD_DISABLE_TRANS_REPORT  : integer := 14;
   
    -- Tag for messages
    constant MEM_BUS_AGENT_TAG : string := "Memory Bus Agent: ";

end package;


package body mem_bus_agent_pkg is
    
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Memory bus agent API
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
   
    procedure mem_bus_agent_start(
        signal      channel     : inout t_com_channel
    ) is
    begin
        info_m(MEM_BUS_AGENT_TAG & "Starting");
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_START);
        debug_m(MEM_BUS_AGENT_TAG & "Started");
    end procedure;


    procedure mem_bus_agent_stop(
        signal      channel     : inout t_com_channel
    ) is
    begin
        info_m(MEM_BUS_AGENT_TAG & "Stopping");
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_STOP);
        debug_m(MEM_BUS_AGENT_TAG & "Stopped");
    end procedure;


    procedure mem_bus_agent_write_non_blocking(
        signal      channel     : inout t_com_channel;
                    address     : in    integer;
                    write_data  : in    std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    )  is
    begin
        --debug_m(MEM_BUS_AGENT_TAG & "Posting non-blocking write, Address: 0x" &
        --       to_hstring(std_logic_vector(to_unsigned(address, 16))) &
        --      " " & to_hstring(write_data));

        -- Pack the transaction to parameter vector
        com_channel_data.set_param(address);
        com_channel_data.set_param(write_data & byte_enable);
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_WRITE_NON_BLOCKING);

        --debug_m(MEM_BUS_AGENT_TAG & "Mem bus agent non-blocking write posted");
    end procedure;


    procedure mem_bus_agent_write_blocking(
        signal      channel     : inout t_com_channel;
                    address     : in    integer;
                    write_data  : in    std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    )  is
    begin
        --debug_m(MEM_BUS_AGENT_TAG & "Blocking write, Address: 0x" &
        --       to_hstring(std_logic_vector(to_unsigned(address, 16))) &
        --      " " & to_hstring(write_data));
        
        com_channel_data.set_param(address);
        com_channel_data.set_param(write_data & byte_enable);
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_WRITE_BLOCKING);

        --debug_m(MEM_BUS_AGENT_TAG & "Blocking write succesfull");
    end procedure;


    procedure mem_bus_agent_read(
        signal      channel     : inout t_com_channel;
                    address     : in    integer;
        variable    read_data   : inout std_logic_vector(31 downto 0);
                    byte_enable : in    std_logic_vector(3 downto 0) 
    ) is
        variable tmp : std_logic_vector(127 downto 0);
    begin
        --info_m(MEM_BUS_AGENT_TAG & "Read, Address: 0x" &
        --       to_hstring(std_logic_vector(to_unsigned(address, 16))));

        com_channel_data.set_param(address);
        com_channel_data.set_param(byte_enable);
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_READ);

        tmp := com_channel_data.get_param;
        read_data := tmp(31 downto 0);
        wait for 0 ns;
        --info_m(MEM_BUS_AGENT_TAG & "Read done, read data: 0x" & to_hstring(read_data));
    end procedure;


    procedure mem_bus_agent_x_mode_start(
        signal      channel     : inout t_com_channel
    ) is
    begin
        info_m(MEM_BUS_AGENT_TAG & "Enabling X mode");
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_X_MODE_START);
        debug_m(MEM_BUS_AGENT_TAG & "X mode enabled");
    end procedure;


    procedure mem_bus_agent_x_mode_stop(
        signal      channel     : inout t_com_channel
    ) is
    begin
        info_m(MEM_BUS_AGENT_TAG & "Disabling X mode");
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_X_MODE_STOP);
        debug_m(MEM_BUS_AGENT_TAG & "X mode disabled");
    end procedure;


    procedure mem_bus_agent_set_x_mode_setup(
        signal      channel     : inout t_com_channel;
                    setup       : in    time
    ) is
    begin
        info_m(MEM_BUS_AGENT_TAG & "Setting X mode setup to: " & time'image(setup));
        com_channel_data.set_param(setup);
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_SET_X_MODE_SETUP);
        debug_m(MEM_BUS_AGENT_TAG & "X mode setup configured");
    end procedure;


    procedure mem_bus_agent_set_x_mode_hold(
        signal      channel     : inout t_com_channel;
                    hold        : in    time
    ) is
    begin
        info_m(MEM_BUS_AGENT_TAG & "Setting X mode hold to: " & time'image(hold));
        com_channel_data.set_param(hold);
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_SET_X_MODE_HOLD);
        debug_m(MEM_BUS_AGENT_TAG & "X mode hold configured");
    end procedure;


    procedure mem_bus_agent_set_output_delay(
        signal      channel     : inout t_com_channel;
                    out_delay   : in    time
    ) is
    begin
        info_m(MEM_BUS_AGENT_TAG & "Setting data out output delay " & time'image(out_delay));
        com_channel_data.set_param(out_delay);
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_SET_OUTPUT_DELAY);
        debug_m(MEM_BUS_AGENT_TAG & "data out output delay set");
    end procedure;


    procedure mem_bus_agent_wait_done(
        signal      channel     : inout t_com_channel
    ) is
    begin
        info_m(MEM_BUS_AGENT_TAG & "Waiting till all accesses are executed");
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_WAIT_DONE);
        info_m(MEM_BUS_AGENT_TAG & "All accesses are executed!");
    end procedure;
    
    
    procedure mem_bus_agent_set_slave_index(
        signal      channel     : inout t_com_channel;
                    node        : in    natural
    ) is
    begin
        com_channel_data.set_param(node);
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_SET_SLAVE_INDEX);
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
        if ((size /= 8) and (size /= 16) and ((size mod 32) /= 0)) then
            return false;
        end if;

        -- Half-word unaligned access
        if ((size = 16) and ((address mod 2) /= 0)) then
            return false;
        end if;

        -- Word unaligned access
        if ((size mod 32) = 0) and ((address mod 4) /= 0) then
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
            case (address mod 4) is
            when 0 =>
                be := "0011";
                write_data(15 downto 0) := data_in;
            when 2 =>
                be := "1100";
                write_data(31 downto 16) := data_in;
            when others =>
                error_m("Unsupported 16 bit write at addr: " & to_string(address));
            end case;
        when others =>
            be := "1111";
            write_data(31 downto 0) := data_in;
        end case;
    end procedure;


    procedure convert_be(
                 address     : in    natural;
                 data_in     : in    std_logic_vector;
        variable be          : out   std_logic_vector(3 downto 0)
    )is
    begin

        case data_in'length is
        when 8 =>
            case (address mod 4) is
            when 0 =>
                be := "0001";
            when 1 =>
                be := "0010";
            when 2 =>
                be := "0100";
            when 3 =>
                be := "1000";
            end case;
        when 16 =>
            case (address mod 4) is
            when 0 =>
                be := "0011";
            when 2 =>
                be := "1100";
            when others =>
                error_m("16 bit access must be half-word aligned!");
            end case;
        when others =>
            be := "1111";
        end case;
    end procedure;

    procedure convert_read_data(
                 address             : in    natural;
                 size                : in    natural;
                 data_in             : in    std_logic_vector(31 downto 0);
        variable data_out            : out   std_logic_vector
    )is
    begin
        case size is
        when 8 =>
            case (address mod 4) is
            when 0 =>
                data_out(7 downto 0) := data_in(7 downto 0);
            when 1 =>
                data_out(7 downto 0) := data_in(15 downto 8);
            when 2 =>
                data_out(7 downto 0) := data_in(23 downto 16);
            when 3 =>
                data_out(7 downto 0) := data_in(31 downto 24);
            when others =>
                error_m("Invalid 32 bit access!");
            end case;
        when 16 =>
            case (address mod 4) is
            when 0 =>
                data_out(15 downto 0) := data_in(15 downto 0);
            when 2 =>
                data_out(15 downto 0) := data_in(31 downto 16);
            when others =>
                error_m("Invalid address for 16 bit access!");
            end case;
        when 32 =>
            data_out := data_in;
        when others =>
            error_m("Unknown access size: " & integer'image(size));
        end case;
    end;

    procedure mem_bus_agent_write(
        signal      channel     : inout t_com_channel;
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
                mem_bus_agent_write_blocking(channel, addr_aligned, data_32, be);
            else
                mem_bus_agent_write_non_blocking(channel, addr_aligned, data_32, be);
            end if;
        else
            loop_count := write_data'length / 32;
            addr_loop := address;
            -- For burst accesses, always 32 bit
            be := x"F";

            for i in 0 to loop_count-1 loop
                data_32 := write_data(i*32+31 downto i*32);
                if (blocking) then
                    mem_bus_agent_write_blocking(channel, addr_loop, data_32, be);
                else
                    mem_bus_agent_write_non_blocking(channel, addr_loop, data_32, be);
                end if;
                addr_loop := addr_loop + 4;
            end loop;
        end if;
    end procedure;
    

    procedure mem_bus_agent_read(
        signal      channel     : inout t_com_channel;
                    address     : in    integer;
        variable    read_data   : inout std_logic_vector;
        constant    stat_burst  : in    boolean := false
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
            convert_be(address, read_data, be);
            mem_bus_agent_read(channel, addr_aligned, data_32, be);
            convert_read_data(address, read_data'length, data_32, read_data);
        else
            loop_count := read_data'length / 32;
            addr_loop := address;
            -- For burst accesses, always 32 bit
            be := x"F";
            
            for i in 0 to loop_count-1 loop
                mem_bus_agent_read(channel, addr_loop, data_32, be);
                read_data(i*32+31 downto i*32) := data_32;
                if (stat_burst = false) then
                    addr_loop := addr_loop + 4;
                end if;
            end loop;
        end if;
    end procedure;
    
    procedure mem_bus_agent_enable_transaction_reporting(
        signal      channel     : inout t_com_channel
    ) is
    begin
        debug_m(MEM_BUS_AGENT_TAG & "Enabling transaction reporting");
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_ENABLE_TRANS_REPORT);
        debug_m(MEM_BUS_AGENT_TAG & "Transaction reporting enabled");
    end procedure;


    procedure mem_bus_agent_disable_transaction_reporting(
        signal      channel     : inout t_com_channel
    ) is
    begin
        debug_m(MEM_BUS_AGENT_TAG & "Disabling transaction reporting");
        send(channel, C_MEM_BUS_AGENT_ID, MEM_BUS_AGNT_CMD_DISABLE_TRANS_REPORT);
        debug_m(MEM_BUS_AGENT_TAG & "Transaction reporting disabled");
    end procedure;

end package body;