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
use ctu_can_fd_tb.tb_shared_vars_pkg.all;

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
