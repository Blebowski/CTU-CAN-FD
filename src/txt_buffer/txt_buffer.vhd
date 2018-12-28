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
-- Purpose:
--  Transmit message buffer. Buffer is accessed via "tran_data" and "tran_addr"
--  signals from user registers. Buffer contains simple FSM for manipulation
--  from HW (HW Commands) as well as SW (SW Commands). Buffer contains 20*32
--  bit RAM which is inferred by synthesis.
--------------------------------------------------------------------------------
-- Revision History:
--
--    July 2015   Created file
--    30.11.2017  Changed the buffer implementation from parallel into 32*20 
--                buffer of data. Reading so far left parallel. User is directly
--                accessing the buffer and storing the data to it.
--    04.12.2017  Buffer split to "Frame metadata" (txt_buffer_info) and "Data" 
--                (txt_buffer_data). Frame metadata consists of first 4 words 
--                (Frame format, Timestamps and Identifier). Frame metadata are
--                available combinationally at all times. Frame data are accessed
--                directly from CAN Core by new pointer "txt_data_addr". 
--                txt_buffer_data is synthesized as RAM memory and significant
--                reource reduction was achieved.
--     15.2.2018  Implemented TXT Buffer state machine. Replaced "empty", "ack"
--                and "allow" signals with HW commands from Protocol Control and
--                SW commands from User registers. Hardware commands have always
--                higher priority.
--     24.3.2018  1. Changed TXT Buffer implementation to have both, metadata
--                    and data in CAN frame.
--                2. Added memory protection on the TXT Buffer. It can NOT be
--                    written when in "ready", "tx in progress" or "abort in
--                    progress" states.
--     06.4.2018  Changed output from side of CAN Core to synchronous. Async.
--                output did not allow inferrence of RAM in Altera FPGA.
--     30.8.2018  Added "txt_hw_cmd_int" output for Interrupt Manager. TXTB HW
--                command Interrupt generated upon move to Done, Failed or
--                Aborted states.
--    10.11.2018  Separated TXT Buffer FSM to a standalone sub-module.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.can_constants.all;
use work.can_components.all;

entity txt_buffer is
    generic(
        constant buf_count            :     natural range 1 to 8;
        constant ID                   :     natural := 1
    );
  PORT(
        ------------------------------------------------------------------------
        -- Clock and reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Driving Registers Interface
        ------------------------------------------------------------------------

        -- Data and address for SW access into the RAM of TXT Buffer
        signal tran_data              :in   std_logic_vector(31 downto 0);
        signal tran_addr              :in   std_logic_vector(4 downto 0);
        signal tran_cs                :in   std_logic;

        -- SW commands from user registers
        signal txt_sw_cmd             :in   txt_sw_cmd_type;
        signal txt_sw_buf_cmd_index   :in   std_logic_vector(
                                                buf_count - 1 downto 0);

        ------------------------------------------------------------------------   
        -- Status signals
        ------------------------------------------------------------------------
        signal txtb_state             :out  std_logic_vector(3 downto 0);

        ------------------------------------------------------------------------
        -- Interrupt Manager
        ------------------------------------------------------------------------
        signal txt_hw_cmd_int         :out  std_logic;

        ------------------------------------------------------------------------
        -- CAN Core and TX Arbiter Interface
        ------------------------------------------------------------------------

        -- Commands from the CAN Core for manipulation of the CAN 
        signal txt_hw_cmd             :in   txt_hw_cmd_type;  
        signal txt_hw_cmd_buf_index   :in   natural range 0 to buf_count - 1;

        -- Buffer output and pointer to the RAM memory
        signal txt_word               :out  std_logic_vector(31 downto 0);
        signal txt_addr               :in   natural range 0 to 19;

        signal bus_off_start          :in   std_logic;

        -- Signals to the TX Arbitrator that it can be selected for transmission
        -- (used as input to priority decoder)
        signal txt_buf_ready          :out  std_logic
    );
             
end entity;


architecture rtl of txt_buffer is

    ----------------------------------------------------------------------------
    -- Internal registers
    ----------------------------------------------------------------------------
    type frame_memory is array(0 to 19) of std_logic_vector(31 downto 0);

    ----------------------------------------------------------------------------
    --Signal aliases
    ----------------------------------------------------------------------------

    -- TXT Buffer memory protection
    signal txtb_user_accessible   : std_logic;

    -- Internal buffer selects for commands. Commands are shared across the
    -- buffers so we need unique identifier
    signal hw_cbs                 : std_logic;
    signal sw_cbs                 : std_logic;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- RAM wrapper signals
    ----------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    -- Write control signal    
    signal RAM_write              : std_logic;

    -- Read address (connected to read pointer)
    signal RAM_read_address       : std_logic_vector(4 downto 0);


begin
        
    -- Command buffer select signals
    hw_cbs <= '1' when txt_hw_cmd_buf_index = ID
                  else
              '0';
  
    sw_cbs <= '1' when txt_sw_buf_cmd_index(ID) = '1' 
                  else
              '0';
    
    -- TXT Buffer RAM write signal
    RAM_write <= '1' when (tran_cs = '1' and txtb_user_accessible = '1')
                     else
                 '0';

    -- TXT Buffer read address (connected to read pointer)
    RAM_read_address   <= std_logic_vector(to_unsigned(
                             txt_addr, RAM_read_address'length)); 


    ----------------------------------------------------------------------------
    -- RAM Memory of TXT Buffer
    ----------------------------------------------------------------------------
    txt_buf_RAM : inf_RAM_wrapper 
    generic map (
        word_width           => 32,
        depth                => 20,
        address_width        => RAM_read_address'length,
        reset_polarity       => ACT_RESET,
        simulation_reset     => true,
        sync_read            => true
    )
    port map(
        clk_sys              => clk_sys,
        res_n                => res_n,
        addr_A               => tran_addr,
        write                => RAM_write, 
        data_in              => tran_data,
        addr_B               => RAM_read_address,
        data_out             => txt_word
    );

    
    ----------------------------------------------------------------------------
    -- TXT Buffer FSM
    ----------------------------------------------------------------------------
    txt_buffer_fsm_comp : txt_buffer_fsm
    generic map(
        ID                     => ID
    )
    port map(
        clk_sys                => clk_sys,
        res_n                  => res_n,

        txt_sw_cmd             => txt_sw_cmd,
        sw_cbs                 => sw_cbs,

        txt_hw_cmd             => txt_hw_cmd,
        hw_cbs                 => hw_cbs,
        bus_off_start          => bus_off_start,

        txtb_user_accessible   => txtb_user_accessible,
        txtb_hw_cmd_int        => txt_hw_cmd_int,
        txtb_state             => txtb_state,
        txt_buf_ready          => txt_buf_ready
    );

end architecture;
