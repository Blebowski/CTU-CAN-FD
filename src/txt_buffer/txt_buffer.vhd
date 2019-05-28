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
--     30.8.2018  Added "txtb_hw_cmd_int" output for Interrupt Manager. TXTB HW
--                command Interrupt generated upon move to Done, Failed or
--                Aborted states.
--    10.11.2018  Separated TXT Buffer FSM to a standalone sub-module.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity txt_buffer is
    generic(
        -- Reset polarity
        G_RESET_POLARITY       :     std_logic := '0';
        
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT     :     natural range 1 to 8;
        
        -- TXT Buffer ID
        G_ID                   :     natural := 1
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous reset
        res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Memory Registers Interface
        ------------------------------------------------------------------------
        -- Data to be written to TXT Buffer RAM
        txtb_port_a_data       :in   std_logic_vector(31 downto 0);
        
        -- Address in TXT Buffer RAM
        txtb_port_a_address    :in   std_logic_vector(4 downto 0);

        -- TXT Buffer RAM chip select
        txtb_port_a_cs         :in   std_logic;

        -- SW commands
        txtb_sw_cmd            :in   t_txtb_sw_cmd;
        
        -- TXT Buffer index for which SW command is valid
        txtb_sw_cmd_index      :in   std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        -- Buffer State (encoded for Memory registers)
        txtb_state             :out  std_logic_vector(3 downto 0);

        ------------------------------------------------------------------------   
        -- Interrupt Manager Interface
        ------------------------------------------------------------------------
        -- HW Command applied
        txtb_hw_cmd_int         :out  std_logic;

        ------------------------------------------------------------------------
        -- CAN Core and TX Arbitrator Interface
        ------------------------------------------------------------------------
        -- HW Commands 
        txtb_hw_cmd            :in   t_txtb_hw_cmd;
        
        -- Index of TXT Buffer for which HW commands is valid          
        txtb_hw_cmd_index      :in   natural range 0 to G_TXT_BUFFER_COUNT - 1;

        -- TXT Buffer RAM data output
        txtb_port_b_data       :out  std_logic_vector(31 downto 0);
        
        -- TXT Buffer RAM address
        txtb_port_b_address    :in   natural range 0 to 19;

        -- Unit just turned bus off.
        is_bus_off             :in   std_logic;

        -- TXT Buffer is ready to be locked by CAN Core for transmission
        txtb_ready             :out  std_logic
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
    hw_cbs <= '1' when (txtb_hw_cmd_index = G_ID)
                  else
              '0';
  
    sw_cbs <= '1' when (txtb_sw_cmd_index(G_ID) = '1') 
                  else
              '0';
    
    -- TXT Buffer RAM write signal
    RAM_write <= '1' when (txtb_port_a_cs = '1' and txtb_user_accessible = '1')
                     else
                 '0';

    -- TXT Buffer read address (connected to read pointer)
    RAM_read_address   <= std_logic_vector(to_unsigned(
                             txtb_port_b_address, RAM_read_address'length)); 


    ----------------------------------------------------------------------------
    -- RAM Memory of TXT Buffer
    ----------------------------------------------------------------------------
    txt_buffer_ram_inst : txt_buffer_ram
    generic map(
        G_RESET_POLARITY     => G_RESET_POLARITY
    )
    port map(
        -- Clock and Asynchronous reset
        clk_sys              => clk_sys,                -- IN
        res_n                => res_n,                  -- IN

        -- Port A - Write (from Memory registers)
        port_a_address       => txtb_port_a_address,    -- IN
        port_a_data_in       => txtb_port_a_data,       -- IN
        port_a_write         => RAM_write,              -- IN

        -- Port B - Read (from CAN Core)
        port_b_address       => RAM_read_address,       -- IN
        port_b_data_out      => txtb_port_b_data        -- OUT
    );

    
    ----------------------------------------------------------------------------
    -- TXT Buffer FSM
    ----------------------------------------------------------------------------
    txt_buffer_fsm_inst : txt_buffer_fsm
    generic map(
        G_RESET_POLARITY       => G_RESET_POLARITY, 
        G_ID                   => G_ID
    )
    port map(
        clk_sys                => clk_sys,                  -- IN
        res_n                  => res_n,                    -- IN

        txtb_sw_cmd            => txtb_sw_cmd,              -- IN
        sw_cbs                 => sw_cbs,                   -- IN

        txtb_hw_cmd            => txtb_hw_cmd,              -- IN
        hw_cbs                 => hw_cbs,                   -- IN
        is_bus_off             => is_bus_off,               -- IN

        txtb_user_accessible   => txtb_user_accessible,     -- OUT
        txtb_hw_cmd_int        => txtb_hw_cmd_int,          -- OUT
        txtb_state             => txtb_state,               -- OUT
        txtb_ready             => txtb_ready                -- OUT
    );

    
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Functional coverage
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    func_cov_block : block
    begin 
    
    -- psl default clock is rising_edge(clk_sys);

    -- Each SW command active
    -- psl txtb_set_ready_cov : cover (txtb_sw_cmd.set_rdy = '1' and sw_cbs = '1');
    -- psl txtb_set_empty_cov : cover (txtb_sw_cmd.set_ety = '1' and sw_cbs = '1');
    -- psl txtb_set_abort_cov : cover (txtb_sw_cmd.set_abt = '1' and sw_cbs = '1');
      
    -- HW Commands
    -- psl txtb_hw_lock : cover (txtb_hw_cmd.lock = '1' and hw_cbs = '1');
    -- psl txtb_hw_unlock : cover (txtb_hw_cmd.unlock = '1' and hw_cbs = '1');
    -- psl txtb_hw_valid : cover (txtb_hw_cmd.valid = '1' and hw_cbs = '1');
    -- psl txtb_hw_err : cover (txtb_hw_cmd.err = '1' and hw_cbs = '1');
    -- psl txtb_hw_arbl : cover (txtb_hw_cmd.arbl = '1' and hw_cbs = '1');
    -- psl txtb_hw_failed : cover (txtb_hw_cmd.failed = '1' and hw_cbs = '1');
    
    end block;

end architecture;