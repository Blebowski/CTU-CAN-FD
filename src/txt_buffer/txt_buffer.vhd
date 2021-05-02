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
-- Module:
--  TXT Buffer
-- 
-- Purpose:
--  Stores single frame for transmission in internal RAM. Accessed from Memory
--  registers via memory bus (to store frame) and SW commands.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer.all;
use ctu_can_fd_rtl.can_constants.all;
use ctu_can_fd_rtl.can_components.all;
use ctu_can_fd_rtl.can_types.all;
use ctu_can_fd_rtl.cmn_lib.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.reduce_lib.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

use ctu_can_fd_rtl.can_registers_pkg.all;

entity txt_buffer is
    generic(
        -- Reset polarity
        G_RESET_POLARITY       :     std_logic := '0';
        
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT     :     natural range 2 to 8;
        
        -- TXT Buffer ID
        G_ID                   :     natural := 1;
        
        -- Technology type
        G_TECHNOLOGY           :     natural := C_TECH_ASIC
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

        -- TXT Buffer bus-off behavior
        txt_buf_failed_bof     :in   std_logic;
        
        -- Restricted operation mode
        drv_rom_ena            :in   std_logic;

        -- Bus monitoring mode
        drv_bus_mon_ena        :in   std_logic;
        
        ------------------------------------------------------------------------
        -- Memory Testability
        ------------------------------------------------------------------------
        -- Test registers
        test_registers_out     :in   test_registers_out_t;
        
        -- TXT buffer RAM test output
        tst_rdata_txt_buf      :out  std_logic_vector(31 downto 0);

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

        -- Clock enable to TXT Buffer port B
        txtb_port_b_clk_en     :in   std_logic;

        -- Unit just turned bus off.
        is_bus_off             :in   std_logic;

        -- TXT Buffer is available to be locked by CAN Core for transmission
        txtb_available         :out  std_logic
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

    -- Unmask TXT Buffer RAM output
    signal txtb_unmask_data_ram   : std_logic;

    -- Output of TXT Buffer RAM
    signal txtb_port_b_data_i     : std_logic_vector(31 downto 0);

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- RAM wrapper signals
    ----------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    -- Write control signal    
    signal ram_write              : std_logic;

    -- Read address (connected to read pointer)
    signal ram_read_address       : std_logic_vector(4 downto 0);

    -- Clock enabled
    signal txtb_ram_clk_en        : std_logic;
    
    -- RAM clocks
    signal clk_ram                : std_logic;

begin
        
    -- Command buffer select signals
    hw_cbs <= '1' when (txtb_hw_cmd_index = G_ID)
                  else
              '0';
  
    sw_cbs <= '1' when (txtb_sw_cmd_index(G_ID) = '1') 
                  else
              '0';
    
    -- TXT Buffer RAM write signal
    ram_write <= '1' when (txtb_port_a_cs = '1' and txtb_user_accessible = '1')
                     else
                 '0';

    -- TXT Buffer read address (connected to read pointer)    
    ram_read_address <= std_logic_vector(to_unsigned(
                        txtb_port_b_address, RAM_read_address'length));

    ----------------------------------------------------------------------------
    -- Output of TXT Buffer RAM is masked when it is not valid. This has
    -- several reasons:
    --  1. RAM content is undefined, therefore before filling RAM, XXXs on
    --     output if further comparator logic of TX Arbitrator will yell a lot.
    --     This saves from flood of simulation warnings!
    --  2. CAN Core and TX Arbitrator should not be reading any data from
    --     TXT Buffer RAM when it is not in Ready, TX in Progress or Abort in
    --     Progress (SW did not fill them yet). So we make sure that they are
    --     not used somewhere when they might be undefined yet!
    ----------------------------------------------------------------------------
    txtb_port_b_data <= txtb_port_b_data_i when (txtb_unmask_data_ram = '1')
                                           else
                           (OTHERS => '0');

    ----------------------------------------------------------------------------
    -- Clock gating for TXT Buffer RAM. Enable when:
    --  1. Read access from CAN core
    --  2. Write access from user
    --  3. Always in test mode
    ----------------------------------------------------------------------------
    txtb_ram_clk_en <= '1' when (txtb_port_b_clk_en = '1' or ram_write = '1')
                           else
                       '1' when (test_registers_out.tst_control(TMAENA_IND) = '1')
                           else
                       '0';

    clk_gate_txt_buffer_ram_comp : clk_gate
    generic map(
        G_TECHNOLOGY       => G_TECHNOLOGY
    )
    port map(
        clk_in             => clk_sys,
        clk_en             => txtb_ram_clk_en,

        clk_out            => clk_ram
    );

    ----------------------------------------------------------------------------
    -- RAM Memory of TXT Buffer
    ----------------------------------------------------------------------------
    txt_buffer_ram_inst : txt_buffer_ram
    generic map(
        G_RESET_POLARITY     => G_RESET_POLARITY,
        G_ID                 => G_ID
    )
    port map(
        -- Clock and Asynchronous reset
        clk_sys              => clk_ram,                -- IN
        res_n                => res_n,                  -- IN

        -- Memory testability
        test_registers_out   => test_registers_out,     -- IN
        tst_rdata_txt_buf    => tst_rdata_txt_buf,      -- OUT

        -- Port A - Write (from Memory registers)
        port_a_address       => txtb_port_a_address,    -- IN
        port_a_data_in       => txtb_port_a_data,       -- IN
        port_a_write         => ram_write,              -- IN

        -- Port B - Read (from CAN Core)
        port_b_address       => ram_read_address,       -- IN
        port_b_data_out      => txtb_port_b_data_i      -- OUT
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
        txt_buf_failed_bof     => txt_buf_failed_bof,       -- IN

        txtb_hw_cmd            => txtb_hw_cmd,              -- IN
        hw_cbs                 => hw_cbs,                   -- IN
        is_bus_off             => is_bus_off,               -- IN
        drv_rom_ena            => drv_rom_ena,              -- IN
        drv_bus_mon_ena        => drv_bus_mon_ena,          -- IN

        txtb_user_accessible   => txtb_user_accessible,     -- OUT
        txtb_hw_cmd_int        => txtb_hw_cmd_int,          -- OUT
        txtb_state             => txtb_state,               -- OUT
        txtb_available         => txtb_available,           -- OUT
        txtb_unmask_data_ram   => txtb_unmask_data_ram      -- OUT
    );

    -- <RELEASE_OFF>
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Functional coverage
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    func_cov_block : block
    begin 
    
    -- psl default clock is rising_edge(clk_sys);

    -- Each SW command active
    -- psl txtb_set_ready_cov : cover {txtb_sw_cmd.set_rdy = '1' and sw_cbs = '1'};
    -- psl txtb_set_empty_cov : cover {txtb_sw_cmd.set_ety = '1' and sw_cbs = '1'};
    -- psl txtb_set_abort_cov : cover {txtb_sw_cmd.set_abt = '1' and sw_cbs = '1'};
      
    -- HW Commands
    -- psl txtb_hw_lock : cover {txtb_hw_cmd.lock = '1' and hw_cbs = '1'};
    -- psl txtb_hw_unlock : cover {txtb_hw_cmd.unlock = '1' and hw_cbs = '1'};
    -- psl txtb_hw_valid : cover {txtb_hw_cmd.valid = '1' and hw_cbs = '1'};
    -- psl txtb_hw_err : cover {txtb_hw_cmd.err = '1' and hw_cbs = '1'};
    -- psl txtb_hw_arbl : cover {txtb_hw_cmd.arbl = '1' and hw_cbs = '1'};
    -- psl txtb_hw_failed : cover {txtb_hw_cmd.failed = '1' and hw_cbs = '1'};
    
    end block;

    -- <RELEASE_ON>
end architecture;