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
--  TXT Buffer RAM
--
-- Purpose:
--  Wrapper for dual port RAM in TXT Buffer.
--
-- Memory parameters:
--  Depth: 20
--  Word size: 32 bits
--  Read: Synchronous
--  Write: Synchronous
--  Port A: Write Only
--  Port B: Read only
--
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

entity txt_buffer_ram is
    generic(
        -- Reset polarity
        G_RESET_POLARITY       :     std_logic := '0';

        -- TXT buffer ID
        G_ID                   :     natural
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
        -- Memory Testability
        ------------------------------------------------------------------------
        -- Test registers
        test_registers_out     :in   test_registers_out_t;
        
        -- TXT buffer RAM test output
        tst_rdata_txt_buf      :out  std_logic_vector(31 downto 0);

        ------------------------------------------------------------------------
        -- Port A - Write (from Memory registers)
        ------------------------------------------------------------------------
        -- Address
        port_a_address       :in     std_logic_vector(4 downto 0);
        
        -- Data
        port_a_data_in       :in     std_logic_vector(31 downto 0);
        
        -- Write signal
        port_a_write         :in     std_logic;

        -----------------------------------------------------------------------
        -- Port B - Read (from CAN Core)
        -----------------------------------------------------------------------
        -- Address
        port_b_address       :in     std_logic_vector(4 downto 0);
        
        -- Data
        port_b_data_out      :out    std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of txt_buffer_ram is
    
    signal port_a_address_i      : std_logic_vector(4 downto 0);
    signal port_a_write_i        : std_logic;
    signal port_a_data_in_i      : std_logic_vector(31 downto 0);

    signal port_b_address_i      : std_logic_vector(4 downto 0);
    signal port_b_data_out_i     : std_logic_vector(31 downto 0);
    
    signal tst_ena               : std_logic;
    signal tst_addr              : std_logic_vector(15 downto 0);
begin
    
    ---------------------------------------------------------------------------
    -- RAM is implemented as synchronous inferred RAM for FPGAs.
    -- Synchronous RAM is chosen since some FPGA families does not provide
    -- inferred RAM for asynchronously read data (in the same clock cycle).
    ---------------------------------------------------------------------------
    txt_buf_ram_inst : inf_RAM_wrapper 
    generic map (
        G_WORD_WIDTH           => 32,
        G_DEPTH                => 20,
        G_ADDRESS_WIDTH        => port_a_address'length,
        G_RESET_POLARITY       => G_RESET_POLARITY,
        G_SYNC_READ            => true
    )
    port map(
        clk_sys              => clk_sys,
        res_n                => res_n,
        
        addr_A               => port_a_address_i,
        write                => port_a_write_i, 
        data_in              => port_a_data_in_i,
        
        addr_B               => port_b_address_i,
        data_out             => port_b_data_out_i
    );
    port_b_data_out <= port_b_data_out_i;

    ---------------------------------------------------------------------------
    -- Memory testability
    ---------------------------------------------------------------------------
    tst_ena <=
        '1' when (test_registers_out.tst_control(TMAENA_IND) = '1') and
                 (test_registers_out.tst_dest(TST_MTGT_H downto TST_MTGT_L) = 
                  std_logic_vector(to_unsigned(G_ID + 2, 4)))
            else
        '0';

    tst_addr <= test_registers_out.tst_dest(TST_ADDR_H downto TST_ADDR_L);

    -- Write port
    port_a_address_i <= port_a_address when (tst_ena = '0') else
                        tst_addr(4 downto 0);

    port_a_write_i <= port_a_write when (tst_ena = '0') else
                      test_registers_out.tst_control(TWRSTB_IND);

    port_a_data_in_i <= port_a_data_in when (tst_ena = '0') else
                        test_registers_out.tst_wdata;

    -- Read port
    port_b_address_i <= port_b_address when (tst_ena = '0') else
                        tst_addr(4 downto 0);

    tst_rdata_txt_buf <= port_b_data_out_i when (tst_ena = '1') else
                         (OTHERS => '0');

    ---------------------------------------------------------------------------
    -- Assertions and functional coverage
    ---------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl txt_ram_0_test_cov : cover
    --   {tst_ena = '0' and test_registers_out.tst_control(TWRSTB_IND) = '1' and G_ID = 0};
    -- psl txt_ram_1_test_cov : cover
    --   {tst_ena = '0' and test_registers_out.tst_control(TWRSTB_IND) = '1' and G_ID = 1};
    -- psl txt_ram_2_test_cov : cover
    --   {tst_ena = '0' and test_registers_out.tst_control(TWRSTB_IND) = '1' and G_ID = 2};
    -- psl txt_ram_3_test_cov : cover
    --   {tst_ena = '0' and test_registers_out.tst_control(TWRSTB_IND) = '1' and G_ID = 3};
    -- psl txt_ram_4_test_cov : cover
    --   {tst_ena = '0' and test_registers_out.tst_control(TWRSTB_IND) = '1' and G_ID = 4};
    -- psl txt_ram_5_test_cov : cover
    --   {tst_ena = '0' and test_registers_out.tst_control(TWRSTB_IND) = '1' and G_ID = 5};
    -- psl txt_ram_6_test_cov : cover
    --   {tst_ena = '0' and test_registers_out.tst_control(TWRSTB_IND) = '1' and G_ID = 6};
    -- psl txt_ram_7_test_cov : cover
    --   {tst_ena = '0' and test_registers_out.tst_control(TWRSTB_IND) = '1' and G_ID = 7};
    

end architecture;