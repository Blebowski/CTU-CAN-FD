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
-- Purpose:
--    Top-level entity for evaluating of synthesis with parity extensions in
--    Libero.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity ctu_can_fd_libero_top is
    port (
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys     : in std_logic;
        
        -- Asynchronous reset
        res_n       : in std_logic;

        -- Synchronized reset
        res_n_out   : out std_logic;

        -----------------------------------------------------------------------
        -- Memory interface
        -----------------------------------------------------------------------
        -- Input data
        data_in     : in  std_logic_vector(31 downto 0);
        
        -- Output data
        data_out    : out std_logic_vector(31 downto 0);
        
        -- Address
        adress      : in  std_logic_vector(15 downto 0);
        
        -- Chip select
        scs         : in  std_logic;
        
        -- Read indication
        srd         : in  std_logic;
        
        -- Write indication
        swr         : in  std_logic;
        
        -- Byte enable
        sbe         : in  std_logic_vector(3 downto 0);
        
        -----------------------------------------------------------------------
        -- Interrupt Interface
        -----------------------------------------------------------------------
        -- Interrupt output
        int         : out std_logic;

        -----------------------------------------------------------------------
        -- CAN Bus Interface
        -----------------------------------------------------------------------
        -- TX signal to CAN bus
        can_tx      : out std_logic;
        
        -- RX signal from CAN bus
        can_rx      : in  std_logic;

        -----------------------------------------------------------------------
        -- Timestamp for time based transmission / reception
        -----------------------------------------------------------------------
        timestamp    : in std_logic_vector(63 downto 0)
  );
end entity ctu_can_fd_libero_top;

architecture rtl of ctu_can_fd_libero_top is
     
begin

    can_top_level_inst : entity ctu_can_fd_rtl.can_top_level 
    generic map (
        -- RX Buffer RAM size (32 bit words)
        rx_buffer_size          => 128,

        -- Number of supported TXT buffers
        txt_buffer_count        => 8,

        -- Synthesize Filter A
        sup_filtA               => true,
        
        -- Synthesize Filter B
        sup_filtB               => true,
        
        -- Synthesize Filter C
        sup_filtC               => true,
        
        -- Synthesize Range Filter
        sup_range               => true,
        
        -- Synthesize Test registers
        sup_test_registers      => true,
        
        -- Insert Traffic counters
        sup_traffic_ctrs        => true,

        -- Add parity bit to TXT Buffer and RX Buffer RAMs
        sup_parity              => true,

        -- Number of active timestamp bits
        active_timestamp_bits   => 63,

        -- Reset TXT / RX Buffer RAMs
        reset_buffer_rams       => true,

        -- Target technology (ASIC or FPGA)
        target_technology       => C_TECH_FPGA
    )
    port map (
        -- Clock and Asynchronous reset
        clk_sys         => clk_sys,
        res_n           => res_n,
        res_n_out       => res_n_out,

        -- DFT support
        scan_enable     => '0',

        -- Memory interface
        data_in         => data_in,
        data_out        => data_out,
        adress          => adress,
        scs             => scs,
        srd             => srd,
        swr             => swr,
        sbe             => sbe,
        
        -- Interrupt Interface
        int             => int,

        -- CAN Bus Interface
        can_tx          => can_tx,
        can_rx          => can_rx,

        -- Debug signals for testbench
        test_probe      => open,

        -- Timestamp for time based transmission / reception
        timestamp       => timestamp
    );

end architecture rtl;