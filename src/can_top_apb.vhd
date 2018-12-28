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
--    Top-level entity using APB4.
--------------------------------------------------------------------------------
-- Revision History:
--    May 2018   First Implementation
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;

entity CTU_CAN_FD_v1_0 is
    generic(
        use_logger       : boolean                := true;
        rx_buffer_size   : natural range 4 to 512 := 128;
        use_sync         : boolean                := true;
        sup_filtA        : boolean                := true;
        sup_filtB        : boolean                := true;
        sup_filtC        : boolean                := true;
        sup_range        : boolean                := true;
        logger_size      : natural range 0 to 512 := 8
    );
    port(
        aclk             : in  std_logic;
        arstn            : in  std_logic;

        irq              : out std_logic;
        CAN_tx           : out std_logic;
        CAN_rx           : in  std_logic;
        time_quanta_clk  : out std_logic;
        timestamp        : in std_logic_vector(63 downto 0);

        -- Ports of APB4
        s_apb_paddr      : in  std_logic_vector(31 downto 0);
        s_apb_penable    : in  std_logic;
        s_apb_pprot      : in  std_logic_vector(2 downto 0);
        s_apb_prdata     : out std_logic_vector(31 downto 0);
        s_apb_pready     : out std_logic;
        s_apb_psel       : in  std_logic;
        s_apb_pslverr    : out std_logic;
        s_apb_pstrb      : in  std_logic_vector(3 downto 0);
        s_apb_pwdata     : in  std_logic_vector(31 downto 0);
        s_apb_pwrite     : in  std_logic
  );
end entity CTU_CAN_FD_v1_0;

architecture rtl of CTU_CAN_FD_v1_0 is
 
    signal reg_data_in      : std_logic_vector(31 downto 0);
    signal reg_data_out     : std_logic_vector(31 downto 0);
    signal reg_addr         : std_logic_vector(23 downto 0);
    signal reg_be           : std_logic_vector(3 downto 0);
    signal reg_rden         : std_logic;
    signal reg_wren         : std_logic;
begin

    i_can : CAN_top_level
        generic map (
            use_logger      => use_logger,
            rx_buffer_size  => rx_buffer_size,
            use_sync        => use_sync,
            sup_filtA       => sup_filtA,
            sup_filtB       => sup_filtB,
            sup_filtC       => sup_filtC,
            sup_range       => sup_range,
            logger_size     => logger_size
        )
        port map (
            clk_sys         => aclk,
            res_n           => arstn,

            data_in         => reg_data_in,
            data_out        => reg_data_out,
            adress          => reg_addr,
            scs             => '1',
            srd             => reg_rden,
            swr             => reg_wren,
            sbe             => reg_be,

            int             => irq,

            CAN_tx          => CAN_tx,
            CAN_rx          => CAN_rx,

            time_quanta_clk => time_quanta_clk,
            timestamp       => timestamp
        );

    i_apb : apb_ifc
        port map (
            aclk           => aclk,
            arstn          => arstn,

            reg_data_in_o  => reg_data_in,
            reg_data_out_i => reg_data_out,
            reg_addr_o     => reg_addr,
            reg_be_o       => reg_be,
            reg_rden_o     => reg_rden,
            reg_wren_o     => reg_wren,

            s_apb_paddr    => s_apb_paddr,
            s_apb_penable  => s_apb_penable,
            s_apb_pprot    => s_apb_pprot,
            s_apb_prdata   => s_apb_prdata,
            s_apb_pready   => s_apb_pready,
            s_apb_psel     => s_apb_psel,
            s_apb_pslverr  => s_apb_pslverr,
            s_apb_pstrb    => s_apb_pstrb,
            s_apb_pwdata   => s_apb_pwdata,
            s_apb_pwrite   => s_apb_pwrite
        );
end architecture rtl;
