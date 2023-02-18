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
--  Interrupt Manager
--
-- Purpose:
--  Captures interrupts to Interrupt vector. Maintains Interrupt Enable and
--  Interrupt Mask registers. Configured (Set and Clear) from Memory registers.
--  An edge is detected on each interrupt source. If an interrupt is unmasked,
--  it is captured to interrupt vector. If an interrupt is enabled and it is
--  active in Interrupt vector, it causes Interrupt output to be asserted.
--  Interrupt output is pipelined to make sure it is glitch free.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity int_manager is
    generic (
        -- Number of supported interrupts
        G_INT_COUNT                     :     natural;

        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT              :     natural
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                         : in  std_logic;
        res_n                           : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Interrupt sources
        -------------------------------------------------------------------------------------------
        -- Error appeared
        err_detected                    : in  std_logic;

        -- Fault confinement state changed
        fcs_changed                     : in  std_logic;

        -- Error warning limit reached
        err_warning_limit_pulse         : in  std_logic;

        -- Arbitration was lost input
        arbitration_lost                : in  std_logic;

        -- Transmitted frame is valid
        tran_valid                      : in  std_logic;

        -- Bit Rate Was Shifted
        br_shifted                      : in  std_logic;

        -- Rx Buffer data overrun
        rx_data_overrun                 : in  std_logic;

        -- Received frame is valid
        rec_valid                       : in  std_logic;

        -- RX Buffer is full
        rx_full                         : in  std_logic;

        -- Recieve buffer is empty
        rx_empty                        : in  std_logic;

        -- HW command on TXT Buffers interrupt
        txtb_hw_cmd_int                 : in  std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        -- Overload frame is being transmitted
        is_overload                     : in  std_logic;

        ------------------------------------------------------------------------
        -- Memory registers Interface
        ------------------------------------------------------------------------
        -- Writes to Interrupt registers
        mr_int_ena_set_int_ena_set      : in  std_logic_vector(G_INT_COUNT - 1 downto 0);
        mr_int_ena_clr_int_ena_clr      : in  std_logic_vector(G_INT_COUNT - 1 downto 0);
        mr_int_mask_set_int_mask_set    : in  std_logic_vector(G_INT_COUNT - 1 downto 0);
        mr_int_mask_clr_int_mask_clr    : in  std_logic_vector(G_INT_COUNT - 1 downto 0);

        mr_int_stat_rxi_i               : in  std_logic;
        mr_int_stat_txi_i               : in  std_logic;
        mr_int_stat_ewli_i              : in  std_logic;
        mr_int_stat_doi_i               : in  std_logic;
        mr_int_stat_fcsi_i              : in  std_logic;
        mr_int_stat_ali_i               : in  std_logic;
        mr_int_stat_bei_i               : in  std_logic;
        mr_int_stat_ofi_i               : in  std_logic;
        mr_int_stat_rxfi_i              : in  std_logic;
        mr_int_stat_bsi_i               : in  std_logic;
        mr_int_stat_rbnei_i             : in  std_logic;
        mr_int_stat_txbhci_i            : in  std_logic;

        -- Reads from Interrupt registers
        mr_int_stat_rxi_o               : out std_logic;
        mr_int_stat_txi_o               : out std_logic;
        mr_int_stat_ewli_o              : out std_logic;
        mr_int_stat_doi_o               : out std_logic;
        mr_int_stat_fcsi_o              : out std_logic;
        mr_int_stat_ali_o               : out std_logic;
        mr_int_stat_bei_o               : out std_logic;
        mr_int_stat_ofi_o               : out std_logic;
        mr_int_stat_rxfi_o              : out std_logic;
        mr_int_stat_bsi_o               : out std_logic;
        mr_int_stat_rbnei_o             : out std_logic;
        mr_int_stat_txbhci_o            : out std_logic;

        mr_int_ena_set_int_ena_set_o    : out std_logic_vector(G_INT_COUNT - 1 downto 0);
        mr_int_mask_set_int_mask_set_o  : out std_logic_vector(G_INT_COUNT - 1 downto 0);

        -------------------------------------------------------------------------------------------
        -- Interrupt output
        -------------------------------------------------------------------------------------------
        int                             : out std_logic
    );
end entity;

architecture rtl of int_manager is

    signal int_input_active                 : std_logic_vector(G_INT_COUNT - 1 downto 0);
    signal int_status_i                     : std_logic_vector(G_INT_COUNT - 1 downto 0);
    signal int_status_clr_i                 : std_logic_vector(G_INT_COUNT - 1 downto 0);

    constant C_ZERO_MASK                    : std_logic_vector(G_INT_COUNT - 1 downto 0) := (others => '0');

    -- Internal value of an interrupt
    signal int_i                            : std_logic;

    -- Internal value of INT_ENA
    signal mr_int_ena_set_int_ena_set_o_i   : std_logic_vector(G_INT_COUNT - 1 downto 0);

begin

    -----------------------------------------------------------------------------------------------
    -- Driving Interrupt output when there is at least one active interrupt enabled.
    -----------------------------------------------------------------------------------------------
    int_i  <= '0' when ((int_status_i and mr_int_ena_set_int_ena_set_o_i) = C_ZERO_MASK)
                  else
              '1';

    -----------------------------------------------------------------------------------------------
    -- Unfold individual bits of interrupt signals
    -----------------------------------------------------------------------------------------------
    -- Input interrupt sources from other modules
    int_input_active(RXI_IND)       <= rec_valid;
    int_input_active(TXI_IND)       <= tran_valid;
    int_input_active(EWLI_IND)      <= err_warning_limit_pulse;
    int_input_active(DOI_IND)       <= rx_data_overrun;
    int_input_active(FCSI_IND)      <= fcs_changed;
    int_input_active(ALI_IND)       <= arbitration_lost;
    int_input_active(BEI_IND)       <= err_detected;
    int_input_active(OFI_IND)       <= is_overload;
    int_input_active(RXFI_IND)      <= rx_full;
    int_input_active(BSI_IND)       <= br_shifted;
    int_input_active(RBNEI_IND)     <= not rx_empty;
    int_input_active(TXBHCI_IND)    <= or_reduce(txtb_hw_cmd_int);

    -- INT_STATUS clear
    int_status_clr_i(RXI_IND)       <= mr_int_stat_rxi_i;
    int_status_clr_i(TXI_IND)       <= mr_int_stat_txi_i;
    int_status_clr_i(EWLI_IND)      <= mr_int_stat_ewli_i;
    int_status_clr_i(DOI_IND)       <= mr_int_stat_doi_i;
    int_status_clr_i(FCSI_IND)      <= mr_int_stat_fcsi_i;
    int_status_clr_i(ALI_IND)       <= mr_int_stat_ali_i;
    int_status_clr_i(BEI_IND)       <= mr_int_stat_bei_i;
    int_status_clr_i(OFI_IND)       <= mr_int_stat_ofi_i;
    int_status_clr_i(RXFI_IND)      <= mr_int_stat_rxfi_i;
    int_status_clr_i(BSI_IND)       <= mr_int_stat_bsi_i;
    int_status_clr_i(RBNEI_IND)     <= mr_int_stat_rbnei_i;
    int_status_clr_i(TXBHCI_IND)    <= mr_int_stat_txbhci_i;

    -- Interrupt status
    mr_int_stat_rxi_o               <= int_status_i(RXI_IND);
    mr_int_stat_txi_o               <= int_status_i(TXI_IND);
    mr_int_stat_ewli_o              <= int_status_i(EWLI_IND);
    mr_int_stat_doi_o               <= int_status_i(DOI_IND);
    mr_int_stat_fcsi_o              <= int_status_i(FCSI_IND);
    mr_int_stat_ali_o               <= int_status_i(ALI_IND);
    mr_int_stat_bei_o               <= int_status_i(BEI_IND);
    mr_int_stat_ofi_o               <= int_status_i(OFI_IND);
    mr_int_stat_rxfi_o              <= int_status_i(RXFI_IND);
    mr_int_stat_bsi_o               <= int_status_i(BSI_IND);
    mr_int_stat_rbnei_o             <= int_status_i(RBNEI_IND);
    mr_int_stat_txbhci_o            <= int_status_i(TXBHCI_IND);

    -----------------------------------------------------------------------------------------------
    -- Interrupt module instances
    -----------------------------------------------------------------------------------------------
    int_module_gen : for i in 0 to G_INT_COUNT - 1 generate

        int_module_inst : entity ctu_can_fd_rtl.int_module
        port map (
            clk_sys             => clk_sys,                                  -- IN
            res_n               => res_n,                                    -- IN

            int_status_set      => int_input_active(i),                      -- IN
            int_status_clear    => int_status_clr_i(i),                      -- IN

            int_mask_set        => mr_int_mask_set_int_mask_set(i),          -- IN
            int_mask_clear      => mr_int_mask_clr_int_mask_clr(i),          -- IN

            int_ena_set         => mr_int_ena_set_int_ena_set(i),            -- IN
            int_ena_clear       => mr_int_ena_clr_int_ena_clr(i),            -- IN

            int_status          => int_status_i(i),                          -- OUT
            int_mask            => mr_int_mask_set_int_mask_set_o(i),        -- OUT
            int_ena             => mr_int_ena_set_int_ena_set_o_i(i)         -- OUT
        );

    end generate int_module_gen;

    -----------------------------------------------------------------------------------------------
    -- Output interrupt DFF to make sure that interrupt output will be glitch free!
    -----------------------------------------------------------------------------------------------
    dff_int_output_reg : entity ctu_can_fd_rtl.dff_arst
    generic map(
        G_RESET_POLARITY        => '0',
        G_RST_VAL               => '0'
    )
    port map(
        arst                    => res_n,                                   -- IN
        clk                     => clk_sys,                                 -- IN
        reg_d                   => int_i,                                   -- IN

        reg_q                   => int                                      -- OUT
    );

    -- Propagation to output
    mr_int_ena_set_int_ena_set_o <= mr_int_ena_set_int_ena_set_o_i;


    -- <RELEASE_OFF>
    -----------------------------------------------------------------------------------------------
    -- Functional coverage
    -----------------------------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl rxi_set_cov : cover
    --  {int_status_i(RXI_IND) = '0';int_status_i(RXI_IND) = '1'};

    -- psl rxi_enable_cov : cover
    --  {int_status_i(RXI_IND) = '1' and mr_int_ena_set_int_ena_set_o(RXI_IND) = '1'};


    -- psl txi_set_cov : cover
    --  {int_status_i(TXI_IND) = '0';int_status_i(TXI_IND) = '1'};

    -- psl txi_enable_cov : cover
    --  {int_status_i(TXI_IND) = '1' and mr_int_ena_set_int_ena_set_o(TXI_IND) = '1'};


    -- psl ewli_int_set_cov : cover
    --  {int_status_i(EWLI_IND) = '0';int_status_i(EWLI_IND) = '1'};

    -- psl ewli_enable_cov : cover
    --  {int_status_i(EWLI_IND) = '1' and mr_int_ena_set_int_ena_set_o(EWLI_IND) = '1'};


    -- psl doi_int_set_cov : cover
    --  {int_status_i(DOI_IND) = '0';int_status_i(DOI_IND) = '1'};

    -- psl doi_enable_cov : cover
    --  {int_status_i(DOI_IND) = '1' and mr_int_ena_set_int_ena_set_o(DOI_IND) = '1'};


    -- psl fcsi_int_set_cov : cover
    --  {int_status_i(FCSI_IND) = '0';int_status_i(FCSI_IND) = '1'};

    -- psl fcsi_enable_cov : cover
    --  {int_status_i(FCSI_IND) = '1' and mr_int_ena_set_int_ena_set_o(FCSI_IND) = '1'};


    -- psl ali_int_set_cov : cover
    --  {int_status_i(ALI_IND) = '0';int_status_i(ALI_IND) = '1'};

    -- psl ali_enable_cov : cover
    --  {int_status_i(ALI_IND) = '1' and mr_int_ena_set_int_ena_set_o(ALI_IND) = '1'};


    -- psl beu_int_set_cov : cover
    --  {int_status_i(BEI_IND) = '0';int_status_i(BEI_IND) = '1'};

    -- psl bei_enable_cov : cover
    --  {int_status_i(BEI_IND) = '1' and mr_int_ena_set_int_ena_set_o(BEI_IND) = '1'};


    -- psl rxfi_int_set_cov : cover
    --  {int_status_i(RXFI_IND) = '0';int_status_i(RXFI_IND) = '1'};

    -- psl rxfi_enable_cov : cover
    --  {int_status_i(RXFI_IND) = '1' and mr_int_ena_set_int_ena_set_o(RXFI_IND) = '1'};


    -- psl bsi_int_set_cov : cover
    --  {int_status_i(BSI_IND) = '0';int_status_i(BSI_IND) = '1'};

    -- psl bsi_enable_cov : cover
    --  {int_status_i(BSI_IND) = '1' and mr_int_ena_set_int_ena_set_o(BSI_IND) = '1'};


    -- psl rbnei_int_set_cov : cover
    --  {int_status_i(RBNEI_IND) = '0';int_status_i(RBNEI_IND) = '1'};

    -- psl rbnei_enable_cov : cover
    --  {int_status_i(RBNEI_IND) = '1' and mr_int_ena_set_int_ena_set_o(RBNEI_IND) = '1'};


    -- psl txbhci_int_set_cov : cover
    --  {int_status_i(TXBHCI_IND) = '0';int_status_i(TXBHCI_IND) = '1'};

    -- psl txbhci_enable_cov : cover
    --  {int_status_i(TXBHCI_IND) = '1' and mr_int_ena_set_int_ena_set_o(TXBHCI_IND) = '1'};


    -- psl ofi_int_set_cov : cover
    --  {int_status_i(OFI_IND) = '0';int_status_i(OFI_IND) = '1'};

    -- psl ofi_enable_cov : cover
    --  {int_status_i(OFI_IND) = '1' and mr_int_ena_set_int_ena_set_o(OFI_IND) = '1'};

    -- <RELEASE_ON>

end architecture;