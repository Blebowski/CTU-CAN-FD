--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Project advisors and co-authors:
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--    Simple test of can_top_axi, performing register write via AXI.
--------------------------------------------------------------------------------
-- Revision History:
--    April 2018   First Implementation - Martin Jerabek
--------------------------------------------------------------------------------

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
library work;
use ieee.std_logic_1164.all;
USE work.CANtestLib.All;
USE work.CANconstants.All;
use work.CAN_FD_register_map.all;

entity tb_can_top_axi_write is
  generic (
    runner_cfg : string
  );
end entity;

architecture tb of tb_can_top_axi_write is
  component CTU_CAN_FD_v1_0 is
    generic(
      use_logger       : boolean                := true;
      rx_buffer_size   : natural range 4 to 512 := 128;
      use_sync         : boolean                := true;
      sup_filtA        : boolean                := true;
      sup_filtB        : boolean                := true;
      sup_filtC        : boolean                := true;
      sup_range        : boolean                := true;
      tx_time_sup      : boolean                := true;
      sup_be           : boolean                := true;
      logger_size      : natural range 0 to 512 := 8
    );
    port(
      -- system clock and reset from AXI
      irq              : out std_logic;
      CAN_tx           : out std_logic;
      CAN_rx           : in  std_logic;
      time_quanta_clk  : out std_logic;
      timestamp        : in std_logic_vector(63 downto 0);

      -- Ports of Axi Slave Bus Interface S00_AXI
      s00_axi_aclk     : in  std_logic;
      s00_axi_aresetn  : in  std_logic;
      s00_axi_awaddr   : in  std_logic_vector(23 downto 0);
      s00_axi_awprot   : in  std_logic_vector(2 downto 0);
      s00_axi_awvalid  : in  std_logic;
      s00_axi_awready  : out std_logic;
      s00_axi_wdata    : in  std_logic_vector(31 downto 0);
      s00_axi_wstrb    : in  std_logic_vector(3 downto 0);
      s00_axi_wvalid   : in  std_logic;
      s00_axi_wready   : out std_logic;
      s00_axi_bresp    : out std_logic_vector(1 downto 0);
      s00_axi_bvalid   : out std_logic;
      s00_axi_bready   : in  std_logic;
      s00_axi_araddr   : in  std_logic_vector(23 downto 0);
      s00_axi_arprot   : in  std_logic_vector(2 downto 0);
      s00_axi_arvalid  : in  std_logic;
      s00_axi_arready  : out std_logic;
      s00_axi_rdata    : out std_logic_vector(31 downto 0);
      s00_axi_rresp    : out std_logic_vector(1 downto 0);
      s00_axi_rvalid   : out std_logic;
      s00_axi_rready   : in  std_logic
    );
  end component CTU_CAN_FD_v1_0;
  signal axi_clk : std_logic := '0';
  signal axi_rstn : std_logic := '0';
  signal axi_awaddr : std_logic_vector(23 downto 0);
  signal axi_awvalid : std_logic;
  signal axi_awready : std_logic;
  signal axi_wdata : std_logic_vector(31 downto 0);
  signal axi_wvalid : std_logic;
  signal axi_wready : std_logic;
  signal axi_bresp : std_logic_vector(1 downto 0);
  signal axi_bvalid : std_logic;
  signal axi_wstrb : std_logic_vector(3 downto 0);

  signal axi_araddr : std_logic_vector(23 downto 0);
  signal axi_arvalid : std_logic;
  signal axi_arready : std_logic;
  signal axi_rdata : std_logic_vector(31 downto 0);
  signal axi_rvalid : std_logic;

  signal start, stop : boolean := false;
begin
  can: CTU_CAN_FD_v1_0
    generic map (
      use_logger => false,
      use_sync    => false,
      sup_filtA   => false,
      sup_filtB   => false,
      sup_filtC   => false,
      sup_range   => false,
      tx_time_sup => false,
      sup_be      => true
    )
    port map (
      CAN_rx           => '1',
      timestamp        => (others => '0'),

      -- Ports of Axi Slave Bus Interface S00_AXI
      s00_axi_aclk     => axi_clk,
      s00_axi_aresetn  => axi_rstn,
      s00_axi_awaddr   => axi_awaddr,
      s00_axi_awprot   => (others => '0'),
      s00_axi_awvalid  => axi_awvalid,
      s00_axi_wdata    => axi_wdata,
      s00_axi_wstrb    => axi_wstrb,
      s00_axi_wvalid   => axi_wvalid,
      s00_axi_wready   => axi_wready,
      s00_axi_bready   => '1',
      s00_axi_bvalid   => axi_bvalid,

      s00_axi_araddr   => axi_araddr,
      s00_axi_arprot   => (others => '0'),
      s00_axi_arvalid  => axi_arvalid,
      s00_axi_arready  => axi_arready,
      s00_axi_rdata    => axi_rdata,
      s00_axi_rvalid   => axi_rvalid,
      s00_axi_rready   => '1'
    );

  clk:process
  begin
    wait for 100 ns;
    axi_clk <= not axi_clk;
  end process;

  main:process
    procedure axi_write_nowait(constant addr : std_logic_vector(11 downto 0);
                               constant data : std_logic_vector(31 downto 0);
                               constant be   : std_logic_vector(3 downto 0)) is
    begin
      axi_awaddr <= (others => '0');
      axi_awaddr(11 downto 0) <= addr;
      axi_awvalid <= '1';
      axi_wdata <= data;
      axi_wvalid <= '1';
      axi_wstrb <= be;
    end procedure;
    procedure axi_write(constant addr : std_logic_vector(11 downto 0);
                        constant data : std_logic_vector(31 downto 0);
                        constant be   : std_logic_vector(3 downto 0)) is
    begin
      axi_write_nowait(addr, data, be);
      l: while true loop
        wait until axi_clk = '1';
        if axi_bvalid = '1' then
          exit l;
        end if;
      end loop;
      axi_awvalid <= '0';
      axi_wvalid <= '0';
      axi_awaddr(11 downto 0) <= (others => 'U');
      axi_wdata <= (others => 'U');
      axi_wstrb <= (others => 'U');
    end procedure;

    procedure axi_read(constant addr : std_logic_vector(11 downto 0)) is
    begin
      axi_araddr <= (others => '0');
      axi_araddr(11 downto 0) <= addr;
      axi_arvalid <= '1';
      l: while true loop
        wait until axi_clk = '1';
        if axi_rvalid then
          exit l;
        end if;
      end loop;
      axi_arvalid <= '0';
      axi_araddr <= (others => 'U');
    end procedure;

    procedure axi_test_pattern(constant addr : std_logic_vector(11 downto 0);
                               constant data : std_logic_vector(31 downto 0)) is
    begin
      axi_write(addr, data, b"1111");
      axi_read(addr);
      assert axi_rdata = data report "pattern " & to_hstring(data) & " mismatch: got " & to_hstring(axi_rdata);
    end procedure;
  begin
    wait until start;
    axi_rstn <= '0';
    axi_awvalid <= '0';
    axi_wvalid <= '0';
    axi_wstrb <= (others => '0');
    axi_arvalid <= '0';
    wait for 400 ns;
    wait until axi_clk = '1';
    axi_rstn <= '1';

    -- must wait 2 cycles after reset!
    wait until axi_clk = '1';
    wait until axi_clk = '1';

    axi_write(BTR_ADR, x"FFFFFFFF", b"1111");
    axi_read(DEVICE_ID_ADR);
    assert axi_rdata = x"0201CAFD" report "CAN ID reg mismatch";
    axi_read(BTR_ADR);
    assert axi_rdata = x"FFFFFFFF" report "readback mismatch";

    axi_write(BTR_ADR, x"00000000", b"0011");
    axi_read(BTR_ADR);
    assert axi_rdata = x"FFFF0000" report "write low word: readback mismatch";

    axi_write(BTR_ADR, x"FFFFFFFF", b"1111");
    axi_write(BTR_ADR, x"00000000", b"1100");
    axi_read(BTR_ADR);
    assert axi_rdata = x"0000FFFF" report "write high word: readback mismatch";

    axi_test_pattern(BTR_ADR, x"AAAAAAAA");
    axi_test_pattern(BTR_ADR, x"55555555");
    axi_test_pattern(BTR_ADR, x"92492492");
    axi_test_pattern(BTR_ADR, x"49249249");
    axi_test_pattern(BTR_ADR, x"24924924");

    axi_write(BTR_ADR, x"87654321", b"1111");
    axi_read(BTR_ADR);
    assert axi_rdata = x"87654321" report "readback mismatch";
    axi_write(BTR_ADR, x"000055aa", b"0011");
    axi_read(BTR_ADR);
    assert axi_rdata = x"876555aa" report "write low word: readback mismatch";


    wait until axi_clk;
    wait until axi_clk;
    wait until axi_clk;
    wait until axi_clk;

    stop <= true;
  end process;


  test:process
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("txb") then
        start <= true;
        wait until stop;
      end if;
    end loop;

    test_runner_cleanup(runner, false);
  end process;
end architecture;
