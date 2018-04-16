library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
library work;
use ieee.std_logic_1164.all;
USE work.CANtestLib.All;
USE work.randomlib.All;

entity tb_axi_ifc is
  generic (
    runner_cfg : string;
    C_S_AXI_DATA_WIDTH  : integer  := 32;
    C_S_AXI_ADDR_WIDTH  : integer  := 8;
    NITERS : positive := 100
  );
end entity;

architecture tb of tb_axi_ifc is
  type axi_aw_t is record
    M_AWADDR  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    M_AWPROT  : std_logic_vector(2 downto 0);
    M_AWVALID : std_logic;
    S_AWREADY : std_logic;
  end record;

  type axi_w_t is record
    M_WDATA   : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    M_WSTRB   : std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
    M_WVALID  : std_logic;
    S_WREADY  : std_logic;
  end record;

  type axi_b_t is record
    S_BRESP   : std_logic_vector(1 downto 0);
    S_BVALID  : std_logic;
    M_BREADY  : std_logic;
  end record;

  type axi_ar_t is record
    M_ARADDR  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    M_ARPROT  : std_logic_vector(2 downto 0);
    M_ARVALID : std_logic;
    S_ARREADY : std_logic;
  end record;

  type axi_r_t is record
    S_RDATA   : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    S_RRESP   : std_logic_vector(1 downto 0);
    S_RVALID  : std_logic;
    M_RREADY  : std_logic;
  end record;

  component axi_ifc is
    generic (
      -- Width of S_AXI data bus
      C_S_AXI_DATA_WIDTH  : integer  := 32;
      -- Width of S_AXI address bus
      C_S_AXI_ADDR_WIDTH  : integer  := 8
    );
    port (
      reg_data_in_o    : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      reg_data_out_i   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      reg_addr_o       : out std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
      reg_be_o         : out std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
      reg_rden_o       : out std_logic;
      reg_wren_o       : out std_logic;

      S_AXI_ACLK    : in  std_logic;
      S_AXI_ARESETN : in  std_logic;

      S_AXI_AWADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
      S_AXI_AWPROT  : in  std_logic_vector(2 downto 0);
      S_AXI_AWVALID : in  std_logic;
      S_AXI_AWREADY : out std_logic;

      S_AXI_WDATA   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      S_AXI_WSTRB   : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
      S_AXI_WVALID  : in  std_logic;
      S_AXI_WREADY  : out std_logic;

      S_AXI_BRESP   : out std_logic_vector(1 downto 0);
      S_AXI_BVALID  : out std_logic;
      S_AXI_BREADY  : in  std_logic;

      S_AXI_ARADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
      S_AXI_ARPROT  : in  std_logic_vector(2 downto 0);
      S_AXI_ARVALID : in  std_logic;
      S_AXI_ARREADY : out std_logic;

      S_AXI_RDATA   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      S_AXI_RRESP   : out std_logic_vector(1 downto 0);
      S_AXI_RVALID  : out std_logic;
      S_AXI_RREADY  : in  std_logic
    );
  end component axi_ifc;

  signal axi_aw : axi_aw_t;
  signal axi_w  : axi_w_t;
  signal axi_b  : axi_b_t;
  signal axi_ar : axi_ar_t;
  signal axi_r  : axi_r_t;

  signal AXI_ACLK    : std_logic := '0';
  signal AXI_ARESETN : std_logic := '0'; -- start active

  signal AXI_AWADDR  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal AXI_AWPROT  : std_logic_vector(2 downto 0);
  signal AXI_AWVALID : std_logic;
  signal AXI_AWREADY : std_logic;

  signal AXI_WDATA   : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal AXI_WSTRB   : std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
  signal AXI_WVALID  : std_logic;
  signal AXI_WREADY  : std_logic;

  signal AXI_BRESP   : std_logic_vector(1 downto 0);
  signal AXI_BVALID  : std_logic;
  signal AXI_BREADY  : std_logic;

  signal AXI_ARADDR  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal AXI_ARPROT  : std_logic_vector(2 downto 0);
  signal AXI_ARVALID : std_logic;
  signal AXI_ARREADY : std_logic;

  signal AXI_RDATA   : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal AXI_RRESP   : std_logic_vector(1 downto 0);
  signal AXI_RVALID  : std_logic;
  signal AXI_RREADY  : std_logic;

  signal reg_data_in  : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal reg_data_out : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal reg_addr     : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal reg_be       : std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
  signal reg_rden     : std_logic;
  signal reg_wren     : std_logic;

  signal test_write_en   : boolean := false;
  signal test_write_done : boolean := false;
  signal test_read_en    : boolean := false;
  signal test_read_done  : boolean := false;

  signal rnd_writer : natural range 0 to RAND_POOL_SIZE;
  signal rnd_reader : natural range 0 to RAND_POOL_SIZE;
begin
  AXI_AWADDR  <= axi_aw.M_AWADDR;
  AXI_AWPROT  <= axi_aw.M_AWPROT;
  AXI_AWVALID <= axi_aw.M_AWVALID;

  AXI_WDATA   <= axi_w.M_WDATA;
  AXI_WSTRB   <= axi_w.M_WSTRB;
  AXI_WVALID  <= axi_w.M_WVALID;

  AXI_BREADY  <= axi_b.M_BREADY;

  AXI_ARADDR  <= axi_ar.M_ARADDR;
  AXI_ARPROT  <= axi_ar.M_ARPROT;
  AXI_ARVALID <= axi_ar.M_ARVALID;

  AXI_RREADY  <= axi_r.M_RREADY;

  axi_aw.S_AWREADY <= AXI_AWREADY;
  axi_w.S_WREADY <= AXI_WREADY;

  axi_b.S_BRESP <= AXI_BRESP;
  axi_b.S_BVALID <= AXI_BVALID;

  axi_ar.S_ARREADY <= AXI_ARREADY;

  axi_r.S_RDATA <= AXI_RDATA;
  axi_r.S_RRESP <= AXI_RRESP;
  axi_r.S_RVALID <= AXI_RVALID;

  ifc: axi_ifc
    generic map (
      C_S_AXI_DATA_WIDTH  => C_S_AXI_DATA_WIDTH,
      C_S_AXI_ADDR_WIDTH  => C_S_AXI_ADDR_WIDTH
    )
    port map (
      reg_data_in_o  => reg_data_in,
      reg_data_out_i => reg_data_out,
      reg_addr_o     => reg_addr,
      reg_be_o       => reg_be,
      reg_rden_o     => reg_rden,
      reg_wren_o     => reg_wren,

      S_AXI_ACLK     => AXI_ACLK,
      S_AXI_ARESETN  => AXI_ARESETN,

      S_AXI_AWADDR   => AXI_AWADDR,
      S_AXI_AWPROT   => AXI_AWPROT,
      S_AXI_AWVALID  => AXI_AWVALID,
      S_AXI_AWREADY  => AXI_AWREADY,

      S_AXI_WDATA    => AXI_WDATA,
      S_AXI_WSTRB    => AXI_WSTRB,
      S_AXI_WVALID   => AXI_WVALID,
      S_AXI_WREADY   => AXI_WREADY,

      S_AXI_BRESP    => AXI_BRESP,
      S_AXI_BVALID   => AXI_BVALID,
      S_AXI_BREADY   => AXI_BREADY,

      S_AXI_ARADDR   => AXI_ARADDR,
      S_AXI_ARPROT   => AXI_ARPROT,
      S_AXI_ARVALID  => AXI_ARVALID,
      S_AXI_ARREADY  => AXI_ARREADY,

      S_AXI_RDATA    => AXI_RDATA,
      S_AXI_RRESP    => AXI_RRESP,
      S_AXI_RVALID   => AXI_RVALID,
      S_AXI_RREADY   => AXI_RREADY
    );

  gen_clk:process
  begin
    wait for 10 ns;
    AXI_ACLK <= not AXI_ACLK;
  end process;

  main:process
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      test_read_en <= false;
      test_write_en <= false;

      report "Main: Resetting AXI ...";
      AXI_ARESETN <= '0';
      wait until rising_edge(AXI_ACLK);
      wait until rising_edge(AXI_ACLK);
      wait until falling_edge(AXI_ACLK);
      AXI_ARESETN <= '1';
      report "Main: ... done";

      if run("read_only") then
        test_read_en <= true;
      elsif run("write_only") then
        test_write_en <= true;
      elsif run("read_write") then
        test_read_en <= true;
        test_write_en <= true;
      end if;

      report "Main: waiting until test end ...";
      wait until (not test_read_en  or test_read_done)
             and (not test_write_en or test_write_done);
      report "Main: ... done";
    end loop;

    test_runner_cleanup(runner, false);
  end process;

--  Write cycle:
--     0:
--        {expect reg wren=0}
--        0+x: set axi write address
--        0+y: set axi write data + be
--        0+z: set axi bready
--     +max(x,y,z):
--     if test_en_read: wait until write_precedence or not want_to_read; wait until rising_edge(S_AXI_ACLK)
--        expect reg write address
--        expect reg write data
--        expect reg be
--        expect reg wren=1
--        expect axi bvalid=1
--     +1:
--        expect axi bvalid=0
--        expect reg wren=0
  p_writer: process
    variable cycle : natural := 0;
    variable x, y, z : natural;
    variable max_xy : natural;
    constant MAXCYC  : positive := 2;
    variable exp_addr : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    variable exp_data : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    variable rnd_writer_var : natural range 0 to RAND_POOL_SIZE;
  begin
    if AXI_ARESETN then
      report "Writer: waiting for AXI reset.";
      wait until not AXI_ARESETN;
    end if;
    axi_aw.M_AWVALID <= '0';
    axi_w.M_WVALID <= '0';
    axi_b.M_BREADY <= '1'; -- !!!!
    axi_aw.M_AWADDR <= (others => 'X');
    axi_w.M_WDATA <= (others => 'X');

    --report "Writer: waiting for not AXI reset.";
    wait until AXI_ARESETN;
    wait until rising_edge(AXI_ACLK);

    report "Writer: waiting until enabled.";
    if not test_write_en then
      wait until test_write_en;
    end if;
    test_write_done <= false;
    rnd_writer <= 0;

    for i in 1 to NITERS loop
      -- check init / last i
      report "Writer: iteration " & integer'image(i);
      rnd_writer_var := rnd_writer;
      rand_int_vv_nowait(rnd_writer_var, MAXCYC, x);
      rand_int_vv_nowait(rnd_writer_var, MAXCYC, y);
--       rand_int_vv_nowait(rnd_writer_var, MAXCYC, z);
      rnd_writer <= rnd_writer_var;
      max_xy := x;
      max_xy := y when y > max_xy else max_xy;

      report "Write: cycle count {aw, w, bready} = {"&natural'image(x)&", "&natural'image(y)&", "&natural'image(z)&"}";
      cycle := 0;
      loop
        if cycle = x then
          report "Writer: generating AWADDR";
          -- generate AWADDR
          rnd_writer_var := rnd_writer;
          rand_logic_vect_vs_nowait(rnd_writer_var, axi_aw.M_AWADDR, 0.5);
          rnd_writer <= rnd_writer_var;
          axi_aw.M_AWVALID <= '1';
          axi_aw.M_AWPROT <= "000";
        end if;
        if cycle = y then
          report "Writer: generating WDATA & WSTRB";
          -- generate WDATA & WSTRB
          rnd_writer_var := rnd_writer;
          rand_logic_vect_vs_nowait(rnd_writer_var, axi_w.M_WDATA, 0.5);
          rand_logic_vect_vs_nowait(rnd_writer_var, axi_w.M_WSTRB, 0.5);
          rnd_writer <= rnd_writer_var;
          axi_w.M_WVALID <= '1';
        end if;
--         if cycle = z then
--           --report "Writer: asserting BREADY";
--           report "Writer: BREADY already on";
--           axi_b.M_BREADY <= '1';
--         end if;

        if cycle <= max_xy then
          -- what can be set is set, and we have to wait out the cycle from previous iteration
          if cycle = 0 then
            wait for 1 ns;
          end if;
          -- if there was no delay, the signals are still asserted
          if not (cycle = 0 and cycle = max_xy) then
            assert reg_wren = '0';
          end if;
        elsif cycle = max_xy+1 then
          exp_addr := axi_aw.M_AWADDR;
          exp_data := axi_w.M_WDATA;

          axi_aw.M_AWVALID <= '0';
          axi_w.M_WVALID <= '0';
          --axi_b.M_BREADY <= '0';
          axi_aw.M_AWADDR <= (others => 'X');
          axi_w.M_WDATA <= (others => 'X');

          assert reg_wren = '1';
          assert reg_data_in = exp_data report "WDATA mismatch: expected " & to_string(exp_data) & ", got " & to_string(reg_data_in);
          assert reg_addr = exp_addr report "AWADDR mismatch: expected " & to_string(exp_addr) & ", got " & to_string(reg_addr);
          assert reg_be = axi_w.M_WSTRB;
          assert axi_w.S_WREADY = '1';
          assert axi_b.S_BVALID = '1';

          --assert reg_wren = '0';
          --assert axi_b.S_BVALID = '0';
          report "Writer: cycle done";
          exit;
        end if;

        wait until rising_edge(AXI_ACLK);
        cycle := cycle + 1;
      end loop;
    end loop;
    test_write_done <= true;
  end process;

--  Read cycle
--     0:
--        {expect reg rden=0}
--        0+x: set axi read address
--        0+y: set axi rready=1
--     +max(x,y):
--     if test_en_read: wait until not write_precedence or not want_to_write; wait until rising_edge(S_AXI_ACLK)
--        expect reg rden=0
--     +1:
--        expect reg read address
--        expect reg rden=1
--        set reg rdata
--     +1:
--        expect axi rdata
--        expect reg rden=0
--        test_write_inprg := false;
  p_reader: process
    variable cycle : natural := 0;
    variable x, y : natural;
    variable max_x : natural;
    constant MAXCYC  : positive := 2;
    variable exp_addr : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    variable exp_data : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    variable rnd_reader_var : natural range 0 to RAND_POOL_SIZE;
  begin
    if AXI_ARESETN then
      report "Reader: waiting for AXI reset.";
      wait until not AXI_ARESETN;
    end if;
    axi_ar.M_ARADDR <= (others => 'X');
    axi_ar.M_ARVALID <= '0';
    axi_r.M_RREADY <= '1'; -- !!!

    --report "Reader: waiting for not AXI reset.";
    wait until AXI_ARESETN;
    wait until rising_edge(AXI_ACLK);

    report "Reader: waiting until enabled.";
    if not test_read_en then
      wait until test_read_en;
    end if;
    test_read_done <= false;
    rnd_reader <= RAND_POOL_SIZE/2;

    for i in 1 to NITERS loop
      -- check init / last i
      reg_data_out <= (others => 'X');
      assert reg_rden = '0';
      report "Reader: iteration " & integer'image(i);
      rnd_reader_var := rnd_reader;
      rand_int_vv_nowait(rnd_reader_var, MAXCYC, x);
      rand_int_vv_nowait(rnd_reader_var, MAXCYC, y);
--       rand_int_vv_nowait(rnd_reader_var, MAXCYC, z);
      rnd_reader <= rnd_reader_var;
      max_x := x;
      --max_xy := y when y > max_xy else max_xy;

      report "Read: cycle count {ar, rready} = {"&natural'image(x)&", "&natural'image(y)&"}";
      cycle := 0;
      loop
        if cycle = x then
          report "Reader: generating ARADDR";
          -- generate ARADDR
          rnd_reader_var := rnd_reader;
          rand_logic_vect_vs_nowait(rnd_reader_var, axi_ar.M_ARADDR, 0.5);
          rnd_reader <= rnd_reader_var;
          axi_ar.M_ARVALID <= '1';
          axi_ar.M_ARPROT <= "000";
          -- generate data
          -- the read interface is asynchronous -> the data must already be there on next cycle
          rnd_reader_var := rnd_reader;
          rand_logic_vect_vs_nowait(rnd_reader_var, reg_data_out, 0.5);
          rnd_reader <= rnd_reader_var;
        end if;
        if cycle = y then
          report "Reader: RREADY always on";
          --axi_r.M_RREADY <= '1';
        end if;

        if cycle <= max_x then
          -- what can be set is set, and we have to wait out the cycle from previous iteration
          if cycle = 0 then
            wait for 1 ns;
          end if;
          -- if there was no delay, the signals are still asserted
          if (cycle = 0 and cycle = max_x) then
            -- insert waitcycle
            max_x := max_x + 1;
            assert reg_rden = '0';
            assert axi_r.S_RVALID = '0';
          else
            assert reg_rden = '0';
            assert axi_r.S_RVALID = '0';
          end if;
        elsif cycle = max_x+1 then
          assert reg_rden = '1';
          --wait for 1 ns;
          assert reg_addr = axi_ar.M_ARADDR;
          -- async signals
          assert axi_r.S_RVALID = '1';
          assert axi_r.S_RRESP = "000";
          assert axi_r.S_RDATA = reg_data_out;

          axi_ar.M_ARVALID <= '0';
          --axi_r.M_RREADY <= '0';
          axi_ar.M_ARADDR <= (others => 'X');
          report "Reader: cycle done";
          exit;
        end if;

        wait until rising_edge(AXI_ACLK);
        cycle := cycle + 1;
      end loop;
    end loop;
    test_read_done <= true;
  end process;
end architecture;
