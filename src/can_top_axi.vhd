library ieee;
use ieee.std_logic_1164.all;

entity CTU_CAN_FD_v1_0 is
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
end entity CTU_CAN_FD_v1_0;

architecture rtl of CTU_CAN_FD_v1_0 is
  component CAN_top_level is
    generic(
      constant use_logger     : boolean                := true;
      constant rx_buffer_size : natural range 4 to 512 := 128;
      constant use_sync       : boolean                := true;
      constant ID             : natural range 0 to 15  := 1;
      constant sup_filtA      : boolean                := true;
      constant sup_filtB      : boolean                := true;
      constant sup_filtC      : boolean                := true;
      constant sup_range      : boolean                := true;
      constant tx_time_sup    : boolean                := true;
      constant sup_be         : boolean                := true;
      constant logger_size    : natural range 0 to 512 := 8
    );
    port(
      signal clk_sys  : in std_logic;
      signal res_n    : in std_logic;

      signal data_in  : in  std_logic_vector(31 downto 0);
      signal data_out : out std_logic_vector(31 downto 0);
      signal adress   : in  std_logic_vector(23 downto 0);
      signal scs      : in  std_logic;    --Chip select
      signal srd      : in  std_logic;    --Serial read
      signal swr      : in  std_logic;    --Serial write
      signal sbe      : in  std_logic_vector(3 downto 0);

      signal int      : out std_logic;

      signal CAN_tx   : out std_logic;
      signal CAN_rx   : in  std_logic;

      signal time_quanta_clk : out std_logic;
      signal timestamp : in std_logic_vector(63 downto 0)
    );
  end component;

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

  signal reg_data_in  : std_logic_vector(31 downto 0);
  signal reg_data_out : std_logic_vector(31 downto 0);
  signal reg_addr     : std_logic_vector(23 downto 0);
  signal reg_be       : std_logic_vector(3 downto 0);
  signal reg_rden     : std_logic;
  signal reg_wren     : std_logic;
begin
  i_can: CAN_top_level
    generic map (
      use_logger     => use_logger,
      rx_buffer_size => rx_buffer_size,
      use_sync       => use_sync,
      sup_filtA      => sup_filtA,
      sup_filtB      => sup_filtB,
      sup_filtC      => sup_filtC,
      sup_range      => sup_range,
      tx_time_sup    => tx_time_sup,
      sup_be         => sup_be,
      logger_size    => logger_size
    )
    port map (
      clk_sys         => s00_axi_aclk,
      res_n           => s00_axi_aresetn,

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
  i_axi: axi_ifc
    generic map (
      C_S_AXI_DATA_WIDTH  => 32,
      C_S_AXI_ADDR_WIDTH  => 24
    )
    port map (
      reg_data_in_o  => reg_data_in,
      reg_data_out_i => reg_data_out,
      reg_addr_o     => reg_addr,
      reg_be_o       => reg_be,
      reg_rden_o     => reg_rden,
      reg_wren_o     => reg_wren,

      S_AXI_ACLK     => s00_axi_aclk,
      S_AXI_ARESETN  => s00_axi_aresetn,
      S_AXI_AWADDR   => s00_axi_awaddr,
      S_AXI_AWPROT   => s00_axi_awprot,
      S_AXI_AWVALID  => s00_axi_awvalid,
      S_AXI_AWREADY  => s00_axi_awready,
      S_AXI_WDATA    => s00_axi_wdata,
      S_AXI_WSTRB    => s00_axi_wstrb,
      S_AXI_WVALID   => s00_axi_wvalid,
      S_AXI_WREADY   => s00_axi_wready,
      S_AXI_BRESP    => s00_axi_bresp,
      S_AXI_BVALID   => s00_axi_bvalid,
      S_AXI_BREADY   => s00_axi_bready,
      S_AXI_ARADDR   => s00_axi_araddr,
      S_AXI_ARPROT   => s00_axi_arprot,
      S_AXI_ARVALID  => s00_axi_arvalid,
      S_AXI_ARREADY  => s00_axi_arready,
      S_AXI_RDATA    => s00_axi_rdata,
      S_AXI_RRESP    => s00_axi_rresp,
      S_AXI_RVALID   => s00_axi_rvalid,
      S_AXI_RREADY   => s00_axi_rready
    );
end architecture rtl;
