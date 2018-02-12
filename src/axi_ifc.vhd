library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axi_ifc is
  generic (
    -- Users to add parameters here

    -- User parameters ends
    -- Do not modify the parameters beyond this line

    -- Width of S_AXI data bus
    C_S_AXI_DATA_WIDTH  : integer  := 32;
    -- Width of S_AXI address bus
    C_S_AXI_ADDR_WIDTH  : integer  := 24
  );
  port (
    -- Users to add ports here
    reg_data_in_o    : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    reg_data_out_i   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    reg_addr_o       : out std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    reg_be_o         : out std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
    reg_rden_o       : out std_logic;
    reg_wren_o       : out std_logic;
    -- User ports ends
    -- Do not modify the ports beyond this line

    -- Global Clock Signal
    S_AXI_ACLK  : in std_logic;
    -- Global Reset Signal. This Signal is Active LOW
    S_AXI_ARESETN  : in std_logic;
    -- Write address (issued by master, acceped by Slave)
    S_AXI_AWADDR  : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    -- Write channel Protection type. This signal indicates the
        -- privilege and security level of the transaction, and whether
        -- the transaction is a data access or an instruction access.
    S_AXI_AWPROT  : in std_logic_vector(2 downto 0);
    -- Write address valid. This signal indicates that the master signaling
        -- valid write address and control information.
    S_AXI_AWVALID  : in std_logic;
    -- Write address ready. This signal indicates that the slave is ready
        -- to accept an address and associated control signals.
    S_AXI_AWREADY  : out std_logic;
    -- Write data (issued by master, acceped by Slave)
    S_AXI_WDATA  : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    -- Write strobes. This signal indicates which byte lanes hold
        -- valid data. There is one write strobe bit for each eight
        -- bits of the write data bus.
    S_AXI_WSTRB  : in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
    -- Write valid. This signal indicates that valid write
        -- data and strobes are available.
    S_AXI_WVALID  : in std_logic;
    -- Write ready. This signal indicates that the slave
        -- can accept the write data.
    S_AXI_WREADY  : out std_logic;
    -- Write response. This signal indicates the status
        -- of the write transaction.
    S_AXI_BRESP  : out std_logic_vector(1 downto 0);
    -- Write response valid. This signal indicates that the channel
        -- is signaling a valid write response.
    S_AXI_BVALID  : out std_logic;
    -- Response ready. This signal indicates that the master
        -- can accept a write response.
    S_AXI_BREADY  : in std_logic;
    -- Read address (issued by master, acceped by Slave)
    S_AXI_ARADDR  : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    -- Protection type. This signal indicates the privilege
        -- and security level of the transaction, and whether the
        -- transaction is a data access or an instruction access.
    S_AXI_ARPROT  : in std_logic_vector(2 downto 0);
    -- Read address valid. This signal indicates that the channel
        -- is signaling valid read address and control information.
    S_AXI_ARVALID  : in std_logic;
    -- Read address ready. This signal indicates that the slave is
        -- ready to accept an address and associated control signals.
    S_AXI_ARREADY  : out std_logic;
    -- Read data (issued by slave)
    S_AXI_RDATA  : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    -- Read response. This signal indicates the status of the
        -- read transfer.
    S_AXI_RRESP  : out std_logic_vector(1 downto 0);
    -- Read valid. This signal indicates that the channel is
        -- signaling the required read data.
    S_AXI_RVALID  : out std_logic;
    -- Read ready. This signal indicates that the master can
        -- accept the read data and response information.
    S_AXI_RREADY  : in std_logic
  );
end entity axi_ifc;

architecture rtl of axi_ifc is

  -- AXI4LITE signals
  signal axi_awaddr  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal axi_awready  : std_logic;
  signal axi_wready  : std_logic;
  signal axi_bresp  : std_logic_vector(1 downto 0);
  signal axi_bvalid  : std_logic;
  signal axi_araddr  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal axi_arready  : std_logic;
  signal axi_rdata  : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal axi_rresp  : std_logic_vector(1 downto 0);
  signal axi_rvalid  : std_logic;

  signal want_to_read     : std_logic;
  signal want_to_write    : std_logic;
  signal want_to_write_q  : std_logic;

  signal write_precedence_set, write_precedence, write_precedence_r : std_logic;
begin
  -- I/O Connections assignments

  S_AXI_AWREADY <= axi_awready;
  S_AXI_WREADY  <= axi_wready;
  S_AXI_BRESP   <= axi_bresp;
  S_AXI_BVALID  <= axi_bvalid;
  S_AXI_ARREADY <= axi_arready;
  S_AXI_RDATA   <= axi_rdata;
  S_AXI_RRESP   <= axi_rresp;
  S_AXI_RVALID  <= axi_rvalid;

  --slv_reg_wren <= want_to_write and (not want_to_read or write_precedence);
  --slv_reg_rden <= want_to_read and not write_precedence;


  reg_data_in_o <= S_AXI_WDATA;
  S_AXI_RDATA <= reg_data_out_i;
  -- arbitrate

  -- "the slave must not wait for the master to assert BREADY before asserting BVALID"!
  want_to_write <= S_AXI_WVALID and S_AXI_AWVALID;
  want_to_read <= S_AXI_RREADY and S_AXI_ARVALID;

  process
  begin
    if (S_AXI_ARESETN = '0') then
      want_to_write_q <= '0';
    elsif rising_edge(S_AXI_ACLK) then
      want_to_write_q <= want_to_write;
    end if;
  end process;

  write_precedence_set <= want_to_write and not want_to_write_q;
  write_precedence <= write_precedence_set or write_precedence_r;

  process
  begin
    if (S_AXI_ARESETN = '0') then
      write_precedence_r <= '0';
      reg_addr_o <= (others => '0');
      reg_be_o <= (others => '0');
      reg_rden_o <= '0';
      reg_wren_o <= '0';
      axi_arready <= '0';
      axi_rvalid <= '0';
      axi_awready <= '0';
      axi_wready <= '0';
      axi_bvalid <= '0';
      axi_bresp <= "00";
    elsif (rising_edge(S_AXI_ACLK)) then
      if (want_to_write = '1' and want_to_write_q = '0') then
        write_precedence_r <= not want_to_read;
      end if;

      if (write_precedence = '0' and want_to_read = '1') then
        reg_addr_o <= S_AXI_ARADDR;
        reg_be_o <= (others => '1');
        reg_rden_o <= '1';

        axi_arready <= '1';
        axi_rvalid <= '1';
      elsif (want_to_write = '1') then
        reg_addr_o <= S_AXI_AWADDR;
        reg_be_o <= S_AXI_WSTRB;
        reg_wren_o <= '1';

        axi_awready <= '1';
        axi_wready <= '1';
        axi_bvalid <= '1';
        axi_bresp <= "00"; -- OK
        write_precedence_r <= '0';
      end if;

      if (axi_bvalid = '1' and S_AXI_BREADY = '1') then
        axi_awready <= '0';
        axi_wready <= '0';
        axi_bvalid <= '0';
        reg_wren_o <= '0';
      end if;

      if (axi_rvalid = '1' and S_AXI_RREADY = '1') then
        axi_rvalid <= '0';
        axi_arready <= '0';
        reg_rden_o <= '0';
        -- TODO: waitcycle?
      end if;
    end if;
  end process;


-- edge on (S_AXI_WVALID and S_AXI_AWVALID and S_AXI_BREADY) -> want to write; write_precedence := not want_to_read
-- edge on (S_AXI_RREDY and S_AXI_ARVALID) -> want to read
--
-- if write_precedence and want_to_write -> route address, pulse slv_reg_wren
-- elsif want_to_read -> route address, pulse slv_reg_rden
-- elsif want_to_write -> route address, pulse slv_reg_wren

  -- User logic ends

end architecture rtl;
