library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.CANconstants.all;

entity axi_ifc is
  generic (
    -- Users to add parameters here

    -- User parameters ends
    -- Do not modify the parameters beyond this line

    -- Width of S_AXI data bus
    C_S_AXI_DATA_WIDTH  : integer  := 32;
    -- Width of S_AXI address bus
    C_S_AXI_ADDR_WIDTH  : integer  := 24;

    -- ID (bits  19-16 of reg_addr_o)
    ID : natural := 1
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
  --signal axi_awaddr   : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal axi_awready  : std_logic;
  signal axi_wready   : std_logic;
  signal axi_bresp    : std_logic_vector(1 downto 0);
  signal axi_bvalid   : std_logic;
  signal axi_araddr   : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal axi_arready  : std_logic;
  signal axi_rresp    : std_logic_vector(1 downto 0);
  signal axi_rvalid   : std_logic;

  signal addr_regoff : std_logic_vector(ID_ADRESS_LOWER-1 downto 0);

  signal want_to_read           : std_logic;
  signal want_to_write          : std_logic;
  signal want_to_write_q        : std_logic;

  signal read_in_progress       : std_logic;
  signal write_in_progress      : std_logic;
  signal write_precedence       : std_logic;
  signal write_precedence_r     : std_logic;
  signal write_precedence_set   : std_logic;
  signal write_precedence_reset : boolean;
  constant READ_STAGE_END : natural := 1;
  signal read_stage             : natural range 0 to READ_STAGE_END;
  signal write_stage            : natural range 0 to 1;
begin
  -- I/O Connections assignments

  S_AXI_AWREADY <= axi_awready;
  S_AXI_WREADY  <= axi_wready;
  S_AXI_BRESP   <= axi_bresp;
  S_AXI_BVALID  <= axi_bvalid;
  S_AXI_ARREADY <= axi_arready;
  S_AXI_RRESP   <= axi_rresp;
  S_AXI_RVALID  <= axi_rvalid;

  reg_data_in_o <= S_AXI_WDATA;
  S_AXI_RDATA <= reg_data_out_i;
  -- arbitrate

  -- "the slave must not wait for the master to assert BREADY before asserting BVALID"!
  want_to_write <= S_AXI_WVALID and S_AXI_AWVALID;
  want_to_read <= S_AXI_RREADY and S_AXI_ARVALID;

  p_wtwq: process(S_AXI_ARESETN, S_AXI_ACLK)
  begin
    if (S_AXI_ARESETN = '0') then
      want_to_write_q <= '0';
    elsif rising_edge(S_AXI_ACLK) then
      want_to_write_q <= want_to_write;
    end if;
  end process;

  p_wpq:process(S_AXI_ARESETN, S_AXI_ACLK)
  begin
    if S_AXI_ARESETN = '0' then
      write_precedence_r <= '0';
    elsif rising_edge(S_AXI_ACLK) then
      if write_precedence_reset then
        write_precedence_r <= not want_to_read;
      elsif write_precedence_set = '1' then
        write_precedence_r <= not want_to_read;
      end if;
    end if;
  end process;

  write_precedence_set <= want_to_write and not want_to_write_q;
  write_precedence <= (write_precedence_set and not want_to_read) or write_precedence_r;

-- write:
-- write_in_progress = want_to_write and (not want_to_read or write_precedence)
--
-- write_in_progress
-- stage 0:
-- -> async latch addr, be
-- -> async set ready, wren
-- -> async set bresp, bvalid
-- -> set write_stage=1
-- stage 1:
-- -> async unlatch everything, unset ready
-- - sync: if bready=1 -> async unset bvalid
--                     -> set write_stage=0
--                     -> sync unset write_precedence_r
--
-- read:
-- read_in_progress = want_to_read and (not want_to_write or not write_precedence)
--
-- read_in_progress
-- stage 0:
-- -> async latch addr
-- -> async set ready, rden
-- -> set read_stage=1
-- stage 1:
-- -> async unlatch everything, unset ready, rden
-- -> set rdata
-- - sync: if rready=1 -> unset rvalid
--                     -> set read_stage=0
  write_in_progress <= want_to_write and (not want_to_read or write_precedence);
  read_in_progress <= want_to_read and not (want_to_write and write_precedence);

  p_read:process(S_AXI_ARESETN, S_AXI_ACLK)
  begin
    if S_AXI_ARESETN = '0' then
      read_stage <= 0;
    elsif rising_edge(S_AXI_ACLK) then
      if read_in_progress = '1' and read_stage < READ_STAGE_END then
        read_stage <= read_stage + 1;
      elsif read_stage = READ_STAGE_END and S_AXI_RREADY = '1' then
        read_stage <= 0;
      end if;
    end if;
  end process;

  write_precedence_reset <= write_stage = 1 and S_AXI_BREADY = '1';
  p_write:process(S_AXI_ARESETN, S_AXI_ACLK)
  begin
    if S_AXI_ARESETN = '0' then
      write_stage <= 0;
    elsif rising_edge(S_AXI_ACLK) then
      if write_stage = 0 and write_in_progress = '1' then
        write_stage <= 1;
      elsif write_stage = 1 and S_AXI_BREADY = '1' then
        write_stage <= 0;
        -- reset write_precedence_r
      end if;
    end if;
  end process;

  reg_addr_o(COMP_TYPE_ADRESS_HIGHER downto COMP_TYPE_ADRESS_LOWER) <= CAN_COMPONENT_TYPE;
  reg_addr_o(ID_ADRESS_HIGHER downto ID_ADRESS_LOWER) <= std_logic_vector(to_unsigned(ID,4));
  reg_addr_o(addr_regoff'range) <= addr_regoff;

  p_async_common:process(S_AXI_ARESETN, write_in_progress, write_stage, S_AXI_BREADY, read_in_progress, read_stage)
  begin
    if S_AXI_ARESETN = '0' then
      addr_regoff <= (others => '0');
    elsif write_in_progress = '1' then
      if write_stage = 0 or S_AXI_BREADY = '1' then
        addr_regoff <= S_AXI_AWADDR(addr_regoff'range);
      end if;
    elsif read_in_progress = '1' then
      if (read_stage = 0) then -- no back-to-back for read (need waitcycle)
        addr_regoff <= S_AXI_ARADDR(addr_regoff'range);
      end if;
    else
      addr_regoff <= (others => '0');
    end if;
  end process;

  p_async_read:process(S_AXI_ARESETN, read_in_progress, read_stage)
  begin
    if S_AXI_ARESETN /= '0' and read_in_progress = '1' then
      reg_rden_o <= '1';
      if read_stage < READ_STAGE_END then
        axi_arready <= '0';
        axi_rvalid <= '0';
      else
        axi_arready <= '1';
        axi_rvalid <= '1';
      end if;
    else
      reg_rden_o <= '0';
      axi_arready <= '0';
      axi_rvalid <= '0';
    end if;
    axi_rresp <= "00"; -- OK
  end process;

  p_async_write:process(S_AXI_ARESETN, write_in_progress, write_stage, S_AXI_BREADY)
  begin
    if S_AXI_ARESETN /= '0' and write_in_progress = '1' and (write_stage = 0 or S_AXI_BREADY = '1') then
      reg_be_o    <= S_AXI_WSTRB;
      reg_wren_o  <= '1';
      axi_awready <= '1';
      axi_wready  <= '1';
      axi_bvalid  <= '1';
      axi_bresp   <= "00"; -- OK
    else
      reg_be_o    <= (others => '1');
      reg_wren_o  <= '0';
      axi_awready <= '0';
      axi_wready  <= '0';
      axi_bvalid  <= '0';
      axi_bresp   <= "00";
    end if;
  end process;
end architecture rtl;
