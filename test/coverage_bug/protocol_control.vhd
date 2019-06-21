Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

entity protocol_control is
    port(
        tran_frame_valid    : in  std_logic;
        drv_bus_mon_ena     : in  std_logic;
        tx_frame_ready      : out std_logic
    );
end entity;

architecture rtl of protocol_control is

begin

    
end architecture;