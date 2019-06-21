Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library work;

entity protocol_control_tb is
end entity;

architecture behav of protocol_control_tb is

    signal tran_frame_valid : std_logic := '0';
    signal drv_bus_mon_ena : std_logic := '0';
    signal tx_frame_ready : std_logic;

begin

    tx_frame_ready_proc : process(tran_frame_valid, drv_bus_mon_ena)
    begin
        if (tran_frame_valid = '1' and drv_bus_mon_ena = '0') then
            tx_frame_ready <= '1';
        else
            tx_frame_ready <= '0';
        end if;
    end process;

    driver_proc : process
    begin
        
        for i in 0 to 10 loop
            drv_bus_mon_ena <= '1';
            wait for 10 ns;
            tran_frame_valid <= '1';
            wait for 10 ns;
            drv_bus_mon_ena <= '0';
            wait for 10 ns;
            drv_bus_mon_ena <= '0';
            wait for 10 ns;
        end loop;
        
        report "Test end" severity failure;
    end process;

end architecture;