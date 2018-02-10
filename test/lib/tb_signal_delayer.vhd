library ieee;
use ieee.std_logic_1164.all;

-- Delay a signal by non-static time.
-- Maintains a FIFO of (time&value) of event (change) on input signal
-- and replays it on the delayed signal with the specified delay.
entity tb_signal_delayer_vec is
    generic (
        NSAMPLES : positive;
        DWIDTH : positive
    );
    port (
        input : in std_logic_vector(DWIDTH-1 downto 0);
        delayed : out std_logic_vector(DWIDTH-1 downto 0);
        delay : in time
    );
end entity;

architecture tb of tb_signal_delayer_vec is
    type data_type is array(0 to NSAMPLES-1) of time;
    type dataval_type is array(0 to NSAMPLES-1) of std_logic_vector(DWIDTH-1 downto 0);
    signal first : boolean := true;

    signal nonempty : boolean;
    signal pop : boolean;-- sensitive to edge, not level!
    signal top : time;
    signal top_val : std_logic_vector(DWIDTH-1 downto 0);
begin
    p_fifo: process
        variable data : data_type;
        variable dataval : dataval_type;
        variable rdidx : natural := 0;
        variable wridx : natural := 0;
    begin
        if first then
            first <= false;
            top_val <= input;
        end if;
        wait until (input'event or pop'event);
        if input'event then
            assert (wridx - rdidx) < NSAMPLES report "FIFO full!" severity failure;
            data(wridx mod NSAMPLES) := now;
            dataval(wridx mod NSAMPLES) := input;
            wridx := wridx + 1;
        elsif pop'event then
            assert (wridx - rdidx) > 0 report "FIFO empty!" severity failure;
            top_val <= dataval(rdidx mod NSAMPLES);
            rdidx := rdidx + 1;
        end if;
        top <= data(rdidx mod NSAMPLES);
        nonempty <= (wridx - rdidx) > 0;
        wait for 0 ns;
    end process;

    p_delay: process
        variable towait : time;
        variable first : boolean := true;
    begin
        if delay < 0 ns then
            wait until delay >= 0 ns;
        end if;
        if first then
            first := false;
            delayed <= input;
        end if;
        if not nonempty then
            wait until nonempty;
        end if;
        towait := top + delay - now;
        --report "Waiting for " & time'image(towait);
        wait for towait;
        delayed <= top_val;
        if not nonempty then
            wait until nonempty;
        end if;
        pop <= not pop;
    end process;
end;

library ieee;
use ieee.std_logic_1164.all;

entity tb_signal_delayer is
    generic (
        NSAMPLES : positive
    );
    port (
        input : in std_logic;
        delayed : out std_logic;
        delay : in time
    );
end entity;

architecture tb of tb_signal_delayer is
    signal input_v, delayed_v : std_logic_vector(0 downto 0);
begin
    i_sdv: entity work.tb_signal_delayer_vec
    generic map (
        NSAMPLES => NSAMPLES,
        DWIDTH => 1
    )
    port map (
        input => input_v,
        delayed => delayed_v,
        delay => delay
    );
    input_v <= (0 => input);
    delayed <= delayed_v(0);
end architecture;
