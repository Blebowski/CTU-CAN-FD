--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2021-2023 Ondrej Ille
-- Copyright (C) 2023-     Logic Design Services Ltd.s
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- non-commercial purposes. Using the Component for commercial purposes is
-- forbidden unless previously agreed with Copyright holder.
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
--  @Purpose:
--    CTU CAN FD main testbench top
--
--------------------------------------------------------------------------------
-- Revision History:
--    26.1.2021   Created file
--------------------------------------------------------------------------------

-- Only top level uses Vunit. This allows keeping CTU CAN FD VIP Vunit-less,
-- when integrating RTL and VIP into other TB!
library vunit_lib;
context vunit_lib.vunit_context;

-- Common contexts
Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;
context ctu_can_fd_tb.tb_agents_context;
context ctu_can_fd_tb.rtl_context;

entity tb_top_ctu_can_fd is
    generic(
        -- Test-bench specific stuff
        runner_cfg              : string := runner_cfg_default;
        test_name               : string := "device_id";
        test_type               : string := "feature"; -- "feature", "compliance" or "reference"
        log_level               : t_log_verbosity := verbosity_info;
        func_cov_en             : boolean := true;

        iterations              : natural := 1;
        timeout                 : string := "10 ms";

        is_force_supported      : boolean := true;

        -- Reference test iterations
        reference_iterations    : natural range 1 to 1000 := 10;

        -- Clock configuration of DUT
        cfg_sys_clk_period      : string := "10 ns";

        -- Finish on report Error
        finish_on_error         : natural := 0;

        -- Bit timing config of DUT on CAN bus (used by compliance tests)
        cfg_brp                 : natural := 4;
        cfg_prop                : natural := 0;
        cfg_ph_1                : natural := 1;
        cfg_ph_2                : natural := 1;
        cfg_sjw                 : natural := 2;
        cfg_brp_fd              : natural := 1;
        cfg_prop_fd             : natural := 3;
        cfg_ph_1_fd             : natural := 1;
        cfg_ph_2_fd             : natural := 2;
        cfg_sjw_fd              : natural := 2;

        -- Secondary sample point config
        cfg_ssp_src             : natural := 0;
        cfg_ssp_offset          : natural := 10;

        -- DUT configuration
        G_RX_BUF_SIZE          : natural := 64;
        G_TXT_BUF_COUNT        : natural range 2 to 8 := 8;
        G_FILT_A_EN               : boolean := true;
        G_FILT_B_EN               : boolean := true;
        G_FILT_C_EN               : boolean := true;
        G_FILT_RANGE_EN               : boolean := true;
        G_TRAFFIC_CTRS_EN        : boolean := true;
        G_PARITY_EN              : boolean := true;
        G_TECHNOLOGY       : natural := C_TECH_ASIC;
        G_RESET_BUF_RAMS       : boolean := false;

        -- Seed
        seed                    : natural := 0
    );
end entity;

architecture tb of tb_top_ctu_can_fd is

   -- DUT interface
   signal clk_sys       : std_logic;
   signal rst_n         : std_logic;

   signal write_data    : std_logic_vector(31 DOWNTO 0);
   signal read_data     : std_logic_vector(31 DOWNTO 0);
   signal address       : std_logic_vector(15 DOWNTO 0);
   signal scs           : std_logic;
   signal swr           : std_logic;
   signal srd           : std_logic;
   signal sbe           : std_logic_vector(3 DOWNTO 0);

   signal int           : std_logic;

   signal can_tx        : std_logic;
   signal can_rx        : std_logic;

   signal timestamp     : std_logic_vector(63 DOWNTO 0);
   signal test_probe    : t_ctu_can_fd_test_probe;

   -- Test control
   signal test_start    : std_logic := '0';
   signal test_done     : std_logic := '0';
   signal test_success  : std_logic := '0'; -- 0 fail / 1 success

   signal scan_enable   : std_logic;

   component ctu_can_fd_vip is
   generic(
       -- Test details
       test_name               : string;
       test_type               : string;
       func_cov_en             : boolean;

       -- DUT configuration
       G_RX_BUF_SIZE          : natural;
       G_TXT_BUF_COUNT        : natural range 2 to 8 := 8;

       -- DUT Clock period
       cfg_sys_clk_period      : string;
       finish_on_error         : natural;

       -- Bit timing cofnig used in; compliance tests
       cfg_brp                 : natural;
       cfg_prop                : natural;
       cfg_ph_1                : natural;
       cfg_ph_2                : natural;
       cfg_sjw                 : natural;
       cfg_brp_fd              : natural;
       cfg_prop_fd             : natural;
       cfg_ph_1_fd             : natural;
       cfg_ph_2_fd             : natural;
       cfg_sjw_fd              : natural;

       -- Secondary sample point config
       cfg_ssp_src             : natural;
       cfg_ssp_offset          : natural;

       -- Seed
       seed                    : natural := 0;

       -- Reference test iterations
        reference_iterations   : natural range 1 to 1000 := 10
    );
    port(
       -- Test control
       test_start          : in  std_logic;
       test_done           : out std_logic := '0';
       test_success        : out std_logic := '0';

       -- DUT interface
       clk_sys             : out std_logic;
       rst_n               : out std_logic;

       scan_enable         : out   std_logic;

       write_data          : out   std_logic_vector(31 DOWNTO 0);
       read_data           : in    std_logic_vector(31 DOWNTO 0);
       adress              : out   std_logic_vector(15 DOWNTO 0);
       scs                 : out   std_logic;
       srd                 : out   std_logic;
       swr                 : out   std_logic;
       sbe                 : out   std_logic_vector(3 DOWNTO 0);

       int                 : in    std_logic;

       can_tx              : in    std_logic;
       can_rx              : out   std_logic;

       test_probe          : in    t_ctu_can_fd_test_probe;
       timestamp           : out   std_logic_vector(63 DOWNTO 0)
    );
    end component;

begin

    ---------------------------------------------------------------------------
    -- DUT (Use RAM-like memory bus)
    ---------------------------------------------------------------------------
    i_dut : entity ctu_can_fd_rtl.ctu_can_fd_top
    generic map(
        G_RX_BUF_SIZE        => G_RX_BUF_SIZE,
        G_TXT_BUF_COUNT      => G_TXT_BUF_COUNT,
        G_FILT_A_EN          => G_FILT_A_EN,
        G_FILT_B_EN          => G_FILT_B_EN,
        G_FILT_C_EN          => G_FILT_C_EN,
        G_FILT_RANGE_EN      => G_FILT_RANGE_EN,
        G_TRAFFIC_CTRS_EN    => G_TRAFFIC_CTRS_EN,
        G_PARITY_EN          => G_PARITY_EN,
        G_TECHNOLOGY         => G_TECHNOLOGY,
        G_RESET_BUF_RAMS     => G_RESET_BUF_RAMS
    )
    port map(
        -- Clock and Asynchronous reset
        clk_sys     => clk_sys,
        rst_n       => rst_n,

        -- DFT support
        scan_mode   => scan_enable,

        -- Memory interface
        data_in     => write_data,
        data_out    => read_data,
        adress      => address,
        scs         => scs,
        srd         => srd,
        swr         => swr,
        sbe         => sbe,

        -- Interrupt Interface
        int         => int,

        -- CAN Bus Interface
        can_tx      => can_tx,
        can_rx      => can_rx,

        -- Test probe
        test_probe  => test_probe,

        -- Timestamp for time based transmission / reception
        timestamp   => timestamp
    );


    ---------------------------------------------------------------------------
    -- CTU CAN FD VIP
    ---------------------------------------------------------------------------
    i_ctu_can_fd_vip : ctu_can_fd_vip
    generic map(
        test_name               => test_name,
        test_type               => test_type,
        func_cov_en             => func_cov_en,

        G_RX_BUF_SIZE          => G_RX_BUF_SIZE,
        G_TXT_BUF_COUNT        => G_TXT_BUF_COUNT,

        cfg_sys_clk_period      => cfg_sys_clk_period,
        finish_on_error         => finish_on_error,

        cfg_brp                 => cfg_brp,
        cfg_prop                => cfg_prop,
        cfg_ph_1                => cfg_ph_1,
        cfg_ph_2                => cfg_ph_2,
        cfg_sjw                 => cfg_sjw,
        cfg_brp_fd              => cfg_brp_fd,
        cfg_prop_fd             => cfg_prop_fd,
        cfg_ph_1_fd             => cfg_ph_1_fd,
        cfg_ph_2_fd             => cfg_ph_2_fd,
        cfg_sjw_fd              => cfg_sjw_fd,

        cfg_ssp_src             => cfg_ssp_src,
        cfg_ssp_offset          => cfg_ssp_offset,

        seed                    => seed,
        reference_iterations    => reference_iterations
    )
    port map(
        -- Test control
        test_start         => test_start,
        test_done          => test_done,
        test_success       => test_success,

        -----------------------------------------------------------------------
        -- DUT interface
        -----------------------------------------------------------------------

        -- Clock, reset
        clk_sys     => clk_sys,
        rst_n       => rst_n,

        -- DFT support
        scan_enable => scan_enable,

        -- Memory bus
        write_data  => write_data,
        read_data   => read_data,
        adress      => address,
        scs         => scs,
        srd         => srd,
        swr         => swr,
        sbe         => sbe,

        -- Interrupt
        int         => int,

        -- CAN bus
        can_tx      => can_tx,
        can_rx      => can_rx,

        -- Test interface
        test_probe  => test_probe,

        -- Timestamp
        timestamp   => timestamp
    );


    ---------------------------------------------------------------------------
    -- Vunit manager - controls CTU CAN FD VIP
    ---------------------------------------------------------------------------
    p_vunit_manager : process
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 10 ns;

        info_m("***************************************************************");
        info_m("CTU CAN FD main testbench");
        info_m("");
        info_m("Test configuration:");
        info_m("  Test type: " & test_type);
        info_m("  Test name: " & test_name);
        info_m("  No. of iterations: " & integer'image(iterations));
        info_m("  System clock period: " & cfg_sys_clk_period);
        info_m("  Log level: " & t_log_verbosity'image(log_level));
        info_m("  Seed: " & integer'image(seed));
        info_m("  Reference test iterations: " & integer'image(reference_iterations));
        info_m("  Timeout: " & timeout);
        info_m("  Finish on error: " & integer'image(finish_on_error));
        info_m("  Force support: " & boolean'image(is_force_supported));
        info_m("");
        info_m("DUT configuration:");
        info_m("  RX buffer size: " & integer'image(G_RX_BUF_SIZE));
        info_m("  TXT Buffer count: " & integer'image(G_TXT_BUF_COUNT));
        info_m("  Filter A: " & boolean'image(G_FILT_A_EN));
        info_m("  Filter B: " & boolean'image(G_FILT_B_EN));
        info_m("  Filter C: " & boolean'image(G_FILT_C_EN));
        info_m("  Range filter: " & boolean'image(G_FILT_RANGE_EN));
        info_m("  Traffic counters: " & boolean'image(G_TRAFFIC_CTRS_EN));
        info_m("  Target technology: " & integer'image(G_TECHNOLOGY));
        info_m("  Reset Buffer RAMS: " & boolean'image(G_RESET_BUF_RAMS));
        info_m("");
        info_m("Bit timing settings (Nominal):");
        info_m("  BRP: " & integer'image(cfg_brp));
        info_m("  PH1: " & integer'image(cfg_ph_1));
        info_m("  PROP: " & integer'image(cfg_prop));
        info_m("  PH2: " & integer'image(cfg_ph_2));
        info_m("  SJW: " & integer'image(cfg_sjw));
        info_m("");
        info_m("Bit timing settings (Data):");
        info_m("  BRP: " & integer'image(cfg_brp_fd));
        info_m("  PH1: " & integer'image(cfg_ph_1_fd));
        info_m("  PROP: " & integer'image(cfg_prop_fd));
        info_m("  PH2: " & integer'image(cfg_ph_2_fd));
        info_m("  SJW: " & integer'image(cfg_sjw_fd));
        info_m("Secondary sample point:");
        info_m("  Source: " & integer'image(cfg_ssp_src));
        info_m("  Offset: " & integer'image(cfg_ssp_offset));
        info_m("");
        info_m("***************************************************************");

        show(get_logger(default_checker), display_handler, pass);
        set_log_verbosity(log_level, global_verbosity);

        tb_force.set_force_supported(is_force_supported);

        for i in 1 to iterations loop
            info_m("***************************************************************");
            info_m(" Iteration nr: " & integer'image(i));
            info_m("***************************************************************");

            -- Execute test
            test_start <= '1';
            wait until test_done = '1';
            wait for 1 ns;

            -- Propagate fail to Vunit if test signals it failed
            -- true indicates fail (exit code 1)
            if (test_success = '0') then
                test_runner_cleanup(runner, true);
            end if;

            -- Finish handshake
            test_start <= '0';
            wait until test_done = '0';
            wait for 10 ns;
        end loop;

        -- Finish succesfully
        test_runner_cleanup(runner);
        std.env.finish;
    end process;

    ---------------------------------------------------------------------------
    -- Spawn watchdog
    ---------------------------------------------------------------------------
    g_watchdog: if time'value(timeout) > 0 ns generate
        test_runner_watchdog(runner, time'value(timeout));
    end generate;

end architecture;
