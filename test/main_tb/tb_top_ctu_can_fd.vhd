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
--  @Purpose:
--    CTU CAN FD main testbench top
--  
--------------------------------------------------------------------------------
-- Revision History:
--    26.1.2021   Created file
--------------------------------------------------------------------------------

-- Only top level uses Vunit. This allows keeping CTU CAN FD VIP Vunit-less!!
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
        test_name               : string := "demo";
        test_type               : string := "compliance"; -- "feature", "compliance" or "reference"
        stand_alone_vip_mode    : boolean := true; 
        log_level               : t_log_verbosity := verbosity_info;

        iterations              : natural := 1;
        timeout                 : string := "10 ms";

        -- Reference test iterations
        reference_iterations    : natural range 1 to 1000 := 1000;
        
        -- Clock configuration of DUT
        cfg_sys_clk_period      : string := "10 ns";
        
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
        
        -- DUT configuration
        rx_buffer_size          : natural := 64;
        txt_buffer_count        : natural range 2 to 8 := 4;
        sup_filtA               : boolean := true;
        sup_filtB               : boolean := true;
        sup_filtC               : boolean := true;
        sup_range               : boolean := true;
        sup_traffic_ctrs        : boolean := true;
        target_technology       : natural := C_TECH_ASIC;

        -- Seed
        seed                    : natural := 0
    );
end entity;

architecture tb of tb_top_ctu_can_fd is
   
   -- DUT interface
   signal clk_sys       : std_logic;
   signal res_n         : std_logic;
   
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


   component ctu_can_fd_vip is
   generic(
       -- Test details
       test_name               : string;
       test_type               : string;
       stand_alone_vip_mode    : boolean;

       -- DUT Clock period
       cfg_sys_clk_period      : string;
       
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
       
       -- Seed
       seed                    : natural := 0;
       
       -- Reference test iterations
        reference_iterations   : natural range 1 to 1000 := 1000
    );
    port(
       -- Test control
       test_start          : in  std_logic;
       test_done           : out std_logic := '0';
       test_success        : out std_logic := '0';
         
       -- DUT interface
       clk_sys             : inout std_logic;
       res_n               : out   std_logic;
        
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
    dut : can_top_level
    generic map(
        rx_buffer_size      => rx_buffer_size,
        txt_buffer_count    => txt_buffer_count,
        sup_filtA           => sup_filtA,
        sup_filtB           => sup_filtB,
        sup_filtC           => sup_filtC,
        sup_range           => sup_range,
        sup_traffic_ctrs    => sup_traffic_ctrs,
        target_technology   => target_technology
    )
    port map(
        -- Clock and Asynchronous reset
        clk_sys     => clk_sys,
        res_n       => res_n,

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
    ctu_can_fd_vip_inst : ctu_can_fd_vip
    generic map(
        test_name               => test_name,
        test_type               => test_type,
        stand_alone_vip_mode    => stand_alone_vip_mode,

        cfg_sys_clk_period      => cfg_sys_clk_period,
        
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
        res_n       => res_n,
        
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
    vunit_manager_proc : process
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 10 ns;

        info("***************************************************************");
        info("CTU CAN FD main testbench");
        info("");
        info("Test configuration:");
        info("  Test type: " & test_type);
        info("  Test name: " & test_name);
        info("  No. of iterations: " & integer'image(iterations));
        info("  Stand-alone VIP: " & boolean'image(stand_alone_vip_mode));
        info("  System clock period: " & cfg_sys_clk_period);
        info("  Log level: " & t_log_verbosity'image(log_level));
        info("  Seed: " & integer'image(seed));
        info("  Reference test iterations: " & integer'image(reference_iterations));
        info("  Timeout: " & timeout);
        info("");
        info("DUT configuration:");
        info("  RX buffer size: " & integer'image(rx_buffer_size));
        info("  TXT Buffer count: " & integer'image(txt_buffer_count));
        info("  Filter A: " & boolean'image(sup_filtA));
        info("  Filter B: " & boolean'image(sup_filtB));
        info("  Filter C: " & boolean'image(sup_filtC));
        info("  Range filter: " & boolean'image(sup_range));
        info("  Traffic counters: " & boolean'image(sup_traffic_ctrs));
        info("  Target technology: " & integer'image(target_technology));
        info("");
        info("Bit timing settings (Nominal):");
        info("  BRP: " & integer'image(cfg_brp));
        info("  PH1: " & integer'image(cfg_ph_1));
        info("  PROP: " & integer'image(cfg_prop));
        info("  PH2: " & integer'image(cfg_ph_2));
        info("  SJW: " & integer'image(cfg_sjw));
        info("");
        info("Bit timing settings (Data):");
        info("  BRP: " & integer'image(cfg_brp));
        info("  PH1: " & integer'image(cfg_ph_1));
        info("  PROP: " & integer'image(cfg_prop));
        info("  PH2: " & integer'image(cfg_ph_2));
        info("  SJW: " & integer'image(cfg_sjw));
        info("");
        info("***************************************************************");

        show(get_logger(default_checker), display_handler, pass);
        set_log_verbosity(log_level, global_verbosity);

        for i in 1 to iterations loop
            info("***************************************************************");
            info(" Iteration nr: " & integer'image(i));
            info("***************************************************************");

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
    end process;

    ---------------------------------------------------------------------------
    -- Spawn watchdog
    ---------------------------------------------------------------------------
    watchdog: if time'value(timeout) > 0 ns generate
        test_runner_watchdog(runner, time'value(timeout));
    end generate;

end architecture;

