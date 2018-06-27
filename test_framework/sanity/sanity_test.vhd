Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

-----------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
--
--    7.7.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Sanity test implementation
--                                       
-----------------------------------------------------------------------------------------------------------------


-----------------------------------------------------------------------------------------------------------------
-- Test implementation                                            
-----------------------------------------------------------------------------------------------------------------

architecture sanity_test of CAN_test is

  ----------------------------------------------
  -- Test specific parameters
  ----------------------------------------------
  constant NODE_COUNT : natural := 4;
  
  ----------------------------------------------
  -- Test configuration
  ----------------------------------------------
  type epsilon_type is array (1 to NODE_COUNT) of natural;
  type trv_del_type is array (1 to NODE_COUNT) of natural;
  
  signal epsilon_v : epsilon_type := (0,150,300,450);
  signal trv_del_v : trv_del_type := (5,5,5,5);
  
  type bus_matrix_type is array(1 to NODE_COUNT,1 to NODE_COUNT) of real;
  signal bus_matrix : bus_matrix_type := ((0.0,10.0,20.0,30.0),(10.0,0.0,10.0,20.0),(20.0,10.0,0.0,10.0),(30.0,20.0,10.0,0.0));
  --signal bus_matrix : bus_matrix_type := ((0.0,60.0,120.0,180.0),(60.0,0.0,60.0,120.0),(120.0,60.0,0.0,60.0),(180.0,120.0,60.0,0.0));
  
  -- Noise parameters
  signal iter_am     : natural := 40;

  signal nw_mean     : real := 15.0;   --Noise pulse width mean in nanaoseconds
  signal nw_var      : real := 5.0;    --Noise pulse width variance
  
  signal ng_mean     : real := 10000.0; --Gap between two noise pulses mean in nanoseconds
  signal ng_var      : real :=  6000.0; -- Gap variance
  
  signal topology    : string (1 to 50) :="                                                  ";
   
  --Default config is the same as in registers...
  --signal timing_config : bit_time_config_type := (5,3,5,3,5,2,3,3,3,2); --TODO:CHANGE!
  
  --With 100 Mhz this is 1Mbit(nom)/2Mbit(Data)
  signal timing_config : bit_time_config_type := (5,5,9,5,5,3,3,3,3,2);
  
  ----------------------------------------------
  -- CAN Node component
  ----------------------------------------------
  component CAN_top_level is
  generic(
      constant use_logger     :boolean  :=true; --Whenever event logger should be synthetised (not yet implemented)
      constant rx_buffer_size :natural  :=128; --Transcieve Buffer size
      constant useFDSize      :boolean  :=true; --Transcieve buffer size should be synthetised as FD Size (640 bits) or normal CAN (128 bits)
      constant use_sync       :boolean  :=true; --Whenever internal synchroniser chain should be used for incoming bus signals
                                                --Dont turn off unless external synchronisation chain is put on input of FPGA by
                                                --synthetiser
      constant ID             :natural  range 0 to 15:=1; --ID (bits  19-16 of adress) 
      constant logger_size    :natural --range 0 to 512:=8
  );
  port(
      signal clk_sys:in std_logic;
      signal res_n:in std_logic;
      signal data_in:in std_logic_vector(31 downto 0);
      signal data_out:out std_logic_vector(31 downto 0);
      signal adress:in std_logic_vector(23 downto 0);
    	 signal scs:in std_logic; --Chip select
      signal srd:in std_logic; --Serial read
      signal swr:in std_logic; --Serial write
      signal int:out std_logic;
      signal CAN_tx:out std_logic;
      signal CAN_rx:in std_logic;
      signal time_quanta_clk:out std_logic; --Time Quantum clocks possible to be used for synchronisation
      signal timestamp:in std_logic_vector(63 downto 0)
  );
  end component;
  
  ----------------------------------------------
  ----------------------------------------------
  -- Internal signals
  ----------------------------------------------
  ----------------------------------------------
  
  --Memory interfaces
  type mem_bus_arr_type is array (1 to NODE_COUNT) of Avalon_mem_type;
  signal mb_arr : mem_bus_arr_type;
  
  --Auxiliarly memory signals used due to no support for connecting
  -- record types to components!
  type mem_vect_arr_type is array (1 to NODE_COUNT) of std_logic_vector(31 downto 0);
  type mem_contr_arr_type is array (1 to NODE_COUNT) of std_logic;
  type mem_addr_arr_type is array (1 to NODE_COUNT) of std_logic_vector(23 downto 0); 
  signal mem_aux_data_in      : mem_vect_arr_type := (OTHERS => (OTHERS => '0'));
  signal mem_aux_data_out     : mem_vect_arr_type := (OTHERS => (OTHERS => '0'));
  signal mem_aux_address      : mem_addr_arr_type := (OTHERS => (OTHERS => '0'));
  signal mem_aux_scs          : mem_contr_arr_type := (OTHERS => '0');
  signal mem_aux_swr          : mem_contr_arr_type := (OTHERS => '0');
  signal mem_aux_srd          : mem_contr_arr_type := (OTHERS => '0');
  signal mem_aux_clk          : mem_contr_arr_type := (OTHERS => '0');
  
  signal res_n_v              : std_logic_vector(1 to NODE_COUNT) := (OTHERS => '0');
  signal int_v                : std_logic_vector(1 to NODE_COUNT) := (OTHERS => '0');
  signal CAN_tx_v             : std_logic_vector(1 to NODE_COUNT);
  signal CAN_rx_v             : std_logic_vector(1 to NODE_COUNT) := (OTHERS => RECESSIVE);
  signal time_quanta_clk_v    : std_logic_vector(1 to NODE_COUNT) := (OTHERS => '0');
  
  type timestamp_arr_type is array (1 to NODE_COUNT) of std_logic_vector(63 downto 0);
  signal timestamp_v          : timestamp_arr_type := (OTHERS => (OTHERS =>'0'));
  
  type trv_del_shift_reg is array (1 to NODE_COUNT) of tran_delay_type;
  signal transciever          : trv_del_shift_reg := (OTHERS => ((OTHERS => RECESSIVE),(OTHERS => RECESSIVE),'1','1'));
  
  --Bus realisation shift registers
  constant BUS_DELAY_MAX      : natural := 4999;
  type bus_delay_type is array (1 to NODE_COUNT) of std_logic_vector(BUS_DELAY_MAX downto 0);
  signal bus_delay_sr         : bus_delay_type := (OTHERS => (OTHERS => RECESSIVE));
  signal bus_clk              : std_logic := '0';
  
  --Noise registers
  signal noise_reg            : std_logic_vector (1 to NODE_COUNT) := (OTHERS => RECESSIVE);
  signal noise_force          : std_logic_vector (1 to NODE_COUNT) := (OTHERS => '0');
  
  --Traffic storage memories
  type storage_mem            is array (0 to 255) of std_logic_vector(31 downto 0);
  type storage_mem_array      is array (1 to NODE_COUNT) of storage_mem;
  signal tx_mems              : storage_mem_array := (OTHERS => (OTHERS => (OTHERS => '0')));
  signal rx_mems              : storage_mem_array := (OTHERS => (OTHERS => (OTHERS => '0')));
  type storage_pointer_array  is array (1 to NODE_COUNT) of natural;
  signal tx_mem_pointers      : storage_pointer_array := (OTHERS => 0);
  signal rx_mem_pointers      : storage_pointer_array := (OTHERS => 0);
  type mem_status_array_type  is array (1 to NODE_COUNT) of boolean;
  signal tx_full              : mem_status_array_type := (OTHERS => false);
  
  --Control signals 
  type control_bool_array_type is array (1 to NODE_COUNT) of boolean;
  signal do_wait              : control_bool_array_type := (OTHERS => true);
  signal do_config            : control_bool_array_type := (OTHERS => false);
  signal do_traffic           : control_bool_array_type := (OTHERS => false);
  signal do_erase_mems        : control_bool_array_type := (OTHERS => false);
  signal do_read_errors       : control_bool_array_type := (OTHERS => false);
  signal config_done          : control_bool_array_type := (OTHERS => false);
  signal do_restart_mem_if    : control_bool_array_type := (OTHERS => true);
  signal do_noise             : boolean := false;

  signal erp_detected         : control_bool_array_type := (OTHERS => false);  
  signal test_desc            : string (1 to 50) :="                                                  ";
  
  --Random counters for number generation
  type rand_ctr_array_type is array (1 to NODE_COUNT) of natural range 0 to RAND_POOL_SIZE;
  signal rand_ctr_gen         : rand_ctr_array_type;
  signal rand_ident_ctr       : natural range 0 to RAND_POOL_SIZE := 0;
  
  signal common_ident         : natural;
  
  ----------------------------------------------
  ----------------------------------------------
  -- Test procedures and functions
  ----------------------------------------------
  ----------------------------------------------
  
  -----------------------------------------------------
  -- Store frame to either TX or RX memory
  -----------------------------------------------------
  procedure store_frame_to_mem(
    variable frame       :  in      SW_CAN_frame_type;
    signal   memory      :  out     storage_mem;
    signal   pointer     :  inout   natural
  )is
  variable ident_vect    :  std_logic_vector(28 downto 0);
  begin
    --Frame format word
    memory(pointer) <= "000000000000000000000"&
                        '0'& --We dont store ESI bit
                        frame.brs&
                        '1'&frame.frame_format&
                        frame.ident_type&
                        frame.rtr&'0'&
                        frame.dlc;
    
    --Timestamp
    memory(pointer+1) <= frame.timestamp(63 downto 32);
    memory(pointer+2) <= frame.timestamp(31 downto 0);
    
    --Identifier
    ident_vect := std_logic_vector(to_unsigned(frame.identifier,29)); 
    memory(pointer+3) <= "000"&ident_vect(17 downto 0)&ident_vect(28 downto 18);
    
    pointer           <= pointer+4;
    wait for 0 ns;
    
    --Data words
    if((frame.rtr='0' or frame.frame_format='1') and (frame.data_length /= 0))then
      
      for i in 0 to (frame.data_length-1)/4 loop
        memory(pointer+i) <= frame.data(511-i*32 downto 480-i*32);
        wait for 0 ns;
      end loop;
      
      pointer <= pointer + ((frame.data_length-1)/4)+1;
      wait for 0 ns;
    end if; 

  end procedure;
    
  -----------------------------------------------------
  -- Read frame from either TX or RX memory
  -----------------------------------------------------
  procedure read_frame_from_mem(
    variable frame       :  inout   SW_CAN_frame_type;
    signal   memory      :  in      storage_mem_array;
    constant mem_index   :  in      natural;
    variable pointer     :  inout   natural
  )is
  variable aux_vect      :  std_logic_vector(28 downto 0);
  begin
    --Erase some unnecessary stuff
    frame.data        := (OTHERS => '0');
    frame.identifier  := 0;
    
    --Frame format
    frame.dlc         := memory(mem_index)(pointer)(3 downto 0);
    frame.rtr         := memory(mem_index)(pointer)(5);
    frame.ident_type  := memory(mem_index)(pointer)(6);
    frame.frame_format := memory(mem_index)(pointer)(7);
    frame.brs         := memory(mem_index)(pointer)(9);
    decode_dlc_v(frame.dlc,frame.data_length);
    pointer           := pointer+1;
    
    --Timestamp
    frame.timestamp   := memory(mem_index)(pointer)&memory(mem_index)(pointer+1);
    pointer           :=pointer+2;
    
    --Identifier
    aux_vect          := memory(mem_index)(pointer)(10 downto 0)&memory(mem_index)(pointer)(28 downto 11);
    frame.identifier  := to_integer(unsigned(aux_vect));
    pointer           := pointer +1;
    
    --Data words
    if((frame.rtr = '0' or frame.frame_format = '1') and frame.data_length /= 0)then
      for i in 0 to (frame.data_length-1)/4 loop
        frame.data(511-i*32 downto 480-i*32) := memory(mem_index)(pointer);
        pointer := pointer+1;
      end loop;
    end if;
    
  end procedure;
  
  
  -----------------------------------------------------
  -- Check if all frames from TX memory are found in
  -- RX memory
  -----------------------------------------------------
  procedure check_memories(
    signal tx_mems        :  in      storage_mem_array;
    signal rx_mems        :  in      storage_mem_array;
    variable outcome      :  inout   boolean
  )is
  variable tx_r_ptr       :  natural := 0;
  variable rx_r_ptr       :  natural := 0;
  variable TX_frame       :  SW_CAN_frame_type;
  variable RX_frame       :  SW_CAN_frame_type;
  variable comp_out       :  boolean;
  variable detected       :  boolean:=false;
  begin
    outcome:= true;
    tx_r_ptr := 0;
    rx_r_ptr := 0;
     
    -- Now we check that content of each TX memory is located
    -- in each RX memory except the one with the same index
    -- node does not recieve its own frames!
    for i in 1 to NODE_COUNT loop
      
      tx_r_ptr := 0;
      for j in 1 to NODE_COUNT loop
        
        if( i /= j) then
            rx_r_ptr := 0;
            --Detect frame based on TBF bit which is always 1. If 
            -- this bit is set we assume frame is there. We have to
            -- keep policy of erasing the memory properly then!
            while(tx_mems(i)(tx_r_ptr)(8)='1')loop     
        
              --Read frame from TX Mem
              read_frame_from_mem(TX_frame,tx_mems,i,tx_r_ptr);
                
              --Now browse trough the RX mem j and check 
              -- if we find this frame. Set comp_out if
              -- frame was found in RX memory
              rx_r_ptr:=0;
              comp_out:=false;
              detected:=false;
              while(rx_mems(j)(rx_r_ptr)(8)='1' and comp_out=false)loop
                read_frame_from_mem(RX_frame,rx_mems,j,rx_r_ptr);
                CAN_compare_frames_v(TX_frame,RX_frame,false,comp_out);
                if(comp_out=true)then
                  detected:=true;
                end if;
              end loop;
        
              if(detected=false)then
                outcome:= false;
              end if;
            end loop;
            
        end if;
      end loop;
      
    end loop;
    
  end procedure;
    
  
  -------------------------------------------------------------
  -- In order to avoid collisions, we have to make sure that
  -- we have different identifiers. So we set last 3 bits
  -- to unsigned value of the controller index.
  -- Additionally we modify the identifiers to have first 
  -- N bits matching (N is random), so that arbitration
  -- is lost in random bit. With absolutely random identifiers
  -- arbitration is usually lost very soon (first 3 bits)...
  -------------------------------------------------------------
  procedure correct_identifiers(
    signal      rand_ctr : inout natural range 0 to RAND_POOL_SIZE;
    variable    frame    : inout SW_CAN_frame_type;
    constant    index    : in    natural;
    signal      com_id   : in    natural
  )is
  variable aux_vect    : std_logic_vector(28 downto 0);
  variable rand_val    : real;
  variable rand_index  : natural;
  variable vect_comm   : std_logic_vector(28 downto 0);
  begin
    
    -------------------------------------------
    -- Correct first bits to have interesting
    -- arbitration...
    -------------------------------------------
    rand_real_v(rand_ctr,rand_val);
    
    if(frame.ident_type =EXTENDED)then
      rand_index := integer(6.0*rand_val);
    else
      rand_index := integer(24.0*rand_val);
    end if;
    
    aux_vect := std_logic_vector(to_unsigned(frame.identifier,29));
    vect_comm := std_logic_vector(to_unsigned(com_id,29));
    
    aux_vect (28 downto 28-rand_index) := vect_comm(28 downto 28-rand_index);
    frame.identifier := to_integer(unsigned(aux_vect));
    
    -------------------------------------------
    -- Correct the last two bits
    -------------------------------------------
    aux_vect := std_logic_vector(to_unsigned(frame.identifier,29)); 
    --On extended identifier we set the last bits of extension
    -- on base we set last bits of base
    if( frame.ident_type = EXTENDED ) then
      aux_vect (2 downto 0)   := std_logic_vector(to_unsigned(index,3));
    else
      aux_vect (20 downto 18) := std_logic_vector(to_unsigned(index,3));
    end if;
    frame.identifier := to_integer(unsigned(aux_vect));
  
  end procedure;
  
  
  procedure restart_mem_bus(
      signal mem_bus  : out   Avalon_mem_type
  )is
  begin
      mem_bus.scs  <= '0';
      mem_bus.swr  <= '0';
      mem_bus.srd  <= '0';
      mem_bus.address  <= (OTHERS =>'0');
      mem_bus.data_in  <= (OTHERS =>'0');
      mem_bus.data_out <= (OTHERS =>'Z');
      mem_bus.clk_sys  <= 'Z';       
  end procedure;
    
  
begin
  
  ----------------------------------------------
  -- CAN Nodes instances
  ----------------------------------------------
  comp_gen:for i in 1 to NODE_COUNT generate
    node_1_comp:CAN_top_level 
    generic map(
         use_logger       => true,
         rx_buffer_size   => 64,
         useFDSize        => true,
         use_sync         => true,
         ID               => i,
         logger_size      => 16
    )
    port map(
         res_n            => res_n_v(i),
         clk_sys          => mem_aux_clk(i),
         data_in          => mem_aux_data_in(i),
         data_out         => mem_aux_data_out(i),
         adress           => mem_aux_address(i),
      	  scs              => mem_aux_scs(i),
         srd              => mem_aux_srd(i),
         swr              => mem_aux_swr(i),
         int              => int_v(i),
         CAN_tx           => CAN_tx_v(i),
         CAN_rx           => CAN_rx_v(i),
         time_quanta_clk  => time_quanta_clk_v(i),
         timestamp        => timestamp_v(i)
    );
  
  mb_arr(i).clk_sys       <=  mem_aux_clk(i);
  mem_aux_data_in(i)      <=  mb_arr(i).data_in;
  mem_aux_address(i)      <=  mb_arr(i).address;
  mem_aux_scs(i)          <=  mb_arr(i).scs;
  mem_aux_swr(i)          <=  mb_arr(i).swr;
  mem_aux_srd(i)          <=  mb_arr(i).srd;
  mb_arr(i).data_out      <=  mem_aux_data_out(i);
  
  end generate comp_gen;  

  
  
  ----------------------------------------------
  -- Clock generation
  ----------------------------------------------
  clock_generic: for i in 1 to NODE_COUNT generate
    clock_gen_1:process
    variable period   :natural:=f100_Mhz;
    variable duty     :natural:=50;
    variable epsilon  :natural:=epsilon_v(i);
    begin
      generate_clock(period,duty,epsilon,mem_aux_clk(i));
      timestamp_v(i) <= std_logic_vector(unsigned(timestamp_v(i))+1);
    end process; 
  end generate clock_generic;
  
  
  ----------------------------------------------
  -- Realisation of transciever delay
  ----------------------------------------------
  tr_del_gen: for i in 1 to NODE_COUNT generate
    trv_del_gen_proc:process
    variable index  : natural;
     begin
       if(res_n_v(i) = ACT_RESET)then
         transciever(i).tx_delay_sr <= (OTHERS => RECESSIVE);
         transciever(i).rx_delay_sr <= (OTHERS => RECESSIVE);
         transciever(i).tx_point    <= RECESSIVE;
         wait for 5 ns;
       else
        wait until rising_edge(mb_arr(i).clk_sys);
        
        --TX shift register
        transciever(i).tx_delay_sr <= transciever(i).tx_delay_sr(254 downto 0)&CAN_tx_v(i);
        index:= trv_del_v(i)/2;
        if(index>1)then
          index:=index-2;
        end if;
        transciever(i).tx_point    <= transciever(i).tx_delay_sr(index);
        
        --RX Shift register   
        transciever(i).rx_delay_sr <= transciever(i).rx_delay_sr(254 downto 0)&
                                      (transciever(i).tx_point AND transciever(i).rx_point);
        CAN_rx_v(i)                <= transciever(i).rx_delay_sr(index);
       end if;   
     end process;
  end generate tr_del_gen;
  
  ----------------------------------------------
  -- Bus clock generation
  -- 500 ps equals approximately
  -- 10 cm of conductor!
  ----------------------------------------------
  bus_clk_proc:process
  begin
    bus_clk <= '1';
    wait for 250 ps;
    bus_clk <= '0';
    wait for 250 ps;
  end process;
  
  ----------------------------------------------
  -- Realisation of CAN bus
  ----------------------------------------------
  bus_gen: for i in 1 to NODE_COUNT generate
    bus_gen_proc:process
    variable rx_lvl : std_logic := RECESSIVE;
    variable index  : natural   := 0;
    begin
      wait until rising_edge(bus_clk);
      rx_lvl := RECESSIVE;
      
      --Realising the delay from transciever
      bus_delay_sr(i) <= bus_delay_sr(i)(BUS_DELAY_MAX-1 downto 0)&transciever(i).tx_point;     
      
      --In one recieving node iterate trough all nodes 
      -- and AND all delayed signals.
      for j in 1 to NODE_COUNT loop
        index  := integer(10.0*bus_matrix(j,i));
        rx_lvl := rx_lvl AND bus_delay_sr(j)(index);
      end loop;  
      
      --Now add the Generated noise to the recieving node
      if(noise_force(i)='1')then
        rx_lvl := noise_reg(i);
      end if;
      
      transciever(i).rx_point <= rx_lvl;
    
    end process;
  end generate bus_gen;
  
  ----------------------------------------------
  -- Noise generator
  ----------------------------------------------
  noise_gen_proc:process
  variable noise_time : real := 0.0;
  variable noise_gap  : real := 0.0;
  variable aux        : std_logic_vector(3 downto 0);
  begin
    if(do_noise)then
      --Generate duration of noise pulse
      -- and inter-noise time
      rand_gauss_v(rand_ctr,iter_am,nw_mean,nw_var,noise_time);
      rand_gauss_v(rand_ctr,iter_am,ng_mean,ng_var,noise_gap);
   
      noise_force <= (OTHERS => '0');
      wait for integer(noise_gap)*1 ns;
      
      --Generate noise polarity and info whether noise
      -- should be forced to the node
      -- We cant put noise to all. That would be global 
      -- error at all times
      rand_logic_vect_v(rand_ctr ,aux,0.5);
      noise_reg   <= aux;
      rand_logic_vect_v(rand_ctr ,aux,0.5);
      noise_force <= aux;
     
      wait for integer(noise_time)*1 ns;
      
    else
      wait for 10 ns;
    end if;
  end process;
  
  ----------------------------------------------
  -- Common identifier generation
  -- we have to set some common base for identifiers
  -- to have interesting arbitration...
  ----------------------------------------------
  ident_gen_proc:process
  variable rand_value: real := 0.0;
  begin
    for i in 0 to 100 loop
      wait until rising_edge(mb_arr(1).clk_sys);
    end loop;
    rand_real_v(rand_ident_ctr,rand_value);
    common_ident <= integer(rand_value*536870911.0);
  end process;
  
  
  ----------------------------------------------
  -- Node access
  -- Traffic generation and error states 
  ----------------------------------------------
  access_gen: for i in 1 to NODE_COUNT generate
    
    node_access_proc:process
    variable rand_real  : real;
    variable TX_frame   : SW_CAN_frame_type;
    variable RX_frame   : SW_CAN_frame_type;
    variable r_data     : std_logic_vector(31 downto 0);
    variable n_index    : natural := i;
    variable frame_sent : boolean := false;
    begin
      if(do_restart_mem_if(i)=true)then
        restart_mem_bus(mb_arr(i));
        wait for 10 ns;
      elsif(do_wait(i)=true)then
        wait until rising_edge(mb_arr(i).clk_sys);
      elsif(do_erase_mems(i)=true)then
        tx_mems(i) <= (OTHERS => (OTHERS => '0'));
        rx_mems(i) <= (OTHERS => (OTHERS => '0'));
        tx_mem_pointers(i) <= 0;
        rx_mem_pointers(i) <= 0;
        erp_detected(i)   <= false;
        wait for 10 ns;
      else
        if(do_config(i)=true)then
          
          config_done (i) <= false;
                  
          --Perform the configuration of the node
          CAN_turn_controller(true,n_index,mb_arr(i));
          CAN_configure_timing(timing_config,n_index,mb_arr(i));       
          
          --Signal back to main process config finished
          config_done(i) <= true;
          wait for 100 ns;       
        else
        
          if(do_traffic(i)=true)then
            
            --Check if first TXT Buffer is empty
            -- We use only 1st in this test
            -- Send Frame if yes
            CAN_read(r_data,TX_STATUS_ADR,n_index,mb_arr(i));
            if(r_data(0)='1')then
              
              --Generate and transmitt frames until tx memory
              -- of given node is full.
              if(tx_full(i)=false)then
                 
                 --To avoid having the same frames from all nodes
                 -- we modify the rand_counter by the index
                 if(rand_ctr_gen(i) + i > RAND_POOL_SIZE)then
                   rand_ctr_gen(i) <= 0;
                 else
                   rand_ctr_gen(i) <= rand_ctr_gen(i)+i;
                 end if;                 
                 wait for 0 ns;
     
                 CAN_generate_frame(rand_ctr_gen(i),TX_frame);
              
                 --Now we have absolutely RANDOM identifiers
                 --To avoid collisions we correct them to have
                 -- unique endings...
                 correct_identifiers(rand_ctr_gen(i),TX_frame,i,common_ident);
              
                 CAN_send_frame(TX_frame,1,n_index,mb_arr(i),frame_sent);
                 store_frame_to_mem(TX_frame,tx_mems(i),tx_mem_pointers(i));
              end if;
            end if;
            
            
            -- Check if RX buffer is non-empty
            -- Read out frames if it is not
            CAN_read(r_data,RX_INFO_1_ADR,n_index,mb_arr(i));
            while (r_data(0)='0') loop
              CAN_read_frame(RX_frame,n_index,mb_arr(i));
              store_frame_to_mem(RX_frame,rx_mems(i),rx_mem_pointers(i));
              CAN_read(r_data,RX_INFO_1_ADR,n_index,mb_arr(i));             
            end loop;
           
          end if;
        
          if(do_read_errors(n_index)=true)then            
            --Check if unit is not error passive
            -- If node is error passive data consitency will be corrupted!!!
            -- that should never happend
            CAN_read(r_data,ERROR_TH_ADR,n_index,mb_arr(i));
            if(r_data(17)='1')then
              erp_detected(i) <= true;
            end if;
          end if;
          
        end if;
        
      end if;      
    end process;
    
    ----------------------------------------------
    -- Memory statuses
    ----------------------------------------------
    --Note that we keep reserve of up to 60 words, thats
    -- exactly how much we need to fit 3 times longest frame...
    tx_full(i) <= true when tx_mem_pointers(i)+230>=tx_mems(i)'length else false;
    
  end generate access_gen;

  
  ----------------------------------------------
  -- Main test process
  ----------------------------------------------
  test_proc:process
  variable outcome    :boolean  := true;
  variable step_done  :boolean  := false;
  begin
      print_test_info(iterations,log_level,error_beh,error_tol);
      
      --log("Restarting TXT Buffer test!",info_l,log_level);
      --TODO: Log with config info
      
      wait for 5 ns;
      do_restart_mem_if <= (OTHERS => false);      
      wait for 5 ns;
      reset_test(res_n_v(1),status,run,error_ctr);
      reset_test(res_n_v(2),status,run,error_ctr);
      reset_test(res_n_v(3),status,run,error_ctr);
      reset_test(res_n_v(4),status,run,error_ctr);
      loop_ctr <= 0;
      
      --Deactivate the wait signal
      do_wait <= (OTHERS => false);
      
      log("Applying Configuration...",info_l,log_level);
      for i in 1 to NODE_COUNT loop
        do_config(i) <= true;
      end loop;
      wait until config_done = (true,true,true,true);
      for i in 1 to NODE_COUNT loop
        do_config(i)      <= false;
        do_read_errors(i) <= true;
      end loop;
      log("Configuration applied",info_l,log_level);
      
      -------------------------------
      --Main loop of the test
      -------------------------------
      while (loop_ctr<iterations  or  exit_imm)
      loop
        log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
        
        --Here is a special case when previous step failed. We need to reset and reconfigure node
        if(erp_detected(1) or erp_detected(2) or erp_detected(3) or erp_detected(4))then
          res_n_v <= (OTHERS => '0');
          wait for 50 ns;
          res_n_v <= (OTHERS => '1');
          wait for 10 ns;
          
          do_config <= (OTHERS => true);
          wait for 1000 ns;
          do_config <= (OTHERS => false);
          wait for 10 ns;
        end if;
        
        --Erase the test memories
        for i in 1 to NODE_COUNT loop
          do_erase_mems(i) <= true;
        end loop;
        wait for 1000 ns;
        for i in 1 to NODE_COUNT loop
          do_erase_mems(i) <= false;
        end loop;
        
        --Start the traffic generation
        for i in 1 to NODE_COUNT loop
          do_traffic(i)     <= true;
          do_noise          <= true;
        end loop;
        
        wait for 2000 ns;
        
        -- Units stop transmiting when they reach tx_full! We wait until everything what was transmitted
        -- is also recieved!
        step_done :=false;
        while (step_done=false) loop
          if(tx_mem_pointers(1)+tx_mem_pointers(2)+tx_mem_pointers(3)=rx_mem_pointers(4) and
             tx_mem_pointers(2)+tx_mem_pointers(3)+tx_mem_pointers(4)=rx_mem_pointers(1) and 
             tx_mem_pointers(3)+tx_mem_pointers(4)+tx_mem_pointers(1)=rx_mem_pointers(2) and
             tx_mem_pointers(1)+tx_mem_pointers(2)+tx_mem_pointers(4)=rx_mem_pointers(3))then
             step_done:= true;
           end if;
           
           --If any of the nodes turned error passive
           if(erp_detected(1) or erp_detected(2) or erp_detected(3) or erp_detected(4))then
             step_done:= true;
             report "Some unit turned error passive -> Most probably traffic consitency check will fail!" severity error;
             --process_error(error_ctr,error_beh,exit_imm); 
           end if;
           wait for 100 ns;
        end loop;
        
        --Stop the traffic generation
        for i in 1 to NODE_COUNT loop
          do_traffic(i)     <= false;
          do_noise          <= false;
        end loop;
        
        wait for 200 ns;
        
        --Now evaluate the memories content...
        check_memories(tx_mems,rx_mems,outcome);
        
        if(outcome=false)then
          log("Traffic consitency check error!",error_l,log_level);
          process_error(error_ctr,error_beh,exit_imm); 
        end if;
        
        wait for 1000 ns;
        loop_ctr <= loop_ctr+1;
        wait for 0 ns;
        
       end loop;
      
      evaluate_test(error_tol,error_ctr,status);   
  end process;
  
  
  errors <= error_ctr;
  
end architecture;



-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------

architecture sanity_test_wrapper of CAN_test_wrapper is
  
  --Test component itself
  component CAN_test is
  port (
    signal run            :in   boolean;                -- Input trigger, test starts running when true
    signal iterations     :in   natural;                -- Number of iterations that test should do
    signal log_level      :in   log_lvl_type;           -- Logging level, severity which should be shown
    signal error_beh      :in   err_beh_type;           -- Test behaviour when error occurs: Quit, or Go on
    signal error_tol      :in   natural;                -- Error tolerance, error counter should not
                                                         -- exceed this value in order for the test to pass
    signal status         :out  test_status_type;      -- Status of the test
    signal errors         :out  natural                -- Amount of errors which appeared in the test
    --TODO: Error log results 
  );
  end component;
  
  --Select architecture of the test
  for test_comp : CAN_test use entity work.CAN_test(sanity_test);
  
    signal run              :   boolean;                -- Input trigger, test starts running when true                                                        -- exceed this value in order for the test to pass
    signal status_int       :   test_status_type;      -- Status of the test
    signal errors           :   natural;                -- Amount of errors which appeared in the test
    signal error_ctr        :   natural;
begin
  
  --In this test wrapper generics are directly connected to the signals
  -- of test entity
  test_comp:CAN_test
  port map(
     run              =>  run,
     iterations       =>  iterations , 
     log_level        =>  log_level,
     error_beh        =>  error_beh,
     error_tol        =>  error_tol,                                                     
     status           =>  status_int,
     errors           =>  errors
  );
  
  status              <= status_int;
  error_ctr           <=  errors;
  
  ------------------------------------
  --Starts the test and lets it run
  ------------------------------------
  test:process
  begin
    run               <= true;
    wait for 1 ns;
    
    --Wait until the only test finishes and then propagate the results
    wait until (status_int=passed or status_int=failed);  
    
    wait for 100 ns;
    run               <= false;
        
  end process;
  
  
end;
