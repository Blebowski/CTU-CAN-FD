Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.ID_transfer.all;

-----------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
--
--    30.5.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Unit test for the message filter circuit                                                 
-----------------------------------------------------------------------------------------------------------------


-----------------------------------------------------------------------------------------------------------------
-- Test implementation                                            
-----------------------------------------------------------------------------------------------------------------

architecture mess_filt_unit_test of CAN_test is

  component messageFilter is
  PORT(
    --Clock an reset signals
    signal clk_sys            :in std_logic;                      --System clock
    signal res_n              :in std_logic;                      --Async reset
    
    --Driving signals from CAN Core
    signal rec_ident_in       :in std_logic_vector(28 downto 0);  --Receieved identifier
    signal ident_type         :in std_logic;                      --Input message identifier type 
                                                                  -- (0-BASE Format, 1-Extended Format);
    signal frame_type         :in std_logic;                      --Input frame type (0-Normal CAN, 1- CAN FD) 
    signal rec_ident_valid    :in std_logic;                      --Identifier valid (active log 1)
    
    --Driving bus from registers
    signal drv_bus            :in std_logic_vector(1023 downto 0);
   
    signal out_ident_valid    :out std_logic --Signal whenever identifier matches the filter identifiers
  );
  end component;

    signal clk_sys            : std_logic;                      --System clock
    signal res_n              : std_logic;                      --Async reset
    signal rec_ident_in       : std_logic_vector(28 downto 0);  --Receieved identifier
    signal ident_type         : std_logic;                      --Input message identifier type 
                                                                  -- (0-BASE Format, 1-Extended Format);
    signal frame_type         : std_logic;                      --Input frame type (0-Normal CAN, 1- CAN FD) 
    signal rec_ident_valid    : std_logic;                      --Identifier valid (active log 1)
    signal drv_bus            : std_logic_vector(1023 downto 0);
    signal out_ident_valid    : std_logic;

    --Internal testbench signals
    signal frame_info         : mess_filter_input_type := ((OTHERS=>'0'),'0','0','0');  
    signal drv_settings       : mess_filter_drv_type   := ((OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),
                                                           (OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),
                                                           (OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),
                                                           (OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),
                                                           '0');
        
     
    procedure generate_input(
      signal rand_ctr        :inout natural range 0 to RAND_POOL_SIZE;
      signal frame_info      :out   mess_filter_input_type
      )is
    begin
      rand_logic_vect (rand_ctr,  frame_info.rec_ident_in     ,0.5);
      rand_logic      (rand_ctr,  frame_info.ident_type       ,0.5);
      rand_logic      (rand_ctr,  frame_info.frame_type       ,0.5);
      rand_logic      (rand_ctr,  frame_info.rec_ident_valid  ,0.9);
    end procedure;
    
    
    procedure generate_setting(
      signal rand_ctr        :inout natural range 0 to RAND_POOL_SIZE;
      signal drv_settings    :inout   mess_filter_drv_type
      )is
    begin
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_A_bits,  0.50);
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_A_mask,  0.15);
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_A_ctrl,  0.50);
      
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_B_bits,  0.50);
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_B_mask,  0.15);
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_B_ctrl,  0.50);
        
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_C_bits,  0.50);
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_C_mask,  0.15);
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_C_ctrl,  0.50);
      
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_ran_hi_th,  0.60);
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_ran_lo_th,  0.40);
      rand_logic_vect    (rand_ctr,  drv_settings.drv_filter_ran_ctrl,   0.50);
      
      rand_logic         (rand_ctr,  drv_settings.drv_filters_ena,  0.9); 
    end procedure;
    
    
    function validate(
      signal drv_settings   :in     mess_filter_drv_type;
      signal filt_res       :in     std_logic;
      signal log_level      :in     log_lvl_type;
      signal frame_info     :in     mess_filter_input_type) return boolean
    is
      variable join           :       std_logic_vector(1 downto 0);
      variable ctrl           :       std_logic_vector(3 downto 0);
      variable A_type         :       boolean;  --Whether input frame matches the frame type
      variable B_type         :       boolean;  --Whether input frame matches the frame type
      variable C_type         :       boolean;  --Whether input frame matches the frame type
      variable ran_type       :       boolean;  --Whether input frame matches the frame type
      variable A_vals         :       boolean;  --Whether input frame matches the frame bits
      variable B_vals         :       boolean;  --Whether input frame matches the frame bits
      variable C_vals         :       boolean;  --Whether input frame matches the frame bits
      variable ran_vals       :       boolean;  --Whether input frame matches the frame range
      variable ident_dec      :       integer;
      variable ran_low_dec    :       integer;
      variable ran_high_dec   :       integer;
      variable frame_conc     :       std_logic_vector(28 downto 0);
      variable low_conc       :       std_logic_vector(28 downto 0);
      variable high_conc      :       std_logic_vector(28 downto 0);
    begin
      
      --Filters disabled but result is positive -> error
      if(drv_settings.drv_filters_ena='0')then
        
        if(frame_info.rec_ident_valid = filt_res)then
            return true;
        else
          log("Filters disabled but result positive",error_l,log_level);
          return false; 
        end if;
      end if;
      
      join:= frame_info.frame_type&frame_info.ident_type;
      case join is
      when "00" => ctrl:= "0001" ; --CAN BASIC
      when "01" => ctrl:= "0010" ; --CAN Extended
      when "10" => ctrl:= "0100" ; --CAN FD Basic
      when "11" => ctrl:= "1000" ; --CAN Fd Extended
      when others => ctrl:= "0000" ; 
      end case;  
      
      --Calculate the values of matching frames
      A_type :=  not ((ctrl AND drv_settings.drv_filter_A_ctrl) = "0000");
      B_type :=  not ((ctrl AND drv_settings.drv_filter_B_ctrl) = "0000");
      C_type :=  not ((ctrl AND drv_settings.drv_filter_C_ctrl) = "0000");
      ran_type :=  not ((ctrl AND drv_settings.drv_filter_ran_ctrl) = "0000");
      
      A_vals :=  ((frame_info.rec_ident_in         AND drv_settings.drv_filter_A_mask) =
                  (drv_settings.drv_filter_A_bits  AND drv_settings.drv_filter_A_mask));
      
      B_vals :=  ((frame_info.rec_ident_in         AND drv_settings.drv_filter_B_mask) =
                  (drv_settings.drv_filter_B_bits  AND drv_settings.drv_filter_B_mask));
                  
      C_vals :=  ((frame_info.rec_ident_in         AND drv_settings.drv_filter_C_mask) =
                  (drv_settings.drv_filter_C_bits  AND drv_settings.drv_filter_C_mask));
      
      frame_conc := frame_info.rec_ident_in(10 downto 0)&frame_info.rec_ident_in(28 downto 11);
      ident_dec  := to_integer(unsigned(frame_conc)); 
      
      --Note that here identifier parts are not swapped since driving bus value is already decimal value!
      low_conc   := drv_settings.drv_filter_ran_lo_th(28 downto 11)&drv_settings.drv_filter_ran_lo_th(10 downto 0);
      ident_dec  := to_integer(unsigned(low_conc));      
      
      high_conc  := drv_settings.drv_filter_ran_hi_th(28 downto 11)&drv_settings.drv_filter_ran_hi_th(10 downto 0);
      ident_dec  := to_integer(unsigned(high_conc));        
            
      ran_vals   := ((frame_conc < high_conc) or (frame_conc = high_conc)) and 
                    ((frame_conc > low_conc) or  (frame_conc = low_conc));        
      
      --------------------------------------------
      --Invalid frame type was not filtered out
      --------------------------------------------
      if( (A_type=false)    AND
          (B_type=false)    AND
          (C_type=false)    AND
          (ran_type=false)  AND
          (filt_res='1')
      )then
        log("Invalid frame type was not filtered out",error_l,log_level);
        return false;
      end if;
      
      -------------------------------------------
      --Valid or invalid frames on input
      -------------------------------------------
      if( ((A_type AND A_vals) OR
          (B_type AND B_vals) OR
          (C_type AND C_vals) OR
          (ran_type AND ran_vals))
          AND
          (drv_settings.drv_filters_ena='1')
          AND
          (frame_info.rec_ident_valid='1')
      )then
        
        if (filt_res='1')then   --Is detected
          return true;
        elsif (filt_res='0')then -- Is not detected
          log("Valid frame not detected",error_l,log_level);
          return false;
        else
          log("Filter res undefined",error_l,log_level);
          return false;
        end if;
        
       else 
         
         if (filt_res='1')then   --Is detected
          log("Invalid frame but frame detected",error_l,log_level);
          
          return false;
         elsif (filt_res='0')then -- Is not detected
          return true;
         else
          log("Filter res undefined",error_l,log_level); 
          return false;
         end if;
        
       end if;
      
    end function;
    
    
begin
  
  ---------------------------------
  --Instance of the message filter
  ---------------------------------
  messageFilter_comp:messageFilter 
  PORT map(
     clk_sys            =>  clk_sys,        
     res_n              =>  res_n,
     rec_ident_in       =>  rec_ident_in,
     ident_type         =>  ident_type,
     frame_type         =>  frame_type,
     rec_ident_valid    =>  rec_ident_valid,   
     drv_bus            =>  drv_bus,
     out_ident_valid    =>  out_ident_valid
  );
  
  ---------------------------------
  --Clock generation
  ---------------------------------
  clock_gen:process
  variable period   :natural:=f100_Mhz;
  variable duty     :natural:=50;
  variable epsilon  :natural:=0;
  begin
    generate_clock(period,duty,epsilon,clk_sys);
  end process;  
  
  --Propagate driving bus to driving bus signals
  drv_bus(DRV_FILTER_A_MASK_HIGH downto DRV_FILTER_A_MASK_LOW)           <= drv_settings.drv_filter_A_mask;
  drv_bus(DRV_FILTER_A_CTRL_HIGH downto DRV_FILTER_A_CTRL_LOW)           <= drv_settings.drv_filter_A_ctrl;
  drv_bus(DRV_FILTER_A_BITS_HIGH downto DRV_FILTER_A_BITS_LOW)           <= drv_settings.drv_filter_A_bits;
  drv_bus(DRV_FILTER_B_MASK_HIGH downto DRV_FILTER_B_MASK_LOW)           <= drv_settings.drv_filter_B_mask;
  drv_bus(DRV_FILTER_B_CTRL_HIGH downto DRV_FILTER_B_CTRL_LOW)           <= drv_settings.drv_filter_B_ctrl;
  drv_bus(DRV_FILTER_B_BITS_HIGH downto DRV_FILTER_B_BITS_LOW)           <= drv_settings.drv_filter_B_bits;
  drv_bus(DRV_FILTER_C_MASK_HIGH downto DRV_FILTER_C_MASK_LOW)           <= drv_settings.drv_filter_C_mask;
  drv_bus(DRV_FILTER_C_CTRL_HIGH downto DRV_FILTER_C_CTRL_LOW)           <= drv_settings.drv_filter_C_ctrl;
  drv_bus(DRV_FILTER_C_BITS_HIGH downto DRV_FILTER_C_BITS_LOW)           <= drv_settings.drv_filter_C_bits;
  drv_bus(DRV_FILTER_RAN_CTRL_HIGH downto DRV_FILTER_RAN_CTRL_LOW)       <= drv_settings.drv_filter_ran_ctrl;
  drv_bus(DRV_FILTER_RAN_LO_TH_HIGH downto DRV_FILTER_RAN_LO_TH_LOW)     <= drv_settings.drv_filter_ran_lo_th;
  drv_bus(DRV_FILTER_RAN_HI_TH_HIGH downto DRV_FILTER_RAN_HI_TH_LOW)     <= drv_settings.drv_filter_ran_hi_th;
  drv_bus(DRV_FILTERS_ENA_INDEX)                                         <= drv_settings.drv_filters_ena;
  
  --Connect input generator to the circuit
     rec_ident_in       <=  frame_info.rec_ident_in;
     ident_type         <=  frame_info.ident_type;
     frame_type         <=  frame_info.frame_type;
     rec_ident_valid    <=  frame_info.rec_ident_valid;   
  
  ---------------------------------
  ---------------------------------
  --Main Test process
  ---------------------------------
  ---------------------------------
  test_proc:process
  begin
    log("Restarting Message filter test!",info_l,log_level);
    wait for 5 ns;
    reset_test(res_n,status,run,error_ctr);
    log("Restarted Message filter test",info_l,log_level);
    print_test_info(iterations,log_level,error_beh,error_tol);
    
    -------------------------------
    --Main loop of the test
    -------------------------------
    log("Starting message filter main loop",info_l,log_level);
    
    while (loop_ctr<iterations  or  exit_imm)
    loop
      --log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
      
      generate_input    (rand_ctr,frame_info);
      generate_setting  (rand_ctr,drv_settings);
      
      wait for 10 ns;
      
      if(validate(drv_settings,out_ident_valid,log_level,frame_info)=false)then
        process_error(error_ctr,error_beh,exit_imm);       
      end if;     
      
      loop_ctr<=loop_ctr+1;
    end loop;
    
    evaluate_test(error_tol,error_ctr,status);
  end process;
  
end architecture;


-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture mess_filt_unit_test_wrapper of CAN_test_wrapper is
  
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
  for test_comp : CAN_test use entity work.CAN_test(mess_filt_unit_test);
  
    signal run              :   boolean;                -- Input trigger, test starts running when true                                                        -- exceed this value in order for the test to pass
    signal status_int       :   test_status_type;      -- Status of the test
    signal errors           :   natural;                -- Amount of errors which appeared in the test

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


