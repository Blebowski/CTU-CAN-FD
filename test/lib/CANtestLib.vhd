--------------------------------------------------------------------------------
-- 
-- CAN with Flexible Data-Rate IP Core 
-- 
-- Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisor: Jiri Novak <jnovak@fel.cvut.cz>
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

-------------------------------------------------------------------------------------------------------------
--  Purpose:
--    Main test library for CAN controller. Contains all test resources for running CANTest framework
--
-------------------------------------------------------------------------------------------------------------
-- Revision History:
--    27.5.2016   Created file
--    13.1.2017   Added formatting of identifier in CAN_send_frame, CAN_read_frame to fit the native
--                decimal interpretation (the same way as in C driver)
--    27.11.2017  Added "reset_test" function fix. Implemented reset synchroniser to avoid async reset in
--                the core. As consequnce after the core reset is released, the core has to wait at least TWO clock
--                cycles till the reset is synchronised and deasserted.
-------------------------------------------------------------------------------------------------------------


Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
USE work.randomLib.All;
use work.CANconstants.all;

use work.CANFD_register_map.all;

package CANtestLib is
-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------
-- Types
-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------
--Common test types
-----------------------------------------------------------------------------------------
type log_lvl_type is (info_l,warning_l,error_l);
type err_beh_type is (quit,go_on);
type test_status_type  is (waiting,running,passed,failed);


-----------------------------------------------------------------------------------------
--Message filter types
-----------------------------------------------------------------------------------------
type mess_filter_input_type is record
    rec_ident_in        :std_logic_vector(28 downto 0);
    ident_type          :std_logic;
    frame_type          :std_logic;
    rec_ident_valid     :std_logic; 
end record;

type mess_filter_drv_type is record
    drv_filter_A_mask    :std_logic_vector(28 downto 0);   --Filter A bit mask
    drv_filter_A_ctrl    :std_logic_vector(3 downto 0);    --Filter A control bits
    drv_filter_A_bits    :std_logic_vector(28 downto 0);   --Filter A bits 
  
    drv_filter_B_mask    :std_logic_vector(28 downto 0);   --Filter B bit mask
    drv_filter_B_ctrl    :std_logic_vector(3 downto 0);    --Filter B control bits
    drv_filter_B_bits    :std_logic_vector(28 downto 0);   --Filter B bits 
  
    drv_filter_C_mask    :std_logic_vector(28 downto 0);   --Filter C bit mask
    drv_filter_C_ctrl    :std_logic_vector(3 downto 0);    --Filter C control bits
    drv_filter_C_bits    :std_logic_vector(28 downto 0);   --Filter C bits 
   
    drv_filter_ran_ctrl  :std_logic_vector(3 downto 0);    --Range filter control bits
    drv_filter_ran_lo_th :std_logic_vector(28 downto 0);   --Lower range filter trehsold
    drv_filter_ran_hi_th :std_logic_vector(28 downto 0);   --Upper range filter trehsold

    drv_filters_ena      :std_logic;
end record;

-----------------------------------------------------------------------------------------
--Prescaler types
-----------------------------------------------------------------------------------------
type presc_drv_type is record
     drv_tq_nbt           :   std_logic_vector (5 downto 0); 
     drv_tq_dbt           :   std_logic_vector (5 downto 0); 
     drv_prs_nbt          :   std_logic_vector (5 downto 0); 
     drv_ph1_nbt          :   std_logic_vector (5 downto 0);  
     drv_ph2_nbt          :   std_logic_vector (5 downto 0); 
     drv_prs_dbt          :   std_logic_vector (3 downto 0); 
     drv_ph1_dbt          :   std_logic_vector (3 downto 0);  
     drv_ph2_dbt          :   std_logic_vector (3 downto 0); 
     drv_sjw_nbt          :   std_logic_vector(3 downto 0); 
     drv_sjw_dbt          :   std_logic_vector(3 downto 0); 
end record;

type presc_triggers_type is record
   sample_nbt           :   std_logic; --Sample signal for nominal bit time
   sample_dbt           :   std_logic; --Sample signal of data bit time
   sample_nbt_del_1     :   std_logic;
   sample_dbt_del_1     :   std_logic;
   sample_nbt_del_2     :   std_logic;
   sample_dbt_del_2     :   std_logic;
   sync_nbt             :   std_logic;
   sync_dbt             :   std_logic;
   sync_nbt_del_1       :   std_logic;
   sync_dbt_del_1       :   std_logic;
end record;


-----------------------------------------------------------------------------------------
--RX Buffer types
-----------------------------------------------------------------------------------------
type CAN_frame_type is record
   rec_ident_in         : std_logic_vector(28 downto 0);    --Message Identifier
   rec_data_in          : std_logic_vector(511 downto 0);   --Message Data (up to 64 bytes);
   rec_dlc_in           : std_logic_vector(3 downto 0);     --Data length code
   rec_ident_type_in    : std_logic;                        --Recieved identifier type (0-BASE Format, 1-Extended Format);
   rec_frame_type_in    : std_logic;                        --Recieved frame type (0-Normal CAN, 1- CAN FD)
   rec_is_rtr           : std_logic;                        --Recieved frame is RTR Frame(0-No, 1-Yes)
   rec_brs              : std_logic;                        --Whenever frame was recieved with BIT Rate shift 
   rec_esi              : std_logic;                        --Error state indicator
   rec_message_valid    : std_logic;                        --Output from acceptance filters (out_ident_valid) if message fits the filters
end record;


-----------------------------------------------------------------------------------------
--Avalon memory interface type for shorter access
-----------------------------------------------------------------------------------------
type Avalon_mem_type is record
      clk_sys    :     std_logic;
      data_in    :     std_logic_vector(31 downto 0);
      data_out   :     std_logic_vector(31 downto 0);
      address    :     std_logic_vector(23 downto 0);
      scs        :     std_logic;
      swr        :     std_logic;
      srd        :     std_logic;
end record;

-----------------------------------------------------------------------------------------
-- Main Bus timing configuration type used in feature and sanity tests
-----------------------------------------------------------------------------------------
type bit_time_config_type is record
     tq_nbt      :    natural;
     tq_dbt      :    natural;
     prop_nbt    :    natural;
     ph1_nbt     :    natural;
     ph2_nbt     :    natural;
     sjw_nbt     :    natural;
     prop_dbt    :    natural;
     ph1_dbt     :    natural;
     ph2_dbt     :    natural;
     sjw_dbt     :    natural;
end record;

type SW_CAN_frame_type is record
   identifier           : natural;    --Message Identifier
   data                 : std_logic_vector(511 downto 0);   
   dlc                  : std_logic_vector(3 downto 0);     --Data length code
   data_length          : natural range 0 to 64;            --Data length in integer type
   ident_type           : std_logic;                        --Recieved identifier type (0-BASE Format, 1-Extended Format);
   frame_format         : std_logic;                        --Recieved frame type (0-Normal CAN, 1- CAN FD)
   rtr                  : std_logic;                        --Recieved frame is RTR Frame(0-No, 1-Yes)
   brs                  : std_logic;                        --Whenever frame was recieved with BIT Rate shift 
   timestamp            : std_logic_vector(63 downto 0);
end record;


type tran_delay_type is record
    tx_delay_sr         : std_logic_vector(255 downto 0);
    rx_delay_sr         : std_logic_vector(255 downto 0);
    tx_point            : std_logic;
    rx_point            : std_logic;
end record;



-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------
-- Constants
-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------
constant f100_MHZ :natural := 10000;

-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------
-- Functions definitions
-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------
--Function called at the end of the test which sets the
--status of the test based on the amount of errors and
--error threshold
-----------------------------------------------------------------------------------------
procedure evaluate_test
  (signal error_th : in natural;
   signal errors   : in natural;
   signal status   : out  test_status_type      -- Status of the test  
  );

-----------------------------------------------------------------------------------------
--Generates clock signal for the test
-----------------------------------------------------------------------------------------

procedure generate_clock
  (variable period      : in natural;       --Period in picoseconds 
   variable duty        : in natural;       --Duty cycle of the clock
   variable epsilon_ppm : in natural;       --Clock uncertainty in ppm
   signal   out_clk     : out std_logic     --Logic type of the clock
  );

-----------------------------------------------------------------------------------------
--Reports the message when severity  level is set lower or
-- equal as the severity of the message
-----------------------------------------------------------------------------------------
procedure log
  (constant Message     : in String;
   constant log_severity: in log_lvl_type;  --Severity of logged message
   signal   log_level   : in log_lvl_type   --Actual log level that is set
   );

-----------------------------------------------------------------------------------------
-- Asserts the res_n signal to active low to restart the operated circuit.
-- Waits until the run signal is true and sets the status type to running
-- and deactivates the res_n signal.
-----------------------------------------------------------------------------------------   
procedure reset_test
   (signal res_n        : out std_logic;        --Reset signal that should be pulled low
    signal status       : out test_status_type; --Output status of the test
    signal run          : in  boolean;          --Trigger that procedure waits for
                                                -- to come true
    signal error_ctr    : out natural                                            
    );
    
procedure process_error
    (signal error_ctr  : inout natural;         --Error counter which will be increased
     signal error_beh  : in  err_beh_type;    --Error behaviour of the test
     signal exit_imm   : out boolean          -- Result whether test should be finished
                                              -- immediately
     );
     
procedure print_test_info
    (signal iterations     :in   natural;                -- Number of iterations that test should do
     signal log_level      :in   log_lvl_type;           -- Logging level, severity which should be shown
     signal error_beh      :in   err_beh_type;           -- Test behaviour when error occurs: Quit, or Go on
     signal error_tol      :in   natural                -- Error tolerance, error counter should not 
    );
    
procedure decode_dlc(
    signal rec_dlc : in std_logic_vector(3 downto 0);
    variable dlc   : out natural
  );
  
procedure decode_dlc_v(
    variable rec_dlc : in std_logic_vector(3 downto 0);
    variable dlc     : out natural
  );
  
procedure decode_dlc_buff(
    signal rec_dlc        : in std_logic_vector(3 downto 0);
    variable buff_space   : out natural
  );
    
procedure generate_simple_trig(
    signal    rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
    signal    sync            : out std_logic;
    signal    sample          : out std_logic;
    signal    clk_sys         : in  std_logic;
    variable  min_diff        : in  natural
  );
  
procedure generate_trig(
    signal    sync            : out std_logic;
    signal    sample          : out std_logic;
    signal    clk_sys         : in  std_logic;
    signal    seg1            : in  natural;
    signal    seg2            : in  natural
  );
  

-----------------------------------------------------------------------------------------
-- Avalon access routines
-----------------------------------------------------------------------------------------  
  procedure aval_write(
    variable  w_data     :   in    std_logic_vector(31 downto 0);
    variable  w_address  :   in    std_logic_vector(23 downto 0);
    signal    mem_bus    :   inout Avalon_mem_type
  );
  
  procedure aval_read(
    variable  r_data     :   out  std_logic_vector(31 downto 0);
    variable  r_address  :   in   std_logic_vector(23 downto 0);
    signal    mem_bus    :   inout Avalon_mem_type
  );
  
  
  procedure CAN_write(
    variable  w_data     :   in    std_logic_vector(31 downto 0);
    constant  w_offset   :   in    std_logic_vector(11 downto 0);
    variable  ID         :   in    natural range 0 to 15;
    signal    mem_bus    :   inout Avalon_mem_type
  );
  
  procedure CAN_read(
    variable  r_data     :   out   std_logic_vector(31 downto 0);
    constant  r_offset   :   in    std_logic_vector(11 downto 0);
    variable  ID         :   in    natural range 0 to 15;
    signal    mem_bus    :   inout Avalon_mem_type
  ); 

-----------------------------------------------------------------------------------------
-- CAN configuration routines
-----------------------------------------------------------------------------------------    
  procedure CAN_configure_timing(
    signal   bus_timing  :   in     bit_time_config_type;
    variable ID          :   in     natural range 0 to 15;
    signal   mem_bus     :   inout  Avalon_mem_type    
  );
  
  procedure CAN_read_timing(
    signal   bus_timing  :   out    bit_time_config_type;
    variable ID          :   in     natural range 0 to 15;
    signal   mem_bus     :   inout  Avalon_mem_type    
  );
  
  procedure CAN_turn_controller(
    constant turn_on     :   in     boolean;
    variable ID          :   in     natural range 0 to 15;
    signal   mem_bus     :   inout  Avalon_mem_type    
  );
  
  procedure CAN_enable_retr_limit(
    constant turn_on     :   in     boolean;
    constant limit       :   in     natural range 0 to 15;
    variable ID          :   in     natural range 0 to 15;
    signal   mem_bus     :   inout  Avalon_mem_type    
  );
  
  procedure CAN_generate_frame(
    signal  rand_ctr     :  inout   natural range 0 to RAND_POOL_SIZE;
    variable frame       :  inout   SW_CAN_frame_type
  );
  
  procedure CAN_compare_frames(
    signal frame_A       :  in      SW_CAN_frame_type;
    signal frame_B       :  in      SW_CAN_frame_type;
    constant comp_ts     :  in      boolean;
    variable outcome     :  inout   boolean
  );
  
  procedure CAN_compare_frames_v(
    variable frame_A     :  in      SW_CAN_frame_type;
    variable frame_B     :  in      SW_CAN_frame_type;
    constant comp_ts     :  in      boolean;
    variable outcome     :  inout   boolean
  );
  
  procedure CAN_send_frame(
    variable frame       :  in      SW_CAN_frame_type;
    constant buf_nr      :  in      natural range 1 to 2;
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type;
    variable outcome     :  out     boolean
  );
  
  procedure CAN_read_frame(
    variable frame       :  inout   SW_CAN_frame_type;
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type
  );
  
  procedure CAN_wait_frame_sent(
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type
  );
  
  procedure CAN_wait_bus_idle(
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type
  );
  
  procedure CAN_wait_error_transmitted(
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type
  );
  
  procedure CAN_calc_frame_length(
  variable frame          : in   SW_CAN_frame_type;
  variable bit_length     : inout  natural
 );
  
end package;



package body CANtestLib is

---------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------
-- Functions implementations
---------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------
-- Evaluates the test based on error tolerance and amoun of errors. Test is passed  
-- when amount of errrors is less than error tolerance. Otherwise it fails...
---------------------------------------------------------------------------------------
procedure evaluate_test
  (signal error_th : in   natural;
   signal errors   : in   natural;
   signal status   : out  test_status_type      -- Status of the test  
  ) is
begin
  
  if (errors < error_th or errors = error_th)then
    status    <= passed;
    wait for 200 ns;
    assert false report "Test finished: SUCCESS" severity failure; 
  else
    status    <= failed;
    wait for 200 ns;
    assert false report "Test finished: FAILURE" severity failure; 
  end if;

end procedure;

---------------------------------------------------------------------------------------
-- Generate clock signal with given period, uncertainty, and duty cycle
---------------------------------------------------------------------------------------
procedure generate_clock
  (variable period      : in natural;    --Period in picoseconds 
   variable duty        : in natural;    --Duty cycle of the clock
   variable epsilon_ppm : in natural;    --Clock uncertainty in ppm   
  signal   out_clk     : out std_logic --Logic type of the clock
  ) is
variable real_period :real;
variable rand_nr     :real;
variable high_per    :real;
variable low_per     :real;
variable high_time   :time;
variable low_time    :time;
begin
  
  --If clock has uncertainty then it is constantly added to the clock!
  --This covers the worst case!!
  real_period   := real(period) + (real(period*epsilon_ppm))/1000000.0; 
  high_per      := ( (real(duty))*real_period)/100.0;
  low_per       := ( (real(100-duty)) *real_period)/100.0;
  high_time     :=  integer(high_per*1000.0) * 1 ns;
  high_time     := high_time/1000000;
  low_time      :=  integer(low_per*1000.0) * 1 ns;
  low_time      := low_time/1000000;
  
  --Generate the clock itself
  out_clk       <= '1';
  wait for high_time;
  out_clk       <= '0';
  wait for low_time;
  
end procedure;


---------------------------------------------------------------------------------------
-- Report the message to the simulator if its severity is higher than log_level.
-- Usually log_level is intended to be single signal for whole test!
---------------------------------------------------------------------------------------
procedure log
  (constant Message       : in String;
   constant log_severity  : in log_lvl_type;  --Severity of logged message
   signal log_level     : in log_lvl_type   --Actual log level that is set
   )is
begin 
  
  if(log_level=info_l)then
    if(log_severity=info_l)then
     report Message severity NOTE;
    elsif(log_severity=warning_l)then
     report Message severity WARNING;
    elsif(log_severity=error_l)then
     report Message severity ERROR; 
    else
     --We should not get here
    end if;
    
  elsif (log_level=warning_l) then
    
    if(log_severity=warning_l)then
     report Message severity WARNING;
    elsif(log_severity=error_l)then
     report Message severity ERROR; 
    else
     --Level set to warning but severity is info dont log
    end if;
  
  elsif (log_level=error_l)then
    
    if(log_severity=error_l)then
     report Message severity ERROR; 
    else
     --Level set to error but severity is info or warning dont log
    end if;
    
  else
    --We should not get here
  end if;
  
end procedure;

---------------------------------------------------------------------------------------
-- Pull the circuit reset low, wait, pull it high and set the status of the test to 
-- running.
---------------------------------------------------------------------------------------
procedure reset_test
   (signal res_n        : out std_logic;        --Reset signal that should be pulled low
    signal status       : out test_status_type; --Output status of the test
    signal run          : in  boolean;          --Trigger that procedure waits for
                                                -- to come true
    signal error_ctr    : out natural
    )is
  begin
    res_n     <= '0';
    status    <= waiting;
    wait for 100 ns;
    while run=false loop
        wait for 10 ns;
    end loop;
    res_n     <= '1';
    status    <= running;
    error_ctr <= 0;
    wait for 250 ns;
  end procedure;

---------------------------------------------------------------------------------------
-- Increment the error counter annd based on error behaviour set the exit_imm 
-- parameter. Exit_imm is intended for debugging purposes. When error is detected to 
-- quit the test immediately...
---------------------------------------------------------------------------------------
procedure process_error
    (signal error_ctr    : inout natural;         --Error counter which will be increased
     signal error_beh    : in  err_beh_type;    --Error behaviour of the test
     signal exit_imm     : out boolean          -- Result whether test should be finished
                                                -- immediately
     )is
  begin
    error_ctr <= error_ctr+1;
        
    if(error_beh=quit)then
      exit_imm <= true;
    else
      exit_imm <= false;
    end if;
    
    wait for 0 ns;
    
  end procedure;

---------------------------------------------------------------------------------------
-- Print the main informations about the test in the beginning of the test
---------------------------------------------------------------------------------------
  procedure print_test_info
    (signal iterations     :in   natural;                -- Number of iterations that test should do
     signal log_level      :in   log_lvl_type;           -- Logging level, severity which should be shown
     signal error_beh      :in   err_beh_type;           -- Test behaviour when error occurs: Quit, or Go on
     signal error_tol      :in   natural                -- Error tolerance, error counter should not 
    )is
  begin
    report "Test info:";
    report "Number of iterations: "&integer'image(iterations);
    
    if(log_level=info_l)then 
      report "Log level: INFO,WARNING,ERROR logs are shown!";
    elsif(log_level=warning_l)then
      report "Log level: WARNING,ERROR logs are shown!";
    else
      report "Log level: ERROR logs are shown!";
    end if;
    
    if(error_beh=go_on)then    
      report "When error is detected test runs on";
    else
      report "When error is detected test quits";
    end if;
      
    report "Error tolerance: "&integer'image(error_tol);
   
  end;

---------------------------------------------------------------------------------------
-- From format of recieved dlc decode the DLC to natural according to CAN FD spec.
---------------------------------------------------------------------------------------
  procedure decode_dlc(
    signal rec_dlc : in std_logic_vector(3 downto 0);
    variable dlc   : out natural
  )is 
  begin
    case rec_dlc is
    when "0000" => dlc:=0;
    when "0001" => dlc:=1;
    when "0010" => dlc:=2;
    when "0011" => dlc:=3;
    when "0100" => dlc:=4;  
    when "0101" => dlc:=5;  
    when "0110" => dlc:=6;  
    when "0111" => dlc:=7;
    when "1000" => dlc:=8;
    when "1001" => dlc:=12;
    when "1010" => dlc:=16;
    when "1011" => dlc:=20;
    when "1100" => dlc:=24;
    when "1101" => dlc:=32;
    when "1110" => dlc:=48;
    when "1111" => dlc:=64;
    when others => dlc:=0;
    end case;  
  end procedure;
  
  
  procedure decode_dlc_v(
    variable rec_dlc : in std_logic_vector(3 downto 0);
    variable dlc   : out natural
  )is 
  begin
    case rec_dlc is
    when "0000" => dlc:=0;
    when "0001" => dlc:=1;
    when "0010" => dlc:=2;
    when "0011" => dlc:=3;
    when "0100" => dlc:=4;  
    when "0101" => dlc:=5;  
    when "0110" => dlc:=6;  
    when "0111" => dlc:=7;
    when "1000" => dlc:=8;
    when "1001" => dlc:=12;
    when "1010" => dlc:=16;
    when "1011" => dlc:=20;
    when "1100" => dlc:=24;
    when "1101" => dlc:=32;
    when "1110" => dlc:=48;
    when "1111" => dlc:=64;
    when others => dlc:=0;
    end case;  
  end procedure;
  
  
  
---------------------------------------------------------------------------------------
-- From recieved dlc format decode how many 32bit words will the frame take together
-- in the RX Buffer. 
---------------------------------------------------------------------------------------  
  procedure decode_dlc_buff(
    signal   rec_dlc        : in  std_logic_vector(3 downto 0);
    variable buff_space     : out natural
  )is
  begin
    case rec_dlc is
      when "0000" => buff_space:=0+4; --Zero bits
      when "0001" => buff_space:=1+4; --1 byte
      when "0010" => buff_space:=1+4; --2 bytes
      when "0011" => buff_space:=1+4; --3 bytes
      when "0100" => buff_space:=1+4; --4 bytes
      when "0101" => buff_space:=2+4; --5 bytes
      when "0110" => buff_space:=2+4; --6 bytes
      when "0111" => buff_space:=2+4; --7 bytes
      when "1000" => buff_space:=2+4; --8 bytes
      when "1001" => buff_space:=3+4; --12 bytes
      when "1010" => buff_space:=4+4; --16 bytes
      when "1011" => buff_space:=5+4; --20 bytes
      when "1100" => buff_space:=6+4; --24 bytes
      when "1101" => buff_space:=8+4; --32 bytes
      when "1110" => buff_space:=12+4; --48 bytes
      when "1111" => buff_space:=16+4; --64 bytes
      when others => buff_space:=0;
    end case;
  end procedure;
  
  
---------------------------------------------------------------------------------------
-- Generate simple trigger signals.
-- Two signals are generated, sync and sample. There is no bit rate switching 
-- implemented in this procedure. There is random amount of clock cycles between
-- sync and sample, but not less than min_diff and not more than 10 clock cycles!
-- min_diff is intended for determining how many clock cycles each circuit determines
-- between bit transmittion and reception!
---------------------------------------------------------------------------------------  
  procedure generate_simple_trig(
    signal    rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
    signal    sync            : out std_logic;
    signal    sample          : out std_logic;
    signal    clk_sys         : in  std_logic;
    variable  min_diff        : in  natural
  )is
  variable diff  : real;
  variable round : natural;
  begin
    wait until rising_edge(clk_sys);
    sample    <=  '0';
    wait until rising_edge(clk_sys);
    wait until rising_edge(clk_sys);
    sync  <=  '1';
    
    rand_real_v(rand_ctr,diff);
    diff:=10.0*diff;
    if(integer(diff)<min_diff)then
      diff:=real(min_diff);
    end if;
    round:=integer(diff);
    
    for i in 0 to round loop
      wait until rising_edge(clk_sys);
      
      if(i=0)then
        sync <= '0';
      end if;
      
      if(i=round)then
        sample <= '1';
      end if;
    end loop;
    
  end procedure;
  
  
  ---------------------------------------------------------------------------------------
  -- Generate trigger signals.
  -- Sync a sample trigger signals are generated! Duration between sync and sample is
  -- given as seg1 and duration between sample and sync is given as seg2.
  ---------------------------------------------------------------------------------------  
  procedure generate_trig(
    signal    sync            : out     std_logic;
    signal    sample          : out     std_logic;
    signal    clk_sys         : in      std_logic;
    signal    seg1            : in      natural;
    signal    seg2            : in      natural
  )is
  begin
    wait until rising_edge(clk_sys);
    sync<='1';
    wait until rising_edge(clk_sys);
    sync<='0';
    
    for i in 1 to seg1-1 loop
      wait until rising_edge(clk_sys);
    end loop;
    
    sample<='1';
    wait until rising_edge(clk_sys);
    sample<='0';
    
    for i in 1 to seg2-1 loop
      wait until rising_edge(clk_sys);
    end loop;
    
  end procedure;
  
  
  
  -----------------------------------------------------------------------------------------
  -- Avalon access routines
  -----------------------------------------------------------------------------------------

------------------------------------------------------------
-- Write on Avalon Bus
-------------------------------------------------------------   
  procedure aval_write(
    variable  w_data     :   in    std_logic_vector(31 downto 0);
    variable  w_address  :   in    std_logic_vector(23 downto 0);
    signal    mem_bus    :   inout Avalon_mem_type
  )is
  begin
    wait until falling_edge(mem_bus.clk_sys);
    mem_bus.scs       <=  '1';
    mem_bus.swr       <=  '1';
    mem_bus.address   <=  w_address;
    mem_bus.data_in   <=  w_data;
    wait until falling_edge(mem_bus.clk_sys);
    mem_bus.scs       <=  '0';
    mem_bus.swr       <=  '0';
    mem_bus.address   <=  (OTHERS => '0');
    mem_bus.data_in   <=  (OTHERS => '0');    
  end procedure;

------------------------------------------------------------
-- Read on Avalon Bus
-------------------------------------------------------------   
  procedure aval_read(
    variable  r_data     :   out  std_logic_vector(31 downto 0);
    variable  r_address  :   in   std_logic_vector(23 downto 0);
    signal    mem_bus    :   inout Avalon_mem_type
  )is
  begin
    wait until falling_edge(mem_bus.clk_sys);
    mem_bus.scs       <=  '1';
    mem_bus.srd       <=  '1';
    mem_bus.address   <=  r_address;
    wait until falling_edge(mem_bus.clk_sys);
    r_data            :=  mem_bus.data_out;
    mem_bus.scs       <=  '0';
    mem_bus.srd       <=  '0';
    mem_bus.address   <=  (OTHERS => '0');    
  end procedure;

------------------------------------------------------------
-- Write to CAN Node
------------------------------------------------------------- 
  procedure CAN_write(
    variable  w_data     :   in    std_logic_vector(31 downto 0);
    constant  w_offset   :   in    std_logic_vector(11 downto 0);
    variable  ID         :   in    natural range 0 to 15;
    signal    mem_bus    :   inout Avalon_mem_type
  )is
  variable int_address   :   std_logic_vector(23 downto 0);
  begin
    int_address       := "0100"&std_logic_vector(to_unsigned(ID,4))&"00"&w_offset&"00";
    aval_write        (w_data,int_address,mem_bus);
  end procedure;
  
------------------------------------------------------------
-- Read from CAN Node
-------------------------------------------------------------  
  procedure CAN_read(
    variable  r_data     :   out   std_logic_vector(31 downto 0);
    constant  r_offset   :   in    std_logic_vector(11 downto 0);
    variable  ID         :   in    natural range 0 to 15;
    signal    mem_bus    :   inout Avalon_mem_type
  )is
  variable int_address   :   std_logic_vector(23 downto 0);
  begin
    int_address       := "0100"&std_logic_vector(to_unsigned(ID,4))&"00"&r_offset&"00";
    aval_read        (r_data,int_address,mem_bus);
  end procedure;
  
  
  -----------------------------------------------------------------------------------------
  -----------------------------------------------------------------------------------------
  -- Controller configuration routines
  -----------------------------------------------------------------------------------------
  ----------------------------------------------------------------------------------------- 

------------------------------------------------------------
-- Set the timing of a Node from timing structure
-------------------------------------------------------------  
  procedure CAN_configure_timing(
    signal   bus_timing  :   in     bit_time_config_type;
    variable ID          :   in     natural range 0 to 15;
    signal   mem_bus     :   inout  Avalon_mem_type    
  )is
  variable data          :          std_logic_vector(31 downto 0):=(OTHERS => '0');
  begin
    data  :=  "00"&std_logic_vector(to_unsigned(bus_timing.tq_dbt,6))&
              "00"&std_logic_vector(to_unsigned(bus_timing.tq_nbt,6))&
              std_logic_vector(to_unsigned(bus_timing.sjw_dbt,4))&
              std_logic_vector(to_unsigned(bus_timing.sjw_nbt,4))&
              "00000000";
    CAN_write(data,ARB_ERROR_PRESC_ADR,ID,mem_bus);  
    
    data  :=  '0'&std_logic_vector(to_unsigned(bus_timing.ph2_dbt,4))&
              '0'&std_logic_vector(to_unsigned(bus_timing.ph1_dbt,4))&
              "00"&std_logic_vector(to_unsigned(bus_timing.prop_dbt,4))&
              std_logic_vector(to_unsigned(bus_timing.ph2_nbt,5))&
              std_logic_vector(to_unsigned(bus_timing.ph1_nbt,5))&
              std_logic_vector(to_unsigned(bus_timing.prop_nbt,6));
    CAN_write(data,TIMING_REG_ADR,ID,mem_bus);                   
  end procedure;
  
  
  procedure CAN_read_timing(
    signal   bus_timing  :   out    bit_time_config_type;
    variable ID          :   in     natural range 0 to 15;
    signal   mem_bus     :   inout  Avalon_mem_type    
  )is
  begin
    --TODO
  end procedure;
  
------------------------------------------------------------
-- Enable/Disable controller functionality
-------------------------------------------------------------    
  procedure CAN_turn_controller(
    constant turn_on     :   in     boolean;
    variable ID          :   in     natural range 0 to 15;
    signal   mem_bus     :   inout  Avalon_mem_type    
  )is
  variable data          :          std_logic_vector(31 downto 0):=(OTHERS => '0');
  begin
    CAN_read(data,MODE_REG_ADR,ID,mem_bus);
    if turn_on then
      data(30):= '1';
    else
      data(30):= '0';
    end if;
    CAN_write(data,MODE_REG_ADR,ID,mem_bus);  
  end procedure;
  
------------------------------------------------------------
-- Enable/disable retransmittion limit of CAN Node
-------------------------------------------------------------    
  procedure CAN_enable_retr_limit(
    constant turn_on     :   in     boolean;
    constant limit       :   in     natural range 0 to 15;
    variable ID          :   in     natural range 0 to 15;
    signal   mem_bus     :   inout  Avalon_mem_type    
  )is
  variable data          :          std_logic_vector(31 downto 0):=(OTHERS => '0');
  begin
    CAN_read(data,MODE_REG_ADR,ID,mem_bus);
    if turn_on then
      data(24):= '1';
    else
      data(24):= '0';
    end if;
    data(28 downto 25):= std_logic_vector(to_unsigned(limit,4));
    CAN_write(data,MODE_REG_ADR,ID,mem_bus);
  end procedure;
  

------------------------------------------------------------
-- Generate random CAN frame
-------------------------------------------------------------    
  procedure CAN_generate_frame(
    signal  rand_ctr     :  inout   natural range 0 to RAND_POOL_SIZE;
    variable frame       :  inout   SW_CAN_frame_type
  )is
  variable rand_value    : real := 0.0;
  variable aux           : std_logic_vector(28 downto 0);
  begin
       
      rand_logic_v(rand_ctr,frame.ident_type,0.5);
      rand_logic_v(rand_ctr,frame.frame_format,0.5);
      rand_logic_v(rand_ctr,frame.rtr,0.5);
      rand_logic_v(rand_ctr,frame.brs,0.5);
      rand_logic_vect_v(rand_ctr,frame.data,0.5);
      rand_logic_vect_v(rand_ctr,frame.dlc,0.3);
      
      rand_real_v(rand_ctr,rand_value);
      rand_value := rand_value*536870911.0;
      
      --We generate only valid frame combinations to avoid problems...
      if(frame.frame_format = '1')then
        frame.rtr := '0';
      end if;
      
      if(frame.frame_format = '0')then
        frame.brs := '0';
      end if; 
      
      --Cut the identifier if it is base!
      aux := std_logic_vector(to_unsigned(integer(rand_value),29));
      if(frame.ident_type = '0')then
        aux(17 downto 0):= (OTHERS => '0');
      end if;
      frame.identifier:=to_integer(unsigned(aux));
      
      decode_dlc_v(frame.dlc,frame.data_length);
      frame.timestamp:=(OTHERS => '0');
      
      if(frame.rtr='1')then
        frame.data := (OTHERS => '0');
        frame.dlc := (OTHERS => '0');
        frame.data_length := 0;
      end if;
      
      frame.data(511-frame.data_length*8 downto 0):= (OTHERS => '0');
      
  end procedure;
  
  
------------------------------------------------------------
-- Compare two frames if they are eqaul (signal version)
-------------------------------------------------------------  
  procedure CAN_compare_frames(
    signal frame_A       :  in      SW_CAN_frame_type;
    signal frame_B       :  in      SW_CAN_frame_type;
    constant comp_ts     :  in      boolean;
    variable outcome     :  inout   boolean
  )is
  begin
    outcome := true;
    
    if(frame_A.frame_format /= frame_B.frame_format)then
      outcome:= false;
    end if;
    
    if(frame_A.ident_type /= frame_B.ident_type)then
      outcome:= false;
    end if;
    
    if(frame_A.rtr  /= frame_B.rtr)then
      outcome:= false;
    end if;
    
    if(frame_A.brs /= frame_B.brs)then
      outcome:= false;
    end if;
    
    --DLC is compared only in non-RTR frames!
    -- In RTR frames it does not necessarily have to be equal due to RTR-pref feature
    if((frame_A.rtr = '0' or frame_A.frame_format = '1') and frame_A.dlc /= frame_B.dlc)then
      outcome:=false;
    end if;
    
    if(outcome = true) then
      if((frame_A.rtr = '0' or frame_A.frame_format = '1') and frame_A.data_length /= 0)then
      
        for i in 0 to (frame_A.data_length-1)/4 loop
          if(frame_A.data(511-i*32 downto 480-i*32) /=
            frame_B.data(511-i*32 downto 480-i*32) )then
            outcome:= false;
          end if;
        end loop;  
          
      end if;
    end if;
    
  end procedure;
  
  
------------------------------------------------------------
-- Compare two frames if they are eqaul (variable version)
-------------------------------------------------------------  
   procedure CAN_compare_frames_v(
    variable frame_A     :  in      SW_CAN_frame_type;
    variable frame_B     :  in      SW_CAN_frame_type;
    constant comp_ts     :  in      boolean;
    variable outcome     :  inout   boolean
  )is
  begin
    outcome := true;
    
    if(frame_A.frame_format /= frame_B.frame_format)then
      outcome:= false;
    end if;
    
    if(frame_A.ident_type /= frame_B.ident_type)then
      outcome:= false;
    end if;
    
    -- RTR should be te same only in normal CAN
    if(frame_A.frame_format='0')then
      if(frame_A.rtr  /= frame_B.rtr)then
        outcome:= false;
      end if;
    end if;
    
    --BRS bit is compared only in FD frame
    if(frame_A.frame_format='1')then
      if(frame_A.brs /= frame_B.brs)then
        outcome:= false;
      end if;
    end if;
    
    --DLC is compared only in non-RTR frames!
    -- In RTR frames it does not necessarily have to be equal due to RTR-pref feature
    if((frame_A.rtr = '0' or frame_A.frame_format = '1') and frame_A.dlc /= frame_B.dlc)then
      outcome:=false;
    end if;
    
    if(outcome = true) then
      if((frame_A.rtr = '0' or frame_A.frame_format = '1') and frame_A.data_length /= 0)then
      
        for i in 0 to (frame_A.data_length-1)/4 loop
          if(frame_A.data(511-i*32 downto 480-i*32) /=
            frame_B.data(511-i*32 downto 480-i*32) )then
            outcome:= false;
          end if;
        end loop;  
          
      end if;
    end if;
    
  end procedure;

------------------------------------------------------------
-- Insert frame for transmittion into the CAN Node
-------------------------------------------------------------
  procedure CAN_send_frame(
    variable frame       :  in      SW_CAN_frame_type;
    constant buf_nr      :  in      natural range 1 to 2;
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type;
    variable outcome     :  out     boolean
  )is
  variable w_data          :          std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable ident_vect      :          std_logic_vector(28 downto 0):=(OTHERS => '0');
  variable length          :          natural;
  variable iter_limit      :          natural;
  variable aux_out         :          boolean;
  begin
   outcome := true;
   
   --Read whether there is place in the TXT buffer
   CAN_read(w_data,TX_STATUS_ADR,ID,mem_bus);
   if(w_data(buf_nr-1)='1')then
     outcome:=true;
     aux_out:=true;
   else
     outcome:=false;
     aux_out:=false;
     report "Unable to send the frame, TX buffer not empty" severity error;
   end if;
   
   --Access to the buffer is done only if it is signalled as empty
   if (aux_out=true) then
     
     --Set the buffer to access (direction) and forbid the buffer transmission!
     CAN_read(w_data,TX_SETTINGS_ADR,ID,mem_bus);
     if (buf_nr=1) then
        w_data(2) := '0';
        w_data(0) := '0';
     elsif (buf_nr=2) then
        w_data(2) := '1';
        w_data(1) := '0';
     else
       report "Unsupported TX buffer number" severity error;
     end if;
     CAN_write(w_data,TX_SETTINGS_ADR,ID,mem_bus); 
        
     --Frame format word
     w_data:= "0000000000000000000000"&frame.brs&'1'&frame.frame_format&frame.ident_type&
               frame.rtr&'0'&frame.dlc;
     CAN_write(w_data,TX_DATA_1_ADR,ID,mem_bus);          
           
     --Identifier
     if(frame.ident_type='1')then
        ident_vect := std_logic_vector(to_unsigned(frame.identifier,29));
        w_data:= "000"&ident_vect(17 downto 0)&ident_vect(28 downto 18);
     else
        ident_vect := "000000000000000000"&std_logic_vector(to_unsigned(frame.identifier,11));
        w_data:= "000000000000000000000"&ident_vect(10 downto 0);
     end if;
     CAN_write(w_data,TX_DATA_2_ADR,ID,mem_bus);
     
      --Timestamp
     w_data:= frame.timestamp(31 downto 0);  
     CAN_write(w_data,TX_DATA_3_ADR,ID,mem_bus);
     w_data:= frame.timestamp(63 downto 32);  
     CAN_write(w_data,TX_DATA_4_ADR,ID,mem_bus);
     
     --Data words
     decode_dlc_v(frame.dlc,length);
     for i in 0 to (length-1)/4 loop
       w_data:= frame.data(511-i*32 downto 480-i*32);
       CAN_write(w_data,TX_DATA_5_ADR+i,ID,mem_bus);
     end loop;
     
     --Signal that the frame is valid by allowing the buffer
     CAN_read(w_data,TX_SETTINGS_ADR,ID,mem_bus);
     if (buf_nr=1) then
        w_data(0) := '1';
     elsif (buf_nr=2) then
        w_data(1) := '1';
     else
       report "Unsupported TX buffer number" severity error;
     end if;
     CAN_write(w_data,TX_SETTINGS_ADR,ID,mem_bus);
     
  end if;
   
  end procedure;
  
------------------------------------------------------------
-- Reads CAN frame from the RX Buffer
-------------------------------------------------------------  
  procedure CAN_read_frame(
    variable frame       :  inout   SW_CAN_frame_type;
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type
  )is
  variable r_data        :          std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable aux_vect      :          std_logic_vector(28 downto 0):=(OTHERS => '0');
  begin
    
    --Read Frame format word
    CAN_read(r_data,RX_DATA_ADR,ID,mem_bus);   
    frame.dlc           := r_data(3 downto 0);
    frame.rtr           := r_data(5);
    frame.ident_type    := r_data(6);
    frame.frame_format  := r_data(7);
    frame.brs           := r_data(9);
    decode_dlc_v(frame.dlc,frame.data_length);
    
    --Read identifier
    CAN_read(r_data,RX_DATA_ADR,ID,mem_bus);
    if(frame.ident_type='1')then
      aux_vect         := r_data(10 downto 0)&r_data(28 downto 11);
      frame.identifier := to_integer(unsigned(aux_vect));
    else
      aux_vect         := "000000000000000000"&r_data(10 downto 0);
      frame.identifier := to_integer(unsigned(aux_vect));
    end if;
    
    --Read timestamp
    CAN_read(r_data,RX_DATA_ADR,ID,mem_bus);  
    frame.timestamp(31 downto 0)  := r_data;
    CAN_read(r_data,RX_DATA_ADR,ID,mem_bus);  
    frame.timestamp(63 downto 32) := r_data;
    
    --Now read data frames
    if((frame.rtr = '0' or frame.frame_format = '1') and frame.data_length /= 0)then
      
      for i in 0 to (frame.data_length-1)/4 loop
        CAN_read(r_data,RX_DATA_ADR,ID,mem_bus);
        frame.data(511-i*32 downto 480-i*32) := r_data;
      end loop;
            
    end if;
  end procedure;
  
------------------------------------------------------------
-- Reads out in a loop status of CAN Node until node starts
-- transmitting frame and until the transmittion is finished
-------------------------------------------------------------  
  procedure CAN_wait_frame_sent(
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type
  )is
  variable r_data        :          std_logic_vector(31 downto 0):=(OTHERS => '0');
  begin
     
     --Wait until unit starts to transmitt or reciesve
     CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     while (r_data(16+RS_IND)='0' and r_data(16+TS_IND)='0') loop
       CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     end loop;
     
     --Wait until bus is idle now
     CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     while (r_data(16+BS_IND)='0') loop
       CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     end loop;
     
  end procedure;
  
------------------------------------------------------------
-- Reads out in a loop status of CAN Node until node is in
-- IDLE state of interframe
-------------------------------------------------------------
  procedure CAN_wait_bus_idle(
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type
  )is
  variable r_data        :          std_logic_vector(31 downto 0):=(OTHERS => '0');
  begin
      --Wait until bus is idle
     CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     while (r_data(16+BS_IND)='0') loop
       CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     end loop;
  end procedure;


------------------------------------------------------------
-- Reads out in a loop status of CAN Node until node starts
-- transmitting error frame
-------------------------------------------------------------  
  procedure CAN_wait_error_transmitted(
    variable ID          :  in      natural range 0 to 15;
    signal   mem_bus     :  inout   Avalon_mem_type
  )is
  variable r_data        :          std_logic_vector(31 downto 0):=(OTHERS => '0');
  begin
     
     --Wait until unit starts to transmitt or recieve
     CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     while (r_data(16+RS_IND)='0' and r_data(16+TS_IND)='0') loop
       CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     end loop;
     
     --Wait until error frame is not being transmitted
     CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     while (r_data(16+ET_IND)='0') loop
       CAN_read(r_data,MODE_REG_ADR,ID,mem_bus);
     end loop;
     
  end procedure;
  
------------------------------------------------------------
-- Calculates length of the frame in bits from the 
-- SW_CAN_frame_type structure
-------------------------------------------------------------
 procedure CAN_calc_frame_length(
  variable frame          : in     SW_CAN_frame_type;
  variable bit_length     : inout  natural
 )is
  variable aux            :   std_logic_vector(1 downto 0);
  variable data_length    :   natural;
 begin
   decode_dlc_v(frame.dlc,data_length);
   if(frame.rtr='1' and frame.frame_format='0')then
     data_length:=0;
   end if;
   
   --Join the ident type and frame type
   aux:= frame.ident_type&frame.frame_format;
   
   --Calculated identifer and control length
   case aux is
   when "00" =>
      bit_length:=18;
   when "01" =>   
      bit_length:=23;
   when "10" =>   
      bit_length:=39;   
   when "11" =>   
      bit_length:=41;
    when others =>
   end case; 
   
   --Add the data length field    
   bit_length:=bit_length+data_length;
   
   --Add CRC
   if(data_length<9)then
     bit_length:=bit_length+15+3;
  elsif(data_length<17)then
     bit_length:=bit_length+17+3;
  else
     bit_length:=bit_length+21+3;
  end if;
     
 end procedure;  
  
  
end package body;


USE work.CANtestLib.All;

-------------------------------------------------------------
--Main test entity. Each test of  the unit test environment
-- implements architecture of this entity!       
-------------------------------------------------------------
entity CAN_test is
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
  
 --Internal test signals
 signal error_ctr   :natural:=0;
 signal loop_ctr    :natural:=0;
 signal exit_imm    :boolean:=false;
 signal rand_ctr    :natural range 0 to 3800 := 0;
 
 end entity;
 
Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
USE work.randomLib.All;
use work.CANconstants.all;
 USE work.CANtestLib.All;
 
 -------------------------------------------------------------
--Test enity for feature tests. Additional signals representing
-- two memory buses are present to connect two DUTs of feature
-- tests!     
-------------------------------------------------------------
 entity CAN_feature_test is
  port (
    signal run            :in   boolean;                -- Input trigger, test starts running when true
    signal iterations     :in   natural;                -- Number of iterations that test should do
    signal log_level      :in   log_lvl_type;           -- Logging level, severity which should be shown
    signal error_beh      :in   err_beh_type;           -- Test behaviour when error occurs: Quit, or Go on
    signal error_tol      :in   natural;                -- Error tolerance, error counter should not
                                                         -- exceed this value in order for the test to pass
    signal status         :out  test_status_type;      -- Status of the test
    signal errors         :out  natural;                -- Amount of errors which appeared in the test
    
    signal mem_bus_1      :inout Avalon_mem_type;
    signal mem_bus_2      :inout Avalon_mem_type;
    
    signal bl_inject      :in   std_logic;
    signal bl_force       :in   boolean
  );
  
 --Internal test signals
 signal error_ctr   :natural:=0;
 signal loop_ctr    :natural:=0;
 signal exit_imm    :boolean:=false;
 signal rand_ctr    :natural range 0 to 3800 := 0;
 
 end entity;
 
 


USE work.CANtestLib.All; 
  
---------------------------------------------------
-- Test wrapper. When executable test is created
-- it implements architecture of this entity! Note
-- that within one test wrapper architecture several
-- tests can be implemented!!
----------------------------------------------------
entity CAN_test_wrapper is
  generic(
    constant iterations   :     natural       :=  1000;
    constant log_level    :     log_lvl_type  :=  warning_l;
    constant error_beh    :     err_beh_type  :=  go_on;
    constant error_tol    :     natural       :=  0
    );
  port(
    signal status         :out  test_status_type
  );    

end entity;