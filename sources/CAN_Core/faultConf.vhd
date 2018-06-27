Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
USE WORK.CANconstants.ALL;

-------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
--
--    June 2015  Created file
--    19.6.2016  Modified counters for error couting in both FD and NORMAL mode. Counters extended to 
--               16 bits wide, to match the format in the registers!
--    27.6.2016  Bug fix. Changed error warning limit reached detection to greater than and equal instead
--                of only equal.
--    30.6.2016  Bug fix. Added equal or greater to fault confinement error passive state. According
--               to CAN spec. error counter value equal or greater than 128 is error passive,
--               not only greater than!
--
-------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Circuit handling Fault Confinement. Bit Error is also detected in this circuit. Error counters   
--  increment is handled here by signals inc_one, inc_eight, dec_one. Logic for signalling this incre
--  ments is in Protocol control. RX TX counters for fault confinement are availiable. Two more coun
--  ters are availiable to distinguish between errors in Data phase and normal phase.                
--  All counters are pressetable from driving bus. Treshold for signalling error warning limit and    
--  transition to error_pssive are also parameters given by driving bus. Default values are compliant 
--  with CAN FD standard.
-------------------------------------------------------------------------------------------------------------

entity faultConf is 
  PORT(
    -------------------
    --Clock and reset--
    -------------------
    signal clk_sys                :in   std_logic; --System clock
    signal res_n                  :in   std_logic; --Async reset
    
    --Driving registers interface-
    signal drv_bus                :in   std_logic_vector(1023 downto 0);
    
    -----------------
    --Error inputs --
    -----------------
    signal stuff_Error            :in   std_logic; --Stuffing Error from bit destuffing
      
    ----------------------------------
    --Error signalling for interrupt--
    ----------------------------------
    signal error_valid            :out  std_logic; --At least one error appeared
    signal error_passive_changed  :out  std_logic; --Error passive state changed
    signal error_warning_limit    :out  std_logic; --Error warning limit was reached
    
    ----------------------
    --OP State interface--
    ----------------------
    signal OP_State               :in   oper_mode_type;
    
    ----------------------------------
    --Driving and trigerring signals--
    ----------------------------------
    --Recieved data. Valid with the same signal as rec_trig in CAN Core
    signal data_rx                :in   std_logic; 
    
    --Transcieved data by CAN Core. 
    --valid with one clk_sys delay from tran_trig! The same trigger signal as Bit-Stuffing!
    signal data_tx                :in   std_logic;   
    signal rec_trig               :in   std_logic; --Recieve data trigger
    
    --Transcieve data trigger one clk_sys delayed behind the tran_trig
    signal tran_trig_1            :in   std_logic; 
    
    ----------------------
    --PC State Interface--
    ----------------------
    signal PC_State               :in   protocol_type;
    signal sp_control             :in   std_logic_vector(1 downto 0);  
    signal form_Error             :in   std_logic; --Form Error from PC State
    signal CRC_Error              :in   std_logic; --CRC Error from PC State
    signal ack_Error              :in   std_logic; --Acknowledge Error from PC State
    signal unknown_state_Error    :in   std_logic; --Some of the state machines, or signals 
                                                   --reached unknown state!! Shouldnt happend!!
    signal bit_stuff_Error_valid  :out  std_logic; --Error signal for PC control FSM from fault
                                                   -- confinement unit (Bit error or Stuff Error appeared)
    signal bit_Error_out          :out  std_logic;
    
    --Note: This new interface is used for error incrementation, decrementation!!
    signal inc_one                :in   std_logic;
    signal inc_eight              :in   std_logic;
    signal dec_one                :in   std_logic;
        
    signal enable                 :in   std_logic; --Enable for error counting
    signal bit_Error_sec_sam      :in   std_logic; --Bit Error detected with secondary sampling point at busSync.vhd
    
    -------------------
    --Status outputs --
    -------------------
    --Error counters
    signal tx_counter_out         :out  std_logic_vector(8 downto 0);
    signal rx_counter_out         :out  std_logic_vector(8 downto 0);
    signal err_counter_norm_out   :out  std_logic_vector(15 downto 0);
    signal err_counter_fd_out     :out  std_logic_vector(15 downto 0);
    
    --Fault confinement status
    signal error_state_out        :out  error_state_type
    
  );

 -----------------------
 --Internal registers --
 -----------------------
 
 --Error signal for PC control FSM from fault confinement unit 
 -- (Bit error or Stuff Error appeared)
 signal bit_stuff_Error_valid_r   :     std_logic; 
 
 --Internal bit Error detection (out of BusSync)
 --Note:Bit Error detection functionality moved from busSync.vhd to this module due to compactness reasons!
 --Note 2: Only bit error detection in busSync.vhd  is for secondary sample point!!
  signal bit_Error_int            :     std_logic;   
 
 --Error state
 signal error_state               :     error_state_type;  
  
 --Error counters
 signal tx_counter                :     natural range 0 to 511;
 signal rx_counter                :     natural range 0 to 511;
 signal err_counter_norm          :     natural range 0 to 65535;
 signal err_counter_fd            :     natural range 0 to 65535;
 --Note: Maximal increase of error counter is 8. Before bus off state the highest value of Error counter is 255.
 --Therefore 303 is the biggest possible value of the counter after switching to bus off!
 --303 is 9 bits. 9 bits cover up to 512 range!!
 
 --Interrupt registers
 signal ewl_reached               :     std_logic; 
 signal error_warning_limit_reg   :     std_logic; --Registred value of error warning limit reached
 signal erp_prev_state            :     error_state_type; --Value of previous fault conf state to detect changes
 signal erp_changed_reg           :     std_logic;
 signal error_valid_reg           :     std_logic; 
 
 signal joined_ctr                :     std_logic_vector(2 downto 0);
 
 -----------------------
 --Driving bus aliases--  
 -----------------------
 signal drv_ewl                   :     std_logic_vector(7 downto 0);
 signal drv_erp                   :     std_logic_vector(7 downto 0);
 signal drv_ctr_val               :     std_logic_vector(8 downto 0);
 signal drv_ctr_sel               :     std_logic_vector(3 downto 0);
 
 --Bus off treshold
 constant bus_off_th              :     natural:=255;
   
end entity;


architecture rtl of faultConf is
begin
  
  --Driving bus aliases
  drv_ewl                 <=  drv_bus(DRV_EWL_HIGH downto DRV_EWL_LOW);
  drv_erp                 <=  drv_bus(DRV_ERP_HIGH downto DRV_ERP_LOW);
  drv_ctr_val             <=  drv_bus(DRV_CTR_VAL_HIGH downto DRV_CTR_VAL_LOW);
  drv_ctr_sel             <=  drv_bus(DRV_CTR_SEL_HIGH downto DRV_CTR_SEL_LOW);
  
  --Counters to output propagation
  tx_counter_out          <=  std_logic_vector(to_unsigned(tx_counter,9));
  rx_counter_out          <=  std_logic_vector(to_unsigned(rx_counter,9));
  err_counter_norm_out    <=  std_logic_vector(to_unsigned(err_counter_norm,16));
  err_counter_fd_out      <=  std_logic_vector(to_unsigned(err_counter_fd,16));
  
  --Fault confinement state propagation
  error_state_out         <=  error_state;
  
  --Interrupt outputs 
  error_warning_limit     <=  error_warning_limit_reg; --Error warning limit reached
  error_passive_changed   <=  erp_changed_reg; --Error passive state changed
  error_valid             <=  error_valid_reg; --At least one valid error appeared
  
  --Bit Error or stuff error register to output propagation
  bit_stuff_Error_valid   <=  bit_stuff_Error_valid_r;
  bit_Error_out           <=  bit_Error_int;
  
  --Detecting bit Error
  bit_Error_int           <=  bit_Error_sec_sam when sp_control=SECONDARY_SAMPLE                  else
                              '1'               when ((rec_trig='1') and (not(data_tx=data_rx)))  else 
                              '0';
  
  joined_ctr              <=  inc_one&inc_eight&dec_one;
  
  ----------------------------------------
  --Bit Error and Stuff Error validation--
  ----------------------------------------
  b_s_error_proc:process(clk_sys,res_n)
  begin
  if res_n=ACT_RESET then
     bit_stuff_Error_valid_r        <=  '0'; 
  elsif rising_edge(clk_sys)then
    if( ( OP_State=transciever 
          and 
          ( 
            PC_State=control or 
            PC_State=data    or 
            PC_State=crc
          ) 
          and 
          (bit_Error_int='1')
        )
        or
        ( (stuff_Error='1') 
          and 
          (not(sp_control=SECONDARY_SAMPLE))
        )
      )then
          bit_stuff_Error_valid_r   <=  '1';
    else
          bit_stuff_Error_valid_r   <=  '0';
    end if; 
  end if;
  end process;
  
  
  -----------------------------------------
  --Transcieve Counter assignment process 
  -----------------------------------------
  tx_err_ctr_proc:process(clk_sys,res_n)
  begin
    if(res_n=ACT_RESET)then
      tx_counter                            <= 0;
    elsif rising_edge(clk_sys)then
      
      tx_counter                            <=  tx_counter;
      
      --Presetting the counter from registers
      if(drv_ctr_sel(0)='1')then 
        tx_counter                          <=  to_integer(unsigned(drv_ctr_val));
      else
        --Counting the errors when transmitting
        if(OP_State=transciever)then
          case joined_ctr is 
          when INC_ONE_CON   => tx_counter  <=  tx_counter+1;
          when INC_EIGHT_CON => tx_counter  <=  tx_counter+8;
          when DEC_ONE_CON   =>  
                if(tx_counter>0)then 
                  tx_counter                <=  tx_counter-1;
                else
                  tx_counter                <=  0;
                end if;
          when others=> tx_counter          <=  tx_counter;
        	 end case;          
        end if;
         
      end if;      
    end if;
  end process tx_err_ctr_proc;
  
  
  ---------------------------------------
  --Recieve Counter assignment process
  ---------------------------------------
  rx_err_ctr_proc:process(clk_sys,res_n)
  begin
    if(res_n=ACT_RESET)then
      rx_counter                            <=  0;
    elsif rising_edge(clk_sys)then
      
      rx_counter                            <=  rx_counter;
      
      --Presetting the counter from registers
      if(drv_ctr_sel(1)='1')then 
        rx_counter                          <=  to_integer(unsigned(drv_ctr_val));
      else
        
        if(OP_State=reciever)then
          case joined_ctr is 
          when INC_ONE_CON   => rx_counter  <=  rx_counter+1;
          when INC_EIGHT_CON => rx_counter  <=  rx_counter+8;
          when DEC_ONE_CON   =>  
                if(rx_counter<127)then
                   if(rx_counter>0)then 
                      rx_counter            <=  rx_counter-1; 
                    else 
                      rx_counter            <=  0; 
                   end if;
                else
                 rx_counter                 <=  120;
                end if;
          when others=> rx_counter          <=  rx_counter;
        	 end case;          
        end if; 

       end if;      
    end if;
  end process rx_err_ctr_proc;

  ------------------------------------------------------------
  --Special counters for counting errors in two data rates
  ------------------------------------------------------------
  spec_err_ctr_proc:process(clk_sys,res_n)
  begin
    if(res_n=ACT_RESET)then
      err_counter_norm                      <=  0;
      err_counter_fd                        <=  0;
    elsif rising_edge(clk_sys)then
      err_counter_norm                      <=  err_counter_norm;
      err_counter_fd                        <=  err_counter_fd;
      
      --Erasing the counters from registers!
      if(drv_ctr_sel(2)='1' or drv_ctr_sel(3)='1')then 
        
        if(drv_ctr_sel(2)='1')then 
          err_counter_norm                  <=  to_integer(unsigned(drv_ctr_val)); 
        end if;
        if(drv_ctr_sel(3)='1')then 
          err_counter_fd                    <=  to_integer(unsigned(drv_ctr_val)); 
        end if;
      
      else
        --Couting the errors in two modes separately
        if(sp_control=NOMINAL_SAMPLE)then
          if(inc_one='1' or inc_eight='1')then
            err_counter_norm                <=  err_counter_norm+1;
          end if;
        else
          if(inc_one='1' or inc_eight='1')then
            err_counter_fd                  <=  err_counter_fd+1;
          end if;
        end if;
      
      end if;
      
    end if; 
  end process;


  ------------------------------------------
  --Fault Confinement process
  ------------------------------------------
  err_state_proc:process(clk_sys,res_n)
  begin
    if(res_n=ACT_RESET)then
      error_state                           <=  error_active;  
    elsif rising_edge(clk_sys)then
      if(tx_counter>bus_off_th or rx_counter>bus_off_th)then
        error_state                         <=  bus_off;  
      elsif(tx_counter>=unsigned(drv_erp) or rx_counter>=unsigned(drv_erp)) then  
        error_state                         <=  error_passive;
      else
        error_state                         <=  error_active;      
      end if;           
    end if;
  end process;
  
  -------------------------------------------------------------------
  --Interrupt Signalling for Error warning limit and State changed 
  -------------------------------------------------------------------
  err_wrn_proc:process(clk_sys,res_n)
  begin
    if(res_n=ACT_RESET)then
      ewl_reached                           <=  '0';
      erp_prev_state                        <=  error_active;
      erp_changed_reg                       <=  '0';
      error_valid_reg                       <=  '0';
      error_warning_limit_reg               <=  '0';
    elsif rising_edge(clk_sys)then
      
      --Error passive transition detection
      erp_prev_state                        <=  error_state;
      if(erp_prev_state=error_state)then
        erp_changed_reg                     <=  '0';
      else
        erp_changed_reg                     <=  '1';
      end if;
      
      --Error warning limit transition detection
      if((tx_counter>=unsigned(drv_ewl) or rx_counter>=unsigned(drv_ewl)) and ewl_reached='0')then
        ewl_reached                         <=  '1';
        error_warning_limit_reg             <=  '1';
      elsif((tx_counter<unsigned(drv_ewl)) and (rx_counter<unsigned(drv_ewl)) )then
        ewl_reached                         <=  '0';
        error_warning_limit_reg             <=  '0';
      else
        ewl_reached                         <=  ewl_reached;
        error_warning_limit_reg             <=  '0';
      end if;
      
      --At least one of the errors immediately appeared
      if(bit_stuff_Error_valid_r='1'  or 
         form_Error='1'               or 
         CRC_Error='1'                or 
         ack_Error='1'
        )
      then
        error_valid_reg                     <=  '1';
      else
        error_valid_reg                     <=  '0';
      end if;
      
    end if;
  end process;
  
  
end architecture;