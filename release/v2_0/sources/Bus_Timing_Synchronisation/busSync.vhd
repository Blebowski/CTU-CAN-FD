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
--    July 2015   Created file
--    19.12.2015  Added tripple sampling mode. Furthermore sampling is disabled when 
--                whole controller is disabled
--    15.6.2016   1.edge_tx_valid signal now provides edge detection only from RECESSIVE to DOMINANT
--                  values! In CAN FD standard EDL and r0 bits are used for TRD measurment! When busSync is
--                  configured to start on TX edge and finish on RX edge.
--                2.Changed trv_delay size to cover mostly 127 clock cycles! Changed shift register sizes
--                  to 130 to avoid missing secondary sample signals!!
--                3.Fixed tripple sampling mode selection from three. Added missing signals
--                4. trv_to_restart signal added to make it impossible to restart trv delay measurment
--                   without rising edge on trv_delay calib signal. This removes a bug when trv_delay_calib
--                   is forgottern active and circuit keeps measuring on every edge...
--    27.6.2016   Bug fix added. Transciever delaye measurment conditions switched. Now starting edge is the
--                most prioritized logic. Thus transciever delay counter is always erased with TX edge!
--                 'elsif(edge_tx_valid='1')' swapped with 'if(trv_running='1')'. Thisway if more TX edge would
--                come before first RX edge is sampled, shorter time would be measured! Note that this is 
--                OK since in CAN spec. EDL bit is in nominal bit time and no other edge can be transmitted
--                before it is recieved (condition of original CAN)
--
-------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------
-- Purpose:
--  1. Implements Generic Synchronisation chain for incoming data
--  2. Detects appropriate edge for synchronisation!!
--  3. Measure Transciever delay compensation on command
--  4. Sample bus values in sample point of Bit time:
--    a) By normal sampling, at position of sample point (Transciever,reciever)
--    b) In secondary sample point for Transciever of CAN FD Data Phase
--  5. Detect bit Error by comparing transmitted values and Sampled values!
--
--Note: this bit error detection used in the end only for data phase transciever of CAN FD Phase!
--      In other cases bit error is detected inside CAN-Core by comparing transmitted and recieved bit.
-------------------------------------------------------------------------------------------------------------

entity busSync is 
  GENERIC (
      use_Sync:boolean:=false --Whenever Synchronisation chain should be used for sampled data from the bus. 
                             --Turn off only when Synthetizer puts synchronisation chain automatically on the
                             --output pins! Otherwise metastability issues will occur!
    );  
  PORT(
    -------------------------
    --Clock and Async reset--
    -------------------------
    signal clk_sys                  :in   std_logic; --System clock
    signal res_n                    :in   std_logic; --Async Reset
    
    ---------------------------
    --Physical layer interface-
    ---------------------------
    signal CAN_rx                   :in   std_logic; --CAN data input from transciever
    signal CAN_tx                   :out  std_logic; --CAN data output to transciever  
    
    -------------------------------
    --Driving registers interface--
    -------------------------------
    signal drv_bus                  :in   std_logic_vector(1023 downto 0);
      
    -----------------------
    --Prescaler interface--
    ----------------------- 
    signal sample_nbt               :in   std_logic; --Sample command for nominal bit time
    signal sample_dbt               :in   std_logic; --Sample command for data bit tim    
    signal sync_edge                :out  std_logic; --Synchronisation edge appeared
        
    ----------------------
    --CAN Core Interface--
    ----------------------
    signal data_tx                  :in   std_logic; --Transcieve data value
    signal data_rx                  :out  std_logic; --Recieved data value
    
    --Sample point control
    signal sp_control               :in   std_logic_vector(1 downto 0);
    --00:sample_nbt used for sampling (Nominal bit time sampling, Transciever and Reciever)
    --01:sample_dbt used for sampling (Data bit time sampling, only Reciever)
    --10:Sampling with transciever delay compensation (Data bit time, transciever)
        
    --Clear the Shift register at the  beginning of Data Phase!!!          
    signal ssp_reset                :in   std_logic; 
    
    --Calibration command for transciever delay compenstation (counter)
    signal trv_delay_calib          :in   std_logic; 
    
    --Bit Error detection enable (Ex. disabled when recieving data)
    signal bit_err_enable           :in   std_logic; 
    
    --Secondary sample signal outputs
    signal sample_sec_out           :out  std_logic; --Secondary sample signal 
    signal sample_sec_del_1_out     :out  std_logic; --Bit destuffing trigger for secondary sample point
    signal sample_sec_del_2_out     :out  std_logic; --Rec trig for secondary sample point
    
    signal trv_delay_out            :out  std_logic_vector(15 downto 0);
    
    ---------------------------
    --Error Handler Interface--
    ---------------------------
    signal bit_Error                :out  std_logic 
    --Bit Error appeared (monitored value different than transcieved value)
    --Used only for bit error with secondary sample point
        
   );
   -----------------------
   --Driving bus aliases--
   -----------------------
   signal drv_sam                   :     std_logic; --Tripple sampling (as SJA1000)
   signal drv_ena                   :     std_logic; --Enable of the whole driver

   ----------------------------------
   --Internal registers and signals--
   ----------------------------------
   --Synchhronisation chain
   signal sync_Chain_1              :     std_logic; --1. Synchronisation chain register
   signal sync_Chain_2              :     std_logic; --2. Synchronisation chain register
   signal sync_Data                 :     std_logic; --Synchronised data value
   
   --Shift registers length
   constant shift_length            :     natural:=130;
   
   --Bus sampling and edge detection
   signal prev_Sample               :     std_logic; --Previously sampled value on CAN bus
   
   --Majority value from all three sampled values in tripple sampling shift register
   signal trs_majority              :     std_logic; 
   
   --Secondary sampling signal (sampling with transciever delay compensation)
   signal sample_sec                :     std_logic; 
   
   signal sample_sec_del_1          :     std_logic; --Secondary sample signal one clk_sys delayed
   signal sample_sec_del_2          :     std_logic; --Secondary sample signal two clk_sys delayed
   
   --Shift Register for storing the TX data for secondary sample point
   signal ssp_shift                 :     std_logic_vector(shift_length-1 downto 0); 
   
   --Shift Register for generating secondary sampling signal
   signal sample_sec_shift          :     std_logic_vector(shift_length-1 downto 0); 
   
   signal edge_rx_det               :     std_logic; --Register for edge detection
   signal edge_rx_valid             :     std_logic; --Appropriate edge appeared at recieved data
   signal edge_tx_det               :     std_logic; --Edge Appeared at transcieved data
   signal edge_tx_valid             :     std_logic; --Edge appeared at transcieved data
   
   signal trs_reg                   :     std_logic_vector(2 downto 0); --Tripple sampling shift register
   
  --Bit Error register
  signal bit_Error_reg              :     std_logic;
   
  --Note: Bit Error is set up at sample point for whole bit time until next sample point!!!!!
  
  --Transciever delay value (in clk_sys clock periods)
  --Length of this vector corresponds to maximal measurable length of TRD!!!
  --It is 256 clock cycles - assuming 100 Mhz clk sys, more than 2,5 us...
  signal trv_delay                  :     std_logic_vector(6 downto 0); 
  
  --Delay compensation measuring is running (between tx edge and rx edge)
  signal trv_running                :     std_logic;    
  signal trv_to_restart             :     std_logic;
  
end entity;


architecture rtl of busSync is
begin
  --Driving bus alias
  drv_sam               <=  drv_bus(DRV_SAM_INDEX);
  drv_ena               <=  drv_bus(DRV_ENA_INDEX);
  
  --Synchronisation-chain enable
  sync_Data             <=  sync_Chain_2 when use_Sync=true else 
                            CAN_rx;
  
  --Output data propagation
  CAN_tx                <=  data_tx;
  
  trv_delay_out         <=  "000000000"&trv_delay;
  
  --Registers to output propagation
  sample_sec_out        <=  sample_sec;
  sample_sec_del_1_out  <=  sample_sec_del_1;
  
  sample_sec_del_2_out  <=  sample_sec_del_2;
  
  --Bit Error propagation
  bit_Error             <=  bit_Error_reg;
  
  --Tripple sampling majority selection
  trs_majority<='1' when trs_reg="110" else
                '1' when trs_reg="101" else
                '1' when trs_reg="011" else
                '0' when trs_reg="001" else
                '0' when trs_reg="010" else
                '0' when trs_reg="100" else
                '0' when trs_reg="000" else
                '1' when trs_reg="111" else
                '1'; --When unknown rather climb to recessive 
  
  --------------------------------------
  --Synchronisation chain
  --------------------------------------
  sync_chain_proc:process(res_n,clk_sys)
  begin
    if(res_n='0')then
      sync_Chain_1      <=  RECESSIVE;
      sync_Chain_2      <=  RECESSIVE; 
    elsif(rising_edge(clk_sys))then
      sync_Chain_1      <=  CAN_rx;
      sync_Chain_2      <=  sync_Chain_1;
    end if;
  end process sync_chain_proc; 
 
 ---------------------------------------------------------
  --Data Edge detection (for one clk_sys period)
  --Note: Sucessful samplng of previous bit is necessary 
  --      (prev_Sample register)
  --------------------------------------------------------
  edge_proc:process(res_n,clk_sys)
  begin
  if(res_n=ACT_RESET)then
    edge_rx_det         <=  RECESSIVE;
    edge_rx_valid       <=  '0';
    edge_tx_det         <=  RECESSIVE;
    edge_tx_valid       <=  '0';
  elsif rising_edge(clk_sys) then
    
    --Rx data
    edge_rx_det         <=  sync_Data;
    --Rx data edge appeared from recessive to dominant
    if((not edge_rx_det=sync_Data)and 
       (not prev_Sample=sync_Data) and (prev_Sample=RECESSIVE)
      )
    then
      edge_rx_valid     <=  '1';
    else
      edge_rx_valid     <=  '0';
    end if;
    
    --Tx data
    edge_tx_det         <=  data_tx;
    --Tx data edge appeared from RECESSIVE to dominant
    if((not (edge_tx_det=data_tx)) and (edge_tx_det=RECESSIVE) )then 
      edge_tx_valid     <=  '1';
    else
      edge_tx_valid     <=  '0';
    end if;
    
  end if;
  end process edge_proc; 
  
  --Propagating edge value on output
   sync_edge            <=  edge_rx_valid;
   
   
  -----------------------------------------------------
  --Measuring transciever delay compenstaion
  -----------------------------------------------------
  trv_delay_proc:process(res_n,clk_sys)
  begin
    if(res_n='0')then    
      trv_delay         <=  (OTHERS=>'0');
      trv_running       <=  '0';
      trv_to_restart    <=  '0';
    elsif(rising_edge(clk_sys))then
       
       if(trv_delay_calib='1')then --Measuring of delay enabled 
       
        --Starting and stopping the counter
          if(edge_rx_valid='1')then
            trv_running     <=  '0';
          elsif(edge_tx_valid='1' and trv_to_restart='0' )then
            trv_running     <=  '1';
            trv_to_restart  <=  '1';
          end if;
          
        --Counting
          if(edge_tx_valid='1')then
            trv_delay   <=  (OTHERS=>'0');
          elsif(trv_running='1')then
            trv_delay   <=  std_logic_vector(unsigned(trv_delay)+1);
          else 
            trv_delay   <=  trv_delay;
          end if;
       else
         trv_running    <=  trv_running;
         trv_to_restart <=  '0';
       end if;
    end if;
  end process trv_delay_proc;
  
  
  ----------------------------------------------------
  --Generating shifted sampling signal (sample_sec)
  ----------------------------------------------------
  ssp_gen_proc:process(res_n,clk_sys)
  begin
  if(res_n='0')then
    sample_sec          <=  '0';
    sample_sec_shift    <=  (OTHERS=>'0');
    ssp_shift           <=  (OTHERS=>RECESSIVE);
    sample_sec_del_1    <=  '0';
    sample_sec_del_2    <=  '0';
  elsif rising_edge(clk_sys)then  
    if(ssp_reset='1')then
      sample_sec_shift  <=  (OTHERS=>'0');  --Erasing secondary sampling signal shift register
      ssp_shift         <=  (OTHERS=>RECESSIVE); --Erasing Shift register for ssp bit error detection
    else
      sample_sec_shift  <=  sample_sec_shift(shift_length-2 downto 0)&sample_dbt; --Shifting DBT sampling signal
      sample_sec        <=  sample_sec_shift(to_integer(unsigned(trv_delay))); --Shifted signal with trv_delay
      sample_sec_del_1  <=  sample_sec_shift(to_integer(unsigned(trv_delay+1))); 
      sample_sec_del_2  <=  sample_sec_shift(to_integer(unsigned(trv_delay+2))); 
      ssp_shift         <=  ssp_shift(shift_length-2 downto 0)&data_tx; --Storing TX data 
    end if;
  end if;
  end process;
  

------------------------------------------------------------------------
--Separate process for tripple sampling with simple shift register
--Sampling is continous into shift register of lenth 3. If this option
--is desired then majority out of whole shift register is selected
--as sampled value!
------------------------------------------------------------------------
  tripple_sam_proc:process(res_n,clk_sys)
  begin
   if(res_n = ACT_RESET) then
      trs_reg             <=  (OTHERS => RECESSIVE); 
   elsif (rising_edge(clk_sys))then
      --Shift register realisation
      trs_reg(0)          <=  sync_Data;
      trs_reg(2 downto 1) <=  trs_reg(1 downto 0);
   end if;
  end process;  
  

------------------------------------------------------------------------
--Sampling of the bus values and bit Error Detection
------------------------------------------------------------------------
  sample_proc:process(res_n,clk_sys)
  begin
  if(res_n='0')then
    prev_Sample           <=  RECESSIVE;
    bit_Error_reg         <=  '0';
  elsif rising_edge(clk_sys)then
   if(drv_ena=ENABLED)then
    case sp_control is
     when NOMINAL_SAMPLE => --Sampling with nominal bit time (normal CAN, transciever, reciever)
        if(sample_nbt='1')then

          --Tripple sampling option selects the majority from last three sampled values
          if(drv_sam='1')then
            prev_Sample<=trs_majority;           
            if(trs_majority=data_tx) or (bit_err_enable='0') then 
              bit_Error_reg<='0';   --Bit Error detection when sampling
            else
              bit_Error_reg<='1';
            end if; 
          else 
            prev_Sample<=sync_Data;
            if(sync_Data=data_tx) or (bit_err_enable='0') then 
              bit_Error_reg<='0';   --Bit Error detection when sampling
            else
              bit_Error_reg<='1';
            end if; 
          end if;
          
        else 
          prev_Sample<=prev_Sample;
          bit_Error_reg<=bit_Error_reg;
        end if;
     when DATA_SAMPLE => --Sampling with data bit time (CAN FD, reciever)
        if(sample_dbt='1')then  
          prev_Sample<=sync_Data;
          if(sync_Data=data_tx) or (bit_err_enable='0') then 
            bit_Error_reg<='0';   --Bit Error detection when sampling
          else
            bit_Error_reg<='1';
          end if; 
        else
          bit_Error_reg<=bit_Error_reg;
          prev_Sample<=prev_Sample; 
        end if;
     when SECONDARY_SAMPLE => --Sampling with transciever delay compensation (CAN FD, transciever)
        if(sample_sec='1')then
          prev_Sample<=sync_Data;
          
          --Bit Error comparison differs in this case, not actual transmitted bit is compared, but delayed bit is compared (in ssp_shift register)
          if(sync_Data=ssp_shift(to_integer(unsigned(trv_delay))) or (bit_err_enable='0'))then
            bit_Error_reg<='0';
          else
            bit_Error_reg<='1';
          end if;
        else
          bit_Error_reg<=bit_Error_reg;
          prev_Sample<=prev_Sample;
        end if; 
        
     when others => prev_Sample<=prev_Sample;
    end case;
    
   else
     prev_Sample<=RECESSIVE;
     bit_Error_reg<='0';
   end if;
   
  end if;  
  end process sample_proc;
  data_rx<=prev_Sample; --Propagating sampled data to CAN Core  
  
end architecture;