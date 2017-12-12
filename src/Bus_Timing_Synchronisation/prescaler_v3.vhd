Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
USE WORK.CANconstants.ALL;
use work.brs_comp_package.all;

--------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy 
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is 
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in 
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
-- IN THE SOFTWARE.
--
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN 
-- protocol license from Bosch.
--
-- Revision History:
--
--    June 2015   Version 1 of circuit
--    July 2015   Version 2 and 3 of circuit
--    19.12.2015  Added minimal information processing time protection. It is no
--	              longer possible to shorten PH2 segment less than 4 clock cycles.
--                No sampling signals are left out in this case!
--
--    14.6.2016   1.Added reset state into the bit time FSM. As long as reset is
--                  active bt_FSM is kept in reset. First clock cyle it comes out 
--                  reset sync is set. This removes the error that Sync sequence
--                  one bit time after async reset was one clock cycle earlier
--                  then in following bit times. Strictly taken this was bug, 
--                  but it would never appear, since nothing has a chance to 
--                  happend during first bit time after reset. CAN Core takes 10s
--                  of bit times to even get configured, not even talking about 
--                  integration period. This change was made purely to make unit
--                  test happy in every case!
--                2.Since propagation segment is defined as 6 bit wide, changed 
--                  bt_counter and tq_dur range from 31 to 63! This was defacto 
--                  bug discovered by simulation! Due to this if user configures
--                  longer PROP segment  than 31 its actual length would be mo-
--                  dulo 31! Making the counter one bit wider solves the problem!
--    24.6.2016   Bug fix of positive resynchronization. Added detection of 
--                transmitted data to avoid positive resync of a node on iits own
--                tranmitted dominant bit! E.g. ACK by reciever
--    18.7.2016   Bug Fix of transition to h_sync state! Now it is additionally
--                checked that unit is not in h_sync already! Noisy bus with 
--                glitches during h_sync state caused that unit did stay in 
--                h_sync very long...
--    1.8.2016    1.Added "is_tran_trig" signal. IT monitors which trigger was 
--                  generated last. Since hard synchroniyation was changed in 
--                  Protocol control SOF, bug appeared that two consecutive
--                  transcieve triggers appeared in SOF state which caused bit
--                  stuffing and caused errors...
--    31.8.2016   1."tq_edge" signal changed from registered value to combina-
--                  tional as part of bugfixing with reference controller with
--                  ATMEL SAMV71 board!
--                2.Added "sp_control_reg" to cover the change of sample control.
--                  When change is registered no synchronisation occurs but, 
--                  "ph2_real" is set to actual value. This bug was revealed with
--                  reference controller testing.
--    1.9.2016    1. Bug fix of positive resynchronisation in Data bit time. 
--                   Sign was flipped! 
--                2. Added bug-fix of positive resynchronisation. Now Ph1 and 
--                   prop segments are properly differed by "if" branch! Without
--                   this it caused that if synchronisation edge came in first 
--                   time quantum of ph1, resynchronisation still did lengthen
--                   the segment only by 1! Now the segment is lengthened pro-
--                   perly by SJW
--    25.11.2017  1. Driving bus aliases converted to integer format to simplify
--                   the code. Integer signals follow the same naming without 
--                   "drv_" prefix.
--    12.12.2017  Added "brs_comp" compensation for the compensation of phase2
--                During the bit where bit rate switch occured.
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Purpose:
--  v.3 of Prescaler circuit. Due to synchronisation issues with two bit rate
--  counters during switching bit Rates only one counter used. Due to this im-
--  plementation *_nbt, *_dbt signals availiable only when sp_control indicates
--  this bit rate is actual transmit/recieve bit rate! Bit timing is set from 
--  driving bus. Contol of generated bit time is made via sp_control signal. 
--  Synchronisation is enabled and control via sync_control input. The edge used
--  for synchronisation is signalised by active signal sync_edge. Synchronisation
--  type has to be valid on sync_control. When hard synchronisation appears then
--  separate state is entered which handles correct sample and sync signals ge-
--  neration so that no error appears during hard synchronisation!                                                                        
--------------------------------------------------------------------------------

entity prescaler_v3 is
  PORT(
    ------------------------
    --Clock and async reset-
    ------------------------
    signal clk_sys              :in std_logic;  --System clock
    signal res_n                :in std_logic;   --Async reset
    
    -----------------------
    --Bus synch Interface--
    -----------------------
    signal sync_edge            :in std_logic;        --Edge for synchronisation
    signal OP_State             :in oper_mode_type;   --Protocol control state
    
    --Driving Bus
    signal drv_bus              :in std_logic_vector(1023 downto 0); 
    
    -------------------------------------
    --Generated clock - Nominal bit time-
    -------------------------------------
    
    --Time quantum clock - Nominal bit time
    signal clk_tq_nbt           :out std_logic;
    
    --Time quantum - Data bit time
    signal clk_tq_dbt           :out std_logic;
    
    --------------------------------------
    --Sample signals and delayed signals
    --------------------------------------
    signal sample_nbt           :out std_logic; --Nominal Bit time
    signal sample_dbt           :out std_logic; --Data Bit time
    signal sample_nbt_del_1     :out std_logic;
    signal sample_dbt_del_1     :out std_logic;
    signal sample_nbt_del_2     :out std_logic;
    signal sample_dbt_del_2     :out std_logic;
    
    
    --------------------------------------
    --Sync Signals
    --------------------------------------
    signal sync_nbt             :out std_logic;
    signal sync_dbt             :out std_logic;
    signal sync_nbt_del_1       :out std_logic;
    signal sync_dbt_del_1       :out std_logic;
    
    signal bt_FSM_out           :out bit_time_type;
    
    --What is actual node transmitting on the bus
    signal data_tx              :in   std_logic;
    
    --Validated hard synchronisation edge to start Protocol control FSM
    --Note: Sync edge from busSync.vhd cant be used! If it comes during sample 
    --      nbt, sequence it causes errors! It needs to be strictly before or 
    --      strictly after this sequence!!! 
    signal hard_sync_edge_valid :out std_logic; 
    
    -------------------------
    --Clock source control --
    -------------------------
    signal sp_control           :in std_logic_vector(1 downto 0);
    signal sync_control         :in std_logic_vector(1 downto 0)
        
  --Note: sp_control is used for generating sample signals based on which bit 
  --      time is used! Therefore two clock lines are not even needed! Two bit 
  --      time signals are kept  because the first implementation supposed to 
  --      have two independent clocks. However due to errors in switching the 
  --      bit rates (clocks not properly synchronised) the clocks were modified
  --      to be synchronous from one source signal!!!
  );
  
  -----------------------
  --Driving bus aliases--
  -----------------------
  
  --Number of mminimal time quantum (sys clock) in time quantum , Nominal 
  --BitTimeNumber of mminimal time quantum (sys clock) in time quantum ,
  --Nominal BitTime
  signal drv_tq_nbt             :   std_logic_vector (5 downto 0);
  
  --Number of mminimal time quantum (sys clock) in time quantum , Data BitTime
  signal drv_tq_dbt             :   std_logic_vector (5 downto 0);
  
  --Propagation segment length in nominal bit time
  signal drv_prs_nbt            :   std_logic_vector (5 downto 0);
  
  --Phase 1 segment length nominal bit time
  signal drv_ph1_nbt            :   std_logic_vector (5 downto 0);
  
  --Phase 2 segment length nominal bit time
  signal drv_ph2_nbt            :   std_logic_vector (5 downto 0);
  
  --Propagation segment length in nominal bit time
  signal drv_prs_dbt            :   std_logic_vector (3 downto 0);
  
  --Phase 1 segment length nominal bit time
  signal drv_ph1_dbt            :   std_logic_vector (3 downto 0);
  
  --Phase 2 segment length nominal bit time
  signal drv_ph2_dbt            :   std_logic_vector (3 downto 0);
  
  --Synchronisation jump width
  signal drv_sjw_nbt            :   std_logic_vector(3 downto 0);
  
  --Synchronisation jump width
  signal drv_sjw_dbt            :   std_logic_vector(3 downto 0);

  ------------------------------------------------------------------------------
  --Driving bus aliases converted to integer (signed, natural)
  -- Integer aliases used for simplification of the code !!
  ------------------------------------------------------------------------------
  signal tq_nbt                 :   natural range 0 to 63;
  signal tq_dbt                 :   natural range 0 to 63;
  signal prs_nbt                 :   natural range 0 to 63;
  signal prs_dbt                 :   natural range 0 to 15;
  signal ph1_nbt                 :   natural range 0 to 63;
  signal ph1_dbt                 :   natural range 0 to 15;
  signal ph2_nbt                 :   natural range 0 to 63;
  signal ph2_dbt                 :   natural range 0 to 15;
  signal sjw_nbt                 :   natural range 0 to 15;
  signal sjw_dbt                 :   natural range 0 to 15;
  
  ----------------------
  --INTERNAL REGISTERS--
  ----------------------
  signal tq_counter             :   natural; --Counter for time quantum 
  
  --Information that the state is visited for the first time
  signal FSM_Preset             :   std_logic;
  signal FSM_Preset_2           :   std_logic;
  
  --Registered previous vlaue of time quantum clock used for edge detection
  signal prev_tq_val            :   std_logic;
  
  --Edge on time quantum clock (new time quantum clock cycle)
  signal tq_edge                :   std_logic;
  
  --Bit time counter
  signal bt_counter             :   natural range 0 to 63;
  
  --Hard synchronisation appeared
  signal hard_sync_valid        :   std_logic;
  
  --Time quantum clock registred value
  signal clk_tq_nbt_r           :   std_logic;
  signal clk_tq_dbt_r           :   std_logic; 
  
  --Sample signals 
  signal sample_nbt_r           :   std_logic; --Nominal Bit Time
  signal sample_dbt_r           :   std_logic; --Data Bit Time
  signal sample_nbt_del_1_r     :   std_logic;
  signal sample_dbt_del_1_r     :   std_logic;
  signal sample_nbt_del_2_r     :   std_logic;
  signal sample_dbt_del_2_r     :   std_logic;
    
  --Sync Signals
  signal sync_nbt_r             :   std_logic;
  signal sync_dbt_r             :   std_logic;
  signal sync_nbt_del_1_r       :   std_logic;
  signal sync_dbt_del_1_r       :   std_logic;
  
  signal sp_control_reg         :   std_logic_vector(1 downto 0);
  
  ---------------------
  --Internal aliases --
  ---------------------
  
  --Time quantum duration 
  signal tq_dur                 :   natural range 0 to 63;
  
  ------------------
  --Bit time type --
  ------------------
  signal bt_FSM                 :   bit_time_type;
  signal is_tran_trig           :   boolean;
 
	--Duration of ph1 segment after synchronisation
  signal ph1_real               :   integer range -63 to 63;
  
  --Duration of ph2 segment after synchronisation
  signal ph2_real               :   integer range -63 to 63;
  
end entity;


architecture rtl of prescaler_v3 is
begin
  
  --Aliases from DRV_BUS to internal names
  drv_tq_nbt        <=  drv_bus(DRV_TQ_NBT_HIGH downto DRV_TQ_NBT_LOW);
  drv_tq_dbt        <=  drv_bus(DRV_TQ_DBT_HIGH downto DRV_TQ_DBT_LOW);
  drv_prs_nbt       <=  drv_bus(DRV_PRS_NBT_HIGH downto DRV_PRS_NBT_LOW);
  drv_ph1_nbt       <=  drv_bus(DRV_PH1_NBT_HIGH downto DRV_PH1_NBT_LOW);
  drv_ph2_nbt       <=  drv_bus(DRV_PH2_NBT_HIGH downto DRV_PH2_NBT_LOW);
  drv_prs_dbt       <=  drv_bus(DRV_PRS_DBT_HIGH downto DRV_PRS_DBT_LOW);
  drv_ph1_dbt       <=  drv_bus(DRV_PH1_DBT_HIGH downto DRV_PH1_DBT_LOW);
  drv_ph2_dbt       <=  drv_bus(DRV_PH2_DBT_HIGH downto DRV_PH2_DBT_LOW);
  drv_sjw_nbt       <=  drv_bus(DRV_SJW_HIGH downto DRV_SJW_LOW);
  drv_sjw_dbt       <=  drv_bus(DRV_SJW_DBT_HIGH downto DRV_SJW_DBT_LOW);
   
  --Conversion of driving bus aliases to integer (signed) signals
  tq_nbt            <=  to_integer(unsigned(drv_tq_nbt));
  tq_dbt            <=  to_integer(unsigned(drv_tq_dbt));
  prs_nbt           <=  to_integer(unsigned(drv_prs_nbt));
  ph1_nbt           <=  to_integer(unsigned(drv_ph1_nbt));
  ph2_nbt           <=  to_integer(unsigned(drv_ph2_nbt));
  prs_dbt           <=  to_integer(unsigned(drv_prs_dbt));
  ph1_dbt           <=  to_integer(unsigned(drv_ph1_dbt));
  ph2_dbt           <=  to_integer(unsigned(drv_ph2_dbt));
  sjw_nbt           <=  to_integer(unsigned(drv_sjw_nbt));
  sjw_dbt           <=  to_integer(unsigned(drv_sjw_dbt));
 
  --Register to output propagation
  clk_tq_nbt        <=  clk_tq_nbt_r;
  clk_tq_dbt        <=  clk_tq_dbt_r;
  bt_FSM_out        <=  bt_FSM;
  
  sample_nbt        <=  sample_nbt_r;
  sample_dbt        <=  sample_dbt_r;
  sample_nbt_del_1  <=  sample_nbt_del_1_r;
  sample_dbt_del_1  <=  sample_dbt_del_1_r;
  sample_nbt_del_2  <=  sample_nbt_del_2_r;
  sample_dbt_del_2  <=  sample_dbt_del_2_r ;   
 
  sync_nbt          <=  sync_nbt_r;
  sync_dbt          <=  sync_dbt_r;
  sync_nbt_del_1    <=  sync_nbt_del_1_r;
  sync_dbt_del_1    <=  sync_dbt_del_1_r;
  
  hard_sync_edge_valid<=hard_sync_valid;
  
  --Internal aliases
  tq_dur            <= to_integer(unsigned(drv_tq_nbt))
	                    when (sp_control=NOMINAL_SAMPLE) else 
                       to_integer(unsigned(drv_tq_dbt));
  
  --------------------------------
  --Time quantum counter process--
  --------------------------------
  tq_process:process(clk_sys,res_n)
  begin
  if(res_n=ACT_RESET)then
    tq_counter      <=  1;
    clk_tq_nbt_r    <=  '0';
    clk_tq_dbt_r    <=  '0';
    prev_tq_val     <=  '0';
  elsif rising_edge(clk_sys)then
    
    if(sp_control=NOMINAL_SAMPLE) then 
      prev_tq_val   <=  clk_tq_nbt_r; 
    else 
      prev_tq_val   <=  clk_tq_dbt_r; 
    end if;
    
    --Time quantum counter
    if(tq_counter<tq_dur)then
        tq_counter  <=  tq_counter+1;
    else
        tq_counter  <=  1; 
    end if;
    
    --Note:Check if barrel Shifter is used for division by 2, 
    --if no then manually shift the indices
    if(tq_counter<tq_dur/2)then
      
      if(sp_control=NOMINAL_SAMPLE) then 
        clk_tq_nbt_r  <=  '1'; 
      else 
        clk_tq_nbt_r  <=  '0'; 
      end if;
      if((sp_control=DATA_SAMPLE) or (sp_control=SECONDARY_SAMPLE))then 
        clk_tq_dbt_r  <=  '1'; 
      else 
        clk_tq_dbt_r  <=  '0'; 
      end if;
    
    else
      clk_tq_nbt_r    <=  '0';
      clk_tq_dbt_r    <=  '0';
    end if;
    
  end if;
  end process;
  
  --New time quantum period detection
  tq_edge <= '1'  when (sp_control=NOMINAL_SAMPLE) and (drv_tq_nbt="000001") else
             '1'  when (sp_control=DATA_SAMPLE) and (drv_tq_dbt="000001") else
             '1'  when (tq_counter=1) else
             '0';
             
  bt_proc:process(clk_sys,res_n)
  begin
    if(res_n=ACT_RESET)then
      bt_FSM          <=  reset;
      bt_counter      <=  0;
      FSM_Preset      <=  '1';
      FSM_Preset_2    <=  '0';
      sync_dbt_r      <=  '0';
      sync_nbt_r      <=  '0';
      sample_nbt_r    <=  '0';
      sample_dbt_r    <=  '0';
      hard_sync_valid <=  '0';
      is_tran_trig    <=  false;
      sp_control_reg  <=  NO_SYNC;
      ph1_real        <=  0;
      ph2_real        <=  0;
    elsif rising_edge(clk_sys)then
                
      sp_control_reg  <=  sp_control;
      bt_FSM          <=  bt_FSM;
      FSM_Preset      <=  FSM_Preset;
      FSM_Preset_2    <=  FSM_Preset_2;
      bt_counter      <=  bt_counter;
      sample_nbt_r    <=  '0';
      sample_dbt_r    <=  '0';
      sync_nbt_r      <=  '0';
      sync_dbt_r      <=  '0';
      is_tran_trig    <=  is_tran_trig;
      ph2_real        <=  ph2_real;
      ph1_real        <=  ph1_real;
      
      if(bt_FSM=reset)then
        bt_FSM<=sync;
      end if;
      
      if(sp_control /= sp_control_reg)then
          -- Perform Ph2 compensation
          brs_comp(tq_nbt,tq_dbt,sp_control,ph2_nbt,ph2_dbt,ph2_real);
      elsif(sync_edge='1' and 
					 (sync_control=HARD_SYNC) and
					 (bt_FSM /= h_sync))
			then
          bt_FSM        <=  h_sync;
          FSM_Preset    <=  '1';
          --It is assumed that hard sync appears only during Nominal bit time,
          -- according to specification!
          if(tq_nbt=1)then 
            bt_counter  <=  3;
          elsif(tq_nbt=2)then
            bt_counter  <=  2;
          else
            bt_counter  <=  1;
          end if;
      else
        if(sync_edge='1' and (sync_control=RE_SYNC))then
          
          if(bt_FSM=ph2)then --Negative resynchronisation
            
            if(sp_control=NOMINAL_SAMPLE)then
             
              if(bt_counter<(ph2_nbt-sjw_nbt))then
                --Resync bigger than SJW, resync. max SJW
                ph2_real<=ph2_nbt-sjw_nbt;
              else
                --Resync smaller than SJW
                if(bt_counter*tq_nbt<4)then 
                  --We have to check for minimal information processing time 
                  --(4 clock cycles) Thus if we get here we cant quit PH2 imme-
                  --diately otherwise we woud miss some of the sampling signals!
                  --So we shorten PH2 only to its minimal possible length. The 
                  -- length is dependent on time quantum duration
                  if(tq_nbt=1)then --Presc=1
                    --This is only case not according to specification
                    ph2_real<=4;
                  elsif (tq_nbt=2) then --Presc=2
                    ph2_real<=2;
                  elsif (tq_nbt=3) then --Presc 3
                    ph2_real<=2;
                  else
                    ph2_real<=1;
                  end if;
                else
                  --This causes finish of ph2 in next time quantum
                  ph2_real<=bt_counter;
                end if;
              end if;
            
            else
               
              if(bt_counter<(ph2_dbt-sjw_dbt))then
                ph2_real<=ph2_dbt-sjw_dbt;
              else
                 --Resync smaller than SJW
                if(bt_counter*tq_dbt<4)then 
                  --We have to check for minimal information processing time 
                  --(4 clock cycles) Thus if we get here we cant quit PH2 imme-
                  --diately otherwise we woud miss some of the sampling signals!
                  --So we shorten PH2 only to its minimal possible length. The 
                  --length is dependent on time quantum duration
                  if(tq_dbt=1)then --Presc=1
                    --This is only case not according to specification
                    ph2_real<=4;
                  elsif (tq_dbt=2) then --Presc=2
                    ph2_real<=2;
                  elsif (tq_dbt=3) then --Presc 3
                    ph2_real<=2;
                  else
                    ph2_real<=1;
                  end if;
                else
                  --This causes finish of ph2 in next time quantum
                  ph2_real<=bt_counter;
                end if;
              end if;
            
            end if;     
          
          --Positive resynchronisation, transciever in data phase does not per-
          --form positive resynchronisation. Also when dominant bit was just
          --send on the bus, no positive resynchronization is performed
          elsif((data_tx=RECESSIVE)
                 and
                (not(OP_State=transciever and sp_control=SECONDARY_SAMPLE)))
					then
            if(bt_FSM=prop)then
              if(sp_control=NOMINAL_SAMPLE)then
            
                if(bt_counter>sjw_nbt)then
                  ph1_real<=ph1_nbt+sjw_nbt;
                else
                  ph1_real<=ph1_nbt+bt_counter;
                end if;
              
              else
            
                if(bt_counter>sjw_dbt)then
                  ph1_real<=ph1_dbt+sjw_dbt;
                else
                  ph1_real<=ph1_dbt+bt_counter;
                end if;
              
              end if;
            elsif(bt_FSM=ph1)then
              if(sp_control=NOMINAL_SAMPLE)then
            
                if(bt_counter+ph1_nbt>sjw_nbt)then
                  ph1_real<=ph1_nbt+sjw_nbt;
                else
                  ph1_real<=ph1_nbt+bt_counter;
                end if;
              
              else
            
                if(bt_counter+ph1_dbt>sjw_dbt)then
                  ph1_real<=ph1_dbt+sjw_dbt;
                else
                  ph1_real<=ph1_dbt+bt_counter;
                end if;
              
              end if; 
            end if;
            
           end if; 
        end if;
        
        case bt_FSM is 
        when sync =>
            if(FSM_Preset='1')then
             is_tran_trig <= true;
             if(sp_control=NOMINAL_SAMPLE)then 
              sync_nbt_r<='1';
              sync_dbt_r<='0';
             else
              sync_nbt_r<='0';
              sync_dbt_r<='1';
             end if;
             FSM_Preset<='0';
            end if;
            if(tq_edge='1')then
              --Logic for state switching if some of segments have zero length
              if(sp_control=NOMINAL_SAMPLE)then
                if(prs_nbt>0)then
                  bt_FSM<=prop;
                elsif (ph1_nbt>0)then  
                  bt_FSM<=ph1;
                else
                  bt_FSM<=ph2;
                  FSM_Preset<='1';
                end if;
              else
                if(prs_dbt>0)then
                  bt_FSM<=prop;
                elsif (ph1_dbt>0)then  
                  bt_FSM<=ph1;
                else
                  bt_FSM<=ph2;
                  FSM_Preset<='1'; 
                end if;
              end if;
              
              --bt_FSM<=prop; 
              bt_counter<=1;
            end if;
            
             if(sp_control=NOMINAL_SAMPLE)then
               ph2_real<=ph2_nbt;
               ph1_real<=ph1_nbt;
             else
               ph2_real<=ph2_dbt;
               ph1_real<=ph1_dbt;
             end if; 
            
        when prop =>
            if(tq_edge='1')then
             if(sp_control=NOMINAL_SAMPLE)then
                if(bt_counter=prs_nbt or bt_counter>prs_nbt)then
                  if (ph1_nbt>0)then
                    bt_FSM<=ph1;
                  else 
                    bt_FSM<=ph2;
                    FSM_Preset<='1';
                  end if;
                  bt_counter<=1;
                else
                  bt_counter<=bt_counter+1;
                end if;  
             else
               if(bt_counter=prs_dbt or  bt_counter>prs_dbt)then
                  if (ph1_dbt>0)then
                    bt_FSM<=ph1;
                  else 
                    bt_FSM<=ph2;
                    FSM_Preset<='1';
                  end if;
                  bt_counter<=1;
                else
                  bt_counter<=bt_counter+1;
                end if;  
             end if;
            end if;
        when ph1 =>
            if(tq_edge='1')then
              if((bt_counter=ph1_real) or (bt_counter>ph1_real))then
                  bt_FSM<=ph2;
                  bt_counter<=1;
                  FSM_Preset<='1';
              else
                  bt_counter<=bt_counter+1;
              end if;  
              
            end if; 
        when ph2 =>
            if(FSM_Preset='1')then
            is_tran_trig <= false;
            if(sp_control=NOMINAL_SAMPLE)then 
              sample_nbt_r<='1';
              sample_dbt_r<='0';
            else
              sample_nbt_r<='0';
              sample_dbt_r<='1';
            end if;
             FSM_Preset<='0';
            end if;
            
            if(tq_edge='1')then
              --Sample signals already have to be sent! Only then node can
              --resynchronize!!
              if(((bt_counter=ph2_real) or (bt_counter>ph2_real)) and
                 (sample_nbt_r='0' 			 and 
                  sample_dbt_r='0' 			 and 
                  sample_nbt_del_1_r='0' and 
                  sample_dbt_del_1_r='0'))
              then
               --If lower than minimal timing is chosen then it minimal 
               --timing will be applied!!
                  bt_FSM<=sync;
                  bt_counter<=1;
                  FSM_Preset<='1';
              else
                  bt_counter<=bt_counter+1;
              end if;  
            end if;
            
        --State which appears after hard synchronisation to handle proper 
        --bit timing!
        when h_sync=> 
                --It is substitute for sync, prop and ph1 segment after
                -- hard synchronisation appeared!
                if(tq_edge='1')then
                  bt_counter<=bt_counter+1;
                end if;
                
                if((sync_nbt_r='0' and sync_dbt_r='0' and
                  sample_nbt_r='0' and sample_dbt_r='0' and 
                  sample_nbt_del_1_r='0' and sample_dbt_del_1_r='0')
                  and (FSM_Preset='1')
                )then
                 --Hard synchronisation appeared during sync or sample sequence.
                 --It has to be finished first then hard synchronise!
                  hard_sync_valid<='1';
                  FSM_Preset<='0';
                elsif(hard_sync_valid='1' and  FSM_Preset='0')then
                  hard_sync_valid<='0';
                elsif(hard_sync_valid='0' and  
                      FSM_Preset='0'      and
                      FSM_Preset_2='0')
                then 
                --One cycle has to be between sync signal! Otherwise PC control 
                --wont be able to react on hard sync valid!
                  --Here sync signal is finally set!
                  FSM_Preset_2<='1';
                  if(is_tran_trig=false)then
                    if(sp_control=NOMINAL_SAMPLE)then 
                      sync_nbt_r<='1';
                      sync_dbt_r<='0';
                    else
                      sync_nbt_r<='0';
                      sync_dbt_r<='1';
                    end if; 
                  end if;
                end if;
                
                --This condition is to satisfy that correct sync signal 
                --will be generated!
                if((bt_counter>=prs_nbt+ph1_nbt) and 
                    ((sync_nbt_r='0') and (sync_dbt_r='0')) )then
                    bt_FSM<=ph2;
                    bt_counter<=1;
                    FSM_preset<='1';
                    FSM_Preset_2<='0';
                end if;
                
          when others =>
          end case;   
        end if;
    end if;
  end process;


  trig_sign_proc:process(clk_sys,res_n)
  begin
    if(res_n=ACT_RESET)then
      sync_nbt_del_1_r<='0';
      sync_dbt_del_1_r<='0';
      sample_nbt_del_2_r<='0';
      sample_dbt_del_2_r<='0';
      sample_nbt_del_1_r<='0';
      sample_dbt_del_1_r<='0';
    elsif rising_edge(clk_sys)then
      if (sync_nbt_r='1') then sync_nbt_del_1_r<='1';
                         else sync_nbt_del_1_r<='0';
			end if;
      if (sync_dbt_r='1') then sync_dbt_del_1_r<='1';
                         else sync_dbt_del_1_r<='0';
			end if;
      
      if (sample_nbt_r='1') then sample_nbt_del_1_r<='1';
                            else sample_nbt_del_1_r<='0';
			end if;
      if (sample_dbt_r='1') then sample_dbt_del_1_r<='1';
	                        else sample_dbt_del_1_r<='0';
			end if;
    
      if (sample_nbt_del_1_r='1') then sample_nbt_del_2_r<='1';
                                  else sample_nbt_del_2_r<='0';
			end if;
      if (sample_dbt_del_1_r='1') then sample_dbt_del_2_r<='1';
                                  else sample_dbt_del_2_r<='0';
		  end if;
    
    end if;
  end process;

  
end architecture;