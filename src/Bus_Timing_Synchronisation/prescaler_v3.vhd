--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisors and co-authors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--    13.03.2018  Modified bit phases length to have more options in SW settings
--                of bit-timings.
--    1.4.2018    1. Changed trigger signals generation from clocked to combina-
--                   tional. Thisway sample point can be truly generated between
--                   PH1 and PH2.
--                2. Changed Bit-rate shift compensation to two counters solution.
--                   Added "ph2_extra" which calculates value to be set on the
--                   extra counter (by DSP device). Extra counter is set during
--                   transition to PH2, and PH2 finishes when either original
--                   "bt_counter" expires and bit-rate shift did no occur or
--                   when "bt_counter_sw" expires and bit-rate was shifted.
--                   Note that with this solution there is no resynchronisation
--                   occuring during BRS bit!
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE WORK.CANconstants.ALL;

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
  signal drv_tq_nbt             :   std_logic_vector (7 downto 0);
  
  --Number of mminimal time quantum (sys clock) in time quantum , Data BitTime
  signal drv_tq_dbt             :   std_logic_vector (7 downto 0);
  
  --Propagation segment length in nominal bit time
  signal drv_prs_nbt            :   std_logic_vector (6 downto 0);
  
  --Phase 1 segment length nominal bit time
  signal drv_ph1_nbt            :   std_logic_vector (5 downto 0);
  
  --Phase 2 segment length nominal bit time
  signal drv_ph2_nbt            :   std_logic_vector (5 downto 0);
  
  --Propagation segment length in nominal bit time
  signal drv_prs_dbt            :   std_logic_vector (5 downto 0);
  
  --Phase 1 segment length nominal bit time
  signal drv_ph1_dbt            :   std_logic_vector (4 downto 0);
  
  --Phase 2 segment length nominal bit time
  signal drv_ph2_dbt            :   std_logic_vector (4 downto 0);
  
  --Synchronisation jump width
  signal drv_sjw_nbt            :   std_logic_vector(4 downto 0);
  
  --Synchronisation jump width
  signal drv_sjw_dbt            :   std_logic_vector(4 downto 0);

  ------------------------------------------------------------------------------
  --Driving bus aliases converted to integer (signed, natural)
  -- Integer aliases used for simplification of the code !!
  ------------------------------------------------------------------------------
  signal tq_nbt                 :   natural range 0 to 255;
  signal tq_dbt                 :   natural range 0 to 255;
  signal prs_nbt                :   natural range 0 to 127;
  signal prs_dbt                :   natural range 0 to 63;
  signal ph1_nbt                :   natural range 0 to 63;
  signal ph1_dbt                :   natural range 0 to 31;
  signal ph2_nbt                :   natural range 0 to 63;
  signal ph2_dbt                :   natural range 0 to 31;
  signal sjw_nbt                :   natural range 0 to 31;
  signal sjw_dbt                :   natural range 0 to 31;
  
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
  signal bt_counter             :   natural range 0 to 255;
  
  -- Secondary counter for duration of PH2 after bit-rate switch
  -- (Counting in clock cycles, not Time Quantas)
  -- 13 bit counter (8 bit prescaler + 5 bit max length of Ph2 + 1)
  signal bt_counter_sw          :   natural range 0 to 16383;

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
  
  signal sp_control_stored      :   std_logic_vector(1 downto 0);

  -- Auxiliarly signals for internal decoders
  signal switch_ph1_to_ph2      :   boolean;
  signal switch_ph2_to_sync     :   boolean;
  signal prop_non_zero          :   boolean;
  signal ph1_non_zero           :   boolean;
  signal switch_prop_to_ph1     :   boolean;
  
  -- Special triggering signal
  signal sync_trig_spec         :   std_logic;

  -- Auxiliarly signal for detection which counter should be used for
  -- finish of PH2 in case of Bit-rate shift
  signal use_default_ctr        :   boolean;
  signal use_spec_ctr           :   boolean;
 
  ---------------------
  --Internal aliases --
  ---------------------
  
  --Time quantum duration 
  signal tq_dur                 :   natural range 0 to 255;
  
  ------------------
  --Bit time type --
  ------------------
  signal bt_FSM                 :   bit_time_type;
  signal bt_FSM_del             :   bit_time_type;
  signal is_tran_trig           :   boolean;
 
  --Duration of ph1 segment after synchronisation
  signal ph1_real               :   integer range -127 to 127;
  
  --Duration of ph2 segment after synchronisation
  signal ph2_real               :   integer range -127 to 127;

  --Length to be set on secondary counter (post switching)
  signal ph2_extra              :   natural range 0 to 16383;  

end entity;


architecture rtl of prescaler_v3 is
    
    ----------------------------------------------------------------------------
    -- Generates either receive (sample) or transceive (sync) triggering signal
    -- based on sample type.
    ----------------------------------------------------------------------------
    procedure generate_trig(
        signal sample_type          : in std_logic_vector(1 downto 0);
        signal nbt_trig             : out std_logic;
        signal dbt_trig             : out std_logic
    ) is
    begin
        if (sample_type = NOMINAL_SAMPLE) then
            nbt_trig    <= '1';
            dbt_trig    <= '0';
        
        -- DATA_SAMPLE as well as SAMPLE_SEC covered
        else
            nbt_trig    <= '0';
            dbt_trig    <= '1';
        end if;  
    end procedure;

    ----------------------------------------------------------------------------
    -- Presetting secondary counter for case of bit-rate shift
    ----------------------------------------------------------------------------
    procedure set_secondary_ctr(
        signal ctr              : out natural range 0 to 16383;
        signal ctr_pres         : in  natural range 0 to 16383;
        signal sp_control       : out std_logic_vector(1 downto 0);
        signal sp_control_pres  : in  std_logic_vector(1 downto 0)
    ) is
    begin
        ctr         <= ctr_pres;
        sp_control  <= sp_control_pres;
    end procedure;

    ----------------------------------------------------------------------------
    -- Negative resynchronisation
    -- TODO: Check if here we could not shorten IPT to 3!!!
    ----------------------------------------------------------------------------
    procedure negative_resync(
        signal bt_counter       : in    natural range 0 to 255;
        signal sp_control       : in    std_logic_vector(1 downto 0);
        signal ph2_nbt          : in    natural range 0 to 63;
        signal ph2_dbt          : in    natural range 0 to 31;
        signal sjw_nbt          : in    natural range 0 to 31;
        signal sjw_dbt          : in    natural range 0 to 31;
        signal tq_nbt           : in    natural range 0 to 255;
        signal tq_dbt           : in    natural range 0 to 255;
        signal ph2_real         : out   integer range -127 to 127
    ) is
        variable ph2            :       natural range 0 to 63;
        variable sjw            :       natural range 0 to 31;
        variable tq             :       integer range -127 to 127;
        variable ph2_min_sjw    :       natural range 0 to 63;
        variable ipt_ok         :       boolean;
    begin
        if (sp_control = NOMINAL_SAMPLE) then
            ph2             := ph2_nbt;
            sjw             := sjw_nbt;
            tq              := tq_nbt;
        else
            ph2             := ph2_dbt;
            sjw             := sjw_dbt;
            tq              := tq_dbt;
        end if;
        ph2_min_sjw         := ph2 - sjw;

        -- First check If information processing time is not corrupted
        if ((tq = 1 and bt_counter < 4) or
            ((tq = 2 or tq = 3) and bt_counter < 2))
        then
            ipt_ok  := false;
        else
            ipt_ok  := true;
        end if;

        -- The edge did not come during the Information processing time
        if (ipt_ok) then

            -- Resynchronisation is bigger than synchronisation jump width,
            -- resynchronize only up to maximum of SJW.
            if (bt_counter < ph2_min_sjw) then
                ph2_real    <= ph2_min_sjw;

            -- Resynchronisation is smaller than synchronisation jump width,
            -- finish the Bit in the next Time quantum!
            else
                ph2_real    <= bt_counter;
            end if;
        
        -- The edge did come during Information processing time, should 
        -- shorten min to 4 clock cycles, but not more than SJW!
        else
            -- SJW setting does corrupt IPT, does not shorten to less than IPT!
            if (ph2_min_sjw < 4) then
                if (tq = 1) then
                    ph2_real    <= 4;
                else
                    ph2_real    <= 2;
                end if;
            
            -- SJW does not corrupt IPT, shorten max to SJW!
            else
                ph2_real        <= ph2_min_sjw;
            end if;
        end if;
    end procedure;

    ----------------------------------------------------------------------------
    -- Positive resynchronisation
    ----------------------------------------------------------------------------
    procedure positive_resync(
        signal bt_FSM           : in    bit_time_type;
        signal bt_counter       : in    natural range 0 to 255;
        signal sp_control       : in    std_logic_vector(1 downto 0);
        signal prop_nbt         : in    natural range 0 to 127;
        signal prop_dbt         : in    natural range 0 to 63;
        signal ph1_nbt          : in    natural range 0 to 63;
        signal ph1_dbt          : in    natural range 0 to 31;
        signal sjw_nbt          : in    natural range 0 to 31;
        signal sjw_dbt          : in    natural range 0 to 31;
        signal tq_nbt           : in    natural range 0 to 255;
        signal tq_dbt           : in    natural range 0 to 255;
        signal ph1_real         : out   integer range -127 to 127
    ) is
        variable ph1_v          :       natural range 0 to 63;
        variable sjw            :       natural range 0 to 31;
        variable tq             :       integer range -127 to 127;
        variable prop_v         :       natural range 0 to 127;
    begin
        if (sp_control = NOMINAL_SAMPLE) then
            sjw           := sjw_nbt;
            ph1_v         := ph1_nbt;
            prop_v        := prop_nbt;
        else
            sjw           := sjw_dbt;
            ph1_v         := ph1_dbt;
            prop_v        := prop_dbt;
        end if;

        -- Resync edge came during PROP segment.
        if (bt_FSM = prop) then

            -- Resynchronisation bigger than SJW, allow max. SJW.
            if (bt_counter > sjw) then
                ph1_real    <= ph1_v + sjw;
            else
                ph1_real    <= ph1_v + bt_counter;
            end if;
        
        -- Resync edge came during PH1 segment (still can, if PROP is very short
        -- or resync is just very long...)
        elsif (bt_FSM = ph1) then

            -- Resync is longer than Synchronisation jump width, count with
            -- the propagation segment which already elapsed!
            if ((bt_counter + prop_v) > sjw) then
                ph1_real    <= ph1_v + sjw;
            else
                ph1_real    <= ph1_v + bt_counter + prop_v;
            end if;
        end if;
    end procedure;

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


  ------------------------------------------------------------------------------
  -- Calculating length to be preset to the extra PH2 counter
  -- This can be clocked, since before sample point there is enough cycles
  -- to have the value stable.
  -- Multiplier is assumed to be automatically inferred to DSP, MAC device!
  -- This should consume two DSP devices, and it should fit within both Altera
  -- and Xilinx DSP, MAC units!
  ------------------------------------------------------------------------------
  extra_ctr_calc_proc : process(res_n, clk_sys)
  begin
    if (res_n = ACT_RESET) then
        ph2_extra   <= 0;
    elsif (rising_edge(clk_sys)) then
        
        -- Value into extra counter will be always taken during PH1 to PH2
        -- switch. We assume that bit-rate switch will occur, and use the
        -- oposite bit-rate as during PH1!
        if (sp_control = NOMINAL_SAMPLE) then
            ph2_extra   <= tq_dbt * ph2_dbt;
        else
            ph2_extra   <= tq_nbt * ph2_nbt;
        end if;

    end if;
  end process;

  ------------------------------------------------------------------------------
  -- Deciding which counter to use during PH2 to SYNC switch in case of
  -- bit-rate shift
  ------------------------------------------------------------------------------
  use_default_ctr   <= true when ((sp_control = NOMINAL_SAMPLE) and
                                  (sp_control_stored = NOMINAL_SAMPLE))
                                 or
                                  ((sp_control = DATA_SAMPLE or
                                    sp_control = SECONDARY_SAMPLE)
                                   and
                                   (sp_control_stored = DATA_SAMPLE or
                                    sp_control_stored = SECONDARY_SAMPLE))
                            else
                      false;

  use_spec_ctr      <= not use_default_ctr;

  
  ------------------------------------------------------------------------------
  -- New time quantum period detection
  ------------------------------------------------------------------------------
  tq_edge <= '1'  when (sp_control=NOMINAL_SAMPLE) and (drv_tq_nbt="00000001") else
             '1'  when (sp_control=DATA_SAMPLE) and (drv_tq_dbt="00000001") else
             '1'  when (tq_counter=1) else
             '0';


    ----------------------------------------------------------------------------
    -- Combinational decoder for Bit time state changes
    ----------------------------------------------------------------------------
    switch_ph1_to_ph2 <= true when ((bt_counter = ph1_real) or
                                    (bt_counter > ph1_real))
                              else
                         false;

    -- Note that we should not have to wait for finish of the sampling signals
    -- since, negative resynchronisation is not allowed to shorten ph2 to less
    -- than Information processing time!
    switch_ph2_to_sync <= true when ((bt_counter = ph2_real) or
                                     (bt_counter > ph2_real))
                               else
                          false;
    
    prop_non_zero      <= true when ((sp_control = NOMINAL_SAMPLE) and
                                      (prs_nbt > 0))
                                     or
                                     ((sp_control = DATA_SAMPLE or
                                       sp_control = SECONDARY_SAMPLE) and
                                      (prs_dbt > 0))
                                else
                          false;

    ph1_non_zero       <= true when ((sp_control = NOMINAL_SAMPLE) and
                                      (ph1_nbt > 0))
                                     or
                                     ((sp_control = DATA_SAMPLE or
                                       sp_control = SECONDARY_SAMPLE) and
                                      (ph1_dbt > 0))
                                else
                          false;
    
    -- Note that bt_counter can be only compared on equality since length
    -- of propagation segment is not affected by resynchronisation.
    switch_prop_to_ph1   <= true when ((sp_control = NOMINAL_SAMPLE) and
                                       (bt_counter = prs_nbt))
                                      or
                                      ((sp_control = DATA_SAMPLE or
                                        sp_control = SECONDARY_SAMPLE) and
                                       (bt_counter = prs_dbt))
                                 else
                            false;


    ----------------------------------------------------------------------------
    -- Triggering signals decoders
    -- Decoded commbinationally to truly sample between PH1 and PH2!
    ----------------------------------------------------------------------------
    sync_nbt_r        <= '1' when (((sp_control  = NOMINAL_SAMPLE) and
                                    (bt_FSM_del /= sync) and
                                    (bt_FSM      = sync)) or
                                   (sync_trig_spec = '1'))
                             else
                         '0';

    sync_dbt_r        <= '1' when ((sp_control  = DATA_SAMPLE or
                                    sp_control  = SECONDARY_SAMPLE) and
                                   (bt_FSM_del /= sync) and
                                   (bt_FSM      = sync))
                             else
                         '0';

    -- Comparison of bt_FSM_del is to avoid glitches between PROP and PH1
    sample_nbt_r      <= '1' when ((sp_control = NOMINAL_SAMPLE) and
                                   (bt_FSM = ph1) and
                                   (switch_ph1_to_ph2) and
                                   (tq_edge = '1'))
                             else
                         '0';

    sample_dbt_r      <= '1' when ((sp_control = DATA_SAMPLE or
                                    sp_control = SECONDARY_SAMPLE) and
                                   (bt_FSM = ph1) and
                                   (switch_ph1_to_ph2) and
                                   (tq_edge = '1'))
                             else
                         '0';                            

   
  bt_proc : process(clk_sys, res_n)
  begin
	if (res_n = ACT_RESET) then
      bt_FSM            <=  reset;
      bt_counter        <=  0;
      FSM_Preset        <=  '1';
      FSM_Preset_2      <=  '0';
      hard_sync_valid   <=  '0';
      is_tran_trig      <=  false;
      sp_control_stored <=  NOMINAL_SAMPLE;
      ph1_real          <=  0;
      ph2_real          <=  0;
      sync_trig_spec    <= '0';
      bt_counter_sw     <=  1;

    elsif rising_edge(clk_sys) then
                
      bt_FSM            <=  bt_FSM;
      FSM_Preset        <=  FSM_Preset;
      FSM_Preset_2      <=  FSM_Preset_2;
      bt_counter        <=  bt_counter;
      is_tran_trig      <=  is_tran_trig;
      ph2_real          <=  ph2_real;
      ph1_real          <=  ph1_real;
      sync_trig_spec    <=  '0';
 
      sp_control_stored <=  sp_control_stored;
      bt_counter_sw     <=  bt_counter_sw;

      if (bt_FSM = reset) then
        bt_FSM        <=  sync;
      end if;
      
      if (sync_edge = '1' and 
         (sync_control = HARD_SYNC) and
         (bt_FSM /= h_sync))
      then
          bt_FSM        <=  h_sync;
          FSM_Preset    <=  '1';
          -- It is assumed that hard sync appears only during Nominal bit time,
          -- according to specification!
          if (tq_nbt = 1) then 
            bt_counter  <=  3;
          elsif (tq_nbt = 2) then
            bt_counter  <=  2;
          else
            bt_counter  <=  1;
          end if;
      else
        if (sync_edge = '1' and (sync_control = RE_SYNC)) then
          
            -- Negative resynchronisation
            if (bt_FSM = ph2) then
                negative_resync(bt_counter, sp_control, ph2_nbt, ph2_dbt,
                                sjw_nbt, sjw_dbt, tq_nbt, tq_dbt, ph2_real);

            -- Positive resynchronisation
            -- Transciever in data phase does not perform positive resynchro-
            -- nisation. Also when dominant bit was just send on the bus, no 
            -- positive resynchronization is performed.
            elsif ((data_tx = RECESSIVE) and
               (not (OP_State = transciever and sp_control = SECONDARY_SAMPLE)))
            then
                positive_resync(bt_FSM, bt_counter, sp_control, prs_nbt, prs_dbt,
                                ph1_nbt, ph1_dbt, sjw_nbt, sjw_dbt,
                                tq_nbt, tq_dbt, ph1_real);
            end if; 
        end if;
        
        case bt_FSM is

        ------------------------------------------------------------------------
        -- Synchronisation segment
        ------------------------------------------------------------------------
        when sync =>
            if (FSM_Preset = '1') then
                is_tran_trig   <= true;
                FSM_Preset     <= '0';
            end if;

            if (tq_edge = '1') then
                if (prop_non_zero) then
                    bt_FSM      <= prop;
                elsif (ph1_non_zero) then
                    bt_FSM      <= ph1;
                else 
                    bt_FSM      <= ph2;
                    FSM_Preset  <= '1';
                    set_secondary_ctr(bt_counter_sw, ph2_extra,
                                      sp_control_stored, sp_control);
                end if;

                bt_counter      <= 1;
            end if;
            
            -- Set the "default" bit time duration. Later in the bit time
            -- resynchronisation can change it!
            if (sp_control = NOMINAL_SAMPLE) then
               ph2_real         <= ph2_nbt;
               ph1_real         <= ph1_nbt;
            else
               ph2_real         <= ph2_dbt;
               ph1_real         <= ph1_dbt;
            end if; 
        
        ------------------------------------------------------------------------
        -- Propagation segment
        ------------------------------------------------------------------------ 
        when prop =>
            if (tq_edge = '1') then
                if (switch_prop_to_ph1) then
                    if (ph1_non_zero) then
                        bt_FSM <= ph1;
                    else
                        bt_FSM <= ph2;
                        set_secondary_ctr(bt_counter_sw, ph2_extra,
                                        sp_control_stored, sp_control);
                    end if;
                    FSM_Preset <= '1';
                    bt_counter <= 1;
                else
                    bt_counter <= bt_counter + 1;
                end if;
            end if;

        ------------------------------------------------------------------------
        -- Phase 1 segment
        ------------------------------------------------------------------------
        when ph1 =>
            if (tq_edge = '1') then
              if (switch_ph1_to_ph2) then
                  bt_FSM        <= ph2;
                  bt_counter    <= 1;
                  FSM_Preset    <= '1';
                  set_secondary_ctr(bt_counter_sw, ph2_extra,
                                      sp_control_stored, sp_control);
              else
                  bt_counter    <= bt_counter + 1;
              end if;
            end if;

        ------------------------------------------------------------------------
        -- Phase 2 segment
        ------------------------------------------------------------------------
        when ph2 =>
            if (FSM_Preset = '1') then
                is_tran_trig <= false;
                FSM_Preset   <= '0';
            end if;
            
            if (tq_edge = '1') then
              if (switch_ph2_to_sync) then
                if (use_default_ctr) then
                  bt_FSM        <= sync;
                  bt_counter    <= 1;
                  FSM_Preset    <= '1';
                end if;
              else
                  bt_counter    <= bt_counter + 1;
              end if;
            end if;

            -- Counting with 1 each clock cycle! No resynchronisation on this
            -- counter, no need to check on underflow. We can count to exact
            -- value (0)!
            if (bt_counter_sw = 0 or bt_counter_sw = 1) then
                if (use_spec_ctr) then
                    bt_FSM      <= sync;
                    bt_counter  <= 1;
                    FSM_Preset  <= '1';
                end if;
            else
                bt_counter_sw   <= bt_counter_sw - 1; 
            end if;

        ------------------------------------------------------------------------
        -- Special state which appears after hard synchronisation to handle 
        -- proper bit timing! It is a substitute for sync, prop and ph1 segment
        -- after hard synchronisation appeared!
        ------------------------------------------------------------------------
        when h_sync => 
            if (tq_edge = '1') then
                bt_counter <= bt_counter + 1;
            end if;
                
            if ((sync_nbt_r = '0') and
                (sync_dbt_r = '0') and
                (sample_nbt_r = '0') and
                (sample_dbt_r = '0') and 
                (sample_nbt_del_1_r = '0') and
                (sample_dbt_del_1_r = '0') and
                (FSM_Preset = '1'))
            then
                --Hard synchronisation appeared during sync or sample sequence.
                --It has to be finished first then hard synchronise!
                hard_sync_valid     <= '1';
                FSM_Preset          <= '0';

            elsif (hard_sync_valid = '1' and  FSM_Preset = '0') then
                hard_sync_valid     <= '0';

            elsif (hard_sync_valid = '0' and  
                   FSM_Preset = '0'      and
                   FSM_Preset_2 = '0')
            then 
                --One cycle has to be between sync signal! Otherwise PC control 
                --wont be able to react on hard sync valid!
                --Here sync signal is finally set!
                FSM_Preset_2        <= '1';
                if (is_tran_trig = false) then
                    sync_trig_spec  <= '1';
                end if;
            end if;
                
                --This condition is to satisfy that correct sync signal 
                --will be generated!
                if ((bt_counter >= prs_nbt + ph1_nbt) and 
                    ((sync_nbt_r = '0') and (sync_dbt_r = '0')))
                then
                    bt_FSM          <= ph2;
                    bt_counter      <= 1;
                    FSM_preset      <= '1';
                    FSM_Preset_2    <= '0';
                end if;    
          when others =>
          end case;   
        end if;
    end if;
  end process;


    ----------------------------------------------------------------------------
    -- Creating delayed sample and synchronisation signals by shift registers
    ----------------------------------------------------------------------------
    trig_sign_proc : process(clk_sys, res_n)
    begin
        if (res_n = ACT_RESET) then
            sync_nbt_del_1_r    <= '0';
            sync_dbt_del_1_r    <= '0';
            sample_nbt_del_2_r  <= '0';
            sample_dbt_del_2_r  <= '0';
            sample_nbt_del_1_r  <= '0';
            sample_dbt_del_1_r  <= '0';
            
            bt_FSM_del          <= sync;
        elsif rising_edge(clk_sys) then

            sync_nbt_del_1_r <= sync_nbt_r;
            sync_dbt_del_1_r <= sync_dbt_r;

            sample_nbt_del_1_r <= sample_nbt_r;
            sample_dbt_del_1_r <= sample_dbt_r;

            sample_nbt_del_2_r <= sample_nbt_del_1_r;
            sample_dbt_del_2_r <= sample_dbt_del_1_r;
    
            bt_FSM_del         <= bt_FSM;
        end if;
    end process;

  
end architecture;
