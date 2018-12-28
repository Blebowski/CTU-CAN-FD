--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
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
-- Purpose:
--  Circuit for selecting the valid frame for CAN Core from generic number of 
--  TXT buffer inputs. Compares priorities of each buffer (SW selected) and
--  picks the highest priority buffer whose input is valid. Timestamp of high-
--  est priority frame is selected and compared with external timestamp. The
--  frame is marked as valid for CAN Core only if this timestamp is lower than
--  value of external Timestamp. This realizes the functionality of transmission
--  at exact time!                                                                                                                                                
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--    17.1.2016   Added ID change from register value to decimal value for case 
--                when identifier needs to decide about priority message (Time 
--                stamps are equal)
--    7.6.2016    Added "less_than" function for comparison of two 64 bit 
--                std_logic_vectors. Integer does not support more than 64 bits.
--                When timestamp higher than 32 bits was simulated in unit test
--                simulator was throwing out milions of warnings!
--    23.6.2016   Added less or equal to the case when both timestamps and both 
--                identifiers are equal. Thisway identifier from Buffer 1 instead
--                of Buffer 2 is propagated!
--    4.12.2017   Added support for split "Data" and "Metadata" into TXT Buffer.
--                Added state machine "tx_arb_fsm". The state machine waits for 
--                CAN Core to finish the transmission before signalling the TXT 
--                Buffer to erase. Output data word is selected based on stored 
--                value of "mess_src" from the time of decision between TXT1 and
--                TXT2 buffer.
--    10.12.2017  Added "tx_time_sup" to enable/disable transmission at given
--                time and save some LUTs.
--    27.12.2017  Added "tran_lock", "tran_unlock", "tran_drop" signals for
--                implementation of frame swapping feature. Replaced 
--                "tran_data_ack" with "tran_lock" signal.
--     14.2.2018  VALENTINE day with work on CAN Core! The best date ever ;)
--                (Just the right moment for the stupid NOOOOT joke that
--                 Americans always do...)
--                Reimplemented the TX Arbitrator to support following shit:
--                1. TXT Buffer priorities combinationally via "priorityDecoder"
--                2. Generic amount of TXT Buffers is now supported.
--                3. Content of buffer is indicated as valid to CAN Core only
--                   if it is highest priority buffer with ready signal active,
--                   and its timestamp is lower than external Timestamp! Thus 
--                   it can happend that lower priority buffer will actually
--                   contain lower timestamp. Then it is responsibility of SW
--                   to put the frame which should be transmitted as first into
--                   the buffer with lower priority!
--                4. "tran_lock", "tran_unlock" and "tran_drop" signals removed
--                   and replaced with structure "txt_hw_cmd" where these signals
--                   are elements.
--    24.3.2018   Serialized loading of metadata from TXT Buffer. State machine
--                is periodically loading metadata and comparing timestamps.
--                At the end of the load, data are committed on the output of
--                TX Arbitrator. This allows single input from TXT Buffer and
--                synthesis of whole TXT Buffer to RAM memory.
--     6.4.2018   1. Removed loading of identifier by TX arbitrator. Only
--                   metadata are loaded by identifier. Since only one word
--                   is updated for CAN Core, there is no need for internal
--                   registers! Identifier is now addressed by CAN Core in the
--                   same way as CAN data.
--                2. Changed pointer to TXT Buffer memory to combinational,
--                   since TXT Buffer must have clocked reading to achieve RAM
--                   inferrence! Every condition which causes transfer to a
--                   state where Arbitrator reads from TXT Buffer, must set
--                   the address from which the state will be reading. This-
--                   way on transfer address is available and TXT Buffer can
--                   provide the data!
--    26.4.2018   1. Pointer to TXT Buffer in last cycle of "arb_locked" is taken
--                   from FSM not from CAN Core. In the next clock cycle FSM is
--                   already taking the data as if coming from Lower timestamp
--                   address! Without it, Frame format word was taken as timestamp
--                   word and timestamp comparison was executed on it! This could
--                   have lead to validating frame for transmission in wrong
--                   moment!
--                2. "tran_frame_valid" set to be in logic 1 during the whole
--                   "arb_lock" state. Protocol control reacts on active "tran_
--                   frame_valid" and locks the buffer. Metadata on outputs 
--                   are used by Protocol control during transmission. It is
--                   better design approach, since during whole TX, the frame
--                   on the output is not updated, "tran_frame_valid" is also
--                   not updated and should stay the same as at the moment of
--                   locking.
--                3. Added waiting in "arb_upp_ts" for timestamp to elapse.
--                   Metadata pointer must have been changed, since if the
--                   FSM stays in "arb_upp_ts" it needs to have Upper Timestamp
--                   on the output in the next clock cycle, not Frame format
--                   word! Note that this change adds one part to metadata poin-
--                   ter multiplexor, but it is now able to react on timestamp
--                   elapse with no jitter! Before it could have happend that
--                   timestamp was elapse during reading lower timestamp word
--                   but circuit reacted only one clock cycle later!
--     2.6.2018   Removed "tx_time_suport".
--   12.11.2018   Decoupled TX Arbitrator FSM to Sub-module. Separated registers
--                to separate processes.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;

entity tx_arbitrator is
  generic(
    constant buf_count   : natural range 1 to 8
  );
  port( 
    ------------------------
    -- Clock and reset    
    ------------------------
    signal clk_sys                :in  std_logic;
    signal res_n                  :in  std_logic;
    
    ------------------------
    -- TX Buffers interface
    ------------------------
   
    -- Data words coming from TXT buffers
    signal txt_buf_in             :in txtb_output_type;
    
    -- Signal that buffer is in "Ready state", it can be selected
    -- by arbitrator
    signal txt_buf_ready          :in std_logic_vector(buf_count - 1 downto 0);
    
    -- Pointer to TXT Buffer
    signal txtb_ptr               :out natural range 0 to 19;
    
    -----------------------
    -- CAN Core Interface
    -----------------------
    
    -- TX Message data
    signal tran_data_word_out     :out std_logic_vector(31 downto 0);
    
    --TX Data length code
    signal tran_dlc_out           :out std_logic_vector(3 downto 0);
    
    --TX is remote frame
    signal tran_is_rtr            :out std_logic;
    
    --TX Identifier type (0-Basic,1-Extended);
    signal tran_ident_type_out    :out std_logic;
    
    --TX Frame type
    signal tran_frame_type_out    :out std_logic;
    
    --Bit rate shift for CAN FD frames 
    signal tran_brs_out           :out std_logic;
    
    -- Signal for CAN Core that frame on the output is valid and can be stored 
    -- for transmitting
    signal tran_frame_valid_out   :out std_logic;
    
    -- Commands from the CAN Core for manipulation of the CAN 
    signal txt_hw_cmd             :in txt_hw_cmd_type;  
    
    -- If error occurs during the transmission, and CAN Core picks
    -- frame again, CAN Core needs to know that different buffer is now
    -- selected, so that it can erase the retransmitt counter (in case
    -- retransmitt limit is enabled).
    signal txtb_changed           :out std_logic;
    
    -- Index of the TXT Buffer for which the actual HW command is valid
    signal txt_hw_cmd_buf_index   :out natural range 0 to buf_count - 1;
    
    -- Pointer to TXT Buffer provided from CAN Core
    signal txtb_core_pointer      :in natural range 0 to 19;
      
    ---------------------
    -- Driving interface
    ---------------------
    
    --Driving bus from registers
    signal drv_bus                :in std_logic_vector(1023 downto 0);

    --Priorities from the registers
    signal txt_buf_prio           :in txtb_priorities_type;
    
    --TimeStamp value
    signal timestamp              :in std_logic_vector(63 downto 0)
        
  );
  
  -----------------------------------------------------------------------------
  -- Internal signals
  -----------------------------------------------------------------------------
  
  -- Indicates the highest selected buffer and its validity from
  -- combinational priority decoder
  signal select_buf_avail           : std_logic;
  signal select_buf_index           : natural range 0 to buf_count - 1;
   
  -- Input word from TXT Buffer !!!
  signal txtb_selected_input        : std_logic_vector(31 downto 0);
  
  -- TXT Buffer timestamp joined combinationally. Given by ts_low_internal and
  -- upper timestamp word, out the output of RAM
  signal txtb_timestamp             : std_logic_vector(63 downto 0);
  
  -- Comparison of loaded timestamp from TXT Buffer.
  signal timestamp_valid            : std_logic;

  -- The output of priority decoder (selected TXT Buffer) has changed, pulse
  -- for one clock cycle
  signal select_index_changed       : std_logic;


  -----------------------------------------------------------------------------
  -- Internal registers
  -----------------------------------------------------------------------------
  
  -- Registered values for detection of change
  signal select_buf_index_reg       : natural range 0 to buf_count - 1;
  
  -- Lower timestamp loaded from TXT Buffer
  signal ts_low_internal            : std_logic_vector(31 downto 0);
  
  -- Internal index of TXT Buffer stored at the time of buffer selection
  signal int_txtb_index             : natural range 0 to buf_count - 1;
  
  -- TXT Buffer internal index of last buffer that was locked
  -- From buffer change, Protocol control can erase retransmitt counter
  signal last_txtb_index            : natural range 0 to buf_count - 1;
  
  -- Pointer to TXT Buffer for loading CAN frame metadata and
  -- timstamp during the selection of TXT Buffer.
  signal txtb_pointer_meta          : natural range 0 to 19;

  -- Comitted values of internal signals
  signal tran_dlc_com               : std_logic_vector(3 downto 0);
  signal tran_is_rtr_com            : std_logic;
  signal tran_ident_type_com        : std_logic;
  signal tran_frame_type_com        : std_logic;
  signal tran_brs_com               : std_logic;
  signal tran_frame_valid_com       : std_logic;


  -----------------------------------------------------------------------------
  -- TX Arbitrator FSM outputs
  -----------------------------------------------------------------------------
  
  -- Load Timestamp lower word to metadata pointer
  signal load_ts_lw_addr            : std_logic;
 
  -- Load Timestamp upper word to metadata pointer
  signal load_ts_uw_addr            : std_logic;

  -- Load Frame format word to metadata pointer
  signal load_ffmt_w_addr           : std_logic;

  -- Store timestamp lower word
  signal store_ts_l_w               : std_logic;

  -- Store metadata (Frame format word) on the output of TX Arbitrator
  signal store_md_w                 : std_logic;

  -- Store last locked TXT Buffer index
  signal store_last_txtb_index      : std_logic;

  -- Set valid selected buffer on TX Arbitrator output.
  signal frame_valid_com_set        : std_logic;

  -- Clear valid selected buffer on TX Arbitrator output.
  signal frame_valid_com_clear      : std_logic;
 
  -- TX Arbitrator is locked
  signal tx_arb_locked              : std_logic;


  -----------------------------------------------------------------------------
  -- Comparing procedure for two 64 bit std logic vectors
  -----------------------------------------------------------------------------
  function less_than(
      signal   a       : in std_logic_vector(63 downto 0);
      signal   b       : in std_logic_vector(63 downto 0)
  )return std_logic is
  begin
      if (unsigned(a(63 downto 32)) < unsigned(b(63 downto 32))) or 
          ((a(63 downto 32) = b(63 downto 32)) and 
          (unsigned(a(31 downto 0)) < unsigned(b(31 downto 0))))then
          return '1';
      else
         return '0';
      end if;
  end function;
  
end entity;

architecture rtl of tx_arbitrator is
begin
  
  ------------------------------------------------------------------------------
  -- Priority decoder on TXT Buffers
  ------------------------------------------------------------------------------
  priority_decoder_comp : priority_decoder 
  generic map(
    buf_count       => buf_count
  )
  port map( 
     prio           => txt_buf_prio,
     prio_valid     => txt_buf_ready,
     output_valid   => select_buf_avail,
     output_index   => select_buf_index
  );
  

  ------------------------------------------------------------------------------
  -- TX Arbitrator FSM
  ------------------------------------------------------------------------------
  tx_arbitrator_fsm_comp : tx_arbitrator_fsm
  port map(

    -- Inputs
    clk_sys                => clk_sys,
    res_n                  => res_n,
    select_buf_avail       => select_buf_avail,
    select_index_changed   => select_index_changed,
    timestamp_valid        => timestamp_valid,
    txt_hw_cmd             => txt_hw_cmd,

    -- Outputs
    load_ts_lw_addr        => load_ts_lw_addr,
    load_ts_uw_addr        => load_ts_uw_addr,
    load_ffmt_w_addr       => load_ffmt_w_addr,
    store_ts_l_w           => store_ts_l_w,
    store_md_w             => store_md_w,
    tx_arb_locked          => tx_arb_locked,
    store_last_txtb_index  => store_last_txtb_index,
    frame_valid_com_set    => frame_valid_com_set,
    frame_valid_com_clear  => frame_valid_com_clear
  );
    


  ------------------------------------------------------------------------------
  -- Selecting TXT Buffer output word based on the chosen TXT Buffer. We use
  -- the combinationally selected buffer. If change on selected buffer occurs
  -- during selection, selection is restarted. Thus we can always during 
  -- selection use combinationally selected buffer !!!
  ------------------------------------------------------------------------------
  txtb_selected_input <= txt_buf_in(select_buf_index);
  

  ------------------------------------------------------------------------------
  -- Joined timestamp from TXT Buffer. Note that it is not always valid!
  -- Only when the TXT Buffer is addressed with upper timestamp word address!
  ------------------------------------------------------------------------------
  txtb_timestamp      <= txtb_selected_input & ts_low_internal;
  

  ------------------------------------------------------------------------------
  -- Comparing timestamp with external timestamp. This assumes that Upper 
  -- timestamp is on the output of buffer and lower timestamp is stored in
  -- "ts_low_internal".
  ------------------------------------------------------------------------------
  timestamp_valid    <= less_than(txtb_timestamp, timestamp);


  ------------------------------------------------------------------------------
  -- Invalid state of the buffer must be immediately available to the
  -- CAN Core, otherwise Core might attempt to lock buffer which was
  -- already aborted!
  -- During transmission, CAN Core is reading metadata from outputs. Since the
  -- frame is valid, it is logical to also have "tran_frame_valid" active!
  ------------------------------------------------------------------------------
  tran_frame_valid_out <= '1' when ((select_buf_avail = '1' and 
                                    tran_frame_valid_com = '1') or
                                    (tx_arb_locked = '1'))
                              else
                          '0';
  

  ------------------------------------------------------------------------------
  -- Output data word is selected based on the stored buffer index at the time
  -- of buffer locking.
  ------------------------------------------------------------------------------  
  tran_data_word_out   <= txt_buf_in(int_txtb_index);


  ------------------------------------------------------------------------------
  -- Output frame metadata and Identifier for CAN Core
  ------------------------------------------------------------------------------
  tran_dlc_out         <= tran_dlc_com;
  tran_is_rtr          <= tran_is_rtr_com;
  tran_ident_type_out  <= tran_ident_type_com;
  tran_frame_type_out  <= tran_frame_type_com;
  tran_brs_out         <= tran_brs_com;
  

  ------------------------------------------------------------------------------
  -- During Buffer selection, TX Arbitrator is addressing TXT Buffers.
  -- During Transmission, the Core is addressing TXT Buffers.
  -- In the last cycle when the unlock command comes, TX Arbitrator must
  -- already provide pointer from FSM, not the one from CAN Core. It is already
  -- addressing the lower timestamp word for timestamp selection!
  ------------------------------------------------------------------------------
  txtb_ptr            <= txtb_core_pointer when (tx_arb_locked = '1')
                                           else
                         txtb_pointer_meta;

  txt_hw_cmd_buf_index <= int_txtb_index;
  

  ------------------------------------------------------------------------------
  -- Register for loading lower 32 bits of CAN Frame timestamp
  ------------------------------------------------------------------------------
  low_ts_reg_proc : process(res_n, clk_sys)
  begin
    if (res_n = ACT_RESET) then
        ts_low_internal             <= (OTHERS => '0');
    elsif (rising_edge(clk_sys)) then
        if (store_ts_l_w = '1') then
            ts_low_internal         <= txtb_selected_input;
        end if;
    end if;
  end process;


  ------------------------------------------------------------------------------
  -- Registers for metadata commited to output of TX Arbitrator.
  ------------------------------------------------------------------------------
  meta_data_reg_proc : process(clk_sys, res_n)
  begin
    if (res_n = ACT_RESET) then
        tran_dlc_com                <= (OTHERS => '0');
        tran_is_rtr_com             <= '0';
        tran_ident_type_com         <= '0';
        tran_frame_type_com         <= '0';
        tran_brs_com                <= '0';
    elsif (rising_edge(clk_sys)) then
        if (store_md_w = '1') then
            tran_frame_type_com     <= txtb_selected_input(FDF_IND);
            tran_ident_type_com     <= txtb_selected_input(IDE_IND);
            tran_dlc_com            <= txtb_selected_input(DLC_H downto DLC_L);
            tran_is_rtr_com         <= txtb_selected_input(RTR_IND);
            tran_brs_com            <= txtb_selected_input(BRS_IND);
        end if;
    end if;
  end process;


  ------------------------------------------------------------------------------
  -- Register for "committed" valid frame output for CAN Core
  ------------------------------------------------------------------------------
  tran_frame_valid_com_proc : process(clk_sys, res_n)
  begin
    if (res_n = ACT_RESET) then
        tran_frame_valid_com        <= '0';
    elsif (rising_edge(clk_sys)) then
        if (frame_valid_com_set = '1') then
            tran_frame_valid_com    <= '1';
        elsif (frame_valid_com_clear = '1') then
            tran_frame_valid_com    <= '0';
        end if;
    end if;
  end process;


  ------------------------------------------------------------------------------
  -- Storing values of selected TXT Buffer index when selection process
  -- ends. Storing TXT Buffer at the time of LOCK from CAN Core. Two values
  -- are needed to determine change of selected TXT Buffer for CAN Core.
  -- CAN Core needs this information for erasing retransmitt limit counter.
  ------------------------------------------------------------------------------
  store_indices_proc : process(clk_sys, res_n)
  begin
    if (res_n = ACT_RESET) then
        last_txtb_index             <= 0;
        int_txtb_index              <= 0;

    elsif (rising_edge(clk_sys)) then

        -- At the time of lock, the last index is stored from the last stored
        -- index.
        if (store_last_txtb_index = '1') then
            last_txtb_index         <= int_txtb_index;
        end if;

        -- Combinationally selected index (select_buf_index) is stored when
        -- metadata are stored.
        if (store_md_w = '1') then
            int_txtb_index          <= select_buf_index;
        end if;

    end if;
  end process;

  txtb_changed        <= '0' when (last_txtb_index = int_txtb_index)
                             else
                         '1';


  ------------------------------------------------------------------------------
  -- Registering value of combinationally selected index by priority decoder
  -- to determine change and signal restarting selection process to 
  -- TX Arbitrator FSM.
  ------------------------------------------------------------------------------
  sel_index_change_proc : process(clk_sys, res_n)
  begin
    if (res_n = ACT_RESET) then
        select_buf_index_reg  <= 0;
    elsif (rising_edge(clk_sys)) then
        select_buf_index_reg        <= select_buf_index;
    end if;
  end process;

  select_index_changed <= '0' when (select_buf_index = select_buf_index_reg)
                              else
                          '1';


  ------------------------------------------------------------------------------
  -- Storing value of metadata pointer to address TXT Buffer Timestamp and
  -- Metadata Words.
  ------------------------------------------------------------------------------
  store_meta_data_ptr_proc : process(clk_sys, res_n)
  begin
    if (res_n = ACT_RESET) then
        txtb_pointer_meta           <= to_integer(unsigned(
                                        TIMESTAMP_L_W_ADR(11 downto 2)));
    elsif (rising_edge(clk_sys)) then

        if (load_ts_lw_addr = '1') then
            txtb_pointer_meta       <= to_integer(unsigned(
                                         TIMESTAMP_L_W_ADR(11 downto 2)));
        elsif (load_ts_uw_addr = '1') then
            txtb_pointer_meta       <= to_integer(unsigned(
                                         TIMESTAMP_U_W_ADR(11 downto 2)));
        elsif (load_ffmt_w_addr = '1') then
            txtb_pointer_meta       <= to_integer(unsigned(
                                         FRAME_FORM_W_ADR(11 downto 2)));
        end if;

    end if;
  end process;


end architecture;
