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
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.ID_transfer.all;
use work.CANComponents.all;

entity txArbitrator is
  generic(
    buf_count   : natural range 1 to 8;
    tx_time_sup : boolean := true
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
    signal txt_buf_data_in        :in txtb_data_type;
    
    -- Meta data available so far paralell on the output of the
    -- buffers.
    signal txt_meta_data_in       :in txtb_meta_data_type;
    
    -- Signal that buffer is in "Ready state", it can be selected
    -- by arbitrator
    signal txt_buf_ready          :in std_logic_vector(buf_count - 1 downto 0);
    
    
    -----------------------
    -- CAN Core Interface
    -----------------------
    
    -- TX Message data
    signal tran_data_word_out     :out std_logic_vector(31 downto 0);
    
    --TX Identifier
    signal tran_ident_out         :out std_logic_vector(28 downto 0);
    
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
    
    --Signal for CAN Core that frame on the output is valid and can be stored 
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
  
  --------------------
  --Internal signals--
  --------------------
  
  --State machine for following when the frame was already transmitted!
  signal tx_arb_fsm               :tx_arb_state_type;  
    
  -- Timestamp from TXT Buffers
  signal buf_ts           : txtb_timestamps_type;
  
  -- Timestamps are considered to be valid when are lower than external timestamp
  signal buf_ts_valid     : std_logic_vector(TXT_BUFFER_COUNT - 1 downto 0);
  
  -- Indicates the highest selected buffer and its validity
  signal select_buf_avail     : boolean;
  signal select_buf_index     : natural range 0 to buf_count - 1;
  
  -- If the Core locks the buffer for transmission, it needs to have
  -- data provided from the frame which was selected at the time
  -- of the locking. During the transmission the actual selected buffer
  -- will change!
  signal stored_buf_index     : natural range 0 to buf_count - 1;
  
  -- Internal value whether the tx buffer changed
  signal txtb_changed_reg     : std_logic;
    
  
  --Comparing procedure for two 64 bit std logic vectors
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

architecture rtl of txArbitrator is
begin
  
  
  ------------------------------------------------------------------------------
  -- Priority decoder on TXT Buffers
  ------------------------------------------------------------------------------
  priorityDecoder_comp : priorityDecoder 
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
  -- Assigning the internal aliases for buffer timestamps.
  -- Comparators of timestamps for each TXT Buffer (with external timestamp)
  --
  -- Note that if TX Transmission is not synthesized the frame is available 
  -- as soon as its buffer is in "Ready" state.
  ------------------------------------------------------------------------------
  ts_alias_gen: for i in 0 to TXT_BUFFER_COUNT - 1 generate
    buf_ts(i)       <= txt_meta_data_in(i)(575 downto 512);
    buf_ts_valid(i) <= less_than(buf_ts(i), timestamp) when tx_time_sup
                                                       else
                        '1';
  end generate;
  
   
  ------------------------------------------------------------------------------
  -- Check whether the timestamp of highest priority valid
  -- buffer is lower than external time. This creates the function
  -- of transmission at given time
  ------------------------------------------------------------------------------
  tran_frame_valid_out  <= '1' when ((select_buf_avail = true) and
                                     buf_ts_valid(select_buf_index) = '1')
                             else
                            '0';     


	------------------------------------------------------------------------------
  -- Multiplexing meta-data from buffers into message lines to CAN Core 
  ------------------------------------------------------------------------------
  tran_dlc_out          <= txt_meta_data_in(select_buf_index)(611 downto 608);
  tran_is_rtr           <= txt_meta_data_in(select_buf_index)(613);
  tran_ident_type_out   <= txt_meta_data_in(select_buf_index)(614);
  tran_frame_type_out   <= txt_meta_data_in(select_buf_index)(615);
  tran_brs_out          <= txt_meta_data_in(select_buf_index)(617);
  tran_ident_out        <= txt_meta_data_in(select_buf_index)
                              (TXT_IDW_HIGH-3 downto TXT_IDW_LOW);
  
  
  ------------------------------------------------------------------------------
  -- Data which goes to the CAN Core has to be decided on message source
  -- which was sampled when frame info was stored into Core. This way if
  -- other buffer (higher priority) is marked as Ready, Protocol control will
  -- still access the original buffer!
  ------------------------------------------------------------------------------
  tran_data_word_out    <= txt_buf_data_in(stored_buf_index);
  
  -- TXT Buffer was changed, this is used in Protocol control to reset the
  -- retransmitt counter
  txtb_changed          <= '1' when (stored_buf_index /= select_buf_index)
                               else
                           '0';
  
  -- When Buffer is unlocked from the core (tx_arb_fsm = idle), then CAN Core
  -- locks the buffer which is actually selected by priority decoder.
  -- When Buffer is locked, the core must send the command to the buffer which
  -- was previously locked!
  txt_hw_cmd_buf_index  <= select_buf_index when (tx_arb_fsm = arb_idle)
                                            else
                           stored_buf_index;   
                                          
  
  ------------------------------------------------------------------------------
  -- State machine for deciding whether the frame transmission finished and
  -- it can be already erased.
  ------------------------------------------------------------------------------
  proc_txarb_fsm:process(clk_sys,res_n)
  begin
    if (res_n=ACT_RESET) then
      tx_arb_fsm            <= arb_idle;
      stored_buf_index      <= 0;
    elsif rising_edge(clk_sys) then
        
        tx_arb_fsm          <= tx_arb_fsm;
        stored_buf_index    <= stored_buf_index;
        txtb_changed_reg    <= txtb_changed_reg;
        
      case tx_arb_fsm is 
      
      --------------------------------------------------------------------------
      -- Waiting for Protocol control to give command that it stored metadata
      -- with information and start the transmission...
      --------------------------------------------------------------------------
      when arb_idle =>
        if (txt_hw_cmd.lock = '1') then
          tx_arb_fsm            <= arb_trans;
          stored_buf_index      <= select_buf_index;    
        end if;
        
      --------------------------------------------------------------------------
      -- Waiting for signal that frame transmission ended succesfully and buffer
      -- can be erased!
      --------------------------------------------------------------------------
      when arb_trans =>
        if (txt_hw_cmd.unlock = '1')then
          tx_arb_fsm      <= arb_idle;
        end if;
        
      when others =>
        report "Error - Unknow TX Arbitrator state" severity error;
      end case;
      
    end if;
  end process;
        
  
end architecture;