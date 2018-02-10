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

--------------------------------------------------------------------------------
-- Purpose:
--  Circuit for selecting the valid message for CAN Core from two TXT buffer in-
--   puts. Circuit compares the timestamp of the message with input timestamp
--   (actual time) and allows the message to be propagated to the CAN Core when 
--  the time Stamp of message is higher than actual timestamp! This realises the
--  functionality of sending the message in exact time! When both timeStamp are 
--  equal and then message with lower identifier is selected!                                                                                                                                                  
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
--   27.12.2017   Added "tran_lock", "tran_unlock", "tran_drop" signals for
--                implementation of frame swapping feature. Replaced 
--                "tran_data_ack" with "tran_lock" signal.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.ID_transfer.all;

entity txArbitrator is
  generic(
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
   
    --Time TX1 buffer input
    signal txt1buf_info_in        :in  std_logic_vector(639 downto 512);
    
    --Time TX1 buffer input
    signal txt1buf_data_in        :in  std_logic_vector(31 downto 0);
    
    --No message in Time TX Buffer
    signal txt1_buffer_empty      :in  std_logic;
    
    --Time buffer acknowledge that message can be erased
    signal txt1_buffer_ack        :out std_logic;
    
    --Time TX2 buffer input
    signal txt2buf_info_in        :in  std_logic_vector(639 downto 512);
    
    --Time TX2 buffer input
    signal txt2buf_data_in        :in  std_logic_vector(31 downto 0);
    
    --No message in Time TX Buffer
    signal txt2_buffer_empty      :in  std_logic;
    
    --Time buffer acknowledge that message can be erased
    signal txt2_buffer_ack        :out std_logic;
    
  
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
    
    -- CAN Core started to transmitt the Data from TXT Buffer
    -- TX Arbitrator should store the source buffer
    signal tran_lock              :in std_logic;
    
    -- CAN Core is not anymore transmitting the Data from the TXT Buffer
    signal tran_unlock            :in std_logic;
    
    -- CAN Core signalises that frame was either succesfully transmitter or 
    -- error limit was reached and it can be dropped.
    signal tran_drop              :in std_logic;
    
    -- If error occurs during the transmission, and CAN Core picks
    -- frame again, CAN Core needs to know that different buffer is now
    -- selected, so that it can erase the retransmitt counter (in case
    -- retransmitt limit is enabled).
    signal mess_src_change        :out std_logic;
    
    ---------------------
    -- Driving interface
    ---------------------
    
    --Driving bus from registers
    signal drv_bus                :in std_logic_vector(1023 downto 0);
    
    --TimeStamp value
    signal timestamp              :in std_logic_vector(63 downto 0)
        
  );
  
  --------------------
  --Internal signals--
  --------------------
  
  --Joined signal for valid signals from buffers
  signal valid_join               :std_logic_vector(1 downto 0);
  
  --Message source (0-normal buffer, 1-Time based buffer)
  signal mess_src                 :std_logic;
  
  -- Message source of the actually transmitted frame
  signal mess_src_reg             :std_logic;
  
  -------------------
  --Internal aliases-
  -------------------
  
  --Time value for message from TXT1 buffer to be sent!
  signal mess_time1               :std_logic_vector(63 downto 0);
  
  --Time value for message from TXT2 buffer to be sent!
  signal mess_time2               :std_logic_vector(63 downto 0);
  
  signal ident1                   :std_logic_vector(28 downto 0);
  signal ident2                   :std_logic_vector(28 downto 0);
  
  --Message time 1 or 2 is lower than timeStam
  signal ts_valid                 :std_logic_vector(1 downto 0);
  
  signal mess_src_change_reg      :std_logic;
  
  --Allow transmit of messages from tx buffers 1,2
  signal drv_allow_txt1           :std_logic;
  signal drv_allow_txt2           :std_logic;
  
  --Decimal values of identifier
  signal id_1_dec                 :natural;
  signal id_2_dec                 :natural;
  
   --Comparing procedure for two 64 bit std logic vectors
    function less_than(
      signal   a       : in std_logic_vector(63 downto 0);
      signal   b       : in std_logic_vector(63 downto 0)
    )return boolean is
    begin
       if (unsigned(a(63 downto 32)) < unsigned(b(63 downto 32))) or 
          ((a(63 downto 32) = b(63 downto 32)) and 
          (unsigned(a(31 downto 0)) < unsigned(b(31 downto 0))))then
          return true;
      else
         return false;
      end if;
   
    end function;
  
  --Message time 1 less than Messa time 2
  signal mt1_lt_mt2                :boolean;
  signal mt1_lt_ts                 :boolean;
  signal mt2_lt_ts                 :boolean;
  
  --State machine for following when the frame was already transmitted!
  signal tx_arb_fsm                :tx_arb_state_type;
  
end entity;

architecture rtl of txArbitrator is
begin
  
  --Driving bus aliases
  drv_allow_txt1      <= drv_bus(DRV_ALLOW_TXT1_INDEX);
  drv_allow_txt2      <= drv_bus(DRV_ALLOW_TXT2_INDEX);
  
  --Joining valid signals into one vector value
  valid_join          <= ((not txt1_buffer_empty) and (drv_allow_txt1))&
                          ((not txt2_buffer_empty) and (drv_allow_txt2));
  
  --Transmit time of TXT Buffer messages (from both buffers)
  mess_time1          <= txt1buf_info_in(TXT_TSUPP_HIGH downto TXT_TSUPP_LOW)&
                         txt1buf_info_in(TXT_TSLOW_HIGH downto TXT_TSLOW_LOW);
  mess_time2          <= txt2buf_info_in(TXT_TSUPP_HIGH downto TXT_TSUPP_LOW)&
                         txt2buf_info_in(TXT_TSLOW_HIGH downto TXT_TSLOW_LOW);
  
  --Transmit identifiers
  ident1              <= txt1buf_info_in(TXT_IDW_HIGH-3 downto TXT_IDW_LOW);
  ident2              <= txt2buf_info_in(TXT_IDW_HIGH-3 downto TXT_IDW_LOW);
  
  --Comparator methods for 64 bit vectors
  tx_gen_true:if (tx_time_sup=true) generate
    mt1_lt_mt2         <= less_than(mess_time1,mess_time2);
    mt1_lt_ts          <= less_than(mess_time1,timestamp);
    mt2_lt_ts          <= less_than(mess_time2,timestamp);
  end generate;
  
  tx_gen_false:if (tx_time_sup=false) generate
    mt1_lt_mt2  <= true;
    mt1_lt_ts   <= true;
    mt2_lt_ts   <= true;
  end generate;
  
  ------------------------------------------------------------------------------
  --Message can be transmitted when transmitt timestamp is lower than the actual
  --timestamp value
  ------------------------------------------------------------------------------
  
    ts_valid(0)   <= '1' when (  --Timestamp higher than transmitt time
                              ( mt2_lt_ts=true ) 
                              and 
                              ( -- Message is available and buffer allowed
                               valid_join(0)='1'
                              )
                              )
                         else 
                      '0';
                                            
    --Buffer 1                                     
    ts_valid(1)   <= '1' when (  --Timestamp higher than transmitt time
                              ( mt1_lt_ts=true ) 
                              and
                              (  -- Message is available and buffer allowed
                              valid_join(1)='1'
                              )
                              ) 
                         else 
                     '0';
  
  ------------------------------------------------------------------------------
  --Determine the Buffer from which the frame should be stored.
  --
  -- Frame is considered from TXT Buffer 1 if:
  --  1. Only buffer 1 has valid frame to transmit
  --  2. Both buffers have valid frames but transmit
  --     time of Buffer 1 Frame is lower.
  --  3. Both buffers have valid frames and the
  --     time to transmitt is equal and Identifier
  --     of buffer 1 frame is lower!
  ------------------------------------------------------------------------------
  ID_reg_to_decimal(ident1,id_1_dec);
  ID_reg_to_decimal(ident2,id_2_dec);

  mess_src    <= '0' when (  --Only buffer 1 has frame
                          (ts_valid     = "10") 
                          or 
                          ( --Both have frame and message time 1 is lower
                            (ts_valid   = "11") 
                            and 
                            ( mt1_lt_mt2 = true ) 
                          ) 
                          or
                          ( -- Both have frames, message time is equal, identifier decides
                            (ts_valid   = "11") 
                            and 
                            (mess_time2 = mess_time1) 
                            and  
                            (id_1_dec   <= id_2_dec)
                          )  
                          ) 
                     else 
                 '1';
                      
  ------------------------------------------------------------------------------
  --Frame on the output is valid if 
  --at least one of the frames is valid
  ------------------------------------------------------------------------------
	tran_frame_valid_out      <= '1' when (ts_valid="10" or 
                                           ts_valid="01" or
                                           ts_valid="11")
                                     else 
	                             '0';
	
	------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  ---- Multiplexing data from buffers into message lines to CAN Core 
  ------------------------------------------------------------------------------
  ------------------------------------------------------------------------------
  
  ---------------------
  --Frame format word--
  ---------------------
  --Data length Code
  tran_dlc_out              <= txt1buf_info_in(611 downto 608)   when mess_src_reg='0' else 
                               txt2buf_info_in(611 downto 608);
  --RTR Frame                       
  tran_is_rtr               <= txt1buf_info_in(613)              when mess_src_reg='0' else 
                               txt2buf_info_in(613);
  --Identifier Type
  tran_ident_type_out       <= txt1buf_info_in(614)              when mess_src_reg='0' else 
                               txt2buf_info_in(614);
  --Frame Type
  tran_frame_type_out       <= txt1buf_info_in(615)              when mess_src_reg='0' else
                               txt2buf_info_in(615);
  --Bit rate shift
  tran_brs_out              <= txt1buf_info_in(617)              when mess_src_reg='0' else 
                               txt2buf_info_in(617);
                               
  ------------------------------------------------------------------------------
  --NOTE: TimeStamp Words skipped since Timestamp prioritization is already 
  --      achieved by comparing actual and message timestamps in TxArbitrator.
  ------------------------------------------------------------------------------
  
  ----------------------------------
  --Identifier word and data words--
  ----------------------------------
  tran_ident_out            <= txt1buf_info_in(TXT_IDW_HIGH-3 downto TXT_IDW_LOW)
                               when mess_src='0' else 
                               txt1buf_info_in(TXT_IDW_HIGH-3 downto TXT_IDW_LOW);
  
  ------------------------------------------------------------------------------
  --Data which goes to the CAN Core has to be decided on message source
  -- which was sampled when frame info was stored into Core. This way even if
  -- the other buffer is modified, the data source remain correct for the whole
  -- duration of transmission!
  ------------------------------------------------------------------------------
  tran_data_word_out <=  txt1buf_data_in when mess_src_reg='0' else
                         txt2buf_data_in when mess_src_reg='1' else
                         (OTHERS => '0');
  
  -- TXT Buffer was changed
  mess_src_change <= mess_src_change_reg;
  
  ------------------------------------------------------------------------------
  -- State machine for deciding whether the frame transmission finished and
  -- it can be already erased.
  ------------------------------------------------------------------------------
  proc_txarb_fsm:process(clk_sys,res_n)
  begin
    if (res_n=ACT_RESET) then
      tx_arb_fsm <= arb_idle;
      mess_src_reg <= '0';
      mess_src_change_reg <= '0';
    elsif rising_edge(clk_sys) then
        
        tx_arb_fsm <= tx_arb_fsm;
        mess_src_reg <= mess_src_reg;
        
        --By default we dont give acknowledge to any buffer
        txt1_buffer_ack <= '0';
        txt2_buffer_ack <= '0';
        
        mess_src_change_reg <= mess_src_change_reg;
        
      case tx_arb_fsm is 
      
      --------------------------------------------------------------------------
      -- Waiting for Protocol control to give command that it stored the 4 words
      -- with information and start the transmission...
      --------------------------------------------------------------------------
      when arb_idle =>
        if (tran_lock = '1') then
          tx_arb_fsm <= arb_trans;
          mess_src_reg <= mess_src; -- Store when frame info goes to the Core
          
          if (mess_src_reg /= mess_src) then
             mess_src_change_reg  <= '1';
          else
             mess_src_change_reg  <= '0';
          end if;
        end if;
        
      --------------------------------------------------------------------------
      -- Waiting for signal that frame transmission ended succesfully and buffer
      -- can be erased!
      --------------------------------------------------------------------------
      when arb_trans =>
        if (tran_unlock = '1')then
          tx_arb_fsm <= arb_idle;
          
          if (tran_drop = '1') then
            if (mess_src_reg = '0') then 
              txt1_buffer_ack <= '1';
            elsif (mess_src_reg = '1') then
              txt2_buffer_ack <= '1';
            else
              txt1_buffer_ack <= '0';
              txt2_buffer_ack <= '0';
            end if;
          else
            txt1_buffer_ack <= '0';
            txt2_buffer_ack <= '0';
          end if;
            
        end if;
        
      when others =>
        report "Error - Unknow TX Arbitrator state" severity error;
      end case;
      
    end if;
  end process;
  
end architecture;