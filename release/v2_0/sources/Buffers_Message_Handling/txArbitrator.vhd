Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
use work.ID_transfer.all;

-------------------------------------------------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
--
--    July 2015   Created file
--    17.1.2016   Added ID change from register value to decimal value for case when identifier needs
--                to decide about priority message (Time stamps are equal)
--    7.6.2016    Added "less_than" function for comparison of two 64 bit std_logic_vectors. Integer
--                does not support more than 64 bits. When timestamp higher than 32 bits was simulated
--                in unit test simulator was throwing out milions of warnings!
--    23.6.2016   Added less or equal to the case when both timestamps and both identifiers are equal.
--                Thisway identifier from Buffer 1 instead of Buffer 2 is propagated!
--
-------------------------------------------------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------
-- Purpose:
--  Circuit for selecting the valid message for CAN Core from two TXT buffer inputs. Circuit compares the
--  timestamp of the message with input timestamp(actual time) and allows the message to be propagated   
--  for CAN Core when the time Stamp of message is higher than actual timestamp! This realises the func- 
--  tionality of sending the message in exact time! When both timeStamp are equal and then message with  
--  lower identifier is selected!                                                                        
--                                                                                                               
--  Both input buffers may be set to "not allowed" by driving bus.                                       
-----------------------------------------------------------------------------------------------------------

entity txArbitrator is 
  port( 
    ------------------------
    --TX Buffers interface--
    ------------------------
    --TXT Buffer 1
    signal txt1_buffer_in         :in  std_logic_vector(639 downto 0);     --Time TX1 buffer input
    signal txt1_buffer_empty      :in  std_logic;                          --No message in Time TX Buffer
    signal txt1_buffer_ack        :out std_logic;                          --Time buffer acknowledge that 
                                                                           -- message can be erased
    
    --TXT Buffer 2
    signal txt2_buffer_in         :in  std_logic_vector(639 downto 0);     --Time TX1 buffer input
    signal txt2_buffer_empty      :in  std_logic;                          --No message in Time TX Buffer
    signal txt2_buffer_ack        :out std_logic;                          --Time buffer acknowledge that 
                                                                           -- message can be erased
  
    -----------------------
    --CAN Core Interface---
    -----------------------
    signal tran_data_out          :out std_logic_vector(511 downto 0);    --TX Message data
    signal tran_ident_out         :out std_logic_vector(28 downto 0);     --TX Identifier 
    signal tran_dlc_out           :out std_logic_vector(3 downto 0);      --TX Data length code
    signal tran_is_rtr            :out std_logic;                         --TX is remote frame
    signal tran_ident_type_out    :out std_logic;                         --TX Identifier type (0-Basic,1-Extended);
    signal tran_frame_type_out    :out std_logic;                         --TX Frame type
    signal tran_brs_out           :out std_logic;                         --Bit rate shift for CAN FD frames
    signal tran_frame_valid_out   :out std_logic;                         --Signal for CAN Core that frame on the 
                                                                          -- output is valid and can be stored for transmitting
    
    signal tran_data_ack          :in  std_logic;                         --Acknowledge from CAN core that acutal message was 
                                                                          -- stored into internal buffer for transmitting   
    ---------------------
    --Driving interface--
    ---------------------
    signal drv_bus                :in std_logic_vector(1023 downto 0);    --Driving bus from registers
    signal timestamp              :in std_logic_vector(63 downto 0)       --TimeStamp value
        
  );
  
  --------------------
  --Internal signals--
  --------------------
  signal valid_join               :std_logic_vector(1 downto 0);          --Joined signal for valid signals from buffers
  signal mess_src                 :std_logic;                             --Message source (0-normal buffer, 1-Time based buffer)
  
  -------------------
  --Internal aliases-
  -------------------
  signal mess_time1               :std_logic_vector(63 downto 0);         --Time value for message from TXT1 buffer to be sent!
  signal mess_time2               :std_logic_vector(63 downto 0);         --Time value for message from TXT2 buffer to be sent!
  
  signal ident1                   :std_logic_vector(28 downto 0);
  signal ident2                   :std_logic_vector(28 downto 0);
  
  signal ts_valid                 :std_logic_vector(1 downto 0);          --Message time 1 or 2 is lower than timeStam
  
  --Driving bus
  signal drv_allow_txt1           :std_logic;                             --Allow transmit of messages from tx buffer
  signal drv_allow_txt2           :std_logic;                             --Allow transmit of messages from txt buffer
  
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
          ((a(63 downto 32) = b(63 downto 32)) and (unsigned(a(31 downto 0)) < unsigned(b(31 downto 0))))then
          return true;
      else
         return false;
      end if;
   
    end function;
  
  --Message time 1 less than Messa time 2
  signal mt1_lt_mt2                :boolean;
  signal mt1_lt_ts                 :boolean;
  signal mt2_lt_ts                 :boolean;
  
  
end entity;

architecture rtl of txArbitrator is
begin
  
  --Driving bus aliases
  drv_allow_txt1            <= drv_bus(DRV_ALLOW_TXT1_INDEX);
  drv_allow_txt2            <= drv_bus(DRV_ALLOW_TXT2_INDEX);
  
  --Joining valid signals into one vector value
  valid_join                <= ((not txt1_buffer_empty) and (drv_allow_txt1))&
                               ((not txt2_buffer_empty) and (drv_allow_txt2)); 
  
  --Transmit time of TXT Buffer messages (from both buffers)
  mess_time1                <= txt1_buffer_in(TXT_TSUPP_HIGH downto TXT_TSLOW_LOW);
  mess_time2                <= txt2_buffer_in(TXT_TSUPP_HIGH downto TXT_TSLOW_LOW); 
  
  --Transmit identifiers
  ident1                    <= txt1_buffer_in(TXT_IDW_HIGH-3 downto TXT_IDW_LOW);
  ident2                    <= txt2_buffer_in(TXT_IDW_HIGH-3 downto TXT_IDW_LOW);
  
  --Comparator methods for 64 bit vectors
   mt1_lt_mt2 <= less_than(mess_time1,mess_time2);
   mt1_lt_ts  <= less_than(mess_time1,timestamp);
   mt2_lt_ts  <= less_than(mess_time2,timestamp);              
  
  -------------------------------------------------------
  --Message can be transmitted when transmitt timestamp--
  --is lower than the actual timestamp value
  -------------------------------------------------------
  
    ts_valid(0)               <= '1' when (  --Timestamp higher than transmitt time
                                            ( mt2_lt_ts=true ) 
                                            and 
                                            ( -- Message is available and buffer allowed
                                            valid_join(0)='1'
                                            )
                                          )
                                     else 
                                 '0';
                                            
    --Buffer 1                                     
    ts_valid(1)               <= '1' when (  --Timestamp higher than transmitt time
                                            ( mt1_lt_ts=true ) 
                                            and
                                            (  -- Message is available and buffer allowed
                                             valid_join(1)='1'
                                            )
                                          ) 
                                     else 
   
                               '0';
  
  ------------------------------------------------
  --Determine the Buffer from which the frame
  --should be stored.
  --
  -- Frame is considered from TXT Buffer 1 if:
  --  1. Only buffer 1 has valid frame to transmit
  --  2. Both buffers have valid frames but transmit
  --     time of Buffer 1 Frame is lower.
  --  3. Both buffers have valid frames and the
  --     time to transmitt is equal and Identifier
  --     of buffer 1 frame is lower!
  ------------------------------------------------
  ID_reg_to_decimal(ident1,id_1_dec);
  ID_reg_to_decimal(ident2,id_2_dec);

  mess_src                  <= '0' when (  --Only buffer 1 has frame
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
                      
  ----------------------------------------                    
  --Frame on the output is valid if 
  --at least one of the frames is valid     
  ----------------------------------------             
	tran_frame_valid_out      <= '1' when (ts_valid="10" or ts_valid="01" or ts_valid="11") else 
	                             '0';
	
	
	----------------------------------------------------------------------
  ----------------------------------------------------------------------
  ----Multiplexing data from buffers into message lines to CAN Core ----
  ----------------------------------------------------------------------
  ----------------------------------------------------------------------
  
  ---------------------
  --Frame format word--
  ---------------------
  --Data length Code
  tran_dlc_out              <= txt1_buffer_in(611 downto 608)   when mess_src='0' else 
                               txt2_buffer_in(611 downto 608);
  --RTR Frame                       
  tran_is_rtr               <= txt1_buffer_in(613)              when mess_src='0' else 
                               txt2_buffer_in(613);
  --Identifier Type
  tran_ident_type_out       <= txt1_buffer_in(614)              when mess_src='0' else 
                               txt2_buffer_in(614);
  --Frame Type
  tran_frame_type_out       <= txt1_buffer_in(615)              when mess_src='0' else
                               txt2_buffer_in(615);
  --Bit rate shift
  tran_brs_out              <= txt1_buffer_in(617)              when mess_src='0' else 
                               txt2_buffer_in(617);
                               
  -----------------------------------------------------------------------                             
  --NOTE: TimeStamp Words skipped since timeStamp prioritization is 
  --       already achieved by comparing actual and message timestamps
  --       inside TxArbitrator.
  -----------------------------------------------------------------------
  
  ----------------------------------
  --Identifier word and data words--
  ----------------------------------
  tran_ident_out            <= txt1_buffer_in(540 downto 512)   when mess_src='0' else 
                               txt2_buffer_in(540 downto 512);
  tran_data_out             <= txt1_buffer_in(511 downto 0)     when mess_src='0' else 
                               txt2_buffer_in(511 downto 0);
  
  -------------------------------------------------------------
  --Confirmation for TX or TXT Buffer that message was stored--
  -- the in CAN Core and it can be erased from TXT buffer    --
  -------------------------------------------------------------
  txt1_buffer_ack           <= '1' when ((tran_data_ack='1') AND (mess_src='0')) else 
                               '0';
  txt2_buffer_ack           <= '1' when ((tran_data_ack='1') AND (mess_src='1')) else 
                               '0'; 
  
end architecture;