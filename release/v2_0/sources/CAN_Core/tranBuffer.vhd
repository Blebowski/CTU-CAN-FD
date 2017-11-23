Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;

-------------------------------------------------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
--
--    July 2015   Created file
-------------------------------------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------
-- Purpose:
--  Buffer of CAN core for storing transcieve data. Message data and parameters are stored  from ---
--  output of TXArbitrator circuit when bus is idle and transmit can start or SOF of other frame ---
--  is detected. Data stored in the buffer stay until it is rewritten by another message.        ---
--  Message is stored until sucessfully transmitted or retransmitt limit is reached              ---
--------------------------------------------------------------------------------------------------

entity tranBuffer is 
  port(
    -------------------
    --Clock and reset--
    -------------------
    signal clk_sys              :in   std_logic; --System clock
    signal res_n                :in   std_logic;
    
    --------------------------
    --TX Arbitrator Interface-
    --------------------------
    signal tran_data_in         :in   std_logic_vector(511 downto 0);
    signal tran_ident_in        :in   std_logic_vector(28 downto 0);
    signal tran_dlc_in          :in   std_logic_vector(3 downto 0);
    signal tran_is_rtr_in       :in   std_logic;
    signal tran_ident_type_in   :in   std_logic; --TX Identifier type (0-Basic,1-Extended);
    signal tran_frame_type_in   :in   std_logic; --TX Frame type (0-CAN Normal, 1-CAN FD)
    signal tran_brs_in          :in   std_logic;
    
    --------------------
    --Control signals --
    --------------------
    signal frame_store          :in   std_logic; --Store the data on input
    
    --------------------------------
    --Stored data register outputs--
    --------------------------------
    signal tran_data            :out  std_logic_vector(511 downto 0);
    signal tran_ident           :out  std_logic_vector(28 downto 0);
    signal tran_dlc             :out  std_logic_vector(3 downto 0);
    signal tran_is_rtr          :out  std_logic;
    signal tran_ident_type      :out  std_logic;
    signal tran_frame_type      :out  std_logic;
    signal tran_brs             :out  std_logic
  
  );
  
  ----------------------
  --Internal registers--
  ----------------------
  signal tran_data_reg:std_logic_vector(511 downto 0);
  signal tran_ident_reg: std_logic_vector(28 downto 0);
  signal tran_dlc_reg: std_logic_vector(3 downto 0);
  signal tran_is_rtr_reg: std_logic;
  signal tran_ident_type_reg: std_logic;
  signal tran_frame_type_reg: std_logic;
  signal tran_brs_reg:std_logic;
  
end entity;


architecture rtl of tranBuffer is 
begin
  tran_data               <=  tran_data_reg;
  tran_ident              <=  tran_ident_reg;
  tran_dlc                <=  tran_dlc_reg;
  tran_is_rtr             <=  tran_is_rtr_reg;
  tran_ident_type         <=  tran_ident_type_reg;
  tran_frame_type         <=  tran_frame_type_reg;
  tran_brs                <=  tran_brs_reg;
  
  data_store:process(clk_sys,res_n)
  begin
  if res_n=ACT_RESET then
    tran_data_reg         <=  (OTHERS =>'0');
    tran_ident_reg        <=  (OTHERS =>'0');
    tran_dlc_reg          <=  (OTHERS =>'0');
    tran_is_rtr_reg       <=  '0';
    tran_ident_type_reg   <=  '0';
    tran_frame_type_reg   <=  '0';
    tran_brs_reg          <=  '0';
  elsif rising_edge(clk_sys)then 
    if(frame_store='1')then
      tran_data_reg       <=  tran_data_in;
      tran_ident_reg      <=  tran_ident_in;
      tran_dlc_reg        <=  tran_dlc_in;
      tran_is_rtr_reg     <=  tran_is_rtr_in;
      tran_ident_type_reg <=  tran_ident_type_in;
      tran_frame_type_reg <=  tran_frame_type_in;
      tran_brs_reg        <=  tran_brs_in;
    else
      tran_data_reg       <=  tran_data_reg;
      tran_ident_reg      <=  tran_ident_reg;
      tran_dlc_reg        <=  tran_dlc_reg;
      tran_is_rtr_reg     <=  tran_is_rtr_reg;
      tran_ident_type_reg <=  tran_ident_type_reg;
      tran_frame_type_reg <=  tran_frame_type_reg;
      tran_brs_reg        <=  tran_brs_reg;
    end if;
  end if;  
  end process;
  
end architecture;