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
--  Buffer of CAN core for storing transcieve info. Frame informations (DLC,
--  Identifier, etc. are stored from output of TXArbitrator circuit when bus is 
--  idle and transmit can start or SOF of other frame is detected. Data stored 
--  in the buffer stay until it is rewritten by another message. Message is 
--  stored until sucessfully transmitted or retransmitt limit is reached.
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--    04.12.2017  Removed "tran_data_in" and "tran_data_reg" from the buffer
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CAN_FD_frame_format.all;

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
    signal tran_ident_in        :in   std_logic_vector(28 downto 0);
    signal tran_dlc_in          :in   std_logic_vector(3 downto 0);
    signal tran_is_rtr_in       :in   std_logic;
    
    --TX Identifier type (0-Basic,1-Extended);
    signal tran_ident_type_in   :in   std_logic;
    
    --TX Frame type (0-CAN Normal, 1-CAN FD)
    signal tran_frame_type_in   :in   std_logic;
    signal tran_brs_in          :in   std_logic;
    
    --------------------
    --Control signals --
    --------------------
    signal frame_store          :in   std_logic; --Store the data on input
    
    --------------------------------
    --Stored data register outputs--
    --------------------------------
    signal tran_ident_base      :out  std_logic_vector(10 downto 0);
    signal tran_ident_ext       :out  std_logic_vector(17 downto 0);
    signal tran_dlc             :out  std_logic_vector(3 downto 0);
    signal tran_is_rtr          :out  std_logic;
    signal tran_ident_type      :out  std_logic;
    signal tran_frame_type      :out  std_logic;
    signal tran_brs             :out  std_logic
  
  );
  
  ----------------------
  --Internal registers--
  ----------------------
  signal tran_ident_base_reg: std_logic_vector(10 downto 0);
  signal tran_ident_ext_reg: std_logic_vector(17 downto 0);
  signal tran_dlc_reg: std_logic_vector(3 downto 0);
  signal tran_is_rtr_reg: std_logic;
  signal tran_ident_type_reg: std_logic;
  signal tran_frame_type_reg: std_logic;
  signal tran_brs_reg:std_logic;
  
end entity;


architecture rtl of tranBuffer is 
begin
  tran_ident_base         <=  tran_ident_base_reg;
  tran_ident_ext          <=  tran_ident_ext_reg;
  tran_dlc                <=  tran_dlc_reg;
  tran_is_rtr             <=  tran_is_rtr_reg;
  tran_ident_type         <=  tran_ident_type_reg;
  tran_frame_type         <=  tran_frame_type_reg;
  tran_brs                <=  tran_brs_reg;
  
  data_store:process(clk_sys,res_n)
  begin
  if res_n=ACT_RESET then
    tran_ident_base_reg   <=  (OTHERS =>'0');
    tran_ident_ext_reg    <=  (OTHERS =>'0');
    tran_dlc_reg          <=  (OTHERS =>'0');
    tran_is_rtr_reg       <=  '0';
    tran_ident_type_reg   <=  '0';
    tran_frame_type_reg   <=  '0';
    tran_brs_reg          <=  '0';
  elsif rising_edge(clk_sys)then 
    if(frame_store='1')then
      tran_ident_base_reg <=  tran_ident_in(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L);
      tran_ident_ext_reg  <=  tran_ident_in(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L);
      tran_dlc_reg        <=  tran_dlc_in;
      tran_is_rtr_reg     <=  tran_is_rtr_in;
      tran_ident_type_reg <=  tran_ident_type_in;
      tran_frame_type_reg <=  tran_frame_type_in;
      tran_brs_reg        <=  tran_brs_in;
    else
      tran_ident_base_reg <=  tran_ident_base_reg;
      tran_ident_ext_reg  <=  tran_ident_ext_reg;
      tran_dlc_reg        <=  tran_dlc_reg;
      tran_is_rtr_reg     <=  tran_is_rtr_reg;
      tran_ident_type_reg <=  tran_ident_type_reg;
      tran_frame_type_reg <=  tran_frame_type_reg;
      tran_brs_reg        <=  tran_brs_reg;
    end if;
  end if;  
  end process;
  
end architecture;