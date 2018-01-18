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
--  Package with Address constants for CAN FD register map.
--------------------------------------------------------------------------------
-- Revision History:
--    20.12.2017	 Created file
--    06.1.2018   Separated registers CTR_PRES to make the IP-XACT spec. simple
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

package CAN_FD_register_map is

  ------------------------------------------------------------------------------
  --Adress Offsets
  ------------------------------------------------------------------------------
  --Note: It is considered that lowest two bits are cut!! Therefore every 
  --register is in processor mapped to 0x4 higher adress!! This is done to
  --achieve the uniform acess, when someone acesses 0x0000,0x0001,0x0002,0x0003 
  --adresses. This way lower bits will be cut and no adress alignment problems
  --will appear. The real adress(in processor) offset is 4 times value of adress
  --constant!
  
  --Control registers region
  constant CTRL_REGION_ADR         : std_logic_vector(3 downto 0) := x"0";
  
  constant DEVICE_ID_ADR           : std_logic_vector(11 downto 0) := x"000";
  constant MODE_REG_ADR            : std_logic_vector(11 downto 0) := x"001";
  constant INTERRUPT_REG_ADR       : std_logic_vector(11 downto 0) := x"002";
  constant TIMING_REG_ADR          : std_logic_vector(11 downto 0) := x"003";
  constant ARB_ERROR_PRESC_ADR     : std_logic_vector(11 downto 0) := x"004";
  constant ERROR_TH_ADR            : std_logic_vector(11 downto 0) := x"005";
  constant ERROR_COUNTERS_ADR      : std_logic_vector(11 downto 0) := x"006";
  constant ERROR_COUNTERS_SPEC_ADR : std_logic_vector(11 downto 0) := x"007";
  constant CTR_PRES                : std_logic_vector(11 downto 0) := x"008";
   
  constant FILTER_A_VAL_ADR        : std_logic_vector(11 downto 0) := x"009";
  constant FILTER_A_MASK_ADR       : std_logic_vector(11 downto 0) := x"00A";
  constant FILTER_B_VAL_ADR        : std_logic_vector(11 downto 0) := x"00B";
  constant FILTER_B_MASK_ADR       : std_logic_vector(11 downto 0) := x"00C";
  constant FILTER_C_VAL_ADR        : std_logic_vector(11 downto 0) := x"00D";
  constant FILTER_C_MASK_ADR       : std_logic_vector(11 downto 0) := x"00E";
  constant FILTER_RAN_LOW_ADR      : std_logic_vector(11 downto 0) := x"00F";
  constant FILTER_RAN_HIGH_ADR     : std_logic_vector(11 downto 0) := x"010";
  constant FILTER_CONTROL_ADR      : std_logic_vector(11 downto 0) := x"011";
  constant RX_INFO_1_ADR           : std_logic_vector(11 downto 0) := x"012";
  constant RX_INFO_2_ADR           : std_logic_vector(11 downto 0) := x"013";
  constant RX_DATA_ADR             : std_logic_vector(11 downto 0) := x"014";
  constant TRV_DELAY_ADR           : std_logic_vector(11 downto 0) := x"015";

  constant TX_STATUS_ADR   : std_logic_vector(11 downto 0) := x"016";
  constant TX_SETTINGS_ADR : std_logic_vector(11 downto 0) := x"017";
  constant ERR_CAPT_ADR : std_logic_vector(11 downto 0) := x"018";
 
  constant RX_COUNTER_ADR : std_logic_vector(11 downto 0) := x"02B";
  constant TX_COUNTER_ADR : std_logic_vector(11 downto 0) := x"02C";

  constant LOG_TRIG_CONFIG_ADR      : std_logic_vector(11 downto 0) := x"02E";
  constant LOG_TRIG_CONFIG_DATA_ADR : std_logic_vector(11 downto 0) := x"02F";
  constant LOG_CAPT_CONFIG_ADR      : std_logic_vector(11 downto 0) := x"030";
  constant LOG_STATUS_ADR           : std_logic_vector(11 downto 0) := x"031";
  constant LOG_CMD_ADR              : std_logic_vector(11 downto 0) := x"032";
  constant LOG_CAPT_EVENT_1_ADR     : std_logic_vector(11 downto 0) := x"033";
  constant LOG_CAPT_EVENT_2_ADR     : std_logic_vector(11 downto 0) := x"034";

  constant DEBUG_REG_ADR : std_logic_vector(11 downto 0) := x"035";
  constant YOLO_REG_ADR  : std_logic_vector(11 downto 0) := x"036";

  constant TX_DATA_REGION : std_logic_vector(3 downto 0)  := x"1"; 
  constant TX_DATA_1_ADR  : std_logic_vector(11 downto 0) := x"100";
  constant TX_DATA_2_ADR  : std_logic_vector(11 downto 0) := x"101";
  constant TX_DATA_3_ADR  : std_logic_vector(11 downto 0) := x"102";
  constant TX_DATA_4_ADR  : std_logic_vector(11 downto 0) := x"103";
  constant TX_DATA_5_ADR  : std_logic_vector(11 downto 0) := x"104";
  constant TX_DATA_6_ADR  : std_logic_vector(11 downto 0) := x"105";
  constant TX_DATA_7_ADR  : std_logic_vector(11 downto 0) := x"106";
  constant TX_DATA_8_ADR  : std_logic_vector(11 downto 0) := x"107";
  constant TX_DATA_9_ADR  : std_logic_vector(11 downto 0) := x"108";
  constant TX_DATA_10_ADR : std_logic_vector(11 downto 0) := x"109";
  constant TX_DATA_11_ADR : std_logic_vector(11 downto 0) := x"10A";
  constant TX_DATA_12_ADR : std_logic_vector(11 downto 0) := x"10B";
  constant TX_DATA_13_ADR : std_logic_vector(11 downto 0) := x"10C";
  constant TX_DATA_14_ADR : std_logic_vector(11 downto 0) := x"10D";
  constant TX_DATA_15_ADR : std_logic_vector(11 downto 0) := x"10E";
  constant TX_DATA_16_ADR : std_logic_vector(11 downto 0) := x"10F";
  constant TX_DATA_17_ADR : std_logic_vector(11 downto 0) := x"110";
  constant TX_DATA_18_ADR : std_logic_vector(11 downto 0) := x"111";
  constant TX_DATA_19_ADR : std_logic_vector(11 downto 0) := x"112";
  constant TX_DATA_20_ADR : std_logic_vector(11 downto 0) := x"113";


  ----------------
  --Register Bits-
  ----------------
  --Status register
  constant BS_IND  : natural := 23;      -- Bus status
  constant ES_IND  : natural := 22;      -- Error Status
  constant TS_IND  : natural := 21;      -- Transmit Status
  constant RS_IND  : natural := 20;      -- Recieve status
  constant ET_IND  : natural := 19;      -- Transmit Buffer Status
  constant TBS_IND : natural := 18;      -- Transmit Buffer Status
  constant DOS_IND : natural := 17;      -- Data OverRun Status
  constant RBS_IND : natural := 16;      -- Recieve Buffer Status

  --Mode register
  constant FDE_IND      : natural := 4;  -- Flexible datarate enable
  constant RTR_PREF_IND : natural := 5;  -- RTR Preffered behaviour
  constant AFM_IND      : natural := 3;  -- Acceptance filters mode (enabled)
  constant LOM_IND      : natural := 1;
  constant STM_IND      : natural := 2;
  constant TSM_IND      : natural := 6;
  constant RST_IND      : natural := 0;

  --Interrupt enable indices
  constant RI_IND  : natural := 0;
  constant TI_IND  : natural := 1;
  constant EI_IND  : natural := 2;
  constant DOI_IND : natural := 3;
  constant EPI_IND : natural := 5;
  constant ALI_IND : natural := 6;
  constant BEI_IND : natural := 7;
  constant LFI_IND : natural := 8;
  constant RFI_IND : natural := 9;
  constant BSI_IND : natural := 10;

  ----------------------------------------------  
  --Logger event types 
  ----------------------------------------------
  constant SOF_EVNT  : std_logic_vector(7 downto 0) := x"01";
  constant ALO_EVNT  : std_logic_vector(7 downto 0) := x"02";
  constant REC_EVNT  : std_logic_vector(7 downto 0) := x"03";
  constant TRAN_EVNT : std_logic_vector(7 downto 0) := x"04";
  constant OVLD_EVNT : std_logic_vector(7 downto 0) := x"05";
  constant ERR_EVNT  : std_logic_vector(7 downto 0) := x"06";
  constant BRS_EVNT  : std_logic_vector(7 downto 0) := x"07";
  constant ARB_EVNT  : std_logic_vector(7 downto 0) := x"08";
  constant CTRL_EVNT : std_logic_vector(7 downto 0) := x"09";
  constant DATA_EVNT : std_logic_vector(7 downto 0) := x"0A";
  constant CRC_EVNT  : std_logic_vector(7 downto 0) := x"0B";
  constant ACK_EVNT  : std_logic_vector(7 downto 0) := x"0C";
  constant NACK_EVNT : std_logic_vector(7 downto 0) := x"0D";
  constant EWL_EVNT  : std_logic_vector(7 downto 0) := x"0E";
  constant ERP_EVNT  : std_logic_vector(7 downto 0) := x"0F";
  constant TXS_EVNT  : std_logic_vector(7 downto 0) := x"10";
  constant RXS_EVNT  : std_logic_vector(7 downto 0) := x"11";
  constant SYNC_EVNT : std_logic_vector(7 downto 0) := x"12";
  constant STUF_EVNT : std_logic_vector(7 downto 0) := x"13";
  constant DSTF_EVNT : std_logic_vector(7 downto 0) := x"14";
  constant OVR_EVNT  : std_logic_vector(7 downto 0) := x"15";
  -----------------------------------------------------
  --constant ERR_EVNT:std_logic_vector(7 downto 0)  := x"01"; 
  -----------------------------------------------------
  
end package;
