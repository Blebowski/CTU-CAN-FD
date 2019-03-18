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
--  Package with Constants, types and other defintions for CAN FD IP Core.
--------------------------------------------------------------------------------
-- Revision History:
--    June 2015   Created file
--    08.12.2017  Code formatting. Replaced hexadecimal values with shorter
--                notation
--    19.12.2017  Moved TX_DATA registers into separate memory location
--                TX_DATA_REGION to make the direct addressing inside the TXT
--                buffer easier.
--    27.12.2017  Added "DRV_FRAME_SWAP_INDEX" into driving bus.
--    20.1.2018   Removed CAN Frame constants FRAME_BASIC and FRAME_EXTENDED
--                Properly only signals from CAN_FD_frame_format package
--                BASE and EXTENDED should be used!
--    28.12.2018  Separated "can_types", "drv_stat_pkg".
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

package can_constants is

    -- IP Core version related constants
    constant CTU_CAN_FD_VERSION_MINOR : std_logic_vector(7 downto 0) := x"01";
    constant CTU_CAN_FD_VERSION_MAJOR : std_logic_vector(7 downto 0) := x"02";

    -- Active value of asynchronous reset 
    constant ACT_RESET : std_logic := '0';

    --Definition of basic logic levels for CAN bus
    constant DOMINANT  : std_logic := '0';
    constant RECESSIVE : std_logic := '1';

    constant INTEGRATING_DURATION : natural := 11;
    constant TRAN_BUFF_SIZE       : natural := 600;

    constant BASE_STUFF_LENGTH : natural := 5;
    constant FD_STUFF_LENGTH   : natural := 4;

    constant CAN_BASE_ID_LENGTH : natural := 11;
    constant CAN_EXT_ID_LENGTH  : natural := 18;

    constant NO_SYNC   : std_logic_vector(1 downto 0) := "00";
    constant HARD_SYNC : std_logic_vector(1 downto 0) := "01";
    constant RE_SYNC   : std_logic_vector(1 downto 0) := "10";

    -- CRC sources
    constant CRC_15_SRC : std_logic_vector(1 downto 0) := "00";
    constant CRC_17_SRC : std_logic_vector(1 downto 0) := "01";
    constant CRC_21_SRC : std_logic_vector(1 downto 0) := "10";

    -- Sample point control constants
    constant NOMINAL_SAMPLE   : std_logic_vector(1 downto 0) := "00";
    constant DATA_SAMPLE      : std_logic_vector(1 downto 0) := "01";
    constant SECONDARY_SAMPLE : std_logic_vector(1 downto 0) := "10";

    -- Tuples definition for older compiler (less than 2008)
    constant DOMINANT_DOMINANT   : std_logic_vector(1 downto 0) := 
                                    DOMINANT & DOMINANT;

    constant DOMINANT_RECESSIVE  : std_logic_vector(1 downto 0) :=
                                    DOMINANT & RECESSIVE;

    constant RECESSIVE_DOMINANT  : std_logic_vector(1 downto 0) :=
                                    RECESSIVE & DOMINANT;

    constant RECESSIVE_RECESSIVE : std_logic_vector(1 downto 0) :=
                                    RECESSIVE & RECESSIVE;

    -- Error flag definitions 
    constant PASSIVE_ERR_FLAG : std_logic := RECESSIVE;
    constant ACTIVE_ERR_FLAG  : std_logic := DOMINANT;

    constant ERROR_FLAG_LENGTH : natural := 6;

    constant INC_ONE_CON   : std_logic_vector(2 downto 0) := "100";
    constant INC_EIGHT_CON : std_logic_vector(2 downto 0) := "010";
    constant DEC_ONE_CON   : std_logic_vector(2 downto 0) := "001";

    -- Common definitions should not be generic at the moment
    constant TXT_BUFFER_COUNT     : natural := 4;

    constant INT_COUNT            : natural := 12;                                    

    constant ZERO      : std_logic := '0';
    constant NO_ACTION : std_logic := '0';

    constant ACK_ALLOWED  : std_logic := '0';
    constant ACK_FORBIDEN : std_logic := '1';

    constant LOOPBACK_ENA : std_logic := '1';
    constant LOOPBACK_DIS : std_logic := '0';

    constant RETR_LIM_DIS : std_logic := '0';
    constant RETR_LIM_ENA : std_logic := '1';

    constant SINGLE_SAMPLING  : std_logic := '0';
    constant TRIPPLE_SAMPLING : std_logic := '1';

    constant ALLOW_BUFFER  : std_logic := '1';
    constant FORBID_BUFFER : std_logic := '0';

    -- Definition of register directions for TXT1 and TXT2 buffers
    constant TXT1_DIR : std_logic := '0';
    constant TXT2_DIR : std_logic := '1';

    -- CRC polynomials
    constant CRC15_POL : std_logic_vector(15 downto 0) := x"C599";
    constant CRC17_POL : std_logic_vector(19 downto 0) := x"3685B";
    constant CRC21_POL : std_logic_vector(23 downto 0) := x"302899";

    ----------------------------------------------------------------------------
    -- Memory Access
    ----------------------------------------------------------------------------
    constant ACT_CSC : std_logic := '1';
    constant ACT_SRD : std_logic := '1';
    constant ACT_SWR : std_logic := '1';

    -- Address ranges for identifier
    constant ID_ADRESS_HIGHER        : natural := 15;
    constant ID_ADRESS_LOWER         : natural := 12;

    constant CAN_DEVICE_ID : std_logic_vector(31 downto 0) := x"0000CAFD";
  
end package;
