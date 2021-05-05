--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2021-present Ondrej Ille
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- educational, research, evaluation, self-interest purposes. Using the
-- Component for commercial purposes is forbidden unless previously agreed with
-- Copyright holder.
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
-- -------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2015-2020 MIT License
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
-- Module:
--  DLC Decoder
-- 
-- Purpose:
--  Decode DLC to byte length of Data field in CAN Frame. Support both CAN 2.0
--  and CAN FD. Output signal 'is_valid' shows if the DLC value is meaningful 
--  for given frame type, ie. values greater than '1000' are also valid for 
--  CAN 2.0 but the value is marked in any case as 8 bytes. Decoder simply 
--  returns '0' if frame type is 'CAN_2_0' and DLC is greater than 8.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity dlc_decoder is
    port (
        -- DLC Input (as in CAN Standard)
        dlc              :   in std_logic_vector(3 downto 0);
        
        -- Frame Type (0 - CAN 2.0, 1 - CAN FD)
        frame_type       :   in std_logic;

        -- Data length (decoded)
        data_length      :   out std_logic_vector(6 downto 0);
        
        -- Validity indication (0 for CAN 2.0 frames with dlc > 0) 
        is_valid         :   out std_logic
    );
end dlc_decoder;

architecture rtl of dlc_decoder is

    signal data_len_8_to_64         :   std_logic_vector(6 downto 0);
    signal data_len_8_to_64_integer :   natural range 0 to 64;
    
    -- Data length fot standard CAN 2.0 frame
    signal data_len_can_2_0     :   std_logic_vector(6 downto 0);       
    
    -- Data length fot standard CAN FD frame
    signal data_len_can_fd      :   std_logic_vector(6 downto 0);

    signal dlc_int              :   natural range 0 to 64;

begin
    
    -- Typecast to natural
    dlc_int <= to_integer(unsigned(dlc));

    -- Decoder for DLCs higher than 8 in CAN FD Frame
    data_len_8_to_64_integer <=
        12 when (dlc = "1001") else
        16 when (dlc = "1010") else
        20 when (dlc = "1011") else
        24 when (dlc = "1100") else
        32 when (dlc = "1101") else
        48 when (dlc = "1110") else
        64 when (dlc = "1111") else
        0;

    -- Typecast byte length in CAN FD frame to vector
    data_len_8_to_64 <= std_logic_vector(to_unsigned(data_len_8_to_64_integer, 7));


    -- Mux for CAN 2.0 DLC:
    --  1. Take DLC itself for values less or equal than 8
    --  2. Hard-code 8 (all DLCs above 8 mean 8 bytes in spec.)
    data_len_can_2_0 <= ("000" & dlc) when (dlc_int <= 8) else
                        "0001000"; 
              
    -- Mux for CAN FD DLC:
    --  1. Take DLC itself for values less or equal than 8 
    --  2. Use decoder above for values higher than 8.                   
    data_len_can_fd <= ("000" & dlc) when (dlc_int <= 8) else
                       data_len_8_to_64;

    -- According the CAN frame type, select output vector
    data_length <= data_len_can_2_0 when (frame_type = NORMAL_CAN) else
                   data_len_can_fd;
                   
    ---------------------------------------------------------------------------
    -- DLC is valid:
    --  1. in every case for CAN FD
    --  2. only for values <= 8 for CAN 2.0                        
    ---------------------------------------------------------------------------
    is_valid <= '1' when ((dlc_int <= 8) and (frame_type = NORMAL_CAN)) else
                '1' when (frame_type = FD_CAN) else  
                '0';             
end rtl;
