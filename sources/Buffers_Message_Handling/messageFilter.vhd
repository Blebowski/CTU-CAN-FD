Library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.ALL;
use ieee.std_logic_unsigned.All;
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
--    17.1.2016   Added ID change from register value to decimal value for range filter comparison
--    1.6.2016    Fixed wrong enable decoding from driving bus signals! Filters were disabled but
--                frame was anyway propagated to the output!
-------------------------------------------------------------------------------------------------------------------------------------------------------


-----------------------------------------------------------------------------------------------------------
-- Purpose:
--  Message filter for recieved data. Combinational circuit with valid data register at output of circuit.-
--  Filter identifier type, frame type are controlled by drv_bus from registers. 13 bit and 29 bit filters-
--  can be compared. If 13 bit filters are compared then MSB 16 bits in rec_ident_in has to be zeros.     -
--  Also mask for the filter in case of 13-bit filter HAS to have 16 uppest bits equal to zero! It is set -
--  by drv_bus signals from control registers. Filters  A,B,C,D are present. If input identifier matches  -
--  at least one it is considered as valid. Frame type (CAN Basic, CAN Extended, CAN FD Basic) are also   -
--  selectable for filtering.                                                                             -
------------------------------------------------------------------------------------------------------------

entity messageFilter is
  PORT(
    ----------
    --INPUTS--
    ----------
    --Clock an reset signals
    signal clk_sys            :in std_logic;                      --System clock
    signal res_n              :in std_logic;                      --Async reset
    
    --Driving signals from CAN Core
    signal rec_ident_in       :in std_logic_vector(28 downto 0);  --Receieved identifier
    signal ident_type         :in std_logic;                      --Input message identifier type 
                                                                  -- (0-BASE Format, 1-Extended Format);
    signal frame_type         :in std_logic;                      --Input frame type (0-Normal CAN, 1- CAN FD) 
    signal rec_ident_valid    :in std_logic;                      --Identifier valid (active log 1)
    
    --Driving bus from registers
    signal drv_bus            :in std_logic_vector(1023 downto 0);
    
    -----------
    --OUTPUTS--
    -----------
    signal out_ident_valid    :out std_logic --Signal whenever identifier matches the filter identifiers
 );
  ---------------------------
  --INTERNAL SIGNAL aliases -
  ---------------------------
  signal drv_filter_A_mask    :std_logic_vector(28 downto 0);   --Filter A bit mask
  signal drv_filter_A_ctrl    :std_logic_vector(3 downto 0);    --Filter A control bits
  signal drv_filter_A_bits    :std_logic_vector(28 downto 0);   --Filter A bits 
  signal int_filter_A_valid   :std_logic;                       --Output of filter A is valid (internal signal)
  
  signal drv_filter_B_mask    :std_logic_vector(28 downto 0);   --Filter B bit mask
  signal drv_filter_B_ctrl    :std_logic_vector(3 downto 0);    --Filter B control bits
  signal drv_filter_B_bits    :std_logic_vector(28 downto 0);   --Filter B bits 
  signal int_filter_B_valid   :std_logic;                       --Output of filter B is valid (internal signal)
  
  signal drv_filter_C_mask    :std_logic_vector(28 downto 0);   --Filter C bit mask
  signal drv_filter_C_ctrl    :std_logic_vector(3 downto 0);    --Filter C control bits
  signal drv_filter_C_bits    :std_logic_vector(28 downto 0);   --Filter C bits 
  signal int_filter_C_valid   :std_logic;                       --Output of filter C is valid (internal signal)
   
  signal drv_filter_ran_ctrl  :std_logic_vector(3 downto 0);    --Range filter control bits
  signal drv_filter_ran_lo_th :std_logic_vector(28 downto 0);   --Lower range filter trehsold
  signal drv_filter_ran_hi_th :std_logic_vector(28 downto 0);   --Upper range filter trehsold
  signal int_filter_ran_valid :std_logic;                       --Output of range filter is valid
  
  signal drv_filters_ena      :std_logic;
  
  --Internal aliases for input frame type
  signal int_data_type        :std_logic_vector(3 downto 0);    --Frame type on input to be compared with driving signal
  signal int_data_ctrl        :std_logic_vector(1 downto 0);    --Concat of types of data on input
 
  signal rec_ident_dec        :natural;                         --Actual decimal value of recieved 
  
  --Decimal values of identifiers
  signal id_1_dec             :natural;
  signal id_2_dec             :natural;
 
  -------------
  --REGISTERS--
  -------------
  signal valid_reg:std_logic;                                  --Register for valid output value
 
end entity;
  
architecture rtl of messageFilter is
begin
    --Driving signal aliases
    drv_filter_A_mask           <= drv_bus(DRV_FILTER_A_MASK_HIGH downto DRV_FILTER_A_MASK_LOW);
    drv_filter_A_ctrl           <= drv_bus(DRV_FILTER_A_CTRL_HIGH downto DRV_FILTER_A_CTRL_LOW);
    drv_filter_A_bits           <= drv_bus(DRV_FILTER_A_BITS_HIGH downto DRV_FILTER_A_BITS_LOW);
    drv_filter_B_mask           <= drv_bus(DRV_FILTER_B_MASK_HIGH downto DRV_FILTER_B_MASK_LOW);
    drv_filter_B_ctrl           <= drv_bus(DRV_FILTER_B_CTRL_HIGH downto DRV_FILTER_B_CTRL_LOW);
    drv_filter_B_bits           <= drv_bus(DRV_FILTER_B_BITS_HIGH downto DRV_FILTER_B_BITS_LOW);
    drv_filter_C_mask           <= drv_bus(DRV_FILTER_C_MASK_HIGH downto DRV_FILTER_C_MASK_LOW);
    drv_filter_C_ctrl           <= drv_bus(DRV_FILTER_C_CTRL_HIGH downto DRV_FILTER_C_CTRL_LOW);
    drv_filter_C_bits           <= drv_bus(DRV_FILTER_C_BITS_HIGH downto DRV_FILTER_C_BITS_LOW);
    drv_filter_ran_ctrl         <= drv_bus(DRV_FILTER_RAN_CTRL_HIGH downto DRV_FILTER_RAN_CTRL_LOW);
    drv_filter_ran_lo_th        <= drv_bus(DRV_FILTER_RAN_LO_TH_HIGH downto DRV_FILTER_RAN_LO_TH_LOW);
    drv_filter_ran_hi_th        <= drv_bus(DRV_FILTER_RAN_HI_TH_HIGH downto DRV_FILTER_RAN_HI_TH_LOW);
    drv_filters_ena             <= drv_bus(DRV_FILTERS_ENA_INDEX);
    
    --Input frame type internal signal
    int_data_ctrl               <= frame_type&ident_type;
    
    --Decoder frame_type&ident_type to one-hot 
    with int_data_ctrl select int_data_type <=
        "0001" when "00", --CAN Basic
        "0010" when "01", --CAN Extended
        "0100" when "10", --CAN FD Basic
        "1000" when "11", --CAN Fd Extended
        "0000" when others;
   
   --Filter A input frame type filtering 
   int_filter_A_valid <= '1' when ( ( --Identifier matches the bits and mask
                                      (rec_ident_in AND drv_filter_A_mask)  =
                                      (drv_filter_A_bits AND drv_filter_A_mask)                 
                                    ) 
                                    AND
                                    ( --Frame type Matches defined frame type
                                      not((drv_filter_A_ctrl AND int_data_type)="0000")         
                                    )  
                                  )                 
                             else '0'; 
  
   --Filter B input frame type filtering 
   int_filter_B_valid <= '1' when ( ( --Identifier matches the bits and mask
                                      (rec_ident_in AND drv_filter_B_mask)  =  
                                      (drv_filter_B_bits AND drv_filter_B_mask)                 
                                    ) 
                                    AND 
                                    ( --Frame type Matches defined frame type
                                      not((drv_filter_B_ctrl AND int_data_type)="0000")         
                                    )
                                  )
                             else '0'; 
  
  --Filter C input frame type filtering 
   int_filter_C_valid <= '1' when ( ( --Identifier matches the bits and mask
                                      (rec_ident_in AND drv_filter_C_mask)  =
                                      (drv_filter_C_bits AND drv_filter_C_mask)                 
                                    ) 
                                    AND 
                                    ( --Frame type Matches defined frame type
                                      not((drv_filter_C_ctrl AND int_data_type)="0000")         
                                    )
                                  )
                          else '0';
                           
  --Range filter for identifiers
  ID_reg_to_decimal(rec_ident_in,rec_ident_dec);
  int_filter_ran_valid  <= '1' when ( --Identifier matches the range set
                                      ( rec_ident_dec<=to_integer(unsigned(drv_filter_ran_hi_th)) )
                                      AND
                                      ( rec_ident_dec>=to_integer(unsigned(drv_filter_ran_lo_th)) )   
                                    )
                                    AND 
                                    ( --Frame type Matches defined frame type
                                      not((drv_filter_ran_ctrl AND int_data_type)="0000")             
                                    )
                          else '0';
  
            
  --If received message is valid and at least one of
  -- the filters is matching the message passed the
  -- filter.
  valid_reg             <=  rec_ident_valid        AND 
                            (
                              int_filter_A_valid   OR 
                              int_filter_B_valid   OR 
                              int_filter_C_valid   OR 
                              int_filter_ran_valid 
                            ) when drv_filters_ena='1'
                            else rec_ident_valid;
                            
  
  
  ---------------------------------------------------
  --To avoid long combinational paths, valid filter
  -- output is pipelined. This is OK since received
  -- frame is valid on input for many clock cycles!
  ---------------------------------------------------
  valid_reg_proc:process(res_n,clk_sys)
  begin
    if(res_n=ACT_RESET)then
      out_ident_valid   <= '0';
    elsif rising_edge(clk_sys) then
      out_ident_valid   <= valid_reg;
    end if;
  end process valid_reg_proc;
  
end architecture;