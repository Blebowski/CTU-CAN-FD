--------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy 
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is 
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in 
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
-- IN THE SOFTWARE.
--
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN 
-- protocol license from Bosch.
--
--------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------
--Serial bit Stuffer with generic number of stuff length (number of maximum following bits of same value). Con----
--tains internal buffer for data. Circuit is started by '1' at start input and valid data size to be stuffed, ----
--and first bit of data on data_in input. Data are propagated to the data_out and if bit is stuffed input data ---
--are stored in internal buffer. Buffer size is calculated in worst possible case (most stuff bits) from maximum--
--data length. Transmission stops by '1' at stop input. If transmision finishes finished output becomes '1'. -----
--Fixed stuff input controls whenever fixed bitStuffing(after length bits of any value reverse bit is inserted) --
--'1' fixed stuff is used, '0' fixed stuff is not used. Fixed stuff for CRC of CAN FD Frames ---------------------
------------------------------------------------------------------------------------------------------------------
-- Revision History:
--
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

--NOTE: Circuit is deprecated, simpler implementation without interal buffer was used!

entity bitStuffer is
  --number of equal bits before sutff bit is inserted
  generic (
    length:natural:=5;
    maxDataSize:natural:=512 --Maximal number of consecutive bits sent
  );
  port(
    --INPUTS
    signal data_in : in std_logic;  --serial data input 
    signal clk_sys : in std_logic; --System clock
    signal clk_nbt : in std_logic; --Nominal bit time clock
    signal clk_dbt : in std_logic; --Data bit time clock
    signal start : in std_logic; --Logic '1' starts bit stuffing when rising edge on clk_sys and bs_trig_XXX is active
    signal data_size : in std_logic_vector (11 downto 0); --must be valid data size to be transmitted when start='1'
    signal fixed_stuff : in std_logic; --fixed stuff insert stuffed bit after 'length' bits even if they are not the same value
    signal stop : in std_logic; --stops the bit stuffing when 1 is present
    signal res_n : in std_logic; --reset (active in '0')
    signal bt_type : in std_logic; --Bit time type, if either clk_nbt or clk_dbt is considered as bit time for transcieving
    signal bs_trig_nbt : in std_logic; --Trigger signal from prescaler to sample inpu value and propagate or bit stuff
    signal bs_trig_dbt : in std_logic; --Trigger signal from prescaler to sample inpu value and propagate or bit stuff
    
    --OUTPUTS
    signal data_out : out std_logic; --serial data output
    signal finished : out std_logic; --signalizing all data are stuffed
    signal valid : out std_logic; --data_out is valid (not valid when stuffing finished,stopped or error)
    signal error : out std_logic; --error occurs when input dataSize is bigger than data_buffer size
    signal data_rem : out std_logic_vector (11 downto 0); --amount of bits remaining to be transmitted
    signal is_stuffed : out std_logic --logic 1 if actual bit is stuffed;
  );
  --STATE MACHINE OF BIT STUFFER
  type stuffing_state is (waiting_s,running_s,stopped_s,finished_s,error_s);
  signal state:stuffing_state:=waiting_s;
  --DATA BUFFER
  signal data_buffer:std_logic_vector(maxDataSize/length downto 0); --maxDataSize/length the worst case, most stuffed bits 
  --Pointer for position in data_buffer where to add incoming data
  signal buffer_pointer:natural:=0;
  --Number of following bits that is already with the same value
  signal same_bits:natural := 0;
  --Value of last bit
  signal last_bit:std_logic :='0';
  --Remaining data counter
  signal data_remaining : std_logic_vector (11 downto 0); --amount of bits remaining to be transmitted
end entity;

architecture rtl of bitStuffer is
 begin
  data_rem<=data_remaining;
  state_machine:process(clk_sys)
  variable data_buffer_var:std_logic_vector(maxDataSize/length downto 0);
  begin
    if(res_n='0')then
      state<=waiting_s;
    elsif rising_edge(clk_sys) then    
      --State machine Handling
      case state is
        when waiting_s => 
          if(start='1')and
            ((bs_trig_nbt='1' and bt_type='0')or
              (bs_trig_dbt='1' and bt_type='1'))then
            if(data_size>std_logic_vector(to_unsigned(maxDataSize,11)))then
              state<=error_s;
            else
             finished<='0';
             error<='0';
             valid<='1';
             data_remaining<=data_size;
             same_bits<=1;
             data_buffer<=(OTHERS =>'0');
             data_out<=data_in; --First bit is sent directly
             last_bit<=data_in;        
             buffer_pointer<=0;
             state<=running_s;
            end if;     
          end if;           
        when running_s => 
           if(bs_trig_nbt='1' and bt_type='0')or
              (bs_trig_dbt='1' and bt_type='1')then
            --Last bit assignment
            if(buffer_pointer=0)then
              last_bit<=data_in;
            else
              last_bit<=data_buffer(0);
            end if;
           
            --Incresing number of same bits
            --and nulling number of same bits when other bit appears!
            --If stuffing just began ,no data in buffer, last_bit is
            --compared with data_in, otherwise it is compared with
            --actual transcieved data!
            if(buffer_pointer=0)then
              --If no data are stored in buffer
              if((last_bit=data_in) OR (fixed_stuff='1'))then
                same_bits<=same_bits+1;
              end if;
              if((fixed_stuff='0') AND (NOT(last_bit=data_in)))then 
                same_bits<=1;
              end if;
             else
              --If data are stored in Buffer
              if((last_bit=data_buffer(0)) OR (fixed_stuff='1'))then
                same_bits<=same_bits+1;
              end if;    
              if((fixed_stuff='0') AND (NOT(last_bit=data_buffer(0))))then 
                same_bits<=1;
              end if;
             end if;
            
            --Number of equal bits reached the length
            if(same_bits=length)then
              data_buffer(buffer_pointer)<=data_in;
              buffer_pointer<=buffer_pointer+1;
              data_out<=not last_bit;
              same_bits<=0;
            else
              --decreasing remaining data
              data_remaining<=std_logic_vector(unsigned(data_remaining)-1);
              valid<='1';
              if(buffer_pointer=0)then
                data_out<=data_in;
              else
                --Shifting data buffer and storing input  data into this position
                data_buffer_var:=data_buffer;
                data_buffer_var(buffer_pointer):=data_in;
                data_buffer_var:='0'&data_buffer_var(maxDataSize/length downto 1); 
                data_buffer<=data_buffer_var;
                data_out<=data_buffer(0); 
              end if;
            end if;
          
            --All the data are sent go to finished state
            if(unsigned(data_remaining)=1)then
              state<=finished_s;
              finished<='1';
              valid<='0';
            end if;
          
            --If stop is active go to stop state
            if(stop='1')then
              state<=stopped_s;
            end if;                  
        end if;
                                       
        when stopped_s =>
          valid<='0';
          state<=waiting_s;
        when finished_s =>
          state<=waiting_s;
        when error_s=>
          valid<='0';
          error<='1';
          state<=waiting_s;
      when others =>
     end case;
  end if;
  
  end process;
  
end architecture;