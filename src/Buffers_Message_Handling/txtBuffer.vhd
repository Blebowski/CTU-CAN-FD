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
--  Transmit message buffer. Buffer is accessed via "tran_data" and "tran_addr"
--  signals from user registers. Buffer contains simple FSM for manipulation
--  from HW (HW Commands) as well as SW (SW Commands). Buffer is split into
--  "data" part which is implemented to be inferred in dual port RAM block,
--  and "metadata" part which is available in paralell on the output. "data"
--  part is addressed by second port from Protocol control.
--------------------------------------------------------------------------------
-- Revision History:
--
--    July 2015   Created file
--    30.11.2017  Changed the buffer implementation from parallel into 32*20 
--                buffer of data. Reading so far left parallel. User is directly
--                accessing the buffer and storing the data to it.
--    04.12.2017  Buffer split to "Frame metadata" (txt_buffer_info) and "Data" 
--                (txt_buffer_data). Frame metadata consists of first 4 words 
--                (Frame format, Timestamps and Identifier). Frame metadata are
--                available combinationally at all times. Frame data are accessed
--                directly from CAN Core by new pointer "txt_data_addr". 
--                txt_buffer_data is synthesized as RAM memory and significant
--                reource reduction was achieved.
--     15.2.2018  Implemented TXT Buffer state machine. Replaced "empty", "ack"
--                and "allow" signals with HW commands from Protocol Control and
--                SW commands from User registers. Hardware commands have always
--                higher priority.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;

entity txtBuffer is
  generic(
    constant buf_count            :     natural range 1 to 8;
    constant ID                   :     natural :=1
  );
  PORT(
    ------------------
    --Clock and reset-
    ------------------
    signal clk_sys                :in   std_logic;
    signal res_n                  :in   std_logic; --Async reset
    
    -------------------------------
    --Driving Registers Interface--
    -------------------------------
    
    -- Driving bus
    signal drv_bus                :in   std_logic_vector(1023 downto 0);
    
    -- Data and address for SW access into the RAM of TXT Buffer
    signal tran_data              :in   std_logic_vector(31 downto 0);
    signal tran_addr              :in   std_logic_vector(4 downto 0);
    
    -- SW commands from user registers
    signal txt_sw_cmd             :in   txt_sw_cmd_type;
    signal txt_sw_buf_cmd_index   :in   std_logic_vector(
                                          buf_count - 1 downto 0);
  
    ------------------     
    --Status signals--
    ------------------
    signal txtb_state             :out  txt_fsm_type;
    
    ------------------------------------
    --CAN Core and TX Arbiter Interface-
    ------------------------------------
    
    -- Commands from the CAN Core for manipulation of the CAN 
    signal txt_hw_cmd             :in   txt_hw_cmd_type;  
    signal txt_hw_cmd_buf_index   :in   natural range 0 to buf_count - 1;
  
    -- Data of the frame to be transmitted and pointer to the RAM memory
    -- of TXT buffer from Protocol control
    signal txt_data_word          :out  std_logic_vector(31 downto 0);
    signal txt_data_addr          :in   natural range 0 to 15;
    
    --First 4 words (frame format, timestamps, identifier) are available 
    --combinationally, to be able instantly decide on higher priority frame
    signal txt_frame_info_out     :out  std_logic_vector(639 downto 512);
    
    -- Signals to the TX Arbitrator that it can be selected for transmission
    -- (used as input to priority decoder)
    signal txt_buf_ready          :out  std_logic
    );
             
end entity;


architecture rtl of txtBuffer is

  ----------------------
  --Internal registers--
  ----------------------
  type frame_data_memory is array(0 to 15) of std_logic_vector(31 downto 0);
  type frame_info_memory is array (0 to 3) of std_logic_vector(31 downto 0);

  ------------------
  --Signal aliases--
  ------------------
  
  -- Time transcieve buffer - Data memory
  signal txt_buffer_data        : frame_data_memory;
  
  -- Frame format, Timestamps and Identifier
  signal txt_buffer_meta_data   : frame_info_memory;
   
  -- Store into TXT buffer 1 or 2 (chip select)
  signal tran_wr                : std_logic_vector(1 downto 0);
  
  -- FSM state of the buffer
  signal buf_fsm                : txt_fsm_type;
  
  -- Internal buffer selects for commands. Commands are shared across the
  -- buffers so we need unique identifier
  signal hw_cbs                 : std_logic;
  signal sw_cbs                 : std_logic;
  
begin
    
    -- Write signals for buffer
    tran_wr             <= drv_bus(DRV_TXT2_WR)&drv_bus(DRV_TXT1_WR);
    
    -- Output data are given by the address from the Core
    txt_data_word       <= txt_buffer_data(txt_data_addr);
    
    -- First 4 words of the Frame are available constantly...
    txt_frame_info_out  <= txt_buffer_meta_data(0)&
													 txt_buffer_meta_data(1)&
													 txt_buffer_meta_data(3)&
													 txt_buffer_meta_data(2);
    
    -- Buffer is ready for selection by TX Arbitrator only in state "Ready"
    txt_buf_ready       <= '1' when buf_fsm = txt_ready
                                else
                           '0';
                               
    
    -- Command buffer select signals
    hw_cbs <= '1' when txt_hw_cmd_buf_index = ID
                  else
              '0';
  
    sw_cbs <= '1' when txt_sw_buf_cmd_index(ID) = '1' 
                  else
              '0';
    
    ----------------------------------------------------------------------------
    -- Buffer access process from SW
    ----------------------------------------------------------------------------
    tx_buf_access_proc:process(res_n,clk_sys)
    begin
      if (res_n = ACT_RESET) then
        
          -- synthesis translate_off
          txt_buffer_data <= (OTHERS => (OTHERS => '0'));
          -- synthesis translate_on
        
          -- Frame info is stored in registers
          txt_buffer_meta_data <= (OTHERS => (OTHERS => '0'));
          
      elsif (rising_edge(clk_sys))then
        
        --Store the data into the Buffer during the access
        if (tran_wr(ID - 1) = '1') then
          if (to_integer(unsigned(tran_addr)) < 4) then
            txt_buffer_meta_data(to_integer(unsigned(tran_addr))) <= tran_data;
          else  
            txt_buffer_data(to_integer(unsigned(tran_addr)) - 4)  <= tran_data;
          end if;
        end if;
        
      end if;
    end process;
    
    
    ----------------------------------------------------------------------------
    -- Buffer FSM process
    ----------------------------------------------------------------------------
    tx_buf_fsm_proc:process(res_n,clk_sys)
    begin
      if (res_n = ACT_RESET) then
          buf_fsm         <= txt_empty;
      elsif (rising_edge(clk_sys))then
        
        buf_fsm           <= buf_fsm;
        
        case buf_fsm is
        
        ------------------------------------------------------------------------
        -- Buffer is empty
        ------------------------------------------------------------------------
        when txt_empty =>

          -- "Set_ready"
          if (txt_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
            buf_fsm       <= txt_ready;
          end if;


        ------------------------------------------------------------------------
        -- Buffer is ready for transmission
        ------------------------------------------------------------------------
        when txt_ready =>
          
          -- Locking for transmission
          if (txt_hw_cmd.lock = '1' and hw_cbs = '1') then
            
            -- Simultaneous "lock" and abort -> transmit, but
            -- with abort pending
            if (txt_sw_cmd.set_abt = '1' and sw_cbs = '1') then
              buf_fsm     <= txt_ab_prog;
            else
              buf_fsm     <= txt_tx_prog;
            end if;
          
          -- Abort the ready buffer
          elsif (txt_sw_cmd.set_abt = '1' and sw_cbs = '1') then
            buf_fsm       <= txt_aborted;
          else
            buf_fsm       <= buf_fsm;
          end if;


        ------------------------------------------------------------------------
        -- Transmission from buffer is in progress
        ------------------------------------------------------------------------          
        when txt_tx_prog =>
          
          -- Unlock the buffer
          if (txt_hw_cmd.unlock = '1' and hw_cbs = '1') then
            
            -- Retransmitt reached, transmitt OK, or try again...
            if (txt_hw_cmd.failed         = '1') then
              buf_fsm     <= txt_error;
            elsif (txt_hw_cmd.valid       = '1') then
              buf_fsm     <= txt_ok;
            elsif (txt_hw_cmd.err         = '1' or 
                   txt_hw_cmd.arbl        = '1') then
              buf_fsm     <= txt_ready;
            else
              buf_fsm     <= buf_fsm;
            end if;
          
          -- Request abort during transmission
          elsif (txt_sw_cmd.set_abt = '1' and sw_cbs = '1') then 
            buf_fsm       <= txt_ab_prog;
          else
            buf_fsm       <= buf_fsm;  
          end if;


        ------------------------------------------------------------------------
        -- Transmission is in progress -> abort at nearest error!
        ------------------------------------------------------------------------
        when txt_ab_prog =>
          
          -- Unlock the buffer
          if (txt_hw_cmd.unlock = '1' and hw_cbs = '1') then
            
            -- Retransmitt reached, transmitt OK, or try again... 
            if (txt_hw_cmd.failed         = '1') then
              buf_fsm     <= txt_error;
            elsif (txt_hw_cmd.valid       = '1') then
              buf_fsm     <= txt_ok;
            elsif (txt_hw_cmd.err         = '1' or 
                   txt_hw_cmd.arbl        = '1') then
              buf_fsm     <= txt_aborted;
            else
              buf_fsm     <= buf_fsm;
            end if;
            
          else
            buf_fsm       <= buf_fsm;
          end if;
        
        
        ------------------------------------------------------------------------
        -- Transmission from buffer failed. Retransmitt limit was reached.
        ------------------------------------------------------------------------
        when txt_error =>

          -- "Set_ready"
          if (txt_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
            buf_fsm       <= txt_ready;
          end if;
          
          -- "Set_empty"
          if (txt_sw_cmd.set_ety = '1' and sw_cbs = '1') then
            buf_fsm       <= txt_empty;
          end if;


        ------------------------------------------------------------------------
        -- Transmission was aborted by user command
        ------------------------------------------------------------------------
        when txt_aborted =>
          
          -- "Set_ready"
          if (txt_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
            buf_fsm       <= txt_ready;
          end if;
          
          -- "Set_empty"
          if (txt_sw_cmd.set_ety = '1' and sw_cbs = '1') then
            buf_fsm       <= txt_empty;
          end if;
    
    
        ------------------------------------------------------------------------
        -- Transmission was succesfull
        ------------------------------------------------------------------------
        when txt_ok =>
          
          -- "Set_ready"
          if (txt_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
            buf_fsm       <= txt_ready;
          end if;
          
          -- "Set_empty"
          if (txt_sw_cmd.set_ety = '1' and sw_cbs = '1') then
            buf_fsm       <= txt_empty;
          end if;
          
        end case;
         
      end if;
    end process;
  
end architecture;
