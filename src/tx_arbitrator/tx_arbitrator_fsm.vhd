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
--  TX arbitrator FSM.
-- 
-- Purpose:
--  Controls selection of CAN frame, comparison of its timestamp and Loading
--  CAN frame metadata to output. When selected TXT Buffer is changed, selection
--  process is restarted.  When LOCK command from CAN Core is issued, locks TXT
--  Buffer for transmission.
--  Each FSM state lasts two clock cycles (wait state is inserted), to give
--  time to TXT Buffer RAM for data read.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity tx_arbitrator_fsm is
    port( 
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                :in  std_logic;
        
        -- Asynchronous reset
        res_n                  :in  std_logic;
        
        -----------------------------------------------------------------------
        -- Priority decoder interface
        -----------------------------------------------------------------------
        -- TXT Buffer is valid and selected for transmission
        select_buf_avail       :in  std_logic;
        
        -- Priority decoder output has changed. TXT Arbitrator FSM has to restart
        -- selection process.
        select_index_changed   :in  std_logic;
        
        -----------------------------------------------------------------------
        -- Timestamp comparison interface
        -----------------------------------------------------------------------
        timestamp_valid        :in  std_logic;
        
        -----------------------------------------------------------------------
        -- CAN Core Interface
        -----------------------------------------------------------------------
        -- HW Commands from CAN Core for manipulation with TXT Buffers 
        txtb_hw_cmd             :in t_txtb_hw_cmd;  
        
        ---------------------------------------------------------------------------
        -- TX Arbitrator FSM outputs
        ---------------------------------------------------------------------------
        -- Load Timestamp lower word to metadata pointer
        load_ts_lw_addr        :out std_logic;
        
        -- Load Timestamp upper word to metadata pointer
        load_ts_uw_addr        :out std_logic;
        
        -- Load Frame format word to metadata pointer
        load_ffmt_w_addr       :out std_logic;

        -- Load identifier word to metadata pointer
        load_ident_w_addr      :out std_logic;

        -- Clock enable for TXT Buffer RAM
        txtb_meta_clk_en       :out std_logic;

        -- Store timestamp lower word
        store_ts_l_w           :out std_logic;
        
        -- Store metadata (Frame format word) on the output of TX Arbitrator
        store_md_w             :out std_logic;
        
        -- Store identifier (Identifier word) on the output of TX Arbitrator
        store_ident_w          :out std_logic;

        -- Store metadata (Frame format word) to double buffer registers.
        buffer_md_w            :out std_logic; 
        
        -- Signals that TX Arbitrator is locked (CAN Core is transmitting from TXT
        -- Buffer)
        tx_arb_locked          :out std_logic;
        
        -- Store last locked TXT Buffer index
        store_last_txtb_index  :out std_logic;
        
        -- Set valid selected buffer on TX Arbitrator output.
        frame_valid_com_set    :out std_logic;    
        
        -- Clear valid selected buffer on TX Arbitrator output.
        frame_valid_com_clear  :out std_logic 
    );
end entity;

architecture rtl of tx_arbitrator_fsm is

  -- TX Arbitrator FSM state
  signal curr_state               : t_tx_arb_state;
  signal next_state               : t_tx_arb_state;
  signal tx_arb_fsm_ce            : std_logic;
 
  -- Wait state DFF
  signal fsm_wait_state_d         : std_logic;
  signal fsm_wait_state_q         : std_logic;

begin

    ----------------------------------------------------------------------------
    -- Next state process
    ----------------------------------------------------------------------------
    tx_arb_fsm_proc : process(curr_state, txtb_hw_cmd, select_buf_avail, 
        select_index_changed, timestamp_valid, fsm_wait_state_q)
    begin
        -- Keeping signals values to avoid latch inference
        next_state                <= curr_state;
        
        case curr_state is   
        
        --------------------------------------------------------------------
        -- No TXT Buffer is available, or was just unlocked after
        -- transmission
        --------------------------------------------------------------------
        when s_arb_idle =>
            if (select_buf_avail = '1') then
            next_state <= s_arb_sel_low_ts;
        end if;
        
        --------------------------------------------------------------------
        -- Read Low timestamp word of Selected TXT buffer.
        --------------------------------------------------------------------
        when s_arb_sel_low_ts =>
            if (txtb_hw_cmd.lock = '1') then
                next_state         <= s_arb_locked;
            elsif (select_buf_avail = '0') then
                next_state         <= s_arb_idle;
            elsif (select_index_changed = '1') then
                next_state         <= s_arb_sel_low_ts;
            elsif (fsm_wait_state_q = '0') then
                next_state         <= s_arb_sel_upp_ts;
            end if;
        
        --------------------------------------------------------------------
        -- Read Upper timestamp word of Selected TXT Buffer. Perform
        -- Timestamp comparison (Lower word is in capture register).
        -- When Timestamp elapses -> proceed with validation!
        --------------------------------------------------------------------  
        when s_arb_sel_upp_ts =>
            if (txtb_hw_cmd.lock = '1') then
                next_state         <= s_arb_locked;
            elsif (select_buf_avail = '0') then
                next_state         <= s_arb_idle;
            elsif (select_index_changed = '1') then
                next_state         <= s_arb_sel_low_ts;
            elsif (fsm_wait_state_q = '0') then
                if (timestamp_valid = '1') then
                    next_state     <= s_arb_sel_ffw;
                end if;
            end if;
        
        --------------------------------------------------------------------
        -- Read Frame format word.
        --------------------------------------------------------------------  
        when s_arb_sel_ffw =>
             if (txtb_hw_cmd.lock = '1') then
                next_state         <= s_arb_locked;
            elsif (select_buf_avail = '0') then
                next_state         <= s_arb_idle;
            elsif (select_index_changed = '1') then
                next_state         <= s_arb_sel_low_ts;
            elsif (fsm_wait_state_q = '0') then
                next_state         <= s_arb_sel_idw;
            end if;
        
        --------------------------------------------------------------------
        -- Read identifier word.
        --------------------------------------------------------------------  
        when s_arb_sel_idw =>
             if (txtb_hw_cmd.lock = '1') then
                next_state         <= s_arb_locked;
            elsif (select_buf_avail = '0') then
                next_state         <= s_arb_idle;
            elsif (select_index_changed = '1') then
                next_state         <= s_arb_sel_low_ts;
            elsif (fsm_wait_state_q = '0') then
                next_state         <= s_arb_validated;
            end if;
        
        --------------------------------------------------------------------
        -- TXT Buffer is validated!
        --------------------------------------------------------------------  
        when s_arb_validated =>
             if (txtb_hw_cmd.lock = '1') then
                next_state         <= s_arb_locked;
            elsif (select_buf_avail = '0') then
                next_state         <= s_arb_idle;
            elsif (select_index_changed = '1') then
                next_state         <= s_arb_sel_low_ts;
            end if;
        
        --------------------------------------------------------------------
        -- Finishing the transmission = unlocking the buffer 
        --------------------------------------------------------------------  
        when s_arb_locked =>
            if (txtb_hw_cmd.unlock = '1') then
                next_state         <= s_arb_idle;
            end if;

        end case;
    end process;


    ------------------------------------------------------------------------------
    -- TX Arbitrator FSM outputs
    ------------------------------------------------------------------------------
    tx_arb_fsm_out_proc : process(curr_state, fsm_wait_state_q, timestamp_valid,
        select_index_changed, select_buf_avail, txtb_hw_cmd)
    begin
        
        -- By default all outputs are inactive
        load_ts_lw_addr        <= '0';
        load_ts_uw_addr        <= '0';
        load_ffmt_w_addr       <= '0';
        load_ident_w_addr      <= '0';
        
        -- By default, clocks for memories are gated
        txtb_meta_clk_en       <= '0';
        
        store_ts_l_w           <= '0';
        store_md_w             <= '0';
        store_ident_w          <= '0';
        buffer_md_w            <= '0'; 
        tx_arb_locked          <= '0';
        frame_valid_com_set    <= '0';
        frame_valid_com_clear  <= '0';
        store_last_txtb_index  <= '0';
        
        fsm_wait_state_d       <= '0';
    
        case curr_state is   
        
        --------------------------------------------------------------------
        -- No TXT Buffer is available, or was just unlocked after
        -- transmission
        --------------------------------------------------------------------
        when s_arb_idle =>
            if (select_buf_avail = '1') then
                fsm_wait_state_d <= '1';
                load_ts_lw_addr  <= '1';
            end if;

        --------------------------------------------------------------------
        -- Read Low timestamp word of Selected TXT buffer.
        --------------------------------------------------------------------
        when s_arb_sel_low_ts =>
            txtb_meta_clk_en <= '1';
            
            if (txtb_hw_cmd.lock = '1') then
                store_last_txtb_index <= '1';

            elsif (select_buf_avail = '0') then
                frame_valid_com_clear <= '1';

            elsif (select_index_changed = '1') then
                fsm_wait_state_d <= '1';
                load_ts_lw_addr <= '1';
                
            elsif (fsm_wait_state_q = '0') then
                fsm_wait_state_d <= '1';
                load_ts_uw_addr  <= '1';
                store_ts_l_w     <= '1';
            end if;
        
        --------------------------------------------------------------------
        -- Read Upper timestamp word of Selected TXT Buffer. Perform
        -- Timestamp comparison (Lower word is in capture register).
        -- When Timestamp elapses -> proceed with validation!
        --------------------------------------------------------------------  
        when s_arb_sel_upp_ts =>
            txtb_meta_clk_en <= '1';
            
            if (txtb_hw_cmd.lock = '1') then
                store_last_txtb_index <= '1';

            elsif (select_buf_avail = '0') then
                frame_valid_com_clear <= '1';

            elsif (select_index_changed = '1') then
                fsm_wait_state_d <= '1';
                load_ts_lw_addr  <= '1';
                
            elsif (fsm_wait_state_q = '0') then
                if (timestamp_valid = '1') then
                    fsm_wait_state_d <= '1';
                    load_ffmt_w_addr <= '1';
                end if;
            end if;
        
        --------------------------------------------------------------------
        -- Read Frame format word.
        --------------------------------------------------------------------  
        when s_arb_sel_ffw =>
            txtb_meta_clk_en <= '1';
            
            if (txtb_hw_cmd.lock = '1') then
                store_last_txtb_index <= '1';

            elsif (select_buf_avail = '0') then
                frame_valid_com_clear <= '1';

            elsif (select_index_changed = '1') then
                fsm_wait_state_d <= '1';
                load_ts_lw_addr  <= '1';
                
            elsif (fsm_wait_state_q = '0') then
                fsm_wait_state_d  <= '1';
                load_ident_w_addr <= '1';
                buffer_md_w       <= '1';
            end if;
        
        --------------------------------------------------------------------
        -- Read identifier word.
        --------------------------------------------------------------------  
        when s_arb_sel_idw =>
            txtb_meta_clk_en <= '1';
            
            if (txtb_hw_cmd.lock = '1') then
                store_last_txtb_index <= '1';

            elsif (select_buf_avail = '0') then
                frame_valid_com_clear <= '1';

            elsif (select_index_changed = '1') then
                fsm_wait_state_d <= '1';
                load_ts_lw_addr  <= '1';

            elsif (fsm_wait_state_q = '0') then
                store_ident_w       <= '1';
                store_md_w          <= '1';
                frame_valid_com_set <= '1';
            end if;

        --------------------------------------------------------------------
        -- TXT Buffer is validated!
        --------------------------------------------------------------------  
        when s_arb_validated =>
            if (txtb_hw_cmd.lock = '1') then
                store_last_txtb_index <= '1';

            elsif (select_buf_avail = '0') then
                frame_valid_com_clear <= '1';

            elsif (select_index_changed = '1') then
                fsm_wait_state_d    <= '1';
                load_ts_lw_addr     <= '1';
            end if;
        
        --------------------------------------------------------------------
        -- Finishing the transmission = unlocking the buffer 
        --------------------------------------------------------------------  
        when s_arb_locked =>
            tx_arb_locked <= '1';

            if (txtb_hw_cmd.unlock = '1') then
                frame_valid_com_clear <= '1';
            end if;

        end case;
    end process;

  
    ---------------------------------------------------------------------------
    -- State register
    ---------------------------------------------------------------------------
    tx_arb_fsm_state_reg : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            curr_state <= s_arb_sel_low_ts;
        elsif (rising_edge(clk_sys)) then
            if (tx_arb_fsm_ce = '1') then
                curr_state <= next_state;
            end if;
        end if;
    end process;
    
    -- State transition enable
    tx_arb_fsm_ce <= '1' when (curr_state /= next_state) else
                     '0';


    ---------------------------------------------------------------------------
    -- Wait state DFF. Wait state inserted when next state must last two clock
    -- cycles!
    ---------------------------------------------------------------------------
    fsm_wait_state_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            fsm_wait_state_q <= '1';
        elsif (rising_edge(clk_sys)) then
            fsm_wait_state_q <= fsm_wait_state_d;
        end if;
    end process;

  -- <RELEASE_OFF>
  
  ------------------------------------------------------------------------------
  -- Functional coverage
  ------------------------------------------------------------------------------
  
  -- psl default clock is rising_edge(clk_sys);

  -- LOCK commands in various parts of TXT Buffer validation

  -- psl txtb_lock_arb_sel_low_cov : cover
  --  {curr_state = s_arb_sel_low_ts and txtb_hw_cmd.lock = '1'}; 
  --
  -- psl txtb_lock_arb_sel_hi_cov : cover
  --  {curr_state = s_arb_sel_upp_ts and txtb_hw_cmd.lock = '1'};
  --
  -- psl txtb_lock_arb_sel_ffw_cov : cover
  --  {curr_state = s_arb_sel_ffw and txtb_hw_cmd.lock = '1'};
  --
  -- psl txtb_lock_arb_sel_idw_cov : cover
  --  {curr_state = s_arb_sel_idw and txtb_hw_cmd.lock = '1'};
  --
  -- psl txtb_lock_arb_sel_validated_cov : cover
  --  {curr_state = s_arb_validated and txtb_hw_cmd.lock = '1'};


  -- TXT Buffer becoming suddenly unavailable during TXT Buffer validation
 
  -- psl txtb_not_available_arb_sel_low_cov : cover
  --  {curr_state = s_arb_sel_low_ts and select_buf_avail = '0'};
  --
  -- psl txtb_not_available_arb_sel_upp_cov : cover
  --  {curr_state = s_arb_sel_upp_ts and select_buf_avail = '0'};
  --
  -- psl txtb_not_available_arb_sel_ffw_cov : cover
  --  {curr_state = s_arb_sel_ffw and select_buf_avail = '0'};
  --
  -- psl txtb_not_available_arb_sel_idw_cov : cover
  --  {curr_state = s_arb_sel_idw and select_buf_avail = '0'};
  --
  -- psl txtb_not_available_arb_validated_cov : cover
  --  {curr_state = s_arb_validated and select_buf_avail = '0'};  


  -- TXT Buffer index change during TXT Buffer validation

  -- psl txtb_changed_arb_sel_low_cov : cover
  --  {curr_state = s_arb_sel_low_ts and select_index_changed = '0'};
  --
  -- psl txtb_changed_arb_sel_upp_cov : cover
  --  {curr_state = s_arb_sel_upp_ts and select_index_changed = '0'};
  --
  -- psl txtb_changed_arb_sel_ffw_cov : cover
  --  {curr_state = s_arb_sel_ffw and select_index_changed = '0'};
  --
  -- psl txtb_changed_arb_sel_idw_cov : cover
  --  {curr_state = s_arb_sel_idw and select_index_changed = '0'};
  --
  -- psl txtb_changed_arb_validated_cov : cover
  --  {curr_state = s_arb_validated and select_index_changed = '0'};  


  -- Waiting till timestamp will be ready (transmission at given time)

  -- psl txt_buf_wait_till_timestamp_cov : cover
  --    {curr_state = s_arb_sel_upp_ts and fsm_wait_state_q = '0' and
  --     timestamp_valid = '0'};

  ------------------------------------------------------------------------------
  -- Assertions
  ------------------------------------------------------------------------------

  -- psl tx_arb_no_lock_when_idle_asrt : assert never
  --    (curr_state = s_arb_idle and txtb_hw_cmd.lock = '1');

  -- psl tx_arb_no_lock_when_locked_asrt : assert never
  --    (curr_state = s_arb_locked and txtb_hw_cmd.lock = '1');

  -- <RELEASE_ON>
end architecture;