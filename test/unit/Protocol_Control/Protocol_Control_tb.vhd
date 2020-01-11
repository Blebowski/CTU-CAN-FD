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
-- @TestInfoStart
--
-- @Purpose:
--  Unit test for Protocol Control. The architecture of this test is depicted
--  in picture below. It is inherited from original Protocol Control testbench
--  created during implementation of the CAN FD IP Core. This testbench only
--  tests the state machine of protocol control!
--
--  Following features of are NOT tested here:
--    - bit stuffing and destuffing (frames transmitted from protocol control
--      is without bit stuffing)
--    - CRC calculation  (CRC is provided to protocol control and in this
--      circuit always fixed value is provided)
--    - stuff and destuff counters are always zero provided to
--      Protocol control
--    - Synchronization - triggerring signals are fixed in this testbench,
--      no synchronization is happenning!
--    - error detection!
--
-- Following features are tested here:
--    - Correct bit sequence on the output of protocol control on TX and RX
--      sides
--    - Frame integrity, TX frame = RX Frame.
--
-- Architecture:
--
--      -----------    ---------------------
--  |---| Receive |<---| Protocol control 2|<-|
--  |   |  frame  |    ---------------------  |  bus_level
--  |   -----------                           |
--  |                                         |
--  |  -----------    ---------------------   |  ---------------------
--  |  | Generate|--->| Protocol control 1|----->|Record the frame as|
--  |  |   frame | |  ---------------------      |   bit sequence    |
--  |  ----------- |                             ---------------------
--  |              |    ----------------                     |
--  |              |--->| SW CAN Model |---------------|     | recorded frame
--  |              |    ----------------   expected    v     v
--  |              |                        frame     ----------
--  -----------    |                                  | compare|
--            |    |                                  ----------
--            v    v                                      |
--           ---------                                    |
--           |compare|                                    |
--           ---------                                    v
--               |                                     Error if
--               v                                   not matching
--            Error if
--          not matching
--
--
--  Test sequence:
--    1. Generate random frame on input of Protocol Control 1
--    2. Calculate expected bit sequence (frame) on the CAN bus including
--       ACK and EOF!
--    3. Transmitt the frame and record the bit sequence!
--    4. Compare if Expected bit sequence is equal to recorded one
--    5. Compare if Generated frame (data,ident,type of frame...) is equal
--       to received one!
--    6. If points 4 or 5 give mismatch increase error counter
--    7. Loop points 1 to 6 until the number of iterations was reached!
--
--    Note that since additional function for buidling CAN frame in "SW" is used
--    it verifies the protocol control towards errors which can be not detected
--    when both PC1 and PC2 have the error. E.g if PC has missing bit then it
--    won't transmitt it, it won't receive it and it will communicate hapilly
--    further! But the transmitted frame would not be according to spec!
--    This is reason for SW model of CAN Frame!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    13.6.2016   Created file
--    22.6.2016   Modified tb to be compliant with latest bugfixes in protocol
--                control!
--    29.5.2018   1. Modified testbench to be compatible with direct loading
--                   of data from TXT Buffer and storing to RX Buffer via
--                   RX Storing protocol.
--                2. Added dynamically generated stuff lenght and SW model
--                   for grey coding of sutff length.
--    13.7.2018   Added Unknown operational state signals!
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

architecture Protocol_Control_unit_test of CAN_test is

    ----------------------------------------------------------------------------
    -- Common Signals from/to DUTs
    ----------------------------------------------------------------------------
    signal clk_sys                  :  std_logic := '0';
    signal res_n                    :  std_logic := '0';

    -- Driving bus
    signal drv_bus                  :  std_logic_vector(1023 downto 0) :=
                                        (OTHERS => '0');

    -- CRC results
    signal crc_15                    :  std_logic_vector(14 downto 0) :=
                                        (OTHERS => '0');
    signal crc_17                    :  std_logic_vector(16 downto 0) :=
                                        (OTHERS => '0');
    signal crc_21                    :  std_logic_vector(20 downto 0) :=
                                        (OTHERS => '0');
                                        
    ----------------------------------------------------------------------------
    -- Protocol Control 1
    ----------------------------------------------------------------------------
    signal alc_1                     :  std_logic_vector(7 downto 0);
    signal erc_capture_1             :  std_logic_vector(7 downto 0);
    signal is_arbitration_1          :  std_logic;
    signal is_control_1              :  std_logic;
    signal is_data_1                 :  std_logic;
    signal is_stuff_count_1          :  std_logic;
    signal is_crc_1                  :  std_logic;
    signal is_crc_delim_1            :  std_logic;
    signal is_ack_field_1            :  std_logic;
    signal is_ack_delim_1            :  std_logic;
    signal is_eof_1                  :  std_logic;
    signal is_intermission_1         :  std_logic;
    signal is_suspend_1              :  std_logic;
    signal is_err_frm_1                :  std_logic;
    signal is_overload_1             :  std_logic;
    
    -- TXT Buffers interface
    signal tran_word_1               :   std_logic_vector(31 downto 0);
    signal tran_dlc_1                :   std_logic_vector(3 downto 0);
    signal tran_is_rtr_1             :   std_logic;
    signal tran_ident_type_1         :   std_logic;
    signal tran_frame_type_1         :   std_logic;
    signal tran_brs_1                :   std_logic;
    signal tran_identifier_1         :   std_logic_vector(28 downto 0);
    signal tran_frame_valid_1        :   std_logic;
    signal txtb_hw_cmd_1             :   t_txtb_hw_cmd;
    signal txtb_ptr_1                :   natural range 0 to 19;
    signal txtb_changed_1            :   std_logic := '0';
    
    -- RX Buffer interface
    signal rec_ident_1               :  std_logic_vector(28 downto 0);
    signal rec_dlc_1                 :  std_logic_vector(3 downto 0);
    signal rec_is_rtr_1              :  std_logic;
    signal rec_ident_type_1          :  std_logic;
    signal rec_frame_type_1          :  std_logic;
    signal rec_brs_1                 :  std_logic;
    signal rec_esi_1                 :  std_logic;
    signal store_metadata_1          :  std_logic;
    signal rec_abort_1               :  std_logic;
    signal store_data_1              :  std_logic;
    signal store_data_word_1         :  std_logic_vector(31 downto 0);
    signal sof_pulse_1               :  std_logic;

    -- Operation control FSM Interface
    signal is_transmitter_1          :  std_logic := '1';
    signal is_receiver_1             :  std_logic := '0';
    signal is_idle_1                 :  std_logic := '0';
    signal arbitration_lost_1        :  std_logic;
    signal set_transmitter_1         :  std_logic;
    signal set_receiver_1            :  std_logic;
    signal set_idle_1                :  std_logic;
    
    -- Fault confinement Interface
    signal is_err_active_1           :  std_logic := '1';
    signal is_err_passive_1          :  std_logic := '0';
    signal is_bus_off_1              :  std_logic := '0';
    signal err_detected_1            :  std_logic;
    signal primary_err_1           :  std_logic;
    signal act_err_ovr_flag_1        :  std_logic;
    signal err_delim_late_1          :  std_logic;
    signal set_err_active_1          :  std_logic;
    signal err_ctrs_unchanged_1      :  std_logic;
    
    -- TX and RX Trigger signals to Sample and Transmitt Data
    signal tx_trigger_1              :  std_logic := '0';
    signal rx_trigger_1              :  std_logic := '0';

    -- CAN Bus serial data stream
    signal tx_data_nbs_1             :  std_logic;

    -- Sampled TX data by TX Trigger (emulation of Bit Stuffing)
    signal tx_data_nbs_1_q           :  std_logic;
    signal tx_data_nbs_2_q           :  std_logic;

    -- Bit Stuffing Interface
    signal stuff_enable_1            :  std_logic;
    signal destuff_enable_1          :  std_logic;
    signal fixed_stuff_1             :  std_logic;
    signal stuff_length_1            :  std_logic_vector(2 downto 0);
    signal dst_ctr_1                 :  std_logic_vector(2 downto 0);
    signal bst_ctr_1                 :  std_logic_vector(2 downto 0);
    signal stuff_err_1             :  std_logic := '0';
    
    -- Bus Sampling Interface
    signal bit_err_1               :  std_logic := '0';
    
    -- CRC Interface
    signal crc_enable_1              :  std_logic;
    signal crc_spec_enable_1         :  std_logic;
    
    -- Control signals
    signal sp_control_1              :  std_logic_vector(1 downto 0);
    signal sync_control_1            :  std_logic_vector(1 downto 0); 
    signal no_pos_resync_1           :  std_logic;
    signal ssp_reset_1               :  std_logic;
    signal tran_delay_meas_1         :  std_logic;
    signal tran_valid_1              :  std_logic;
    signal rec_valid_1               :  std_logic;

    -- Status signals
    signal ack_received_1            :  std_logic;
    signal br_shifted_1              :  std_logic;
    signal form_err_1              :  std_logic;
    signal ack_err_1               :  std_logic;
    signal crc_err_1               :  std_logic;
 
 
    ----------------------------------------------------------------------------
    -- Protocol Control 2
    ----------------------------------------------------------------------------
    signal alc_2                     :  std_logic_vector(7 downto 0);
    signal erc_capture_2             :  std_logic_vector(7 downto 0);
    signal is_arbitration_2          :  std_logic;
    signal is_control_2              :  std_logic;
    signal is_data_2                 :  std_logic;
    signal is_stuff_count_2          :  std_logic;
    signal is_crc_2                  :  std_logic;
    signal is_crc_delim_2            :  std_logic;
    signal is_ack_field_2            :  std_logic;
    signal is_ack_delim_2            :  std_logic;
    signal is_eof_2                  :  std_logic;
    signal is_intermission_2         :  std_logic;
    signal is_suspend_2              :  std_logic;
    signal is_err_frm_2                :  std_logic;
    signal is_overload_2             :  std_logic;
    
    -- TXT Buffers interface
    signal tran_word_2               :   std_logic_vector(31 downto 0)
         := (OTHERS => '0');
         
    signal tran_dlc_2                :   std_logic_vector(3 downto 0)
        := (OTHERS => '0');
        
    signal tran_is_rtr_2             :   std_logic := '0';
    signal tran_ident_type_2         :   std_logic := '0';
    signal tran_frame_type_2         :   std_logic := '0';
    signal tran_brs_2                :   std_logic := '0';
    signal tran_identifier_2         :   std_logic_vector(28 downto 0) := (OTHERS => '0');
    signal tran_frame_valid_2        :   std_logic := '0'; 
    signal txtb_hw_cmd_2             :   t_txtb_hw_cmd;
    signal txtb_ptr_2                :   natural range 0 to 19;
    signal txtb_changed_2            :   std_logic := '0';
    
    -- RX Buffer interface
    signal rec_ident_2               :  std_logic_vector(28 downto 0);
    signal rec_dlc_2                 :  std_logic_vector(3 downto 0);
    signal rec_is_rtr_2              :  std_logic;
    signal rec_ident_type_2          :  std_logic;
    signal rec_frame_type_2          :  std_logic;
    signal rec_brs_2                 :  std_logic;
    signal rec_esi_2                 :  std_logic;
    signal store_metadata_2          :  std_logic;
    signal rec_abort_2               :  std_logic;
    signal store_data_2              :  std_logic;
    signal store_data_word_2         :  std_logic_vector(31 downto 0);
    signal sof_pulse_2               :  std_logic;

    -- Operation control FSM Interface
    signal is_transmitter_2          :  std_logic := '0';
    signal is_receiver_2             :  std_logic := '1';
    signal is_idle_2                 :  std_logic := '0';
    signal arbitration_lost_2        :  std_logic;
    signal set_transmitter_2         :  std_logic;
    signal set_receiver_2            :  std_logic;
    signal set_idle_2                :  std_logic;
    
    -- Fault confinement Interface
    signal is_err_active_2           :  std_logic := '1';
    signal is_err_passive_2          :  std_logic := '0';
    signal is_bus_off_2              :  std_logic := '0';
    signal err_detected_2            :  std_logic;
    signal primary_err_2           :  std_logic;
    signal act_err_ovr_flag_2        :  std_logic;
    signal err_delim_late_2          :  std_logic;
    signal set_err_active_2          :  std_logic;
    signal err_ctrs_unchanged_2      :  std_logic;
    
    -- TX and RX Trigger signals to Sample and Transmitt Data
    signal tx_trigger_2              :   std_logic := '0';
    signal rx_trigger_2              :   std_logic := '0';

    -- CAN Bus serial data stream
    signal tx_data_nbs_2             :  std_logic;

    -- Bit Stuffing Interface
    signal stuff_enable_2            :  std_logic;
    signal destuff_enable_2          :  std_logic;
    signal fixed_stuff_2             :  std_logic;
    signal stuff_length_2            :  std_logic_vector(2 downto 0);
    signal dst_ctr_2                 :  std_logic_vector(2 downto 0);
    signal bst_ctr_2                 :  std_logic_vector(2 downto 0);
    signal stuff_err_2             :  std_logic := '0';
    
    -- Bus Sampling Interface
    signal bit_err_2               :   std_logic := '0';
    
    -- CRC Interface
    signal crc_enable_2              :  std_logic;
    signal crc_spec_enable_2         :  std_logic;
    
    -- Control signals
    signal sp_control_2              :  std_logic_vector(1 downto 0);
    signal sync_control_2            :  std_logic_vector(1 downto 0); 
    signal no_pos_resync_2           :  std_logic;
    signal ssp_reset_2               :  std_logic;
    signal tran_delay_meas_2         :  std_logic;
    signal tran_valid_2              :  std_logic;
    signal rec_valid_2               :  std_logic;

    -- Status signals
    signal ack_received_2            :  std_logic;
    signal br_shifted_2              :  std_logic;
    signal form_err_2              :  std_logic;
    signal ack_err_2               :  std_logic;
    signal crc_err_2               :  std_logic;
    

    ----------------------------------------------------------------------------
    -- Internal testbench signals
    ----------------------------------------------------------------------------

    -- Logic level on CAN bus
    signal bus_level                :  std_logic;

    -- Random pool pointer
    signal rnd_ctr_tr               :  natural range 0 to RAND_POOL_SIZE := 0;

    -- TXT Buffer memory model
    signal txtb_mem                 :  test_mem_type :=
                                        (OTHERS => (OTHERS => '0'));
    signal txtb_mem_ptr             :  natural := 0;

    -- RX Buffer memory model
    signal rxb_mem                  :  test_mem_type :=
                                        (OTHERS => (OTHERS => '0'));
    signal rxb_mem_ptr              :  natural := 0;

    -- Type of FD Format Frame (ISO,non-ISO)
    signal drv_fd_type              :     std_logic;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Testbench procedures and functions
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    procedure stuff_count_grey_code(
        constant st_ctr           :in   natural range 0 to 7;
        variable parity           :out  std_logic;
        variable result           :out  std_logic_vector(2 downto 0)
    ) is
        variable tmp              :     std_logic_vector(2 downto 0);
    begin
        case st_ctr is
            when 0 =>   tmp  := "000";
            when 1 =>   tmp  := "001";
            when 2 =>   tmp  := "011";
            when 3 =>   tmp  := "010";
            when 4 =>   tmp  := "110";
            when 5 =>   tmp  := "111";
            when 6 =>   tmp  := "101";
            when 7 =>   tmp  := "100";
            when others =>
                -- LCOV_EXCL_START
                failure("Invalid stuff counter value");
                -- LCOV_EXCL_STOP
        end case;

        parity := tmp(0) xor tmp(1) xor tmp(2);
        result := tmp;
    end procedure;


    ----------------------------------------------------------------------------
    -- SW model of Protocol control
    ----------------------------------------------------------------------------
    procedure gen_sw_CAN(
        constant   tx_frame         :in   SW_CAN_frame_type;
        constant   is_err_active    :in   std_logic;
        constant   drv_iso_fd       :in   std_logic;
        constant   crc_15           :in   std_logic_vector(14 downto 0);
        constant   crc_17           :in   std_logic_vector(16 downto 0);
        constant   crc_21           :in   std_logic_vector(20 downto 0);
        constant   stuff_cnt        :in   natural range 0 to 7;

        -- Generated bit sequence
        variable seq              :out  std_logic_vector(639 downto 0);

        -- Length of the generated sequence
        variable length           :out  natural
    )is
        variable join           :     std_logic_vector(1 downto 0);
        variable crc_length     :     natural;
        variable id_word        :     std_logic_vector(28 downto 0);
        variable ptr            :     natural := 0;
        variable stuff_parity   :     std_logic;
        variable stuff_cnt_grey :     std_logic_vector(2 downto 0);
        variable tmp_crc        :     std_logic_vector(20 downto 0);
    begin

        -- SOF bit
        seq(ptr)    := DOMINANT;
        ptr         := ptr + 1;

        -- Identifier BASE
        id_sw_to_hw(tx_frame.identifier, tx_frame.ident_type, id_word);
        for i in 0 to 10 loop
            seq(ptr)    := id_word(IDENTIFIER_BASE_H - i);
            ptr         := ptr + 1;
        end loop;

        -- Two bits between BASE and EXTENSION (RTR,IDE / r1,IDE / SRR,IDE)
        if (tx_frame.ident_type = EXTENDED) then
            seq(ptr)        := RECESSIVE;   -- SRR
            seq(ptr + 1)    := RECESSIVE;   -- IDE

        else
            if (tx_frame.frame_format = NORMAL_CAN) then
                seq(ptr)        := tx_frame.rtr; -- RTR
            else
                seq(ptr)        := DOMINANT;     -- r1
            end if;
            seq(ptr + 1)        := DOMINANT;     -- IDE
        end if;
        ptr := ptr + 2;

        -- Identifier EXTENSION
        if (tx_frame.ident_type = EXTENDED) then
            for i in 0 to 17 loop
                seq(ptr)    := id_word(IDENTIFIER_EXT_H - i);
                ptr         := ptr + 1;
            end loop;
        end if;

        -- Remaining bits of control field (apart from DLC)
        join       := tx_frame.frame_format & tx_frame.ident_type;
        case join is
        when NORMAL_CAN & BASE =>
            seq(ptr)        := DOMINANT;       -- r0
            ptr             := ptr + 1;

        when NORMAL_CAN & EXTENDED =>
            seq(ptr)        := tx_frame.rtr;   -- RTR
            seq(ptr + 1)    := DOMINANT;       -- r1
            seq(ptr + 2)    := DOMINANT;       -- r0
            ptr             := ptr + 3;

        when FD_CAN & BASE =>
            seq(ptr)        := RECESSIVE;      -- EDL
            seq(ptr + 1)    := DOMINANT;       -- r0
            seq(ptr + 2)    := tx_frame.brs;   -- BRS

            -- ESI bit
            if (is_err_active = '1') then
                seq(ptr + 3) := DOMINANT;
            else
                seq(ptr + 3) := RECESSIVE;
            end if;
            ptr             := ptr + 4;

        when FD_CAN & EXTENDED =>
            seq(ptr)        := DOMINANT;       -- r1
            seq(ptr + 1)    := RECESSIVE;      -- EDL
            seq(ptr + 2)    := DOMINANT;       -- r0
            seq(ptr + 3)    := tx_frame.brs;   -- BRS

            -- ESI bit
            if (is_err_active = '1') then
                seq(ptr + 4) := DOMINANT;
            else
                seq(ptr + 4) := RECESSIVE;
            end if;
            ptr             := ptr + 5;
        when others =>
            -- LCOV_EXCL_START
            failure("Invalid CAN FD Frame settings");
            -- LCOV_EXCL_STOP
        end case;

        -- DLC field
        for i in 0 to 3 loop
            seq(ptr)        := tx_frame.dlc(3 - i);
            ptr             := ptr + 1;
        end loop;

        -- Data field
        if (tx_frame.data_length > 0) then
            for i in 0 to tx_frame.data_length - 1 loop
                for j in 7 downto 0 loop
                    seq(ptr) := tx_frame.data(i)(j);
                    ptr      := ptr + 1;
                end loop;
            end loop;
        end if;

        -- Stuff count field (Grey coded)
        if (drv_iso_fd = ISO_FD and tx_frame.frame_format = FD_CAN) then
            stuff_count_grey_code(stuff_cnt, stuff_parity, stuff_cnt_grey);
            seq(ptr)       :=  stuff_cnt_grey(2);
            seq(ptr + 1)   :=  stuff_cnt_grey(1);
            seq(ptr + 2)   :=  stuff_cnt_grey(0);
            seq(ptr + 3)   :=  stuff_parity;
            ptr            :=  ptr + 4;
        end if;

        -- CRC sequence
        if (tx_frame.frame_format = NORMAL_CAN) then
            crc_length     := 15;
            tmp_crc        := crc_15 & "000000";
        elsif (tx_frame.data_length <= 16) then
            crc_length     := 17;
            tmp_crc        := crc_17 & "0000";
        else
            crc_length     := 21;
            tmp_crc        := crc_21;
        end if;

        for i in 20 downto 20 - crc_length + 1 loop
            seq(ptr)       := tmp_crc(i);
            ptr            := ptr + 1;
        end loop;

        -- CRC delimiter
        seq(ptr)           := RECESSIVE;
        ptr                := ptr + 1;

        -- ACK bit (assume frame will be received correctly)
        seq(ptr)           := DOMINANT;
        ptr                := ptr + 1;

        -- ACK delimiter
        seq(ptr)           := RECESSIVE;
        ptr                := ptr + 1;

        -- EOF
        for i in 0 to 6 loop
            seq(ptr)       := RECESSIVE;
            ptr            := ptr + 1;
        end loop;

        -- Now we propagate the length to the output
        length              := ptr + 1;

    end procedure;


    ----------------------------------------------------------------------------
    -- Record what is on bus! With known length from SW CAN, we know
    -- what should be there!
    ----------------------------------------------------------------------------
    procedure record_bit_seq(
        -- Bus level to be recorded
        signal   bus_line             :in   std_logic;

        -- Signal to sample the bus level
        signal   sample               :in   std_logic;

        -- How many bits should be recorded
        variable length               :in   natural;
        variable recorded             :out  std_logic_vector(639 downto 0)
    )is
    begin

        -- Wait until we are on SOF bit
        wait until bus_line = DOMINANT;

        for i in 0 to length - 1 loop
            wait until rising_edge(sample);
            recorded(i) := bus_line;
        end loop;

    end procedure;


    ----------------------------------------------------------------------------
    -- Compare what is expected to be transmitted and what is transmitted
    ----------------------------------------------------------------------------
    procedure compare_bit_seq(
        variable bs1        :in   std_logic_vector(639 downto 0);
        variable bs2        :in   std_logic_vector(639 downto 0);
        variable mut_length :in   natural; -- Mutual length of both vectors
        variable outcome    :out  boolean
    ) is
    begin
        outcome:= true;

        for i in 1 to mut_length - 1 loop
            if (bs1(i - 1) /= bs2(i - 1)) then
                outcome := false;
            end if;
        end loop;

    end procedure;


begin

    protocol_control_inst_1 : protocol_control
    generic map(
        G_RESET_POLARITY        => C_RESET_POLARITY,
        G_CTRL_CTR_WIDTH        => C_CTRL_CTR_WIDTH,
        G_RETR_LIM_CTR_WIDTH    => C_RETR_LIM_CTR_WIDTH,
        G_ERR_VALID_PIPELINE    => C_ERR_VALID_PIPELINE
    )
    port map(
        clk_sys                 => clk_sys,
        res_n                   => res_n,
        
        -- Memory registers interface
        drv_bus                 => drv_bus,
        alc                     => alc_1,
        erc_capture             => erc_capture_1,
        is_arbitration          => is_arbitration_1,
        is_control              => is_control_1,
        is_data                 => is_data_1,
        is_stuff_count          => is_stuff_count_1,
        is_crc                  => is_crc_1,
        is_crc_delim            => is_crc_delim_1,
        is_ack_field            => is_ack_field_1,
        is_ack_delim            => is_ack_delim_1,
        is_eof                  => is_eof_1,
        is_intermission         => is_intermission_1,
        is_suspend              => is_suspend_1,
        is_err_frm                => is_err_frm_1,
        is_overload             => is_overload_1,
        
        -- TXT Buffers interface
        tran_word               => tran_word_1,
        tran_dlc                => tran_dlc_1,
        tran_is_rtr             => tran_is_rtr_1,
        tran_ident_type         => tran_ident_type_1,
        tran_frame_type         => tran_frame_type_1,
        tran_brs                => tran_brs_1,
        tran_identifier         => tran_identifier_1,
        tran_frame_valid        => tran_frame_valid_1,
        txtb_hw_cmd             => txtb_hw_cmd_1,
        txtb_ptr                => txtb_ptr_1,
        txtb_changed            => txtb_changed_1,
        
        -- RX Buffer interface
        rec_ident               => rec_ident_1,
        rec_dlc                 => rec_dlc_1,
        rec_is_rtr              => rec_is_rtr_1,
        rec_ident_type          => rec_ident_type_1,
        rec_frame_type          => rec_frame_type_1,
        rec_brs                 => rec_brs_1,
        rec_esi                 => rec_esi_1,
        store_metadata          => store_metadata_1,
        rec_abort               => rec_abort_1,
        store_data              => store_data_1,
        store_data_word         => store_data_word_1,
        sof_pulse               => sof_pulse_1,
    
        -- Operation control FSM Interface
        is_transmitter          => is_transmitter_1,
        is_receiver             => is_receiver_1,
        is_idle                 => is_idle_1,
        arbitration_lost        => arbitration_lost_1,
        set_transmitter         => set_transmitter_1,
        set_receiver            => set_receiver_1,
        set_idle                => set_idle_1,
        
        -- Fault confinement Interface
        is_err_active           => is_err_active_1,
        is_err_passive          => is_err_passive_1,
        is_bus_off              => is_bus_off_1,
        err_detected            => err_detected_1,
        primary_err           => primary_err_1,
        act_err_ovr_flag        => act_err_ovr_flag_1,
        err_delim_late          => err_delim_late_1,
        set_err_active          => set_err_active_1,
        err_ctrs_unchanged      => err_ctrs_unchanged_1,
        
        -- TX and RX Trigger signals to Sample and Transmitt Data
        tx_trigger              => tx_trigger_1,
        rx_trigger              => rx_trigger_1,

        -- CAN Bus serial data stream
        tx_data_nbs             => tx_data_nbs_1,
        tx_data_wbs             => tx_data_nbs_1_q,
        rx_data_nbs             => bus_level,

        -- Bit Stuffing Interface
        stuff_enable            => stuff_enable_1,
        destuff_enable          => destuff_enable_1,
        fixed_stuff             => fixed_stuff_1,
        stuff_length            => stuff_length_1,
        dst_ctr                 => dst_ctr_1,
        bst_ctr                 => bst_ctr_1,
        stuff_err             => stuff_err_1,
        
        -- Bus Sampling Interface
        bit_err               => bit_err_1,
        
        -- CRC Interface
        crc_enable              => crc_enable_1,
        crc_spec_enable         => crc_spec_enable_1,
        crc_15                  => crc_15,
        crc_17                  => crc_17,
        crc_21                  => crc_21,
        
        -- Control signals
        sp_control              => sp_control_1,
        sync_control            => sync_control_1,
        no_pos_resync           => no_pos_resync_1,
        ssp_reset               => ssp_reset_1,
        tran_delay_meas         => tran_delay_meas_1,
        tran_valid              => tran_valid_1,
        rec_valid               => rec_valid_1,

        -- Status signals
        ack_received            => ack_received_1,
        br_shifted              => br_shifted_1,
        form_err              => form_err_1,
        ack_err               => ack_err_1,
        crc_err               => crc_err_1
    );
    
    
    protocol_control_inst_2 : protocol_control
    generic map(
        G_RESET_POLARITY        => C_RESET_POLARITY,
        G_CTRL_CTR_WIDTH        => C_CTRL_CTR_WIDTH,
        G_RETR_LIM_CTR_WIDTH    => C_RETR_LIM_CTR_WIDTH,
        G_ERR_VALID_PIPELINE    => C_ERR_VALID_PIPELINE
    )
    port map(
        clk_sys                 => clk_sys,
        res_n                   => res_n,
        
        -- Memory registers interface
        drv_bus                 => drv_bus,
        alc                     => alc_2,
        erc_capture             => erc_capture_2,
        is_arbitration          => is_arbitration_2,
        is_control              => is_control_2,
        is_data                 => is_data_2,
        is_stuff_count          => is_stuff_count_2,
        is_crc                  => is_crc_2,
        is_crc_delim            => is_crc_delim_2,
        is_ack_field            => is_ack_field_2,
        is_ack_delim            => is_ack_delim_2,
        is_eof                  => is_eof_2,
        is_intermission         => is_intermission_2,
        is_suspend              => is_suspend_2,
        is_err_frm              => is_err_frm_2,
        is_overload             => is_overload_2,
        
        -- TXT Buffers interface
        tran_word               => tran_word_2,
        tran_dlc                => tran_dlc_2,
        tran_is_rtr             => tran_is_rtr_2,
        tran_ident_type         => tran_ident_type_2,
        tran_frame_type         => tran_frame_type_2,
        tran_brs                => tran_brs_2,
        tran_identifier         => tran_identifier_2,
        tran_frame_valid        => tran_frame_valid_2,
        txtb_hw_cmd             => txtb_hw_cmd_2,
        txtb_ptr                => txtb_ptr_2,
        txtb_changed            => txtb_changed_2,
        
        -- RX Buffer interface
        rec_ident               => rec_ident_2,
        rec_dlc                 => rec_dlc_2,
        rec_is_rtr              => rec_is_rtr_2,
        rec_ident_type          => rec_ident_type_2,
        rec_frame_type          => rec_frame_type_2,
        rec_brs                 => rec_brs_2,
        rec_esi                 => rec_esi_2,
        store_metadata          => store_metadata_2,
        rec_abort               => rec_abort_2,
        store_data              => store_data_2,
        store_data_word         => store_data_word_2,
        sof_pulse               => sof_pulse_2,
    
        -- Operation control FSM Interface
        is_transmitter          => is_transmitter_2,
        is_receiver             => is_receiver_2,
        is_idle                 => is_idle_2,
        arbitration_lost        => arbitration_lost_2,
        set_transmitter         => set_transmitter_2,
        set_receiver            => set_receiver_2,
        set_idle                => set_idle_2,
        
        -- Fault confinement Interface
        is_err_active           => is_err_active_2,
        is_err_passive          => is_err_passive_2,
        is_bus_off              => is_bus_off_2,
        err_detected            => err_detected_2,
        primary_err             => primary_err_2,
        act_err_ovr_flag        => act_err_ovr_flag_2,
        err_delim_late          => err_delim_late_2,
        set_err_active          => set_err_active_2,
        err_ctrs_unchanged      => err_ctrs_unchanged_2,
        
        -- TX and RX Trigger signals to Sample and Transmitt Data
        tx_trigger              => tx_trigger_1,
        rx_trigger              => rx_trigger_1,

        -- CAN Bus serial data stream
        tx_data_nbs             => tx_data_nbs_2,
        tx_data_wbs             => tx_data_nbs_2_q,
        rx_data_nbs             => bus_level,

        -- Bit Stuffing Interface
        stuff_enable            => stuff_enable_2,
        destuff_enable          => destuff_enable_2,
        fixed_stuff             => fixed_stuff_2,
        stuff_length            => stuff_length_2,
        dst_ctr                 => dst_ctr_2,
        bst_ctr                 => bst_ctr_2,
        stuff_err               => stuff_err_2,
        
        -- Bus Sampling Interface
        bit_err                 => bit_err_2,
        
        -- CRC Interface
        crc_enable              => crc_enable_2,
        crc_spec_enable         => crc_spec_enable_2,
        crc_15                  => crc_15,
        crc_17                  => crc_17,
        crc_21                  => crc_21,
        
        -- Control signals
        sp_control              => sp_control_2,
        sync_control            => sync_control_2,
        no_pos_resync           => no_pos_resync_2,
        ssp_reset               => ssp_reset_2,
        tran_delay_meas         => tran_delay_meas_2,
        tran_valid              => tran_valid_2,
        rec_valid               => rec_valid_2,

        -- Status signals
        ack_received            => ack_received_2,
        br_shifted              => br_shifted_2,
        form_err                => form_err_2,
        ack_err                 => ack_err_2,
        crc_err                 => crc_err_2
    );
    
    ----------------------------------------------------------------------------
    -- Sampling of TX Data
    -- emulation of Bit Stuffing
    ----------------------------------------------------------------------------
    tx_sample_proc : process(clk_sys, res_n)
    begin
        if (res_n = C_RESET_POLARITY) then
            tx_data_nbs_1_q <= RECESSIVE;
            tx_data_nbs_2_q <= RECESSIVE;
        elsif (rising_edge(clk_sys)) then
            if (tx_trigger_1 = '1') then
                tx_data_nbs_1_q <= tx_data_nbs_1;
                tx_data_nbs_2_q <= tx_data_nbs_2;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Creating bus level
    ----------------------------------------------------------------------------
    bus_level             <= tx_data_nbs_1_q AND tx_data_nbs_2_q;


    ----------------------------------------------------------------------------
    -- Driving bus aliases (equal for both nodes)
    ----------------------------------------------------------------------------
    drv_bus(DRV_CAN_FD_ENA_INDEX)                     <=  '1';
    drv_bus(DRV_BUS_MON_ENA_INDEX)                    <=  '0';
    drv_bus(DRV_RETR_LIM_ENA_INDEX)                   <=  '0';
    drv_bus(DRV_RETR_TH_HIGH downto DRV_RETR_TH_LOW)  <=  (OTHERS => '0');
    drv_bus(DRV_SELF_TEST_ENA_INDEX)                  <=  '0';
    drv_bus(DRV_ACK_FORB_INDEX)                       <=  '0';
    drv_bus(DRV_ENA_INDEX)                            <=  '1';
    drv_bus(DRV_FD_TYPE_INDEX)                        <=  drv_fd_type;


    -- CRCs are hardcoded, no need to check proper CRC Calculation. Only need
    -- to check that CRC is matching
    crc_15                     <= "101010101010101";
    crc_17                     <= "10101010101010101";
    crc_21                     <= "101010101010101010101";


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);


    ----------------------------------------------------------------------------
    -- Sampling signals generation
    ----------------------------------------------------------------------------
    sample_gen : process

        variable min_diff   :   natural := 4;
        --Note: With this setting there must be minnimally two clock cycles
        --      between the RX trig and TX trig. When 2 or 1 is set here
        --      test fails since gap between sample and sync is shorter
        --      than information processing time!!
        --      Note that official IPT is 4 clock cycles (3 between) thus
        --      in this test PC can do it with 3. One clock cycle is
        --      reserved and is of use in situations where both nodes
        --      are transmitting at the same time!

    begin
        generate_simple_trig(rnd_ctr_tr, tx_trigger_1, rx_trigger_1, clk_sys, min_diff);
    end process;


    ----------------------------------------------------------------------------
    -- Emulation of TXT Buffer. Generated frame is stored here.
    ----------------------------------------------------------------------------
    txtb_emul_proc : process
    begin
        wait until rising_edge(clk_sys);
        tran_word_1 <= txtb_mem(txtb_ptr_1);
        tran_identifier_1 <= txtb_mem(1)(28 downto 0);
    end process;


    ----------------------------------------------------------------------------
    -- Emulation of RX Buffer. Emulates storing protocol of RX Buffer. Note
    -- that no Error frames are used in this testbench, thus we don't need to
    -- follow "rec_abort".
    ----------------------------------------------------------------------------
    rxb_emul_proc : process
    begin
        while (res_n = C_RESET_POLARITY) loop
            wait until rising_edge(clk_sys);
        end loop;

        rxb_mem_ptr <= 0;
        wait until rising_edge(clk_sys) and (store_metadata_2 = '1');

        -- So far we leave out RWCNT, we store it at the end so that we don't
        -- have to decode it!
        rxb_mem(rxb_mem_ptr) <= "000000000000000000000" &
                                rec_esi_2 &
                                rec_brs_2 & '1' &
                                rec_frame_type_2 &
                                rec_ident_type_2 &
                                rec_is_rtr_2 & '0' &
                                rec_dlc_2;
        rxb_mem(rxb_mem_ptr + 1) <= "000" & rec_ident_2;

        -- No timestamp needed!
        rxb_mem(rxb_mem_ptr + 2) <= (OTHERS => '0');
        rxb_mem(rxb_mem_ptr + 3) <= (OTHERS => '0');

        rxb_mem_ptr <= rxb_mem_ptr + 4;

        -- We are sure that frame is received! If not maximum amount of received
        -- words is 16, break if this is exceeded!
        while true loop
            wait until rising_edge(clk_sys) and
                        ((store_data_2 = '1') or (rec_valid_2 = '1'));

            -- FInish if frame is received OK!
            if (rec_valid_2 = '1') then

                -- Finally store RWCNT here (we dont care when it is stored)!
                rxb_mem(0)(RWCNT_H downto RWCNT_L) <=
                    std_logic_vector(to_unsigned(rxb_mem_ptr - 1,
                                        RWCNT_H - RWCNT_L + 1));
                exit;
            end if;

            -- Break if there is more data than expected!
            check(rxb_mem_ptr <= 19, "Data size exceeds 64 bytes");
            if (rxb_mem_ptr > 19) then
                exit;
            end if;

            -- Store data word
            if (store_data_2 = '1') then
                rxb_mem(rxb_mem_ptr) <= store_data_word_2;
                rxb_mem_ptr          <= rxb_mem_ptr + 1;
            end if;
        end loop;

    end process;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
        variable sw_seq         : std_logic_vector(639 downto 0) :=
                                    (OTHERS => '0');
        variable rec_seq        : std_logic_vector(639 downto 0) :=
                                    (OTHERS => '0');
        variable seq_length     : natural;
        variable out_seq        : boolean;
        variable out_frm        : boolean := true;
        variable tx_frame       : SW_CAN_frame_type;
        variable rx_frame       : SW_CAN_frame_type;
        variable msg1           : line;
        variable msg2           : line;
        variable stf_length     : natural;
        variable rx_ptr         : natural := 0;
    begin
        info("Restarting Protocol control unit test!");
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 0, rand_ctr);
        info("Restarted Protocol control unit test");
        print_test_info(iterations, log_level, error_beh, error_tol);

        info("Wait till Integration is over!");
        for i in 0 to 10 loop
            wait until rising_edge(rx_trigger_1);
        end loop;

        -------------------------------
        -- Main loop of the test
        -------------------------------
        info("Starting Protocol control main loop");

        while (loop_ctr < iterations  or  exit_imm)
        loop
            info("Starting loop nr " & integer'image(loop_ctr));

            --------------------------------------------------------------------
            -- Erase SW bit sequence and recorded bit sequence
            --------------------------------------------------------------------
            sw_seq     := (OTHERS => '0');
            rec_seq    := (OTHERS => '0');

            --------------------------------------------------------------------
            -- Generate random frame and store it ot test memory which emulates
            -- TXT Buffer as selected by TX Arbitrator. Put metadata on inputs
            -- of the Frame.
            --------------------------------------------------------------------
            info("Generating frame for transmittion");
            CAN_generate_frame(rand_ctr, tx_frame);
            txtb_mem_ptr <= 0;
            wait for 0 ns;
            store_frame_to_test_mem(tx_frame, txtb_mem, txtb_mem_ptr);
            tran_dlc_1            <= tx_frame.dlc;
            tran_is_rtr_1         <= tx_frame.rtr;
            tran_ident_type_1     <= tx_frame.ident_type;
            tran_frame_type_1     <= tx_frame.frame_format;
            tran_brs_1            <= tx_frame.brs;

            --------------------------------------------------------------------
            -- Generate random settings for frame which are not inlcuded in
            -- frame itself:
            --      1. ISO vs NON-ISO FD CAN
            --      2. Bit stuffing counters
            --------------------------------------------------------------------
            rand_logic_s(rand_ctr, drv_fd_type, 0.5);
            rand_int_v(rand_ctr, 7, stf_length);
            dst_ctr_2 <= std_logic_vector(to_unsigned(stf_length, 3));
            dst_ctr_1 <= std_logic_vector(to_unsigned(stf_length, 3));
            bst_ctr_1 <= std_logic_vector(to_unsigned(stf_length, 3));
            bst_ctr_2 <= std_logic_vector(to_unsigned(stf_length, 3));
            wait for 0 ns;

            --------------------------------------------------------------------
            -- Calculate expected bitstream by SW model of CAN.
            --------------------------------------------------------------------
            info("Calculating expected frame by SW CAN");
            gen_sw_CAN(tx_frame, is_err_active_1, drv_fd_type, crc_15, crc_17, crc_21,
                        to_integer(unsigned(dst_ctr_1)), sw_seq, seq_length);

            --------------------------------------------------------------------
            -- Start transmitting by Protocol control 1
            --------------------------------------------------------------------
            info("Starting transmittion and recording on bus");
            wait until rx_trigger_1 = '1';
            tran_frame_valid_1 <= '1';

            --------------------------------------------------------------------
            -- Record what comes out of Protocol control 1, read frame which
            -- was received!
            --------------------------------------------------------------------
            record_bit_seq(bus_level, rx_trigger_1, seq_length, rec_seq);
            rx_ptr := 0;
            read_frame_from_test_mem(rx_frame, rxb_mem, rx_ptr);
            tran_frame_valid_1 <= '0';

            -- Compare results
            info("Comparing results");
            CAN_compare_frames(tx_frame, rx_frame, false, out_frm);
            compare_bit_seq(sw_seq, rec_seq, seq_length, out_seq);

            -- Print the frames in the end
            info("Generated CAN frame:");
            CAN_print_frame(tx_frame);

            info("Received CAN frame:");
            CAN_print_frame(rx_frame);

            info("Sequence length: " & to_string(seq_length));

            info("Expected bit sequence:");
            info(to_string(sw_seq(seq_length - 1 downto 0)));
            
            info("Received bit sequence:");
            info(to_string(rec_seq(seq_length - 1 downto 0)));

            -- Process possible error in TX/RX Frames or Bit sequences mismatch
            check(out_seq, "Bit sequence is not matching!");
            check(out_frm, "Received frame is not matching!");
            
            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

    errors <= error_ctr;

end architecture;