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

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.ID_transfer.all;
use work.CAN_FD_frame_format.all;
use work.CAN_FD_register_map.all;
use STD.textio.all;
use IEEE.std_logic_textio.all;

architecture Protocol_Control_unit_test of CAN_test is

    ----------------------------------------------------------------------------
    -- Common Signals from/to PC1 and PC2
    ----------------------------------------------------------------------------
    signal clk_sys                  :  std_logic := '0';
    signal res_n                    :  std_logic := '0';

    -- Driving bus
    signal drv_bus                  :  std_logic_vector(1023 downto 0) :=
                                        (OTHERS => '0');

    -- CRC results
    signal crc15                    :  std_logic_vector(14 downto 0) :=
                                        (OTHERS => '0');
    signal crc17                    :  std_logic_vector(16 downto 0) :=
                                        (OTHERS => '0');
    signal crc21                    :  std_logic_vector(20 downto 0) :=
                                        (OTHERS => '0');

    ----------------------------------------------------------------------------
    -- Signals from/to PC1
    ----------------------------------------------------------------------------

    -- TX Metadata
    signal tran_dlc_1               :  std_logic_vector(3 downto 0) := "0000";
    signal tran_is_rtr_1            :  std_logic := '0';
    signal tran_ident_type_1        :  std_logic := '0';
    signal tran_frame_type_1        :  std_logic := '0';
    signal tran_brs_1               :  std_logic := '0';

    -- TX Buffer interface (Identifier word + data words)
    signal txt_data_word_1          :  std_logic_vector(31 downto 0) :=
                                         (OTHERS => '0');
    signal txt_buf_ptr_1            :  natural range 0 to 19;

    -- HW Command for TXT Buffers
    signal txt_hw_cmd_1             :  txt_hw_cmd_type := ('0', '0', '0',
                                                           '0', '0', '0');

    -- TXT Buffer source has changed.
    signal txtb_changed_1           :  std_logic;

    -- Frame on output of TX Arbitrator is valid and can be locked for
    -- transmission
    signal tran_frame_valid_in_1    :  std_logic := '0';

    -- RX metadata
    signal rec_dlc_1                :  std_logic_vector(3 downto 0);
    signal rec_is_rtr_1             :  std_logic;
    signal rec_ident_type_1         :  std_logic;
    signal rec_frame_type_1         :  std_logic;
    signal rec_brs_1                :  std_logic;
    signal rec_esi_1                :  std_logic;

    -- RX Buffer storing protocol
    signal store_metadata_1         :  std_logic;
    signal rec_abort_1              :  std_logic;
    signal store_data_1             :  std_logic;
    signal store_data_word_1        :  std_logic_vector(31 downto 0);

    -- RX Identifier
    signal rec_ident_1              :  std_logic_vector(28 downto 0);

    -- Recieved CRC value
    signal rec_crc_1                :  std_logic_vector(20 downto 0);

    -- Operation mode state
    signal OP_state_1               :  oper_mode_type;

    -- Signal for Operational mode state mahine about loosing arbitration
    signal arbitration_lost_1       :  std_logic;

    -- Signal to indicate transcieve or recieve finished and bus is idle
    signal is_idle_1                :  std_logic;

    -- Set OP_State FSM into transciever state (Used at SOF)
    signal set_transciever_1        :  std_logic;

    -- Set OP_State FSM into reciever state
    signal set_reciever_1           :  std_logic;

    -- Arbitration lost capture
    signal alc_1                    :  std_logic_vector(7 downto 0);

    -- Fault confinement state
    signal error_state_1            :  error_state_type;

    -- Form Error
    signal form_Error_1             :  std_logic;

    -- CRC Error
    signal CRC_Error_1              :  std_logic;

    -- Acknowledge error
    signal ack_Error_1              :  std_logic;

    -- Bit error is valid
    signal bit_Error_valid_1        :  std_logic;

    -- Stuff error is valid
    signal stuff_Error_valid_1      :  std_logic;

    -- Increment error counter by 1.
    signal inc_one_1                :  std_logic;

    -- Increment error counter by 8.
    signal inc_eight_1              :  std_logic;

    -- Decrement error counter by 1.
    signal dec_one_1                :  std_logic;

    -- Transmission was finished succesfully!
    signal tran_valid_1             :  std_logic;

    -- Frame was received succesfully
    signal rec_valid_1              :  std_logic;

    -- Acknowledge received
    signal ack_recieved_out_1       :  std_logic;

    -- Bit rate was shifted
    signal br_shifted_1             :  std_logic;

    -- Transcieved data on CAN Bus
    signal data_tx_1                :  std_logic;

    -- Bit stuffing enabled
    signal stuff_enable_1           :  std_logic;

    -- Fixed bit stuffing ('1') / Standard Bit stuffing ('0')
    signal fixed_stuff_1            :  std_logic;

    -- Length of bit stuffing rule
    signal stuff_length_1           :  std_logic_vector(2 downto 0);

    -- Enable bit destuffing
    signal destuff_enable_1         :  std_logic;

    -- Enable stuff error detection
    signal stuff_error_enable_1     :  std_logic;

    -- Fixed bit de-stuffing ('1') / Standard Bit de-stuffing ('0')
    signal fixed_destuff_1          :  std_logic;

    -- Length of bit destuffing rule
    signal destuff_length_1         :  std_logic_vector(2 downto 0);

    -- Bit destuffing counter mod 8
    signal dst_ctr_1                :  natural range 0 to 7;

    -- Transition from 0 to 1 erases the CRC and operation holds as
    -- long as "crc_enable = 1"
    signal crc_enable_1             :  std_logic;

    -- Synchronisation control (NO_SYNC, HARD_SYNC, RE_SYNC)
    signal sync_control_1           :  std_logic_vector(1 downto 0);

    -- Sample point control (NOMINAL_SAMPLE, DATA_SAMPLE, SECONDARY_SAMPLE)
    signal sp_control_1             :  std_logic_vector(1 downto 0);

    -- Clear Secondary sampling point shift register (in begining of
    -- Data phase)
    signal ssp_reset_1              :  std_logic;

    -- Calibration command for transciever delay compenstation
    signal trv_delay_calib_1        :  std_logic;

    -- Synchronisation edge validated by prescaler!!!
    signal hard_sync_edge_1         :  std_logic;

    -- Internal loopBack enabled (for Bus monitoring mode)
    signal int_loop_back_ena_1      :  std_logic;

    -- Unknown operational state
    signal unknown_OP_state_1       :  std_logic;

    -- Protocol state output.
    signal PC_State_out_1           :  protocol_type;

    -- Start of frame pulse
    signal sof_pulse_1              :  std_logic;


    ----------------------------------------------------------------------------
    -- Signals from/to PC2
    ----------------------------------------------------------------------------

    -- TX Metadata
    signal tran_dlc_2               :  std_logic_vector(3 downto 0) := "0000";
    signal tran_is_rtr_2            :  std_logic := '0';
    signal tran_ident_type_2        :  std_logic := '0';
    signal tran_frame_type_2        :  std_logic := '0';
    signal tran_brs_2               :  std_logic := '0';

    -- TX Buffer interface (Identifier word + data words)
    signal txt_data_word_2          :  std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
    signal txt_buf_ptr_2            :  natural range 0 to 19;

    -- TXT Buffer source has changed.
    signal txtb_changed_2           :  std_logic;

    -- HW Command for TXT Buffers
    signal txt_hw_cmd_2             :  txt_hw_cmd_type := ('0', '0', '0',
                                                           '0', '0', '0');

    -- Frame on output of TX Arbitrator is valid and can be locked for
    -- transmission
    signal tran_frame_valid_in_2    :  std_logic := '0';

    -- RX metadata
    signal rec_dlc_2                :  std_logic_vector(3 downto 0);
    signal rec_is_rtr_2             :  std_logic;
    signal rec_ident_type_2         :  std_logic;
    signal rec_frame_type_2         :  std_logic;
    signal rec_brs_2                :  std_logic;
    signal rec_esi_2                :  std_logic;

    signal store_metadata_2         :  std_logic;
    signal rec_abort_2              :  std_logic;
    signal store_data_2             :  std_logic;
    signal store_data_word_2        :  std_logic_vector(31 downto 0);

    -- RX Identifier
    signal rec_ident_2              :  std_logic_vector(28 downto 0);

    -- Recieved CRC value
    signal rec_crc_2                :  std_logic_vector(20 downto 0);

    -- Operation mode state
    signal OP_state_2               :  oper_mode_type;

    -- Signal for Operational mode state mahine about loosing arbitration
    signal arbitration_lost_2       :  std_logic;

    -- Signal to indicate transcieve or recieve finished and bus is idle
    signal is_idle_2                :  std_logic;

    -- Set OP_State FSM into transciever state (Used at SOF)
    signal set_transciever_2        :  std_logic;

    -- Set OP_State FSM into reciever state
    signal set_reciever_2           :  std_logic;

    -- Arbitration lost capture
    signal alc_2                    :  std_logic_vector(7 downto 0);

    -- Fault confinement state
    signal error_state_2            :  error_state_type;

    -- Form Error
    signal form_Error_2             :  std_logic;

    -- CRC Error
    signal CRC_Error_2              :  std_logic;

    -- Acknowledge error
    signal ack_Error_2              :  std_logic;

    -- Bit error is valid
    signal bit_Error_valid_2        :  std_logic;

    -- Stuff error is valid
    signal stuff_Error_valid_2      :  std_logic;


    -- Increment error counter by 1.
    signal inc_one_2                :  std_logic;

    -- Increment error counter by 8.
    signal inc_eight_2              :  std_logic;

    -- Decrement error counter by 1.
    signal dec_one_2                :  std_logic;

    -- Transmission was finished succesfully!
    signal tran_valid_2             :  std_logic;

    -- Frame was received succesfully
    signal rec_valid_2              :  std_logic;

    -- Acknowledge received
    signal ack_recieved_out_2       :  std_logic;

    -- Bit rate was shifted
    signal br_shifted_2             :  std_logic;

    -- Transcieved data on CAN Bus
    signal data_tx_2                :  std_logic;

    -- Bit stuffing enabled
    signal stuff_enable_2           :  std_logic;

    -- Fixed bit stuffing ('1') / Standard Bit stuffing ('0')
    signal fixed_stuff_2            :  std_logic;

    -- Length of bit stuffing rule
    signal stuff_length_2           :  std_logic_vector(2 downto 0);

    -- Enable bit destuffing
    signal destuff_enable_2         :  std_logic;

    -- Enable stuff error detection
    signal stuff_error_enable_2     :  std_logic;

    -- Fixed bit de-stuffing ('1') / Standard Bit de-stuffing ('0')
    signal fixed_destuff_2          :  std_logic;

    -- Length of bit destuffing rule
    signal destuff_length_2         :  std_logic_vector(2 downto 0);

    -- Bit destuffing counter mod 8
    signal dst_ctr_2                :  natural range 0 to 7;

    -- Transition from 0 to 1 erases the CRC and operation holds as
    -- long as "crc_enable = 1"
    signal crc_enable_2             :  std_logic;

    -- Synchronisation control (NO_SYNC, HARD_SYNC, RE_SYNC)
    signal sync_control_2           :  std_logic_vector(1 downto 0);

    -- Sample point control (NOMINAL_SAMPLE, DATA_SAMPLE, SECONDARY_SAMPLE)
    signal sp_control_2             :  std_logic_vector(1 downto 0);

    -- Clear Secondary sampling point shift register (in begining of
    -- Data phase)
    signal ssp_reset_2              :  std_logic;

    -- Calibration command for transciever delay compenstation
    signal trv_delay_calib_2        :  std_logic;

    -- Synchronisation edge validated by prescaler!!!
    signal hard_sync_edge_2         :  std_logic;

    -- Internal loopBack enabled (for Bus monitoring mode)
    signal int_loop_back_ena_2      :  std_logic;

    -- Unknown operational state
    signal unknown_OP_state_2       :  std_logic;

    -- Protocol state output.
    signal PC_State_out_2           :  protocol_type;

    -- Start of frame pulse
    signal sof_pulse_2              :  std_logic;


    ----------------------------------------------------------------------------
    -- Internal testbench signals
    ----------------------------------------------------------------------------

    -- Logic level on CAN bus
    signal bus_level                :  std_logic;

    -- Random pool pointer
    signal rnd_ctr_tr               :  natural range 0 to RAND_POOL_SIZE := 0;

    -- Transmitt trigger
    signal tx_trig                  :  std_logic := '0';

    -- Receive trigger
    signal rx_trig                  :  std_logic := '0';

    -- Previous value of "bus_level" for sync. edge generation.
    signal prev_bus                 :  std_logic;

    -- Fault confinement state.
    signal error_st                 :  error_state_type := error_active;

    -- TXT Buffer memory model
    signal txtb_mem                 :  test_mem_type :=
                                        (OTHERS => (OTHERS => '0'));
    signal txtb_mem_ptr             :  natural := 0;

    -- RX Buffer memory model
    signal rxb_mem                  :  test_mem_type :=
                                        (OTHERS => (OTHERS => '0'));
    signal rxb_mem_ptr              :  natural := 0;

    ------------------------
    -- Driving bus aliases
    ------------------------

    -- RTR behavior setting
    signal drv_rtr_pref             :     std_logic;

    -- Whenever FD Frames are supported for reciever
    signal drv_CAN_fd_ena           :     std_logic;

    -- Bus Monitoring mode enabled
    signal drv_bus_mon_ena          :     std_logic;

    -- Retransmition limit enabled for errornous frames
    signal drv_retr_lim_ena         :     std_logic;

    -- Retransmittion treshold
    signal drv_retr_th              :     std_logic_vector(3 downto 0);

    -- Self Test Mode enabled
    signal drv_self_test_ena        :     std_logic;

    -- Immediately abort transmittion
    signal drv_abort_tran           :     std_logic;

    -- Forbidding acknowledge mode
    signal drv_ack_forb             :     std_logic;

    -- Enabling whole controller
    signal drv_ena                  :     std_logic;

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
                report "Invalid stuff counter value" severity failure;
                -- LCOV_EXCL_STOP
        end case;

        parity := tmp(0) xor tmp(1) xor tmp(2);
        result := tmp;
    end procedure;


    ----------------------------------------------------------------------------
    -- SW model of Protocol control
    ----------------------------------------------------------------------------
    procedure gen_sw_CAN(
        constant   tx_frame       :in   SW_CAN_frame_type;
        constant   error_st       :in   error_state_type;
        constant   drv_iso_fd     :in   std_logic;
        constant   crc15          :in   std_logic_vector(14 downto 0);
        constant   crc17          :in   std_logic_vector(16 downto 0);
        constant   crc21          :in   std_logic_vector(20 downto 0);
        constant   stuff_cnt      :in   natural range 0 to 7;

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
            if (error_st = error_active) then
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
            if (error_st = error_active) then
                seq(ptr + 4) := DOMINANT;
            else
                seq(ptr + 4) := RECESSIVE;
            end if;
            ptr             := ptr + 5;
        when others =>
            -- LCOV_EXCL_START
            report "Invalid CAN FD Frame settings" severity error;
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
            tmp_crc        := crc15 & "000000";
        elsif (tx_frame.data_length <= 16) then
            crc_length     := 17;
            tmp_crc        := crc17 & "0000";
        else
            crc_length     := 21;
            tmp_crc        := crc21;
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

    protocol_control_1_comp : protocol_control
    port map(
        clk_sys               => clk_sys,
        res_n                 => res_n,
        drv_bus               => drv_bus,
        int_loop_back_ena     => int_loop_back_ena_1,
        PC_State_out          => PC_State_out_1,
        alc                   => alc_1,
        tran_data             => txt_data_word_1,
        tran_dlc              => tran_dlc_1,
        tran_is_rtr           => tran_is_rtr_1,
        tran_ident_type       => tran_ident_type_1,
        tran_frame_type       => tran_frame_type_1,
        tran_brs              => tran_brs_1,
        txt_buf_ptr           => txt_buf_ptr_1,
        tran_frame_valid_in   => tran_frame_valid_in_1,
        txt_hw_cmd            => txt_hw_cmd_1,
        txtb_changed          => txtb_changed_1,
        br_shifted            => br_shifted_1,
        rec_ident             => rec_ident_1,
        rec_dlc               => rec_dlc_1,
        rec_is_rtr            => rec_is_rtr_1,
        rec_ident_type        => rec_ident_type_1,
        rec_frame_type        => rec_frame_type_1,
        rec_brs               => rec_brs_1,
        rec_crc               => rec_crc_1,
        rec_esi               => rec_esi_1,
        store_metadata        => store_metadata_1,
        rec_abort             => rec_abort_1,
        store_data            => store_data_1,
        store_data_word       => store_data_word_1,
        OP_state              => OP_state_1,
        arbitration_lost      => arbitration_lost_1,
        is_idle               => is_idle_1,
        set_transciever       => set_transciever_1,
        set_reciever          => set_reciever_1,
        ack_recieved_out      => ack_recieved_out_1,
        error_state           => error_state_1,
        form_Error            => form_Error_1,
        CRC_Error             => CRC_Error_1,
        ack_Error             => ack_Error_1,
        bit_Error_valid       => bit_Error_valid_1,
        stuff_Error_valid     => stuff_Error_valid_1,
        unknown_OP_state      => unknown_OP_state_1,
        inc_one               => inc_one_1,
        inc_eight             => inc_eight_1,
        dec_one               => dec_one_1,
        tran_valid            => tran_valid_1,
        rec_valid             => rec_valid_1,
        tran_trig             => tx_trig,
        rec_trig              => rx_trig,
        data_tx               => data_tx_1,
        stuff_enable          => stuff_enable_1,
        fixed_stuff           => fixed_stuff_1,
        stuff_length          => stuff_length_1,
        data_rx               => bus_level,
        destuff_enable        => destuff_enable_1,
        stuff_error_enable    => stuff_error_enable_1,
        fixed_destuff         => fixed_destuff_1,
        destuff_length        => destuff_length_1,
        dst_ctr               => dst_ctr_1,
        crc_enable            => crc_enable_1,
        crc15                 => crc15,
        crc17                 => crc17,
        crc21                 => crc21,
        sync_control          => sync_control_1,
        sp_control            => sp_control_1,
        ssp_reset             => ssp_reset_1,
        trv_delay_calib       => trv_delay_calib_1,
        hard_sync_edge        => hard_sync_edge_1,
        sof_pulse             => sof_pulse_1
      );


    protocol_control_2_comp : protocol_control
    port map(
        clk_sys               => clk_sys,
        res_n                 => res_n,
        drv_bus               => drv_bus,
        int_loop_back_ena     => int_loop_back_ena_2,
        PC_State_out          => PC_State_out_2,
        alc                   => alc_2,
        tran_data             => txt_data_word_2,
        tran_dlc              => tran_dlc_2,
        tran_is_rtr           => tran_is_rtr_2,
        tran_ident_type       => tran_ident_type_2,
        tran_frame_type       => tran_frame_type_2,
        tran_brs              => tran_brs_2,
        txt_buf_ptr           => txt_buf_ptr_2,
        tran_frame_valid_in   => tran_frame_valid_in_2,
        txt_hw_cmd            => txt_hw_cmd_2,
        txtb_changed          => txtb_changed_2,
        br_shifted            => br_shifted_2,
        rec_ident             => rec_ident_2,
        rec_dlc               => rec_dlc_2,
        rec_is_rtr            => rec_is_rtr_2,
        rec_ident_type        => rec_ident_type_2,
        rec_frame_type        => rec_frame_type_2,
        rec_brs               => rec_brs_2,
        rec_crc               => rec_crc_2,
        rec_esi               => rec_esi_2,
        store_metadata        => store_metadata_2,
        rec_abort             => rec_abort_2,
        store_data            => store_data_2,
        store_data_word       => store_data_word_2,
        OP_state              => OP_state_2,
        arbitration_lost      => arbitration_lost_2,
        is_idle               => is_idle_2,
        set_transciever       => set_transciever_2,
        set_reciever          => set_reciever_2,
        ack_recieved_out      => ack_recieved_out_2,
        error_state           => error_state_2,
        form_Error            => form_Error_2,
        CRC_Error             => CRC_Error_2,
        ack_Error             => ack_Error_2,
        bit_Error_valid       => bit_Error_valid_2,
        stuff_Error_valid     => stuff_Error_valid_2,
        unknown_OP_state      => unknown_OP_state_2,
        inc_one               => inc_one_2,
        inc_eight             => inc_eight_2,
        dec_one               => dec_one_2,
        tran_valid            => tran_valid_2,
        rec_valid             => rec_valid_2,
        tran_trig             => tx_trig,
        rec_trig              => rx_trig,
        data_tx               => data_tx_2,
        stuff_enable          => stuff_enable_2,
        fixed_stuff           => fixed_stuff_2,
        stuff_length          => stuff_length_2,
        data_rx               => bus_level,
        destuff_enable        => destuff_enable_2,
        stuff_error_enable    => stuff_error_enable_2,
        fixed_destuff         => fixed_destuff_2,
        destuff_length        => destuff_length_2,
        dst_ctr               => dst_ctr_2,
        crc_enable            => crc_enable_2,
        crc15                 => crc15,
        crc17                 => crc17,
        crc21                 => crc21,
        sync_control          => sync_control_2,
        sp_control            => sp_control_2,
        ssp_reset             => ssp_reset_2,
        trv_delay_calib       => trv_delay_calib_2,
        hard_sync_edge        => hard_sync_edge_2,
        sof_pulse             => sof_pulse_2
      );


    ----------------------------------------------------------------------------
    -- Creating bus level
    ----------------------------------------------------------------------------
    bus_level             <= data_tx_2 AND data_tx_1;


    ----------------------------------------------------------------------------
    -- Driving bus aliases (equal for both nodes)
    ----------------------------------------------------------------------------
    drv_bus(DRV_RTR_PREF_INDEX)                       <=  drv_rtr_pref;
    drv_bus(DRV_CAN_FD_ENA_INDEX)                     <=  drv_CAN_fd_ena;
    drv_bus(DRV_BUS_MON_ENA_INDEX)                    <=  drv_bus_mon_ena;
    drv_bus(DRV_RETR_LIM_ENA_INDEX)                   <=  drv_retr_lim_ena;
    drv_bus(DRV_RETR_TH_HIGH downto DRV_RETR_TH_LOW)  <=  drv_retr_th;
    drv_bus(DRV_SELF_TEST_ENA_INDEX)                  <=  drv_self_test_ena;
    drv_bus(DRV_ABORT_TRAN_INDEX)                     <=  drv_abort_tran;
    drv_bus(DRV_ACK_FORB_INDEX)                       <=  drv_ack_forb;
    drv_bus(DRV_ENA_INDEX)                            <=  drv_ena;
    drv_bus(DRV_FD_TYPE_INDEX)                        <=  drv_fd_type;

    ----------------------------------------------------------------------------
    -- Some configurations are extra
    ----------------------------------------------------------------------------
    drv_rtr_pref                                      <= '0';
    drv_CAN_fd_ena                                    <= '1';
    drv_bus_mon_ena                                   <= '0';
    drv_retr_lim_ena                                  <= '0';
    drv_retr_th                                       <= (OTHERS => '0');
    drv_self_test_ena                                 <= '0';
    drv_abort_tran                                    <= '0';
    drv_ack_forb                                      <= '0';
    drv_ena                                           <= '1';

    ----------------------------------------------------------------------------
    -- Following test the protocol control properly
    ----------------------------------------------------------------------------
    OP_state_2                <= reciever;
    OP_state_1                <= transciever;
    error_state_2             <= error_active;
    error_state_1             <= error_active;
    bit_Error_valid_1         <= '0';
    bit_Error_valid_2         <= '0';
    stuff_Error_valid_2       <= '0';
    stuff_Error_valid_2       <= '0';

    -- CRCs are hardcoded, no need to check proper CRC Calculation. Only need
    -- to check that CRC is matching
    crc15                     <= "101010101010101";
    crc17                     <= "10101010101010101";
    crc21                     <= "101010101010101010101";


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
        generate_simple_trig(rnd_ctr_tr, tx_trig, rx_trig, clk_sys, min_diff);
    end process;


    ----------------------------------------------------------------------------
    -- Hard sync edge generation
    --  emulates the bus_sync
    ----------------------------------------------------------------------------
    hs_edge_gen : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            hard_sync_edge_2 <= '0';
            hard_sync_edge_1 <= '0';
            prev_bus         <= RECESSIVE;
        elsif rising_edge(clk_sys) then

            prev_bus <= bus_level;

            if (prev_bus /= bus_level and bus_level = DOMINANT) then
                hard_sync_edge_2 <= '1';
                hard_sync_edge_1 <= '1';
            else
                hard_sync_edge_2 <= '0';
                hard_sync_edge_1 <= '0';
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Emulation of TXT Buffer. Generated frame is stored here.
    ----------------------------------------------------------------------------
    txtb_emul_proc : process
    begin
        wait until rising_edge(clk_sys);
        txt_data_word_1 <= txtb_mem(txt_buf_ptr_1);
    end process;


    ----------------------------------------------------------------------------
    -- Emulation of RX Buffer. Emulates storing protocol of RX Buffer. Note
    -- that no Error frames are used in this testbench, thus we don't need to
    -- follow "rec_abort".
    ----------------------------------------------------------------------------
    rxb_emul_proc : process
    begin
        while (res_n = ACT_RESET) loop
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
            if (rxb_mem_ptr > 19) then
                -- LCOV_EXCL_START
                log("Data size exceeds 64 bytes", error_l, log_level);
                exit;
                -- LCOV_EXCL_STOP
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
        log("Restarting Protocol control unit test!", info_l, log_level);
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 0, rand_ctr);
        log("Restarted Protocol control unit test", info_l, log_level);
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        log("Starting Protocol control main loop", info_l, log_level);

        while (loop_ctr < iterations  or  exit_imm)
        loop
            log("Starting loop nr " & integer'image(loop_ctr), info_l,
                log_level);

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
            log("Generating frame for transmittion", info_l, log_level);
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
            dst_ctr_2 <= stf_length;
            dst_ctr_1 <= stf_length;
            wait for 0 ns;

            --------------------------------------------------------------------
            -- Calculate expected bitstream by SW model of CAN.
            --------------------------------------------------------------------
            log("Calculating expected frame by SW CAN", info_l, log_level);
            gen_sw_CAN(tx_frame, error_st, drv_fd_type, CRC15, CRC17, CRC21,
                        dst_ctr_1, sw_seq, seq_length);

            --------------------------------------------------------------------
            -- Start transmitting by Protocol control 1
            --------------------------------------------------------------------
            log("Starting transmittion and recording on bus", info_l, log_level);
            tran_frame_valid_in_1 <= '1';

            --------------------------------------------------------------------
            -- Record what comes out of Protocol control 1, read frame which
            -- was received!
            --------------------------------------------------------------------
            record_bit_seq(bus_level, rx_trig, seq_length, rec_seq);
            rx_ptr := 0;
            read_frame_from_test_mem(rx_frame, rxb_mem, rx_ptr);
            tran_frame_valid_in_1 <= '0';

            -- Compare results
            log("Comparing results", info_l, log_level);
            CAN_compare_frames(tx_frame, rx_frame, false, out_frm);
            compare_bit_seq(sw_seq, rec_seq, seq_length, out_seq);

            -- Process possible error in TX/RX Frames or sequences mismatch
            if (out_seq = false or out_frm = false) then
                -- LCOV_EXCL_START
                log("Generated CAN frame:", error_l, log_level);
                CAN_print_frame(tx_frame, info_l);

                log("Receied CAN frame:", error_l, log_level);
                CAN_print_frame(rx_frame, info_l);

                log("Expected bit sequence:", error_l, log_level);
                write(msg1, sw_seq(seq_length - 1 downto 0));
                writeline(output, msg1);

                log("Received bit sequence:", error_l, log_level);
                write(msg2, rec_seq(seq_length - 1 downto 0));
                writeline(output, msg2);

                process_error(error_ctr, error_beh, exit_imm);
                -- LCOV_EXCL_STOP
                wait;
            end if;

            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

    errors <= error_ctr;

end architecture;
