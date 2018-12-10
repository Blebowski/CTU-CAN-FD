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
--  Memory registers which control functionality of CAN FD core. Memory inter-
--  face is 32 bit avalon compatible. Registers create drv_bus signal which is
--  used in whole CAN FD IP function to control all modules. Memory Reads and
--  writes to any location need to be executed as one read, write. No extended
--  cycles are allowed.
--  Write to register as following:
--    1. SCS <= ACT_SCS, data_in <= valid_data, adress <= valid_adress
--    2. SWR <= ACT_SWR, wait at least one clock cycle
--    3. SWR <= not ACT_SWR SCS <= not ACT_SCS
--  Read from register as following:
--    1. SCS <= ACT_SCS, adress<=valid_adress
--    2. SRD <= ACT_SRD, wait at least one clock cycle
--    3. Capture valid data on data_out output
--    4. SRD <= not ACT_SRD, SCS <= not ACT_SCS
--------------------------------------------------------------------------------
-- Note: You must wait at least 1 cycle after deasserting async reset, since
--       a synchronous reset is performed the next cycle.
--------------------------------------------------------------------------------
--Note: All control signals which command any event execution which lasts one
--      clock cycle has negative edge detection. Therefore once srd or swr is
--      active to finish the read or write it has to become inactive!--
-- 2018-04-10 MJ: this looks untrue!
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--    19.12.2015  RETR register changed for settings register, added configura-
--                tion options for enabling and disabling whole controller, and
--                selecting ISO FD option. (Not yet implemented)
--    16.5.2016   Added restart function. Code formatting and constant replace-
--                ment
--    19.6.2016   Changed tx_data reg to be array 5*128 bits instead of 640
--                std_logic vector. This should ease the automatic inference
--                into RAM memory...
--    20.6.2016   Added ET bit in status register to monitor transmittion of
--                error frame!
--    21.6.2016   Fixed SETTINGS registers some of the bits were not read back
--                correctly
--    23.6.2016   Added DEBUG_REG for some additional debugging purposes
--    15.7.2016   Added "RX_buff_read_first" and "aux_data" signals. Changed han-
--                dling of moving to next word in RX buffer RX_DATA. Now first
--                cycle of memory access is detected, here and  "rx_read_start"
--                is set to active value for only one clock cycle! Even if bus
--                access lasts several clock cycles data output is captured only
--                in the first cycle and then held until the end of access.
--                Additionally "rx_read_start" signal is now combinationall, not
--                registered output. Thisway latency is shortened. Without this
--                precaution it was necessary to add empty cycles between reads
--                from RX_DATA!!!
--    24.8.2016   Added "use_logger" generic and LOG_EXIST bit to the LOG_STATUS
--                register to provide way how to find out from SW if logger is
--                actually present. Size is not deciding since HW developer can
--                set the size to e.g. 32 and use_logger to false!
--    1.9.2016    Moved SJW values to separate register! Now SJW has 4 bits
--                instead of two bits! This is compliant with CAN FD specifi-
--                cation.
--    30.11.2017  Changed implementation of TX_DATA registers. Registers removed
--                and access into these registers is now directly accessing RAM
--                in TXT buffer. Note that buffer must be first forbidden in
--                TX_SETTINGS register so that half written frame is not commi-
--                tted to CAN Core for transmission. Added BUF_DIR bit and remo-
--                ved TXT1_COMMIT and TXT2_COMMIT bits
--    12.12.2017  Renamed entity to  "canfd_registers" instead of "registers"
--                to avoid possible name conflicts.
--    20.12.2017  Removed obsolete tran_data_in signal. Removed obsolete
--                tx_data_reg. Added supoort for byte enable signal on register
--                writes and reads.
--    27.12.2017  Added "txt_frame_swap" bit for frame swapping after the
--                frame retransmission.
--    28.12.2017  Added support for "tx_time_suport" and Filter Status register.
--    18.01.2018  Removed txt1_disc, txt2_disc, txt1_commit and txt2_disc
--                obsolete signals
--    21.02.2018  Removed "txt_frame_swap" since it is not needed with new,
--                priority based implementation of TX Buffers.
--      2.6.2018  Removed "tx_time_suport".
--     29.7.2018  Removed "RX_buff_read_first" to have only single clock 
--                Avalon cycles available. Thus now there is no register
--                remaining which would require gap cycle between two cycles!
--                Burst reads are now supported!
--     9.12.2018  Replaced register implementation by Register map generated
--                by Register Map Generator. Two instances (Control Registers
--                and Event Logger Registers) are present. Connected register
--                modules to Driving and Status Bus. Added VERSION generics.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CANcomponents.all;
use work.CAN_FD_register_map.all;
use work.can_registers_pkg.all;

entity canfd_registers is
    generic(
        constant compType     :std_logic_vector(3 downto 0)    := CAN_COMPONENT_TYPE;

        -- Whenever event logger is present
        constant use_logger   :boolean                         := true;

        -- Optional synthesis of received message filters
        constant sup_filtA    :boolean                         := true;

        -- By default the behaviour is as if all the filters are present
        constant sup_filtB    :boolean                         := true;
        constant sup_filtC    :boolean                         := true;
        constant sup_range    :boolean                         := true;

        -- Support of byte enable signal on memory bus interface
        constant sup_be       :boolean                         := false;

        -- Number of TXT Buffers
        constant buf_count    :natural range 0 to 7            := 4;

        -- ID of the component
        constant ID           :natural                         := 1;

        -- DEVICE_ID (read from register)
        constant DEVICE_ID    :std_logic_vector(15 downto 0);

        -- MINOR Design version
        constant VERSION_MINOR : std_logic_vector(7 downto 0);

        -- MAJOR Design version
        constant VERSION_MAJOR : std_logic_vector(7 downto 0)
    );
    port(
        -- Clock and asynchronous reset
        signal clk_sys              :in   std_logic;
        signal res_n                :in   std_logic;
        signal res_out              :out  std_logic;

        ------------------------------------------------------------------------
        -- Memory Interface
        ------------------------------------------------------------------------
        signal data_in              :in   std_logic_vector(31 downto 0);
        signal data_out             :out  std_logic_vector(31 downto 0);
        signal adress               :in   std_logic_vector(23 downto 0);
        signal scs                  :in   std_logic;
        signal srd                  :in   std_logic;
        signal swr                  :in   std_logic;
        signal sbe                  :in   std_logic_vector(3 downto 0);
          
        -- Driving and Status Bus
        signal drv_bus              :out  std_logic_vector(1023 downto 0)
                                            := (OTHERS => '0');
        signal stat_bus             :in   std_logic_vector(511 downto 0);

        ------------------------------------------------------------------------
        -- RX Buffer Interface
        ------------------------------------------------------------------------

        -- Actually loaded data for reading
        signal rx_read_buff         :in   std_logic_vector(31 downto 0);

        -- Size of  message buffer (in words)
        signal rx_buf_size          :in   std_logic_vector(12 downto 0);

        -- Signal whenever buffer is full
        signal rx_full              :in   std_logic;

        -- Signal whenever buffer is empty
        signal rx_empty             :in   std_logic;

        -- Number of frames in recieve buffer
        signal rx_message_count     :in   std_logic_vector(10 downto 0);

        -- Number of free 32 bit wide ''windows''
        signal rx_mem_free          :in   std_logic_vector(12 downto 0);

        -- Position of read pointer
        signal rx_read_pointer_pos  :in   std_logic_vector(11 downto 0);

        -- Position of write pointer
        signal rx_write_pointer_pos :in   std_logic_vector(11 downto 0);
            
        -- Some data were discarded, register
        signal rx_data_overrun      :in   std_logic;

        ------------------------------------------------------------------------
        -- Optimized, direct interface to TXT1 and TXT2 buffers
        ------------------------------------------------------------------------

        -- Data and address for access to RAM of TXT Buffer
        signal tran_data            :out  std_logic_vector(31 downto 0);
        signal tran_addr            :out  std_logic_vector(4 downto 0);
        signal txtb_cs              :out  std_logic_vector(buf_count - 1 downto 0);

        -- Buffer status signals
        signal txtb_state           :in   txtb_state_type;

        -- Buffer commands + command index
        signal txt_sw_cmd           :out  txt_sw_cmd_type;
        signal txt_buf_cmd_index    :out  std_logic_vector(buf_count - 1 downto 0);
        signal txt_buf_prior_out    :out  txtb_priorities_type;
         
        ------------------------------------------------------------------------
        -- Bus synchroniser interface
        ------------------------------------------------------------------------
        signal trv_delay_out        :in   std_logic_vector(15 downto 0);

        ------------------------------------------------------------------------
        -- Event logger interface
        ------------------------------------------------------------------------
        signal loger_act_data       :in   std_logic_vector(63 downto 0);
        signal log_write_pointer    :in   std_logic_vector(7 downto 0);
        signal log_read_pointer     :in   std_logic_vector(7 downto 0);
        signal log_size             :in   std_logic_vector(7 downto 0);
        signal log_state_out        :in   logger_state_type;
            
        ------------------------------------------------------------------------
        -- Interrrupt Interface
        ------------------------------------------------------------------------
        signal int_vector           :in   std_logic_vector(INT_COUNT - 1 downto 0);
        signal int_ena              :in   std_logic_vector(INT_COUNT - 1 downto 0);
        signal int_mask             :in   std_logic_vector(INT_COUNT - 1 downto 0)
    );  
end entity;


architecture rtl of canfd_registers is

    -- Control registers output
    signal Control_registers_out    : Control_registers_out_t;

    -- Control registers input
    signal Control_registers_in     : Control_registers_in_t;

    -- Event Logger output
    signal Event_Logger_out         : Event_Logger_out_t;

    -- Event Logger input
    signal Event_Logger_in          : Event_Logger_in_t;

    -- Status register - combinational decoder
    signal status_comb              : std_logic_vector(7 downto 0);

    -- Padding for interrupt read data
    constant INT_PAD_H_IND          : natural :=
        Control_registers_in.int_stat'length - INT_COUNT;

    constant INT_PADDING            : std_logic_vector(INT_PAD_H_IND -1 downto 0) :=
        (OTHERS => '0');

    -- Main chip select signal
    signal can_core_cs                : std_logic;

    -- Chip select signals for each memory sub-block
    signal control_registers_cs       : std_logic;
    signal evnt_logger_cs             : std_logic;  
    signal control_registers_cs_reg   : std_logic;
    signal evnt_logger_cs_reg         : std_logic;

    -- Read data from register sub-modules
    signal control_registers_rdata    : std_logic_vector(31 downto 0);
    signal event_logger_rdata         : std_logic_vector(31 downto 0);

    -- Auxiliarly signals
    signal PC_state               :     protocol_type;
    signal PC_state_reg_vect      :     std_logic_vector(6 downto 0);
    signal error_state            :     error_state_type;
    signal OP_State               :     oper_mode_type;


    ---------------------------------------------------------------------------
    -- 
    ---------------------------------------------------------------------------
    function align_wrd_to_reg(
        reg_val         :   std_logic_vector;
        index           :   natural
    ) return std_logic is
    begin
        return reg_val(index mod reg_val'length);
    end function;

    ---------------------------------------------------------------------------
    --
    ---------------------------------------------------------------------------
    function align_wrd_to_reg(
        val             :   std_logic_vector;
        h_index         :   natural;
        l_index         :   natural
    ) return std_logic_vector is
        variable h_ind_mod  :  natural;
        variable l_ind_mod  :  natural;
    begin
        h_ind_mod := h_index mod val'length;
        l_ind_mod := l_index mod val'length;

        return val(h_ind_mod downto l_ind_mod); 
    end function;

    ---------------------------------------------------------------------------
    --
    ---------------------------------------------------------------------------    
    function align_reg_to_wrd(
        constant index          : in  natural;
        constant length         : in  natural
    ) return natural is
    begin
        return index mod length;
    end function;

begin
    
    ----------------------------------------------------------------------------
    -- Propagation of Avalon Data Bus to TXT Buffer RAM
    ----------------------------------------------------------------------------
    tran_data             <= data_in;

    ----------------------------------------------------------------------------
    -- Since TX_DATA registers are in separate region, which is word aligned,
    -- it is enough to take the lowest bits to create the address offset.
    ----------------------------------------------------------------------------
    tran_addr             <= adress(6 downto 2);
  
    ---------------------------------------------------------------------------
    -- TXT Buffer RAMs chip select signals.
    ---------------------------------------------------------------------------
    txtb_cs_gen : for i in 0 to TXT_BUFFER_COUNT - 1 generate
        type tx_buff_addr_type is array (0 to TXT_BUFFER_COUNT - 1) of
            std_logic_vector(3 downto 0);
        signal buf_addr : tx_buff_addr_type := (TX_BUFFER_1_BLOCK,
                                                TX_BUFFER_2_BLOCK,
                                                TX_BUFFER_3_BLOCK,
                                                TX_BUFFER_4_BLOCK);
    begin
        txtb_cs(i)       <= '1' when ((adress(11 downto 8) = buf_addr(i)) and
                                   scs='1' and swr='1')
                            else
                            '0';
    end generate txtb_cs_gen;

    can_core_cs <= '1' when (scs = ACT_CSC) and
                            (adress(COMP_TYPE_ADRESS_HIGHER downto
                                    COMP_TYPE_ADRESS_LOWER) = compType) and
                            (adress(ID_ADRESS_HIGHER downto ID_ADRESS_LOWER) =
                             std_logic_vector(to_unsigned(ID, 4)))
                       else
                   '0';


    ----------------------------------------------------------------------------
    -- Control registers chip select signals
    ----------------------------------------------------------------------------
    control_registers_cs <= '1' when (adress(11 downto 8) = CONTROL_REGISTERS_BLOCK)
                                      and (can_core_cs = '1')
                                else
                            '0';

    ----------------------------------------------------------------------------
    -- Event logger chip select signals
    ----------------------------------------------------------------------------
    evnt_logger_cs <= '1' when (adress(11 downto 8) = EVENT_LOGGER_BLOCK)
                                and (can_core_cs = '1')
                                else
                      '0';

    ----------------------------------------------------------------------------
    -- Registering control registers chip select
    ----------------------------------------------------------------------------
    chip_sel_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            control_registers_cs_reg  <= '0';
            evnt_logger_cs_reg        <= '0';
        elsif (rising_edge(clk_sys)) then
            control_registers_cs_reg  <= control_registers_cs;
            evnt_logger_cs_reg        <= evnt_logger_cs;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Read data multiplexor. Use registered version of chip select signals
    -- since read data are returned one clock cycle later!
    ----------------------------------------------------------------------------
    data_out <= control_registers_rdata when (control_registers_cs_reg = '1')
                                        else
                event_logger_rdata when (evnt_logger_cs_reg = '1')
                                   else
                (OTHERS => '0');

    ----------------------------------------------------------------------------
    -- Control registers instance
    ----------------------------------------------------------------------------
    control_registers_reg_map_comp : control_registers_reg_map
    generic map(
        DATA_WIDTH            => 32,
        ADDRESS_WIDTH         => 24,
        REGISTERED_READ       => true,
        CLEAR_READ_DATA       => true,
        RESET_POLARITY        => ACT_RESET,
        SUP_FILT_A            => sup_filtA,
        SUP_RANGE             => sup_range,
        SUP_FILT_C            => sup_filtC,
        SUP_FILT_B            => sup_filtB
    )
    port map(
        clk_sys               => clk_sys,
        res_n                 => res_n,
        address               => adress,
        w_data                => data_in,
        r_data                => control_registers_rdata,
        cs                    => control_registers_cs,
        read                  => srd,
        write                 => swr,
        be                    => sbe,
        control_registers_out => control_registers_out,
        control_registers_in  => control_registers_in
    );

    ----------------------------------------------------------------------------
    -- Event Logger registers instance - 
    -- Synthesized only when Logger should be synthesized! Otherwise all zeroes
    -- are returned. Note that reading LOG_EXIST bit by SW from LOG_STATUS
    -- register will return 0 (logger not present) even when all the registers
    -- are not instantiated, thus we can still use this approach!
    ----------------------------------------------------------------------------
    log_pres_gen : if (use_logger) generate
        event_logger_reg_map_comp : event_logger_reg_map
        generic map(
            DATA_WIDTH            => 32,
            ADDRESS_WIDTH         => 24,
            CLEAR_READ_DATA       => true,
            REGISTERED_READ       => true,
            RESET_POLARITY        => ACT_RESET
        )
        port map(
            clk_sys               => clk_sys,
            res_n                 => res_n,
            address               => adress,
            w_data                => data_in,
            r_data                => event_logger_rdata,
            cs                    => evnt_logger_cs,
            read                  => srd,
            write                 => swr,
            be                    => sbe,
            event_logger_out      => event_logger_out,
            event_logger_in       => event_logger_in
        );
    end generate log_pres_gen;

    ---------------------------------------------------------------------------
    -- When event logger is not present, read data are driven to zeroes,
    -- control signals for Event Logger are driven to 0!
    ---------------------------------------------------------------------------
    log_not_pres_gen : if (not use_logger) generate

        event_logger_rdata <= (OTHERS => '0');

        event_logger_out.log_trig_config <= (OTHERS => '0');
        event_logger_out.log_capt_config <= (OTHERS => '0');
        event_logger_out.log_command     <= (OTHERS => '0');

    end generate log_not_pres_gen;


    ----------------------------------------------------------------------------
    -- Reset propagation to output
    -- Note: this works only for reset active in logic zero
    ----------------------------------------------------------------------------
    res_out  <=  ACT_RESET when (res_n = ACT_RESET) else
                 ACT_RESET when (control_registers_out.mode(RST_IND) = '1') else
                 (not ACT_RESET);


    ----------------------------------------------------------------------------
    -- Extract Protocol control state from Status Bus
    ----------------------------------------------------------------------------
    PC_state <=  protocol_type'VAL(to_integer(unsigned(
                    stat_bus(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))));

    ----------------------------------------------------------------------------
    -- Extract Fault confinement state from Status Bus
    ----------------------------------------------------------------------------
    error_state <= error_state_type'VAL(
                    to_integer(unsigned(stat_bus(STAT_ERROR_STATE_HIGH downto
                                                 STAT_ERROR_STATE_LOW))));

    ----------------------------------------------------------------------------
    -- Extract Operation Control state from Status Bus
    ----------------------------------------------------------------------------
    OP_State <= oper_mode_type'VAL(to_integer(unsigned(
                                    stat_bus(STAT_OP_STATE_HIGH downto
                                             STAT_OP_STATE_LOW))));

    ---------------------------------------------------------------------------
    -- Status register - combinational decoder
    ---------------------------------------------------------------------------
    status_comb(IDLE_IND mod 8) <= '1' when (error_state = bus_off) else
                                   '1' when (OP_State = integrating) else
                                   '1' when (OP_State = idle) else
                                   '0';

    status_comb(EWL_IND mod 8) <= '1' when 
                                    (control_registers_out.ewl 
                                      < 
                                     stat_bus(STAT_TX_COUNTER_HIGH downto
                                              STAT_TX_COUNTER_LOW) or
                                    (control_registers_out.ewl 
                                      < 
                                     stat_bus(STAT_RX_COUNTER_HIGH downto
                                              STAT_RX_COUNTER_LOW)))
                                    else
                                  '0';
                                
    status_comb(TXS_IND mod 8) <= '1' when (OP_State = transciever) else
                                  '0';

    status_comb(RXS_IND mod 8) <= '1' when (OP_State = reciever) else
                                  '0';
  
    status_comb(TXNF_IND mod 8) <= '1' when (txtb_state(0) = TXT_ETY or
                                             txtb_state(1) = TXT_ETY or
                                             txtb_state(2) = TXT_ETY or
                                             txtb_state(3) = TXT_ETY)
                                       else
                                   '0';
  
    -- When at least one message is availiable in the buffer
    status_comb(RXNE_IND mod 8) <= not rx_empty;

    status_comb(DOR_IND mod 8) <= rx_data_overrun;

    status_comb(EFT_IND mod 8)  <= '1' when (PC_state = error)
                                       else
                                   '0';


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Control registers - Write Data to Driving Bus connection
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- MODE Register
    ---------------------------------------------------------------------------

    -- FDE - Flexible Data-Rate enabled
    drv_bus(DRV_CAN_FD_ENA_INDEX) <= align_wrd_to_reg(
        control_registers_out.mode, FDE_IND);

    -- RTRP - RTR Preffered behaviour
    drv_bus(DRV_RTR_PREF_INDEX) <= align_wrd_to_reg(
        control_registers_out.mode, RTRP_IND);

    -- Bus monitoring = listen only mode
    drv_bus(DRV_BUS_MON_ENA_INDEX) <= align_wrd_to_reg(
        control_registers_out.mode, LOM_IND);

    -- STM - Self test mode 
    drv_bus(DRV_SELF_TEST_ENA_INDEX) <= align_wrd_to_reg(
        control_registers_out.mode, STM_IND);

    -- AFM - Acceptance Filters mode
    drv_bus(DRV_FILTERS_ENA_INDEX) <= align_wrd_to_reg(
        control_registers_out.mode, AFM_IND);

    -- TSM - Tripple sampling mode
    drv_bus(DRV_SAM_INDEX) <= align_wrd_to_reg(
        control_registers_out.mode, TSM_IND);

    -- ACF - Acknowledge forbidden mode
    drv_bus(DRV_ACK_FORB_INDEX) <= align_wrd_to_reg(
        control_registers_out.mode, ACF_IND);


    ---------------------------------------------------------------------------
    -- COMMAND Register
    ---------------------------------------------------------------------------

    -- CDO - Clear data overrun Flag    
    drv_bus(DRV_CLR_OVR_INDEX) <= align_wrd_to_reg(
        control_registers_out.command, AFM_IND);
    
    -- ABT - Immediate transmission abort   
    drv_bus(DRV_ABORT_TRAN_INDEX) <= align_wrd_to_reg(
        control_registers_out.command, ABT_IND);

    -- RRB - Release Receive Buffer
    drv_bus(DRV_ERASE_RX_INDEX) <= align_wrd_to_reg(
        control_registers_out.command, RRB_IND);

    -- ERCRST - Error counter reset
    drv_bus(DRV_ERR_CTR_CLR) <= align_wrd_to_reg(
        control_registers_out.command, ERCRST_IND);

    -- RXFCRST - RX Frame counter reset
    drv_bus(DRV_CLR_RX_CTR_INDEX) <= align_wrd_to_reg(
        control_registers_out.command, RXFCRST_IND);

    -- TXFCRST - TX Frame counter reset
    drv_bus(DRV_CLR_TX_CTR_INDEX) <= align_wrd_to_reg(
        control_registers_out.command, TXFCRST_IND);


    ---------------------------------------------------------------------------
    -- SETTINGS Register
    ---------------------------------------------------------------------------

    -- RETR_LIM_ENA - Rettransmitt limit enable
    drv_bus(DRV_RETR_LIM_ENA_INDEX) <= align_wrd_to_reg(
        control_registers_out.settings, RTRLE_IND);

    -- RETR_TH - Rettransmitt limit threshold
    drv_bus(DRV_RETR_TH_HIGH downto DRV_RETR_TH_LOW) <= align_wrd_to_reg(
        control_registers_out.settings, RTRTH_H, RTRTH_L);

    -- ENA - CTU CAN FD Core enabled
    drv_bus(DRV_ENA_INDEX) <= align_wrd_to_reg(
        control_registers_out.settings, ENA_IND);
    
    -- NISOFD - Non - ISO FD Flag
    drv_bus(DRV_FD_TYPE_INDEX) <= align_wrd_to_reg(
        control_registers_out.settings, NISOFD_IND);

    -- INT_LOOPBACK - Acknowledge forbidden mode
    drv_bus(DRV_INT_LOOBACK_ENA_INDEX) <= align_wrd_to_reg(
        control_registers_out.settings, ILBP_IND);


    ---------------------------------------------------------------------------
    -- INT_STAT - Clearing interrupt vector by write
    ---------------------------------------------------------------------------

    -- Set of all Interrupt clears at the same time. We assume that vectors
    -- are addressed at LSB bits!
    drv_bus(DRV_INT_CLR_HIGH downto DRV_INT_CLR_LOW) <= align_wrd_to_reg(
        control_registers_out.int_stat, INT_COUNT - 1, 0);

    ---------------------------------------------------------------------------
    -- INT_ENA_SET - Interrupt enable set
    ---------------------------------------------------------------------------
    drv_bus(DRV_INT_ENA_SET_HIGH downto DRV_INT_ENA_SET_LOW) <= align_wrd_to_reg(
            control_registers_out.int_ena_set, INT_COUNT - 1, 0);


    ---------------------------------------------------------------------------
    -- INT_ENA_CLR - Interrupt enable clear
    ---------------------------------------------------------------------------
    drv_bus(DRV_INT_ENA_CLR_HIGH downto DRV_INT_ENA_CLR_LOW) <= align_wrd_to_reg(
            control_registers_out.int_ena_clr, INT_COUNT - 1, 0);

     
    ---------------------------------------------------------------------------
    -- INT_MASK_SET - Interrupt mask set
    ---------------------------------------------------------------------------
    drv_bus(DRV_INT_MASK_SET_HIGH downto DRV_INT_MASK_SET_LOW) <= align_wrd_to_reg(
            control_registers_out.int_mask_set, INT_COUNT - 1, 0);


    ---------------------------------------------------------------------------
    -- INT_MASK_CLR - Interrupt mask clear
    ---------------------------------------------------------------------------
    drv_bus(DRV_INT_MASK_CLR_HIGH downto DRV_INT_MASK_CLR_LOW) <= align_wrd_to_reg(
            control_registers_out.int_mask_clr, INT_COUNT - 1, 0);


    ---------------------------------------------------------------------------
    -- BTR - Bit Timing register, Nominal Bit-rate
    ---------------------------------------------------------------------------

    -- TQ_NBT - Time Quanta, Nominal Bit Time
    drv_bus(DRV_TQ_NBT_HIGH downto DRV_TQ_NBT_LOW) <= align_wrd_to_reg(
            control_registers_out.btr, BRP_H, BRP_L);

    -- PRS_NBT - Propagation segment, Nominal Bit Time
    drv_bus(DRV_PRS_NBT_HIGH downto DRV_PRS_NBT_LOW) <= align_wrd_to_reg(
            control_registers_out.btr, PROP_H, PROP_L);    

    -- PH1_NBT - Phase 1, Nominal Bit Time
    drv_bus(DRV_PH1_NBT_HIGH downto DRV_PH1_NBT_LOW) <= align_wrd_to_reg(
            control_registers_out.btr, PH1_H, PH1_L);

    -- PH2_NBT - Phase 2, Nominal Bit Time
    drv_bus(DRV_PH2_NBT_HIGH downto DRV_PH2_NBT_LOW) <= align_wrd_to_reg(
            control_registers_out.btr, PH2_H, PH2_L); 

    -- SJW_NBT - Synchronisation Jump Width, Nominal Bit Time
    drv_bus(DRV_SJW_HIGH downto DRV_SJW_LOW) <= align_wrd_to_reg(
            control_registers_out.btr, SJW_H, SJW_L); 


    ---------------------------------------------------------------------------
    -- BTR FD - Bit Timing register, Data Bit-rate
    ---------------------------------------------------------------------------

    -- TQ_NBT - Time Quanta, Nominal Bit Time
    drv_bus(DRV_TQ_DBT_HIGH downto DRV_TQ_DBT_LOW) <= align_wrd_to_reg(
            control_registers_out.btr_fd, BRP_FD_H, BRP_FD_L);

    -- PRS_NBT - Propagation segment, Nominal Bit Time
    drv_bus(DRV_PRS_DBT_HIGH downto DRV_PRS_DBT_LOW) <= align_wrd_to_reg(
            control_registers_out.btr_fd, PROP_FD_H, PROP_FD_L);    

    -- PH1_NBT - Phase 1, Nominal Bit Time
    drv_bus(DRV_PH1_DBT_HIGH downto DRV_PH1_DBT_LOW) <= align_wrd_to_reg(
            control_registers_out.btr_fd, PH1_FD_H, PH1_FD_L);

    -- PH2_NBT - Phase 2, Nominal Bit Time
    drv_bus(DRV_PH2_DBT_HIGH downto DRV_PH2_DBT_LOW) <= align_wrd_to_reg(
            control_registers_out.btr_fd, PH2_FD_H, PH2_FD_L); 

    -- SJW_NBT - Synchronisation Jump Width, Nominal Bit Time
    drv_bus(DRV_SJW_DBT_HIGH downto DRV_SJW_DBT_LOW) <= align_wrd_to_reg(
            control_registers_out.btr_fd, SJW_FD_H, SJW_FD_L); 


    ---------------------------------------------------------------------------
    -- EWL - Error warning limit
    ---------------------------------------------------------------------------
    drv_bus(DRV_EWL_HIGH downto DRV_EWL_LOW) <= align_wrd_to_reg(
            control_registers_out.ewl, EW_LIMIT_H, EW_LIMIT_L); 


    ---------------------------------------------------------------------------
    -- ERP - Error passive threshold
    ---------------------------------------------------------------------------
    drv_bus(DRV_ERP_HIGH downto DRV_ERP_LOW) <= align_wrd_to_reg(
            control_registers_out.erp, ERP_LIMIT_H, ERP_LIMIT_L); 


    ---------------------------------------------------------------------------
    -- CTR_PRES - Counter preset
    ---------------------------------------------------------------------------

    -- Counter preset value    
    drv_bus(DRV_CTR_VAL_HIGH downto DRV_CTR_VAL_LOW) <= align_wrd_to_reg(
            control_registers_out.ctr_pres, CTPV_H, CTPV_L); 
    
    -- Counter preset mask
    drv_bus(DRV_CTR_SEL_HIGH downto DRV_CTR_SEL_LOW) <= align_wrd_to_reg(
            control_registers_out.ctr_pres, EFD_IND, PTX_IND);


    ---------------------------------------------------------------------------
    -- FILTER_A_MASK 
    ---------------------------------------------------------------------------    
    drv_bus(DRV_FILTER_A_MASK_HIGH downto DRV_FILTER_A_MASK_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_a_mask, BIT_MASK_A_VAL_H, BIT_MASK_A_VAL_L); 

   
    ---------------------------------------------------------------------------
    -- FILTER_A_VAL
    ---------------------------------------------------------------------------
    drv_bus(DRV_FILTER_A_BITS_HIGH downto DRV_FILTER_A_BITS_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_a_val, BIT_VAL_A_VAL_H, BIT_VAL_A_VAL_L);


    ---------------------------------------------------------------------------
    -- FILTER_B_MASK 
    ---------------------------------------------------------------------------    
    drv_bus(DRV_FILTER_B_MASK_HIGH downto DRV_FILTER_B_MASK_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_b_mask, BIT_MASK_B_VAL_H, BIT_MASK_B_VAL_L); 

   
    ---------------------------------------------------------------------------
    -- FILTER_B_VAL
    ---------------------------------------------------------------------------
    drv_bus(DRV_FILTER_B_BITS_HIGH downto DRV_FILTER_B_BITS_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_b_val, BIT_VAL_B_VAL_H, BIT_VAL_B_VAL_L);


    ---------------------------------------------------------------------------
    -- FILTER_C_MASK 
    ---------------------------------------------------------------------------    
    drv_bus(DRV_FILTER_C_MASK_HIGH downto DRV_FILTER_C_MASK_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_c_mask, BIT_MASK_C_VAL_H, BIT_MASK_C_VAL_L); 

  
    ---------------------------------------------------------------------------
    -- FILTER_C_VAL
    ---------------------------------------------------------------------------
    drv_bus(DRV_FILTER_C_BITS_HIGH downto DRV_FILTER_C_BITS_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_c_val, BIT_VAL_C_VAL_H, BIT_VAL_C_VAL_L);


    ---------------------------------------------------------------------------
    -- FILTER_RAN_LOW
    ---------------------------------------------------------------------------
    drv_bus(DRV_FILTER_RAN_LO_TH_HIGH downto DRV_FILTER_RAN_LO_TH_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_ran_low, BIT_RAN_LOW_VAL_H, BIT_RAN_LOW_VAL_L);


    ---------------------------------------------------------------------------
    -- FILTER_RAN_HIGH
    ---------------------------------------------------------------------------
    drv_bus(DRV_FILTER_RAN_HI_TH_HIGH downto DRV_FILTER_RAN_HI_TH_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_ran_high, BIT_RAN_HIGH_VAL_H, BIT_RAN_HIGH_VAL_L);


    --------------------------------------------------------------------------
    -- FILTER_CONTROL
    ---------------------------------------------------------------------------

    -- Filter A Control
    drv_bus(DRV_FILTER_A_CTRL_HIGH downto DRV_FILTER_A_CTRL_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_control, FAFE_IND, FANB_IND);

    -- Filter B Control
    drv_bus(DRV_FILTER_B_CTRL_HIGH downto DRV_FILTER_B_CTRL_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_control, FBFE_IND, FBNB_IND);

    -- Filter C Control
    drv_bus(DRV_FILTER_C_CTRL_HIGH downto DRV_FILTER_C_CTRL_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_control, FCFE_IND, FCNB_IND);

    -- Filter Range Control
    drv_bus(DRV_FILTER_RAN_CTRL_HIGH downto DRV_FILTER_RAN_CTRL_LOW) <= align_wrd_to_reg(
            control_registers_out.filter_control, FRFE_IND, FRNB_IND);


    --------------------------------------------------------------------------
    -- RX_SETTINGS
    ---------------------------------------------------------------------------

    -- RTSOP - Receive Timestamp options
    drv_bus(DRV_RTSOPT_INDEX) <= align_wrd_to_reg(
        control_registers_out.rx_settings, RTSOP_IND);


    --------------------------------------------------------------------------
    -- RX_DATA
    ---------------------------------------------------------------------------

    -- Not writable, only read is signalled!
    drv_bus(DRV_READ_START_INDEX) <= control_registers_out.rx_data_read;


    --------------------------------------------------------------------------
    -- TX_COMMAND
    ---------------------------------------------------------------------------
    
    -- TX SW CMD - Set ready
    txt_sw_cmd.set_rdy <= align_wrd_to_reg(
        control_registers_out.tx_command, TXCR_IND);
   
    -- TX SW CMD - Set empty
    txt_sw_cmd.set_ety <= align_wrd_to_reg(
        control_registers_out.tx_command, TXCE_IND);

    -- TX SW CMD - Set abort
    txt_sw_cmd.set_abt <= align_wrd_to_reg(
        control_registers_out.tx_command, TXCA_IND);

    -- TXT Buffer command indices
    txt_buf_cmd_index(0) <= align_wrd_to_reg(
        control_registers_out.tx_command, TXB1_IND);

    txt_buf_cmd_index(1) <= align_wrd_to_reg(
        control_registers_out.tx_command, TXB2_IND);

    txt_buf_cmd_index(2) <= align_wrd_to_reg(
        control_registers_out.tx_command, TXB3_IND);

    txt_buf_cmd_index(3) <= align_wrd_to_reg(
        control_registers_out.tx_command, TXB4_IND);


    ---------------------------------------------------------------------------
    -- TX_PRIORITY
    ---------------------------------------------------------------------------

    -- TXT Buffer 1 priority
    txt_buf_prior_out(0) <= align_wrd_to_reg(
        control_registers_out.tx_priority, TXT1P_H, TXT1P_L);

    -- TXT Buffer 2 priority
    txt_buf_prior_out(1) <= align_wrd_to_reg(
        control_registers_out.tx_priority, TXT2P_H, TXT2P_L);

    -- TXT Buffer 3 priority
    txt_buf_prior_out(2) <= align_wrd_to_reg(
        control_registers_out.tx_priority, TXT3P_H, TXT3P_L);

    -- TXT Buffer 4 priority
    txt_buf_prior_out(3) <= align_wrd_to_reg(
        control_registers_out.tx_priority, TXT4P_H, TXT4P_L);


    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Control registers - Read Data to Status Bus connection
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- DEVICE_ID register
    ---------------------------------------------------------------------------
    Control_registers_in.device_id <= DEVICE_ID;

    
    ---------------------------------------------------------------------------
    -- VERSION register
    ---------------------------------------------------------------------------
    version_reg_block : block
        constant length : natural := Control_registers_in.version'length;
    begin
        -- Version minor
        Control_registers_in.version(
            align_reg_to_wrd(VER_MINOR_H, length) downto
            align_reg_to_wrd(VER_MINOR_L, length)) <=
            VERSION_MINOR;

        -- Version major
        Control_registers_in.version(
            align_reg_to_wrd(VER_MAJOR_H, length) downto
            align_reg_to_wrd(VER_MAJOR_L, length)) <=
            VERSION_MAJOR;
    end block version_reg_block;


    ---------------------------------------------------------------------------
    -- STATUS register
    ---------------------------------------------------------------------------
    Control_registers_in.status <= status_comb;


    ---------------------------------------------------------------------------
    -- INT_STAT register - reading interrupt vector
    ---------------------------------------------------------------------------
    Control_registers_in.int_stat <= INT_PADDING & int_vector;


    ---------------------------------------------------------------------------
    -- INT_ENA_SET register - reading interrupt enable
    ---------------------------------------------------------------------------
    Control_registers_in.int_ena_set <= INT_PADDING & int_ena;


    ---------------------------------------------------------------------------
    -- INT_MASK_SET register - reading interrupt mask
    ---------------------------------------------------------------------------
    Control_registers_in.int_mask_set <= INT_PADDING & int_mask;


    ---------------------------------------------------------------------------
    -- FAULT_STATE register - 
    ---------------------------------------------------------------------------
    fault_state_reg_block : block
        constant length : natural := Control_registers_in.fault_state'length;
    begin
        -- ERA field - Error active
        Control_registers_in.fault_state(align_reg_to_wrd(ERA_IND, length)) <=
            '1' when (error_state = error_active) else
            '0';

        -- ERP field - Error passive
        Control_registers_in.fault_state(align_reg_to_wrd(ERP_IND, length)) <=
            '1' when (error_state = error_passive) else
            '0';

        -- BOF field - Bus off
        Control_registers_in.fault_state(align_reg_to_wrd(BOF_IND, length)) <=
            '1' when (error_state = bus_off) else
            '0';

        -- Pad rest by zeroes
        Control_registers_in.fault_state(
            Control_registers_in.fault_state'length - 1 downto 3) <=
            (OTHERS => '0');
    end block fault_state_reg_block;


    ---------------------------------------------------------------------------
    -- RXC Register - Receive error counter
    ---------------------------------------------------------------------------
    rxc_reg_block : block
        constant length : natural := Control_registers_in.rxc'length;
    begin
        Control_registers_in.rxc(
            align_reg_to_wrd(RXC_VAL_H, length) downto
            align_reg_to_wrd(RXC_VAL_L, length)) <=
            "0000000" & stat_bus(STAT_RX_COUNTER_HIGH downto STAT_RX_COUNTER_LOW);
    end block rxc_reg_block;


    ---------------------------------------------------------------------------
    -- TXC Register - Transmitt error counter
    ---------------------------------------------------------------------------
    txc_reg_block : block
        constant length : natural := Control_registers_in.txc'length;
    begin
        Control_registers_in.txc(
            align_reg_to_wrd(TXC_VAL_H, length) downto
            align_reg_to_wrd(TXC_VAL_L, length)) <= 
            "0000000" & stat_bus(STAT_TX_COUNTER_HIGH downto STAT_TX_COUNTER_LOW);
    end block txc_reg_block;

    ---------------------------------------------------------------------------
    -- ERR_NORM - Error counter Nominal Bit-Rate
    ---------------------------------------------------------------------------
    err_norm_block : block
        constant length : natural := Control_registers_in.err_norm'length;
    begin
        Control_registers_in.err_norm(
            align_reg_to_wrd(ERR_NORM_VAL_H, length) downto
            align_reg_to_wrd(ERR_NORM_VAL_L, length)) <= 
            stat_bus(STAT_ERROR_COUNTER_NORM_HIGH downto STAT_ERROR_COUNTER_NORM_LOW);
    end block err_norm_block;


    ---------------------------------------------------------------------------
    -- ERR_FD - Error counter Nominal Data-Rate
    ---------------------------------------------------------------------------
    err_fd_block : block
        constant length : natural := Control_registers_in.err_fd'length;
    begin
        Control_registers_in.err_fd(
            align_reg_to_wrd(ERR_FD_VAL_H, length) downto
            align_reg_to_wrd(ERR_FD_VAL_L, length)) <= 
            stat_bus(STAT_ERROR_COUNTER_FD_HIGH downto STAT_ERROR_COUNTER_FD_LOW);
    end block err_fd_block;


    ---------------------------------------------------------------------------
    -- FILTER_STATUS
    ---------------------------------------------------------------------------
    filter_status_block : block
        constant length : natural := Control_registers_in.filter_status'length;
    begin
    
        -- SFA - Support Filter A -> yes
        sup_filt_A_gen : if (sup_filtA) generate
            Control_registers_in.filter_status(
                align_reg_to_wrd(SFA_IND, length)) <= '1';
        end generate sup_filt_A_gen;

        -- SFA - Support filter A -> no
        not_sup_filt_A_gen : if (not sup_filtA) generate
            Control_registers_in.filter_status(
                align_reg_to_wrd(SFA_IND, length)) <= '0';
        end generate not_sup_filt_A_gen;

        -- SFB - Support Filter B -> yes
        sup_filt_B_gen : if (sup_filtB) generate
            Control_registers_in.filter_status(
                align_reg_to_wrd(SFB_IND, length)) <= '1';
        end generate sup_filt_B_gen;

        -- SFB - Support filter B -> no
        not_sup_filt_B_gen : if (not sup_filtB) generate
            Control_registers_in.filter_status(
                align_reg_to_wrd(SFB_IND, length)) <= '0';
        end generate not_sup_filt_B_gen;

        -- SFC - Support Filter C -> yes
        sup_filt_C_gen : if (sup_filtC) generate
            Control_registers_in.filter_status(
                align_reg_to_wrd(SFC_IND, length)) <= '1';
        end generate sup_filt_C_gen;

        -- SFC - Support filter C -> no
        not_sup_filt_C_gen : if (not sup_filtC) generate
            Control_registers_in.filter_status(
                align_reg_to_wrd(SFC_IND, length)) <= '0';
        end generate not_sup_filt_C_gen;

        -- SFR - Support Filter Range -> yes
        sup_filt_range_gen : if (sup_range) generate
            Control_registers_in.filter_status(
                align_reg_to_wrd(SFR_IND, length)) <= '1';
        end generate sup_filt_range_gen;

        -- SFR - Support filter Range -> no
        not_sup_filt_range_gen : if (not sup_range) generate
            Control_registers_in.filter_status(
                align_reg_to_wrd(SFR_IND, length)) <= '0';
        end generate not_sup_filt_range_gen;

        -- Pad rest by zeroes
        Control_registers_in.filter_status(
            Control_registers_in.filter_status'length - 1 downto 4) <=
            (OTHERS => '0');

    end block filter_status_block;


    ---------------------------------------------------------------------------
    -- RX_MEM_INFO register
    ---------------------------------------------------------------------------
    rx_mem_info_block : block
        constant length : natural := Control_registers_in.rx_mem_info'length;
    begin

        -- RX_BUFF_SIZE field
        Control_registers_in.rx_mem_info(
            align_reg_to_wrd(RX_BUFF_SIZE_H, length) downto
            align_reg_to_wrd(RX_BUFF_SIZE_L, length)) <=
            rx_buf_size;

        -- RX_MEM_FREE field
        Control_registers_in.rx_mem_info(
            align_reg_to_wrd(RX_MEM_FREE_H, length) downto
            align_reg_to_wrd(RX_MEM_FREE_L, length)) <=
            rx_mem_free;

        -- Padd rest by zeroes
        Control_registers_in.rx_mem_info(31 downto 29) <= (OTHERS => '0');
        Control_registers_in.rx_mem_info(15 downto 13) <= (OTHERS => '0');

    end block rx_mem_info_block;


    ---------------------------------------------------------------------------
    -- RX_POINTERS register
    ---------------------------------------------------------------------------
    rx_pointers_block : block
        constant length : natural := Control_registers_in.rx_pointers'length;
    begin

        -- RX_WPP field - RX Write Pointer position
        Control_registers_in.rx_pointers(
            align_reg_to_wrd(RX_WPP_H, length) downto
            align_reg_to_wrd(RX_WPP_L, length)) <=
            rx_write_pointer_pos;

        -- RX_RPP field - RX Read Pointer position
        Control_registers_in.rx_pointers(
            align_reg_to_wrd(RX_RPP_H, length) downto
            align_reg_to_wrd(RX_RPP_L, length)) <=
            rx_read_pointer_pos;

        -- Padd rest by zeroes
        Control_registers_in.rx_pointers(31 downto 28) <= (OTHERS => '0');
        Control_registers_in.rx_pointers(15 downto 12) <= (OTHERS => '0');

    end block rx_pointers_block;


    ---------------------------------------------------------------------------
    -- RX_STATUS register
    ---------------------------------------------------------------------------
    rx_status_block : block
        constant length : natural := Control_registers_in.rx_status'length;
    begin

        -- RXE field - RX Buffer Empty field
        Control_registers_in.rx_status(
            align_reg_to_wrd(RXE_IND, length)) <=
            rx_empty;

        -- RXF field - RX Buffer Full field
        Control_registers_in.rx_status(
            align_reg_to_wrd(RXF_IND, length)) <=
            rx_full;

        -- RXFRC field - RX Buffer Frame count
        Control_registers_in.rx_status(
            align_reg_to_wrd(RXFRC_H, length) downto
            align_reg_to_wrd(RXFRC_L, length)) <=
            rx_message_count;

        -- Padd rest by zeroes
        Control_registers_in.rx_status(15) <= '0';
        Control_registers_in.rx_status(3 downto 2) <= (OTHERS => '0');

    end block rx_status_block;


    ---------------------------------------------------------------------------
    -- RX_DATA register - Read data word from RX Buffer FIFO.
    ---------------------------------------------------------------------------
    Control_registers_in.rx_data <= rx_read_buff;


    ---------------------------------------------------------------------------
    -- TX_STATUS register
    ---------------------------------------------------------------------------
    tx_status_block : block
        constant length : natural := Control_registers_in.tx_status'length;
    begin

        -- TX1S - TXT Buffer 1 status field
        Control_registers_in.tx_status(
            align_reg_to_wrd(TX1S_H, length) downto
            align_reg_to_wrd(TX1S_L, length)) <=
            txtb_state(0);
     
        -- TX2S - TXT Buffer 2 status field
        Control_registers_in.tx_status(
            align_reg_to_wrd(TX2S_H, length) downto
            align_reg_to_wrd(TX2S_L, length)) <=
            txtb_state(1);

        -- TX3S - TXT Buffer 3 status field
        Control_registers_in.tx_status(
            align_reg_to_wrd(TX3S_H, length) downto
            align_reg_to_wrd(TX3S_L, length)) <=
            txtb_state(2);

        -- TX4S - TXT Buffer 4 status field
        Control_registers_in.tx_status(
            align_reg_to_wrd(TX4S_H, length) downto
            align_reg_to_wrd(TX4S_L, length)) <=
            txtb_state(3);

    end block tx_status_block;


    ---------------------------------------------------------------------------
    -- ERR_CAPT register
    ---------------------------------------------------------------------------
    err_capt_block : block
        constant length : natural := Control_registers_in.err_capt'length;
    begin

        -- ERR_POS - Error position field
        Control_registers_in.err_capt(
            align_reg_to_wrd(ERR_POS_H, length) downto
            align_reg_to_wrd(ERR_POS_L, length)) <=
            stat_bus(STAT_ERC_ERR_POS_HIGH downto STAT_ERC_ERR_POS_LOW);

        -- ERR_TYPE - Error type field
        Control_registers_in.err_capt(
            align_reg_to_wrd(ERR_TYPE_H, length) downto
            align_reg_to_wrd(ERR_TYPE_L, length)) <=
            stat_bus(STAT_ERC_ERR_TYPE_HIGH downto STAT_ERC_ERR_TYPE_LOW);

    end block err_capt_block;


    ---------------------------------------------------------------------------
    -- ALC register
    ---------------------------------------------------------------------------
    alc_block : block
        constant length : natural := Control_registers_in.alc'length;
    begin
    
        -- ALC_ID_FIELD - Arbitration lost capture ID field
        Control_registers_in.alc(
            align_reg_to_wrd(ALC_ID_FIELD_H, length) downto
            align_reg_to_wrd(ALC_ID_FIELD_L, length)) <=
            stat_bus(STAT_ALC_ID_FIELD_HIGH downto STAT_ALC_ID_FIELD_LOW);

        -- ALC_ID_BIT - Arbitration lost capture bit position
        Control_registers_in.alc(
            align_reg_to_wrd(ALC_BIT_H, length) downto
            align_reg_to_wrd(ALC_BIT_L, length)) <=
            stat_bus(STAT_ALC_BIT_HIGH downto STAT_ALC_BIT_LOW);

    end block alc_block;


    ---------------------------------------------------------------------------
    -- TRV_DELAY register
    ---------------------------------------------------------------------------
    trv_delay_block : block
        constant length : natural := Control_registers_in.trv_delay'length;
    begin
    
        Control_registers_in.trv_delay(
            align_reg_to_wrd(TRV_DELAY_VALUE_H, length) downto
            align_reg_to_wrd(TRV_DELAY_VALUE_L, length)) <=
            trv_delay_out;
 
    end block trv_delay_block;


    ---------------------------------------------------------------------------
    -- RX_COUNTER register
    ---------------------------------------------------------------------------
    rx_counter_block : block
        constant length : natural := Control_registers_in.rx_counter'length;
    begin

        Control_registers_in.rx_counter(
            align_reg_to_wrd(RX_COUNTER_VAL_H, length) downto
            align_reg_to_wrd(RX_COUNTER_VAL_L, length)) <=
            stat_bus(STAT_RX_CTR_HIGH downto STAT_RX_CTR_LOW);

    end block rx_counter_block;


    ---------------------------------------------------------------------------
    -- TX_COUNTER register
    ---------------------------------------------------------------------------
    tx_counter_block : block
        constant length : natural := Control_registers_in.tx_counter'length;
    begin

        Control_registers_in.tx_counter(
            align_reg_to_wrd(TX_COUNTER_VAL_H, length) downto
            align_reg_to_wrd(TX_COUNTER_VAL_L, length)) <=
            stat_bus(STAT_TX_CTR_HIGH downto STAT_TX_CTR_LOW);

    end block tx_counter_block;


    ---------------------------------------------------------------------------
    -- DEBUG register
    ---------------------------------------------------------------------------
    debug_register_block : block
        constant length : natural := Control_registers_in.debug_register'length;
    begin

        -- STUFF_COUNT - Counter of stuffed bits modulo 8    
        Control_registers_in.debug_register(
            align_reg_to_wrd(STUFF_COUNT_H, length) downto
            align_reg_to_wrd(STUFF_COUNT_L, length)) <=
            stat_bus(STAT_BS_CTR_HIGH downto STAT_BS_CTR_LOW);

        -- DESTUFF_COUNT - Counter of de-stuffed bits modulo 8
        Control_registers_in.debug_register(
            align_reg_to_wrd(DESTUFF_COUNT_H, length) downto
            align_reg_to_wrd(DESTUFF_COUNT_L, length)) <=
            stat_bus(STAT_BD_CTR_HIGH downto STAT_BD_CTR_LOW);

        -- PC_ARB field - Protocol control FSM - arbitration field
        Control_registers_in.debug_register(
            align_reg_to_wrd(PC_ARB_IND, length)) <=
            '1' when PC_State = arbitration else '0';

        -- PC_CON field - Protocol control FSM - control field
        Control_registers_in.debug_register(
            align_reg_to_wrd(PC_CON_IND, length)) <=
            '1' when PC_State = control else '0';

        -- PC_DAT field - Protocol control FSM - data field
        Control_registers_in.debug_register(
            align_reg_to_wrd(PC_DAT_IND, length)) <=
            '1' when PC_State = data else '0';

        -- PC_CRC field - Protocol control FSM - CRC field
        Control_registers_in.debug_register(
            align_reg_to_wrd(PC_CRC_IND, length)) <=
            '1' when PC_State = crc else '0';

        -- PC_EOF field - Protocol control FSM - EOF field
        Control_registers_in.debug_register(
            align_reg_to_wrd(PC_EOF_IND, length)) <=
            '1' when PC_State = eof else '0';

        -- PC_OVR field - Protocol control FSM - Overload frame field
        Control_registers_in.debug_register(
            align_reg_to_wrd(PC_OVR_IND, length)) <=
            '1' when PC_State = overload else '0';

        -- PC_INT field - Protocol control FSM - Interframe frame field
        Control_registers_in.debug_register(
            align_reg_to_wrd(PC_INT_IND, length)) <=
            '1' when PC_State = interframe else '0';

        -- Pad rest by zeroes
        Control_registers_in.debug_register(31 downto 13) <= (OTHERS => '0');

    end block debug_register_block;


    ---------------------------------------------------------------------------
    -- YOLO register
    ---------------------------------------------------------------------------
    yolo_register_block : block
        constant length : natural := Control_registers_in.yolo_reg'length;
    begin

        Control_registers_in.yolo_reg(
            align_reg_to_wrd(YOLO_VAL_H, length) downto
            align_reg_to_wrd(YOLO_VAL_L, length)) <=
            YOLO_VAL_RSTVAL;

    end block yolo_register_block;


    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Event Logger - Write registers to Driving Bus Connection
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    ---------------------------------------------------------------------------
    -- LOG_TRIG_CONFIG - Logger triggering configuration
    ---------------------------------------------------------------------------

    -- T_SOF - Trigger on Start of frame
    drv_bus(DRV_TRIG_SOF_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_SOF_IND);

    -- T_ARBL - Trigger on Arbitration lost
    drv_bus(DRV_TRIG_ARB_LOST_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_ARBL_IND);

    -- T_REV - Trigger on RX Frame valid
    drv_bus(DRV_TRIG_REC_VALID_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_REV_IND);

    -- T_REV - Trigger on TX Frame valid
    drv_bus(DRV_TRIG_TRAN_VALID_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_TRV_IND);

    -- T_OVL - Trigger on Overload Frame
    drv_bus(DRV_TRIG_OVL_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_OVL_IND);

    -- T_ERR - Trigger on Error Frame
    drv_bus(DRV_TRIG_ERROR_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_ERR_IND);

    -- T_BRS - Trigger on Bit-rate shift
    drv_bus(DRV_TRIG_BRS_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_BRS_IND);

    -- T_USRW - Trigger on User write
    drv_bus(DRV_TRIG_USER_WRITE_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_USRW_IND);

    -- T_ARBS - Trigger on Arbitration field start
    drv_bus(DRV_TRIG_ARB_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_ARBS_IND);

    -- T_CTRS - Trigger on Control field start
    drv_bus(DRV_TRIG_CONTR_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_CTRS_IND);

    -- T_DATS - Trigger on Data field start
    drv_bus(DRV_TRIG_DATA_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_DATS_IND);

    -- T_CRCS - Trigger on CRC field start
    drv_bus(DRV_TRIG_CRC_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_CRCS_IND);

    -- T_ACKR - Trigger on ACK Received
    drv_bus(DRV_TRIG_ACK_REC_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_ACKR_IND);

    -- T_ACKNR - Trigger on ACK Not Received
    drv_bus(DRV_TRIG_ACK_N_REC_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_ACKNR_IND);

    -- T_EWLR - Trigger on Error Warning limit reached
    drv_bus(DRV_TRIG_EWL_REACHED_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_EWLR_IND);

    -- T_ERPC - Trigger on Error Passive changed
    drv_bus(DRV_TRIG_ERP_CHANGED_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_ERPC_IND);

    -- T_TRS - Trigger on Transmission started
    drv_bus(DRV_TRIG_TRAN_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_TRS_IND);

    -- T_RES - Trigger on Reception started
    drv_bus(DRV_TRIG_REC_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_trig_config, T_RES_IND);


    ---------------------------------------------------------------------------
    -- LOG_CAPT_CONFIG - Logger capture configuration
    ---------------------------------------------------------------------------

    -- C_SOF - Capture Start of Frame
    drv_bus(DRV_CAP_SOF_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_SOF_IND);

    -- C_ARBL - Capture Arbitration lost
    drv_bus(DRV_CAP_ARB_LOST_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_ARBL_IND);

    -- C_REV - Capture RX Frame valid
    drv_bus(DRV_CAP_REC_VALID_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_REV_IND);

    -- C_TRAN - Capture TX Frame valid
    drv_bus(DRV_CAP_TRAN_VALID_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_TRV_IND);

    -- C_OVL - Capture Overload Frame
    drv_bus(DRV_CAP_OVL_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_OVL_IND);

    -- C_ERR - Capture Error Frame
    drv_bus(DRV_CAP_ERROR_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_ERR_IND);

    -- C_BRS - Capture Bit-Rate shift Frame
    drv_bus(DRV_CAP_BRS_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_BRS_IND);

    -- C_ARBS - Capture Arbitration field start
    drv_bus(DRV_CAP_ARB_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_ARBS_IND);

    -- C_CTRS - Capture Control field start
    drv_bus(DRV_CAP_CONTR_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_CTRS_IND);

    -- C_DATS - Capture Data field start
    drv_bus(DRV_CAP_DATA_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_DATS_IND);

    -- C_CRCS - Capture CRC field start
    drv_bus(DRV_CAP_CRC_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_CRCS_IND);

    -- C_ACKR - Capture ACK received
    drv_bus(DRV_CAP_ACK_REC_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_ACKR_IND);

    -- C_ACKNR - Capture ACK received
    drv_bus(DRV_CAP_ACK_N_REC_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_ACKNR_IND);

    -- C_EWLR - Capture Error warning limit reached
    drv_bus(DRC_CAP_EWL_REACHED_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_EWLR_IND);

    -- C_ERC - Capture Error passive state changed
    drv_bus(DRV_CAP_ERP_CHANGED_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_ERC_IND);

    -- C_TRS - Capture Transmission started
    drv_bus(DRV_CAP_TRAN_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_TRS_IND);

    -- C_RES - Capture Reception started
    drv_bus(DRV_CAP_REC_START_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_RES_IND);

    -- C_SYNE - Capture Synchronisation edge event
    drv_bus(DRV_CAP_SYNC_EDGE_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_SYNE_IND);
    
    -- C_STUFF - Capture Stuff Bit insertion
    drv_bus(DRV_CAP_STUFFED_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_STUFF_IND);

    -- C_DESTUFF - Capture Stuff Bit discarded
    drv_bus(DRV_CAP_DESTUFFED_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_DESTUFF_IND);

    -- C_OVR - Capture Data Overrun
    drv_bus(DRV_CAP_OVR_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_capt_config, C_OVR_IND);


    ---------------------------------------------------------------------------
    -- LOG_COMMAND
    ---------------------------------------------------------------------------

    -- LOG_STR - Logger start
    drv_bus(DRV_LOG_CMD_STR_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_command, LOG_STR_IND);

    -- LOG_ABT - Logger Abort
    drv_bus(DRV_LOG_CMD_ABT_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_command, LOG_ABT_IND);

    -- LOG_UP - Logger Read Pointer UP
    drv_bus(DRV_LOG_CMD_UP_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_command, LOG_UP_IND);

    -- LOG_DOWN - Logger Read Pointer Down
    drv_bus(DRV_LOG_CMD_DOWN_INDEX) <= align_wrd_to_reg(
        Event_Logger_out.log_command, LOG_UP_IND);


    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Event Logger - Readable registers
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- LOG_STATUS register
    ---------------------------------------------------------------------------
    log_status_block : block
        constant length : natural := Event_Logger_in.log_status'length;
    begin

        -- LOG_CFG - Logger in config state
        Event_Logger_in.log_status(align_reg_to_wrd(LOG_CFG_IND, length)) <=
            '1' when (log_state_out = config)
                else
            '0';

        -- LOG_CFG - Logger in ready state
        Event_Logger_in.log_status(align_reg_to_wrd(LOG_RDY_IND, length)) <=
            '1' when (log_state_out = ready)
                else
            '0';

        -- LOG_RUN - Logger in running state
        Event_Logger_in.log_status(align_reg_to_wrd(LOG_RUN_IND, length)) <=
            '1' when (log_state_out = running)
                else
            '0';

        -- LOG_EXIST - Whether Event logger is supported
        log_exist_gen : if (use_logger) generate
            Event_Logger_in.log_status(
                align_reg_to_wrd(LOG_EXIST_IND, length)) <= '1';
        end generate log_exist_gen;

        log_not_exist_gen : if (not use_logger) generate
            Event_Logger_in.log_status(
                align_reg_to_wrd(LOG_EXIST_IND, length)) <= '0';
        end generate log_not_exist_gen;

        -- LOG_SIZE - Size of event logger memory
        Event_Logger_in.log_status(
            align_reg_to_wrd(LOG_SIZE_H, length) downto
            align_reg_to_wrd(LOG_SIZE_L, length)) <=
            log_size;

        -- Pad unused by zeroes
        Event_Logger_in.log_status(15 downto 8) <= (OTHERS => '0');
        Event_Logger_in.log_status(6 downto 3) <= (OTHERS => '0');

    end block log_status_block;


    ---------------------------------------------------------------------------
    -- LOG_POINTERS
    ---------------------------------------------------------------------------
    log_pointers_block : block
        constant length : natural := Event_Logger_in.log_pointers'length;
    begin

        -- LOG_RPP - Read pointer position
        Event_Logger_in.log_pointers(
            align_reg_to_wrd(LOG_RPP_H, length) downto
            align_reg_to_wrd(LOG_RPP_L, length)) <=
            log_read_pointer;

        -- LOG_WPP - Write pointer position
        Event_Logger_in.log_pointers(
            align_reg_to_wrd(LOG_WPP_H, length) downto
            align_reg_to_wrd(LOG_WPP_L, length)) <=
            log_write_pointer;

    end block log_pointers_block;


    ---------------------------------------------------------------------------
    -- LOG_CAPT_EVENT_1 - Read word from Logger RAM
    ---------------------------------------------------------------------------
    Event_Logger_in.log_capt_event_1 <= loger_act_data(63 downto 32);

    ---------------------------------------------------------------------------
    -- LOG_CAPT_EVENT_2 - Read word from Logger RAM
    ---------------------------------------------------------------------------
    Event_Logger_in.log_capt_event_2 <= loger_act_data(31 downto 0);

   
    ----------------------------------------------------------------------------
    -- Driving bus assignment
    ----------------------------------------------------------------------------
    --Note:  All unused signals indices should be assigned to zero!
    drv_bus(80 downto 61)                             <=  (OTHERS => '0');
    drv_bus(349 downto 330)                           <=  (OTHERS => '0');
    drv_bus(355 downto 354)                           <=  (OTHERS => '0');
    drv_bus(360 downto 358)                           <=  (OTHERS => '0');
    drv_bus(362 downto 361)                           <=  (OTHERS => '0');
    drv_bus(365 downto 363)                           <=  (OTHERS => '0');
    drv_bus(370 downto 368)                           <=  (OTHERS => '0');
    drv_bus(371)                                      <=  '0';
    drv_bus(375 downto 373)                           <=  (OTHERS => '0');
    drv_bus(399 downto 388)                           <=  (OTHERS => '0');
    drv_bus(459 downto 445)                           <=  (OTHERS => '0');
    drv_bus(464 downto 462)                           <=  (OTHERS => '0');
    drv_bus(609 downto 601)                           <=  (OTHERS => '0');
    drv_bus(579 downto 570)                           <=  (OTHERS => '0');
    drv_bus(519 downto 511)                           <=  (OTHERS => '0');
    drv_bus(506 downto 475)                           <=  (OTHERS => '0');
    drv_bus(444 downto 430)                           <=  (OTHERS => '0');

    drv_bus(1023 downto 876)                          <=  (OTHERS => '0');

    drv_bus(863 downto 844)                          <=  (OTHERS => '0');
    drv_bus(831 downto 812)                          <=  (OTHERS => '0');
    drv_bus(799 downto 780)                          <=  (OTHERS => '0');
    drv_bus(767 downto 748)                          <=  (OTHERS => '0');
    drv_bus(735 downto 614)                          <=  (OTHERS => '0');

    drv_bus(387 downto 376)                          <= (OTHERS => '0');
    drv_bus(367)                                     <= '0';
    drv_bus(357)                                     <= '0';

    -- TXT Buffer and TX Buffer (Obsolete)
    drv_bus(DRV_ERASE_TXT1_INDEX)                     <=  '0';
    drv_bus(DRV_ERASE_TXT2_INDEX)                     <=  '0';
   
    -- Obsolete - TODO: Clean-up
    drv_bus(DRV_TRIG_CONFIG_DATA_HIGH downto
            DRV_TRIG_CONFIG_DATA_LOW) <= (OTHERS => '0');

end architecture;
