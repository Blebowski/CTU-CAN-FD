/*
 * CAN with Flexible Data-Rate IP Core 
 * Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 *  ************************
 *  ** CAN_FD_IP_driver.h **
 *  ************************
 *
 *  Created on: 	10.8.2016
 *      Author: 	Ondrej Ille , Czech Technical University, FEL
 * 	   Project:     CAN FD IP Core Project
 *
 *  Revision History Date Author Comments:
 *    10.8.2016   Created file
 *
 */

#include "sys_common.h"

#ifndef CAN_FD_IP_DRIVER_H_
#define CAN_FD_IP_DRIVER_H_

	/********************
	 ********************
	 * MACROS definitions
	 ********************
	 ********************/

	//Registers memory offset
	#define IDENT_REG					0x0
	#define MODE_REG					0x4
	#define INTERRUPT_REG				0x8
	#define TIMING_REG					0xC
	#define ARB_PRESC_REG				0x10
	#define ERROR_TH_REG				0x14
	#define ERROR_COUNTERS_REG			0x18
	#define ERROR_COUNTERS_SPEC_REG		0x1C
	#define FILTER_A_MASK_REG			0x20
	#define FILTER_A_VAL_REG			0x24
	#define FILTER_B_MASK_REG			0x28
	#define FILTER_B_VAL_REG			0x2C
	#define FILTER_C_MASK_REG			0x30
	#define FILTER_C_VAL_REG			0x34
	#define FILTER_RAN_LOW_REG			0x38
	#define FILTER_RAN_HIGH_REG			0x3C
	#define FILTER_CONTROL_REG			0x40
	#define RX_INFO_1_REG				0x44
	#define RX_INFO_2_REG				0x48
	#define RX_DATA_REG					0x4C
	#define TRV_DELAY_REG				0x50
	#define TX_STATUS_REG				0x54
	#define TX_SETTINGS_REG				0x58
	#define TX_DATA_1_REG				0x5C
	#define TX_DATA_2_REG				0x60
	#define TX_DATA_3_REG				0x64
	#define TX_DATA_4_REG				0x68
	#define TX_DATA_5_REG				0x6C
	#define RX_COUNTER_REG				0xAC
	#define TX_COUNTER_REG				0xB0
	#define LOG_TRIG_CONFIG_REG			0xB8
	#define LOG_CAPT_CONFIG_REG			0xC0
	#define LOG_STATUS_REG				0xC4
	#define LOG_COMMAND_REG				0xC8
	#define LOG_CAPT_EVENT_1_REG		0xCC
	#define LOG_CAPT_EVENT_2_REG		0xD0
	#define DEBUG_REG					0xD4
	#define YOLO_REG					0xD8


	/*
	 * Mode register bits
	 */

	//MODE
	#define RST_BIT						0		//Reset
	#define LOM_BIT						1		//Listen only mode
	#define STM_BIT						2		//Self-test mode
	#define AFM_BIT						3		//Acceptance filters mode
	#define FDE_BIT						4		//Flexible data-rate bit
	#define RTRP_BIT					5		//RTR Preffered behaviour
	#define TSM_BIT						6		//Tripple sampling mode
	#define ACF_BIT						7		//Acknowledge forbidden mode

	#define MODE_BYTE					1

	//COMMAND
	#define AT_BIT						9		//Abort transmittion
	#define RRB_BIT						10		//Release receive buffer
	#define CDO_BIT						11		//Clear data overrun

	#define COMMAND_BYTE				2

	//STATUS
	#define RBS_BIT						16		//Receive buffer status
	#define DOS_BIT						17		//Data ovverun status
	#define TBS_BIT						18		//Transmitt buffer status
	#define ET_BIT						19		//Error frame transmitted
	#define RS_BIT						20		//Recieve status
	#define TS_BIT						21		//Transmitt status
	#define ES_BIT						22		//Error warning limit reached
	#define BS_BIT						23		//Bus status

	#define STATUS_BYTE					3

	//SETTINGS
	#define RTRLE_BIT					24		//Retransmitt limit enable
	#define RTR_TH_LOW					25		//Retransmitt limit value lowest bit
	#define RTR_TH_HIGH					28		//Retransmitt limit value highest bit
	#define INT_LOOP_BIT				29		//Internal loopback bit
	#define ENA_BIT						30		//Enable bit
	#define FD_TYPE_BIT					31		//Flexible data-rate type

	#define SETTINGS_BYTE				4

	/*
	 * Interrupt register bits
	 */

	//INT
	#define RI_BIT						0		//Recieve interrupt
	#define TI_BIT						1		//Transmitt interrupt
	#define EI_BIT						2		//Error warning limit reached interrupt
	#define DOI_BIT						3		//Data overrun interrupt
	#define EPI_BIT						5		//Error passive interrupt
	#define ALI_BIT						6		//Arbitration lost interrupt
	#define BEI_BIT						7		//Bus error interrupt
	#define LFI_BIT						8		//Event logging finished interrupt
	#define RFI_BIT						9		//Recieve buffer full interrupt
	#define BSI_BIT						10		//Bit rate shifted interrupt

	//INT_ENA
	#define RI_ENA_BIT					16		//Recieve interrupt enable
	#define TI_ENA_BIT					17		//Transmitt interrupt enable
	#define EI_ENA_BIT					18		//Error warning limit reached interrupt enable
	#define DOI_ENA_BIT					19		//Data overrun interrupt enable
	#define EPI_ENA_BIT					21		//Error passive interrupt enable
	#define ALI_ENA_BIT					22		//Arbitration lost interrupt enable
	#define BEI_ENA_BIT					23		//Bus error interrupt enable
	#define LFI_ENA_BIT					24		//Event logging finished interrupt enable
	#define RFI_ENA_BIT					25		//Recieve buffer full interrupt enable
	#define BSI_ENA_BIT					26		//Bit rate shifted interrupt enable



	/*
	 * Timing register bits
	 */

	//BTR
	#define PROP_LOW					0		//Propagation segment (Nominal)
	#define PROP_HIGH					5
	#define PH_1_LOW					6		//Phase 1 segment (Nominal)
	#define PH_1_HIGH					10
	#define PH_2_LOW					11		//Phase 2 segment (Nominal)
	#define PH_2_HIGH					15

	//BTR_FD
	#define PROP_FD_LOW					16		//Propagation segment (Data)
	#define PROP_FD_HIGH				19
	#define PH_1_FD_LOW					22		//Phase 1 segment (Data)
	#define PH_1_FD_HIGH				25
	#define PH_2_FD_LOW					27		//Phase 2 segment (Data)
	#define PH_2_FD_HIGH				30

	/*
	 * Arbitration lost capture, prescaler register bits
	 */

	//ALC
	#define ALC_LOW						0		//Arbitration lost capture value
	#define ALC_HIGH					4

	//BRP
	#define BRP_LOW						16		//Baud rate prescaler (Nominal)
	#define BRP_HIGH					21
	#define SJW_LOW						8		//Synchronisation jump width (Nominal)
	#define SJW_HIGH					11

	//BRP_FD
	#define BRP_FD_LOW					24		//Baud rate prescaler (Data)
	#define BRP_FD_HIGH					29
	#define SJW_FD_LOW					12		//Synchronisation jump width (Data)
	#define SJW_FD_HIGH					15


	/*
	 * Error treshold
	 */

	//EWL
	#define EWL_LOW						0		//Error warning limit
	#define EWL_HIGH					7

	//ERP
	#define ERP_LOW						8		//Error passive limit
	#define ERP_HIGH					15

	//FAULT_STATE
	#define ERA_BIT						16		//Error active
	#define ERP_BIT						17		//Error passive
	#define BOF_BIT						18		//Bus off


	/*
	 * Error counters
	 */

	//RXC/CTR_PRES
	#define RXC_VAL_LOW					0		//Receive frame counter
	#define RXC_VAL_HIGH				15
	#define CTR_PRES_VAL_LOW			0		//Counter preset value
	#define CTR_PRES_VAL_HIGH			8
	#define PTX_BIT						9		//Preset TX error counter
	#define PRX_BIT						10		//Preset RX error counter

	//TXC
	#define TXC_VAL_LOW					16		//Transcieve value counter
	#define TXC_VAL_HIGH				31


	/*
	 * Error counters special
	 */

	//ERR_NORM/CTR_PRES
	#define ERR_NORM_VAL_LOW			0		//Errors in nominal data rate
	#define ERR_NORM_VAL_HIGH			15
	#define ENORM						11		//Erase nominal data-rate error counter
	#define EFD							12		//Erase data data-rate counter

	//ERR_FD
	#define ERR_FD_VAL_LOW				16		//Errors in data data-rate
	#define ERR_FD_VAL_HIGH				31


	/*
	 * Filter settings
	 */

	//Filter_X_Mask
	#define BIT_MASK_X_VAL_LOW			0		//Bit mask of filter
	#define BIT_MASK_X_VAL_HIGH			28

	//Filter_X_Value
	#define BIT_VAL_X_VAL_LOW			0		//Bit value of filter
	#define BIT_VAL_X_VAL_HIGH			28

	//Filter_Control
	#define BASIC_FRAME_FILT_A_BIT		0		//CAN Basic frames accepted
	#define BASIC_FRAME_FILT_B_BIT		4
	#define BASIC_FRAME_FILT_C_BIT		8
	#define BASIC_FRAME_FILT_RAN_BIT	12
	#define EXT_FRAME_FILT_A_BIT		1		//CAN Extended frames accepted
	#define EXT_FRAME_FILT_B_BIT		5
	#define EXT_FRAME_FILT_C_BIT		9
	#define EXT_FRAME_FILT_RAN_BIT		13
	#define FD_FRAME_FILT_A_BIT			2		//CAN FD Basic frames accepted
	#define FD_FRAME_FILT_B_BIT			6
	#define FD_FRAME_FILT_C_BIT			10
	#define FD_FRAME_FILT_RAN_BIT		14
	#define FD_EXT_FRAME_FILT_A_BIT		3		//CAN FD Extended frames accepted
	#define FD_EXT_FRAME_FILT_B_BIT		7
	#define FD_EXT_FRAME_FILT_C_BIT		11
	#define FD_EXT_FRAME_FILT_RAN_BIT	15


	/*
	 * RX Info 1
	 */

	//RX_STATUS
	#define RX_EMPTY_BIT				0		//RX Buffer is empty
	#define RX_FULL_BIT					1		//RX Buffer is full

	//RX_MC
	#define RX_MC_VALUE_LOW				8		//RX Buffer Message count
	#define RX_MC_VALUE_HIGH			15

	//RX_MF
	#define RX_MF_VALUE_LOW				16		//RX Buffer free memory
	#define RX_MF_VALUE_HIGH			23


	/*
	 * RX Info 2
	 */

	//RX_BUFF_SIZE
	#define RX_BUFF_SIZE_LOW			0		//Size of RX BUffer
	#define RX_BUFF_SIZE_HIGH			7

	//RX_MC
	#define RX_WPP_VALUE_LOW			8		//RX Buffer write pointer
	#define RX_WPP_VALUE_HIGH			15

	//RX_MF
	#define RX_RPP_VALUE_LOW			16		//RX Buffer read pointer
	#define RX_RPP_VALUE_HIGH			23


	/*
	 * RX_DATA and TX_DATA_X
	 */
	//FRAME_FORM
	#define DLC_LOW						0
	#define DLC_HIGH					3
	#define RTR_BIT						5
	#define ID_TYPE_BIT					6
	#define FR_TYPE_BIT					7
	#define TBF_BIT						8
	#define BRS_BIT						9
	#define ESI_BIT						10

	//IDENTIFIER
	#define ID_BASE_LOW					0
	#define ID_BASE_HIGH				10
	#define ID_EXT_LOW					11
	#define ID_EXT_HIGH					29


	/*
	 * TX_STATUS
	 */
	#define TXT_1_EMPTY_BIT				0
	#define TXT_2_EMPTY_BIT				1


	/*
	 * TX_SETTINGS
	 */
	#define TXT_1_ALLOW_BIT				0
	#define TXT_2_ALLOW_BIT				1
	#define TXT_1_COMMIT_BIT			2
	#define TXT_2_COMMIT_BIT			3


	/*
	 * LOG_TRIG_CONFIG
	 */
	#define T_SOF_BIT					0
	#define T_ARBL_BIT					1
	#define T_REV_BIT					2
	#define T_TRV_BIT					3
	#define T_OVL_BIT					4
	#define T_ERR_BIT					5
	#define T_BRS_BIT					6
	#define T_USRW_BIT					7
	#define T_ARBS_BIT					8
	#define T_CTRS_BIT					9
	#define T_DATS_BIT					10
	#define T_CRCS_BIT					11
	#define T_ACKR_BIT					12
	#define T_ACKNR_BIT					13
	#define T_EWLR_BIT					14
	#define T_ERPC_BIT					15
	#define T_TRS_BIT					16
	#define T_RES_BIT					17

	/*
	 * LOG_CAPT_CONFIG
	 */
	#define C_SOF_BIT					0
	#define C_ARBL_BIT					1
	#define C_REV_BIT					2
	#define C_TRV_BIT					3
	#define C_OVL_BIT					4
	#define C_ERR_BIT					5
	#define C_BRS_BIT					6
	#define C_ARBS_BIT					7
	#define C_CTRS_BIT					8
	#define C_DATS_BIT					9
	#define C_CRCS_BIT					10
	#define C_ACKR_BIT					11
	#define C_ACKNR_BIT					12
	#define C_EWLR_BIT					13
	#define C_ERPC_BIT					14
	#define C_TRS_BIT					15
	#define C_RES_BIT					16
	#define C_SYNE_BIT					17
	#define C_STUFF_BIT					18
	#define C_DESTUFF_BIT				19
	#define C_OVR_BIT					20


	/*
	 * LOG_CAPT_CONFIG
	 */
	//LOG_STAT
	#define LOG_CFG_BIT					0
	#define LOG_RDY_BIT					1
	#define LOG_RUN_BIT					2
	#define LOG_EXIST_BIT				7
	#define LOG_SIZE_LOW				8
	#define LOG_SIZE_HIGH				15

	//LOG_WPP
	#define LOG_WPP_LOW					16
	#define LOG_WPP_HIGH				23

	//LOG_RPP
	#define LOG_RPP_LOW					24
	#define LOG_RPP_HIGH				31

	//LOG_COMMAND
	#define LOG_STR_BIT					0
	#define LOG_ABT_BIT					1
	#define LOG_UP_BIT					2
	#define LOG_DOWN_BIT				3


	/*
	 * LOG_CAPT_EVENT_2
	 */
	#define EVENT_TYPE_LOW				0
	#define EVENT_TYPE_HIGH				7
	#define EVENT_DETAILS_LOW			8
	#define EVENT_DETAILS_HIGH			15

	#define EVNT_BIT_ERR				8
	#define EVNT_ST_ERR					9
	#define EVNT_CRC_ERR				10
	#define EVNT_ACK_ERR				11
	#define EVNT_FRM_ERR				12

	#define EVNT_BRS_UP_BIT				8
	#define EVNT_BRS_DOWN_BIT			9

	#define EVNT_SYNC_SYNC_BIT			8
	#define EVNT_SYNC_PROP_BIT			9
	#define EVNT_SYNC_PH1_BIT			10
	#define EVNT_SYNC_PH2_BIT			11
	#define EVNT_SYNC_TYPE_LOW			12
	#define EVNT_SYNC_TYPE_HIGH			13

	#define EVNT_STUFF_LENGHT_LOW		8
	#define EVNT_STUFF_LENGHT_HIGH		10
	#define EVNT_STUFF_FIXED_BIT		11

	#define EVNT_DESTUFF_LENGHT_LOW		8
	#define EVNT_DESTUFF_LENGHT_HIGH	10
	#define EVNT_DESTUFF_FIXED_BIT		11

	#define EVENT_DETAILS_LOW			8
	#define EVENT_DETAILS_HIGH			15

	/*
	 * DEBUG REGISTER
	 */
	#define STUFF_COUNT_LOW				0
	#define STUFF_COUNT_HIGH			2
	#define DESTUFF_COUNT_LOW			3
	#define DESTUFF_COUNT_HIGH			5
	#define PC_ARB_BIT					6
	#define PC_CON_BIT					7
	#define PC_DAT_BIT					8
	#define PC_CRC_BIT					9
	#define PC_EOF_BIT					10
	#define PC_OVR_BIT					11
	#define PC_INT_BIT					12

	/*
	 * Vector creation macros
	 */
	#define CREATE_MODE_VECTOR(LOM,STM,AFM,FDE,RTRP,TSM,ACF) \
					(LOM << LOM_BIT) & \
					(STM << STM_BIT) & \
					(AFM << AFM_BIT) & \
					(FDE << FDE_BIT) & \
					(RTRP << RTRP_BIT) & \
					(TSM << TSM_BIT) & \
					(ACF << ACF_BIT)

	#define CREATE_INT_ENA_VECTOR(RIE,TIE,EIE,DOIE,EPIE,ALIE,BEIE,LFIE,RFIE,BSIE) \
					(RIE << RI_ENA_BIT) & \
					(TIE << TI_ENA_BIT) & \
					(EIE << EI_ENA_BIT) & \
					(DOIE << DOI_ENA_BIT) & \
					(EPIE << EPI_ENA_BIT) & \
					(ALIE << ALI_ENA_BIT) & \
					(BEIE << BEI_ENA_BIT) & \
					(LFIE << LFI_ENA_BIT) & \
					(RFIE << RFI_ENA_BIT) & \
					(BSIE << BSI_ENA_BIT)

	/*
	 * Memory access and bit manipulation macros
	 */
	#define READ_VALUE_32BIT(BASE, OFFSET) \
			(*((volatile unsigned int *) ((BASE) + (OFFSET))))

	#define WRITE_VALUE_32BIT(BASE, OFFSET, VALUE) \
			(*((volatile unsigned int *) ((BASE) + (OFFSET))) = (VALUE))


	#define SET_BIT(VAR, BIT) VAR | (1 << BIT)

	#define CLEAR_BIT(VAR, BIT) VAR & (~(1 << BIT))

	#define PLACE_BIT(VAR, BIT, VALUE) (VAR & (~(1 << BIT))) | (VALUE << BIT)


	#define CLEAR_BYTE(VAR, BYTE) VAR & (~(0xFF << (BYTE-1)*8))

	#define SET_BYTE(VAR, BYTE) VAR | (0xFF << (BYTE-1)*8)

	#define PLACE_BYTE(VAR, BYTE, VALUE) VAR | (VALUE << (BYTE-1)*8)

	#define IS_BIT_SET(VAR, BIT) (VAR >> BIT) % 2 ? true : false

	/*
	 * Identifier formatting macros
	 */
	#define IDENT_UNS_TO_REG(IDENT) ((0x3FFFF & IDENT) << 11) | ((0x1FFFFFFF & IDENT) >> 18)
	#define IDENT_REG_TO_UNS(IDENT)	((0x7FF & IDENT) << 18) | ((0x1FFFFFFF & IDENT) >> 11)


	/*
	 * Other macros
	 */
	#define CAN_IP_CORE_IDENTIFIER  0x0000CAFD

	#define _2BIT_MASK					0x3
	#define _3BIT_MASK					0x7
	#define _4BIT_MASK					0xF
	#define _5BIT_MASK					0x1F
	#define _6BIT_MASK					0x3F
	#define _8BIT_MASK					0xFF
	#define _11BIT_MASK					0x7FF

	/***************************
	 ***************************
	 * Types and structures
	 ***************************
	 ***************************/

	typedef uint32_t can_node_t;  //Base address of the CAN controller that should
								// be accessed

	typedef uint32 canid_t;	//CAN identifier type

	typedef enum
	{
		RESET				= 0x0,
		ABORT_TRANSMISSION	= 0x1,
		RELEASE_RX_BUFFER	= 0x2,
		CLEAR_DATA_OVERRUN	= 0x3
	}can_command_t;

	typedef enum
	{
		NORMAL				= 0x0,
		FD					= 0x1
	}can_frame_format_t;

	typedef enum
	{
		BASIC				= 0x0,
		EXTENDED			= 0x1
	}can_ident_type_t;

	typedef enum
	{
		NO_RTR				= 0x0,
		RTR_FRAME			= 0x1
	}can_rtr_flag_t;

	typedef enum
	{
		NO_SHIFT			= 0x0,
		SHIFT				= 0x1
	}can_brs_flag_t;

	typedef enum
	{
		ERROR_ACTIVE 		= 0x0,
		ERROR_PASSIVE		= 0x1,
		BUS_OFF				= 0x2
	}can_error_state_t;

	typedef enum
	{
		ISO_FD				= 0x0,
		NON_ISO_FD			= 0x1
	}can_fd_type_t;

	typedef enum
	{
		CONFIG				= 0x0,
		READY				= 0x1,
		RUNNING				= 0x2
	}can_log_state_t;

	typedef enum
	{
		START				= 0x0,
		ABORT				= 0x1,
		RPP_UP				= 0x2,
		RPP_DOWN			= 0x3
	}can_log_command_t;

	typedef enum
	{
		START_OF_FRAME		= 0x1,
		ARBITR_LOST			= 0x2,
		FRAME_RECEIVED  	= 0x3,
		FRAME_TRANSCEIVED	= 0x4,
		OVERLOAD_FRAME		= 0x5,
		ERROR_APPEARED		= 0x6,
		BIT_RATE_SHIFT		= 0x7,
		ARBITRATION_START	= 0x8,
		CONTROL_START		= 0x9,
		DATA_START			= 0xA,
		CRC_START			= 0xB,
		ACK_RECEIVED		= 0xC,
		ACK_NOT_RECEIVED	= 0xD,
		EWL_REACHED			= 0xE,
		FAULT_STATE_CHANGE  = 0xF,
		TRANSCEIVE_START    = 0x10,
		RECEIVE_START		= 0x11,
		SYNC_EDGE			= 0x12,
		STUFF_BIT			= 0x13,
		DESTUFF_BIT			= 0x14,
		DATA_OVERRUN		= 0x15
	}can_event_type_t;

	typedef enum
	{
		FILTER_A			= 0x0,
		FILTER_B			= 0x1,
		FILTER_C			= 0x2,
		FILTER_RAN			= 0x3
	}can_filter_t;

	typedef enum
	{
		SUCESS				= 0x0,
		INVALID_PTR			= 0x1,			//Pointer that function should fill is NULL
		INCORRECT_VAL		= 0x2,			//Register did not have expected value
		NO_BUF_SPACE		= 0x3			//Both TXT_Buffers are full frame cannot
											// be sent!
	}STATUS;

	/*
	 * CAN and CAN FD frame structure for transmittion and reception
	 *
	 * If frame is about to be sent following attributes should be set:
	 * 		data_length		(not dlc attribute !!! )
	 * 		timestamp (if it should be send immediatelly fill zeroes)
	 *		identifier
	 *		frame_format
	 *		ident_type
	 *
	 *		rtr_flag should be set only if frame_format is NORMAL
	 *		brs_flag should be set only if frame_format is FD
	 *
	 *		esi_flag will have no effect in transmitted frames and will be
	 *		automatically filled in received frames!
	 *
	 *		dlc attribute will be automatically filled by the library when
	 *		function can_send_frame is called over frame structure!
	 */
	struct can_frame_t
	{
		uint8_t				dlc;			//Data length code
		uint8_t				data_len;		//Data length in bytes
		uint64_t			timestamp;		//Timestamp when frame was received or
											// when it should be transmitted
		uint32_t			identifier;		//Unsigned value of the identifier!
											//Note this is not the same value as written
											//to the IP Core!!!
		can_frame_format_t	frame_format;	//Frame format (Normal or FD)
		can_ident_type_t	ident_type;		//Identifier type (Basic or extended)
		can_rtr_flag_t		rtr_flag;		//Remote transmittion request
											// (valid only for Non-FD frames)
		can_brs_flag_t		brs_flag;		//Bit rate shift flag!
											//(Valid only for received CAN FD frames!)
		can_error_state_t 	esi_flag;		//Error state indicator! This value is always
											//(Valid only for received CAN FD frames!)
		uint8				*data;			//Pointer to data
											//Note that from this pointer on there must
											//be enough memory available for all data
											//of the frame
	};


	/*
	 * Timing setting of CAN controller
	 */
	struct can_bus_timing_t
	{
		uint8_t		presc_nbt;		//Prescaler (Nominal)
		uint8_t 	presc_dbt;		//Prescaler (Data)
		uint8_t		prs_nbt;		//Propagation segment (Nominal)
		uint8_t		ph1_nbt;		//PH1 segment (Nominal)
		uint8_t		ph2_nbt;		//PH2 segment (Nominal)
		uint8_t		prs_dbt;		//Propagation segment (Data)
		uint8_t		ph1_dbt;		//PH1 segment (Data)
		uint8_t		ph2_dbt;		//PH2 segment (Data)
		uint8_t		sjw_nbt;		//Synchronisation jump width (Nominal)
		uint8_t		sjw_dbt;		//Synchronisation jump width (Data)
	};

	typedef uint8_t	can_contr_mode_t;		//Controller mode	(MODE)
	typedef	uint8_t can_contr_status_t;		//Controller status (STATUS)

	struct can_settings_type_t
	{
		boolean			retr_limit_ena;
		uint8_t			retr_treshold;
		boolean			int_loopback;
		boolean			enable;
		can_fd_type_t	fd_type;
	};

	typedef uint16_t can_int_vector;		//Interrupt vector type

	struct can_error_state_struct_t
	{
		uint8_t				ewl;		//Error warning limit
		uint8_t				erp;		//Error passive
		can_error_state_t	err_state;  //Error state type (Read only)
	};

	struct can_error_ctr_t
	{
		uint16_t			tx_ctr;		//Transmitt error counter
		uint16_t			rx_ctr;		//Receive error counter
		uint16_t			norm_ctr;	//Error counter in nominal bit time
		uint16_t			data_ctr;	//Error counter in Data bit time
	};

	struct can_traffic_ctr_t
	{
		uint32_t			tx_ctr;		//Transmitt counter
		uint32_t			rx_ctr;		//Receive counter
	};

	struct can_rxbuf_stat_t
	{
		boolean				full;
		boolean				empty;
		uint8_t				frames;			//Number of frames stored in the buffer
		uint8_t				mem_free;	 	//Free 32 bit words in the buffer
		uint8_t				buff_size;   	//Size of RX Buffer in words
		uint8_t				read_pointer; 	//Read pointer position
		uint8_t				write_pointer;	//Write pointer position
	};

	struct can_log_info_t
	{
		can_log_state_t 	state;			//Event logger state
		boolean				log_exist;		//Logger is synthesized
		uint8_t				size;
		uint8_t				read_pointer;	//Read pointer position
		uint8_t				write_pointer;	//Write pointer position
	};

	struct can_event_t
	{
		uint64_t 			timestamp;		//Timestamp when event occured
											//Only lowest 48 bits are valid!
	};

	struct can_bit_filter_set_t
	{
		uint32_t			mask;			//Bit mask for filter
		uint32_t			value;			//Bit value to compare

											//These two values has the same format
											//for identifier as in TX_DATA_4 register!!

		uint8_t				control;		//Frame types and Identifier formats
											//to accept as defined in FILTER_CONTROL
											//register
	};

	struct can_range_filter_set_t
	{
		uint32_t			ran_low;		//Lower unsigned value of identifier
		uint32_t			ran_high;		//Higher unsigned value of identifier

											//These two values does NOT have the same format
											//for identifier as in TX_DATA_4 register!!
											//Unsigned representation of identifiers decimal
											//value should be written here!

		uint8_t				control;		//Frame types and Identifier formats
											// to accept as defined in FILTER_CONTROL
											//register
	};


	/***************************
	 ***************************
	 * Functions
	 ***************************
	 ***************************/

	/*
	 * Check if node at given address returns correct identifier
	 * as in register DEVICE_ID
	 *
	 * Parameters:
	 * 	node		- Base address of the node to check
	 *
	 * Return:
	 * 	Node OK/Not OK
	 */
	STATUS check_node_access(can_node_t node);


	/*
	 * Enable/Disable the controller by writing to ENA bit
	 * in SETTINGS register
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	enable		- Enable the controller
	 */
	void enable_controller(can_node_t node, bool enable);


	/*
	 * Enable/Disable the individual mode of the controller
	 * given by the offset
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	enable		- Enable/Disable the mode
	 * 	mode_off	- Offset of the mode as in MODE register
	 */
	void set_mode_ind(can_node_t node, uint8_t mode_off, bool enable);


	/*
	 * Enable/Disable modes of the controller all at once. 8 bit
	 * vector written into MODE register is needed
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	vector		- 8 bit vector written to the MODE register
	 */
	void set_mode_all(can_node_t node, uint8_t vector);


	/*
	 * Give command to the CAN controller. Function activates bits
	 * in the COMMAND and MODE registers.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	command		- Command to give to the node
	 */
	void give_command(can_node_t node, can_command_t command);


	/*
	 * Obtain status of the CAN Node as in the STATUS register
	 * into 8-bit vector.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *
	 * Return:
	 *  Controller status vector
	 */
	uint8_t get_status(can_node_t node);


	/*
	 * Set the contents of SETTINGS register.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	*settings   - Pointer to settings structure to apply
	 */
	void set_settings(can_node_t node, struct can_settings_type_t *settings);


	/*
	 * Get the settings structure of the controller
	 * from the SETTINGS register.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	*settings   - Pointer to fill return data
	 *
	 * 	Return:
	 *	 If struct was filled OK or not!
	 */
	STATUS get_settings(can_node_t node, struct can_settings_type_t *settings);


	/*
	 * Enable/Disable individual interrupt by writing
	 * a bit to INT_ENA register.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	int_offset  - Interrupt offset as in INT_ENA register
	 * 	enable		- If this interrupt should be enabled or disabled
	 */
	void enable_interrupt_ind(can_node_t node, uint8_t int_offset, boolean enable);


	/*
	 * Enable/Disable all interrupt by writing
	 * a vector to INT_ENA register.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	int_vect   	- Content of the INT_ENA register
	 */
	void enable_interrupt_all(can_node_t node, uint16_t int_vect);


	/*
	 * Read interrupt vector from the INT register.
	 * Interrupt vector is erased afterwards!
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *
	 * 	Return:
	 * 	 Interrupt vector
	 */
	uint16_t get_interrupt_vect(can_node_t node);


	/*
	 * Set the controller timing parameters
	 * (phase segment, synchronisation jump width)
	 * by writing to the TIMING_REG and ALC_PRESC
	 * registers.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	*timing   	- Pointer to the timing structure
	 */
	void set_bus_timing(can_node_t node, struct can_bus_timing_t *timing);


	/*
	 * Set the controller timing parameters
	 * (phase segment, synchronisation jump width)
	 * by writing to the TIMING_REG and ALC_PRESC
	 * registers.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	*timing   	- Pointer to the timing structure
	 * 				  to fill
	 *
	 * Return:
	 * 	Structure filled corectly
	 */
	STATUS get_bus_timing(can_node_t node, struct can_bus_timing_t *timing);


	 /*
	  * Get the error thresholds of the CAN controller and
	  * Error state (Active, Passive, Bus off).
	  *
	  * Parameters:
	  * 	node		- Base address of the node
	  *	*err_state  - Pointer to error structure to fill
	  *
	  * Return:
	  * 	If structure was filled OK
	  */
	 STATUS get_err_state(can_node_t node, struct can_error_state_struct_t *error_state);


	/*
	 * Set the error thresholds of the CAN controller.
	 * Note that error-state itself is read only and its
	 * value will not modify the error state of the
	 * controller! This function will also not set the
	 * error counters! Use "set_error_ctrs" for this
	 * purpose!
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	*err_state  - Pointer to the error state structure holding
	 * 				  error state thresholds
	 */
	void set_err_th(can_node_t node, struct can_error_state_struct_t *err_state);


	/*
	 * Set error counters in register ERROR_COUNTERS.
	 * This way fault confinement state can be modified!
	 * Note that this function will not set the thresholds
	 * in the "err_ctr" structure! Use "set_err_th" for this
	 * purpose!
	 *
	 * Parameters:
	 * 	node			- Base address of the node
	 * 	err_ctr			- Counter structure with counter values
	 * 	set_tx			- TX counter should be set to value
	 * 					  from the "err_ctr" structure
	 * 	set_rx			- RX counter should be set to value
	 * 					  from the "err_ctr" structure
	 * 	erase_tx_spec	- Erase Error counter of Nominal bit time
	 * 	erase_rx_spec	- Erase Error counter of Data bit time
	 *
	 * 	Note special counters ERR_NORM and ERR_FD are read and
	 * 	erase only. It is not possible to set custom values there!
	 */
	void set_error_ctrs(can_node_t node,struct can_error_ctr_t err_ctr,
						boolean set_tx,
						boolean set_rx,
						boolean erase_norm,
						boolean erase_fd);


	/*
	 * Get error counters from register ERROR_COUNTERS.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	*error_ctrs - Pointer to structure to fill
	 *
	 * Return:
	 * 	Structure filled OK
	 */
	STATUS get_error_ctrs(can_node_t node, struct can_error_ctr_t *error_ctrs);


	/*
	 * Set the bit filter.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	filter		- Filter to set (A,B,C)
	 * 	settings    - Filter settings
	 */
	void set_bit_filter(can_node_t node, can_filter_t filter,
							struct can_bit_filter_set_t *settings);


	/*
	 * Set the range filter.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	filter		- Filter to set (RANGE)
	 * 	settings    - Filter settings
	 */
	void set_range_filter(can_node_t node, can_filter_t filter,
							struct can_range_filter_set_t *settings);


	/*
	 * Get error counters from register ERROR_COUNTERS.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	*status     - Pointer to status structure to fill
	 *
	 * Return:
	 * 	Rx Buffer status structure filled Ok
	 */
	STATUS get_rx_buf_status(can_node_t node, struct can_rxbuf_stat_t *status);


	/*
	 * Convert DLC to data length in bytes
	 *
	 * Parameters:
	 * 	dlc 	- Data length code
	 *
	 * Return:
	 *  Data length in bytes
	 */
	uint8_t	get_byte_length(uint8_t dlc);


	/*
	 * Convert data length in bytes to DLC. If invalid
	 * value is on input of this function nearest bigger
	 * value is returned so that data cant be lost!! If
	 * value higher than 64 is on input then DLC for
	 * 64 is returned!
	 *
	 * Parameters:
	 * 	data_length 	- Data length in bytes
	 *
	 * Return:
	 *  DLC
	 */
	uint8_t	get_dlc(uint8_t data_length);


	/*
	 * Read the frame from the RX Buffer. If there is no
	 * frame it is signalled by returning false.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 * 	*frame		- Pointer where to frame struct to fill
	 *
	 * Return:
	 * 	Frame read sucesfully/unsucesfully
	 */
	STATUS can_read_frame(can_node_t node,struct can_frame_t *frame);


	/*
	 * Insert frame for transmission and commit it into the
	 * lowest available TXT Buffer. If no TXT Buffer is available
	 * then Error code is returned. Also note that function will
	 * commit the frame even if buffer is forbidden by bits
	 * TXT_X_ALLOW !! This function does not modify these bits!
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *
	 * Return:
	 * 	Frame sent or Error code
	 */
	STATUS can_send_frame(can_node_t node,struct can_frame_t *frame);


	/*
	 * Read content of TRV_DELAY register. Note that this
	 * register has valid value only from EDL bit until
	 * the end of frame. Then it is erased!
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *
	 * Return:
	 * 	Transciever delay counted by the main clock of the
	 * 	controller.
	 */
	uint16_t can_get_trv_delay(can_node_t node);


	/*
	 * Read traffic counters of the controller.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	traf_ctr	- Traffic counter structure to fill
	 *
	 * Return:
	 * 	Traffic counters structure filled OK
	 */
	STATUS can_read_traffic_ctrs(can_node_t node, struct can_traffic_ctr_t *traf_ctr);


	/*
	 * Enable/Disable individual trigger event for event logging.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	enable		- Enable/Disable the event type for triggering
	 *	trig_offset - Offset of event type as in LOG_TRIG_CONFIG
	 */
	void set_trig_config_ind(can_node_t node, boolean enable, uint8_t trig_offset);


	/*
	 * Enable/Disable trigger for all events for event logging.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	trig_vector	- Trigger vector to write to LOG_TRIG_CONFIG
	 */
	void set_trig_config_all(can_node_t node, uint32_t trig_vector);


	/*
	 * Enable/Disable individual captured event for event logging.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	enable		- Enable/Disable the event type for capturing
	 *	capt_offset - Offset of event type as in LOG_CAPT_CONFIG
	 */
	void set_capt_config_ind(can_node_t node, boolean enable, uint8_t capt_offset);


	/*
	 * Enable/Disable capture of all events for event logging.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	capt_vector	- Trigger vector to write to LOG_CAPT_CONFIG
	 */
	void set_capt_config_all(can_node_t node, uint32_t capt_vector);


	/*
	 * Give command to optional event logger circuit.
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	command		- Command to give to the event logger
	 */
	void give_log_command(can_node_t node, can_log_command_t command);


	/*
	 * Read the status of the event logger circuit
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	status		- Struct to fill status into
	 *
	 * 	Return:
	 * 	 Return Struct filled Ok!
	 */
	STATUS get_logger_status(can_node_t node, struct can_log_info_t *status);


	/*
	 * Read the actual event from event logger
	 *
	 * Parameters:
	 * 	node		- Base address of the node
	 *	*event		- Pointer to event to be read
	 */
	STATUS	read_event(can_node_t node, can_event_type_t *event);


#endif /* CAN_FD_IP_DRIVER_H_ */
