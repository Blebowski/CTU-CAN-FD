/*******************************************************************************
 * 
 * CTU CAN FD IP Core
 * Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
 * 
 * Project advisors and co-authors: 
 * 	Jiri Novak <jnovak@fel.cvut.cz>
 * 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
 * 	Martin Jerabek <jerabma7@fel.cvut.cz>
 * 
 * Department of Measurement         (http://meas.fel.cvut.cz/)
 * Faculty of Electrical Engineering (http://www.fel.cvut.cz)
 * Czech Technical University        (http://www.cvut.cz/)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * 
*******************************************************************************/

#ifndef __CTU_CAN_FD_HW__
#define __CTU_CAN_FD_HW__

#include "ctu_can_fd_regs.h"

/*
    MJ TODO:
    - move small functions to *.h, make them inline
    - either pass union arguments by value or just as u32;
      this way they are forced on stack instead of passing in register
      + one level of pointer indirection, which sucks performance-wise
*/

#define CTU_CAN_FD_RETR_MAX 15

#define CTU_CAN_FD_FILTER_A 0x01
#define CTU_CAN_FD_FILTER_B 0x02
#define CTU_CAN_FD_FILTER_C 0x03

#define CTU_CAN_FD_TXT_BUFFER_COUNT 4

#define CTU_CAN_FD_TXT_BUFFER_1 0x1
#define CTU_CAN_FD_TXT_BUFFER_2 0x2
#define CTU_CAN_FD_TXT_BUFFER_3 0x3
#define CTU_CAN_FD_TXT_BUFFER_4 0x4

/* 
 * Status macros -> pass "ctu_can_get_status" result
 */

// True if Core is transceiver of current frame
#define CTU_CAN_FD_IS_TRANSMITTER(stat) (!!(stat).ts)

// True if Core is receiver of current frame
#define CTU_CAN_FD_IS_RECEIVER(stat) (!!(stat).s.rs)

// True if Core is idle (integrating or interfame space)
#define CTU_CAN_FD_IS_IDLE(stat) (!!(stat).s.bs)

// True if Core is transmitting error frame
#define CTU_CAN_FD_ERR_FRAME(stat) (!!(stat).s.et)

// True if Error warning limit was reached
#define CTU_CAN_FD_EWL(stat) (!!(stat).s.ewl)

// True if at least one TXT Buffer is empty
#define CTU_CAN_FD_TXTNE(stat) (!!(stat).s.tbs)

// True if data overrun flag of RX Buffer occurred
#define CTU_CAN_FD_DATA_OVERRUN(stat) (!!(stat).s.dos)

// True if RX Buffer is not empty
#define CTU_CAN_FD_RX_BUF_NEMPTY(stat) (!!(stat).s.rbs)


/* 
 * Interrupt macros -> pass "ctu_can_fd_int_sts" result
 */

// Frame reveived interrupt
#define CTU_CAN_FD_RX_INT(int_stat) (!!(int_stat).s.ri)

// Frame transceived interrupt
#define CTU_CAN_FD_TX_INT(int_stat) (!!(int_stat).s.ti)

// Error warning limit reached interrupt
#define CTU_CAN_FD_EWL_INT(int_stat) (!!(int_stat).s.ei)

// RX Buffer data overrun interrupt
#define CTU_CAN_FD_OVERRUN_INT(int_stat) (!!(int_stat).s.doi)

// Core turned error passive interrupt
#define CTU_CAN_FD_ERR_PASSIVE_INT(int_stat) (!!(int_stat).s.epi)

// Error frame transmission started interrupt
#define CTU_CAN_FD_BUS_ERROR_INT(int_stat) (!!(int_stat).s.bei)

// Event logger finished interrupt
#define CTU_CAN_FD_LOGGER_FIN_INT(int_stat) (!!(int_stat).s.lfi)

// RX Buffer full interrupt
#define CTU_CAN_FD_RX_FULL_INT(int_stat) (!!(int_stat).s.rfi)

// Bit-rate shifted interrupt
#define CTU_CAN_FD_BIT_RATE_SHIFT_INT(int_stat) (!!(int_stat).s.bsi)

// Receive buffer not empty interrupt
#define CTU_CAN_FD_RX_BUF_NEPMTY_INT(int_stat) (!!(int_stat).s.rbnei)

// TX Buffer received HW command interrupt
#define CTU_CAN_FD_TXT_BUF_HWCMD_INT(int_stat) (!!(int_stat).s.txbhci)


/*
 * Checks whether the core is mapped correctly at it's base address.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *
 * Returns:
 *	true if the core is accessible correctly, false otherwise.
 */
bool ctu_can_fd_check_access(const void *base);


/*
 * Returns version of CTU CAN FD IP Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *
 * Returns:
 *	IP Core version in format major*10 + minor
 */
u32 ctu_can_fd_get_version(const void *base);


/*
 * Enables/disables the operation of CTU CAN FD Core. If disabled, the Core will
 * never start transmitting on the CAN bus, nor receiving.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	enable	Enable/disable the core.
 *
 */
void ctu_can_fd_enable(void *base, bool enable);


/*
 * Configures CTU CAN FD Core to limit the amount of retransmit attempts after
 * occurence of error (Error frame, Arbitration lost). If retransmitt limit is 
 * disabled, the Core will attempt to retransmitt inifinitely. If rettransmitt 
 * limit is reached, the Core will finish and according TXT buffer will end up
 * in TX Error state.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	enable	Enable/disable the retransmitt limitation
 *      limit	Number to which limit the retransmission (1-CTU_CAN_FD_RETR_MAX) 
 * Returns:
 *	True if set correctly. False if "limit" is too high.
 */
bool ctu_can_fd_set_ret_limit(void *base, bool enable, u8 limit);


/*
 * Configures CTU CAN FD Core for special operating modes by access to MODE
 * register. Following flags from "mode" are not configured by this function:
 *  CAN_CTRLMODE_ONE_SHOT, CAN_CTRLMODE_BERR_REPORTING.
 * Following flags are configured:
 *  	CAN_CTRLMODE_LOOPBACK	- Bit loopback mode. Every dominant bit is 
*				  re-routed internally and not send on the bus.		
 *	CAN_CTRLMODE_LISTENONLY	- No frame is transmitted, no dominant bit is
 *				  sent on the bus.
 *	CAN_CTRLMODE_3_SAMPLES  - Tripple sampling mode
 *	CAN_CTRLMODE_FD		- Flexible data-rate support. When not set, Core
 *				  does not accept CAN FD Frames and interprets,
 *				  them as form error. Capability to transmitt
 *				  CAN FD Frames is not affected by this setting.
 *	CAN_CTRLMODE_PRESUME_ACK - When set, Core does not require dominant bit
 *				   in ACK field to consider the transmission as
 *				   valid.
 *	CAN_CTRLMODE_FD_NON_ISO  - When set, the Core transmitts the frames
 *				   according to NON-ISO FD standard.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	mode	CAN mode to be set to on the Core. 
 */
void ctu_can_fd_set_mode_reg(void *base, const struct can_ctrlmode *mode);


/*
 * Gives command to CTU CAN FD Core to erase and reset the RX FIFO. This
 * action is finished immediately and does not need waiting.
 * 
 * Arguments:
 *	base	Pointer to the base address
 */
void ctu_can_fd_rel_rx_buf(void *base);


/*
 * Gives command to CTU CAN FD Core to clear the Data overrun flag on 
 * the RX FIFO Buffer.
 * 
 * Arguments:
 *	base	Pointer to the base address
 */
void ctu_can_fd_clr_overrun_flag(void *base);


/*
 * Gives command to CTU CAN FD Core to abbort the transmission immediately.
 * This action will most likely result in transmission of Error frame.
 * 
 * Arguments:
 *	base	Pointer to the base address
 */
void ctu_can_fd_abort_tx(void *base);


/*
 * Returns mode/status vector of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 
 * Returns:
 *	Mode/status structure with multiple mode flags.
 */
union ctu_can_fd_mode_command_status_settings ctu_can_get_status(const void *base);

/*
 * Reads the interrupt status vector from CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 
 * Returns:
 *	Interrupt status vector.
 */
union ctu_can_fd_int_stat ctu_can_fd_int_sts(const void *base);


/*
 * Clears the interrupts from CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	mask	Mask of interrupts which should be cleared.
 */
void ctu_can_fd_int_clr(void *base, const union ctu_can_fd_int_stat *mask);


/*
 * Enable/Disable interrupts of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	mask	Mask of interrupts which should be enabled/disabled.
 * 	val	0 - disable, 1 - enable the interrupt.
 */
void ctu_can_fd_int_ena(void *base, const union ctu_can_fd_int_stat *mask,
			const union ctu_can_fd_int_stat *val);


/*
 * Mask/Unmask interrupts of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	mask	Mask of interrupts which should be enabled/disabled.
 * 	val	0 - unmask, 1 - mask the interrupt.
 */
void ctu_can_fd_int_mask(void *base, const union ctu_can_fd_int_stat *mask,
				const union ctu_can_fd_int_stat *val);


/*
 * Set the modes of CTU CAN FD IP Core. All flags from "ctu_can_fd_set_mode_reg"
 * are configured, plus CAN_CTRLMODE_ONE_SHOT, CAN_CTRLMODE_BERR_REPORTING,
 * which are configured via "rettransmitt limit" and enabling error interrupts.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	mode	Mode of the controller from Socket CAN.
 */
void ctu_can_fd_set_mode(void *base, const struct can_ctrlmode *mode);


/*
 * Set Nominal bit timing of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	nbt	Nominal bit timing settings of CAN Controller.
 */
void ctu_can_fd_set_nom_bittiming(void *base, const struct can_bittiming *nbt);


/*
 * Set Data bit timing of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	nbt	Data bit timing settings of CAN Controller.
 */
void ctu_can_fd_set_data_bittiming(void *base, const struct can_bittiming *dbt);


/*
 * Set error limit when CTU CAN FD Core should transfer to Error warning
 * and error passive states. If any of RX/TX counters reach this value
 * according state is changed. By default these counters are set as in
 * CAN Standard (96, 128).
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	ewl	Error warning limit
 *	erp	Error passive limit
 */
void ctu_can_fd_set_err_limits(void *base, u8 ewl, u8 erp);


/*
 * Set default error limits to the CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 */
void ctu_can_fd_set_def_err_limits(void *base);


/*
 * Read TX/RX error counters of CTU CAN FD IP Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	ctr	Pointer to error counter structure to fill
 * Returns:
 *	True if read succesfully, false otherwise.
 */
bool ctu_can_fd_read_err_ctrs(const void *base, struct can_berr_counter *ctr);


/*
 * Read special error counter which returns number of Errors which were
 * detected during Nominal Bit-rate.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	Number of Error frames detected during Nominal Bit-rate
 */
u16 ctu_can_fd_read_nom_errs(const void *base);


/*
 * Give command to CTU CAN FD Core to erase the nominal error counter.
 * 
 * Arguments:
 *	base	Pointer to the base address
 */
void ctu_can_fd_erase_nom_errs(void *base);


/*
 * Read special error counter which returns number of Errors which were
 * detected during Data Bit-rate.
 *
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	Number of Error frames detected during Data Bit-rate
 */
u16 ctu_can_fd_read_fd_errs(const void *base);


/*
 * Give command to CTU CAN FD Core to erase the Data error counter.
 * 
 * Arguments:
 *	base	Pointer to the base address
 */
void ctu_can_fd_erase_fd_errs(void *base);


/*
 * Read fault confinement state of CTU CAN FD Core 
 * (determined by TX/RX Counters).
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	Error state of the CTU CAN FD Core.
 */
enum can_state ctu_can_fd_read_error_state(const void *base);


/*
 * Set value to TX/RX error counters of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	Error state of the CTU CAN FD Core.
 */
void ctu_can_fd_set_err_ctrs(void *base, const struct can_berr_counter *ctr);


/*
 * Check Mask filters support of given filter.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * 	fnum	Filter number.
 * Returns:
 *	True if filter is present and can be used, False otherwise.
 */
bool ctu_can_fd_get_mask_filter_support(const void *base, u8 fnum);


/*
 * Check Range filter support of given filter.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	True if Range filter is present and can be used, False otherwise.
 */
bool ctu_can_fd_get_range_filter_support(const void *base);


/*
 * Configure mask filter of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	fnum	Filter number.
 *	enable  True if filter should be enabled.
 *	filter	Filter configuration.
 * Returns:
 *	True if mask filter was configured properly, false otherwise.
 */
bool ctu_can_fd_set_mask_filter(void *base, u8 fnum, bool enable,
				const struct can_filter *filter);

/*
 * Configure range filter of CTU CAN FD Core. An identifier of RX Frame
 * will pass the Range filter if its decimal value is between lower and
 * upper threshold of range filter.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	low_th	Lower threshold of identifiers which should be accepted
 *	high_th	Upper threshold of identifiers which should be accepted
 *	enable	Enable the range filter.
 */
void ctu_can_fd_set_range_filter(void *base, canid_t low_th,
				 canid_t high_th, bool enable);

/*
 * Get size of the RX FIFO Buffer of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	Size of the RX Buffer in words (32 bit)
 */
u16 ctu_can_fd_get_rx_fifo_size(const void *base);


/*
 * Get number of free words in RX FIFO Buffer of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	Number of free words (32 bit) in RX Buffer.
 */
u16 ctu_can_fd_get_rx_fifo_mem_free(const void *base);


/*
 * Check if RX FIFO Buffer is empty.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	True if empty, false otherwise.
 */
bool ctu_can_fd_is_rx_fifo_empty(const void *base);


/*
 * Check if RX FIFO Buffer is full.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	True if Full, false otherwise.
 */
bool ctu_can_fd_is_rx_fifo_full(const void *base);


/*
 * Get number of CAN Frames stored in RX Buffer of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	True if Full, false otherwise.
 */
u16 ctu_can_fd_get_rx_frame_count(const void *base);


/*
 * Set timestamp option on RX Frame.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	val	Timestamp option settings.
 */
void ctu_can_fd_set_rx_tsop(void *base, enum ctu_can_fd_rx_settings_rtsop val);


/*
 * Reads CAN Frame from RX FIFO Buffer and stores it to a buffer. 
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	data	Pointer to buffer where the CAN Frame should be stored.
 *	ts	Pointer to u64 where RX Timestamp should be stored.
 */
void ctu_can_fd_read_rx_frame(const void *base, unsigned char *data, u64 *ts);


/*
 * Returns status of TXT Buffer.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	buf	TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Returns:
 *	Status of the TXT Buffer.
 */
enum ctu_can_fd_tx_status_tx1s ctu_can_fd_get_tx_status(const void *base, u8 buf);


/*
 * Checks if TXT Buffer is accessible and can be written to.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	buf	TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Returns:
 *	Status of the TXT Buffer.
 */
bool ctu_can_fd_is_txt_buf_accessible(const void *base, u8 buf);


/*
 * Give command to TXT Buffer of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	cmd	Command line buffer. 
 *	buf	TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Returns:
 *	Status of the TXT Buffer.
 */
bool ctu_can_fd_txt_buf_give_command(void *base, u8 cmd, u8 buf);


/*
 * Give "set_empty" command to TXT Buffer.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	buf	TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Returns:
 *	Status of the TXT Buffer.
 */
static inline void ctu_can_fd_txt_set_empty(void *base, u8 buf)
{
    ctu_can_fd_txt_buf_give_command(base, 0x1, buf);
}


/*
 * Give "set_ready" command to TXT Buffer.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	buf	TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Returns:
 *	Status of the TXT Buffer.
 */
static inline void ctu_can_fd_txt_set_rdy(void *base, u8 buf)
{
    ctu_can_fd_txt_buf_give_command(base, 0x2, buf);
}


/*
 * Give "set_abort" command to TXT Buffer.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	buf	TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Returns:
 *	Status of the TXT Buffer.
 */
static inline void ctu_can_fd_txt_set_abort(void *base, u8 buf)
{
    ctu_can_fd_txt_buf_give_command(base, 0x4, buf);
}


/*
 * Set priority of TXT Buffers in CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	prio	Pointer to array with CTU_CAN_FD_TXT_BUFFER_COUNT number
 *		of elements with TXT Buffer priorities.
 */
void ctu_can_fd_set_txt_priority(void *base, const u8 *prio);


/*
 * Insert CAN FD frame to TXT Buffer of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 *	data	Pointer to CAN Frame buffer.
 *	u64	Timestamp when the buffer should be sent.
 *	buf	Index of TXT Buffer where to insert the CAN Frame.
 * Returns:
 *	True if the frame was inserted succesfully, False otherwise.
 */
bool ctu_can_fd_insert_frame(void *base, const unsigned char *data, u64 ts,
				u8 buf);

/*
 * Read transceiver delay as measured by CTU CAN FD Core. Note that
 * transceiver delay can be measured only after at least one CAN FD Frame with
 * BRS bit was sent since the last re-start of the Core!
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	True if the frame was inserted succesfully, False otherwise.
 */
u16 ctu_can_fd_get_tran_delay(const void *base);


/*
 * Read number of transmitted CAN/CAN FD Frames by CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	Number of received CAN/CAN FD frames.
 */
u32 ctu_can_fd_get_tx_frame_ctr(const void *base);


/*
 * Read number of received CAN/CAN FD Frames by CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	Number of received CAN/CAN FD frames.
 */
u32 ctu_can_fd_get_rx_frame_ctr(const void *base);


/*
 * Returns debug information of CTU CAN FD Core.
 * 
 * Arguments:
 *	base	Pointer to the base address
 * Returns:
 *	Content of Debug register.
 */
union ctu_can_fd_debug_register ctu_can_fd_read_debug_info(const void *base);

extern const struct can_bittiming_const ctu_can_fd_bit_timing_max;
extern const struct can_bittiming_const ctu_can_fd_bit_timing_data_max;

#endif
