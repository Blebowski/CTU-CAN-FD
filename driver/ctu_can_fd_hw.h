// SPDX-License-Identifier: GPL-2.0+
/*******************************************************************************
 *
 * CTU CAN FD IP Core
 * Copyright (C) 2015-2018
 *
 * Authors:
 *     Ondrej Ille <ondrej.ille@gmail.com>
 *     Martin Jerabek <martin.jerabek01@gmail.com>
 *
 * Project advisors:
 *     Jiri Novak <jnovak@fel.cvut.cz>
 *     Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
 ******************************************************************************/

#ifndef __CTU_CAN_FD_HW__
#define __CTU_CAN_FD_HW__

#include <asm/byteorder.h>

#if defined(__LITTLE_ENDIAN_BITFIELD) == defined(__BIG_ENDIAN_BITFIELD)
# error Either __BIG_ENDIAN_BITFIELD or __LITTLE_ENDIAN_BITFIELD must be defined.
#endif

#include "ctu_can_fd_regs.h"
#include "ctu_can_fd_frame.h"

/*
	MJ TODO:
	+ consider move of more small functions to *.h, make them inline
	+ either pass union arguments by value or just as u32;
	  this way they are forced on stack instead of passing in register
	  + one level of pointer indirection, which sucks performance-wise
	  - use u32 directly, as non-primitive types (however small )are not
		guaranteed to be passed in registers across all ABIs
*/

#define CTU_CAN_FD_RETR_MAX 15

#define CTU_CAN_FD_FILTER_A 0
#define CTU_CAN_FD_FILTER_B 1
#define CTU_CAN_FD_FILTER_C 2

#define CTU_CAN_FD_TXT_BUFFER_COUNT 4

#define CTU_CAN_FD_TXT_BUFFER_1 0
#define CTU_CAN_FD_TXT_BUFFER_2 1
#define CTU_CAN_FD_TXT_BUFFER_3 2
#define CTU_CAN_FD_TXT_BUFFER_4 3

/*
 * Status macros -> pass "ctu_can_get_status" result
 */

// True if Core is transceiver of current frame
#define CTU_CAN_FD_IS_TRANSMITTER(stat) (!!(stat).ts)

// True if Core is receiver of current frame
#define CTU_CAN_FD_IS_RECEIVER(stat) (!!(stat).s.rxs)

// True if Core is idle (integrating or interfame space)
#define CTU_CAN_FD_IS_IDLE(stat) (!!(stat).s.idle)

// True if Core is transmitting error frame
#define CTU_CAN_FD_ERR_FRAME(stat) (!!(stat).s.eft)

// True if Error warning limit was reached
#define CTU_CAN_FD_EWL(stat) (!!(stat).s.ewl)

// True if at least one TXT Buffer is empty
#define CTU_CAN_FD_TXTNF(stat) (!!(stat).s.txnf)

// True if data overrun flag of RX Buffer occurred
#define CTU_CAN_FD_DATA_OVERRUN(stat) (!!(stat).s.dor)

// True if RX Buffer is not empty
#define CTU_CAN_FD_RX_BUF_NEMPTY(stat) (!!(stat).s.rxne)

/*
 * Interrupt macros -> pass "ctu_can_fd_int_sts" result
 */

// Frame reveived interrupt
#define CTU_CAN_FD_RX_INT(int_stat) (!!(int_stat).s.rxi)

// Frame transceived interrupt
#define CTU_CAN_FD_TX_INT(int_stat) (!!(int_stat).s.txi)

// Error warning limit reached interrupt
#define CTU_CAN_FD_EWL_INT(int_stat) (!!(int_stat).s.ewli)

// RX Buffer data overrun interrupt
#define CTU_CAN_FD_OVERRUN_INT(int_stat) (!!(int_stat).s.doi)

// Fault confinement changed interrupt
#define CTU_CAN_FD_FAULT_STATE_CHANGED_INT(int_stat) (!!(int_stat).s.fcsi)

// Error frame transmission started interrupt
#define CTU_CAN_FD_BUS_ERROR_INT(int_stat) (!!(int_stat).s.bei)

// Event logger finished interrupt
#define CTU_CAN_FD_LOGGER_FIN_INT(int_stat) (!!(int_stat).s.lfi)

// RX Buffer full interrupt
#define CTU_CAN_FD_RX_FULL_INT(int_stat) (!!(int_stat).s.rxfi)

// Bit-rate shifted interrupt
#define CTU_CAN_FD_BIT_RATE_SHIFT_INT(int_stat) (!!(int_stat).s.bsi)

// Receive buffer not empty interrupt
#define CTU_CAN_FD_RX_BUF_NEPMTY_INT(int_stat) (!!(int_stat).s.rbnei)

// TX Buffer received HW command interrupt
#define CTU_CAN_FD_TXT_BUF_HWCMD_INT(int_stat) (!!(int_stat).s.txbhci)

static inline bool CTU_CAN_FD_INT_ERROR(union ctu_can_fd_int_stat i)
{
	return i.s.ewli || i.s.doi || i.s.fcsi || i.s.ali;
}

struct ctucanfd_priv;
#ifndef ctucanfd_priv
struct ctucanfd_priv {
	void __iomem *mem_base;
	u32 (*read_reg)(struct ctucanfd_priv *priv,
			enum ctu_can_fd_can_registers reg);
	void (*write_reg)(struct ctucanfd_priv *priv,
			enum ctu_can_fd_can_registers reg, u32 val);
};
#endif

void ctu_can_fd_write32(struct ctucanfd_priv *priv,
			enum ctu_can_fd_can_registers reg, u32 val);
void ctu_can_fd_write32_be(struct ctucanfd_priv *priv,
			enum ctu_can_fd_can_registers reg, u32 val);
u32 ctu_can_fd_read32(struct ctucanfd_priv *priv,
			enum ctu_can_fd_can_registers reg);
u32 ctu_can_fd_read32_be(struct ctucanfd_priv *priv,
			enum ctu_can_fd_can_registers reg);

/**
 * ctu_can_fd_check_access - Checks whether the core is mapped correctly at it's base address.
 *
 * @priv: Private info
 *
 * Return: true if the core is accessible correctly, false otherwise.
 */
bool ctu_can_fd_check_access(struct ctucanfd_priv *priv);

/**
 * ctu_can_fd_get_version - Returns version of CTU CAN FD IP Core.
 *
 * @priv: Private info
 *
 * Return: IP Core version in format major*10 + minor
 */
u32 ctu_can_fd_get_version(struct ctucanfd_priv *priv);

/**
 * ctu_can_fd_enable - Enables/disables the operation of CTU CAN FD Core.
 *
 * If disabled, the Core will never start transmitting on the CAN bus, nor receiving.
 *
 * @priv: Private info
 * @enable: Enable/disable the core.
 */
void ctu_can_fd_enable(struct ctucanfd_priv *priv, bool enable);

/**
 * ctu_can_fd_reset - Resets the CTU CAN FD Core.
 *
 * NOTE: After resetting, you must wait until ctu_can_fd_check_access() succeeds!
 *
 * @priv: Private info
 */
void ctu_can_fd_reset(struct ctucanfd_priv *priv);

/**
 * ctu_can_fd_set_ret_limit - Set retransmit limit for sent messages
 *
 * Configures CTU CAN FD Core to limit the amount of retransmit attempts after
 * occurence of error (Error frame, Arbitration lost). If retransmit limit is
 * disabled, the Core will attempt to retransmit inifinitely. If retransmit
 * limit is reached, the Core will finish and according TXT buffer will end up
 * in TX Error state.
 *
 * @priv: Private info
 * @enable: Enable/disable the retransmit limitation
 * @limit: Number to which limit the retransmission (1-CTU_CAN_FD_RETR_MAX)
 * Return: True if set correctly. False if "limit" is too high.
 */
bool ctu_can_fd_set_ret_limit(struct ctucanfd_priv *priv, bool enable,
			      u8 limit);

/**
 * ctu_can_fd_set_mode_reg - Configures CTU CAN FD Core for special operating modes by access to MODEregister.
 *
 * Following flags from "mode" are not configured by this function:
 *  CAN_CTRLMODE_ONE_SHOT, CAN_CTRLMODE_BERR_REPORTING.
 *
 * Following flags are configured:
 *
 *	CAN_CTRLMODE_LOOPBACK	- Bit loopback mode. Every dominant bit is
 *				  re-routed internally and not send on the bus.
 *
 *	CAN_CTRLMODE_LISTENONLY	- No frame is transmitted, no dominant bit is
 *				  sent on the bus.
 *
 *	CAN_CTRLMODE_3_SAMPLES  - Tripple sampling mode
 *
 *	CAN_CTRLMODE_FD		- Flexible data-rate support. When not set, Core
 *				  does not accept CAN FD Frames and interprets,
 *				  them as form error. Capability to transmit
 *				  CAN FD Frames is not affected by this setting.
 *
 *	CAN_CTRLMODE_PRESUME_ACK - When set, Core does not require dominant bit
 *				   in ACK field to consider the transmission as
 *				   valid.
 *
 *	CAN_CTRLMODE_FD_NON_ISO  - When set, the Core transmits the frames
 *				   according to NON-ISO FD standard.
 *
 * @priv: Private info
 * @mode: CAN mode to be set to on the Core.
 */
void ctu_can_fd_set_mode_reg(struct ctucanfd_priv *priv,
			     const struct can_ctrlmode *mode);

/**
 * ctu_can_fd_rel_rx_buf - Gives command to CTU CAN FD Core to erase and reset the RX FIFO.
 *
 * This action is finished immediately and does not need waiting.
 *
 * @priv: Private info
 */
void ctu_can_fd_rel_rx_buf(struct ctucanfd_priv *priv);

/**
 * ctu_can_fd_clr_overrun_flag - Gives command to CTU CAN FD Core to clear the Data overrun flag on the RX FIFO Buffer.
 *
 * @priv: Private info
 */
void ctu_can_fd_clr_overrun_flag(struct ctucanfd_priv *priv);

/**
 * ctu_can_fd_abort_tx - Gives command to CTU CAN FD Core to abort the transmission immediately.
 *
 * This action will most likely result in transmission of Error frame.
 *
 * @priv: Private info
 */
void ctu_can_fd_abort_tx(struct ctucanfd_priv *priv);

/**
 * ctu_can_get_status - Returns mode/status vector of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Mode/status structure with multiple mode flags.
 */
static inline union ctu_can_fd_status
	ctu_can_get_status(struct ctucanfd_priv *priv)
{
	/* MODE and STATUS are within the same word */
	union ctu_can_fd_status res;

	res.u32 = priv->read_reg(priv, CTU_CAN_FD_STATUS);
	return res;
}

/**
 * ctu_can_fd_is_enabled - Test if core is enabled..
 *
 * @priv: Private info
 *
 * Return: Return true if core is in enabled/active state..
 */
static inline bool ctu_can_fd_is_enabled(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_mode_settings reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_MODE);
	return reg.s.ena == CTU_CAN_ENABLED;
}

/**
 * ctu_can_fd_int_sts - Reads the interrupt status vector from CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Interrupt status vector.
 */
static inline union ctu_can_fd_int_stat
	ctu_can_fd_int_sts(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_int_stat res;

	res.u32 = priv->read_reg(priv, CTU_CAN_FD_INT_STAT);
	return res;
}

/**
 * ctu_can_fd_int_clr - Clears the interrupts from CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be cleared.
 */
static inline void ctu_can_fd_int_clr(struct ctucanfd_priv *priv,
				      union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_STAT, mask.u32);
}

/**
 * ctu_can_fd_int_ena_set - Sets enable interrupt bits.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be disabled.
 */
static inline void ctu_can_fd_int_ena_set(struct ctucanfd_priv *priv,
					  union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_ENA_SET, mask.u32);
}

/**
 * ctu_can_fd_int_ena_clr - Clears enable interrupt bits.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be disabled.
 */
static inline void ctu_can_fd_int_ena_clr(struct ctucanfd_priv *priv,
					  union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_ENA_CLR, mask.u32);
}

/**
 * ctu_can_fd_int_ena - Enable/Disable interrupts of CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be enabled/disabled.
 * @val: 0 - disable, 1 - enable the interrupt.
 */
void ctu_can_fd_int_ena(struct ctucanfd_priv *priv,
			union ctu_can_fd_int_stat mask,
			union ctu_can_fd_int_stat val);

/**
 * ctu_can_fd_int_mask_set - Mask interrupts of CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be masked.
 */
static inline void ctu_can_fd_int_mask_set(struct ctucanfd_priv *priv,
					   union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_MASK_SET, mask.u32);
}

/**
 * ctu_can_fd_int_mask_clr - Unmask interrupts of CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be unmasked.
 */
static inline void ctu_can_fd_int_mask_clr(struct ctucanfd_priv *priv,
					   union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_MASK_CLR, mask.u32);
}

/**
 * ctu_can_fd_int_mask - Mask/Unmask interrupts of CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be enabled/disabled.
 * @val: 0 - unmask, 1 - mask the interrupt.
 */
void ctu_can_fd_int_mask(struct ctucanfd_priv *priv,
			 union ctu_can_fd_int_stat mask,
			 union ctu_can_fd_int_stat val);

/**
 * ctu_can_fd_set_mode - Set the modes of CTU CAN FD IP Core. All flags from "ctu_can_fd_set_mode_reg"
 * are configured, plus CAN_CTRLMODE_ONE_SHOT, CAN_CTRLMODE_BERR_REPORTING,
 * which are configured via "retransmit limit" and enabling error interrupts.
 *
 * @priv: Private info
 * @mode: Mode of the controller from Socket CAN.
 */
void ctu_can_fd_set_mode(struct ctucanfd_priv *priv,
			 const struct can_ctrlmode *mode);

/**
 * ctu_can_fd_set_nom_bittiming - Set Nominal bit timing of CTU CAN FD Core.
 *
 * NOTE: phase_seg1 and prop_seg may be modified if phase_seg1 > 63
 *       This is because in Linux, the constraints are only
 *       on phase_seg1+prop_seg.
 *
 * @priv: Private info
 * @nbt: Nominal bit timing settings of CAN Controller.
 */
void ctu_can_fd_set_nom_bittiming(struct ctucanfd_priv *priv,
				  struct can_bittiming *nbt);

/**
 * ctu_can_fd_set_data_bittiming - Set Data bit timing of CTU CAN FD Core.
 *
 * NOTE: phase_seg1 and prop_seg may be modified if phase_seg1 > 63
 *       This is because in Linux, the constraints are only
 *       on phase_seg1+prop_seg.
 *
 * @priv: Private info
 * @dbt: Data bit timing settings of CAN Controller.
 */
void ctu_can_fd_set_data_bittiming(struct ctucanfd_priv *priv,
				   struct can_bittiming *dbt);

/**
 * ctu_can_fd_set_err_limits - Set limits for error warning and passive transition
 *
 * Set error limit when CTU CAN FD Core should transfer to Error warning
 * and error passive states. If any of RX/TX counters reach this value
 * according state is changed. By default these counters are set as in
 * CAN Standard (96, 128).
 *
 * @priv: Private info
 * @ewl: Error warning limit
 * @erp: Error passive limit
 */
void ctu_can_fd_set_err_limits(struct ctucanfd_priv *priv, u8 ewl, u8 erp);

/**
 * ctu_can_fd_set_def_err_limits - Set default error limits to the CTU CAN FD Core.
 *
 * @priv: Private info
 */
static inline void ctu_can_fd_set_def_err_limits(struct ctucanfd_priv *priv)
{
	ctu_can_fd_set_err_limits(priv, 96, 128);
}

/**
 * ctu_can_fd_read_err_ctrs - Read TX/RX error counters of CTU CAN FD IP Core.
 *
 * @priv: Private info
 * @ctr: Pointer to error counter structure to fill
 */
void ctu_can_fd_read_err_ctrs(struct ctucanfd_priv *priv,
			      struct can_berr_counter *ctr);

/**
 * ctu_can_fd_read_nom_errs - Read special error counter which returns number of Errors which were
 * detected during Nominal Bit-rate.
 *
 * @priv: Private info
 * Return: Number of Error frames detected during Nominal Bit-rate
 */
static inline u16 ctu_can_fd_read_nom_errs(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_err_norm_err_fd reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_ERR_NORM);
	return reg.s.err_norm_val;
}

/**
 * ctu_can_fd_erase_nom_errs - Give command to CTU CAN FD Core to erase the nominal error counter.
 *
 * @priv: Private info
 */
static inline void ctu_can_fd_erase_nom_errs(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_ctr_pres reg;

	reg.u32 = 0;
	reg.s.enorm = 1;
	priv->write_reg(priv, CTU_CAN_FD_CTR_PRES, reg.u32);
}

/**
 * ctu_can_fd_read_fd_errs - Read special error counter which returns number of Errors which were
 * detected during Data Bit-rate.
 *
 * @priv: Private info
 * Return: Number of Error frames detected during Data Bit-rate
 */
static inline u16 ctu_can_fd_read_fd_errs(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_err_norm_err_fd reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_ERR_NORM);
	return reg.s.err_fd_val;
}

/**
 * ctu_can_fd_erase_fd_errs - Give command to CTU CAN FD Core to erase the Data error counter.
 *
 * @priv: Private info
 */
static inline void ctu_can_fd_erase_fd_errs(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_ctr_pres reg;

	reg.u32 = 0;
	reg.s.efd = 1;
	priv->write_reg(priv, CTU_CAN_FD_CTR_PRES, reg.u32);
}

/**
 * ctu_can_fd_read_error_state - Read fault confinement state of CTU CAN FD Core (determined by TX/RX Counters).
 *
 * @priv: Private info
 * Return: Error state of the CTU CAN FD Core.
 */
enum can_state ctu_can_fd_read_error_state(struct ctucanfd_priv *priv);

/**
 * ctu_can_fd_set_err_ctrs - Set value to TX/RX error counters of CTU CAN FD Core.
 *
 * @priv: Private info
 * @ctr: Value to be set into counters
 * Return: Error state of the CTU CAN FD Core.
 */
void ctu_can_fd_set_err_ctrs(struct ctucanfd_priv *priv,
			     const struct can_berr_counter *ctr);

/*
 * ctu_can_fd_read_err_capt_alc - Read core captured last error or arbitration lost reason.
 *
 * @priv: Private info
 * Return: Error state of the CTU CAN FD.
 */
static inline union ctu_can_fd_err_capt_alc
        ctu_can_fd_read_err_capt_alc(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_err_capt_alc res;
	res.u32 = priv->read_reg(priv, CTU_CAN_FD_ERR_CAPT);
	return res;
}

/**
 * ctu_can_fd_get_mask_filter_support - Check Mask filters support of given filter.
 *
 * @priv: Private info
 * @fnum: Filter number.
 * Return: True if filter is present and can be used, False otherwise.
 */
bool ctu_can_fd_get_mask_filter_support(struct ctucanfd_priv *priv, u8 fnum);

/**
 * ctu_can_fd_get_range_filter_support - Check Range filter support of given filter.
 *
 * @priv: Private info
 * Return: True if Range filter is present and can be used, False otherwise.
 */
bool ctu_can_fd_get_range_filter_support(struct ctucanfd_priv *priv);

/**
 * ctu_can_fd_set_mask_filter - Configure mask filter of CTU CAN FD Core.
 *
 * @priv: Private info
 * @fnum: Filter number.
 * @enable: True if filter should be enabled.
 * @filter: Filter configuration.
 * Return: True if mask filter was configured properly, false otherwise.
 */
bool ctu_can_fd_set_mask_filter(struct ctucanfd_priv *priv, u8 fnum,
				bool enable, const struct can_filter *filter);

/**
 * ctu_can_fd_set_range_filter - Configure range filter of CTU CAN FD Core.
 *
 * An identifier of RX Frame will pass the Range filter if its decimal value
 * is between lower and upper threshold of range filter.
 *
 * @priv: Private info
 * @low_th: Lower threshold of identifiers which should be accepted
 * @high_th: Upper threshold of identifiers which should be accepted
 * @enable: Enable the range filter.
 */
void ctu_can_fd_set_range_filter(struct ctucanfd_priv *priv, canid_t low_th,
				 canid_t high_th, bool enable);

/**
 * ctu_can_fd_get_rx_fifo_size - Get size of the RX FIFO Buffer of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Size of the RX Buffer in words (32 bit)
 */
static inline u16 ctu_can_fd_get_rx_fifo_size(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_rx_mem_info reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_MEM_INFO);
	return reg.s.rx_buff_size;
}

/**
 * ctu_can_fd_get_rx_fifo_mem_free - Get number of free words in RX FIFO Buffer of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Number of free words (32 bit) in RX Buffer.
 */
static inline u16 ctu_can_fd_get_rx_fifo_mem_free(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_rx_mem_info reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_MEM_INFO);
	return reg.s.rx_mem_free;
}

/**
 * ctu_can_fd_is_rx_fifo_empty - Check if RX FIFO Buffer is empty.
 *
 * @priv: Private info
 * Return: True if empty, false otherwise.
 */
static inline bool ctu_can_fd_is_rx_fifo_empty(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_rx_status_rx_settings reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_STATUS);
	return reg.s.rxe;
}

/**
 * ctu_can_fd_is_rx_fifo_full - Check if RX FIFO Buffer is full.
 *
 * @priv: Private info
 * Return: True if Full, false otherwise.
 */
static inline bool ctu_can_fd_is_rx_fifo_full(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_rx_status_rx_settings reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_STATUS);
	return reg.s.rxf;
}

/**
 * ctu_can_fd_get_rx_frame_count - Get number of CAN Frames stored in RX Buffer of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: True if Full, false otherwise.
 */
static inline u16 ctu_can_fd_get_rx_frame_count(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_rx_status_rx_settings reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_STATUS);
	return reg.s.rxfrc;
}

/**
 * ctu_can_fd_set_rx_tsop - Set timestamp option on RX Frame.
 *
 * @priv: Private info
 * @val: Timestamp option settings.
 */
void ctu_can_fd_set_rx_tsop(struct ctucanfd_priv *priv,
			    enum ctu_can_fd_rx_settings_rtsop val);

/*
 * ctu_can_fd_read_rx_ffw - Reads the first word of CAN Frame from RX FIFO Buffer.
 *
 * @priv: Private info
 *
 * Return: The firts word of received frame
 */
static inline union ctu_can_fd_frame_form_w
	ctu_can_fd_read_rx_ffw(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_frame_form_w ffw;

	ffw.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_DATA);
	return ffw;
}

/**
 * ctu_can_fd_read_rx_word - Reads one word of CAN Frame from RX FIFO Buffer.
 *
 * @priv: Private info
 *
 * Return: One wword of received frame
 */
static inline u32 ctu_can_fd_read_rx_word(struct ctucanfd_priv *priv)
{
	return priv->read_reg(priv, CTU_CAN_FD_RX_DATA);
}

/**
 * ctu_can_fd_read_rx_frame - Reads CAN Frame from RX FIFO Buffer and stores it to a buffer.
 *
 * @priv: Private info
 * @data: Pointer to buffer where the CAN Frame should be stored.
 * @ts: Pointer to u64 where RX Timestamp should be stored.
 */
void ctu_can_fd_read_rx_frame(struct ctucanfd_priv *priv,
			      struct canfd_frame *data, u64 *ts);

/**
 * ctu_can_fd_read_rx_frame_ffw - Reads rest of CAN Frame from RX FIFO Buffer and stores it to a buffer.
 *
 * @priv: Private info
 * @cf: Pointer to buffer where the CAN Frame should be stored.
 * @ts: Pointer to u64 where RX Timestamp should be stored.
 * @ffw: Already read the first frame control word by the caller
 */
void ctu_can_fd_read_rx_frame_ffw(struct ctucanfd_priv *priv,
				  struct canfd_frame *cf, u64 *ts,
				  union ctu_can_fd_frame_form_w ffw);

/*
 * ctu_can_fd_get_tx_status - Returns status of TXT Buffer.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Return: Status of the TXT Buffer.
 */
enum ctu_can_fd_tx_status_tx1s
	ctu_can_fd_get_tx_status(struct ctucanfd_priv *priv, u8 buf);

/**
 * ctu_can_fd_is_txt_buf_accessible - Checks if TXT Buffer is accessible and can be written to.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Return: Status of the TXT Buffer.
 */
bool ctu_can_fd_is_txt_buf_accessible(struct ctucanfd_priv *priv, u8 buf);

/**
 * ctu_can_fd_txt_buf_give_command - Give command to TXT Buffer of CTU CAN FD Core.
 *
 * @priv: Private info
 * @cmd: Command line buffer.
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Return: Status of the TXT Buffer.
 */
bool ctu_can_fd_txt_buf_give_command(struct ctucanfd_priv *priv, u8 cmd,
				     u8 buf);

/**
 * ctu_can_fd_txt_set_empty - Give "set_empty" command to TXT Buffer.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Return: Status of the TXT Buffer.
 */
static inline void ctu_can_fd_txt_set_empty(struct ctucanfd_priv *priv, u8 buf)
{
	ctu_can_fd_txt_buf_give_command(priv, 0x1, buf);
}

/**
 * ctu_can_fd_txt_set_rdy - Give "set_ready" command to TXT Buffer.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Return: Status of the TXT Buffer.
 */
static inline void ctu_can_fd_txt_set_rdy(struct ctucanfd_priv *priv, u8 buf)
{
	ctu_can_fd_txt_buf_give_command(priv, 0x2, buf);
}

/**
 * ctu_can_fd_txt_set_abort - Give "set_abort" command to TXT Buffer.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Return: Status of the TXT Buffer.
 */
static inline void ctu_can_fd_txt_set_abort(struct ctucanfd_priv *priv, u8 buf)
{
	ctu_can_fd_txt_buf_give_command(priv, 0x4, buf);
}

/**
 * ctu_can_fd_set_txt_priority - Set priority of TXT Buffers in CTU CAN FD Core.
 *
 * @priv: Private info
 * @prio: Pointer to array with CTU_CAN_FD_TXT_BUFFER_COUNT number
 *		of elements with TXT Buffer priorities.
 */
void ctu_can_fd_set_txt_priority(struct ctucanfd_priv *priv, const u8 *prio);

/**
 * ctu_can_fd_insert_frame - Insert CAN FD frame to TXT Buffer of CTU CAN FD Core.
 *
 * @priv: Private info
 * @data: Pointer to CAN Frame buffer.
 * @ts: Timestamp when the buffer should be sent.
 * @buf: Index of TXT Buffer where to insert the CAN Frame.
 * @isfdf: True if the frame is a FD frame.
 * Return: True if the frame was inserted successfully, False otherwise.
 */
bool ctu_can_fd_insert_frame(struct ctucanfd_priv *priv,
			     const struct canfd_frame *data, u64 ts,
			     u8 buf, bool isfdf);

/**
 * ctu_can_fd_get_tran_delay - Read transceiver delay as measured by CTU CAN FD Core. Note that
 * transceiver delay can be measured only after at least one CAN FD Frame with
 * BRS bit was sent since the last re-start of the Core!
 *
 * @priv: Private info
 * Return: True if the frame was inserted successfully, False otherwise.
 */
static inline u16 ctu_can_fd_get_tran_delay(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_trv_delay_ssp_cfg reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_TRV_DELAY);
	return reg.s.trv_delay_value;
}

/**
 * ctu_can_fd_get_tx_frame_ctr - Read number of transmitted CAN/CAN FD Frames by CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Number of received CAN/CAN FD frames.
 */
static inline u32 ctu_can_fd_get_tx_frame_ctr(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_tx_counter reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_TX_COUNTER);
	return reg.s.tx_counter_val;
}

/**
 * ctu_can_fd_get_rx_frame_ctr - Read number of received CAN/CAN FD Frames by CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Number of received CAN/CAN FD frames.
 */
static inline u32 ctu_can_fd_get_rx_frame_ctr(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_rx_counter reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_COUNTER);
	return reg.s.rx_counter_val;
}

/*
 * ctu_can_fd_read_debug_info - Returns debug information of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Content of Debug register.
 */
static inline union ctu_can_fd_debug_register
	ctu_can_fd_read_debug_info(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_debug_register reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_DEBUG_REGISTER);
	return reg;
}

/**
 * ctu_can_fd_read_timestamp - Read timestamp value which is used internally by CTU CAN FD Core.
 * Reads timestamp twice and checks consistency betwen upper and
 * lower timestamp word.
 *
 * @priv: Private info
 * Return: Value of timestamp in CTU CAN FD Core
 */
u64 ctu_can_fd_read_timestamp(struct ctucanfd_priv *priv);

extern const struct can_bittiming_const ctu_can_fd_bit_timing_max;
extern const struct can_bittiming_const ctu_can_fd_bit_timing_data_max;

#endif
