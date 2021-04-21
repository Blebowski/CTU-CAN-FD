/* SPDX-License-Identifier: GPL-2.0-or-later */
/*******************************************************************************
 *
 * CTU CAN FD IP Core
 *
 * Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com> FEE CTU
 * Copyright (C) 2018-2020 Ondrej Ille <ondrej.ille@gmail.com> self-funded
 * Copyright (C) 2018-2019 Martin Jerabek <martin.jerabek01@gmail.com> FEE CTU
 * Copyright (C) 2018-2020 Pavel Pisa <pisa@cmp.felk.cvut.cz> FEE CTU/self-funded
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

#ifndef __CTUCANFD_HW__
#define __CTUCANFD_HW__

#include <asm/byteorder.h>

#if defined(__LITTLE_ENDIAN_BITFIELD) == defined(__BIG_ENDIAN_BITFIELD)
# error __BIG_ENDIAN_BITFIELD or __LITTLE_ENDIAN_BITFIELD must be defined.
#endif

#include "ctucanfd_regs.h"
#include "ctucanfd_frame.h"

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

/* True if Core is transceiver of current frame */
#define CTU_CAN_FD_IS_TRANSMITTER(stat) (!!(stat).ts)

/* True if Core is receiver of current frame */
#define CTU_CAN_FD_IS_RECEIVER(stat) (!!(stat).s.rxs)

/* True if Core is idle (integrating or interfame space) */
#define CTU_CAN_FD_IS_IDLE(stat) (!!(stat).s.idle)

/* True if Core is transmitting error frame */
#define CTU_CAN_FD_ERR_FRAME(stat) (!!(stat).s.eft)

/* True if Error warning limit was reached */
#define CTU_CAN_FD_EWL(stat) (!!(stat).s.ewl)

/* True if at least one TXT Buffer is empty */
#define CTU_CAN_FD_TXTNF(stat) (!!(stat).s.txnf)

/* True if data overrun flag of RX Buffer occurred */
#define CTU_CAN_FD_DATA_OVERRUN(stat) (!!(stat).s.dor)

/* True if RX Buffer is not empty */
#define CTU_CAN_FD_RX_BUF_NEMPTY(stat) (!!(stat).s.rxne)

/*
 * Interrupt macros -> pass "ctu_can_fd_int_sts" result
 */

/* Frame reveived interrupt */
#define CTU_CAN_FD_RX_INT(int_stat) (!!(int_stat).s.rxi)

/* Frame transceived interrupt */
#define CTU_CAN_FD_TX_INT(int_stat) (!!(int_stat).s.txi)

/* Error warning limit reached interrupt */
#define CTU_CAN_FD_EWL_INT(int_stat) (!!(int_stat).s.ewli)

/* RX Buffer data overrun interrupt */
#define CTU_CAN_FD_OVERRUN_INT(int_stat) (!!(int_stat).s.doi)

/* Fault confinement changed interrupt */
#define CTU_CAN_FD_FAULT_STATE_CHANGED_INT(int_stat) (!!(int_stat).s.fcsi)

/* Error frame transmission started interrupt */
#define CTU_CAN_FD_BUS_ERROR_INT(int_stat) (!!(int_stat).s.bei)

/* Event logger finished interrupt */
#define CTU_CAN_FD_LOGGER_FIN_INT(int_stat) (!!(int_stat).s.lfi)

/* RX Buffer full interrupt */
#define CTU_CAN_FD_RX_FULL_INT(int_stat) (!!(int_stat).s.rxfi)

/* Bit-rate shifted interrupt */
#define CTU_CAN_FD_BIT_RATE_SHIFT_INT(int_stat) (!!(int_stat).s.bsi)

/* Receive buffer not empty interrupt */
#define CTU_CAN_FD_RX_BUF_NEPMTY_INT(int_stat) (!!(int_stat).s.rbnei)

/* TX Buffer received HW command interrupt */
#define CTU_CAN_FD_TXT_BUF_HWCMD_INT(int_stat) (!!(int_stat).s.txbhci)

static inline bool CTU_CAN_FD_INT_ERROR(union ctu_can_fd_int_stat i)
{
	return i.s.ewli || i.s.doi || i.s.fcsi || i.s.ali;
}

struct ctucan_hw_priv {
	void __iomem *mem_base;
	u32 (*read_reg)(struct ctucan_hw_priv *priv,
			enum ctu_can_fd_can_registers reg);
	void (*write_reg)(struct ctucan_hw_priv *priv,
			  enum ctu_can_fd_can_registers reg, u32 val);
};

void ctucan_hw_write32(struct ctucan_hw_priv *priv,
		       enum ctu_can_fd_can_registers reg, u32 val);
void ctucan_hw_write32_be(struct ctucan_hw_priv *priv,
			  enum ctu_can_fd_can_registers reg, u32 val);
u32 ctucan_hw_read32(struct ctucan_hw_priv *priv,
		     enum ctu_can_fd_can_registers reg);
u32 ctucan_hw_read32_be(struct ctucan_hw_priv *priv,
			enum ctu_can_fd_can_registers reg);

/**
 * ctucan_hw_check_access - Checks whether the core is mapped correctly
 *                           at it's base address.
 *
 * @priv: Private info
 *
 * Return: true if the core is accessible correctly, false otherwise.
 */
bool ctucan_hw_check_access(struct ctucan_hw_priv *priv);

/**
 * ctucan_hw_get_version - Returns version of CTU CAN FD IP Core.
 *
 * @priv: Private info
 *
 * Return: IP Core version in format major*10 + minor
 */
u32 ctucan_hw_get_version(struct ctucan_hw_priv *priv);

/**
 * ctucan_hw_enable - Enables/disables the operation of CTU CAN FD Core.
 *
 * If disabled, the Core will never start transmitting on the CAN bus,
 * nor receiving.
 *
 * @priv: Private info
 * @enable: Enable/disable the core.
 */
void ctucan_hw_enable(struct ctucan_hw_priv *priv, bool enable);

/**
 * ctucan_hw_reset - Resets the CTU CAN FD Core.
 *
 * NOTE: After resetting, you must wait until ctucan_hw_check_access()
 * succeeds!
 *
 * @priv: Private info
 */
void ctucan_hw_reset(struct ctucan_hw_priv *priv);

/**
 * ctucan_hw_set_ret_limit - Set retransmit limit for sent messages
 *
 * Configures CTU CAN FD Core to limit the amount of retransmit attempts after
 * occurrence of error (Error frame, Arbitration lost). If retransmit limit is
 * disabled, the Core will attempt to retransmit inifinitely. If retransmit
 * limit is reached, the Core will finish and according TXT buffer will end up
 * in TX Error state.
 *
 * @priv: Private info
 * @enable: Enable/disable the retransmit limitation
 * @limit: Number to which limit the retransmission (1-CTU_CAN_FD_RETR_MAX)
 * Return: True if set correctly. False if "limit" is too high.
 */
bool ctucan_hw_set_ret_limit(struct ctucan_hw_priv *priv, bool enable,
			     u8 limit);

/**
 * ctucan_hw_set_mode_reg - Configures CTU CAN FD Core for special operating
 *                           modes by access to MODEregister.
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
void ctucan_hw_set_mode_reg(struct ctucan_hw_priv *priv,
			    const struct can_ctrlmode *mode);

/**
 * ctucan_hw_rel_rx_buf - Gives command to CTU CAN FD Core to erase
 *                         and reset the RX FIFO.
 *
 * This action is finished immediately and does not need waiting.
 *
 * @priv: Private info
 */
void ctucan_hw_rel_rx_buf(struct ctucan_hw_priv *priv);

/**
 * ctucan_hw_clr_overrun_flag - Gives command to CTU CAN FD Core to clear
 *                               the Data overrun flag on the RX FIFO Buffer.
 *
 * @priv: Private info
 */
void ctucan_hw_clr_overrun_flag(struct ctucan_hw_priv *priv);

/**
 * ctu_can_get_status - Returns mode/status vector of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Mode/status structure with multiple mode flags.
 */
static inline union ctu_can_fd_status
	ctu_can_get_status(struct ctucan_hw_priv *priv)
{
	/* MODE and STATUS are within the same word */
	union ctu_can_fd_status res;

	res.u32 = priv->read_reg(priv, CTU_CAN_FD_STATUS);
	return res;
}

/**
 * ctucan_hw_is_enabled - Test if core is enabled..
 *
 * @priv: Private info
 *
 * Return: Return true if core is in enabled/active state..
 */
static inline bool ctucan_hw_is_enabled(struct ctucan_hw_priv *priv)
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
	ctu_can_fd_int_sts(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_int_stat res;

	res.u32 = priv->read_reg(priv, CTU_CAN_FD_INT_STAT);
	return res;
}

/**
 * ctucan_hw_int_clr - Clears the interrupts from CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be cleared.
 */
static inline void ctucan_hw_int_clr(struct ctucan_hw_priv *priv,
				     union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_STAT, mask.u32);
}

/**
 * ctucan_hw_int_ena_set - Sets enable interrupt bits.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be disabled.
 */
static inline void ctucan_hw_int_ena_set(struct ctucan_hw_priv *priv,
					 union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_ENA_SET, mask.u32);
}

/**
 * ctucan_hw_int_ena_clr - Clears enable interrupt bits.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be disabled.
 */
static inline void ctucan_hw_int_ena_clr(struct ctucan_hw_priv *priv,
					 union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_ENA_CLR, mask.u32);
}

/**
 * ctucan_hw_int_ena - Enable/Disable interrupts of CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be enabled/disabled.
 * @val: 0 - disable, 1 - enable the interrupt.
 */
void ctucan_hw_int_ena(struct ctucan_hw_priv *priv,
		       union ctu_can_fd_int_stat mask,
		       union ctu_can_fd_int_stat val);

/**
 * ctucan_hw_int_mask_set - Mask interrupts of CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be masked.
 */
static inline void ctucan_hw_int_mask_set(struct ctucan_hw_priv *priv,
					  union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_MASK_SET, mask.u32);
}

/**
 * ctucan_hw_int_mask_clr - Unmask interrupts of CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be unmasked.
 */
static inline void ctucan_hw_int_mask_clr(struct ctucan_hw_priv *priv,
					  union ctu_can_fd_int_stat mask)
{
	priv->write_reg(priv, CTU_CAN_FD_INT_MASK_CLR, mask.u32);
}

/**
 * ctucan_hw_int_mask - Mask/Unmask interrupts of CTU CAN FD Core.
 *
 * @priv: Private info
 * @mask: Mask of interrupts which should be enabled/disabled.
 * @val: 0 - unmask, 1 - mask the interrupt.
 */
void ctucan_hw_int_mask(struct ctucan_hw_priv *priv,
			union ctu_can_fd_int_stat mask,
			union ctu_can_fd_int_stat val);

/**
 * ctucan_hw_set_mode - Set the modes of CTU CAN FD IP Core.
 *
 *All flags from "ctucan_hw_set_mode_reg" are configured,
 * plus CAN_CTRLMODE_ONE_SHOT, CAN_CTRLMODE_BERR_REPORTING,
 * which are configured via "retransmit limit" and enabling error interrupts.
 *
 * @priv: Private info
 * @mode: Mode of the controller from Socket CAN.
 */
void ctucan_hw_set_mode(struct ctucan_hw_priv *priv,
			const struct can_ctrlmode *mode);

/**
 * ctucan_hw_set_nom_bittiming - Set Nominal bit timing of CTU CAN FD Core.
 *
 * NOTE: phase_seg1 and prop_seg may be modified if phase_seg1 > 63
 *       This is because in Linux, the constraints are only
 *       on phase_seg1+prop_seg.
 *
 * @priv: Private info
 * @nbt: Nominal bit timing settings of CAN Controller.
 */
void ctucan_hw_set_nom_bittiming(struct ctucan_hw_priv *priv,
				 struct can_bittiming *nbt);

/**
 * ctucan_hw_set_data_bittiming - Set Data bit timing of CTU CAN FD Core.
 *
 * NOTE: phase_seg1 and prop_seg may be modified if phase_seg1 > 63
 *       This is because in Linux, the constraints are only
 *       on phase_seg1+prop_seg.
 *
 * @priv: Private info
 * @dbt: Data bit timing settings of CAN Controller.
 */
void ctucan_hw_set_data_bittiming(struct ctucan_hw_priv *priv,
				  struct can_bittiming *dbt);

/**
 * ctucan_hw_set_err_limits - Set limits for error warning and passive
 *                             transition
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
void ctucan_hw_set_err_limits(struct ctucan_hw_priv *priv, u8 ewl, u8 erp);

/**
 * ctucan_hw_set_def_err_limits - Set default error limits
 *                                 to the CTU CAN FD Core.
 *
 * @priv: Private info
 */
static inline void ctucan_hw_set_def_err_limits(struct ctucan_hw_priv *priv)
{
	ctucan_hw_set_err_limits(priv, 96, 128);
}

/**
 * ctucan_hw_read_err_ctrs - Read TX/RX error counters of CTU CAN FD IP Core.
 *
 * @priv: Private info
 * @ctr: Pointer to error counter structure to fill
 */
void ctucan_hw_read_err_ctrs(struct ctucan_hw_priv *priv,
			     struct can_berr_counter *ctr);

/**
 * ctucan_hw_read_nom_errs - Read special error counter which returns number
 *                        of Errors which were detected during Nominal Bit-rate.
 *
 * @priv: Private info
 * Return: Number of Error frames detected during Nominal Bit-rate
 */
static inline u16 ctucan_hw_read_nom_errs(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_err_norm_err_fd reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_ERR_NORM);
	return reg.s.err_norm_val;
}

/**
 * ctucan_hw_erase_nom_errs - Give command to CTU CAN FD Core to erase
 *                             the nominal error counter.
 *
 * @priv: Private info
 */
static inline void ctucan_hw_erase_nom_errs(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_ctr_pres reg;

	reg.u32 = 0;
	reg.s.enorm = 1;
	priv->write_reg(priv, CTU_CAN_FD_CTR_PRES, reg.u32);
}

/**
 * ctucan_hw_read_fd_errs - Read special error counter which returns number
 *                           of Errors which were detected during Data Bit-rate.
 *
 * @priv: Private info
 * Return: Number of Error frames detected during Data Bit-rate
 */
static inline u16 ctucan_hw_read_fd_errs(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_err_norm_err_fd reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_ERR_NORM);
	return reg.s.err_fd_val;
}

/**
 * ctucan_hw_erase_fd_errs - Give command to CTU CAN FD Core to erase the Data
 *                            error counter.
 *
 * @priv: Private info
 */
static inline void ctucan_hw_erase_fd_errs(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_ctr_pres reg;

	reg.u32 = 0;
	reg.s.efd = 1;
	priv->write_reg(priv, CTU_CAN_FD_CTR_PRES, reg.u32);
}

/**
 * ctucan_hw_read_error_state - Read fault confinement state of CTU CAN FD Core
 *                               (determined by TX/RX Counters).
 *
 * @priv: Private info
 * Return: Error state of the CTU CAN FD Core.
 */
enum can_state ctucan_hw_read_error_state(struct ctucan_hw_priv *priv);

/**
 * ctucan_hw_set_err_ctrs - Set value to TX/RX error counters
 *                           of CTU CAN FD Core.
 *
 * @priv: Private info
 * @ctr: Value to be set into counters
 * Return: Error state of the CTU CAN FD Core.
 */
void ctucan_hw_set_err_ctrs(struct ctucan_hw_priv *priv,
			    const struct can_berr_counter *ctr);

/**
 * ctu_can_fd_read_err_capt_alc - Read core captured last error or arbitration
 *                                lost reason.
 *
 * @priv: Private info
 * Return: Error state of the CTU CAN FD.
 */
static inline union ctu_can_fd_err_capt_retr_ctr_alc
		ctu_can_fd_read_err_capt_alc(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_err_capt_retr_ctr_alc res;

	res.u32 = priv->read_reg(priv, CTU_CAN_FD_ERR_CAPT);
	return res;
}

/**
 * ctucan_hw_get_mask_filter_support - Check Mask filters support
 *                                      of given filter.
 *
 * @priv: Private info
 * @fnum: Filter number.
 * Return: True if filter is present and can be used, False otherwise.
 */
bool ctucan_hw_get_mask_filter_support(struct ctucan_hw_priv *priv, u8 fnum);

/**
 * ctucan_hw_get_range_filter_support - Check Range filter support
 *                                       of given filter.
 *
 * @priv: Private info
 * Return: True if Range filter is present and can be used, False otherwise.
 */
bool ctucan_hw_get_range_filter_support(struct ctucan_hw_priv *priv);

/**
 * ctucan_hw_set_mask_filter - Configure mask filter of CTU CAN FD Core.
 *
 * @priv: Private info
 * @fnum: Filter number.
 * @enable: True if filter should be enabled.
 * @filter: Filter configuration.
 * Return: True if mask filter was configured properly, false otherwise.
 */
bool ctucan_hw_set_mask_filter(struct ctucan_hw_priv *priv, u8 fnum,
			       bool enable, const struct can_filter *filter);

/**
 * ctucan_hw_set_range_filter - Configure range filter of CTU CAN FD Core.
 *
 * An identifier of RX Frame will pass the Range filter if its decimal value
 * is between lower and upper threshold of range filter.
 *
 * @priv: Private info
 * @low_th: Lower threshold of identifiers which should be accepted
 * @high_th: Upper threshold of identifiers which should be accepted
 * @enable: Enable the range filter.
 */
void ctucan_hw_set_range_filter(struct ctucan_hw_priv *priv, canid_t low_th,
				canid_t high_th, bool enable);

/**
 * ctucan_hw_get_rx_fifo_size - Get size of the RX FIFO Buffer
 *                               of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Size of the RX Buffer in words (32 bit)
 */
static inline u16 ctucan_hw_get_rx_fifo_size(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_rx_mem_info reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_MEM_INFO);
	return reg.s.rx_buff_size;
}

/**
 * ctucan_hw_get_rx_fifo_mem_free - Get number of free words in RX FIFO Buffer
 *                                   of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Number of free words (32 bit) in RX Buffer.
 */
static inline u16 ctucan_hw_get_rx_fifo_mem_free(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_rx_mem_info reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_MEM_INFO);
	return reg.s.rx_mem_free;
}

/**
 * ctucan_hw_is_rx_fifo_empty - Check if RX FIFO Buffer is empty.
 *
 * @priv: Private info
 * Return: True if empty, false otherwise.
 */
static inline bool ctucan_hw_is_rx_fifo_empty(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_rx_status_rx_settings reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_STATUS);
	return reg.s.rxe;
}

/**
 * ctucan_hw_is_rx_fifo_full - Check if RX FIFO Buffer is full.
 *
 * @priv: Private info
 * Return: True if Full, false otherwise.
 */
static inline bool ctucan_hw_is_rx_fifo_full(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_rx_status_rx_settings reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_STATUS);
	return reg.s.rxf;
}

/**
 * ctucan_hw_get_rx_frame_count - Get number of CAN Frames stored in RX Buffer
 *                                 of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: True if Full, false otherwise.
 */
static inline u16 ctucan_hw_get_rx_frame_count(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_rx_status_rx_settings reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_STATUS);
	return reg.s.rxfrc;
}

/**
 * ctucan_hw_set_rx_tsop - Set timestamp option on RX Frame.
 *
 * @priv: Private info
 * @val: Timestamp option settings.
 */
void ctucan_hw_set_rx_tsop(struct ctucan_hw_priv *priv,
			   enum ctu_can_fd_rx_settings_rtsop val);

/**
 * ctu_can_fd_read_rx_ffw - Reads the first word of CAN Frame from RX FIFO
 *                          Buffer.
 *
 * @priv: Private info
 *
 * Return: The firts word of received frame
 */
static inline union ctu_can_fd_frame_format_w
	ctu_can_fd_read_rx_ffw(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_frame_format_w ffw;

	ffw.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_DATA);
	return ffw;
}

/**
 * ctucan_hw_read_rx_word - Reads one word of CAN Frame from RX FIFO Buffer.
 *
 * @priv: Private info
 *
 * Return: One word of received frame
 */
static inline u32 ctucan_hw_read_rx_word(struct ctucan_hw_priv *priv)
{
	return priv->read_reg(priv, CTU_CAN_FD_RX_DATA);
}

/**
 * ctucan_hw_read_rx_frame - Reads CAN Frame from RX FIFO Buffer and stores it
 *                            to a buffer.
 *
 * @priv: Private info
 * @data: Pointer to buffer where the CAN Frame should be stored.
 * @ts: Pointer to u64 where RX Timestamp should be stored.
 */
void ctucan_hw_read_rx_frame(struct ctucan_hw_priv *priv,
			     struct canfd_frame *data, u64 *ts);

/**
 * ctucan_hw_read_rx_frame_ffw - Reads rest of CAN Frame from RX FIFO Buffer
 *                                and stores it to a buffer.
 *
 * @priv: Private info
 * @cf: Pointer to buffer where the CAN Frame should be stored.
 * @ts: Pointer to u64 where RX Timestamp should be stored.
 * @ffw: Already read the first frame control word by the caller
 */
void ctucan_hw_read_rx_frame_ffw(struct ctucan_hw_priv *priv,
				 struct canfd_frame *cf, u64 *ts,
				 union ctu_can_fd_frame_format_w ffw);

/**
 * ctucan_hw_get_tx_status - Returns status of TXT Buffer.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Return: Status of the TXT Buffer.
 */
enum ctu_can_fd_tx_status_tx1s
	ctucan_hw_get_tx_status(struct ctucan_hw_priv *priv, u8 buf);

/**
 * ctucan_hw_is_txt_buf_accessible - Checks if TXT Buffer is accessible
 *                                    and can be written to.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 * Return: Status of the TXT Buffer.
 */
bool ctucan_hw_is_txt_buf_accessible(struct ctucan_hw_priv *priv, u8 buf);

/**
 * ctucan_hw_txt_buf_give_command - Give command to TXT Buffer
 *                                   of CTU CAN FD Core.
 *
 * @priv: Private info
 * @cmd: Command to issue for given Tx buffer.
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 */
static inline void ctucan_hw_txt_buf_give_command(struct ctucan_hw_priv *priv,
				union ctu_can_fd_tx_command_txtb_info cmd, u8 buf)
{
	union ctu_can_fd_tx_command_txtb_info reg;

	reg.u32 = 0;
	reg.s.txb1 = 1;

	reg.u32 <<= buf - CTU_CAN_FD_TXT_BUFFER_1;
	reg.u32 |= cmd.u32;

	priv->write_reg(priv, CTU_CAN_FD_TX_COMMAND, reg.u32);
}

/**
 * ctucan_hw_txt_set_empty - Give "set_empty" command to TXT Buffer.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 */
static inline void ctucan_hw_txt_set_empty(struct ctucan_hw_priv *priv, u8 buf)
{
	union ctu_can_fd_tx_command_txtb_info cmd;

	cmd.u32 = 0;
	cmd.s.txce = 1;

	ctucan_hw_txt_buf_give_command(priv, cmd, buf);
}

/**
 * ctucan_hw_txt_set_rdy - Give "set_ready" command to TXT Buffer.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 */
static inline void ctucan_hw_txt_set_rdy(struct ctucan_hw_priv *priv, u8 buf)
{
	union ctu_can_fd_tx_command_txtb_info cmd;

	cmd.u32 = 0;
	cmd.s.txcr = 1;

	ctucan_hw_txt_buf_give_command(priv, cmd, buf);
}

/**
 * ctucan_hw_txt_set_abort - Give "set_abort" command to TXT Buffer.
 *
 * @priv: Private info
 * @buf: TXT Buffer index (1 to CTU_CAN_FD_TXT_BUFFER_COUNT)
 */
static inline void ctucan_hw_txt_set_abort(struct ctucan_hw_priv *priv, u8 buf)
{
	union ctu_can_fd_tx_command_txtb_info cmd;

	cmd.u32 = 0;
	cmd.s.txca = 1;

	ctucan_hw_txt_buf_give_command(priv, cmd, buf);
}

/**
 * ctucan_hw_set_txt_priority - Set priority of TXT Buffers in CTU CAN FD Core.
 *
 * @priv: Private info
 * @prio: Pointer to array with CTU_CAN_FD_TXT_BUFFER_COUNT number
 *		of elements with TXT Buffer priorities.
 */
void ctucan_hw_set_txt_priority(struct ctucan_hw_priv *priv, const u8 *prio);

/**
 * ctucan_hw_insert_frame - Insert CAN FD frame to TXT Buffer
 *                           of CTU CAN FD Core.
 *
 * @priv: Private info
 * @data: Pointer to CAN Frame buffer.
 * @ts: Timestamp when the buffer should be sent.
 * @buf: Index of TXT Buffer where to insert the CAN Frame.
 * @isfdf: True if the frame is a FD frame.
 * Return: True if the frame was inserted successfully, False otherwise.
 */
bool ctucan_hw_insert_frame(struct ctucan_hw_priv *priv,
			    const struct canfd_frame *data, u64 ts,
			    u8 buf, bool isfdf);

/**
 * ctucan_hw_get_tran_delay - Read transceiver delay as measured
 *                             by CTU CAN FD Core.
 *
 * Note that transceiver delay can be measured only after at least
 * one CAN FD Frame with BRS bit was sent since the last re-start of the Core.
 *
 * @priv: Private info
 * Return: True if the frame was inserted successfully, False otherwise.
 */
static inline u16 ctucan_hw_get_tran_delay(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_trv_delay_ssp_cfg reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_TRV_DELAY);
	return reg.s.trv_delay_value;
}

/**
 * ctucan_hw_get_tx_frame_ctr - Read number of transmitted CAN/CAN FD Frames
 *                               by CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Number of received CAN/CAN FD frames.
 */
static inline u32 ctucan_hw_get_tx_frame_ctr(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_tx_fr_ctr reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_TX_FR_CTR);
	return reg.s.tx_fr_ctr_val;
}

/**
 * ctucan_hw_get_rx_frame_ctr - Read number of received CAN/CAN FD Frames
 *                               by CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Number of received CAN/CAN FD frames.
 */
static inline u32 ctucan_hw_get_rx_frame_ctr(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_rx_fr_ctr reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_FR_CTR);
	return reg.s.rx_fr_ctr_val;
}

/**
 * ctu_can_fd_read_debug_info - Returns debug information of CTU CAN FD Core.
 *
 * @priv: Private info
 * Return: Content of Debug register.
 */
static inline union ctu_can_fd_debug_register
	ctu_can_fd_read_debug_info(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_debug_register reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_DEBUG_REGISTER);
	return reg;
}

/**
 * ctucan_hw_read_timestamp - Read timestamp value which is used internally
 *                             by CTU CAN FD Core.
 *
 * Reads timestamp twice and checks consistency betwen upper and
 * lower timestamp word.
 *
 * @priv: Private info
 * Return: Value of timestamp in CTU CAN FD Core
 */
u64 ctucan_hw_read_timestamp(struct ctucan_hw_priv *priv);

/**
 * ctucan_hw_configure_ssp - Configure Secondary sample point usage and
 *			     position.
 *
 * @priv: Private info
 * @enable_ssp: Enable Secondary Sampling point. When false, regular sampling
 *	       point is used.
 * @use_trv_delay: Add Transmitter delay to secondary sampling point position.
 * @ssp_offset: Position of secondary sampling point.
 */
void ctucan_hw_configure_ssp(struct ctucan_hw_priv *priv, bool enable_ssp,
			     bool use_trv_delay, int ssp_offset);

extern const struct can_bittiming_const ctu_can_fd_bit_timing_max;
extern const struct can_bittiming_const ctu_can_fd_bit_timing_data_max;

#endif
