// SPDX-License-Identifier: GPL-2.0-or-later
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


#ifdef __KERNEL__
# include <linux/can/dev.h>
#else
/* The hardware registers mapping and low level layer should build
 * in userspace to allow development and verification of CTU CAN IP
 * core VHDL design when loaded into hardware. Debugging hardware
 * from kernel driver is really difficult, leads to system stucks
 * by error reporting etc. Testing of exactly the same code
 * in userspace together with headers generated automatically
 * generated from from IP-XACT/cactus helps to driver to hardware
 * and QEMU emulation model consistency keeping.
 */
# include "ctucanfd_linux_defs.h"
#endif

#include "ctucanfd_frame.h"
#include "ctucanfd_hw.h"


void ctucan_hw_write32(struct ctucan_hw_priv *priv,
		       enum ctu_can_fd_can_registers reg, u32 val)
{
	iowrite32(val, priv->mem_base + reg);
}

void ctucan_hw_write32_be(struct ctucan_hw_priv *priv,
			  enum ctu_can_fd_can_registers reg, u32 val)
{
	iowrite32be(val, priv->mem_base + reg);
}

u32 ctucan_hw_read32(struct ctucan_hw_priv *priv,
		     enum ctu_can_fd_can_registers reg)
{
	return ioread32(priv->mem_base + reg);
}

u32 ctucan_hw_read32_be(struct ctucan_hw_priv *priv,
			enum ctu_can_fd_can_registers reg)
{
	return ioread32be(priv->mem_base + reg);
}

static void ctucan_hw_write_txt_buf(struct ctucan_hw_priv *priv,
				    enum ctu_can_fd_can_registers buf_base,
				    u32 offset, u32 val)
{
	priv->write_reg(priv, buf_base + offset, val);
}

static union ctu_can_fd_identifier_w ctucan_hw_id_to_hwid(canid_t id)
{
	union ctu_can_fd_identifier_w hwid;

	hwid.u32 = 0;

	if (id & CAN_EFF_FLAG) {
		hwid.s.identifier_base = (id & CAN_EFF_MASK) >> 18;

		/* getting lowest 18 bits, replace with sth nicer... */
		hwid.s.identifier_ext = (id & 0x3FFFF);
	} else {
		hwid.s.identifier_base = id & CAN_SFF_MASK;
	}
	return hwid;
}

static u32 ctucan_hw_hwid_to_id(union ctu_can_fd_identifier_w hwid,
				 enum ctu_can_fd_frame_format_w_ide type)
{
	u32 id;

	if (type == EXTENDED) {
		id = CAN_EFF_FLAG;
		id |= hwid.s.identifier_base << 18;
		id |= hwid.s.identifier_ext;
	} else {
		id = hwid.s.identifier_base;
	}

	return id;
}

bool ctucan_hw_check_access(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_device_id_version reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_DEVICE_ID);

	if (reg.s.device_id != CTU_CAN_FD_ID)
		return false;

	return true;
}

u32 ctucan_hw_get_version(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_device_id_version reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_DEVICE_ID);
	return reg.s.ver_major * 10 + reg.s.ver_minor;
}

void ctucan_hw_enable(struct ctucan_hw_priv *priv, bool enable)
{
	union ctu_can_fd_mode_settings reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_MODE);
	reg.s.ena = enable ? CTU_CAN_ENABLED : CTU_CAN_DISABLED;
	priv->write_reg(priv, CTU_CAN_FD_MODE, reg.u32);
}

void ctucan_hw_reset(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_mode_settings mode;

	mode.u32 = 0;
	mode.s.rst = 1;
	/* it does not matter that we overwrite the rest of the reg
	 * - we're resetting
	 */
	priv->write_reg(priv, CTU_CAN_FD_MODE, mode.u32);
}

bool ctucan_hw_set_ret_limit(struct ctucan_hw_priv *priv, bool enable, u8 limit)
{
	union ctu_can_fd_mode_settings reg;

	if (limit > CTU_CAN_FD_RETR_MAX)
		return false;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_MODE);
	reg.s.rtrle = enable ? RTRLE_ENABLED : RTRLE_DISABLED;
	reg.s.rtrth = limit & 0xF;
	priv->write_reg(priv, CTU_CAN_FD_MODE, reg.u32);
	return true;
}

void ctucan_hw_set_mode_reg(struct ctucan_hw_priv *priv,
			    const struct can_ctrlmode *mode)
{
	u32 flags = mode->flags;
	union ctu_can_fd_mode_settings reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_MODE);

	if (mode->mask & CAN_CTRLMODE_LOOPBACK)
		reg.s.ilbp = flags & CAN_CTRLMODE_LOOPBACK ?
					INT_LOOP_ENABLED : INT_LOOP_DISABLED;

	if (mode->mask & CAN_CTRLMODE_LISTENONLY)
		reg.s.bmm = flags & CAN_CTRLMODE_LISTENONLY ?
					BMM_ENABLED : BMM_DISABLED;

	if (mode->mask & CAN_CTRLMODE_FD)
		reg.s.fde = flags & CAN_CTRLMODE_FD ?
				FDE_ENABLE : FDE_DISABLE;

	if (mode->mask & CAN_CTRLMODE_PRESUME_ACK)
		reg.s.stm = flags & CAN_CTRLMODE_PRESUME_ACK ?
				STM_ENABLED : STM_DISABLED;

	if (mode->mask & CAN_CTRLMODE_FD_NON_ISO)
		reg.s.nisofd = flags & CAN_CTRLMODE_FD_NON_ISO ?
				NON_ISO_FD : ISO_FD;

	priv->write_reg(priv, CTU_CAN_FD_MODE, reg.u32);
}

void ctucan_hw_rel_rx_buf(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_command reg;

	reg.u32 = 0;
	reg.s.rrb = 1;
	priv->write_reg(priv, CTU_CAN_FD_COMMAND, reg.u32);
}

void ctucan_hw_clr_overrun_flag(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_command reg;

	reg.u32 = 0;
	reg.s.cdo = 1;
	priv->write_reg(priv, CTU_CAN_FD_COMMAND, reg.u32);
}

static void ctucan_hw_int_conf(struct ctucan_hw_priv *priv,
			       enum ctu_can_fd_can_registers sreg,
			       enum ctu_can_fd_can_registers creg,
			       union ctu_can_fd_int_stat mask,
			       union ctu_can_fd_int_stat val)
{
	priv->write_reg(priv, sreg, mask.u32 & val.u32);
	priv->write_reg(priv, creg, mask.u32 & (~val.u32));
}

void ctucan_hw_int_ena(struct ctucan_hw_priv *priv,
		       union ctu_can_fd_int_stat mask,
		       union ctu_can_fd_int_stat val)
{
	ctucan_hw_int_conf(priv, CTU_CAN_FD_INT_ENA_SET,
			   CTU_CAN_FD_INT_ENA_CLR, mask, val);
}

void ctucan_hw_int_mask(struct ctucan_hw_priv *priv,
			union ctu_can_fd_int_stat mask,
			union ctu_can_fd_int_stat val)
{
	ctucan_hw_int_conf(priv, CTU_CAN_FD_INT_MASK_SET,
			   CTU_CAN_FD_INT_MASK_CLR, mask, val);
}

void ctucan_hw_set_mode(struct ctucan_hw_priv *priv,
			const struct can_ctrlmode *mode)
{
	ctucan_hw_set_mode_reg(priv, mode);

	/* One shot mode supported indirectly via Retransmitt limit */
	if (mode->mask & CAN_CTRLMODE_ONE_SHOT)
		ctucan_hw_set_ret_limit(priv, !!(mode->flags &
					 CAN_CTRLMODE_ONE_SHOT), 0);

	/* Bus error reporting -> Allow Error interrupt */
	if (mode->mask & CAN_CTRLMODE_BERR_REPORTING) {
		union ctu_can_fd_int_stat ena, mask;

		ena.u32 = 0;
		mask.u32 = 0;
		ena.s.bei = !!(mode->flags & CAN_CTRLMODE_ONE_SHOT);
		mask.s.bei = 1;
		ctucan_hw_int_ena(priv, ena, mask);
	}
}

void ctucan_hw_set_nom_bittiming(struct ctucan_hw_priv *priv,
				 struct can_bittiming *nbt)
{
	union ctu_can_fd_btr btr;

	/* The timing calculation functions have only constraints on tseg1,
	 * which is prop_seg + phase1_seg combined. tseg1 is then split in half
	 * and stored into prog_seg and phase_seg1. In CTU CAN FD, PROP is
	 * 7 bits wide but PH1 only 6, so we must re-distribute the values here.
	 */
	u32 prop_seg = nbt->prop_seg;
	u32 phase_seg1 = nbt->phase_seg1;

	if (phase_seg1 > 63) {
		prop_seg += phase_seg1 - 63;
		phase_seg1 = 63;
		nbt->prop_seg = prop_seg;
		nbt->phase_seg1 = phase_seg1;
	}

	btr.u32 = 0;
	btr.s.prop = prop_seg;
	btr.s.ph1 = phase_seg1;
	btr.s.ph2 = nbt->phase_seg2;
	btr.s.brp = nbt->brp;
	btr.s.sjw = nbt->sjw;

	priv->write_reg(priv, CTU_CAN_FD_BTR, btr.u32);
}

void ctucan_hw_set_data_bittiming(struct ctucan_hw_priv *priv,
				  struct can_bittiming *dbt)
{
	union ctu_can_fd_btr_fd btr_fd;

	/* The timing calculation functions have only constraints on tseg1,
	 * which is prop_seg + phase1_seg combined. tseg1 is then split in half
	 * and stored into prog_seg and phase_seg1. In CTU CAN FD, PROP_FD is
	 * 6 bits wide but PH1_FD only 5, so we must re-distribute the values
	 * here.
	 */
	u32 prop_seg = dbt->prop_seg;
	u32 phase_seg1 = dbt->phase_seg1;

	if (phase_seg1 > 31) {
		prop_seg += phase_seg1 - 31;
		phase_seg1 = 31;
		dbt->prop_seg = prop_seg;
		dbt->phase_seg1 = phase_seg1;
	}

	btr_fd.u32 = 0;
	btr_fd.s.prop_fd = prop_seg;
	btr_fd.s.ph1_fd = phase_seg1;
	btr_fd.s.ph2_fd = dbt->phase_seg2;
	btr_fd.s.brp_fd = dbt->brp;
	btr_fd.s.sjw_fd = dbt->sjw;

	priv->write_reg(priv, CTU_CAN_FD_BTR_FD, btr_fd.u32);
}

void ctucan_hw_set_err_limits(struct ctucan_hw_priv *priv, u8 ewl, u8 erp)
{
	union ctu_can_fd_ewl_erp_fault_state reg;

	reg.u32 = 0;
	reg.s.ew_limit = ewl;
	reg.s.erp_limit = erp;
	// era, bof, erp are read-only

	priv->write_reg(priv, CTU_CAN_FD_EWL, reg.u32);
}

void ctucan_hw_read_err_ctrs(struct ctucan_hw_priv *priv,
			     struct can_berr_counter *ctr)
{
	union ctu_can_fd_rec_tec reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_REC);
	ctr->txerr = reg.s.tec_val;
	ctr->rxerr = reg.s.rec_val;
}

enum can_state ctucan_hw_read_error_state(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_ewl_erp_fault_state reg;
	union ctu_can_fd_rec_tec err;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_EWL);
	err.u32 = priv->read_reg(priv, CTU_CAN_FD_REC);

	if (reg.s.era) {
		if (reg.s.ew_limit > err.s.rec_val &&
		    reg.s.ew_limit > err.s.tec_val)
			return CAN_STATE_ERROR_ACTIVE;
		else
			return CAN_STATE_ERROR_WARNING;
	} else if (reg.s.erp) {
		return CAN_STATE_ERROR_PASSIVE;
	} else if (reg.s.bof) {
		return CAN_STATE_BUS_OFF;
	}
	WARN(true, "Invalid error state");
	return CAN_STATE_ERROR_PASSIVE;
}

void ctucan_hw_set_err_ctrs(struct ctucan_hw_priv *priv,
			    const struct can_berr_counter *ctr)
{
	union ctu_can_fd_ctr_pres reg;

	reg.u32 = 0;

	reg.s.ctpv = ctr->txerr;
	reg.s.ptx = 1;
	priv->write_reg(priv, CTU_CAN_FD_CTR_PRES, reg.u32);

	reg.s.ctpv = ctr->rxerr;
	reg.s.ptx = 0;
	reg.s.prx = 1;
	priv->write_reg(priv, CTU_CAN_FD_CTR_PRES, reg.u32);
}

bool ctucan_hw_get_mask_filter_support(struct ctucan_hw_priv *priv, u8 fnum)
{
	union ctu_can_fd_filter_control_filter_status reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_FILTER_CONTROL);

	switch (fnum) {
	case CTU_CAN_FD_FILTER_A:
		if (reg.s.sfa)
			return true;
		break;
	case CTU_CAN_FD_FILTER_B:
		if (reg.s.sfb)
			return true;
		break;
	case CTU_CAN_FD_FILTER_C:
		if (reg.s.sfc)
			return true;
		break;
	}

	return false;
}

bool ctucan_hw_get_range_filter_support(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_filter_control_filter_status reg;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_FILTER_CONTROL);

	return !!reg.s.sfr;
}

bool ctucan_hw_set_mask_filter(struct ctucan_hw_priv *priv, u8 fnum,
			       bool enable, const struct can_filter *filter)
{
	union ctu_can_fd_filter_control_filter_status creg;
	enum ctu_can_fd_can_registers maddr, vaddr;
	union ctu_can_fd_identifier_w hwid_mask;
	union ctu_can_fd_identifier_w hwid_val;
	u8 val = 0;

	if (!ctucan_hw_get_mask_filter_support(priv, fnum))
		return false;

	if (enable)
		val = 1;

	creg.u32 = priv->read_reg(priv, CTU_CAN_FD_FILTER_CONTROL);

	switch (fnum) {
	case CTU_CAN_FD_FILTER_A:
		maddr = CTU_CAN_FD_FILTER_A_MASK;
		vaddr = CTU_CAN_FD_FILTER_A_VAL;
		creg.s.fanb = val;
		creg.s.fane = val;
		creg.s.fafb = val;
		creg.s.fafe = val;
	break;
	case CTU_CAN_FD_FILTER_B:
		maddr = CTU_CAN_FD_FILTER_B_MASK;
		vaddr = CTU_CAN_FD_FILTER_B_VAL;
		creg.s.fbnb = val;
		creg.s.fbne = val;
		creg.s.fbfb = val;
		creg.s.fbfe = val;
	break;
	case CTU_CAN_FD_FILTER_C:
		maddr = CTU_CAN_FD_FILTER_C_MASK;
		vaddr = CTU_CAN_FD_FILTER_C_VAL;
		creg.s.fcnb = val;
		creg.s.fcne = val;
		creg.s.fcfb = val;
		creg.s.fcfe = val;
	break;
	default:
		return false;
	}

	hwid_mask = ctucan_hw_id_to_hwid(filter->can_id);
	hwid_val = ctucan_hw_id_to_hwid(filter->can_mask);
	priv->write_reg(priv, CTU_CAN_FD_FILTER_CONTROL, creg.u32);
	priv->write_reg(priv, maddr, hwid_mask.u32);
	priv->write_reg(priv, vaddr, hwid_val.u32);
	return true;
}

void ctucan_hw_set_range_filter(struct ctucan_hw_priv *priv, canid_t low_th,
				canid_t high_th, bool enable)
{
	union ctu_can_fd_identifier_w hwid_low;
	union ctu_can_fd_identifier_w hwid_high;
	union ctu_can_fd_filter_control_filter_status creg;

	hwid_low = ctucan_hw_id_to_hwid(low_th);
	hwid_high = ctucan_hw_id_to_hwid(high_th);

	creg.u32 = priv->read_reg(priv, CTU_CAN_FD_FILTER_CONTROL);

	creg.s.frnb = enable;
	creg.s.frne = enable;
	creg.s.frfb = enable;
	creg.s.frfe = enable;

	priv->write_reg(priv, CTU_CAN_FD_FILTER_CONTROL, creg.u32);
	priv->write_reg(priv, CTU_CAN_FD_FILTER_RAN_LOW, hwid_low.u32);
	priv->write_reg(priv, CTU_CAN_FD_FILTER_RAN_HIGH, hwid_high.u32);
}

void ctucan_hw_set_rx_tsop(struct ctucan_hw_priv *priv,
			   enum ctu_can_fd_rx_settings_rtsop val)
{
	union ctu_can_fd_rx_status_rx_settings reg;

	reg.u32 = 0;
	reg.s.rtsop = val;
	priv->write_reg(priv, CTU_CAN_FD_RX_STATUS, reg.u32);
}

void ctucan_hw_read_rx_frame(struct ctucan_hw_priv *priv,
			     struct canfd_frame *cf, u64 *ts)
{
	union ctu_can_fd_frame_format_w ffw;

	ffw.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_DATA);
	ctucan_hw_read_rx_frame_ffw(priv, cf, ts, ffw);
}

void ctucan_hw_read_rx_frame_ffw(struct ctucan_hw_priv *priv,
				 struct canfd_frame *cf, u64 *ts,
				 union ctu_can_fd_frame_format_w ffw)
{
	union ctu_can_fd_identifier_w idw;
	unsigned int i;
	unsigned int wc;
	unsigned int len;
	enum ctu_can_fd_frame_format_w_ide ide;

	idw.u32 = priv->read_reg(priv, CTU_CAN_FD_RX_DATA);

	ide = (enum ctu_can_fd_frame_format_w_ide)ffw.s.ide;
	cf->can_id = ctucan_hw_hwid_to_id(idw, ide);

	/* BRS, ESI, RTR Flags */
	cf->flags = 0;
	if (ffw.s.fdf == FD_CAN) {
		if (ffw.s.brs == BR_SHIFT)
			cf->flags |= CANFD_BRS;
		if (ffw.s.esi_rsv == ESI_ERR_PASIVE)
			cf->flags |= CANFD_ESI;
	} else if (ffw.s.rtr == RTR_FRAME) {
		cf->can_id |= CAN_RTR_FLAG;
	}

	wc = ffw.s.rwcnt - 3;

	/* DLC */
	if (ffw.s.dlc <= 8) {
		len = ffw.s.dlc;
	} else {
		if (ffw.s.fdf == FD_CAN)
			len = wc << 2;
		else
			len = 8;
	}
	cf->len = len;
	if (unlikely(len > wc * 4))
		len = wc * 4;

	/* Timestamp */
	*ts = (u64)(priv->read_reg(priv, CTU_CAN_FD_RX_DATA));
	*ts |= ((u64)priv->read_reg(priv, CTU_CAN_FD_RX_DATA) << 32);

	/* Data */
	for (i = 0; i < len; i += 4) {
		u32 data = priv->read_reg(priv, CTU_CAN_FD_RX_DATA);
		*(__le32 *)(cf->data + i) = cpu_to_le32(data);
	}
	while (unlikely(i < wc * 4)) {
		priv->read_reg(priv, CTU_CAN_FD_RX_DATA);
		i += 4;
	}
}

enum ctu_can_fd_tx_status_tx1s ctucan_hw_get_tx_status(struct ctucan_hw_priv
							*priv, u8 buf)
{
	union ctu_can_fd_tx_status reg;
	u32 status;

	reg.u32 = priv->read_reg(priv, CTU_CAN_FD_TX_STATUS);

	switch (buf) {
	case CTU_CAN_FD_TXT_BUFFER_1:
		status = reg.s.tx1s;
		break;
	case CTU_CAN_FD_TXT_BUFFER_2:
		status = reg.s.tx2s;
		break;
	case CTU_CAN_FD_TXT_BUFFER_3:
		status = reg.s.tx3s;
		break;
	case CTU_CAN_FD_TXT_BUFFER_4:
		status = reg.s.tx4s;
		break;
	default:
		status = ~0;
	}
	return (enum ctu_can_fd_tx_status_tx1s)status;
}

bool ctucan_hw_is_txt_buf_accessible(struct ctucan_hw_priv *priv, u8 buf)
{
	enum ctu_can_fd_tx_status_tx1s buf_status;

	buf_status = ctucan_hw_get_tx_status(priv, buf);
	if (buf_status == TXT_RDY || buf_status == TXT_TRAN || buf_status == TXT_ABTP)
		return false;

	return true;
}

void ctucan_hw_set_txt_priority(struct ctucan_hw_priv *priv, const u8 *prio)
{
	union ctu_can_fd_tx_priority reg;

	reg.u32 = 0;
	reg.s.txt1p = prio[0];
	reg.s.txt2p = prio[1];
	reg.s.txt3p = prio[2];
	reg.s.txt4p = prio[3];

	priv->write_reg(priv, CTU_CAN_FD_TX_PRIORITY, reg.u32);
}

static const enum ctu_can_fd_can_registers
	tx_buf_bases[CTU_CAN_FD_TXT_BUFFER_COUNT] = {
		CTU_CAN_FD_TXTB1_DATA_1, CTU_CAN_FD_TXTB2_DATA_1,
		CTU_CAN_FD_TXTB3_DATA_1, CTU_CAN_FD_TXTB4_DATA_1
};

bool ctucan_hw_insert_frame(struct ctucan_hw_priv *priv,
			    const struct canfd_frame *cf, u64 ts, u8 buf,
			    bool isfdf)
{
	enum ctu_can_fd_can_registers buf_base;
	union ctu_can_fd_frame_format_w ffw;
	union ctu_can_fd_identifier_w idw;
	unsigned int i;

	ffw.u32 = 0;
	idw.u32 = 0;

	if (buf >= CTU_CAN_FD_TXT_BUFFER_COUNT)
		return false;
	buf_base = tx_buf_bases[buf];

	if (!ctucan_hw_is_txt_buf_accessible(priv, buf))
		return false;

	if (cf->can_id & CAN_RTR_FLAG)
		ffw.s.rtr = RTR_FRAME;

	if (cf->can_id & CAN_EFF_FLAG)
		ffw.s.ide = EXTENDED;
	else
		ffw.s.ide = BASE;

	idw = ctucan_hw_id_to_hwid(cf->can_id);

	if (cf->len > CANFD_MAX_DLEN)
		return false;

	ffw.s.dlc = can_len2dlc(cf->len);

	if (isfdf) {
		ffw.s.fdf = FD_CAN;
		if (cf->flags & CANFD_BRS)
			ffw.s.brs = BR_SHIFT;
	}

	ctucan_hw_write_txt_buf(priv, buf_base,
				CTU_CAN_FD_FRAME_FORMAT_W, ffw.u32);

	ctucan_hw_write_txt_buf(priv, buf_base,
				CTU_CAN_FD_IDENTIFIER_W, idw.u32);

	ctucan_hw_write_txt_buf(priv, buf_base,
				CTU_CAN_FD_TIMESTAMP_L_W, (u32)(ts));

	ctucan_hw_write_txt_buf(priv, buf_base,
				CTU_CAN_FD_TIMESTAMP_U_W, (u32)(ts >> 32));

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		for (i = 0; i < cf->len; i += 4) {
			u32 data = le32_to_cpu(*(__le32 *)(cf->data + i));

			ctucan_hw_write_txt_buf(priv, buf_base,
						CTU_CAN_FD_DATA_1_4_W + i, data);
		}
	}

	return true;
}

u64 ctucan_hw_read_timestamp(struct ctucan_hw_priv *priv)
{
	union ctu_can_fd_timestamp_low ts_low;
	union ctu_can_fd_timestamp_high ts_high;
	union ctu_can_fd_timestamp_high ts_high_2;

	ts_high.u32 = priv->read_reg(priv, CTU_CAN_FD_TIMESTAMP_HIGH);
	ts_low.u32 = priv->read_reg(priv, CTU_CAN_FD_TIMESTAMP_LOW);
	ts_high_2.u32 = priv->read_reg(priv, CTU_CAN_FD_TIMESTAMP_HIGH);

	if (ts_high.u32 != ts_high_2.u32)
		ts_low.u32 = priv->read_reg(priv, CTU_CAN_FD_TIMESTAMP_LOW);

	return (((u64)ts_high_2.u32) << 32) | ((u64)ts_low.u32);
}

void ctucan_hw_configure_ssp(struct ctucan_hw_priv *priv, bool enable_ssp,
			     bool use_trv_delay, int ssp_offset)
{
	union ctu_can_fd_trv_delay_ssp_cfg ssp_cfg;

	ssp_cfg.u32 = 0;
	if (enable_ssp) {
		if (use_trv_delay)
			ssp_cfg.s.ssp_src = SSP_SRC_MEAS_N_OFFSET;
		else
			ssp_cfg.s.ssp_src = SSP_SRC_OFFSET;
	} else {
		ssp_cfg.s.ssp_src = SSP_SRC_NO_SSP;
	}

	ssp_cfg.s.ssp_offset = (uint32_t)ssp_offset;
	priv->write_reg(priv, CTU_CAN_FD_TRV_DELAY, ssp_cfg.u32);
}
