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

#ifndef __KERNEL__
# include "ctu_can_fd_linux_defs.h"
#endif

#include "ctu_can_fd_frame.h"
//#include "ctu_can_fd_regs.h"
#include "ctu_can_fd_hw.h"


static inline union ctu_can_fd_identifier_w ctu_can_fd_id_to_hwid(canid_t id)
{
	union ctu_can_fd_identifier_w hwid;
	hwid.u32 = 0;

	if (id & CAN_EFF_FLAG){
		hwid.s.identifier_base = (id & CAN_EFF_MASK) >> 18;

		// getting lowest 18 bits, replace with sth nicer...
		hwid.s.identifier_ext = (id & 0x3FFFF);
	}else
		hwid.s.identifier_base = id & CAN_SFF_MASK;
	return hwid;
}

// TODO: rename or do not depend on previous value of id
static inline void ctu_can_fd_hwid_to_id(union ctu_can_fd_identifier_w hwid,
					 canid_t *id,
					 enum ctu_can_fd_frame_form_w_id_type type)
{	
	// Preserve flags which we dont set
	*id &= ~(CAN_EFF_FLAG | CAN_EFF_MASK);
	
	if (type == EXTENDED){
		*id |= CAN_EFF_FLAG;
		*id |= hwid.s.identifier_base << 18;
		*id |= hwid.s.identifier_ext;
	}else
		*id = hwid.s.identifier_base;
}


// TODO: use can_len2dlc
static bool ctu_can_fd_len_to_dlc(u8 len, u8 *dlc)
{
	*dlc = can_len2dlc(len);
	return true;
	/*
	if (unlikely(len > 64)) {
		*dlc = 0;
		return false;
	} else {
		*dlc = can_len2dlc(len);
		if (unlikely(*dlc == can_len2dlc(len-1))) {
			*dlc = 0;
			return false;
		}
	}*/

	/*
	if (len <= 8){
		*dlc = len;
		goto exit_ok;
	}

	switch (len){
	case 12 : *dlc = 0x9;
	break;
	case 16 : *dlc = 0xA;
	break;
	case 20 : *dlc = 0xB;
	break;
	case 24 : *dlc = 0xC;
	break;
	case 32 : *dlc = 0xD;
	break;
	case 48 : *dlc = 0xE;
	break;
	case 64 : *dlc = 0xF;
	break;
	default : *dlc = 0x0;
	}
	
	if (*dlc == 0)
		return false;
exit_ok:
	*/
	return true;
}


bool ctu_can_fd_check_access(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_device_id_version reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_DEVICE_ID);
	
	if (reg.s.device_id != CTU_CAN_FD_ID)
		return false;
		
	return true;
}

u32 ctu_can_fd_get_version(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_device_id_version reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_DEVICE_ID);
	return reg.s.ver_major * 10 + reg.s.ver_minor;
}

void ctu_can_fd_enable(struct ctucanfd_priv *priv, bool enable)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_MODE);
	reg.s.ena = enable ? ENABLED : DISABLED;
	ctu_can_fd_write32(priv, CTU_CAN_FD_MODE, reg.u32);
}

bool ctu_can_fd_set_ret_limit(struct ctucanfd_priv *priv, bool enable, u8 limit)
{
	union ctu_can_fd_mode_command_status_settings reg;
	
	if (limit > CTU_CAN_FD_RETR_MAX)
		return false;
 	
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_MODE);
	reg.s.rtrle = enable ? RTRLE_ENABLED : RTRLE_DISABLED;
	reg.s.rtr_th = limit & 0xF;
	ctu_can_fd_write32(priv, CTU_CAN_FD_MODE, reg.u32);
	return true;
}

void ctu_can_fd_set_mode_reg(struct ctucanfd_priv *priv, const struct can_ctrlmode *mode)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_MODE);
	
	if (mode->mask & CAN_CTRLMODE_LOOPBACK)
		reg.s.int_loop = mode->flags & CAN_CTRLMODE_LOOPBACK ?
					INT_LOOP_ENABLED : INT_LOOP_DISABLED;
	
	if (mode->mask & CAN_CTRLMODE_LISTENONLY)
		reg.s.lom = mode->flags & CAN_CTRLMODE_LISTENONLY ?
					LOM_ENABLED : LOM_DISABLED;
	
	if (mode->mask & CAN_CTRLMODE_3_SAMPLES)
		reg.s.tsm = mode->flags & CAN_CTRLMODE_3_SAMPLES ?
				TSM_ENABLE : TSM_DISABLE;
	
	if (mode->mask & CAN_CTRLMODE_FD)
		reg.s.fde = mode->flags & CAN_CTRLMODE_FD ?
				FDE_ENABLE : FDE_DISABLE;
	
	if (mode->mask & CAN_CTRLMODE_PRESUME_ACK)
		reg.s.stm = mode->flags & CAN_CTRLMODE_PRESUME_ACK ?
				STM_ENABLED : STM_DISABLED;
	
	if (mode->mask & CAN_CTRLMODE_FD_NON_ISO)
		reg.s.fd_type = mode->flags & CAN_CTRLMODE_FD_NON_ISO ?
				NON_ISO_FD : ISO_FD;

	ctu_can_fd_write32(priv, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_rel_rx_buf(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_MODE);
	reg.s.rrb = 1;
	ctu_can_fd_write32(priv, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_clr_overrun_flag(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_MODE);
	reg.s.cdo = 1;
	ctu_can_fd_write32(priv, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_abort_tx(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_MODE);
	reg.s.at = 1;
	ctu_can_fd_write32(priv, CTU_CAN_FD_MODE, reg.u32);
}

// TODO: rather than set(value, mask) interface, provide native set(val), clr(val)
//       interface to potentially avoid unnecessary write
static void ctu_can_fd_int_conf(struct ctucanfd_priv *priv, enum ctu_can_fd_regs sreg,
				enum ctu_can_fd_regs creg, 
				union ctu_can_fd_int_stat mask,
				union ctu_can_fd_int_stat val)
{
	//union ctu_can_fd_int_stat reg;
	//reg.u32 = ctu_can_fd_read32(priv, sreg);
	
	ctu_can_fd_write32(priv, sreg, mask.u32 & val.u32);
	ctu_can_fd_write32(priv, creg, mask.u32 & (~val.u32));
}

void ctu_can_fd_int_ena(struct ctucanfd_priv *priv, union ctu_can_fd_int_stat mask,
			union ctu_can_fd_int_stat val)
{
	ctu_can_fd_int_conf(priv, CTU_CAN_FD_INT_ENA_SET, CTU_CAN_FD_INT_ENA_CLR,
				mask, val);
}

void ctu_can_fd_int_mask(struct ctucanfd_priv *priv, union ctu_can_fd_int_stat mask,
			 union ctu_can_fd_int_stat val)
{
	ctu_can_fd_int_conf(priv, CTU_CAN_FD_INT_MASK_SET, CTU_CAN_FD_INT_MASK_CLR,
				mask, val);
}

void ctu_can_fd_set_mode(struct ctucanfd_priv *priv, const struct can_ctrlmode *mode)
{
	ctu_can_fd_set_mode_reg(priv, mode);
	
	// One shot mode supported indirectly via Retransmitt limit
	if (mode->mask & CAN_CTRLMODE_ONE_SHOT)
		ctu_can_fd_set_ret_limit(priv, true, 0);

	// Bus error reporting -> Allow Error interrupt
	union ctu_can_fd_int_stat reg;
	reg.u32 = 0;
	reg.s.bei = 1;
	ctu_can_fd_int_ena(priv, reg, reg);
}


const struct can_bittiming_const ctu_can_fd_bit_timing_max = {
	.name = "ctu_can_fd",
	.tseg1_min = 2,
	.tseg1_max = 190,
	.tseg2_min = 1,
	.tseg2_max = 63,
	.sjw_max = 31,
	.brp_min = 1,
	.brp_max = 255,
	.brp_inc = 1,
};

const struct can_bittiming_const ctu_can_fd_bit_timing_data_max = {
	.name = "ctu_can_fd",
	.tseg1_min = 2,
	.tseg1_max = 94,
	.tseg2_min = 1,
	.tseg2_max = 31,
	.sjw_max = 31,
	.brp_min = 1,
	.brp_max = 255,
	.brp_inc = 1,
};

void ctu_can_fd_set_nom_bittiming(struct ctucanfd_priv *priv,
				  const struct can_bittiming *nbt)
{
	union ctu_can_fd_btr btr;
	btr.u32 = 0;
	btr.s.prop = nbt->prop_seg;
	btr.s.ph1 = nbt->phase_seg1;
	btr.s.ph2 = nbt->phase_seg2;
	btr.s.brp = nbt->brp;
	btr.s.sjw = nbt->sjw;

	ctu_can_fd_write32(priv, CTU_CAN_FD_BTR, btr.u32);
}

void ctu_can_fd_set_data_bittiming(struct ctucanfd_priv *priv,
				   const struct can_bittiming *dbt)
{
	union ctu_can_fd_btr_fd btr_fd;
	btr_fd.u32 = 0;
	btr_fd.s.prop_fd = dbt->prop_seg;
	btr_fd.s.ph1_fd = dbt->phase_seg1;
	btr_fd.s.ph2_fd = dbt->phase_seg2;
	btr_fd.s.brp_fd = dbt->brp;
	btr_fd.s.sjw_fd = dbt->sjw;
	
	ctu_can_fd_write32(priv, CTU_CAN_FD_BTR_FD, btr_fd.u32);
}

void ctu_can_fd_set_err_limits(struct ctucanfd_priv *priv, u8 ewl, u8 erp)
{
	union ctu_can_fd_ewl_erp_fault_state reg;
	reg.u32 = 0;
	reg.s.ewl_limit = ewl;
	reg.s.erp_limit = erp;
	// era, bof, erp are read-only
	
	ctu_can_fd_write32(priv, CTU_CAN_FD_EWL, reg.u32);
}

bool ctu_can_fd_read_err_ctrs(struct ctucanfd_priv *priv, struct can_berr_counter *ctr)
{
	union ctu_can_fd_rxc_txc reg;
	
	if (ctr == NULL)
		return false;
	
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_RXC);
	ctr->txerr = reg.s.rxc_val;
	ctr->rxerr = reg.s.txc_val;
	return true;
}

enum can_state ctu_can_fd_read_error_state(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_ewl_erp_fault_state reg;
	union ctu_can_fd_rxc_txc err;

	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_EWL);
	err.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_RXC);

	if (reg.s.era){
		if (reg.s.ewl_limit > err.s.rxc_val && reg.s.ewl_limit > err.s.txc_val)
			return CAN_STATE_ERROR_ACTIVE;
		else
			return CAN_STATE_ERROR_WARNING;
	}else if (reg.s.erp)
		return CAN_STATE_ERROR_PASSIVE;
	else if (reg.s.bof)
		return CAN_STATE_BUS_OFF;
	WARN(true, "Invalid error state");
	return CAN_STATE_ERROR_PASSIVE;
}

void ctu_can_fd_set_err_ctrs(struct ctucanfd_priv *priv, const struct can_berr_counter *ctr)
{
	union ctu_can_fd_ctr_pres reg;
	reg.u32 = 0;

	reg.s.ctpv = ctr->txerr;
	reg.s.ptx = 1;
	ctu_can_fd_write32(priv, CTU_CAN_FD_CTR_PRES, reg.u32);

	reg.s.ctpv = ctr->rxerr;
	reg.s.ptx = 0;
	reg.s.prx = 1;
	ctu_can_fd_write32(priv, CTU_CAN_FD_CTR_PRES, reg.u32);
}


bool ctu_can_fd_get_mask_filter_support(struct ctucanfd_priv *priv, u8 fnum)
{
	union ctu_can_fd_filter_control_filter_status reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_FILTER_CONTROL);

	switch (fnum){
	case CTU_CAN_FD_FILTER_A :
		if (reg.s.sfa) return true;
	break;
	case CTU_CAN_FD_FILTER_B :
		if (reg.s.sfb) return true;
	break;
	case CTU_CAN_FD_FILTER_C :
		if (reg.s.sfc) return true;
	break;
	}

	return false;
}

bool ctu_can_fd_get_range_filter_support(struct ctucanfd_priv *priv)
{
	union ctu_can_fd_filter_control_filter_status reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_FILTER_CONTROL);

	if (reg.s.sfr)
		return true;

	return false;
}

bool ctu_can_fd_set_mask_filter(struct ctucanfd_priv *priv, u8 fnum, bool enable,
				const struct can_filter *filter)
{
	union ctu_can_fd_filter_control_filter_status creg;
	enum ctu_can_fd_regs maddr,vaddr;
	union ctu_can_fd_identifier_w hwid_mask;	
	union ctu_can_fd_identifier_w hwid_val;	
	uint8_t val = 0;

	if (!ctu_can_fd_get_mask_filter_support(priv, fnum))
		return false;

	if (enable)
		val = 1;

	creg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_FILTER_CONTROL);
	//maddr = 0;
	//vaddr = 0;

	switch (fnum){
	case CTU_CAN_FD_FILTER_A :
		maddr = CTU_CAN_FD_FILTER_A_MASK;
		vaddr = CTU_CAN_FD_FILTER_A_VAL;	
		creg.s.fanb = val;
		creg.s.fane = val;
		creg.s.fafb = val;
		creg.s.fafe = val;	
	break;
	case CTU_CAN_FD_FILTER_B :
		maddr = CTU_CAN_FD_FILTER_B_MASK;
		vaddr = CTU_CAN_FD_FILTER_B_VAL;
		creg.s.fbnb = val;
		creg.s.fbne = val;
		creg.s.fbfb = val;
		creg.s.fbfe = val;
	break;
	case CTU_CAN_FD_FILTER_C :
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

	hwid_mask = ctu_can_fd_id_to_hwid(filter->can_id);
	hwid_val = ctu_can_fd_id_to_hwid(filter->can_mask);
	ctu_can_fd_write32(priv, CTU_CAN_FD_FILTER_CONTROL, creg.u32);
	ctu_can_fd_write32(priv, maddr, hwid_mask.u32);
	ctu_can_fd_write32(priv, vaddr, hwid_val.u32);
	return true;
}

void ctu_can_fd_set_range_filter(struct ctucanfd_priv *priv, canid_t low_th,
				 canid_t high_th, bool enable)
{
	union ctu_can_fd_identifier_w hwid_low;	
	union ctu_can_fd_identifier_w hwid_high;
	union ctu_can_fd_filter_control_filter_status creg;
	
	hwid_low = ctu_can_fd_id_to_hwid(low_th);
	hwid_high = ctu_can_fd_id_to_hwid(high_th);

	creg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_FILTER_CONTROL);
	
	creg.s.frnb = enable;
	creg.s.frne = enable;
	creg.s.frfb = enable;
	creg.s.frfe = enable;

	ctu_can_fd_write32(priv, CTU_CAN_FD_FILTER_CONTROL, creg.u32);
	ctu_can_fd_write32(priv, CTU_CAN_FD_FILTER_RAN_LOW, hwid_low.u32);
	ctu_can_fd_write32(priv, CTU_CAN_FD_FILTER_RAN_HIGH, hwid_high.u32);
}

void ctu_can_fd_set_rx_tsop(struct ctucanfd_priv *priv, enum ctu_can_fd_rx_settings_rtsop val)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = 0;
	reg.s.rtsop = val;
	ctu_can_fd_write32(priv, CTU_CAN_FD_RX_STATUS, reg.u32);
}

void ctu_can_fd_read_rx_frame(struct ctucanfd_priv *priv, struct canfd_frame *data, u64 *ts)
{
	struct canfd_frame *cf = (struct canfd_frame *)data; // TODO: may break alignment rules
	union ctu_can_fd_frame_form_w ffw;
	union ctu_can_fd_identifier_w idw;
	unsigned i;
	
	ffw.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_RX_DATA);
	idw.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_RX_DATA);
	cf->can_id = 0;
	cf->flags = 0;
	
	// BRS, ESI, RTR Flags
	if (ffw.s.fr_type == FD_CAN){
		if (ffw.s.brs == BR_SHIFT)
			cf->flags |= CANFD_BRS;
		if (ffw.s.esi_resvd == ESI_ERR_PASIVE)
			cf->flags |= CANFD_ESI;
	}else if (ffw.s.rtr == RTR_FRAME)
		cf->can_id |= CAN_RTR_FLAG;
	
	// DLC
	if (ffw.s.dlc <= 8){
		cf->len = ffw.s.dlc;
	}else{
		if (ffw.s.fr_type == FD_CAN)
			cf->len = (ffw.s.rwcnt - 3) << 2;
		else
			cf->len = 8;
	}

	ctu_can_fd_hwid_to_id(idw, &(cf->can_id), (enum ctu_can_fd_frame_form_w_id_type) ffw.s.id_type);
	
	// Timestamp
	*ts = (u64)(ctu_can_fd_read32(priv, CTU_CAN_FD_RX_DATA));
	*ts |= ((u64)ctu_can_fd_read32(priv, CTU_CAN_FD_RX_DATA) << 32);
	
	// Data
	for (i = 0; i < cf->len; i += 4)
		*(u32 *)(cf->data + i) = ctu_can_fd_read32(priv, CTU_CAN_FD_RX_DATA);
}

enum ctu_can_fd_tx_status_tx1s ctu_can_fd_get_tx_status(struct ctucanfd_priv *priv, u8 buf)
{
	union ctu_can_fd_tx_status reg;
	reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_TX_STATUS);
	
	uint32_t status;
	switch (buf) {
	case CTU_CAN_FD_TXT_BUFFER_1 : status = reg.s.tx1s;
	break;
	case CTU_CAN_FD_TXT_BUFFER_2 : status = reg.s.tx2s;
	break;
	case CTU_CAN_FD_TXT_BUFFER_3 : status = reg.s.tx3s;
	break;
	case CTU_CAN_FD_TXT_BUFFER_4 : status = reg.s.tx4s;
	break;
	default :
		status = ~0;
	}
	return (enum ctu_can_fd_tx_status_tx1s) status;
}

bool ctu_can_fd_is_txt_buf_accessible(struct ctucanfd_priv *priv, u8 buf)
{
	enum ctu_can_fd_tx_status_tx1s buf_status;

	buf_status = ctu_can_fd_get_tx_status(priv, buf);
	if (buf_status == TXT_RDY || buf_status == TXT_TRAN 
		|| buf_status == TXT_ABTP)
 		return false;

	return true;
}

bool ctu_can_fd_txt_buf_give_command(struct ctucanfd_priv *priv, u8 cmd, u8 buf)
{
	union ctu_can_fd_tx_command reg;
	reg.u32 = 0;
	
	switch (buf){
	case CTU_CAN_FD_TXT_BUFFER_1: reg.s.txi1 = 1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_2: reg.s.txi2 = 1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_3: reg.s.txi3 = 1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_4: reg.s.txi4 = 1;
	break;
	default:
		return false;
	}
	
	// TODO: use named constants for the command
	// TODO: is it wise to have it bitwise?
	if (cmd & 0x1) {
		reg.s.txce = 1;
	} else if (cmd & 0x2) {
		reg.s.txcr = 1;
	} else if (cmd & 0x4) {
		reg.s.txca = 1;
	} else {
		return false;
	}
	
	ctu_can_fd_write32(priv, CTU_CAN_FD_TX_COMMAND, reg.u32);
	return true;
}

void ctu_can_fd_set_txt_priority(struct ctucanfd_priv *priv, const u8 *prio)
{
	union ctu_can_fd_tx_priority reg;
	reg.u32 = 0;
	reg.s.txt1p = prio[0];
	reg.s.txt2p = prio[1];
	reg.s.txt3p = prio[2];
	reg.s.txt4p = prio[3];
	
	ctu_can_fd_write32(priv, CTU_CAN_FD_TX_PRIORITY, reg.u32);
}

bool ctu_can_fd_insert_frame(struct ctucanfd_priv *priv, const struct canfd_frame *data, u64 ts,
				u8 buf)
{
	enum ctu_can_fd_regs buf_base;
	union ctu_can_fd_frame_form_w ffw;
	union ctu_can_fd_identifier_w idw;
	struct canfd_frame *cf = (struct canfd_frame *)data; // TODO: may break alignment constraints
	u8 dlc;
	unsigned i;
	
	ffw.u32 = 0;
	idw.u32 = 0;
	 
	switch (buf){
	case CTU_CAN_FD_TXT_BUFFER_1: buf_base = CTU_CAN_FD_TXTB1_DATA_1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_2: buf_base = CTU_CAN_FD_TXTB2_DATA_1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_3: buf_base = CTU_CAN_FD_TXTB3_DATA_1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_4: buf_base = CTU_CAN_FD_TXTB4_DATA_1;
	break;
	default:
		return false;
	}

	if (!ctu_can_fd_is_txt_buf_accessible(priv, buf)) {
		return false;
	}

	if (cf->can_id & CAN_RTR_FLAG)
		ffw.s.rtr = RTR_FRAME;

	if (cf->can_id & CAN_EFF_FLAG)
		ffw.s.id_type = EXTENDED;
	else
		ffw.s.id_type = BASE;
	
	ffw.s.tbf = TIME_BASED;

	idw = ctu_can_fd_id_to_hwid(cf->can_id);

	if (!ctu_can_fd_len_to_dlc(cf->len, &dlc)) {
		return false;
	}
	ffw.s.dlc = dlc;

	// Larger data chunks and the ones where bit rate should be shifted
	// are sent as CAN FD Frames. TODO: Think here, and discuss this with Martin.
	// How does the Socket CAN distinguish beween normal and FD Frame (without BRS)
	// Once BRS flag is present it is clear. Without it, we dont know whether
	// 8 byte frame should be CAN FD Frame without bit rate shifted or CAN Frame
	// So we send FD Frame only if BRS present or higher length than 8 ...
	if (cf->len > 8)
		ffw.s.fr_type = FD_CAN;
	
	if (cf->flags & CANFD_BRS){
		ffw.s.fr_type = FD_CAN;
		ffw.s.brs = BR_SHIFT;
	}
	ctu_can_fd_write_txt_buf(priv, buf_base, CTU_CAN_FD_FRAME_FORM_W, ffw.u32);
	ctu_can_fd_write_txt_buf(priv, buf_base, CTU_CAN_FD_IDENTIFIER_W, idw.u32);
	
	ctu_can_fd_write_txt_buf(priv, buf_base, CTU_CAN_FD_TIMESTAMP_L_W, (u32)(ts));
	ctu_can_fd_write_txt_buf(priv, buf_base, CTU_CAN_FD_TIMESTAMP_U_W, (u32)(ts >> 32));
	
	for (i = 0; i < cf->len; i += 4)
		ctu_can_fd_write_txt_buf(priv, buf_base, CTU_CAN_FD_DATA_1_4_W + i,
							*(u32 *)(cf->data + i));

	return true;
}

// TODO: AL_CAPTURE and ERROR_CAPTURE

