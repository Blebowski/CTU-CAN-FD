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

#include "ctu_can_fd_frame.h"
#include "ctu_can_fd_regs.h"
#include "ctu_can_fd_hw.h"

#ifndef __linux__

#include "ctu_can_fd_linux_defs.h"

#endif


inline void ctu_can_fd_write32(const void *base, enum ctu_can_fd_regs reg,
				u32 val)
{
	iowrite32(val, base + reg);
}

inline void ctu_can_fd_write16(const void *base, enum ctu_can_fd_regs reg,
				u16 val)
{
	iowrite16(val, base + reg);
}

inline void ctu_can_fd_write8(const void *base, enum ctu_can_fd_regs reg,
				u8 val)
{
	iowrite8(val, base + reg);
}

inline void ctu_can_fd_write_txt_buf(const void *base,
					enum ctu_can_fd_regs buf_base,
					u32 offset, u32 val)
{
	iowrite32(val, base + buf_base + offset);
}

inline u32 ctu_can_fd_read32(const void *base, enum ctu_can_fd_regs reg)
{
	return ioread32(base + reg);
}

inline u16 ctu_can_fd_read16(const void *base, enum ctu_can_fd_regs reg)
{
	return ioread16(base + reg);
}

inline u8 ctu_can_fd_read8(const void *base, enum ctu_can_fd_regs reg)
{
	return ioread8(base + reg);
}

static inline void ctu_can_fd_id_to_hwid(canid_t id, union ctu_can_fd_identifier_w *hwid)
{
	hwid->u32 = 0;

	if (id & CAN_EFF_FLAG){
		hwid->s.identifier_base = (id & CAN_EFF_MASK) >> 18;

		// getting lowest 18 bits, replace with sth nicer...
		hwid->s.identifier_ext = (id & 0x3FFFF);
	}else
		hwid->s.identifier_base = id & CAN_SFF_MASK;
}
 
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


static bool ctu_can_fd_len_to_dlc(u8 len, u8 *dlc)
{
	if (len =< 8){
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
	
	if (!dlc)
		return false;
exit:
	return true;
}


bool ctu_can_fd_check_access(const void *base)
{
	union ctu_can_fd_device_id_version reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_DEVICE_ID);
	
	if (reg.s.device_id != CTU_CAN_FD_ID)
		return false;
		
	return true;
}

u32 ctu_can_fd_get_version(const void *base)
{
	union ctu_can_fd_device_id_version reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_DEVICE_ID);
	return reg.s.ver_major * 10 + reg.s.ver_minor;
}

void ctu_can_fd_enable(const void *base, bool enable)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	if (enable)
		reg.s.ena = ENABLED;
	else
		reg.s.ena = DISABLED;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

bool ctu_can_fd_set_ret_limit(const void *base, bool enable, u8 limit)
{
	union ctu_can_fd_mode_command_status_settings reg;
	
	if (limit > CTU_CAN_FD_RETR_MAX)
		return false;
 	
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	reg.s.rtrle = enable ? RTRLE_ENABLED : RTRLE_DISABLED;
	reg.s.rtrth = limit & 0xF;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
	return true;
}

void ctu_can_fd_set_mode_reg(const void *base, const struct can_ctrlmode *mode)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	
	if (mode->mask & CAN_CTRLMODE_LOOPBACK)
		reg.s.int_loop = mode->flag & CAN_CTRLMODE_LOOPBACK ?
					INT_LOOP_ENABLED : INT_LOOP_DISABLED;
	
	if (mode->mask & CAN_CTRLMODE_LISTENONLY)
		reg.s.lom = mode->flag & CAN_CTRLMODE_LISTENONLY ?
					LOM_ENABLED : LOM_DISABLED;
	
	if (mode->mask & CAN_CTRLMODE_3_SAMPLES)
		reg.s.tsm = mode->flag & CAN_CTRLMODE_3_SAMPLES ? 
				TSM_ENABLE : TSM_DISABLE;						
	
	if (mode->mask & CAN_CTRLMODE_FD)
		reg.s.fde = mode->flag & CAN_CTRLMODE_FD ? 
				FDE_ENABLE : FDE_DISABLE;
	
	if (mode->mask & CAN_CTRLMODE_PRESUME_ACK)
		reg.s.stm = mode->flag & CAN_CTRLMODE_PRESUME_ACK ? 
				STM_ENABLED : STM_DISABLED;
	
	if (mode->mask & CAN_CTRLMODE_FD_NON_ISO)
		reg.s.fd_type = mode->flag & CAN_CTRLMODE_FD_NON_ISO ?
				NON_ISO_FD : ISO_FD;
								
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_rel_rx_buf(const void *base)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	reg.s.rrb = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_clr_overrun_flag(const void *base)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	reg.s.cdo = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_abort_tx(const void *base)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	reg.s.at = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

union ctu_can_fd_mode_command_status_settings ctu_can_get_status(const void *base)
{
	// MODE and STATUS are within the same word
	return ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
}

union ctu_can_fd_int_stat ctu_can_fd_int_sts(const void *base)
{
	return ctu_can_fd_read32(base, CTU_CAN_FD_INT_STAT);
}

void ctu_can_fd_int_clr(const void *base, const union ctu_can_fd_int_stat *mask)
{
	ctu_can_fd_write32(base, CTU_CAN_FD_INT_STAT, mask->u32);
}

static void ctu_can_fd_int_conf(const void *base, enum ctu_can_fd_regs sreg,
				enum ctu_can_fd_regs creg, 
				const union ctu_can_fd_int_stat *mask,
				const union ctu_can_fd_int_stat *val)
{
	union ctu_can_fd_int_stat reg;
	reg.u32 = ctu_can_fd_read32(base, sreg);
	
	ctu_can_fd_write32(base, sreg, mask.u32 & val.u32);
	ctu_can_fd_write32(base, creg, mask.u32 & (~val.u32));
}

void ctu_can_fd_int_ena(const void *base, const union ctu_can_fd_int_stat *mask,
			const union ctu_can_fd_int_stat *val)
{
	ctu_can_fd_int_conf(base, CTU_CAN_FD_INT_ENA_SET, CTU_CAN_FD_INT_ENA_CLR,
				mask, val);
}

void ctu_can_fd_int_mask(const void *base, const union ctu_can_fd_int_stat *mask,
			 const union ctu_can_fd_int_stat *val)
{
	ctu_can_fd_int_conf(base, CTU_CAN_FD_INT_MASK_SET, CTU_CAN_FD_INT_MASK_CLR,
				mask, val);
}

void ctu_can_fd_set_mode(const void *base, const struct can_ctrlmode *mode)
{
	ctu_can_fd_set_mode_reg(base, mode);
	
	// One shot mode supported indirectly via Retransmitt limit
	if (mode->mask & CAN_CTRLMODE_ONE_SHOT)
		ctu_can_fd_set_ret_limit(base, true, 0);

	// Bus error reporting -> Allow Error interrupt
	union ctu_can_fd_int_stat reg;
	reg.u32 = 0;
	reg.s.bei = 1;
	ctu_can_fd_int_ena(base, &reg, &reg);
}


static const struct can_bittiming_const ctu_can_fd_bit_timing_max = {
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

static const struct can_bittiming_const ctu_can_fd_bit_timing_data_max = {
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

void ctu_can_fd_set_nom_bittiming(const void *base,
				  const struct can_bittiming *nbt)
{
	union ctu_can_fd_btr btr;
	btr.u32 = 0;
	btr.prop = nbt->s.prop_seg;
	btr.ph1 = nbt->s.phase_seg_1;
	btr.ph2 = nbt->s.phase_seg_2;
	btr.brp = nbt->s.brp;
	btr.sjw = nbt->s.sjw;

	ctu_can_fd_write32(base, CTU_CAN_FD_BTR, btr->u32);
}

void ctu_can_fd_set_data_bittiming(const void *base,
				   const struct can_bittiming *dbt)
{
	union ctu_can_fd_btr_fd btr_fd;
	btr_fd.u32 = 0;
	btr_fd.prop_fd = dbt->s.prop_seg;
	btr_fd.ph1_fd = dbt->s.phase_seg_1;
	btr_fd.ph2_fd = dbt->s.phase_seg_2;
	btr_fd.brp_fd = dbt->s.brp;
	btr_fd.sjw_fd = dbt->s.sjw;
	
	ctu_can_fd_write32(base, CTU_CAN_FD_BTR_FD, btr->u32);
}

void ctu_can_fd_set_err_limits(const void *base, u8 ewl, u8 erp)
{
	union ctu_can_fd_ewl_erp_fault_state reg;
	reg.u32 = 0;
	reg.s.ewl_limit = ewl;
	reg.s.erp_limit = erp;
	// era, bof, erp are read-only
	
	ctu_can_fd_write32(base, CTU_CAN_FD_EWL, reg->u32);
}

void ctu_can_fd_set_def_err_limits(const void *base)
{
	ctu_can_fd_set_ewl_limits(base, 96, 128);
}

bool ctu_can_fd_read_err_ctrs(const void *base, struct can_berr_counter *ctr)
{
	union ctu_can_fd_rxc_txc reg;
	
	if (ctr == NULL)
		return false;
	
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RXC);
	ctr->txerr = reg.s.rxc_val;
	ctr->rxerr = reg.s.txc_val;
	return true;
}

u16 ctu_can_fd_read_nom_errs(const void *base)
{
	union ctu_can_fd_err_norm_err_fd reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_ERR_NORM);
	return reg.s.err_norm_val;
}

void ctu_can_fd_erase_nom_errs(const void *base)
{
	union ctu_can_fd_ctr_pres reg;
	reg.u32 = 0;
	reg.s.enorm = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_CTR_PRES, reg->u32);
}

u16 ctu_can_fd_read_fd_errs(const void *base)
{
	union ctu_can_fd_err_norm_err_fd reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_ERR_NORM);
	return reg.s.err_fd_val;
}

void ctu_can_fd_erase_fd_errs(const void *base)
{
	union ctu_can_fd_ctr_pres reg;
	reg.u32 = 0;
	reg.s.efd = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_CTR_PRES, reg->u32);
}

enum can_state ctu_can_fd_read_error_state(const void *base)
{
	union ctu_can_fd_ewl_erp_fault_state reg;
	union ctu_can_fd_rxc_txc err;

	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_EWL);
	err.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RXC);

	if (reg.s.era){
		if (reg.s.ewl_limit > err.s.rxc && reg.s.ewl_limit > err.s.txc) 
			return CAN_STATE_ERROR_ACTIVE;
		else
			return CAN_STATE_ERROR_WARNING;
	}else if (reg.s.erp)
		return CAN_STATE_ERROR_PASSIVE;
	else if (reg.s.bof)
		return CAN_STATE_BUS_OFF;
}

void ctu_can_fd_set_err_ctrs(const void *base, const struct can_berr_counter *ctr)
{
	union ctu_can_fd_ctr_pres reg;
	reg.u32 = 0;

	reg.s.ctpv = ctr->txerr;
	reg.s.ptx = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_CTR_PRES, reg->u32);

	reg.s.ctpv = ctr->rxerr;
	reg.s.ptx = 0;
	reg.s.prx = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_CTR_PRES, reg->u32);
}


bool ctu_can_fd_get_mask_filter_support(const void *base, u8 fnum)
{
	union ctu_can_fd_filter_control_filter_status reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_FILTER_CONTROL);

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

bool ctu_can_fd_get_range_filter_support(const void *base)
{
	union ctu_can_fd_filter_control_filter_status reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_FILTER_CONTROL);

	if (reg.s.sfr)
		return true;

	return false;
}

bool ctu_can_fd_set_mask_filter(const void *base, u8 fnum, bool enable,
				const struct can_filter *filter)
{
	union ctu_can_fd_filter_control_filter_status creg;
	enum ctu_can_fd_regs maddr,vaddr;
	union ctu_can_fd_identifier_w hwid_mask;	
	union ctu_can_fd_identifier_w hwid_val;	
	uint8_t val = 0;

	if (!ctu_can_fd_get_mask_filter_support(base, fnum))
		return false;

	if (enable)
		val = 1;

	creg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_FILTER_CONTROL);
	maddr.u32 = 0;
	vaddr.u32 = 0;

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

	ctu_can_fd_id_to_hwid(filter->can_id, &hwid_mask);
	ctu_can_fd_id_to_hwid(filter->can_mask, &hwid_val);
	ctu_can_fd_write32(base, CTU_CAN_FD_FILTER_CONTROL, creg.u32);
	ctu_can_fd_write32(base, maddr, hwid_mask.u32);
	ctu_can_fd_write32(base, vaddr, hwid_val.u32);
	return true;
}

void ctu_can_fd_set_range_filter(const void *base, canid_t low_th,
				 canid_t high_th, bool enable)
{
	union ctu_can_fd_identifier_w hwid_low;	
	union ctu_can_fd_identifier_w hwid_high;
	union ctu_can_fd_filter_control_filter_status creg;
	
	hwid_low.u32 = ctu_can_fd_id_to_hwid(low_th, &hwid_low);
	hwid_high.u32 = ctu_can_fd_id_to_hwid(high_th, &hwid_high);

	creg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_FILTER_CONTROL);
	
	if (enable){
		creg.s.frnb = 1;
		creg.s.frne = 1;
		creg.s.frfb = 1;
		creg.s.frfe = 1;
	} else {
		creg.s.frnb = 0;
		creg.s.frne = 0;
		creg.s.frfb = 0;
		creg.s.frfe = 0;
	}
	ctu_can_fd_write32(base, CTU_CAN_FD_FILTER_CONTROL, creg.u32);
	ctu_can_fd_write32(base, CTU_CAN_FD_FILTER_RAN_LOW, hwid_low.u32);
	ctu_can_fd_write32(base, CTU_CAN_FD_FILTER_RAN_HIGH, hwid_high.u32);
}

u16 ctu_can_fd_get_rx_fifo_size(const void *base)
{
	union ctu_can_fd_rx_mem_info reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_MEM_INFO);
	return reg.s.rx_buff_size;
}

u16 ctu_can_fd_get_rx_fifo_mem_free(const void *base)
{
	union ctu_can_fd_rx_mem_info reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_MEM_INFO);
	return reg.s.rx_mem_free;
}

bool ctu_can_fd_is_rx_fifo_empty(const void *base)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_STATUS);
	return reg.s.rx_empty;
}

bool ctu_can_fd_is_rx_fifo_full(const void *base)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_STATUS);
	return reg.s.rx_full;
}

u16 ctu_can_fd_get_rx_frame_count(const void *base)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_STATUS);
	return reg.s.rx_frc;
}

void ctu_can_fd_set_rx_tsop(const void *base, enum ctu_can_fd_rx_settings_rtsop val)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = 0;
	reg.s.rtsop = val;
	ctu_can_fd_write32(base, CTU_CAN_FD_RX_STATUS, reg->u32);
}

void ctu_can_fd_read_rx_frame(const void *base, unsigned char *data, u64 *ts)
{
	struct can_fd_frame *cf = (struct can_fd_frame *)data;
	union ctu_can_fd_frame_form_w ffw;
	union ctu_can_fd_identifier_w idw;
	
	ffw.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_DATA);
	idw.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_DATA);
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
	if (ffw.s.dlc =< 8){
		cf->len = ffw.dlc;
	}else{
		if (ffw.s.fr_type == FD_CAN)
			cf->len = (ffw.s.rwcnt - 3) << 2;
		else
			cf->len = 8;
	}

	ctu_can_fd_hwid_to_id(idw, &(cf->can_id), ffw.s.id_type);
	
	// Timestamp
	*ts = (u64)(ctu_can_fd_read32(base, CTU_CAN_FD_RX_DATA));
	*ts |= ((u64)ctu_can_fd_read32(base, CTU_CAN_FD_RX_DATA) << 32);
	
	// Data
	for (i = 0; i < cf->len; i += 4)
		*(u32 *)(cf->data + i) = ctu_can_fd_read32(base, CTU_CAN_FD_RX_DATA);
}

enum ctu_can_fd_tx_status_tx1s ctu_can_fd_get_tx_status(const void *base, u8 buf)
{
	union ctu_can_fd_tx_status reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_TX_STATUS);
	
	switch (buf) {
	case CTU_CAN_FD_TXT_BUFFER_1 : return reg.s.tx1s;
	break;
	case CTU_CAN_FD_TXT_BUFFER_2 : return reg.s.tx2s;
	break;
	case CTU_CAN_FD_TXT_BUFFER_3 : return reg.s.tx3s;
	break;
	case CTU_CAN_FD_TXT_BUFFER_4 : return reg.s.tx4s;
	break;
	default :
		return ~0;
	}
}

bool ctu_can_fd_is_txt_buf_accessible(const void *base, u8 buf)
{
	enum ctu_can_fd_tx_status_tx1s buf_status;

	buf_status = ctu_can_fd_get_tx_status(base, buf)
	if (buf_status == TXT_RDY || buf_status == TXT_TRAN 
		|| buf_status == TXT_ABTP)  
 		return false;

	return true;
}

static bool ctu_can_fd_txt_buf_give_command(const void *base, u8 cmd, u8 buf)
{
	union ctu_can_fd_tx_command reg;
	reg.u32 = 0;
	
	switch (buf){
	case CTU_CAN_FD_TXT_BUFFER_1: reg.s.tx1 = 1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_2: reg.s.tx2 = 1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_3: reg.s.tx3 = 1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_4: reg.s.tx4 = 1;
	break;
	default:		
		return false;
	}
	
	if (cmd & 0x1) {
		reg.s.txce = 1;
	} else if (cmd & 0x2) {
		reg.s.txcr = 1;
	} else if (cmd & 0x4) {
		reg.s.txca = 1;
	} else {
		return false;
	}
	
	ctu_can_fd_write32(base, CTU_CAN_FD_TX_COMMAND, reg->u32);
}

inline void ctu_can_fd_txt_set_empty(const void *base, u8 buf)
{
	ctu_can_fd_txt_buf_give_command(base, 0x1, buf)
}

inline void ctu_can_fd_txt_set_rdy(const void *base, u8 buf)
{
	ctu_can_fd_txt_buf_give_command(base, 0x2, buf)
}

inline void ctu_can_fd_txt_set_abort(const void *base, u8 buf)
{
	ctu_can_fd_txt_buf_give_command(base, 0x4, buf)
}

void ctu_can_fd_set_txt_priority(const void *base, const u8 *prio)
{
	union ctu_can_fd_tx_priority reg;
	reg.u32 = 0;
	reg.s.txt1p = prio[0];
	reg.s.txt2p = prio[1];
	reg.s.txt3p = prio[2];
	reg.s.txt4p = prio[3];
	
	ctu_can_fd_write32(base, CTU_CAN_FD_TX_PRIORITY, reg->u32);
}

bool ctu_can_fd_insert_frame(const void *base, const unsigned char *data, u64 ts,
				u8 buf)
{
	enum ctu_can_fd_regs buf;
	union ctu_can_fd_frame_form_w ffw;
	union ctu_can_fd_identifier_w idw;
	struct canfd_frame *cf = (struct canfd_frame *)data;
	u8 dlc;
	
	ffw.u32 = 0;
	idw.u32 = 0;
	 
	switch (buf){
	case CTU_CAN_FD_TXT_BUFFER_1: buf = CTU_CAN_FD_TXTB1_DATA_1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_2: buf = CTU_CAN_FD_TXTB2_DATA_1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_3: buf = CTU_CAN_FD_TXTB3_DATA_1;
	break;
	case CTU_CAN_FD_TXT_BUFFER_4: buf = CTU_CAN_FD_TXTB4_DATA_1;
	break;
	default:
		return false;
	}

	if (!ctu_can_fd_is_txt_buf_accessible(base, buf))
		return false;

	if (cf->ident & CAN_RTR_FLAG)
		ffw.s.rtr = RTR_FRAME;

	if (cf->ident & CAN_EFF_FLAG)
		ffw.s.id_type = EXTENDED;
 	else
		ffw.s.id_type = BASE;
	
	ffw.s.tbf = TIME_BASED;

	ctu_can_fd_id_to_hwid(cf->ident, &idw);

	if (!ctu_can_fd_len_to_dlc(cf->len, &dlc))
		return false;
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
	ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_FRAME_FORM_W, ffw.u32);
	ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_IDENTIFIER_W, idw.u32);
	
	ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_TIMESTAMP_L_W, (u32)(*ts));
	ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_TIMESTAMP_U_W, (u32)(*ts >> 32));
	
	for (i = 0; i < cf->len; i += 4)
		ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_DATA_1_4_W + i,
							*(u32 *)(cf->data + i));

	return true;
}

// TODO: AL_CAPTURE and ERROR_CAPTURE

u16 ctu_can_fd_get_tran_delay(const void *base)
{
	union ctu_can_fd_trv_delay reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_TRV_DELAY);
	return reg.s.trv_delay_value;
}

u32 ctu_can_fd_get_tx_frame_ctr(const void *base)
{
	union ctu_can_fd_rx_counter reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_TX_COUNTER);
	return reg.s.tx_counter_val;
}

u32 ctu_can_fd_get_rx_frame_ctr(const void *base)
{
	union ctu_can_fd_rx_counter reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_COUNTER);
	return reg.s.rx_counter_val;
}

union ctu_can_fd_debug_register ctu_can_fd_read_debug_info(const void *base)
{
	return ctu_can_fd_read32(base, CTU_CAN_FD_DEBUG_REGISTER);
}

