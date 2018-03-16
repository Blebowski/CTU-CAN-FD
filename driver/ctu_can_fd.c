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

#include <ctu_can_fd_frame.h>
#include <ctu_can_fd_regs.h>

#include <ctu_can_fd_linux_defs.h>


/* Memory access functions */
static inline void ctu_can_fd_write32(const void *base, enum ctu_can_fd_regs reg,
					u32 val)
{
	 iowrite32(val, base + reg);
}

static inline void ctu_can_fd_write16(const void *base, enum ctu_can_fd_regs reg,
					u16 val)
{
	 iowrite16(val, base + reg);
}

static inline void ctu_can_fd_write8(const void *base, enum ctu_can_fd_regs reg,
					u8 val)
{
	 iowrite8(val, base + reg);
}

static inline void ctu_can_fd_write_txt_buf(const void *base,
						enum ctu_can_fd_regs buf_base,
						u32 offset, u32 val)
{
	iowrite32(val, base + buf_base + offset);
}

static inline u32 ctu_can_fd_read32(const void *base, enum ctu_can_fd_regs reg)
{
	 return ioread32(base + reg);
}

static inline u16 ctu_can_fd_read16(const void *base, enum ctu_can_fd_regs reg)
{
	 return ioread16(base + reg);
}

static inline u8 ctu_can_fd_read8(const void *base, enum ctu_can_fd_regs reg)
{
	 return ioread8(base + reg);
}

bool ctu_can_fd_check_access(const void *base)
{
	union ctu_can_fd_device_id_version reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_DEVICE_ID);
	
	if (reg.device_id != CTU_CAN_FD_ID)
		return FALSE;
		
	return TRUE;
}

u32 ctu_can_fd_get_version(const void *base)
{
	union ctu_can_fd_device_id_version reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_DEVICE_ID);
	return reg.ver_major * 10 + reg.ver_minor;
}

void ctu_can_fd_enable(const void *base, bool enable)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	if (enable)
		reg.ena = ENABLED;
	else
		reg.ena = DISABLED;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_set_mode(const void *base, struct can_ctrlmode *mode)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	
	// TODO: What to do with "One shot mode" and "Bus error reporting
	
	if (mode->mask & CAN_CTRLMODE_LOOPBACK)
		reg.int_loop = mode->flag & CAN_CTRLMODE_LOOPBACK ?
					INT_LOOP_ENABLED : INT_LOOP_DISABLED;
	
	if (mode->mask & CAN_CTRLMODE_LISTENONLY)
		reg.lom = mode->flag & CAN_CTRLMODE_LISTENONLY ?
					LOM_ENABLED : LOM_DISABLED;
	
	if (mode->mask & CAN_CTRLMODE_3_SAMPLES)
		reg.tsm = mode->flag & CAN_CTRLMODE_3_SAMPLES ? 
				TSM_ENABLE : TSM_DISABLE;						
	
	if (mode->mask & CAN_CTRLMODE_FD)
		reg.fde = mode->flag & CAN_CTRLMODE_FD ? 
				FDE_ENABLE : FDE_DISABLE;
	
	if (mode->mask & CAN_CTRLMODE_PRESUME_ACK)
		reg.stm = mode->flag & CAN_CTRLMODE_PRESUME_ACK ? 
				STM_ENABLED : STM_DISABLED;
	
	if (mode->mask & CAN_CTRLMODE_FD_NON_ISO)
		reg.fd_type = mode->flag & CAN_CTRLMODE_FD_NON_ISO ?
				NON_ISO_FD : ISO_FD;
								
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_set_ret_limit(const void *base, bool enable, u8 limit)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);

	reg.rtrle = enable ? RTRLE_ENABLED : RTRLE_DISABLED;
	reg.rtrth = limit & 0xF;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_rel_rx_buf(const void *base)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	reg.rrb = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_clr_overrun_flag(const void *base)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	reg.cdo = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}

void ctu_can_fd_abort_tx(const void *base)
{
	union ctu_can_fd_mode_command_status_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_MODE);
	reg.at = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_MODE, reg.u32);
}


// TODO: What to do with the status register ???


union ctu_can_fd_int_stat ctu_can_fd_int_sts(const void *base)
{
	return ctu_can_fd_read32(base, CTU_CAN_FD_INT_STAT);
}

void ctu_can_fd_int_clr(const void *base, union ctu_can_fd_int_stat *mask)
{
	ctu_can_fd_write32(base, CTU_CAN_FD_INT_STAT, mask->u32);
}

static void ctu_can_fd_int_conf(const void *base,
				const enum ctu_can_fd_regs sreg,
				const enum ctu_can_fd_regs creg, 
				union ctu_can_fd_int_stat *mask,
				union ctu_can_fd_int_stat *val)
{
	union ctu_can_fd_int_stat reg;
	reg.u32 = ctu_can_fd_read32(base, sreg);
	
	ctu_can_fd_write32(base , sreg, mask.u32 & val.u32);
	ctu_can_fd_write32(base , creg, mask.u32 & (~val.u32));
}

void ctu_can_fd_int_ena(const void *base, union ctu_can_fd_int_stat *mask,
			union ctu_can_fd_int_stat *val)
{
	ctu_can_fd_int_conf(base, CTU_CAN_FD_INT_ENA_SET, CTU_CAN_FD_INT_ENA_CLR,
				mask, val);
}

void ctu_can_fd_int_mask(const void *base, union ctu_can_fd_int_stat *mask,
				union ctu_can_fd_int_stat *val)
{
	ctu_can_fd_int_conf(base, CTU_CAN_FD_INT_MASK_SET, CTU_CAN_FD_INT_MASK_CLR,
						mask, val);
}

static const struct can_bittiming_const ctu_can_fd_bit_timing_max = {
	.name = "TODO",
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
	.name = "TODO",
	.tseg1_min = 2,
	.tseg1_max = 94,
	.tseg2_min = 1,
	.tseg2_max = 31,
	.sjw_max = 31,
	.brp_min = 1,
	.brp_max = 255,
	.brp_inc = 1,
};


// TODO: Higher level Socket CAN module should perform check (I assume during
//  IOCTL from user-land) whether passed parameters does not exceed the maximal
//  values!

void ctu_can_fd_set_nom_bittiming(const void *base, struct can_bittiming *nbt)
{
	union ctu_can_fd_btr btr;
	btr.u32 = 0;
	btr.prop = nbt->prop_seg;
	btr.ph1 = nbt->phase_seg_1;
	btr.ph2 = nbt->phase_seg_2;
	btr.brp = nbt->brp;
	btr.sjw = nbt->sjw;
	
	ctu_can_fd_write32(base, CTU_CAN_FD_BTR, btr->u32);
}

void ctu_can_fd_set_data_bittiming(const void *base, struct can_bittiming *dbt)
{
	union ctu_can_fd_btr_fd btr_fd;
	btr_fd.u32 = 0;
	btr_fd.prop_fd = dbt->prop_seg;
	btr_fd.ph1_fd = dbt->phase_seg_1;
	btr_fd.ph2_fd = dbt->phase_seg_2;
	btr_fd.brp_fd = dbt->brp;
	btr_fd.sjw_fd = dbt->sjw;
	
	ctu_can_fd_write32(base, CTU_CAN_FD_BTR_FD, btr->u32);
}

void ctu_can_fd_set_err_limits(const void *base, u8 ewl, u8 erp)
{
	union ctu_can_fd_ewl_erp_fault_state reg;
	reg.u32 = 0;
	reg.ewl_limit = ewl;
	reg.erp_limit = erp;
	// era, bof, erp are read-only
	
	ctu_can_fd_write32(base, CTU_CAN_FD_EWL, reg->u32);
}

void ctu_can_fd_set_def_err_limits(const void *base)
{
	ctu_can_fd_set_ewl_limits(base, 96, 128);
}

void ctu_can_fd_read_err_ctrs(const void *base, struct can_berr_counter *ctr)
{
	union ctu_can_fd_rxc_txc reg;
	
	//TODO: Check the counter struct pointer
	
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RXC);
	ctr->txerr = reg.rxc_val;
	ctr->rxerr = reg.txc_val;
}

u16 ctu_can_fd_read_nom_errs(const void *base)
{
	union ctu_can_fd_err_norm_err_fd reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_ERR_NORM);
	return reg.err_norm_val;
}

void ctu_can_fd_erase_nom_errs(const void *base)
{
	union ctu_can_fd_ctr_pres reg;
	reg.u32 = 0;
	reg.enorm = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_CTR_PRES, reg->u32);
}

void ctu_can_fd_erase_nom_errs(const void *base)
{
	union ctu_can_fd_ctr_pres reg;
	reg.u32 = 0;
	reg.efd = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_CTR_PRES, reg->u32);
}

u16 ctu_can_fd_read_fd_errs(const void *base)
{
	union ctu_can_fd_err_norm_err_fd reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_ERR_NORM);
	return reg.err_fd_val;
}

enum can_state ctu_can_fd_read_error_state(const void *base)
{
	union ctu_can_fd_ewl_erp_fault_state reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_BTR);
	
	if (reg.era)
		return CAN_STATE_ERROR_ACTIVE;
		// TODO: read the error counters and recognize ewl!
	else if (reg.erp)
		return CAN_STATE_ERROR_PASSIVE;
	else if (reg.bof)
		return CAN_STATE_BUS_OFF;
}

void ctu_can_fd_set_err_ctrs(const void *base, struct can_berr_counter *ctr)
{
	union ctu_can_fd_ctr_pres reg;
	reg.u32 = 0;
	
	reg.ctpv = ctr->txerr;
	reg.ptx = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_CTR_PRES, reg->u32);
	
	reg.ctpv = ctr->rxerr;
	reg.ptx = 0;
	reg.prx = 1;
	ctu_can_fd_write32(base, CTU_CAN_FD_CTR_PRES, reg->u32);
}

// TODO: Configuration of fitlers! We should unify filters A,B,C to have
//       common handling function! 
//		 We will need to define fitler enums manually!


u16 ctu_can_fd_get_rx_fifo_size(const void *base)
{
	union ctu_can_fd_rx_mem_info reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_MEM_INFO);
	return reg.rx_buff_size;
}

u16 ctu_can_fd_get_rx_fifo_mem_free(const void *base)
{
	union ctu_can_fd_rx_mem_info reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_MEM_INFO);
	return reg.rx_mem_free;
}

bool ctu_can_fd_is_rx_fifo_empty(const void *base)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_STATUS);
	return reg.rx_empty;
}

bool ctu_can_fd_is_rx_fifo_full(const void *base)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_STATUS);
	return reg.rx_full;
}

bool ctu_can_fd_is_rx_fifo_full(const void *base)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_STATUS);
	return reg.rx_full;
}

u16 ctu_can_fd_get_rx_frame_count(const void *base)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_STATUS);
	return reg.rx_frc;
}

void ctu_can_fd_set_rx_tsop(const void *base, enum ctu_can_fd_rx_settings_rtsop val)
{
	union ctu_can_fd_rx_status_rx_settings reg;
	reg.u32 = 0;
	reg.rtsop = val;
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
	if (ffw.fr_type == FD_CAN){
		if (ffw.brs == BR_SHIFT)
			cf->flags |= CANFD_BRS; 
		if (ffw.esi_resvd == ESI_ERR_PASIVE)
			cf->flags |= CANFD_ESI;
	}else if (ffw.rtr == RTR_FRAME)
		cf->can_id |= CAN_RTR_FLAG;
	
	// DLC
	if (ffw.dlc =< 8){
		cf->len = ffw.dlc;
	}else{
		if (ffw.fr_type == FD_CAN)
			cf->len = (ffw.rwcnt - 3) << 2;
		else
			cf->len = 8;
	}

	// Identifier
	if (ffw.id_type == EXTENDED){
		cf->can_id |= CAN_EFF_FLAG;
		cf->can_id |= idw & CAN_EFF_MASK;
	} else
		cf->can_id |= (idw >> (CAN_EFF_ID_BITS - CAN_SFF_ID_BITS)) & CAN_SFF_MASK;
	
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
	case 1 : return reg.tx1s;
	break;
	case 2 : return reg.tx2s;
	break;
	case 3 : return reg.tx3s;
	break;
	case 4 : return reg.tx4s;
	break;
	default :
		// TODO: print error is non-of the above
		// Should we return some standard error code ???
		return ~0;
	break;
	}
}

static void ctu_can_fd_txt_buf_give_command(const void *base, u8 cmd, u8 buf)
{
	union ctu_can_fd_tx_command reg;
	reg.u32 = 0;
	
	switch (buf)
	case 1: reg.tx1 = 1;
	break;
	case 2: reg.tx2 = 1;
	break;
	case 3: reg.tx3 = 1;
	break;
	case 4: reg.tx4 = 1;
	break;
	default:
		// TODO: log the error, invalid buffer
		return;
	
	if (cmd & 0x1) {
		reg.txce = 1;
	} else if (cmd & 0x2) {
		reg.txcr = 1;
	} else if (cmd & 0x4) {
		reg.txca = 1;
	} else {
		// TODO: Log the invalid command error
		return;
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

// TODO: Should I replace literals in the upper functions by enum value?
//		 Should this macro be generated or harcdoded? Generated would involve
//		 changing the command field in the IP-XACT


// TODO: Replace the buffer count with macro defined value
void ctu_can_fd_set_txt_priority(const void *base,
								 u8 *prio[4])
{
	union ctu_can_fd_tx_priority reg;
	reg.u32 = 0;
	reg.txt1p = prio[0];
	reg.txt2p = prio[1];
	reg.txt3p = prio[2];
	reg.txt4p = prio[3];
	
	ctu_can_fd_write32(base, CTU_CAN_FD_TX_PRIORITY, reg->u32);
}

void ctu_can_fd_insert_frame(const void *base, unsigned char *data, u64 *ts,
								u8 buf)
{
	enum ctu_can_fd_regs buf;
	union ctu_can_fd_frame_form_w ffw;
	union ctu_can_fd_identifier_w idw;
	struct canfd_frame *cf = (struct canfd_frame *)data;
	ffw.u32 = 0;
	idw.u32 = 0;
	 
	switch (buf){
	case 1: buf = CTU_CAN_FD_TXTB1_DATA_1;
	break;
	case 2: buf = CTU_CAN_FD_TXTB2_DATA_1;
	break;
	case 3: buf = CTU_CAN_FD_TXTB3_DATA_1;
	break;
	case 4: buf = CTU_CAN_FD_TXTB4_DATA_1;
	break;
	default:
		//TODO: print buffer not supported
		return;
	}
	 
	if (cf->ident & CAN_RTR_FLAG)
		ffw.rtr = RTR_FRAME;
	if (cf->ident & CAN_EFF_FLAG){
		ffw.id_type = EXTENDED;
		idw.identifier_base = (cf->ident & CAN_EFF_MASK) >> 
					(CAN_EFF_ID_BITS - CAN_SFF_ID_BITS);
		idw.identifier_ext = cf->ident & (CAN_EFF_MASK >> 
					(CAN_EFF_ID_BITS - CAN_SFF_ID_BITS));
	} else {
		ffw.id_type = BASE;
		idw.identifer_base = (cf->ident & CAN_SFF_MASK);
	}
	ffw.tbf = TIME_BASED;
	
	// TODO: DLC!!!
	
	// Larger data chunks and the ones where bit rate should be shifted
	// are sent as CAN FD Frames.
	if (cf->len > 8)
		ffw.fr_type = FD_CAN;
	
	if (cf->flags & CANFD_BRS){
		ffw.fr_type = FD_CAN;
		ffw.brs = BR_SHIFT; 
	}
	ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_FRAME_FORM_W, ffw.u32);
	ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_IDENTIFIER_W, idw.u32);
	
	ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_TIMESTAMP_L_W, (u32)(*ts));
	ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_TIMESTAMP_U_W, (u32)(*ts >> 32));
	
	for (i = 0; i < cf->len; i += 4)
		ctu_can_fd_write_txt_buf(base, buf, CTU_CAN_FD_DATA_1_4_W + i,
							*(u32 *)(cf->data + i));
}

// TODO: AL_CAPTURE and ERROR_CAPTURE

u16 ctu_can_fd_get_tran_delay(const void *base)
{
	union ctu_can_fd_trv_delay reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_TRV_DELAY);
	return reg.trv_delay_value;
}

u32 ctu_can_fd_get_tx_frame_ctr(const void *base)
{
	union ctu_can_fd_rx_counter reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_TX_COUNTER);
	return reg.tx_counter_val;
}

u32 ctu_can_fd_get_rx_frame_ctr(const void *base)
{
	union ctu_can_fd_rx_counter reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_COUNTER);
	return reg.rx_counter_val;
}

u32 ctu_can_fd_get_rx_frame_ctr(const void *base)
{
	union ctu_can_fd_rx_counter reg;
	reg.u32 = ctu_can_fd_read32(base, CTU_CAN_FD_RX_COUNTER);
	return reg.rx_counter_val;
}

// TODO: Debug and YOLO registers!


