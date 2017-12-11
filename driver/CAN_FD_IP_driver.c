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
 *
 *  ************************
 *  ** CAN_FD_IP_driver.c **
 *  ************************
 *
 *  Revision History Date Author Comments:
 *    24.8.2016   Created file
 *    26.8.2016	  Replaced boolean return values with STATUS enum with more understandable
 *    			  	error codes!
 *
 */

#include "CAN_FD_IP_driver.h"



STATUS check_node_access(can_node_t node)
{
	uint32_t val = READ_VALUE_32BIT(node, IDENT_REG);
	if ( val == CAN_IP_CORE_IDENTIFIER )
		return SUCESS;
	return INCORRECT_VAL;
}


void enable_controller(can_node_t node, bool enable)
{
	uint32_t val = READ_VALUE_32BIT(node, MODE_REG);
	val = enable ? SET_BIT(val, ENA_BIT) : CLEAR_BIT(val, ENA_BIT);
	WRITE_VALUE_32BIT(node, MODE_REG, val);
}


void set_mode_ind(can_node_t node, uint8_t mode_off, bool enable)
{
	uint32_t val = READ_VALUE_32BIT(node, MODE_REG);
	val = enable ? SET_BIT(val, mode_off) : CLEAR_BIT(val, mode_off);
	WRITE_VALUE_32BIT(node, MODE_REG, val);
}


void set_mode_all(can_node_t node, uint8_t vector)
{
	uint32_t val = READ_VALUE_32BIT(node, MODE_REG);
	val = CLEAR_BYTE(val, MODE_BYTE);
	val = PLACE_BYTE(val, MODE_BYTE, vector);
	WRITE_VALUE_32BIT(node, MODE_REG, val);
}


void give_command(can_node_t node, can_command_t command)
{
	uint32_t val;
	val = READ_VALUE_32BIT(node, MODE_REG);
	if (command == RESET){
		val = SET_BIT(val, RST_BIT);
	}else if (command == ABORT_TRANSMISSION){
		val = SET_BIT(val, AT_BIT);
	}else if (command == RELEASE_RX_BUFFER){
		val = SET_BIT(val, RRB_BIT);
	}else if (command == CLEAR_DATA_OVERRUN){
		val = SET_BIT(val, CDO_BIT);
	}
	WRITE_VALUE_32BIT(node, MODE_REG, val);
}


uint8_t get_status(can_node_t node)
{
	uint32_t val = READ_VALUE_32BIT(node, MODE_REG);
	return (uint8_t) (val >> RBS_BIT);
}


void set_settings(can_node_t node,struct can_settings_type_t *settings)
{
	uint32_t val = READ_VALUE_32BIT(node, MODE_REG);
	val = CLEAR_BYTE(val, SETTINGS_BYTE);

	val = settings->retr_limit_ena ? SET_BIT(val, RTRLE_BIT) : CLEAR_BIT(val, RTRLE_BIT);
	val |= (settings->retr_treshold & 0xF) << RTR_TH_LOW;
	val = settings->int_loopback ? SET_BIT(val, INT_LOOP_BIT) : CLEAR_BIT(val, INT_LOOP_BIT);
    val = settings->enable ? SET_BIT(val, ENA_BIT) : CLEAR_BIT(val, ENA_BIT);
	val = PLACE_BIT(val, FD_TYPE_BIT, settings->fd_type);

	WRITE_VALUE_32BIT(node, MODE_REG, val);
}


STATUS get_settings(can_node_t node, struct can_settings_type_t *settings)
{
	if (settings == NULL)
		return INVALID_PTR;

	uint32_t val = READ_VALUE_32BIT(node, MODE_REG);

	settings->enable 			= IS_BIT_SET(val, ENA_BIT);
	settings->int_loopback 		= IS_BIT_SET(val, INT_LOOP_BIT);
	settings->retr_limit_ena 	= IS_BIT_SET(val, RTRLE_BIT);
	settings->fd_type 			= (IS_BIT_SET(val, FD_TYPE_BIT)) ? NON_ISO_FD : ISO_FD;
	settings->retr_treshold 	= 0xF & (val >> RTR_TH_LOW);

	return SUCESS;
}


void enable_interrupt_ind(can_node_t node, uint8_t int_offset, boolean enable)
{
	uint32_t val = READ_VALUE_32BIT(node, INTERRUPT_REG);
	val = enable ? SET_BIT(val, int_offset) : CLEAR_BIT(val, int_offset);
	WRITE_VALUE_32BIT(node, INTERRUPT_REG, val);
}


void enable_interrupt_all(can_node_t node, uint16_t int_vect)
{
	uint32_t val = 0x0; //Don't read here it would erase actual vector
	val = int_vect << RI_ENA_BIT;
	WRITE_VALUE_32BIT(node, INTERRUPT_REG, val);
}


uint16_t get_interrupt_vect(can_node_t node)
{
	uint32_t val = READ_VALUE_32BIT(node, INTERRUPT_REG);
	return (uint16_t) (val & 0xFFFF);
}


void set_bus_timing(can_node_t node,struct can_bus_timing_t *timing)
{
	uint32_t val;

	val  = (_6BIT_MASK & timing->prs_nbt) << PROP_LOW;
	val |= (_5BIT_MASK & timing->ph1_nbt) << PH_1_LOW;
	val |= (_5BIT_MASK & timing->ph2_nbt) << PH_2_LOW;
	val |= (_4BIT_MASK & timing->prs_dbt) << PROP_FD_LOW;
	val |= (_4BIT_MASK & timing->ph1_dbt) << PH_1_FD_LOW;
	val |= (_4BIT_MASK & timing->ph2_dbt) << PH_2_FD_LOW;
	WRITE_VALUE_32BIT(node, TIMING_REG, val);

	val	 = (_6BIT_MASK & timing->presc_nbt) << BRP_LOW;
	val	|= (_4BIT_MASK & timing->sjw_nbt) << SJW_LOW;
	val	|= (_6BIT_MASK & timing->presc_dbt) << BRP_FD_LOW;
	val	|= (_4BIT_MASK & timing->sjw_dbt) << SJW_FD_LOW;
	WRITE_VALUE_32BIT(node, ARB_PRESC_REG, val);
}


STATUS get_bus_timing(can_node_t node, struct can_bus_timing_t *timing)
{
	if (timing == NULL)
		return INVALID_PTR;

	uint32_t val = READ_VALUE_32BIT(node, TIMING_REG);
	timing->prs_nbt = (uint8_t)(_6BIT_MASK & (val >> PROP_LOW));
	timing->ph1_nbt = (uint8_t)(_5BIT_MASK & (val >> PH_1_LOW));
	timing->ph2_nbt = (uint8_t)(_5BIT_MASK & (val >> PH_2_LOW));
	timing->prs_dbt = (uint8_t)(_4BIT_MASK & (val >> PROP_FD_LOW));
	timing->ph1_dbt = (uint8_t)(_4BIT_MASK & (val >> PH_1_FD_LOW));
	timing->ph2_dbt = (uint8_t)(_4BIT_MASK & (val >> PH_2_FD_LOW));

	val = READ_VALUE_32BIT(node, ARB_PRESC_REG);
	timing->presc_nbt = (uint8_t)(_6BIT_MASK & (val >> BRP_LOW));
	timing->sjw_nbt = (uint8_t)(_4BIT_MASK & (val >> SJW_LOW));
	timing->presc_dbt = (uint8_t)(_6BIT_MASK & (val >> BRP_FD_LOW));
	timing->sjw_dbt = (uint8_t)(_4BIT_MASK & (val >> SJW_FD_LOW));

	return SUCESS;
}


STATUS get_err_state(can_node_t node, struct can_error_state_struct_t *error_state)
{
	if (error_state == NULL)
		return INVALID_PTR;

	uint32_t val = READ_VALUE_32BIT(node, ERROR_TH_REG);
	error_state->ewl = (uint8_t)(_8BIT_MASK & (val >> EWL_LOW));
	error_state->erp = (uint8_t)(_8BIT_MASK& (val >> ERP_LOW));

	if (IS_BIT_SET(val, ERA_BIT))
		error_state->err_state = ERROR_ACTIVE;
	else if (IS_BIT_SET(val, ERP_BIT))
		error_state->err_state = ERROR_PASSIVE;
	else if (IS_BIT_SET(val, BOF_BIT))
		error_state->err_state = BUS_OFF;

	return SUCESS;
}


void set_err_th(can_node_t node, struct can_error_state_struct_t *err_state)
{
	uint32_t val;
	val  = ((uint32_t)err_state->ewl << EWL_LOW);
	val |= ((uint32_t)err_state->erp << ERP_LOW);
	WRITE_VALUE_32BIT(node, ERROR_TH_REG, val);
}


void set_error_ctrs(can_node_t node,struct can_error_ctr_t err_ctr, boolean set_tx,
						boolean set_rx, boolean erase_norm, boolean erase_fd)
{
	uint32_t val;

	if (set_tx) {
		val = err_ctr.tx_ctr << CTR_PRES_VAL_LOW;
		val = SET_BIT(val, PTX_BIT);
		WRITE_VALUE_32BIT(node, ERROR_COUNTERS_REG, val);
	}

	if (set_rx) {
		val = err_ctr.rx_ctr << CTR_PRES_VAL_LOW;
		val = SET_BIT(val, PRX_BIT);
		WRITE_VALUE_32BIT(node, ERROR_COUNTERS_REG, val);
	}

	val = 0x0;
	val = erase_norm ? SET_BIT(val, ENORM) : val;
	val = erase_fd ? SET_BIT(val, EFD) : val;
	if (val != 0)
		WRITE_VALUE_32BIT(node, ERROR_COUNTERS_SPEC_REG, val);
}


STATUS get_error_ctrs(can_node_t node, struct can_error_ctr_t *error_ctrs)
{
	if (error_ctrs == NULL)
		return INVALID_PTR;

	uint32_t val = READ_VALUE_32BIT(node, ERROR_COUNTERS_REG);
	error_ctrs->rx_ctr		= (uint16_t)(val >> RXC_VAL_LOW);
	error_ctrs->tx_ctr		= (uint16_t)(val >> TXC_VAL_LOW);

	val = READ_VALUE_32BIT(node, ERROR_COUNTERS_SPEC_REG);
	error_ctrs->norm_ctr	= (uint16_t)(val >> ERR_NORM_VAL_LOW);
	error_ctrs->data_ctr	= (uint16_t)(val >> ERR_FD_VAL_LOW);

	return SUCESS;
}


void set_bit_filter(can_node_t node, can_filter_t filter, struct can_bit_filter_set_t *settings)
{
	uint32_t mask = settings->mask;
	uint32_t val  = settings->value;
	uint32_t ctrl = (uint32_t)(_4BIT_MASK & settings->control);

	if (filter == FILTER_A){
		WRITE_VALUE_32BIT(node, FILTER_A_MASK_REG, mask);
		WRITE_VALUE_32BIT(node, FILTER_A_VAL_REG, val);
		WRITE_VALUE_32BIT(node, FILTER_CONTROL_REG, ctrl);
	}else if (filter == FILTER_B){
		WRITE_VALUE_32BIT(node, FILTER_B_MASK_REG, mask);
		WRITE_VALUE_32BIT(node, FILTER_B_VAL_REG, val);
		ctrl = ctrl << BASIC_FRAME_FILT_B_BIT;
		WRITE_VALUE_32BIT(node, FILTER_CONTROL_REG, ctrl);
	}else if (filter == FILTER_C){
		WRITE_VALUE_32BIT(node, FILTER_C_MASK_REG, mask);
		WRITE_VALUE_32BIT(node, FILTER_C_VAL_REG, val);
		ctrl = ctrl << BASIC_FRAME_FILT_C_BIT;
		WRITE_VALUE_32BIT(node, FILTER_CONTROL_REG, ctrl);
	}
}


void set_range_filter(can_node_t node, can_filter_t filter, struct can_range_filter_set_t *settings)
{
	uint32_t low  = settings->ran_low;
	uint32_t high = settings->ran_high;
	uint32_t ctrl = (((uint32_t)(_4BIT_MASK & settings->control)) << BASIC_FRAME_FILT_RAN_BIT);

	if (filter == FILTER_RAN){
		WRITE_VALUE_32BIT(node, FILTER_RAN_LOW_REG, low);
		WRITE_VALUE_32BIT(node, FILTER_RAN_HIGH_REG, high);
		WRITE_VALUE_32BIT(node, FILTER_CONTROL_REG, ctrl);
	}
}


STATUS get_rx_buf_status(can_node_t node, struct can_rxbuf_stat_t *status)
{
	if (status == NULL)
		return INVALID_PTR;

	uint32_t val 			= READ_VALUE_32BIT(node, RX_INFO_1_REG);
	status->empty 			= IS_BIT_SET(val,RX_EMPTY_BIT);
	status->full  			= IS_BIT_SET(val,RX_FULL_BIT);
	status->frames 			= _8BIT_MASK & (val >> RX_MC_VALUE_LOW);
	status->mem_free 		= _8BIT_MASK & (val >> RX_MF_VALUE_LOW);

	val 					= READ_VALUE_32BIT(node, RX_INFO_2_REG);
	status->buff_size 		= _8BIT_MASK & (val >> RX_BUFF_SIZE_LOW);
	status->write_pointer 	= _8BIT_MASK & (val >> RX_WPP_VALUE_LOW);
	status->read_pointer 	= _8BIT_MASK & (val >> RX_RPP_VALUE_LOW);

	return SUCESS;
}


uint8_t	get_byte_length(uint8_t dlc)
{
	uint8_t val = 0xF & dlc;

	if (val < 9){
		return val;
	}else{
		switch (val) {
		case 0x9	:
			return 12;
		case 0xA	:
			return 16;
		case 0xB	:
			return 20;
		case 0xC	:
			return 24;
		case 0xD	:
			return 32;
		case 0xE	:
			return 48;
		case 0xF	:
			return 64;
		default:
			return 0;
		}

	}
}


uint8_t	get_dlc(uint8_t data_length)
{
	if (data_length < 9){
		return data_length;
	}else if (data_length < 13){
		return 9;
	}else if (data_length < 17){
		return 10;
	}else if (data_length < 21){
		return 11;
	}else if (data_length < 25){
		return 12;
	}else if (data_length < 33){
		return 13;
	}else if (data_length < 49){
		return 14;
	}else{
		return 15;
	}
}


STATUS can_read_frame(can_node_t node,struct can_frame_t *frame)
{
	uint32_t val = READ_VALUE_32BIT(node, RX_DATA_REG);
	int wrd_data_len;
	int	i;

	if (frame == NULL || frame->data == NULL)
		return INVALID_PTR;

	frame->dlc 		= _4BIT_MASK & (val >> DLC_LOW);
	frame->rtr_flag 	= (IS_BIT_SET(val, RTR_BIT)) ? RTR_FRAME : NO_RTR;
	frame->ident_type	= (IS_BIT_SET(val, ID_TYPE_BIT)) ? EXTENDED : BASIC;
	frame->frame_format = (IS_BIT_SET(val, FR_TYPE_BIT)) ? FD : NORMAL;
	frame->brs_flag		= (IS_BIT_SET(val, BRS_BIT)) ? SHIFT : NO_SHIFT;
	frame->esi_flag 	= (IS_BIT_SET(val, ESI_BIT)) ? ERROR_PASSIVE : ERROR_ACTIVE;

	val = READ_VALUE_32BIT(node, RX_DATA_REG);
	frame->timestamp 	= (uint64_t)val << 32;
	val = READ_VALUE_32BIT(node, RX_DATA_REG);
	frame->timestamp  	|= val;

	val = READ_VALUE_32BIT(node, RX_DATA_REG);

	if (frame->ident_type == EXTENDED)
		frame->identifier 	= IDENT_REG_TO_UNS(val);
	else
		frame->identifier 	= _11BIT_MASK & val;

	frame->data_len = get_byte_length(frame->dlc);

	if ((frame->rtr_flag == NO_RTR || frame->frame_format == FD) & (frame->dlc != 0x0)){
		wrd_data_len = (frame->data_len-1)/4 + 1;
		for (i=0;i<wrd_data_len;i++){
			val = READ_VALUE_32BIT(node, RX_DATA_REG);

			//Memory in the IP Core is Big Endian
			//Since we dont know Endianess of the system we
			//have to store by byte not whole uint32_t at once!
			*(frame->data+4*i) 		= (uint8_t)(val >> 24);
			*(frame->data+4*i+1) 	= (uint8_t)(0xFF & (val >> 16));
			*(frame->data+4*i+2) 	= (uint8_t)(0xFF & (val >> 8));
			*(frame->data+4*i+3) 	= (uint8_t)(0xFF & val);
		}
	}

	return SUCESS;
}


STATUS can_send_frame(can_node_t node,struct can_frame_t *frame)
{
	uint32_t val = 0x0;
	int i;
	uint32_t dest;	//Buffer where to store the frame (TXT_1 or TXT_2)

	if (frame == NULL || frame->data == NULL)
		return INVALID_PTR;

	//Check status of the TXT Buffers
	val = READ_VALUE_32BIT(node, TX_STATUS_REG);
	if ((!IS_BIT_SET(val, TXT_1_EMPTY_BIT)) & (!IS_BIT_SET(val, TXT_2_EMPTY_BIT)))
		return NO_BUF_SPACE;
	else if (IS_BIT_SET(val, TXT_1_EMPTY_BIT)){
		dest = TXT_1_COMMIT_BIT;
	}else{
		dest = TXT_2_COMMIT_BIT;
	}

	//Send frame
	frame->dlc = get_dlc(frame->data_len);
	val  = (uint32_t)frame->dlc;
	val  = (frame->rtr_flag == RTR_FRAME) ? SET_BIT(val, RTR_BIT) : val;
	val  = (frame->brs_flag == SHIFT) ? SET_BIT(val ,BRS_BIT) : val;
	val  = (frame->frame_format == FD) ? SET_BIT(val, FR_TYPE_BIT) : val;
	val  = (frame->ident_type == EXTENDED) ? SET_BIT(val, ID_TYPE_BIT) : val;
	val  = SET_BIT(val, TBF_BIT);
	WRITE_VALUE_32BIT(node, TX_DATA_1_REG, val);

	val = (uint32_t)(frame->timestamp >> 32);
	WRITE_VALUE_32BIT(node, TX_DATA_2_REG, val);

	val = (uint32_t)frame->timestamp;
	WRITE_VALUE_32BIT(node, TX_DATA_3_REG, val);

	if (frame->ident_type == EXTENDED)
		val = IDENT_UNS_TO_REG(frame->identifier);
	else
		val = (uint32_t)frame->identifier;

	WRITE_VALUE_32BIT(node, TX_DATA_4_REG, val);

	int wrd_data_len = (frame->data_len-1)/4 + 1;
	for (i=0; i<wrd_data_len; i++){

		//Memory in the IP Core is Big Endian
		//Since we dont know Endianess of the system we
		//have to store by byte not whole uint32_t at once!
		val  = ((uint32_t)(*(frame->data+4*i))) << 24;
		val |= ((uint32_t)(*(frame->data+4*i+1))) << 16;
		val |= ((uint32_t)(*(frame->data+4*i+2))) << 8;
		val |= ((uint32_t)(*(frame->data+4*i+3)));
		WRITE_VALUE_32BIT(node, TX_DATA_5_REG + 4*i, val);
	}

	val  = READ_VALUE_32BIT(node, TX_SETTINGS_REG);
	val |= 1 << dest;
	WRITE_VALUE_32BIT(node, TX_SETTINGS_REG , val);

	return SUCESS;
}


uint16_t can_get_trv_delay(can_node_t node)
{
	uint32_t val = READ_VALUE_32BIT(node, TRV_DELAY_REG);
	return (uint16_t)(0xFFFF & TRV_DELAY_REG);
}


STATUS can_read_traffic_ctrs(can_node_t node, struct can_traffic_ctr_t *traf_ctr)
{
	if(traf_ctr == NULL)
		return INVALID_PTR;

	traf_ctr->rx_ctr = READ_VALUE_32BIT(node, RX_COUNTER_REG);
	traf_ctr->tx_ctr = READ_VALUE_32BIT(node, TX_COUNTER_REG);

	return SUCESS;
}


void set_trig_config_ind(can_node_t node, boolean enable, uint8_t trig_offset)
{
	uint32_t val = READ_VALUE_32BIT(node, LOG_TRIG_CONFIG_REG);
	val = enable ? SET_BIT(val, trig_offset) : CLEAR_BIT(val, trig_offset);
	WRITE_VALUE_32BIT(node, LOG_TRIG_CONFIG_REG, val);
}


void set_trig_config_all(can_node_t node, uint32_t trig_vector)
{
	WRITE_VALUE_32BIT(node, LOG_TRIG_CONFIG_REG, trig_vector);
}


void set_capt_config_ind(can_node_t node, boolean enable, uint8_t capt_offset)
{
	uint32_t val = READ_VALUE_32BIT(node, LOG_CAPT_CONFIG_REG);
	val = enable ? SET_BIT(val, capt_offset) : CLEAR_BIT(val, capt_offset);
	WRITE_VALUE_32BIT(node, LOG_CAPT_CONFIG_REG, val);
}


void set_capt_config_all(can_node_t node, uint32_t capt_vector)
{
	WRITE_VALUE_32BIT(node, LOG_CAPT_CONFIG_REG, capt_vector);
}


void give_log_command(can_node_t node, can_log_command_t command)
{
	uint32_t val = 0x0;
	switch (command){
	case START:
		val = SET_BIT(val, LOG_STR_BIT);
		break;
	case ABORT:
		val = SET_BIT(val, LOG_ABT_BIT);
		break;
	case RPP_UP:
		val = SET_BIT(val, LOG_UP_BIT);
		break;
	case RPP_DOWN:
		val = SET_BIT(val, LOG_DOWN_BIT);
		break;
	}

	WRITE_VALUE_32BIT(node, LOG_COMMAND_REG, val);
}


STATUS get_logger_status(can_node_t node, struct can_log_info_t *status)
{
	if (status == NULL)
		return INVALID_PTR;

	uint32_t val = READ_VALUE_32BIT(node, LOG_STATUS_REG);
	status->log_exist = IS_BIT_SET(val, LOG_EXIST_BIT);

	if (IS_BIT_SET(val, LOG_CFG_BIT)){
		status->state = CONFIG;
	}else if (IS_BIT_SET(val, LOG_RDY_BIT)){
		status->state = READY;
	}else if (IS_BIT_SET(val, LOG_RUN_BIT)){
		status->state = RUNNING;
	}

	status->size 			= (uint8_t)(0xFF & (val >> LOG_SIZE_LOW));
	status->write_pointer 	= (uint8_t)(0xFF & (val >> LOG_WPP_LOW));
	status->read_pointer	= (uint8_t)(0xFF & (val >> LOG_RPP_LOW));

	return SUCESS;
}



