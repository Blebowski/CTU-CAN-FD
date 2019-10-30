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

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>
#include <linux/pm_runtime.h>

#include "ctu_can_fd_hw.h"
#include "ctu_can_fd_regs.h"

#define DRIVER_NAME	"ctucanfd"

static bool use_msi = 1;
module_param(use_msi, bool, 0444);
MODULE_PARM_DESC(use_msi, "PCIe implementation use MSI interrupts. Default: 1 (yes)");

static bool pci_use_second = 1;
module_param(pci_use_second, bool, 0444);
MODULE_PARM_DESC(pci_use_second, "Use the second CAN core on PCIe card. Default: 1 (yes)");

/* TX buffer rotation:
 * - when a buffer transitions to empty state, rotate order and priorities
 * - if more buffers seem to transition at the same time, rotate
 *   by the number of buffers
 * - it may be assumed that buffers transition to empty state in FIFO order
 *   (because we manage priorities that way)
 * - at frame filling, do not rotate anything, just increment buffer modulo
 *   counter
 */

struct ctucan_priv {
	struct can_priv can; // must be first member!
	struct ctucan_hw_priv p;

	unsigned int txb_head;
	unsigned int txb_tail;
	u32 txb_prio;
	unsigned int txb_mask;
	spinlock_t tx_lock;

	struct napi_struct napi;
	struct device *dev;
	struct clk *can_clk;

	int irq_flags;
	unsigned long drv_flags;

	union ctu_can_fd_frame_form_w rxfrm_first_word;

	struct list_head peers_on_pdev;
};

#define CTUCAN_FLAG_RX_FFW_BUFFERED	1

static int ctucan_reset(struct net_device *ndev)
{
	int i;
	struct ctucan_priv *priv = netdev_priv(ndev);

	netdev_dbg(ndev, "ctucan_reset");

	ctucan_hw_reset(&priv->p);
	for (i = 0; i < 100; ++i) {
		if (ctucan_hw_check_access(&priv->p))
			return 0;
		usleep_range(100, 200);
	}

	netdev_warn(ndev, "device did not leave reset\n");
	return -ETIMEDOUT;
}

/**
 * ctucan_set_bittiming - CAN set bit timing routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the driver set bittiming routine.
 * Return: 0 on success and failure value on error
 */
static int ctucan_set_bittiming(struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bt = &priv->can.bittiming;

	netdev_dbg(ndev, "ctucan_set_bittiming");

	if (ctucan_hw_is_enabled(&priv->p)) {
		netdev_alert(ndev,
			     "BUG! Cannot set bittiming - CAN is enabled\n");
		return -EPERM;
	}

	/* Note that bt may be modified here */
	ctucan_hw_set_nom_bittiming(&priv->p, bt);

	return 0;
}

/**
 * ctucan_set_data_bittiming - CAN set data bit timing routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the driver set data bittiming routine.
 * Return: 0 on success and failure value on error
 */
static int ctucan_set_data_bittiming(struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	struct can_bittiming *dbt = &priv->can.data_bittiming;

	netdev_dbg(ndev, "ctucan_set_data_bittiming");

	if (ctucan_hw_is_enabled(&priv->p)) {
		netdev_alert(ndev,
			     "BUG! Cannot set bittiming - CAN is enabled\n");
		return -EPERM;
	}

	/* Note that dbt may be modified here */
	ctucan_hw_set_data_bittiming(&priv->p, dbt);

	return 0;
}

/**
 * ctucan_chip_start - This the drivers start routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the drivers start routine.
 *
 * Return: 0 on success and failure value on error
 */
static int ctucan_chip_start(struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	union ctu_can_fd_int_stat int_ena, int_msk, int_enamask_mask;
	int err;
	struct can_ctrlmode mode;

	netdev_dbg(ndev, "ctucan_chip_start");

	err = ctucan_reset(ndev);
	if (err < 0)
		return err;

	priv->txb_prio = 0x01234567;
	priv->txb_head = 0;
	priv->txb_tail = 0;
	priv->p.write_reg(&priv->p, CTU_CAN_FD_TX_PRIORITY, priv->txb_prio);

	err = ctucan_set_bittiming(ndev);
	if (err < 0)
		return err;

	err = ctucan_set_data_bittiming(ndev);
	if (err < 0)
		return err;

	/* Enable interrupts */
	int_ena.u32 = 0;
	int_ena.s.rbnei = 1;
	int_ena.s.txbhci = 1;

	int_ena.s.ewli = 1;
	int_ena.s.fcsi = 1;

	int_enamask_mask.u32 = 0xFFFFFFFF;

	mode.flags = priv->can.ctrlmode;
	mode.mask = 0xFFFFFFFF;
	ctucan_hw_set_mode_reg(&priv->p, &mode);

	/* One shot mode supported indirectly via Retransmit limit */
	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		ctucan_hw_set_ret_limit(&priv->p, true, 0);

	/* Bus error reporting -> Allow Error interrupt */
	if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) {
		int_ena.s.ali = 1;
		int_ena.s.bei = 1;
	}

	int_msk.u32 = ~int_ena.u32; /* mask all disabled interrupts */

	/* It's after reset, so there is no need to clear anything */
	ctucan_hw_int_mask_set(&priv->p, int_msk);
	ctucan_hw_int_ena_set(&priv->p, int_ena);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* Enable the controller */
	ctucan_hw_enable(&priv->p, true);

	return 0;
}

/**
 * ctucan_do_set_mode - This sets the mode of the driver
 * @ndev:	Pointer to net_device structure
 * @mode:	Tells the mode of the driver
 *
 * This check the drivers state and calls the
 * the corresponding modes to set.
 *
 * Return: 0 on success and failure value on error
 */
static int ctucan_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	int ret;

	netdev_dbg(ndev, "ctucan_do_set_mode");

	switch (mode) {
	case CAN_MODE_START:
		ret = ctucan_chip_start(ndev);
		if (ret < 0) {
			netdev_err(ndev, "ctucan_chip_start failed!\n");
			return ret;
		}
		netif_wake_queue(ndev);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

/**
 * ctucan_start_xmit - Starts the transmission
 * @skb:	sk_buff pointer that contains data to be Txed
 * @ndev:	Pointer to net_device structure
 *
 * This function is invoked from upper layers to initiate transmission. This
 * function uses the next available free txbuf and populates their fields to
 * start the transmission.
 *
 * Return: 0 on success and failure value on error
 */
static int ctucan_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	u32 txb_id;
	bool ok;
	unsigned long flags;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	/* Check if the TX buffer is full */
	if (unlikely(!CTU_CAN_FD_TXTNF(ctu_can_get_status(&priv->p)))) {
		netif_stop_queue(ndev);
		netdev_err(ndev, "BUG!, no TXB free when queue awake!\n");
		return NETDEV_TX_BUSY;
	}

	txb_id = priv->txb_head & priv->txb_mask;
	netdev_dbg(ndev, "%s: using TXB#%u", __func__, txb_id);
	ok = ctucan_hw_insert_frame(&priv->p, cf, 0, txb_id,
				     can_is_canfd_skb(skb));

	if (!ok) {
		netdev_err(ndev,
            "BUG! TXNF set but cannot insert frame into TXTB! HW Bug?");
		return NETDEV_TX_BUSY;
	}
	can_put_echo_skb(skb, ndev, txb_id);

	if (!(cf->can_id & CAN_RTR_FLAG))
		stats->tx_bytes += cf->len;

	spin_lock_irqsave(&priv->tx_lock, flags);

	ctucan_hw_txt_set_rdy(&priv->p, txb_id);

	priv->txb_head++;

	/* Check if all TX buffers are full */
	if (!CTU_CAN_FD_TXTNF(ctu_can_get_status(&priv->p)))
		netif_stop_queue(ndev);

	spin_unlock_irqrestore(&priv->tx_lock, flags);

	return NETDEV_TX_OK;
}

/**
 * xcan_rx -  Is called from CAN isr to complete the received
 *		frame  processing
 * @ndev:	Pointer to net_device structure
 *
 * This function is invoked from the CAN isr(poll) to process the Rx frames. It
 * does minimal processing and invokes "netif_receive_skb" to complete further
 * processing.
 * Return: 1 on success and 0 on failure.
 */
static int ctucan_rx(struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u64 ts;
	union ctu_can_fd_frame_form_w ffw;

	if (test_bit(CTUCAN_FLAG_RX_FFW_BUFFERED, &priv->drv_flags)) {
		ffw = priv->rxfrm_first_word;
		clear_bit(CTUCAN_FLAG_RX_FFW_BUFFERED, &priv->drv_flags);
	} else {
		ffw = ctu_can_fd_read_rx_ffw(&priv->p);
	}

	if (ffw.s.fdf == FD_CAN)
		skb = alloc_canfd_skb(ndev, &cf);
	else
		skb = alloc_can_skb(ndev, (struct can_frame **)&cf);

	if (unlikely(!skb)) {
		priv->rxfrm_first_word = ffw;
		set_bit(CTUCAN_FLAG_RX_FFW_BUFFERED, &priv->drv_flags);
		return 0;
	}

	ctucan_hw_read_rx_frame_ffw(&priv->p, cf, &ts, ffw);

	stats->rx_bytes += cf->len;
	stats->rx_packets++;
	netif_receive_skb(skb);

	return 1;
}

/**
 * ctucan_err_interrupt - error frame Isr
 * @ndev:	net_device pointer
 * @isr:	interrupt status register value
 *
 * This is the CAN error interrupt and it will
 * check the the type of error and forward the error
 * frame to upper layers.
 */
static void ctucan_err_interrupt(struct net_device *ndev,
				 union ctu_can_fd_int_stat isr)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct can_berr_counter berr;
	union ctu_can_fd_err_capt_alc err_capt_alc;
	int dologerr = net_ratelimit();

	ctucan_hw_read_err_ctrs(&priv->p, &berr);

	err_capt_alc = ctu_can_fd_read_err_capt_alc(&priv->p);

	if (dologerr)
		netdev_info(ndev, "%s: ISR = 0x%08x, rxerr %d, txerr %d,"
			" error type %u, pos %u, ALC id_field %u, bit %u\n",
			__func__, isr.u32, berr.rxerr, berr.txerr,
			err_capt_alc.s.err_type, err_capt_alc.s.err_pos,
			err_capt_alc.s.alc_id_field, err_capt_alc.s.alc_bit);

	skb = alloc_can_err_skb(ndev, &cf);

	/* EWI:  error warning
	 * FCSI: Fault confinement State changed
	 * ALI:  arbitration lost (just informative)
	 * BEI:  bus error interrupt
	 */
	if (isr.s.fcsi) {
		/* error passive or bus off */
		enum can_state state = ctucan_hw_read_error_state(&priv->p);

		netdev_info(ndev, "  Fault conf: state = %u", state);
		priv->can.state = state;
		if (state == CAN_STATE_BUS_OFF) {
			priv->can.can_stats.bus_off++;
			netdev_info(ndev, "    bus_off");
			can_bus_off(ndev);
			if (skb)
				cf->can_id |= CAN_ERR_BUSOFF;
		} else if (state == CAN_STATE_ERROR_PASSIVE) {
			priv->can.can_stats.error_passive++;
			netdev_info(ndev, "    error_passive");
			if (skb) {
				cf->can_id |= CAN_ERR_CRTL;
				cf->data[1] = (berr.rxerr > 127) ?
						CAN_ERR_CRTL_RX_PASSIVE :
						CAN_ERR_CRTL_TX_PASSIVE;
				cf->data[6] = berr.txerr;
				cf->data[7] = berr.rxerr;
			}
		} else if (state == CAN_STATE_ERROR_WARNING) {
			netdev_warn(ndev, "    error_warning, but ISR[FCSI] was set! (HW bug?)");
			goto err_warning;
		} else if (state == CAN_STATE_ERROR_ACTIVE) {
			netdev_info(ndev, "    reached error active state");
			cf->data[1] = CAN_ERR_CRTL_ACTIVE;
			cf->data[6] = berr.txerr;
			cf->data[7] = berr.rxerr;
		} else {
			netdev_warn(ndev, "    unhandled error state (%d)!",
				    state);
		}
	} else if (isr.s.ewli) {
err_warning:
		/* error warning */
		priv->can.state = CAN_STATE_ERROR_WARNING;
		priv->can.can_stats.error_warning++;
		if (dologerr)
			netdev_info(ndev, "  error_warning");
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= (berr.txerr > berr.rxerr) ?
					CAN_ERR_CRTL_TX_WARNING :
					CAN_ERR_CRTL_RX_WARNING;
			cf->data[6] = berr.txerr;
			cf->data[7] = berr.rxerr;
		}
	}

	/* Check for Arbitration Lost interrupt */
	if (isr.s.ali) {
		if (dologerr)
			netdev_info(ndev, "  arbitration lost");
		priv->can.can_stats.arbitration_lost++;
		if (skb) {
			cf->can_id |= CAN_ERR_LOSTARB;
			cf->data[0] = CAN_ERR_LOSTARB_UNSPEC;
		}
	}

	/* Check for Bus Error interrupt */
	if (isr.s.bei) {
		netdev_info(ndev, "  bus error");
		priv->can.can_stats.bus_error++;
		stats->tx_errors++; // TODO: really?
		if (skb) {
			cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
			cf->data[2] = CAN_ERR_PROT_UNSPEC;
			cf->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
		}
	}

	if (skb) {
		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);
	}
}

/**
 * ctucan_rx_poll - Poll routine for rx packets (NAPI)
 * @napi:	napi structure pointer
 * @quota:	Max number of rx packets to be processed.
 *
 * This is the poll routine for rx part.
 * It will process the packets maximux quota value.
 *
 * Return: number of packets received
 */
static int ctucan_rx_poll(struct napi_struct *napi, int quota)
{
	struct net_device *ndev = napi->dev;
	struct ctucan_priv *priv = netdev_priv(ndev);
	int work_done = 0;
	union ctu_can_fd_status status;
	u32 framecnt;

	framecnt = ctucan_hw_get_rx_frame_count(&priv->p);
	netdev_dbg(ndev, "rx_poll: %d frames in RX FIFO", framecnt);
	while (framecnt && work_done < quota) {
		ctucan_rx(ndev);
		work_done++;
		framecnt = ctucan_hw_get_rx_frame_count(&priv->p);
	}

	/* Check for RX FIFO Overflow */
	status = ctu_can_get_status(&priv->p);
	if (status.s.dor) {
		struct net_device_stats *stats = &ndev->stats;
		struct can_frame *cf;
		struct sk_buff *skb;

		netdev_info(ndev, "  rx fifo overflow");
		stats->rx_over_errors++;
		stats->rx_errors++;
		skb = alloc_can_err_skb(ndev, &cf);
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
			stats->rx_packets++;
			stats->rx_bytes += cf->can_dlc;
			netif_rx(skb);
		}

		/* Clear Data Overrun */
		ctucan_hw_clr_overrun_flag(&priv->p);
	}

	if (work_done)
		can_led_event(ndev, CAN_LED_EVENT_RX);

	if (!framecnt) {
		if (napi_complete_done(napi, work_done)) {
			union ctu_can_fd_int_stat iec;
			/* Clear and enable RBNEI. It is level-triggered, so
			 * there is no race condition.
			 */
			iec.u32 = 0;
			iec.s.rbnei = 1;
			ctucan_hw_int_clr(&priv->p, iec);
			ctucan_hw_int_mask_clr(&priv->p, iec);
		}
	}

	return work_done;
}

static void ctucan_rotate_txb_prio(struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	u32 prio = priv->txb_prio;
	u32 nbuffersm1 = priv->txb_mask; /* nbuffers - 1 */

	prio = (prio << 4) | ((prio >> (nbuffersm1 * 4)) & 0xF);
	netdev_dbg(ndev, "%s: from 0x%08x to 0x%08x",
		   __func__, priv->txb_prio, prio);
	priv->txb_prio = prio;
	priv->p.write_reg(&priv->p, CTU_CAN_FD_TX_PRIORITY, prio);
}

/**
 * xcan_tx_interrupt - Tx Done Isr
 * @ndev:	net_device pointer
 */
static void ctucan_tx_interrupt(struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	bool first = true;
	union ctu_can_fd_int_stat icr;
	bool some_buffers_processed;
	unsigned long flags;

	netdev_dbg(ndev, "%s", __func__);

	/*  read tx_status
	 *  if txb[n].finished (bit 2)
	 *	if ok -> echo
	 *	if error / aborted -> ?? (find how to handle oneshot mode)
	 *	txb_tail++
	 */

	icr.u32 = 0;
	icr.s.txbhci = 1;
	do {
		spin_lock_irqsave(&priv->tx_lock, flags);

		some_buffers_processed = false;
		while ((int)(priv->txb_head - priv->txb_tail) > 0) {
			u32 txb_idx = priv->txb_tail & priv->txb_mask;
			u32 status = ctucan_hw_get_tx_status(&priv->p, txb_idx);

			netdev_dbg(ndev, "TXI: TXB#%u: status 0x%x",
				   txb_idx, status);

			switch (status) {
			case TXT_TOK:
				netdev_dbg(ndev, "TXT_OK");
				can_get_echo_skb(ndev, txb_idx);
				stats->tx_packets++;
				break;
			case TXT_ERR:
				/* This indicated that retransmit limit has been
				 * reached. Obviously we should not echo the
				 * frame, but also not indicate any kind
				 * of error. If desired, it was already reported
				 * (possible multiple times) on each arbitration
				 * lost.
				 */
				netdev_warn(ndev, "TXB in Error state");
				can_free_echo_skb(ndev, txb_idx);
				stats->tx_dropped++;
				break;
			case TXT_ABT:
				/* Same as for TXT_ERR, only with different
				 * cause. We *could* re-queue the frame, but
				 * multiqueue/abort is not supported yet anyway.
				 */
				netdev_warn(ndev, "TXB in Aborted state");
				can_free_echo_skb(ndev, txb_idx);
				stats->tx_dropped++;
				break;
			default:
				/* Bug only if the first buffer is not finished,
				 * otherwise it is pretty much expected
				 */
				if (first) {
					netdev_err(ndev, "BUG: TXB#%u not in a finished state (0x%x)!",
						   txb_idx, status);
					spin_unlock_irqrestore(&priv->tx_lock, flags);
					/* do not clear nor wake */
					return;
				}
				goto clear;
			}
			priv->txb_tail++;
			first = false;
			some_buffers_processed = true;
			/* Adjust priorities *before* marking the buffer
			 * as empty.
			 */
			ctucan_rotate_txb_prio(ndev);
			ctucan_hw_txt_set_empty(&priv->p, txb_idx);
		}
clear:
		spin_unlock_irqrestore(&priv->tx_lock, flags);

		/* If no buffers were processed this time, wa cannot
		 * clear - that would introduce a race condition.
		 */
		if (some_buffers_processed) {
			/* Clear the interrupt again as not to receive it again
			 * for a buffer we already handled (possibly causing
			 * the bug log)
			 */
			ctucan_hw_int_clr(&priv->p, icr);
		}
	} while (some_buffers_processed);

	can_led_event(ndev, CAN_LED_EVENT_TX);

	spin_lock_irqsave(&priv->tx_lock, flags);

	/* Check if at least one TX buffer is free */
	if (CTU_CAN_FD_TXTNF(ctu_can_get_status(&priv->p)))
		netif_wake_queue(ndev);

	spin_unlock_irqrestore(&priv->tx_lock, flags);
}

/**
 * xcan_interrupt - CAN Isr
 * @irq:	irq number
 * @dev_id:	device id poniter
 *
 * This is the CTU CAN FD ISR. It checks for the type of interrupt
 * and invokes the corresponding ISR.
 *
 * Return:
 * IRQ_NONE - If CAN device is in sleep mode, IRQ_HANDLED otherwise
 */
static irqreturn_t ctucan_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct ctucan_priv *priv = netdev_priv(ndev);
	union ctu_can_fd_int_stat isr, icr;
	int irq_loops = 0;

	netdev_dbg(ndev, "ctucan_interrupt");

	do {
		/* Get the interrupt status */
		isr = ctu_can_fd_int_sts(&priv->p);

		if (!isr.u32)
			return irq_loops ? IRQ_HANDLED : IRQ_NONE;

		/* Receive Buffer Not Empty Interrupt */
		if (isr.s.rbnei) {
			netdev_dbg(ndev, "RXBNEI");
			icr.u32 = 0;
			icr.s.rbnei = 1;
			/* Mask RXBNEI the first then clear interrupt,
			 * then schedule NAPI. Even if another IRQ fires,
			 * isr.s.rbnei will always be 0 (masked).
			 */
			ctucan_hw_int_mask_set(&priv->p, icr);
			ctucan_hw_int_clr(&priv->p, icr);
			napi_schedule(&priv->napi);
		}

		/* TX Buffer HW Command Interrupt */
		if (isr.s.txbhci) {
			netdev_dbg(ndev, "TXBHCI");
			/* Cleared inside */
			ctucan_tx_interrupt(ndev);
		}

		/* Error interrupts */
		if (isr.s.ewli || isr.s.fcsi || isr.s.ali) {
			union ctu_can_fd_int_stat ierrmask = { .s = {
				  .ewli = 1, .fcsi = 1, .ali = 1, .bei = 1 } };
			icr.u32 = isr.u32 & ierrmask.u32;

			netdev_dbg(ndev, "some ERR interrupt: clearing 0x%08x",
				   icr.u32);
			ctucan_hw_int_clr(&priv->p, icr);
			ctucan_err_interrupt(ndev, isr);
		}
		/* Ignore RI, TI, LFI, RFI, BSI */
	} while (irq_loops++ < 10000);

	netdev_err(ndev, "%s: stuck interrupt (isr=0x%08x), stopping\n",
		   __func__, isr.u32);

	if (isr.s.txbhci) {
		int i;

		netdev_err(ndev, "txb_head=0x%08x txb_tail=0x%08x\n",
			priv->txb_head, priv->txb_tail);
		for (i = 0; i <= priv->txb_mask; i++) {
			u32 status = ctucan_hw_get_tx_status(&priv->p, i);

			netdev_err(ndev, "txb[%d] txb status=0x%08x\n",
				i, status);
		}
	}

	{
		union ctu_can_fd_int_stat imask;

		imask.u32 = 0xffffffff;
		ctucan_hw_int_ena_clr(&priv->p, imask);
		ctucan_hw_int_mask_set(&priv->p, imask);
	}

	return IRQ_HANDLED;
}

/**
 * ctucan_chip_stop - Driver stop routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the drivers stop routine. It will disable the
 * interrupts and disable the controller.
 */
static void ctucan_chip_stop(struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	union ctu_can_fd_int_stat mask;

	netdev_dbg(ndev, "ctucan_chip_stop");

	mask.u32 = 0xffffffff;

	/* Disable interrupts and disable can */
	ctucan_hw_int_mask_set(&priv->p, mask);
	ctucan_hw_enable(&priv->p, false);
	priv->can.state = CAN_STATE_STOPPED;
}

/**
 * ctucan_open - Driver open routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the driver open routine.
 * Return: 0 on success and failure value on error
 */
static int ctucan_open(struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	int ret;

	netdev_dbg(ndev, "ctucan_open");

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		netdev_err(ndev, "%s: pm_runtime_get failed(%d)\n",
			   __func__, ret);
		return ret;
	}

	ret = request_irq(ndev->irq, ctucan_interrupt, priv->irq_flags,
			  ndev->name, ndev);
	if (ret < 0) {
		netdev_err(ndev, "irq allocation for CAN failed\n");
		goto err;
	}

	/* Common open */
	ret = open_candev(ndev);
	if (ret) {
		netdev_warn(ndev, "open_candev failed!\n");
		goto err_irq;
	}

	ret = ctucan_chip_start(ndev);
	if (ret < 0) {
		netdev_err(ndev, "ctucan_chip_start failed!\n");
		goto err_candev;
	}

	netdev_info(ndev, "ctu_can_fd device registered");
	can_led_event(ndev, CAN_LED_EVENT_OPEN);
	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	return 0;

err_candev:
	close_candev(ndev);
err_irq:
	free_irq(ndev->irq, ndev);
err:
	pm_runtime_put(priv->dev);

	return ret;
}

/**
 * ctucan_close - Driver close routine
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0 always
 */
static int ctucan_close(struct net_device *ndev)
{
	struct ctucan_priv *priv = netdev_priv(ndev);

	netdev_dbg(ndev, "ctucan_close");

	netif_stop_queue(ndev);
	napi_disable(&priv->napi);
	ctucan_chip_stop(ndev);
	free_irq(ndev->irq, ndev);
	close_candev(ndev);

	can_led_event(ndev, CAN_LED_EVENT_STOP);
	pm_runtime_put(priv->dev);

	return 0;
}

/**
 * ctucan_get_berr_counter - error counter routine
 * @ndev:	Pointer to net_device structure
 * @bec:	Pointer to can_berr_counter structure
 *
 * This is the driver error counter routine.
 * Return: 0 on success and failure value on error
 */
static int ctucan_get_berr_counter(const struct net_device *ndev,
				   struct can_berr_counter *bec)
{
	struct ctucan_priv *priv = netdev_priv(ndev);
	int ret;

	netdev_dbg(ndev, "ctucan_get_berr_counter");

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		netdev_err(ndev, "%s: pm_runtime_get failed(%d)\n",
			   __func__, ret);
		return ret;
	}

	ctucan_hw_read_err_ctrs(&priv->p, bec);

	pm_runtime_put(priv->dev);

	return 0;
}

static const struct net_device_ops ctucan_netdev_ops = {
	.ndo_open	= ctucan_open,
	.ndo_stop	= ctucan_close,
	.ndo_start_xmit	= ctucan_start_xmit,
	.ndo_change_mtu	= can_change_mtu,
};

static __maybe_unused int ctucan_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct ctucan_priv *priv = netdev_priv(ndev);

	netdev_dbg(ndev, "ctucan_suspend");

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
	}

	priv->can.state = CAN_STATE_SLEEPING;

	return 0;
}

static __maybe_unused int ctucan_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct ctucan_priv *priv = netdev_priv(ndev);

	netdev_dbg(ndev, "ctucan_resume");

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	if (netif_running(ndev)) {
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}

static const struct dev_pm_ops ctucan_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ctucan_suspend, ctucan_resume)
};

/**
 * ctucan_probe_common - Device type independent registration call
 *
 * This function does all the memory allocation and registration for the CAN
 * device.
 *
 * @dev:	Handle to the generic device structure
 * @addr:	Base address of CTU CAN FD core address
 * @irq:	Interrupt number
 * @ntxbufs:	Number of implemented Tx buffers
 * @can_clk_rate: Clock rate, if 0 then clock are taken from device node
 * @pm_enable_call: Whether pm_runtime_enable should be called
 * @set_drvdata_fnc: Function to set network driver data for physical device
 *
 * Return: 0 on success and failure value on error
 */
static int ctucan_probe_common(struct device *dev, void __iomem *addr,
		int irq, unsigned int ntxbufs, unsigned long can_clk_rate,
		int pm_enable_call, void (*set_drvdata_fnc)(struct device *dev,
		struct net_device *ndev))
{
	struct ctucan_priv *priv;
	struct net_device *ndev;
	int ret;

	/* Create a CAN device instance */
	ndev = alloc_candev(sizeof(struct ctucan_priv), ntxbufs);
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	spin_lock_init(&priv->tx_lock);
	INIT_LIST_HEAD(&priv->peers_on_pdev);
	priv->txb_mask = ntxbufs - 1;
	priv->dev = dev;
	priv->can.bittiming_const = &ctu_can_fd_bit_timing_max;
	priv->can.data_bittiming_const = &ctu_can_fd_bit_timing_data_max;
	priv->can.do_set_mode = ctucan_do_set_mode;

	/* Needed for timing adjustment to be performed as soon as possible */
	priv->can.do_set_bittiming = ctucan_set_bittiming;
	priv->can.do_set_data_bittiming = ctucan_set_data_bittiming;

	priv->can.do_get_berr_counter = ctucan_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK
					| CAN_CTRLMODE_LISTENONLY
					| CAN_CTRLMODE_FD
					| CAN_CTRLMODE_PRESUME_ACK
					| CAN_CTRLMODE_BERR_REPORTING
					| CAN_CTRLMODE_FD_NON_ISO
					| CAN_CTRLMODE_ONE_SHOT;
	priv->p.mem_base = addr;

	/* Get IRQ for the device */
	ndev->irq = irq;
	ndev->flags |= IFF_ECHO;	/* We support local echo */

	if (set_drvdata_fnc != NULL)
		set_drvdata_fnc(dev, ndev);
	SET_NETDEV_DEV(ndev, dev);
	ndev->netdev_ops = &ctucan_netdev_ops;

	/* Getting the CAN can_clk info */
	if (can_clk_rate == 0) {
		priv->can_clk = devm_clk_get(dev, NULL);
		if (IS_ERR(priv->can_clk)) {
			dev_err(dev, "Device clock not found.\n");
			ret = PTR_ERR(priv->can_clk);
			goto err_free;
		}
		can_clk_rate = clk_get_rate(priv->can_clk);
	}

	priv->p.write_reg = ctucan_hw_write32;
	priv->p.read_reg = ctucan_hw_read32;

	if (pm_enable_call)
		pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		netdev_err(ndev, "%s: pm_runtime_get failed(%d)\n",
			   __func__, ret);
		goto err_pmdisable;
	}

	if ((priv->p.read_reg(&priv->p, CTU_CAN_FD_DEVICE_ID) &
			    0xFFFF) != CTU_CAN_FD_ID) {
		priv->p.write_reg = ctucan_hw_write32_be;
		priv->p.read_reg = ctucan_hw_read32_be;
		if ((priv->p.read_reg(&priv->p, CTU_CAN_FD_DEVICE_ID) &
			      0xFFFF) != CTU_CAN_FD_ID) {
			netdev_err(ndev, "CTU_CAN_FD signature not found\n");
			goto err_disableclks;
		}
	}

	ret = ctucan_reset(ndev);
	if (ret < 0)
		goto err_pmdisable;

	priv->can.clock.freq = can_clk_rate;

	netif_napi_add(ndev, &priv->napi, ctucan_rx_poll, NAPI_POLL_WEIGHT);

	ret = register_candev(ndev);
	if (ret) {
		dev_err(dev, "fail to register failed (err=%d)\n", ret);
		goto err_disableclks;
	}

	devm_can_led_init(ndev);

	pm_runtime_put(dev);

	netdev_dbg(ndev, "mem_base=0x%p irq=%d clock=%d, txb mask:%d\n",
		   priv->p.mem_base, ndev->irq, priv->can.clock.freq,
		   priv->txb_mask);

	return 0;

err_disableclks:
	pm_runtime_put(priv->dev);
err_pmdisable:
	if (pm_enable_call)
		pm_runtime_disable(dev);
err_free:
	list_del_init(&priv->peers_on_pdev);
	free_candev(ndev);
	return ret;
}

#ifdef CONFIG_OF

static void ctucan_platform_set_drvdata(struct device *dev,
					struct net_device *ndev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device,
						    dev);

	platform_set_drvdata(pdev, ndev);
}

/**
 * ctucan_platform_probe - Platform registration call
 * @pdev:	Handle to the platform device structure
 *
 * This function does all the memory allocation and registration for the CAN
 * device.
 *
 * Return: 0 on success and failure value on error
 */
static int ctucan_platform_probe(struct platform_device *pdev)
{
	struct resource *res; /* IO mem resources */
	struct device	*dev = &pdev->dev;
	void __iomem *addr;
	int ret;
	unsigned int ntxbufs;
	int irq;

	/* Get the virtual base address for the device */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(addr)) {
		dev_err(dev, "Cannot remap address.\n");
		ret = PTR_ERR(addr);
		goto err;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "Cannot find interrupt.\n");
		ret = irq;
		goto err;
	}

#if 0
	/* tntxbufs should be used in future */
	ret = of_property_read_u32(pdev->dev.of_node, "tntxbufs", &ntxbufs);
	if (ret < 0)
		goto err;
#endif

	ntxbufs = 4;

	ret = ctucan_probe_common(dev, addr, irq, ntxbufs, 0,
				  1, ctucan_platform_set_drvdata);

	if (ret < 0)
		platform_set_drvdata(pdev, NULL);
err:
	return ret;
}

/**
 * ctucan_platform_remove - Unregister the device after releasing the resources
 * @pdev:	Handle to the platform device structure
 *
 * This function frees all the resources allocated to the device.
 * Return: 0 always
 */
static int ctucan_platform_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct ctucan_priv *priv = netdev_priv(ndev);

	netdev_dbg(ndev, "ctucan_remove");

	unregister_candev(ndev);
	pm_runtime_disable(&pdev->dev);
	netif_napi_del(&priv->napi);
	free_candev(ndev);

	return 0;
}

/* Match table for OF platform binding */
static const struct of_device_id ctucan_of_match[] = {
	{ .compatible = "ctu,canfd-2", },
	{ .compatible = "ctu,ctucanfd", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, ctucan_of_match);

static struct platform_driver ctucanfd_driver = {
	.probe	= ctucan_platform_probe,
	.remove	= ctucan_platform_remove,
	.driver	= {
		.name = DRIVER_NAME,
		.pm = &ctucan_dev_pm_ops,
		.of_match_table	= ctucan_of_match,
	},
};

module_platform_driver(ctucanfd_driver);

#endif /*CONFIG_OF*/

#ifdef CONFIG_PCI

#include <linux/pci.h>

#ifndef PCI_DEVICE_DATA
#define PCI_DEVICE_DATA(vend, dev, data) \
	.vendor = PCI_VENDOR_ID_##vend, \
	.device = PCI_DEVICE_ID_##vend##_##dev, \
	.subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID, 0, 0, \
	.driver_data = (kernel_ulong_t)(data)
#endif
#ifndef PCI_VENDOR_ID_TEDIA
#define PCI_VENDOR_ID_TEDIA 0x1760
#endif

#define PCI_DEVICE_ID_ALTERA_CTUCAN_TEST  0xCAFD
#define PCI_DEVICE_ID_TEDIA_CTUCAN_VER21 0xff00

#define CTUCAN_BAR0_CTUCAN_ID 0x0000
#define CTUCAN_BAR0_CRA_BASE  0x4000
#define CYCLONE_IV_CRA_A2P_IE (0x0050)

#define CTUCAN_WITHOUT_CTUCAN_ID  0
#define CTUCAN_WITH_CTUCAN_ID     1

struct ctucan_pci_board_data {
	void __iomem *bar0_base;
	void __iomem *cra_base;
	void __iomem *bar1_base;
	struct list_head ndev_list_head;
	int use_msi;
};

struct ctucan_pci_board_data *ctucan_pci_get_bdata(struct pci_dev *pdev)
{
	return (struct ctucan_pci_board_data *)pci_get_drvdata(pdev);
}

static void ctucan_pci_set_drvdata(struct device *dev,
				   struct net_device *ndev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct ctucan_priv *priv = netdev_priv(ndev);
	struct ctucan_pci_board_data *bdata = ctucan_pci_get_bdata(pdev);

	list_add(&priv->peers_on_pdev, &bdata->ndev_list_head);
	priv->irq_flags = IRQF_SHARED;
}

/**
 * ctucan_pci_probe - PCI registration call
 * @pdev:	Handle to the pci device structure
 * @ent:	Pointer to the entry from ctucan_pci_tbl
 *
 * This function does all the memory allocation and registration for the CAN
 * device.
 *
 * Return: 0 on success and failure value on error
 */
static int ctucan_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *ent)
{
	struct device	*dev = &pdev->dev;
	unsigned long driver_data = ent->driver_data;
	struct ctucan_pci_board_data *bdata;
	void __iomem *addr;
	void __iomem *cra_addr;
	void __iomem *bar0_base;
	u32 cra_a2p_ie;
	u32 ctucan_id = 0;
	int ret;
	unsigned int ntxbufs;
	unsigned int num_cores = 1;
	unsigned int core_i = 0;
	int irq;
	int msi_ok = 0;

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(dev, "pci_enable_device FAILED\n");
		goto err;
	}

	ret = pci_request_regions(pdev, KBUILD_MODNAME);
	if (ret) {
		dev_err(dev, "pci_request_regions FAILED\n");
		goto err_disable_device;
	}

	if (use_msi) {
		ret = pci_enable_msi(pdev);
		if (!ret) {
			dev_info(dev, "MSI enabled\n");
			pci_set_master(pdev);
			msi_ok = 1;
		}
	}

	dev_info(dev, "ctucan BAR0 0x%08llx 0x%08llx\n",
		 (long long)pci_resource_start(pdev, 0),
		 (long long)pci_resource_len(pdev, 0));

	dev_info(dev, "ctucan BAR1 0x%08llx 0x%08llx\n",
		 (long long)pci_resource_start(pdev, 1),
		 (long long)pci_resource_len(pdev, 1));

	addr = pci_iomap(pdev, 1, pci_resource_len(pdev, 1));
	if (!addr) {
		dev_err(dev, "PCI BAR 1 cannot be mapped\n");
		ret = -ENOMEM;
		goto err_release_regions;
	}

	/* Cyclone IV PCI Express Control Registers Area */
	bar0_base = pci_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if (!bar0_base) {
		dev_err(dev, "PCI BAR 0 cannot be mapped\n");
		ret = -EIO;
		goto err_pci_iounmap_bar1;
	}

	if (driver_data == CTUCAN_WITHOUT_CTUCAN_ID) {
		cra_addr = bar0_base;
		num_cores = 2;
	} else {
		cra_addr = (char *)bar0_base + CTUCAN_BAR0_CRA_BASE;
		ctucan_id = ioread32((char *)bar0_base + CTUCAN_BAR0_CTUCAN_ID);
		dev_info(dev, "ctucan_id 0x%08lx\n", (unsigned long)ctucan_id);
		num_cores = ctucan_id & 0xf;
	}

	irq = pdev->irq;

	ntxbufs = 4;

	bdata = kzalloc(sizeof(*bdata), GFP_KERNEL);
	if (!bdata) {
		dev_err(dev, "Cannot allocate board private data\n");
		ret = -ENOMEM;
		goto err_pci_iounmap_bar0;
	}

	INIT_LIST_HEAD(&bdata->ndev_list_head);
	bdata->bar0_base = bar0_base;
	bdata->cra_base = cra_addr;
	bdata->bar1_base = addr;
	bdata->use_msi = msi_ok;

	pci_set_drvdata(pdev, bdata);

	ret = ctucan_probe_common(dev, addr, irq, ntxbufs, 100000000,
				  0, ctucan_pci_set_drvdata);
	if (ret < 0)
		goto err_free_board;

	core_i++;

	while (pci_use_second && (core_i < num_cores)) {
		addr += 0x4000;
		ret = ctucan_probe_common(dev, addr, irq, ntxbufs, 100000000,
					  0, ctucan_pci_set_drvdata);
		if (ret < 0) {
			dev_info(dev, "CTU CAN FD core %d initialization failed\n",
				 core_i);
			break;
		}
		core_i++;
	}

	/* enable interrupt in
	 * Avalon-MM to PCI Express Interrupt Enable Register
	 */
	cra_a2p_ie = ioread32((char *)cra_addr + CYCLONE_IV_CRA_A2P_IE);
	dev_info(dev, "cra_a2p_ie 0x%08x\n", cra_a2p_ie);
	cra_a2p_ie |= 1;
	iowrite32(cra_a2p_ie, (char *)cra_addr + CYCLONE_IV_CRA_A2P_IE);
	cra_a2p_ie = ioread32((char *)cra_addr + CYCLONE_IV_CRA_A2P_IE);
	dev_info(dev, "cra_a2p_ie 0x%08x\n", cra_a2p_ie);

	return 0;

err_free_board:
	pci_set_drvdata(pdev, NULL);
	kfree(bdata);
err_pci_iounmap_bar0:
	pci_iounmap(pdev, cra_addr);
err_pci_iounmap_bar1:
	pci_iounmap(pdev, addr);
err_release_regions:
	if (msi_ok) {
		pci_disable_msi(pdev);
		pci_clear_master(pdev);
	}
	pci_release_regions(pdev);
err_disable_device:
	pci_disable_device(pdev);
err:
	return ret;
}

/**
 * ctucan_pci_remove - Unregister the device after releasing the resources
 * @pdev:	Handle to the pci device structure
 *
 * This function frees all the resources allocated to the device.
 * Return: 0 always
 */
static void ctucan_pci_remove(struct pci_dev *pdev)
{
	struct net_device *ndev;
	struct ctucan_priv *priv = NULL;
	struct ctucan_pci_board_data *bdata = ctucan_pci_get_bdata(pdev);

	dev_dbg(&pdev->dev, "ctucan_remove");

	if (!bdata) {
		dev_err(&pdev->dev, "%s: no list of devices\n", __func__);
		return;
	}

	/* disable interrupt in
	 * Avalon-MM to PCI Express Interrupt Enable Register
	 */
	if (bdata->cra_base)
		iowrite32(0, (char *)bdata->cra_base + CYCLONE_IV_CRA_A2P_IE);

	while ((priv = list_first_entry_or_null(&bdata->ndev_list_head,
				struct ctucan_priv, peers_on_pdev)) != NULL) {
		ndev = priv->can.dev;

		unregister_candev(ndev);

		netif_napi_del(&priv->napi);

		list_del_init(&priv->peers_on_pdev);
		free_candev(ndev);
	}

	pci_iounmap(pdev, bdata->bar1_base);

	if (bdata->use_msi) {
		pci_disable_msi(pdev);
		pci_clear_master(pdev);
	}

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	pci_iounmap(pdev, bdata->bar0_base);

	pci_set_drvdata(pdev, NULL);
	kfree(bdata);
}

static SIMPLE_DEV_PM_OPS(ctucan_pci_pm_ops, ctucan_suspend, ctucan_resume);

static const struct pci_device_id ctucan_pci_tbl[] = {
	{PCI_DEVICE_DATA(ALTERA, CTUCAN_TEST,
		CTUCAN_WITHOUT_CTUCAN_ID)},
	{PCI_DEVICE_DATA(TEDIA, CTUCAN_VER21,
		CTUCAN_WITH_CTUCAN_ID)},
	{},
};

static struct pci_driver ctucan_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = ctucan_pci_tbl,
	.probe = ctucan_pci_probe,
	.remove = ctucan_pci_remove,
	.driver.pm = &ctucan_pci_pm_ops,
};

module_pci_driver(ctucan_pci_driver);

#endif /*CONFIG_PCI*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin Jerabek");
MODULE_DESCRIPTION("CTU CAN FD interface");
