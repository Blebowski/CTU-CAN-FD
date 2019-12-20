/* SPDX-License-Identifier: GPL-2.0+
 *******************************************************************************
 *
 * CTU CAN FD IP Core
 * Copyright (C) 2015-2018
 *
 * Authors:
 *     Ondrej Ille <ondrej.ille@gmail.com>
 *     Martin Jerabek <martin.jerabek01@gmail.com>
 *     Jaroslav Beran <jara.beran@gmail.com>
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

#ifndef __CTU_CAN_FD__
#define __CTU_CAN_FD__

#include <linux/netdevice.h>
#include <linux/can/dev.h>
#include <linux/list.h>

#include "ctu_can_fd_hw.h"

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
int ctucan_probe_common(struct device *dev, void __iomem *addr,
			int irq, unsigned int ntxbufs,
			unsigned long can_clk_rate,
			int pm_enable_call,
			void (*set_drvdata_fnc)(struct device *dev,
			struct net_device *ndev));

int ctucan_suspend(struct device *dev) __maybe_unused;
int ctucan_resume(struct device *dev) __maybe_unused;

#endif /*__CTU_CAN_FD__*/
