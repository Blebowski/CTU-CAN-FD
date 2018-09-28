/*******************************************************************************
 * 
 * CTU CAN FD IP Core
 * Copyright (C) 2015-2018
 * 
 * Authors:
 *     Ondrej Ille <ondrej.ille@gmail.com>
 *     Martin Jerabek <jerabma7@fel.cvut.cz>
 * 
 * Project advisors: 
 * 	Jiri Novak <jnovak@fel.cvut.cz>
 * 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
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

#include "ctu_can_fd_linux_defs.h"

#include <errno.h>
#include <limits.h>

/* CAN DLC to real data length conversion helpers */

static const u8 dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7,
			     8, 12, 16, 20, 24, 32, 48, 64};

/* get data length from can_dlc with sanitized can_dlc */
u8 can_dlc2len(u8 can_dlc)
{
	return dlc2len[can_dlc & 0x0F];
}

static const u8 len2dlc[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,		/* 0 - 8 */
			     9, 9, 9, 9,			/* 9 - 12 */
			     10, 10, 10, 10,			/* 13 - 16 */
			     11, 11, 11, 11,			/* 17 - 20 */
			     12, 12, 12, 12,			/* 21 - 24 */
			     13, 13, 13, 13, 13, 13, 13, 13,	/* 25 - 32 */
			     14, 14, 14, 14, 14, 14, 14, 14,	/* 33 - 40 */
			     14, 14, 14, 14, 14, 14, 14, 14,	/* 41 - 48 */
			     15, 15, 15, 15, 15, 15, 15, 15,	/* 49 - 56 */
			     15, 15, 15, 15, 15, 15, 15, 15};	/* 57 - 64 */

/* map the sanitized data length to an appropriate data length code */
u8 can_len2dlc(u8 len)
{
	if (unlikely(len > 64))
		return 0xF;

	return len2dlc[len];
}

#define CAN_CALC_MAX_ERROR 50 /* in one-tenth of a percent */
#define CAN_CALC_SYNC_SEG 1

/*
 * Bit-timing calculation derived from:
 *
 * Code based on LinCAN sources and H8S2638 project
 * Copyright 2004-2006 Pavel Pisa - DCE FELK CVUT cz
 * Copyright 2005      Stanislav Marek
 * email: pisa@cmp.felk.cvut.cz
 *
 * Calculates proper bit-timing parameters for a specified bit-rate
 * and sample-point, which can then be used to set the bit-timing
 * registers of the CAN controller. You can find more information
 * in the header file linux/can/netlink.h.
 */
static int can_update_sample_point(const struct can_bittiming_const *btc,
                                   unsigned int sample_point_nominal, unsigned int tseg,
                                   unsigned int *tseg1_ptr, unsigned int *tseg2_ptr,
                                   unsigned int *sample_point_error_ptr)
{
    unsigned int sample_point_error, best_sample_point_error = UINT_MAX;
    unsigned int sample_point, best_sample_point = 0;
    unsigned int tseg1, tseg2;
    int i;

    for (i = 0; i <= 1; i++) {
        tseg2 = tseg + CAN_CALC_SYNC_SEG - (sample_point_nominal * (tseg + CAN_CALC_SYNC_SEG)) / 1000 - i;
        tseg2 = clamp(tseg2, btc->tseg2_min, btc->tseg2_max);
        tseg1 = tseg - tseg2;
        if (tseg1 > btc->tseg1_max) {
            tseg1 = btc->tseg1_max;
            tseg2 = tseg - tseg1;
        }

        sample_point = 1000 * (tseg + CAN_CALC_SYNC_SEG - tseg2) / (tseg + CAN_CALC_SYNC_SEG);
        sample_point_error = abs(sample_point_nominal - sample_point);

        if ((sample_point <= sample_point_nominal) && (sample_point_error < best_sample_point_error)) {
            best_sample_point = sample_point;
            best_sample_point_error = sample_point_error;
            *tseg1_ptr = tseg1;
            *tseg2_ptr = tseg2;
        }
    }

    if (sample_point_error_ptr)
        *sample_point_error_ptr = best_sample_point_error;

    return best_sample_point;
}

static int can_calc_bittiming(struct net_device *dev, struct can_bittiming *bt,
                              const struct can_bittiming_const *btc)
{
    struct can_priv *priv = netdev_priv(dev);
    unsigned int bitrate;			/* current bitrate */
    unsigned int bitrate_error;		/* difference between current and nominal value */
    unsigned int best_bitrate_error = UINT_MAX;
    unsigned int sample_point_error;	/* difference between current and nominal value */
    unsigned int best_sample_point_error = UINT_MAX;
    unsigned int sample_point_nominal;	/* nominal sample point */
    unsigned int best_tseg = 0;		/* current best value for tseg */
    unsigned int best_brp = 0;		/* current best value for brp */
    unsigned int brp, tsegall, tseg, tseg1 = 0, tseg2 = 0;
    u64 v64;

    /* Use CiA recommended sample points */
    if (bt->sample_point) {
        sample_point_nominal = bt->sample_point;
    } else {
        if (bt->bitrate > 800000)
            sample_point_nominal = 750;
        else if (bt->bitrate > 500000)
            sample_point_nominal = 800;
        else
            sample_point_nominal = 875;
    }

    /* tseg even = round down, odd = round up */
    for (tseg = (btc->tseg1_max + btc->tseg2_max) * 2 + 1;
         tseg >= (btc->tseg1_min + btc->tseg2_min) * 2; tseg--) {
        tsegall = CAN_CALC_SYNC_SEG + tseg / 2;

    /* Compute all possible tseg choices (tseg=tseg1+tseg2) */
    brp = priv->clock.freq / (tsegall * bt->bitrate) + tseg % 2;

    /* choose brp step which is possible in system */
    brp = (brp / btc->brp_inc) * btc->brp_inc;
    if ((brp < btc->brp_min) || (brp > btc->brp_max))
        continue;

    bitrate = priv->clock.freq / (brp * tsegall);
    bitrate_error = abs(bt->bitrate - bitrate);

    /* tseg brp biterror */
    if (bitrate_error > best_bitrate_error)
        continue;

    /* reset sample point error if we have a better bitrate */
    if (bitrate_error < best_bitrate_error)
        best_sample_point_error = UINT_MAX;

    can_update_sample_point(btc, sample_point_nominal, tseg / 2, &tseg1, &tseg2, &sample_point_error);
    if (sample_point_error > best_sample_point_error)
        continue;

    best_sample_point_error = sample_point_error;
    best_bitrate_error = bitrate_error;
    best_tseg = tseg / 2;
    best_brp = brp;

    if (bitrate_error == 0 && sample_point_error == 0)
        break;
         }

         if (best_bitrate_error) {
             /* Error in one-tenth of a percent */
             v64 = (u64)best_bitrate_error * 1000;
             do_div(v64, bt->bitrate);
             bitrate_error = (u32)v64;
             if (bitrate_error > CAN_CALC_MAX_ERROR) {
                 netdev_err(dev,
                            "bitrate error %d.%d%% too high\n",
                            bitrate_error / 10, bitrate_error % 10);
                 return -EDOM;
             }
             netdev_warn(dev, "bitrate error %d.%d%%\n",
                         bitrate_error / 10, bitrate_error % 10);
         }

         /* real sample point */
         bt->sample_point = can_update_sample_point(btc, sample_point_nominal, best_tseg,
                                                    &tseg1, &tseg2, NULL);

         v64 = (u64)best_brp * 1000 * 1000 * 1000;
         do_div(v64, priv->clock.freq);
         bt->tq = (u32)v64;
         bt->prop_seg = tseg1 / 2;
         bt->phase_seg1 = tseg1 - bt->prop_seg;
         bt->phase_seg2 = tseg2;

         /* check for sjw user settings */
         if (!bt->sjw || !btc->sjw_max) {
             bt->sjw = 1;
         } else {
             /* bt->sjw is at least 1 -> sanitize upper bound to sjw_max */
             if (bt->sjw > btc->sjw_max)
                 bt->sjw = btc->sjw_max;
             /* bt->sjw must not be higher than tseg2 */
             if (tseg2 < bt->sjw)
                 bt->sjw = tseg2;
         }

         bt->brp = best_brp;

         /* real bitrate */
         bt->bitrate = priv->clock.freq / (bt->brp * (CAN_CALC_SYNC_SEG + tseg1 + tseg2));

         return 0;
}

/*
 * Checks the validity of the specified bit-timing parameters prop_seg,
 * phase_seg1, phase_seg2 and sjw and tries to determine the bitrate
 * prescaler value brp. You can find more information in the header
 * file linux/can/netlink.h.
 */
static int can_fixup_bittiming(struct net_device *dev, struct can_bittiming *bt,
                               const struct can_bittiming_const *btc)
{
    struct can_priv *priv = netdev_priv(dev);
    int tseg1, alltseg;
    u64 brp64;

    tseg1 = bt->prop_seg + bt->phase_seg1;
    if (!bt->sjw)
        bt->sjw = 1;
    if (bt->sjw > btc->sjw_max ||
        tseg1 < btc->tseg1_min || tseg1 > btc->tseg1_max ||
        bt->phase_seg2 < btc->tseg2_min || bt->phase_seg2 > btc->tseg2_max)
        return -ERANGE;

    brp64 = (u64)priv->clock.freq * (u64)bt->tq;
    if (btc->brp_inc > 1)
        do_div(brp64, btc->brp_inc);
    brp64 += 500000000UL - 1;
    do_div(brp64, 1000000000UL); /* the practicable BRP */
    if (btc->brp_inc > 1)
        brp64 *= btc->brp_inc;
    bt->brp = (u32)brp64;

    if (bt->brp < btc->brp_min || bt->brp > btc->brp_max)
        return -EINVAL;

    alltseg = bt->prop_seg + bt->phase_seg1 + bt->phase_seg2 + 1;
    bt->bitrate = priv->clock.freq / (bt->brp * alltseg);
    bt->sample_point = ((tseg1 + 1) * 1000) / alltseg;

    return 0;
}

/* Checks the validity of predefined bitrate settings */
static int can_validate_bitrate(struct net_device *dev, struct can_bittiming *bt,
                                const u32 *bitrate_const,
                                const unsigned int bitrate_const_cnt)
{
    struct can_priv *priv = netdev_priv(dev);
    unsigned int i;

    for (i = 0; i < bitrate_const_cnt; i++) {
        if (bt->bitrate == bitrate_const[i])
            break;
    }

    if (i >= priv->bitrate_const_cnt)
        return -EINVAL;

    return 0;
}

int can_get_bittiming(struct net_device *dev, struct can_bittiming *bt,
                      const struct can_bittiming_const *btc,
                      const u32 *bitrate_const,
                      const unsigned int bitrate_const_cnt)
{
    int err;

    /*
     * Depending on the given can_bittiming parameter structure the CAN
     * timing parameters are calculated based on the provided bitrate OR
     * alternatively the CAN timing parameters (tq, prop_seg, etc.) are
     * provided directly which are then checked and fixed up.
     */
    if (!bt->tq && bt->bitrate && btc)
        err = can_calc_bittiming(dev, bt, btc);
    else if (bt->tq && !bt->bitrate && btc)
        err = can_fixup_bittiming(dev, bt, btc);
    else if (!bt->tq && bt->bitrate && bitrate_const)
        err = can_validate_bitrate(dev, bt, bitrate_const,
                                   bitrate_const_cnt);
        else
            err = -EINVAL;

        return err;
}
