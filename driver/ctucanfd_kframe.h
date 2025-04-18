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

/* This file is autogenerated, DO NOT EDIT! */

#ifndef __CTU_CAN_FD_CAN_FD_FRAME_FORMAT__
#define __CTU_CAN_FD_CAN_FD_FRAME_FORMAT__

#include <linux/bits.h>

/* CAN_Frame_format memory map */
enum ctu_can_fd_can_frame_format {
	CTUCANFD_FRAME_FORMAT_W       = 0x0,
	CTUCANFD_IDENTIFIER_W         = 0x4,
	CTUCANFD_TIMESTAMP_L_W        = 0x8,
	CTUCANFD_TIMESTAMP_U_W        = 0xc,
	CTUCANFD_DATA_1_4_W          = 0x10,
	CTUCANFD_DATA_5_8_W          = 0x14,
	CTUCANFD_DATA_9_12_W         = 0x18,
	CTUCANFD_DATA_13_16_W        = 0x1c,
	CTUCANFD_DATA_17_20_W        = 0x20,
	CTUCANFD_DATA_21_24_W        = 0x24,
	CTUCANFD_DATA_25_28_W        = 0x28,
	CTUCANFD_DATA_29_32_W        = 0x2c,
	CTUCANFD_DATA_33_36_W        = 0x30,
	CTUCANFD_DATA_37_40_W        = 0x34,
	CTUCANFD_DATA_41_44_W        = 0x38,
	CTUCANFD_DATA_45_48_W        = 0x3c,
	CTUCANFD_DATA_49_52_W        = 0x40,
	CTUCANFD_DATA_53_56_W        = 0x44,
	CTUCANFD_DATA_57_60_W        = 0x48,
	CTUCANFD_DATA_61_64_W        = 0x4c,
	CTUCANFD_FRAME_TEST_W        = 0x50,
};
/* CAN_FD_Frame_format memory region */

/*  FRAME_FORMAT_W registers */
#define REG_FRAME_FORMAT_W_DLC GENMASK(3, 0)
#define REG_FRAME_FORMAT_W_ERF BIT(4)
#define REG_FRAME_FORMAT_W_RTR BIT(5)
#define REG_FRAME_FORMAT_W_IDE BIT(6)
#define REG_FRAME_FORMAT_W_FDF BIT(7)
#define REG_FRAME_FORMAT_W_LBPF BIT(8)
#define REG_FRAME_FORMAT_W_BRS BIT(9)
#define REG_FRAME_FORMAT_W_ESI_RSV BIT(10)
#define REG_FRAME_FORMAT_W_RWCNT GENMASK(15, 11)
#define REG_FRAME_FORMAT_W_ERF_POS GENMASK(19, 16)
#define REG_FRAME_FORMAT_W_ERF_ERP BIT(20)
#define REG_FRAME_FORMAT_W_ERF_TYPE GENMASK(23, 21)
#define REG_FRAME_FORMAT_W_IVLD BIT(24)
#define REG_FRAME_FORMAT_W_LBTBI GENMASK(27, 25)

/*  IDENTIFIER_W registers */
#define REG_IDENTIFIER_W_IDENTIFIER_EXT GENMASK(17, 0)
#define REG_IDENTIFIER_W_IDENTIFIER_BASE GENMASK(28, 18)

/*  TIMESTAMP_L_W registers */
#define REG_TIMESTAMP_L_W_TIME_STAMP_L_W GENMASK(31, 0)

/*  TIMESTAMP_U_W registers */
#define REG_TIMESTAMP_U_W_TIMESTAMP_U_W GENMASK(31, 0)

/*  DATA_1_4_W registers */
#define REG_DATA_1_4_W_DATA_1 GENMASK(7, 0)
#define REG_DATA_1_4_W_DATA_2 GENMASK(15, 8)
#define REG_DATA_1_4_W_DATA_3 GENMASK(23, 16)
#define REG_DATA_1_4_W_DATA_4 GENMASK(31, 24)

/*  DATA_5_8_W registers */
#define REG_DATA_5_8_W_DATA_5 GENMASK(7, 0)
#define REG_DATA_5_8_W_DATA_6 GENMASK(15, 8)
#define REG_DATA_5_8_W_DATA_7 GENMASK(23, 16)
#define REG_DATA_5_8_W_DATA_8 GENMASK(31, 24)

/*  DATA_9_12_W registers */
#define REG_DATA_9_12_W_DATA_9 GENMASK(7, 0)
#define REG_DATA_9_12_W_DATA_10 GENMASK(15, 8)
#define REG_DATA_9_12_W_DATA_11 GENMASK(23, 16)
#define REG_DATA_9_12_W_DATA_12 GENMASK(31, 24)

/*  DATA_13_16_W registers */
#define REG_DATA_13_16_W_DATA_13 GENMASK(7, 0)
#define REG_DATA_13_16_W_DATA_14 GENMASK(15, 8)
#define REG_DATA_13_16_W_DATA_15 GENMASK(23, 16)
#define REG_DATA_13_16_W_DATA_16 GENMASK(31, 24)

/*  DATA_17_20_W registers */
#define REG_DATA_17_20_W_DATA_17 GENMASK(7, 0)
#define REG_DATA_17_20_W_DATA_18 GENMASK(15, 8)
#define REG_DATA_17_20_W_DATA_19 GENMASK(23, 16)
#define REG_DATA_17_20_W_DATA_20 GENMASK(31, 24)

/*  DATA_21_24_W registers */
#define REG_DATA_21_24_W_DATA_21 GENMASK(7, 0)
#define REG_DATA_21_24_W_DATA_22 GENMASK(15, 8)
#define REG_DATA_21_24_W_DATA_23 GENMASK(23, 16)
#define REG_DATA_21_24_W_DATA_24 GENMASK(31, 24)

/*  DATA_25_28_W registers */
#define REG_DATA_25_28_W_DATA_25 GENMASK(7, 0)
#define REG_DATA_25_28_W_DATA_26 GENMASK(15, 8)
#define REG_DATA_25_28_W_DATA_27 GENMASK(23, 16)
#define REG_DATA_25_28_W_DATA_28 GENMASK(31, 24)

/*  DATA_29_32_W registers */
#define REG_DATA_29_32_W_DATA_29 GENMASK(7, 0)
#define REG_DATA_29_32_W_DATA_30 GENMASK(15, 8)
#define REG_DATA_29_32_W_DATA_31 GENMASK(23, 16)
#define REG_DATA_29_32_W_DATA_32 GENMASK(31, 24)

/*  DATA_33_36_W registers */
#define REG_DATA_33_36_W_DATA_33 GENMASK(7, 0)
#define REG_DATA_33_36_W_DATA_34 GENMASK(15, 8)
#define REG_DATA_33_36_W_DATA_35 GENMASK(23, 16)
#define REG_DATA_33_36_W_DATA_36 GENMASK(31, 24)

/*  DATA_37_40_W registers */
#define REG_DATA_37_40_W_DATA_37 GENMASK(7, 0)
#define REG_DATA_37_40_W_DATA_38 GENMASK(15, 8)
#define REG_DATA_37_40_W_DATA_39 GENMASK(23, 16)
#define REG_DATA_37_40_W_DATA_40 GENMASK(31, 24)

/*  DATA_41_44_W registers */
#define REG_DATA_41_44_W_DATA_41 GENMASK(7, 0)
#define REG_DATA_41_44_W_DATA_42 GENMASK(15, 8)
#define REG_DATA_41_44_W_DATA_43 GENMASK(23, 16)
#define REG_DATA_41_44_W_DATA_44 GENMASK(31, 24)

/*  DATA_45_48_W registers */
#define REG_DATA_45_48_W_DATA_45 GENMASK(7, 0)
#define REG_DATA_45_48_W_DATA_46 GENMASK(15, 8)
#define REG_DATA_45_48_W_DATA_47 GENMASK(23, 16)
#define REG_DATA_45_48_W_DATA_48 GENMASK(31, 24)

/*  DATA_49_52_W registers */
#define REG_DATA_49_52_W_DATA_49 GENMASK(7, 0)
#define REG_DATA_49_52_W_DATA_50 GENMASK(15, 8)
#define REG_DATA_49_52_W_DATA_51 GENMASK(23, 16)
#define REG_DATA_49_52_W_DATA_52 GENMASK(31, 24)

/*  DATA_53_56_W registers */
#define REG_DATA_53_56_W_DATA_53 GENMASK(7, 0)
#define REG_DATA_53_56_W_DATA_56 GENMASK(15, 8)
#define REG_DATA_53_56_W_DATA_55 GENMASK(23, 16)
#define REG_DATA_53_56_W_DATA_54 GENMASK(31, 24)

/*  DATA_57_60_W registers */
#define REG_DATA_57_60_W_DATA_57 GENMASK(7, 0)
#define REG_DATA_57_60_W_DATA_58 GENMASK(15, 8)
#define REG_DATA_57_60_W_DATA_59 GENMASK(23, 16)
#define REG_DATA_57_60_W_DATA_60 GENMASK(31, 24)

/*  DATA_61_64_W registers */
#define REG_DATA_61_64_W_DATA_61 GENMASK(7, 0)
#define REG_DATA_61_64_W_DATA_62 GENMASK(15, 8)
#define REG_DATA_61_64_W_DATA_63 GENMASK(23, 16)
#define REG_DATA_61_64_W_DATA_64 GENMASK(31, 24)

/*  FRAME_TEST_W registers */
#define REG_FRAME_TEST_W_FSTC BIT(0)
#define REG_FRAME_TEST_W_FCRC BIT(1)
#define REG_FRAME_TEST_W_SDLC BIT(2)
#define REG_FRAME_TEST_W_TPRM GENMASK(12, 8)

#endif
