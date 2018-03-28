#include "ctu_can_fd_linux_defs.h"

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
