/*
 * sbus.c
 *
 *  Created on: Jun 29, 2018
 *      Author: lidq
 */

#include <sbus.h>

int sbus_read(uint16_t *rc_values, uint8_t *rc_flag)
{
	int result = 0;
	int cnt = uart2_buff_count();
	if (cnt >= 25)
	{
		char buff[0x100];
		cnt = uart2_read(buff, 0x100);
		int len = sbus_parse(buff, cnt, rc_values, &rc_flag, &result);
		if (result)
		{
			return 1;
		}
	}
	return 0;
}

int sbus_parse(char *buff, int len, uint16_t *val, uint8_t *flag, int *result)
{
	*result = 0;
	if (buff == NULL)
	{
		return 0;
	}

	if (flag == NULL)
	{
		return 0;
	}

	if (len < 25)
	{
		return 0;
	}

	int ind = 0;
	for (ind = 0; ind < len; ind++)
	{
		if (buff[ind] == 0x0f)
		{
			goto _found_head;
		}
	}

	return 0;

	_found_head: ;

	if (ind + 24 > len)
	{
		return ind;
	}

	*flag = buff[ind + 23];

	for (int i = 0; i < 16; i++)
	{
		val[i] = 0;
	}

//	ind++;
//	int t = 0;
//	int k = 0;
//	for (int i = ind; i <= ind + 22; i++)
//	{
//		for (int j = 7; j >= 0; j--)
//		{
//			uint16_t tmp = (buff[i] >> j) & 1;
//			tmp <<= k;
//			val[t] |= tmp;
//			k++;
//			if (k >= 11)
//			{
//				k = 0;
//				t++;
//			}
//		}
//	}

	val[0] = ((buff[ind + 1] | buff[ind + 2] << 8) & 0x07FF);
	val[1] = ((buff[ind + 2] >> 3 | buff[ind + 3] << 5) & 0x07FF);
	val[2] = ((buff[ind + 3] >> 6 | buff[ind + 4] << 2 | buff[ind + 5] << 10) & 0x07FF);
	val[3] = ((buff[ind + 5] >> 1 | buff[ind + 6] << 7) & 0x07FF);
	val[4] = ((buff[ind + 6] >> 4 | buff[ind + 7] << 4) & 0x07FF);
	val[5] = ((buff[ind + 7] >> 7 | buff[ind + 8] << 1 | buff[ind + 9] << 9) & 0x07FF);
	val[6] = ((buff[ind + 9] >> 2 | buff[ind + 10] << 6) & 0x07FF);
	val[7] = ((buff[ind + 10] >> 5 | buff[ind + 11] << 3) & 0x07FF);
	val[8] = ((buff[ind + 12] | buff[ind + 13] << 8) & 0x07FF);
	val[9] = ((buff[ind + 13] >> 3 | buff[ind + 14] << 5) & 0x07FF);
	val[10] = ((buff[ind + 14] >> 6 | buff[ind + 15] << 2 | buff[ind + 16] << 10) & 0x07FF);
	val[11] = ((buff[ind + 16] >> 1 | buff[ind + 17] << 7) & 0x07FF);
	val[12] = ((buff[ind + 17] >> 4 | buff[ind + 18] << 4) & 0x07FF);
	val[13] = ((buff[ind + 18] >> 7 | buff[ind + 19] << 1 | buff[ind + 20] << 9) & 0x07FF);
	val[14] = ((buff[ind + 20] >> 2 | buff[ind + 21] << 6) & 0x07FF);
	val[15] = ((buff[ind + 21] >> 5 | buff[ind + 22] << 3) & 0x07FF);

	for (int i = 0; i < 16; i++)
	{
		val[i] = (uint16_t)(val[i] * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	}

	*result = 1;
	return ind + 24;
}

