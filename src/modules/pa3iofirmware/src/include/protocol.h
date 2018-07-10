/*
 * protocol.h
 *
 *  Created on: Jun 27, 2018
 *      Author: lidq
 */

#ifndef SRC_DRIVERS_PROTOCOL_H_
#define SRC_DRIVERS_PROTOCOL_H_

#include <typedef.h>

#define FRM_HEAD_0			0X55
#define FRM_HEAD_1			0XAA
#define FRM_FOOT_0			0XA5
#define FRM_FOOT_1			0X5A

#define PAR_HEAD			0
#define PAR_LEN				1
#define PAR_END				2

#define SIZE_BUFF			(0x800)

#define PROTOCAL_LIFE		(100)

typedef struct s_buff
{
	int16_t head;
	int16_t tail;
	int16_t size;
	uint8_t buff[SIZE_BUFF];
	uint32_t total_len;
	uint32_t over;
	uint32_t user_buf_over;
} s_buff;

typedef struct s_frame_pos
{
	int head0;
	int head1;
	int len_frame;
	int type;
	int len_data;
	int data;
	int crc0;
	int crc1;
	int foot0;
	int foot1;
} s_frame_pos;

int protocal_init(void);

int protocal_write(void *data, int data_type, int data_len);

int protocal_read(char *out_buff, int *out_len, int *out_type);


#endif /* SRC_DRIVERS_PROTOCOL_H_ */
