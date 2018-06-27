/*
 * extctl_protocol.h
 *
 *  Created on: Jun 27, 2018
 *      Author: lidq
 */

#ifndef SRC_MODULES_EXTCTL_EXTCTL_PROTOCOL_H_
#define SRC_MODULES_EXTCTL_EXTCTL_PROTOCOL_H_

#include "extctl_typedef.h"

#define FRM_HEAD_0			0X55
#define FRM_HEAD_1			0XAA
#define FRM_FOOT_0			0XA5
#define FRM_FOOT_1			0X5A

#define PAR_HEAD			0
#define PAR_LEN				1
#define PAR_END				2

#define SIZE_BUFF			(0x800)

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

int extctl_protocal_init(int fd);

int extctl_protocal_write(void *data, int data_type, int data_len);

int extctl_protocal_read(char *out_buff, int *out_len, int *out_type);

static int protocal_frame_pos(int len_data);

static int protocal_frame_mk_data(char *frame, int len_frame, char *data, int type, int len_data);

static int protocal_send_frame_write(char *frame, int len);

static int protocal_frame_count(s_buff *lb);

static void protocal_frame_read_data(void);

static uint16_t protocal_crc16_value(uint8_t *buff, uint8_t len);

static int protocal_crc16_check(uint8_t *buff, uint8_t len, uint16_t crc16);


#endif /* SRC_DRIVERS_extctl_extctl_PROTOCOL_H_ */
