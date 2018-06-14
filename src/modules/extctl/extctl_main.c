/*
 * Extern_control.cpp
 *
 *  Created on: Apr 21, 2018
 *      Author: lidq
 */

#include "extctl_main.h"

//#define __EXTCTL_DEBUG_

bool _extctl_should_exit = false;
orb_advert_t _extctl_mavlink_log_pub;

static uint16_t _crc16table[256] = { 0x2ae2, 0xf965, 0xa429, 0x1b33, 0xd0ce, 0xd13a, 0x69c2, 0xe829, 0xa785, 0xbec6, 0xe141, 0x3093, 0xb528, 0x94d3, 0xdfb9, 0x5c62, 0x98c2, 0x6fad, 0x234b, 0x8f03, 0xba66, 0x1ebb, 0x2060, 0xb68e, 0x68c9, 0xa3b5, 0xbceb, 0x17dc, 0x7b28, 0x63ba, 0x91aa, 0xa60b, 0x5d20, 0x35d4, 0xc13e, 0x2def, 0x870e, 0x2b01, 0x9618, 0xae94, 0x69c7, 0x775a, 0xdf27, 0x1ef1, 0x0c2e, 0x3ee1, 0xfb52, 0x24f0, 0xae8e, 0x1e9e, 0x33f3, 0xe8f4, 0xbd58, 0xd452, 0x1f83, 0x2622, 0x7808, 0x5c6f, 0x3dfe, 0x7331, 0xc029, 0x4fa8, 0x193d, 0x1d4a, 0x057d, 0xda7b, 0xcb38, 0x8c8b, 0x857d, 0x6151, 0x3b20, 0x6f44, 0x58ab, 0x9a47, 0x8e35, 0xe4d9, 0x5928, 0x0988, 0x09ca, 0x87b6, 0xa825, 0xbdbd, 0x70ab, 0x657e, 0x9210, 0x102f, 0x0ba1, 0x8a19, 0x6c9e, 0xc99e, 0xfd4a, 0x2cc8, 0x1948, 0x9687, 0xca11, 0x1ec5, 0xf102, 0x954a, 0x2b50, 0x7680, 0x769b, 0xe670, 0xe5c5, 0xcf46, 0x00b8, 0xf3fa, 0x3420, 0x59e1, 0x7d83, 0xbdea, 0xe197, 0x25a9, 0x7ba8, 0xd243, 0x8b28, 0x8db8, 0xe272, 0x96c9, 0x17d2, 0x4f11, 0xe067, 0x951c, 0x7bd9, 0xf9af, 0x2ba4, 0x45eb, 0x9874, 0x9ca7, 0x5b35, 0xc3c5, 0x9327, 0x51d0, 0x2a36, 0x78ed, 0x2117, 0x2aee, 0xece8, 0xd537, 0x84cf, 0xea6b, 0x9322, 0xe667, 0x1015, 0x0ecb, 0xb8ab, 0x9b3d, 0x9c83, 0x9b1e, 0xb206, 0x3456, 0xea2f, 0x126f, 0x4972, 0xe608, 0x0c1f, 0xf516, 0xabf3, 0x2494, 0x91be, 0x0729, 0x6859, 0x24e6, 0xd8f8, 0x928f, 0x1dd4, 0x7a0f, 0xbd7d, 0x8abc, 0x4f47, 0xc24d, 0x7528, 0xe269, 0x28b5, 0x853d, 0xf134, 0xe160, 0xa07b, 0x0db9, 0x7c7f, 0xd281, 0xc20e, 0xe6ae, 0xe4f0, 0x0b81, 0xccb7, 0x7110, 0x0098, 0xf8aa, 0x95a4, 0x1256, 0x7fd3, 0xfdfd, 0x373d, 0x58cc, 0x908d, 0xd510, 0xd2db, 0xce0a, 0x5fcd, 0x2224, 0x1058, 0xd4f5, 0x048e, 0x390d, 0xda32, 0x75c3, 0x9a6d, 0xfaad, 0x837c, 0x16ed, 0xcd30, 0xc58a, 0x7d9b, 0x3221, 0xd10b, 0xca52, 0xa331, 0x51a4, 0xc2fd, 0x38d6, 0xe3f9, 0x42d1, 0xb6d3, 0x1b37, 0x9b9d, 0xc760, 0x7047, 0x6e7a, 0x956c, 0xd014, 0x109e, 0x25c4, 0x250a, 0x952b, 0xded1, 0x7f3d, 0x0aef, 0x793f, 0x79eb, 0x0e6b, 0x902d, 0xc71b, 0xd3f5, 0x8dc8, 0xf93d, 0x2502, 0x581c, 0x9c6f, 0xf6a5, 0x1b1a, 0x5546, 0xda9f, 0x5dec, 0x8c19, 0x75d7, 0x7989 };
static s_buff _recv;
static uint8_t _buff[SIZE_BUFF];
static sem_t _sem_w;

static int _serial_fd = -1;
static int _frame_pos_head0 = 0;
static int _frame_pos_head1 = 0;
static int _frame_pos_len_frame = 0;
static int _frame_pos_type = 0;
static int _frame_pos_len_data = 0;
static int _frame_pos_data = 0;
static int _frame_pos_crc0 = 0;
static int _frame_pos_crc1 = 0;
static int _frame_pos_foot0 = 0;
static int _frame_pos_foot1 = 0;

int start(int argc, char *argv[])
{
	sem_init(&_sem_w, 0, 1);
	_recv.head = 0;
	_recv.tail = 0;
	_recv.size = SIZE_BUFF;
	memset(_recv.buff, 0x00, SIZE_BUFF);
	
	px4_main_t entry_point = (px4_main_t) extctl_read;
	int task_id = px4_task_spawn_cmd("exctl", SCHED_DEFAULT, SCHED_PRIORITY_FAST_DRIVER, CONFIG_PTHREAD_STACK_DEFAULT, entry_point, (char * const *) argv);
	
	return task_id;
}

int extctl_read(int argc, char *argv[])
{
	_serial_fd = open(DEV_NAME, O_RDWR | O_NONBLOCK);
	if (_serial_fd < 0)
	{
		return -1;
	}
	set_opt(_serial_fd, DEV_BAUDRATE, 8, 'N', 1);
	
	extctl_sp_init();
	extctl_cmd_init();
	
	pthread_t pthddr;
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &extctl_sp_send, NULL);
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &extctl_pos_send, NULL);
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &extctl_rc_send, NULL);
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &extctl_status_send, NULL);
	
	int (*p_handle)(void *) = NULL;
	
	while (!_extctl_should_exit)
	{
		frame_read_data();
		if (frame_parse())
		{
			p_handle = NULL;
			
			switch (_buff[_frame_pos_type])
			{
				case DATA_TYPE_POS:
					p_handle = &extctl_pos_handle;
					break;
					
				case DATA_TYPE_SP:
					p_handle = &extctl_sp_handle;
					break;
					
				case DATA_TYPE_RC:
					p_handle = &extctl_rc_handle;
					
				case DATA_TYPE_CMD:
					p_handle = &extctl_cmd_handle;
					break;
					
				case DATA_TYPE_STATUS:
					p_handle = &extctl_status_handle;
					break;

				default:
					break;
			}
			
			if (p_handle != NULL)
			{
				p_handle(&_buff[_frame_pos_data]);
			}
		}
		
		usleep(DEV_RATE_READ);
	}
	
	//wait write thread exit.
	usleep(DEV_RATE_BASE);
	
	close(_serial_fd);
	
	return 0;
}

int frame_pos(int len_data)
{
	//head(2) + len_frame(1) + type_data(1) + len_data(1) + data(len) + crc(2) + foot(2)
	int len_frame = 2 + 1 + 1 + 1 + len_data + 2 + 2;
	_frame_pos_head0 = 0;
	_frame_pos_head1 = _frame_pos_head0 + 1;
	_frame_pos_len_frame = _frame_pos_head1 + 1;
	_frame_pos_type = _frame_pos_len_frame + 1;
	_frame_pos_len_data = _frame_pos_type + 1;
	_frame_pos_data = _frame_pos_len_data + 1;
	_frame_pos_crc0 = _frame_pos_data + len_data;
	_frame_pos_crc1 = _frame_pos_crc0 + 1;
	_frame_pos_foot0 = _frame_pos_crc1 + 1;
	_frame_pos_foot1 = _frame_pos_foot0 + 1;
	return len_frame;
}

int frame_mk_data(char *frame, int len_frame, char *data, int type, int len_data)
{
	memset(frame, 0, len_data);
	
	frame[_frame_pos_head0] = FRM_HEAD_0;
	frame[_frame_pos_head1] = FRM_HEAD_1;
	
	frame[_frame_pos_len_frame] = len_frame;
	frame[_frame_pos_type] = type;
	frame[_frame_pos_len_data] = len_data;
	
	memcpy(&frame[_frame_pos_data], data, len_data);
	
	//len_frame - crc(2) - foot(2)
	uint16_t crc16 = crc16_value((uint8_t*) frame, len_frame - 4);
	
	frame[_frame_pos_crc0] = crc16 >> 8;
	frame[_frame_pos_crc1] = crc16 & 0xff;
	
	frame[_frame_pos_foot0] = FRM_FOOT_0;
	frame[_frame_pos_foot1] = FRM_FOOT_1;
	
	return OK;
}

int send_data_buff(void *data, int data_type, int data_len)
{
	if (data == NULL)
	{
		return -1;
	}
	
	sem_wait(&_sem_w);
	
	int len_frame = frame_pos(data_len);
	char frame[len_frame];
	frame_mk_data(frame, len_frame, (char *) data, data_type, data_len);
	send_frame_write(frame, len_frame);
	
	sem_post(&_sem_w);
	
	return 0;
}

int send_frame_write(char *frame, int len)
{
	int wlen = 0;
	if (_serial_fd > 0)
	{
		wlen = write(_serial_fd, frame, len);
	}
	return wlen;
}

int stop(void)
{
	_extctl_should_exit = true;
	//wait task_main exit
	usleep(200 * 1000);
	
	return OK;
}

int frame_count(s_buff *lb)
{
	int16_t n = lb->head - lb->tail;
	if (n < 0)
	{
		n += lb->size;
	}
	return n;
}

int frame_parse()
{
	uint16_t data_cnt = 0;
	uint16_t parse_packet_step = 0;
	uint16_t packet_index = 0;
	uint16_t frameLen = 0;
	uint8_t frame_type = 0;
	uint8_t frame_data_len = 0;
	int ret = 0;
	data_cnt = frame_count(&_recv);
	int16_t tail = _recv.tail;
	
	_frame_pos_head0 = 0;
	_frame_pos_head1 = _frame_pos_head0 + 1;
	_frame_pos_len_frame = _frame_pos_head1 + 1;
	_frame_pos_type = _frame_pos_len_frame + 1;
	_frame_pos_len_data = _frame_pos_type + 1;
	_frame_pos_data = _frame_pos_len_data + 1;
	
	for (uint16_t i = 0; i < data_cnt; i++)
	{
		_buff[packet_index++] = _recv.buff[tail];
		switch (parse_packet_step)
		{
			case PAR_HEAD:
				if (packet_index >= 2)
				{
					if ((_buff[_frame_pos_head0] == FRM_HEAD_0) && (_buff[_frame_pos_head1] == FRM_HEAD_1))
					{
						parse_packet_step = PAR_LEN;
					}
					else if (_buff[_frame_pos_head1] == FRM_HEAD_0)
					{
						_buff[_frame_pos_head0] = FRM_HEAD_0;
						packet_index = 1;
					}
					else
					{
						packet_index = 0;
					}
				}
				break;
				
			case PAR_LEN:
				if (packet_index >= 5)
				{
					frameLen = _buff[_frame_pos_len_frame];
					frame_type = _buff[_frame_pos_type];
					frame_data_len = _buff[_frame_pos_len_data];
					_frame_pos_crc0 = _frame_pos_data + frame_data_len;
					_frame_pos_crc1 = _frame_pos_crc0 + 1;
					_frame_pos_foot0 = _frame_pos_crc1 + 1;
					_frame_pos_foot1 = _frame_pos_foot0 + 1;
					if (frameLen <= SIZE_BUFF)
					{
						parse_packet_step = PAR_END;
					}
					else
					{
						parse_packet_step = PAR_HEAD;
						packet_index = 0;
						memset(_buff, 0x00, sizeof(_buff));
					}
				}
				break;
				
			case PAR_END:
				if (packet_index == frameLen)
				{
					uint16_t crc16 = _buff[_frame_pos_crc0] << 8 | _buff[_frame_pos_crc1];
					if (crc16_check(&_buff[_frame_pos_head0], frameLen - 4, crc16))
					{
						ret = 1;
						goto _ret;
					}
					parse_packet_step = PAR_HEAD;
					packet_index = 0;
				}
				break;
				
			default:
				break;
		}
		tail = (tail + 1) % _recv.size;
	}
	
	_ret: ;
	
	if (ret == 1)
	{
		_recv.tail = tail;
	}
	
	if (frame_type)
	{
	}
	return ret;
}

void frame_read_data(void)
{
	uint8_t tmp_serial_buf[SIZE_BUFF];
	int len = read(_serial_fd, tmp_serial_buf, SIZE_BUFF);
	for (int i = 0; i < len; i++)
	{
		_recv.buff[_recv.head] = tmp_serial_buf[i];
		_recv.head = (_recv.head + 1) % _recv.size;
		if (_recv.head == _recv.tail)
		{
			_recv.over++;
		}
	}
}

uint16_t crc16_value(uint8_t *buff, uint8_t len)
{
	uint16_t crc16 = 0;
	for (uint8_t i = 0; i < len; i++)
	{
		crc16 = _crc16table[((crc16 >> 8) ^ buff[i]) & 0xff] ^ (crc16 << 8);
	}
	return crc16;
}

int crc16_check(uint8_t *buff, uint8_t len, uint16_t crc16)
{
	uint16_t sum = 0;
	for (uint8_t i = 0; i < len; i++)
	{
		sum = _crc16table[((sum >> 8) ^ buff[i]) & 0xff] ^ (sum << 8);
	}
	if (sum == crc16)
	{
		return 1;
	}
	return 0;
}

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio, oldtio;
	//保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	bzero(&newtio, sizeof(newtio));
	//步骤一，设置字符大小
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	//设置停止位
	switch (nBits)
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}
	//设置奇偶校验位
	switch (nEvent)
	{
		case 'O': //奇数
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK | ISTRIP);
			break;
		case 'E': //偶数
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N': //无奇偶校验位
			newtio.c_cflag &= ~PARENB;
			break;
	}
	cfsetispeed(&newtio, nSpeed);
	cfsetospeed(&newtio, nSpeed);
	//设置停止位
	if (nStop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if (nStop == 2)
	{
		newtio.c_cflag |= CSTOPB;
	}
	//设置等待时间和最小接收字符
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	//处理未接收字符
	tcflush(fd, TCIFLUSH);
	//激活新配置
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}
	printf("set done!\n");
	return 0;
}

int extctl_main(int argc, char *argv[])
{
	if (strcmp(argv[1], "start") == 0)
	{
		start(argc, argv);
		return OK;
	}
	
	if (strcmp(argv[1], "stop") == 0)
	{
		stop();
		return OK;
	}
	
	return -1;
}
