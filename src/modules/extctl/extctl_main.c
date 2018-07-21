/*
 * Extern_control.cpp
 *
 *  Created on: Apr 21, 2018
 *      Author: lidq
 */

#include "extctl_main.h"

//#define __EXTCTL_DEBUG_
extern int _socket_id;
bool _extctl_should_exit = false;
orb_advert_t _extctl_mavlink_log_pub;

static int _serial_fd = -1;

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

int start(int argc, char *argv[])
{
	px4_main_t entry_point = (px4_main_t) extctl_read;
	int task_id = px4_task_spawn_cmd("exctl", SCHED_DEFAULT, SCHED_PRIORITY_FAST_DRIVER, CONFIG_PTHREAD_STACK_DEFAULT, entry_point, (char * const *) argv);
	
	return task_id;
}

int stop(void)
{
	_extctl_should_exit = true;
	usleep(200 * 1000);

	return OK;
}

int extctl_read(int argc, char *argv[])
{
	pthread_t pthddr;

#ifdef __PX4_POSIX
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &server_start, NULL);
#else
	_serial_fd = open(DEV_NAME, O_RDWR | O_NONBLOCK);
	if (_serial_fd < 0)
	{
		return -1;
	}
	set_opt(_serial_fd, DEV_BAUDRATE, 8, 'N', 1);
#endif

	extctl_protocal_init(_serial_fd);

	extctl_sp_init();
	extctl_cmd_init();
	
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &extctl_sp_send, NULL);
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &extctl_pos_send, NULL);
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &extctl_rc_send, NULL);
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &extctl_status_send, NULL);
	
	int (*p_handle)(void *) = NULL;
	
	char buff[SIZE_BUFF] = { 0 };
	int len = 0;
	int type = 0;

#ifdef __EXTCTL_DEBUG_
	mavlink_log_info(&_extctl_mavlink_log_pub, "[extctl] start ....");
#endif

	while (!_extctl_should_exit)
	{
		if (extctl_protocal_read(buff, &len, &type))
		{
#ifdef __EXTCTL_DEBUG_
	mavlink_log_info(&_extctl_mavlink_log_pub, "[extctl] %d %d", len, type);
#endif
			p_handle = NULL;

			switch (type)
			{
				case DATA_TYPE_POS:
					p_handle = &extctl_pos_handle;
					break;
					
				case DATA_TYPE_SP:
					p_handle = &extctl_sp_handle;
					break;
					
				case DATA_TYPE_RC:
					p_handle = &extctl_rc_handle;
					break;
					
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
				p_handle(buff);
			}
		}
		
		usleep(DEV_RATE_READ);
	}
	
	//wait write thread exit.
	usleep(DEV_RATE_BASE);
	
	close(_serial_fd);
	
	return 0;
}

#ifdef __PX4_POSIX
#else
int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio, oldtio;
	//保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	memset(&newtio, 0, sizeof(newtio));
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
	switch (nSpeed)
	{
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;

		case 19200:
			cfsetispeed(&newtio, B19200);
			cfsetospeed(&newtio, B19200);
			break;

		case 38400:
			cfsetispeed(&newtio, B38400);
			cfsetospeed(&newtio, B38400);
			break;

		case 57600:
			cfsetispeed(&newtio, B57600);
			cfsetospeed(&newtio, B57600);
			break;

		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;

		case 230400:
			cfsetispeed(&newtio, B230400);
			cfsetospeed(&newtio, B230400);
			break;

		default:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
	}
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
#endif

