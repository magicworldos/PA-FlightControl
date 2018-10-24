/*
 * process_test.c
 *
 *  Created on: Oct 22, 2018
 *      Author: lidq
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <drivers/drv_board_led.h>

static int _run = 0;

int process_test_main(int argc, char *argv[]);
int process_test_task(int argc, char *argv[]);
void* pth_run(void *arg);

int process_test_task(int argc, char *argv[])
{
	//初始化线程属性并设定栈内存1000字节
	pthread_attr_t pth_attr;
	pthread_attr_init(&pth_attr);
	pthread_attr_setstacksize(&pth_attr, 1000);
	//设置线程优先级为100
	struct sched_param pth_param;
	pthread_attr_getschedparam(&pth_attr, &pth_param);
	pth_param.sched_priority = 100;
	pthread_attr_setschedparam(&pth_attr, &pth_param);
	//创建并运行两个线程，参数分别为1和2
	pthread_t pth;
	pthread_create(&pth, &pth_attr, pth_run, (void *) 1);
	pthread_create(&pth, &pth_attr, pth_run, (void *) 2);
	pthread_attr_destroy(&pth_attr);

	int fd = open("/dev/led0", O_RDWR);
	int i = 0;
	while (_run)
	{
		if (i++ % 2)
		{
			ioctl(fd, LED_ON, 1);
		}
		else
		{
			ioctl(fd, LED_OFF, 1);
		}

		printf("Process: running.\n");
		sleep(1);
	}

	close(fd);

	return 0;
}

void* pth_run(void *arg)
{
	//设置线程名称
	char pth_name[20];
	snprintf(pth_name, 20, "pth_%d", getpid());
	prctl(PR_SET_NAME, pth_name, getpid());

	while (_run)
	{
		printf("Pthread: arg is %d\n", (int) arg);
		sleep(1);
	}
	return NULL;
}

int process_test_main(int argc, char *argv[])
{
	if (argc < 2)
	{
		return -1;
	}
	if (!strcmp(argv[1], "start"))
	{
		if (_run)
		{
			return -1;
		}
		_run = 1;
		px4_task_spawn_cmd("process_test", SCHED_DEFAULT, 100, 2048, (px4_main_t) process_test_task, (char * const *) argv);
		return 0;
	}
	if (!strcmp(argv[1], "stop"))
	{
		_run = 0;
	}
	return 0;
}
