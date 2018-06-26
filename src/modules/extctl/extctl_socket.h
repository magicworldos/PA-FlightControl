/*
 * server.h
 *
 *  Created on: Mar 11, 2016
 *      Author: lidq
 */

#ifndef _INCLUDE_SERVER_H_
#define _INCLUDE_SERVER_H_

#include "extctl_typedef.h"

#ifdef __PX4_POSIX

#define	CONFIG_BIND_WAIT_SECS		(60)
#define CONFIG_MAX_CLIENT_COUNT		(1)
#define CONFIG_MAX_ACCEPT_COUNT		(1)

#define SOCLET_NONBLOCK_COUNT		(100)
#define SOCLET_NONBLOCK_USLEEP		(1000)

typedef struct server
{
	//网络套接字
	int socket_descriptor;
	//套接字IP地址
	struct sockaddr_un servaddr;
	//连接信号量
	sem_t sem_engine;
	//连接信号量
	sem_t sem_connection;
	//连接信号量
	sem_t sem_new_accept;
	//服务器连接
	pthread_t thread_engine;
	//服务器连接
	pthread_t thread_conn;
} s_server;

//启动服务
int server_start(void);

//停止服务
void server_stop(void);

//设定非阻塞
bool make_socket_non_blocking(int sfd);

//创建新的等待连接
bool server_create_connection(void);

//关闭套接字
void server_close_socket(int socket);

void exception_handler(int sig);

void server_engine(void);

#endif

#endif /* INCLUDE_SERVER_H_ */

