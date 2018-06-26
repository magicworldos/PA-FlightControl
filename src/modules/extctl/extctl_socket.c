/*
 * server.c
 *
 *  Created on: Mar 11, 2016
 *      Author: lidq
 */

#include "extctl_socket.h"

int _socket_id = -1;

#ifdef __PX4_POSIX

static s_server server;
static uint64_t main_pthread = 0;
static pthread_t pthread_master_id;

int server_start(void)
{
	printf("[ OK ] Initialize server start.\n");

	sem_init(&server.sem_engine, 0, 1);
	//服务器允许客户端接入的最大连接数
	sem_init(&server.sem_connection, 0, CONFIG_MAX_CLIENT_COUNT);
	//允许新客户端接入信号量
	sem_init(&server.sem_new_accept, 0, CONFIG_MAX_ACCEPT_COUNT);

	unlink(UNIX_DOMAIN);

	//套接字
	server.socket_descriptor = socket(PF_UNIX, SOCK_STREAM, 0);
	if (server.socket_descriptor <= 0)
	{
		printf("[ ERROR ] Socket_descriptor.\n");
		return -1;
	}

	make_socket_non_blocking(server.socket_descriptor);

	//服务地址与端口
	memset(&server.servaddr, 0, sizeof(server.servaddr));
	server.servaddr.sun_family = PF_UNIX;
	strcpy(server.servaddr.sun_path, UNIX_DOMAIN);

	//绑定套接字
	int bind_status = -1;
	for (int i = 0; bind_status == -1 && i < CONFIG_BIND_WAIT_SECS; i++)
	{
		bind_status = bind(server.socket_descriptor, (struct sockaddr*) &server.servaddr, sizeof(server.servaddr));
		if (bind_status == 0)
		{
			break;
		}
		unlink(UNIX_DOMAIN);
		printf("[ RUNNING ] Waiting for host port ready... %d/%ds\n", (i + 1), CONFIG_BIND_WAIT_SECS);
		sleep(1);
	}
	if (bind_status == 0)
	{
		printf("[ OK ] Bind host port.\n");
	}
	else
	{
		printf("[ ERROR ] Bind host port.\n");
		return -1;
	}

	//开始监听
	int listen_id = listen(server.socket_descriptor, CONFIG_MAX_CLIENT_COUNT);
	if (listen_id == -1)
	{
		printf("[ ERROR ] Listen host connection.\n");
		return -1;
	}

	main_pthread = pthread_self();

	signal(SIGSEGV, exception_handler);
	signal(SIGSTKFLT, exception_handler);
	signal(SIGPIPE, exception_handler);

	printf("[ OK ] Server is ready.\n");

	//循环创建新的空闲连接
	while (true)
	{
		sem_wait(&server.sem_engine);

		sem_close(&server.sem_connection);
		sem_init(&server.sem_connection, 0, CONFIG_MAX_CLIENT_COUNT);
		sem_close(&server.sem_new_accept);
		sem_init(&server.sem_new_accept, 0, CONFIG_MAX_ACCEPT_COUNT);

		int t = pthread_create(&server.thread_engine, NULL, (void*) &server_engine, NULL);
		if (t < 0)
		{
			printf("pthread_create call error, erro: [%d][%s]\n", errno, strerror(errno));
			sleep(1);
		}
	}

	printf("Server Stop.\n");

	return 1;
}

//停止服务
void server_stop(void)
{
	_socket_id = -1;
	if (server.socket_descriptor > 0)
	{
		//关闭套接字
		int st = close(server.socket_descriptor);
		printf("[ OK ] Close socket descriptor... %d\n", st);
	}
}

bool make_socket_non_blocking(int sfd)
{
	int flags, s;

	flags = fcntl(sfd, F_GETFL, 0);
	if (flags == -1)
	{
		printf("fcntl error\n");
		return false;
	}

	flags |= O_NONBLOCK;
	s = fcntl(sfd, F_SETFL, flags);
	if (s == -1)
	{
		printf("fcntl error\n");
		return false;
	}

	return true;
}

void server_engine(void)
{
	pthread_master_id = pthread_self();
	//循环创建新的空闲连接
	while (true)
	{
		//最多允许接入新客户端数
		sem_wait(&server.sem_connection);
		sem_wait(&server.sem_new_accept);
		//创建新的等待连接
		int t = pthread_create(&server.thread_conn, NULL, (void*) &server_create_connection, NULL);
		if (t < 0)
		{
			printf("pthread_create call error, erro: [%d][%s]\n", errno, strerror(errno));
			sem_post(&server.sem_new_accept);
			sem_wait(&server.sem_connection);
			sleep(1);
		}
	}
}

//创建新的等待连接
bool server_create_connection(void)
{
	fd_set connfd;
	FD_ZERO(&connfd);
	FD_SET(server.socket_descriptor, &connfd);

	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;

	int sret = select(server.socket_descriptor + 1, &connfd, NULL, NULL, &timeout);
	if (sret < 0)
	{
		sem_post(&server.sem_new_accept);
		server_close_socket(-1);
		return false;
	}

	if (sret == 0)
	{
		sem_post(&server.sem_new_accept);
		server_close_socket(-1);
		return false;
	}

	if (!FD_ISSET(server.socket_descriptor, &connfd))
	{
		sem_post(&server.sem_new_accept);
		server_close_socket(-1);
		return false;
	}

	//等待接入
	int socket = -1;
	for (int i = 0; socket <= 0 && i < SOCLET_NONBLOCK_COUNT; i++)
	{
		socket = accept(server.socket_descriptor, (struct sockaddr*) NULL, NULL);
		usleep(SOCLET_NONBLOCK_USLEEP);
	}

	//一但有客户端接入，释放new_appept信号量，允许创建一个新的空闲连接
	sem_post(&server.sem_new_accept);
	if (socket == -1)
	{
		server_close_socket(socket);
		return false;
	}

	make_socket_non_blocking(socket);
	_socket_id = socket;
	printf("Accept socket %d\n", _socket_id);

	return true;
}

//关闭套接字
void server_close_socket(int socket)
{
	close(socket);
	sem_post(&server.sem_connection);
}

void exception_handler(int sig)
{
	if (pthread_self() == pthread_master_id)
	{
		printf("Socket Engine Exception %d\n", sig);
		pthread_detach(pthread_self());
		sem_post(&server.sem_engine);
		pthread_exit(NULL);
	}
	else
	{
		printf("Release socket %d\n", _socket_id);
		server_close_socket(_socket_id);
		_socket_id = -1;
	}
}

#endif
