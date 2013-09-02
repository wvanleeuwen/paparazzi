#ifndef SOCKET_H
#define SOCKET_H

#include <unistd.h>             /*  for ssize_t data type  */

#define LISTENQ        (1024)   /*  Backlog for listen()   */


/*  Function declarations  */
	int initSocket(unsigned int port) ;
	ssize_t Readline_socket(char * data, size_t maxlen);
	ssize_t Writeline_socket(char * text, size_t n);
	int Read_msg_socket(char * data, unsigned int size);
	ssize_t Write_msg_socket(char * data, unsigned int size);
	int closeSocket(void);

#endif  /*  SOCKET_H  */

