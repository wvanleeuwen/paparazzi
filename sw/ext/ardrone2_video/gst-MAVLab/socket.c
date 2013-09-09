#include "socket.h"
#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */
#include <errno.h>
#include <string.h> 		/* memset */

#include <stdlib.h>
#include <stdio.h>
#include <gst/gst.h>



/*  Global constants  */

#define MAX_LINE           (1000)

/*  Global variables  */

int       conn_s;                /*  connection socket         */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char     *endptr;                /*  for strtol()              */


int closeSocket(void) {
	return close(conn_s);
}

int initSocket(unsigned int tcpport) {
	int       list_s;                /*  listening socket          */
    /*  Create the listening socket  */
    if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
		fprintf(stderr, "tcp server: Error creating listening socket.\n");
		return 0;
    }


    /*  Set all bytes in socket address structure to
        zero, and fill in the relevant data members   */

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port        = htons(tcpport);

	/*  Bind our socket addresss to the 
	listening socket, and call listen()  */

    if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 ) {
	fprintf(stderr, "tcp server: Error calling bind()\n");
	exit(EXIT_FAILURE);
    }

    if ( listen(list_s, LISTENQ) < 0 ) {
	fprintf(stderr, "tcp server: Error calling listen()\n");
	exit(EXIT_FAILURE);
    }

	
	
	/*  Wait for a connection, then accept() it  */	
	if ( (conn_s = accept(list_s, NULL, NULL) ) < 0 ) {
	    fprintf(stderr, "tcp server: Error calling function accept()\n");
	    return 0;
	}
	return 1;
}

/*  Read a line from a socket  */

ssize_t Readline_socket(char * data, size_t maxlen) {
    size_t n, rc;
    char   *inbuffer, c;

	inbuffer = data ;

    for ( n = 1; n <= maxlen; n++ ) {
	
	if ( (rc = read(conn_s, &c, 1)) == 1 ) {
	    inbuffer[n] = c;
	}
	else if ( rc == 0 ) {
	    if ( n == 1 )
		return 0;
	    else
		break;
	}
	else {
	    if ( errno == EINTR )
		continue;
	    return -1;
	}
    }

    //*inbuffer = 0;
    return n;
}


/*  Write a line to a socket  */

ssize_t Writeline_socket(char * text, size_t n) {
    size_t      nleft;
    ssize_t     nwritten;
	nleft  = n;
	
    while ( nleft > 0 ) {
	if ( (nwritten = write(conn_s, text, nleft)) <= 0 ) {
	    if ( errno == EINTR )
		nwritten = 0;
	    else
		return -1;
	}
	nleft  -= nwritten;
	text += nwritten;
    }

    return n;
	
}

/*  Read a line from a socket  */

int Read_msg_socket(char * data, unsigned int size) {
	int n;
	n = read(conn_s, data, size);
    return n;

}


/*  Write a line to a socket  */

ssize_t Write_msg_socket(char * data, unsigned int size) {
    size_t      nleft;
    ssize_t     nwritten = 0;
	nleft  = size;
	
    while ( nleft > 0 ) {
	if ( (nwritten = write(conn_s, data, nleft)) <= 0 ) {
	    if ( errno == EINTR )
		nwritten = 0;
	    else
		return -1;
	}
	nleft  -= nwritten;
	data += nwritten;
    }

    return nwritten;
	
}





