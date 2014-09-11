/* Sample UDP server */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h> 
#include <string.h>
#include "socket.h"


#define SOCKET_PORT 32000

static int socket_fd, socket_is_server;
static struct sockaddr_in socket_server, socket_client;

void socket_init(int is_server)
{
   socket_is_server = is_server;

   // Initialize socket
   if((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
   {
      perror("Socket: socket");
      exit(1);
   }

   bzero(&socket_server,sizeof(socket_server));
   socket_server.sin_family = AF_INET;
   socket_server.sin_port = htons(SOCKET_PORT);
   inet_aton("127.0.0.1", &socket_server.sin_addr);

   if(is_server)
   {
      if(bind(socket_fd, (struct sockaddr *)&socket_server, sizeof(socket_server)) <= 0)
      {
         perror("Socket: bind");
         exit(1);
      }
   }
}

int socket_recv(char *buffer, int len)
{
   if(socket_is_server)
      return recvfrom(socket_fd, buffer, len, 0, (struct sockaddr*)&socket_client, sizeof(socket_client));

   return recvfrom(socket_fd, buffer, len, 0, (struct sockaddr*)&socket_Server, sizeof(socket_server));
}

int socket_send(char *buffer, int len)
{
   if(socket_is_server)
      return sendto(socket_fd, buffer, len, 0, (struct sockaddr*)&socket_client, sizeof(socket_client));

   return sendto(socket_fd, buffer, len, 0, (struct sockaddr*)&socket_server, sizeof(socket_server));
}
