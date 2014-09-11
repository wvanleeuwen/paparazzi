#include <stdio.h>
#include <stdlib.h>

#include "serial.h"
#include "chdk_pipe.h"

#define MAX_FILENAME 512

int main(int argc, char* argv[])
{
  int i;
  char filename[MAX_FILENAME];
  char* buff = "This is a nice demo\n";

  printf("Start Shoot Server\n");
  chdk_pipe_init();

  // Open
  serial_init("/dev/ttySAC0");

  // Send
  write(fd, buff, 10);

  for (;;)
  {
    char c;
    int ret = read(fd, &c, 1);
    if (ret != -1)
    {
      if (c == 0x20)
      {
        chdk_pipe_shoot(filename);
        printf("Shot image: %s\n", filename);
      }
      //printf("%02X,(%d)\n",c, ret);
    }
  }


  // Close
  close(fd);
  chdk_pipe_deinit();

  printf("Ready!\n");
  return 0;
}

