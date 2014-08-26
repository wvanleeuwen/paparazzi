#include <stdio.h>

#include "serial.h"

int main(int argc, char* argv[])
{
  char* buff = "This is a nice demo\n";

  // Open
  serial_init();

  // Send
  write(fd, buff, 10);

  // Close
  close(fd);

  printf("Ready!\n");
  return 0;
}

