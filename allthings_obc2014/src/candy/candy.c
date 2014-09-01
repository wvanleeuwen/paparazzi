#include <stdio.h>
#include <stdlib.h>

#include "serial.h"

int main(int argc, char* argv[])
{
  int i;
  char* buff = "This is a nice demo\n";

  printf("Start Shoot\n");

  system("cd ../popcorn/chdkptp/lua/ && ../chdkptp -c -e'rec' -e'rs'");
  for (i=0;i<10;i++)
    system("cd ../popcorn/chdkptp/lua/ && ../chdkptp -c -e'rs'");


  printf("Ready\n");

  // Open
  serial_init();

  // Send
  write(fd, buff, 10);

  // Close
  close(fd);

  printf("Ready!\n");
  return 0;
}

