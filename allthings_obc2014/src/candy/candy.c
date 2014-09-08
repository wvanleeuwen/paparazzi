#include <stdio.h>
#include <stdlib.h>

#include "serial.h"

int main(int argc, char* argv[])
{
  int i;
  char* buff = "This is a nice demo\n";

  printf("Start Shoot Server\n");
  //system("cd ../popcorn/chdkptp/lua/ && ../chdkptp -c -e'rec' -e'rs'");
  //for (i=0;i<10;i++)
  //  system("cd ../popcorn/chdkptp/lua/ && ../chdkptp -c -e'rs'");
  //printf("Ready\n");

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
        system("cd ../popcorn/chdkptp/lua/ && ../chdkptp -c -e'rec' -e'rs'");
      printf("%02X,(%d)\n",c, ret);
    }
  }


  // Close
  close(fd);

  printf("Ready!\n");
  return 0;
}

