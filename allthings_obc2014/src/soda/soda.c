#include <stdio.h>
#include <stdlib.h>
#include "jpeg_decode.h"
#include "rgb2hsv.h"

int main(int argc, char* argv[])
{
  unsigned char* buff = new unsigned char [4000*3000*3];
  unsigned char* hsv = new unsigned char [4000*3000*3];

  printf("Start Superb Onboard Recognition Application\n");

  loadJpg("/root/IMG_0263.jpg", buff);

  printf("JPEG Loaded\n");

  for (int i=0;i<10;i++)
  {
    printf("%d:\n",i);
    for (int x=0;x<4000;x++)
    {
      for (int y=0;y<3000;y++)
      {
        RgbToHsvP( buff+12000*y+3*x, hsv+12000*y+3*x);
      }
    }
  }

  printf("Ready!\n");
  return 0;
}

