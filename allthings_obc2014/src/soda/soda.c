#include <stdio.h>
#include <stdlib.h>
#include "jpeg_decode.h"
#include "rgb2hsv.h"

int main(int argc, char* argv[])
{
  char* filename = "/root/IMG_0200.jpg";

  unsigned char* buff = new unsigned char [4000*3000*3];
  unsigned char* hsv = new unsigned char [4000*3000*3];

  printf("Start Superb Onboard Recognition Application\n");

  //////////////////////////////////////////////
  // Load Jpeg

  if (argc >= 2)
  {
    filename = argv[1];
  }

  int ret = loadJpg(filename, buff);
  if (ret < 0)
  {
    fprintf(stderr, "Failed to load '%s'\n",filename);
    return -1;
  }

  printf("JPEG '%s' Loaded\n",filename);

  //////////////////////////////////////////////
  // Convert to HSV
  unsigned char* p = buff;
  unsigned char* q = hsv;
  unsigned char* end = buff + 4000*3000*3;
  for (;p<end;p+=3)
  {
    RgbToHsvP( p, q);
    q+=3;
  }

  

  printf("Ready!\n");
  return 0;
}

