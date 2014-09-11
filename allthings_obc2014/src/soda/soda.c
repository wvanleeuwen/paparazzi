#include <stdio.h>
#include <stdlib.h>
#include "jpeg_decode.h"
#include "rgb2hsv.h"

int main(int argc, char* argv[])
{
  unsigned char* buff = new unsigned char [4000*3000*3];
  unsigned char* hsv = new unsigned char [4000*3000*3];

  printf("Start Superb Onboard Recognition Application\n");

  loadJpg("/root/IMG_0269.jpg", buff);

  printf("JPEG Loaded\n");

  for (int i=0;i<10;i++)
  {
    printf("%d:\n",i);
    unsigned char* p = buff;
    unsigned char* q = hsv;
    unsigned char* end = buff + 4000*3000*3;
    //for (int x=0;x<(4000*3000*3);x+=3)
    for (;p<end;p+=3)
    {
      //for (int y=0;y<3000;y++)
      //{
        RgbToHsvP( p, q);
        q+=3;
      //}
    }
  }

  printf("Ready!\n");
  return 0;
}

