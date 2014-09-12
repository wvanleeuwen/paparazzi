#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jpeg_decode.h"
#include "rgb2hsv.h"
#include "color_probs.h"
#include "../candy/socket.h"

#define IMG_HEIGHT 3000
#define IMG_WIDTH 4000
#define IMG_SIZE_BW (IMG_HEIGHT*IMG_WIDTH)
#define IMG_SIZE (IMG_HEIGHT*IMG_WIDTH*3)

int main(int argc, char* argv[])
{
  char filename[512];

  unsigned char* rgb = new unsigned char [IMG_SIZE];
  unsigned char* hsv = new unsigned char [IMG_SIZE];
  unsigned char* prob_yellow = new unsigned char [IMG_SIZE_BW];
  unsigned char* prob_blue = new unsigned char [IMG_SIZE_BW];;

  char  outfile[1024];

  strcpy(filename, "/root/IMG_0200.jpg");

  printf("SODA:\tSuperb Onboard Recognition Application\n");

  //////////////////////////////////////////////
  // Load Jpeg

  if (argc >= 2)
  {
    strcpy(filename, argv[1]);
  }

  int w,h;
  int ret = loadJpg(filename, rgb, &w, &h);
  if (ret <= 0)
  {
    fprintf(stderr, "SODA:\tFailed to load '%s'\n",filename);
    return -1;
  }

  printf("SODA:\tJPEG '%s' Loaded: W x H = %d x %d\n",filename, w, h);

  //////////////////////////////////////////////
  // Convert to HSV
  unsigned char* p = rgb;
  unsigned char* q = hsv;
  unsigned char* end = rgb + IMG_SIZE;
  for (;p<end;p+=3)
  {
    RgbToHsvP( p, q);
    q+=3;
  }

  //////////////////////////////////////////////
  // Fill two additional images with probabilities
  q = hsv;
  end = hsv + IMG_SIZE;
  unsigned char* yellow = prob_yellow;
  unsigned char* blue = prob_blue;
  for(;q < end; q +=3, yellow++, blue++)
  {
    (*yellow) = get_prob_color(q, probabilities_yellow, sat_yellow, val_yellow, threshold_saturation_yellow, threshold_value_low_yellow, threshold_value_high_yellow);
    (*blue) = get_prob_color(q, probabilities_blue, sat_blue, val_blue, threshold_saturation_blue, threshold_value_low_blue, threshold_value_high_blue);

  }

  //////////////////////////////////////////////
  // Sum probabilities over a Joe-sized window:
  
  // first make an integral image of blue and yellow
  // then determine the window sums with a certain step size
  

  int joe_x = 2000;
  int joe_y = 1500;

  #define THUMB_W  32
  #define THUMB_SIZE	(THUMB_W*THUMB_W*3)

  //////////////////////////////////////////////
  // Create Thumbnail:

  unsigned char thumb[THUMB_SIZE];

  // Fix borders
  if ((joe_x+THUMB_W/2) >  IMG_WIDTH)
    joe_x = IMG_WIDTH - THUMB_W/2;
  if ((joe_x-THUMB_W/2) < 0)
    joe_x = THUMB_W/2;
  if ((joe_y+THUMB_W/2) >  IMG_HEIGHT)
    joe_y = IMG_HEIGHT - THUMB_W/2;
  if ((joe_y-THUMB_W/2) < 0)
    joe_y = THUMB_W/2;

  // source
  p = rgb + ((joe_y-THUMB_W/2) * IMG_WIDTH * 3) + ((joe_x-THUMB_W/2) * 3);
  // dest
  q = thumb;

  // create thumbnail
  for (int x=0;x<THUMB_W;x++)
  {
    for (int y=0;y<THUMB_W;y++)
    {
      // Copy thumbnail
      *q = *p;
      q++;p++;
    }
    // Skip remainder of the source line
    p += (IMG_WIDTH-THUMB_W) * 3;
  }

  // compress thumbnail


  //////////////////////////////////////////////
  // Send resulting thumbnail to CANDY:

  socket_init(0);

  socket_send((char*)thumb, 70);

  strcpy(outfile,filename);
  strcat(outfile,".txt");
  FILE* fp = fopen(outfile, "w+b");
  fprintf(fp, "processed  %s \n", filename);
  fclose(fp);

  return 0;
}

