#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jpeg_decode.h"
#include "jpeg_encode.h"
#include "rgb2hsv.h"
#include "color_probs.h"
#include "../lib/udp/socket.h"
#include "integral_image.h"
#include "../candy/protocol.h"

#define IMG_HEIGHT 3000
#define IMG_WIDTH 4000
#define IMG_SIZE_BW (IMG_HEIGHT*IMG_WIDTH)
#define IMG_SIZE (IMG_HEIGHT*IMG_WIDTH*3)

int main(int argc, char* argv[])
{
  union dc_shot_union shotinfo;
  char filename[512];
  char  outfile[1024];

  unsigned char* rgb = new unsigned char [IMG_SIZE];
  unsigned char* hsv = new unsigned char [IMG_SIZE];
  unsigned char* prob_yellow = new unsigned char [IMG_SIZE_BW];
  unsigned char* prob_blue = new unsigned char [IMG_SIZE_BW];
  unsigned long* integral_yellow = new unsigned long [IMG_SIZE_BW];
  unsigned long* integral_blue = new unsigned long [IMG_SIZE_BW];
  unsigned long* sum_yellow = new unsigned long [IMG_SIZE_BW];
  unsigned long* sum_blue = new unsigned long [IMG_SIZE_BW];
  unsigned long* prob_joe = new unsigned long [IMG_SIZE_BW];

  int imgnr=0;

  //strcpy(filename, "./test_images/IMG_0254.jpg");
  //strcpy(filename, "./test_images/test_image.jpg");
  strcpy(filename, "./test_images/IMG.jpg");

  printf("SODA:\tSuperb Onboard Recognition Application\n");

  //////////////////////////////////////////////
  // Load Jpeg

  if (argc >= 2)
  {
    strcpy(filename, argv[1]);
  }

  printf("SODA: ");
  for (int i = 0; (i < (argc-2)) && (i < 10); i++)
  {
    int val = atof(argv[i+2]);
    printf("%d, ",val);
    shotinfo.i[i] = val;
  }
  printf("\n");

  int nr = shotinfo.data.nr;
  printf("SODA:\tImage nr: %d\n",nr);

  int x,y;
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

  // @Christophe: code is not yet optimized for memory / speed... but let's first see if it works.

  //////////////////////////////////////////////
  // Fill two additional images with probabilities
  printf("SODA:\tCalculate probabilities\n");
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
  printf("SODA:\tCalculate integral images\n");
  int joe_size = 60; // @Christophe: should best come from autopilot!

  // Get integral images, which allow to make window sums with 4 operations:
  get_integral_image(prob_yellow, integral_yellow, IMG_HEIGHT, IMG_WIDTH);
  get_integral_image(prob_blue, integral_blue, IMG_HEIGHT, IMG_WIDTH);

  // step determines at how many locations we look:
  // we can adapt this to the required computational speed
  printf("SODA:\tSum and multiply\n");
  int step = 1;
  for(x = 0; x < IMG_WIDTH - joe_size; x++)
  {
    for(y = 0; y < IMG_HEIGHT - joe_size; y++)
    {
      // REMARK: not sure whether it is y + x * IMG_HEIGHT or y * IMG_WIDTH + x... (change also in integral_image.c if necessary)
      sum_yellow[y * IMG_WIDTH + x] = integral_yellow[y * IMG_WIDTH + x] + integral_yellow[(y+joe_size) * IMG_WIDTH + (x+joe_size)]
                    - integral_yellow[(y+joe_size) * IMG_WIDTH + x] - integral_yellow[y * IMG_WIDTH + (x+joe_size)];
      sum_blue[y * IMG_WIDTH + x] = integral_blue[y * IMG_WIDTH + x] + integral_blue[(y+joe_size) * IMG_WIDTH + (x+joe_size)]
                    - integral_blue[(y+joe_size) * IMG_WIDTH + x] - integral_blue[y * IMG_WIDTH + (x+joe_size)];
      prob_joe[y * IMG_WIDTH + x] = sum_yellow[y * IMG_WIDTH + x] * sum_blue[y * IMG_WIDTH + x];
    }
  }  

  // find maximum (only one for now):
  printf("SODA:\tFind maximum\n");
  unsigned long maximum = 0;
  int joe_x = 0;
  int joe_y = 0;
  for(x = 0; x < IMG_WIDTH - joe_size; x++)
  {
    for(y = 0; y < IMG_HEIGHT - joe_size; y++)
    {
      if(prob_joe[y * IMG_WIDTH + x] > maximum)
      {
        maximum = prob_joe[y * IMG_WIDTH + x];
        joe_x = x;
        joe_y = y;
      }
    }
  }

  // if no yellow and blue, send just yellow
  if(maximum == 0) 
  {
    printf("SODA:\tJust yellow...\n");
    for(x = 0; x < IMG_WIDTH - joe_size; x++)
    {
      for(y = 0; y < IMG_HEIGHT - joe_size; y++)
      {
        if(sum_yellow[y * IMG_WIDTH + x] > maximum)
        {
          maximum = sum_yellow[y * IMG_WIDTH + x];
          joe_x = x;
          joe_y = y;
        }
      }
    }
  }

  joe_x += joe_size/2;
  joe_y += joe_size/2;  
  
  printf("SODA: Joe: (x,y) = (%d, %d) max=%lu\n", joe_x, joe_y, maximum);

  #define THUMB_W  128
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

  // create thumbnail buffer needed by jpeg
  for (int x=0;x<THUMB_W;x++)
  {
    for (int y=0;y<THUMB_W;y++)
    {
      // Copy thumbnail
      *q++ = *p++;
      *q++ = *p++;
      *q++ = *p++;

      /*
          // Convert to 256 colors web pallete
          unsigned char web = tr * 6 / 256 * 36;
          web += tg * 6 / 256 * 6;
          web += tb * 6 / 256;
          *q = web;
          q++;
       */
    }
    // Skip remainder of the source line
    p += (IMG_WIDTH-THUMB_W) * 3;
  }

  ////////////////////////////////////////////////
  // compress thumbnail & store
  strcpy(outfile,filename);
  strcat(outfile,".thumb.jpg");
  int jret = storeJpg(outfile, thumb, THUMB_W, THUMB_W, 25);

  //////////////////////////////////////////////
  // Send resulting thumbnail to CANDY:

  socket_init(0);

  // socket_send((char*)thumb, 70);

  unsigned char * j = jpeg_start;
  int size = jpeg_end - jpeg_start;
  char blocks = (size+63)/64;
  int score = maximum / 200000;
  if (score > 255)
    score = 255;

  // Geo Info:
  {
    char buff[70];
    // Packet header
    buff[0] = imgnr;
    buff[1] = 255;
    buff[2] = blocks;
    buff[3] = 0;
    buff[4] = THUMB_W;
    buff[5] = (char) ((unsigned char) score);

    uint16_t val;
    val = joe_x;
    buff[6] = (val & 0x00ff);
    buff[7] = (val >> 8);
    val = joe_x;
    buff[6] = (val & 0x00ff);
    buff[7] = (val >> 8);
    for (int i=0; i < MORA_SHOOT_MSG_SIZE; i++)
      buff[i+10] = shotinfo.bin[i];
    socket_send(buff,70);
  }

  for (int b = blocks-1; b >= 0; b--)
  {
    char buff[70];
    // Packet header
    buff[0] = imgnr;
    buff[1] = b;
    buff[2] = blocks;
    buff[3] = (b == (blocks-1)? size%64 : 64);
    buff[4] = THUMB_W;
    buff[5] = (char) ((unsigned char) score);

    for (int i=6;i<(64+6);i++)
    {
      buff[i] = (char) j[b*64+i-6];

      //if (cnt == 1)
      //  buff[i] = 0;
    }
    socket_send(buff,70);
    size -= 64;
    printf("Sent: %x %x %x ...\n", buff[0], buff[1], buff[2]);
  }

  //////////////////////////////////////////////
  // Log onboard:

  strcpy(outfile,filename);
  strcat(outfile,".txt");
  FILE* fp = fopen(outfile, "w+b");
  fprintf(fp, "processed  %s \n", filename);
  fprintf(fp, "Joe: (x,y) = (%d, %d) max=%lu\n", joe_x, joe_y, maximum);
  fclose(fp);

  return 0;
}

