#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jpeg_decode.h"
#include "rgb2hsv.h"
#include "color_probs.h"


int main(int argc, char* argv[])
{
  char filename[512];

  unsigned char* buff = new unsigned char [4000*3000*3];
  unsigned char* hsv = new unsigned char [4000*3000*3];
  unsigned char* prob_yellow = new unsigned char [4000*3000];
  unsigned char* prob_blue = new unsigned char [4000*3000];

  char  outfile[1024];

  strcpy(filename, "/root/IMG_0200.jpg");

  printf("Start Superb Onboard Recognition Application\n");

  //////////////////////////////////////////////
  // Load Jpeg

  if (argc >= 2)
  {
    strcpy(filename, argv[1]);
  }

  int ret = loadJpg(filename, buff);
  if (ret <= 0)
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

  //////////////////////////////////////////////
  // Fill two additional images with probabilities
  q = hsv;
  end = hsv + 4000*3000*3;
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
  

  strcpy(outfile,filename);
  strcat(outfile,".txt");

  FILE* fp = fopen(outfile, "w+b");
  fprintf(fp, "processed  %s \n", filename);
  fclose(fp);

  printf("Ready!\n");
  return 0;
}

