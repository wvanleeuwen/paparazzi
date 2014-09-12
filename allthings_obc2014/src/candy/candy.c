#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>

#include "serial.h"
#include "chdk_pipe.h"
#include "protocol.h"

#define MAX_FILENAME 512
#define MAX_PROCESSING_THREADS 25
#define MAX_IMAGE_BUFFERS 25
#define IMAGE_SIZE 70
#define SODA "/root/develop/allthings_obc2014/src/soda/soda"

static void *handle_msg_shoot(void *ptr);
static inline void handle_msg_buffer(void);

static volatile int is_shooting, image_idx, image_count;
static char image_buffer[MAX_IMAGE_BUFFERS][IMAGE_SIZE];
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char* argv[])
{
  pthread_t shooting_threads[MAX_PROCESSING_THREADS];
  char c;
  int shooting_idx = 0;

  // Initialization
  printf("CANDY:\tStarting\n");
  chdk_pipe_init();
  int ret = serial_init("/dev/ttySAC0");
  if (ret < 0)
  {
    printf("CANDY:\tfailed to open /dev/ttySAC0\n");
    return -1;
  }
  pthread_mutex_init(&mut, NULL);
  socket_init(1);

  // Initial settings
  is_shooting = 0;
  mora_protocol.status = 0;
  image_idx = 0;
  image_count = 0;

  // MAIN loop
  while (1)
  {

    // Read the serial
    if (read(fd, &c, 1) > 0)
      parse_mora(&mora_protocol, c);
    else if (errno != 11)
      printf("CANDY:\nSerial error: %d\n" ,errno);

    // Parse serial commands
    if (mora_protocol.msg_received)
    {
      // Process Only Once
      mora_protocol.msg_received = FALSE;

      // Shoot an image if not busy
      if (mora_protocol.msg_id == MORA_SHOOT)
      {
        int i;
        union dc_shot_union shoot;
        for (i=0; i<MORA_SHOOT_MSG_SIZE;i++)
          shoot.bin[i] = mora_protocol.payload[i];
        printf("CANDY:\tSHOT %d,%d\n",shoot.data.nr,shoot.data.phi);
        pthread_create(&shooting_threads[(shooting_idx++ % MAX_PROCESSING_THREADS)], NULL, handle_msg_shoot, (void*)shooting_idx);
      }

      // Fill the image buffer (happens busy because needs fd anyway)
      if (mora_protocol.msg_id == MORA_BUFFER_EMPTY)
        handle_msg_buffer();
    }

    // Read the socket
    if (socket_recv(image_buffer[image_idx], IMAGE_SIZE) == IMAGE_SIZE) {
      image_idx = (image_idx + 1) % MAX_IMAGE_BUFFERS;

      if(image_count < MAX_IMAGE_BUFFERS)
        image_count++;
    }

  }

  // Close
  close(fd);
  chdk_pipe_deinit();

  printf("CANDY:\tShutdown\n");
  return 0;
}

static void *handle_msg_shoot(void *ptr)
{
  printf("CANDY:\thandle_msg_shoot\n");
  char filename[MAX_FILENAME], soda_call[512];

  // Test if can shoot
  pthread_mutex_lock(&mut);
  if(is_shooting)
  {
    printf("CANDY-%p:\tShooting: too fast\n",ptr);
    pthread_mutex_unlock(&mut);
    return NULL;
  }


  is_shooting = 1;
  pthread_mutex_unlock(&mut);

  printf("CANDY-%p:\tShooting: start\n",ptr);
  chdk_pipe_shoot(filename);
  printf("CANDY-%p:\tShooting: got image %s\n", ptr, filename);

  pthread_mutex_lock(&mut);
  is_shooting = 0;
  pthread_mutex_unlock(&mut);

  //Parse the image
  sprintf(soda_call, "%s %s", SODA, filename);
  int ret = system(soda_call);
  printf("CANDY-%p:\tShooting: soda return %d of image %s\n", ptr, ret, filename);
}

static inline void handle_msg_buffer(void)
{
  printf("CANDY:\thandle_msg_buffer\n");
  int i;

  // Check if image is available
  if(image_count <= 0)
    printf("CANDY:\tBuffer: no image available\n");
  else {
    // Send the image
    image_idx = (MAX_IMAGE_BUFFERS + image_idx - 1) % MAX_IMAGE_BUFFERS;
    image_count--;
    
    MoraHeader(MORA_PAYLOAD, MORA_PAYLOAD_MSG_SIZE);
    for(i = 0; i < IMAGE_SIZE; i++)
      MoraPutUint8(image_buffer[image_idx][i]);
    MoraTrailer();
  }
}
