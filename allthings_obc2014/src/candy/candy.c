#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "serial.h"
#include "chdk_pipe.h"

#define MAX_FILENAME 512
#define MAX_PROCESSING_THREADS 5
#define SODA "/root/develop/allthings_obc2014/src/soda/soda"

static void *handle_msg_shoot(void *ptr);
static void *handle_msg_buffer_empty(void *ptr);

static int is_shooting;
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char* argv[])
{
  pthread_t shooting_threads[MAX_PROCESSING_THREADS], buffer_thread;
  int shooting_idx = 0;
  char c;

  // Initialization
  printf("Starting\n");
  chdk_pipe_init();
  serial_init("/dev/ttySAC0");
  pthread_mutex_init(&mut, NULL);
  is_shooting = 0;

  // MAIN loop
  while (1)
  {
    if (read(fd, &c, 1) != -1)
    {

      // Shoot an image if not busy
      pthread_mutex_lock(&mut);
      if (c == 0x20 && !is_shooting)
        pthread_create(&shooting_threads[(shooting_idx++ % MAX_PROCESSING_THREADS)], NULL, handle_msg_shoot, NULL);
      else if(c == 0x20)
        printf("Shooting: busy\n");
      pthread_mutex_unlock(&mut);

      // Fill the image buffer
      if (c == 0x60)
        pthread_create(&buffer_thread, NULL, handle_msg_buffer_empty, NULL);
    }
  }

  // Close
  close(fd);
  chdk_pipe_deinit();

  printf("Shutdown\n");
  return 0;
}

static void *handle_msg_shoot(void *ptr)
{
  char filename[MAX_FILENAME], soda_call[512];

  // Shoot the image
  pthread_mutex_lock(&mut);
  is_shooting = 1;
  pthread_mutex_unlock(&mut);

  printf("Shooting: start\n");
  chdk_pipe_shoot(filename);
  printf("Shooting: got image %s\n", filename);

  pthread_mutex_lock(&mut);
  is_shooting = 0;
  pthread_mutex_unlock(&mut);

  //Parse the image
  sprintf(soda_call, "%s %s", SODA, filename);
  int ret = system(soda_call);
  printf("Shooting: soda return %d of image %s\n", ret, filename);
}

static void *handle_msg_buffer_empty(void *ptr)
{
  printf("Buffer: empty\n");
}
