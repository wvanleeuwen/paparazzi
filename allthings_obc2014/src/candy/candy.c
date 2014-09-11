#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "serial.h"
#include "chdk_pipe.h"

#define MAX_FILENAME 512
#define MAX_PROCESSING_THREADS 5

static void *handle_msg_shoot(void *ptr);
static void *handle_msg_buffer_empty(void *ptr);

static int is_shooting;
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char* argv[])
{
  pthread_t shooting_threads[MAX_PROCESSING_THREADS], buffer_thread;
  char filename[MAX_FILENAME];
  int i;
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
      {
        for(i=0; i <MAX_PROCESSING_THREADS; i++)
        {
          if(pthread_kill(shooting_threads[i], 0) == 0)
          {
            pthread_create(&shooting_threads[i], NULL, handle_msg_shoot, NULL);
            break;
          }
        }
      }
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
  char filename[MAX_FILENAME];

  // Shoot the image
  pthread_mutex_lock(&mut);
  is_shooting = 1;
  pthread_mutex_unlock(&mut);

  printf("Shooting: start\n");
  chdk_pipe_shoot((char *)filename);
  printf("Shooting: got image %s\n", (char *)filename);

  pthread_mutex_lock(&mut);
  is_shooting = 0;
  pthread_mutex_unlock(&mut);

  //Parse the image
}

static void *handle_msg_buffer_empty(void *ptr)
{
  printf("Buffer: empty\n");
}
