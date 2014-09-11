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
static void *handle_msg_buffer_empty(void *ptr);

static volatile int is_shooting, image_idx, image_count;
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char* argv[])
{
  pthread_t shooting_threads[MAX_PROCESSING_THREADS], buffer_thread;
  char c, image_buffer[MAX_IMAGE_BUFFERS][IMAGE_SIZE];
  int shooting_idx = 0;

  // Initialization
  printf("Starting\n");
  chdk_pipe_init();
  serial_init("/dev/ttySAC0");
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
      printf("Serial error: %d\n" ,errno);

    // Parse serial commands
    if (mora_protocol.msg_received)
    {
      // Process Only Once
      mora_protocol.msg_received = FALSE;

      // Shoot an image if not busy
      if (mora_protocol.msg_id == MORA_SHOOT)
        pthread_create(&shooting_threads[(shooting_idx++ % MAX_PROCESSING_THREADS)], NULL, handle_msg_shoot, NULL);

      // Fill the image buffer
      if (mora_protocol.msg_id == MORA_BUFFER_EMPTY)
        pthread_create(&buffer_thread, NULL, handle_msg_buffer_empty, NULL);

      mora_protocol.msg_received = FALSE;
    }

    // Read the socket
    pthread_mutex_lock(&mut);
    if (socket_recv(image_buffer[image_idx], IMAGE_SIZE) == IMAGE_SIZE) {
      image_idx = (image_idx + 1) % MAX_IMAGE_BUFFERS;

      if(image_count < MAX_IMAGE_BUFFERS)
        image_count++;
    }
    pthread_mutex_unlock(&mut);
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

  // Test if can shoot
  pthread_mutex_lock(&mut);
  if(is_shooting)
  {
    printf("Shooting: too fast\n");
    return NULL;
  }

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
  pthread_mutex_lock(&mut);
  int saved_img_idx = image_idx;
  // Check if image is available
  if(image_count <= 0)
  {
    pthread_mutex_unlock(&mut);
    printf("Buffer: no image available\n");
    return NULL;
  }
  pthread_mutex_unlock(&mut);

  // Need to send the image here

  pthread_mutex_lock(&mut);
  image_idx = (MAX_IMAGE_BUFFERS + image_idx - 1) % MAX_IMAGE_BUFFERS;
  image_count--;
  pthread_mutex_unlock(&mut);
}
