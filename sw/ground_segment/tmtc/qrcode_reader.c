#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>


static void on_Debug(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  int i;
  guint ac_id = atoi(argv[0]);
  printf("QRCODE AC=%d len=%d ",ac_id,argc);

  // Split the string
  char buffer[16];
  int buffer_idx;
  for (i=0;i<strlen(argv[1]);i++)
  {
    if(argv[1][i] == '\0' || argv[1][i] == ' ' || argv[1][i] == ',') {
      buffer[buffer_idx] = 0;
      int v = atoi(buffer);
      char c =  (char) v;
      printf("%c",c);

      buffer_idx = 0;
    }

    buffer[buffer_idx] = argv[1][i];
    buffer_idx++;
  }
  printf("\n");

  fflush(stdout);
}


int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("Example1", "Example1 READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Debug, NULL, "^(\\S*) DEBUG (\\S*)");

  IvyStart("127.255.255.255");

  g_main_loop_run(ml);

  return 0;
}

