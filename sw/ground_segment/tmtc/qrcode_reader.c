#include <glib.h>
#include <stdio.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>


static void on_Debug(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  int i;
  guint ac_id = atoi(argv[0]);
  printf("QRCODE AC=%d len=%d ",ac_id,argc);
  for (i=1;i<argc;i++)
  {
    int v = atoi(argv[i]);
    char c =  (char) v;
    printf("%c",c);
  }
  printf("\n");

  fflush(stdout);
}


int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("Example1", "Example1 READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Debug, NULL, "^(\\S*) DEBUG *");

  IvyStart("127.255.255.255");

  g_main_loop_run(ml);

  return 0;
}

