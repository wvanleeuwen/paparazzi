#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>
#include <string.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>


using namespace std;

static void on_Debug(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  guint ac_id = atoi(argv[0]);

  time_t t = time(0);
  struct tm * now = localtime( & t );

  cout << "DATE ";

  cout << (now->tm_year + 1900) << '-' 
       << (now->tm_mon + 1) << '-'
       <<  now->tm_mday << " TIME "
       <<  now->tm_hour << '-'
       <<  now->tm_min << '-'
       <<  now->tm_sec 
       <<  " ";

  int code = 0;
  printf("AC %d QRCODE %d ",ac_id, code);

  printf("\n");

  fflush(stdout);
}


int main ( int argc, char** argv)
{
  printf("qrcodelist\n");
  fflush(stdout);

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  IvyInit ("qrcodereader", "qrcodereader READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Debug, NULL, "^(\\S*) DEBUG *");
  IvyStart("127.255.255.255");

  g_main_loop_run(ml);

  return 0;
}
