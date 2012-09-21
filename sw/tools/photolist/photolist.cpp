#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "photolist.h"
#include "Conversions.h"

int verbose = 0;
double lat,lon,h;

Conversions ecef;

using namespace std;

static void on_Shot(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  guint ac_id = atoi(argv[0]);

  if (verbose > 0)
  {

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

  }

/*

  <message name="DC_SHOT" id="110">
    <field name="photo_nr" type="int16"/>
    <field name="utm_east"  type="int32" unit="cm"/>
    <field name="utm_north" type="int32" unit="cm"/>
    <field name="z" type="float" unit="m"/>
    <field name="utm_zone"  type="uint8"/>
    <field name="phi" type="int16" unit="decideg"/>
    <field name="theta" type="int16" unit="decideg"/>
    <field name="course" type="int16" unit="decideg"/>
    <field name="speed"  type="uint16" unit="cm/s"/>
    <field name="itow"  type="uint32" unit="ms"/>
  </message>

*/

  int photo_nr = atoi(argv[1]);
  double utm_east = ((double)atof(argv[2])) / 100.0;
  double utm_north = ((double)atof(argv[3])) / 100.0;
  double utm_z = ((double)atof(argv[4]));
  int utm_zone = atoi(argv[5]);

  ecef.utm2llh(utm_north,utm_east,utm_z,utm_zone,&lat,&lon,&h);

  printf("AC %d PHOTO %d ",ac_id, photo_nr);

  if (verbose > 1)
  {
    printf("UTM");

    for (int i=2;i<6;i++)
    {
      cout << argv[i] << ' ';
    }

    printf("ATTITUDE ");

    for (int i=6;i<9;i++)
    {
      float ang = atof(argv[i]);
      cout << ang / 10.0f << ' ';
    }

    printf("ITOW %s ", argv[9]);
  }

  printf("LLH %lf %lf %lf",lat, lon, h);

  printf("\n");

  fflush(stdout);
}


int main ( int argc, char** argv)
{
  printf("photolist with no arguments = short list, with one argument medium list, and 2 arguments a full list\n");
  fflush(stdout);

  if ((argc >= 3) && (strlen(argv[2]) > 1))
  {
    //printf("arg[2] = %s\n", argv[2]);
    verbose = 2;
  }
  else if ((argc >= 2) && (strlen(argv[1]) > 1))
    verbose = 1;
  else
    verbose = 0;

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("mavlink-ivy-interface", "mavlink-ivy-interface READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Shot, NULL, "^(\\S*) DC_SHOT (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

  IvyStart("127.255.255.255");

  g_main_loop_run(ml);

  return 0;
}
