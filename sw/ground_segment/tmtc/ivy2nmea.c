/*
 * Name: IVY2NMEA
 * Author: OpenUAS (Thanks to CDW for basis and Tim for NMEA library work) 
 * URL: http://ivy2nmea.openuas.org
 * License: http://www.gnu.org/licenses/lgpl.html
 * Info: IVY AC messages to NMEA GPRMC and GPGGA sentences out to serial port but clearly for *Position only*
 * (OBC2014 RULES: an NMEA 0183 serial output with GPRMC and GPGGA sentences for aircraft current location)
   Absolute Location = position?
   Location - Position Definition: A point on the earth's surface expressed by a coordinate system such as latitude and longitude.

 * Goal: This application listens on the ivy bus extracts data needed and outputs NMEA sentences on a serial port
 * Why: An UAS challenge OBC 2014 demands NMEA serial data out, the sole reason for this application
 * Version: 0.1
 * Date: 20131029
 * Notes: Alpha state
 * $Id:  $
*/

/* NMEA Introduction

NMEA 0183 is a combined electrical and data specification for communication
between marine electronic devices such as echo sounder, sonars, anemometer,
gyrocompass, autopilot, GPS receivers and many other types of instruments.

NMEA 0183 standard uses text-based (ASCII), serial communications protocol.
It defines rules for transmitting "sentences" from one "talker" to
multiple listeners.

About NMEA 0183 protocols

There are two layers in NMEA 0183:

# data link layer
# application layer protocol

In fact, data link layer defines only serial configuration:

* bit rate (typically 4800)
* 8 data bits
* no parity checking
* 1 stop bit
* no handshake

Application layer more complex, but not really to complex. Common NMEA sentence format listed below:

 $<talker ID><sentence ID,>[parameter 1],[parameter 2],...[<*checksum>]<CR><LF>

That else it is necessary to specify:

NMEA defines two types of sentences: proprietary and non-proprietary.
Non-proprietary sentences has a one of standard two-letter talker ID
(e.g. GP for GPS unit, GL for glonass unit etc.)
and one of standard three-letter sentence ID
(e.g. GLL for geographic location GPS data, DBK - depth below keel and so on)
all of this talker IDs and sentence IDs can be found in official paper.
Non-proprietary sentences has a 'P' letter instead of standard talker ID,
followed by three-letter standard manufacturer code
(GRM for Garmin, MTK for MTK etc.),
further follows any string - name of proprietary command
(depends on specific manufacturer). Maximum length for sentences– 82 characters.

Explanation via an example for (standard) 'GLL' GPS sentence:

GLL - means geographic location

 $GPGLL,1111.11,a,yyyyy.yy,a,hhmmss.ss, A*hh <CR><LF>

Parameters list description:

* llll.ll - latitude
* 'N' letter for North, 'S' - for south
* yyyyy.yy - longitude
* 'E' letter - for east, 'W' - for west
* UTC time in moment of measurement
* 'A' letter - data valid, 'V' - data not valid
* checksum

Example: $GPGLL,5532.8492,N,03729.0987,E,004241.469,A*33

and proprietary Garmin 'E' sentence:

PGRME - means Estimated Error Information

 $PGRME,x.x,M,x.x,M,x.x,M*hh <CR><LF>

Parameters list description:

* x.x - Estimated horizontal position error (HPE) 0.0 to 999.9 meters
* M - means meters
* x.x - Estimated vertical error (VPE) 0.0 to 999.9 meters
* M - means meters
* x.x - Estimated position error (EPE) 0.0 to 999.9 meters
* checksum

Types

For the OBC2014 we only need to provide two types: GPGGA and GPRMC

 $GPGGA  Global positioning system fixed data
 $GPRMC  Recommended minimum specific GNSS data

!!!GPRMC

The $GPRMC Sentence (Position and time)

* Example (signal not acquired):
  $GPRMC,235947.000,V,0000.0000,N,00000.0000,E,,,041299,,*1D
* Example (signal acquired):
  $GPRMC,092204.999,A,4250.5589,S,14718.5084,E,0.00,89.68,211200,,*25

!!!GPGGA

The $GPGGA Sentence (Fix data)

* Example (signal not acquired): $GPGGA,235947.000,0000.0000,N,00000.0000,E,0,00,0.0,0.0,M,,,,0000*00
* Example (signal acquired): $GPGGA,092204.999,4250.5589,S,14718.5084,E,1,04,24.4,19.7,M,,,,0000*1F

*/

#include <glib.h>
//#include <gtk/gtk.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <float.h>
#include <time.h>

#include <signal.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/timer.h>
#include <Ivy/version.h>
//#include <Ivy/ivyglibloop.h>

/*
 * Debug print extras
 */
#define Dprintf(X, ... )
//#define Dprintf printf

/*
 * Application version
 */
#define IVY2NMEA_VERSION        ("0.1.0")
#define IVY2NMEA_VERSION_MAJOR  (0)
#define IVY2NMEA_VERSION_MINOR  (1)
#define IVY2NMEA_VERSION_MICRO  (0)

/*
 * Buffer
 */
#define NMEA_CONVSTR_BUF    (256)
#define NMEA_TIMEPARSE_BUF  (256)

#define NMEA_POSIX(x)  x
#define NMEA_INLINE    inline

/*
 * Distance units
 */
#define NMEA_TUD_YARDS      (1.0936)        /**< Yeards, meter * NMEA_TUD_YARDS = yard */
#define NMEA_TUD_KNOTS      (1.852)         /**< Knots, kilometer / NMEA_TUD_KNOTS = knot */
#define NMEA_TUD_MILES      (1.609)         /**< Miles, kilometer / NMEA_TUD_MILES = mile */

/*
 * Fixed for conversion
 */
#define NMEA_PI                     (3.141592653589793)             /**< PI value */
#define NMEA_PI180                  (NMEA_PI / 180)                 /**< PI division by 180 */
#define NMEA_EARTHRADIUS_KM         (6378)                          /**< Earth's mean radius in km */
#define NMEA_EARTHRADIUS_M          (NMEA_EARTHRADIUS_KM * 1000)    /**< Earth's mean radius in m */
#define NMEA_EARTH_SEMIMAJORAXIS_M  (6378137.0)                     /**< Earth's semi-major axis in m according WGS84 */
#define NMEA_EARTH_SEMIMAJORAXIS_KM (NMEA_EARTHMAJORAXIS_KM / 1000) /**< Earth's semi-major axis in km according WGS 84 */
#define NMEA_EARTH_FLATTENING       (1 / 298.257223563)             /**< Earth's flattening according WGS 84 */
#define NMEA_DOP_FACTOR             (5)                             /**< Factor for translating DOP to meters */

/*
 * Speed units
 */
#define NMEA_TUS_MS         (3.6)           /**< Meters per seconds, (k/h) / NMEA_TUS_MS= (m/s) */

/*
 * IVY and Bus
 */
long delay = 1000;/* Serial Repeat Rate */

//GtkWidget *status_ivy;
//GtkWidget *status_serial;
//GtkWidget *status;

char status_str[256];
char status_ivy_str[256];
char status_serial_str[256];
char *port = "";

long int count_ivy = 0;
long int count_serial = 0;

long int rx_bytes = 0;
long int tx_bytes = 0;
long int rx_error = 0;


/**
 * NMEA packets type which are generated by library
 */
enum nmeaPACKETTYPE
{
    GPNON   = 0x0000,   /**< Unknown packet type. */
    GPGGA   = 0x0001,   /**< GGA - Essential fix data which provide 3D location and accuracy data. */
    GPRMC   = 0x0002,   /**< RMC - Recommended Minimum Specific GPS/TRANSIT Data. */
};

/**
 * Date and time data
 */
typedef struct _nmeaDATETIME
{
    int     year;       /**< Years since 1900 */
    int     mon;        /**< Months since January - [0,11] */
    int     day;        /**< Day of the month - [1,31] */
    int     hour;       /**< Hours since midnight - [0,23] */
    int     min;        /**< Minutes after the hour - [0,59] */
    int     sec;        /**< Seconds after the minute - [0,59] */
    int     hsec;       /**< Hundredth part of second - [0,99] */

} nmeaDATETIME;

/**
 * GGA packet information structure (Global Positioning System Fix Data)
 */
typedef struct _nmeaGPGGA
{
    nmeaDATETIME utc;        /**< UTC of position (just time) */
	double   lat;        /**< Latitude in NDEG - [degree][min].[sec/60] */
    char     ns;         /**< [N]orth or [S]outh */
	double   lon;        /**< Longitude in NDEG - [degree][min].[sec/60] */
    char     ew;         /**< [E]ast or [W]est */
    int      sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
	int      satinuse;   /**< Number of satellites in use (not those in view) */
    double   HDOP;       /**< Horizontal dilution of precision */
    double   elv;        /**< Antenna altitude above/below mean sea level (geoid) */
    char     elv_units;  /**< [M]eters (Antenna height unit) */
    double   diff;       /**< Geoidal separation (Diff. between WGS-84 earth ellipsoid and mean sea level. '-' = geoid is below WGS-84 ellipsoid) */
    char     diff_units; /**< [M]eters (Units of geoidal separation) */
    double   dgps_age;   /**< Time in seconds since last DGPS update */
    int      dgps_sid;   /**< DGPS station ID number */

} nmeaGPGGA;

/**
 * RMC packet information structure (Recommended Minimum sentence C)
 */
typedef struct _nmeaGPRMC
{
    nmeaDATETIME utc;         /**< UTC of position */
    char     status;      /**< Status (A = active or V = void) */
	double   lat;         /**< Latitude in NDEG - [degree][min].[sec/60] */
    char     ns;          /**< [N]orth or [S]outh */
	double   lon;         /**< Longitude in NDEG - [degree][min].[sec/60] */
    char     ew;          /**< [E]ast or [W]est */
    double   speed;       /**< Speed over the ground in knots */
    double   direction;   /**< Track angle in degrees True */
    double   declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */
    char     declin_ew;   /**< [E]ast or [W]est */
    char     mode;        /**< Mode indicator of fix type (A = autonomous, D = differential, E = estimated, N = not valid, S = simulator) */

} nmeaGPRMC;

/**
 * nmeaINFO_of_UAV_from_UA DATA
 */
typedef struct _nmeaINFO_of_UAV //Where data is stored to be saved via NMA out
{

	unsigned char ac_id; /**< the Aircraft ID of the data */
    int     smask;       /**< Mask specifying types of packages to use */
    nmeaDATETIME utc;        /**< UTC of position */
    int     sig;         /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
    int     fix;         /**< Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) */
    double  PDOP;        /**< Position Dilution Of Precision */
    double  HDOP;        /**< Horizontal Dilution Of Precision */
    double  VDOP;        /**< Vertical Dilution Of Precision */
    double  lat;         /**< Latitude in NDEG - +/-[degree][min].[sec/60] */
    double  lon;         /**< Longitude in NDEG - +/-[degree][min].[sec/60] */
    double  elv;         /**< Antenna altitude above/below mean sea level (geoid) in meters */
    double  speed;       /**< Speed over the ground in kilometers/hour */
    double  direction;   /**< Track angle in degrees True */
    double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */

} nmeaINFO_of_UAV;

void nmea_zero_INFO(nmeaINFO_of_UAV *info);

volatile unsigned char new_ivy_data = 0;
volatile unsigned char new_serial_data = 0;

/*************************************************************************/

int     nmea_generate(
        char *buff, int buff_sz,    /* buffer */
        const nmeaINFO_of_UAV *info,       /* source info */
        int generate_mask           /* mask of sentences ( GPGGA | GPRMC ) */
        );

/**
 * \fn nmea_degree2radian
 * \brief Convert degree to radian
 */
double nmea_degree2radian(double val)
{ return (val * NMEA_PI180); }

/**
 * \fn nmea_radian2degree
 * \brief Convert radian to degree
 */
double nmea_radian2degree(double val)
{ return (val / NMEA_PI180); }

/**
 * \brief Convert NDEG (NMEA degree) to fractional degree
 */
double nmea_ndeg2degree(double val)
{
    double deg = ((int)(val / 100));
    val = deg + (val - deg * 100) / 60;
    return val;
}

/**
 * \brief Convert fractional degree to NDEG (NMEA degree)
 */
double nmea_degree2ndeg(double val)
{
    double int_part;
    double fra_part;
    fra_part = modf(val, &int_part);
    val = int_part * 100 + fra_part * 60;
    return val;
}

/**
 * \fn nmea_ndeg2radian
 * \brief Convert NDEG (NMEA degree) to radian
 */
double nmea_ndeg2radian(double val)
{ return nmea_degree2radian(nmea_ndeg2degree(val)); }

/**
 * \fn nmea_radian2ndeg
 * \brief Convert radian to NDEG (NMEA degree)
 */
double nmea_radian2ndeg(double val)
{ return nmea_degree2ndeg(nmea_radian2degree(val)); }

/**
 * \brief Calculate control sum of binary buffer
 */
int nmea_calc_crc(const char *buff, int buff_sz)
{
    int chsum = 0,
        it;

    for(it = 0; it < buff_sz; ++it)
        chsum ^= (int)buff[it];

    return chsum;
}

/**
 * \brief Formating string (like standart printf) with CRC tail (*CRC)
 */
int nmea_printf(char *buff, int buff_sz, const char *format, ...)
{
    int retval, add = 0;
    va_list arg_ptr;

    if(buff_sz <= 0)
        return 0;

    va_start(arg_ptr, format);

    retval = NMEA_POSIX(vsnprintf)(buff, buff_sz, format, arg_ptr);

    if(retval > 0)
    {
        add = NMEA_POSIX(snprintf)(
            buff + retval, buff_sz - retval, "*%02x\r\n",
            nmea_calc_crc(buff + 1, retval - 1));
    }

    retval += add;

    if(retval < 0 || retval > buff_sz)
    {
        memset(buff, ' ', buff_sz);
        retval = buff_sz;
    }

    va_end(arg_ptr);

    return retval;
}
/**
 * \brief Get time now to nmeaTIME structure {TODO: determine UAtime or GCS time}
 */
void nmea_time_now(nmeaDATETIME *stm)
{
    time_t lt;
    struct tm *tt;

    time(&lt);
    tt = gmtime(&lt);

    stm->year = tt->tm_year;
    stm->mon = tt->tm_mon;
    stm->day = tt->tm_mday;
    stm->hour = tt->tm_hour;
    stm->min = tt->tm_min;
    stm->sec = tt->tm_sec;
    stm->hsec = 0;
}

/**
 * Clear GPGGA data
 */
void nmea_zero_GPGGA(nmeaGPGGA *pack)
{
    memset(pack, 0, sizeof(nmeaGPGGA));
    nmea_time_now(&pack->utc);
    pack->ns = 'N';
    pack->ew = 'E';
    pack->elv_units = 'M';
    pack->diff_units = 'M';
}

/**
 * Clear GPRMC data
 */
void nmea_zero_GPRMC(nmeaGPRMC *pack)
{
    memset(pack, 0, sizeof(nmeaGPRMC));
    nmea_time_now(&pack->utc);
    pack->status = 'V';
    pack->ns = 'N';
    pack->ew = 'E';
    pack->declin_ew = 'E';
}

/**
 * \brief Generate NMEA GPGGA line
 */
int nmea_gen_GPGGA(char *buff, int buff_sz, nmeaGPGGA *pack)
{
    return nmea_printf(buff, buff_sz,
        "$GPGGA,%02d%02d%02d.%02d,%07.4f,%C,%07.4f,%C,%1d,%02d,%03.1f,%03.1f,%C,%03.1f,%C,%03.1f,%04d",
        pack->utc.hour, pack->utc.min, pack->utc.sec, pack->utc.hsec,
        pack->lat, pack->ns, pack->lon, pack->ew,
        pack->sig, pack->satinuse, pack->HDOP, pack->elv, pack->elv_units,
        pack->diff, pack->diff_units, pack->dgps_age, pack->dgps_sid);
}

/**
 * \brief Generate NMEA GPRMC line
 */
int nmea_gen_GPRMC(char *buff, int buff_sz, nmeaGPRMC *pack)
{
    return nmea_printf(buff, buff_sz,
        "$GPRMC,%02d%02d%02d.%02d,%C,%07.4f,%C,%07.4f,%C,%03.1f,%03.1f,%02d%02d%02d,%03.1f,%C,%C",
        pack->utc.hour, pack->utc.min, pack->utc.sec, pack->utc.hsec,
        pack->status, pack->lat, pack->ns, pack->lon, pack->ew,
        pack->speed, pack->direction,
        pack->utc.day, pack->utc.mon + 1, pack->utc.year - 100,
        pack->declination, pack->declin_ew, pack->mode);
}

/**
 * \brief Generate from UAV GPGGA line
 */
void nmea_info2GPGGA(const nmeaINFO_of_UAV *info, nmeaGPGGA *pack)
{
    nmea_zero_GPGGA(pack);

    pack->utc = info->utc;
    pack->lat = fabs(info->lat);
    pack->ns = ((info->lat > 0)?'N':'S');
    pack->lon = fabs(info->lon);
    pack->ew = ((info->lon > 0)?'E':'W');
    pack->sig = info->sig;
    //pack->satinuse = info->satinfo.inuse;
    pack->HDOP = info->HDOP;
    pack->elv = info->elv;
}

/**
 * \brief Generate from UAV GPRMC line
 */
void nmea_info2GPRMC(const nmeaINFO_of_UAV *info, nmeaGPRMC *pack)
{
    nmea_zero_GPRMC(pack);

    pack->utc = info->utc;
    pack->status = ((info->sig > 0)?'A':'V');
    pack->lat = fabs(info->lat);
    pack->ns = ((info->lat > 0)?'N':'S');
    pack->lon = fabs(info->lon);
    pack->ew = ((info->lon > 0)?'E':'W');
    pack->speed = info->speed / NMEA_TUD_KNOTS;
    pack->direction = info->direction;
    pack->declination = info->declination;
    pack->declin_ew = 'E';
    pack->mode = ((info->sig > 0)?'A':'N');
}


/**
 * \brief IVY Read and store in UAV info structure
 */
static void on_Gps(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
/*
TODO: ad allo same for on_Gps_Int
   <message name="GPS" id="8">
     <field name="mode"       type="uint8"  unit="byte_mask"/>
     <field name="utm_east"   type="int32"  unit="cm" alt_unit="m"/>
     <field name="utm_north"  type="int32"  unit="cm" alt_unit="m"/>
     <field name="course"     type="int16"  unit="decideg" alt_unit="deg"/>
     <field name="alt"        type="int32"  unit="mm" alt_unit="m"/>
     <field name="speed"      type="uint16" unit="cm/s" alt_unit="m/s"/>
     <field name="climb"      type="int16"  unit="cm/s" alt_unit="m/s"/>
     <field name="week"       type="uint16" unit="weeks"/>
     <field name="itow"       type="uint32" unit="ms"/>
     <field name="utm_zone"   type="uint8"/>
     <field name="gps_nb_err" type="uint8"/>
   </message>

   <message name="GPS_INT" id="155">
    <field name="ecef_x"  type="int32" unit="cm"   alt_unit="m"/>
    <field name="ecef_y"  type="int32" unit="cm"   alt_unit="m"/>
    <field name="ecef_z"  type="int32" unit="cm"   alt_unit="m"/>
    <field name="lat"     type="int32" alt_unit="deg" alt_unit_coef="0.0000057296"/>
    <field name="lon"     type="int32" alt_unit="deg" alt_unit_coef="0.0000057296"/>
    <field name="alt"     type="int32" unit="mm"   alt_unit="m"/>
    <field name="hmsl"    type="int32" unit="mm"   alt_unit="m"/>
    <field name="ecef_xd" type="int32" unit="cm/s" alt_unit="m/s"/>
    <field name="ecef_yd" type="int32" unit="cm/s" alt_unit="m/s"/>
    <field name="ecef_zd" type="int32" unit="cm/s" alt_unit="m/s"/>
    <field name="pacc"    type="int32" unit="cm"   alt_unit="m"/>
    <field name="sacc"    type="int32" unit="cm/s" alt_unit="m/s"/>
    <field name="tow"     type="uint32"/>
    <field name="pdop"    type="uint16"/>
    <field name="numsv"   type="uint8"/>
    <field name="fix"     type="uint8" values="NONE|UKN1|UKN2|3D"/>
  </message>

*/

  nmeaINFO_of_UAV info;
  char buff[2048];
  int gen_sz;
//  int it;

//Clean it
  nmea_zero_INFO(&info);

//Fill with data from the IVY message

  info.ac_id=1; //TODO
  info.smask= GPGGA | GPRMC;
  //nmeaDATETIME=now//TODO
  info.sig=atoi(argv[1]);
  info.fix=atoi(argv[1]);
  info.PDOP=atoi(argv[1]);
  info.HDOP=atoi(argv[1]);
  info.VDOP=atoi(argv[1]);
  info.lat=atoi(argv[1]);
  info.lon=atoi(argv[1]);
  info.elv=atoi(argv[1]);
  info.speed=atoi(argv[5]);
  info.direction=atoi(argv[1]);
  info.declination=atoi(argv[1]);

  gen_sz = nmea_generate( &buff[0], 2048, &info, GPGGA | GPRMC );
  buff[gen_sz] = 0;
  printf("%s\n", &buff[0]);

  //Dprintf("GPS ac=%d %d %d %d %d\n",nmeaINFO_of_UAV.ac_id, nmeaINFO_of_UAV.utm_east, nmeaINFO_of_UAV.utm_north, nmeaINFO_of_UAV.utm_z, nmeaINFO_of_UAV.utm_zone);

  new_ivy_data = 1;
}


/**
 * \brief Set Serial Port
 */

/* pointer */
int fd = 0;//FIXME move to better spot

/**
 * Open port
 */
void open_port(const char* device)
{
  fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    fprintf(stderr, "open_port: unable to open device %s - ", device);
    perror(NULL);
    exit(EXIT_FAILURE);
  }
  /* setup connection options */
  struct termios options;

  /* get the current options */
  tcgetattr(fd, &options);

  /*  
   set local mode, enable receiver, set comm. options:
   NMEA Defines, where bit rate (typically 4800)
   8 data bits, 1 stop bit, no parity, 4800 Baud, no handshake
  */
  options.c_cflag = CLOCAL | CREAD | CS8 | B4800;

  /* write options back to port */
  tcsetattr(fd, TCSANOW, &options);
}

//unsigned char* buf_tx = (unsigned char*) &nmeaINFO_of_UAV;

/**
 * \brief Send NMEA sentence to Serial Port
 */
void nmea_send_sentence_to_port(void)
{

  int bytes;
  int i = 0;

  //EXAMPLE CHECKSUM
  //char mystring[] = "GPRMC,092751.000,A,5321.6802,N,00630.3371,W,0.06,31.66,280511,,,A";

  //printf("String: %s\nChecksum: 0x%02X\n", mystring, checksum(mystring));

  if (new_ivy_data == 0)
    return;

  new_ivy_data = 0;
/*
  gen_sz = nmea_generate( &buff[0], 2048, &info, GPGGA | GPRMC );
  buff[gen_sz] = 0;
  printf("%s\n", &buff[0]);
*/
  // Checksum
  //*
/*
  for (i=0;i<(sizeof(nmeaINFO_of_UAV_from_UA)-1);i++)
  {
    nmeaINFO_of_UAV.footer += buf_tx[i];
    Dprintf("%x ", buf_tx[i]);
  }
  */
 // bytes = write(fd, &nmeaINFO_of_UAV, sizeof(nmeaINFO_of_UAV));

  tx_bytes += bytes;

//  Dprintf("SENT: %d (%d bytes)\n",nmeaINFO_of_UAV.ac_id, bytes);

  count_ivy++;

  //printf(status_ivy_str, "Received %d from IVY: forwarding to '%s' [%ld] {Tx=%ld}", ac_id, port, count_ivy, tx_bytes);
  //gtk_label_set_text( GTK_LABEL(status_ivy), status_ivy_str );
}


/**
 * Close Serial Port
 */
void close_port(void)
{
  close(fd);
}

/**
 * \brief Timer
 */
gboolean timeout_callback(gpointer data)
{
  static unsigned char dispatch = 0;

  // One out of 4
  if (dispatch > 2)
  {
    send_port();
    dispatch = 0;
  }
  else
  {
    dispatch ++;
  }
  return TRUE;
}


/**
 * \brief GTK delete_event
 *
gint delete_event( GtkWidget *widget,
                   GdkEvent  *event,
                   gpointer   data )
{
  g_print ("CLEAN STOP\n");

  close_port();
  IvyStop();

  exit(0); //FIXME rela return values from previous Ivytop etc.

  return(FALSE); // false = delete window, FALSE = keep active
}

*/

/**
 * \brief MAIN
 */
int main ( int argc, char** argv)
{
  int s = sizeof(nmeaINFO_of_UAV);

  //gtk_init(&argc, &argv);

  if (argc < 2)
  {
    printf("Use: ivy2nmea ac_id serial_device\n");
    return -1;
  }

  int ac_id = 1;

  sprintf(status_str, "Listening to AC=%d, Serial Data Size = %d",ac_id, s);
  sprintf(status_ivy_str, "---");
  sprintf(status_serial_str, "---");
  printf("%s\n",status_str);

  /* Open Serial or Die */
  port = argv[2];
  open_port(port);

  // Start IVY
  IvyInit ("IVY <-> Serial", "IVY <-> Serial READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Gps, NULL, "^%d GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)",ac_id);
  //IvyBindMsg(on_Gps_Int, NULL, "^%d GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)",ac_id);
  IvyStart("127.255.255.255");//FIXME make compliant

  /*

  // Add Timer
  gtk_timeout_add(delay / 4, timeout_callback, NULL);

  // GTK Window
  GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window), "IVY_to_NMEA_serial_out");

  gtk_signal_connect (GTK_OBJECT (window), "delete_event",
                        GTK_SIGNAL_FUNC (delete_event), NULL);

  GtkWidget *box = gtk_vbox_new(TRUE, 1);
  gtk_container_add (GTK_CONTAINER (window), box);

  GtkWidget *hbox = gtk_hbox_new(FALSE, 1);
  gtk_container_add (GTK_CONTAINER (box), hbox);
  status = gtk_label_new( "Status:" );
  gtk_box_pack_start(GTK_BOX(hbox), status, FALSE, FALSE, 1);
  gtk_label_set_justify( (GtkLabel*) status, GTK_JUSTIFY_LEFT );
  status = gtk_label_new( status_str );
  gtk_box_pack_start(GTK_BOX(hbox), status, FALSE, FALSE, 1);
  gtk_label_set_justify( (GtkLabel*) status, GTK_JUSTIFY_LEFT );

  hbox = gtk_hbox_new(FALSE, 1);
  gtk_container_add (GTK_CONTAINER (box), hbox);
  status_ivy = gtk_label_new( "IVY->SERIAL:" );
  gtk_box_pack_start(GTK_BOX(hbox), status_ivy, FALSE, FALSE, 1);
  gtk_label_set_justify( (GtkLabel*) status_ivy, GTK_JUSTIFY_LEFT );
  status_ivy = gtk_label_new( status_ivy_str );
  gtk_box_pack_start(GTK_BOX(hbox), status_ivy, FALSE, FALSE, 1);
  gtk_label_set_justify( (GtkLabel*) status_ivy, GTK_JUSTIFY_LEFT );

  gtk_widget_show_all(window);

  gtk_main();

  */

  for (;;);

  /* Clean up */
  fprintf(stderr,"Stopping\n");

  g_print ("CLEAN STOP\n");

  close_port();
  IvyStop();

  exit(0); //FIXME rela return values from previous Ivytop etc.


  return 0;//FIXME posix const compliant return
}

