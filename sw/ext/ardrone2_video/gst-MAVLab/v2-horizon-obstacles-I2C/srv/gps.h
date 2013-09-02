void gps_show();
int gps_parse();
unsigned char read_ublox();
int cos(int);
int sin(int);
int tan(int);
int acos(int, int);
int asin(int, int);
int atan(int, int);
int gps_head(int, int, int, int);
int gps_dist(int, int, int, int);
unsigned int isqrt(unsigned int);

typedef struct gps_data {
    int lat;  // lat degrees x 10^6
    int lon;  // lon degrees x 10^6
    int alt;
    int fix;
    int sat;
    int utc;
} gps_data;

extern struct gps_data gps_gga;

