/**
 *  CANDY Protocol
 *
 *
 */



struct __attribute__ ((__packed__)) dc_cam_shot_data
{
  int cnt;
  double lat;
  double lon;
  float h_msl;
  float h_agl;
  float phi;
  float theta;
  float psi;
  uint32_t itow;
};


#define OPTIMAL_MODEM_PACKET_SIZE 64
#define MAX_DATA_SIZE (OPTIMAL_MODEM_PACKET_SIZE-10)
#define QUEUE_SIZE  100


struct __attribute__ ((__packed__)) soda_thumb_data
{
  uint8_t image_nr;
  uint8_t image_fragment;
  uint8_t data[MAX_DATA_SIZE];
};


struct soda_thumb_manager_struct
{
  struct soda_thumb_data queue[QUEUE_SIZE];
  int read_p;
  int write_p;
};



