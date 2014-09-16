
#include <inttypes.h>

extern uint8_t* jpeg_start;
extern uint8_t* jpeg_end;

int storeJpg(const char* Name, unsigned char* image_buffer, int w, int h, int quality);


