#ifndef GUIDO
#define GUIDO

extern unsigned int imgWidth, imgHeight;
extern void segment_no_yco_AdjustTree(unsigned char *frame_buf, unsigned char *frame_buf2, int adjust_factor);
void skyseg_interface_n(unsigned char *frame_buf, unsigned char *frame_buf2, char adjust_factor, unsigned int counter, int pitch, int roll);

#endif /* GUIDO */