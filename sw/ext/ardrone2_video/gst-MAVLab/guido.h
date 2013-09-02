#ifndef GUIDO
#define GUIDO

#define image_index(xx, yy)  ((yy * imgWidth + xx) * 2) & 0xFFFFFFFC  // always a multiple of 4

extern unsigned int imgWidth, imgHeight;
// Feature extraction:
extern void getGradientPixel(unsigned char *frame_buf, int x, int y, int* dx, int* dy);
extern int getGradient(unsigned char *frame_buf, int x, int y);
extern void getGradientImage(unsigned char *frame_buf, unsigned char *frame_buf2, unsigned char *frame_buf3);
extern unsigned int getMaximumY(unsigned char *frame_buf);
extern unsigned int getMinimumY(unsigned char *frame_buf);
extern int getHarrisPixel(unsigned char *frame_buf, int x, int y);
extern int getNoblePixel(unsigned char *frame_buf, int x, int y);
extern int getPatchTexture(unsigned char *frame_buf, int x, int y, int patch_size);
extern int getPatchMean(unsigned char *frame_buf, int x, int y, int patch_size);
extern int get_FD_YCV(unsigned char *frame_buf, int x, int y);
extern int get_FD_CV(unsigned char *frame_buf, int x, int y);

extern void segmentSkyUncertainty2(unsigned char *frame_buf, unsigned char *frame_buf2);
extern void segment_no_yco(unsigned char *frame_buf, unsigned char *frame_buf2);
extern void segment_no_yco_AdjustTree(unsigned char *frame_buf, unsigned char *frame_buf2, int adjust_factor);

void skyseg_interface_i(char adjust_factor);

#endif /* GUIDO */