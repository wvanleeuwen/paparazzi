// defined functions:
#define image_index(xx, yy)  ((yy * imgWidth + xx) * 2) & 0xFFFFFFFC  // always a multiple of 4

// Horizon detection specific:
extern int perceptronPitchRoll(unsigned char *frame_buf, int *pitch_pixel, int *roll_angle, int accuracy_range);
extern int perceptronPitchRollEfficient(unsigned char *frame_buf, int *pitch_pixel, int *roll_angle, int n_train_samples, int n_test_samples, int accuracy_range);
extern void drawLine(unsigned char *frame_buf, int a, int b, int resolution);
extern void drawRectangle(unsigned char *frame_buf, int x_left, int x_right, int y_top, int y_bottom);
extern void horizonToLineParameters(int pitch_pixel, int roll_angle, int* a, int* b);

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

// Obstacle avoidance:
extern void getObstacles(unsigned int* obstacles, unsigned int n_bins, unsigned char *frame_buf, unsigned int* max_bin, unsigned int* obstacle_total, int MAX_SIGNAL);
void getObstacles2Way(unsigned int* obstacles, unsigned int n_bins, unsigned char *frame_buf, unsigned int* max_bin, unsigned int* obstacle_total, int MAX_SIGNAL, int pitch_pixels, int roll_angle);
extern void getUncertainty(unsigned int* uncertainty, unsigned int n_bins, unsigned char *frame_buf);

// Decision trees for segmentation:
extern void segmentPatch(unsigned char *frame_buf, unsigned char *frame_buf2);
extern void segmentBWboard(unsigned char *frame_buf, unsigned char *frame_buf2);
extern void segmentSkyUncertainty2(unsigned char *frame_buf, unsigned char *frame_buf2);
extern void segment_no_yco(unsigned char *frame_buf, unsigned char *frame_buf2);
extern void segment_no_yco_AdjustTree(unsigned char *frame_buf, unsigned char *frame_buf2, int adjust_factor);
extern int segment_no_yco_PerPixel(unsigned char *frame_buf, int x, int y, unsigned int maxY, int* uncertainty);
/*
extern int segmentSkyPixelNoYCO(unsigned char *frame_buf, int x, int y, unsigned int maxY, int* uncertainty);
extern void segmentSky(unsigned char *frame_buf);
extern void segmentSkyUncertainty(unsigned char *frame_buf, unsigned char *frame_buf2);
extern void segmentNoIllumination(unsigned char *frame_buf, unsigned char *frame_buf2);
extern void segmentOnlyGradient(unsigned char *frame_buf, unsigned char *frame_buf2);
*/
