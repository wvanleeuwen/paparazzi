#ifndef COLOR
#define COLOR

#define MAX_BLOBS  63
#define MAX_COLORS 17  // reserve color #16 for internal use
#define MIN_BLOB_SIZE 10
#define MAX_IMWIDTH 1280 // used in findCenter
#define MAX_IMHEIGHT 960

#define index(xx, yy)  ((yy * imgWidth + xx) * 2) & 0xFFFFFFFC  // always a multiple of 4

//#define index(xx, yy)   (scaling_enabled > 0) ? ((imgWidth * (xx + ((63-yy) * imgWidth))) / 40) : ((yy * imgWidth + xx) * 2)


// function for color blob tracking
 void findCenter(unsigned char *frame_buf, unsigned int ii, int * coords);

// standard functions:
 unsigned int vblob(unsigned char *, unsigned char *, unsigned int);
 unsigned int vpix(unsigned char *, unsigned int, unsigned int);
 unsigned int vfind(unsigned char *frame_buf, unsigned int clr, unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2);
 void init_colors();
 void vhist(unsigned char *frame_buf);
 void vmean(unsigned char *frame_buf);
 void color_segment(unsigned char *frame_buf);
 void edge_detect(unsigned char *outbuf, unsigned char *inbuf, int threshold);

 extern unsigned int ymax[], ymin[], umax[], umin[], vmax[], vmin[];
 extern unsigned int blobx1[], blobx2[], bloby1[], bloby2[], blobcnt[], blobix[];
 extern unsigned int hist0[], hist1[], hist2[], mean[];

unsigned int vscan(unsigned char *outbuf, unsigned char *inbuf, int thresh, 
           unsigned int columns, unsigned int *outvect);
unsigned int vhorizon(unsigned char *outbuf, unsigned char *inbuf, int thresh, 
           int columns, unsigned int *outvect, int *slope, int *intercept, int filter);

void svs_segcode(unsigned char *outbuf, unsigned char *inbuf, int thresh);
void svs_segview(unsigned char *inbuf, unsigned char *outbuf);
void addvect(unsigned char *outbuf, unsigned int columns, unsigned int *vect);
void addline(unsigned char *outbuf, int slope, int intercept);
void addbox(unsigned char *outbuf, unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2);

#endif
