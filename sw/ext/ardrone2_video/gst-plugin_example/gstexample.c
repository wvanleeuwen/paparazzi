
/**
 * SECTION:element-example
 *
 * The Example plugin for gstreamer will show how to make a plugin. It will output the maximum brightness of the image.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=5/1' ! videoscale ! video/x-raw-yuv, width=640, height=368 ! example ! fakesink

 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gst/gst.h>
#include <stdlib.h>     /* calloc, exit, free */
#include <pthread.h>
#include <unistd.h>	//usleep

#include "gstexample.h"
#include "socket.h"
#include "video_message_structs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define image_index(xx, yy)  ((yy * imgWidth + xx) * 2)
unsigned int imgWidth, imgHeight;
unsigned int tcpport;
unsigned char threshtune;
unsigned int counter;
unsigned int socketIsReady;
struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;

const int dx[] = {+1, 0, -1, 0};
const int dy[] = {0, +1, 0, -1};
int label[320][240];
unsigned int area = 0;
unsigned int sumX = 0;
unsigned int sumY = 0;
unsigned int minArea = 400;

void brightspotDetector(unsigned char *frame_buf, int blob[], unsigned int * max_idx,unsigned int * max_idy) ;
void get1DHist(unsigned char *frame_buf, unsigned int * OneDHist);
unsigned char getThreshold(unsigned int * OneDHist);
void createBinaryImage(unsigned char threshold, unsigned char * frame_buf);
void get2DHist(unsigned char * frame_buf, unsigned int * hist_x, unsigned int * hist_y);
unsigned int cmpfunc (const void * a, const void * b);
unsigned int getMedian(unsigned int * hist,  unsigned int size);

void blobLabeling(unsigned char *frame_buf, unsigned int x, unsigned int y, unsigned int current_label);

void *TCP_threat( void *ptr);

GST_DEBUG_CATEGORY_STATIC (gst_example_debug);
#define GST_CAT_DEFAULT gst_example_debug

/* Filter signals and args */
enum
{
  /* FILL ME */
  LAST_SIGNAL
};

enum
{
  PROP_0,
  PROP_SILENT,
  PROP_TCP,
  PROP_THRESHTUNE
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */
static GstStaticPadTemplate sink_factory = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw-yuv, format=(fourcc)UYVY"	)
    );

static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw-yuv, format=(fourcc)UYVY"	)
    );

GST_BOILERPLATE (Gstexample, gst_example, GstElement,
    GST_TYPE_ELEMENT);

static void gst_example_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_example_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_example_set_caps (GstPad * pad, GstCaps * caps);
static GstFlowReturn gst_example_chain (GstPad * pad, GstBuffer * buf);

/* GObject vmethod implementations */

static void
gst_example_base_init (gpointer gclass)
{

  GstElementClass *element_class = GST_ELEMENT_CLASS (gclass);

  gst_element_class_set_details_simple(element_class,
    "example",
    "Passthrough element",
    "Calculates stuff on the video, to be fed to an autopilot",
    "Kevin van Hecke");

  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&src_factory));
  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&sink_factory));
}

/* initialize the example's class */
static void
gst_example_class_init (GstexampleClass * klass)
{
  GObjectClass *gobject_class;
  //GstElementClass *gstelement_class;

  gobject_class = (GObjectClass *) klass;
 // gstelement_class = (GstElementClass *) klass;

  gobject_class->set_property = gst_example_set_property;
  gobject_class->get_property = gst_example_get_property;

  g_object_class_install_property (gobject_class, PROP_SILENT,
      g_param_spec_boolean ("silent", "Silent", "Produce verbose output.",
          FALSE, G_PARAM_READWRITE));
		  
  g_object_class_install_property (gobject_class, PROP_TCP,
      g_param_spec_uint ("tcp_port", "TCP port", "Output results over tcp",0,65535,
          0, G_PARAM_READWRITE));	

  g_object_class_install_property (gobject_class, PROP_THRESHTUNE,
      g_param_spec_uint ("threshtune", "threshtune tune", "Changes output of binary image function",0,65535,
          0, G_PARAM_READWRITE));		  
		  
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void
gst_example_init (Gstexample * filter,
    GstexampleClass * gclass)
{
	
  filter->sinkpad = gst_pad_new_from_static_template (&sink_factory, "sink");
  gst_pad_set_setcaps_function (filter->sinkpad,
                                GST_DEBUG_FUNCPTR(gst_example_set_caps));
  gst_pad_set_getcaps_function (filter->sinkpad,
                                GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));
  gst_pad_set_chain_function (filter->sinkpad,
                              GST_DEBUG_FUNCPTR(gst_example_chain));

  filter->srcpad = gst_pad_new_from_static_template (&src_factory, "src");
  gst_pad_set_getcaps_function (filter->srcpad,
                                GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));

  gst_element_add_pad (GST_ELEMENT (filter), filter->sinkpad);
  gst_element_add_pad (GST_ELEMENT (filter), filter->srcpad);
  filter->silent = FALSE;
    
}

static void
gst_example_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  Gstexample *filter = GST_EXAMPLE (object);

  switch (prop_id) {
    case PROP_SILENT:
      filter->silent = g_value_get_boolean (value);
      break;	  
    case PROP_TCP:
      tcpport = g_value_get_uint (value);
      break;	
    case PROP_THRESHTUNE:
      threshtune = g_value_get_uint (value);
      break;	  
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_example_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  Gstexample *filter = GST_EXAMPLE (object);

  switch (prop_id) {
    case PROP_SILENT:
      g_value_set_boolean (value, filter->silent);
      break;
	case PROP_TCP:
      g_value_set_uint (value, tcpport);
      break;
	case PROP_THRESHTUNE:
      g_value_set_uint (value, threshtune);
      break;	  
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/* GstElement vmethod implementations */

/* this function handles the link with other elements */
static gboolean
gst_example_set_caps (GstPad * pad, GstCaps * caps)
{
  Gstexample *filter;
  GstPad *otherpad;

  filter = GST_EXAMPLE (gst_pad_get_parent (pad));
  otherpad = (pad == filter->srcpad) ? filter->sinkpad : filter->srcpad;
  gst_object_unref (filter);

    
  //make the image size known
   const GstStructure *str;
	str = gst_caps_get_structure (caps, 0);
	gint tmp;
	gst_structure_get_int (str, "width", &tmp);
	imgWidth = (unsigned int)tmp;  
	gst_structure_get_int (str, "height", &tmp);
	imgHeight = (unsigned int)tmp;
	g_print ("The video size is %dx%d\n", imgWidth, imgHeight);
	counter =0;
	
	//ppz2gst = (ppz2gst_message_struct *) malloc(sizeof(ppz2gst));

  //initialise socket:

	if (tcpport>0) {
		//start seperate threat to connect
		//seperate threat is needed because otherwise big delays can exist in the init or chain function
		//causing the gst to crash
	
		pthread_t th1;
		int th1_r;
		pthread_create(&th1,NULL,TCP_threat,&th1_r);
		//pthread_join(th1,NULL);

	}
  return gst_pad_set_caps (otherpad, caps);
}


void *TCP_threat( void *ptr) {
	g_print("Waiting for connection on port %d\n",tcpport);
	socketIsReady = initSocket(tcpport);
   	if (!socketIsReady) { 
		g_print("Error initialising connection\n");	
	} else {
		g_print("Connected!\n");
	}


	while(1) {
		int res = Read_msg_socket((char *) &ppz2gst,sizeof(ppz2gst));
		if	(res>0) {
			int tmp;
			tmp = (int)counter - (int)ppz2gst.heading;
			g_print("Current counter: %d, Received counter: %d, diff: %d\n",counter, ppz2gst.heading, tmp);
			ppz2gst.heading = 6;
		} else {
			g_print("Nothing received: %d\n",res);
			usleep(100000);
		}
	}



}

/* chain function
 * this function does the actual processing
 */
static GstFlowReturn gst_example_chain (GstPad * pad, GstBuffer * buf)
{
	Gstexample *filter;

	filter = GST_EXAMPLE (GST_OBJECT_PARENT (pad));

	unsigned char * img = GST_BUFFER_DATA(buf);  
    signed int blobP[8];
    memset(blobP, 0, sizeof(blobP[0]) * 8);
	unsigned int max_idx, max_idy;
	brightspotDetector(img,blobP,&max_idx,&max_idy);

			//g_print("Max_idx: %d Max_idy: %d Counter: %d\n",max_idx,max_idy,counter);


		if (tcpport>0) { 	//if network was enabled by user
			if (socketIsReady) { 
				gst2ppz.blob_x1 = blobP[0];
				gst2ppz.blob_y1 = blobP[1];
				gst2ppz.blob_x2 = blobP[2];
				gst2ppz.blob_y2 = blobP[3];
				gst2ppz.blob_x3 = blobP[4];
				gst2ppz.blob_y3 = blobP[5];
				gst2ppz.blob_x4 = blobP[6];
				gst2ppz.blob_y4 = blobP[7];
				gst2ppz.counter = counter;
				Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
			}

		}
    //memset(blobP, 0, sizeof(blobP[0]) * 8);	
	counter++;
	
	  
  return gst_pad_push (filter->srcpad, buf);
}




void brightspotDetector(unsigned char *frame_buf, int blob[], unsigned int * max_idx,unsigned int * max_idy) 
{
	unsigned char thresh;
	unsigned int * OneDHist =(unsigned int *) calloc(256,sizeof(unsigned int));
	unsigned int * hist_x =(unsigned int *) calloc(imgWidth,sizeof(unsigned int));
	unsigned int * hist_y = (unsigned int *)calloc(imgHeight,sizeof(unsigned int));
	get1DHist(frame_buf,OneDHist);
	thresh = getThreshold(OneDHist);
	createBinaryImage(thresh,frame_buf);
	//get2DHist(frame_buf,hist_x,hist_y);
	//*max_idx = getMedian(hist_x,imgWidth);
	//*max_idy = getMedian(hist_y,imgHeight);
    unsigned int i, j, ix;
    unsigned int component = 0;
    area = 0;
    sumX = 0;
    sumY = 0;
    unsigned int blobAccepted = 0;
    unsigned int blobOutlier = 0;
    unsigned int totalLabel = 0;

    for (i = 0; i < imgWidth; ++i)
    {
        for (j = 0; j < imgHeight; ++j)
        {
        	ix = image_index(i,j);
            if (!label[i][j] && frame_buf[ix+1])
            {
                totalLabel++;
                blobLabeling(frame_buf,i, j, ++component);
                if(area<minArea)
                {
                    area = 0;
                    sumX = 0;
                    sumY = 0;
                    blobOutlier++;
                	continue;
                }
                if(blobAccepted<8){
                    //blob in body axis
                    blob[blobAccepted] = -((sumY/area)-120);
                    blob[blobAccepted+1] = (sumX/area)-160;
                }                
                blobAccepted = blobAccepted + 2;
                //g_print("blob %d : area = %d xdel = %d ydel = %d \n", blobAccepted/2, area, -((sumY/area)-120), sumX/area-160);
                area = 0;
                sumX = 0;
                sumY = 0;
            }
        }
    }

    memset(label, 0, sizeof(label[0][0]) * imgWidth * imgHeight);	
	
	free(OneDHist);
	free(hist_x);
	free(hist_y);
}
void get1DHist(unsigned char *frame_buf, unsigned int * OneDHist) {
	unsigned int ix;
	unsigned int color_channels = 4;
	unsigned int step = 1 * color_channels;
	
	
	for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
	{ 		
        OneDHist[frame_buf[ix+1]]++;
		OneDHist[frame_buf[ix+3]]++;		
    }

} 

unsigned char getThreshold(unsigned int * OneDHist) {

	unsigned int total= (unsigned int)((float)(imgWidth*imgHeight)*((float)threshtune/100));
	unsigned char i;
	unsigned int tmptotal = 0;
	
	for (i = 0; i<255; i++) {
		tmptotal+=OneDHist[i];
		if (tmptotal> total)
			return i;
	}
	return 255;
}

void createBinaryImage(unsigned char threshold, unsigned char * frame_buf) {
	unsigned int ix;
	unsigned int color_channels = 4;
	unsigned int step = 1 * color_channels;
		
	for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
	{ 		
		//frame_buf[ix] = 0;
		//frame_buf[ix+2] = 0;
        if (frame_buf[ix+1] < threshold)
			frame_buf[ix+1] = 0;
		else
			frame_buf[ix+1] = 255;
        if (frame_buf[ix+3] < threshold)
			frame_buf[ix+3] = 0;	
		else
			frame_buf[ix+3] = 255;			
    }

}

void get2DHist(unsigned char * frame_buf, unsigned int * hist_x, unsigned int * hist_y) {
	unsigned int x,y,ix;
		
	for (x=0; x<(imgWidth); x++)
	{
		unsigned int tmpsum = 0;
		for (y=0; y<(imgHeight); y++)
		{ 
			ix = image_index(x,y);
			tmpsum+=frame_buf[ix+1] > 0;
		}
		hist_x[x] = tmpsum;
		
	}

	//TODO: optimize loop below to integrate with loop above...
	for (y=0; y<(imgHeight); y++)
	{
		unsigned int tmpsum = 0;
		for (x=0; x<(imgWidth); x++)
		{ 
			tmpsum+=frame_buf[image_index(x,y) +1] > 0;
		}
		hist_y[y] = tmpsum;
	}
	
}
unsigned int cmpfunc (const void * a, const void * b)
{
   return ( *(unsigned int*)a - *(unsigned int*)b );
}
unsigned int getMedian(unsigned int * hist,  unsigned int size) {
	
	unsigned int total = 0;
	unsigned int tmptotal = 0;
	
	for (unsigned int i = 0 ; i< size; i++) {	
		total+=hist[i];	
	}
	
	for (unsigned int i = 0 ; i< size; i++) {
		tmptotal+=hist[i];
	if (tmptotal>total/2) 
		return i;
	}
	return 0;	
}

/*
void getxy(unsigned int max_y_ix, unsigned int * max_idx, unsigned int * max_idy) {	
	max_y_ix/=2;
	*max_idy = (max_y_ix / imgWidth);
	*max_idx = (max_y_ix) - *max_idy*imgWidth;
}
*/
void blobLabeling(unsigned char *frame_buf, unsigned int x, unsigned int y, unsigned int current_label)
{
	unsigned int ix;
	
	if (x >= imgWidth) return;
	if (y >= imgHeight) return; 
    ix = image_index(x,y);
	if (label[x][y] || !frame_buf[ix+1]) return;
    //if (*area > (imgWidth*imgHeight) || *sumX > 4294967295 || *sumY > 4294967295) return;

	label[x][y] = current_label;
	area = area + 1;
	sumX = sumX + x;
	sumY = sumY + y;

	int direction;
	for (direction = 0; direction < 4; ++direction)
	blobLabeling(frame_buf, x + dx[direction], y + dy[direction], current_label);

}




/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features
 */
static gboolean
example_init (GstPlugin * example)
{
  /* debug category for fltering log messages
   *
   * exchange the string 'Template example' with your description
   */   
	 
  GST_DEBUG_CATEGORY_INIT (gst_example_debug, "example",
      0, "The Example plugin for gstreamer will output the maximum brightness of the frames flowing by");

  return gst_element_register (example, "example", GST_RANK_NONE,
      GST_TYPE_EXAMPLE);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "Example"
#endif

/* gstreamer looks for this structure to register examples
 *
 * exchange the string 'Template example' with your example description
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    "example",
    "The Example plugin for gstreamer will output the maximum brightness of the frames flowing by",
    example_init,
    VERSION,
    "LGPL",
    "Example",
    "http://gstreamer.net/"
)

