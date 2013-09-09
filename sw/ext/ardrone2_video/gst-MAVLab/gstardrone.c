
/**
 * SECTION:element-mavlab
 *
 * The MAVLab plugin for gstreamer will contain algorithms to perform analysis on the realtime video on board, to feed to the autopilot
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=5/1' ! videoscale ! video/x-raw-yuv, width=640, height=368 ! mavlab ! fakesink

 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <stdlib.h>     /* calloc, exit, free */
#include <unistd.h>		//usleep
#include <stdio.h> /* printf */

#include "socket.h"
#include "video_message_structs_sky.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gst/gst.h>
#include "gstardrone.h"
#include "guido.h"
#include "optic_flow.h"
#include "gps.h"

unsigned int imgWidth, imgHeight;
int mode;
gint adjust_factor;
unsigned char * img_uncertainty;

//optical flow
unsigned char * old_img;
int old_pitch,old_roll,old_alt;


unsigned int counter;

unsigned int socketIsReady;
unsigned int tcpport;
struct gst2ppz_message_struct_sky gst2ppz;
struct ppz2gst_message_struct_sky ppz2gst;


void makeCross(unsigned char * img, int x,int y, int imw, int imh);
void *TCP_threat( void *ptr);

GST_DEBUG_CATEGORY_STATIC (gst_mavlab_debug);
#define GST_CAT_DEFAULT gst_mavlab_debug

/* Filter signals and args */
enum
{
  /* FILL ME */
  LAST_SIGNAL
};

//adjustable parameters
enum
{
  PROP_0,
  PROP_SILENT,
  PROP_TCP,
  ADJUST,
  MODE
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

GST_BOILERPLATE (Gstmavlab, gst_mavlab, GstElement,
    GST_TYPE_ELEMENT);

static void gst_mavlab_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_mavlab_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_mavlab_set_caps (GstPad * pad, GstCaps * caps);
static GstFlowReturn gst_mavlab_chain (GstPad * pad, GstBuffer * buf);

/* GObject vmethod implementations */

static void
gst_mavlab_base_init (gpointer gclass)
{

  GstElementClass *element_class = GST_ELEMENT_CLASS (gclass);

  gst_element_class_set_details_simple(element_class,
    "mavlab",
    "Passthrough element",
    "Calculates stuff on the video, to be fed to an autopilot",
    "Kevin van Hecke");

  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&src_factory));
  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&sink_factory));
}

/* initialize the mavlab's class */
static void
gst_mavlab_class_init (GstmavlabClass * klass)
{
  GObjectClass *gobject_class;
  //GstElementClass *gstelement_class;

  gobject_class = (GObjectClass *) klass;
 // gstelement_class = (GstElementClass *) klass;

  gobject_class->set_property = gst_mavlab_set_property;
  gobject_class->get_property = gst_mavlab_get_property;

	g_object_class_install_property (gobject_class, PROP_SILENT,
    g_param_spec_boolean ("silent", "Silent", "Produce verbose output.",
    FALSE, G_PARAM_READWRITE));
		  
	g_object_class_install_property (gobject_class, PROP_TCP,
    g_param_spec_uint ("tcp_port", "TCP port", "Output results over tcp",0,65535,
    0, G_PARAM_READWRITE));		  
		  
	g_object_class_install_property (gobject_class, ADJUST,
      g_param_spec_int ("adjust", "Adjust factor", "Change adjust factor for sky segmentation.",-1000,1000,
          0, G_PARAM_READWRITE));
	
	g_object_class_install_property (gobject_class, MODE,
      g_param_spec_int ("mode", "Change the mode", "Change the mode to do either only segmentation [0] or the whole bunch [1].",0,10,
          0, G_PARAM_READWRITE));
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void
gst_mavlab_init (Gstmavlab * filter,
    GstmavlabClass * gclass)
{
	
  filter->sinkpad = gst_pad_new_from_static_template (&sink_factory, "sink");
  gst_pad_set_setcaps_function (filter->sinkpad,
                                GST_DEBUG_FUNCPTR(gst_mavlab_set_caps));
  gst_pad_set_getcaps_function (filter->sinkpad,
                                GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));
  gst_pad_set_chain_function (filter->sinkpad,
                              GST_DEBUG_FUNCPTR(gst_mavlab_chain));

  filter->srcpad = gst_pad_new_from_static_template (&src_factory, "src");
  gst_pad_set_getcaps_function (filter->srcpad,
                                GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));

  gst_element_add_pad (GST_ELEMENT (filter), filter->sinkpad);
  gst_element_add_pad (GST_ELEMENT (filter), filter->srcpad);
  filter->silent = FALSE;
    
}

static void
gst_mavlab_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  Gstmavlab *filter = GST_MAVLAB (object);

  switch (prop_id) {
    case PROP_SILENT:
      filter->silent = g_value_get_boolean (value);
      break;
    case ADJUST:
      adjust_factor = g_value_get_int (value);
      break;	
    case PROP_TCP:
      tcpport = g_value_get_uint (value);
      break;		  
	case MODE:
      mode = g_value_get_int (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_mavlab_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  Gstmavlab *filter = GST_MAVLAB (object);

  switch (prop_id) {
    case PROP_SILENT:
      g_value_set_boolean (value, filter->silent);
      break;
	case ADJUST:
      g_value_set_int (value, adjust_factor);
      break;
	case PROP_TCP:
      g_value_set_uint (value, tcpport);
      break;
	case MODE:
      g_value_set_int (value, mode);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/* GstElement vmethod implementations */

/* this function handles the link with other elements */
static gboolean
gst_mavlab_set_caps (GstPad * pad, GstCaps * caps)
{
  Gstmavlab *filter;
  GstPad *otherpad;

  filter = GST_MAVLAB (gst_pad_get_parent (pad));
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

	counter = 0;
	img_uncertainty= (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char)); //TODO: find place to put: free(img_uncertainty);
	old_img = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
	old_pitch = 0;
	old_roll = 0;
	old_alt=0;
	ppz2gst.pitch = 0;
	ppz2gst.roll = 0;
  
  
  	if (tcpport>0) {
		//start seperate threat to connect
		//seperate threat is needed because otherwise big delays can exist in the init or chain function
		//causing the gst to crash
	
		pthread_t th1;
		int th1_r;
		pthread_create(&th1,NULL,TCP_threat,&th1_r);	
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
		if	(res>1) {
			int tmp;
			tmp = (int)counter - (int)ppz2gst.counter;			
			if (tmp>6) {
				g_print("Current counter: %d, Received counter: %d, diff: %d\n",counter, ppz2gst.counter, tmp); //delay of 3 is caused by the fact not every frame is used (15fps mod 3)
			}	
						
		} else {
			g_print("Nothing received: %d\n",res);
			usleep(100000);
		}
	}
}

/* chain function
 * this function does the actual processing
 */
static GstFlowReturn gst_mavlab_chain (GstPad * pad, GstBuffer * buf)
{
	Gstmavlab *filter;

	filter = GST_MAVLAB (GST_OBJECT_PARENT (pad));	
	unsigned char * img = GST_BUFFER_DATA(buf);   
	
	//if GST_BUFFER_SIZE(buf) <> imgheight*imgwidth*2 -> wrong color space!!!
	if (mode==0) 
	{
		segment_no_yco_AdjustTree(img,img_uncertainty,adjust_factor); 
	}
	else if (mode==1)
	{
		skyseg_interface_n(img, img_uncertainty, adjust_factor, counter, ppz2gst.pitch/36, ppz2gst.roll/36); 

		if (tcpport>0) { 	//if network was enabled by user
			if (socketIsReady) { 
				//gst2ppz.blob_x1 = blobP[0];
				gst2ppz.counter = counter;
				Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
			}
		}
	}
	else if (mode==2 && counter % 3 == 0)
	{	
		int MAX_POINTS, error;
		int suppression_distance_squared,n_found_points,mark_points;
		int *x, *y, *new_x, *new_y, *status;
		
		int current_pitch = ppz2gst.pitch;
		int current_roll = ppz2gst.roll;
		int current_alt = ppz2gst.alt;
	
		x = (int *) calloc(40,sizeof(int));
		new_x = (int *) calloc(40,sizeof(int));
		y = (int *) calloc(40,sizeof(int));
		new_y = (int *) calloc(40,sizeof(int));
		
		status = (int *) calloc(40,sizeof(int));
		
		
		
		
		MAX_POINTS = 40;
		suppression_distance_squared = 3 * 3;
		mark_points = 0;
		error = findCorners(img, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points,imgWidth,imgHeight);
		
		if(error == 0)
		{
			error = opticFlowLK(img, old_img, x, y, n_found_points, imgWidth, imgHeight, new_x, new_y, status, 5, MAX_POINTS);	
			
			
			//calculate roll and pitch diff:
			int diff_roll = (current_roll- old_roll)/36;
			int diff_pitch = (current_pitch- old_pitch)/36;
			
			//calculate mean altitude between the to samples:
			int mean_alt;
			if (current_alt>old_alt)
				mean_alt = (current_alt-old_alt)/2 + old_alt;
			else
				mean_alt = (old_alt-current_alt)/2 + current_alt;			
			
			
			//remember the frame and meta info
			memcpy(old_img,img,imgHeight*imgWidth*2);
			old_pitch = current_pitch;
			old_roll = current_roll;
			old_alt = current_alt;
			
			
			
			if(error == 0)
			{
				showFlow(img, x, y, status, n_found_points, new_x, new_y, imgWidth, imgHeight);
				
				int tot_x=0;
				int tot_y=0;
				for (int i=0; i<n_found_points;i++) {
					tot_x = tot_x+(new_x[i]-x[i]);
					tot_y = tot_y+(new_y[i]-y[i]);		
					//g_print("x: %d, y: %d.....new_x: %d, new_y: %d\n",x[i],y[i],new_x[i],new_y[i]);					
				}
				//g_print("Optic flow: x: %d, y: %d, Pitch: %d, Roll: %d, Height: %d\n",tot_x,tot_y,current_pitch,current_roll,current_alt);
				

								
				//convert pixels/frame to centimeter/frame
				float cmfactor = 1.322*mean_alt; // view angle of 64 degrees (2x tan(32) = 1.322) (vertial camera!)
				float opt_x = (float)tot_x / (float)imgWidth  * (float)cmfactor;
				float opt_y = (float)tot_y / (float)imgHeight  * (float)cmfactor;
				

				
				
				
				
				
				g_print("Optic flow1: x: %f, y: %f, Pitch: %d, Roll: %d, Height: %d\n",opt_x,opt_y,current_pitch,current_roll,current_alt);
				
				//compensate optic flow for attitude change:
				opt_x -=  tan_zelf(diff_roll) * mean_alt/1000;
				opt_y -= tan_zelf(diff_pitch) * mean_alt/1000;
				
				g_print("Optic flow2: x: %f, y: %f, Pitch: %d, Roll: %d, Height: %d\n",opt_x,opt_y,diff_pitch,diff_roll,mean_alt);
			
				
				if (tcpport>0) { 	//if network was enabled by user
					if (socketIsReady) { 
						gst2ppz.counter = counter;
						gst2ppz.optic_flow_x = tot_x;
						gst2ppz.optic_flow_y = tot_y;
						Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
					}
				}
				
				
			} else g_print("error1\n");			
		} else g_print("error2\n");	
		
		free(x);
		free(new_x);
		free(y);
		free(new_y);
		free(status);
			

		
		if (filter->silent == FALSE) {
			g_print("Errorh: %d, n_found_points: %d\n",error,n_found_points);
		}
	
	}
		
	counter++; // to keep track of data through ppz communication
	
	  
  return gst_pad_push (filter->srcpad, buf);
}

/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features

 */
static gboolean
mavlab_init (GstPlugin * mavlab)
{
  /* debug category for fltering log messages
   *
   * exchange the string 'Template mavlab' with your description
   */   
	 
  GST_DEBUG_CATEGORY_INIT (gst_mavlab_debug, "mavlab",
      0, "The MAVLab plugin for gstreamer will contain algorithms to perform analysis on the realtime video on board, to feed to the autopilot");

  return gst_element_register (mavlab, "mavlab", GST_RANK_NONE,
      GST_TYPE_MAVLAB);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "MAVLab"
#endif

/* gstreamer looks for this structure to register mavlabs
 *
 * exchange the string 'Template mavlab' with your mavlab description
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    "mavlab",
    "The MAVLab plugin for gstreamer will contain algorithms to perform analysis on the realtime video on board, to feed to the autopilot",
    mavlab_init,
    VERSION,
    "LGPL",
    "MAVLab",
    "http://gstreamer.net/"
)
