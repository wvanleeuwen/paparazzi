
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

#include <gst/gst.h>

#include "gstardrone.h"
//#include "optic_flow.h"
#include "guido.h"
//#include "sky_segmentation.h"
unsigned int imgWidth, imgHeight;
gint adjust_factor;
int skipFrames; // used to skip the first x frames in order to prevent a memory problem with the segmentation algorithm
unsigned char * img_uncertainty;

unsigned char * prev_image;

void makeCross(unsigned char * img, int x,int y, int imw, int imh);

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
  ADJUST
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */
static GstStaticPadTemplate sink_factory = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("ANY")
    );

static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("ANY")
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
		  
	g_object_class_install_property (gobject_class, ADJUST,
      g_param_spec_int ("adjust", "Adjust factor", "Change adjust factor for sky segmentation.",-1000,1000,
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

  skipFrames=0;
  img_uncertainty= (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char)); //TODO: find place to put: free(img_uncertainty);
   
  return gst_pad_set_caps (otherpad, caps);
}

/* chain function
 * this function does the actual processing
 */
static GstFlowReturn gst_mavlab_chain (GstPad * pad, GstBuffer * buf)
{
	Gstmavlab *filter;

	filter = GST_MAVLAB (GST_OBJECT_PARENT (pad));

	unsigned char * img = GST_BUFFER_DATA(buf);   
  img_uncertainty= (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));

	if (skipFrames>10) //first frame crashes the algorithm... depending on adjust_factor even more than one frame... weird 
		segment_no_yco_AdjustTree(img,img_uncertainty,adjust_factor);
	else
		skipFrames++;
	free(img_uncertainty);
	if (filter->silent == FALSE) {
		g_print(".");
	}
	
	//	GST_BUFFER_DATA(buf) = img_uncertainty;
	/*
	int MAX_POINTS, error,p;
	int x[40], new_x[40];
	int y[40], new_y[40];
	int status[40];
	int suppression_distance_squared,n_found_points,mark_points;
	
	MAX_POINTS = 40;
	suppression_distance_squared = 3 * 3;
	mark_points = 0;
	error = findCorners(img, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points,imgWidth,imgHeight);
	
	for (int i = 0 ; i< n_found_points; i++) {
		makeCross(img,x[i],y[i],imgWidth,imgHeight);	
	}
	
	if (filter->silent == FALSE) {
		g_print("Error: %d, n_found_points: %d\n",error,n_found_points);
	}
	*/
	  
  return gst_pad_push (filter->srcpad, buf);
}


void makeCross(unsigned char * img, int x,int y, int imw, int imh) {


	int tmpm = (y*imw+x)*2;
	int tmpl = tmpm-1;
	int tmpr = tmpm+1;
	int tmpu = ((y-1)*imw+x)*2;
	int tmpd = ((y+1)*imw+x)*2;

	int ims = imw*imh*2;
	img[tmpm]=255;
	if (tmpl>0)
		img[tmpl] = 255;
	if (tmpu>0)
		img[tmpu] = 255;
	if (tmpr<ims)
		img[tmpr] = 255;
	if (tmpd<ims)
		img[tmpr] = 255;




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
