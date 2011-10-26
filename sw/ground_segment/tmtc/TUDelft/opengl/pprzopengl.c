/*
 * An example of using GtkGLExt and Ivy in C
 *
 * 
 */

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <GL/gl.h>
#include <gdk/gdkx.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <getopt.h>

static GLfloat pitch = 0.f;
static GLfloat roll = 0.f;
static double altitude =0.0;
static double speed=0.0;
static double vario=0.0;
static double heading=0.0;


void openglinit()
{
			
  glClearDepth(1.0);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glMatrixMode(GL_PROJECTION);
  glOrtho(-1.0,1.0,-1.0,1.0,-10,10);
  glMatrixMode(GL_MODELVIEW);

}

void openglrender()
{
  glClear(GL_DEPTH_BUFFER_BIT);
glClear(GL_COLOR_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  glBegin(GL_POLYGON);
  glColor3f(0.0, 0.0, 0.0);
  glVertex3f(-1, 1, -1);
  glVertex3f(1, 1, -1);
  glColor3f(0.0, 0.0, 1.0);
  glVertex3f(1, -1, -1);
  glVertex3f(-1, -1, -1);
  glEnd();
glPushMatrix();
     glTranslatef(0.0, 0.0, 1.0);
      glRotatef(290.0, 1.0, 0.0, 0.0);
glRotatef(pitch, 1.0, 0.0, 0.0);
glRotatef(roll, 0.0, 1.0, 0.0);
glRotatef(heading, 0.0, 0.0, -1.0);
glScalef(1.0 / 6.0, 1.0 / 8.0, 1.0 / 8.0);
      glTranslatef(0.0, -4.0, -1.5);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_FLAT);
glBegin(GL_TRIANGLE_STRIP);
glColor3f(1.0, 1.0,0.0);
glVertex3f(-7.0,0.0,2.0);
glVertex3f(-1.0,0.0,3.0);
glVertex3f(-1.0,7.0,3.0);
glVertex3f(0.0,0.0,0.0);
glVertex3f(0.0,8.0,0.0);
glVertex3f(1.0,0.0,3.0);
glVertex3f(1.0,7.0,3.0);
glVertex3f(7.0,0.0,2.0);
      glEnd();
	
        glPopMatrix();
	glFlush ();
}

static gboolean
expose (GtkWidget *da, GdkEventExpose *event, gpointer user_data)
{
	GdkGLContext *glcontext = gtk_widget_get_gl_context (da);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (da);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
		g_assert_not_reached ();
	}

	/* draw in here */
	openglrender();

	/* end */


	if (gdk_gl_drawable_is_double_buffered (gldrawable))
		gdk_gl_drawable_swap_buffers (gldrawable);

	else
		glFlush ();

	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}

static gboolean
configure (GtkWidget *da, GdkEventConfigure *event, gpointer user_data)
{
	GdkGLContext *glcontext = gtk_widget_get_gl_context (da);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (da);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
		g_assert_not_reached ();
	}

	glLoadIdentity();
	glViewport (0, 0, da->allocation.width, da->allocation.height);
	openglinit();
	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}

static gboolean
update (gpointer user_data)
{
	GtkWidget *da = GTK_WIDGET (user_data);

	//update code starts here

	//end of update code

	gdk_window_invalidate_rect (da->window, &da->allocation, FALSE);
	gdk_window_process_updates (da->window, FALSE);

	return TRUE;
}

/* jump here when a GPS message is received */
void on_GPS_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  
  heading= atof(argv[3])/10.;
  altitude = atof(argv[4])/100.;
  speed = atof(argv[5])/100.;
  vario = atof(argv[6])/100.;
  printf("got a gps message heading %f  altitude %f speed %f vario %f   \n ",heading,altitude,speed,vario);
}



/* jump here when a ATTITUDE message is received */
void on_ATTITUDE(IvyClientPtr app, void *user_data, int argc, char *argv[]){

    roll = atof(argv[0])*180./3.14159;
    pitch = atof(argv[2])*180./3.14159;
    printf("got an attitude message pitch %f   roll %f   \n ",pitch,roll);
}


int main (int argc, char **argv)
{
	GtkWidget *window;
	GtkWidget *da;
	GdkGLConfig *glconfig;
	int xid=-1;

	/* handling of -b option */
	const char* bus = 0;
	char c;

	while ((c = getopt (argc, argv, "b:p:")) != EOF) {
		switch (c) {
		case 'b':
			bus = optarg;
			break;
		case 'p':
			xid = strtol(optarg,NULL,0);
			break;
		}
	}
	printf("xid %d \nbus %s \n",xid,bus);

	/* handling of environment variable */
	if (!bus) bus = getenv ("IVYBUS");



	gtk_init (&argc, &argv);
	gtk_gl_init (&argc, &argv);

	if (xid==-1){	
		window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
		gtk_window_set_default_size (GTK_WINDOW (window), 512, 400);		
 	}
	else {
		window = gtk_window_new(GTK_WINDOW_POPUP);
	}
	
		
	da = gtk_drawing_area_new ();


	gtk_container_add (GTK_CONTAINER (window), da);
	g_signal_connect_swapped (window, "destroy",
			G_CALLBACK (gtk_main_quit), NULL);
	gtk_widget_set_events (da, GDK_EXPOSURE_MASK);

	gtk_widget_show (window);

	if (xid!=-1){
		XReparentWindow(GDK_WINDOW_XDISPLAY(window->window), GDK_WINDOW_XWINDOW(window->window), xid, 0,0);
		gtk_widget_map(window); 
	}

	/* prepare GL */
	glconfig = gdk_gl_config_new_by_mode (
			GDK_GL_MODE_RGB |
			GDK_GL_MODE_DEPTH |
			GDK_GL_MODE_DOUBLE);

	if (!glconfig)
	{
		g_assert_not_reached ();
	}

	if (!gtk_widget_set_gl_capability (da, glconfig, NULL, TRUE,
				GDK_GL_RGBA_TYPE))
	{
		g_assert_not_reached ();
	}

	g_signal_connect (da, "configure-event",
			G_CALLBACK (configure), NULL);
	g_signal_connect (da, "expose-event",
			G_CALLBACK (expose), NULL);

	gtk_widget_show_all (window);

	g_timeout_add (1000 / 60, update, da);

	IvyInit ("pprzopengl", "pprzopengl READY", NULL, NULL, NULL, NULL);
 	IvyBindMsg(on_GPS_STATUS, NULL, "^\\S* GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
 	IvyBindMsg(on_ATTITUDE, NULL, "^\\S* ATTITUDE (\\S*) (\\S*) (\\S*)");
 	IvyStart(bus);

	gtk_main ();
	return(0);
}
