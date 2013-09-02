
#include "interface_sky_segmentation.h"
//#include "string.h"
#include "print.h"
#include "srv.h"
#include "config.h"
#include "sky_segmentation.h"

#define uint8_t unsigned char



/**********************************************************************
 AFSPRAKEN MET PAPARAZZI ./modules/onboardcam/onboardi2c.c
 *********************************************************************/

const int MAX_ROLL_ANGLE = 60;
const int MAX_PITCH_ANGLE = 40;
const int MAX_I2C_BYTE = 254;

typedef struct
{
  int pitch;
  int roll;
} paparazzi_state_struct;

typedef struct
{
  int pitch_pixel;
  int roll_angle;
  int error;
} vision_state_struct;

static inline uint8_t scale_to_range(int x, int min, int max, int range)
{
	if (x < min)
		x = min;
	else if (x > max)
		x = max;

	x -= min;
	x *= range;
	x /= (max - min);
	return (uint8_t) x;
}

static inline int unscale_from_range(uint8_t x, int range, int min, int max)
{
	int v = x;

	v *= (max - min);
	v /= range;
	v += min;

	return v;
}

/**********************************************************************
 *********************************************************************/

inline uint8_t scale_pitch_pixel_to_pwm(int x)
{
	return scale_to_range(x,-MAX_PITCH_ANGLE,MAX_PITCH_ANGLE,99);
}

inline uint8_t scale_angle_to_pwm(int x)
{
	return scale_to_range(x,-MAX_ROLL_ANGLE,MAX_ROLL_ANGLE,99);
}

const int INACCURATE = 50;
const int ACCURACY_RANGE = 250;
const int VFOV = 45;

static inline int pitch_angle_to_pitch_pixel(int pitch)
{
	int pitch_pixel = scale_to_range(pitch, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE, imgHeight);
	pitch_pixel -= imgHeight / 2;
	return pitch_pixel;
}

static inline void send_horizon_to_autopilot(vision_state_struct *x)
{
	unsigned int horizon_message[3];
	int pitch = (x->pitch_pixel * VFOV) / 120;
	//printf("xpp * VFOV = %d\n", (x->pitch_pixel * VFOV));
	//printf("pitch_pixel = %d, pitch = %d\n", x->pitch_pixel, pitch);
	// pitch -= VFOV / 2;
	// printf("pitch - after = %d\n", pitch);

	if (x->error > INACCURATE)
	{
		motor_command2(scale_pitch_pixel_to_pwm(0),scale_angle_to_pwm(0),0);
	}
	else
	{
		motor_command2(scale_pitch_pixel_to_pwm(pitch),scale_angle_to_pwm(x->roll_angle),0);
	}

	horizon_message[0] = scale_to_range(pitch, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE, MAX_I2C_BYTE);
	horizon_message[1] = scale_to_range(x->roll_angle, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE, MAX_I2C_BYTE);
	horizon_message[2] = x->error;

	sendToAutoPilot(horizon_message, 3);
}

static inline void send_obstacles_to_autopilot(int max_bin, int bin_total, unsigned int* obstacles, int n_bins)
{
	// send relevant variables to autopilot via ADC channels:
	motor_command2(scale_to_range(max_bin,0,MAX_I2C_BYTE,99), scale_to_range(bin_total,0,MAX_I2C_BYTE, 99),0);		
	
	// send all obstacle variables via UART:
	sendToAutoPilot(obstacles, n_bins);
}

static inline void get_state_from_autopilot(paparazzi_state_struct* x)
{
	unsigned char autopilot_message[12];

	getFromAutoPilot(autopilot_message);

	x->pitch = unscale_from_range(autopilot_message[3], MAX_I2C_BYTE, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
	x->roll = unscale_from_range(autopilot_message[4], MAX_I2C_BYTE, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
}

void interface_sky_segmentation(char ch1, char ch2) 
{

	/* COMMANDS
	*
	* ^: needs extra input, either for determining the numbers of samples, or 
	*    the adjustment factor.
	*
	* Produce Sky Segmented image
	* a: segment on the basis of minimal and maximal illumination (MM)
	* b: segment on the basis of a fixed YCV-tree (YCV)
	* c^: segment on the basis of an adjustable YCV-tree (aYCV)
	*
	* Detect Horizon and Show in Image
	* d: MM
	* e: YCV
	* f^: aYCV
	*
	* Continuous Horizon Detection for in Flight
	* g: MM, segmenting the entire image
	* h^: YCV, segmenting only the used pixels (random sampling)
	*
	* Continuous Horizon Detection for Determining Execution Frequency
	* i^: YCV
	*
	* Detecting obstacles in single frame for pre-flight testing
	* j: MM
	* k: YCV
	* l^: aYCV
	*
	* Detecting obstacles in a loop for during flight
	* m: MM
	* n^: aYCV
	*/

	paparazzi_state_struct state;
	vision_state_struct vision_state;

	int MAX_BIN_VALUE = MAX_I2C_BYTE;
	int it;
	int adjust_factor;

	unsigned char ch;
	unsigned int obstacles[N_BINS]; 
	unsigned int uncertainty[N_BINS];

	unsigned int bin, counter, max_count, print_frequency, max_bin, bin_total;   
	int sample_index;
	int n_train_samples, n_test_samples;

	printf("INTERFACE SKY SEGMENTATION\n");
	printf("Ch1 = .%c., Ch2 = .%c.\n", ch1, ch2);

	switch (ch1) {

	/******************

	 SKY SEGMENTATION	

	******************/

	case 'a': // segmenting the image on the basis of maximal and minimal illumination:
		grab_frame();
		segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		send_frame_nr(1);
		break;

	case 'b': // segmenting the image on the basis of YCV-tree:
		grab_frame();
		segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		send_frame_nr(1);
		break;

	case 'c': // segment with adjustable YCV-tree (slower)
			  // the c-command should be followed by a number in [0,9]
			  // lower numbers result in fewer sky-pixels
			  // higher numbers result in more sky-pixels
			  // In our experiments, c4 is often used.
		
		adjust_factor = 0;
		if(ch2 == 0)
			ch2 = getch();
		
		adjust_factor = atoi(&ch2);
		if(adjust_factor < 3)
		{
			if(adjust_factor == 0) adjust_factor = -10;
			if(adjust_factor == 1) adjust_factor = -5;
			if(adjust_factor == 2) adjust_factor = -2;
		}
		else if(adjust_factor == 3)
		{
			adjust_factor = 0;
		}
		else
		{
			if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
			if(adjust_factor == 7) adjust_factor = 5;
			if(adjust_factor == 8) adjust_factor = 8;
			if(adjust_factor == 9) adjust_factor = 11;
			if(adjust_factor == 10) adjust_factor = 15;
		}

		grab_frame();
		segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
		send_frame_nr(1);
		break;

	/********************************************

	 PITCH & ROLL ESTIMATION BY HORIZON DETECTION	

	********************************************/	

	// Single horizon estimation for pre-flight testing:

	case 'd': // segmenting the image on the basis of maximal and minimal illumination:
			  // estimate pitch and roll
			  // & draw a box around the region of interest
			  // & draw the horizon line
			  // & send the signal via two ADC channels

		grab_frame();
		segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
		send_horizon_to_autopilot(&vision_state);

		send_frame_nr(1);
		break;

	case 'e': // same as d, but with YCV-tree:
		grab_frame();
		segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);

		vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
		send_horizon_to_autopilot(&vision_state);

		send_frame_nr(1);
		break;

	case 'f': // as d,e, but with adjustable YCV-tree (slower)
			  // the c-command should be followed by a number in [0,9]
			  // lower numbers result in fewer sky-pixels
			  // higher numbers result in more sky-pixels
			  // In our experiments, c4 is often used.
		
		adjust_factor = 0;
		if(ch2 == 0)
			ch2 = getch();
		adjust_factor = atoi(&ch2);
		if(adjust_factor < 3)
		{
			if(adjust_factor == 0) adjust_factor = -10;
			if(adjust_factor == 1) adjust_factor = -5;
			if(adjust_factor == 2) adjust_factor = -2;
		}
		else if(adjust_factor == 3)
		{
			adjust_factor = 0;
		}
		else
		{
			if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
			if(adjust_factor == 7) adjust_factor = 5;
			if(adjust_factor == 8) adjust_factor = 8;
			if(adjust_factor == 9) adjust_factor = 11;
			if(adjust_factor == 10) adjust_factor = 15;
		}

		grab_frame();
		segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);

		vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
		send_horizon_to_autopilot(&vision_state);

		send_frame_nr(1);
		break;	

	// segment and determine horizon continuously for in flight:

	case 'g':	// determine the horizon in the image with BW

		for(it = 0; it < 60000; it++)
		{

			grab_frame(); // always grab a frame
			// always segment the entire image
			segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2); 

			if (getchar(&ch)) 
			{
				// if user input:
				switch (ch) 
			    {
					case 'm': // return to the main menu
				    	printf("Back to menu!\n\r");
					    return;
				    break;
					case 'I': // segment the entire image and show the horizon line:
						vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
						send_horizon_to_autopilot(&vision_state);

						send_frame_nr(1);
					break;
				}
			}
			else
			{
				// if no user input, determine the horizon and send it to the motors
				vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
				send_horizon_to_autopilot(&vision_state);

			}
		}
	break;

	case 'h':	// determine the horizon in the image efficiently with YCV-tree
				// and random sampling

		printf("h");

		sample_index = 0;
		if(ch2 == 0)
		{
			printf("GetCh()\n");
			ch2 = getch();
		}

		sample_index = atoi(&ch2);
		if(sample_index == 0)
		{
			n_train_samples = 45; n_test_samples = 5;
		}
		else if(sample_index == 1)
		{
			n_train_samples = 90; n_test_samples = 10;
		}
		else if(sample_index == 2)
		{
			n_train_samples = 225; n_test_samples = 25;
		}
		else if(sample_index == 3)
		{
			n_train_samples = 450; n_test_samples = 50;
		}
		else if(sample_index == 4)
		{
			n_train_samples = 900; n_test_samples = 100;
		}
		else if(sample_index == 5)
		{
			n_train_samples = 2400; n_test_samples = 100;
		}
		else if(sample_index == 6)
		{
			n_train_samples = 4900; n_test_samples = 100;
		}
		else
		{
			// full sampling:
			n_train_samples = 13900; n_test_samples = 100;
		}


		for(it = 0; it < 60000; it++)
		{
			grab_frame(); // always grab a frame

			if (getchar(&ch)) 
			{
				// if user input:
				switch (ch) 
			    {
					case 'm': // return to the main menu
				    	printf("Back to menu!\n\r");
					    return;
				    break;
					
					case 'I': // segment the entire image and show the horizon line:

						segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);

						vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
						send_horizon_to_autopilot(&vision_state);

						send_frame_nr(1);
					break;
				}
			}
			else
			{
				vision_state.error = perceptronPitchRollEfficient((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), 
					n_train_samples, n_test_samples, ACCURACY_RANGE);
/*
				segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
				vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL, ACCURACY_RANGE);
*/

				// vision_state.pitch_pixel = imgHeight / 4;
				// vision_state.roll_angle = -30;

				send_horizon_to_autopilot(&vision_state);

				if(vision_state.error < INACCURATE)
				{
					printf(".");
				}
				else
				{
					printf("-%d-", vision_state.error);
				}

			}
		}
	break;


	// segment and determine horizon continuously for determining the execution frequency:	
	// the test consists of running this script for a few minutes and noting the last iteration
	// number.

	case 'i':	// determine the horizon in the image efficiently with YCV-tree
				// and random sampling.

		sample_index = 0;
		ch = getch();
		sample_index = atoi(&ch);
		if(sample_index == 0)
		{
			n_train_samples = 45; n_test_samples = 5;
		}
		else if(sample_index == 1)
		{
			n_train_samples = 90; n_test_samples = 10;
		}
		else if(sample_index == 2)
		{
			n_train_samples = 225; n_test_samples = 25;
		}
		else if(sample_index == 3)
		{
			n_train_samples = 450; n_test_samples = 50;
		}
		else if(sample_index == 4)
		{
			n_train_samples = 900; n_test_samples = 100;
		}
		else if(sample_index == 5)
		{
			n_train_samples = 2400; n_test_samples = 100;
		}
		else if(sample_index == 6)
		{
			n_train_samples = 4900; n_test_samples = 100;
		}
		else
		{
			// full sampling:
			n_train_samples = 13900; n_test_samples = 100;
		}


		for(it = 0; it < 10000; it++)
		{
			grab_frame(); // always grab a frame

			if(it % 10 == 0) printf("*%d*", it);
			else printf(".");

			vision_state.error = perceptronPitchRollEfficient((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), 
				n_train_samples, n_test_samples, ACCURACY_RANGE);
			send_horizon_to_autopilot(&vision_state);
		}
	break;

	/**********************************************

	SKY SEGMENTATION APPROACH TO OBSTACLE AVOIDANCE	

	***********************************************/	

	// single-frame for pre-flight testing:

	case 'j': // segmentation on the basis of minimal and maximal illumination
		grab_frame();
		get_state_from_autopilot(&state);
		segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);		
		// determine the amount of obstacle per orientation segment
		//getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		
		getObstacles2Way(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE, pitch_angle_to_pitch_pixel(state.pitch), state.roll);

		/*printf("*od*"); // protocol start for obstacle info
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", obstacles[bin]);
		}
		printf("u");
		// determine the amount of uncertainty in the segmentation per orientation segment
		getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", uncertainty[bin]);
		}
		printf("s\n"); // protocol end
*/
		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);

		//int a,b;
		//horizonToLineParameters(pitch_angle_to_pitch_pixel(state.pitch), state.roll, &a, &b);
		// 1000 is the resolution of the tan-function
		//drawLine((unsigned char *)FRAME_BUF, a, b, 1000); 
		send_frame(1);

		break;

	case 'k': // segmentation on the basis of fixed YCV-tree
		grab_frame();
		get_state_from_autopilot(&state);
		segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		// determine the amount of obstacle per orientation segment
		getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		printf("*od*"); // protocol start for obstacle info
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", obstacles[bin]);
		}
		printf("u");
		// determine the amount of uncertainty in the segmentation per orientation segment
		getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", uncertainty[bin]);
		}
		printf("s\n"); // protocol end

		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);
		break;

	case 'l': // with adjustable YCV-tree

		adjust_factor = 0;
		if(ch2 == 0)
			ch2 = getch(); // getchar(&ch) was NOT WORKING!		
		adjust_factor = atoi(&ch2);
		if(adjust_factor < 3)
		{
			if(adjust_factor == 0) adjust_factor = -10;
			if(adjust_factor == 1) adjust_factor = -5;
			if(adjust_factor == 2) adjust_factor = -2;
		}
		else if(adjust_factor == 3)
		{
			adjust_factor = 0;
		}
		else
		{
			if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
			if(adjust_factor == 7) adjust_factor = 5;
			if(adjust_factor == 8) adjust_factor = 8;
			if(adjust_factor == 9) adjust_factor = 11;
			if(adjust_factor == 10) adjust_factor = 15;
		}

		grab_frame();
		get_state_from_autopilot(&state);
		segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);

		// determine the amount of obstacle per orientation segment
		getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		printf("*od*"); // protocol start for obstacle info
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", obstacles[bin]);
		}
		printf("u");
		// determine the amount of uncertainty in the segmentation per orientation segment
		getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", uncertainty[bin]);
		}
		printf("s\n"); // protocol end

		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);
		break;

	// continuous segmentation and obstacle detection for in flight:

	case 'm': // with maximal and minimal illumination:
		max_count = 60000;
		print_frequency = 60;
		for(counter = 0; counter < max_count; counter++)
		{
		    grab_frame();
			get_state_from_autopilot(&state);
			segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
			
	    	// determine the amount of obstacle per orientation segment
	    	getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);
		    if(counter % print_frequency == 0)
		    {
				printf("*od*"); // protocol start for obstacle info
				for(bin = 0; bin < N_BINS; bin++)
				{
				    printf("%d,", obstacles[bin]);
				}
		    	printf("u");
		    	// determine the amount of uncertainty in the segmentation per orientation segment
		    	getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		    	for(bin = 0; bin < N_BINS; bin++)
		    	{
		    	    printf("%d,", uncertainty[bin]);
		    	}
		    	printf("s\n"); // protocol end
		    }
		}
		break;
	case 'n': // with adjustable tree:

		adjust_factor = 0;
		if(ch2 == 0)
			ch2 = getch(); // getchar(&ch) was NOT WORKING!		
		//printf("%c=", ch);
		adjust_factor = atoi(&ch2);
		if(adjust_factor < 3)
		{
			if(adjust_factor == 0) adjust_factor = -10;
			if(adjust_factor == 1) adjust_factor = -5;
			if(adjust_factor == 2) adjust_factor = -2;
		}
		else if(adjust_factor == 3)
		{
			adjust_factor = 0;
		}
		else
		{
			if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
			if(adjust_factor == 7) adjust_factor = 5;
			if(adjust_factor == 8) adjust_factor = 8;
			if(adjust_factor == 9) adjust_factor = 11;
			if(adjust_factor == 10) adjust_factor = 15;
		}

		max_count = 60000;
		print_frequency = 5;

		for(counter = 0; counter < max_count; counter++)
		{
			grab_frame();
			get_state_from_autopilot(&state);
			segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
			//segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
	    	// determine the amount of obstacle per orientation segment
	    	getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);
		    if(counter % print_frequency == 0)
		    {
				printf("*od*"); // protocol start for obstacle info
				for(bin = 0; bin < N_BINS; bin++)
				{
				    printf("%d,", obstacles[bin]);
				}
		    	printf("u");
		    	// determine the amount of uncertainty in the segmentation per orientation segment
		    	getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		    	for(bin = 0; bin < N_BINS; bin++)
		    	{
		    	    printf("%d,", uncertainty[bin]);
		    	}
		    	printf("s\n"); // protocol end
		    }
		}
		break;
	case 'z': // test communication
		vision_state.error = 10;
		vision_state.roll_angle = 0;
		vision_state.pitch_pixel = 0;
		send_horizon_to_autopilot(&vision_state);
		get_state_from_autopilot(&state);
		printf("state.pitch = %d, state.roll = %d, pitch_pixels = %d\n\r", state.pitch, state.roll, pitch_angle_to_pitch_pixel(state.pitch));
		break;

    } // end switch statement

    return; // return to main menu
}

void interface_sky_segmentation(char ch1, char ch2) 
{

	/* COMMANDS
	*
	* ^: needs extra input, either for determining the numbers of samples, or 
	*    the adjustment factor.
	*
	* Produce Sky Segmented image
	* a: segment on the basis of minimal and maximal illumination (MM)
	* b: segment on the basis of a fixed YCV-tree (YCV)
	* c^: segment on the basis of an adjustable YCV-tree (aYCV)
	*
	* Detect Horizon and Show in Image
	* d: MM
	* e: YCV
	* f^: aYCV
	*
	* Continuous Horizon Detection for in Flight
	* g: MM, segmenting the entire image
	* h^: YCV, segmenting only the used pixels (random sampling)
	*
	* Continuous Horizon Detection for Determining Execution Frequency
	* i^: YCV
	*
	* Detecting obstacles in single frame for pre-flight testing
	* j: MM
	* k: YCV
	* l^: aYCV
	*
	* Detecting obstacles in a loop for during flight
	* m: MM
	* n^: aYCV
	*/

	paparazzi_state_struct state;
	vision_state_struct vision_state;

	int MAX_BIN_VALUE = MAX_I2C_BYTE;
	int it;
	int adjust_factor;

	unsigned char ch;
	unsigned int obstacles[N_BINS]; 
	unsigned int uncertainty[N_BINS];

	unsigned int bin, counter, max_count, print_frequency, max_bin, bin_total;   
	int sample_index;
	int n_train_samples, n_test_samples;

	printf("INTERFACE SKY SEGMENTATION\n");
	printf("Ch1 = .%c., Ch2 = .%c.\n", ch1, ch2);

	switch (ch1) {

	/******************

	 SKY SEGMENTATION	

	******************/

	case 'a': // segmenting the image on the basis of maximal and minimal illumination:
		grab_frame();
		segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		send_frame_nr(1);
		break;

	case 'b': // segmenting the image on the basis of YCV-tree:
		grab_frame();
		segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		send_frame_nr(1);
		break;

	case 'c': // segment with adjustable YCV-tree (slower)
			  // the c-command should be followed by a number in [0,9]
			  // lower numbers result in fewer sky-pixels
			  // higher numbers result in more sky-pixels
			  // In our experiments, c4 is often used.
		
		adjust_factor = 0;
		if(ch2 == 0)
			ch2 = getch();
		
		adjust_factor = atoi(&ch2);
		if(adjust_factor < 3)
		{
			if(adjust_factor == 0) adjust_factor = -10;
			if(adjust_factor == 1) adjust_factor = -5;
			if(adjust_factor == 2) adjust_factor = -2;
		}
		else if(adjust_factor == 3)
		{
			adjust_factor = 0;
		}
		else
		{
			if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
			if(adjust_factor == 7) adjust_factor = 5;
			if(adjust_factor == 8) adjust_factor = 8;
			if(adjust_factor == 9) adjust_factor = 11;
			if(adjust_factor == 10) adjust_factor = 15;
		}

		grab_frame();
		segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
		send_frame_nr(1);
		break;

	/********************************************

	 PITCH & ROLL ESTIMATION BY HORIZON DETECTION	

	********************************************/	

	// Single horizon estimation for pre-flight testing:

	case 'd': // segmenting the image on the basis of maximal and minimal illumination:
			  // estimate pitch and roll
			  // & draw a box around the region of interest
			  // & draw the horizon line
			  // & send the signal via two ADC channels

		grab_frame();
		segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
		send_horizon_to_autopilot(&vision_state);

		send_frame_nr(1);
		break;

	case 'e': // same as d, but with YCV-tree:
		grab_frame();
		segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);

		vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
		send_horizon_to_autopilot(&vision_state);

		send_frame_nr(1);
		break;

	case 'f': // as d,e, but with adjustable YCV-tree (slower)
			  // the c-command should be followed by a number in [0,9]
			  // lower numbers result in fewer sky-pixels
			  // higher numbers result in more sky-pixels
			  // In our experiments, c4 is often used.
		
		adjust_factor = 0;
		if(ch2 == 0)
			ch2 = getch();
		adjust_factor = atoi(&ch2);
		if(adjust_factor < 3)
		{
			if(adjust_factor == 0) adjust_factor = -10;
			if(adjust_factor == 1) adjust_factor = -5;
			if(adjust_factor == 2) adjust_factor = -2;
		}
		else if(adjust_factor == 3)
		{
			adjust_factor = 0;
		}
		else
		{
			if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
			if(adjust_factor == 7) adjust_factor = 5;
			if(adjust_factor == 8) adjust_factor = 8;
			if(adjust_factor == 9) adjust_factor = 11;
			if(adjust_factor == 10) adjust_factor = 15;
		}

		grab_frame();
		segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);

		vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
		send_horizon_to_autopilot(&vision_state);

		send_frame_nr(1);
		break;	

	// segment and determine horizon continuously for in flight:

	case 'g':	// determine the horizon in the image with BW

		for(it = 0; it < 60000; it++)
		{

			grab_frame(); // always grab a frame
			// always segment the entire image
			segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2); 

			if (getchar(&ch)) 
			{
				// if user input:
				switch (ch) 
			    {
					case 'm': // return to the main menu
				    	printf("Back to menu!\n\r");
					    return;
				    break;
					case 'I': // segment the entire image and show the horizon line:
						vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
						send_horizon_to_autopilot(&vision_state);

						send_frame_nr(1);
					break;
				}
			}
			else
			{
				// if no user input, determine the horizon and send it to the motors
				vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
				send_horizon_to_autopilot(&vision_state);

			}
		}
	break;

	case 'h':	// determine the horizon in the image efficiently with YCV-tree
				// and random sampling

		printf("h");

		sample_index = 0;
		if(ch2 == 0)
		{
			printf("GetCh()\n");
			ch2 = getch();
		}

		sample_index = atoi(&ch2);
		if(sample_index == 0)
		{
			n_train_samples = 45; n_test_samples = 5;
		}
		else if(sample_index == 1)
		{
			n_train_samples = 90; n_test_samples = 10;
		}
		else if(sample_index == 2)
		{
			n_train_samples = 225; n_test_samples = 25;
		}
		else if(sample_index == 3)
		{
			n_train_samples = 450; n_test_samples = 50;
		}
		else if(sample_index == 4)
		{
			n_train_samples = 900; n_test_samples = 100;
		}
		else if(sample_index == 5)
		{
			n_train_samples = 2400; n_test_samples = 100;
		}
		else if(sample_index == 6)
		{
			n_train_samples = 4900; n_test_samples = 100;
		}
		else
		{
			// full sampling:
			n_train_samples = 13900; n_test_samples = 100;
		}


		for(it = 0; it < 60000; it++)
		{
			grab_frame(); // always grab a frame

			if (getchar(&ch)) 
			{
				// if user input:
				switch (ch) 
			    {
					case 'm': // return to the main menu
				    	printf("Back to menu!\n\r");
					    return;
				    break;
					
					case 'I': // segment the entire image and show the horizon line:

						segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);

						vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), ACCURACY_RANGE);
						send_horizon_to_autopilot(&vision_state);

						send_frame_nr(1);
					break;
				}
			}
			else
			{
				vision_state.error = perceptronPitchRollEfficient((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), 
					n_train_samples, n_test_samples, ACCURACY_RANGE);
/*
				segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
				vision_state.error = perceptronPitchRoll((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL, ACCURACY_RANGE);
*/

				// vision_state.pitch_pixel = imgHeight / 4;
				// vision_state.roll_angle = -30;

				send_horizon_to_autopilot(&vision_state);

				if(vision_state.error < INACCURATE)
				{
					printf(".");
				}
				else
				{
					printf("-%d-", vision_state.error);
				}

			}
		}
	break;


	// segment and determine horizon continuously for determining the execution frequency:	
	// the test consists of running this script for a few minutes and noting the last iteration
	// number.

	case 'i':	// determine the horizon in the image efficiently with YCV-tree
				// and random sampling.

		sample_index = 0;
		ch = getch();
		sample_index = atoi(&ch);
		if(sample_index == 0)
		{
			n_train_samples = 45; n_test_samples = 5;
		}
		else if(sample_index == 1)
		{
			n_train_samples = 90; n_test_samples = 10;
		}
		else if(sample_index == 2)
		{
			n_train_samples = 225; n_test_samples = 25;
		}
		else if(sample_index == 3)
		{
			n_train_samples = 450; n_test_samples = 50;
		}
		else if(sample_index == 4)
		{
			n_train_samples = 900; n_test_samples = 100;
		}
		else if(sample_index == 5)
		{
			n_train_samples = 2400; n_test_samples = 100;
		}
		else if(sample_index == 6)
		{
			n_train_samples = 4900; n_test_samples = 100;
		}
		else
		{
			// full sampling:
			n_train_samples = 13900; n_test_samples = 100;
		}


		for(it = 0; it < 10000; it++)
		{
			grab_frame(); // always grab a frame

			if(it % 10 == 0) printf("*%d*", it);
			else printf(".");

			vision_state.error = perceptronPitchRollEfficient((unsigned char *)FRAME_BUF, &(vision_state.pitch_pixel), &(vision_state.roll_angle), 
				n_train_samples, n_test_samples, ACCURACY_RANGE);
			send_horizon_to_autopilot(&vision_state);
		}
	break;

	/**********************************************

	SKY SEGMENTATION APPROACH TO OBSTACLE AVOIDANCE	

	***********************************************/	

	// single-frame for pre-flight testing:

	case 'j': // segmentation on the basis of minimal and maximal illumination
		grab_frame();
		get_state_from_autopilot(&state);
		segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);		
		// determine the amount of obstacle per orientation segment
		//getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		
		getObstacles2Way(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE, pitch_angle_to_pitch_pixel(state.pitch), state.roll);

		/*printf("*od*"); // protocol start for obstacle info
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", obstacles[bin]);
		}
		printf("u");
		// determine the amount of uncertainty in the segmentation per orientation segment
		getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", uncertainty[bin]);
		}
		printf("s\n"); // protocol end
*/
		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);

		//int a,b;
		//horizonToLineParameters(pitch_angle_to_pitch_pixel(state.pitch), state.roll, &a, &b);
		// 1000 is the resolution of the tan-function
		//drawLine((unsigned char *)FRAME_BUF, a, b, 1000); 
		send_frame(1);

		break;

	case 'k': // segmentation on the basis of fixed YCV-tree
		grab_frame();
		get_state_from_autopilot(&state);
		segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		// determine the amount of obstacle per orientation segment
		getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		printf("*od*"); // protocol start for obstacle info
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", obstacles[bin]);
		}
		printf("u");
		// determine the amount of uncertainty in the segmentation per orientation segment
		getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", uncertainty[bin]);
		}
		printf("s\n"); // protocol end

		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);
		break;

	case 'l': // with adjustable YCV-tree

		adjust_factor = 0;
		if(ch2 == 0)
			ch2 = getch(); // getchar(&ch) was NOT WORKING!		
		adjust_factor = atoi(&ch2);
		if(adjust_factor < 3)
		{
			if(adjust_factor == 0) adjust_factor = -10;
			if(adjust_factor == 1) adjust_factor = -5;
			if(adjust_factor == 2) adjust_factor = -2;
		}
		else if(adjust_factor == 3)
		{
			adjust_factor = 0;
		}
		else
		{
			if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
			if(adjust_factor == 7) adjust_factor = 5;
			if(adjust_factor == 8) adjust_factor = 8;
			if(adjust_factor == 9) adjust_factor = 11;
			if(adjust_factor == 10) adjust_factor = 15;
		}

		grab_frame();
		get_state_from_autopilot(&state);
		segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);

		// determine the amount of obstacle per orientation segment
		getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		printf("*od*"); // protocol start for obstacle info
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", obstacles[bin]);
		}
		printf("u");
		// determine the amount of uncertainty in the segmentation per orientation segment
		getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		for(bin = 0; bin < N_BINS; bin++)
		{
		    printf("%d,", uncertainty[bin]);
		}
		printf("s\n"); // protocol end

		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);
		break;

	// continuous segmentation and obstacle detection for in flight:

	case 'm': // with maximal and minimal illumination:
		max_count = 60000;
		print_frequency = 60;
		for(counter = 0; counter < max_count; counter++)
		{
		    grab_frame();
			get_state_from_autopilot(&state);
			segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
			
	    	// determine the amount of obstacle per orientation segment
	    	getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);
		    if(counter % print_frequency == 0)
		    {
				printf("*od*"); // protocol start for obstacle info
				for(bin = 0; bin < N_BINS; bin++)
				{
				    printf("%d,", obstacles[bin]);
				}
		    	printf("u");
		    	// determine the amount of uncertainty in the segmentation per orientation segment
		    	getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		    	for(bin = 0; bin < N_BINS; bin++)
		    	{
		    	    printf("%d,", uncertainty[bin]);
		    	}
		    	printf("s\n"); // protocol end
		    }
		}
		break;
	case 'n': // with adjustable tree:

		adjust_factor = 0;
		if(ch2 == 0)
			ch2 = getch(); // getchar(&ch) was NOT WORKING!		
		//printf("%c=", ch);
		adjust_factor = atoi(&ch2);
		if(adjust_factor < 3)
		{
			if(adjust_factor == 0) adjust_factor = -10;
			if(adjust_factor == 1) adjust_factor = -5;
			if(adjust_factor == 2) adjust_factor = -2;
		}
		else if(adjust_factor == 3)
		{
			adjust_factor = 0;
		}
		else
		{
			if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
			if(adjust_factor == 7) adjust_factor = 5;
			if(adjust_factor == 8) adjust_factor = 8;
			if(adjust_factor == 9) adjust_factor = 11;
			if(adjust_factor == 10) adjust_factor = 15;
		}

		max_count = 60000;
		print_frequency = 5;

		for(counter = 0; counter < max_count; counter++)
		{
			grab_frame();
			get_state_from_autopilot(&state);
			segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
			//segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
	    	// determine the amount of obstacle per orientation segment
	    	getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_BIN_VALUE);
		send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS);
		    if(counter % print_frequency == 0)
		    {
				printf("*od*"); // protocol start for obstacle info
				for(bin = 0; bin < N_BINS; bin++)
				{
				    printf("%d,", obstacles[bin]);
				}
		    	printf("u");
		    	// determine the amount of uncertainty in the segmentation per orientation segment
		    	getUncertainty(uncertainty, N_BINS, (unsigned char *)FRAME_BUF2);
		    	for(bin = 0; bin < N_BINS; bin++)
		    	{
		    	    printf("%d,", uncertainty[bin]);
		    	}
		    	printf("s\n"); // protocol end
		    }
		}
		break;
	case 'z': // test communication
		vision_state.error = 10;
		vision_state.roll_angle = 0;
		vision_state.pitch_pixel = 0;
		send_horizon_to_autopilot(&vision_state);
		get_state_from_autopilot(&state);
		printf("state.pitch = %d, state.roll = %d, pitch_pixels = %d\n\r", state.pitch, state.roll, pitch_angle_to_pitch_pixel(state.pitch));
		break;

    } // end switch statement

    return; // return to main menu
}
