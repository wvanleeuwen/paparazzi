#include "myfunc.h"
#include "print.h"
#include "srv.h"
#include "config.h"
#include "sky_segmentation.h"


void myfunc() {
    unsigned char ch;
	unsigned char message_from_autopilot[12];
    unsigned int obstacles[N_BINS]; 
    unsigned int uncertainty[N_BINS];
    unsigned int bin, counter, max_count, print_frequency, max_bin, bin_total;   
	// PWM:
    int OFF_SET = 0;
    int MAX_SIGNAL = 100;
	int INACCURATE = 20;
	unsigned int delay = 0;
	// find Corners:
	int MAX_POINTS, error,p;
	int x[40], new_x[40];
	int y[40], new_y[40];
	int status[40];
	int suppression_distance_squared,n_found_points,mark_points;
	int size_in_bytes, width, height, half_window_size, max_iterations;
	int it;
	// pitch roll:
	int pitch, roll, err;
	int adjust_factor;

    ch = getch();
    switch (ch) {
	case 'b':	// determine the horizon in the image

		adjust_factor = 0;
		ch = getch(); // getchar(&ch) was NOT WORKING!		
		//printf("%c=", ch);
		adjust_factor = atoi(&ch);
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

		//while(1)
		//{
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
						err = perceptronPitchRoll((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL);
						if(err < INACCURATE)
							motor_command2(pitch+OFF_SET, roll+OFF_SET, delay);
						else
							motor_command2(MAX_SIGNAL/2, MAX_SIGNAL/2,delay);								


						send_frame_nr(1);
							
						//printf("e=%d,p=%d,r=%d\n\r", err, pitch, roll);
					break;
				}
			}
			else
			{
				// if no user input, determine the horizon and send it to the motors
				// err = perceptronPitchRollEfficient((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL);
				segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
				err = perceptronPitchRoll((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL);

				if(err < INACCURATE)
				{
					// only send the state if we are sufficiently certain:
					motor_command2(pitch+OFF_SET, roll+OFF_SET, delay);
				}
				else
				{
					motor_command2(MAX_SIGNAL/2, MAX_SIGNAL/2,delay);
					printf("-%d-", err);
				}
			}
		}
		//}
	break;
	case 'c':	// determine the horizon in the image with BW

		//while(1)
		//{
		for(it = 0; it < 60000; it++)
		{
			grab_frame(); // always grab a frame
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

						err = perceptronPitchRoll((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL);
						if(err < INACCURATE)
							motor_command2(pitch+OFF_SET, roll+OFF_SET, delay);
						else
							motor_command2(MAX_SIGNAL/2, MAX_SIGNAL/2,delay);
						send_frame_nr(1);
							
						//printf("e=%d,p=%d,r=%d\n\r", err, pitch, roll);
					break;
				}
			}
			else
			{

				// if no user input, determine the horizon and send it to the motors
				err = perceptronPitchRoll((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL);
				if(err < INACCURATE)
				{
					// only send the state if we are sufficiently certain:
					motor_command2(pitch+OFF_SET, roll+OFF_SET, delay);
					printf(".");
				}
				else
				{
					motor_command2(MAX_SIGNAL/2, MAX_SIGNAL/2,delay);
					printf("-%d-", err);
				}
			}
		}
		//}
	break;

	case 'd':
		// delay instellen?
		for(it = 0; it < 1000; it++)
		motor_command2(90, 90, delay);
		for(it = 0; it < 1000; it++)
		motor_command2(70, 70, delay);
		for(it = 0; it < 1000; it++)
		motor_command2(50, 50, delay);
		for(it = 0; it < 1000; it++)
		motor_command2(30, 30, delay);
		for(it = 0; it < 1000; it++)
		motor_command2(0, 0, delay);
		for(it = 0; it < 1000; it++)
		motor_command2(90, 90, delay);

	break;
	case 'a':
		/*	   
		 // track the target
	    grab_frame();
	    int coords[2];
	    findCenter((unsigned char *)FRAME_BUF, 0, &coords);
	    // led indicates left / right
	    if(coords[0] < imgWidth / 2)
	    {
	        led0_on();
	    }
	    else
	    {
	        led1_on();
	    }
	    // send PWM signals
	    int cs[2];
	    cs[0] = ((coords[0] * MAX_SIGNAL) / imgWidth) + OFF_SET;
	    cs[1] = ((coords[1] * MAX_SIGNAL) / imgHeight) + OFF_SET;
	    motor_command2(cs[0], cs[1], delay);
	    // communicate to the ground station:
	    printf("c%d,%d,s\n\r", coords[0], coords[1]);
		*/
	  clearRTC();
	    //while(1)
	    //{
	    	grab_frame();
		int coords[2];
	    	findCenter((unsigned char *)FRAME_BUF, 0, &coords);
		printf("coords[0] = %d, coords[1] = %d\n\r", coords[0], coords[1]);
		if(coords[0] < imgWidth / 2)
		{
		    led0_on();
			printf("0");
/*		    if(coords[1] < imgHeight / 2) led0_on();
		    else led1_on();*/
		}
		else
		{
		    led1_on();
			printf("1");
/*		    if(coords[1] < imgHeight / 2) ;
		    else ;*/
		}

		int cs[2];
		cs[0] = ((coords[0] * MAX_SIGNAL) / imgWidth) + OFF_SET;
		cs[1] = ((coords[1] * MAX_SIGNAL) / imgHeight) + OFF_SET;
		//printf("coords[0] = %d, coords[1] = %d\n\r", cs[0], cs[1]);
		motor_command2(cs[0], cs[1], delay);

		/*int ms = readRTC();
		if(ms > 5000)
		{
		    printf("\n\n*** IMAGE ***\n\n");
		    clearRTC();
		}*/


		if (getchar(&ch)) 
		{
            	    switch (ch) 
		    {
                	case 'I':
                    	    send_frame();
		            break;
	                case 'a':   // 160 x 120
        	            camera_reset(160);
        	            break;
        	        case 'b':   // 320 x 240
        	            camera_reset(320);
        	            break;
        	        case 'c':   // 640 x 480
        	            camera_reset(640);
        	            break;
			case 'p': // print a sign of life
			    printf("Hello!\n\r");
			    break;
			case 'm': // return to the main menu
			    printf("Back to menu!\n\r");
			    return;
			    break;
		    }
		}

	    //}
	break;
    case 'h':   // test myfunc - string is "%h"
        printf("myfunc:  hello Guido ! \r\n");
        break;
/*	case 'c':   // find corners:
		grab_frame();
		MAX_POINTS = 40;
		suppression_distance_squared = 3 * 3;
		mark_points = 1;
		width = imgWidth;
		height = imgHeight;
		size_in_bytes = width*height*2;
*/
/*
		error = findCorners((unsigned char *) FRAME_BUF, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points);
		send_frame_nr(1);
*/
		//putThumbnailInFrame((unsigned char *) FRAME_BUF, (unsigned char *) FRAME_BUF2, &size_in_bytes, &width, &height, 4);
		//printf("width = %d, height = %d\n\r", width, height);
		//error = findCorners((unsigned char *) FRAME_BUF2, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points, width, height);		
		//send_frame_nr_size(2, size_in_bytes, width, height);
/*		error = findCorners((unsigned char *) FRAME_BUF, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points, width, height);	
		send_frame_nr(1);	
		break;
*/
/*	case 'f': // optic flow
		MAX_POINTS = 40;
		suppression_distance_squared = 3 * 3;
		half_window_size = 5;
		max_iterations = 20;
		mark_points = 0;
		width = imgWidth;
		height = imgHeight;
		size_in_bytes = width*height*2;
		
		// like this, the frames are relatively close to each other:
		grab_frame();
		copy_image((unsigned char *) FRAME_BUF,(unsigned char *) FRAME_BUF2, (unsigned int)width, (unsigned int) height);

		error = findCorners((unsigned char *) FRAME_BUF2, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points, width, height);		
		grab_frame(); // grab a new frame and store it in FRAME_BUF
		//printf("n found points: %d\n\r", n_found_points);

		if(error == 0)
		{
			error = opticFlowLK((unsigned char *) FRAME_BUF, (unsigned char *) FRAME_BUF2, x, y, n_found_points, width, height, new_x, new_y, status, half_window_size, max_iterations);

			
			if(error == 0)
			{
				showFlow((unsigned char *) FRAME_BUF, x, y, status, n_found_points, new_x, new_y, width, height);
				send_frame_nr(1);
			}
			
		}
		break;
*/
	case 'x': // get the image gradient dx
		grab_frame();
		getGradientImage((unsigned char *) FRAME_BUF, (unsigned char *) FRAME_BUF2, (unsigned char *) FRAME_BUF3);
		send_frame_nr(2);
		break;
	case 'y': // get the image gradient dy
		grab_frame();
		getGradientImage((unsigned char *) FRAME_BUF, (unsigned char *) FRAME_BUF2, (unsigned char *) FRAME_BUF3);
		send_frame_nr(3);
		break;
	case 's': // segmenting the image
		grab_frame();
		//segmentSky((unsigned char *)FRAME_BUF);

		segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);

		err = perceptronPitchRoll((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL);

		if(err < INACCURATE)
			motor_command2(pitch+OFF_SET, roll+OFF_SET, delay);
		else
			motor_command2(MAX_SIGNAL/2, MAX_SIGNAL/2,delay);

		send_frame_nr(1);
		// printf("p=%d,r=%d\n\r", pitch, roll);
		break;
	case 'S': // segmenting the image
		grab_frame();
		//segmentSky((unsigned char *)FRAME_BUF);
		segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		err = perceptronPitchRoll((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL);
		if(err < INACCURATE)
			motor_command2(pitch+OFF_SET, roll+OFF_SET, delay);
		else
			motor_command2(MAX_SIGNAL/2, MAX_SIGNAL/2,delay);

		// send_frame_nr(1);
		printf("p=%d,r=%d\n\r", pitch+OFF_SET, roll+OFF_SET);
		break;
	case 't': // segmenting the image with the second tree
		grab_frame();
		//segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		//send_frame_nr(1);
		send_thumbnail();
		break;
	
	case 'u': // segment with adjustable tree:
		adjust_factor = 0;
		ch = getch(); // getchar(&ch) was NOT WORKING!		
		//printf("%c=", ch);
		adjust_factor = atoi(&ch);
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

		// printf("%d\n", adjust_factor);

		grab_frame();
		segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
		// segmentSkyUncertainty2((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		// segmentPatch((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		// segmentNoIllumination((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		// segmentOnlyGradient((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);\

		err = perceptronPitchRoll((unsigned char *)FRAME_BUF, &pitch, &roll, MAX_SIGNAL);

		if(err < INACCURATE)
			motor_command2(pitch+OFF_SET, roll+OFF_SET, delay);
		else
			motor_command2(MAX_SIGNAL/2, MAX_SIGNAL/2,delay);
		send_frame_nr(1);
		break;
/*
	case 'v': // segmenting the image with the second tree returning uncertainty
		grab_frame();
		segmentSkyUncertainty2((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		send_frame_nr(2);
		break;
	*/
	case 'o': // use the sky segmentation only for determining the obstacles and uncertainty
		grab_frame();
		// segmentSkyUncertainty((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
		// determine the amount of obstacle per orientation segment
		getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_SIGNAL);
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
		motor_command2(max_bin, bin_total, delay);
		
		//sendToAutoPilot(obstacles, uncertainty, N_BINS);
		break;
	case 'O': // do a loop of max_count segmentations and always send the results
		max_count = 60000;
		print_frequency = 60;
		for(counter = 0; counter < max_count; counter++)
		{
		    grab_frame();
		    // segmentSkyUncertainty((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
			// segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
			segmentBWboard((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2);
			
	    	// determine the amount of obstacle per orientation segment
	    	getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_SIGNAL);
			motor_command2(max_bin, bin_total, delay);

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
			//printf("-%d-", counter);
		}
		break;
	case 'p': // use the sky segmentation only for determining the obstacles and uncertainty

		adjust_factor = 0;
		ch = getch(); // getchar(&ch) was NOT WORKING!		
		//printf("%c=", ch);
		adjust_factor = atoi(&ch);
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

		// printf("%d\n", adjust_factor);

		grab_frame();
		segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);

		// determine the amount of obstacle per orientation segment
		getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_SIGNAL);
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
		motor_command2(max_bin, bin_total, delay);
		
		//sendToAutoPilot(obstacles, uncertainty, N_BINS);
		break;
	case 'P': // do a loop of max_count segmentations and always send the results
		adjust_factor = 0;
		ch = getch(); // getchar(&ch) was NOT WORKING!		
		//printf("%c=", ch);
		adjust_factor = atoi(&ch);
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
			segment_no_yco_AdjustTree((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
			//segment_no_yco((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, adjust_factor);
	    	// determine the amount of obstacle per orientation segment
	    	getObstacles(obstacles, N_BINS, (unsigned char *)FRAME_BUF, &max_bin, &bin_total, MAX_SIGNAL);
			motor_command2(max_bin, bin_total, delay);

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
			//printf("-%d-", counter);
		}
		break;
		
    }
    return;
}

