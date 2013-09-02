/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  main.c - main control loop for SRV-1 robot
 *    Copyright (C) 2005-2009  Surveyor Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "srv.h"
#include "print.h"
#include "string.h"
#include "gps.h"
#include "interface_sky_segmentation.h"
#include "config.h"

#ifdef STEREO
#include "stereo.h"
#endif

#include "optic_flow.h"

extern int picoc(char *);
extern void httpd_request(char firstChar);

int main() {
    unsigned char ch;
	unsigned char ch2;
    int ix, i;
	int COMM_AUTOPILOT;

    init_heap();
    init_io(); // Initialise LED, GPIO, serial flow & lasers.
    initRTC();
    initTMR4();
    init_uart0(UART0_BAUDRATE);
    init_uart1(UART1_BAUDRATE);
    init_colors();
    disable_failsafe();
    clear_sdram(); // Clears from 0x00100000 to 0x02000000
    camera_setup(); // Sets up the camera to 320x240
	camera_reset(160);
    led1_on();

    #ifdef STEREO
    if (master)
        serial_out_version();
    init_svs();
    if (master)
        check_for_autorun();
    #else
    serial_out_version();
    check_for_autorun();
    #endif /* STEREO */
    
	COMM_AUTOPILOT = 0;
	for(i = 0; i < 1000; i++)
	{
		if(uart1GetChar(&ch))
		{
			COMM_AUTOPILOT = 1;
			// get message from autopilot about what to do
			unsigned char autopilot_message[12];
			getFromAutoPilot(autopilot_message);
			switch(autopilot_message[0])
			{
				case 0:
					// detecting horizon line continuously:
					if(autopilot_message[1] == 0)
					{
						// with minimum-maximum illumination:
						ch = 'g';
						ch2 = 0;
						interface_sky_segmentation(ch, ch2);
					}
					else if(autopilot_message[1] == 1)
					{
						// with an efficient decision tree:
						ch = 'h';
						// the third value determines the number of samples:
						// default should be 4 (1000 samples)
						itoa((int)autopilot_message[2], &ch2);
						interface_sky_segmentation(ch, ch2);
					}
					break;
				case 1:
					// detecting obstacles continuously:
					if(autopilot_message[1] == 0)
					{
						// with minimum-maximum illumination:
						ch = 'm';
						ch2 = 0;
						interface_sky_segmentation(ch, ch2);
					}
					else if(autopilot_message[1] == 1)
					{
						// with an adjustable decision tree:
						ch = 'n';
						// the third value determines the adjustion: 0 - 9 
						// default should be 4 (+ 0.25 standard deviation)
						itoa((int)autopilot_message[2], &ch2);
						interface_sky_segmentation(ch, ch2);
					}
					break;
				}					
			break;
		}
	}

	printf("NORMAL MENU\n");

    while (1) {

        if (getchar(&ch)) {
            switch (ch) {
                case 'I':
                    grab_frame();
                    send_frame();
                    break;
                case 'J':
                    send_80x64planar();
                    break;
                case 'G':
                case 'P':
                    httpd_request(ch);
                    break;
                case 'y':
                    invert_video();
                    break;
                case 'Y':
                    restore_video();
                    break;
                case 'o':   // turn on overlay_flag
                    overlay_on();
                    break;
                case 'O':   // turn off overlay_flag
                    overlay_off();
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
                case 'd':   // 1280 x 1024
                case 'A':   // 1280 x 1024
                    camera_reset(1280);
                    break;
                case 'V':   // send version string
                    serial_out_version();
                    break;
                case 'D':   // check battery
                    check_battery();
                    break;
                case 'R':   // show laser range
                    show_laser_range(0);
                    break;
                case 'r':   // show laser range with debug
                    show_laser_range(1);
                    break;
                case 'q':  // change image quality
                    change_image_quality();
                    break;
                case 'X':  // xmodem receive
                    xmodem_receive();
                    break;
                case 'Q': // execute C program from flash buffer
                    picoc((char *)FLASH_BUFFER);
                    break;
                case '!': // run C interpreter interactively
                    picoc((char *)0);
                    break;
                case '$': // prototype zone
                    switch (getch()) {
                        case '!':  // reset processor
                            reset_cpu();
                        #ifdef STEREO
                        case 'X':
                            svs_master((unsigned short *)FLASH_BUFFER, 
                                (unsigned short *)(FLASH_BUFFER+131072), 131072);
                            ix = (unsigned int)crc16_ccitt(FLASH_BUFFER, 131064);
                            printf("     CRC-sent: 0x%x\r\n", ix);
                            break;
                        case 'R':
                            svs_slave((unsigned short *)FLASH_BUFFER,
                                (unsigned short *)(FLASH_BUFFER+131072), 131072);
                            printf("##$R SPI Slave\r\n");
                            ix = (unsigned int)crc16_ccitt(FLASH_BUFFER, 131064);
                            printf("     CRC-received: 0x%x\r\n", ix);
                            break;
                        case '1':
                            /* toggle mapping on or off */
                            svs_enable_mapping = 1 - svs_enable_mapping;
                            break;
                        case '2':
                            /* right camera */
                            svs_grab(svs_calibration_offset_x, svs_calibration_offset_y, 0, 0);
                            svs_send_features();
                            break;
                        case '3':
                            /* left camera */
                            if (svs_receive_features() > -1) {
                                svs_match(200, 40, 18, 7, 3, 10, 200, 0);
                            }                            
                            break;
                        case '4':
                            /* read calibration params */
                            svs_read_calib_params();
                            break;
                        case 's':  // stereo + send disparities, should be called on the master
                            svs_stereo(1);
                            break;
                        #endif /* STEREO */
                        case 'g':  // gps test
                            gps_show();
                            break;
                        case 'm':  // 80x64 motion vect test
                            motion_vect80x64();
                            break;
                        case 'M':  // full frame motion vect test
                            ix = getch() & 0x0F;
                            motion_vect_test(ix);
                            break;
                        case 'A':  // read analog channel 1-8, 11-18 or 21-28
                            read_analog();
                            break;
                        case 'a':  // read analog channels from SRV_4WD
                            read_analog_4wd();
                            break;
                        case 'C':  // read HMC6352 compass on i2c port 0x22
                            show_compass2x();
                            break;
                        case 'c':  // read HMC5843 compass on i2c port 0x1E
                            show_compass3x();
                            break;
                        case 'E':  // read motor encoders on GPIO-H14 / H15
                            read_encoders();
                            break;
                        case 'e':  // read motor encoders on SRV-4WD
                            read_encoder_4wd();
                            break;
                        case 'T':  // read tilt sensor channel 1-3 (x, y or z axis)
                            read_tilt();
                            break;
                        case 'S':  // test SD card interface on RCM
                            testSD();
                            break;
                    }
                    break;
                case '%': // jump to user defined functions in interface_sky_segmentation.c
					ch = getch();
					ch2 = 0;
                    interface_sky_segmentation(ch, ch2);
                    break;
                case 'E': // launch flash buffer editor
                    launch_editor();
                    break;
                case 't':
                    serial_out_time();
                    break;
                case 'T':
                    set_edge_thresh();
                    break;
                case 'z':  // read, write or dump flash sectors
                    switch (getch()) {  
                                // zAxx - read double sector xx, xx+1 to buffer
                                // zBxx - write buffer to double sector xx, xx+1
                                // zr   - read sector 4 to buffer
                                // zRxx - read sector xx to buffer
                                // zw   - write buffer to sector 4
                                // zWxx - write buffer to sector xx
                                // zZ   - write buffer to boot flash sectors
                                // zd   - display contents of flash buffer
                                // zh   - display contents of http buffer
                                // zc   - clear buffer
                                // zC   - compute checksum of buffer contents 
                        case 'A':
                              // read double user sector 00 - 63  from flash buffer
                            ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                            read_double_sector(ix, 0);
                            break;
                         case 'B':
                              // write double user sector 00 - 63  from flash buffer
                            ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                            write_double_sector(ix, 0);
                            break;
                        case 'r':
                              // read user flash sector into flash buffer
                            read_user_flash();
                            break;
                        case 'R':
                              // read user sector 00 - 63  to flash buffer
                            ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                            read_user_sector(ix);
                            break;
                        case 'w':
                              // write user flash sector from flash buffer
                            write_user_flash();
                            break;
                        case 'W':
                              // write user sector 00 - 63  from flash buffer
                            ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                            write_user_sector(ix);
                            break;
                       case 'Z':
                              // write boot flash sectors (1-2) from flash buffer
                            write_boot_flash();
                            break;
                        case 'd':
                              // dump flash buffer contents to console
                            serial_out_flashbuffer();
                            break;
                        case 'h':
                              // dump HTTP buffer contents to console
                            serial_out_httpbuffer();
                            break;
                        case 'c':
                              // clear flash buffer
                            clear_flash_buffer();
                            break;
                        case 'C':
                              // compute flash buffer checksum
                            crc_flash_buffer();
                            break;
                    }
                    break;
                case 'p':   // ping sonar - supports up to 4 transducers
                    ping_sonar();
                    break;
                case 'S':   // grab 2 character servo 2/3 command string (left, right)
                    ppm1_command();
                    break;
                case 's':   // grab 2 character servo 6/7 command string (left, right)
                    ppm2_command();
                    break;
                case 'M':   // grab 3 character motor command string (left, right, delay)
                    motor_command();
                    break;
                case 'm':   // use 2nd set of timers for PWM
                    motor2_command();
                    break;
                case 'x':
                    if (xwd_init == 0) {
                        xwd_init = 1;
                        init_uart1(115200);
                        delayMS(10);
                    }
                    printf("#x");
                    uart1SendChar('x');
                    uart1SendChar(getch());
                    uart1SendChar(getch());
                    while (uart1GetChar(&ch))  // flush the receive buffer
                        continue;
                    break;
                case '+':   // increase base motor speed
                    motor_increase_base_speed();
                    break;
                case '-':   // decrease base motor speed
                    motor_decrease_base_speed();
                    break;
                case '<':   // bias motor drive to left
                    motor_trim_left();
                    break;
                case '>':   // bias motor drive to right
                    motor_trim_right();
                    break;
                case '7':   // drift left
                case '8':   // forward
                case '9':   // drift right
                case '4':   // turn left
                case '5':   // stop
                case '6':   // turn right
                case '1':   // back left
                case '2':   // back
                case '3':   // back right
                case '0':   // counter clockwise turn
                case '.':   // clockwise turn                    
                    motor_action(ch);
                    break;
                case 'l':   // lasers on
                    lasers_on();
                    break; 
                case 'L':   // lasers off
                    lasers_off();
                    break; 
                case 'v':   // vision commands
                    process_colors();
                    break;
                case 'n':   // neural net commands
                    process_neuralnet();
                    break;
                case 'F':   // set failsafe mode
                    enable_failsafe();
                    break;
                case 'f':   // clear failsafe mode
                    disable_failsafe();
                    break;
                case 'g':   // g0 = grab a reference frame and enable differencing
                            // g1 = enable color segmentation
                            // g2 = enable edge detection
                            // g3 = enable horizon detection
                            // g4 = enable obstacle detection
                            // g5 = enable stereo processing (SVS only)
                            // g6 = enable blob display
                            // g_ = anything else turns them all off
                    switch (getch()) {
                        case '0':
                            enable_frame_diff();
                            break;
                        case '1':
                            enable_segmentation();
                            break;
                        case '2':
                            enable_edge_detect();
                            break;
                        case '3':
                            enable_horizon_detect();
                            break;
                        case '4':
                            enable_obstacle_detect();
                            break;
                        #ifdef STEREO
                        case '5':
                            enable_stereo_processing();
                            break;
                        #endif /* STEREO */
                        case '6':
                            enable_blob_display(); 
                            break;
                        default:  // no match - turn them all off
                            disable_frame_diff();
                            break;
                    }
                    break;
                case 'i':   // i2c read / write
                    process_i2c();
                    break;
                default:
                    printf("#?");  // unknown command
            }
            reset_failsafe_clock();
            delayUS(12000000/UART0_BAUDRATE);  // flush recv buffer
            while (getchar(&ch)) 
                delayUS(12000000/UART0_BAUDRATE);  // allow enough time for characters to flow in
        }
        #ifdef STEREO
        if (!master) {  // if slave processor on SVS, check whether stereo_sync_flag has triggered
            if ((stereo_sync_flag == 0x0100) && (check_stereo_sync() == 0)) {
                stereo_sync_flag = 0;
                continue;
            }
            if ((stereo_sync_flag == 0) && (check_stereo_sync() == 0x0100)) {
                //printf("SVS slave:  stereo sync toggle\r\n");
                stereo_sync_flag = 0x0100;
                svs_grab(svs_calibration_offset_x, svs_calibration_offset_y, 0, 0);
                svs_send_features();
            }
        }
        #endif /* STEREO */
        check_failsafe();
        led0_on();
    }
}


