#ifndef VMSTRTS_H
#define VMSTRTS_H

//an exact copy of this file to exist in ppz
//this files keeps the structs that are serialized and streamed over tcp/ip through localhost

struct gst2ppz_message_struct_sky {
	unsigned int counter;		//counter to keep track of data
	int optic_flow_x;		//optical flow output, shift in x direction
	int optic_flow_y;		//optical flow output, shift in y direction

};
extern struct gst2ppz_message_struct_sky gst2ppz;

struct ppz2gst_message_struct_sky {
	unsigned int counter;		//counter to keep track of data
	int pitch;
	int roll;
	int alt; // sonar altitude
};
extern struct ppz2gst_message_struct_sky ppz2gst;

#endif  /*  VMSTRTS_H  */

