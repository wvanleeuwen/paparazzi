#ifndef VMSTRT_H
#define VMSTRT_H

//an exact copy of this file to exist in ppz
//this files keeps the structs that are serialized and streamed over tcp/ip through localhost

struct gst2ppz_message_struct {
  unsigned int counter;		//counter to keep track of data
  signed int blob_x1;
  signed int blob_y1;
  signed int blob_x2;
  signed int blob_y2;
  signed int blob_x3;
  signed int blob_y3;
  signed int blob_x4;
  signed int blob_y4;
};
extern struct gst2ppz_message_struct gst2ppz;

struct ppz2gst_message_struct {
  unsigned int heading;
};
extern struct ppz2gst_message_struct ppz2gst;

#endif  /*  VMSTRT_H  */

