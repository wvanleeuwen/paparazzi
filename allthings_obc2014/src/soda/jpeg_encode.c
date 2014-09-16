#include "jpeg_encode.h"

#include <stdio.h>
#include <string.h>
#include <jpeglib.h>


#define BLOCK_SIZE (4*1024*1024)
JOCTET my_buffer[BLOCK_SIZE];

uint8_t* jpeg_start = &my_buffer[0];
uint8_t* jpeg_end = jpeg_start;

void my_init_destination(j_compress_ptr cinfo)
{
    cinfo->dest->next_output_byte = &my_buffer[0];
    cinfo->dest->free_in_buffer = BLOCK_SIZE;
}

boolean my_empty_output_buffer(j_compress_ptr cinfo)
{
    fprintf(stderr,"SODA: Error JPEG-Compress Buffer Empty\n");
    // make buffer larger or restart filling buffer
    cinfo->dest->next_output_byte = &my_buffer[0];
    cinfo->dest->free_in_buffer = BLOCK_SIZE;
    // or return that is failed
    return (boolean)true;
}

void my_term_destination(j_compress_ptr cinfo)
{
  jpeg_end = cinfo->dest->next_output_byte;
}

int storeJpg(const char* filename, unsigned char* image_buffer, int w, int h, int quality)
{
  unsigned char r,g,b;
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;

  JSAMPROW row_pointer[1];   	/* Output row buffer */
  int row_stride;   	/* physical row width in output buffer */
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  //jpeg_stdio_dest(&cinfo, outfile);
  jpeg_destination_mgr dmgr;
  cinfo.dest = &dmgr;
  cinfo.dest->init_destination = &my_init_destination;
  cinfo.dest->empty_output_buffer = &my_empty_output_buffer;
  cinfo.dest->term_destination = &my_term_destination;

  cinfo.image_width = w;
  cinfo.image_height = h;
  cinfo.input_components= 3;
  cinfo.in_color_space = JCS_RGB;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);
  cinfo.scale_denom = 2;
  cinfo.smoothing_factor = 1;


  (void) jpeg_start_compress(&cinfo,TRUE);
  row_stride = w * 3;

  // Start reading (write tables)
  row_pointer[0] = & image_buffer[cinfo.next_scanline * row_stride];
  (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);

#ifdef MAKE_HEADERS
  int hdr_size = cinfo.dest->next_output_byte - &my_buffer[0];
  FILE* hfp = fopen("decode_headers.h","w");
  fprintf(hfp,"const unsigned char jpeg_header[%d] = {\n",hdr_size);
  int nl = 0;
  for (unsigned char* p=&my_buffer[0];p<cinfo.dest->next_output_byte;p++)
  {
    fprintf(hfp,"0x%02x,",*p);
    if (nl == 7)
      fprintf(hfp," ");
    if (++nl >= 16)
    {
      nl = 0;
      fprintf(hfp,"\n");
    }
  }
  fprintf(hfp,"};\n\n");
  fclose(hfp);
#endif


  // Write image header
  FILE *fp = fopen(filename,"w+b");
  unsigned char* p;
  for (p=&my_buffer[0]; p < cinfo.dest->next_output_byte;p++)
    fputc(*p,fp);

  // Reset buffer: throw away tables
  my_init_destination(&cinfo);

  while (cinfo.next_scanline < cinfo.image_height) {
    /* jpeg_write_scanlines expects an array of pointers to scanlines.
     * Here the array is only one element long, but you could pass
     * more than one scanline at a time if that's more convenient.
     */

    //int bsize = cinfo.dest->next_output_byte - &my_buffer[0];
    //printf("LINE #%d: %d bytes\n",cinfo.next_scanline, bsize);

    row_pointer[0] = & image_buffer[cinfo.next_scanline * row_stride];
    (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }

  (void) jpeg_finish_compress(&cinfo);
  

  // Write image data
  for (p=my_buffer; p < cinfo.dest->next_output_byte;p++)
    fputc(*p,fp);
  fclose(fp);

  int bsize = jpeg_end -jpeg_start;
  printf("JPEG_ENCODE: %d bytes\n",bsize);

  jpeg_destroy_compress(&cinfo);

  return 1;
}
