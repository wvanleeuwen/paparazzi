#include <stdio.h>
#include <string.h>
#include <jpeglib.h>

int storeJpg(const char* Name, unsigned char* image_buffer, int w, int h, int quality)
{
  unsigned char r,g,b;
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;

  FILE * outfile;    	/* source file */
  JSAMPROW row_pointer[1];   	/* Output row buffer */
  int row_stride;   	/* physical row width in output buffer */
  if ((outfile = fopen(Name, "wb")) == NULL) 
  {
    fprintf(stderr, "SODA:\tcan't open %s\n", Name);
    return 0;
  }
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  jpeg_stdio_dest(&cinfo, outfile);
  cinfo.image_width = w;
  cinfo.image_height = h;
  cinfo.input_components= 3;
  cinfo.in_color_space = JCS_RGB;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);

  (void) jpeg_start_compress(&cinfo,TRUE);
  row_stride = w * 3;
  
  while (cinfo.next_scanline < cinfo.image_height) {
    /* jpeg_write_scanlines expects an array of pointers to scanlines.
     * Here the array is only one element long, but you could pass
     * more than one scanline at a time if that's more convenient.
     */
    row_pointer[0] = & image_buffer[cinfo.next_scanline * row_stride];
    (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }

  (void) jpeg_finish_compress(&cinfo);
  
  fclose(outfile);
  jpeg_destroy_compress(&cinfo);

  return 1;
}
