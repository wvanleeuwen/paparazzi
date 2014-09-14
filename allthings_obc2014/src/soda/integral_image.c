void get_integral_image(unsigned char* image, unsigned long* integral_image, int height, int width)
{

  int r, c;

  // top left elements are equal:
  integral_image[0] = (unsigned long) image[0];

  for(r = 0; r < height; r++)
  {
    for(c = 0; c < width; c++)
    {
      if(c > 0 && r > 0)
      {
        integral_image[r * width+ c] = (unsigned long) image[r * width + c] + integral_image[(r - 1) * width + c] + integral_image[r * width + (c-1)] - integral_image[(r - 1) * width + (c-1)];
      }
      else if(c > 0)
      {
        integral_image[r * width + c] = (unsigned long) image[r * width + c] + integral_image[r * width + (c-1)];
      }
      else if(r > 0)
      {
        integral_image[r * width + c] = (unsigned long) image[r * width + c] + integral_image[(r - 1) * width + c];
      }
    }
  }
}

