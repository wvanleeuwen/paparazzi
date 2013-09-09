/*******************************************************************************
Copyright 2001, 2002 Georges Menie (<URL snipped>)
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

/*******************************************************************************
putchar is the only external dependency for this file, 
if you have a working putchar, just remove the following
define. If the function should be called something else,
replace outbyte(c) by your own function call.
*/
//*******************************************************************************
//  Updated by DDM 11/06/09 11:42 - adding floating-point support
//*******************************************************************************

/********************************************************************************
Updated by Marc Shapiro 12/06/2009
 Modified to work as a replacement for the printf.c file in the
 firmware for the SRV1 (Surveyor) robot using a Blackfin 537 processor

  To add true floating point variables, calculations and printing:
   1) Make the following changes to the Makefile:
    a. Add a line: 'CFLAGS += -mfast-fp'
            after: 'CFLAGS += $(CPUDEFINES)'
    b. Change the line: '$(LD) -T srv1.x -Map srv1.map -O binary -o srv1.bin $(OBJS)'
           to the line" '$(LD) -T srv1.x -Map srv1.map -O binary -o srv1.bin $(OBJS) -lbffastfp'
    c. Remove float.c from the CSRCS (first line of Makefile)

   2) In srv1.x, remove float.o(.sdram.text) from the first .sdram.text line

   3) Backup the original printf.c file and replace it with this file

   4) Recompile.

To run in standalone mode, either uncomment the define, below, or
define TEST_PRINTF in a Makefile. 
*/


//#define TEST_PRINTF 1

#ifdef TEST_PRINTF
#include <stdio.h>
#else
#include "print.h"
#endif

typedef  unsigned int  uint ;

//****************************************************************************
static void printchar (char **str, int c)
{
  if (str) {
    **str = c;
    ++(*str);
  }
  else (void)putchar(c);
}

//****************************************************************************
uint my_strlen(char *str)
{
  if (str == 0)
     return 0;
  uint slen = 0 ;
  while (*str != 0) {
     slen++ ;
     str++ ;
  }
  return slen;
}

//****************************************************************************
//  This version returns the length of the output string.
//  It is more useful when implementing a walking-string function.
//****************************************************************************
unsigned dbl2stri(char *outbfr, double dbl, unsigned dec_digits)
{
  static char local_bfr[81] ;
  char *output = (outbfr == 0) ? local_bfr : outbfr ;

  //*******************************************
  //  extract negative info
  //*******************************************
  int negative = (dbl < 0.0) ? 1 : 0 ;
  if (negative) {
     *output++ = '-' ;
     dbl *= -1.0 ;
  }

  //**************************************************************************
  //  construct fractional multiplier for specified number of digits.
  //  Note that we draw one digit more than requested in order
  //  to handle last-digit rounding.
  //**************************************************************************
  uint mult = 10 ;
  uint idx ;
  for (idx=0; idx < dec_digits; idx++)
     mult *= 10 ;

  // printf("mult=%u\n", mult) ;
  int wholeNum = (int) dbl ;
  int decimalNum = (int) ((dbl - wholeNum) * mult);
  //  round off low digit
  decimalNum += 5 ;
  decimalNum /= 10 ;

  //*******************************************
  //  convert integer portion
  //*******************************************
  char tbfr[40] ;
  idx = 0 ;
  while (wholeNum != 0) {
     tbfr[idx++] = '0' + (wholeNum % 10) ;
     wholeNum /= 10 ;
  }
  // printf("%.3f: whole=%s, dec=%d\n", dbl, tbfr, decimalNum) ;
  if (idx == 0) {
     *output++ = '0' ;
  } else {
     while (idx > 0) {
        *output++ = tbfr[idx-1] ;  //lint !e771
        idx-- ;
     }
  }
  if (dec_digits > 0) {
     *output++ = '.' ;

     //*******************************************
     //  convert fractional portion
     //*******************************************
     idx = 0 ;
     while (decimalNum != 0) {
        tbfr[idx++] = '0' + (decimalNum % 10) ;
        decimalNum /= 10 ;
     }
     //  pad the decimal portion with 0s as necessary;
     //  We wouldn't want to report 3.093 as 3.93, would we??
     while (idx < dec_digits) {
        tbfr[idx++] = '0' ;
     }
     // printf("decimal=%s\n", tbfr) ;
     if (idx == 0) {
        *output++ = '0' ;
     } else {
        while (idx > 0) {
           *output++ = tbfr[idx-1] ;
           idx-- ;
        }
     }
  }
  *output = 0 ;

  //  prepare output
  output = (outbfr == 0) ? local_bfr : outbfr ;
  return my_strlen(output) ;
}

//****************************************************************************
#define  PAD_RIGHT   1
#define  PAD_ZERO    2

static int prints (char **out, const char *string, int width, int pad)
{
  register int pc = 0, padchar = ' ';
  if (width > 0) {
     int len = 0;
     const char *ptr;
     for (ptr = string; *ptr; ++ptr)
     ++len;
     if (len >= width)
        width = 0;
     else
     width -= len;
     if (pad & PAD_ZERO)
        padchar = '0';
  }
  if (!(pad & PAD_RIGHT)) {
     for (; width > 0; --width) {
        printchar (out, padchar);
        ++pc;
     }
  }
  for (; *string; ++string) {
     printchar (out, *string);
     ++pc;
  }
  for (; width > 0; --width) {
     printchar (out, padchar);
     ++pc;
  }
  return pc;
}

//****************************************************************************
/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12
static int printi (char **out, int i, int b, int sg, int width, int pad, int letbase)
{
  char print_buf[PRINT_BUF_LEN];
  char *s;
  int t, neg = 0, pc = 0;
  unsigned u = (unsigned) i;

  if (i == 0) {
     print_buf[0] = '0';
     print_buf[1] = '\0';
     return prints (out, print_buf, width, pad);
  }
  if (sg && b == 10 && i < 0) {
     neg = 1;
     u = (unsigned) -i;
  }
  s = print_buf + PRINT_BUF_LEN - 1;
  *s = '\0';
  while (u) {
     t = u % b;  //lint !e573  Warning 573: Signed-unsigned mix with divide
     if (t >= 10)
        t += letbase - '0' - 10;
        *--s = t + '0';
        u /= b;  //lint !e573  Warning 573: Signed-unsigned mix with divide
     }
     if (neg) {
        if (width && (pad & PAD_ZERO)) {
           printchar (out, '-');
           ++pc;
           --width;
        } else {
           *--s = '-';
        }
     }
  return pc + prints (out, s, width, pad);
}

//****************************************************************************
static int print (char **out, int *varg)
{
  register int width, pad ;
  register int pc = 0;
  register char *format = (char *) (*varg++);
  register int post_decimal ;
  register unsigned dec_width = 0 ;

  char scr[2];

  for (; *format != 0; ++format) {
     if (*format == '%') {
        ++format;
        width = pad = 0;
        if (*format == '\0')
           break;
        if (*format == '%')
           goto out;
        if (*format == '-') {
           ++format;
           pad = PAD_RIGHT;
        }
     while (*format == '0') {
        ++format;
        pad |= PAD_ZERO;
     }
        post_decimal = 0 ;
        if (*format == '.' || (*format >= '0' &&  *format <= '9')) {

           while (1) {
           if (*format == '.') {
              post_decimal = 1 ;
              dec_width = 0 ;
              format++ ;
           } else if ((*format >= '0' &&  *format <= '9')) {
              if (post_decimal) {
                 dec_width *= 10;
                 dec_width += *format - '0';
              } else {
                 width *= 10;
                 width += *format - '0';
              }
              format++ ;
           } else {
              break;
           }
        }
     }
     if (*format == 'l')
        ++format;
        switch (*format) {
           case 's':
              {
                 char *s = *((char **) varg++);   //lint !e740
                 // printf("[%s] w=%u\n", s, width) ;
                 pc += prints (out, s ? s : "(null)", width, pad);
              }
              break;
           case 'd':
              pc += printi (out, *varg++, 10, 1, width, pad, 'a');
              break;
           case 'x':
              pc += printi (out, *varg++, 16, 0, width, pad, 'a');
              break;
           case 'X':
              pc += printi (out, *varg++, 16, 0, width, pad, 'A');
              break;
           case 'u':
              pc += printi (out, *varg++, 10, 0, width, pad, 'a');
              break;
           case 'c':
              /* char are converted to int then pushed on the stack */
              scr[0] = *varg++;
              scr[1] = '\0';
              pc += prints (out, scr, width, pad);
              break;
           case 'f':
              {
                 double *dblptr = (double *) varg ;  //lint !e740 !e826  convert to double pointer
                 double dbl = *dblptr++ ;   //  increment double pointer
                 varg = (int *) dblptr ;    //lint !e740  copy updated pointer back to base pointer
                 char bfr[81] ;
                 // unsigned slen =
                 dbl2stri(bfr, dbl, dec_width) ;
                 // stuff_talkf("[%s], width=%u, dec_width=%u\n", bfr, width, dec_width) ;
                 pc += prints (out, bfr, width, pad);
              }
              break;

           default:
              printchar (out, '%');
              printchar (out, *format);
              break;
        }
     } else {
        out:
        printchar (out, *format);
        ++pc;
     }
  }
  if (out)
  **out = '\0';
  return pc;
}

//****************************************************************************
//  assuming sizeof(void *) == sizeof(int)
//  This function is not valid in embedded projects which don't
//  have a meaningful stdout device.
//****************************************************************************
int printf (const char *format, ...)
{
  int *varg = (int *) (&format);
  return print (0, varg);
}

//****************************************************************************
int sprintf (char *out, const char *format, ...)
{
  //  create a pointer into the stack.
  //  Thematically this should be a void*, since the actual usage of the
  //  pointer will vary.  However, int* works fine too.
  //  Either way, the called function will need to re-cast the pointer
  //  for any type which isn't sizeof(int)
  int *varg = (int *) (&format);
    return print (&out, varg);
}

//****************************************************************************

#ifdef TEST_PRINTF
int main (void)
{
  char *ptr = "Hello world!";
  char *np = 0;
  int i = 5;
  unsigned int bs = sizeof (int) * 8;
  int mi;
  char buf[80];

  mi = (1 << (bs - 1)) + 1;

  printf ("%s\n", ptr);
  printf ("termf test\n");
  printf ("%s is null pointer\n", np);
  printf ("%d = 5\n", i);
  printf ("%d = - max int\n", mi);
  printf ("char %c = 'a'\n", 'a');
  printf ("hex %x = ff\n", 0xff);
  printf ("hex %02x = 00\n", 0);
  printf ("signed %d = unsigned %u = hex %X\n", -3, -3, -3);
  printf ("%d %s(s)%%", 0, "message");
  printf ("\n");
  printf ("%d %s(s) with %%\n", 0, "message");
  sprintf (buf, "justif: \"%-10s\"\n", "left");
  printf ("%s", buf);
  sprintf (buf, "justif: \"%10s\"\n", "right");
  printf ("%s", buf);
  sprintf (buf, " 3: %04d zero padded\n", 3);
  printf ("%s", buf);
  sprintf (buf, " 3: %-4d left justif.\n", 3);
  printf ("%s", buf);
  sprintf (buf, " 3: %4d right justif.\n", 3);
  printf ("%s", buf);
  sprintf (buf, "-3: %04d zero padded\n", -3);
  printf ("%s", buf);
  sprintf (buf, "-3: %-4d left justif.\n", -3);
  printf ("%s", buf);
  sprintf (buf, "-3: %4d right justif.\n", -3);
  printf ("%s", buf);

  sprintf (buf, "%.2f is a double\n", 3.31) ;
  printf ("%s", buf);

  sprintf (buf, "multiple unsigneds: %u %u %2u %X\n", 15, 5, 23, 0xB38F) ;
  printf ("%s", buf);

  sprintf (buf, "multiple chars: %c %c %c %c\n", 'a', 'b', 'c', 'd') ;
  printf ("%s", buf);

  sprintf (buf, "multiple doubles: %.2f %2.0f %.2f %.3f %.2f [%-8.3f]\n",
                 3.31, 2.45, -1.1, 3.093, 13.72, -4.382) ;
  printf ("%s", buf);

  return 0;
}
#endif
