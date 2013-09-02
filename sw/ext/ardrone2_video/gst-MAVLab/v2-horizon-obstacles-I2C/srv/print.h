#include "uart.h"
int printf(const char *format, ...);
int sprintf(char *out, const char *format, ...);
#define putchar uart0SendChar
#define putchars uart0SendChars
#define getch uart0GetCh
#define getchar uart0GetChar
#define getsignal uart0Signal

