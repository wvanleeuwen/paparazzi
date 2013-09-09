/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  uart.c - uart functions for the SRV-1 Blackfin robot.
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

#include <cdefBF537.h>
#include "config.h"
#include "uart.h"
#include "srv.h"
#include "malloc.h"

#define SSYNC    asm("ssync;")
#define SUART_SEND0 *pPORTHIO |= 0x4000
#define SUART_SEND1 *pPORTHIO &= 0xBFFF
#define SUART_RECV  (*pPORTHIO & 0x8000)

static int suart_timebase;

void suartInit(int baud) {  // use GPIO-H14 and H15 for soft uart.  H14 = TX, H15 = RX
    suart_timebase = 1000000000 / baud;  // define bit period in nanoseconds
    *pPORTHIO_DIR &= 0x3FFF;   // set H14 as output, H15 as input
    SSYNC;
    *pPORTHIO_DIR |= 0x4000;   // enable H14 output
    SSYNC;
    *pPORTHIO_INEN |= 0x8000;  // set H15 as input
    SSYNC;
    *pPORTHIO &= 0xBFFF;  // set H14 low
    SSYNC;
}

void suartPutChar(unsigned char ch)  // soft uart send - transmit on GPIO-H14
{
    int ix;
    
    SUART_SEND1;  // send start bit
    delayNS(suart_timebase);
    for (ix=0; ix<8; ix++) {
        if (ch & 0x01)
            SUART_SEND0;  // output is inverted
        else
            SUART_SEND1;
        delayNS(suart_timebase);
        ch = ch >> 1;
    }
    SUART_SEND0;  // send 2 stop bits
    delayNS(suart_timebase*2);
}

unsigned short suartGetChar(int timeout)  // check for incoming character, wait for "timeout" milliseconds
{
    int t0;
    unsigned short sx, smask;
    
    t0 = readRTC();
    sx = 0;
    
    while ((readRTC()-t0) < timeout) {  // wait for start bit
        if (SUART_RECV)
            continue;
        delayNS((suart_timebase * 4) / 3);  // wait for completion of start bit, then go 30% into next bit
        for (smask=1; smask<256; smask*=2) {
            if (SUART_RECV)
                sx += smask;
            delayNS(suart_timebase);  // skip to next bit
        }
        delayNS((suart_timebase*2)/3);  // wait for stop bit
        return (sx + 0x8000);  // set high bit to indicate received character
    }
    return 0;
}

void init_uart0(int baudrate)
{
    int uart_divider;

    uart_divider = (((MASTER_CLOCK * VCO_MULTIPLIER) / SCLK_DIVIDER) / 16) / baudrate;
    *pPORTF_FER |= 0x0003;  // enable UART0 pins
    *pUART0_GCTL = UCEN;
    *pUART0_LCR = DLAB;
    *pUART0_DLL = uart_divider;
    *pUART0_DLH = uart_divider >> 8;
    *pUART0_LCR = WLS(8); // 8 bit, no parity, one stop bit

    // dummy reads to clear possible pending errors / irqs
    char dummy = *pUART0_RBR;
    dummy = *pUART0_LSR;
    dummy = *pUART0_IIR;
    SSYNC;
}

void uart0_CTS(int ix)  // set ~CTS signal.  1 = clear to send   0 = not clear to send
{
    if (ix == 0)
        *pPORTHIO |= 0x0040;  // block incoming data 
    else
        *pPORTHIO &= 0xFFBF;  // allow incoming data 
}

void init_uart1(int baudrate)
{
    int uart_divider;

    uart_divider = (((MASTER_CLOCK * VCO_MULTIPLIER) / SCLK_DIVIDER) / 16) / baudrate;
    *pPORTF_FER |= 0x000C;  // enable UART1 pins
    *pUART1_GCTL = UCEN;
    *pUART1_LCR = DLAB;
    *pUART1_DLL = uart_divider;
    *pUART1_DLH = uart_divider >> 8;
    *pUART1_LCR = WLS(8); // 8 bit, no parity, one stop bit

    char dummy = *pUART1_RBR;
    dummy = *pUART1_LSR;
    dummy = *pUART1_IIR;
    SSYNC;
}

void uart0SendChar(unsigned char c)
{
    while (*pPORTHIO & 0x0001)  // hardware serial flow control - 
        continue;               //    S32 pin 17 should be grounded to disable
    while (!(*pUART0_LSR & THRE))
        continue;
    *pUART0_THR = c;
}

void uart0SendString(unsigned char *s)
{
    char a;
    while ((a = *s++)) {
        uart0SendChar(a);
    }
}

void uart0SendChars(unsigned char *buf, unsigned int size)
{
    while (size--) {
        uart0SendChar(*buf++);
    }
}

unsigned char uart0GetCh()
{
    while (!(*pUART0_LSR & DR));
    return *pUART0_RBR;
}

unsigned char uart0GetChar(unsigned char *a)
{
    if (!(*pUART0_LSR & DR))
        return 0;
    *a = *pUART0_RBR;
    return 1;
}

unsigned char uart0Signal()
{
    if (!(*pUART0_LSR & DR))
        return 0;
    return 1;
}

void uart1SendChar(unsigned char c)
{
    while (!(*pUART1_LSR & THRE));
    *pUART1_THR = c;
}

void uart1SendString(unsigned char *s)
{
    char a;
    while ((a = *s++)) {
        uart1SendChar(a);
    }
}

void uart1SendChars(unsigned char *buf, unsigned int size)
{
    while (size--) {
        uart1SendChar(*buf++);
    }
}

unsigned char uart1GetCh()
{
    while (!(*pUART1_LSR & DR));
        return *pUART1_RBR;
}

unsigned char uart1GetChar(unsigned char *a)
{
    if (!(*pUART1_LSR & DR))
        return 0;
    *a = *pUART1_RBR;
    return 1;
}

unsigned char uart1Signal()
{
    if (!(*pUART1_LSR & DR))
        return 0;
    return 1;
}

/*****************************************************************************
 *
 * Description:
 *    Routine for printing integer numbers in various formats. The number is 
 *    printed in the specified 'base' using exactly 'noDigits', using +/- if 
 *    signed flag 'sign' is TRUE, and using the character specified in 'pad' 
 *    to pad extra characters. 
 *
 * Params:
 *    [in] base     - Base to print number in (2-16) 
 *    [in] noDigits - Number of digits to print (max 32) 
 *    [in] sign     - Flag if sign is to be used (TRUE), or not (FALSE) 
 *    [in] pad      - Character to pad any unused positions 
 *    [in] number   - Signed number to print 
 *
 ****************************************************************************/
void
printNumber(unsigned char  base,
            unsigned char  noDigits,
            unsigned char  sign,
            unsigned char  pad,
            int number)
{
  static unsigned char  hexChars[16] = "0123456789ABCDEF";
  unsigned char        *pBuf;
  unsigned char         buf[32];
  unsigned int        numberAbs;
  unsigned int        count;

  // prepare negative number
  if(sign && (number < 0))
    numberAbs = -number;
  else
    numberAbs = number;

  // setup little string buffer
  count = (noDigits - 1) - (sign ? 1 : 0);
  pBuf = buf + sizeof(buf);
  *--pBuf = '\0';

  // force calculation of first digit
  // (to prevent zero from not printing at all!!!)
  *--pBuf = hexChars[(numberAbs % base)];
  numberAbs /= base;

  // calculate remaining digits
  while(count--)
  {
    if(numberAbs != 0)
    {
      //calculate next digit
      *--pBuf = hexChars[(numberAbs % base)];
      numberAbs /= base;
    }
    else
      // no more digits left, pad out to desired length
      *--pBuf = pad;
  }

  // apply signed notation if requested
  if(sign)
  {
    if(number < 0)
      *--pBuf = '-';
    else if(number > 0)
       *--pBuf = '+';
    else
       *--pBuf = ' ';
  }

  // print the string right-justified
  uart0SendString(pBuf);
}

// COMMUNICATION VIA UART1 WITH PAPARAZZI AUTOPILOT:

void getFromAutoPilot(unsigned char *message)
{
	unsigned char result;
	unsigned int length, it;
	unsigned int start = 0;
	unsigned int n_bytes = 12;
	unsigned char CK;
	const unsigned int DEBUG = 0;	

	length = n_bytes + 2;

	while(!start)
	{	
		result = uart1GetCh();
		if(DEBUG) printf("%d - ", result);
		if(result == 0xff) start = 1;
	}
	
	CK = 0xff;
	for(it = 0; it < n_bytes; it++)
	{
		result = uart1GetCh();		
		if(DEBUG) printf("%d,", result);
		message[it] = result;
		CK += result;
	}

	result = uart1GetCh();
	if(DEBUG) printf(" CK_autopilot = %d, CK_blackfin = %d\n\r", (int) result, (int) CK);
	if(result != CK)
	{
		/*for(it = 0; it < n_bytes; it++)
		{
			message[it] = 0x7f;
		}*/
		if(DEBUG)printf("?");
	}
	else
	{
		if(DEBUG)printf("!");
	}
	return;
}

void sendToAutoPilot(unsigned int* obstacles, int n_bins)
{
	unsigned int n_bytes = 12;
	unsigned int length, it;
	unsigned char* message;
	unsigned char CK;
	const unsigned int DEBUG = 0;
	
	length = n_bytes + 2;
	message = (unsigned char*) malloc((length) * sizeof(unsigned char));
	if(message == 0) return; // not enough memory
	message[0] = 0xff;
	if(DEBUG) printf("%d - ", message[0]);
	
	CK = message[0];
	for(it = 0; it < n_bytes - 1; it++)
	{
		if(it < n_bins)
		{
			message[1+it] = (unsigned char) (obstacles[it]);
			if (message[1+it] >= 255)
			{
				message[1+it] = 254;
			}
		}
		else
		{
			message[1+it] = 0;
		}
		if(DEBUG) printf("%d,", message[1+it]);
		CK += message[1+it];
	} 
	message[length-1] = CK;
	if(DEBUG) printf(" CK = %d\n\r", (int) CK);
	uart1SendChars(message, length);	
	free((char*)message);
}

/*
void sendToAutoPilot(unsigned int* obstacles, unsigned int* uncertainty, int n_bins)
{
	// Format: 
	// 181, 98, class-byte, id-byte, length-short, <message>, CK_A, CK_B
	unsigned int length, it;
	unsigned char* message;
	unsigned char CK_A, CK_B;

	length = 2 * (unsigned int) n_bins;
	message = (unsigned char*) malloc((length+8) * sizeof(unsigned char));
	if(message == 0) return; // not enough memory
	message[0] = 0xb5;
	message[1] = 0x62;
	message[2] = (unsigned char) 99; // class
	message[3] = (unsigned char) 86; // id
	message[4] = ((short) length) >> 8;
	message[5] = ((short) length) & 0x00ff;
	for(it = 0; it < n_bins; it++)
	{
		message[6+it] = (unsigned char) (obstacles[it]);
		message[6+n_bins+it] = (unsigned char) (uncertainty[it]);
	}
	CK_A = 0; CK_B = 0;
	for(it = 2; it < length+6; it++)
	{
		CK_A += message[it];
		CK_B += CK_A;
	}
	message[length+6] = CK_A;
	message[length+7] = CK_B;
	printf("CK_A = %d, CK_B = %d\n\r", (int) CK_A, (int) CK_B);
	uart1SendChars(message, length+8);
	
	free((char*)message);
}
*/
