/*
 * Copyright (C) 2013  Christophe De Wagter
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "fireswarm_communication.h"

//#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <sys/termios.h>
#include <sys/ioctl.h>


int sim_uart_p = 0;

#define failwith(X)     \
{                       \
  fprintf(stderr,X);    \
  return;               \
}


void fireswarm_payload_link_init(void)
{
  sim_uart_p = open("/dev/ttyUSB0", O_RDWR|O_NONBLOCK);
  if (sim_uart_p == -1)
  {
    fprintf(stderr,"Error: FireswarmCommunicationSim failed to open %s", "/dev/ttyUSB0");
  }
  else
  {
    struct termios orig_termios, cur_termios;

    if (tcgetattr(sim_uart_p, &orig_termios)) failwith("getting modem serial device attr");
    cur_termios = orig_termios;

    // input modes
    cur_termios.c_iflag &= ~(IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK|ISTRIP|INLCR|IGNCR
                  |ICRNL |IXON|IXANY|IXOFF|IMAXBEL);
    // pas IGNCR sinon il vire les 0x0D
    cur_termios.c_iflag |= BRKINT;

    // output_flags
    cur_termios.c_oflag  &=~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET);

    // control modes
    cur_termios.c_cflag &= ~(CSIZE|CSTOPB|CREAD|PARENB|PARODD|HUPCL|CLOCAL|CRTSCTS);
    cur_termios.c_cflag |= CREAD|CS8|CLOCAL;

    // local modes
    cur_termios.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO|FLUSHO|PENDIN);
    cur_termios.c_lflag |= NOFLSH;

    if (cfsetspeed(&cur_termios, B57600)) failwith("setting modem serial device speed");

    if (tcsetattr(sim_uart_p, TCSADRAIN, &cur_termios)) failwith("setting modem serial device attr");

  }
}

uint8_t fsw_crc = 0;

void fireswarm_payload_link_start(void)
{
  fsw_crc = 0;
}

void fireswarm_payload_link_crc(void)
{
  if (sim_uart_p == -1) return;

  int n = write(sim_uart_p, &fsw_crc, 1);
  if (n < 0)
  {
    fprintf(stderr,"Error: Write Failed");
  }
}


void fireswarm_payload_link_transmit(uint8_t* buff, int size)
{
  if (sim_uart_p == -1) return;

  for (int i=0; i<size;i++) fsw_crc += buff[i];

  int n = write(sim_uart_p, buff, size);
  if (n < 0)
  {
    fprintf(stderr, "Error: Write Failed");
  }
}

int fireswarm_payload_link_has_data(void)
{
  if (sim_uart_p == -1) return 0;

  int bytes;
  ioctl(sim_uart_p, FIONREAD, &bytes);
  return bytes;
}

char fireswarm_payload_link_get(void)
{
  char c;
  int ret;

  if (sim_uart_p == -1) return ' ';

  ret = read(sim_uart_p, &c, 1);
  if (ret > 0)
  {
    fprintf(stderr,"%x ",(uint8_t)c);
    return c;
  }
  else
    fprintf(stderr, "Error: read(1)_error\n");
  return 0;
}
