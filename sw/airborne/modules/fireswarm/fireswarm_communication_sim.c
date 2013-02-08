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

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

int sim_uart_p = 0;


void fireswarm_payload_link_init(void)
{
  sim_uart_p = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (sim_uart_p == -1)
  {
    fprintf(stderr,"Error: FireswarmCommunicationSim failed to open %s", "/dev/ttyUSB0");
  }
  else
  {
    struct termios options;

    fcntl(sim_uart_p, F_SETFL, 0);
    fcntl(sim_uart_p, F_SETFL, FNDELAY);

    tcgetattr(sim_uart_p, &options);
    cfsetispeed(&options, B57600);
    cfsetospeed(&options, B57600);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(sim_uart_p, TCSANOW, &options);
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

/*

#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <sys/termios.h>
#include <sys/ioctl.h>

#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/alloc.h>

static int baudrates[] = { B0, B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400 };


// Open serial device for requested protocoll
value c_init_serial(value device, value speed, value hw_flow_control)
{
  struct termios orig_termios, cur_termios;

  int br = baudrates[Int_val(speed)];

  int fd = open(String_val(device), O_RDWR|O_NONBLOCK);

  if (fd == -1) failwith("opening modem serial device : fd < 0");

  if (tcgetattr(fd, &orig_termios)) failwith("getting modem serial device attr");
  cur_termios = orig_termios;

  // input modes
  cur_termios.c_iflag &= ~(IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK|ISTRIP|INLCR|IGNCR
			    |ICRNL |IXON|IXANY|IXOFF|IMAXBEL);
  // pas IGNCR sinon il vire les 0x0D
  cur_termios.c_iflag |= BRKINT;

  // output_flags
  cur_termios.c_oflag  &=~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET);

  // control modes
  if (Bool_val(hw_flow_control)) {
    cur_termios.c_cflag &= ~(CSIZE|CSTOPB|CREAD|PARENB|PARODD|HUPCL|CLOCAL);
    cur_termios.c_cflag |= CREAD|CS8|CLOCAL|CRTSCTS;
  }
  else {
    cur_termios.c_cflag &= ~(CSIZE|CSTOPB|CREAD|PARENB|PARODD|HUPCL|CLOCAL|CRTSCTS);
    cur_termios.c_cflag |= CREAD|CS8|CLOCAL;
  }

  // local modes
  cur_termios.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO|FLUSHO|PENDIN);
  cur_termios.c_lflag |= NOFLSH;

  if (cfsetspeed(&cur_termios, br)) failwith("setting modem serial device speed");

  if (tcsetattr(fd, TCSADRAIN, &cur_termios)) failwith("setting modem serial device attr");

  return Val_int(fd);
}

*/




