/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#if defined(CYGWIN) || defined(__APPLE__)
#include <sys/socket.h>
#else
#include <linux/serial.h>
#endif

#define READ_TIMEOUT          250000      /* less than 1e6 */
#define _POSIX

int carmen_serial_connect(int *dev_fd, char *dev_name)
{
  int BAUDRATE = B9600;
  struct termios newtio;

  *dev_fd = open(dev_name, O_RDWR | O_SYNC | O_NOCTTY, 0);
  if(*dev_fd < 0) {
    fprintf(stderr,"Serial I/O Error:  Could not open port %s\n", dev_name);
    return -1;
  }
  tcgetattr(*dev_fd, &newtio);
  cfsetispeed(&newtio, BAUDRATE);
  cfsetospeed(&newtio, BAUDRATE);
  newtio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  newtio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON | IXOFF);
  newtio.c_cflag &= ~(CSIZE | PARENB | PARODD);
  newtio.c_cflag |= (CS8);       
  newtio.c_oflag &= ~(OPOST);
  newtio.c_cc[VTIME] = 1;      
  newtio.c_cc[VMIN] = 0;       
  tcflush(*dev_fd, TCIFLUSH);
  tcsetattr(*dev_fd, TCSANOW, &newtio);
  return 0;
}

#if (defined(TIOCM_RTS) && defined(TIOCMODG)) || defined(_COHERENT)
void carmen_serial_setrts(int fd)
#else
void carmen_serial_setrts(int fd  __attribute__ ((unused)))
#endif
{
#if defined(TIOCM_RTS) && defined(TIOCMODG)
  int mcs = 0;
  
  ioctl(fd, TIOCMODG, &mcs);
  mcs |= TIOCM_RTS;
  ioctl(fd, TIOCMODS, &mcs);
#endif
#ifdef _COHERENT
  ioctl(fd, TIOCSRTS, 0);
#endif
}

void carmen_serial_setparms(int fd, const char *baudr, const char *par, char *bits,
		     int hwf, int swf)
{
  int spd = -1;
  int newbaud = 0;
  int bit = bits[0];
#ifdef _POSIX
  struct termios tty;
  tcgetattr(fd, &tty);
#else
  struct sgttyb tty;
  ioctl(fd, TIOCGETP, &tty);
#endif

  /* We generate mark and space parity ourself. */
  if(bit == '7' && (par[0] == 'M' || par[0] == 'S'))
    bit = '8';
  /* Check if 'baudr' is really a number */
  if((newbaud = (atol(baudr) / 100)) == 0 && baudr[0] != '0')
    newbaud = -1;

  switch(newbaud) {
  case 0:
#ifdef B0
    spd = B0;	   break;
#else
    spd = 0;  	   break;
#endif
  case 3:	
    spd = B300;	   break;
  case 6:
    spd = B600;	   break;
  case 12:
    spd = B1200;   break;
  case 24:
    spd = B2400;   break;
  case 48:
    spd = B4800;   break;
  case 96:
    spd = B9600;   break;
#ifdef B19200
  case 192:	
    spd = B19200;  break;
#else
#ifdef EXTA
  case 192:
    spd = EXTA;    break;
#else
  case 192:
    spd = B9600;   break;
#endif	
#endif	
#ifdef B38400
  case 384:
    spd = B38400;  break;
#else
#ifdef EXTB
  case 384:
    spd = EXTB;    break;
#else
  case 384:	
    spd = B9600;   break;
#endif
#endif	
#ifdef B57600
  case 576:	
    spd = B57600;  break;
#endif
#ifdef B115200
  case 1152:
    spd = B115200; break;
#endif
#ifdef B500000
  case 5000:
    spd = B500000; break;
#endif
  }
  
#if defined (_BSD43) && !defined(_POSIX)
  if(spd != -1) tty.sg_ispeed = tty.sg_ospeed = spd;
  /* Number of bits is ignored */
  tty.sg_flags = RAW | TANDEM;
  if(par[0] == 'E')
    tty.sg_flags |= EVENP;
  else if(par[0] == 'O')
    tty.sg_flags |= ODDP;
  else
    tty.sg_flags |= PASS8 | ANYP;
  ioctl(fd, TIOCSETP, &tty);
#ifdef TIOCSDTR
  /* FIXME: huh? - MvS */
  ioctl(fd, TIOCSDTR, 0);
#endif
#endif
  
#if defined (_V7) && !defined(_POSIX)
  if(spd != -1) tty.sg_ispeed = tty.sg_ospeed = spd;
  tty.sg_flags = RAW;
  if(par[0] == 'E')
    tty.sg_flags |= EVENP;
  else if(par[0] == 'O')
    tty.sg_flags |= ODDP;
  ioctl(fd, TIOCSETP, &tty);
#endif

#ifdef _POSIX
  if(spd != -1) {
    cfsetospeed(&tty, (speed_t)spd);
    cfsetispeed(&tty, (speed_t)spd);
  }
  switch (bit) {
  case '5':
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS5;
    break;
  case '6':
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS6;
    break;
  case '7':
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;
    break;
  case '8':
  default:
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    break;
  }		
  /* Set into raw, no echo mode */
#if !defined(_DGUX_SOURCE)
  tty.c_iflag &= ~(IGNBRK | IGNCR | INLCR | ICRNL | // IUCLC | 
		   IXANY | IXON | IXOFF | INPCK | ISTRIP);
  tty.c_iflag |= (BRKINT | IGNPAR);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag = ~(ICANON | ISIG | ECHO | ECHONL | ECHOE | ECHOK | IEXTEN);
  tty.c_cflag |= CREAD | CRTSCTS;
#else /* Okay, this is better. XXX - Fix the above. */
  tty.c_iflag =  IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cflag |= CLOCAL | CREAD;
#endif
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 5;
  
  /* Flow control. */
  if(hwf) {
    tty.c_cflag |= CRTSCTS;
    tty.c_cflag &= ~CLOCAL;
  }
  else {
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL;
  }

  if(swf) {
    tty.c_iflag |= (IXON | IXOFF);
  }
  else {
    tty.c_iflag &= ~(IXON | IXOFF);
  }
  
  tty.c_cflag &= ~(PARENB | PARODD);
  
  if(par[0] == 'E')
    tty.c_cflag |= PARENB;
  else if(par[0] == 'O')
    tty.c_cflag |= PARODD;

  tcsetattr(fd, TCSANOW, &tty);

  carmen_serial_setrts(fd);
#ifdef _DGUX_SOURCE
  serial_sethwf(fd, hwf);
#endif
#endif
}

void carmen_serial_configure(int dev_fd, int baudrate, const char *parity)
{
  switch(baudrate) {
  case 9600: 
    carmen_serial_setparms(dev_fd, "9600", parity, "8", 0, 0);
    break;
  case 19200: 
    carmen_serial_setparms(dev_fd, "19200", parity, "8", 0, 0);
    break;
  case 38400: 
    carmen_serial_setparms(dev_fd, "38400", parity, "8", 0, 0);
    break;
  case 57600:
  case 55555:
    carmen_serial_setparms(dev_fd, "57600", parity, "8", 0, 0);
    break;
  case 115200:
    carmen_serial_setparms(dev_fd, "115200", parity, "8", 0, 0);
  case 500000:
    carmen_serial_setparms(dev_fd, "500000", parity, "8", 0, 0);
    break;
  }
}

long carmen_serial_numChars(int dev_fd)
{
  long available = 0;
  
  if(ioctl(dev_fd, FIONREAD, &available) == 0)
    return available;
  else
    return -1;
}

int carmen_serial_ClearInputBuffer(int dev_fd)
{
  int max_serial_buffer_size = 16384; 
  unsigned char buffer[max_serial_buffer_size]; 
  int val = 0;
  int val_total = 0;
  
  val_total = carmen_serial_numChars(dev_fd);
  val = val_total;
  
  while (val > max_serial_buffer_size) {
    read(dev_fd, &buffer, max_serial_buffer_size);
    val -= max_serial_buffer_size;
  }
  
  if(val > 0) 
    read(dev_fd, &buffer, val);
  
  return(val_total);
}

int carmen_serial_writen(int dev_fd, unsigned char *buf, int nChars)
{
  int amountWritten = 0;
  
  while(nChars > 0) {
    amountWritten = write(dev_fd, buf, nChars);
    if(amountWritten < 0) {
      if(errno == EWOULDBLOCK) {
        fprintf(stderr, "\nWARNING: writeN: EWOULDBLOCK: trying again!\n");
      } 
      else {
        return -1;
      }
    }
    else {
      nChars -= amountWritten;
      buf += amountWritten;
    }
  }
  return 0;
}

int carmen_serial_readn(int dev_fd, unsigned char *buf, int nChars)
{
  int amountRead = 0, bytes_read = 0;
  struct timeval t;
  fd_set set;
  int err;

  while(nChars > 0) {
    t.tv_sec = 0;
    t.tv_usec = READ_TIMEOUT;
    FD_ZERO(&set);
    FD_SET(dev_fd, &set);
    err = select(dev_fd + 1, &set, NULL, NULL, &t);
    if(err == 0)
      return -2;

    amountRead = read(dev_fd, buf, nChars);
    if(amountRead < 0 && errno != EWOULDBLOCK)
      return -1;
    else if(amountRead > 0) {
      bytes_read += amountRead;
      nChars -= amountRead;
      buf += amountRead;
    }
  }
  return bytes_read;
}


int carmen_serial_close(int dev_fd) {
  return close(dev_fd);
}


int carmen_serial_set_low_latency(int fd){
#ifdef CYGWIN
  fd = fd;
  return -1;
#else

  struct serial_struct serial; 
  int result;
  result=ioctl(fd, TIOCGSERIAL, &serial);
  
  if (result) {
/*     carmen_warn("libcarmenserial: Cannot get the serial attributes for low latency serial mode.\n Switching to normal mode\n"); */
    return result;
  } else {
    serial.flags |= ASYNC_LOW_LATENCY;
    serial.xmit_fifo_size = 1;
    ioctl(fd, TIOCSSERIAL, &serial); 
    if (result){
/*       carmen_warn("libcarmenserial: Cannot activeate low latency mode\nSwitching to normal mode\n"); */
      return result;
    }
  }
  return result;
#endif
}

