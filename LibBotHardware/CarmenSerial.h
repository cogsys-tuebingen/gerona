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


/** @addtogroup global libcarmenserial **/
// @{

/** \file carmenserial.h
 * \brief Serial library.
 *
 * Library to read data from and write data to the serial line.
 **/


#ifndef CARMEN_SERIAL_H
#define CARMEN_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

  /** Open a connection to the serial line 
   *
   * @param *dev_fd returns the file descriptor associated to the serial line
   * @param dev_name the name of the serial device, e.g. /dev/ttyS0
   * @return 0 if everything is fine, -1 in case of an error.
   **/
int carmen_serial_connect(int *dev_fd, char *dev_name);


  /** Sets the parameters for (a connected) serial line.
   *
   * @param dev_fd The file descriptor associated to the serial line.
   * @param baudrate The baud rate to use (e.g. 9600, 19200, etc.).
   * @param partity The parity to use (e.g. "N" for none).
   **/
void carmen_serial_configure(int dev_fd, int baudrate, const char *parity);

  /** Returns the number of availabe bytes
   *
   * @param *dev_fd returns the file descriptor associated to the serial line
   * @return number of available bytes or -1 in case of an error.
   **/
long carmen_serial_numChars(int dev_fd);

  /** Clears the buffer of the serial line.
   *
   * @param *dev_fd returns the file descriptor associated to the serial line
   * @return number of bytes removed from the serial line.
   **/
int carmen_serial_ClearInputBuffer(int dev_fd);

  /** Reads data from the serial line
   *
   * @param dev_fd The file descriptor associated to the serial line.
   * @param *buf Pointer to unsiged char buffer to the data to be send over the serial line
   * @param nChars Number of bytes in buf
   * @return The number of bytes sent to the serial line.
   **/
int carmen_serial_writen(int dev_fd, unsigned char *buf, int nChars);

  /** Reads data from the serial line
   *
   * @param dev_fd The file descriptor associated to the serial line.
   * @param *buf Pointer to unsiged char buffer for the data to be read
   * @param nChars Number of bytes to be read (<= size of the buffer array).
   * @return The number of bytes read.
   **/
int carmen_serial_readn(int dev_fd, unsigned char *buf, int nChars);

  /** Closes the file descriptior.
   *
   * @param dev_fd The file descriptor associated to the serial line.
   * @return The results reported by close.
   **/
int carmen_serial_close(int dev_fd);

  /** Avtivates the low_latency mode for the serial line. This works
   * with real serial devices, USB-to-RS232 often does not work. In
   * this case, the system continues in the standard operational
   * mode. Note: Low latency does not run with cygwin.
   *
   * @param dev_fd The file descriptor associated to the serial line.
   * @return 0=successful switched to low latency mode. 0=continue in normal mode.
   **/
int carmen_serial_set_low_latency(int fd);

#ifdef __cplusplus
}
#endif

#endif
// @}
