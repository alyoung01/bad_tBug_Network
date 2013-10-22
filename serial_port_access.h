/*
 * serial_port_access.h
 *
 *  Created on: Sep 18, 2013
 *      Author: al
 */

#ifndef SERIAL_PORT_ACCESS_H_
#define SERIAL_PORT_ACCESS_H_
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#define BAUDRATE	B115200
#define MODEMDEVICE "/dev/ttyO1"


int serialconfiguration();


#endif /* SERIAL_PORT_ACCESS_H_ */
