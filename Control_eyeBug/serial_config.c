// $Id: serial_config.c,v 1.2 2013/02/01 06:06:32 ahmet Exp ahmet $
/**
 * Library for Microsoft kinect device
 * Autor : Alexandre PROUST
 * Date : 16/06/2011
 *
 * configure serial port for future communication
 */

#ifndef _SERIALCONFIG
#define _SERIALCONFIG

//#include "serial_config.h"
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#define BAUDRATE	B115200
#define MODEMDEVICE "/dev/ttyO1"


		///*************///
		///Serial config///
		///*************///
		
int serialconfig()
{
    int srl_handle;
    struct termios options;
    
    srl_handle = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if(srl_handle < 0) {
	perror("serial port open");
	exit(-1);
    }
    //~ else {	
    //~ fcntl(srl_handle,F_SETFL,0);
    //~ }
    
    tcgetattr(srl_handle, &options);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(srl_handle, TCSANOW, &options);
    return srl_handle;
}

#endif //_SERIALCONFIG
