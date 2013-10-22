/*
 * serial_port_access.cpp
 *
 *  Created on: Sep 18, 2013
 *      Author: al
 */


#include "serial_port_access.h"

int serialconfiguration()
{

    int srl_handle;
    struct termios options;

    srl_handle = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY);

    if(srl_handle < 0)
    {
    	perror("serial port open");
    	exit(-1);
    }


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
