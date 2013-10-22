/*
 * sender.h
 *
 *  Created on: Sep 16, 2013
 *      Author: al
 */


#ifndef SENDER_H_
#define SENDER_H_

using namespace std;

#include <fstream>
#include <unistd.h>
#include "serial_port_access.h"
#include "Control_eyeBug/eBugAPI.h"

void sender(std::vector<char> command_packet);

#endif /* SENDER_H_ */
