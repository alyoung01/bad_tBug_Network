/*
 * sender.cpp
 *
 *  Created on: Sep 16, 2013
 *      Author: al
 */

#include "sender.h"

void sender(std::vector<char> command_packet)
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************

	results << "\t Sending command to eBug via the UART" << endl;
	int srl_handle = serialconfiguration();
//
	write(srl_handle, &command_packet[0],command_packet.size());

	// Close the results file
	//*****************************************************************************************************
	results.flush();
	results.close();
}
