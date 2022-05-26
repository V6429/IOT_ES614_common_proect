/*
 * message_1.h
 *
 *  Created on: May 23, 2022
 *      Author: Victor
 */

#ifndef SRC_MESSAGE_1_H_
#define SRC_MESSAGE_1_H_
#include <ctype.h>
#include <stdint.h>
typedef struct message_recv{
	char   ID;
	char digital;
//	uint32_t  analog_converted;
	float  analog_converted;
	char analog[2]; // we are using a 12 bit adc right aligned here
	char future[4];
}MSGrcv;

typedef struct message_send{
	char   ID;
	char digital;
	uint32_t  analog_converted;
//	float  analog_converted;
	char analog[2]; // we are using a 12 bit adc right aligned here
	char future[4];
}MSGsend;

// takes data from data buffer and puts it onto a message structure
void message1_getData(MSGrcv* msg,char* databuffer, char size);

// takes data from a message structure and adds it to a data buffer
void message1_addData(MSGrcv* msg,char* databuffer, char size);



#endif /* SRC_MESSAGE_1_H_ */
