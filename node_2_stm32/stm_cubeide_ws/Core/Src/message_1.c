/*
 * message_1.c
 *
 *  Created on: May 23, 2022
 *      Author: Victor
 */

#include "message_1.h"

//// this function on receive
message1_getData(MSGrcv* msg,char* databuffer, char size){
	uint32_t temp;

	if(size == 12)
	{
		msg->ID=databuffer[0];
		msg->digital=databuffer[1];

		temp  = (uint32_t)databuffer[5] << 24 ;       //MSB  //reassembling in opposite way
		temp |= (uint32_t)databuffer[4] << 16;
		temp |= (uint32_t)databuffer[3] << 8;
		temp |= (uint32_t)databuffer[2];  ///LSB

		memcpy(&(msg->analog_converted), &temp, sizeof(float));
//		msg->analog_converted = databuffer[2] <<24 | databuffer[3] <<16 | databuffer[4] <<8 | databuffer[5] ;
		msg->analog[0] = databuffer[6];
		msg->analog[1] = databuffer[7];
		msg->future[0] = databuffer[8];
		msg->future[1] = databuffer[9];
		msg->future[2] = databuffer[10];
		msg->future[3] = databuffer[11];
	}

}

message1_addData(MSGrcv* msg,char* databuffer, char size){
	databuffer[0]=msg->ID;
	databuffer[1]=msg->digital;


//	databuffer[2] = char (((msg->analog_converted & 0xFF000000) >>24) & 0xFF);  // last FF for safety //MSB
//	databuffer[3] = (((msg->analog_converted & 0x00FF0000) >>24) & 0xFF); // last FF for safety
//	databuffer[4] = (((msg->analog_converted & 0x0000FF00) >>24) & 0xFF); // last FF for safety
//	databuffer[5] = (((msg->analog_converted & 0x000000FF) >>24) & 0xFF); // last FF for safety //LSB
	//msg->analog_converted = databuffer[2] <<24 | databuffer[3] <<16 | databuffer[4] <<8 | databuffer[5] ;

	memcpy((databuffer+2), &(msg->analog_converted), sizeof(float)); // this will put MSB first out

	databuffer[6]=msg->analog[0] ;
	databuffer[7]=msg->analog[1];
	databuffer[8]=msg->future[0];
    databuffer[9]=msg->future[1];
	databuffer[10]=msg->future[2];
	databuffer[11]=msg->future[3];


}
