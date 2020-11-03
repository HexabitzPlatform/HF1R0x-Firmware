/*
    BitzOS (BOS) V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : BOS_Message.cpp
    Description   : Hexabitz Messaging source file. .
*/

/* Includes ------------------------------------------------------------------*/

#include "BOS_Message.h"

/* Function prototypes -------------------------------------------------------*/

void SendMessage(uint16_t src, uint16_t dst, uint16_t code, uint16_t numberOfParams);
uint16_t *reverse_array(uint16_t *array, uint8_t length);
uint32_t crc32b(uint16_t *message, size_t l);
uint8_t *ReceiveMessage(uint8_t *buffer,int length);
uint8_t *Receivedata(uint8_t *buffer,int length,uint32_t period , uint32_t timeout);
/*definitions -------------------------------------------------------*/

#define MAX_MESSAGE_SIZE 56
#define MAX_PARAMS_PER_MESSAGE (MAX_MESSAGE_SIZE - 10)    // H + Z + length + Dst + Src + 1 x Options + 2 x Code + CRC + 1 x reserved = 10
#define Null 0

/*variables ---------------------------------------------------------*/
uint8_t messageParams[MAX_PARAMS_PER_MESSAGE] = {0};
int fd;
typedef unsigned char uchar;

/*----------------------------------APIS-------------------------------------*/

/**
  * @brief Send Frame of data to one of Modules.
  * @param src: source module.
  * @param dst: destination module.
  * @param code: which code to be sent.
  * @param numberOfParams: number of parameters to be sent.
  * @retval void
  */
void SendMessage(uint16_t src, uint16_t dst, uint16_t code, uint16_t numberOfParams)
{
	uint16_t message[MAX_MESSAGE_SIZE] = {0};
	uint16_t message1[MAX_MESSAGE_SIZE] = {0};
	uint8_t length = 0, length1 = 0, shift = 0, ptrshift = 0,crcshift=0;
	uint16_t *buf1;
	uint32_t crc=0;
	bool extendOptions=false,extendCode=false;
	static uint16_t totalNumberOfParams = 0;
	
	/*Try to open serial port */
	if ((fd = serialOpen("/dev/serial0", 921600)) < 0)
		std::cout << "not Connected successfully" << std::endl;
		
		/*setup Raspberry pi GPIO*/

	if (wiringPiSetup() == -1)
		std::cout << "Raspberry pi is not setuped  successfully" << std::endl;

	/* HZ Delimiter */
	message[0] = 'H';
	message[1] = 'Z';
	
	/* Header */
	message[3] = dst;
	message[4] = src;
	
	if(code> 0xFF) extendCode=true;

	/* Options */
	/*Long Message (8th-MSB) Response (7th - 6th) : Reserved (5th) : Trace (4th-3rd) : Extended Code (2nd) : Extended Options (1st-LSB) */
	message[5] = (BOS.response)| (BOS.trace<<2) | (extendCode<<1) | (extendOptions);
	if(extendOptions==true) {shift++;}
		
	/*Code-LSB first */	
	message[6+shift] = uint8_t(code);
	if(extendCode==true){
		shift++;
		message[6+shift]=uint8_t(code>>8);
	}
	
	/*Parameters*/
		if(numberOfParams<=MAX_PARAMS_PER_MESSAGE){
			for(int ii=0;ii<numberOfParams;ii++){
			message[7+shift+ii]=messageParams[0+ii];}
	
	/* Calculate message length */
			length = numberOfParams + shift + 4;

			} else {
			/* Long message: Set Options byte 8th bit */
			message[5] |= 0x80;		
			totalNumberOfParams = numberOfParams;
			numberOfParams = MAX_PARAMS_PER_MESSAGE;
			/* Break into multiple messages */
				while (totalNumberOfParams != 0)
			{		
				if ( (totalNumberOfParams/numberOfParams) >= 1) 
				{	
					/* Call this function recursively */
					SendMessage(src,dst,code,numberOfParams);
					delay_s(1);
					/* Update remaining number of parameters */
					totalNumberOfParams -= numberOfParams;
					ptrshift += numberOfParams;
				} 
				else 
				{
					message[5] &= 0x7F;		/* Last message. Reset long message flag */
					numberOfParams = totalNumberOfParams;
					memcpy((char*)&message[7+shift], (&messageParams[0]+ptrshift), numberOfParams);
					ptrshift = 0; totalNumberOfParams = 0;
					/* Calculate message length */
					length = numberOfParams + shift + 4;
				}
			}
		}	

		length1 = length + 4;
		message[2] = length;
		for(int g=0;g<length1;g++){
			message1[g]=message[g];
		}

	
		/* End of message - Calculate CRC8 */	
		uint8_t totallength=length+3;

		if(totallength%2!=0) totallength++;
	
		buf1 = reverse_array(message, totallength);
		crc= crc32b(buf1,totallength);
	 
		message1[length+3] = crc;
		
	/* Transmit the message from this port */

	for (int i = 0; i <length1; i++)
	{
		serialPutchar(fd, message1[i]);
		std::cout<<"Message["<<i<<"]=0x"<<std::hex<<(message1[i])<<std::endl;
	}
	
	
	serialClose(fd);
}

/**
  * @brief Compute the CRC value of data buffer
  * @param message: data buffer.
  * @param length: data buffer length.
  * @retval uint32_t CRC(returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t crc32b(uint16_t *message, size_t length)
{
	size_t i, j;
	unsigned int msb;
	unsigned int crc = 0xFFFFFFFF;
	for (i = 0; i < length; i++)
	{
		// xor next byte to upper bits of crc
		crc ^= (((unsigned int)message[i]) << 24);
		for (j = 0; j < 8; j++)
		{ // Do eight times.
			msb = crc >> 31;
			crc <<= 1;
			crc ^= (0 - msb) & 0x4C11DB7;
		}
	}
	return crc; // don't complement crc on output
}
/**
  * @brief reverse data of array to use it in CRC calcultion function.
  * @param array: array that you want to reverse.
  * @param length: the length of array
  * @retval reversed array
  */
uint16_t *reverse_array(uint16_t *array, uint8_t length)
{
	int l=ceil(length/4);

	if(l<2)l=2;
	
	std::cout<<"l="<<l<<std::endl;

	static uint16_t array_temp[20]={0};
	uint8_t shift=0;
	uint8_t jj=3;
	for(int i=1;i<=l;i++){
	
	for (int ii = shift+0; ii <shift+4; ++ii)
	{
		array_temp[ii] = array[jj+shift];
		jj--;
	}
	jj=3;
	shift=shift+4;
	}
	
for(int g=0;g<length;g++){
			array[g]=array_temp[g];
		}
			
	return array;
}
/**
  * @brief Receive an amount of data.
  * @param buffer: pointer to data buffer.
  * @param length: amount of data to be receeived.
  * @retval data buffer
  */
uint8_t *ReceiveMessage(uint8_t *buffer,int length)
{
	fd_set set;
	int rv,x=0;
	char array1[length]={0};
	int data=0,i=0;

		memset(buffer,0,length);
		serialFlush(fd);
	
	
	if ((fd = serialOpen("/dev/serial0", 921600)) < 0)
		std::cout << "not Connected successfully" << std::endl;
		
		FD_ZERO(&set);
		FD_SET(fd,&set);
			
		struct timeval timeout1;
		timeout1.tv_sec=0;
		timeout1.tv_usec=10000;
		
		rv=select(fd+1,&set,Null,Null,&timeout1);
		
		if(rv==-1)
		std::cout<<"select"<<std::endl;
		else if(rv=0)
		std::cout<<"timout"<<std::endl;
		else{
		int rv1=read(fd,array1,sizeof(array1));
		if(rv1<0) std::cout<<"read failed"<<std::endl;
		}

		
	for(int g=0;g<length;g++){
			buffer[g]=array1[g];
		}
	serialClose(fd);
		
	return buffer;
}



/* convert received Bytes into float */
float bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3)
{
    float output;

    *((uchar*)(&output) + 3) = b0;
    *((uchar*)(&output) + 2) = b1;
    *((uchar*)(&output) + 1) = b2;
    *((uchar*)(&output) + 0) = b3;

    return output;
}


uint8_t *Receivedata(uint8_t *buffer,int length ,uint32_t period , uint32_t timeout)
{
	  uint8_t array1[length]={0};
	  float temp1=0, temp2=0,temp3=0;

		memset(buffer,0,length);
		serialFlush(fd);
	
	int i=0;
	if ((fd = serialOpen("/dev/serial0", 921600)) < 0)
		std::cout << "not Connected successfully" << std::endl;
		
			long numTimes = timeout / period;
		while(numTimes-- >0){
			
			do{
			array1[i]=serialGetchar(fd);
			i++;
			if(i==3){
				temp1=bytesToFloat(array1[0], array1[1], array1[2], array1[3]);
				std::cout<<"Gyro X="<<temp1<<" | ";}


			if(i==7){
				temp2=bytesToFloat(array1[4], array1[5], array1[6], array1[7]);
				std::cout<<" Y="<<temp2<<" | ";}
					
			if(i==11){
				temp3=bytesToFloat(array1[8], array1[9], array1[10], array1[11]);
				std::cout<<" Z="<<temp3<<std::endl; i=0;}
				
			} while(serialDataAvail(fd)>0);
		}
		
		std::cout<<"no more data to receive"<<std::endl;

	serialClose(fd);
		
	return buffer;
}
/*-----------------------------------------------------------*/
