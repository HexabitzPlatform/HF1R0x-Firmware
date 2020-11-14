/*
    BitzOS (BOS) V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : BOS_Porting.cpp
    Description   : Hexabitz Messaging source file. .
*/
/* Includes ------------------------------------------------------------------*/

#include "BOS_Porting.h"

/*variables ---------------------------------------------------------*/

typedef unsigned char uchar;

/* Function prototypes -------------------------------------------------------*/
uint8_t *ReceiveIMU(uint8_t *buffer,int length,uint32_t period , uint32_t timeout);
uint8_t *ReceiveIR(uint8_t *buffer,int length,uint32_t period , uint32_t timeout,const std::string& unit);
uint8_t *ReceiveWeight(uint8_t *buffer,int length,uint32_t period , uint32_t timeout,const std::string& unit);
float bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3);



/**
  * @brief Receive an amount of From IMU Module(H0BR4x).
  * @param buffer: pointer to data buffer.
  * @param dst: amount of data to be receeived.
  * @param period: Period duration.
  * @param timeout: Timeout duration.
  * @retval data buffer
  */

uint8_t *ReceiveIMU(uint8_t *buffer,int length,uint32_t period , uint32_t timeout)
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
/**
  * @brief Receive an amount of data from IR Modules (H08R6x or P08R6x).
  * @param buffer: pointer to data buffer.
  * @param dst: amount of data to be receeived.
  * @param period: Period duration.
  * @param timeout: Timeout duration.
  * @param unit: it either MM,CM or inch.
  * @retval data buffer
  */
uint8_t *ReceiveIR(uint8_t *buffer,int length,uint32_t period , uint32_t timeout,const std::string& unit)
{
	  uint8_t array2[length]={0};
	  float temp1=0;

		memset(buffer,0,length);
		serialFlush(fd);
	
	int i=0;
	if ((fd = serialOpen("/dev/serial0", 921600)) < 0)
		std::cout << "not Connected successfully" << std::endl;
		
		long numTimes = timeout / period;
		while(numTimes-- >0){
			
			do{
		read (fd,(void*)&array2[i], 1);	
		std::cout<<"array2 <"<<i<<">:"<<std::dec<<int(array2[i])<<std::endl;

		if(array2[i]==CODE_H08R6_MAX_RANGE) std::cout<<"distance : max"<<std::endl;
		else if (array2[i]==CODE_H08R6_MIN_RANGE) std::cout<<"distance : min"<<std::endl;
		else{	
		if(i==3){
		//temp1=( (uint8_t) array1[3] << 24 ) + ( (uint8_t) array1[2] << 16 ) + ( (uint8_t) array1[1] << 8 ) + ((uint8_t) array1[0]);
		temp1=bytesToFloat(array2[0],array2[1],array2[2],array2[3]);
		std::cout<<"distance <"<<unit<<">:"<<std::dec<<temp1<<std::endl;
		}
			
			i++;
			if(i==4) i=0;}				
			} while(serialDataAvail(fd)>0);
		}
		
		std::cout<<"no more data to receive"<<std::endl;

	serialClose(fd);
		
	return buffer;
}
/**
  * @brief Receive an amount of data from weight Module (H26R0x).
  * @param buffer: pointer to data buffer.
  * @param dst: amount of data to be receeived.
  * @param period: Period duration.
  * @param timeout: Timeout duration.
	@param unit: it either G , KG , pound or ounce.
  * @retval data buffer
  */
uint8_t *ReceiveWeight(uint8_t *buffer,int length,uint32_t period , uint32_t timeout,const std::string& unit)
{
	 
	  uint8_t array2[length]={0};
	  uint8_t temp1=0,i=0;	

		memset(buffer,0,length);
		serialFlush(fd);
	
	if ((fd = serialOpen("/dev/serial0", 921600)) < 0)
		std::cout << "not Connected successfully" << std::endl;
		
		long numTimes = timeout / period;
		while(numTimes-- >0){
			
		do{
		read (fd,(void*)&array2[i], 1);	
		std::cout<<"array2 <"<<i<<">:"<<std::dec<<int(array2[i])<<std::endl;

		if(array2[i]==CODE_H08R6_MAX_RANGE) std::cout<<"distance : max"<<std::endl;
		else if (array2[i]==CODE_H08R6_MIN_RANGE) std::cout<<"distance : min"<<std::endl;
		else{	
		if(i==3){
		temp1=( (uint8_t) array2[0] << 24 ) + ( (uint8_t) array2[1] << 16 ) + ( (uint8_t) array2[2] << 8 ) + ((uint8_t) array2[3]);
		//temp1=bytesToFloat(array2[0],array2[1],array2[2],array2[3]);
		std::cout<<"Weight <"<<unit<<">:"<<std::dec<<int(temp1)<<std::endl;}
			
		i++;
		if(i==4) i=0;}				
			
			} while(serialDataAvail(fd)>0);
		}
		
	std::cout<<"no more data to receive"<<std::endl;
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
/*----------------------------------------------------------------------------*/
