/*
    BitzOS (BOS) V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : BOS_Porting.cpp
    Description   : Hexabitz Messaging source file. .
*/
/*  ----------------------------Includes--------------------------------------*/

#include "BOS_Porting.h"

/* -----------------------------variables-------------------------------------*/

typedef unsigned char uchar;

/*  ------------------------------Function prototypes-------------------------*/
uint8_t *ReceiveIMU(uint8_t *buffer,int length,uint32_t period , uint32_t timeout);
uint8_t *ReceiveIR(uint8_t *buffer,int length,uint32_t period , uint32_t timeout,const std::string& unit);
uint8_t *ReceiveWeight(uint8_t *buffer,int length,uint32_t period , uint32_t timeout,const std::string& unit);
float bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3);


/*------------------------------API ------------------------------------------*/

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
	  uint8_t array_temp[length]={0};
	  float temp1=0, temp2=0,temp3=0;

		memset(buffer,0,length);
		serialFlush(fd);
	
	int i=0;
	if ((fd = serialOpen("/dev/serial0", 921600)) < 0)
		std::cout << "not Connected successfully" << std::endl;
		
		long numTimes = timeout / period;
		while(numTimes-- >0){
			
			do{
			array_temp[i]=serialGetchar(fd);
			
			if(i==3){
				temp1=bytesToFloat(array_temp[0], array_temp[1], array_temp[2], array_temp[3]);
				std::cout<<"Gyro X="<<temp1<<" | ";}


			if(i==7){
				temp2=bytesToFloat(array_temp[4], array_temp[5], array_temp[6], array_temp[7]);
				std::cout<<" Y="<<temp2<<" | ";}
					
			if(i==11){
				temp3=bytesToFloat(array_temp[8], array_temp[9], array_temp[10], array_temp[11]);
				std::cout<<" Z="<<temp3<<std::endl;}
				i++;
				
			if(i==12) i=0;
			
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
	  uint8_t array_temp[length]={0};
	  float temp1=5;

		memset(buffer,0,length);
		serialFlush(fd);
	
	int i=0;
	if ((fd = serialOpen("/dev/serial0", 921600)) < 0)
		std::cout << "not Connected successfully" << std::endl;
		
		long numTimes = timeout / period;
		while(numTimes-- >0){
			
	 do{
		array_temp[i]=serialGetchar(fd);

		if(array_temp[i]=='H') std::cout<<"out of range"<<std::endl;
		else{	
		if(i==3){

		temp1=bytesToFloat(array_temp[0],array_temp[1],array_temp[2],array_temp[3]);
		
		if(temp1>800) 	std::cout<<"distance <"<<unit<<">: MAX"<<std::endl;
		
		else if(temp1<5) std::cout<<"distance <"<<unit<<">: MIN"<<std::endl;
		
		else std::cout<<"distance <"<<unit<<">:"<<std::dec<<temp1<<std::endl;
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
	 
	  uint8_t array_temp[length]={0};
	  float temp1=0;
	  uint8_t i=0;	

		memset(buffer,0,length);
		serialFlush(fd);
	
	if ((fd = serialOpen("/dev/serial0", 921600)) < 0)
		std::cout << "not Connected successfully" << std::endl;
		
		long numTimes = timeout / period;
		while(numTimes-- >0){
			
		do{
		array_temp[i]=serialGetchar(fd);

		if(i==3){
		temp1=bytesToFloat(array_temp[0],array_temp[1],array_temp[2],array_temp[3]);
		std::cout<<"Weight <"<<unit<<">:"<<std::dec<<temp1<<std::endl;}
	
		i++;
		if(i==4) i=0;			
			
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
