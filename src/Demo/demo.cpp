/*
    BitzOS (BOS) V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : demo.cpp
    Description   : Hexabitz demo source file. .
*/
/* Includes ------------------------------------------------------------------*/

#include "../hexabitz/BOS.h"
  
uint8_t array[10]={0};


int main(int argc, char *argv[])
{
  
      init();
	
	SendMessage(1,2,CODE_PING,0);
	delay_s(1);

	uint32_t period=1000;
	uint32_t timeout=20000;
	memcpy(&messageParams[0],&period,4);
	memcpy(&messageParams[4],&timeout,4);
	SendMessage(1,2,CODE_H0BR4_STREAM_GYRO,8);

	Receivedata(array,12,period,timeout);

	
	
	std::cout<<"Press any Key and enter to exit"<<std::endl;
	while(1){
	  
	  if(getchar()!=0) break;
	  
	  delay_s(1);
	      }
	std::cout<<"exiting ..."<<std::endl;
  
	return 0;
}
