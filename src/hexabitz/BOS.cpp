/*
    BitzOS (BOS) V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved

    File Name     : BOS.cpp
    Description   : Source code for Bitz Operating System (BOS).
		
*/

/* Includes ------------------------------------------------------------------*/

#include "BOS.h"

/* Welcome Message ------------------------------------------------------------*/

const char *WelcomeMessage =	
"\n\r\n\r====================================================	\
     \n\r====================================================	\
     \n\r||            Welcome to BitzOS CLI!              ||	\
	 \n\r||       (C) COPYRIGHT HEXABITZ 2017-2020.        ||	\
     \n\r||                                                ||	\
	 \n\r||      Please check the project website at       ||	\
	 \n\r||             http://hexabitz.com/               ||	\
     \n\r||                                                ||	\
     \n\r||   This BitzOS is running on Raspberry Pi       ||	\
     \n\r====================================================	\
     \n\r====================================================	\
     \r";
     
/* Function prototypes -------------------------------------------------------*/
void init(void);
void delay_s(uint8_t duration);


BOS_t BOS;
BOS_t BOS_default = {.response = BOS_RESPONSE_NONE, .trace = TRACE_NONE };
											 

/*----------------------------------APIS-------------------------------------*/

void init(void){
  
  std::cout<<WelcomeMessage<<std::endl;
  std::cout<<"Bitzos version V"<<_firmMajor<<"."<<_firmMinor<<"."<<_firmPatch<<"\n"<<std::endl;
}

void delay_s(uint8_t duration){
    
    using namespace std::this_thread; // sleep_for, sleep_until
    using namespace std::chrono; // nanoseconds, system_clock, seconds
    
    
    sleep_for(seconds(duration));
    sleep_until(system_clock::now() + seconds(1));
  

}
    
/*-------------------------APIS----------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
