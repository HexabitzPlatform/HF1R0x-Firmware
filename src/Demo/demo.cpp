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


	
	std::cout<<"Press any Key and enter to exit"<<std::endl;
	while(1){
	  
	  
	  if(getchar()!=0) break;
	  
	  delay_s(1);
	      }
	std::cout<<"exiting ..."<<std::endl;
  
	return 0;
}
/*----------------------------------------------------------------------------*/
