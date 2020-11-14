/*
    BitzOS (BOS) V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : BOS_Message.h
    Description   : Hexabitz Messaging header file. .
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef BOS_PORTING_H
#define BOS_PORTING_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <string>
/* exported Variables ------------------------------------------------------------------*/
extern int fd;

/* exported Functions ------------------------------------------------------------------*/
extern uint8_t *ReceiveIMU(uint8_t *buffer,int length,uint32_t period , uint32_t timeout);
extern uint8_t *ReceiveIR(uint8_t *buffer,int length,uint32_t period , uint32_t timeout,const std::string& unit);
extern uint8_t *ReceiveWeight(uint8_t *buffer,int length,uint32_t period , uint32_t timeout,const std::string& unit);
#endif /* BOS_PORTING_H */
