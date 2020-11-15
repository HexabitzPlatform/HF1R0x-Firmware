/*
    BitzOS (BOS) V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : BOS_Message.h
    Description   : Hexabitz Messaging header file. .
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef BOSMESSAGE_H
#define BOSMESSAGE_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <string>
/* exported Variables ------------------------------------------------------------------*/
extern int fd;

/* exported Functions ------------------------------------------------------------------*/
extern void SendMessageToModule(uint16_t dst, uint16_t code, uint16_t numberOfParams);
extern uint8_t *ReceiveMessage(uint8_t *buffer,int length);
extern uint8_t *Receivedata(uint8_t *buffer,int length,uint32_t period , uint32_t timeout);
extern uint8_t messageParams[46];

#endif /* BOSMESSAGE_H */

		
