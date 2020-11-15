/*
    BitzOS (BOS) V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : BOS.h
    Description   : Header file for Bitz Operating System (BOS).
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOS_H
#define BOS_H

/* Includes ------------------------------------------------------------------*/

/* C STD Libraries */
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include <thread>
#include <chrono>
#include <ctype.h>
#include <tgmath.h>
#include <cstring>
#include <fcntl.h>

#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <iostream>

/*Raspberry pi Libraries */
#include "wiringPi.h"
#include "wiringSerial.h"

/*Hexabitz Libraries */
#include "MessageCode.h"
#include "BOS_Message.h"
#include "BOS_Porting.h"

/* Firmware version */
#define	_firmMajor			0
#define	_firmMinor			2
#define	_firmPatch			3

#define BOS_RESPONSE_ALL							0x60			// Send response messages for both Messaging and CLI
#define BOS_RESPONSE_MSG							0x20			// Send response messages for Messaging only (no CLI)
#define BOS_RESPONSE_CLI							0x40			// Send response messages for CLI only (no messages)
#define BOS_RESPONSE_NONE							0x00			// Do not send any response messages

typedef enum { TRACE_NONE = 0, TRACE_MESSAGE, TRACE_RESPONSE, TRACE_BOTH } traceOptions_t;

/* BOS Struct Type Definition */  

typedef struct
{
	uint8_t response;
	traceOptions_t trace;
} 
BOS_t;

/* BOS_Status Type Definition */  
typedef enum 
{   BOS_OK = 0,
    BOS_ERR_UnknownMessage = 1,
    BOS_ERR_NoResponse = 2,
    BOS_MULTICAST = 254,
    BOS_BROADCAST = 255,
    } BOS_Status;

/* Exported internal functions ---------------------------------------------------------*/
extern void init(void);
extern void delay_s(uint8_t duration);
extern BOS_t BOS; 

#endif /* BOS_H */


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
