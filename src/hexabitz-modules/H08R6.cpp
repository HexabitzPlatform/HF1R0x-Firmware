#include "hexabitz-modules/H08R6.h"

#include "helper/helper.h"

#include "hexabitz/Service.h"
#include "hexabitz/BOSMessage.h"
#include "hexabitz/BOSMessageBuilder.h"

#include <iostream>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include <float.h>
#include <errno.h>


#define CODE_H08R6_GET_INFO                         800
#define CODE_H08R6_SAMPLE                           801
#define CODE_H08R6_STREAM_PORT                      802
#define CODE_H08R6_STREAM_MEM                       803
#define CODE_H08R6_RESULT_MEASUREMENT               804
#define CODE_H08R6_STOP_RANGING                     805
#define CODE_H08R6_SET_UNIT                         806
#define CODE_H08R6_GET_UNIT                         807
#define CODE_H08R6_RESPOND_GET_UNIT                 808
#define CODE_H08R6_MAX_RANGE                        809
#define CODE_H08R6_MIN_RANGE                        810
#define CODE_H08R6_TIMEOUT                	        811



void H08R6::addCodes(void)
{
    BOS_ADD_CODE(CODE_H08R6_GET_INFO);
    BOS_ADD_CODE(CODE_H08R6_SAMPLE);
    BOS_ADD_CODE(CODE_H08R6_STREAM_PORT);
    BOS_ADD_CODE(CODE_H08R6_STREAM_MEM);
    BOS_ADD_CODE(CODE_H08R6_RESULT_MEASUREMENT);
    BOS_ADD_CODE(CODE_H08R6_STOP_RANGING);
    BOS_ADD_CODE(CODE_H08R6_SET_UNIT);
    BOS_ADD_CODE(CODE_H08R6_GET_UNIT);
    BOS_ADD_CODE(CODE_H08R6_RESPOND_GET_UNIT);
    BOS_ADD_CODE(CODE_H08R6_MAX_RANGE);
    BOS_ADD_CODE(CODE_H08R6_MIN_RANGE);
    BOS_ADD_CODE(CODE_H08R6_TIMEOUT);
}

bool H08R6::setMeasurementUnit(MeasurementUnit unit)
{
	hstd::Message msg = hstd::make_message(id_, CODE_H08R6_SET_UNIT);
	msg.getParams().append(static_cast<uint8_t>(unit));
	return send(msg);
}

bool H08R6::getMeasurementUnit(MeasurementUnit& unit)
{
    // return false;
	if (!send(hstd::make_message(id_, CODE_H08R6_GET_UNIT)))
        return false;
    hstd::Message msg;
    if (!receive(msg))
        return false;
    if (msg.getCode() != CODE_H08R6_RESPOND_GET_UNIT)
        return false;
    unit = static_cast<MeasurementUnit>(msg.getParams().popui8());
    return true;
}

bool H08R6::sample(float& value)
{
	if (!send(hstd::make_message(id_, CODE_H08R6_SAMPLE)))
        return false;
    hstd::Message msg;
    if (!receive(msg))
        return false;
    if (msg.getCode() != CODE_H08R6_RESULT_MEASUREMENT)
        return false;
    value = msg.getParams().popfloat();
    return true;
}