#include "hexabitz-modules/H01R0.h"

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

#define	CODE_H01R0_ON							100
#define	CODE_H01R0_OFF							101
#define	CODE_H01R0_TOGGLE						102
#define	CODE_H01R0_COLOR						103
#define	CODE_H01R0_PULSE						104
#define	CODE_H01R0_SWEEP						105
#define	CODE_H01R0_DIM							106


bool H01R0::setRGB(int red, int green, int blue, int intensity)
{
	hstd::Message msg = hstd::make_message(id_, CODE_H01R0_COLOR);
	msg.getParams().append(static_cast<uint8_t>(1));
	msg.getParams().append(static_cast<uint8_t>(red));
	msg.getParams().append(static_cast<uint8_t>(green));
	msg.getParams().append(static_cast<uint8_t>(blue));
	msg.getParams().append(static_cast<uint8_t>(intensity));

	return send(msg);
}