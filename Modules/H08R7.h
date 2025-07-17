#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H08R7 User Interface ***************************************************************************/
/**************************************************************************************************/
struct H08R7_TOF
{
    BOSStatus status;
    uint16_t distance = 0;
};

class H08R7
{
public:
    static std::promise<H08R7_TOF> distancePromise;

    static H08R7_TOF RequestTOF(uint8_t moduleID);

};