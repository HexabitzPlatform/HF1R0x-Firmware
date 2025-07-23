#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H08R6 User Interface ***************************************************************************/
/**************************************************************************************************/
struct H08R6_DistanceAverage
{
    BOSStatus status;
    int16_t average = 0;
};

class H08R6
{
public:
    static std::promise<H08R6_DistanceAverage> averagePromise;

    static H08R6_DistanceAverage RequestDistanceAverage(uint8_t moduleID);

};