#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H09R9 User Interface ***************************************************************************/
/**************************************************************************************************/
struct H09R9_Temp
{
    BOSStatus status;
    float temp = 0.0f;
};

class H09R9
{
public:
    static std::promise<H09R9_Temp> TempPromise;

    static H09R9_Temp RequestTemp(uint8_t moduleID);

};