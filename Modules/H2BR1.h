#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H2BR1 User Interface ***************************************************************************/
/**************************************************************************************************/
struct H2BR1_HR
{
    BOSStatus status;
    uint8_t heartRate = 0;
};

struct H2BR1_SPO2
{
    BOSStatus status;
    uint8_t SPO2 = 0;
};

class H2BR1
{
public:
    static std::promise<H2BR1_HR> HRPromise;
    static std::promise<H2BR1_SPO2> SPO2Promise;

    static H2BR1_HR RequestHR(uint8_t moduleID);
    static H2BR1_SPO2 RequestSPO2(uint8_t moduleID);
};