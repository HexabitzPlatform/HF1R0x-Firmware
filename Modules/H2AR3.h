#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H2AR3 User Interface ***************************************************************************/
/**************************************************************************************************/
struct H2AR3_Voltage
{
    BOSStatus status;
    float volt = 0.0f;
};

struct H2AR3_Current
{
    BOSStatus status;
    float current = 0.0f;
};

class H2AR3
{
public:
    static std::promise<H2AR3_Voltage> VoltagePromise;
    static std::promise<H2AR3_Current> CurrentPromise;

    static H2AR3_Voltage RequestVoltage(uint8_t moduleID);
    static H2AR3_Current RequestCurrent(uint8_t moduleID);

};