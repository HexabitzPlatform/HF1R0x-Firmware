#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H0BR4 User Interface ***************************************************************************/
/**************************************************************************************************/
struct H0BR4_Acc
{
    BOSStatus status;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct H0BR4_Gyro
{
    BOSStatus status;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct H0BR4_Mag
{
    BOSStatus status;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct H0BR4_Temp
{
    BOSStatus status;
    float temp = 0.0f;
};

class H0BR4
{
public:
    static std::promise<H0BR4_Acc> AccPromise;
    static std::promise<H0BR4_Gyro> GyroPromise;
    static std::promise<H0BR4_Mag> MagPromise;
    static std::promise<H0BR4_Temp> TempPromise;

    static H0BR4_Acc RequestAcc(uint8_t moduleID);
    static H0BR4_Gyro RequestGyro(uint8_t moduleID);
    static H0BR4_Mag RequestMag(uint8_t moduleID);
    static H0BR4_Temp RequestTemp(uint8_t moduleID);
};
