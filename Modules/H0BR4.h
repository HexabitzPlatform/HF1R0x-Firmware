#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H0BR4 User Interface ***************************************************************************/
/**************************************************************************************************/
struct AccResult
{
    BOSStatus status;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct GyroResult
{
    BOSStatus status;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct MagResult
{
    BOSStatus status;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct IMU_TempResult
{
    BOSStatus status;
    float temp = 0.0f;
};

class H0BR4
{
public:
    static std::promise<AccResult> AccPromise;
    static std::promise<GyroResult> GyroPromise;
    static std::promise<MagResult> MagPromise;
    static std::promise<IMU_TempResult> TempPromise;

    static AccResult RequestAcc(uint8_t moduleID);
    static GyroResult RequestGyro(uint8_t moduleID);
    static MagResult RequestMag(uint8_t moduleID);
    static IMU_TempResult RequestTemp(uint8_t moduleID);
};
