#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H0AR9 User Interface ***************************************************************************/
/**************************************************************************************************/
struct Color
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

class H0AR9
{
public:
    static std::promise<bool> PIRPromise;
    static std::mutex PIRMutex;

    static std::promise<uint16_t> redColorPromise;
    static std::promise<uint16_t> greenColorPromise;
    static std::promise<uint16_t> blueColorPromise;
    static std::mutex ColorMutex;

    static std::promise<uint16_t> DistancePromise;
    static std::mutex DistanceMutex;

    static std::promise<float> TempPromise;
    static std::mutex TempMutex;

    static std::promise<float> HumidityPromise;
    static std::mutex HumidityMutex;

    /* User Functions */
    static bool RequestPIR(uint8_t moduleID);
    static Color RequestColor(uint8_t moduleID);
    static uint16_t RequestDistance(uint8_t moduleID);
    static float RequestTemp(uint8_t moduleID);
    static float RequestHumidity(uint8_t moduleID);
};