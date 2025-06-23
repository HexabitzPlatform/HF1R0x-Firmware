#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H0AR9 User Interface ***************************************************************************/
/**************************************************************************************************/
struct PIRResult
{
    BOSStatus status;
    bool pir = false;
};

struct ColorResult
{
    BOSStatus status;

    uint16_t red = 0;
    uint16_t green = 0;
    uint16_t blue = 0;
};

struct DistanceResult
{
    BOSStatus status;
    uint16_t distance = 0;
};

struct TempResult
{
    BOSStatus status;
    float temp = 0.0f;
};

struct HumidityResult
{
    BOSStatus status;
    float humidity = 0.0f;
};

class H0AR9
{
public:
    static std::promise<bool> PIRPromise;

    static std::promise<ColorResult> colorPromise;

    static std::promise<uint16_t> DistancePromise;

    static std::promise<float> TempPromise;

    static std::promise<float> HumidityPromise;

    /* User Functions */
    static PIRResult RequestPIR(uint8_t moduleID);
    static ColorResult RequestColor(uint8_t moduleID);
    static DistanceResult RequestDistance(uint8_t moduleID);
    static TempResult RequestTemp(uint8_t moduleID);
    static HumidityResult RequestHumidity(uint8_t moduleID);
};