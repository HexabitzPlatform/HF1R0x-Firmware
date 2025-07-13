#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H0AR9 User Interface ***************************************************************************/
/**************************************************************************************************/
struct H0AR9_PIR
{
    BOSStatus status;
    bool pir = false;
};

struct H0AR9_Color
{
    BOSStatus status;
    uint16_t red = 0;
    uint16_t green = 0;
    uint16_t blue = 0;
};

struct H0AR9_Distance
{
    BOSStatus status;
    uint16_t distance = 0;
};

struct H0AR9_Temp
{
    BOSStatus status;
    float temp = 0.0f;
};

struct H0AR9_Humidity
{
    BOSStatus status;
    float humidity = 0.0f;
};

class H0AR9
{
public:
    static std::promise<H0AR9_PIR> PIRPromise;
    static std::promise<H0AR9_Color> colorPromise;
    static std::promise<H0AR9_Distance> DistancePromise;
    static std::promise<H0AR9_Temp> TempPromise;
    static std::promise<H0AR9_Humidity> HumidityPromise;

    /* User Functions */
    static H0AR9_PIR RequestPIR(uint8_t moduleID);
    static H0AR9_Color RequestColor(uint8_t moduleID);
    static H0AR9_Distance RequestDistance(uint8_t moduleID);
    static H0AR9_Temp RequestTemp(uint8_t moduleID);
    static H0AR9_Humidity RequestHumidity(uint8_t moduleID);
};