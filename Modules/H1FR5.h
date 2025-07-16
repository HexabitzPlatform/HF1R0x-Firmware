#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H1FR5 User Interface ***************************************************************************/
/**************************************************************************************************/
struct H1FR5_GetHeight
{
    BOSStatus status;
    float height = 0.0f;
};

struct H1FR5_GetSpeed
{
    BOSStatus status;
    float speedinch = 0.0f;
    float speedkm = 0.0f;
};

struct H1FR5_GetUTC
{
    BOSStatus status;
    uint8_t hours = 0;
    uint8_t min = 0;
    uint8_t sec = 0;
};

struct H1FR5_GetPosition
{
    BOSStatus status;
    float longdegree = 0.0f;
    float latdegree = 0.0f;
    char longindicator = 0;
    char latindicator = 0;
};

class H1FR5
{
public:
    static std::promise<H1FR5_GetHeight> HeightPromise;
    static std::promise<H1FR5_GetSpeed> SpeedPromise;
    static std::promise<H1FR5_GetUTC> UTCPromise;
    static std::promise<H1FR5_GetPosition> PositionPromise;

    static H1FR5_GetHeight RequestHeight(uint8_t moduleID);
    static H1FR5_GetSpeed RequestSpeed(uint8_t moduleID);
    static H1FR5_GetUTC RequestUTC(uint8_t moduleID);
    static H1FR5_GetPosition RequestPosition(uint8_t moduleID);

};