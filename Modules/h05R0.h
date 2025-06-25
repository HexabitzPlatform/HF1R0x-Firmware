#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H05R0 User Interface ***************************************************************************/
/**************************************************************************************************/
struct CellVoltageResult
{
    BOSStatus status;
    float voltage = 0.0f;
};

struct CellCurrentResult
{
    BOSStatus status;
    float current = 0.0f;
};

struct CellPowerResult
{
    BOSStatus status;
    float power = 0.0f;
};

struct CellTempResult
{
    BOSStatus status;
    float temp = 0.0f;
};

struct CellCapacityResult
{
    BOSStatus status;
    float capacity = 0.0f;
};

struct SOCResult
{
    BOSStatus status;
    uint8_t SOC = 0;
};

struct CellAgeResult
{
    BOSStatus status;
    uint8_t age = 0;
};

struct CellCyclesResult
{
    BOSStatus status;
    uint16_t cycles = 0;
};

class H05R0
{
public:
    static std::promise<CellVoltageResult> VoltagePromise;
    static std::promise<CellCurrentResult> CurrentPromise;
    static std::promise<CellPowerResult> powerPromise;
    static std::promise<CellTempResult> TempPromise;
    static std::promise<CellCapacityResult> CapacityPromise;
    static std::promise<SOCResult> SOCPromise;
    static std::promise<CellAgeResult> AgePromise;
    static std::promise<CellCyclesResult> CyclesPromise;

    static CellVoltageResult RequestVoltage(uint8_t moduleID);
    static CellCurrentResult RequestCurrent(uint8_t moduleID);
    static CellPowerResult RequestPower(uint8_t moduleID);
    static CellTempResult RequestTemp(uint8_t moduleID);
    static CellCapacityResult RequestCapacity(uint8_t moduleID);
    static SOCResult RequestSOC(uint8_t moduleID);
    static CellAgeResult RequestAge(uint8_t moduleID);
    static CellCyclesResult RequestCycles(uint8_t moduleID);
};