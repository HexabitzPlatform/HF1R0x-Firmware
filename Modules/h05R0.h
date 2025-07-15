#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H05R0 User Interface ***************************************************************************/
/**************************************************************************************************/
enum class Batterystate : uint8_t
{
    charging = 0,
    discharging = 1,
    error = 2
};

struct H05R0_CellVoltage
{
    BOSStatus status;
    float voltage = 0.0f;
};

struct H05R0_CellCurrent
{
    BOSStatus status;
    Batterystate batteryState;
    float current = 0.0f;
};

struct H05R0_CellPower
{
    BOSStatus status;
    float power = 0.0f;
};

struct H05R0_CellTemp
{
    BOSStatus status;
    float temp = 0.0f;
};

struct H05R0_CellCapacity
{
    BOSStatus status;
    float capacity = 0.0f;
};

struct H05R0_SOC
{
    BOSStatus status;
    uint8_t SOC = 0;
};

struct H05R0_CellAge
{
    BOSStatus status;
    uint8_t age = 0;
};

struct H05R0_CellCycles
{
    BOSStatus status;
    uint16_t cycles = 0;
};

class H05R0
{
public:
    static std::promise<H05R0_CellVoltage> VoltagePromise;
    static std::promise<H05R0_CellCurrent> CurrentPromise;
    static std::promise<H05R0_CellPower> powerPromise;
    static std::promise<H05R0_CellTemp> TempPromise;
    static std::promise<H05R0_CellCapacity> CapacityPromise;
    static std::promise<H05R0_SOC> SOCPromise;
    static std::promise<H05R0_CellAge> AgePromise;
    static std::promise<H05R0_CellCycles> CyclesPromise;

    static H05R0_CellVoltage RequestVoltage(uint8_t moduleID);
    static H05R0_CellCurrent RequestCurrent(uint8_t moduleID);
    static H05R0_CellPower RequestPower(uint8_t moduleID);
    static H05R0_CellTemp RequestTemp(uint8_t moduleID);
    static H05R0_CellCapacity RequestCapacity(uint8_t moduleID);
    static H05R0_SOC RequestSOC(uint8_t moduleID);
    static H05R0_CellAge RequestAge(uint8_t moduleID);
    static H05R0_CellCycles RequestCycles(uint8_t moduleID);
};