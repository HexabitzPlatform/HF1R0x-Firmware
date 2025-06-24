#include <h05R0.h>

std::promise<CellVoltageResult> H05R0::VoltagePromise;
std::promise<CellCurrentResult> H05R0::CurrentPromise;
std::promise<CellPowerResult> H05R0::powerPromise;
std::promise<CellTempResult> H05R0::TempPromise;
std::promise<CellCapacityResult> H05R0::CapacityPromise;
std::promise<SOCResult> H05R0::SOCPromise;
std::promise<CellAgeResult> H05R0::AgePromise;
std::promise<CellCyclesResult> H05R0::CyclesPromise;

/**************************************************************************************************/
/* H05R0 User Interface ***************************************************************************/
/**************************************************************************************************/
CellVoltageResult RequestVoltage(uint8_t moduleID)
{
}
/**************************************************************************************************/
CellCurrentResult RequestCurrent(uint8_t moduleID)
{
}
/**************************************************************************************************/
CellPowerResult RequestPower(uint8_t moduleID)
{
}
/**************************************************************************************************/
CellTempResult RequestTemp(uint8_t moduleID)
{
}
/**************************************************************************************************/
CellCapacityResult RequestCapacity(uint8_t moduleID)
{
}
/**************************************************************************************************/
SOCResult RequestSOC(uint8_t moduleID)
{
}
/**************************************************************************************************/
CellAgeResult RequestAge(uint8_t moduleID)
{
}
/**************************************************************************************************/
CellCyclesResult RequestCycles(uint8_t moduleID)
{
}
/**************************************************************************************************/
