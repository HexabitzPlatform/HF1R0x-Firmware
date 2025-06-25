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
CellVoltageResult H05R0::RequestVoltage(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    VoltagePromise = std::promise<CellVoltageResult>();

    // Get futures to wait for response
    std::future<CellVoltageResult> future = VoltagePromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H05R0_CELL_VOLTAGE);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Cell Voltage request\n";
        return {status, 0.0f};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (future.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return future.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Cell Voltage\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}
/**************************************************************************************************/
CellCurrentResult H05R0::RequestCurrent(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    CurrentPromise = std::promise<CellCurrentResult>();

    // Get futures to wait for response
    std::future<CellCurrentResult> future = CurrentPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H05R0_CELL_CURRENT);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Cell Current request\n";
        return {status, 0.0f};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (future.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return future.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Cell Current\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}
/**************************************************************************************************/
CellPowerResult H05R0::RequestPower(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    powerPromise = std::promise<CellPowerResult>();

    // Get futures to wait for response
    std::future<CellPowerResult> future = powerPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H05R0_CELL_POWER);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Cell power request\n";
        return {status, 0.0f};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (future.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return future.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Cell power\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}
/**************************************************************************************************/
CellTempResult H05R0::RequestTemp(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    TempPromise = std::promise<CellTempResult>();

    // Get futures to wait for response
    std::future<CellTempResult> future = TempPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H05R0_CELL_TEMPERATURE);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Cell Temerature request\n";
        return {status, 0.0f};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (future.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return future.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Cell Temperature\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}
/**************************************************************************************************/
CellCapacityResult H05R0::RequestCapacity(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    CapacityPromise = std::promise<CellCapacityResult>();

    // Get futures to wait for response
    std::future<CellCapacityResult> future = CapacityPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H05R0_CELL_CAPACITY);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Cell Capacity request\n";
        return {status, 0.0f};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (future.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return future.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Cell Capacity\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}
/**************************************************************************************************/
SOCResult H05R0::RequestSOC(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    SOCPromise = std::promise<SOCResult>();

    // Get futures to wait for response
    std::future<SOCResult> future = SOCPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H05R0_STATE_OF_CHARGE);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send SOC request\n";
        return {status, 0};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (future.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return future.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for SOC\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0};
}
/**************************************************************************************************/
CellAgeResult H05R0::RequestAge(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    AgePromise = std::promise<CellAgeResult>();

    // Get futures to wait for response
    std::future<CellAgeResult> future = AgePromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H05R0_CELL_AGE);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Cell Age request\n";
        return {status, 0};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (future.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return future.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Cell Age\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0};
}
/**************************************************************************************************/
CellCyclesResult H05R0::RequestCycles(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    CyclesPromise = std::promise<CellCyclesResult>();

    // Get futures to wait for response
    std::future<CellCyclesResult> future = CyclesPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H05R0_CELL_CYCLES);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Cell Cycles request\n";
        return {status, 0};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (future.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return future.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Cell Cycles\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0};
}
/**************************************************************************************************/
