#include <h05R0.h>

std::promise<H05R0_CellVoltage> H05R0::VoltagePromise;
std::promise<H05R0_CellCurrent> H05R0::CurrentPromise;
std::promise<H05R0_CellPower> H05R0::powerPromise;
std::promise<H05R0_CellTemp> H05R0::TempPromise;
std::promise<H05R0_CellCapacity> H05R0::CapacityPromise;
std::promise<H05R0_SOC> H05R0::SOCPromise;
std::promise<H05R0_CellAge> H05R0::AgePromise;
std::promise<H05R0_CellCycles> H05R0::CyclesPromise;

/**************************************************************************************************/
/* H05R0 User Interface ***************************************************************************/
/**************************************************************************************************/
H05R0_CellVoltage H05R0::RequestVoltage(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 500;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    VoltagePromise = std::promise<H05R0_CellVoltage>();

    // Get futures to wait for response
    std::future<H05R0_CellVoltage> future = VoltagePromise.get_future();

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
H05R0_CellCurrent H05R0::RequestCurrent(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    CurrentPromise = std::promise<H05R0_CellCurrent>();

    // Get futures to wait for response
    std::future<H05R0_CellCurrent> future = CurrentPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H05R0_CELL_CURRENT);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Cell Current request\n";
        return {status, Batterystate::error, 0.0f};
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
    return {BOSStatus::BOS_ERR_TIMEOUT, Batterystate::error, 0.0f};
}
/**************************************************************************************************/
H05R0_CellPower H05R0::RequestPower(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    powerPromise = std::promise<H05R0_CellPower>();

    // Get futures to wait for response
    std::future<H05R0_CellPower> future = powerPromise.get_future();

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
H05R0_CellTemp H05R0::RequestTemp(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    TempPromise = std::promise<H05R0_CellTemp>();

    // Get futures to wait for response
    std::future<H05R0_CellTemp> future = TempPromise.get_future();

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
H05R0_CellCapacity H05R0::RequestCapacity(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    CapacityPromise = std::promise<H05R0_CellCapacity>();

    // Get futures to wait for response
    std::future<H05R0_CellCapacity> future = CapacityPromise.get_future();

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
/***************************************************************************************************/
 H05R0_SOC H05R0::RequestSOC(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 300;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    SOCPromise = std::promise<H05R0_SOC>();

    // Get futures to wait for response
    std::future<H05R0_SOC> future = SOCPromise.get_future();

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
H05R0_CellAge H05R0::RequestAge(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    AgePromise = std::promise<H05R0_CellAge>();

    // Get futures to wait for response
    std::future<H05R0_CellAge> future = AgePromise.get_future();

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
H05R0_CellCycles H05R0::RequestCycles(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 500;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    CyclesPromise = std::promise<H05R0_CellCycles>();

    // Get futures to wait for response
    std::future<H05R0_CellCycles> future = CyclesPromise.get_future();

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
