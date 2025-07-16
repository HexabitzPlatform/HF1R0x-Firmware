#include <H1FR5.h>

std::promise<H1FR5_GetHeight> H1FR5::HeightPromise;
std::promise<H1FR5_GetSpeed> H1FR5::SpeedPromise;
std::promise<H1FR5_GetUTC> H1FR5::UTCPromise;
std::promise<H1FR5_GetPosition> H1FR5::PositionPromise;

/**************************************************************************************************/
/* H1FR5 User Interface ***************************************************************************/
/**************************************************************************************************/
H1FR5_GetHeight H1FR5::RequestHeight(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    HeightPromise = std::promise<H1FR5_GetHeight>();

    // Get futures to wait for response
    std::future<H1FR5_GetHeight> future = HeightPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H1FR5_GET_HEIGHT);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Get Height request\n";
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

    std::cerr << "Timeout while waiting for Get Height\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
   
}
/**************************************************************************************************/
H1FR5_GetSpeed H1FR5::RequestSpeed(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    SpeedPromise = std::promise<H1FR5_GetSpeed>();

    // Get futures to wait for response
    std::future<H1FR5_GetSpeed> future = SpeedPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H1FR5_GET_SPEED);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Get Speed request\n";
        return {status, 0.0f, 0.0f};
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

    std::cerr << "Timeout while waiting for Get Speed\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f, 0.0f};
  
}
/**************************************************************************************************/
H1FR5_GetUTC H1FR5::RequestUTC(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    UTCPromise = std::promise<H1FR5_GetUTC>();

    // Get futures to wait for response
    std::future<H1FR5_GetUTC> future = UTCPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H1FR5_GET_UTC);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Get UTC request\n";
        return {status, 0, 0, 0};
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

    std::cerr << "Timeout while waiting for  Get UTC\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0, 0, 0};
   
}
/**************************************************************************************************/
H1FR5_GetPosition H1FR5::RequestPosition(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    PositionPromise = std::promise<H1FR5_GetPosition>();

    // Get futures to wait for response
    std::future<H1FR5_GetPosition> future = PositionPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H1FR5_GET_POSITION);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Get Position request\n";
        return {status, 0.0f, 0.0f, 0, 0};
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

    std::cerr << "Timeout while waiting for Get Position\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f, 0.0f, 0, 0};
   
}
/**************************************************************************************************/
