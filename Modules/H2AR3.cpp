#include <H2AR3.h>

std::promise<H2AR3_Voltage> H2AR3::VoltagePromise;
std::promise<H2AR3_Current> H2AR3::CurrentPromise;

/**************************************************************************************************/
/* H2AR3 User Interface ***************************************************************************/
/**************************************************************************************************/
H2AR3_Voltage H2AR3::RequestVoltage(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    VoltagePromise = std::promise<H2AR3_Voltage>();

    // Get futures to wait for response
    std::future<H2AR3_Voltage> future = VoltagePromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H2AR3_SAMPLE_VOLT);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Voltage request\n";
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

    std::cerr << "Timeout while waiting for Voltage\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
   
}
/**************************************************************************************************/
H2AR3_Current H2AR3::RequestCurrent(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    CurrentPromise = std::promise<H2AR3_Current>();

    // Get futures to wait for response
    std::future<H2AR3_Current> future = CurrentPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H2AR3_SAMPLE_CURRENT);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Current request\n";
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

    std::cerr << "Timeout while waiting for Current\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
   
}
/**************************************************************************************************/
