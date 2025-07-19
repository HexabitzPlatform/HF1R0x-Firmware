#include <H2BR1.h>

std::promise<H2BR1_HR> H2BR1::HRPromise;
std::promise<H2BR1_SPO2> H2BR1::SPO2Promise;

/**************************************************************************************************/
/* H2BR1 User Interface ***************************************************************************/
/**************************************************************************************************/
H2BR1_HR H2BR1::RequestHR(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    HRPromise = std::promise<H2BR1_HR>();

    // Get futures to wait for response
    std::future<H2BR1_HR> future = HRPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H2BR1_HR_Sample);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send HR request\n";
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

    std::cerr << "Timeout while waiting for HR\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0};
   
}
/**************************************************************************************************/
H2BR1_SPO2 H2BR1::RequestSPO2(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    SPO2Promise = std::promise<H2BR1_SPO2>();

    // Get futures to wait for response
    std::future<H2BR1_SPO2> future = SPO2Promise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H2BR1_SPO2_Sample);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send SPO2 request\n";
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

    std::cerr << "Timeout while waiting for SPO2\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0};
   
}
/**************************************************************************************************/
