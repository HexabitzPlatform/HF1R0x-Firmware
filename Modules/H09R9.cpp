#include <H09R9.h>

std::promise<H09R9_Temp> H09R9::TempPromise;

/**************************************************************************************************/
/* H09R9 User Interface ***************************************************************************/
/**************************************************************************************************/
H09R9_Temp H09R9::RequestTemp(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 300;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    TempPromise = std::promise<H09R9_Temp>();

    // Get futures to wait for response
    std::future<H09R9_Temp> future = TempPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H09R9_SAMPLE_TEMP);
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
