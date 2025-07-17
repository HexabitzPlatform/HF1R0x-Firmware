#include <H08R7.h>

std::promise<H08R7_TOF> H08R7::distancePromise;

/**************************************************************************************************/
/* H08R7 User Interface ***************************************************************************/
/**************************************************************************************************/
H08R7_TOF H08R7::RequestTOF(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    distancePromise = std::promise<H08R7_TOF>();

    // Get futures to wait for response
    std::future<H08R7_TOF> future = distancePromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H08R7_SAMPLE_DISTANCE);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send distance request\n";
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

    std::cerr << "Timeout while waiting for distance\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0};
   
}
/**************************************************************************************************/
