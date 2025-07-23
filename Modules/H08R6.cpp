#include <H08R6.h>

std::promise<H08R6_DistanceAverage> H08R6::averagePromise;

/**************************************************************************************************/
/* H08R7 User Interface ***************************************************************************/
/**************************************************************************************************/
H08R6_DistanceAverage H08R6::RequestDistanceAverage(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    averagePromise = std::promise<H08R6_DistanceAverage>();

    // Get futures to wait for response
    std::future<H08R6_DistanceAverage> future = averagePromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H08R6_SAMPLE_DISTANCE_AVRG);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Distance Average request\n";
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

    std::cerr << "Timeout while waiting for Distance Average\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0};
   
}
/**************************************************************************************************/
