#include <H2BR0.h>

std::promise<H2BR0_ECG> H2BR0::ECGPromise;
std::promise<H2BR0_EOG> H2BR0::EOGPromise;
std::promise<H2BR0_EEG> H2BR0::EEGPromise;
std::promise<H2BR0_EMG> H2BR0::EMGPromise;

/**************************************************************************************************/
/* H2BR0 User Interface ***************************************************************************/
/**************************************************************************************************/
H2BR0_ECG H2BR0::RequestECG(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    ECGPromise = std::promise<H2BR0_ECG>();

    // Get futures to wait for response
    std::future<H2BR0_ECG> future = ECGPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H2BR0_ECG_Sample);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send ECG request\n";
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

    std::cerr << "Timeout while waiting for ECG\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f, 0.0f};
   
}
/**************************************************************************************************/
H2BR0_EOG H2BR0::RequestEOG(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    EOGPromise = std::promise<H2BR0_EOG>();

    // Get futures to wait for response
    std::future<H2BR0_EOG> future = EOGPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H2BR0_EOG_Sample);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send EOG request\n";
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

    std::cerr << "Timeout while waiting for EOG\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f, 0.0f};
   
}
/**************************************************************************************************/
H2BR0_EEG H2BR0::RequestEEG(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    EEGPromise = std::promise<H2BR0_EEG>();

    // Get futures to wait for response
    std::future<H2BR0_EEG> future = EEGPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H2BR0_EEG_Sample);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send EEG request\n";
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

    std::cerr << "Timeout while waiting for EEG\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f, 0.0f};
   
}
/**************************************************************************************************/
H2BR0_EMG H2BR0::RequestEMG(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    EMGPromise = std::promise<H2BR0_EMG>();

    // Get futures to wait for response
    std::future<H2BR0_EMG> future = EMGPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H2BR0_EMG_Sample);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send EMG request\n";
        return {status, 0.0f, 0.0f, 0.0f, 0.0f};
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

    std::cerr << "Timeout while waiting for EMG\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f, 0.0f, 0.0f, 0.0f};
   
}
/**************************************************************************************************/
