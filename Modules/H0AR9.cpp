#include "H0AR9.h"

std::promise<bool> H0AR9::PIRPromise;

std::promise<ColorResult> H0AR9::colorPromise;

std::promise<uint16_t> H0AR9::DistancePromise;

std::promise<float> H0AR9::TempPromise;

std::promise<float> H0AR9::HumidityPromise;

/**************************************************************************************************/
/* H0AR9 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
PIRResult H0AR9::RequestPIR(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 20;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    PIRPromise = std::promise<bool>();

    // Get futures to wait for response
    std::future<bool> futurePIR = PIRPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0AR9_SAMPLE_PIR);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send PIR Sample request\n";
        return {status, false};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (futurePIR.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return {BOSStatus::BOS_OK, futurePIR.get()};
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for PIR Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, false};
}
/**************************************************************************************************/
ColorResult H0AR9::RequestColor(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 20;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    colorPromise = std::promise<ColorResult>();

    // Get futures to wait for response
    std::future<ColorResult> future = colorPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0AR9_SAMPLE_COLOR);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send PIR Sample request\n";
        return {status, 0, 0, 0};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (future.wait_for(std::chrono::milliseconds(step)) != std::future_status::ready)
        {
            return future.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for PIR Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0, 0, 0};
}
/**************************************************************************************************/
DistanceResult H0AR9::RequestDistance(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 20;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    DistancePromise = std::promise<uint16_t>();

    // Get futures to wait for response
    std::future<uint16_t> futureDistance = DistancePromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0AR9_SAMPLE_DISTANCE);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Distance Sample request\n";
        return {status, 0};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (futureDistance.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return {BOSStatus::BOS_OK, futureDistance.get()};
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Distance Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0};
}
/**************************************************************************************************/
TempResult H0AR9::RequestTemp(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 20;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    TempPromise = std::promise<float>();

    // Get futures to wait for response
    std::future<float> futureTemp = TempPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0AR9_SAMPLE_TEMP);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Temperature Sample request\n";
        return {status, 0.0f};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (futureTemp.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return {BOSStatus::BOS_OK, futureTemp.get()};
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Temperature Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}
/**************************************************************************************************/
HumidityResult H0AR9::RequestHumidity(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 20;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    HumidityPromise = std::promise<float>();

    // Get futures to wait for response
    std::future<float> futureHumidity = HumidityPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0AR9_SAMPLE_HUMIDITY);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Humidity Sample request\n";
        return {status, 0.0f};
    }

    // Wait for response
    while (wait < timeout)
    {
        if (futureHumidity.wait_for(std::chrono::milliseconds(step)) == std::future_status::ready)
        {
            return {BOSStatus::BOS_OK, futureHumidity.get()};
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Humidity Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}
/**************************************************************************************************/