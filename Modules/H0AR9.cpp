#include "H0AR9.h"

std::promise<H0AR9_PIR> H0AR9::PIRPromise;

std::promise<H0AR9_Color> H0AR9::colorPromise;

std::promise<H0AR9_Distance> H0AR9::DistancePromise;

std::promise<H0AR9_Temp> H0AR9::TempPromise;

std::promise<H0AR9_Humidity> H0AR9::HumidityPromise;

/**************************************************************************************************/
/* H0AR9 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
H0AR9_PIR H0AR9::RequestPIR(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 500;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    PIRPromise = std::promise<H0AR9_PIR>();

    // Get futures to wait for response
    std::future<H0AR9_PIR> futurePIR = PIRPromise.get_future();

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
        if (futurePIR.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready)
        {
            return futurePIR.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for PIR Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, false};
}
/**************************************************************************************************/
H0AR9_Color H0AR9::RequestColor(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    colorPromise = std::promise<H0AR9_Color>();

    // Get futures to wait for response
    std::future<H0AR9_Color> future = colorPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0AR9_SAMPLE_COLOR);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Color Sample request\n";
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

    std::cerr << "Timeout while waiting for Color Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0, 0, 0};
}
/**************************************************************************************************/
H0AR9_Distance H0AR9::RequestDistance(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 1000;
    uint16_t step = 5;

    // Reset promises to ensure no old value remains
    DistancePromise = std::promise<H0AR9_Distance>();

    // Get futures to wait for response
    std::future<H0AR9_Distance> futureDistance = DistancePromise.get_future();

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
        if (futureDistance.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready)
        {
            return futureDistance.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Distance Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0};
}
/**************************************************************************************************/
H0AR9_Temp H0AR9::RequestTemp(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    TempPromise = std::promise<H0AR9_Temp>();

    // Get futures to wait for response
    std::future<H0AR9_Temp> futureTemp = TempPromise.get_future();

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
            return futureTemp.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Temperature Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}
/**************************************************************************************************/
H0AR9_Humidity H0AR9::RequestHumidity(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 200;
    uint16_t step = 5;

    // Reset promises to ensure no old value remains
    HumidityPromise = std::promise<H0AR9_Humidity>();

    // Get futures to wait for response
    std::future<H0AR9_Humidity> futureHumidity = HumidityPromise.get_future();

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
            return futureHumidity.get();
        }
        wait += step;
    }

    std::cerr << "Timeout while waiting for Humidity Sample\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}
/**************************************************************************************************/