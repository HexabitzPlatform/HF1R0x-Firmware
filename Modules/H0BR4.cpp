#include "H0BR4.h"

std::promise<AccResult> H0BR4::AccPromise;
std::promise<GyroResult> H0BR4::GyroPromise;
std::promise<MagResult> H0BR4::MagPromise;
std::promise<IMU_TempResult> H0BR4::TempPromise;

/**************************************************************************************************/
/* H0BR4 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
AccResult H0BR4::RequestAcc(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 5;

    // Reset promises to ensure no old value remains
    AccPromise = std::promise<AccResult>();

    // Get futures to wait for response
    std::future<AccResult> future = AccPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0BR4_SAMPLE_ACC);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Accelerometer request\n";
        return {status, 0.0f, 0.0f, 0.0f};
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

    std::cerr << "Timeout while waiting for Accelerometer\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f, 0.0f, 0.0f};
}

/**************************************************************************************************/
GyroResult H0BR4::RequestGyro(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    GyroPromise = std::promise<GyroResult>();

    // Get futures to wait for response
    std::future<GyroResult> future = GyroPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0BR4_SAMPLE_GYRO);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Gyroscope request\n";
        return {status, 0.0f, 0.0f, 0.0f};
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

    std::cerr << "Timeout while waiting for Gyroscope\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f, 0.0f, 0.0f};
}

/**************************************************************************************************/
MagResult H0BR4::RequestMag(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset promises to ensure no old value remains
    MagPromise = std::promise<MagResult>();

    // Get futures to wait for response
    std::future<MagResult> future = MagPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0BR4_SAMPLE_MAG);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Magnometer request\n";
        return {status, 0.0f, 0.0f, 0.0f};
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

    std::cerr << "Timeout while waiting for Magnometer\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f, 0.0f, 0.0f};
}

/**************************************************************************************************/
IMU_TempResult H0BR4::RequestTemp(uint8_t moduleID)
{
    uint16_t wait = 0;
    uint16_t timeout = 100;
    uint16_t step = 2;

    // Reset the promise
    TempPromise = std::promise<IMU_TempResult>();

    // Get futures to wait for response
    std::future<IMU_TempResult> future = TempPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0BR4_SAMPLE_TEMP);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Temperature request\n";
        return {status, 0.0f}; // Error value
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

    std::cerr << "Timeout while waiting for Temperature\n";
    return {BOSStatus::BOS_ERR_TIMEOUT, 0.0f};
}