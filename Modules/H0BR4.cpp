#include "H0BR4.h"

std::promise<float> H0BR4::xAccPromise;
std::promise<float> H0BR4::yAccPromise;
std::promise<float> H0BR4::zAccPromise;
std::mutex H0BR4::AccMutex;

std::promise<float> H0BR4::xGyroPromise;
std::promise<float> H0BR4::yGyroPromise;
std::promise<float> H0BR4::zGyroPromise;
std::mutex H0BR4::GyroMutex;

std::promise<float> H0BR4::xMagPromise;
std::promise<float> H0BR4::yMagPromise;
std::promise<float> H0BR4::zMagPromise;
std::mutex H0BR4::MagMutex;

std::promise<float> H0BR4::TempPromise;
std::mutex H0BR4::TempMutex;

/**************************************************************************************************/
/* H0BR4 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
// std::tuple<float, float, float> H0BR4::RequestAcc(uint8_t moduleID)
AccData H0BR4::RequestAcc(uint8_t moduleID)
{
    std::lock_guard<std::mutex> lock(AccMutex);

    // Reset promises to ensure no old value remains
    xAccPromise = std::promise<float>();
    yAccPromise = std::promise<float>();
    zAccPromise = std::promise<float>();

    // Get futures to wait for response
    std::future<float> fx = xAccPromise.get_future();
    std::future<float> fy = yAccPromise.get_future();
    std::future<float> fz = zAccPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0BR4_SAMPLE_ACC);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Accelerometer request\n";
        return {-1.0f, -1.0f, -1.0f};
    }

    // Wait for response (timeout after 20 milliseconds)
    if (fx.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready ||
        fy.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready ||
        fz.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready)
    {
        std::cerr << "Timeout while waiting for Accelerometer\n";
        return {-1.0f, -1.0f, -1.0f};
    }

    return {fx.get(), fy.get(), fz.get()};
}

/**************************************************************************************************/
// void H0BR4::RequestGyro(uint8_t moduleID, float xGyro, float yGyro, float zGyro)
GyroData H0BR4::RequestGyro(uint8_t moduleID)
{
    std::lock_guard<std::mutex> lock(GyroMutex); // Lock in case of concurrent calls

    // Reset promises to ensure no old value remains
    xGyroPromise = std::promise<float>();
    yGyroPromise = std::promise<float>();
    zGyroPromise = std::promise<float>();

    // Get futures to wait for response
    std::future<float> fx = xGyroPromise.get_future();
    std::future<float> fy = yGyroPromise.get_future();
    std::future<float> fz = zGyroPromise.get_future();

    // Send the actual BOS request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0BR4_SAMPLE_GYRO);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Gyroscope request\n";
        return {-1.0f, -1.0f, -1.0f};
    }

    // Wait for response (timeout after 20 milliseconds)
    if (fx.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready ||
        fy.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready ||
        fz.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready)
    {
        std::cerr << "Timeout while waiting for Gyroscope\n";
        return {-1.0f, -1.0f, -1.0f};
    }

    return {fx.get(), fy.get(), fz.get()};
}

// /**************************************************************************************************/
// void H0BR4::RequestMag(uint8_t moduleID, float xMag, float yMag, float zMag)
MagData H0BR4::RequestMag(uint8_t moduleID)
{
    std::lock_guard<std::mutex> lock(MagMutex); // Lock in case of concurrent calls

    // Reset promises to ensure no old value remains
    xMagPromise = std::promise<float>();
    yMagPromise = std::promise<float>();
    zMagPromise = std::promise<float>();

    // Get futures to wait for response
    std::future<float> fx = xMagPromise.get_future();
    std::future<float> fy = yMagPromise.get_future();
    std::future<float> fz = zMagPromise.get_future();

    // Send the actual BOS request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0BR4_SAMPLE_GYRO);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Magnetometer request\n";
        return {-1.0f, -1.0f, -1.0f};
    }

    // Wait for response (timeout after 20 milliseconds)
    if (fx.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready ||
        fy.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready ||
        fz.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready)
    {
        std::cerr << "Timeout while waiting for Magnetometer\n";
        return {-1.0f, -1.0f, -1.0f};
    }

    return {fx.get(), fy.get(), fz.get()};
}

/**************************************************************************************************/
Temp H0BR4::RequestTemp(uint8_t moduleID)
{
    std::lock_guard<std::mutex> lock(TempMutex); // Lock in case of concurrent calls

    // Reset the promise
    TempPromise = std::promise<float>();
    std::future<float> future = TempPromise.get_future();

    // Send the actual BOS request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0BR4_SAMPLE_TEMP);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send Temperature request\n";
        return {-1.0f}; // Error value
    }

    // Wait for the response from parser
    // std::future_status waitStatus = future.wait_for(std::chrono::seconds(2));
    if (future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready)
    {
        std::cerr << "Timeout while waiting for Temperature\n";
        return {-1.0f};
    }

    return {future.get()};
}