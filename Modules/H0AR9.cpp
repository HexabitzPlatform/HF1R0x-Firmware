#include "H0AR9.h"

std::promise<bool> H0AR9::PIRPromise;
std::mutex H0AR9::PIRMutex;

std::promise<uint16_t> H0AR9::redColorPromise;
std::promise<uint16_t> H0AR9::greenColorPromise;
std::promise<uint16_t> H0AR9::blueColorPromise;
std::mutex H0AR9::ColorMutex;

std::promise<uint16_t> H0AR9::DistancePromise;
std::mutex H0AR9::DistanceMutex;

std::promise<float> H0AR9::TempPromise;
std::mutex H0AR9::TempMutex;

std::promise<float> H0AR9::HumidityPromise;
std::mutex H0AR9::HumidityMutex;

/**************************************************************************************************/
/* H0AR9 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
static bool RequestPIR(uint8_t moduleID)
{
    std::lock_guard<std::mutes> lock(PIRMutex);

    // Reset promises to ensure no old value remains
    PIRpromise = std::promise<bool>();

    // Get futures to wait for response
    std::future<bool> futurePIR = PIRPromise.get_future();

    // Send request
    BOSStatus status = Messaging::SendDataRequestToModule(moduleID, BOSMessageCode::CODE_H0BR4_SAMPLE_ACC);
    if (status != BOSStatus::BOS_OK)
    {
        std::cerr << "Failed to send PIR Sample request\n";
        return static_cast<BOSStatus>(status);
    }

    if (futurePIR.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready)
    {
        std::cerr << "Timeout while waiting for PIR Sample\n";
        return {0};
    }

    return {futurePIR.get()};
}
/**************************************************************************************************/
static Color RequestColor(uint8_t moduleID)
{
}
/**************************************************************************************************/
static uint16_t RequestDistance(uint8_t moduleID)
{
}
/**************************************************************************************************/
static float RequestTemp(uint8_t moduleID)
{
}
/**************************************************************************************************/
static float RequestHumidity(uint8_t moduleID)
{
}
/**************************************************************************************************/