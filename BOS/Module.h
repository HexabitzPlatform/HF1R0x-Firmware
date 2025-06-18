#pragma once

#include "BOS.h"

#include <future>
#include <mutex>

/**************************************************************************************************/
/* H0BR4 User Interface ***************************************************************************/
/**************************************************************************************************/
struct AccData
{
    float x;
    float y;
    float z;
};

struct GyroData
{
    float x;
    float y;
    float z;
};

struct MagData
{
    float x;
    float y;
    float z;
};

struct Temp
{
    float value;
};

class H0BR4
{
public:
    static std::promise<float> xAccPromise;
    static std::promise<float> yAccPromise;
    static std::promise<float> zAccPromise;
    static std::mutex AccMutex;

    static std::promise<float> xGyroPromise;
    static std::promise<float> yGyroPromise;
    static std::promise<float> zGyroPromise;
    static std::mutex GyroMutex;

    static std::promise<float> xMagPromise;
    static std::promise<float> yMagPromise;
    static std::promise<float> zMagPromise;
    static std::mutex MagMutex;

    static std::promise<float> TempPromise;
    static std::mutex TempMutex;

public:
    // std::tuple<float, float, float> RequestAcc(uint8_t moduleID);
    // GyroData RequestGyro(uint8_t moduleID, float xGyro, float yGyro, float zGyro);
    // MagData RequestMag(uint8_t moduleID, float xMag, float yMag, float zMag);

    AccData RequestAcc(uint8_t moduleID);
    GyroData RequestGyro(uint8_t moduleID);
    MagData RequestMag(uint8_t moduleID);
    Temp RequestTemp(uint8_t moduleID);
};
