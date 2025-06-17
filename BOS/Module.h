#include "BOS.h"

#include <future>
#include <mutex>

class H0BR4
{
private:
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
    std::tuple<float, float, float> RequestAcc(uint8_t moduleID);
    void RequestGyro(uint8_t moduleID, float xGyro, float yGyro, float zGyro);
    void RequestMag(uint8_t moduleID, float xMag, float yMag, float zMag);
    void RequestTemp(uint8_t moduleID);
}
