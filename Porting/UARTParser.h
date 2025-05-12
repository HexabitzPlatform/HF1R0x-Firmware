// Porting/UARTParser.h
#pragma once
#include <functional>
#include <vector>
#include <cstdint>

class UARTParser
{
public:
    using Callback = std::function<void(const std::vector<uint8_t> &)>;

    void feed(uint8_t byte);
    void onMessageReceived(Callback cb);

private:
    enum class State
    {
        WaitForH,
        WaitForZ,
        WaitForLength,
        Collecting
    } state = State::WaitForH;

    std::vector<uint8_t> buffer;
    size_t expectedLength = 0;
    Callback callback;
};

uint8_t calculateCRC8(const std::vector<uint8_t>& data);
