
#include "BOS.h"

int main()
{

    // Initialize GPIO PIN on Raspberry
    led.blink(LEDConfig::INITIAL_BLINK_TIMES, LEDConfig::BLINK_DELAY_MS);

    // Initialize UART on Raspberry Pi UART port (TX=GPIO14, RX=GPIO15, baudrate=115200)
    Porting::initUART(UARTConfig::TX_PIN, UARTConfig::RX_PIN, UARTConfig::BAUDRATE);

    std::cout << "UART initialized. Listening for BOS messages ...\n";

    Module_MessageParser bosParser;
    // BOS_MessageParser bosParser;
    UARTParser uartParser;

    // Connect UARTParser to BOS message parser
    uartParser.onMessageReceived([&bosParser](const std::vector<uint8_t> &payload)
                                 {
                                             std::cout << "\nValid BOS message received. Passing to BOS parser...\n";
                                             bosParser.parseMessage(payload); });

    // Setup UART receive callback to feed bytes into UARTParser
    Porting::setUartReceiveCallback([&uartParser](char byte)
                                    { uartParser.feed(static_cast<uint8_t>(byte)); });

    // Initialize BOS
    // initBOS();

    // led.blink(4 , 100);
    std::vector<uint8_t> Parameters = {2};

    // Messaging::SendMessagetoModule(1, BOSMessageCode::CODE_H01R0_ON, Parameters);
    PIConfig::piID = 4;

    // AccResult acc;
    // GyroResult gyro;
    // MagResult mag;
    // IMU_TempResult temp;

    // ColorResult Color;
    // PIRResult PIR;
    // TempResult Temp;
    // DistanceResult Distance;
    // HumidityResult Humidity;

    CellAgeResult age;
    CellCapacityResult capacity;
    CellCurrentResult current;
    CellCyclesResult cycles;
    CellPowerResult power;
    CellTempResult temp;
    CellVoltageResult voltage;
    SOCResult soc;

    std::vector<uint8_t> hello;

    // for (uint8_t i = 0; i < 80; i++)
    // {
    //     hello.push_back(i);
    // }

    // Keep main thread alive indefinitely to allow background UART reading thread to run
    while (true)
    {
        age = H05R0::RequestAge(1);
        // Messaging::SendLargMessagetoModule(1, BOSMessageCode::CODE_RAW_DATA, hello);

        /**************************************************************************************************/
        // acc = H0BR4::RequestAcc(2);
        // if (acc.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[ACC] X = " << acc.x
        //               << "\n Y = " << acc.y
        //               << "\n Z = " << acc.z
        //               << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // gyro = H0BR4::RequestGyro(2);
        // if (gyro.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Gyro] X = " << gyro.x
        //               << "\n Y = " << gyro.y
        //               << "\n Z = " << gyro.z
        //               << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // mag = H0BR4::RequestMag(2);
        // if (mag.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Mag] X = " << mag.x
        //               << "\n Y = " << mag.y
        //               << "\n Z = " << mag.z
        //               << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // temp = H0BR4::RequestTemp(2);
        // if (temp.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Temp] = " << temp.temp
        //               << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        /**************************************************************************************************/

        // Color = H0AR9::RequestColor(1);
        // if (Color.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Color] R: " << Colo        // for (uint8_t i = 0; i < 513; i++)
        // {
        //     hello.push_back(i);
        // }r.red
        //               << " G: " << Color.green
        //               << " B: " << Color.blue << "\n";
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // PIR = H0AR9::RequestPIR(1);
        // if (PIR.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[PIR]: " << PIR.pir
        //               << "\n";
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Temp = H0AR9::RequestTemp(1);
        // if (Temp.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Temperature]: " << Temp.temp
        //               << "\n";
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Distance = H0AR9::RequestDistance(1);
        // if (Distance.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Distance]: " << Distance.distance
        //               << "\n";
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Humidity = H0AR9::RequestHumidity(1);
        // if (Humidity.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Humidity]: " << Humidity.humidity
        //               << "\n";
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // led.blink(4 , 100);
    }

    return 0;
}
