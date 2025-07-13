
#include "BOS.h"

int main()
{
    // Initialize GPIO PIN on Raspberry
    led.blink(LEDConfig::INITIAL_BLINK_TIMES, LEDConfig::BLINK_DELAY_MS);

    // Initialize UART on Raspberry Pi UART port
    Porting::initUART(UARTConfig::TX_PIN, UARTConfig::RX_PIN, UARTConfig::BAUDRATE);

    std::cout << "UART initialized. Listening for BOS messages ...\n";

    Module_MessageParser bosParser;
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

    // H0BR4_Acc acc;
    // H0BR4_Gyro gyro;
    // H0BR4_Mag mag;
    // H0BR4_Temp temp;

    H0AR9_Color Color;
    H0AR9_PIR PIR;
    H0AR9_Temp Temp;
    H0AR9_Distance Distance;
    H0AR9_Humidity Humidity;

    // CellAgeResult age;
    // CellCapacityResult capacity;
    // CellCurrentResult current;
    // CellCyclesResult cycles;
    // CellPowerResult power;
    // CellH0AR9_Temp temp;
    // CellVoltageResult voltage;
    // SOCResult soc;

    /* RPI ID using explore feature*/
    PIConfig::piID = 2;

    // Keep main thread alive indefinitely to allow background UART reading thread to run
    while (true)
    {

        /**************************************************************************************************/
        // age = H05R0::RequestAge(1);
        // if (age.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Age: " << age.age << std::endl;
        // }

        // capacity = H05R0::RequestCapacity(1);
        // if (capacity.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Capacity: " << capacity.capacity << std::endl;
        // }

        // current = H05R0::RequestCurrent(1);
        // if (current.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Current: " << current.current << std::endl;
        // }

        // cycles = H05R0::RequestCycles(1);
        // if (cycles.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Cycles: " << cycles.cycles << std::endl;
        // }

        // power = H05R0::RequestPower(1);
        // if (power.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Power: " << power.power << std::endl;
        // }

        // temp = H05R0::RequestTemp(1);
        // if (temp.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Temerature: " << temp.temp << std::endl;
        // }

        // voltage = H05R0::RequestVoltage(1);
        // if (voltage.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Voltage: " << voltage.voltage << std::endl;
        // }

        // soc = H05R0::RequestSOC(1);
        // if (soc.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery SOC: " << soc.SOC << std::endl;
        // }

        /**************************************************************************************************/
        // acc = H0BR4::RequestAcc(1);
        // if (acc.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[ACC] \n X = " << acc.x
        //               << "\n Y = " << acc.y
        //               << "\n Z = " << acc.z
        //               << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // gyro = H0BR4::RequestGyro(1);
        // if (gyro.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Gyro] \n X = " << gyro.x
        //               << "\n Y = " << gyro.y
        //               << "\n Z = " << gyro.z
        //               << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // mag = H0BR4::RequestMag(1);
        // if (mag.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Mag] \n X = " << mag.x
        //               << "\n Y = " << mag.y
        //               << "\n Z = " << mag.z
        //               << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // temp = H0BR4::RequestTemp(1);
        // if (temp.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Temp] = \n temp = "
        //               << temp.temp
        //               << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        /**************************************************************************************************/

        Color = H0AR9::RequestColor(1);
        if (Color.status == BOSStatus::BOS_OK)
        {
            std::cout << "[Color] \n red = " << Color.red
                      << " \n green = " << Color.green
                      << " \n blue = " << Color.blue << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        PIR = H0AR9::RequestPIR(1);
        if (PIR.status == BOSStatus::BOS_OK)
        {
            std::cout << "[PIR] \n pir = " << PIR.pir
                      << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        Temp = H0AR9::RequestTemp(1);
        if (Temp.status == BOSStatus::BOS_OK)
        {
            std::cout << "[Temperature] \n temp = " << Temp.temp
                      << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        Distance = H0AR9::RequestDistance(1);
        if (Distance.status == BOSStatus::BOS_OK)
        {
            std::cout << "[Distance] \n distance = " << Distance.distance
                      << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        Humidity = H0AR9::RequestHumidity(1);
        if (Humidity.status == BOSStatus::BOS_OK)
        {
            std::cout << "[Humidity] \n humidity = " << Humidity.humidity
                      << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
