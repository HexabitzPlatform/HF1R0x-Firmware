
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

    // H0AR9_Color Color;
    // H0AR9_PIR PIR;
    // H0AR9_Temp Temp;
    // H0AR9_Distance Distance;
    // H0AR9_Humidity Humidity;

    // H05R0_CellAge age;
    // H05R0_CellCapacity capacity;
    // H05R0_CellCurrent current;
    // H05R0_CellCycles cycles;
    // H05R0_CellPower power;
    // H05R0_CellTemp temp;
    // H05R0_CellVoltage voltage;
    // H05R0_SOC soc;
    
    // H09R9_Temp Temp;

    // H1FR5_GetHeight height;
    // H1FR5_GetSpeed speed;
    // H1FR5_GetUTC utc;
    // H1FR5_GetPosition position;
 
    // H08R7_TOF distance;

    // H2AR3_Voltage voltage;
    // H2AR3_Current currentCR8450;   
    
    
    /* RPI ID using explore feature*/
    PIConfig::piID = 2;
  
    // Keep main thread alive indefinitely to allow background UART reading thread to run
    while (true)
    {

        /**************************************************************************************************/
    
        // voltage = H05R0::RequestVoltage(1);
        // if (voltage.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Voltage: " << voltage.voltage
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // current = H05R0::RequestCurrent(1);
        // if (current.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Current: " << current.current
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // power = H05R0::RequestPower(1);
        // if (power.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Power: " << power.power
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // temp = H05R0::RequestTemp(1);
        // if (temp.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Temerature: " << temp.temp
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // capacity = H05R0::RequestCapacity(1);
        // if (capacity.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Capacity: " << capacity.capacity
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // soc = H05R0::RequestSOC(1);
        // if (soc.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery SOC: " << static_cast<int>(soc.SOC)
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        //   age = H05R0::RequestAge(1);
        // if (age.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Age = " << static_cast<int>(age.age)
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // cycles = H05R0::RequestCycles(1);
        // if (cycles.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Battery Cycles: " << cycles.cycles
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        

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

        // Color = H0AR9::RequestColor(1);
        // if (Color.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Color] \n red = " << Color.red
        //               << " \n green = " << Color.green
        //               << " \n blue = " << Color.blue << "\n";
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // PIR = H0AR9::RequestPIR(1);
        // if (PIR.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[PIR] \n pir = " << PIR.pir
        //               << "\n";
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Temp = H0AR9::RequestTemp(1);
        // if (Temp.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Temperature] \n temp = " << Temp.temp
        //               << "\n";
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Distance = H0AR9::RequestDistance(1);
        // if (Distance.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Distance] \n distance = " << Distance.distance
        //               << "\n";
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Humidity = H0AR9::RequestHumidity(1);
        // if (Humidity.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Humidity] \n humidity = " << Humidity.humidity
        //               << "\n";
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        /**************************************************************************************************/

        // Temp = H09R9::RequestTemp(1);
        // if (Temp.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Temperature] \n temp = " << Temp.temp
        //               << "\n";
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

         /**************************************************************************************************/
        // height = H1FR5::RequestHeight(1);
        // if (height.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Height : " << height.height
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
     
        // speed = H1FR5::RequestSpeed(1);
        // if (speed.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Speed]  \n speedinch = " << speed.speedinch
        //               << "\n speedkm = " << speed.speedkm
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // utc = H1FR5::RequestUTC(1);
        // if (utc.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[UTC]  \n hours = " << static_cast<int>(utc.hours)
        //               << "\n min = "  << static_cast<int>(utc.min)
        //               << "\n sec = "  << static_cast<int>(utc.sec)
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // position = H1FR5::RequestPosition(1);
        // if (position.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[Position]  \n longdegree = " << position.longdegree
        //               << "\n latdegree = " << position.latdegree
        //               << "\n longindicator = "  << static_cast<int>(position.longindicator)
        //               << "\n latindicator = "  << static_cast<int>(position.latindicator)
        //               << "\n longindicator = "  << position.longindicator
        //               << "\n latindicator = "  << position.latindicator
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
         /**************************************************************************************************/

        // distance = H08R7::RequestTOF(1);
        // if (distance.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "[TOF Distance] \n distance = " << distance.distance
        //               << "\n";
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));     
    
        /**************************************************************************************************/
        // voltage = H2AR3::RequestVoltage(1);
        // if (voltage.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Voltage: " << voltage.volt
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // currentCR8450 = H2AR3::RequestCurrent(1);
        // if (currentCR8450.status == BOSStatus::BOS_OK)
        // {
        //     std::cout << "Current from CR8450_1000 transformer: " << currentCR8450.current
        //               << "\n"
        //               << std::endl;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        /**************************************************************************************************/

    }

    return 0;
}
