
#include "BOS.h"

int main()
{

    // Initialize BOS
    initBOS();

    // led.blink(4 , 100);

    // Keep main thread alive indefinitely to allow background UART reading thread to run
    while (true)
    {
        // led.blink(4 , 100);
        Messaging::SendMessagetoModule(1, BOSMessageCode::CODE_PING, {});

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
