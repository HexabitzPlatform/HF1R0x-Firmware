// src/main.cpp
#include "Porting.h"
#include <iostream>
#include <unistd.h> // for sleep

int main() {
    const int ledPin = 17; // GPIO17 (BCM numbering)

    std::cout << "Blinking LED on GPIO" << ledPin << "...\n";

    while (true) {
        Porting::writeGPIO(ledPin, true);  // LED ON
        sleep(1);
        Porting::writeGPIO(ledPin, false); // LED OFF
        sleep(1);
    }

    return 0;
}
