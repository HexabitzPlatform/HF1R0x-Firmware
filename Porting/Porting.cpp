// Porting/Porting.cpp
#include "Porting.h"
#include <gpiod.h>
#include <iostream>
#include <unistd.h> // for sleep

#define CHIP_NAME "gpiochip0"

namespace Porting {

    void initGPIO(int pin) {
        // No explicit init needed for libgpiod, just placeholder
        (void)pin;
    }

    void writeGPIO(int pin, bool value) {
        gpiod_chip* chip = gpiod_chip_open_by_name(CHIP_NAME);
        if (!chip) {
            std::cerr << "Failed to open chip\n";
            return;
        }

        gpiod_line* line = gpiod_chip_get_line(chip, pin);
        if (!line) {
            std::cerr << "Failed to get GPIO line\n";
            gpiod_chip_close(chip);
            return;
        }

        if (gpiod_line_request_output(line, "BOS", 0) < 0) {
            std::cerr << "Failed to request line as output\n";
            gpiod_chip_close(chip);
            return;
        }

        gpiod_line_set_value(line, value ? 1 : 0);
        gpiod_line_release(line);
        gpiod_chip_close(chip);
    }

    bool readGPIO(int pin) {
        gpiod_chip* chip = gpiod_chip_open_by_name(CHIP_NAME);
        gpiod_line* line = gpiod_chip_get_line(chip, pin);

        gpiod_line_request_input(line, "BOS");
        int value = gpiod_line_get_value(line);

        gpiod_line_release(line);
        gpiod_chip_close(chip);

        return value == 1;
    }

}
