// Porting/Porting.h
#ifndef PORTING_H
#define PORTING_H

namespace Porting {
    void initGPIO(int pin);
    void writeGPIO(int pin, bool value);
    bool readGPIO(int pin);
}

#endif
