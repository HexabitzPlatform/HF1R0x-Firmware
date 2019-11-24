#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <string>

// Define config for Serial.begin(baud, config);
#define _DEF_SERIALCNF_VALUE(d,p,s)         ((unsigned(d) << 8) | (p << 4) | s)

#define _DEF_SERIALCNF_VALUE_NO_PAR(d,s)    _DEF_SERIALCNF_VALUE(d,0,s)
#define _DEF_SERIALCNF_VALUE_O_PAR(d,s)     _DEF_SERIALCNF_VALUE(d,1,s)
#define _DEF_SERIALCNF_VALUE_E_PAR(d,s)     _DEF_SERIALCNF_VALUE(d,2,s)
    

#define SERIAL_5N1						_DEF_SERIALCNF_VALUE_NO_PAR(5,1)
#define SERIAL_6N1						_DEF_SERIALCNF_VALUE_NO_PAR(6,1)
#define SERIAL_7N1						_DEF_SERIALCNF_VALUE_NO_PAR(7,1)
#define SERIAL_8N1						_DEF_SERIALCNF_VALUE_NO_PAR(8,1)
#define SERIAL_5N2						_DEF_SERIALCNF_VALUE_NO_PAR(5,2)
#define SERIAL_6N2						_DEF_SERIALCNF_VALUE_NO_PAR(6,2)
#define SERIAL_7N2						_DEF_SERIALCNF_VALUE_NO_PAR(7,2)
#define SERIAL_8N2						_DEF_SERIALCNF_VALUE_NO_PAR(8,2)
#define SERIAL_5E1						_DEF_SERIALCNF_VALUE_E_PAR(5,1)
#define SERIAL_6E1						_DEF_SERIALCNF_VALUE_E_PAR(6,1)
#define SERIAL_7E1						_DEF_SERIALCNF_VALUE_E_PAR(7,1)
#define SERIAL_8E1						_DEF_SERIALCNF_VALUE_E_PAR(8,1)
#define SERIAL_5E2						_DEF_SERIALCNF_VALUE_E_PAR(5,2)
#define SERIAL_6E2						_DEF_SERIALCNF_VALUE_E_PAR(6,2)
#define SERIAL_7E2						_DEF_SERIALCNF_VALUE_E_PAR(7,2)
#define SERIAL_8E2						_DEF_SERIALCNF_VALUE_E_PAR(8,2)
#define SERIAL_5O1						_DEF_SERIALCNF_VALUE_O_PAR(5,1)
#define SERIAL_6O1						_DEF_SERIALCNF_VALUE_O_PAR(6,1)
#define SERIAL_7O1						_DEF_SERIALCNF_VALUE_O_PAR(7,1)
#define SERIAL_8O1						_DEF_SERIALCNF_VALUE_O_PAR(8,1)
#define SERIAL_5O2						_DEF_SERIALCNF_VALUE_O_PAR(5,2)
#define SERIAL_6O2						_DEF_SERIALCNF_VALUE_O_PAR(6,2)
#define SERIAL_7O2						_DEF_SERIALCNF_VALUE_O_PAR(7,2)
#define SERIAL_8O2						_DEF_SERIALCNF_VALUE_O_PAR(8,2)


class HardwareSerial {

public:
    bool open(const char *pathname);
    bool isOpen(void) const;

public:
	bool begin(unsigned long baud) { return begin(baud, SERIAL_8N1); }
    bool begin(unsigned long baud, unsigned cfg);
    void end(void);

public:
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    // virtual int availableForWrite(void);
    virtual void flush(void);

    virtual size_t write(uint8_t c);
    virtual size_t write(const char c)      { return write((uint8_t)c); }
    inline size_t write(unsigned long n) 	{ return write((uint8_t)n); }
    inline size_t write(long n)				{ return write((uint8_t)n); }
    inline size_t write(unsigned int n) 	{ return write((uint8_t)n); }
    inline size_t write(int n) 				{ return write((uint8_t)n); }

    operator bool(void) 					{ return true; }

public:
    int println(const char *cstr = nullptr);
    std::string readLine(void);

public:
	HardwareSerial(const char *pathname);
	~HardwareSerial(void);

private:
	int fd_;
};


#endif