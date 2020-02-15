#ifndef H08R6_H
#define H08R6_H

#include "hexabitz/ProxyModule.h"

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>


class H08R6: public ProxyModule {
public:
    enum class MeasurementUnit {
        MM=0,       // 0
        CM,         // 1
        INCH        // 2
    };

public:
    bool setMeasurementUnit(MeasurementUnit unit);
    bool getMeasurementUnit(MeasurementUnit& unit); 
    bool sample(float& value);


public:
    static void addCodes(void);


public:
	H08R6(void): ProxyModule(BOS::H08R6)
	{
		id_ = Service::getInstance()->getIDOfPartNum(getPartNum(), 1);
		std::cout << "ID to BOS::H08R6: " << id_ << std::endl;
	}

	H08R6(unsigned nth): ProxyModule(BOS::H08R6)
	{
		id_ = Service::getInstance()->getIDOfPartNum(getPartNum(), nth);
		std::cout << "ID to BOS::H08R6: " << id_ << std::endl;
	}
	~H08R6(void)
	{

	}
};

#endif /* H08R6_H */