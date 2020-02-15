#ifndef H01R0_H
#define H01R0_H

#include "hexabitz/ProxyModule.h"

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>


class H01R0: public ProxyModule {

public:
	bool setRGB(int red, int green, int blue, int intensity);


public:
	H01R0(void): ProxyModule(BOS::H01R0)
	{
		id_ = Service::getInstance()->getIDOfPartNum(getPartNum(), 1);
		std::cout << "ID to BOS::H01R0: " << id_ << std::endl;
	}

	H01R0(unsigned nth): ProxyModule(BOS::H01R0)
	{
		id_ = Service::getInstance()->getIDOfPartNum(getPartNum(), nth);
		std::cout << "ID to BOS::H01R0: " << id_ << std::endl;
	}
	~H01R0(void)
	{

	}
};

#endif /* H01R0_H */