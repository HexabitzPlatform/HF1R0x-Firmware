#include "hexabitz-modules/H08R6.h"
#include "helper/helper.h"

#include "hal/Serial.h"
#include "hexabitz/BOS.h"
#include "hexabitz/Service.h"
#include "hexabitz/BOSMessage.h"
#include "hexabitz/BOSMessageBuilder.h"

#include "Config.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <memory>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include <float.h>
#include <errno.h>
#include <signal.h>


void sigintHandler(int signum, siginfo_t *info, void *context)
{
	H_UNUSED(signum); H_UNUSED(info); H_UNUSED(context);
	
	std::cerr << "-------- SIGINT --------" << std::endl;
	std::cerr << "Waiting for send to exit normally" << std::endl;
	std::cerr << "-------- Existing --------" << std::endl;

	exit(0);
}

void init(void)
{
	struct sigaction act;
	memset(&act, 0, sizeof(act));

	act.sa_sigaction = sigintHandler;
	act.sa_flags |= SA_SIGINFO;

	sigemptyset(&act.sa_mask);

	if (sigaction(SIGINT, &act, NULL))
		perror("sigaction");
}

int main(int argc, char *argv[])
{
	std::string port = "/dev/ttyS0";

	std::cout << "Program Started (";
	std::cout << "Major: " << VERSION_MAJOR << " ";
	std::cout << "Minor: " << VERSION_MINOR << ")" << std::endl;

	init();

	if (argc > 1)
		port = std::string(argv[1]);

	std::cout << "Connecting to port " << port << std::endl;
	Service::getInstance()->init(port);
	std::shared_ptr<ProxyModule> master = std::make_shared<ProxyModule>(BOS::HF1R0);
	master->setUID(1);
	Service::getInstance()->setOwn(master);

	std::cout << "---------------- Start EXPLORE ----------------" << std::endl;
	int status = Service::getInstance()->Explore();
	std::cout << "Status: " << strerror(-status) << std::endl;
	std::cout << "---------------- Stop  EXPLORE ----------------" << std::endl;

	std::cout << Service::getInstance()->getModulesInfo().toBOSFmtString();

	H08R6 module;
    H08R6::addCodes();
	while (true) {
        H08R6::MeasurementUnit unit;
		module.setMeasurementUnit(H08R6::MeasurementUnit::CM);
        // module.getMeasurementUnit(unit);
        float value = 0;
		module.sample(value);
        std::cout << "Sample Value: " << value << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(5));
	}

	std::cout << "Closing Program" << std::endl;
	return 0;
}