#include "hexabitz/BOSMessage.h"



hstd::Message hstd::read(HardwareSerial& serial)
{
	Message m;
	BinaryBuffer buffer;
	while (1) {
		if (!serial.available())
			continue;
		char c = serial.read();
		std::cout << "Received: " << std::hex << int(c) << std::endl;
		buffer.append(uint8_t(c));
		if (m.parse(buffer))
			break;
	}

	return m;
}

bool hstd::write(HardwareSerial& serial, Message& m)
{
	BinaryBuffer buffer = m.getBinaryStream();
	for (int i = 0; i < buffer.getLength(); i++)
		serial.write(buffer[i]);

	return true;
}

