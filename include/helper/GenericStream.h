#ifndef GENERIC_STREAM_H
#define GENERIC_STREAM_H

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdbool.h>


class OBinaryStream {

public:
	virtual size_t getLength(void) const = 0;
	virtual void flush(void) = 0;

public:
	virtual uint8_t popui8(void) = 0;
	uint16_t popui16(void);
	uint32_t popui32(void);
	float popfloat(void);

public:
	OBinaryStream(void)	 { }
	~OBinaryStream(void) { }
};


class IBinaryStream {

public:
	virtual size_t getLength(void) const = 0;
	virtual void flush(void) = 0;

public:
	virtual IBinaryStream& append(uint8_t ui8) = 0;
	IBinaryStream& append(uint16_t ui16);
	IBinaryStream& append(uint32_t ui32);
	IBinaryStream& append(float f);
	IBinaryStream& append(const uint8_t *array, size_t num);
	IBinaryStream& append(OBinaryStream& ostream, size_t num);
	IBinaryStream& append(OBinaryStream& ostream);

public:
	IBinaryStream(void)	 { }
	~IBinaryStream(void) { }
};


#endif // GENERIC_STREAM_H