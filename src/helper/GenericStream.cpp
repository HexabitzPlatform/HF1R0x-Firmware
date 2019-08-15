#include "helper/GenericStream.h"
#include "helper/helper.h"

#include <math.h>
#include <endian.h>


IBinaryStream& IBinaryStream::append(uint16_t ui16)
{
	ui16 = htobe16(ui16);
	return append(reinterpret_cast<uint8_t *>(&ui16), sizeof(ui16));
}

IBinaryStream& IBinaryStream::append(uint32_t ui32)
{
	ui32 = htobe32(ui32);
	return append(reinterpret_cast<uint8_t *>(&ui32), sizeof(ui32));
}

IBinaryStream& IBinaryStream::append(float f)
{
	uint32_t value = htole32(hstd::pack754(f, 32, 8));
	return append(value);
}

IBinaryStream& IBinaryStream::append(const uint8_t *array, size_t num)
{
	IBinaryStream& self = *this;
	for (size_t i = 0; i < num; i++)
		append(array[i]);
	return self;
}

IBinaryStream& IBinaryStream::append(OBinaryStream& ostream, size_t num)
{
	IBinaryStream& self = *this;
	while (ostream.getLength() and num--)
		append(ostream.popui8());
	return self;
}



IBinaryStream& IBinaryStream::append(OBinaryStream& ostream)
{
	return append(ostream, ostream.getLength());
}

uint16_t OBinaryStream::popui16(void)
{
	uint16_t value;
	uint8_t *ptr = reinterpret_cast<uint8_t *>(&value);
	int sizeOfValue = sizeof(value);
	while (sizeOfValue--)
		*ptr++ = popui8();
	return le16toh(value);
}

uint32_t OBinaryStream::popui32(void)
{
	uint32_t value;
	uint8_t *ptr = reinterpret_cast<uint8_t *>(&value);
	int sizeOfValue = sizeof(value);
	while (sizeOfValue--)
		*ptr++ = popui8();
	return le32toh(value);
}

float OBinaryStream::popfloat(void)
{
	uint32_t value = popui32();
	return hstd::unpack754(value, 32, 8);
}