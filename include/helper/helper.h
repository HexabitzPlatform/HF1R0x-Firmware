#ifndef HELPER_H
#define HELPER_H

#include <math.h>
#include <ctype.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#include <algorithm>
#include <bitset>

#define H_UNUSED(x)						(x)

#define HIGH							true
#define LOW								false

#define PI								(3.1415926535897932384626433832795)
#define HALF_PI							(1.5707963267948966192313216916398)
#define TWO_PI							(6.283185307179586476925286766559)

#define DEG_TO_RAD						(0.017453292519943295769236907684886)
#define RAD_TO_DEG						(57.295779513082320876798154814105)

#define EULER							(2.718281828459045235360287471352)

#define CR								('\r')   // Carriage Return Character
#define LF								('\n')   // Line Feed or New Line Character

#define TO_STR(token)					#token
#define TO_ENUM(token)					token,


#define radians(deg)					((deg) * DEG_TO_RAD)
#define degrees(rad)					((rad) * RAD_TO_DEG)

#define lowByte(w)						((uint8_t)((w) & 0xFFUL))
#define highByte(w)						((uint8_t)(((w) >> 8)) & 0xFFUL)

#define lowWord(ui32)					((uint16_t)((ui32) & 0xFFFFUL))
#define highWord(ui32)					((uint16_t)(((ui32) >> 16)) & 0xFFFFUL)

#define bitRead(value,bit) 				(((value) >> (bit)) & 0x01)
#define bitSet(value,bit) 				((value) |= (1UL << (bit)))
#define bitClear(value,bit) 			((value) &= ~(1UL << (bit)))
#define bitWrite(value,bit,bitvalue) 	(bitvalue ? bitSet(value, bit) : bitClear(value, bit))


#define isEven(x)						((x) & 0x01)
#define isOdd(x)						(!isEven(x))

/* Double Negation Normalize Boolean values i {0,1} */
#define bit_at(a,i)						(!!((a)[(unsigned)(i) >> 3] & (1 << ((unsigned)(i) & 0x07))))
	
#define ARRAY_SIZE(a)					(sizeof(a) / sizeof(a[0]))


#define isBank(c)						(((c) == ' ') || ((c) == '\t') || ((c) == '\v'))
#define isBacksplash(c)					((c) == '/')
#define isHex(c)						(isxdigit(c))

#define isCR(c)							((c) == CR)
#define isLF(c)							((c) == LF)



namespace hstd {
	template <typename T>
	static inline T constrain(T value, T lower, T upper) { return std::max(lower, std::min(value, upper)); }

	template <typename T>
	static inline T constrain(T value, size_t lower, size_t upper) { return std::max(lower, std::min(value, upper)); }

	template <typename T>
	static inline T constrainLower(T value, T lower, T upper) { return std::max(lower, value); }

	template <typename T>
	static inline T constrainUpper(T value, T upper) { return std::min(value, upper); }

	template <typename T>
	static inline T mask(T value, long start, long end) { return (value >> start) & (~(~0 << (end - start + 1))); }

	template <typename T>
	static inline T map(T x, T in_min, T in_max, T out_min, T out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }


	template <typename T>
	std::bitset<sizeof(T) * 8> make_bitset(const T& value)
	{
		return std::bitset<sizeof(T) * 8>(value);
	}


	uint32_t pack754(float f, const unsigned bits, const unsigned expbits);
	float unpack754(uint32_t data, const unsigned bits, const unsigned expbits);
}


#endif	// HELPER_H