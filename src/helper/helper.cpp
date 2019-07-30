#include "helper/helper.h"

#include <endian.h>


// TODO: Handle different radix
// TODO: Check the standard of Float and issue error if appriopriate
// TODO: Handle Subnormal Number, Infinity, NaN

uint32_t hstd::pack754(float f, const unsigned bits, const unsigned expbits)
{
	if (f == 0.0)  // Get this special case out of the way
		return 0;

	const unsigned significandbits = bits - expbits - 1; // -1 for sign bit

	// Check sign and begin Normalization
	int exponent = 0;
	long long sign = (f < 0) ? 1 : 0;
	float fnorm = frexpf(static_cast<float>(fabs(f)), &exponent);

	// calculate the binary form (non-float) of the significand data
	long long significand = fnorm * ((1LL << significandbits) + 0.5f);

	// Get the biased exponent
	exponent = exponent + ((1 << (expbits - 1)) - 1);
 
	// return the final answer
	uint32_t result = (sign << (bits - 1)) | (exponent << (bits - expbits - 1)) | significand;
	return result;
}



float hstd::unpack754(uint32_t data, const unsigned bits, const unsigned expbits)
{
	if (data == 0)  // Get this special case out of the way
		return 0.0;

	const unsigned significandbits = bits - expbits - 1; // -1 for sign bit
	float result = float(data & ~(~0 << significandbits)) / (1LL << significandbits);
	unsigned sign = data & (1ULL << (bits - 1));

	if (sign)
		result = -result;

	int exponent = mask(data, significandbits, significandbits + expbits) - ((1 << (expbits - 1)) - 1);
	result = ldexpf(result, exponent);

	// return the final answer
	return result;
}

