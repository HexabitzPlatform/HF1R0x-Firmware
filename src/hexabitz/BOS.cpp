#include "hexabitz/BOS.h"

#include <vector>

#define CONVERT_TO_STR(x)				TO_STR(x), 

static std::vector<std::string> enumStrMap = {
	SEQ(CONVERT_TO_STR)
};


std::string BOS::toString(enum module_pn_e pn)
{
	if (pn == BOS::INVALID)
		return std::string();
	return std::string(enumStrMap[static_cast<int>(pn)]);
}

enum BOS::module_pn_e BOS::toPartNumberEnum(std::string str)
{
	for (int i = 0; i < enumStrMap.size(); i++) {
		if (!enumStrMap[i].compare(str))
			return static_cast<enum BOS::module_pn_e>(i);
	}
	return BOS::INVALID;
}

std::vector<std::string> BOS::getPartNumberList(void)
{
	return enumStrMap;
}