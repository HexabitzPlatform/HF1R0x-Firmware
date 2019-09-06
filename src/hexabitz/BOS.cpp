#include "hexabitz/BOS.h"

#include <vector>

#define CONVERT_TO_STR_PART_NUMBER(x,y)					TO_STR(x), 
#define CONVERT_TO_NUM_OF_PORTS(x,y)					(y), 


static const std::vector<std::string> enumStrMap = {
	SEQ(CONVERT_TO_STR_PART_NUMBER)
};

static const std::vector<int> enumNumPortsMap = {
	SEQ(CONVERT_TO_NUM_OF_PORTS)
};

#undef CONVERT_TO_STR_PART_NUMBER
#undef CONVERT_TO_NUM_OF_PORTS


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

int BOS::getNumOfPorts(enum module_pn_e partNum)
{
	int index = static_cast<int>(partNum);
	if (index >= enumNumPortsMap.size())
		return -1;
	return enumNumPortsMap[index];
}