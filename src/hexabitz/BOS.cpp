#include "hexabitz/BOS.h"

#include <vector>
#include <map>

#define CONVERT_TO_STR_PART_NUMBER(x,y)					TO_STR(x), 
#define CONVERT_TO_NUM_OF_PORTS(x,y)					(y), 

#define CONVERT_TO_CODE_VALUE_PAIR(x,y)					{ y, TO_STR(x) }, 

static const std::vector<std::string> enumStrMap = {
	_PARTNUM_SEQ(CONVERT_TO_STR_PART_NUMBER)
};

static const std::vector<int> enumNumPortsMap = {
	_PARTNUM_SEQ(CONVERT_TO_NUM_OF_PORTS)
};

static std::map<int, std::string> codeStrMap = {
	_CODE_SEQ(CONVERT_TO_CODE_VALUE_PAIR)
};

#undef CONVERT_TO_STR_PART_NUMBER
#undef CONVERT_TO_NUM_OF_PORTS
#undef CONVERT_TO_CODE_VALUE_PAIR


std::string BOS::toString(enum module_pn_e pn)
{
	if (pn == BOS::INVALID)
		return std::string();
	if (pn >= enumStrMap.size())
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

std::string BOS::getStrOfCode(int code)
{
	return codeStrMap[code];
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