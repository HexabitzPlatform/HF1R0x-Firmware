#ifndef BOS_H
#define BOS_H

#include "helper/helper.h"

#include <stdint.h>
#include <stdbool.h>

#include <vector>
#include <string>



#define _CODE_SEQ(OPERATION)													\
		OPERATION(		CODE_unknown_message			,			0		)	\
		OPERATION(		CODE_ping						,			1		)	\
		OPERATION(		CODE_ping_response				,			2		)	\
		OPERATION(		CODE_IND_on						,			3		)	\
		OPERATION(		CODE_IND_off					,			4		)	\
		OPERATION(		CODE_IND_toggle					,			5		)	\
		OPERATION(		CODE_hi							,			10		)	\
		OPERATION(		CODE_hi_response				,			11		)	\
		OPERATION(		CODE_explore_adj				,			12		)	\
		OPERATION(		CODE_explore_adj_response		,			13		)	\
		OPERATION(		CODE_port_dir					,			14		)	\
		OPERATION(		CODE_baudrate					,			15		)	\
		OPERATION(		CODE_module_id					,			16		)	\
		OPERATION(		CODE_topology					,			17		)	\
		OPERATION(		CODE_broadcast_plan				,			18		)	\
		OPERATION(		CODE_read_port_dir				,			19		)	\
		OPERATION(		CODE_read_port_dir_response		,			20		)	\
		OPERATION(		CODE_exp_eeprom	 				,			21		)	\
		OPERATION(		CODE_def_array	 				,			22		)	\
		OPERATION(		CODE_CLI_command 				,			23		)	\
		OPERATION(		CODE_CLI_response  				,			24		)	\
		OPERATION(		CODE_update  					,			25		)	\
		OPERATION(		CODE_update_via_port  			,			26		)	\
		OPERATION(		CODE_DMA_channel  				,			27		)	\
		OPERATION(		CODE_DMA_scast_stream  			,			28		)	\
		OPERATION(		CODE_read_remote  				,			30		)	\
		OPERATION(		CODE_read_remote_response  		,			31		)	\
		OPERATION(		CODE_write_remote  				,			32		)	\
		OPERATION(		CODE_write_remote_response  	,			33		)	\
		OPERATION(		CODE_write_remote_force			,			34		)	\



#define _PARTNUM_SEQ(OPERATION) 						\
		OPERATION(		INVALID		,	   -1		)  	\
		OPERATION(		H01R0		,		6		) 	\
		OPERATION(		P01R0		,		6		) 	\
		OPERATION(		H23R0		,		6		) 	\
		OPERATION(		H23R1		,		6		) 	\
		OPERATION(		H07R3		,		6		) 	\
		OPERATION(		H08R6		,		6		) 	\
		OPERATION(		H09R0		,		6		) 	\
		OPERATION(		H1BR6		,		6		) 	\
		OPERATION(		H12R0		,		6		) 	\
		OPERATION(		H13R7		,		6		) 	\
		OPERATION(		H0FR6		,		6		) 	\
		OPERATION(		H1AR2		,		6		) 	\
		OPERATION(		H0AR9		,		6		) 	\
		OPERATION(		H1DR1		,		6		) 	\
		OPERATION(		H1DR5		,		6		) 	\
		OPERATION(		H0BR4		,		6		) 	\
		OPERATION(		H18R0		,		6		) 	\
		OPERATION(		H26R0		,		6		)	\
		OPERATION(		HF1R0		,		2		)


enum code_e {
_CODE_SEQ(TO_ENUM_VALUE)
};



namespace BOS {

static constexpr int MAX_NUM_OF_PORTS = 10;
static constexpr int MAX_NUM_OF_MODULES = 25;

enum module_pn_e {
#define TO_ENUM_PART_NUMBER(p,n)					TO_ENUM(p)

	_PARTNUM_SEQ(TO_ENUM_PART_NUMBER)
	
#undef TO_ENUM_PART_NUMBER
};


enum class PortDir {
	NORMAL, 
	REVERSED
};


std::string toString(enum module_pn_e pn);
enum module_pn_e toPartNumberEnum(std::string str);
std::string getStrOfCode(int code);
std::vector<std::string> getPartNumberList(void);
int getNumOfPorts(enum module_pn_e partNum);

void addCode(int code, const std::string& name);
#define BOS_ADD_CODE(code)	BOS::addCode(code, #code)

}



#endif /* BOS_H */