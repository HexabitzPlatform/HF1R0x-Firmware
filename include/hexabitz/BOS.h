#ifndef BOS_H
#define BOS_H

#include <stdint.h>
#include <stdbool.h>

#define	CODE_unknown_message							0
#define	CODE_ping										1
#define	CODE_ping_response								2
#define	CODE_IND_on										3
#define	CODE_IND_off									4
#define	CODE_IND_toggle									5

#define	CODE_hi											10
#define	CODE_hi_response								11
#define	CODE_explore_adj								12
#define	CODE_explore_adj_response						13
#define	CODE_port_dir									14
#define	CODE_baudrate									15
#define	CODE_module_id									16
#define	CODE_topology									17
#define	CODE_broadcast_plan								18
#define	CODE_read_port_dir								19
#define	CODE_read_port_dir_response						20
#define	CODE_exp_eeprom	 								21
#define	CODE_def_array	 								22
#define	CODE_CLI_command 								23
#define	CODE_CLI_response  								24
#define	CODE_update  									25
#define	CODE_update_via_port  							26
#define	CODE_DMA_channel  								27
#define	CODE_DMA_scast_stream  							28

#define	CODE_read_remote  								30
#define	CODE_read_remote_response  						31
#define	CODE_write_remote  								32
#define	CODE_write_remote_response  					33
#define	CODE_write_remote_force							34


namespace BOS {

static const int MAX_NUM_OF_PORTS = 8;
static const int MAX_NUM_OF_MODULES = 25;

enum module_pn_e {
	_H01R0 = 1, 
	_P01R0, 
	_H23R0, 
	_H23R1, 
	_H07R3, 
	_H08R6, 
	_H09R0, 
	_H1BR6, 
	_H12R0, 
	_H13R7, 
	_H0FR6, 
	_H1AR2, 
	_H0AR9, 
	_H1DR1, 
	_H1DR5, 
	_H0BR4, 
	_H18R0, 
	_H26R0
};

}



#endif /* BOS_H */