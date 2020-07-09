#include "hexabitz/ProxyModule.h"
#include "helper/helper.h"

#include "hal/Serial.h"
#include "hexabitz/Service.h"
#include "hexabitz/BOSMessage.h"
#include "hexabitz/BOSMessageBuilder.h"

#include <iostream>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include <float.h>
#include <errno.h>

// H01R0x
#define	CODE_H01R0_ON							100
#define	CODE_H01R0_OFF							101
#define	CODE_H01R0_TOGGLE						102
#define	CODE_H01R0_COLOR						103
#define	CODE_H01R0_PULSE						104
#define	CODE_H01R0_SWEEP						105
#define	CODE_H01R0_DIM							106

 // H07R3x
 #define CODE_H07R3_PLAY_SINE 350
 #define CODE_H07R3_PLAY_WAVE 351
 #define CODE_H07R3_PLAY_TUNE 352

 // H08R6x
 #define CODE_H08R6_GET_INFO 400
 #define CODE_H08R6_SAMPLE 401
 #define CODE_H08R6_STREAM_PORT 402
 #define CODE_H08R6_STREAM_MEM 403
 #define CODE_H08R6_RESULT_MEASUREMENT 404
 #define CODE_H08R6_STOP_RANGING 405
 #define CODE_H08R6_SET_UNIT 406
 #define CODE_H08R6_GET_UNIT 407
 #define CODE_H08R6_RESPOND_GET_UNIT 408
 #define CODE_H08R6_MAX_RANGE 409
 #define CODE_H08R6_MIN_RANGE 410
 #define CODE_H08R6_TIMEOUT 411

 // H0BR4x
 #define CODE_H0BR4_GET_GYRO 550
 #define CODE_H0BR4_GET_ACC 551
 #define CODE_H0BR4_GET_MAG 552
 #define CODE_H0BR4_GET_TEMP 553
 #define CODE_H0BR4_RESULT_GYRO 554
 #define CODE_H0BR4_RESULT_ACC 555
 #define CODE_H0BR4_RESULT_MAG 556
 #define CODE_H0BR4_RESULT_TEMP 557
 #define CODE_H0BR4_STREAM_GYRO 558
 #define CODE_H0BR4_STREAM_ACC 559
 #define CODE_H0BR4_STREAM_MAG 560
 #define CODE_H0BR4_STREAM_TEMP 561
 #define CODE_H0BR4_STREAM_STOP 562

 // H0FR6x
 #define CODE_H0FR6_ON 750
 #define CODE_H0FR6_OFF 751
 #define CODE_H0FR6_TOGGLE 752
 #define CODE_H0FR6_PWM 753

 // H1BR6x
 // H23R0x
 #define CODE_H23Rx_GET_INFO 1700
 #define CODE_H23Rx_DOWNLOAD_SCRIPT_OTA 1701
 #define CODE_H23Rx_DOWNLOAD_SCRIPT_UART 1702
 #define CODE_H23Rx_RUN_AUTORUN_SCRIPT 1703
 #define CODE_H23Rx_VSP_COMMAND_MODE 1704
 #define CODE_H23Rx_VSP_BRIDGE_MODE 1705
 #define CODE_H23Rx_SPP_MODE 1706
 #define CODE_H23Rx_LED_STATUS_ON 1707
 #define CODE_H23Rx_LED_STATUS_OFF 1708
 #define CODE_H23Rx_BTC_DEL_ALL_DATA_SEG 1709
 #define CODE_H23Rx_EVBTC_SPPCONN 1710
 #define CODE_H23Rx_EVBTC_SPPDISCON 1711
 #define CODE_H23Rx_EVBTC_PAIR_REQUEST 1712
 #define CODE_H23Rx_EVBTC_PIN_REQUEST 1713
 #define CODE_H23Rx_EVBTC_PAIR_RESULT 1714
 #define CODE_H23Rx_EVBTC_AUTHREQ 1715
 #define CODE_H23Rx_EVBTC_PASSKEY 1716
 #define CODE_H23Rx_SHOW_DEBUG_INFO 1717
 #define CODE_H23Rx_SCAN_INQUIRE 1718
 #define CODE_H23Rx_SCAN_RESPOND 1719
 #define CODE_H23Rx_SCAN_RESPOND_ERR 1720
 #define CODE_H23Rx_CONNECT_INQUIRE 1721
 #define CODE_H23Rx_CONNECT_RESPOND 1722
 #define CODE_H23Rx_FINISHED_SCAN 1723
 #define CODE_H23Rx_UNKNOWN_CMD 1799

 // H26R0x
 #define CODE_H26R0_SET_RATE 1900
 #define CODE_H26R0_STREAM_PORT_GRAM 1901
 #define CODE_H26R0_STREAM_PORT_KGRAM 1902
 #define CODE_H26R0_STREAM_PORT_OUNCE 1903
 #define CODE_H26R0_STREAM_PORT_POUND 1904
 #define CODE_H26R0_STOP 1905
 #define CODE_H26R0_SAMPLE_GRAM 1906
 #define CODE_H26R0_SAMPLE_KGRAM 1907
 #define CODE_H26R0_SAMPLE_OUNCE 1908
 #define CODE_H26R0_SAMPLE_POUND 1909
 #define CODE_H26R0_ZEROCAL 1910
 #define CODE_H26R0_STREAM_RAW 1911
 #define CODE_H26R0_SAMPLE_RAW 1912
 #define CODE_H26R0_STREAM_FORMAT 1913


ProxyModule::ProxyModule(std::string part, int numPorts, hstd::uid_t uid): id_(uid), numOfPorts_(numPorts), info_(part)
{

}

ProxyModule::ProxyModule(std::string part, int numPorts): id_(hstd::Addr_t::INVALID_UID), numOfPorts_(numPorts), info_(part)
{
	if (numOfPorts_ <= 0)
		numOfPorts_ = BOS::getNumOfPorts(BOS::toPartNumberEnum(part));
}

ProxyModule::ProxyModule(BOS::module_pn_e part): id_(hstd::Addr_t::INVALID_UID), numOfPorts_(-1), info_()
{
	info_ = BOS::toString(part);
	numOfPorts_ = BOS::getNumOfPorts(part);
}

ProxyModule::~ProxyModule(void)
{

}

hstd::uid_t ProxyModule::getUID(void) const
{
	return id_;
}

void ProxyModule::setUID(hstd::uid_t newID)
{
	id_ = newID;
}

int ProxyModule::getNumOfPorts(void) const
{
	return numOfPorts_;
}

bool ProxyModule::isMaster(void) const
{
	return hstd::Addr_t::isMaster(id_);
}

bool ProxyModule::isValidID(void) const
{
	return hstd::Addr_t::isValidUID(id_);
}

void ProxyModule::setNumOfPorts(int ports)
{
	numOfPorts_ = ports;
}

std::string ProxyModule::getPartStr(void) const
{
	return info_;
}

BOS::module_pn_e ProxyModule::getPartNum(void) const
{
	return BOS::toPartNumberEnum(info_);
}

uint16_t ProxyModule::getPartNum_ui16(void) const
{
	return static_cast<uint16_t>(getPartNum());
}

bool ProxyModule::send(hstd::Message m)
{
	if (hstd::Addr_t::isValidUID(id_))
		m.getDest().setUID(id_);
	return Service::getInstance()->send(m);
}

bool ProxyModule::receive(hstd::Message& m, long timeout)
{
	return Service::getInstance()->receive(m, timeout);
}


bool H01R0::setRGB(int red, int green, int blue, int intensity)
{
	hstd::Message msg = hstd::make_message(id_, CODE_H01R0_COLOR);
	msg.getParams().append(static_cast<uint8_t>(1));
	msg.getParams().append(static_cast<uint8_t>(red));
	msg.getParams().append(static_cast<uint8_t>(green));
	msg.getParams().append(static_cast<uint8_t>(blue));
	msg.getParams().append(static_cast<uint8_t>(intensity));

	return send(msg);
}

bool H0FR6::relayon(int timeout)
{
  hstd::Message msg = hstd::make_message(id_, CODE_H0FR6_ON);
  msg.getParams().append(static_cast<uint8_t>(1));
  msg.getParams().append(static_cast<uint8_t>(timeout));
  return send(msg);
}

bool H0FR6::relayoff(void)
{
  hstd::Message msg = hstd::make_message(id_, CODE_H0FR6_OFF);
  msg.getParams().append(static_cast<uint8_t>(1));
  return send(msg);
}

bool H0FR6::rpwm(int dutycycle)
{
  hstd::Message msg = hstd::make_message(id_, CODE_H0FR6_PWM);
  msg.getParams().append(static_cast<uint8_t>(1));
  msg.getParams().append(static_cast<uint8_t>(dutycycle));
  return send(msg);
}
