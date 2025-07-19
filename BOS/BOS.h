#pragma once

/* C++ Libraries */
#include <vector>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>
#include <future>

/* BOS Files */
#include "Porting.h"
#include "UARTParser.h"
#include "BOS_Constanats.h"
#include "BOS_MessageCodes.h"

/* BOS message option byte structure ***************************************************************/
typedef struct
{
    uint8_t ExtendedOptions : 1;     /* If set, additional option byte follows */
    uint8_t ExtendedMessageCode : 1; /* If set, message codes are 16-bit */
    uint8_t Trace : 1;               /* If set, message trace (ping) is enabled */
    uint8_t Acknowledgment : 1;      /* Message acknowledgment flag */
    uint8_t Reserved : 1;            /* Reserved for future use */
    uint8_t Response : 2;            /* Response type */
    uint8_t LongMessage : 1;         /* If set, message continues in next packet */
} BOSOptionByte_t;

typedef struct
{
    std::vector<uint8_t> buffer = {};
    uint8_t length = buffer.size();
    uint8_t receiveFlag = 0;
} RawData_t;

/**************************************************************************************************/
/**************************************** Enum Class Definitions **********************************/
/**************************************************************************************************/
enum class BOSStatus : int16_t
{
    // BOS Status:
    BOS_OK = 0,                 /* Operation successful */
    BOS_ERR_UnknownMessage = 1, /* Unknown message received */
    BOS_ERR_NoResponse = 2,     /* No response from module */
    BOS_ERR_MSG_Reflection = 3, /* Message reflection detected */
    BOS_ERR_UnIDedModule = 5,   /* Unidentified module */

    BOS_ERR_Keyword = 6,       /* Invalid keyword */
    BOS_ERR_ExistingAlias = 7, /* Alias already exists */

    BOS_ERR_REMOTE_READ_TIMEOUT = 15,  /* Timeout during remote read */
    BOS_ERR_REMOTE_READ_NO_VAR = 16,   /* No variable found during remote read */
    BOS_ERR_REMOTE_WRITE_TIMEOUT = 17, /* Timeout during remote write */
    BOS_ERR_REMOTE_WRITE_INDEX = 19,   /* Invalid remote write index */
    BOS_ERR_LOCAL_FORMAT_UPDATED = 20, /* Local format updated */
    BOS_ERR_REMOTE_WRITE_ADDRESS = 21, /* Invalid remote write address */

    BOS_ERR_PORT_BUSY = 23,               /* Communication port busy */
    BOS_ERR_TIMEOUT = 24,                 /* Operation timeout */
    BOS_ERR_WrongName = 100,              /* Incorrect name */
    BOS_ERR_WrongGroup = 101,             /* Incorrect group */
    BOS_ERR_WrongID = 102,                /* Incorrect ID */
    BOS_ERR_WrongParam = 103,             /* Incorrect parameter */
    BOS_ERR_WrongValue = 104,             /* Incorrect value */
    BOS_ERR_MSG_DOES_NOT_FIT = 105,       /* Message does not fit */
    BOS_ERR_OVER_MSG_PARAMS_LENGTH = 106, /*message parames is over 46 bytes */

    BOS_MULTICAST = 254, /* Multicast message */
    BOS_BROADCAST = 255, /* Broadcast message */

    BOS_ERROR = -1 /* Generic error */

};

/* Module PN Strings Definition ******************************************************************/
enum class ModulePN : uint8_t
{
    H01R0,
    P01R0,
    H23R0,
    H23R1,
    H23R3,
    H07R3,
    H08R6,
    P08R6,
    H09R0,
    H09R9,
    H1BR6,
    H12R0,
    H13R7,
    H0FR1,
    H0FR6,
    H0FR7,
    H1AR2,
    H0AR9,
    H1DR1,
    H1DR5,
    H0BR4,
    H18R0,
    H26R0,
    H15R0,
    H10R4,
    H2AR3,
    H41R6,
    H3BR6,
    H18R1,
    H1FR5,
    H3BR2,
    H21R2,
    H17R1,
    H15R8,
    H2BR0,
    H05R0,
    H3BR7,
    H2BR1,
    H07R8,
    H08R7,
    H16R6,
    P08R7,
    H19R0,
    Raspberry_PI,
    H14RA
};

/* Modules */
#include "H0BR4.h"
#include "H0AR9.h"
#include "h05R0.h"
#include "H09R9.h"
#include "H1FR5.h"
#include "H08R7.h"
#include "H2AR3.h"
#include "H2BR1.h"
/**************************************************************************************************/
/******************************************  Class Definitions ************************************/
/**************************************************************************************************/

/* BOS Message Parser Class Definitions ***********************************************************/
class BOS_MessageParser
{

public:
    uint8_t NumberofModules = 0;
    std::array<std::array<uint16_t, 11>, 26> Array; // raw: 11 , colom: 26

    virtual BOSStatus parseMessage(const std::vector<uint8_t> &payload);

private:
    std::array<uint16_t, 2> NeighborsInfo{};
    std::vector<uint8_t> MessageParames;

    // Indicator-Related Message Codes Functions:
    BOSStatus handlePingCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleIndicatorOnCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleIndicatorOffCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleIndicatorToggleCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Explore-Related Message Codes Functions:
    BOSStatus handleHiCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleExploreADJCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleExploreADJResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handlePortDirectionCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleModuleIDCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleTopologyCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Read/Write Remote-Related Message Codes Functions:
    BOSStatus handleReadRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleReadRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleWriteRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleWriteRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    BOSStatus handleRawDataCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Overridable method to handle unknown/module-specific codes
    virtual BOSStatus handleModuleMessageCode(uint8_t dst, uint8_t source, BOSMessageCode code, const std::vector<uint8_t> &params);
};

/* Module Message Parser Class Definitions ********************************************************/
class Module_MessageParser : public BOS_MessageParser
{
protected:
    BOSStatus handleModuleMessageCode(uint8_t dst, uint8_t source, BOSMessageCode code, const std::vector<uint8_t> &params) override;

private:
    // H05R0x - 1S Lipo Charger w/ USB-C
    BOSStatus handleH05R0_CellVoltageCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH05R0_CellCurrentCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH05R0_CellPowerCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH05R0_CellTemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH05R0_CellCapacityCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH05R0_StateofChargeCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH05R0_CellAgeCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH05R0_CellCyclesCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);

    // H08R7 - TOF
    BOSStatus handleH08R7_DistanceCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);

    // H09R9 - Thermobile
    BOSStatus handleH09R9_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);

    // H0AR9 - Sensor Hub
    BOSStatus handleH0AR9_ColorCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH0AR9_DistanceCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH0AR9_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH0AR9_HumidityCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH0AR9_PIRCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);

    // H0BR4 - IMU
    BOSStatus handleH0BR4_GyroCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH0BR4_AccCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH0BR4_MagCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH0BR4_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);

    // H1FR5 - GPS
    BOSStatus handleH1FR5_PositionCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH1FR5_UTCCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH1FR5_SpeedCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH1FR5_HeightCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);

    // H2AR3 AC Monitor
    BOSStatus handleH2AR3_VoltCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH2AR3_CurrentCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);

    // H2BR1 SPO2
    BOSStatus handleH2BR1_HRCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleH2BR1_SPO2Code(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params);



};

/* Messaging APIs Class Definitions ***************************************************************/
namespace Messaging
{
    // public:
    BOSStatus SendMessagetoModule(uint8_t dstID, BOSMessageCode code, const std::vector<uint8_t> &params);
    BOSStatus SendLargMessagetoModule(uint8_t dstID, BOSMessageCode code, const std::vector<uint8_t> &data);
    BOSStatus SendDataRequestToModule(uint8_t dstID, BOSMessageCode code);
};

/* External Class Instances  Definitions **********************************************************/
extern LED led;
extern BOSOptionByte_t OptionByte;
extern RawData_t rawData;

/**************************************************************************************************/
/******************************************  General Functions ************************************/
/**************************************************************************************************/
void initBOS(void);
void DisplayTopology(void);
std::string to_string(ModulePN pn);
