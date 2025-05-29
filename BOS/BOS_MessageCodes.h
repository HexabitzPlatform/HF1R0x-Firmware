#pragma once

#include <cstdint>

// Enum class to represent BOS message codes clearly and safely
enum class BOSMessageCode : uint16_t
{

    /***************************************************************************/
    /* BOS Message Codes *******************************************************/
    /***************************************************************************/
    CODE_UNKNOWN_MESSAGE = 0,
    CODE_PING = 1,
    CODE_CODE_IND_ON = 3,
    CODE_IND_OFF = 4,
    CODE_IND_TOGGLE = 5,

    CODE_HI = 10,
    CODE_HI_RESPONSE = 11,
    CODE_EXPLORE_ADJ = 12,
    CODE_EXPLORE_ADJ_RESPONSE = 13,
    CODE_PORT_DIRECTION = 14,
    CODE_MODULE_IDE = 16,
    CODE_TOPOLOGY = 17,

    CODE_READ_REMOTE = 30,
    CODE_READ_REMOTE_RESPONSE = 31,
    CODE_WRITE_REMOTE = 32,
    CODE_WRITE_REMOTE_RESPONSE = 33,

    CODE_READ_RESPONSE = 46,

    ENABLE_STOP_MODE_UARTX = 47,

    /***************************************************************************/
    /* Module Message Codes ****************************************************/
    /***************************************************************************/
    /* Reserve 50 messages for each PN based on its decimal value **************/

    // H01R0x - RGB
    CODE_H01R0_ON = 100,
    CODE_H01R0_OFF = 101,
    CODE_H01R0_TOGGLE = 102,
    CODE_H01R0_COLOR = 103,
    CODE_H01R0_PULSE = 104,
    CODE_H01R0_SWEEP = 105,
    CODE_H01R0_DIM = 106,

    // H05R0x - 1S Lipo Charger w/ USB-C
    CODE_H05R0_CELLVOLTAGE = 250,
    CODE_H05R0_CELLCURRENT = 251,
    CODE_H05R0_CELLPOWER = 252,
    CODE_H05R0_CELLTEMPERATURE = 253,
    CODE_H05R0_CELLCAPACITY = 254,
    CODE_H05R0_STATEOFCHARGE = 255,
    CODE_H05R0_CELLAGE = 258,
    CODE_H05R0_CELLCYCLES = 259,

    // H08R7x TOF (VL53L1)
    CODE_H08R7_SAMPLE_PORT = 400,

    // H09R0 - Thermocouple Temperature Sensor
    CODE_H09R0_STREAM_PORT_C = 450,
    CODE_H09R0_STREAM_PORT_F = 451,
    CODE_H09R0_STREAM_PORT_K = 452,
    CODE_H09R0_SAMPLE_PORT_C = 453,
    CODE_H09R0_SAMPLE_PORT_F = 454,
    CODE_H09R0_SAMPLE_PORT_K = 455,
    CODE_H09R0_STOP = 456,

    // H09R9 - Thermopile Sensor
    CODE_H09R9_SAMPLE_TEMP = 475,
    CODE_H09R9_STREAM_TEMP = 476,
    CODE_H09R9_STREAM_STOP = 477,

    // H0AR9 - Sensor Hub
    CODE_H0AR9_SAMPLE_COLOR = 500,
    CODE_H0AR9_SAMPLE_DISTANCE = 501,
    CODE_H0AR9_SAMPLE_TEMP = 502,
    CODE_H0AR9_SAMPLE_HUMIDITY = 503,
    CODE_H0AR9_SAMPLE_PIR = 504,

    // H0BR4 - IMU
    CODE_H0BR4_SAMPLE_GYRO = 550,
    CODE_H0BR4_SAMPLE_ACC = 551,
    CODE_H0BR4_SAMPLE_MAG = 552,
    CODE_H0BR4_SAMPLE_TEMP = 553,

    // H16R6 - RGB Led Matrix Display
    CODE_H16R6_SETCOLOR = 1100,
    CODE_H16R6_SETALLCOLOR = 1101,
    CODE_H16R6_SETRGB = 1102,
    CODE_H16R6_SETALLRGB = 1103,
    CODE_H16R6_SETLEDOFF = 1104,
    CODE_H16R6_SETALLLEDOFF = 1105,
    CODE_H16R6_SETLEDON = 1106,
    CODE_H16R6_SETALLLEDON = 1107,
    CODE_H16R6_SCROLLMODE = 1108,
    CODE_H16R6_FLASHMODE = 1109,
    CODE_H16R6_COLORPICKERMODE = 1110,
    CODE_H16R6_SETCOLORSOMELED = 1111,
    CODE_H16R6_MOTIONMODE = 1112,
    CODE_H16R6_CROSSFADEMODE = 1113,
    CODE_H16R6_CROSSFADEMODELEDRGB = 1114,
    CODE_H16R6_CROSSFADEMODEALLLEDRGB = 1115,
    CODE_H16R6_SPRINKLEMODE = 1116,

    // H17R1 - Stepper Driver
    CODE_H17R1_StepperIcInit = 1150,
    CODE_H17R1_STEPPER_MOVE = 1151,
    CODE_H17R1_StepperRun = 1152,
    CODE_H17R1_StepperStop = 1153,

    // H18R1 - Dual H-Bridge Motor Driver
    CODE_H18R1_Turn_ON = 1200,
    CODE_H18R1_Turn_OFF = 1201,
    CODE_H18R1_Turn_PWM = 1202,

    // H21R2x - ESP32-C3 WiFi+BLE
    CODE_H21R2_ESP_RESET = 1650,
    CODE_H21R2_ESP_BOOT = 1651,
    CODE_H21R2_ESP_SERVER = 1652,
    CODE_H21R2_ESP_CLIENT = 1653,
    CODE_H21R2_ESP_ACCESS_POINT = 1654,
    CODE_H21R2_ESP_STATION = 1655,
    CODE_H21R2_ESP_READ_FROM_SERVER = 1656,
    CODE_H21R2_ESP_WRITE_TO_SERVER = 1657,
    CODE_H21R2_ESP_READ_FROM_CLIENT = 1658,
    CODE_H21R2_ESP_WRITE_TO_CLIENT = 1659,

    // H2AR3 - AC Current and Voltage Sensor
    CODE_H2AR3_SAMPLE_VOLT = 2100,
    CODE_H2AR3_SAMPLE_CURR = 2101,

    // H3BR6 and H3BR7 - 6 Digits Seven Segment
    CODE_H3BRx_SevenDisplayNumber = 2950,
    CODE_H3BRx_SevenDisplayNumberF = 2951,
    CODE_H3BRx_SevenDisplayQuantities = 2952,
    CODE_H3BRx_SevenDisplayLetter = 2953,
    CODE_H3BRx_SevenDisplaySentence = 2954,
    CODE_H3BRx_SevenDisplayMovingSentence = 2955,
    CODE_H3BRx_SevenDisplayOff = 2956,
    CODE_H3BRx_SetIndicator = 2957,
    CODE_H3BRx_ClearIndicator = 2958,

    // H3BR2x - 2 Digits Seven Segment
    CODE_H3BR2_SevenDisplayNumber = 2975,
    CODE_H3BR2_SevenDisplayNumberHexa = 2976,
    CODE_H3BR2_SevenDisplayOneDigit = 2977,
    CODE_H3BR2_SevenDisplayOneDigitHexa = 2978,
    CODE_H3BR2_SevenDisplayOff = 2979,
    CODE_H3BR2_SevenDisplayNumberF = 2980
};
