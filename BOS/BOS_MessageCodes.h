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
    CODE_H05R0_CELL_VOLTAGE = 250,
    CODE_H05R0_CELL_CURRENT = 251,
    CODE_H05R0_CELL_POWER = 252,
    CODE_H05R0_CELL_TEMPERATURE = 253,
    CODE_H05R0_CELL_CAPACITY = 254,
    CODE_H05R0_STATE_OF_CHARGE = 255,
    CODE_H05R0_CELL_AGE = 258,
    CODE_H05R0_CELL_CYCLES = 259,

    // H08R7x TOF (VL53L1)
    CODE_H08R7_SAMPLE_DISTANCE = 400,

    // H09R0 - Thermocouple Temperature Sensor

    // H09R9 - Thermopile Sensor
    CODE_H09R9_SAMPLE_TEMP = 475,

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
    CODE_H16R6_SET_COLOR = 1100,
    CODE_H16R6_SET_ALL_COLOR = 1101,
    CODE_H16R6_SET_RGB = 1102,
    CODE_H16R6_SET_ALL_RGB = 1103,
    CODE_H16R6_SET_LED_OFF = 1104,
    CODE_H16R6_SETALLLEDOFF = 1105,
    CODE_H16R6_SETLEDON = 1106,
    CODE_H16R6_SET_ALL_LED_ON = 1107,
    CODE_H16R6_SCROLL_MODE = 1108,
    CODE_H16R6_FLASH_MODE = 1109,
    CODE_H16R6_COLOR_PICKER_MODE = 1110,
    CODE_H16R6_SET_COLOR_SOME_LED = 1111,
    CODE_H16R6_MOTION_MODE = 1112,
    CODE_H16R6_CROSS_FADE_MODE = 1113,
    CODE_H16R6_CROSS_FADE_MODE_LED_RGB = 1114,
    CODE_H16R6_CROSS_FADE_MODE_ALL_LED_RGB = 1115,
    CODE_H16R6_SPRINKLE_MODE = 1116,

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
    CODE_H2AR3_SAMPLE_CURRENT = 2101,

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
