/** =========================================================================
 * @file tu_ec01.cpp
 * @brief A simple relative EC logging program.
 *
 * @author Neil Hancock based on the example simple_logger 
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @copyright (c) 2017-2020 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 *
 * Build Environment: Visual Studios Code with PlatformIO
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */

// ==========================================================================
//  Defines for the Arduino IDE
//  NOTE:  These are ONLY needed to compile with the Arduino IDE.
//         If you use PlatformIO, you should set these build flags in your
//         platformio.ini
// ==========================================================================
/** Start [defines] */
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 64
#endif
#ifndef TINY_GSM_YIELD_MS
#define TINY_GSM_YIELD_MS 2
#endif
/** End [defines] */

// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
/** Start [includes] */
#include "ms_cfg.h"  //must be before ms_common.h & Arduino.h
#ifdef MS_TU_EC_DEBUG
#undef MS_DEBUGGING_STD
#define MS_DEBUGGING_STD "tu_ec"
#define MS_DEBUG_THIS_MODULE 1
#endif  // MS_TU_EC_DEBUG

#ifdef MS_TU_EC_DEBUG_DEEP
#undef MS_DEBUGGING_DEEP
#define MS_DEBUGGING_DEEP "tu_ecD"
#undef MS_DEBUG_THIS_MODULE
#define MS_DEBUG_THIS_MODULE 2
#endif  // MS_TU_EC_DEBUG_DEEP
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#undef MS_DEBUGGING_DEEP
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
#include <EnableInterrupt.h>

// Include the main header for ModularSensors
#include <ModularSensors.h>
#if defined USE_PS_EEPROM
#include "EEPROM.h"
#endif  // USE_PS_EEPROM
#include "ms_common.h"
/** End [includes] */

// ==========================================================================
//  Data Logging Options
// ==========================================================================
// The name of this file
extern const String build_ref = "examples\\" __FILE__ " " __DATE__ " " __TIME__ " ";
#ifdef PIO_SRC_REV
const char git_branch[] = PIO_SRC_REV;
#else
const char git_branch[] = ".";
#endif
#ifdef PIO_SRC_USR
const char git_usr[] = PIO_SRC_USR;
#else
const char git_usr[] = "usr";
#endif

/** Start [logging_options] */
// The name of this program file
//const char* sketchName = "simple_logging.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = LOGGERID_DEF_STR;
const char* configIniID_def   = configIniID_DEF_STR;
const char* configDescription = CONFIGURATION_DESCRIPTION_STR;

// How frequently (in minutes) to log data
const uint8_t loggingIntervaldef = loggingInterval_CDEF_MIN;

// ==========================================================================
//     Local storage - evolving
// ==========================================================================
#ifdef USE_MS_SD_INI
persistent_store_t ps_ram;
#define epc ps_ram
#endif  //#define USE_MS_SD_INI

//  Wifi/Cellular Modem Options
// ==========================================================================
/** Start [xbee_cell_transparent] */
// For any Digi Cellular XBee's
// NOTE:  The u-blox based Digi XBee's (3G global and LTE-M global)
// are more stable used in bypass mode (below)
// The Telit based Digi XBees (LTE Cat1) can only use this mode.
#include <modems/DigiXBeeCellularTransparent.h>
// Create a reference to the serial port for the modem
HardwareSerial& modemSerial = Serial1;  // Use hardware serial if possible
const int32_t   modemBaud   = 9600;     // All XBee's use 9600 by default

// Modem Pins - Describe the physical pin connection of your modem to your board
// NOTE:  Use -1 for pins that do not apply
const int8_t modemVccPin    = 18;    // MCU pin controlling modem power
const int8_t modemStatusPin = 19;    // MCU pin used to read modem status
const bool useCTSforStatus = false;  // Flag to use the modem CTS pin for status
const int8_t modemResetPin = 20;     // MCU pin connected to modem reset pin
const int8_t modemSleepRqPin = 23;   // MCU pin for modem sleep/wake request
//const int8_t modemLEDPin = redLED;   // MCU pin connected an LED to show modem
                                     // status (-1 if unconnected)
// Network connection information
const char* apn = "hologram";  // The APN for the gprs connection

DigiXBeeCellularTransparent modemXBCT(&modemSerial, modemVccPin, modemStatusPin,
                                      useCTSforStatus, modemResetPin,
                                      modemSleepRqPin, apn);
// Create an extra reference to the modem by a generic name
DigiXBeeCellularTransparent modemPHY = modemXBCT;
/** End [xbee_cell_transparent] */// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <BatteryManagement.h>
BatteryManagement bms;
#include <sensors/ProcessorStats.h>

// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 115200;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t  buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t  wakePin    = 31;  // MCU interrupt/alarm pin to wake from sleep
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
/** End [logging_options] */
// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoardPhy(mcuBoardVersion);
/** End [processor_sensor] */


// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [ds3231] */
#include <sensors/MaximDS3231.h>  // Includes wrapper functions for Maxim DS3231 RTC

// Create a DS3231 sensor object, using this constructor function:
MaximDS3231 ds3231(1);
/** End [ds3231] */

// ==========================================================================
//   Analog Electrical Conductivity using the processors analog pins
// ==========================================================================
#ifdef AnalogProcEC_ACT
/** Start [AnalogElecConductivity] */
#include <sensors/AnalogElecConductivityM.h>
const int8_t ECpwrPin   = ECpwrPin_DEF;
const int8_t ECdataPin1 = ECdataPin1_DEF;

#define EC_RELATIVE_OHMS 2200
AnalogElecConductivityM analogEC_phy(ECpwrPin, ECdataPin1, EC_RELATIVE_OHMS);
/** End [AnalogElecConductivity] */
#endif  // AnalogProcEC_ACT

//#ifdef MAYFLY_BAT_AA0
#if defined ExternalVoltage_Volt0_UUID
// ==========================================================================
//    External Voltage via TI ADS1115
// ==========================================================================
#include <sensors/ExternalVoltage.h>

const int8_t ADSPower = 1;     // sensorPowerPin;  // Pin to switch power on and
                               // off (-1 if unconnected)
const int8_t ADSChannel0 = 0;  // The ADS channel of interest
const int8_t ADSChannel1 = 1;  // The ADS channel of interest
const int8_t ADSChannel2 = 2;  // The ADS channel of interest
const int8_t ADSChannel3 = 3;  // The ADS channel of interest
const float  dividerGain = 11;  // Gain RevR02 1/Gain 1M+100K
// The Mayfly is modified for ECN R04 or divide by 11
// Vbat is expected to be 3.2-4.2, so max V to ads is 0.38V,
// Practically the defaule GAIN_ONE for ADS1115 provide the best performance.
// 2020Nov13 Characterizing the ADS1115 for different gains seems to fall far
// short of the datasheet. Very frustrating.

const uint8_t ADSi2c_addr    = 0x48;  // The I2C address of the ADS1115 ADC
const uint8_t VoltReadsToAvg = 1;     // Only read one sample stable input

// Create an External Voltage sensor object
ExternalVoltage extvolt_AA0(ADSPower, ADSChannel0, dividerGain, ADSi2c_addr,
                         VoltReadsToAvg);
// ExternalVoltage extvolt1(ADSPower, ADSChannel1, dividerGain, ADSi2c_addr,
// VoltReadsToAvg); special Vcc 3.3V

//#define PRINT_EXTADC_BATV_VAR
#if defined PRINT_EXTADC_BATV_VAR
bool userPrintExtBatV_avlb=false;
#endif  // PRINT_EXTADC_BATV_VAR
// Create a capability to read the battery Voltage asynchronously,
// and have that voltage used on logging event
Variable* varExternalVoltage_Volt = new ExternalVoltage_Volt(&extvolt_AA0, "NotUsed");


float wLionBatExt_worker(void) {  // get the Battery Reading
    // Get new reading
   float flLionBatExt_V = varExternalVoltage_Volt->getValue(true);
    // float depth_ft = convert_mtoFt(depth_m);
    // MS_DBG(F("wLionBatExt_worker"), flLionBatExt_V);
#if defined MS_TU_XX_DEBUG
    DEBUGGING_SERIAL_OUTPUT.print(F("  wLionBatExt_worker "));
    DEBUGGING_SERIAL_OUTPUT.print(flLionBatExt_V, 4);
    DEBUGGING_SERIAL_OUTPUT.println();
#endif  // MS_TU_XX_DEBUG
#if defined PRINT_EXTADC_BATV_VAR
    if (userPrintExtBatV_avlb) {
        userPrintExtBatV_avlb = false;
        STANDARD_SERIAL_OUTPUT.print(F("  LiionBatExt(V) "));
        STANDARD_SERIAL_OUTPUT.print(flLionBatExt_V, 4);
        STANDARD_SERIAL_OUTPUT.println();       
    }
#endif  // PRINT_EXTADC_BATV_VAR
    return flLionBatExt_V;
}
float getLionBatExt_V(void) {
    return varExternalVoltage_Volt->getValue(false);
}
// Setup the object that does the operation
Variable* pLionBatExt_var =
    new Variable(wLionBatExt_worker,  // function that does the calculation
                 4,                    // resolution
                 "batteryVoltage",     // var name. This must be a value from
                                    // http://vocabulary.odm2.org/variablename/
                 "volts",  // var unit. This must be a value from This must be a
                           // value from http://vocabulary.odm2.org/units/
                 "extVolt0",  // var code
                 ExternalVoltage_Volt0_UUID);
#endif  // MAYFLY_BAT_AA0

#if defined MAYFLY_BAT_CHOICE
#if MAYFLY_BAT_CHOICE == MAYFLY_BAT_STC3100
#define bms_SetBattery() bms.setBatteryV(wLionBatStc3100_worker());
#elif MAYFLY_BAT_CHOICE == MAYFLY_BAT_AA0 
// Need for internal battery 
#define bms_SetBattery() bms.setBatteryV(wLionBatExt_worker());
#elif  MAYFLY_BAT_CHOICE == MAYFLY_BAT_A6
#warning need to test mcuBoardPhy, interface 
// Read's the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltageProc() {
    if (mcuBoardPhy.sensorValues[0] == PS_SENSOR_INVALID) mcuBoardPhy.update();
    return mcuBoardPhy.sensorValues[0];
}
#define bms_SetBattery() bms.setBatteryV(getBatteryVoltageProc());
#endif  //MAYFLY_BAT_A6
#else 
#warning MAYFLY_BAT_CHOICE not defined
//Leave battery settings at default or off
#define bms_SetBattery()
#endif  //defined MAYFLY_BAT_CHOICE
#if defined ProcVolt_ACT
// ==========================================================================
//    Internal  ProcessorAdc
// ==========================================================================
#include <sensors/processorAdc.h>
const int8_t  procVoltPower      = -1;
const uint8_t procVoltReadsToAvg = 1;  // Only read one sample

#if defined ARDUINO_AVR_ENVIRODIY_MAYFLY
// Only support Mayfly rev5 10M/2.7M &  10bit ADC (3.3Vcc / 1023)
const int8_t sensor_Vbatt_PIN    = A6;
const float  procVoltDividerGain = 4.7;
#else
#error define other processors ADC pins here
#endif  //
processorAdc sensor_batt_V(procVoltPower, sensor_Vbatt_PIN, procVoltDividerGain,
                           procVoltReadsToAvg);
// processorAdc sensor_V3v6_V(procVoltPower, sensor_V3V6_PIN,
// procVoltDividerGain, procVoltReadsToAvg);

#endif  // ProcVolt_ACT

// ==========================================================================
//    Settings for Additional Sensors
// ==========================================================================
// Additional sensors can setup here, similar to the RTC, but only if
//   they have been supported with ModularSensors wrapper functions. See:
//   https://github.com/EnviroDIY/ModularSensors/wiki#just-getting-started
// Syntax for the include statement and constructor function for each sensor is
// at
//   https://github.com/EnviroDIY/ModularSensors/wiki#these-sensors-are-currently-supported
//   or can be copied from the `menu_a_la_carte.ino` example


// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
/** Start [variable_arrays] */
Variable* variableList[] = {
    new ProcessorStats_SampleNumber(&mcuBoardPhy),
    new ProcessorStats_Battery(&mcuBoardPhy), 
    new MaximDS3231_Temp(&ds3231),
#if defined AnalogProcEC_ACT
    // Do Analog processing measurements.
    new AnalogElecConductivityM_EC(&analogEC_phy), // EC1_UUID),
#endif  // AnalogProcEC_ACT

#if defined(ExternalVoltage_Volt1_UUID)
    new ExternalVoltage_Volt(&extvolt1, ExternalVoltage_Volt1_UUID),
#endif
};

// All UUID's, device registration, and sampling feature information can be
// pasted directly from Monitor My Watershed.
// To get the list, click the "View  token UUID list" button on the upper right
// of the site page.

// *** CAUTION --- CAUTION --- CAUTION --- CAUTION --- CAUTION ***
// Check the order of your variables in the variable list!!!
// Be VERY certain that they match the order of your UUID's!
// Rearrange the variables in the variable list ABOVE if necessary to match!
// Do not change the order of the variables in the section below.
// *** CAUTION --- CAUTION --- CAUTION --- CAUTION --- CAUTION ***

// Replace all of the text in the following section with the UUID array from
// MonitorMyWatershed

// ---------------------   Beginning of Token UUID List
// ---------------------------------------


// Need to audit with variables
const char* UUIDs[] =  // UUID array for device sensors
    {
        "8c57835f-a32f-4d62-82dc-0ba09f04cf52",  // Specific conductance
                                                 // (Meter_Hydros21_Cond)
        "1f2c9e91-3aa6-44d7-9312-160d04fbf877",  // Water depth
                                                 // (Meter_Hydros21_Depth)
        "65e0a9e5-cc8a-4ed6-8d28-127b5ec5e8e9",  // Temperature
                                                 // (Meter_Hydros21_Temp)
        "a0e41a66-875a-44fc-9e2f-02c6e25f6063",  // Turbidity
                                                 // (Campbell_OBS3_Turb) (Low)
        "12345678-abcd-1234-ef00-1234567890ab",  // Turbidity
                                                 // (Campbell_OBS3_Turb) (High)
        "12345678-abcd-1234-ef00-1234567890ab",  // Battery voltage
                                                 // (EnviroDIY_Mayfly_Batt)
        "12345678-abcd-1234-ef00-1234567890ab",  // Temperature
                                                 // (Maxim_DS3231_Temp)
        "12345678-abcd-1234-ef00-1234567890ab",  // Percent full scale
                                                 // (Digi_Cellular_SignalPercent)
};
const char* registrationToken =
    "0cf7c40a-232e-457d-87d6-cea5c0757fec";  // Device registration token
const char* samplingFeature =
    "236c674b-69b9-43af-b0d6-33d67b870ecc";  // Sampling feature UUID


// -----------------------   End of Token UUID List
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
  VariableArray varArray(variableCount, variableList, UUIDs);
//VariableArray varArray;
/** End [variable_arrays] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a logger instance
Logger dataLogger(LoggerID, loggingIntervaldef, &varArray);
/** End [loggers] */

// ==========================================================================
//  Creating Data Publisher[s]
// ==========================================================================
/** Start [publishers] */
// Create a data publisher for the Monitor My Watershed/EnviroDIY POST endpoint
#include <publishers/EnviroDIYPublisher.h>
EnviroDIYPublisher EnviroDIYPOST(dataLogger, &modemPHY.gsmClient,
                                 registrationToken, samplingFeature);
/** End [publishers] */
#if defined MS_TTY_USER_INPUT
// ==========================================================================
bool userButton1Act = false;
void userButtonISR() {
    //Need setting up to actiavted by appropiate buttonPin
    MS_DBG(F("ISR userButton!"));
    if (digitalRead(buttonPin)) {
        userButton1Act =true;
    } 

} //userButtonISR

// ==========================================================================
 void setupUserButton () {
    if (buttonPin >= 0) {
        pinMode(buttonPin, INPUT_PULLUP);
        enableInterrupt(buttonPin, userButtonISR, CHANGE);
        MS_DBG(F("Button on pin"), buttonPin,
               F("user input."));
    }
} // setupUserButton
#include "tu_serialCmd.h"
#else 
#define setupUserButton()

#if defined MS_TTY_SERIAL_COUNT

long ch_count_tot=0; 
uint8_t serialInputCount() 
{
    //char incoming_ch;
    uint8_t ch_count_now=0;
 
    //Read any input queue
    while (Serial.available()) {
        //incoming_ch = 
        Serial.read();
        if (++ch_count_now > 250) break;
    }
    ch_count_tot+=ch_count_now;

    return ch_count_now;
}
#endif // MS_TTY_SERIAL_COUNT
#endif //MS_TTY_USER_INPUT

// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
//tbd
#define SerialStd Serial
#include "iniHandler.h"

// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75) {
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}
/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {
    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("\n---Boot Sw Build: "));
    Serial.print(build_ref);
    Serial.print(" ");
    Serial.println(git_branch);
    Serial.print(" ");
    Serial.println(git_usr);

    Serial.print(F("Sw Name: "));
    Serial.println(configDescription);

    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);
    Serial.print(F("TinyGSM Library version "));
    Serial.println(TINYGSM_VERSION);
    Serial.println();

    dataLogger.startFixedWatchdog();
    readAvrEeprom();
#if defined USE_PS_HW_BOOT
    //Print sames as .csv header, used in LoggerBaseExtCpp.h 
    Serial.print(F("Board: "));
    Serial.print((char*)epc.hw_boot.board_name);
    Serial.print(F(" rev:'"));
    Serial.print((char*)epc.hw_boot.rev);
    Serial.print(F("' sn:'"));
    Serial.print((char*)epc.hw_boot.serial_num);
    Serial.println(F("'"));
#endif  // USE_PS_HW_BOOT

    // Set up pins for the LED's - LOW is ON
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();
    pinMode(20, OUTPUT);  // for proper operation of the onboard flash memory
                          // chip's ChipSelect (Mayfly v1.0 and later)
        //set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);
    // Attach the modem and information pins to the logger
    dataLogger.attachModem(modemPHY);
   // modem.setModemLED(modemLEDPin);
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, -1, greenLED);
    setupUserButton(); //used for serialInput

#ifdef USE_MS_SD_INI
    // Set up SD card access
    PRINTOUT(F("---parseIni Start"));
    dataLogger.setPs_cache(&ps_ram);
    dataLogger.parseIniSd(configIniID_def, inihUnhandledFn);
    epcParser(); //use ps_ram to update classes
    PRINTOUT(F("---parseIni complete\n"));
#endif  // USE_MS_SD_INI


    // Begin the variable array[s], logger[s], and publisher[s]
    varArray.begin(variableCount, variableList);
    dataLogger.begin();

    // Set up the sensors
    Serial.println(F("Setting up sensors..."));
    varArray.setupSensors();
    
    Serial.println(F("Waking modem and setting Cellular Carrier Options..."));
    modemPHY.modemWake();  // NOTE:  This will also set up the modem
    dataLogger.syncRTC();
    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    dataLogger.createLogFile(true);  // true = write a new header

    // Call the processor sleep
    dataLogger.systemSleep();
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [loop] */
void loop() {
    dataLogger.logData();
}
/** End [loop] */
