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
extern const String build_ref = "a\\" __FILE__ " " __DATE__ " " __TIME__ " ";
#ifdef PIO_SRC_REV
const char git_branch[] = PIO_SRC_REV;
#else
const char git_branch[] = ".";
#endif
/** Start [logging_options] */
// The name of this program file
//const char* sketchName = "simple_logging.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = LOGGERID_DEF_STR;
const char* configIniID_def   = configIniID_DEF_STR;
const char* configDescription = CONFIGURATION_DESCRIPTION_STR;
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 15;
// Your logger's timezone.
const int8_t timeZone = -8;  // PST
// NOTE:  Only supports standard time! Daylight savings varies from year2year

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 115200;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t  buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t  wakePin    = 31;  // MCU interrupt/alarm pin to wake from sleep
// Mayfly 0.x D31 = A7
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
/** End [logging_options] */

// ==========================================================================
//     Local storage - evolving
// ==========================================================================
#ifdef USE_MS_SD_INI
//tbd
persistent_store_t ps_ram;
#define epc ps_ram
#endif  //#define USE_MS_SD_INI

// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);
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

#define EC_RELATIVE_OHMS 100000
AnalogElecConductivityM analogEC_phy(ECpwrPin, ECdataPin1, EC_RELATIVE_OHMS);
/** End [AnalogElecConductivity] */
#endif  // AnalogProcEC_ACT

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
    new ProcessorStats_SampleNumber(&mcuBoard),
    new ProcessorStats_Battery(&mcuBoard), new MaximDS3231_Temp(&ds3231),
    #if defined AnalogProcEC_ACT
    // Do Analog processing measurements.
    new AnalogElecConductivityM_EC(&analogEC_phy, EC1_UUID),
#endif  // AnalogProcEC_ACT
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray;
/** End [variable_arrays] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a logger instance
Logger dataLogger;
/** End [loggers] */


// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
//tbd
//#define SerialStd Serial
//#include "iniHandler.h"

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

    Serial.print(F("Sw Name: "));
    Serial.println(configDescription);

    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    dataLogger.startFixedWatchdog();
    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Set information pins
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);

    // Begin the variable array[s], logger[s], and publisher[s]
    varArray.begin(variableCount, variableList);
    dataLogger.begin(LoggerID, loggingInterval, &varArray);

    // Set up the sensors
    Serial.println(F("Setting up sensors..."));
    varArray.setupSensors();

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
