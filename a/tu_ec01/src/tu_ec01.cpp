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
const uint8_t loggingIntervaldef = loggingInterval_CDEF_MIN;

// ==========================================================================
//     Local storage - evolving
// ==========================================================================
#ifdef USE_MS_SD_INI
persistent_store_t ps_ram;
#define epc ps_ram
#endif  //#define USE_MS_SD_INI


// ==========================================================================
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

#define EC_RELATIVE_OHMS 100000
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
    new ProcessorStats_Battery(&mcuBoardPhy), new MaximDS3231_Temp(&ds3231),
#if defined AnalogProcEC_ACT
    // Do Analog processing measurements.
    new AnalogElecConductivityM_EC(&analogEC_phy, EC1_UUID),
#endif  // AnalogProcEC_ACT

#if defined(ExternalVoltage_Volt1_UUID)
    new ExternalVoltage_Volt(&extvolt1, ExternalVoltage_Volt1_UUID),
#endif
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

    Serial.print(F("Sw Name: "));
    Serial.println(configDescription);

    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    dataLogger.startFixedWatchdog();
    // Set up pins for the LED's - LOW is ON
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();
    
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

    //set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Begin the variable array[s], logger[s], and publisher[s]
    varArray.begin(variableCount, variableList);
    dataLogger.begin(LoggerID, loggingIntervaldef, &varArray);

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
