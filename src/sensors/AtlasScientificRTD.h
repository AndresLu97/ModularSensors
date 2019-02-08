/*
 *AtlasScientificRTD.h
 *This file is part of the EnviroDIY modular sensors library for Arduino
 *
 *Initial developement for Atlas Sensors was done by Adam Gold
 *Files were edited by Sara Damiano
 *
 * The output from the Atlas Scientifc RTD is the range in degrees C.
 *     Accuracy is ± __
 *     Range is __
 *
 * Warm up time to completion of header:  __ ms
 */

// Header Guards
#ifndef AtlasScientificRTD_h
#define AtlasScientificRTD_h

// Debugging Statement
// #define DEBUGGING_SERIAL_OUTPUT Serial

// Included Dependencies
#include "VariableBase.h"
#include "sensors/AtlasParent.h"

// I2C address
#define ATLAS_RTD_I2C_ADDR 0x66  // 102

// Sensor Specific Defines
#define ATLAS_RTD_NUM_VARIABLES 1
#define ATLAS_RTD_WARM_UP_TIME_MS 0
#define ATLAS_RTD_STABILIZATION_TIME_MS 0
#define ATLAS_RTD_MEASUREMENT_TIME_MS 0

#define ATLAS_RTD_RESOLUTION 3
#define ATLAS_RTD_VAR_NUM 0

// The main class for the Atlas Scientific RTD temperature sensor
class AtlasScientificRTD : public AtlasParent
{
public:
    AtlasScientificRTD(int8_t powerPin, uint8_t i2cAddressHex = ATLAS_RTD_I2C_ADDR, uint8_t measurementsToAverage = 1)
     : SDI12Sensors(powerPin, i2cAddressHex, measurementsToAverage,
                    "AtlasScientificDO", ATLAS_RTD_NUM_VARIABLES,
                    ATLAS_RTD_WARM_UP_TIME_MS, ATLAS_RTD_STABILIZATION_TIME_MS, ATLAS_RTD_MEASUREMENT_TIME_MS)
    ~AtlasScientificRTD();
};

// The class for the Temp Variable
class AtlasScientificRTD_Temp : public Variable
{
public:
    AtlasScientificRTD_Temp(Sensor *parentSense,
                        const char *UUID = "", const char *customVarCode = "")
      : Variable(parentSense, ATLAS_RTD_VAR_NUM,
                 "temperature", "degreeCelsius",
                 ATLAS_RTD_RESOLUTION,
                 "AtlasTemp", UUID, customVarCode)
    {}
    ~AtlasScientificRTD_Temp(){}
};

#endif  // Header Guard
