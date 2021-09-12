/*
 * @file InsituTrollSdi12.h
 * @copyright 2020 Stroud Water Research Center
 * Part of the EnviroDIY modular sensors 
 * @author Neil Hancock  https://github.com/neilh10/ModularSensors/
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Contains the InsituTrollSdi12 subclass of the SDI12Sensors class along
 * with the variable subclasses InsituTrollSdi12_Pressure, InsituTrollSdi12_Temp, 
 * and InsituTrollSdi12_Depth
 *
 * These are used for the Insitu Troll.
 *
 * This depends on the EnviroDIY SDI-12 library and the SDI12Sensors super
 * class.
 *
 */
/* clang-format off */
/**
 * @defgroup sensor_instutroll Insitu LevelTroll 500
 * Classes for the Insitru LevelTroll  pressure, temperature, and depth sensor.
 *
 * @ingroup sdi12_group
 *
 * @tableofcontents
 * @m_footernavigation
 *
 * @section sensor_instutroll_intro Introduction
 *
 * > A slim 1.8 cm diameter sensor, 
 * > accurate, temperature compensated to 0.1% across Full Scale and across temperature range, 
 * > with internal logger for reliable data collection,
 * > temperature, and depth in both groundwater and surface water.
 *
 * The Insitu Aqua/Level Troll require 8-36VDC
 * This can be achieved a Polo #boost device, instructions are at the end
 *
 * @section sensor_instutroll_datasheet Sensor Datasheet
 * Documentation for the SDI-12 Protocol commands and responses
 * The Insitu Level/Aqua Troll can be found at:
 * Insitu SDI-12-Commands-and-Level-TROLL-400-500-700-Responses 20140210.pdf
 * Insitu SDI-12-Commands-and-Aqua-TROLL-100-200-Responses 20070123.pdf
 *
 * I haven't audited why Insitu have a different SDI manual for Level and Aqua
 *
 * For Pressure:
 *  Resolution is 0.001
 *  Accuracy is ±?
 *  Range is 0 – ?
 *
 * For Temperature:
 *  Resolution is 0.1°C
 *  Accuracy is ±1°C
 *  Range is -11°C to +49°C
 *
 * For Depth:
 *  Resolution is 2 mm
 *  Accuracy is ±0.05% of full scale
 *  Range is 0 to 5 m or 0 to 10 m, depending on model
 *
 * Maximum warm-up time in SDI-12 mode: 500ms, assume stability at warm-up
 * Maximum measurement duration: 500ms
 *
 * The Insitu Aqua/Level Trolls are programmed through WinSitu
 * Parameters are very flexible and need to be aligned with this program
 * The SDI address needs to be changed to what the class is set to - default is
 *'1' The depth sensor third paramter needs to be created. The expected
 *paramters and order are 0 Pressure (PSI)   ITROLL_PRESSURE_VAR_NUM 1
 *Temperature (C)  ITROLL_TEMP_VAR_NUM 2 Depth (ft)       ITROLL_DEPTH_VAR_NUM
 *Resolution 0.005% For 11.5ft +/ 0.00005ft
 *
 * @menusnip{instutroll}
 */
/* clang-format on */

// Header Guards
#ifndef SRC_SENSORS_INSITUTROLLSDI12_H_
#define SRC_SENSORS_INSITUTROLLSDI12_H_

// Included Dependencies
#include "sensors/SDI12Sensors.h"

// Sensor Specific Defines
/** @ingroup sensor_insitutroll */
/**@{*/
/// @brief Sensor::_numReturnedValues; the Troll 500 can report 3 values.
#define ITROLL_NUM_VARIABLES 3

/**
 * @anchor sensor_insitutroll_timing
 * @name Sensor Timing
 * The sensor timing for a Insitu Troll
 */
/**@{*/
/// @brief Sensor::_warmUpTime_ms; maximum warm-up time in SDI-12 mode: 500ms
#define ITROLL_WARM_UP_TIME_MS 500


/// @brief Sensor::_stabilizationTime_ms; the Hydros 21 is stable as soon as it
/// warms up (0ms stabilization).
#define ITROLL_STABILIZATION_TIME_MS 0

/// @brief Sensor::_measurementTime_ms; maximum measurement duration: 500ms.
#define ITROLL_MEASUREMENT_TIME_MS 500

/**
 * @anchor sensor_insitutroll_pressure
 * @name Pressure
 * The pressue variable from a Insitu Troll
 * - Range is 0 – x (depends on range eg 5psig)

 *
 * {{ @ref InsituTrollSdi12_Pressure::InsituTrollSdi12_Pressure }}
 */
/**@{*/
/**
 * @brief Decimals places in string representation; conductivity should have 1.
 *
 * 0 are reported, adding extra digit to resolution to allow the proper number
 * of significant figures for averaging - resolution is 0.001 mS/cm = 1 µS/cm
 */
#define ITROLL_PRESSURE_RESOLUTION 5
/// @brief Sensor variable number; pressure is stored in sensorValues[0].
#define ITROLL_PRESSURE_VAR_NUM 0
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "specificConductance"
#define ITROLL_PRESSURE_VAR_NAME "pressureGauge"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "pounds per square inch" (psi)
#define ITROLL_PRESSURE_UNIT_NAME "psi"
/// @brief Default variable short code; "ITROLLpressure"
#define ITROLL_PRESSURE_DEFAULT_CODE "ITROLLpressure"
/**@}*/

/**
 * @anchor sensor_insitutroll_temp
 * @name Temperature
 * The temperature variable from a Insitu Troll
 * - Range is -11°C to +49°C
 * - Accuracy is ±1°C
 *
 * {{ @ref InsituTrollSdi12_Temp::InsituTrollSdi12_Temp }}
 */
/**@{*/
/**
 * @brief Decimals places in string representation; temperature should have 2.
 *
 * 1 is reported, adding extra digit to resolution to allow the proper number
 * of significant figures for averaging  - resolution is 0.1°C
 */
#define ITROLL_TEMP_RESOLUTION 2
/// @brief Sensor variable number; temperature is stored in sensorValues[1].
#define ITROLL_TEMP_VAR_NUM 1
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "temperature"
#define ITROLL_TEMP_TEMP_VAR_NAME "temperature"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "degreeCelsius" (°C)
#define ITROLL_TEMP_TEMP_UNIT_NAME "degreeCelsius"
/// @brief Default variable short code; "ITROLLtemp"
#define ITROL_TEMP_DEFAULT_CODE "ITROLLtemp"
/**@}*/

/**
 * @anchor sensor_insitutroll_depth
 * @name Water Depth
 * The water depth variable from a Insitu Troll
 * - Range is 0 to 5 m or 0 to 10 m, depending on model
 * - Accuracy is ±0.05% of full scale
 *
 * {{ @ref InsituTrollSdi12_Depth::InsituTrollSdi12_Depth }}
 */
/**@{*/
/**
 * @brief Decimals places in string representation; depth should have 1.
 *
 * 0 are reported, adding extra digit to resolution to allow the proper number
 * of significant figures for averaging - resolution is 2 mm
 */
#define ITROLL_DEPTH_RESOLUTION 5
/// @brief Sensor variable number; depth is stored in sensorValues[2].
#define ITROLL_DEPTH_VAR_NUM 2
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "waterDepth"
#define ITROLL_DEPTH_VAR_NAME "waterDepth"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "millimeter"
#define ITROLL_DEPTH_UNIT_NAME "feet"
/// @brief Default variable short code; "ITROLLdepth"
#define ITROLL_DEPTH_DEFAULT_CODE "ITROLLdepth"
/**@}*/


/* clang-format off */
/**
 * @brief The Sensor sub-class for the
 * [Insitu Level/Aqua Troll pressure, temperature, and depth sensor](@ref sensor_insitutroll)
 *
 * @ingroup sensor_insitutroll
 */
/* clang-format on */

class InsituTrollSdi12 : public SDI12Sensors {
 public:
    // Constructors with overloads
    /**
     * @brief Construct a new ITROLL object.
     *
     * The SDI-12 address of the sensor, the Arduino pin controlling power
     * on/off, and the Arduino pin sending and receiving data are required for
     * the sensor constructor.  Optionally, you can include a number of distinct
     * readings to average.  The data pin must be a pin that supports pin-change
     * interrupts.
     *
     * @param SDI12address The SDI-12 address; can be a char,
     * char*, or int.
     * @warning The SDI-12 address **must** be changed from the factory
     * programmed value of "0" before the sensor can be used with
     * ModularSensors!
     * @param powerPin The pin on the mcu controlling power to the sensor.
     * Use -1 if it is continuously powered.
     * - The ITROLL requires a power supply, which can be turned off
     * between measurements
     * @param dataPin The pin on the mcu connected to the data line of the
     * SDI-12 circuit.
     * @param measurementsToAverage The number of measurements to take and
     * average before giving a "final" result from the sensor; optional with a
     * default value of 1.
     */
    InsituTrollSdi12(char SDI12address, int8_t powerPin, int8_t dataPin,
                     uint8_t measurementsToAverage = 1)
        : SDI12Sensors(SDI12address, powerPin, dataPin, measurementsToAverage,
                       "InsituTrollSdi12", ITROLL_NUM_VARIABLES,
                       ITROLL_WARM_UP_TIME_MS, ITROLL_STABILIZATION_TIME_MS,
                       ITROLL_MEASUREMENT_TIME_MS) {}
    InsituTrollSdi12(char* SDI12address, int8_t powerPin, int8_t dataPin,
                     uint8_t measurementsToAverage = 1)
        : SDI12Sensors(SDI12address, powerPin, dataPin, measurementsToAverage,
                       "InsituTrollSdi12", ITROLL_NUM_VARIABLES,
                       ITROLL_WARM_UP_TIME_MS, ITROLL_STABILIZATION_TIME_MS,
                       ITROLL_MEASUREMENT_TIME_MS) {}
    InsituTrollSdi12(int SDI12address, int8_t powerPin, int8_t dataPin,
                     uint8_t measurementsToAverage = 1)
        : SDI12Sensors(SDI12address, powerPin, dataPin, measurementsToAverage,
                       "InsituTrollSdi12", ITROLL_NUM_VARIABLES,
                       ITROLL_WARM_UP_TIME_MS, ITROLL_STABILIZATION_TIME_MS,
                       ITROLL_MEASUREMENT_TIME_MS) {}
    /**
     * @brief Destroy the ITROL object
     */
    ~InsituTrollSdi12() {}
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [pressure output](@ref sensor_insitutroll_pressure) from a
 * [Insitu Troll 3-in-1 water level sensor.](@ref sensor_insitutroll)
 *
 * @ingroup sensor_insitutroll
 */
/* clang-format on */
class InsituTrollSdi12_Pressure : public Variable {
 public:
    /**
     * @brief Construct a new InsituTrollSdi12_Pressure object.
     *
     * @param parentSense The parent InsituTrollSdi12 providing the result values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "ITROLLPressure".
     */
    InsituTrollSdi12_Pressure(
        Sensor* parentSense, const char* uuid = "",
        const char* varCode = ITROLL_PRESSURE_DEFAULT_CODE)
        : Variable(parentSense, (const uint8_t)ITROLL_PRESSURE_VAR_NUM,
                   (uint8_t)ITROLL_PRESSURE_RESOLUTION, ITROLL_PRESSURE_VAR_NAME,
                   ITROLL_PRESSURE_UNIT_NAME, varCode, uuid) {}
    /**
     * @brief Construct a new InsituTrollSdi12_Pressure object.
     *
     * @note This must be tied with a parent InsituTrollSdi12 before it can be
     * used.
     */
    InsituTrollSdi12_Pressure()
        : Variable((const uint8_t)ITROLL_PRESSURE_VAR_NUM,
                   (uint8_t)ITROLL_PRESSURE_RESOLUTION, ITROLL_PRESSURE_VAR_NAME,
                   ITROLL_PRESSURE_UNIT_NAME, ITROLL_PRESSURE_DEFAULT_CODE) {}
    /**
     * @brief Destroy the InsituTrollSdi12_Pressure object - no action needed.
     */
    ~InsituTrollSdi12_Pressure() {}
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [temperature Output](@ref sensor_insitutroll_temp) from a
 * [Insitu Troll 3-in-1 water level sensor.](@ref sensor_insitutroll)
 *
 * @ingroup sensor_insitutroll
 */
/* clang-format on */
class InsituTrollSdi12_Temp : public Variable {
 public:
    /**
     * @brief Construct a new InsituTrollSdi12_Temp object.
     *
     * @param parentSense The parent InsituTrollSdi12 providing the result values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "ITROLLtemp".
     */
    InsituTrollSdi12_Temp(Sensor* parentSense, const char* uuid = "",
                          const char* varCode = ITROL_TEMP_DEFAULT_CODE)
        : Variable(parentSense, (const uint8_t)ITROLL_TEMP_VAR_NUM,
                   (uint8_t)ITROLL_TEMP_RESOLUTION, ITROLL_TEMP_TEMP_VAR_NAME,
                   ITROLL_TEMP_TEMP_UNIT_NAME, varCode, uuid) {}

    /**
     * @brief Construct a new InsituTrollSdi12_Temp object.
     *
     * @note This must be tied with a parent InsituTrollSdi12 before it can be
     * used.
     */
    InsituTrollSdi12_Temp()
        : Variable((const uint8_t)ITROLL_TEMP_VAR_NUM,
                   (uint8_t)ITROLL_TEMP_RESOLUTION, ITROLL_TEMP_TEMP_VAR_NAME,
                   ITROLL_TEMP_TEMP_UNIT_NAME, ITROL_TEMP_DEFAULT_CODE) {}
    /**
     * @brief Destroy the InsituTrollSdi12_Temp object - no action needed.
     */
    ~InsituTrollSdi12_Temp() {}
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [depth output](@ref sensor_insitutroll_depth) from a
 * [Insitu Troll 3-in-1 water level sensor.](@ref sensor_insitutroll)
 *
 * @ingroup sensor_insitutroll
 */
/* clang-format on */
class InsituTrollSdi12_Depth : public Variable {
 public:
    /**
     * @brief Construct a new InsituTrollSdi12_Depth object.
     *
     * @param parentSense The parent InsituTrollSdi12 providing the result values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "ITROLLdepth".
     */
    InsituTrollSdi12_Depth(Sensor* parentSense, const char* uuid = "",
        const char* varCode = ITROLL_DEPTH_DEFAULT_CODE)
        : Variable(parentSense, (const uint8_t)ITROLL_DEPTH_VAR_NUM,
                   (uint8_t)ITROLL_DEPTH_RESOLUTION, ITROLL_DEPTH_VAR_NAME,
                   ITROLL_DEPTH_UNIT_NAME, varCode, uuid) {}
    /**
     * @brief Construct a new InsituTrollSdi12_Depth object.
     *
     * @note This must be tied with a parent InsituTrollSdi12 before it can be
     * used.
     */
    InsituTrollSdi12_Depth()
        : Variable((const uint8_t)ITROLL_DEPTH_VAR_NUM,
                   (uint8_t)ITROLL_DEPTH_RESOLUTION, ITROLL_DEPTH_VAR_NAME,
                   ITROLL_DEPTH_UNIT_NAME, ITROLL_DEPTH_DEFAULT_CODE) {}
    /**
     * @brief Destroy the InsituTrollSdi12_Depth object - no action needed.
     */
    ~InsituTrollSdi12_Depth() {}
};
/**@}*/
#endif  // SRC_SENSORS_INSITUTROLLSDI12_H_
