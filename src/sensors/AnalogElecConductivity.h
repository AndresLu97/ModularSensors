/**
 * @file AnalogElecConductivity.h
 * @copyright 2020 Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library
 * @copyright 2020 Neil Hancock
 * @author Written By: Neil Hancock <neilh20+aec2008@wLLw.net>; Edited by Sara
 * Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 *
 * @brief Encapsulates an Electrical Conductivity sensors using an anlog
 * input and onboard ADC and ADC ref.
 */
/* clang-format off */
/**
 * @defgroup sensor_analog_cond Conductivity via Analog Electrical Resistance
 * Classes for measuring conductivity using a simple analog voltage divider.
 *
 * @ingroup the_sensors
 *
 * @tableofcontents
 * @m_footernavigation
 *
 * @section sensor_analog_cond_notes Introduction
 * This is for a very basic conductivity circuit built with a single resistor
 * and an old power cord.  DC power is briefly supplied across the power cord
 * causing the water to act as one of the resistors on a voltage divider.
 * Knowing the voltage of the other resistor in the divider, we can calculate to
 * resistance from the water (and then its electrical conductivity) based on the
 * drop in volage across the divider.
 *
 * For this to work, the power across the circuit **MUST** be turned off between
 * readings.  If the power to the circuit is left on the water will become
 * polarized and the values will not be valid. The water temperature (if used)
 * must be suplied separately for a calculation.
 *
 * @section sensor_analog_cond_circuit The Circuit
 * One pole of the power cord wire is connected to the ground of the main logger
 * board.  The other pole is connected to the sensor power supply via a resistor
 * of known resistance (R1) and then to an analog pin to measure the voltage.
 *
 * So the circuit is:
 *
 *  Vin (sensor power) ---- R1 ---- power cord  ---- Vout
 *                                       |
 *                                       |
 *                          water between prongs (Rwater)
 *                                       |
 *                                       |
 *                                    ground
 *
 * The above diagram and the calculations assume the reistance of the analog
 * pins themselves on the Arduino is negligible.
 *
 * @section sensor_analog_cond_calcs Calculating the Conductivity
 * First, we need to convert the bit reading of the ADC into volts based on the
 * range of the ADC (1 bit more than the resolution):
 *
 * meas_voltage = (analog_ref_voltage * raw_adc_bits) / ANALOG_EC_ADC_RANGE
 *
 * Assuming the voltage of the ADC reference is the same as that used to power
 * the EC resistor circuit we can replace the reference voltage with the sensor
 * power voltage:
 *
 * meas_voltage = (sensor_power_voltage * raw_adc_bits) / ANALOG_EC_ADC_RANGE
 *
 * @note The Vcc going to the circuit (~3.3V) can and will vary, as battery
 * level gets low.  If possible, you should use setup the processor to use an
 * external reference and tie the Aref pin to the sensor power pin.
 *
 * @note The analog reference of the Mayfly is not broken out (and is tied to
 * ground).  If using a Mayfly, you have no choice by to use the internal analog
 * reference.
 *
 * Now we can calculate the resistance of the water, knowing the resistance of
 * the resistor we put in the circuit and the voltage drop:
 *
 * Rwater_ohms = (meas_voltage * Rseries_ohms) / (sensor_power_voltage - meas_voltage);
 *
 * Combining the above equations and doing some rearranging, we get:
 *
 * Rwater_ohms = Rseries_ohms / ((ANALOG_EC_ADC_RANGE / raw_adc_bits) - 1)
 *
 * The conductivity is then the inverse of the resistance - multiplied by a
 * measured cell constant and a 10^6 conversion to µS/cm.
 *
 * water_conductivity = 1000000 / (Rwater_ohms * sensorEC_Konst)
 *
 * The real cell constant will vary based on the size of the "cell" - that is,
 * the size of the plug on the power cord.  You can calculate the cell constant
 * for each power cord sensor you use following the calibration program here:
 * https://hackaday.io/project/7008-fly-wars-a-hackers-solution-to-world-hunger/log/24646-three-dollar-ec-ppm-meter-arduino.
 * For one AC Power Cord 12t with male IEC 320-C8 connector the cell constant
 * was 2.88.
 *
 * @note These calulations are for the on-board processor ADC, not an external
 * ACD like the TI ADS1115 built into the Mayfly!
 *
 * @note The return from this "sensor" is conductivity - not the typically
 * reported specific conductance, which is referenced to 25°C.  The temperature
 * compensation, if desired, should be done via a calculated variable.  See the
 * example code below as a guide.
 *
 * @section sensor_analog_cond_ref References
 * For the sensor setup and calculations:
 * https://hackaday.io/project/7008-fly-wars-a-hackers-solution-to-world-hunger/log/24646-three-dollar-ec-ppm-meter-arduino
 * For temperature compensation:
 * https://link.springer.com/article/10.1023/B:EMAS.0000031719.83065.68
 *
 * @section sensor_atlas_cond_flags Build flags
 * - `-D ANALOG_EC_ADC_RESOLUTION=##`
 *      - used to set the resolution of the processor ADC
 *      - @see #ANALOG_EC_ADC_RESOLUTION
 * - `-D ANALOG_EC_ADC_REFERENCE_MODE=xxx`
 *      - used to set the processor ADC value reference mode
 *      - @see #ANALOG_EC_ADC_REFERENCE_MODE
 *
 * @section sensor_analog_cond_ctor Sensor Constructors
 * {{ @ref AnalogElecConductivity::AnalogElecConductivity }}
 *
 * ___
 * @section sensor_analog_cond_examples Example Code
 * The analog electrical conductivity sensor is used in the
 * @menulink{analog_cond} example.
 *
 * @menusnip{analog_cond}
 *
 */
/* clang-format on */

// Header Guards
#ifndef SRC_SENSORS_ANALOGELECCONDUCTIVITY_H_
#define SRC_SENSORS_ANALOGELECCONDUCTIVITY_H_

#ifdef MS_ANALOGELECCONDUCTIVITY_DEBUG
#define MS_DEBUGGING_STD "AnalogElecConductivity"
#endif
#ifdef MS_ANALOGELECCONDUCTIVITY_DEBUG_DEEP
#define MS_DEBUGGING_DEEP "AnalogElecConductivity"
#endif
// Included Dependencies
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#undef MS_DEBUGGING_DEEP
#include "SensorBase.h"
#include "VariableBase.h"
#include "math.h"

// Sensor Specific Defines
/** @ingroup sensor_analog_cond */
/**@{*/

#define ANALOGELECCONDUCTIVITY_NUM_VARIABLES 1

/**
 * @anchor sensor_analog_cond_parts_timing
 * @name Sensor Timing
 * The timing for analog conductivity via resistance.
 */
/// @brief Sensor::_warmUpTime_ms; giving 2ms for warm-up.
#define ANALOGELECCONDUCTIVITY_WARM_UP_TIME_MS 2
/// @brief Sensor::_stabilizationTime_ms; we give just a tiny delay for
/// stabilization.
#define ANALOGELECCONDUCTIVITY_STABILIZATION_TIME_MS 1
/**
 * @brief Sensor::_measurementTime_ms; we assume the analog voltage is measured
 * instantly.  It's not really *quite* instantly, but it is very fast and the
 * time to measure is included in the read function.
 * On ATmega based boards (UNO, Nano, Mini, Mega), it takes about 100
 * microseconds (0.0001 s) to read an analog input, so the maximum reading rate
 * is about 10,000 times a second.
 */
#define ANALOGELECCONDUCTIVITY_MEASUREMENT_TIME_MS 0
/**@}*/


/**
 * @anchor sensor_analog_cond_parts_ec
 * @name Electrical Conductance
 * The humidity variable from an AOSong DHT
 * - Range: low 100's when open air, for short circuit: a high number
 * - Accuracy: needs determining for each combination of ADC. ADC_REF, and
 * series R. its designed as a very simple relative EC measurement
 *
 * {{ @ref AnalogElecConductivity_EC::AnalogElecConductivity_EC }}
 */
/**@{*/
/**
 * @brief Decimals places in string representation; EC should have 1
 *
 * Range of 0-3V3 with 10bit ADC - resolution of 0.003 = 3 µS/cm.
 */
#define ANALOGELECCONDUCTIVITY_EC_RESOLUTION 1
/// @brief Variable number; EC is stored in sensorValues[0].
#define ANALOGELECCONDUCTIVITY_EC_VAR_NUM 0
/// @brief Variable name; "electricalConductivity"
#define ANALOGELECCONDUCTIVITY_EC_VAR_NAME "electricalConductivity"
/// @brief Variable unit name; "microsiemenPerCentimeter" (µS/cm)
#define ANALOGELECCONDUCTIVITY_EC_UNIT_NAME "microsiemenPerCentimeter"
/// @brief Default variable short code; "anlgEc"
#define ANALOGELECCONDUCTIVITY_EC_DEFAULT_CODE "anlgEc"
/**@}*/

#if !defined ANALOG_EC_ADC_RESOLUTION
/**
 * @brief Default resolution (in bits) of the voltage measurement
 *
 * This is the default for all boards, use a build flag to change this, if
 * necessary.
 */
#define ANALOG_EC_ADC_RESOLUTION 10
#endif  // ANALOG_EC_ADC_RESOLUTION
/// @brief The maximum possible value of the ADC - one less than the resolution
/// shifted up one bit.
#define ANALOG_EC_ADC_MAX ((1 << ANALOG_EC_ADC_RESOLUTION) - 1)
/// @brief The maximum possible range of the ADC - the resolution shifted up one
/// bit.
#define ANALOG_EC_ADC_RANGE (1 << ANALOG_EC_ADC_RESOLUTION)

/* clang-format off */
#if !defined ANALOG_EC_ADC_REFERENCE_MODE
#ifdef ARDUINO_ARCH_SAMD
/**
 * @brief The voltage reference mode for the processor's ADC.
 *
 * For a SAMD board, this must be one of:
 * - AR_DEFAULT: the default analog reference of 3.3V
 * - AR_INTERNAL: a built-in 2.23V reference
 * - AR_INTERNAL1V0: a built-in 1.0V reference
 * - AR_INTERNAL1V65: a built-in 1.65V reference
 * - AR_INTERNAL2V23: a built-in 2.23V reference
 * - AR_EXTERNAL: the voltage applied to the AREF pin is used as the reference
 *
 * @see https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
 */
#define ANALOG_EC_ADC_REFERENCE_MODE AR_DEFAULT
#elif defined ARDUINO_ARCH_AVR
/**
 * @brief The voltage reference mode for the processor's ADC.
 *
 * For an AVR board, this must be one of:
 * - DEFAULT: the default analog reference of 5 volts (on 5V Arduino boards)
 * or 3.3 volts (on 3.3V Arduino boards)
 * - INTERNAL: a built-in reference, equal to 1.1 volts on the ATmega168 or
 * ATmega328P and 2.56 volts on the ATmega32U4 and ATmega8 (not available on the
 * Arduino Mega)
 * - INTERNAL1V1: a built-in 1.1V reference (Arduino Mega only)
 * - INTERNAL2V56: a built-in 2.56V reference (Arduino Mega only)
 * - EXTERNAL: the voltage applied to the AREF pin (0 to 5V only) is used as the
 * reference.
 *
 * @see https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
 */
#define ANALOG_EC_ADC_REFERENCE_MODE DEFAULT
#else
#error The processor ADC reference type must be defined!
#endif
#endif  // ANALOG_EC_ADC_REFERENCE_MODE
/* clang-format on */

#if !defined RSERIES_OHMS_DEF
/**
 * @brief The default resistance (in ohms) of the measuring resistor.
 * This should not be less than 300 ohms when measuring EC in water.
 */
#define RSERIES_OHMS_DEF 499
#endif  // RSERIES_OHMS_DEF

#if !defined SENSOREC_KONST_DEF
/**
 * @brief Cell Constant For EC Measurements.
 *
 * This should be measured following the calibration example on
 * https://hackaday.io/project/7008-fly-wars-a-hackers-solution-to-world-hunger/log/24646-three-dollar-ec-ppm-meter-arduino.
 *
 * Mine was around 2.9 with plugs being a standard size they should all be
 * around the same. If you get bad readings you can use the calibration script
 * and fluid to get a better estimate for K.
 */
#define SENSOREC_KONST_DEF 2.88
#endif  // SENSOREC_KONST_DEF

/**
 * @brief Class for the analog Electrical Conductivity monitor
 *
 * @ingroup sensor_analog_cond
 */
class AnalogElecConductivity : public Sensor {
 public:
    /**
     * @brief Construct a new AnalogElecConductivity object.
     *
     * @param powerPin The port pin providing power to the EC probe.
     * Needs to be switched, and assumed to be same V as the dataPin's ADC.
     * @param dataPin The processor ADC port pin to read the voltage from the EC
     * probe.  Not all processor pins can be used as analog pins.  Those usable
     * as analog pins generally are numbered with an "A" in front of the number
     * - ie, A1.
     * @param Rseries_ohms The resistance of the resistor series (R) in the
     * line. Used to calculate the measured value. optional with default value
     * of 499, that maybe overridden with a define/build flag.
     * @param measurementsToAverage The number of measurements to average;
     * optional with default value of 1.
     *
     */
    AnalogElecConductivity(int8_t powerPin, int8_t dataPin,
                           float   Rseries_ohms          = RSERIES_OHMS_DEF,
                           float   sensorEC_Konst        = SENSOREC_KONST_DEF,
                           uint8_t measurementsToAverage = 1);

    /**
     * @brief Destroy the AnalogElecConductivity object - no action needed.
     */
    ~AnalogElecConductivity();

    /**
     * @brief Report the sensor info.
     *
     * @return **String** Text describing how the sensor is attached to the mcu.
     */
    String getSensorLocation(void) override;

    /**
     * @copydoc Sensor::addSingleMeasurementResult()
     */
    bool addSingleMeasurementResult(void) override;

    /**
     * @brief Set EC constants for internal calculations.
     * Needs to be set at startup if different from defaults
     *
     * @param sourceResistance_ohms series R used in calculations for EC
     *
     * other possible K, not specified yet:
     *    float  appliedV_V,
     *    uint8_t probeType
     */
    void setEC_k(float sourceResistance_ohms) {
        _Rseries_ohms = sourceResistance_ohms;
    }

    /**
     * @brief reads the calculated EC from an analog pin.
     *
     * @param analogPinNum (optional) Analog Port, or if not supplied the
     * internal portNumber.
     *
     * @return EC value
     */
    float readEC(void);
    float readEC(uint8_t analogPinNum);

 private:
    int8_t _EcPowerPin;
    int8_t _EcAdcPin;

    float* _ptrWaterTemperature_C;

    /// @brief The resistance of the circiut resistor plus any series port
    /// resistance
    float _Rseries_ohms = RSERIES_OHMS_DEF;

    /// @brief the cell constant for the circuit
    float _sensorEC_Konst = SENSOREC_KONST_DEF;
};

/**
 * @brief The variable class used for electricalConductivity measured using an
 * analog pin connected to electrodes submerged in the medium
 *
 * @ingroup sensor_analog_cond
 *
 */
class AnalogElecConductivity_EC : public Variable {
 public:
    /**
     * @brief Construct a new  AnalogElecConductivity_EC object.
     *
     * @param parentSense The parent AnalogElecConductivity providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "anlgEc".
     */
    AnalogElecConductivity_EC(
        AnalogElecConductivity* parentSense, const char* uuid = "",
        const char* varCode = ANALOGELECCONDUCTIVITY_EC_DEFAULT_CODE)
        : Variable(parentSense,
                   (const uint8_t)ANALOGELECCONDUCTIVITY_EC_VAR_NUM,
                   (uint8_t)ANALOGELECCONDUCTIVITY_EC_RESOLUTION,
                   ANALOGELECCONDUCTIVITY_EC_VAR_NAME,
                   ANALOGELECCONDUCTIVITY_EC_UNIT_NAME, varCode, uuid) {}

    /**
     * @brief Construct a new AnalogElecConductivity_EC object.
     *
     * @note This must be tied with a parent AnalogElecConductivity before it
     * can be used.
     */
    AnalogElecConductivity_EC()
        : Variable((const uint8_t)ANALOGELECCONDUCTIVITY_EC_VAR_NUM,
                   (uint8_t)ANALOGELECCONDUCTIVITY_EC_RESOLUTION,
                   ANALOGELECCONDUCTIVITY_EC_VAR_NAME,
                   ANALOGELECCONDUCTIVITY_EC_UNIT_NAME,
                   ANALOGELECCONDUCTIVITY_EC_DEFAULT_CODE) {}
    /**
     * @brief Destroy the AnalogElecConductivity_EC object - no action needed.
     */
    ~AnalogElecConductivity_EC() {}
};
/**@}*/
#endif  // SRC_SENSORS_ANALOGELECCONDUCTIVITY_H_