/**
 * @file QuectelBG96.h
 * @copyright 2020 Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library for Arduino
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Contains the QuectelBG96 subclass of loggerModem for Dragino,
 * Nimbelink, or any other module based on the Quectel BG96.
 */

// Header Guards
#ifndef SRC_MODEMS_QUECTELBG96_H_
#define SRC_MODEMS_QUECTELBG96_H_

// Debugging Statement
// #define MS_QUECTELBG96_DEBUG
// #define MS_QUECTELBG96_DEBUG_DEEP

#ifdef MS_QUECTELBG96_DEBUG
#define MS_DEBUGGING_STD "QuectelBG96"
#endif

/**
 * @brief The modem type for the underlying TinyGSM library.
 */
#define TINY_GSM_MODEM_BG96
#ifndef TINY_GSM_RX_BUFFER
/**
 * @brief The size of the buffer for incoming data.
 */
#define TINY_GSM_RX_BUFFER 64
#endif


/**
 * @brief The loggerModem::_statusLevel.
 *
 * Status of the BG96 can be monitored on the `STATUS(D0)` pin.  Time after end
 * pulse until status pin becomes active is 4.8s.
 */
#define BG96_STATUS_LEVEL HIGH
/**
 * @brief The loggerModem::_statusTime_ms.
 * @copydetails #BG96_STATUS_LEVEL
 */
#define BG96_STATUS_TIME_MS 5000L

/**
 * @brief The loggerModem::_resetLevel.
 *
 * BG96 is reset with a 150-460ms low pulse on the `RESET_N` pin
 */
#define BG96_RESET_LEVEL LOW
/**
 * @brief The loggerModem::_resetPulse_ms.
 * @copydetails #BG96_RESET_LEVEL
 */
#define BG96_RESET_PULSE_MS 300


/**
 * @brief The loggerModem::_wakeLevel.
 *
 * Module is switched on by a >100 millisecond `LOW` pulse on the `PWRKEY` pin.
 * Module is switched on by a >650 millisecond `LOW` pulse on the `PWRKEY` pin.
 * Using something between those times for wake and using AT commands for sleep,
 * we should keep in the proper state.
 *
 * @note If at all possible, the BG96 status pin should be monitored to confirm
 * active status.
 */
#define BG96_WAKE_LEVEL LOW
/**
 * @brief The loggerModem::_wakePulse_ms.
 * @copydetails #BG96_WAKE_LEVEL
 */
#define BG96_WAKE_PULSE_MS 110

/**
 * @brief The loggerModem::_wakeDelayTime_ms.
 *
 * Time after `VBAT` is stable before `PWRKEY` can be used on BG96 is >30ms
 */
#define BG96_WAKE_DELAY_MS 100
/**
 * @brief The loggerModem::_max_atresponse_time_ms.
 *
 * The BG96 has USB active at >4.2 sec, status at >4.8 sec, URAT at >4.9
 */
#define BG96_ATRESPONSE_TIME_MS 10000L

/**
 * @brief The loggerModem::_disconnetTime_ms.
 *
 * Documentation for the BG96 says to allow >2s for clean shutdown.
 */
#define BG96_DISCONNECT_TIME_MS 5000L

// Included Dependencies
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#include "TinyGsmClient.h"
#include "LoggerModem.h"

#ifdef MS_QUECTELBG96_DEBUG_DEEP
#include <StreamDebugger.h>
#endif


/**
 * @brief The loggerModem subclass for Dragino, Nimbelink, or any other module
 * based on the Quectel BG96.
 *
 * #### Pin and timing information for the BG96
 *
 * @copydetails #BG96_RESET_LEVEL
 *
 * @copydetails #BG96_ATRESPONSE_TIME_MS
 *
 * @copydetails #BG96_DISCONNECT_TIME_MS
 *
 * @copydetails #BG96_STATUS_LEVEL
 *
 * @copydetails #BG96_WAKE_DELAY_MS
 *
 * @copydetails #BG96_WAKE_LEVEL
 *
 * @see @ref page_bg96
 */
class QuectelBG96 : public loggerModem {
 public:
    /**
     * @brief Construct a new Quectel BG96 object
     *
     * The constuctor initializes all of the provided member variables,
     * constructs a loggerModem parent class with the appropriate timing for the
     * module, calls the constructor for a TinyGSM modem on the provided
     * modemStream, and creates a TinyGSM Client linked to the modem.
     *
     * @param modemStream The Arduino stream instance for serial communication.
     * @param powerPin @copydoc loggerModem::_powerPin
     * @param statusPin @copydoc loggerModem::_statusPin
     * This is the pin labeled `STATUS` in Quectel's integration guide.
     * @param modemResetPin @copydoc loggerModem::_modemResetPin
     * This is the pin labeled `RESET_N` in Quectel's integration guide.
     * @param modemSleepRqPin @copydoc loggerModem::_modemSleepRqPin
     * This is the pin labeled `PWRKEY` in Quectel's integration guide.
     * @param apn The Access Point Name (APN) for the SIM card.
     *
     * @see loggerModem::loggerModem
     */
    QuectelBG96(Stream* modemStream, int8_t powerPin, int8_t statusPin,
                int8_t modemResetPin, int8_t modemSleepRqPin, const char* apn);
    /**
     * @brief Destroy the Quectel BG96 object - no action taken
     */
    ~QuectelBG96();

    bool modemWake(void) override;

    bool connectInternet(uint32_t maxConnectionTime = 50000L) override;
    void disconnectInternet(void) override;

    uint32_t getNISTTime(void) override;

    bool  getModemSignalQuality(int16_t& rssi, int16_t& percent) override;
    bool  getModemBatteryStats(uint8_t& chargeState, int8_t& percent,
                               uint16_t& milliVolts) override;
    float getModemChipTemperature(void) override;

    bool modemHardReset(void) override;

#ifdef MS_QUECTELBG96_DEBUG_DEEP
    StreamDebugger _modemATDebugger;
#endif

    /**
     * @brief Public reference to the TinyGSM modem.
     */
    TinyGsm gsmModem;
    /**
     * @brief Public reference to the TinyGSM Client.
     */
    TinyGsmClient gsmClient;

 protected:
    bool isInternetAvailable(void) override;
    bool modemSleepFxn(void) override;
    bool modemWakeFxn(void) override;
    bool extraModemSetup(void) override;
    bool isModemAwake(void) override;

 private:
    const char* _apn;
};

#endif  // SRC_MODEMS_QUECTELBG96_H_
