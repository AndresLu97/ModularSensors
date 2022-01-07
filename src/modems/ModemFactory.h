/**
 * @file ModemFactory.h
 * @copyright 2022 Neil Hancock assigned Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library 
 * @author Neil Hancock<neilh20@ >
 *
 * @brief Contains the ModemFactoryclass 
  */
/**
 * @defgroup the_modems Supported Modems and Communication Modules
 * All implemented loggerModem classes
 *
 * @copydetails loggerModem
 *
 * @see @ref page_modem_notes
 */

// https://www.codeproject.com/Articles/3734/Different-Ways-of-Implementing-Factories

#ifndef SRC_MODEMFACTORY_H_
#define SRC_MODEMFACTORY_H_

// FOR DEBUGGING
// #define MS_MODEMFACTORY_DEBUG
// #define MS_MODEMFACTORY_DEBUG_DEEP

#ifdef MS_MODEMFACTORY_DEBUG
#define MS_DEBUGGING_STD "ModemFactory"
#endif

// Included Dependencies
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
//#include "VariableBase.h"
#include <Arduino.h>
#include "LoggerModem.h"
// The modems used need 
#include "DigiXBeeCellularTransparent.h"
#include "DigiXBeeWifi.h"
//#include "SIMComSIM7080.h"
#include "ModemTypes.h"

class LoggerModemFactory 
   {
   public:
      loggerModem  *createInstance(modemTypesCurrent_t mdmType,
            Stream* modemStream, int8_t powerPin, 
            int8_t statusPin, bool useCTSStatus,
            int8_t modemResetPin, int8_t modemSleepRqPin);
   };

#endif  // SRC_MODEMFACTORY_H_