
#include "ModemFactory.h"


#if 1
loggerModem   *LoggerModemFactory::createInstance (modemTypesCurrent_t mdmType,
    Stream* modemStream, int8_t powerPin, 
    int8_t statusPin, bool useCTSStatus,
    int8_t modemResetPin, int8_t modemSleepRqPin)
{
    loggerModem *retPtr =NULL;

    switch (mdmType)
    {
    case MODEMT_WIFI_DIGI_S6:
        retPtr = new DigiXBeeWifi(modemStream, powerPin, 
            statusPin, useCTSStatus,
            modemResetPin, modemSleepRqPin);
        break;
    case MODEMT_LTE_DIGI_CATM1:
        retPtr = new DigiXBeeCellularTransparent(modemStream, powerPin, 
            statusPin, useCTSStatus,
            modemResetPin, modemSleepRqPin);
        break;
    case MODEMT_LTE_SIM7080:
        /*retPtr = new SIMComSIM7080(modemStream, powerPin, 
            statusPin, useCTSStatus,
            modemResetPin, modemSleepRqPin); / * */
        break;
    default: break;
    }

    return retPtr;
}
#endif