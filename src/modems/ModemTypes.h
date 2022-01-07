

#ifndef SRC_MODEMS_MODEMTYPESH_
#define SRC_MODEMS_MODEMTYPESH_

//#include "DigiXBeeCellularTransparent.h"
//#include "DigiXBeeWifi.h"
//#include "SIMComSIM7080.h"
//#include <Stream.h>


typedef enum {
    MODEMT_WIFI_DIGI_S6=1,
    MODEMT_LTE_DIGI_CATM1=2,
    MODEMT_LTE_SIM7080=10,
    MODEMT_END
} modemTypes_01_t;
#define modemTypesCurrent_t modemTypes_01_t 
typedef struct {
    uint8_t modemTypseVersion;
    modemTypesCurrent_t modemType; 
} modemTypeTupl_t;
    
#endif  // SRC_MODEMS_MODEMTYPESH_