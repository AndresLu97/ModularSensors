// ==========================================================================
// tu2cmd.h
// Utilites for processing user input.
// This is initailly from Serial Uart, 
// future could be from other serial sources, including BT
// ==========================================================================
#ifndef SRC_TU_SERIALCMD_H_
#define SRC_TU_SERIALCMD_H_

#include <SerialCommand.h>

SerialCommand tu2sc;

#define  USERHELP "\n\
LR - list the readings\n\
L? - list the directory\n\
D,yymmdd,hhmm<cr> to set date/time\n\
D?<cr> to print current date/time\n\
?<cr> for this help\n"

void tu2Help() {
    PRINTOUT(F(USERHELP));
}

void tu2cmdListDir() {
    dataLogger.SD1_ListDir();
}
void tu2cmdListReadings() {
    dataLogger.SD1_ListReadings();
}
void tu2Unknown(const char *command) {
    PRINTOUT(F("Unknown input <tbd>"));
    tu2Help();
}

void tu2cmdDateNow (){
    // date
    PRINTOUT(F("Local Time "),dataLogger.formatDateTime_ISO8601(dataLogger.getNowEpochTz()));
}

// ==========================================================================
// parseTwoDigits
// Simple helper function
uint8_t parseTwoDigitsError =0;
uint16_t parseTwoDigits(const char *digits) {
    uint16_t  num=0; 
    if ( !isdigit(digits[0]) || !isdigit(digits[1]) ) {
        MS_DBG(F("parseTwoDigits error with "),digits[0],digits[1] );
        parseTwoDigitsError =1;
    } else {
        num = (digits[0]-'0')*10 +
            (digits[1]-'0');
    }
    return num;
}  // parseTwoDigits

void tu2cmdDate (){
    // date
    PRINTOUT(F("Local Time "),dataLogger.formatDateTime_ISO8601(dataLogger.getNowEpochTz()));
    char *argcin;

    argcin = tu2sc.next();
    if (NULL != argcin) { 
        #ifdef SERIALCOMMAND_DEBUG
        Serial.print("parsing: '");
        Serial.println(argcin);
        #endif
        parseTwoDigitsError =0;
        uint16_t year = parseTwoDigits(&argcin[1]);
        uint8_t month = parseTwoDigits(&argcin[3]);
        uint8_t day   = parseTwoDigits(&argcin[5]);
        uint8_t hour  = parseTwoDigits(&argcin[8]);
        uint8_t minute= parseTwoDigits(&argcin[10]);
        if (0==parseTwoDigitsError) {
            DateTime dt(year,month,day,hour,minute,0,0);
            dataLogger.setRTClock(dt.getEpoch()-dataLogger.getTZOffset()*HOURS_TO_SECS);
            PRINTOUT(F("Time set to "),dataLogger.formatDateTime_ISO8601(dataLogger.getNowEpochTz()));
            //    DateTime (uint16_t year, uint8_t month, uint8_t date,
            //uint8_t hour, uint8_t min, uint8_t sec, uint8_t wday);
        } //else err
    }

    #if 0
    // format d?\n OR dyymmdd-hhmm\n
    if ('?'==serialInputBuffer[1]) {
        PRINTOUT(F("Local Time "),dataLogger.formatDateTime_ISO8601(dataLogger.getNowEpochTz()));
    } else {
        const char *cin = serialInputBuffer.c_str();
        int ser_len = serialInputBuffer.length();
        //MS_DBG(F("**sid("),ser_len,F(")="),serialInputBuffer);
        if (12>ser_len) {
            PRINTOUT(F("date invalid, got"),ser_len,F(" expect at least 11 chars :'"),&cin[1],"'");
        } else {
            parseTwoDigitsError =0;
            uint16_t year = parseTwoDigits(&cin[1]);
            uint8_t month = parseTwoDigits(&cin[3]);
            uint8_t day   = parseTwoDigits(&cin[5]);
            uint8_t hour  = parseTwoDigits(&cin[8]);
            uint8_t minute= parseTwoDigits(&cin[10]);
            if (0==parseTwoDigitsError) {
                DateTime dt(year,month,day,hour,minute,0,0);
                dataLogger.setRTClock(dt.getEpoch()-dataLogger.getTZOffset()*HOURS_TO_SECS);
                PRINTOUT(F("Time set to "),dataLogger.formatDateTime_ISO8601(dataLogger.getNowEpochTz()));
                //    DateTime (uint16_t year, uint8_t month, uint8_t date,
                //uint8_t hour, uint8_t min, uint8_t sec, uint8_t wday);
            }
        }
    }
    #endif //
}

#define SC_COMMAND_NUMBER 5
const static PROGMEM SerialCommand::SerialCommandCallback sc_commandList[SC_COMMAND_NUMBER ] =
{
    //List in search order, 1st match is used
    {"D",tu2cmdDate},
    {"D?",tu2cmdDateNow},
    {"LR",tu2cmdListReadings},
    {"L?",tu2cmdListDir},
    {"?",tu2Help},
};


void tu2setup() {
    tu2sc.setCommandList(sc_commandList,SC_COMMAND_NUMBER);
    tu2sc.setDefaultHandler(tu2Unknown);
}


//tu_cli tucli;
// ==========================================================================
// Data section for userTuple processing
//#define  USER_INPUT_BUF_SZ 64
//String serialInputBuffer = ""; //Could this be reason #55, Avoid String
//bool  serial_1st_char_bool =true;

bool   userInputCollection=false;

// ==========================================================================
// userTupleParse
// When a user tuple has been detected, parse it,
// take a appropiate action
//
// this could be  https://github.com/Uberi/Arduino-CommandParser  ~ KISS
//

// ==========================================================================
// Serial Input Driver
// serial Input & FUT: eRPC (eRPC needs to enable UART interrupt)
//
// Serial buffer max SERIAL_RX_BUFFER_SIZE  64 chars
//
// The serial input is very error pront  
// 

void serialInputCheck() 
{
    //char incoming_ch;
    long timer_start_ms, timer_activity_ms,timer_now_ms;
    timer_start_ms=timer_activity_ms=millis();
    //20 seconds between key strokes
#define TIMER_TIMEOUT_NOACTIVITY_MS 20000
    // 180sec total timer
#define TIMER_TIMEOUT_LIMIT_MS 180000

    PRINTOUT(F("\n\n"), (char*)epc.app.msc.s.logger_id,configDescription);
    PRINTOUT(MODULAR_SENSORS_VERSION,F("@"), epc.app.msc.s.logging_interval_min,
        F("min,"),dataLogger.formatDateTime_ISO8601(dataLogger.getNowEpochTz()));
    PRINTOUT(F(" Enter cmd: ?<CR> for help.(need a key to be typed every "), TIMER_TIMEOUT_NOACTIVITY_MS/1000,F("secs)"));
    while (userInputCollection ) {
        if(Serial.available() != 0) {
            dataLogger.watchDogTimer.resetWatchDog();
            timer_activity_ms = millis();
            tu2sc.readSerial();
        }
        //delay(1); // limit polling ~ the single character input is error prone ??

        timer_now_ms = millis();
        if (TIMER_TIMEOUT_NOACTIVITY_MS < (timer_now_ms - timer_activity_ms) ) {
            PRINTOUT(F(" No keyboard activity for"), TIMER_TIMEOUT_NOACTIVITY_MS/1000,F("secs. Returning to normal logging."));
            tu2sc.clearBuffer();
            break;
        } 
        if (TIMER_TIMEOUT_LIMIT_MS < (timer_now_ms - timer_start_ms) ) {
            PRINTOUT(F(" Took too long, need to complete within "), TIMER_TIMEOUT_LIMIT_MS/1000,F("secs. Returning to normal logging."));
            tu2sc.clearBuffer();
            break;
        } 

    } //while

    dataLogger.watchDogTimer.resetWatchDog();
}//serialInputCheck
#endif  //SRC_TU_SERIALCMD_H_