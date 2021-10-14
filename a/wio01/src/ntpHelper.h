
#ifndef SRC_NTP_HELPER_H_
#define SRC_NTP_HELPER_H_
#include "Arduino.h"
#include <rpcWiFi.h>
#include <millisDelay.h>
//#include "RTC_SAMD51.h"
#include <SPI.h>
//#include "TFT_eSPI.h"
//#include "Free_Fonts.h" 

const unsigned int localPort = 2390;      // local port to listen for UDP packets
#ifdef USELOCALNTP
    char timeServer[] = "n.n.n.n"; // local NTP server 
#else
    const char timeServer[] = "time.nist.gov"; // extenral NTP server e.g. time.nist.gov
#endif
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

// CA time offset from UTC is -8 hours (28,800 secs)
#define UTC_OFFSET_HRS -8

class ntpHelper {
public:
     bool connectToWiFi(const char* ssid, const char* pwd);
     void printWifiStatus();
     unsigned long getNTPtime();
     //void timeUpdate();

//#define byte char
uint8_t packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

private :
unsigned long sendNTPpacket(const char* address) ;

const char *_ssid ; //= "ArthurGuestSsid"; // add your required ssid
const char *_password ;// = "Arthur8166";//guest1234"your-passowrd"; // add your own netywork password

};


#endif // SRC_NTP_HELPER_H_
