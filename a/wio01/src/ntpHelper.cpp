/**
 * @file ntpHelper.cpp
 * @copyright 2020 Neil Hancock
 * Used for Wio Terminal testing,  mostly mashup 
 * @author neil hancock <neilh20@wllw.net>
 *
 * @brief NTP interface
 */

// Included Dependencies
#include "ntpHelper.h"

// define WiFI client
WiFiClient client;

//The udp library class
WiFiUDP udp;
wl_status_t  wifi_status=WL_IDLE_STATUS;
bool wifi_pwr_off=true;

//ntpHelper::ntpHelper() {}
//ntpHelper::~ntpHelper() {}

//ntpHelper:: {}
bool ntpHelper::connectToWiFi(const char* ssid, const char* pwd) {
    Serial.println("Connecting to WiFi network: " + String(ssid));

    // delete old config
    WiFi.disconnect(true);

    Serial.println("Waiting for WIFI connection...");
    _ssid = ssid;
    _password= pwd;
    //Initiate connection
    WiFi.begin(ssid, pwd);

    delay(100); 
    while ((wifi_status=  WiFi.status()) != WL_CONNECTED) {
        Serial.printf("Waiting for WIFI connection (%d)...",wifi_status);
        WiFi.begin(ssid, pwd);
        delay(500);
    }
    //Serial.println("Connected.");

    return true;

}

unsigned long ntpHelper::getNTPtime() {
 
    // module returns a unsigned long time valus as secs since Jan 1, 1970 
    // unix time or 0 if a problem encounted
 
    //only send data when connected
    wifi_status = WiFi.status();
    if ( WL_DISCONNECTED == wifi_status) {
        // try and connect
        for (int wifi_lp=0; wifi_lp<5;wifi_lp++) {
            Serial.printf("WiFi disconnected status %d try aain \n\r",wifi_status);
            connectToWiFi(_ssid, _password);
            wifi_status = WiFi.status();
            if (WL_CONNECTED == wifi_status ) {
                delay(100);
                break;}
        }

    } 
    if (WL_CONNECTED == wifi_status) {
        //initializes the UDP state
        //This initializes the transfer buffer
        udp.begin(WiFi.localIP(), localPort);
 
        sendNTPpacket(timeServer); // send an NTP packet to a time server
        // wait to see if a reply is available
        delay(1000);
 
        if (udp.parsePacket()) {
            //Serial.println("udp packet received");
            //Serial.println("");
            // We've received a packet, read the data from it
            udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
 
            //the timestamp starts at byte 40 of the received packet and is four bytes,
            // or two words, long. First, extract the two words:
 
            unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
            unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
            // combine the four bytes (two words) into a long integer
            // this is NTP time (seconds since Jan 1 1900):
            unsigned long secsSince1900 = highWord << 16 | lowWord;
            // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
            const unsigned long seventyYears = 2208988800UL;
            // subtract seventy years:
            unsigned long epoch = secsSince1900 - seventyYears;
 
            // adjust time for timezone offset in secs +/- from UTC
            long tzOffset = (UTC_OFFSET_HRS*60*60);
 
                unsigned long adjustedTime;
            return adjustedTime = epoch + tzOffset;
        }
        else {
            // were not able to parse the udp packet successfully
            // clear down the udp connection
            udp.stop();
            Serial.println("WiFi UDP garbeled");
            return 0; // zero indicates a failure
        }
        // not calling ntp time frequently, stop releases resources
        udp.stop();
    }
    else {
        // network not connected
        Serial.printf("WiFi not conencted status: %d\n\r", wifi_status);
        return 0;
    }
 
} //getNTPtime

// send an NTP request to the time server at the given address
unsigned long ntpHelper::sendNTPpacket(const char* address) {
    // set all bytes in the buffer to 0
    for (int i = 0; i < NTP_PACKET_SIZE; ++i) {
        packetBuffer[i] = 0;
    }
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    return udp.endPacket();
}

#if 0
void ntpHelper::timeUpdate(){

    if ((WL_CONNECTED != wifi_status) || wifi_pwr_off) {
        Serial.print("connectionToWiFi attempt, status: ");
        Serial.print(wifi_status);
        Serial.print(" pwr_off: ");
        Serial.println(wifi_pwr_off);
        if (wifi_pwr_off) {
            //digitalWrite(RTL8720D_CHIP_PU, LOW); assume
            //delay(500);
            digitalWrite(RTL8720D_CHIP_PU, HIGH);
            delay(500);
            tcpip_adapter_init();
        }

        connectToWiFi(_ssid, _password);
    }
    // update rtc time
    devicetime = getNTPtime();
    lcd_clear();
    tu_total++;
    if (devicetime == 0) {
        Serial.println("Failed to get time from network time server.");
        tu_fail++;
        now = rtc.now();
        tft.print(tu_fail);
        tft.println("/");
        tft.print(tu_total);
        tft.println(" err ");
        String now_str(now.timestamp(DateTime::TIMESTAMP_FULL));
        tft.println(now_str);
    }
    else {
        rtc.adjust(DateTime(devicetime));
        now = rtc.now();
        Serial.print(tu_total);
        Serial.print(" times/err: ");
        Serial.print(tu_fail);
        Serial.print(" ] Adjusted RTC time (UTC): ");
        String now_str(now.timestamp(DateTime::TIMESTAMP_FULL));
        Serial.println(now_str);
        /// Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
        tft.print("NTP ");
        tft.println(now_str);
        tft.print("tot ");
        tft.print(tu_total);
        tft.print(" err ");
        tft.print(tu_fail);
    }

    //Turn off WiFi ~ a guess, would be nice to have an API 
    while (!WiFi.disconnect(true) ) {
        Serial.print("Attempting WiFi.disconnect");
        delay(1000);
    }
    wifi_status=  WiFi.status();
    if (WL_CONNECTED == wifi_status) {
        Serial.println("WiFi not disconnected");
    } else {
        Serial.print("WiFi disconnected: ");
        Serial.println(wifi_status);

        digitalWrite(RTL8720D_CHIP_PU, LOW);//seeed_wio_terminal\Seeed Arduino rpcUnified\src\erpc_Unified_init.cpp
        //delay(100);
        //digitalWrite(RTL8720D_CHIP_PU, HIGH);
        wifi_pwr_off = true;
    }
}  //timeUpdate
#endif 
void  ntpHelper::printWifiStatus() {
    // print the SSID of the network you're attached to:
    Serial.println("");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
    Serial.println("");
}
