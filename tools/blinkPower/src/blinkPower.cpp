
#include <Arduino.h>
//#include "Sodaq_DS3231.h"
#include "RTClib.h"

RTC_DS3231 rtc;

// Testing with V1.1A
// Tested with V1.0a3
const char*    mcuBoardVersion = "v0.5b";

// NOTE:  Use -1 for pins that do not apply
//const int32_t serialBaud = 57600;  // Baud rate for debugging
const int32_t serialBaud = 115200;
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t  buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t  wakePin    = 31;  // MCU interrupt/alarm pin to wake from sleep
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power

int8_t powerPin = 22;

void printTime() {
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('.');
    Serial.print(now.month(), DEC);
    Serial.print('.');
    Serial.print(now.day(), DEC);
    Serial.print("-");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
}

void printTimeln() {
  printTime();
  Serial.println();
}

const int8_t sensor_Vbatt_PIN    = A6;
float readVcc() {
    uint32_t rawAdc = analogRead(sensor_Vbatt_PIN);

    #define PROCADC_REF_V 3.3
    #define ProcAdcDef_Resolution 10
    #define ProcAdc_Max ((1 << ProcAdcDef_Resolution) - 1)
    #define PROCADC_RANGE_MIN_V -0.3  
    const float  procVoltDividerGain = 4.7;
    float adcVoltage = (PROCADC_REF_V / ProcAdc_Max) * (float)rawAdc;

    return adcVoltage*procVoltDividerGain;
 }

void setup(void) 
{
  Serial.begin(serialBaud);
  while (!Serial) {
      delay(1);
  }

  Serial.println("blinkPower v0.2 to test switched power Mayfly 1.1A");
  do {
    if (rtc.begin()) break;
    Serial.println("Couldn't find RTC");
    Serial.flush();
    delay(10);
  } while (1);
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, set time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
     rtc.adjust(DateTime(2022, 1, 1, 0, 0, 0));
  }
  Serial.print("Boot Time ");
  printTimeln();
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, LOW); //Off
  //pinMode(A5, OUTPUT);
  //digitalWrite(A5, HIGH);
  //pinMode(10, OUTPUT);
  //digitalWrite(10, HIGH);
  analogRead(sensor_Vbatt_PIN); // Throw away

  // Set up pins for the LED's HIGH is on
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED, LOW); //off
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, LOW); //off

}

int elapsed_passes=0;
#define DELAY_ON_SEC 10
#define DELAY_OFF_SEC  2
uint8_t PowerState=1;

void loop(void) 
{

  
  if (PowerState) {
    elapsed_passes++;
    printTime();
    Serial.print(" Pass "); Serial.print(elapsed_passes);
    Serial.print (" StartV " );
    Serial.print(readVcc());
    digitalWrite(powerPin, LOW);
    digitalWrite(greenLED, LOW);
    PowerState =0;
    delay(DELAY_OFF_SEC*1000);
    Serial.print (" EndV " );
    Serial.println(readVcc());
  } else {

    digitalWrite(powerPin, HIGH); //ON
    digitalWrite(greenLED, HIGH); // ON
    PowerState = 1;
    delay(DELAY_ON_SEC*1000);
  }

} //loop()
