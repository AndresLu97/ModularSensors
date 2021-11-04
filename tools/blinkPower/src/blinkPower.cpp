
#include <Arduino.h>


// Actually used on V1.0a3
const char*    mcuBoardVersion = "v0.5b";

// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 57600;  // Baud rate for debugging
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



void setup(void) 
{
  Serial.begin(serialBaud);
  while (!Serial) {
      delay(1);
  }
 
  Serial.println("blinkPower v0.1 to test switched power");
  
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, LOW); //Off
  //pinMode(A5, OUTPUT);
  //digitalWrite(A5, HIGH);
  //pinMode(10, OUTPUT);
  //digitalWrite(10, HIGH);

  // Set up pins for the LED's HIGH is on
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED, LOW); //off
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, LOW); //off

}

int elapsed_time_sec=0;
#define DELAY_ON_SEC 10
#define DELAY_OFF_SEC  2
uint8_t PowerState=1;

void loop(void) 
{

  elapsed_time_sec++;
  Serial.print("Time"); Serial.println(elapsed_time_sec);
  if (PowerState) {
    digitalWrite(powerPin, LOW);
    digitalWrite(greenLED, LOW);
    PowerState =0;
    delay(DELAY_OFF_SEC*1000);
  } else {
    digitalWrite(powerPin, HIGH); //ON
    digitalWrite(greenLED, HIGH); // ON
    PowerState = 1;
    delay(DELAY_ON_SEC*1000);
  }

} //loop()
