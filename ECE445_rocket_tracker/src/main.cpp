#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "SparkFun_Ublox_Arduino_Library.h" // http://librarymanager/All#SparkFun_u-blox_GNSS

SFE_UBLOX_GPS myGPS;
long lastTime = 0;

void setup() {
  // TODO: remove serial stuff on final version
  Serial.begin(9600);   // open the serial port at 9600 bps:
  while(!Serial);       // wait for serial to start

  Wire.begin();         // join i2c bus (address optional for master)

  //!-------------------GPS STUFF-------------------
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  //!-------------------END GPS STUFF-------------------

  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
}

// the loop function runs over and over again forever
void loop() {
  // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  // Serial.print("on\n");
  // delay(100);                       // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  // Serial.print("off\n");
  // delay(100);                       // wait for a second
  
  
}

struct beacon_transmission_struct {
  uint8_t error_mask;
  float coords_longitude;
  float coords_latitude;
  float elevation;
  int callsign;
};

struct handheld_transmission_struct {
  int command_number;
  float command_data;
  int callsign;
};