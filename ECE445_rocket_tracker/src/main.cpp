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

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

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
  
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGPS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    long altitudeMSL = myGPS.getAltitudeMSL();
    Serial.print(F(" AltMSL: "));
    Serial.print(altitudeMSL);
    Serial.print(F(" (mm)"));

    Serial.println();
  }

  
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