#include <Arduino.h>
#include <Wire.h>
#include "pins.h"

void setup() {
  Wire.begin();         // join i2c bus (address optional for master)
  Serial.begin(9600);   // open the serial port at 9600 bps:

  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.print("on\n");
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  Serial.print("off\n");
  delay(100);                       // wait for a second
  // wait for BEN to get real
  Serial.print("ya mama\n");
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