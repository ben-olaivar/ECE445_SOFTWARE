// Lora receive a structure containg seq number and a flaot

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
//#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "SparkFun_Ublox_Arduino_Library.h"
// #include "gps.h"

// 4011055399719058, -8822548777629069
struct Data_out {
    long lat = 0;   // dummy vals to start
    long lon = 0;   // ^^^^
    char* FAA_id = "KD9UGU";
} beaconData;

struct Data_in {
    long freq = 433E6;
    char* FAA_id = "KD9UGU";
} trackerData;

unsigned long timestamp = 0;

SFE_UBLOX_GPS myGPS;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Wire.begin();
  

  Serial.println("LoRa Receiver");
  LoRa.setPins(10, 9, 2);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  if (!myGPS.begin()) {
    Serial.println("GPS failed to init!");
    while(1);
  }

  Serial.println("Setup done");
}

void loop() {

  // try to parse packet
  int packetSize = LoRa.parsePacket();

  if (packetSize) {           // check if got a packet, if yes then read

    LoRa.readBytes((byte *)&trackerData, packetSize);

    Serial.println("Got a new freq:");
    Serial.println(trackerData.freq);

    LoRa.setFrequency(trackerData.freq);      // change freq based on new incoming freq

    Serial.println("Properly set");

  }

  if (millis() - timestamp > 3000) {        // every 3 seconds sendout current beacon data

    Serial.println("------------------");
    beaconData.lat = myGPS.getLatitude();
    beaconData.lon = myGPS.getLongitude();

    Serial.println("beacon lat: " + String(beaconData.lat));
    Serial.println("beacon long: " + String(beaconData.lon));
    LoRa.beginPacket();

    LoRa.write((byte *)&beaconData, sizeof(beaconData));

    LoRa.endPacket();

    timestamp = millis();                   // reset 3 second timer


  }

}