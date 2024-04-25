// Lora receive a structure containg seq number and a flaot

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

struct Data_out {
    long lat = 123;   // dummy vals to start
    long lon = 456;   // ^^^^
    char* FAA_id = "KD9UGU";
} beaconData;

struct Data_in {
    long freq = 433E6;
    char* FAA_id = "KD9UGU";
} trackerData;

int timestamp = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("LoRa Receiver");
  LoRa.setPins(10, 9, 2);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
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

    LoRa.beginPacket();

    LoRa.write((byte *)&beaconData, sizeof(beaconData));

    LoRa.endPacket();

    timestamp = millis();                   // reset 3 second timer

    Serial.println("end beacon data send (dummy vals rn)");

  }

}