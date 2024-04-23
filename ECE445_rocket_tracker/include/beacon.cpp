// Lora transmit a structure
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

struct Data_in {
    long new_freq;
    long old_freq;
};

struct Data_out {
    long lat;
    long lon;
    char FAA_id;
};

Data_in freqencies = {433, 433};    // defaults 433 just in case

Data_out beaconData = {0, 0, 0};    // data will default to 0 before proper data can be sent out.

void setup() {
  
  LoRa.setPins(10, 9, 2);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {

    //! beaconData.lat = dummy.getLatitude();
    //! beaconData.lon = dummy.getLongitude();
    //! dummy is whatever the real GPS object have is named in full code

    LoRa.beginPacket();

    LoRa.write((byte *)&beaconData, sizeof(beaconData));

    LoRa.endPacket();

    delay(500);

    // TX ABOVE /\/\/\/\/\/\/\
    // -----------------------
    // RX BELOW \/\/\/\/\/\/\/
  
    int packetSize = LoRa.parsePacket();
  
    if (packetSize) {
    // received a packet
    Serial.print("\nReceived packet size ");
    Serial.print(packetSize);
    Serial.print(" data ");
    LoRa.readBytes((byte *)&freqencies.new_freq, packetSize);   // reads received freq into stored data

    if (freqencies.new_freq != freqencies.old_freq) {

        freqencies.old_freq = freqencies.new_freq;         // update what we consider old or current freq
        LoRa.setFrequency(freqencies.new_freq);            // sets new freq on LoRa

    }

  }

}