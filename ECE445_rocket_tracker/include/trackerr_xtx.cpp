// Lora transmit a structure
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

struct Data_in {
    long lat;
    long lon;
    char FAA_id;
};

long Data_out_freqency = 433;    // defaults 433 just in case

Data_in beaconData = {0, 0, 0};    // data will default to 0 before proper data can be sent out

long dummy_freq = 0;        //! this is new requested freq. connect to whatever thr variable is that receives user input

void setup() {
  
  LoRa.setPins(10, 9, 2);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {

    if (dummy_freq != Data_out_freqency) { // only send if new freq asked for

        Data_out_freqency = dummy_freq;

        LoRa.beginPacket();

        LoRa.write((byte *)&Data_out_freqency, sizeof(Data_out_freqency));

        LoRa.endPacket();

        delay(500);

    }

    // TX ABOVE /\/\/\/\/\/\/\
    // -----------------------
    // RX BELOW \/\/\/\/\/\/\/
  
    int packetSize = LoRa.parsePacket();
  
    if (packetSize) {
    // received a packet
    Serial.print("\nReceived packet size ");
    Serial.print(packetSize);
    Serial.print(" data ");
    LoRa.readBytes((byte *)&beaconData, packetSize);   // reads received freq into stored data

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());

    Serial.print("lat = ");
    Serial.print(beaconData.lat);

    Serial.print("lon = ");
    Serial.print(beaconData.lon);

    Serial.print("FAA_id = ");
    Serial.print(beaconData.FAA_id);

  }

}