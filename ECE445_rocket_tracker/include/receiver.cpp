// Lora receive a structure containg seq number and a flaot

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

struct Data {
  int seq;
  float y;
} data;


void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("LoRa Receiver");
  LoRa.setPins(10, 9, 2);  // for Lora 32u4
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
}

void loop() {
  static int counter = 0, errors = 0, seq = 0;
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("\nReceived packet size ");
    Serial.print(packetSize);
    Serial.print(" data ");
    LoRa.readBytes((byte *)&data, packetSize);
    // read packet
    //while (LoRa.available())
    for (int i = 0; i < packetSize; i++) {
      // ((byte *) &data)[i] = LoRa.read();
      Serial.print(' ');
      Serial.print(((byte *)&data)[i]);
    }
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    Serial.print("seq = ");
    Serial.print(data.seq);
    Serial.print(" y = ");
    Serial.print(data.y);
    Serial.print(" received ");
    Serial.print(++counter);
    Serial.print(" errors ");
    Serial.println(errors);
    if (seq != data.seq) {
      Serial.print("sequence number error expected ");
      Serial.println(seq);
      seq = data.seq;
      errors++;
    }
    seq++;
  }
}