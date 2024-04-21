// Lora transmit a structure
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

struct Data {
  int seq;
  float y;
};

Data data={0 , 4.5};

void setup() {
  // Serial.begin(9600);
//   while (!Serial);
  // Serial.println("LoRa Sender");
  //  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
  LoRa.setPins(10, 9, 2); // for Lora 32u4
  if (!LoRa.begin(433E6)) {
    // Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // Serial.print("\nSending packet: ");
  //  Serial.print("seq = ");
    // Serial.print(data.seq);
    // Serial.print(" y = ");
    // Serial.println(data.y);
  // send packet
  LoRa.beginPacket();
  LoRa.write((byte *)&data, sizeof(data));
  for (unsigned int i = 0; i < sizeof(Data);i++) {
    // Serial.print(' ');
    //LoRa.write(((byte *) &data)[i]);
    // Serial.print(((byte *) &data)[i]);
  }
  LoRa.endPacket();
  data.seq += 1;
  data.y += 10;
  delay(50);
}