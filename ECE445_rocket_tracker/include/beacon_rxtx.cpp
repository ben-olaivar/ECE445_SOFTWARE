// Lora transmit a structure
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

byte localAddress = 0xBB;
byte destinationAddress = 0xAA;
long lastSendTime = 0;
int interval = 2000;
int count = 0;


struct Data_in {
    long lat;
    long lon;
    char* FAA_id;
};

struct Data_out {
    long freq = 433E6;
    char* FAA_id = "KD9UGU";
} data_out;

long Data_out_freqency = 433E6;    // defaults 433 just in case

Data_in beaconData = {0, 0, 0};    // data will default to 0 before proper data can be sent out

// long dummy_freq = 0;        //! this is new requested freq. connect to whatever thr variable is that receives user input

void setup() {
  Serial.begin(9600);
  while(!Serial);
  
  LoRa.setPins(10, 9, 2);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}



void sendMessage(String outgoing) {
  LoRa.beginPacket();
  LoRa.write(destinationAddress);
  LoRa.write(localAddress);
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);
  LoRa.endPacket();
}

void receiveMessage(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingLength = LoRa.read();

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {
    Serial.println("Error: Message length does not match length");
    return;
  }

  if (recipient != localAddress) {
    Serial.println("Error: Recipient address does not match local address");
    return;
  }

  Serial.print("Received data " + incoming);
  Serial.print(" from 0x" + String(sender, HEX));
  Serial.println(" to 0x" + String(recipient, HEX));
}


// void loop() {
//   Serial.println("Loop");

//     // if (dummy_freq != Data_out_freqency) { // only send if new freq asked for

//         // Data_out_freqency = dummy_freq;

//         LoRa.beginPacket();

//         LoRa.write((byte *)&data_out, sizeof(data_out));

//         LoRa.endPacket();

//         delay(500);

//     // }

//     // TX ABOVE /\/\/\/\/\/\/\
//     // -----------------------
//     // RX BELOW \/\/\/\/\/\/\/
  
//     int packetSize = LoRa.parsePacket();
  
//     if (packetSize) {
//     // received a packet
//     Serial.print("\nReceived packet size ");
//     Serial.print(packetSize);
//     Serial.print(" data ");
//     LoRa.readBytes((byte *)&beaconData, packetSize);   // reads received freq into stored data

//     // print RSSI of packet
//     Serial.print("' with RSSI ");
//     Serial.println(LoRa.packetRssi());

//     Serial.print("lat = ");
//     Serial.print(beaconData.lat);

//     Serial.print("lon = ");
//     Serial.print(beaconData.lon);

//     Serial.print("FAA_id = ");
//     Serial.print(beaconData.FAA_id);

//   }
// }

void loop() {
  if (millis() - lastSendTime > interval) {
    String sensorData = String(count++);
    sendMessage(sensorData);

    Serial.print("Sending data " + sensorData);
    Serial.print(" from 0x" + String(localAddress, HEX));
    Serial.println(" to 0x" + String(destinationAddress, HEX));

    lastSendTime = millis();
    interval = random(2000) + 1000;
  }

  receiveMessage(LoRa.parsePacket());
}
