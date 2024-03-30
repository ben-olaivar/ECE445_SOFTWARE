// #include <Arduino.h>
// #include <Wire.h>
// #include <LoRa.h>
// #include "pins.h"
// #include "SparkFun_Ublox_Arduino_Library.h" // http://librarymanager/All#SparkFun_u-blox_GNSS

// SFE_UBLOX_GPS myGPS;
// long lastTime = 0;

// void setup() {
//   // TODO: remove serial stuff on final version
//   Serial.begin(9600);   // open the serial port at 9600 bps:
//   while(!Serial);       // wait for serial to start

//   Wire.begin();         // join i2c bus (address optional for master)

//   //!-------------------GPS STUFF-------------------
//   if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
//   {
//     Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
//     while (1);
//   }
//   //!-------------------END GPS STUFF-------------------

//   pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
// }

// // the loop function runs over and over again forever
// void loop() {
//   // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//   // Serial.print("on\n");
//   // delay(100);                       // wait for a second
//   // digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//   // Serial.print("off\n");
//   // delay(100);                       // wait for a second
  
//   //!-------------------GPS STUFF-------------------
//   if (millis() - lastTime > 1000)
//   {
//     lastTime = millis(); //Update the timer
    
//     long latitude = myGPS.getLatitude();
//     Serial.print(F("Lat: "));
//     Serial.print(latitude);

//     long longitude = myGPS.getLongitude();
//     Serial.print(F(" Long: "));
//     Serial.print(longitude);
//     Serial.print(F(" (degrees * 10^-7)"));

//     long altitude = myGPS.getAltitude();
//     Serial.print(F(" Alt: "));
//     Serial.print(altitude);
//     Serial.print(F(" (mm)"));

//     long altitudeMSL = myGPS.getAltitudeMSL();
//     Serial.print(F(" AltMSL: "));
//     Serial.print(altitudeMSL);
//     Serial.print(F(" (mm)"));

//     Serial.println();
//   }
//   //!-------------------END GPS STUFF-------------------

  
// }

// struct beacon_transmission_struct {
//   uint8_t error_mask;
//   float coords_longitude;
//   float coords_latitude;
//   float elevation;
//   int callsign;
// };

// struct handheld_transmission_struct {
//   int command_number;
//   float command_data;
//   int callsign;
// };





/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/
#include<Wire.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>

const int csPin = 8;          // LoRa radio chip select
const int resetPin = 3;       // LoRa radio reset
const int irqPin = 4;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}


void loop() {
  if ((int)(millis() - lastSendTime) > (int)interval) {
    String message = "HeLoRa World!";   // send a message
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
    interval = random(2000) + 1000;    // 2-3 seconds
  }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

