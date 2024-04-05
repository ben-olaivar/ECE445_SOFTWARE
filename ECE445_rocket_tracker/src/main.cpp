

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


void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  
}

void radio_init() {

}

void transmit() {
  
}



void loop() {
  
}

