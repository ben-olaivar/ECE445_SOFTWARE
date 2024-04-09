

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
#include <RH_RF95.h>
#define RF95_FREQ 434.0
#define RFM96_RST 4

RH_RF95 radio;

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  pinMode(RFM96_RST, OUTPUT);
  digitalWrite(RFM96_RST, HIGH);
  delay(10);

  // manual reset

  digitalWrite(RFM96_RST, LOW);
  delay(10);
  digitalWrite(RFM96_RST, HIGH);
  delay(10);

  if (!radio.init()) {
    // return ErrorCode::RADIO_INIT_FAILED;
    Serial.println("Radio init failed");
  }
  Serial.println("[DEBUG]: Radio Initialized");

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf =
  // 128chips/symbol, CRC on

  if (!radio.setFrequency(RF95_FREQ)) {
    // return ErrorCode::RADIO_SET_FREQUENCY_FAILED;
    Serial.println("Radio init failed");
  }

  /*
  * The default transmitter power is 13dBm, using PA_BOOST.
  * If you are using RFM95/96/97/98 modules which uses the PA_BOOST
  * transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
  */
  radio.setTxPower(6, false);
}



void loop() {
  
}

