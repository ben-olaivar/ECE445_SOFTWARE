

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

// TRANCEIVER CODE START
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
/*

#include <Wire.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <RH_RF95.h>
#define RF95_FREQ 434.0
#define RFM96_RST 4

#define RFM95_INT 3
#define RFM95_CS 8

RH_RF95 radio(RFM95_CS, RFM95_INT);

long cur_long;
long cur_lat;

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

  
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST
  // transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
  
  radio.setTxPower(6, false);
}

int16_t packet_num = 0;

void loop() {
  
  cur_long = 123;
  cur_lat = 789;

  delay(2000);

  //char read_data = 0; // read GPS data to verify it exists
  //
  //  if (0) { // if we received something
  //    // use read data to check if we recieved proper gps stuff (i think?)
  //  }
  

  char packet[20] = {cur_long, cur_lat}; // add callsign of ben
  // call sign is 6 byte (KD9UGU)
  // 

  delay(10);

  radio.send((uint8_t *)packet, 20);

  delay(10);

  radio.waitPacketSent();

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

}
*/

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// TRANCEIVER CODE END


// RECEIVER CODE START
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------


#include <Wire.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <RH_RF95.h>
#define RF95_FREQ 434.0
#define RFM96_RST 4

#define RFM95_INT 3
#define RFM95_CS 8

RH_RF95 radio(RFM95_CS, RFM95_INT);

long cur_long;
long cur_lat;

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

  
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST
  // transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
  
  radio.setTxPower(6, false);
}

void loop() {

Serial.begin(115200);

if (radio.available())
{

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; // maybe make smaller if needed (255-4 bytes currently)
  uint8_t len = sizeof(buf);

  if (radio.recv(buf, &len))
  {
    // example does display tap here fwiw

    uint8_t data[] = "And hello to back to you fdhjdjf";
    radio.send(data, sizeof(data));
    radio.waitPacketSent();
    delay(1000);

  }

}

}


// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// RECEIVER CODE END
