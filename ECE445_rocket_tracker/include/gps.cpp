#include "gps.h"
// #include "SparkFun_Ublox_Arduino_Library.h"


SFE_UBLOX_GPS::SFE_UBLOX_GPS(void)
{
  // Constructor
  currentGeofenceParams.numFences = 0; // Zero the number of geofences currently in use
  moduleQueried.versionNumber = false;

  if (checksumFailurePin >= 0)
  {
    pinMode((uint8_t)checksumFailurePin, OUTPUT);
    digitalWrite((uint8_t)checksumFailurePin, HIGH);
  }

  //Define the size of the I2C buffer based on the platform the user has
  //In general we found that most platforms use 32 bytes as the I2C buffer size. We could
  //implement platform gaurds here but as you can see, none currently benefit from >32
  //so we'll leave it up to the user to set it using setI2CTransactionSize if they will benefit from it
  // //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

  // i2cTransactionSize = 32;

  // #elif defined(__SAMD21G18A__)

  // i2cTransactionSize = 32;

  //#elif __MK20DX256__
  //Teensy

  // #elif defined(ARDUINO_ARCH_ESP32)

  // i2cTransactionSize = 32; //The ESP32 has an I2C buffer length of 128. We reduce it to 32 bytes to increase stability with the module

  // #endif
}

//Get the current latitude in degrees
//Returns a long representing the number of degrees *10^-7
int32_t SFE_UBLOX_GPS::getLatitude(uint16_t maxWait)
{
  if (moduleQueried.latitude == false)
    getPVT(maxWait);
  moduleQueried.latitude = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (latitude);
}

//Get the current longitude in degrees
//Returns a long representing the number of degrees *10^-7
int32_t SFE_UBLOX_GPS::getLongitude(uint16_t maxWait)
{
  if (moduleQueried.longitude == false)
    getPVT(maxWait);
  moduleQueried.longitude = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (longitude);
}

//Get the heading of motion (as opposed to heading of car) in degrees * 10^-5
int32_t SFE_UBLOX_GPS::getHeading(uint16_t maxWait)
{
  if (moduleQueried.headingOfMotion == false)
    getPVT(maxWait);
  moduleQueried.headingOfMotion = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (headingOfMotion);
}


//Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
//This is called before we send a command message
void SFE_UBLOX_GPS::calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

//Initialize the Serial port
boolean SFE_UBLOX_GPS::begin(TwoWire &wirePort, uint8_t deviceAddress)
{
  commType = COMM_TYPE_I2C;
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  //We expect caller to begin their I2C port, with the speed of their choice external to the library
  //But if they forget, we start the hardware here.

  //We're moving away from the practice of starting Wire hardware in a library. This is to avoid cross platform issues.
  //ie, there are some platforms that don't handle multiple starts to the wire hardware. Also, every time you start the wire
  //hardware the clock speed reverts back to 100kHz regardless of previous Wire.setClocks().
  //_i2cPort->begin();

  _gpsI2Caddress = deviceAddress; //Store the I2C address from user

  // Attempt isConnected up to 3 times if required
  boolean success = isConnected();

  if (!success)
    success = isConnected();

  if (!success)
    success = isConnected();

  return (success);
}

//Initialize the Serial port
boolean SFE_UBLOX_GPS::begin(Stream &serialPort)
{
  commType = COMM_TYPE_SERIAL;
  _serialPort = &serialPort; //Grab which port the user wants us to use

  // Attempt isConnected up to 3 times if required
  boolean success = isConnected();

  if (!success)
    success = isConnected();

  if (!success)
    success = isConnected();

  return (success);
}



//Returns false if sensor fails to respond to I2C traffic
sfe_ublox_status_e SFE_UBLOX_GPS::sendI2cCommand(ubxPacket *outgoingUBX, uint16_t maxWait)
{
  //Point at 0xFF data register
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
  _i2cPort->write(0xFF);
  if (_i2cPort->endTransmission(false) != 0)         //Don't release bus
    return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK

  //Write header bytes
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
  _i2cPort->write(UBX_SYNCH_1);                         //μ - oh ublox, you're funny. I will call you micro-blox from now on.
  _i2cPort->write(UBX_SYNCH_2);                         //b
  _i2cPort->write(outgoingUBX->cls);
  _i2cPort->write(outgoingUBX->id);
  _i2cPort->write(outgoingUBX->len & 0xFF);     //LSB
  _i2cPort->write(outgoingUBX->len >> 8);       //MSB
  if (_i2cPort->endTransmission(false) != 0)    //Do not release bus
    return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK

  //Write payload. Limit the sends into 32 byte chunks
  //This code based on ublox: https://forum.u-blox.com/index.php/20528/how-to-use-i2c-to-get-the-nmea-frames
  uint16_t bytesToSend = outgoingUBX->len;

  //"The number of data bytes must be at least 2 to properly distinguish
  //from the write access to set the address counter in random read accesses."
  uint16_t startSpot = 0;
  while (bytesToSend > 1)
  {
    uint8_t len = bytesToSend;
    if (len > i2cTransactionSize)
      len = i2cTransactionSize;

    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
    //_i2cPort->write(outgoingUBX->payload, len); //Write a portion of the payload to the bus

    for (uint16_t x = 0; x < len; x++)
      _i2cPort->write(outgoingUBX->payload[startSpot + x]); //Write a portion of the payload to the bus

    if (_i2cPort->endTransmission(false) != 0)    //Don't release bus
      return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK

    //*outgoingUBX->payload += len; //Move the pointer forward
    startSpot += len; //Move the pointer forward
    bytesToSend -= len;
  }

  //Write checksum
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
  if (bytesToSend == 1)
    _i2cPort->write(outgoingUBX->payload, 1);
  _i2cPort->write(outgoingUBX->checksumA);
  _i2cPort->write(outgoingUBX->checksumB);

  //All done transmitting bytes. Release bus.
  if (_i2cPort->endTransmission() != 0)
    return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK
  return (SFE_UBLOX_STATUS_SUCCESS);
}

//Given a packet and payload, send everything including CRC bytesA via Serial port
void SFE_UBLOX_GPS::sendSerialCommand(ubxPacket *outgoingUBX)
{
  //Write header bytes
  _serialPort->write(UBX_SYNCH_1); //μ - oh ublox, you're funny. I will call you micro-blox from now on.
  _serialPort->write(UBX_SYNCH_2); //b
  _serialPort->write(outgoingUBX->cls);
  _serialPort->write(outgoingUBX->id);
  _serialPort->write(outgoingUBX->len & 0xFF); //LSB
  _serialPort->write(outgoingUBX->len >> 8);   //MSB

  //Write payload.
  for (int i = 0; i < outgoingUBX->len; i++)
  {
    _serialPort->write(outgoingUBX->payload[i]);
  }

  //Write checksum
  _serialPort->write(outgoingUBX->checksumA);
  _serialPort->write(outgoingUBX->checksumB);
}

boolean SFE_UBLOX_GPS::checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (commType == COMM_TYPE_I2C)
    return (checkUbloxI2C(incomingUBX, requestedClass, requestedID));
  else if (commType == COMM_TYPE_SERIAL)
    return (checkUbloxSerial(incomingUBX, requestedClass, requestedID));
  return false;
}

//Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got an ACK and a valid packetCfg (module is responding with register content)
//Returns SFE_UBLOX_STATUS_DATA_SENT if we got an ACK and no packetCfg (no valid packetCfg needed, module absorbs new register data)
//Returns SFE_UBLOX_STATUS_FAIL if something very bad happens (e.g. a double checksum failure)
//Returns SFE_UBLOX_STATUS_COMMAND_NACK if the packet was not-acknowledged (NACK)
//Returns SFE_UBLOX_STATUS_CRC_FAIL if we had a checksum failure
//Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
//Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an ACK and a valid packetCfg but that the packetCfg has been
// or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e SFE_UBLOX_GPS::waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() - startTime < maxTime)
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) //See if new data is available. Process bytes as they come in.
    {
      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: valid data and valid ACK received after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data and a correct ACK!
      }

      // We can be confident that the data packet (if we are going to get one) will always arrive
      // before the matching ACK. So if we sent a config packet which only produces an ACK
      // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
      // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
      // as these may have been changed by a PVT packet.
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: no data and valid ACK after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_SENT); //We got an ACK but no data...
      }

      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: data being OVERWRITTEN after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
      // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
      else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: CRC failed after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_CRC_FAIL); //Checksum fail
      }

      // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
      // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
      // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
      // but outgoingUBX->cls and outgoingUBX->id would not match...
      // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
      // the packet was definitely NACK'd otherwise we are possibly just guessing...
      // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
      else if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_COMMAND_NACK); //We received a NACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
      // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
      // If we were playing safe, we should return FAIL instead
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: VALID data and INVALID ACK received after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
      // then we return a FAIL. This must be a double checksum failure?
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: INVALID data and INVALID ACK received after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_FAIL); //We received invalid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
      // then the ACK has not yet been received and we should keep waiting for it
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: valid data after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec. Waiting for ACK."));
        }
      }

    } //checkUbloxInternal == true

    delayMicroseconds(500);
  } //while (millis() - startTime < maxTime)

  // We have timed out...
  // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
  // even though we did not get an ACK
  if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
  {
    if (_printDebug == true)
    {
      _debugSerial->print(F("waitForACKResponse: TIMEOUT with valid data after "));
      _debugSerial->print(millis() - startTime);
      _debugSerial->println(F(" msec. "));
    }
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data... But no ACK!
  }

  if (_printDebug == true)
  {
    _debugSerial->print(F("waitForACKResponse: TIMEOUT after "));
    _debugSerial->print(millis() - startTime);
    _debugSerial->println(F(" msec."));
  }

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

//For non-CFG queries no ACK is sent so we use this function
//Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of response data that has CLS/ID match to our query packet
//Returns SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID match to our query packet
//Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
//Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg but that the packetCfg has been
// or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e SFE_UBLOX_GPS::waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() - startTime < maxTime)
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) //See if new data is available. Process bytes as they come in.
    {

      // If outgoingUBX->classAndIDmatch is VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForNoACKResponse: valid data with CLS/ID match after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data!
      }

      // If the outgoingUBX->classAndIDmatch is VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForNoACKResponse: data being OVERWRITTEN after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If outgoingUBX->classAndIDmatch is NOT_DEFINED
      // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForNoACKResponse: valid but UNWANTED data after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->print(F(" msec. Class: "));
        //   _debugSerial->print(outgoingUBX->cls);
        //   _debugSerial->print(F(" ID: "));
        //   _debugSerial->print(outgoingUBX->id);
        // }
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
      else if (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForNoACKResponse: CLS/ID match but failed CRC after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_CRC_FAIL); //We received invalid data
      }
    }

    delayMicroseconds(500);
  }

  if (_printDebug == true)
  {
    _debugSerial->print(F("waitForNoACKResponse: TIMEOUT after "));
    _debugSerial->print(millis() - startTime);
    _debugSerial->println(F(" msec. No packet received."));
  }

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

//Given a packet and payload, send everything including CRC bytes via I2C port
sfe_ublox_status_e SFE_UBLOX_GPS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait)
{
  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

  calcChecksum(outgoingUBX); //Sets checksum A and B bytes of the packet

  if (_printDebug == true)
  {
    _debugSerial->print(F("\nSending: "));
    printPacket(outgoingUBX);
  }

  if (commType == COMM_TYPE_I2C)
  {
    retVal = sendI2cCommand(outgoingUBX, maxWait);
    if (retVal != SFE_UBLOX_STATUS_SUCCESS)
    {
      if (_printDebug == true)
      {
        _debugSerial->println(F("Send I2C Command failed"));
      }
      return retVal;
    }
  }
  else if (commType == COMM_TYPE_SERIAL)
  {
    sendSerialCommand(outgoingUBX);
  }

  if (maxWait > 0)
  {
    //Depending on what we just sent, either we need to look for an ACK or not
    if (outgoingUBX->cls == UBX_CLASS_CFG)
    {
      if (_printDebug == true)
      {
        _debugSerial->println(F("sendCommand: Waiting for ACK response"));
      }
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); //Wait for Ack response
    }
    else
    {
      if (_printDebug == true)
      {
        _debugSerial->println(F("sendCommand: Waiting for No ACK response"));
      }
      retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); //Wait for Ack response
    }
  }
  return retVal;
}

//Polls I2C for data, passing any new bytes to process()
//Returns true if new bytes are available
boolean SFE_UBLOX_GPS::checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (millis() - lastCheck >= i2cPollingWait)
  {
    //Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _i2cPort->beginTransmission(_gpsI2Caddress);
    _i2cPort->write(0xFD);                     //0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
      return (false);                          //Sensor did not ACK

    _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)2);
    if (_i2cPort->available())
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      if (lsb == 0xFF)
      {
        //I believe this is a u-blox bug. Device should never present an 0xFF.
        if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
        {
          _debugSerial->println(F("checkUbloxI2C: u-blox bug, length lsb is 0xFF"));
        }
        if (checksumFailurePin >= 0)
        {
          digitalWrite((uint8_t)checksumFailurePin, LOW);
          delay(10);
          digitalWrite((uint8_t)checksumFailurePin, HIGH);
        }
        lastCheck = millis(); //Put off checking to avoid I2C bus traffic
        return (false);
      }
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    if (bytesAvailable == 0)
    {
      if (_printDebug == true)
      {
        _debugSerial->println(F("checkUbloxI2C: OK, zero bytes available"));
      }
      lastCheck = millis(); //Put off checking to avoid I2C bus traffic
      return (false);
    }

    //Check for undocumented bit error. We found this doing logic scans.
    //This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
    //then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
    if (bytesAvailable & ((uint16_t)1 << 15))
    {
      //Clear the MSbit
      bytesAvailable &= ~((uint16_t)1 << 15);

      if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
      {
        _debugSerial->print(F("checkUbloxI2C: Bytes available error:"));
        _debugSerial->println(bytesAvailable);
        if (checksumFailurePin >= 0)
        {
          digitalWrite((uint8_t)checksumFailurePin, LOW);
          delay(10);
          digitalWrite((uint8_t)checksumFailurePin, HIGH);
        }
      }
    }

    if (bytesAvailable > 100)
    {
      if (_printDebug == true)
      {
        _debugSerial->print(F("checkUbloxI2C: Large packet of "));
        _debugSerial->print(bytesAvailable);
        _debugSerial->println(F(" bytes received"));
      }
    }
    else
    {
      if (_printDebug == true)
      {
        _debugSerial->print(F("checkUbloxI2C: Reading "));
        _debugSerial->print(bytesAvailable);
        _debugSerial->println(F(" bytes"));
      }
    }

    while (bytesAvailable)
    {
      _i2cPort->beginTransmission(_gpsI2Caddress);
      _i2cPort->write(0xFF);                     //0xFF is the register to read data from
      if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
        return (false);                          //Sensor did not ACK

      //Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable;
      if (bytesToRead > i2cTransactionSize)
        bytesToRead = i2cTransactionSize;

    TRY_AGAIN:

      _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)bytesToRead);
      if (_i2cPort->available())
      {
        for (uint16_t x = 0; x < bytesToRead; x++)
        {
          uint8_t incoming = _i2cPort->read(); //Grab the actual character

          //Check to see if the first read is 0x7F. If it is, the module is not ready
          //to respond. Stop, wait, and try again
          if (x == 0)
          {
            if (incoming == 0x7F)
            {
              if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
              {
                _debugSerial->println(F("checkUbloxU2C: u-blox error, module not ready with data"));
              }
              delay(5); //In logic analyzation, the module starting responding after 1.48ms
              if (checksumFailurePin >= 0)
              {
                digitalWrite((uint8_t)checksumFailurePin, LOW);
                delay(10);
                digitalWrite((uint8_t)checksumFailurePin, HIGH);
              }
              goto TRY_AGAIN;
            }
          }

          process(incoming, incomingUBX, requestedClass, requestedID); //Process this valid character
        }
      }
      else
        return (false); //Sensor did not respond

      bytesAvailable -= bytesToRead;
    }
  }

  return (true);

} //end checkUbloxI2C()


//Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
//This is used when receiving messages from module
void SFE_UBLOX_GPS::addToChecksum(uint8_t incoming)
{
  rollingChecksumA += incoming;
  rollingChecksumB += rollingChecksumA;
}

//Pretty prints the current ubxPacket
void SFE_UBLOX_GPS::printPacket(ubxPacket *packet)
{
  if (_printDebug == true)
  {
    _debugSerial->print(F("CLS:"));
    if (packet->cls == UBX_CLASS_NAV) //1
      _debugSerial->print(F("NAV"));
    else if (packet->cls == UBX_CLASS_ACK) //5
      _debugSerial->print(F("ACK"));
    else if (packet->cls == UBX_CLASS_CFG) //6
      _debugSerial->print(F("CFG"));
    else if (packet->cls == UBX_CLASS_MON) //0x0A
      _debugSerial->print(F("MON"));
    else
    {
      _debugSerial->print(F("0x"));
      _debugSerial->print(packet->cls, HEX);
    }

    _debugSerial->print(F(" ID:"));
    if (packet->cls == UBX_CLASS_NAV && packet->id == UBX_NAV_PVT)
      _debugSerial->print(F("PVT"));
    else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_RATE)
      _debugSerial->print(F("RATE"));
    else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_CFG)
      _debugSerial->print(F("SAVE"));
    else
    {
      _debugSerial->print(F("0x"));
      _debugSerial->print(packet->id, HEX);
    }

    _debugSerial->print(F(" Len: 0x"));
    _debugSerial->print(packet->len, HEX);

    // Only print the payload is ignoreThisPayload is false otherwise
    // we could be printing gibberish from beyond the end of packetBuf
    if (ignoreThisPayload == false)
    {
      _debugSerial->print(F(" Payload:"));

      for (int x = 0; x < packet->len; x++)
      {
        _debugSerial->print(F(" "));
        _debugSerial->print(packet->payload[x], HEX);
      }
    }
    else
    {
      _debugSerial->print(F(" Payload: IGNORED"));
    }
    _debugSerial->println();
  }
}


//Given a spot in the payload array, extract four bytes and build a long
uint32_t SFE_UBLOX_GPS::extractLong(uint8_t spotToStart)
{
  uint32_t val = 0;
  val |= (uint32_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (uint32_t)payloadCfg[spotToStart + 1] << 8 * 1;
  val |= (uint32_t)payloadCfg[spotToStart + 2] << 8 * 2;
  val |= (uint32_t)payloadCfg[spotToStart + 3] << 8 * 3;
  return (val);
}

//Just so there is no ambiguity about whether a uint32_t will cast to a int32_t correctly...
int32_t SFE_UBLOX_GPS::extractSignedLong(uint8_t spotToStart)
{
  union // Use a union to convert from uint32_t to int32_t
  {
      uint32_t unsignedLong;
      int32_t signedLong;
  } unsignedSigned;

  unsignedSigned.unsignedLong = extractLong(spotToStart);
  return (unsignedSigned.signedLong);
}

//Given a spot in the payload array, extract two bytes and build an int
uint16_t SFE_UBLOX_GPS::extractInt(uint8_t spotToStart)
{
  uint16_t val = 0;
  val |= (uint16_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (uint16_t)payloadCfg[spotToStart + 1] << 8 * 1;
  return (val);
}

//Just so there is no ambiguity about whether a uint16_t will cast to a int16_t correctly...
int16_t SFE_UBLOX_GPS::extractSignedInt(int8_t spotToStart)
{
  union // Use a union to convert from uint16_t to int16_t
  {
      uint16_t unsignedInt;
      int16_t signedInt;
  } stSignedInt;

  stSignedInt.unsignedInt = extractInt(spotToStart);
  return (stSignedInt.signedInt);
}

//Given a spot, extract a byte from the payload
uint8_t SFE_UBLOX_GPS::extractByte(uint8_t spotToStart)
{
  return (payloadCfg[spotToStart]);
}

//Given a spot, extract a signed 8-bit value from the payload
int8_t SFE_UBLOX_GPS::extractSignedChar(uint8_t spotToStart)
{
  return ((int8_t)payloadCfg[spotToStart]);
}

//Once a packet has been received and validated, identify this packet's class/id and update internal flags
//Note: if the user requests a PVT or a HPPOSLLH message using a custom packet, the data extraction will
//      not work as expected beacuse extractLong etc are hardwired to packetCfg payloadCfg. Ideally
//      extractLong etc should be updated so they receive a pointer to the packet buffer.
void SFE_UBLOX_GPS::processUBXpacket(ubxPacket *msg)
{
  switch (msg->cls)
  {
  case UBX_CLASS_NAV:
    if (msg->id == UBX_NAV_PVT && msg->len == 92)
    {
      //Parse various byte fields into global vars
      constexpr int startingSpot = 0; //fixed value used in processUBX

      // timeOfWeek = extractLong(0);
      // gpsMillisecond = extractLong(0) % 1000; //Get last three digits of iTOW
      // gpsYear = extractInt(4);
      // gpsMonth = extractByte(6);
      // gpsDay = extractByte(7);
      // gpsHour = extractByte(8);
      // gpsMinute = extractByte(9);
      // gpsSecond = extractByte(10);
      // gpsDateValid = extractByte(11) & 0x01;
      // gpsTimeValid = (extractByte(11) & 0x02) >> 1;
      // gpsNanosecond = extractSignedLong(16); //Includes milliseconds

      // fixType = extractByte(20 - startingSpot);
      gnssFixOk = extractByte(21 - startingSpot) & 0x1; //Get the 1st bit
      diffSoln = (extractByte(21 - startingSpot) >> 1) & 0x1; //Get the 2nd bit
      // carrierSolution = extractByte(21 - startingSpot) >> 6; //Get 6th&7th bits of this byte
      headVehValid = (extractByte(21 - startingSpot) >> 5) & 0x1; // Get the 5th bit
      // SIV = extractByte(23 - startingSpot);
      longitude = extractSignedLong(24 - startingSpot);
      latitude = extractSignedLong(28 - startingSpot);
      altitude = extractSignedLong(32 - startingSpot);
      altitudeMSL = extractSignedLong(36 - startingSpot);
      // horizontalAccEst = extractLong(40 - startingSpot);
      // verticalAccEst = extractLong(44 - startingSpot);
      // nedNorthVel = extractSignedLong(48 - startingSpot);
      // nedEastVel = extractSignedLong(52 - startingSpot);
      // nedDownVel = extractSignedLong(56 - startingSpot);
      // groundSpeed = extractSignedLong(60 - startingSpot);
      headingOfMotion = extractSignedLong(64 - startingSpot);
      // speedAccEst = extractLong(68 - startingSpot);
      // headingAccEst = extractLong(72 - startingSpot);
      pDOP = extractInt(76 - startingSpot);
      invalidLlh = extractByte(78 - startingSpot) & 0x1;
      // headVeh = extractSignedLong(84 - startingSpot);
      // magDec = extractSignedInt(88 - startingSpot);
      // magAcc = extractInt(90 - startingSpot);

      //Mark all datums as fresh (not read before)
      moduleQueried.gpsiTOW = true;
      moduleQueried.gpsYear = true;
      moduleQueried.gpsMonth = true;
      moduleQueried.gpsDay = true;
      moduleQueried.gpsHour = true;
      moduleQueried.gpsMinute = true;
      moduleQueried.gpsSecond = true;
      moduleQueried.gpsDateValid = true;
      moduleQueried.gpsTimeValid = true;
      moduleQueried.gpsNanosecond = true;

      moduleQueried.all = true;
      moduleQueried.gnssFixOk = true;
      moduleQueried.diffSoln = true;
      moduleQueried.headVehValid = true;
      moduleQueried.longitude = true;
      moduleQueried.latitude = true;
      moduleQueried.altitude = true;
      moduleQueried.altitudeMSL = true;
      moduleQueried.horizontalAccEst = true;
      moduleQueried.verticalAccEst = true;
      moduleQueried.nedNorthVel = true;
      moduleQueried.nedEastVel = true;
      moduleQueried.nedDownVel = true;
      moduleQueried.SIV = true;
      moduleQueried.fixType = true;
      moduleQueried.carrierSolution = true;
      moduleQueried.groundSpeed = true;
      moduleQueried.headingOfMotion = true;
      moduleQueried.speedAccEst = true;
      moduleQueried.headingAccEst = true;
      moduleQueried.pDOP = true;
      moduleQueried.invalidLlh = true;
      moduleQueried.headVeh = true;
      moduleQueried.magDec = true;
      moduleQueried.magAcc = true;
    }
    else if (msg->id == UBX_NAV_HPPOSLLH && msg->len == 36)
    {
      // timeOfWeek = extractLong(4);
      highResLongitude = extractSignedLong(8);
      highResLatitude = extractSignedLong(12);
      elipsoid = extractSignedLong(16);
      meanSeaLevel = extractSignedLong(20);
      highResLongitudeHp = extractSignedChar(24);
      highResLatitudeHp = extractSignedChar(25);
      elipsoidHp = extractSignedChar(26);
      meanSeaLevelHp = extractSignedChar(27);
      horizontalAccuracy = extractLong(28);
      verticalAccuracy = extractLong(32);

      highResModuleQueried.all = true;
      highResModuleQueried.highResLatitude = true;
      highResModuleQueried.highResLatitudeHp = true;
      highResModuleQueried.highResLongitude = true;
      highResModuleQueried.highResLongitudeHp = true;
      highResModuleQueried.elipsoid = true;
      highResModuleQueried.elipsoidHp = true;
      highResModuleQueried.meanSeaLevel = true;
      highResModuleQueried.meanSeaLevelHp = true;
      highResModuleQueried.geoidSeparation = true;
      highResModuleQueried.horizontalAccuracy = true;
      highResModuleQueried.verticalAccuracy = true;
      moduleQueried.gpsiTOW = true; // this can arrive via HPPOS too.

/*
      if (_printDebug == true)
      {
        _debugSerial->print(F("Sec: "));
        _debugSerial->print(((float)extractLong(4)) / 1000.0f);
        _debugSerial->print(F(" "));
        _debugSerial->print(F("LON: "));
        _debugSerial->print(((float)(int32_t)extractLong(8)) / 10000000.0f);
        _debugSerial->print(F(" "));
        _debugSerial->print(F("LAT: "));
        _debugSerial->print(((float)(int32_t)extractLong(12)) / 10000000.0f);
        _debugSerial->print(F(" "));
        _debugSerial->print(F("ELI M: "));
        _debugSerial->print(((float)(int32_t)extractLong(16)) / 1000.0f);
        _debugSerial->print(F(" "));
        _debugSerial->print(F("MSL M: "));
        _debugSerial->print(((float)(int32_t)extractLong(20)) / 1000.0f);
        _debugSerial->print(F(" "));
        _debugSerial->print(F("LON HP: "));
        _debugSerial->print(extractSignedChar(24));
        _debugSerial->print(F(" "));
        _debugSerial->print(F("LAT HP: "));
        _debugSerial->print(extractSignedChar(25));
        _debugSerial->print(F(" "));
        _debugSerial->print(F("ELI HP: "));
        _debugSerial->print(extractSignedChar(26));
        _debugSerial->print(F(" "));
        _debugSerial->print(F("MSL HP: "));
        _debugSerial->print(extractSignedChar(27));
        _debugSerial->print(F(" "));
        _debugSerial->print(F("HA 2D M: "));
        _debugSerial->print(((float)(int32_t)extractLong(28)) / 10000.0f);
        _debugSerial->print(F(" "));
        _debugSerial->print(F("VERT M: "));
        _debugSerial->println(((float)(int32_t)extractLong(32)) / 10000.0f);
      }
*/
    }
    else if (msg->id == UBX_NAV_DOP && msg->len == 18)
    {
      geometricDOP = extractInt(4);
      positionDOP = extractInt(6);
      // timeDOP = extractInt(8);
      // verticalDOP = extractInt(10);
      // horizontalDOP = extractInt(12);
      // northingDOP = extractInt(14);
      // eastingDOP = extractInt(16);
      dopModuleQueried.all = true;
      dopModuleQueried.geometricDOP = true;
      dopModuleQueried.positionDOP = true;
      dopModuleQueried.timeDOP = true;
      dopModuleQueried.verticalDOP = true;
      dopModuleQueried.horizontalDOP = true;
      dopModuleQueried.northingDOP = true;
      dopModuleQueried.eastingDOP = true;
    }
    break;
  case UBX_CLASS_HNR:
    if (msg->id == UBX_HNR_ATT && msg->len == 32)
    {
      //Parse various byte fields into global vars
      hnrAtt.iTOW = extractLong(0);
      hnrAtt.roll = extractSignedLong(8);
      hnrAtt.pitch = extractSignedLong(12);
      hnrAtt.heading = extractSignedLong(16);
      hnrAtt.accRoll = extractLong(20);
      hnrAtt.accPitch = extractLong(24);
      hnrAtt.accHeading = extractLong(28);

      hnrAttQueried = true;
    }
    else if (msg->id == UBX_HNR_INS && msg->len == 36)
    {
      //Parse various byte fields into global vars
      // hnrVehDyn.iTOW = extractLong(8);
      // hnrVehDyn.xAngRate = extractSignedLong(12);
      // hnrVehDyn.yAngRate = extractSignedLong(16);
      // hnrVehDyn.zAngRate = extractSignedLong(20);
      // hnrVehDyn.xAccel = extractSignedLong(24);
      // hnrVehDyn.yAccel = extractSignedLong(28);
      // hnrVehDyn.zAccel = extractSignedLong(32);

      // uint32_t bitfield0 = extractLong(0);
      // hnrVehDyn.xAngRateValid = (bitfield0 & 0x00000100) > 0;
      // hnrVehDyn.yAngRateValid = (bitfield0 & 0x00000200) > 0;
      // hnrVehDyn.zAngRateValid = (bitfield0 & 0x00000400) > 0;
      // hnrVehDyn.xAccelValid = (bitfield0 & 0x00000800) > 0;
      // hnrVehDyn.yAccelValid = (bitfield0 & 0x00001000) > 0;
      // hnrVehDyn.zAccelValid = (bitfield0 & 0x00002000) > 0;

      hnrDynQueried = true;
    }
    else if (msg->id == UBX_HNR_PVT && msg->len == 72)
    {
      //Parse various byte fields into global vars
      // hnrPVT.iTOW = extractLong(0);
      // hnrPVT.year = extractInt(4);
      // hnrPVT.month = extractByte(6);
      // hnrPVT.day = extractByte(7);
      // hnrPVT.hour = extractByte(8);
      // hnrPVT.min = extractByte(9);
      // hnrPVT.sec = extractByte(10);
      // hnrPVT.nano = extractSignedLong(12);
      hnrPVT.gpsFix = extractByte(16);
      hnrPVT.lon = extractSignedLong(20);
      hnrPVT.lat = extractSignedLong(24);
      hnrPVT.height = extractSignedLong(28);
      // hnrPVT.hMSL = extractSignedLong(32);
      // hnrPVT.gSpeed = extractSignedLong(36);
      // hnrPVT.speed = extractSignedLong(40);
      // hnrPVT.headMot = extractSignedLong(44);
      // hnrPVT.headVeh = extractSignedLong(48);
      // hnrPVT.hAcc = extractLong(52);
      // hnrPVT.vAcc = extractLong(56);
      // hnrPVT.sAcc = extractLong(60);
      // hnrPVT.headAcc = extractLong(64);

      uint8_t valid = extractByte(11);
      // hnrPVT.validDate = (valid & 0x01) > 0;
      // hnrPVT.validTime = (valid & 0x02) > 0;
      // hnrPVT.fullyResolved = (valid & 0x04) > 0;

      uint8_t flags = extractByte(17);
      hnrPVT.gpsFixOK = (flags & 0x01) > 0;
      hnrPVT.diffSoln = (flags & 0x02) > 0;
      // hnrPVT.WKNSET = (flags & 0x04) > 0;
      // hnrPVT.TOWSET = (flags & 0x08) > 0;
      hnrPVT.headVehValid = (flags & 0x10) > 0;

      hnrPVTQueried = true;
    }
  }
}

//Given a character, file it away into the uxb packet structure
//Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
//The payload portion of the packet can be 100s of bytes but the max array
//size is MAX_PAYLOAD_SIZE bytes. startingSpot can be set so we only record
//a subset of bytes within a larger packet.
void SFE_UBLOX_GPS::processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
   size_t max_payload_size = (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG) ? MAX_PAYLOAD_SIZE : 2;
   bool overrun = false;

  //Add all incoming bytes to the rolling checksum
  //Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
  if (incomingUBX->counter < incomingUBX->len + 4)
    addToChecksum(incoming);

  if (incomingUBX->counter == 0)
  {
    incomingUBX->cls = incoming;
  }
  else if (incomingUBX->counter == 1)
  {
    incomingUBX->id = incoming;
  }
  else if (incomingUBX->counter == 2) //Len LSB
  {
    incomingUBX->len = incoming;
  }
  else if (incomingUBX->counter == 3) //Len MSB
  {
    incomingUBX->len |= incoming << 8;
  }
  else if (incomingUBX->counter == incomingUBX->len + 4) //ChecksumA
  {
    incomingUBX->checksumA = incoming;
  }
  else if (incomingUBX->counter == incomingUBX->len + 5) //ChecksumB
  {
    incomingUBX->checksumB = incoming;

    currentSentence = NONE; //We're done! Reset the sentence to being looking for a new start char

    //Validate this sentence
    if ((incomingUBX->checksumA == rollingChecksumA) && (incomingUBX->checksumB == rollingChecksumB))
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid

      // Let's check if the class and ID match the requestedClass and requestedID
      // Remember - this could be a data packet or an ACK packet
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_NACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
        if (_printDebug == true)
        {
          _debugSerial->print(F("processUBX: NACK received: Requested Class: 0x"));
          _debugSerial->print(incomingUBX->payload[0], HEX);
          _debugSerial->print(F(" Requested ID: 0x"));
          _debugSerial->println(incomingUBX->payload[1], HEX);
        }
      }

      //This is not an ACK and we do not have a complete class and ID match
      //So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
      else if ((incomingUBX->cls == requestedClass) &&
        (((incomingUBX->id == UBX_NAV_PVT) && (requestedID == UBX_NAV_HPPOSLLH || requestedID == UBX_NAV_DOP)) ||
        ((incomingUBX->id == UBX_NAV_HPPOSLLH) && (requestedID == UBX_NAV_PVT || requestedID == UBX_NAV_DOP)) ||
        ((incomingUBX->id == UBX_NAV_DOP) && (requestedID == UBX_NAV_PVT || requestedID == UBX_NAV_HPPOSLLH))))
      {
        // This isn't the message we are looking for...
        // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
        if (_printDebug == true)
        {
          _debugSerial->print(F("processUBX: auto NAV PVT/HPPOSLLH/DOP collision: Requested ID: 0x"));
          _debugSerial->print(requestedID, HEX);
          _debugSerial->print(F(" Message ID: 0x"));
          _debugSerial->println(incomingUBX->id, HEX);
        }
      }
      // Let's do the same for the HNR messages
      else if ((incomingUBX->cls == requestedClass) &&
        (((incomingUBX->id == UBX_HNR_ATT) && (requestedID == UBX_HNR_INS || requestedID == UBX_HNR_PVT)) ||
         ((incomingUBX->id == UBX_HNR_INS) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_PVT)) ||
         ((incomingUBX->id == UBX_HNR_PVT) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_INS))))
       {
         // This isn't the message we are looking for...
         // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
         if (_printDebug == true)
         {
           _debugSerial->print(F("processUBX: auto HNR ATT/INS/PVT collision: Requested ID: 0x"));
           _debugSerial->print(requestedID, HEX);
           _debugSerial->print(F(" Message ID: 0x"));
           _debugSerial->println(incomingUBX->id, HEX);
         }
       }

      if (_printDebug == true)
      {
        _debugSerial->print(F("Incoming: Size: "));
        _debugSerial->print(incomingUBX->len);
        _debugSerial->print(F(" Received: "));
        printPacket(incomingUBX);

        if (incomingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg now valid"));
        }
        if (packetAck.valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck now valid"));
        }
        if (incomingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg classAndIDmatch"));
        }
        if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck classAndIDmatch"));
        }
      }

      //We've got a valid packet, now do something with it but only if ignoreThisPayload is false
      if (ignoreThisPayload == false)
      {
        processUBXpacket(incomingUBX);
      }
    }
    else // Checksum failure
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

      // Let's check if the class and ID match the requestedClass and requestedID.
      // This is potentially risky as we are saying that we saw the requested Class and ID
      // but that the packet checksum failed. Potentially it could be the class or ID bytes
      // that caused the checksum error!
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }
      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }

      if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
      {
        //Drive an external pin to allow for easier logic analyzation
        if (checksumFailurePin >= 0)
        {
          digitalWrite((uint8_t)checksumFailurePin, LOW);
          delay(10);
          digitalWrite((uint8_t)checksumFailurePin, HIGH);
        }

        _debugSerial->print(F("Checksum failed:"));
        _debugSerial->print(F(" checksumA: "));
        _debugSerial->print(incomingUBX->checksumA);
        _debugSerial->print(F(" checksumB: "));
        _debugSerial->print(incomingUBX->checksumB);

        _debugSerial->print(F(" rollingChecksumA: "));
        _debugSerial->print(rollingChecksumA);
        _debugSerial->print(F(" rollingChecksumB: "));
        _debugSerial->print(rollingChecksumB);
        _debugSerial->println();

        _debugSerial->print(F("Failed  : "));
        _debugSerial->print(F("Size: "));
        _debugSerial->print(incomingUBX->len);
        _debugSerial->print(F(" Received: "));
        printPacket(incomingUBX);
      }
    }
  }
  else //Load this byte into the payload array
  {
    //If a UBX_NAV_PVT packet comes in asynchronously, we need to fudge the startingSpot
    uint16_t startingSpot = incomingUBX->startingSpot;
    if (incomingUBX->cls == UBX_CLASS_NAV && incomingUBX->id == UBX_NAV_PVT)
      startingSpot = 0;
    // Check if this is payload data which should be ignored
    if (ignoreThisPayload == false)
    {
      //Begin recording if counter goes past startingSpot
      if ((incomingUBX->counter - 4) >= startingSpot)
      {
        //Check to see if we have room for this byte
        if (((incomingUBX->counter - 4) - startingSpot) < max_payload_size) //If counter = 208, starting spot = 200, we're good to record.
        {
          incomingUBX->payload[incomingUBX->counter - 4 - startingSpot] = incoming; //Store this byte into payload array
        }
        else
        {
          overrun = true;
        }
      }
    }
  }

  //Increment the counter
  incomingUBX->counter++;

  if (overrun || (incomingUBX->counter == MAX_PAYLOAD_SIZE))
  {
    //Something has gone very wrong
    currentSentence = NONE; //Reset the sentence to being looking for a new start char
    if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
    {
      if (overrun)
        _debugSerial->println(F("processUBX: buffer overrun detected"));
      else
        _debugSerial->println(F("processUBX: counter hit MAX_PAYLOAD_SIZE"));
    }
  }
}

//This is the default or generic NMEA processor. We're only going to pipe the data to serial port so we can see it.
//User could overwrite this function to pipe characters to nmea.process(c) of tinyGPS or MicroNMEA
//Or user could pipe each character to a buffer, radio, etc.
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  //If user has assigned an output port then pipe the characters there
  if (_nmeaOutputPort != NULL)
    _nmeaOutputPort->write(incoming); //Echo this byte to the serial port
}

//We need to be able to identify an RTCM packet and then the length
//so that we know when the RTCM message is completely received and we then start
//listening for other sentences (like NMEA or UBX)
//RTCM packet structure is very odd. I never found RTCM STANDARD 10403.2 but
//http://d1.amobbs.com/bbs_upload782111/files_39/ourdev_635123CK0HJT.pdf is good
//https://dspace.cvut.cz/bitstream/handle/10467/65205/F3-BP-2016-Shkalikava-Anastasiya-Prenos%20polohove%20informace%20prostrednictvim%20datove%20site.pdf?sequence=-1
//Lead me to: https://forum.u-blox.com/index.php/4348/how-to-read-rtcm-messages-from-neo-m8p
//RTCM 3.2 bytes look like this:
//Byte 0: Always 0xD3
//Byte 1: 6-bits of zero
//Byte 2: 10-bits of length of this packet including the first two-ish header bytes, + 6.
//byte 3 + 4 bits: Msg type 12 bits
//Example: D3 00 7C 43 F0 ... / 0x7C = 124+6 = 130 bytes in this packet, 0x43F = Msg type 1087
void SFE_UBLOX_GPS::processRTCMframe(uint8_t incoming)
{
  if (rtcmFrameCounter == 1)
  {
    rtcmLen = (incoming & 0x03) << 8; //Get the last two bits of this byte. Bits 8&9 of 10-bit length
  }
  else if (rtcmFrameCounter == 2)
  {
    rtcmLen |= incoming; //Bits 0-7 of packet length
    rtcmLen += 6;        //There are 6 additional bytes of what we presume is header, msgType, CRC, and stuff
  }
  /*else if (rtcmFrameCounter == 3)
  {
    rtcmMsgType = incoming << 4; //Message Type, MS 4 bits
  }
  else if (rtcmFrameCounter == 4)
  {
    rtcmMsgType |= (incoming >> 4); //Message Type, bits 0-7
  }*/

  rtcmFrameCounter++;

  processRTCM(incoming); //Here is where we expose this byte to the user

  if (rtcmFrameCounter == rtcmLen)
  {
    //We're done!
    currentSentence = NONE; //Reset and start looking for next sentence type
  }
}


//This function is called for each byte of an RTCM frame
//Ths user can overwrite this function and process the RTCM frame as they please
//Bytes can be piped to Serial or other interface. The consumer could be a radio or the internet (Ntrip broadcaster)
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
  //Radio.sendReliable((String)incoming); //An example of passing this byte to a radio

  //_debugSerial->write(incoming); //An example of passing this byte out the serial port

  //Debug printing
  //  _debugSerial->print(F(" "));
  //  if(incoming < 0x10) _debugSerial->print(F("0"));
  //  if(incoming < 0x10) _debugSerial->print(F("0"));
  //  _debugSerial->print(incoming, HEX);
  //  if(rtcmFrameCounter % 16 == 0) _debugSerial->println();
}



//Processes NMEA and UBX binary sentences one byte at a time
//Take a given byte and file it into the proper array
void SFE_UBLOX_GPS::process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if ((currentSentence == NONE) || (currentSentence == NMEA))
  {
    if (incoming == 0xB5) //UBX binary frames start with 0xB5, aka μ
    {
      //This is the start of a binary sentence. Reset flags.
      //We still don't know the response class
      ubxFrameCounter = 0;
      currentSentence = UBX;
      //Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
      packetBuf.counter = 0;
      ignoreThisPayload = false; //We should not ignore this payload - yet
      //Store data in packetBuf until we know if we have a requested class and ID match
      activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
    }
    else if (incoming == '$')
    {
      currentSentence = NMEA;
    }
    else if (incoming == 0xD3) //RTCM frames start with 0xD3
    {
      rtcmFrameCounter = 0;
      currentSentence = RTCM;
    }
    else
    {
      //This character is unknown or we missed the previous start of a sentence
    }
  }

  //Depending on the sentence, pass the character to the individual processor
  if (currentSentence == UBX)
  {
    //Decide what type of response this is
    if ((ubxFrameCounter == 0) && (incoming != 0xB5))      //ISO 'μ'
      currentSentence = NONE;                              //Something went wrong. Reset.
    else if ((ubxFrameCounter == 1) && (incoming != 0x62)) //ASCII 'b'
      currentSentence = NONE;                              //Something went wrong. Reset.
    // Note to future self:
    // There may be some duplication / redundancy in the next few lines as processUBX will also
    // load information into packetBuf, but we'll do it here too for clarity
    else if (ubxFrameCounter == 2) //Class
    {
      // Record the class in packetBuf until we know what to do with it
      packetBuf.cls = incoming; // (Duplication)
      rollingChecksumA = 0;     //Reset our rolling checksums here (not when we receive the 0xB5)
      rollingChecksumB = 0;
      packetBuf.counter = 0;                                   //Reset the packetBuf.counter (again)
      packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
      packetBuf.startingSpot = incomingUBX->startingSpot;      //Copy the startingSpot
    }
    else if (ubxFrameCounter == 3) //ID
    {
      // Record the ID in packetBuf until we know what to do with it
      packetBuf.id = incoming; // (Duplication)
      //We can now identify the type of response
      //If the packet we are receiving is not an ACK then check for a class and ID match
      if (packetBuf.cls != UBX_CLASS_ACK)
      {
        //This is not an ACK so check for a class and ID match
        if ((packetBuf.cls == requestedClass) && (packetBuf.id == requestedID))
        {
          //This is not an ACK and we have a class and ID match
          //So start diverting data into incomingUBX (usually packetCfg)
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; //Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; //Copy over the .counter too
        }
        //This is not an ACK and we do not have a complete class and ID match
        //So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
        else if ((packetBuf.cls == requestedClass) &&
          (((packetBuf.id == UBX_NAV_PVT) && (requestedID == UBX_NAV_HPPOSLLH || requestedID == UBX_NAV_DOP)) ||
           ((packetBuf.id == UBX_NAV_HPPOSLLH) && (requestedID == UBX_NAV_PVT || requestedID == UBX_NAV_DOP)) ||
           ((packetBuf.id == UBX_NAV_DOP) && (requestedID == UBX_NAV_PVT || requestedID == UBX_NAV_HPPOSLLH))))
        {
          //This is not the message we were expecting but we start diverting data into incomingUBX (usually packetCfg) and process it anyway
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; //Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; //Copy over the .counter too
          if (_printDebug == true)
          {
            _debugSerial->print(F("process: auto NAV PVT/HPPOSLLH/DOP collision: Requested ID: 0x"));
            _debugSerial->print(requestedID, HEX);
            _debugSerial->print(F(" Message ID: 0x"));
            _debugSerial->println(packetBuf.id, HEX);
          }
        }
        else if ((packetBuf.cls == requestedClass) &&
          (((packetBuf.id == UBX_HNR_ATT) && (requestedID == UBX_HNR_INS || requestedID == UBX_HNR_PVT)) ||
           ((packetBuf.id == UBX_HNR_INS) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_PVT)) ||
           ((packetBuf.id == UBX_HNR_PVT) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_INS))))
        {
          //This is not the message we were expecting but we start diverting data into incomingUBX (usually packetCfg) and process it anyway
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; //Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; //Copy over the .counter too
          if (_printDebug == true)
          {
            _debugSerial->print(F("process: auto HNR ATT/INS/PVT collision: Requested ID: 0x"));
            _debugSerial->print(requestedID, HEX);
            _debugSerial->print(F(" Message ID: 0x"));
            _debugSerial->println(packetBuf.id, HEX);
          }
        }
        else
        {
          //This is not an ACK and we do not have a class and ID match
          //so we should keep diverting data into packetBuf and ignore the payload
          ignoreThisPayload = true;
        }
      }
      else
      {
        // This is an ACK so it is to early to do anything with it
        // We need to wait until we have received the length and data bytes
        // So we should keep diverting data into packetBuf
      }
    }
    else if (ubxFrameCounter == 4) //Length LSB
    {
      //We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len = incoming; // (Duplication)
    }
    else if (ubxFrameCounter == 5) //Length MSB
    {
      //We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len |= incoming << 8; // (Duplication)
    }
    else if (ubxFrameCounter == 6) //This should be the first byte of the payload unless .len is zero
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
        if (_printDebug == true)
        {
          _debugSerial->print(F("process: ZERO LENGTH packet received: Class: 0x"));
          _debugSerial->print(packetBuf.cls, HEX);
          _debugSerial->print(F(" ID: 0x"));
          _debugSerial->println(packetBuf.id, HEX);
        }
        //If length is zero (!) this will be the first byte of the checksum so record it
        packetBuf.checksumA = incoming;
      }
      else
      {
        //The length is not zero so record this byte in the payload
        packetBuf.payload[0] = incoming;
      }
    }
    else if (ubxFrameCounter == 7) //This should be the second byte of the payload unless .len is zero or one
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
        //If length is zero (!) this will be the second byte of the checksum so record it
        packetBuf.checksumB = incoming;
      }
      else if (packetBuf.len == 1) // Check if length is one
      {
        //The length is one so this is the first byte of the checksum
        packetBuf.checksumA = incoming;
      }
      else // Length is >= 2 so this must be a payload byte
      {
        packetBuf.payload[1] = incoming;
      }
      // Now that we have received two payload bytes, we can check for a matching ACK/NACK
      if ((activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
          && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
          && (packetBuf.payload[0] == requestedClass)        // and if the class matches
          && (packetBuf.payload[1] == requestedID))          // and if the ID matches
      {
        if (packetBuf.len == 2) // Check if .len is 2
        {
          // Then this is a matching ACK so copy it into packetAck
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETACK;
          packetAck.cls = packetBuf.cls;
          packetAck.id = packetBuf.id;
          packetAck.len = packetBuf.len;
          packetAck.counter = packetBuf.counter;
          packetAck.payload[0] = packetBuf.payload[0];
          packetAck.payload[1] = packetBuf.payload[1];
        }
        else // Length is not 2 (hopefully this is impossible!)
        {
          if (_printDebug == true)
          {
            _debugSerial->print(F("process: ACK received with .len != 2: Class: 0x"));
            _debugSerial->print(packetBuf.payload[0], HEX);
            _debugSerial->print(F(" ID: 0x"));
            _debugSerial->print(packetBuf.payload[1], HEX);
            _debugSerial->print(F(" len: "));
            _debugSerial->println(packetBuf.len);
          }
        }
      }
    }

    //Divert incoming into the correct buffer
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETACK)
      processUBX(incoming, &packetAck, requestedClass, requestedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
      processUBX(incoming, incomingUBX, requestedClass, requestedID);
    else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
      processUBX(incoming, &packetBuf, requestedClass, requestedID);

    //Finally, increment the frame counter
    ubxFrameCounter++;
  }
  else if (currentSentence == NMEA)
  {
    processNMEA(incoming); //Process each NMEA character
  }
  else if (currentSentence == RTCM)
  {
    processRTCMframe(incoming); //Deal with RTCM bytes
  }
}


//Checks Serial for data, passing any new bytes to process()
boolean SFE_UBLOX_GPS::checkUbloxSerial(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  while (_serialPort->available())
  {
    process(_serialPort->read(), incomingUBX, requestedClass, requestedID);
  }
  return (true);

} //end checkUbloxSerial()

//Called regularly to check for available bytes on the user' specified port
boolean SFE_UBLOX_GPS::checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (commType == COMM_TYPE_I2C)
    return (checkUbloxI2C(incomingUBX, requestedClass, requestedID));
  else if (commType == COMM_TYPE_SERIAL)
    return (checkUbloxSerial(incomingUBX, requestedClass, requestedID));
  return false;
}

//Get the latest Position/Velocity/Time solution and fill all global variables
boolean SFE_UBLOX_GPS::getPVT(uint16_t maxWait)
{
  if (autoPVT && autoPVTImplicitUpdate)
  {
    //The GPS is automatically reporting, we just check whether we got unread data
    if (_printDebug == true)
    {
      _debugSerial->println(F("getPVT: Autoreporting"));
    }
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_PVT);
    return moduleQueried.all;
  }
  else if (autoPVT && !autoPVTImplicitUpdate)
  {
    //Someone else has to call checkUblox for us...
    if (_printDebug == true)
    {
      _debugSerial->println(F("getPVT: Exit immediately"));
    }
    return (false);
  }
  else
  {
    if (_printDebug == true)
    {
      _debugSerial->println(F("getPVT: Polling"));
    }

    //The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_PVT;
    packetCfg.len = 0;
    //packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can record up to 20+MAX_PAYLOAD_SIZE = 84 bytes Note:now hard-coded in processUBX

    //The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_NAV))
    {
      if (_printDebug == true)
      {
        _debugSerial->println(F("getPVT: data was OVERWRITTEN by another NAV message (but that's OK)"));
      }
      return (true);
    }

    if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_HNR))
    {
      if (_printDebug == true)
      {
        _debugSerial->println(F("getPVT: data was OVERWRITTEN by a HNR message (and that's not OK)"));
      }
      return (false);
    }

    if (_printDebug == true)
    {
      _debugSerial->print(F("getPVT retVal: "));
      _debugSerial->println(statusString(retVal));
    }
    return (false);
  }
}
