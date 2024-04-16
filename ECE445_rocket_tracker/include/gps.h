/*
	This is a library written for the u-blox ZED-F9P and NEO-M8P-2
	SparkFun sells these at its website: www.sparkfun.com
	Do you like this library? Help support SparkFun. Buy a board!
	https://www.sparkfun.com/products/16481
	https://www.sparkfun.com/products/15136
	https://www.sparkfun.com/products/15005
	https://www.sparkfun.com/products/15733
	https://www.sparkfun.com/products/15193
	https://www.sparkfun.com/products/15210

	Written by Nathan Seidle @ SparkFun Electronics, September 6th, 2018

	This library handles configuring and handling the responses
	from a u-blox GPS module. Works with most modules from u-blox including
	the Zed-F9P, NEO-M8P-2, NEO-M9N, ZOE-M8Q, SAM-M8Q, and many others.

	https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

	Development environment specifics:
	Arduino IDE 1.8.5

	SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
	The MIT License (MIT)
	Copyright (c) 2016 SparkFun Electronics
	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
	associated documentation files (the "Software"), to deal in the Software without restriction,
	including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
	and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
	do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial
	portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
	NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
	IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/



#ifndef _gps_H
#define _gps_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

// const uint8_t VAL_LAYER_RAM = (1 << 0);
// const uint8_t VAL_LAYER_BBR = (1 << 1);
// const uint8_t VAL_LAYER_FLASH = (1 << 2);
// const uint8_t VAL_LAYER_ALL = VAL_LAYER_RAM | VAL_LAYER_BBR | VAL_LAYER_FLASH; //Not valid with getVal()


// #include "gps_config_keys.h"
// #include "u-blox_config_keys.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Define a digital pin to aid checksum failure capture and analysis
//Leave set to -1 if not needed
const int checksumFailurePin = -1;

// Global Status Returns
typedef enum
{
	SFE_UBLOX_STATUS_SUCCESS,
	SFE_UBLOX_STATUS_FAIL,
	SFE_UBLOX_STATUS_CRC_FAIL,
	SFE_UBLOX_STATUS_TIMEOUT,
	SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
	SFE_UBLOX_STATUS_OUT_OF_RANGE,
	SFE_UBLOX_STATUS_INVALID_ARG,
	SFE_UBLOX_STATUS_INVALID_OPERATION,
	SFE_UBLOX_STATUS_MEM_ERR,
	SFE_UBLOX_STATUS_HW_ERR,
	SFE_UBLOX_STATUS_DATA_SENT,		// This indicates that a 'set' was successful
	SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
	SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
	SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} sfe_ublox_status_e;

// ubxPacket validity
typedef enum
{
	SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
	SFE_UBLOX_PACKET_VALIDITY_VALID,
	SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
	SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} sfe_ublox_packet_validity_e;

// Identify which packet buffer is in use:
// packetCfg (or a custom packet), packetAck or packetBuf
typedef enum
{
	SFE_UBLOX_PACKET_PACKETCFG,
	SFE_UBLOX_PACKET_PACKETACK,
	SFE_UBLOX_PACKET_PACKETBUF
} sfe_ublox_packet_buffer_e;












// //Registers
const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;

// //The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
const uint8_t UBX_CLASS_NAV = 0x01;	 //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
// const uint8_t UBX_CLASS_RXM = 0x02;	 //Receiver Manager Messages: Satellite Status, RTC Status
// const uint8_t UBX_CLASS_INF = 0x04;	 //Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
const uint8_t UBX_CLASS_ACK = 0x05;	 //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
const uint8_t UBX_CLASS_CFG = 0x06;	 //Configuration Input Messages: Configure the receiver.
// const uint8_t UBX_CLASS_UPD = 0x09;	 //Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
// const uint8_t UBX_CLASS_MON = 0x0A;	 //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
// const uint8_t UBX_CLASS_AID = 0x0B;	 //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
// const uint8_t UBX_CLASS_TIM = 0x0D;	 //Timing Messages: Time Pulse Output, Time Mark Results
// const uint8_t UBX_CLASS_ESF = 0x10;	 //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
// const uint8_t UBX_CLASS_MGA = 0x13;	 //Multiple GNSS Assistance Messages: Assistance data for various GNSS
// const uint8_t UBX_CLASS_LOG = 0x21;	 //Logging Messages: Log creation, deletion, info and retrieval
// const uint8_t UBX_CLASS_SEC = 0x27;	 //Security Feature Messages
const uint8_t UBX_CLASS_HNR = 0x28;	 //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
// const uint8_t UBX_CLASS_NMEA = 0xF0; //NMEA Strings: standard NMEA strings

// //The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
// const uint8_t UBX_CFG_ANT = 0x13;		//Antenna Control Settings. Used to configure the antenna control settings
// const uint8_t UBX_CFG_BATCH = 0x93;		//Get/set data batching configuration.
const uint8_t UBX_CFG_CFG = 0x09;		//Clear, Save, and Load Configurations. Used to save current configuration
// const uint8_t UBX_CFG_DAT = 0x06;		//Set User-defined Datum or The currently defined Datum
// const uint8_t UBX_CFG_DGNSS = 0x70;		//DGNSS configuration
// const uint8_t UBX_CFG_ESFALG = 0x56;		//ESF alignment
// const uint8_t UBX_CFG_ESFA = 0x4C;		//ESF accelerometer
// const uint8_t UBX_CFG_ESFG = 0x4D;		//ESF gyro
// const uint8_t UBX_CFG_GEOFENCE = 0x69;	//Geofencing configuration. Used to configure a geofence
// const uint8_t UBX_CFG_GNSS = 0x3E;		//GNSS system configuration
// const uint8_t UBX_CFG_HNR = 0x5C;		//High Navigation Rate
// const uint8_t UBX_CFG_INF = 0x02;		//Depending on packet length, either: poll configuration for one protocol, or information message configuration
// const uint8_t UBX_CFG_ITFM = 0x39;		//Jamming/Interference Monitor configuration
// const uint8_t UBX_CFG_LOGFILTER = 0x47; //Data Logger Configuration
// const uint8_t UBX_CFG_MSG = 0x01;		//Poll a message configuration, or Set Message Rate(s), or Set Message Rate
// const uint8_t UBX_CFG_NAV5 = 0x24;		//Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
// const uint8_t UBX_CFG_NAVX5 = 0x23;		//Navigation Engine Expert Settings
// const uint8_t UBX_CFG_NMEA = 0x17;		//Extended NMEA protocol configuration V1
// const uint8_t UBX_CFG_ODO = 0x1E;		//Odometer, Low-speed COG Engine Settings
// const uint8_t UBX_CFG_PM2 = 0x3B;		//Extended power management configuration
// const uint8_t UBX_CFG_PMS = 0x86;		//Power mode setup
// const uint8_t UBX_CFG_PRT = 0x00;		//Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
// const uint8_t UBX_CFG_PWR = 0x57;		//Put receiver in a defined power state
const uint8_t UBX_CFG_RATE = 0x08;		//Navigation/Measurement Rate Settings. Used to set port baud rates.
// const uint8_t UBX_CFG_RINV = 0x34;		//Contents of Remote Inventory
// const uint8_t UBX_CFG_RST = 0x04;		//Reset Receiver / Clear Backup Data Structures. Used to reset device.
// const uint8_t UBX_CFG_RXM = 0x11;		//RXM configuration
// const uint8_t UBX_CFG_SBAS = 0x16;		//SBAS configuration
// const uint8_t UBX_CFG_TMODE3 = 0x71;	//Time Mode Settings 3. Used to enable Survey In Mode
// const uint8_t UBX_CFG_TP5 = 0x31;		//Time Pulse Parameters
// const uint8_t UBX_CFG_USB = 0x1B;		//USB Configuration
// const uint8_t UBX_CFG_VALDEL = 0x8C;	//Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
// const uint8_t UBX_CFG_VALGET = 0x8B;	//Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
// const uint8_t UBX_CFG_VALSET = 0x8A;	//Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

// //The following are used to enable NMEA messages. Descriptions come from the NMEA messages overview in the ZED-F9P Interface Description
// const uint8_t UBX_NMEA_MSB = 0xF0;	//All NMEA enable commands have 0xF0 as MSB
// const uint8_t UBX_NMEA_DTM = 0x0A;	//GxDTM (datum reference)
// const uint8_t UBX_NMEA_GAQ = 0x45;	//GxGAQ (poll a standard message (if the current talker ID is GA))
// const uint8_t UBX_NMEA_GBQ = 0x44;	//GxGBQ (poll a standard message (if the current Talker ID is GB))
// const uint8_t UBX_NMEA_GBS = 0x09;	//GxGBS (GNSS satellite fault detection)
// const uint8_t UBX_NMEA_GGA = 0x00;	//GxGGA (Global positioning system fix data)
// const uint8_t UBX_NMEA_GLL = 0x01;	//GxGLL (latitude and long, whith time of position fix and status)
// const uint8_t UBX_NMEA_GLQ = 0x43;	//GxGLQ (poll a standard message (if the current Talker ID is GL))
// const uint8_t UBX_NMEA_GNQ = 0x42;	//GxGNQ (poll a standard message (if the current Talker ID is GN))
// const uint8_t UBX_NMEA_GNS = 0x0D;	//GxGNS (GNSS fix data)
// const uint8_t UBX_NMEA_GPQ = 0x040; //GxGPQ (poll a standard message (if the current Talker ID is GP))
// const uint8_t UBX_NMEA_GRS = 0x06;	//GxGRS (GNSS range residuals)
// const uint8_t UBX_NMEA_GSA = 0x02;	//GxGSA (GNSS DOP and Active satellites)
// const uint8_t UBX_NMEA_GST = 0x07;	//GxGST (GNSS Pseudo Range Error Statistics)
// const uint8_t UBX_NMEA_GSV = 0x03;	//GxGSV (GNSS satellites in view)
// const uint8_t UBX_NMEA_RMC = 0x04;	//GxRMC (Recommended minimum data)
// const uint8_t UBX_NMEA_TXT = 0x41;	//GxTXT (text transmission)
// const uint8_t UBX_NMEA_VLW = 0x0F;	//GxVLW (dual ground/water distance)
// const uint8_t UBX_NMEA_VTG = 0x05;	//GxVTG (course over ground and Ground speed)
// const uint8_t UBX_NMEA_ZDA = 0x08;	//GxZDA (Time and Date)

// //The following are used to configure the NMEA protocol main talker ID and GSV talker ID
// const uint8_t UBX_NMEA_MAINTALKERID_NOTOVERRIDDEN = 0x00; //main talker ID is system dependent
// const uint8_t UBX_NMEA_MAINTALKERID_GP = 0x01;			  //main talker ID is GPS
// const uint8_t UBX_NMEA_MAINTALKERID_GL = 0x02;			  //main talker ID is GLONASS
// const uint8_t UBX_NMEA_MAINTALKERID_GN = 0x03;			  //main talker ID is combined receiver
// const uint8_t UBX_NMEA_MAINTALKERID_GA = 0x04;			  //main talker ID is Galileo
// const uint8_t UBX_NMEA_MAINTALKERID_GB = 0x05;			  //main talker ID is BeiDou
// const uint8_t UBX_NMEA_GSVTALKERID_GNSS = 0x00;			  //GNSS specific Talker ID (as defined by NMEA)
// const uint8_t UBX_NMEA_GSVTALKERID_MAIN = 0x01;			  //use the main Talker ID

// //The following are used to configure the HNR message rates
const uint8_t UBX_HNR_ATT = 0x01;			  //HNR Attitude
const uint8_t UBX_HNR_INS = 0x02;			  //HNR Vehicle Dynamics
const uint8_t UBX_HNR_PVT = 0x00;			  //HNR PVT

// //The following are used to configure INF UBX messages (information messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
// const uint8_t UBX_INF_CLASS = 0x04;	  //All INF messages have 0x04 as the class
// const uint8_t UBX_INF_DEBUG = 0x04;	  //ASCII output with debug contents
// const uint8_t UBX_INF_ERROR = 0x00;	  //ASCII output with error contents
// const uint8_t UBX_INF_NOTICE = 0x02;  //ASCII output with informational contents
// const uint8_t UBX_INF_TEST = 0x03;	  //ASCII output with test contents
// const uint8_t UBX_INF_WARNING = 0x01; //ASCII output with warning contents

// //The following are used to configure LOG UBX messages (loggings messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
// const uint8_t UBX_LOG_CREATE = 0x07;		   //Create Log File
// const uint8_t UBX_LOG_ERASE = 0x03;			   //Erase Logged Data
// const uint8_t UBX_LOG_FINDTIME = 0x0E;		   //Find index of a log entry based on a given time, or response to FINDTIME requested
// const uint8_t UBX_LOG_INFO = 0x08;			   //Poll for log information, or Log information
// const uint8_t UBX_LOG_RETRIEVEPOSEXTRA = 0x0F; //Odometer log entry
// const uint8_t UBX_LOG_RETRIEVEPOS = 0x0B;	   //Position fix log entry
// const uint8_t UBX_LOG_RETRIEVESTRING = 0x0D;   //Byte string log entry
// const uint8_t UBX_LOG_RETRIEVE = 0x09;		   //Request log data
// const uint8_t UBX_LOG_STRING = 0x04;		   //Store arbitrary string on on-board flash

// //The following are used to configure MGA UBX messages (Multiple GNSS Assistance Messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
// const uint8_t UBX_MGA_ACK_DATA0 = 0x60;		 //Multiple GNSS Acknowledge message
// const uint8_t UBX_MGA_BDS_EPH = 0x03;		 //BDS Ephemeris Assistance
// const uint8_t UBX_MGA_BDS_ALM = 0x03;		 //BDS Almanac Assistance
// const uint8_t UBX_MGA_BDS_HEALTH = 0x03;	 //BDS Health Assistance
// const uint8_t UBX_MGA_BDS_UTC = 0x03;		 //BDS UTC Assistance
// const uint8_t UBX_MGA_BDS_IONO = 0x03;		 //BDS Ionospheric Assistance
// const uint8_t UBX_MGA_DBD = 0x80;			 //Either: Poll the Navigation Database, or Navigation Database Dump Entry
// const uint8_t UBX_MGA_GAL_EPH = 0x02;		 //Galileo Ephemeris Assistance
// const uint8_t UBX_MGA_GAL_ALM = 0x02;		 //Galileo Almanac Assitance
// const uint8_t UBX_MGA_GAL_TIMOFFSET = 0x02;	 //Galileo GPS time offset assistance
// const uint8_t UBX_MGA_GAL_UTC = 0x02;		 //Galileo UTC Assistance
// const uint8_t UBX_MGA_GLO_EPH = 0x06;		 //GLONASS Ephemeris Assistance
// const uint8_t UBX_MGA_GLO_ALM = 0x06;		 //GLONASS Almanac Assistance
// const uint8_t UBX_MGA_GLO_TIMEOFFSET = 0x06; //GLONASS Auxiliary Time Offset Assistance
// const uint8_t UBX_MGA_GPS_EPH = 0x00;		 //GPS Ephemeris Assistance
// const uint8_t UBX_MGA_GPS_ALM = 0x00;		 //GPS Almanac Assistance
// const uint8_t UBX_MGA_GPS_HEALTH = 0x00;	 //GPS Health Assistance
// const uint8_t UBX_MGA_GPS_UTC = 0x00;		 //GPS UTC Assistance
// const uint8_t UBX_MGA_GPS_IONO = 0x00;		 //GPS Ionosphere Assistance
// const uint8_t UBX_MGA_INI_POS_XYZ = 0x40;	 //Initial Position Assistance
// const uint8_t UBX_MGA_INI_POS_LLH = 0x40;	 //Initial Position Assitance
// const uint8_t UBX_MGA_INI_TIME_UTC = 0x40;	 //Initial Time Assistance
// const uint8_t UBX_MGA_INI_TIME_GNSS = 0x40;	 //Initial Time Assistance
// const uint8_t UBX_MGA_INI_CLKD = 0x40;		 //Initial Clock Drift Assitance
// const uint8_t UBX_MGA_INI_FREQ = 0x40;		 //Initial Frequency Assistance
// const uint8_t UBX_MGA_INI_EOP = 0x40;		 //Earth Orientation Parameters Assistance
// const uint8_t UBX_MGA_QZSS_EPH = 0x05;		 //QZSS Ephemeris Assistance
// const uint8_t UBX_MGA_QZSS_ALM = 0x05;		 //QZSS Almanac Assistance
// const uint8_t UBX_MGA_QZAA_HEALTH = 0x05;	 //QZSS Health Assistance

// //The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
// const uint8_t UBX_MON_COMMS = 0x36; //Comm port information
// const uint8_t UBX_MON_GNSS = 0x28;	//Information message major GNSS selection
// const uint8_t UBX_MON_HW2 = 0x0B;	//Extended Hardware Status
// const uint8_t UBX_MON_HW3 = 0x37;	//HW I/O pin information
// const uint8_t UBX_MON_HW = 0x09;	//Hardware Status
// const uint8_t UBX_MON_IO = 0x02;	//I/O Subsystem Status
// const uint8_t UBX_MON_MSGPP = 0x06; //Message Parse and Process Status
// const uint8_t UBX_MON_PATCH = 0x27; //Output information about installed patches
// const uint8_t UBX_MON_RF = 0x38;	//RF information
// const uint8_t UBX_MON_RXBUF = 0x07; //Receiver Buffer Status
// const uint8_t UBX_MON_RXR = 0x21;	//Receiver Status Information
// const uint8_t UBX_MON_TXBUF = 0x08; //Transmitter Buffer Status. Used for query tx buffer size/state.
// const uint8_t UBX_MON_VER = 0x04;	//Receiver/Software Version. Used for obtaining Protocol Version.

// //The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
// const uint8_t UBX_NAV_ATT = 0x05;		//Vehicle "Attitude" Solution
// const uint8_t UBX_NAV_CLOCK = 0x22;		//Clock Solution
const uint8_t UBX_NAV_DOP = 0x04;		//Dilution of precision
// const uint8_t UBX_NAV_EOE = 0x61;		//End of Epoch
// const uint8_t UBX_NAV_GEOFENCE = 0x39;	//Geofencing status. Used to poll the geofence status
// const uint8_t UBX_NAV_HPPOSECEF = 0x13; //High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
const uint8_t UBX_NAV_HPPOSLLH = 0x14;	//High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
// const uint8_t UBX_NAV_ODO = 0x09;		//Odometer Solution
// const uint8_t UBX_NAV_ORB = 0x34;		//GNSS Orbit Database Info
// const uint8_t UBX_NAV_POSECEF = 0x01;	//Position Solution in ECEF
// const uint8_t UBX_NAV_POSLLH = 0x02;	//Geodetic Position Solution
const uint8_t UBX_NAV_PVT = 0x07;		//All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
// const uint8_t UBX_NAV_RELPOSNED = 0x3C; //Relative Positioning Information in NED frame
// const uint8_t UBX_NAV_RESETODO = 0x10;	//Reset odometer
// const uint8_t UBX_NAV_SAT = 0x35;		//Satellite Information
// const uint8_t UBX_NAV_SIG = 0x43;		//Signal Information
// const uint8_t UBX_NAV_STATUS = 0x03;	//Receiver Navigation Status
// const uint8_t UBX_NAV_SVIN = 0x3B;		//Survey-in data. Used for checking Survey In status
// const uint8_t UBX_NAV_TIMEBDS = 0x24;	//BDS Time Solution
// const uint8_t UBX_NAV_TIMEGAL = 0x25;	//Galileo Time Solution
// const uint8_t UBX_NAV_TIMEGLO = 0x23;	//GLO Time Solution
// const uint8_t UBX_NAV_TIMEGPS = 0x20;	//GPS Time Solution
// const uint8_t UBX_NAV_TIMELS = 0x26;	//Leap second event information
// const uint8_t UBX_NAV_TIMEUTC = 0x21;	//UTC Time Solution
// const uint8_t UBX_NAV_VELECEF = 0x11;	//Velocity Solution in ECEF
// const uint8_t UBX_NAV_VELNED = 0x12;	//Velocity Solution in NED

// //The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
// const uint8_t UBX_RXM_MEASX = 0x14; //Satellite Measurements for RRLP
// const uint8_t UBX_RXM_PMREQ = 0x41; //Requests a Power Management task (two differenent packet sizes)
// const uint8_t UBX_RXM_RAWX = 0x15;	//Multi-GNSS Raw Measurement Data
// const uint8_t UBX_RXM_RLM = 0x59;	//Galileo SAR Short-RLM report (two different packet sizes)
// const uint8_t UBX_RXM_RTCM = 0x32;	//RTCM input status
// const uint8_t UBX_RXM_SFRBX = 0x13; //Boradcast Navigation Data Subframe

// //The following are used to configure the SEC UBX messages (security feature messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
// const uint8_t UBX_SEC_UNIQID = 0x03; //Unique chip ID

// //The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
// const uint8_t UBX_TIM_TM2 = 0x03;  //Time mark data
// const uint8_t UBX_TIM_TP = 0x01;   //Time Pulse Timedata
// const uint8_t UBX_TIM_VRFY = 0x06; //Sourced Time Verification

// //The following are used to configure the UPD UBX messages (firmware update messages). Descriptions from UBX messages overview (ZED-F9P Interface Description Document page 36)
// const uint8_t UBX_UPD_SOS = 0x14; //Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup

// //The following are used to enable RTCM messages
// const uint8_t UBX_RTCM_MSB = 0xF5;	  //All RTCM enable commands have 0xF5 as MSB
// const uint8_t UBX_RTCM_1005 = 0x05;	  //Stationary RTK reference ARP
// const uint8_t UBX_RTCM_1074 = 0x4A;	  //GPS MSM4
// const uint8_t UBX_RTCM_1077 = 0x4D;	  //GPS MSM7
// const uint8_t UBX_RTCM_1084 = 0x54;	  //GLONASS MSM4
// const uint8_t UBX_RTCM_1087 = 0x57;	  //GLONASS MSM7
// const uint8_t UBX_RTCM_1094 = 0x5E;	  //Galileo MSM4
// const uint8_t UBX_RTCM_1097 = 0x61;	  //Galileo MSM7
// const uint8_t UBX_RTCM_1124 = 0x7C;	  //BeiDou MSM4
// const uint8_t UBX_RTCM_1127 = 0x7F;	  //BeiDou MSM7
// const uint8_t UBX_RTCM_1230 = 0xE6;	  //GLONASS code-phase biases, set to once every 10 seconds
// const uint8_t UBX_RTCM_4072_0 = 0xFE; //Reference station PVT (ublox proprietary RTCM message)
// const uint8_t UBX_RTCM_4072_1 = 0xFD; //Additional reference station information (ublox proprietary RTCM message)

const uint8_t UBX_ACK_NACK = 0x00;
const uint8_t UBX_ACK_ACK = 0x01;
// const uint8_t UBX_ACK_NONE = 0x02; //Not a real value

// // The following constants are used to get External Sensor Measurements and Status
// // Information.
// const uint8_t UBX_ESF_MEAS = 0x02;
// const uint8_t UBX_ESF_RAW = 0x03;
// const uint8_t UBX_ESF_STATUS = 0x10;
// const uint8_t UBX_ESF_INS = 0x15; //36 bytes

// const uint8_t SVIN_MODE_DISABLE = 0x00;
// const uint8_t SVIN_MODE_ENABLE = 0x01;

// //The following consts are used to configure the various ports and streams for those ports. See -CFG-PRT.
// const uint8_t COM_PORT_I2C = 0;
const uint8_t COM_PORT_UART1 = 1;
// const uint8_t COM_PORT_UART2 = 2;
// const uint8_t COM_PORT_USB = 3;
// const uint8_t COM_PORT_SPI = 4;

// const uint8_t COM_TYPE_UBX = (1 << 0);
// const uint8_t COM_TYPE_NMEA = (1 << 1);
// const uint8_t COM_TYPE_RTCM3 = (1 << 5);

// // Configuration Sub-Section mask definitions for saveConfigSelective (UBX-CFG-CFG)
// const uint32_t VAL_CFG_SUBSEC_IOPORT = 0x00000001;	 // ioPort - communications port settings (causes IO system reset!)
// const uint32_t VAL_CFG_SUBSEC_MSGCONF = 0x00000002;	 // msgConf - message configuration
// const uint32_t VAL_CFG_SUBSEC_INFMSG = 0x00000004;	 // infMsg - INF message configuration
// const uint32_t VAL_CFG_SUBSEC_NAVCONF = 0x00000008;	 // navConf - navigation configuration
// const uint32_t VAL_CFG_SUBSEC_RXMCONF = 0x00000010;	 // rxmConf - receiver manager configuration
// const uint32_t VAL_CFG_SUBSEC_SENCONF = 0x00000100;	 // senConf - sensor interface configuration (requires protocol 19+)
// const uint32_t VAL_CFG_SUBSEC_RINVCONF = 0x00000200; // rinvConf - remove inventory configuration
// const uint32_t VAL_CFG_SUBSEC_ANTCONF = 0x00000400;	 // antConf - antenna configuration
// const uint32_t VAL_CFG_SUBSEC_LOGCONF = 0x00000800;	 // logConf - logging configuration
// const uint32_t VAL_CFG_SUBSEC_FTSCONF = 0x00001000;	 // ftsConf - FTS configuration (FTS products only)

// // Bitfield wakeupSources for UBX_RXM_PMREQ
// const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX = 0x00000008;	// uartrx
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0 = 0x00000020; // extint0
// const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1 = 0x00000040; // extint1
// const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS = 0x00000080;	// spics


























enum dynModel // Possible values for the dynamic platform model, which provide more accuract position output for the situation. Description extracted from ZED-F9P Integration Manual
{
	DYN_MODEL_PORTABLE = 0, //Applications with low acceleration, e.g. portable devices. Suitable for most situations.
	// 1 is not defined
	DYN_MODEL_STATIONARY = 2, //Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed.
	DYN_MODEL_PEDESTRIAN,	  //Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed.
	DYN_MODEL_AUTOMOTIVE,	  //Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed
	DYN_MODEL_SEA,			  //Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
	DYN_MODEL_AIRBORNE1g,	  //Airborne <1g acceleration. Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported.
	DYN_MODEL_AIRBORNE2g,	  //Airborne <2g acceleration. Recommended for typical airborne environments. No 2D position fixes supported.
	DYN_MODEL_AIRBORNE4g,	  //Airborne <4g acceleration. Only recommended for extremely dynamic environments. No 2D position fixes supported.
	DYN_MODEL_WRIST,		  // Not supported in protocol versions less than 18. Only recommended for wrist worn applications. Receiver will filter out arm motion.
	DYN_MODEL_BIKE,			  // Supported in protocol versions 19.2
};

#ifndef MAX_PAYLOAD_SIZE

#define MAX_PAYLOAD_SIZE 256 //We need ~220 bytes for getProtocolVersion on most ublox modules
//#define MAX_PAYLOAD_SIZE 768 //Worst case: UBX_CFG_VALSET packet with 64 keyIDs each with 64 bit values

#endif

//-=-=-=-=- UBX binary specific variables
typedef struct
{
	uint8_t cls;
	uint8_t id;
	uint16_t len;		   //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter;	   //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload;
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
	sfe_ublox_packet_validity_e valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
	sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
} ubxPacket;

// Struct to hold the results returned by getGeofenceState (returned by UBX-NAV-GEOFENCE)
typedef struct
{
	uint8_t status;	   // Geofencing status: 0 - Geofencing not available or not reliable; 1 - Geofencing active
	uint8_t numFences; // Number of geofences
	uint8_t combState; // Combined (logical OR) state of all geofences: 0 - Unknown; 1 - Inside; 2 - Outside
	uint8_t states[4]; // Geofence states: 0 - Unknown; 1 - Inside; 2 - Outside
} geofenceState;

// Struct to hold the current geofence parameters
typedef struct
{
	uint8_t numFences; // Number of active geofences
	int32_t lats[4];   // Latitudes of geofences (in degrees * 10^-7)
	int32_t longs[4];  // Longitudes of geofences (in degrees * 10^-7)
	uint32_t rads[4];  // Radii of geofences (in m * 10^-2)
} geofenceParams;

class SFE_UBLOX_GPS
{
public:
	SFE_UBLOX_GPS(void);

// A default of 250ms for maxWait seems fine for I2C but is not enough for SerialUSB.
// If you know you are only going to be using I2C / Qwiic communication, you can
// safely reduce defaultMaxWait to 250.
#ifndef defaultMaxWait // Let's allow the user to define their own value if they want to
#define defaultMaxWait 1100
#endif

	//By default use the default I2C address, and use Wire port
	boolean begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = 0x42); //Returns true if module is detected
	//serialPort needs to be perviously initialized to correct baud rate
	boolean begin(Stream &serialPort); //Returns true if module is detected

	//Control the size of the internal I2C transaction amount
	void setI2CTransactionSize(uint8_t bufferSize);
	uint8_t getI2CTransactionSize(void);

	//Set the max number of bytes set in a given I2C transaction
	uint8_t i2cTransactionSize = 32; //Default to ATmega328 limit

	//Returns true if device answers on _gpsI2Caddress address or via Serial
	//maxWait is only used for Serial
	boolean isConnected(uint16_t maxWait = 1100);

	//Changed in V1.8.1: provides backward compatibility for the examples that call checkUblox directly
	//Will default to using packetCfg to look for explicit autoPVT packets so they get processed correctly by processUBX
	boolean checkUblox(uint8_t requestedClass = UBX_CLASS_NAV, uint8_t requestedID = UBX_NAV_PVT); //Checks module with user selected commType

	boolean checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);	   //Method for I2C polling of data, passing any new bytes to process()
	boolean checkUbloxSerial(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID); //Method for serial polling of data, passing any new bytes to process()

	void process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);	//Processes NMEA and UBX binary sentences one byte at a time
	void processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID); //Given a character, file it away into the uxb packet structure
	void processRTCMframe(uint8_t incoming);																//Monitor the incoming bytes for start and length bytes
	void processRTCM(uint8_t incoming) __attribute__((weak));												//Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.

	void processUBXpacket(ubxPacket *msg);				   //Once a packet has been received and validated, identify this packet's class/id and update internal flags
	void processNMEA(char incoming) __attribute__((weak)); //Given a NMEA character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries

	void calcChecksum(ubxPacket *msg);														   //Sets the checksumA and checksumB of a given messages
	sfe_ublox_status_e sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait = defaultMaxWait); //Given a packet and payload, send everything including CRC bytes, return true if we got a response
	sfe_ublox_status_e sendI2cCommand(ubxPacket *outgoingUBX, uint16_t maxWait = 250);
	void sendSerialCommand(ubxPacket *outgoingUBX);

	void printPacket(ubxPacket *packet); //Useful for debugging

	void factoryReset(); //Send factory reset sequence (i.e. load "default" configuration and perform hardReset)
	void hardReset();	 //Perform a reset leading to a cold start (zero info start-up)

	boolean setI2CAddress(uint8_t deviceAddress, uint16_t maxTime = 250);										 //Changes the I2C address of the u-blox module
	void setSerialRate(uint32_t baudrate, uint8_t uartPort = COM_PORT_UART1, uint16_t maxTime = defaultMaxWait); //Changes the serial baud rate of the u-blox module, uartPort should be COM_PORT_UART1/2
	void setNMEAOutputPort(Stream &nmeaOutputPort);																 //Sets the internal variable for the port to direct NMEA characters to

	boolean setNavigationFrequency(uint8_t navFreq, uint16_t maxWait = defaultMaxWait);	 //Set the number of nav solutions sent per second
	uint8_t getNavigationFrequency(uint16_t maxWait = defaultMaxWait);					 //Get the number of nav solutions sent per second currently being output by module
	boolean saveConfiguration(uint16_t maxWait = defaultMaxWait);						 //Save current configuration to flash and BBR (battery backed RAM)
	boolean factoryDefault(uint16_t maxWait = defaultMaxWait);							 //Reset module to factory defaults
	boolean saveConfigSelective(uint32_t configMask, uint16_t maxWait = defaultMaxWait); //Save the selected configuration sub-sections to flash and BBR (battery backed RAM)

	sfe_ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = defaultMaxWait);	 //Poll the module until a config packet and an ACK is received
	sfe_ublox_status_e waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = defaultMaxWait); //Poll the module until a config packet is received

// getPVT will only return data once in each navigation cycle. By default, that is once per second.
// Therefore we should set getPVTmaxWait to slightly longer than that.
// If you change the navigation frequency to (e.g.) 4Hz using setNavigationFrequency(4)
// then you should use a shorter maxWait for getPVT. 300msec would be about right: getPVT(300)
// The same is true for getHPPOSLLH.
#define getPVTmaxWait 1100		// Default maxWait for getPVT and all functions which call it
#define getHPPOSLLHmaxWait 1100 // Default maxWait for getHPPOSLLH and all functions which call it
#define getDOPmaxWait 1100 // Default maxWait for getDOP and all functions which all it

	boolean assumeAutoPVT(boolean enabled, boolean implicitUpdate = true);							//In case no config access to the GPS is possible and PVT is send cyclically already
	boolean setAutoPVT(boolean enabled, uint16_t maxWait = defaultMaxWait);							//Enable/disable automatic PVT reports at the navigation frequency
	boolean setAutoPVT(boolean enabled, boolean implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic PVT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	boolean getPVT(uint16_t maxWait = getPVTmaxWait);												//Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new PVT is available.
	boolean assumeAutoHPPOSLLH(boolean enabled, boolean implicitUpdate = true);							//In case no config access to the GPS is possible and HPPOSLLH is send cyclically already
	boolean setAutoHPPOSLLH(boolean enabled, uint16_t maxWait = defaultMaxWait);							//Enable/disable automatic HPPOSLLH reports at the navigation frequency
	boolean setAutoHPPOSLLH(boolean enabled, boolean implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HPPOSLLH reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	boolean getHPPOSLLH(uint16_t maxWait = getHPPOSLLHmaxWait);										//Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new HPPOSLLH is available.
  boolean assumeAutoDOP(boolean enabled, boolean implicitUpdate = true);              //In case no config access to the GPS is possible and DOP is send cyclically already
  boolean setAutoDOP(boolean enabled, uint16_t maxWait = defaultMaxWait);              //Enable/disable automatic DOP reports at the navigation frequency
  boolean setAutoDOP(boolean enabled, boolean implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic DOP reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  boolean getDOP(uint16_t maxWait = getDOPmaxWait);                   //Query module for latest dilution of precision values and load global vars:. If autoDOP is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new DOP is available.
	void flushPVT();																				//Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure
	void flushHPPOSLLH();																				//Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure
  void flushDOP();                                       //Mark all the DOP data as read/stale. This is handy to get data alignment after CRC failure











// 	bool getGnssFixOk(uint16_t maxWait = getPVTmaxWait);          //Get whether we have a valid fix (i.e within DOP & accuracy masks)
// 	bool getDiffSoln(uint16_t maxWait = getPVTmaxWait);           //Get whether differential corrections were applied
// 	bool getHeadVehValid(uint16_t maxWait = getPVTmaxWait);
	int32_t getLatitude(uint16_t maxWait = getPVTmaxWait);			  //Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
	int32_t getLongitude(uint16_t maxWait = getPVTmaxWait);			  //Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
// 	int32_t getAltitude(uint16_t maxWait = getPVTmaxWait);			  //Returns the current altitude in mm above ellipsoid
// 	int32_t getAltitudeMSL(uint16_t maxWait = getPVTmaxWait);		  //Returns the current altitude in mm above mean sea level
// 	int32_t getHorizontalAccEst(uint16_t maxWait = getPVTmaxWait);
// 	int32_t getVerticalAccEst(uint16_t maxWait = getPVTmaxWait);
// 	int32_t getNedNorthVel(uint16_t maxWait = getPVTmaxWait);
// 	int32_t getNedEastVel(uint16_t maxWait = getPVTmaxWait);
// 	int32_t getNedDownVel(uint16_t maxWait = getPVTmaxWait);
// 	uint8_t getSIV(uint16_t maxWait = getPVTmaxWait);				  //Returns number of sats used in fix
// 	uint8_t getFixType(uint16_t maxWait = getPVTmaxWait);			  //Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning
// 	uint8_t getCarrierSolutionType(uint16_t maxWait = getPVTmaxWait); //Returns RTK solution: 0=no, 1=float solution, 2=fixed solution
// 	int32_t getGroundSpeed(uint16_t maxWait = getPVTmaxWait);		  //Returns speed in mm/s
	// int32_t getHeading(uint16_t maxWait = getPVTmaxWait);			  //Returns heading in degrees * 10^-5
// 	uint16_t getPDOP(uint16_t maxWait = getPVTmaxWait);				  //Returns positional dillution of precision * 10^-2 (dimensionless)
// 	uint16_t getYear(uint16_t maxWait = getPVTmaxWait);
// 	uint8_t getMonth(uint16_t maxWait = getPVTmaxWait);
// 	uint8_t getDay(uint16_t maxWait = getPVTmaxWait);
// 	uint8_t getHour(uint16_t maxWait = getPVTmaxWait);
// 	uint8_t getMinute(uint16_t maxWait = getPVTmaxWait);
// 	uint8_t getSecond(uint16_t maxWait = getPVTmaxWait);
// 	uint16_t getMillisecond(uint16_t maxWait = getPVTmaxWait);
// 	int32_t getNanosecond(uint16_t maxWait = getPVTmaxWait);
// 	uint32_t getTimeOfWeek(uint16_t maxWait = getPVTmaxWait);
// 	bool getDateValid(uint16_t maxWait = getPVTmaxWait);
// 	bool getTimeValid(uint16_t maxWait = getPVTmaxWait);
// 	uint32_t getSpeedAccEst(uint16_t maxWait = getPVTmaxWait);
// 	uint32_t getHeadingAccEst(uint16_t maxWait = getPVTmaxWait);
// 	bool getInvalidLlh(uint16_t maxWait = getPVTmaxWait);
// 	int32_t getHeadVeh(uint16_t maxWait = getPVTmaxWait);
// 	int16_t getMagDec(uint16_t maxWait = getPVTmaxWait);
// 	uint16_t getMagAcc(uint16_t maxWait = getPVTmaxWait);

// 	int32_t getHighResLatitude(uint16_t maxWait = getHPPOSLLHmaxWait);
// 	int8_t getHighResLatitudeHp(uint16_t maxWait = getHPPOSLLHmaxWait);
// 	int32_t getHighResLongitude(uint16_t maxWait = getHPPOSLLHmaxWait);
// 	int8_t getHighResLongitudeHp(uint16_t maxWait = getHPPOSLLHmaxWait);
// 	int32_t getElipsoid(uint16_t maxWait = getHPPOSLLHmaxWait);
// 	int8_t getElipsoidHp(uint16_t maxWait = getHPPOSLLHmaxWait);
// 	int32_t getMeanSeaLevel(uint16_t maxWait = getHPPOSLLHmaxWait);
// 	int8_t getMeanSeaLevelHp(uint16_t maxWait = getHPPOSLLHmaxWait);
// 	int32_t getGeoidSeparation(uint16_t maxWait = getHPPOSLLHmaxWait);
	// uint32_t getHorizontalAccuracy(uint16_t maxWait = getHPPOSLLHmaxWait);
	// uint32_t getVerticalAccuracy(uint16_t maxWait = getHPPOSLLHmaxWait);

//   uint16_t getGeometricDOP(uint16_t maxWait = getDOPmaxWait);
//   uint16_t getPositionDOP(uint16_t maxWait = getDOPmaxWait);
//   uint16_t getTimeDOP(uint16_t maxWait = getDOPmaxWait);
//   uint16_t getVerticalDOP(uint16_t maxWait = getDOPmaxWait);
//   uint16_t getHorizontalDOP(uint16_t maxWait = getDOPmaxWait);
//   uint16_t getNorthingDOP(uint16_t maxWait = getDOPmaxWait);
//   uint16_t getEastingDOP(uint16_t maxWait = getDOPmaxWait);

// 	//Port configurations
// 	boolean setPortOutput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = defaultMaxWait); //Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
// 	boolean setPortInput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = defaultMaxWait);  //Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof
// 	boolean getPortSettings(uint8_t portID, uint16_t maxWait = defaultMaxWait);					   //Returns the current protocol bits in the UBX-CFG-PRT command for a given port

// 	boolean setI2COutput(uint8_t comSettings, uint16_t maxWait = 250);				//Configure I2C port to output UBX, NMEA, RTCM3 or a combination thereof
// 	boolean setUART1Output(uint8_t comSettings, uint16_t maxWait = defaultMaxWait); //Configure UART1 port to output UBX, NMEA, RTCM3 or a combination thereof
// 	boolean setUART2Output(uint8_t comSettings, uint16_t maxWait = defaultMaxWait); //Configure UART2 port to output UBX, NMEA, RTCM3 or a combination thereof
// 	boolean setUSBOutput(uint8_t comSettings, uint16_t maxWait = 250);				//Configure USB port to output UBX, NMEA, RTCM3 or a combination thereof
// 	boolean setSPIOutput(uint8_t comSettings, uint16_t maxWait = 250);				//Configure SPI port to output UBX, NMEA, RTCM3 or a combination thereof

// 	//Functions to turn on/off message types for a given port ID (see COM_PORT_I2C, etc above)
// 	boolean configureMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate, uint16_t maxWait = defaultMaxWait);
// 	boolean enableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate = 1, uint16_t maxWait = defaultMaxWait);
// 	boolean disableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint16_t maxWait = defaultMaxWait);
// 	boolean enableNMEAMessage(uint8_t msgID, uint8_t portID, uint8_t sendRate = 1, uint16_t maxWait = defaultMaxWait);
// 	boolean disableNMEAMessage(uint8_t msgID, uint8_t portID, uint16_t maxWait = defaultMaxWait);
// 	boolean enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t sendRate, uint16_t maxWait = defaultMaxWait); //Given a message number turns on a message ID for output over given PortID
// 	boolean disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait = defaultMaxWait);				   //Turn off given RTCM message from a given port

// 	//General configuration (used only on protocol v27 and higher - ie, ZED-F9P)
// 	//It is probably safe to assume that users of the ZED-F9P will be using I2C / Qwiic.
// 	//If they are using Serial then the higher baud rate will also help. So let's leave maxWait set to 250ms.
// 	uint32_t createKey(uint16_t group, uint16_t id, uint8_t size); //Form 32-bit key from group/id/size

// 	sfe_ublox_status_e getVal(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);					 //Load payload with response
// 	uint8_t getVal8(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);								 //Returns the value at a given key location
// 	uint16_t getVal16(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);							 //Returns the value at a given key location
// 	uint32_t getVal32(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);							 //Returns the value at a given key location
// 	uint8_t getVal8(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);	 //Returns the value at a given group/id/size location
// 	uint16_t getVal16(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250); //Returns the value at a given group/id/size location
// 	uint32_t getVal32(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250); //Returns the value at a given group/id/size location
// 	uint8_t setVal(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = 250);				 //Sets the 16-bit value at a given group/id/size location
// 	uint8_t setVal8(uint32_t keyID, uint8_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = 250);				 //Sets the 8-bit value at a given group/id/size location
// 	uint8_t setVal16(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = 250);			 //Sets the 16-bit value at a given group/id/size location
// 	uint8_t setVal32(uint32_t keyID, uint32_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = 250);			 //Sets the 32-bit value at a given group/id/size location
// 	uint8_t newCfgValset8(uint32_t keyID, uint8_t value, uint8_t layer = VAL_LAYER_ALL);								 //Define a new UBX-CFG-VALSET with the given KeyID and 8-bit value
// 	uint8_t newCfgValset16(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_ALL);								 //Define a new UBX-CFG-VALSET with the given KeyID and 16-bit value
// 	uint8_t newCfgValset32(uint32_t keyID, uint32_t value, uint8_t layer = VAL_LAYER_ALL);								 //Define a new UBX-CFG-VALSET with the given KeyID and 32-bit value
// 	uint8_t addCfgValset8(uint32_t keyID, uint8_t value);																 //Add a new KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket
// 	uint8_t addCfgValset16(uint32_t keyID, uint16_t value);																 //Add a new KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket
// 	uint8_t addCfgValset32(uint32_t keyID, uint32_t value);																 //Add a new KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket
// 	uint8_t sendCfgValset8(uint32_t keyID, uint8_t value, uint16_t maxWait = 250);										 //Add the final KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
// 	uint8_t sendCfgValset16(uint32_t keyID, uint16_t value, uint16_t maxWait = 250);									 //Add the final KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
// 	uint8_t sendCfgValset32(uint32_t keyID, uint32_t value, uint16_t maxWait = 250);									 //Add the final KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket and send it









	//Functions used for RTK and base station setup
	//It is probably safe to assume that users of the RTK will be using I2C / Qwiic. So let's leave maxWait set to 250ms.
	// boolean getSurveyMode(uint16_t maxWait = 250);																   //Get the current TimeMode3 settings
	// boolean setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250); //Control survey in mode
	// boolean enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250);			   //Begin Survey-In for NEO-M8P
	// boolean disableSurveyMode(uint16_t maxWait = 250);															   //Stop Survey-In mode

	// boolean getSurveyStatus(uint16_t maxWait); //Reads survey in status and sets the global variables

	// uint32_t getPositionAccuracy(uint16_t maxWait = 1100); //Returns the 3D accuracy of the current high-precision fix, in mm. Supported on NEO-M8P, ZED-F9P,

	// uint8_t getProtocolVersionHigh(uint16_t maxWait = 500); //Returns the PROTVER XX.00 from UBX-MON-VER register
	// uint8_t getProtocolVersionLow(uint16_t maxWait = 500);	//Returns the PROTVER 00.XX from UBX-MON-VER register
	// boolean getProtocolVersion(uint16_t maxWait = 500);		//Queries module, loads low/high bytes

	// boolean getRELPOSNED(uint16_t maxWait = 1100); //Get Relative Positioning Information of the NED frame

	// Enable debug messages using the chosen Serial port (Stream)
	// Boards like the RedBoard Turbo use SerialUSB (not Serial).
	// But other boards like the SAMD51 Thing Plus use Serial (not SerialUSB).
	// These lines let the code compile cleanly on as many SAMD boards as possible.
	// #if defined(ARDUINO_ARCH_SAMD)	// Is this a SAMD board?
	// #if defined(USB_VID)						// Is the USB Vendor ID defined?
	// #if (USB_VID == 0x1B4F)					// Is this a SparkFun board?
	// #if !defined(ARDUINO_SAMD51_THING_PLUS) & !defined(ARDUINO_SAMD51_MICROMOD) // If it is not a SAMD51 Thing Plus or SAMD51 MicroMod
	// void enableDebugging(Stream &debugPort = SerialUSB, boolean printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	// #else
	// void enableDebugging(Stream &debugPort = Serial, boolean printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	// #endif
	// #else
	// void enableDebugging(Stream &debugPort = Serial, boolean printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	// #endif
	// #else
	// void enableDebugging(Stream &debugPort = Serial, boolean printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	// #endif
	// #else
	// void enableDebugging(Stream &debugPort = Serial, boolean printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	// #endif










	// void disableDebugging(void);														 //Turn off debug statements
	// void debugPrint(char *message);														 //Safely print debug statements
	// void debugPrintln(char *message);													 //Safely print debug statements
	const char *statusString(sfe_ublox_status_e stat);									 //Pretty print the return value

	//Support for geofences
	boolean addGeofence(int32_t latitude, int32_t longitude, uint32_t radius, byte confidence = 0, byte pinPolarity = 0, byte pin = 0, uint16_t maxWait = 1100); // Add a new geofence
	boolean clearGeofences(uint16_t maxWait = 1100);																											 //Clears all geofences
	boolean getGeofenceState(geofenceState &currentGeofenceState, uint16_t maxWait = 1100);																		 //Returns the combined geofence state
	boolean clearAntPIO(uint16_t maxWait = 1100);																												 //Clears the antenna control pin settings to release the PIOs
	geofenceParams currentGeofenceParams;																														 // Global to store the geofence parameters

	// boolean powerSaveMode(bool power_save = true, uint16_t maxWait = 1100);
	// uint8_t getPowerSaveMode(uint16_t maxWait = 1100); // Returns 255 if the sendCommand fails
	// boolean powerOff(uint32_t durationInMs, uint16_t maxWait = 1100);
	// boolean powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources = VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0, boolean forceWhileUsb = true, uint16_t maxWait = 1100);

	//Change the dynamic platform model using UBX-CFG-NAV5
	boolean setDynamicModel(dynModel newDynamicModel = DYN_MODEL_PORTABLE, uint16_t maxWait = 1100);
	uint8_t getDynamicModel(uint16_t maxWait = 1100); // Get the dynamic model - returns 255 if the sendCommand fails

	boolean getEsfInfo(uint16_t maxWait = 1100);
	boolean getEsfIns(uint16_t maxWait = 1100);
	boolean getEsfDataInfo(uint16_t maxWait = 1100);
	boolean getEsfRawDataInfo(uint16_t maxWait = 1100);
	sfe_ublox_status_e getSensState(uint8_t sensor, uint16_t maxWait = 1100);
	boolean getVehAtt(uint16_t maxWait = 1100);

	// Given coordinates, put receiver into static position. Set latlong to true to pass in lat/long values instead of ecef.
	// For ECEF the units are: cm, 0.1mm, cm, 0.1mm, cm, 0.1mm
	// For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
	bool setStaticPosition(int32_t ecefXOrLat, int8_t ecefXOrLatHP, int32_t ecefYOrLon, int8_t ecefYOrLonHP, int32_t ecefZOrAlt, int8_t ecefZOrAltHP, bool latLong = false, uint16_t maxWait = 250);
	bool setStaticPosition(int32_t ecefXOrLat, int32_t ecefYOrLon, int32_t ecefZOrAlt, bool latLong = false, uint16_t maxWait = 250);

	// Push (e.g.) RTCM data directly to the module
	// Warning: this function does not check that the data is valid. It is the user's responsibility to ensure the data is valid before pushing.
	boolean pushRawData(uint8_t *dataBytes, size_t numDataBytes);

	//Survey-in specific controls
	struct svinStructure
	{
		boolean active;
		boolean valid;
		uint16_t observationTime;
		float meanAccuracy;
	} svin;

	//Relative Positioning Info in NED frame specific controls
	// struct frelPosInfoStructure
	// {
	// 	uint16_t refStationID;

	// 	float relPosN;
	// 	float relPosE;
	// 	float relPosD;

	// 	long relPosLength;
	// 	long relPosHeading;

	// 	int8_t relPosHPN;
	// 	int8_t relPosHPE;
	// 	int8_t relPosHPD;
	// 	int8_t relPosHPLength;

	// 	float accN;
	// 	float accE;
	// 	float accD;

	// 	bool gnssFixOk;
	// 	bool diffSoln;
	// 	bool relPosValid;
	// 	uint8_t carrSoln;
	// 	bool isMoving;
	// 	bool refPosMiss;
	// 	bool refObsMiss;
	// } relPosInfo;

	//The major datums we want to globally store
	// uint16_t gpsYear;
	// uint8_t gpsMonth;
	// uint8_t gpsDay;
	// uint8_t gpsHour;
	// uint8_t gpsMinute;
	// uint8_t gpsSecond;
	// uint16_t gpsMillisecond;
	// int32_t gpsNanosecond;
	// bool gpsDateValid;
	// bool gpsTimeValid;

	// bool gnssFixOk;      //valid fix (i.e within DOP & accuracy masks)
	// bool diffSoln;       //Differential corrections were applied
	// bool headVehValid;
	int32_t latitude;		 //Degrees * 10^-7 (more accurate than floats)
	int32_t longitude;		 //Degrees * 10^-7 (more accurate than floats)
	// int32_t altitude;		 //Number of mm above ellipsoid
	// int32_t altitudeMSL;	 //Number of mm above Mean Sea Level
	// uint32_t horizontalAccEst;
	// uint32_t verticalAccEst;
	// int32_t nedNorthVel;
	// int32_t nedEastVel;
	// int32_t nedDownVel;
	// uint8_t SIV;			 //Number of satellites used in position solution
	// uint8_t fixType;		 //Tells us when we have a solution aka lock
	// uint8_t carrierSolution; //Tells us when we have an RTK float/fixed solution
	// int32_t groundSpeed;	 //mm/s
	// int32_t headingOfMotion; //degrees * 10^-5
	// uint32_t speedAccEst;
	// uint32_t headingAccEst;
	// uint16_t pDOP;			 //Positional dilution of precision * 10^-2 (dimensionless)
	// bool invalidLlh;
	// int32_t headVeh;
	// int16_t magDec;
	// uint16_t magAcc;
	// uint8_t versionLow;		 //Loaded from getProtocolVersion().
	// uint8_t versionHigh;

	// uint32_t timeOfWeek;		 // ms
	int32_t highResLatitude;	 // Degrees * 10^-7
	int32_t highResLongitude;	 // Degrees * 10^-7
	// int32_t elipsoid;			 // Height above ellipsoid in mm (Typo! Should be eLLipsoid! **Uncorrected for backward-compatibility.**)
	// int32_t meanSeaLevel;		 // Height above mean sea level in mm
	// int32_t geoidSeparation;	 // This seems to only be provided in NMEA GGA and GNS messages
	// uint32_t horizontalAccuracy; // mm * 10^-1 (i.e. 0.1mm)
	// uint32_t verticalAccuracy;	 // mm * 10^-1 (i.e. 0.1mm)
	// int8_t elipsoidHp;			 // High precision component of the height above ellipsoid in mm * 10^-1 (Deliberate typo! Should be eLLipsoidHp!)
	// int8_t meanSeaLevelHp;		 // High precision component of Height above mean sea level in mm * 10^-1
	int8_t highResLatitudeHp;	 // High precision component of latitude: Degrees * 10^-9
	int8_t highResLongitudeHp;	 // High precision component of longitude: Degrees * 10^-9

	uint16_t rtcmFrameCounter = 0; //Tracks the type of incoming byte inside RTCM frame

	// uint16_t geometricDOP; // Geometric dilution of precision * 10^-2
	// uint16_t positionDOP; // Posoition dilution of precision * 10^-2
	// uint16_t timeDOP; // Time dilution of precision * 10^-2
	// uint16_t verticalDOP; // Vertical dilution of precision * 10^-2
	// uint16_t horizontalDOP; // Horizontal dilution of precision * 10^-2
	// uint16_t northingDOP; // Northing dilution of precision * 10^-2
	// uint16_t eastingDOP; // Easting dilution of precision * 10^-2

// #define DEF_NUM_SENS 7
// 	struct deadReckData
// 	{
// 		uint8_t version;
// 		uint8_t fusionMode;

// 		uint8_t xAngRateVald;
// 		uint8_t yAngRateVald;
// 		uint8_t zAngRateVald;
// 		uint8_t xAccelVald;
// 		uint8_t yAccelVald;
// 		uint8_t zAccelVald;

// 		int32_t xAngRate;
// 		int32_t yAngRate;
// 		int32_t zAngRate;

// 		int32_t xAccel;
// 		int32_t yAccel;
// 		int32_t zAccel;

// 		// The array size is based on testing directly on M8U and F9R
// 		uint32_t rawData;
// 		uint32_t rawDataType;
// 		uint32_t rawTStamp;

// 		uint32_t data[DEF_NUM_SENS];
// 		uint32_t dataType[DEF_NUM_SENS];
// 		uint32_t dataTStamp[DEF_NUM_SENS];
// 	} imuMeas;

	// struct indivImuData
	// {

	// 	uint8_t numSens;

	// 	uint8_t senType;
	// 	boolean isUsed;
	// 	boolean isReady;
	// 	uint8_t calibStatus;
	// 	uint8_t timeStatus;

	// 	uint8_t freq; // Hz

	// 	boolean badMeas;
	// 	boolean badTag;
	// 	boolean missMeas;
	// 	boolean noisyMeas;
	// } ubloxSen;

	// struct vehicleAttitude
	// {
	// 	// All values in degrees
	// 	int32_t roll;
	// 	int32_t pitch;
	// 	int32_t heading;
	// 	uint32_t accRoll;
	// 	uint32_t accPitch;
	// 	uint32_t accHeading;
	// } vehAtt;

	//HNR-specific structs
	// struct hnrAttitudeSolution
	// {
	// 	uint32_t iTOW;
	// 	int32_t roll; // Degrees * 1e-5
	// 	int32_t pitch; // Degrees * 1e-5
	// 	int32_t heading; // Degrees * 1e-5
	// 	uint32_t accRoll; // Degrees * 1e-5
	// 	uint32_t accPitch; // Degrees * 1e-5
	// 	uint32_t accHeading; // Degrees * 1e-5
	// } hnrAtt;









	// struct hnrVehicleDynamics
	// {
	// 	boolean xAngRateValid;
	// 	boolean yAngRateValid;
	// 	boolean zAngRateValid;
	// 	boolean xAccelValid;
	// 	boolean yAccelValid;
	// 	boolean zAccelValid;
	// 	uint32_t iTOW;
	// 	int32_t xAngRate; // Degrees/s * 1e-3
	// 	int32_t yAngRate; // Degrees/s * 1e-3
	// 	int32_t zAngRate; // Degrees/s * 1e-3
	// 	int32_t xAccel; // m/s^2 * 1e-2
	// 	int32_t yAccel; // m/s^2 * 1e-2
	// 	int32_t zAccel; // m/s^2 * 1e-2
	// } hnrVehDyn;

	struct hnrPosVelTime
	{
		// uint32_t iTOW;
		// uint16_t year;
		// uint8_t month;
		// uint8_t day;
		// uint8_t hour;
		// uint8_t min;
		// uint8_t sec;
		// boolean validDate;
		// boolean validTime;
		// boolean fullyResolved;
		// int32_t nano;
		uint8_t gpsFix;
		// boolean gpsFixOK;
		// boolean diffSoln;
		// boolean WKNSET;
		// boolean TOWSET;
		// boolean headVehValid;
		int32_t lon; // Degrees * 1e-7
		int32_t lat; // Degrees * 1e-7
		int32_t height; // mm above ellipsoid
		// int32_t hMSL; // mm above MSL
		// int32_t gSpeed; // mm/s 2D
		// int32_t speed; // mm/s 3D
		// int32_t headMot; // Degrees * 1e-5
		// int32_t headVeh; // Degrees * 1e-5
		// uint32_t hAcc; // mm
		// uint32_t vAcc; // mm
		// uint32_t sAcc; // mm
		// uint32_t headAcc; // Degrees * 1e-5
	} hnrPVT;

	//HNR functions
// 	boolean setHNRNavigationRate(uint8_t rate, uint16_t maxWait = 1100); // Returns true if the setHNRNavigationRate is successful
// 	uint8_t getHNRNavigationRate(uint16_t maxWait = 1100); // Returns 0 if the getHNRNavigationRate fails
// 	boolean assumeAutoHNRAtt(boolean enabled, boolean implicitUpdate = true);              //In case no config access to the GPS is possible and HNR Attitude is send cyclically already
//   boolean setAutoHNRAtt(boolean enabled, uint16_t maxWait = defaultMaxWait);              //Enable/disable automatic HNR Attitude reports at the HNR rate
//   boolean setAutoHNRAtt(boolean enabled, boolean implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HNR Attitude reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
// 	boolean getHNRAtt(uint16_t maxWait = 1100); // Returns true if the get HNR attitude is successful. Data is returned in hnrAtt
// 	boolean assumeAutoHNRDyn(boolean enabled, boolean implicitUpdate = true);              //In case no config access to the GPS is possible and HNR dynamics is send cyclically already
//   boolean setAutoHNRDyn(boolean enabled, uint16_t maxWait = defaultMaxWait);              //Enable/disable automatic HNR dynamics reports at the HNR rate
//   boolean setAutoHNRDyn(boolean enabled, boolean implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HNR dynamics reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
// 	boolean getHNRDyn(uint16_t maxWait = 1100); // Returns true if the get HNR dynamics is successful. Data is returned in hnrVehDyn
// 	boolean assumeAutoHNRPVT(boolean enabled, boolean implicitUpdate = true);              //In case no config access to the GPS is possible and HNR PVT is send cyclically already
//   boolean setAutoHNRPVT(boolean enabled, uint16_t maxWait = defaultMaxWait);              //Enable/disable automatic HNR PVT reports at the HNR rate
//   boolean setAutoHNRPVT(boolean enabled, boolean implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HNR PVT reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
// 	boolean getHNRPVT(uint16_t maxWait = 1100); // Returns true if the get HNR PVT is successful. Data is returned in hnrPVT

private:
	//Depending on the sentence type the processor will load characters into different arrays
	enum SentenceTypes
	{
		NONE = 0,
		NMEA,
		UBX,
		RTCM
	} currentSentence = NONE;

	//Depending on the ubx binary response class, store binary responses into different places
	// enum classTypes
	// {
	// 	CLASS_NONE = 0,
	// 	CLASS_ACK,
	// 	CLASS_NOT_AN_ACK
	// } ubxFrameClass = CLASS_NONE;

	enum commTypes
	{
		COMM_TYPE_I2C = 0,
		COMM_TYPE_SERIAL,
		COMM_TYPE_SPI
	} commType = COMM_TYPE_I2C; //Controls which port we look to for incoming bytes

	//Functions
	boolean checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass = 255, uint8_t requestedID = 255); //Checks module with user selected commType
	uint32_t extractLong(uint8_t spotToStart);																	 //Combine four bytes from payload into long
	int32_t extractSignedLong(uint8_t spotToStart);																//Combine four bytes from payload into signed long (avoiding any ambiguity caused by casting)
	uint16_t extractInt(uint8_t spotToStart);																	 //Combine two bytes from payload into int
	int16_t extractSignedInt(int8_t spotToStart);
	uint8_t extractByte(uint8_t spotToStart);																	 //Get byte from payload
	int8_t extractSignedChar(uint8_t spotToStart);																 //Get signed 8-bit value from payload
	void addToChecksum(uint8_t incoming);																		 //Given an incoming byte, adjust rollingChecksumA/B

	//Variables
	TwoWire *_i2cPort;				//The generic connection to user's chosen I2C hardware
	Stream *_serialPort;			//The generic connection to user's chosen Serial hardware
	Stream *_nmeaOutputPort = NULL; //The user can assign an output port to print NMEA sentences if they wish
	// Stream *_debugSerial;			//The stream to send debug messages to if enabled

	uint8_t _gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series
	//This can be changed using the ublox configuration software

	// boolean _printDebug = false;		//Flag to print the serial commands we are sending to the Serial port for debug
	// boolean _printLimitedDebug = false; //Flag to print limited debug messages. Useful for I2C debugging or high navigation rates

	//The packet buffers
	//These are pointed at from within the ubxPacket
	uint8_t payloadAck[2];				  // Holds the requested ACK/NACK
	uint8_t payloadCfg[MAX_PAYLOAD_SIZE]; // Holds the requested data packet
	uint8_t payloadBuf[2];				  // Temporary buffer used to screen incoming packets or dump unrequested packets

	//Init the packet structures and init them with pointers to the payloadAck, payloadCfg and payloadBuf arrays
	ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
	ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
	ubxPacket packetBuf = {0, 0, 0, 0, 0, payloadBuf, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

	//Flag if this packet is unrequested (and so should be ignored and not copied into packetCfg or packetAck)
	boolean ignoreThisPayload = false;

	//Identify which buffer is in use
	//Data is stored in packetBuf until the requested class and ID can be validated
	//If a match is seen, data is diverted into packetAck or packetCfg
	sfe_ublox_packet_buffer_e activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;

	//Limit checking of new data to every X ms
	//If we are expecting an update every X Hz then we should check every half that amount of time
	//Otherwise we may block ourselves from seeing new data
	uint8_t i2cPollingWait = 100; //Default to 100ms. Adjusted when user calls setNavigationFrequency()

	unsigned long lastCheck = 0;
	boolean autoPVT = false;			  //Whether autoPVT is enabled or not
	boolean autoPVTImplicitUpdate = true; // Whether autoPVT is triggered by accessing stale data (=true) or by a call to checkUblox (=false)
// 	boolean autoHPPOSLLH = false;			  //Whether autoHPPOSLLH is enabled or not
// 	boolean autoHPPOSLLHImplicitUpdate = true; // Whether autoHPPOSLLH is triggered by accessing stale data (=true) or by a call to checkUblox (=false)
//   boolean autoDOP = false;       //Whether autoDOP is enabled or not
//   boolean autoDOPImplicitUpdate = true; // Whether autoDOP is triggered by accessing stale data (=true) or by a call to checkUblox (=false)
// 	boolean autoHNRAtt = false;       //Whether auto HNR attitude is enabled or not
//   boolean autoHNRAttImplicitUpdate = true; // Whether auto HNR attitude is triggered by accessing stale data (=true) or by a call to checkUblox (=false)
// 	boolean autoHNRDyn = false;       //Whether auto HNR dynamics is enabled or not
//   boolean autoHNRDynImplicitUpdate = true; // Whether auto HNR dynamics is triggered by accessing stale data (=true) or by a call to checkUblox (=false)
// 	boolean autoHNRPVT = false;       //Whether auto HNR PVT is enabled or not
//   boolean autoHNRPVTImplicitUpdate = true; // Whether auto HNR PVT is triggered by accessing stale data (=true) or by a call to checkUblox (=false)

	uint16_t ubxFrameCounter;			  //It counts all UBX frame. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]

	uint8_t rollingChecksumA; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
	uint8_t rollingChecksumB; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

	//Create bit field for staleness of each datum in PVT we want to monitor
	// moduleQueried.latitude goes true each time we call getPVT()
	//This reduces the number of times we have to call getPVT as this can take up to ~1s per read
	//depending on update rate
	struct
	{
		// uint32_t gpsiTOW : 1;
		// uint32_t gpsYear : 1;
		// uint32_t gpsMonth : 1;
		// uint32_t gpsDay : 1;
		// uint32_t gpsHour : 1;
		// uint32_t gpsMinute : 1;
		// uint32_t gpsSecond : 1;
		// uint32_t gpsDateValid : 1;
		// uint32_t gpsTimeValid : 1;
		// uint32_t gpsNanosecond : 1;

		uint32_t all : 1;
		// uint32_t gnssFixOk : 1;
		// uint32_t diffSoln : 1;
		// uint32_t headVehValid : 1;
		uint32_t longitude : 1;
		uint32_t latitude : 1;
		// uint32_t altitude : 1;
		// uint32_t altitudeMSL : 1;
		// uint32_t horizontalAccEst : 1;
		// uint32_t verticalAccEst : 1;
		// uint32_t nedNorthVel : 1;
		// uint32_t nedEastVel : 1;
		// uint32_t nedDownVel : 1;
		// uint32_t SIV : 1;
		// uint32_t fixType : 1;
		// uint32_t carrierSolution : 1;
		// uint32_t groundSpeed : 1;
		// uint32_t headingOfMotion : 1;
		// uint32_t speedAccEst : 1;
		// uint32_t headingAccEst : 1;
		// uint32_t pDOP : 1;
		// uint32_t invalidLlh : 1;
		// uint32_t headVeh : 1;
		// uint32_t magDec : 1;
		// uint32_t magAcc : 1;
		// uint32_t versionNumber : 1;
	} moduleQueried;

	struct
	{
		// uint16_t all : 1;
		// uint16_t timeOfWeek : 1;
		uint16_t highResLatitude : 1;
		uint16_t highResLongitude : 1;
		// uint16_t elipsoid : 1;
		// uint16_t meanSeaLevel : 1;
		// uint16_t geoidSeparation : 1; // Redundant but kept for backward-compatibility
		// uint16_t horizontalAccuracy : 1;
		// uint16_t verticalAccuracy : 1;
		// uint16_t elipsoidHp : 1;
		// uint16_t meanSeaLevelHp : 1;
		uint16_t highResLatitudeHp : 1;
		uint16_t highResLongitudeHp : 1;
	} highResModuleQueried;

//   struct
//   {
//     uint16_t all : 1;
//     uint16_t geometricDOP : 1;
//     uint16_t positionDOP : 1;
//     uint16_t timeDOP : 1;
//     uint16_t verticalDOP : 1;
//     uint16_t horizontalDOP : 1;
//     uint16_t northingDOP : 1;
//     uint16_t eastingDOP : 1;
//   } dopModuleQueried;

	// boolean hnrAttQueried;
	// boolean hnrDynQueried;
	// boolean hnrPVTQueried;

	uint16_t rtcmLen = 0;
};

#endif


SFE_UBLOX_GPS::SFE_UBLOX_GPS(void)
{
  // Constructor
  currentGeofenceParams.numFences = 0; // Zero the number of geofences currently in use
  // moduleQueried.versionNumber = false;

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
  // moduleQueried.all = false;

  return (latitude);
}

//Get the current longitude in degrees
//Returns a long representing the number of degrees *10^-7
int32_t SFE_UBLOX_GPS::getLongitude(uint16_t maxWait)
{
  if (moduleQueried.longitude == false)
    getPVT(maxWait);
  moduleQueried.longitude = false; //Since we are about to give this to user, mark this data as stale
  // moduleQueried.all = false;

  return (longitude);
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

//Returns true if I2C device ack's
boolean SFE_UBLOX_GPS::isConnected(uint16_t maxWait)
{
  if (commType == COMM_TYPE_I2C)
  {
    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
    if (_i2cPort->endTransmission() != 0)
      return false; //Sensor did not ack
  }
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
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: valid data and valid ACK received after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec"));
        // }
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data and a correct ACK!
      }

      // We can be confident that the data packet (if we are going to get one) will always arrive
      // before the matching ACK. So if we sent a config packet which only produces an ACK
      // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
      // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
      // as these may have been changed by a PVT packet.
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: no data and valid ACK after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec"));
        // }
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
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: data being OVERWRITTEN after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec"));
        // }
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
      // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
      else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: CRC failed after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec"));
        // }
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
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec"));
        // }
        return (SFE_UBLOX_STATUS_COMMAND_NACK); //We received a NACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
      // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
      // If we were playing safe, we should return FAIL instead
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: VALID data and INVALID ACK received after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec"));
        // }
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
      // then we return a FAIL. This must be a double checksum failure?
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: INVALID data and INVALID ACK received after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec"));
        // }
        return (SFE_UBLOX_STATUS_FAIL); //We received invalid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
      // then the ACK has not yet been received and we should keep waiting for it
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: valid data after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec. Waiting for ACK."));
        // }
      }

    } //checkUbloxInternal == true

    delayMicroseconds(500);
  } //while (millis() - startTime < maxTime)

  // We have timed out...
  // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
  // even though we did not get an ACK
  if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("waitForACKResponse: TIMEOUT with valid data after "));
    //   _debugSerial->print(millis() - startTime);
    //   _debugSerial->println(F(" msec. "));
    // }
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data... But no ACK!
  }

  // if (_printDebug == true)
  // {
  //   _debugSerial->print(F("waitForACKResponse: TIMEOUT after "));
  //   _debugSerial->print(millis() - startTime);
  //   _debugSerial->println(F(" msec."));
  // }

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
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForNoACKResponse: valid data with CLS/ID match after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec"));
        // }
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
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForNoACKResponse: data being OVERWRITTEN after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec"));
        // }
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
      //   if (_printDebug == true)
      //   {
      //     _debugSerial->print(F("waitForNoACKResponse: CLS/ID match but failed CRC after "));
      //     _debugSerial->print(millis() - startTime);
      //     _debugSerial->println(F(" msec"));
      //   }
      //   return (SFE_UBLOX_STATUS_CRC_FAIL); //We received invalid data
      }
    }

    delayMicroseconds(500);
  }

  // if (_printDebug == true)
  // {
  //   _debugSerial->print(F("waitForNoACKResponse: TIMEOUT after "));
  //   _debugSerial->print(millis() - startTime);
  //   _debugSerial->println(F(" msec. No packet received."));
  // }

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

//Given a packet and payload, send everything including CRC bytes via I2C port
sfe_ublox_status_e SFE_UBLOX_GPS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait)
{
  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

  calcChecksum(outgoingUBX); //Sets checksum A and B bytes of the packet

  // if (_printDebug == true)
  // {
  //   _debugSerial->print(F("\nSending: "));
  //   printPacket(outgoingUBX);
  // }

  if (commType == COMM_TYPE_I2C)
  {
    retVal = sendI2cCommand(outgoingUBX, maxWait);
    if (retVal != SFE_UBLOX_STATUS_SUCCESS)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("Send I2C Command failed"));
      // }
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
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("sendCommand: Waiting for ACK response"));
      // }
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); //Wait for Ack response
    }
    else
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("sendCommand: Waiting for No ACK response"));
      // }
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
        // if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
        // {
        //   _debugSerial->println(F("checkUbloxI2C: u-blox bug, length lsb is 0xFF"));
        // }
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
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("checkUbloxI2C: OK, zero bytes available"));
      // }
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

      // if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
      // {
      //   _debugSerial->print(F("checkUbloxI2C: Bytes available error:"));
      //   _debugSerial->println(bytesAvailable);
      //   if (checksumFailurePin >= 0)
      //   {
      //     digitalWrite((uint8_t)checksumFailurePin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)checksumFailurePin, HIGH);
      //   }
      // }
    }

    if (bytesAvailable > 100)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->print(F("checkUbloxI2C: Large packet of "));
      //   _debugSerial->print(bytesAvailable);
      //   _debugSerial->println(F(" bytes received"));
      // }
    }
    else
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->print(F("checkUbloxI2C: Reading "));
      //   _debugSerial->print(bytesAvailable);
      //   _debugSerial->println(F(" bytes"));
      // }
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
              // if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
              // {
              //   _debugSerial->println(F("checkUbloxU2C: u-blox error, module not ready with data"));
              // }
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
  // if (_printDebug == true)
  // {
  //   _debugSerial->print(F("CLS:"));
  //   if (packet->cls == UBX_CLASS_NAV) //1
  //     _debugSerial->print(F("NAV"));
  //   else if (packet->cls == UBX_CLASS_ACK) //5
  //     _debugSerial->print(F("ACK"));
  //   else if (packet->cls == UBX_CLASS_CFG) //6
  //     _debugSerial->print(F("CFG"));
  //   else if (packet->cls == UBX_CLASS_MON) //0x0A
  //     _debugSerial->print(F("MON"));
  //   else
  //   {
  //     _debugSerial->print(F("0x"));
  //     _debugSerial->print(packet->cls, HEX);
  //   }

  //   _debugSerial->print(F(" ID:"));
  //   if (packet->cls == UBX_CLASS_NAV && packet->id == UBX_NAV_PVT)
  //     _debugSerial->print(F("PVT"));
  //   else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_RATE)
  //     _debugSerial->print(F("RATE"));
  //   else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_CFG)
  //     _debugSerial->print(F("SAVE"));
  //   else
  //   {
  //     _debugSerial->print(F("0x"));
  //     _debugSerial->print(packet->id, HEX);
  //   }

  //   _debugSerial->print(F(" Len: 0x"));
  //   _debugSerial->print(packet->len, HEX);

  //   // Only print the payload is ignoreThisPayload is false otherwise
  //   // we could be printing gibberish from beyond the end of packetBuf
  //   if (ignoreThisPayload == false)
  //   {
  //     _debugSerial->print(F(" Payload:"));

  //     for (int x = 0; x < packet->len; x++)
  //     {
  //       _debugSerial->print(F(" "));
  //       _debugSerial->print(packet->payload[x], HEX);
  //     }
  //   }
  //   else
  //   {
  //     _debugSerial->print(F(" Payload: IGNORED"));
  //   }
  //   _debugSerial->println();
  // }
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
      // gnssFixOk = extractByte(21 - startingSpot) & 0x1; //Get the 1st bit
      // diffSoln = (extractByte(21 - startingSpot) >> 1) & 0x1; //Get the 2nd bit
      // carrierSolution = extractByte(21 - startingSpot) >> 6; //Get 6th&7th bits of this byte
      // headVehValid = (extractByte(21 - startingSpot) >> 5) & 0x1; // Get the 5th bit
      // SIV = extractByte(23 - startingSpot);
      longitude = extractSignedLong(24 - startingSpot);
      latitude = extractSignedLong(28 - startingSpot);
      // altitude = extractSignedLong(32 - startingSpot);
      // altitudeMSL = extractSignedLong(36 - startingSpot);
      // horizontalAccEst = extractLong(40 - startingSpot);
      // verticalAccEst = extractLong(44 - startingSpot);
      // nedNorthVel = extractSignedLong(48 - startingSpot);
      // nedEastVel = extractSignedLong(52 - startingSpot);
      // nedDownVel = extractSignedLong(56 - startingSpot);
      // groundSpeed = extractSignedLong(60 - startingSpot);
      // headingOfMotion = extractSignedLong(64 - startingSpot);
      // speedAccEst = extractLong(68 - startingSpot);
      // headingAccEst = extractLong(72 - startingSpot);
      // pDOP = extractInt(76 - startingSpot);
      // invalidLlh = extractByte(78 - startingSpot) & 0x1;
      // headVeh = extractSignedLong(84 - startingSpot);
      // magDec = extractSignedInt(88 - startingSpot);
      // magAcc = extractInt(90 - startingSpot);

      //Mark all datums as fresh (not read before)
      // moduleQueried.gpsiTOW = true;
      // moduleQueried.gpsYear = true;
      // moduleQueried.gpsMonth = true;
      // moduleQueried.gpsDay = true;
      // moduleQueried.gpsHour = true;
      // moduleQueried.gpsMinute = true;
      // moduleQueried.gpsSecond = true;
      // moduleQueried.gpsDateValid = true;
      // moduleQueried.gpsTimeValid = true;
      // moduleQueried.gpsNanosecond = true;

      // moduleQueried.all = true;
      // moduleQueried.gnssFixOk = true;
      // moduleQueried.diffSoln = true;
      // moduleQueried.headVehValid = true;
      moduleQueried.longitude = true;
      moduleQueried.latitude = true;
      // moduleQueried.altitude = true;
      // moduleQueried.altitudeMSL = true;
      // moduleQueried.horizontalAccEst = true;
      // moduleQueried.verticalAccEst = true;
      // moduleQueried.nedNorthVel = true;
      // moduleQueried.nedEastVel = true;
      // moduleQueried.nedDownVel = true;
      // moduleQueried.SIV = true;
      // moduleQueried.fixType = true;
      // moduleQueried.carrierSolution = true;
      // moduleQueried.groundSpeed = true;
      // moduleQueried.headingOfMotion = true;
      // moduleQueried.speedAccEst = true;
      // moduleQueried.headingAccEst = true;
      // moduleQueried.pDOP = true;
      // moduleQueried.invalidLlh = true;
      // moduleQueried.headVeh = true;
      // moduleQueried.magDec = true;
      // moduleQueried.magAcc = true;
    }
    else if (msg->id == UBX_NAV_HPPOSLLH && msg->len == 36)
    {
      // timeOfWeek = extractLong(4);
      highResLongitude = extractSignedLong(8);
      highResLatitude = extractSignedLong(12);
      // elipsoid = extractSignedLong(16);
      // meanSeaLevel = extractSignedLong(20);
      highResLongitudeHp = extractSignedChar(24);
      highResLatitudeHp = extractSignedChar(25);
      // elipsoidHp = extractSignedChar(26);
      // meanSeaLevelHp = extractSignedChar(27);
      // horizontalAccuracy = extractLong(28);
      // verticalAccuracy = extractLong(32);

      // highResModuleQueried.all = true;
      highResModuleQueried.highResLatitude = true;
      highResModuleQueried.highResLatitudeHp = true;
      highResModuleQueried.highResLongitude = true;
      highResModuleQueried.highResLongitudeHp = true;
      // highResModuleQueried.elipsoid = true;
      // highResModuleQueried.elipsoidHp = true;
      // highResModuleQueried.meanSeaLevel = true;
      // highResModuleQueried.meanSeaLevelHp = true;
      // highResModuleQueried.geoidSeparation = true;
      // highResModuleQueried.horizontalAccuracy = true;
      // highResModuleQueried.verticalAccuracy = true;
      // moduleQueried.gpsiTOW = true; // this can arrive via HPPOS too.

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
      // geometricDOP = extractInt(4);
      // positionDOP = extractInt(6);
      // timeDOP = extractInt(8);
      // verticalDOP = extractInt(10);
      // horizontalDOP = extractInt(12);
      // northingDOP = extractInt(14);
      // eastingDOP = extractInt(16);
      // dopModuleQueried.all = true;
      // dopModuleQueried.geometricDOP = true;
      // dopModuleQueried.positionDOP = true;
      // dopModuleQueried.timeDOP = true;
      // dopModuleQueried.verticalDOP = true;
      // dopModuleQueried.horizontalDOP = true;
      // dopModuleQueried.northingDOP = true;
      // dopModuleQueried.eastingDOP = true;
    }
    break;
  case UBX_CLASS_HNR:
    if (msg->id == UBX_HNR_ATT && msg->len == 32)
    {
      //Parse various byte fields into global vars
      // hnrAtt.iTOW = extractLong(0);
      // hnrAtt.roll = extractSignedLong(8);
      // hnrAtt.pitch = extractSignedLong(12);
      // hnrAtt.heading = extractSignedLong(16);
      // hnrAtt.accRoll = extractLong(20);
      // hnrAtt.accPitch = extractLong(24);
      // hnrAtt.accHeading = extractLong(28);

      // hnrAttQueried = true;
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

      // hnrDynQueried = true;
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
      // hnrPVT.gpsFixOK = (flags & 0x01) > 0;
      // hnrPVT.diffSoln = (flags & 0x02) > 0;
      // hnrPVT.WKNSET = (flags & 0x04) > 0;
      // hnrPVT.TOWSET = (flags & 0x08) > 0;
      // hnrPVT.headVehValid = (flags & 0x10) > 0;

      // hnrPVTQueried = true;
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
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("processUBX: NACK received: Requested Class: 0x"));
        //   _debugSerial->print(incomingUBX->payload[0], HEX);
        //   _debugSerial->print(F(" Requested ID: 0x"));
        //   _debugSerial->println(incomingUBX->payload[1], HEX);
        // }
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
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("processUBX: auto NAV PVT/HPPOSLLH/DOP collision: Requested ID: 0x"));
        //   _debugSerial->print(requestedID, HEX);
        //   _debugSerial->print(F(" Message ID: 0x"));
        //   _debugSerial->println(incomingUBX->id, HEX);
        // }
      }
      // Let's do the same for the HNR messages
      else if ((incomingUBX->cls == requestedClass) &&
        (((incomingUBX->id == UBX_HNR_ATT) && (requestedID == UBX_HNR_INS || requestedID == UBX_HNR_PVT)) ||
         ((incomingUBX->id == UBX_HNR_INS) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_PVT)) ||
         ((incomingUBX->id == UBX_HNR_PVT) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_INS))))
       {
         // This isn't the message we are looking for...
         // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
        //  if (_printDebug == true)
        //  {
        //    _debugSerial->print(F("processUBX: auto HNR ATT/INS/PVT collision: Requested ID: 0x"));
        //    _debugSerial->print(requestedID, HEX);
        //    _debugSerial->print(F(" Message ID: 0x"));
        //    _debugSerial->println(incomingUBX->id, HEX);
        //  }
       }

      // if (_printDebug == true)
      // {
      //   _debugSerial->print(F("Incoming: Size: "));
      //   _debugSerial->print(incomingUBX->len);
      //   _debugSerial->print(F(" Received: "));
      //   printPacket(incomingUBX);

      //   if (incomingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
      //   {
      //     _debugSerial->println(F("packetCfg now valid"));
      //   }
      //   if (packetAck.valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
      //   {
      //     _debugSerial->println(F("packetAck now valid"));
      //   }
      //   if (incomingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
      //   {
      //     _debugSerial->println(F("packetCfg classAndIDmatch"));
      //   }
      //   if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
      //   {
      //     _debugSerial->println(F("packetAck classAndIDmatch"));
      //   }
      // }

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

      // if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
      // {
      //   //Drive an external pin to allow for easier logic analyzation
      //   if (checksumFailurePin >= 0)
      //   {
      //     digitalWrite((uint8_t)checksumFailurePin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)checksumFailurePin, HIGH);
      //   }

      //   _debugSerial->print(F("Checksum failed:"));
      //   _debugSerial->print(F(" checksumA: "));
      //   _debugSerial->print(incomingUBX->checksumA);
      //   _debugSerial->print(F(" checksumB: "));
      //   _debugSerial->print(incomingUBX->checksumB);

      //   _debugSerial->print(F(" rollingChecksumA: "));
      //   _debugSerial->print(rollingChecksumA);
      //   _debugSerial->print(F(" rollingChecksumB: "));
      //   _debugSerial->print(rollingChecksumB);
      //   _debugSerial->println();

      //   _debugSerial->print(F("Failed  : "));
      //   _debugSerial->print(F("Size: "));
      //   _debugSerial->print(incomingUBX->len);
      //   _debugSerial->print(F(" Received: "));
      //   printPacket(incomingUBX);
      // }
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
    // if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
    // {
    //   if (overrun)
    //     _debugSerial->println(F("processUBX: buffer overrun detected"));
    //   else
    //     _debugSerial->println(F("processUBX: counter hit MAX_PAYLOAD_SIZE"));
    // }
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
          // if (_printDebug == true)
          // {
          //   _debugSerial->print(F("process: auto NAV PVT/HPPOSLLH/DOP collision: Requested ID: 0x"));
          //   _debugSerial->print(requestedID, HEX);
          //   _debugSerial->print(F(" Message ID: 0x"));
          //   _debugSerial->println(packetBuf.id, HEX);
          // }
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
          // if (_printDebug == true)
          // {
          //   _debugSerial->print(F("process: auto HNR ATT/INS/PVT collision: Requested ID: 0x"));
          //   _debugSerial->print(requestedID, HEX);
          //   _debugSerial->print(F(" Message ID: 0x"));
          //   _debugSerial->println(packetBuf.id, HEX);
          // }
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
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("process: ZERO LENGTH packet received: Class: 0x"));
        //   _debugSerial->print(packetBuf.cls, HEX);
        //   _debugSerial->print(F(" ID: 0x"));
        //   _debugSerial->println(packetBuf.id, HEX);
        // }
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
          // if (_printDebug == true)
          // {
          //   _debugSerial->print(F("process: ACK received with .len != 2: Class: 0x"));
          //   _debugSerial->print(packetBuf.payload[0], HEX);
          //   _debugSerial->print(F(" ID: 0x"));
          //   _debugSerial->print(packetBuf.payload[1], HEX);
          //   _debugSerial->print(F(" len: "));
          //   _debugSerial->println(packetBuf.len);
          // }
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

//Get the latest Position/Velocity/Time solution and fill all global variables
boolean SFE_UBLOX_GPS::getPVT(uint16_t maxWait)
{
  if (autoPVT && autoPVTImplicitUpdate)
  {
    //The GPS is automatically reporting, we just check whether we got unread data
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getPVT: Autoreporting"));
    // }
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_PVT);
    return moduleQueried.all;
  }
  else if (autoPVT && !autoPVTImplicitUpdate)
  {
    //Someone else has to call checkUblox for us...
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getPVT: Exit immediately"));
    // }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getPVT: Polling"));
    // }

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
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getPVT: data was OVERWRITTEN by another NAV message (but that's OK)"));
      // }
      return (true);
    }

    if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_HNR))
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getPVT: data was OVERWRITTEN by a HNR message (and that's not OK)"));
      // }
      return (false);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getPVT retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }
}