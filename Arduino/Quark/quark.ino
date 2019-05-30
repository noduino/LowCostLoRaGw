/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *  Based on the Arduino_LoRa_Demo_Sensor sketch
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/

#include <SPI.h>
#include "sx1272.h"
#include "pressure.h"
#include "vbat.h"

///////////////////////////////////////////////////////////////////
//#define WITH_EEPROM
//#define WITH_APPKEY
#define LOW_POWER
//#define WITH_ACK
//#define LOW_POWER_TEST
///////////////////////////////////////////////////////////////////

// IMPORTANT SETTINGS
///////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
//#define ETSI_EUROPE_REGULATION
//#define FCC_US_REGULATION
//#define SENEGAL_REGULATION
#define LONG_RANG_TESTING
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
//#define BAND868
//#define BAND900
#define BAND433
//#define BAND470
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// uncomment if the rf output of your radio module use the PABOOST
// line instead of the RFO line
#define PABOOST
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE NODE ADDRESS 
#define node_addr		254
#define DEST_ADDR		1
//////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE LORA MODE
#define LORAMODE		11	// BW=125KHz, SF=12, CR=4/5, sync=0x34
//////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE TIME IN SECONDS BETWEEN 2 READING & TRANSMISSION
unsigned int idlePeriod = 40;	// 40 seconds
///////////////////////////////////////////////////////////////////

#ifdef WITH_APPKEY
// CHANGE HERE THE APPKEY, BUT IF GW CHECKS FOR APPKEY, MUST BE
// IN THE APPKEY LIST MAINTAINED BY GW.
uint8_t my_appKey[4] = { 5, 6, 8, 8 };
#endif

///////////////////////////////////////////////////////////////////
// IF YOU SEND A LONG STRING, INCREASE THE SIZE OF MESSAGE
uint8_t message[50];
///////////////////////////////////////////////////////////////////

#define PRINTLN                   Serial.println("")
#define PRINT_CSTSTR(fmt,param)   Serial.print(F(param))
#define PRINT_STR(fmt,param)      Serial.print(param)
#define PRINT_VALUE(fmt,param)    Serial.print(param)
#define FLUSHOUTPUT               Serial.flush();

#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif

#ifdef ETSI_EUROPE_REGULATION
#define MAX_DBM 14
// previous way for setting output power
// char powerLevel='M';
#elif defined SENEGAL_REGULATION
#define MAX_DBM 10
// previous way for setting output power
// 'H' is actually 6dBm, so better to use the new way to set output power
// char powerLevel='H';
#elif defined FCC_US_REGULATION
#define MAX_DBM 14
#elif defined LONG_RANG_TESTING
#define MAX_DBM 20
#endif

#ifdef BAND868
#ifdef SENEGAL_REGULATION
const uint32_t DEFAULT_CHANNEL = CH_04_868;
#else
const uint32_t DEFAULT_CHANNEL = CH_10_868;
#endif
#elif defined BAND900
//const uint32_t DEFAULT_CHANNEL=CH_05_900;
// For HongKong, Japan, Malaysia, Singapore, Thailand, Vietnam: 920.36MHz     
const uint32_t DEFAULT_CHANNEL = CH_08_900;
#elif defined BAND433
const uint32_t DEFAULT_CHANNEL = CH_00_433;	// 433.3MHz
//const uint32_t DEFAULT_CHANNEL = CH_03_433;	// 434.3MHz
#elif defined BAND470
const uint32_t DEFAULT_CHANNEL = CH_00_470;	// 470.0MHz
#endif

#ifdef WITH_ACK
#define	NB_RETRIES			2
#endif

#ifdef LOW_POWER
#define	LOW_POWER_PERIOD	8
// you need the LowPower library from RocketScream
// https://github.com/rocketscream/Low-Power
#include "LowPower.h"
unsigned int nCycle = idlePeriod / LOW_POWER_PERIOD;
#endif

unsigned long nextTransmissionTime = 0L;
int loraMode = LORAMODE;

#ifdef WITH_EEPROM
struct sx1272config {

	uint8_t flag1;
	uint8_t flag2;
	uint8_t seq;
	// can add other fields such as LoRa mode,...
};

sx1272config my_sx1272config;
#endif

char *ftoa(char *a, double f, int precision)
{
	long p[] =
	    { 0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000 };

	char *ret = a;
	long heiltal = (long)f;
	itoa(heiltal, a, 10);
	while (*a != '\0')
		a++;
	*a++ = '.';
	long desimal = abs((long)((f - heiltal) * p[precision]));
	if (desimal < p[precision - 1]) {
		*a++ = '0';
	}
	itoa(desimal, a, 10);
	return ret;
}

void power_on_dev()
{
	digitalWrite(6, HIGH);
}

void power_off_dev()
{
	digitalWrite(6, LOW);
}

void setup()
{
	int e;

	pinMode(6, OUTPUT);

	// Open serial communications and wait for port to open:
	Serial.begin(115200);

	// Print a start message
	PRINT_CSTSTR("%s", "Noduino Quark LoRa Node\n");

// See http://www.nongnu.org/avr-libc/user-manual/using_tools.html
// for the list of define from the AVR compiler
#ifdef __AVR_ATmega328P__
	PRINT_CSTSTR("%s", "ATmega328P detected\n");
#endif

	power_on_dev();		// turn on device power

	pressure_init();	// initialization of the sensor
	sx1272.ON();		// power on the module

#ifdef WITH_EEPROM
	// get config from EEPROM
	EEPROM.get(0, my_sx1272config);

	// found a valid config?
	if (my_sx1272config.flag1 == 0x12 && my_sx1272config.flag2 == 0x34) {
		PRINT_CSTSTR("%s", "Get back previous sx1272 config\n");

		// set sequence number for SX1272 library
		sx1272._packetNumber = my_sx1272config.seq;
		PRINT_CSTSTR("%s", "Using packet sequence number of ");
		PRINT_VALUE("%d", sx1272._packetNumber);
		PRINTLN;
	} else {
		// otherwise, write config and start over
		my_sx1272config.flag1 = 0x12;
		my_sx1272config.flag2 = 0x34;
		my_sx1272config.seq = sx1272._packetNumber;
	}
#endif

	// We use the LoRaWAN mode:
	// BW=125KHz, SF=12, CR=4/5, sync=0x34
	e = sx1272.setMode(loraMode);
	PRINT_CSTSTR("%s", "Setting Mode: state ");
	PRINT_VALUE("%d", e);
	PRINTLN;

	// Select frequency channel
	e = sx1272.setChannel(DEFAULT_CHANNEL);
	PRINT_CSTSTR("%s", "Setting Channel: state ");
	PRINT_VALUE("%d", e);
	PRINTLN;

	// Select amplifier line; PABOOST or RFO
#ifdef PABOOST
	sx1272._needPABOOST = true;
#endif

	e = sx1272.setPowerDBM((uint8_t) MAX_DBM);
	PRINT_CSTSTR("%s", "Setting Power: state ");
	PRINT_VALUE("%d", e);
	PRINTLN;

	// Set the node address and print the result
	e = sx1272.setNodeAddress(node_addr);
	PRINT_CSTSTR("%s", "Setting node addr: state ");
	PRINT_VALUE("%d", e);
	PRINTLN;

	// enable carrier sense
	//sx1272._enableCarrierSense = true;

#ifdef LOW_POWER
	// TODO: with low power, when setting the radio module in sleep mode
	// there seem to be some issue with RSSI reading
	//sx1272._RSSIonSend = false;
#endif

	// Set CRC off
	//e = sx1272.setCRC_OFF();
	//PRINT_CSTSTR("%s","Setting CRC off: state ");
	//PRINT_VALUE("%d", e);
	//PRINTLN;

	PRINT_CSTSTR("%s", "SX1272 successfully configured\n");
}

void loop(void)
{
	long startSend;
	long endSend;
	uint8_t app_key_offset = 0;
	int e;
	float pres, vbat;

#ifndef LOW_POWER
	// 600000+random(15,60)*1000
	if (millis() > nextTransmissionTime) {
#endif

		pres = get_pressure();
		vbat = get_vbat();

		PRINT_CSTSTR("%s", "Pressure is ");
		PRINT_VALUE("%f", pres);
		PRINTLN;

#ifdef WITH_APPKEY
		app_key_offset = sizeof(my_appKey);
		// set the app key in the payload
		memcpy(message, my_appKey, app_key_offset);
#endif

		uint8_t r_size;

		// the recommended format if now \!TC/22.5
		char vbat_s[10], pres_s[10];
		ftoa(vbat_s, vbat, 2);
		ftoa(pres_s, pres, 2);
		PRINT_CSTSTR("%s", "Vbat = [");
		PRINT_STR("%s", (char *)vbat_s);
		PRINT_CSTSTR("%s", "]");
		PRINTLN;

		// this is for testing, uncomment if you just want to test, without a real pressure sensor plugged
		//strcpy(vbat_s, "noduino");
		r_size = sprintf((char *)message + app_key_offset, "\\!U/%s/P/%s", vbat_s, pres_s);

		PRINT_CSTSTR("%s", "Sending ");
		PRINT_STR("%s", (char *)(message + app_key_offset));
		PRINTLN;

		PRINT_CSTSTR("%s", "Real payload size is ");
		PRINT_VALUE("%d", r_size);
		PRINTLN;

		int pl = r_size + app_key_offset;

		//sx1272.CarrierSense();

		startSend = millis();

#ifdef WITH_APPKEY
		// indicate that we have an appkey
		sx1272.setPacketType(PKT_TYPE_DATA | PKT_FLAG_DATA_WAPPKEY);
#else
		// just a simple data packet
		sx1272.setPacketType(PKT_TYPE_DATA);
#endif

		// Send message to the gateway and print the result
		// with the app key if this feature is enabled
#ifdef WITH_ACK
		int n_retry = NB_RETRIES;

		do {
			e = sx1272.sendPacketTimeoutACK(DEST_ADDR,
							message, pl);

			if (e == 3)
				PRINT_CSTSTR("%s", "No ACK");

			n_retry--;

			if (n_retry)
				PRINT_CSTSTR("%s", "Retry");
			else
				PRINT_CSTSTR("%s", "Abort");

		} while (e && n_retry);
#else
		e = sx1272.sendPacketTimeout(DEST_ADDR, message, pl);
#endif
		endSend = millis();

#ifdef WITH_EEPROM
		// save packet number for next packet in case of reboot
		my_sx1272config.seq = sx1272._packetNumber;
		EEPROM.put(0, my_sx1272config);
#endif

		PRINT_CSTSTR("%s", "LoRa pkt size ");
		PRINT_VALUE("%d", pl);
		PRINTLN;

		PRINT_CSTSTR("%s", "LoRa pkt seq ");
		PRINT_VALUE("%d", sx1272.packet_sent.packnum);
		PRINTLN;

		PRINT_CSTSTR("%s", "LoRa Sent in ");
		PRINT_VALUE("%ld", endSend - startSend);
		PRINTLN;

		PRINT_CSTSTR("%s", "LoRa Sent w/CAD in ");
		PRINT_VALUE("%ld", endSend - sx1272._startDoCad);
		PRINTLN;

		PRINT_CSTSTR("%s", "Packet sent, state ");
		PRINT_VALUE("%d", e);
		PRINTLN;

		PRINT_CSTSTR("%s", "Remaining ToA is ");
		PRINT_VALUE("%d", sx1272.getRemainingToA());
		PRINTLN;

#ifdef LOW_POWER
		PRINT_CSTSTR("%s", "Switch to power saving mode\n");

		e = sx1272.setSleepMode();

		if (!e)
			PRINT_CSTSTR("%s", "Successfully switch LoRa into sleep mode\n");
		else
			PRINT_CSTSTR("%s", "Could not switch LoRa into sleep mode\n");

		FLUSHOUTPUT
#ifdef LOW_POWER_TEST
		delay(10000);
#else
		delay(50);
#endif

		for (int i = 0; i < nCycle; i++) {

			// ATmega328P, ATmega168, ATmega32U4
			LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

			PRINT_CSTSTR("%s", ".");
			FLUSHOUTPUT delay(10);
		}

		delay(50);
#else
		PRINT_VALUE("%ld", nextTransmissionTime);
		PRINTLN;
		PRINT_CSTSTR("%s", "Will send next value at\n");
		// use a random part also to avoid collision
		nextTransmissionTime =
		    millis() + (unsigned long)idlePeriod * 1000 +
		    (unsigned long)random(15, 60) * 10;
		PRINT_VALUE("%ld", nextTransmissionTime);
		PRINTLN;
	}
#endif
}
