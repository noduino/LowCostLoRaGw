/*
 *  Library for LoRa 868 / 915MHz SX1272 LoRa module
 *
 *  Copyright (C) Libelium Comunicaciones Distribuidas S.L.
 *  http://www.libelium.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Version:           1.1
 *  Design:            David Gascón
 *  Implementation:    Covadonga Albiñana & Victor Boria
 */

#include "SX1272.h"
#include "softspi.h"

// based on SIFS=3CAD
uint8_t sx1272_SIFS_value[11] = { 0, 183, 94, 44, 47, 23, 24, 12, 12, 7, 4 };
uint8_t sx1272_CAD_value[11] = { 0, 62, 31, 16, 16, 8, 9, 5, 3, 1, 1 };

//#define LIMIT_TOA
// 0.1% for testing
//#define MAX_DUTY_CYCLE_PER_HOUR 3600L
// 1%, regular mode
#define MAX_DUTY_CYCLE_PER_HOUR 36000L
// normally 1 hour, set to smaller value for testing
#define DUTYCYCLE_DURATION 3600000L
// 4 min for testing
//#define DUTYCYCLE_DURATION 240000L

SX1272::SX1272()
{
	//set the Chip Select pin
	_SX1272_SS = SX1272_SS;

	// Initialize class variables
	_bandwidth = BW_125;
	_codingRate = CR_5;
	_spreadingFactor = SF_12;
	_channel = CH_00_433;
	_header = HEADER_ON;
	_CRC = CRC_ON;
	_modem = LORA;
	_power = 17;
	_packetNumber = 0;
	_reception = CORRECT_PACKET;
	_retries = 0;

	_defaultSyncWord = 0x34;
	_rawFormat = false;
	_extendedIFS = false;
	_RSSIonSend = true;
	_enableCarrierSense = false;		// disabled by default

	// DIFS by default
	_send_cad_number = 9;
#ifdef PABOOST
	_needPABOOST = true;
#else
	_needPABOOST = false;
#endif
	_limitToA = false;
	_startToAcycle = millis();
	_remainingToA = MAX_DUTY_CYCLE_PER_HOUR;
	_endToAcycle = _startToAcycle + DUTYCYCLE_DURATION;
#ifdef W_REQUESTED_ACK
	_requestACK = 0;
#endif
#ifdef W_NET_KEY
	_my_netkey[0] = net_key_0;
	_my_netkey[1] = net_key_1;
#endif
	// we use the same memory area to reduce memory footprint
	packet_sent.data = packet_data;
	packet_received.data = packet_data;

	// ACK packet has a very small separate memory area
	ACK.data = ack_data;

	_maxRetries = 0;
	packet_sent.retry = _retries;

	pinMode(SX1272_RST, OUTPUT);
	digitalWrite(SX1272_RST, HIGH);

	pinMode(_SX1272_SS, OUTPUT);
	digitalWrite(_SX1272_SS, HIGH);

#ifdef SX1272_led_send_receive
	pinMode(SX1272_led_send, OUTPUT);
	pinMode(SX1272_led_receive, OUTPUT);
#endif
};

void SX1272::RxChainCalibration()
{
	if (_board == SX1276Chip) {

		Serial.println(F("SX1276 LF/HF calibration"));

		// Cut the PA just in case, RFO output, power = -1 dBm
		writeRegister(REG_PA_CONFIG, 0x00);

		// Launch Rx chain calibration for LF band
		writeRegister(REG_IMAGE_CAL,
			      (readRegister(REG_IMAGE_CAL) &
			       RF_IMAGECAL_IMAGECAL_MASK) |
			      RF_IMAGECAL_IMAGECAL_START);
		while ((readRegister(REG_IMAGE_CAL) &
			RF_IMAGECAL_IMAGECAL_RUNNING) ==
		       RF_IMAGECAL_IMAGECAL_RUNNING) {
			   // wait calibration ok
		}

#ifndef SX1278Chip
		// Sets a Frequency in HF band
		setChannel(CH_17_868);

		// Launch Rx chain calibration for HF band
		writeRegister(REG_IMAGE_CAL,
			      (readRegister(REG_IMAGE_CAL) &
			       RF_IMAGECAL_IMAGECAL_MASK) |
			      RF_IMAGECAL_IMAGECAL_START);
		while ((readRegister(REG_IMAGE_CAL) &
			RF_IMAGECAL_IMAGECAL_RUNNING) ==
		       RF_IMAGECAL_IMAGECAL_RUNNING) {
		}
#endif
	}
}

void SX1272::reset()
{
	digitalWrite(SX1272_RST, LOW);
	delay(200);
	digitalWrite(SX1272_RST, HIGH);
	delay(500);
}

void SX1272::sx1278_qsetup(uint32_t freq)
{
#ifdef USE_SOFTSPI
	spi_init();
#else
	SPI.begin();
#endif

	reset();

	_board = SX1276Chip;

	RxChainCalibration();

	setLORA();
	//writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
	//_modem = LORA;

#if 0
	//////////////////////////////////////////////////
	//setCRC_ON();
	uint8_t config2 = readRegister(REG_MODEM_CONFIG2);
	writeRegister(REG_MODEM_CONFIG2, config2 | B00000100);
	//////////////////////////////////////////////////

	//////////////////////////////////////////////////
	// BW=125KHz, SF=12, CR=4/5, sync=0x34
	//setMode(11);

	//setCR(CR_5);
	config1 = config1 & B11110001;
	config1 = config1 | B00000010;
	///////////////////////////////////

	//setSF(SF_12);
	config2 = config2 & B11001111;
	config2 = config2 | B11000000;

	byte config3 = readRegister(REG_MODEM_CONFIG3);
	config3 = config3 | B00001000;
	// set the AgcAutoOn in bit 2 of REG_MODEM_CONFIG3
	config3 = config3 | B00000100;
	writeRegister(REG_MODEM_CONFIG3, config3);

	//setHeaderON();
	config1 = config1 & B11111110;
	writeRegister(REG_MODEM_CONFIG1, config1);
	///////////////////////////////////

	// LoRa detection Optimize: 0x03 --> SF7 to SF12
	writeRegister(REG_DETECT_OPTIMIZE, 0x03);
	// LoRa detection threshold: 0x0A --> SF7 to SF12
	writeRegister(REG_DETECTION_THRESHOLD, 0x0A);

	writeRegister(REG_MODEM_CONFIG2, config2);
	///////////////////////////////////


	//setBW(BW_125);	// BW = 125 KHz
	config1 = config1 & B00001111;
	config1 = config1 | B01110000;

	byte config3 = readRegister(REG_MODEM_CONFIG3);
	config3 = config3 | B00001000;
	writeRegister(REG_MODEM_CONFIG3, config3);
	//////////////////////////////////////////////////
#endif
	getPreambleLength();

	randomSeed(millis());	//init random generator

	// 125KHz, 4/5, Explicit Header
	writeRegister(REG_MODEM_CONFIG1, 0x72);
	_bandwidth = BW_125;
	_codingRate = CR_5;
	_header = HEADER_ON;

	// SF = 12, TxContin single pkt, CRC On, Rx Timeout msb - 0x0
	writeRegister(REG_MODEM_CONFIG2, 0xC4);
	_spreadingFactor = SF_12;
	_CRC = CRC_ON;

	// set the AgcAutoOn in bit 2 of REG_MODEM_CONFIG3
	writeRegister(REG_MODEM_CONFIG3, 0x0C);

	// LoRa detection Optimize: 0x03 --> SF7 to SF12
	writeRegister(REG_DETECT_OPTIMIZE, 0x03);

	// LoRa detection threshold: 0x0A --> SF7 to SF12
	writeRegister(REG_DETECTION_THRESHOLD, 0x0A);

	//setSyncWord(0x34);
	writeRegister(REG_SYNC_WORD, 0x34);

	// Select frequency channel
	//setChannel(CH_433_00);
	writeRegister(REG_FRF_MSB, (freq >> 16) & 0xFF);
	writeRegister(REG_FRF_MID, (freq >> 8) & 0xFF);
	writeRegister(REG_FRF_LSB, freq & 0xFF);
	_channel = freq;

	_needPABOOST = true;

	//setPowerDBM(20);
	#define REG_PADAC		0x4D
	writeRegister(REG_PADAC, 0x87);
	writeRegister(REG_PA_CONFIG, 0xFF);

	//setMaxCurrent(0x0B);	// 100mA
	/*
	 * 0x12 -- 150mA
	 * 0x10 -- 130mA
	 * 0x0B -- 100mA
	 * 0x1B -- 240mA
	*/
	writeRegister(REG_OCP, 0x1B | 0b00100000);

	_power = 20;
}

uint8_t SX1272::ON()
{
	uint8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'ON'"));
#endif

#ifdef USE_SOFTSPI
	spi_init();
#else
	//Configure the MISO, MOSI, CS, SPCR.
	SPI.begin();
	//Set Most significant bit first
	SPI.setBitOrder(MSBFIRST);

	// for the DUE, set to 4MHz
	SPI.setClockDivider(42);

	// for the MEGA, set to 2MHz
	//SPI.setClockDivider(SPI_CLOCK_DIV8);

	//Set data mode
	SPI.setDataMode(SPI_MODE0);
#endif

	reset();

	uint8_t version = readRegister(REG_VERSION);

	if (version == 0x22) {
		// sx1272
		Serial.println(F("SX1272 detected, starting"));
		_board = SX1272Chip;
	} else if (version == 0x12) {
		// sx1276/78
		Serial.println(F("SX1276/78 detected, starting"));
		_board = SX1276Chip;
	} else {
		Serial.println(F("Unrecognized transceiver"));
	}

	RxChainCalibration();

	//setMaxCurrent(0x1B);
#if (SX1272_debug_mode > 1)
	Serial.println(F("## Setting ON with maximum current supply ##"));
	Serial.println();
#endif

	state = setLORA();	// enter in lora_stand_by mode

	setCRC_ON();

	// for ToA computation
	getPreambleLength();
	Serial.print("Preamble len is: ");
	Serial.println(_preamblelength + 4);
#ifdef W_NET_KEY
	//#if (SX1272_debug_mode > 1)
	Serial.println(F("## SX1272 layer has net key##"));
	//#endif
#endif

#ifdef LIMIT_TOA
	uint16_t remainingToA = limitToA();
	Serial.println(F("## Limit ToA ON ##"));
	Serial.print(F("cycle begins at "));
	Serial.print(_startToAcycle);
	Serial.print(F(" cycle ends at "));
	Serial.print(_endToAcycle);
	Serial.print(F(" remaining ToA is "));
	Serial.print(remainingToA);
	Serial.println();
#endif

	randomSeed(millis());	//init random generator

	return state;
}

void SX1272::OFF()
{
#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'OFF'"));
#endif

#ifdef USE_SOFTSPI
#else
	SPI.end();
#endif

#if (SX1272_debug_mode > 1)
	Serial.println(F("## Setting OFF ##"));
#endif
}

byte SX1272::readRegister(byte address)
{
	byte value = 0x00;

	digitalWrite(_SX1272_SS, LOW);

#ifdef USE_SOFTSPI
	value = spi_read_reg(address & 0x7f);
#else
	bitClear(address, 7);	// Bit 7 cleared to write in registers
	SPI.transfer(address);
	value = SPI.transfer(0x00);
#endif
	digitalWrite(_SX1272_SS, HIGH);

#if (SX1272_debug_mode > 1)
	Serial.print(F("## Reading:  ##\t"));
	Serial.print(F("Register "));
	Serial.print(address, HEX);
	Serial.print(F(":  "));
	Serial.print(value, HEX);
	Serial.println();
#endif

	return value;
}

void SX1272::writeRegister(byte address, byte data)
{
	digitalWrite(_SX1272_SS, LOW);

#ifdef USE_SOFTSPI
	spi_write_reg(address | 0x80, data);
#else
	bitSet(address, 7);	// Bit 7 set to read from registers
	SPI.transfer(address);
	SPI.transfer(data);
#endif
	digitalWrite(_SX1272_SS, HIGH);

#if (SX1272_debug_mode > 1)
	Serial.print(F("## Writing:  ##\t"));
	Serial.print(F("Register "));
	bitClear(address, 7);
	Serial.print(address, HEX);
	Serial.print(F(":  "));
	Serial.print(data, HEX);
	Serial.println();
#endif

}

/*
 * Function: Clears the interruption flags
*/
void SX1272::clearFlags()
{
	byte st0, stnew;

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	if (_modem == LORA) {	// LoRa mode
		stnew = LORA_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby mode to write in registers
		writeRegister(REG_IRQ_FLAGS, 0xFF);	// LoRa mode flags register

		if (st0 != stnew)
			writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
#if (SX1272_debug_mode > 1)
		Serial.println(F("## LoRa flags cleared ##"));
#endif

#ifdef ENABLE_FSK
	} else {		// FSK mode
		stnew = FSK_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby mode to write in registers

		writeRegister(REG_IRQ_FLAGS1, 0xFF);	// FSK mode flags1 register
		writeRegister(REG_IRQ_FLAGS2, 0xFF);	// FSK mode flags2 register

		if (st0 != stnew)
			writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
#if (SX1272_debug_mode > 1)
		Serial.println(F("## FSK flags cleared ##"));
#endif

#endif
	}
}

uint8_t SX1272::setLORA()
{
	uint8_t state = 2;
	uint8_t retry = 0;

	byte st0;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setLORA'"));
#endif

	do {
		writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);		// Sleep mode (mandatory to set LoRa mode)
		writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);	// LoRa sleep mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
		delay(50 + retry * 10);

		st0 = readRegister(REG_OP_MODE);

		retry++;

		delay(200);
	} while (st0 != LORA_STANDBY_MODE && retry < 8);

	if (st0 == LORA_STANDBY_MODE) {
		_modem = LORA;
		state = 0;
#if (SX1272_debug_mode > 1)
		Serial.println(F("## LoRa set with success ##"));
#endif
	} else {
		state = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** There has been an error while setting LoRa **"));
#endif
	}
	return state;
}

#ifdef ENABLE_FSK
uint8_t SX1272::setFSK()
{
	uint8_t state = 2;
	byte st0;
	byte config1;

	if (_board == SX1276Chip)
		Serial.println(F("Warning: FSK has not been tested on SX1276!"));

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setFSK'"));
#endif

	writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);	// Sleep mode (mandatory to change mode)
	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// FSK standby mode
	config1 = readRegister(REG_PACKET_CONFIG1);
	config1 = config1 & B01111101;	// clears bits 8 and 1 from REG_PACKET_CONFIG1
	config1 = config1 | B00000100;	// sets bit 2 from REG_PACKET_CONFIG1
	writeRegister(REG_PACKET_CONFIG1, config1);	// AddressFiltering = NodeAddress + BroadcastAddress
	writeRegister(REG_FIFO_THRESH, 0x80);	// condition to start packet tx
	config1 = readRegister(REG_SYNC_CONFIG);
	config1 = config1 & B00111111;
	writeRegister(REG_SYNC_CONFIG, config1);

	delay(100);

	st0 = readRegister(REG_OP_MODE);	// Reading config mode
	if (st0 == FSK_STANDBY_MODE) {	// FSK mode
		_modem = FSK;
		state = 0;
#if (SX1272_debug_mode > 1)
		Serial.println(F("## FSK set with success ##"));
#endif
	} else {		// LoRa mode
		_modem = LORA;
		state = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** There has been an error while setting FSK **"));
#endif
	}
	return state;
}
#endif

uint8_t SX1272::getMode()
{
	int8_t state = 2;
	byte value = 0x00;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getMode'"));
#endif

	if (_modem == FSK) {
		setLORA();
	}

	value = readRegister(REG_MODEM_CONFIG1);

	if (_board == SX1272Chip) {
		_bandwidth = (value >> 6);	// Storing 2 MSB from REG_MODEM_CONFIG1 (=_bandwidth)
		// added by C. Pham
		// convert to common bandwidth values used by both SX1272 and SX1276
		_bandwidth += 7;
	} else
		_bandwidth = (value >> 4);	// Storing 4 MSB from REG_MODEM_CONFIG1 (=_bandwidth)

	if (_board == SX1272Chip)
		_codingRate = (value >> 3) & 0x07;	// Storing third, forth and fifth bits from
	else
		_codingRate = (value >> 1) & 0x07;	// Storing 3-1 bits REG_MODEM_CONFIG1 (=_codingRate)

	value = readRegister(REG_MODEM_CONFIG2);
	_spreadingFactor = (value >> 4) & 0x0F;	// Storing 4 MSB from REG_MODEM_CONFIG2 (=_spreadingFactor)
	state = 1;

	if (isBW(_bandwidth))
	{
		if (isCR(_codingRate))
		{
			if (isSF(_spreadingFactor)) {
				state = 0;
			}
		}
	}
#if (SX1272_debug_mode > 1)
	Serial.println(F("## Parameters from configuration mode are:"));
	Serial.print(F("Bandwidth: "));
	Serial.print(_bandwidth, HEX);
	Serial.println();
	Serial.print(F("\t Coding Rate: "));
	Serial.print(_codingRate, HEX);
	Serial.println();
	Serial.print(F("\t Spreading Factor: "));
	Serial.print(_spreadingFactor, HEX);
	Serial.println(F(" ##"));
#endif

	delay(100);
	return state;
}

int8_t SX1272::setMode(uint8_t mode)
{
	int8_t state = 2;
	byte st0, stnew;
	byte config1 = 0x00;
	byte config2 = 0x00;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setMode'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	if (_modem == FSK) {
		setLORA();
	}

	stnew = LORA_STANDBY_MODE;
	if (st0 != stnew)
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode

	switch (mode) {
		// mode 1 (better reach, medium time on air)
	case 1:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_12);	// SF = 12
		setBW(BW_125);	// BW = 125 KHz
		break;

		// mode 2 (medium reach, less time on air)
	case 2:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_12);	// SF = 12
		setBW(BW_250);	// BW = 250 KHz
		break;

		// mode 3 (worst reach, less time on air)
	case 3:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_10);	// SF = 10
		setBW(BW_125);	// BW = 125 KHz
		break;

		// mode 4 (better reach, low time on air)
	case 4:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_12);	// SF = 12
		setBW(BW_500);	// BW = 500 KHz
		break;

		// mode 5 (better reach, medium time on air)
	case 5:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_10);	// SF = 10
		setBW(BW_250);	// BW = 250 KHz
		break;

		// mode 6 (better reach, worst time-on-air)
	case 6:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_11);	// SF = 11
		setBW(BW_500);	// BW = 500 KHz
		break;

		// mode 7 (medium-high reach, medium-low time-on-air)
	case 7:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_9);	// SF = 9
		setBW(BW_250);	// BW = 250 KHz
		break;

		// mode 8 (medium reach, medium time-on-air)
	case 8:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_9);	// SF = 9
		setBW(BW_500);	// BW = 500 KHz
		break;

		// mode 9 (medium-low reach, medium-high time-on-air)
	case 9:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_8);	// SF = 8
		setBW(BW_500);	// BW = 500 KHz
		break;

		// mode 10 (worst reach, less time_on_air)
	case 10:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_7);	// SF = 7
		setBW(BW_500);	// BW = 500 KHz
		break;

		// added by C. Pham
		// test for LoRaWAN channel
	case 11:
		setCR(CR_5);	// CR = 4/5
		setSF(SF_12);	// SF = 12
		setBW(BW_125);	// BW = 125 KHz
		// set the sync word to the LoRaWAN sync word which is 0x34
		setSyncWord(0x34);
		Serial.print(F("** Using sync word of 0x"));
		Serial.println(_syncWord, HEX);
		break;

	default:
		state = -1;	// The indicated mode doesn't exist
	};

	if (state == -1) {
#if (SX1272_debug_mode > 1)
		Serial.print(F("** The indicated mode doesn't exist, "));
		Serial.println(F("please select from 1 to 10 **"));
#endif
	} else {
		state = 1;
		config1 = readRegister(REG_MODEM_CONFIG1);

		switch (mode) {
			// (config1 >> 3) ---> take out bits 7-3 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
			// (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)

			// mode 1: BW = 125 KHz, CR = 4/5, SF = 12.
		case 1:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x01)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x39)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_12) {
					state = 0;
				}
			}
			break;

			// mode 2: BW = 250 KHz, CR = 4/5, SF = 12.
		case 2:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x09)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x41)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_12) {
					state = 0;
				}
			}
			break;

			// mode 3: BW = 125 KHz, CR = 4/5, SF = 10.
		case 3:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x01)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x39)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_10) {
					state = 0;
				}
			}
			break;

			// mode 4: BW = 500 KHz, CR = 4/5, SF = 12.
		case 4:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x11)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x49)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_12) {
					state = 0;
				}
			}
			break;

			// mode 5: BW = 250 KHz, CR = 4/5, SF = 10.
		case 5:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x09)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x41)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_10) {
					state = 0;
				}
			}
			break;

			// mode 6: BW = 500 KHz, CR = 4/5, SF = 11.
		case 6:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x11)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x49)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_11) {
					state = 0;
				}
			}
			break;

			// mode 7: BW = 250 KHz, CR = 4/5, SF = 9.
		case 7:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x09)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x41)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_9) {
					state = 0;
				}
			}
			break;

			// mode 8: BW = 500 KHz, CR = 4/5, SF = 9.
		case 8:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x11)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x49)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_9) {
					state = 0;
				}
			}
			break;

			// mode 9: BW = 500 KHz, CR = 4/5, SF = 8.
		case 9:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x11)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x49)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_8) {
					state = 0;
				}
			}
			break;

			// mode 10: BW = 500 KHz, CR = 4/5, SF = 7.
		case 10:

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x11)
					state = 0;
			} else {
				// (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
				if ((config1 >> 1) == 0x49)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_7) {
					state = 0;
				}
			}
			break;

		case 11:
			// mode 11: BW = 125 KHz, CR = 4/5, SF = 12.

			if (_board == SX1272Chip) {
				if ((config1 >> 3) == 0x01)
					state = 0;
			} else {
				if ((config1 >> 1) == 0x39)
					state = 0;
			}

			if (state == 0) {
				state = 1;
				config2 = readRegister(REG_MODEM_CONFIG2);

				if ((config2 >> 4) == SF_12) {
					state = 0;
				}
			}
			break;
		}

		if (mode != 11) {
			setSyncWord(_defaultSyncWord);
#if (SX1272_debug_mode > 1)
			Serial.print(F("** Using sync word of 0x"));
			Serial.println(_defaultSyncWord, HEX);
#endif
		}
	}

	if (state == 0)
		_loraMode = mode;

#if (SX1272_debug_mode > 1)
	if (state == 0) {
		Serial.print(F("Mode "));
		Serial.print(mode, DEC);
		Serial.println(F(" configured with success"));
	} else {
		Serial.
		    print(F
			  ("There has been an error while configuring mode: "));
		Serial.println(mode, DEC);
	}
#endif

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	delay(100);
	return state;
}

uint8_t SX1272::getHeader()
{
	int8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getHeader'"));
#endif

	// added by C. Pham
	uint8_t theHeaderBit;

	if (_board == SX1272Chip)
		theHeaderBit = 2;
	else
		theHeaderBit = 0;

	// take out bit 2 from REG_MODEM_CONFIG1 indicates ImplicitHeaderModeOn
	if (bitRead(REG_MODEM_CONFIG1, theHeaderBit) == 0) {
		_header = HEADER_ON;
		state = 1;
	} else {
		_header = HEADER_OFF;
		state = 1;
	}

	state = 0;

	if (_modem == FSK) {	// header is not available in FSK mode
#if (SX1272_debug_mode > 1)
		Serial.println(F("Notice that FSK mode packets hasn't header"));
#endif
	} else {		// header in LoRa mode
#if (SX1272_debug_mode > 1)
		Serial.print(F("Header is "));
		if (_header == HEADER_ON) {
			Serial.println(F("in explicit header mode"));
		} else {
			Serial.println(F("in implicit header mode"));
		}
#endif
	}
	return state;
}

int8_t SX1272::setHeaderON()
{
	int8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setHeaderON'"));
#endif

	if (_modem == FSK) {
		state = -1;	// header is not available in FSK mode
#if (SX1272_debug_mode > 1)
		Serial.println(F("FSK mode packets hasn't header"));
#endif
	} else {
		config1 = readRegister(REG_MODEM_CONFIG1);

		if (_spreadingFactor == 6) {
			// Mandatory headerOFF with SF = 6
			state = -1;
#if (SX1272_debug_mode > 1)
			Serial.println(F("Mandatory implicit header mode with spreading factor = 6"));
#endif
		} else {
			if (_board == SX1272Chip)
				config1 = config1 & B11111011;
			else
				config1 = config1 & B11111110;

			writeRegister(REG_MODEM_CONFIG1, config1);
		}

		uint8_t theHeaderBit;

		if (_board == SX1272Chip)
			theHeaderBit = 2;
		else
			theHeaderBit = 0;

		if (_spreadingFactor != 6) {
			config1 = readRegister(REG_MODEM_CONFIG1);

			if (bitRead(config1, theHeaderBit) == HEADER_ON) {
				state = 0;
				_header = HEADER_ON;
#if (SX1272_debug_mode > 1)
				Serial.println(F("Header has been activated"));
#endif
			} else {
				state = 1;
			}
		}
	}
	return state;
}

int8_t SX1272::setHeaderOFF()
{
	uint8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setHeaderOFF'"));
#endif

	if (_modem == FSK) {	// header is not available in FSK mode
		state = -1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("Notice that FSK mode packets hasn't header"));
#endif
	} else {
		config1 = readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the header bit

		if (_board == SX1272Chip)
			config1 = config1 | B00000100;	// sets bit 2 from REG_MODEM_CONFIG1 = headerOFF
		else
			config1 = config1 | B00000001;	// sets bit 0 from REG_MODEM_CONFIG1 = headerOFF

		writeRegister(REG_MODEM_CONFIG1, config1);	// Update config1

		config1 = readRegister(REG_MODEM_CONFIG1);

		// added by C. Pham
		uint8_t theHeaderBit;

		if (_board == SX1272Chip)
			theHeaderBit = 2;
		else
			theHeaderBit = 0;

		if (bitRead(config1, theHeaderBit) == HEADER_OFF) {	// checking headerOFF taking out bit 2 from REG_MODEM_CONFIG1
			state = 0;
			_header = HEADER_OFF;

#if (SX1272_debug_mode > 1)
			Serial.println(F("Header has been desactivated"));
#endif
		} else {
			state = 1;
#if (SX1272_debug_mode > 1)
			Serial.println(F("Header hasn't been desactivated"));
#endif
		}
	}
	return state;
}

uint8_t SX1272::getCRC()
{
	int8_t state = 2;
	byte value;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getCRC'"));
#endif

	if (_modem == LORA) {	// LoRa mode

		// added by C. Pham
		uint8_t theRegister;
		uint8_t theCrcBit;

		if (_board == SX1272Chip) {
			theRegister = REG_MODEM_CONFIG1;
			theCrcBit = 1;
		} else {
			theRegister = REG_MODEM_CONFIG2;
			theCrcBit = 2;
		}

		// take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
		value = readRegister(theRegister);
		if (bitRead(value, theCrcBit) == CRC_OFF) {
			_CRC = CRC_OFF;
#if (SX1272_debug_mode > 1)
			Serial.println(F("CRC is desactivated"));
#endif
			state = 0;
		} else {
			_CRC = CRC_ON;
#if (SX1272_debug_mode > 1)
			Serial.println(F("CRC is activated"));
#endif
			state = 0;
		}
#ifdef ENABLE_FSK
	} else {

		// take out bit 2 from REG_PACKET_CONFIG1 indicates CrcOn
		value = readRegister(REG_PACKET_CONFIG1);
		if (bitRead(value, 4) == CRC_OFF) {
			_CRC = CRC_OFF;
#if (SX1272_debug_mode > 1)
			Serial.println(F("CRC is desactivated"));
#endif
			state = 0;
		} else {	// CRCon
			_CRC = CRC_ON;
#if (SX1272_debug_mode > 1)
			Serial.println(F("CRC is activated"));
#endif
			state = 0;
		}
#endif
	}

	if (state != 0) {
		state = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("Setting CRC error"));
#endif
	}
	return state;
}

uint8_t SX1272::setCRC_ON()
{
	uint8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setCRC_ON'"));
#endif

	if (_modem == LORA) {

		uint8_t theRegister;
		uint8_t theCrcBit;

		if (_board == SX1272Chip) {
			theRegister = REG_MODEM_CONFIG1;
			theCrcBit = 1;
		} else {
			theRegister = REG_MODEM_CONFIG2;
			theCrcBit = 2;
		}

		config1 = readRegister(theRegister);

		if (_board == SX1272Chip)
			config1 = config1 | B00000010;
		else
			config1 = config1 | B00000100;

		writeRegister(theRegister, config1);

		state = 1;

		config1 = readRegister(theRegister);

		if (bitRead(config1, theCrcBit) == CRC_ON) {
			state = 0;
			_CRC = CRC_ON;
#if (SX1272_debug_mode > 1)
			Serial.println(F("CRC has been activated"));
#endif
		}
#ifdef ENABLE_FSK
	} else {		// FSK mode
		config1 = readRegister(REG_PACKET_CONFIG1);
		config1 = config1 | B00010000;
		writeRegister(REG_PACKET_CONFIG1, config1);

		state = 1;

		config1 = readRegister(REG_PACKET_CONFIG1);

		if (bitRead(config1, 4) == CRC_ON) {
			state = 0;
			_CRC = CRC_ON;
#if (SX1272_debug_mode > 1)
			Serial.println(F("CRC has been activated"));
#endif
		}
#endif
	}

	if (state != 0) {
		state = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("Setting CRC_ON error"));
#endif
	}
	return state;
}

uint8_t SX1272::setCRC_OFF()
{
	int8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setCRC_OFF'"));
#endif

	if (_modem == LORA) {	// LORA mode

		// added by C. Pham
		uint8_t theRegister;
		uint8_t theCrcBit;

		if (_board == SX1272Chip) {
			theRegister = REG_MODEM_CONFIG1;
			theCrcBit = 1;
		} else {
			theRegister = REG_MODEM_CONFIG2;
			theCrcBit = 2;
		}

		config1 = readRegister(theRegister);	// Save config1 to modify only the CRC bit
		if (_board == SX1272Chip)
			config1 = config1 & B11111101;	// clears bit 1 from config1 = CRC_OFF
		else
			config1 = config1 & B11111011;	// clears bit 2 from config1 = CRC_OFF

		writeRegister(theRegister, config1);

		config1 = readRegister(theRegister);
		if ((bitRead(config1, theCrcBit)) == CRC_OFF) {	// take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
			state = 0;
			_CRC = CRC_OFF;
#if (SX1272_debug_mode > 1)
			Serial.println(F("## CRC has been desactivated ##"));
			Serial.println();
#endif
		}
	} else {		// FSK mode
		config1 = readRegister(REG_PACKET_CONFIG1);	// Save config1 to modify only the CRC bit
		config1 = config1 & B11101111;	// clears bit 4 from config1 = CRC_OFF
		writeRegister(REG_PACKET_CONFIG1, config1);

		config1 = readRegister(REG_PACKET_CONFIG1);
		if (bitRead(config1, 4) == CRC_OFF) {	// take out bit 4 from REG_PACKET_CONFIG1 indicates RxPayloadCrcOn
			state = 0;
			_CRC = CRC_OFF;
#if (SX1272_debug_mode > 1)
			Serial.println(F("## CRC has been desactivated ##"));
			Serial.println();
#endif
		}
	}
	if (state != 0) {
		state = 1;
#if (SX1272_debug_mode > 1)
		Serial.
		    println(F
			    ("** There has been an error while setting CRC OFF **"));
		Serial.println();
#endif
	}
	return state;
}

boolean SX1272::isSF(uint8_t spr)
{
#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'isSF'"));
#endif

	// Checking available values for _spreadingFactor
	switch (spr) {
	case SF_6:
	case SF_7:
	case SF_8:
	case SF_9:
	case SF_10:
	case SF_11:
	case SF_12:
		return true;
		break;

	default:
		return false;
	}
#if (SX1272_debug_mode > 1)
	Serial.println(F("## Finished 'isSF' ##"));
	Serial.println();
#endif
}

int8_t SX1272::getSF()
{
	int8_t state = 2;
	byte config2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getSF'"));
#endif

	if (_modem == FSK) {
		state = -1;	// SF is not available in FSK mode
#if (SX1272_debug_mode > 1)
		Serial.println(F("** FSK mode hasn't spreading factor **"));
		Serial.println();
#endif
	} else {
		// take out bits 7-4 from REG_MODEM_CONFIG2 indicates _spreadingFactor
		config2 = (readRegister(REG_MODEM_CONFIG2)) >> 4;
		_spreadingFactor = config2;
		state = 1;

		if ((config2 == _spreadingFactor) && isSF(_spreadingFactor)) {
			state = 0;
#if (SX1272_debug_mode > 1)
			Serial.print(F("## Spreading factor is "));
			Serial.print(_spreadingFactor, HEX);
			Serial.println(F(" ##"));
			Serial.println();
#endif
		}
	}
	return state;
}

uint8_t SX1272::setSF(uint8_t spr)
{
	byte st0, stnew;
	int8_t state = 2;
	byte config1 = 0;
	byte config2 = 0;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setSF'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	if (_modem == FSK) {
#if (SX1272_debug_mode > 1)
		Serial.print(F("FSK hasn't Spreading Factor parameter, "));
		Serial.println(F("transfer to LoRa mode"));
#endif
		state = setLORA();	// Setting LoRa mode
	} else {		// LoRa mode
		stnew = LORA_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);

		config2 = (readRegister(REG_MODEM_CONFIG2));

		switch (spr) {
		case SF_6:
			config2 = config2 & B01101111;
			config2 = config2 | B01100000;
			setHeaderOFF();
			break;
		case SF_7:
			config2 = config2 & B01111111;
			config2 = config2 | B01110000;
			break;
		case SF_8:
			config2 = config2 & B10001111;
			config2 = config2 | B10000000;
			break;
		case SF_9:
			config2 = config2 & B10011111;
			config2 = config2 | B10010000;
			break;
		case SF_10:
			config2 = config2 & B10101111;
			config2 = config2 | B10100000;
			break;
		case SF_11:
			config2 = config2 & B10111111;
			config2 = config2 | B10110000;
			getBW();

			if (_bandwidth == BW_125) {
				// LowDataRateOptimize (Mandatory with SF_11 if BW_125)

				if (_board == SX1272Chip) {

					// Save config1 to modify only the LowDataRateOptimize
					config1 = (readRegister(REG_MODEM_CONFIG1));
					config1 = config1 | B00000001;
					writeRegister(REG_MODEM_CONFIG1, config1);

				} else {
					byte config3 = readRegister(REG_MODEM_CONFIG3);
					config3 = config3 | B00001000;
					writeRegister(REG_MODEM_CONFIG3, config3);
				}
			}
			break;
		case SF_12:
			config2 = config2 & B11001111;
			config2 = config2 | B11000000;

			if (_bandwidth == BW_125) {

				// LowDataRateOptimize (Mandatory with SF_12 if BW_125)
				if (_board == SX1272Chip) {
					config1 = (readRegister(REG_MODEM_CONFIG1));
					config1 = config1 | B00000001;
					writeRegister(REG_MODEM_CONFIG1,
						      config1);
				} else {
					byte config3 = readRegister(REG_MODEM_CONFIG3);
					config3 = config3 | B00001000;
					writeRegister(REG_MODEM_CONFIG3, config3);
				}
			}
			break;
		}

		// Check if it is neccesary to set special settings for SF=6
		if (spr == SF_6) {
			// Mandatory headerOFF with SF = 6 (Implicit mode)
			setHeaderOFF();

			// Set the bit field DetectionOptimize of
			// register RegLoRaDetectOptimize to value "0b101".
			writeRegister(REG_DETECT_OPTIMIZE, 0x05);

			// Write 0x0C in the register RegDetectionThreshold.
			writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
		} else {
			// added by C. Pham
			setHeaderON();

			// LoRa detection Optimize: 0x03 --> SF7 to SF12
			writeRegister(REG_DETECT_OPTIMIZE, 0x03);

			// LoRa detection threshold: 0x0A --> SF7 to SF12
			writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
		}

		// added by C. Pham
		if (_board == SX1272Chip) {
			// comment by C. Pham
			// bit 9:8 of SymbTimeout are then 11
			// single_chan_pkt_fwd uses 00 and then 00001000
			// why?
			// sets bit 2-0 (AgcAutoOn and SymbTimout) for any SF value
			//config2 = config2 | B00000111;
			// modified by C. Pham
			config2 = config2 | B00000100;
			writeRegister(REG_MODEM_CONFIG1, config1);	// Update config1
		} else {
			// set the AgcAutoOn in bit 2 of REG_MODEM_CONFIG3
			uint8_t config3 = (readRegister(REG_MODEM_CONFIG3));
			config3 = config3 | B00000100;
			writeRegister(REG_MODEM_CONFIG3, config3);
		}

		// here we write the new SF
		writeRegister(REG_MODEM_CONFIG2, config2);	// Update config2

		delay(100);

		// added by C. Pham
		byte configAgc;
		uint8_t theLDRBit;

		if (_board == SX1272Chip) {
			config1 = (readRegister(REG_MODEM_CONFIG1));
			config2 = (readRegister(REG_MODEM_CONFIG2));

			// (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)
			// bitRead(config1, 0) ---> take out bits 1 from config1 (=LowDataRateOptimize)
			// config2 is only for the AgcAutoOn
			configAgc = config2;
			theLDRBit = 0;
		} else {
			config1 = (readRegister(REG_MODEM_CONFIG3));
			config2 = (readRegister(REG_MODEM_CONFIG2));

			// LowDataRateOptimize is in REG_MODEM_CONFIG3
			// AgcAutoOn is in REG_MODEM_CONFIG3
			configAgc = config1;
			theLDRBit = 3;
		}

		switch (spr) {
		case SF_6:
			if (((config2 >> 4) == spr)
			    && (bitRead(configAgc, 2) == 1)
			    && (_header == HEADER_OFF)) {
				state = 0;
			}
			break;
		case SF_7:
			if (((config2 >> 4) == 0x07)
			    && (bitRead(configAgc, 2) == 1)) {
				state = 0;
			}
			break;
		case SF_8:
			if (((config2 >> 4) == 0x08)
			    && (bitRead(configAgc, 2) == 1)) {
				state = 0;
			}
			break;
		case SF_9:
			if (((config2 >> 4) == 0x09)
			    && (bitRead(configAgc, 2) == 1)) {
				state = 0;
			}
			break;
		case SF_10:
			if (((config2 >> 4) == 0x0A)
			    && (bitRead(configAgc, 2) == 1)) {
				state = 0;
			}
			break;
		case SF_11:
			if (((config2 >> 4) == 0x0B)
			    && (bitRead(configAgc, 2) == 1)
			    && (bitRead(config1, theLDRBit) == 1)) {
				state = 0;
			}
			break;
		case SF_12:
			if (((config2 >> 4) == 0x0C)
			    && (bitRead(configAgc, 2) == 1)
			    && (bitRead(config1, theLDRBit) == 1)) {
				state = 0;
			}
			break;
		default:
			state = 1;
		}
	}

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);	// Go back to previous status

	delay(100);

	if (isSF(spr)) {
		// Checking available value for _spreadingFactor
		state = 0;
		_spreadingFactor = spr;
#if (SX1272_debug_mode > 1)
		Serial.print(F("Spreading factor "));
		Serial.print(_spreadingFactor, DEC);
		Serial.println(F(" has been successfully set"));
#endif
	} else {
		if (state != 0) {
#if (SX1272_debug_mode > 1)
			Serial.print(F("Setting the spreading factor error"));
#endif
		}
	}
	return state;
}

boolean SX1272::isBW(uint16_t band)
{
#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'isBW'"));
#endif

	// Checking available values for _bandwidth
	// added by C. Pham
	if (_board == SX1272Chip) {
		switch (band) {
		case BW_125:
		case BW_250:
		case BW_500:
			return true;
			break;

		default:
			return false;
		}
	} else {
		switch (band) {
		case BW_7_8:
		case BW_10_4:
		case BW_15_6:
		case BW_20_8:
		case BW_31_25:
		case BW_41_7:
		case BW_62_5:
		case BW_125:
		case BW_250:
		case BW_500:
			return true;
			break;

		default:
			return false;
		}
	}

#if (SX1272_debug_mode > 1)
	Serial.println(F("## Finished 'isBW' ##"));
	Serial.println();
#endif
}

int8_t SX1272::getBW()
{
	uint8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getBW'"));
#endif

	if (_modem == FSK) {
		state = -1;	// BW is not available in FSK mode
#if (SX1272_debug_mode > 1)
		Serial.println(F("** FSK mode hasn't bandwidth **"));
		Serial.println();
#endif
	} else {
		// added by C. Pham
		if (_board == SX1272Chip) {
			// take out bits 7-6 from REG_MODEM_CONFIG1 indicates _bandwidth
			config1 = (readRegister(REG_MODEM_CONFIG1)) >> 6;
		} else {
			// take out bits 7-4 from REG_MODEM_CONFIG1 indicates _bandwidth
			config1 = (readRegister(REG_MODEM_CONFIG1)) >> 4;
		}

		_bandwidth = config1;

		if ((config1 == _bandwidth) && isBW(_bandwidth)) {
			state = 0;
#if (SX1272_debug_mode > 1)
			Serial.print(F("## Bandwidth is "));
			Serial.print(_bandwidth, HEX);
			Serial.println(F(" ##"));
			Serial.println();
#endif
		} else {
			state = 1;
#if (SX1272_debug_mode > 1)
			Serial.
			    print(F
				  ("** There has been an error while getting bandwidth **"));
			Serial.println();
#endif
		}
	}
	return state;
}

int8_t SX1272::setBW(uint16_t band)
{
	byte st0, stnew;
	int8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setBW'"));
#endif

	if (!isBW(band)) {
		state = 1;
#if (SX1272_debug_mode > 1)
		Serial.print(F("Bandwidth "));
		Serial.print(band, HEX);
		Serial.println(F(" is not a correct value"));
#endif
		return state;
	}

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	if (_modem == FSK) {
#if (SX1272_debug_mode > 1)
		Serial.print(F("## FSK hasn't Bandwidth parameter, "));
		Serial.println(F("transfter to LoRa mode ##"));
#endif
		state = setLORA();
	}

	stnew = LORA_STANDBY_MODE;
	if (st0 != stnew)
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);

	config1 = readRegister(REG_MODEM_CONFIG1);

	if (_board == SX1272Chip) {
		switch (band) {
		case BW_125:
			config1 = config1 & B00111111;
			getSF();
			if (_spreadingFactor == 11) {
				config1 = config1 | B00000001;
			}
			if (_spreadingFactor == 12) {
				config1 = config1 | B00000001;
			}
			break;
		case BW_250:
			config1 = config1 & B01111111;
			config1 = config1 | B01000000;
			break;
		case BW_500:
			config1 = config1 & B10111111;
			config1 = config1 | B10000000;
			break;
		}
	} else {
		// SX1276
		config1 = config1 & B00001111;
		switch (band) {
		case BW_125:
			// 0111
			config1 = config1 | B01110000;
			getSF();

			if (_spreadingFactor == 11 || _spreadingFactor == 12) {

				byte config3 = readRegister(REG_MODEM_CONFIG3);
				config3 = config3 | B00001000;
				writeRegister(REG_MODEM_CONFIG3, config3);
			}
			break;
		case BW_250:
			// 1000
			config1 = config1 | B10000000;
			break;
		case BW_500:
			// 1001
			config1 = config1 | B10010000;
			break;
		}
	}

	writeRegister(REG_MODEM_CONFIG1, config1);

	delay(100);

	config1 = (readRegister(REG_MODEM_CONFIG1));

	if (_board == SX1272Chip) {
		// (config1 >> 6) ---> take out bits 7-6 from REG_MODEM_CONFIG1 (=_bandwidth)
		switch (band) {
		case BW_125:
			if ((config1 >> 6) == SX1272_BW_125) {
				state = 0;
				if (_spreadingFactor == 11) {
					if (bitRead(config1, 0) == 1) {
						state = 0;
					} else {
						state = 1;
					}
				}
				if (_spreadingFactor == 12) {
					if (bitRead(config1, 0) == 1) {
						state = 0;
					} else {
						state = 1;
					}
				}
			}
			break;
		case BW_250:
			if ((config1 >> 6) == SX1272_BW_250) {
				state = 0;
			}
			break;
		case BW_500:
			if ((config1 >> 6) == SX1272_BW_500) {
				state = 0;
			}
			break;
		}
	} else {
		// (config1 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG1 (=_bandwidth)
		switch (band) {
		case BW_125:
			if ((config1 >> 4) == BW_125) {
				state = 0;

				byte config3 =
				    (readRegister(REG_MODEM_CONFIG3));

				if (_spreadingFactor == 11) {
					if (bitRead(config3, 3) == 1) {
						state = 0;
					} else {
						state = 1;
					}
				}
				if (_spreadingFactor == 12) {
					if (bitRead(config3, 3) == 1) {
						state = 0;
					} else {
						state = 1;
					}
				}
			}
			break;
		case BW_250:
			if ((config1 >> 4) == BW_250) {
				state = 0;
			}
			break;
		case BW_500:
			if ((config1 >> 4) == BW_500) {
				state = 0;
			}
			break;
		}
	}

	if (state == 0) {
		_bandwidth = band;
#if (SX1272_debug_mode > 1)
		Serial.print(F("Bandwidth "));
		Serial.print(band, HEX);
		Serial.println(F(" has been successfully set"));
#endif
	}

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);

	delay(100);
	return state;
}

boolean SX1272::isCR(uint8_t cod)
{
#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'isCR'"));
#endif

	// Checking available values for _codingRate
	switch (cod) {
	case CR_5:
	case CR_6:
	case CR_7:
	case CR_8:
		return true;
		break;

	default:
		return false;
	}
#if (SX1272_debug_mode > 1)
	Serial.println(F("## Finished 'isCR' ##"));
	Serial.println();
#endif
}

int8_t SX1272::getCR()
{
	int8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getCR'"));
#endif

	if (_modem == FSK) {
		state = -1;	// CR is not available in FSK mode
#if (SX1272_debug_mode > 1)
		Serial.println(F("** FSK mode hasn't coding rate **"));
		Serial.println();
#endif
	} else {
		// added by C. Pham
		if (_board == SX1272Chip) {
			// take out bits 7-3 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
			config1 = (readRegister(REG_MODEM_CONFIG1)) >> 3;
			config1 = config1 & B00000111;	// clears bits 7-3 ---> clears _bandwidth
		} else {
			// take out bits 7-1 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
			config1 = (readRegister(REG_MODEM_CONFIG1)) >> 1;
			config1 = config1 & B00000111;	// clears bits 7-3 ---> clears _bandwidth
		}

		_codingRate = config1;
		state = 1;

		if ((config1 == _codingRate) && isCR(_codingRate)) {
			state = 0;
#if (SX1272_debug_mode > 1)
			Serial.print(F("## Coding rate is "));
			Serial.print(_codingRate, HEX);
			Serial.println(F(" ##"));
			Serial.println();
#endif
		}
	}
	return state;
}

int8_t SX1272::setCR(uint8_t cod)
{
	byte st0, stnew;
	int8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setCR'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	if (_modem == FSK) {
#if (SX1272_debug_mode > 1)
		Serial.print(F("## FSK hasn't Coding Rate parameter, "));
		Serial.println(F("transfer to LoRa mode ##"));
#endif
		state = setLORA();
	}

	stnew = LORA_STANDBY_MODE;
	if (st0 != stnew)
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);

	config1 = readRegister(REG_MODEM_CONFIG1);

	if (_board == SX1272Chip) {
		switch (cod) {
		case CR_5:
			config1 = config1 & B11001111;
			config1 = config1 | B00001000;
			break;
		case CR_6:
			config1 = config1 & B11010111;
			config1 = config1 | B00010000;
			break;
		case CR_7:
			config1 = config1 & B11011111;
			config1 = config1 | B00011000;
			break;
		case CR_8:
			config1 = config1 & B11100111;
			config1 = config1 | B00100000;
			break;
		}
	} else { // SX1276
		config1 = config1 & B11110001;

		switch (cod) {
		case CR_5:
			config1 = config1 | B00000010;
			break;
		case CR_6:
			config1 = config1 | B00000100;
			break;
		case CR_7:
			config1 = config1 | B00000110;
			break;
		case CR_8:
			config1 = config1 | B00001000;
			break;
		}
	}
	writeRegister(REG_MODEM_CONFIG1, config1);	// Update config1

	delay(100);

	config1 = readRegister(REG_MODEM_CONFIG1);

	uint8_t nshift = 3;

	// only 1 right shift for SX1276
	if (_board == SX1276Chip)
		nshift = 1;

	// ((config1 >> 3) & B0000111) ---> take out bits 5-3 from REG_MODEM_CONFIG1 (=_codingRate)
	switch (cod) {
	case CR_5:
		if (((config1 >> nshift) & B0000111) == 0x01) {
			state = 0;
		}
		break;
	case CR_6:
		if (((config1 >> nshift) & B0000111) == 0x02) {
			state = 0;
		}
		break;
	case CR_7:
		if (((config1 >> nshift) & B0000111) == 0x03) {
			state = 0;
		}
		break;
	case CR_8:
		if (((config1 >> nshift) & B0000111) == 0x04) {
			state = 0;
		}
		break;
	}

	if (isCR(cod)) {
		_codingRate = cod;
#if (SX1272_debug_mode > 1)
		Serial.print(F("Coding Rate "));
		Serial.print(cod, HEX);
		Serial.println(F(" has been successfully set"));
#endif
	} else {
		state = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("Setting Coding Rate error!"));
#endif
	}
	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	delay(100);
	return state;
}

boolean SX1272::isChannel(uint32_t ch)
{
#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'isChannel'"));
#endif

	// Checking available values for _channel
	switch (ch) {
#if 0
	case CH_04_868:
	case CH_05_868:
	case CH_06_868:
	case CH_07_868:
	case CH_08_868:
	case CH_09_868:
	case CH_10_868:
	case CH_11_868:
	case CH_12_868:
	case CH_13_868:
	case CH_14_868:
	case CH_15_868:
	case CH_16_868:
	case CH_17_868:
	case CH_18_868:
	case CH_00_900:
	case CH_01_900:
	case CH_02_900:
	case CH_03_900:
	case CH_04_900:
	case CH_05_900:
	case CH_06_900:
	case CH_07_900:
	case CH_08_900:
	case CH_09_900:
	case CH_10_900:
	case CH_11_900:
	case CH_12_900:
#endif
	case CH_00_433:
	case CH_01_433:
	case CH_02_433:
	case CH_03_433:
		return true;
		break;

	default:
		return false;
	}
#if (SX1272_debug_mode > 1)
	Serial.println(F("## Finished 'isChannel' ##"));
	Serial.println();
#endif
}

/*
 Function: Indicates the frequency channel within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getChannel()
{
	uint8_t state = 2;
	uint32_t ch;
	uint8_t freq3;
	uint8_t freq2;
	uint8_t freq1;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getChannel'"));
#endif

	freq3 = readRegister(REG_FRF_MSB);	// frequency channel MSB
	freq2 = readRegister(REG_FRF_MID);	// frequency channel MID
	freq1 = readRegister(REG_FRF_LSB);	// frequency channel LSB
	ch = ((uint32_t) freq3 << 16) + ((uint32_t) freq2 << 8) +
	    (uint32_t) freq1;
	_channel = ch;		// frequency channel

	if ((_channel == ch) && isChannel(_channel)) {
		state = 0;
#if (SX1272_debug_mode > 1)
		Serial.print(F("## Frequency channel is "));
		Serial.print(_channel, HEX);
		Serial.println(F(" ##"));
		Serial.println();
#endif
	} else {
		state = 1;
	}
	return state;
}

int8_t SX1272::setChannel(uint32_t ch)
{
	byte st0, stnew;
	int8_t state = 2;
	unsigned int freq3;
	unsigned int freq2;
	uint8_t freq1;
	uint32_t freq;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setChannel'"));
#endif

	_starttime = millis();

	st0 = readRegister(REG_OP_MODE);

	if (_modem == LORA) {
		// LoRa Stdby mode to write in registers
		stnew = LORA_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
#ifdef ENABLE_FSK
	} else {
		// FSK Stdby mode to write in registers
		stnew = FSK_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
#endif
	}

	freq3 = ((ch >> 16) & 0x0FF);	// frequency channel MSB
	freq2 = ((ch >> 8) & 0x0FF);	// frequency channel MIB
	freq1 = (ch & 0xFF);			// frequency channel LSB

	writeRegister(REG_FRF_MSB, freq3);
	writeRegister(REG_FRF_MID, freq2);
	writeRegister(REG_FRF_LSB, freq1);

	_stoptime = millis();

	delay(100);

	// storing MSB in freq channel value
	freq3 = (readRegister(REG_FRF_MSB));
	freq = (freq3 << 8) & 0xFFFFFF;

	// storing MID in freq channel value
	freq2 = (readRegister(REG_FRF_MID));
	freq = (freq << 8) + ((freq2 << 8) & 0xFFFFFF);

	// storing LSB in freq channel value
	freq = freq + ((readRegister(REG_FRF_LSB)) & 0xFFFFFF);

	if (freq == ch) {
		state = 0;
		_channel = ch;
#if (SX1272_debug_mode > 1)
		Serial.print(F("## Frequency channel "));
		Serial.print(ch, HEX);
		Serial.println(F(" has been successfully set ##"));
#endif
	} else {
		state = 1;
	}

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	delay(100);
	return state;
}

uint8_t SX1272::getPower()
{
	uint8_t state = 2;
	byte value = 0x00;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getPower'"));
#endif

	value = readRegister(REG_PA_CONFIG);
	state = 1;

	// modified by C. Pham
	// get only the OutputPower
	_power = value & B00001111;

	//if( (value > -1) & (value < 16) )
	if (_power < 16) {
		state = 0;
#if (SX1272_debug_mode > 1)
		Serial.print(F("Output power is "));
		Serial.println(_power, HEX);
#endif
	}

	return state;
}

int8_t SX1272::setPower(char p)
{
	byte st0, stnew;
	int8_t state = 2;
	byte value = 0x00;

	byte RegPaDacReg = (_board == SX1272Chip) ? 0x5A : 0x4D;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setPower'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	if (_modem == LORA) {	// LoRa Stdby mode to write in registers
		stnew = LORA_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
#ifdef ENABLE_FSK
	} else {		// FSK Stdby mode to write in registers
		stnew = FSK_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
#endif
	}

	switch (p) {
		// L = Low. On SX1272/76: PA0 on RFO setting
		// H = High. On SX1272/76: PA0 on RFO setting
		// M = MAX. On SX1272/76: PA0 on RFO setting

		// x = extreme; added by C. Pham. On SX1272/76: PA1&PA2 PA_BOOST setting
		// X = eXtreme; added by C. Pham. On SX1272/76: PA1&PA2 PA_BOOST setting + 20dBm settings

		// added by C. Pham
		//
	case 'x':
	case 'X':
	case 'M':
		value = 0x0F;
		// SX1272/76: 14dBm
		break;

		// modified by C. Pham, set to 0x03 instead of 0x00
	case 'L':
		value = 0x03;
		// SX1272/76: 2dBm
		break;

	case 'H':
		value = 0x07;
		// SX1272/76: 6dBm
		break;

	default:
		state = -1;
		break;
	}

	if (p == 'x') {
		// we set only the PA_BOOST pin
		// limit to 14dBm
		value = 0x0C;
		value = value | B10000000;
		// set RegOcp for OcpOn and OcpTrim
		// 130mA
		setMaxCurrent(0x10);

	} else if (p == 'X') {
		// normally value = 0x0F;
		// we set the PA_BOOST pin
		value = value | B10000000;
		// and then set the high output power config with register REG_PA_DAC
		writeRegister(RegPaDacReg, 0x87);	// +20dBm on PA_BOOST
		// set RegOcp for OcpOn and OcpTrim

		setMaxCurrent(0x12);	// 150mA

	} else {
		// disable high power output in all other cases
		writeRegister(RegPaDacReg, 0x84);

		setMaxCurrent(0x0B);	// 100mA

	}

	// added by C. Pham
	if (_board == SX1272Chip) {
		// Pout = -1 + _power[3:0] on RFO
		// Pout = 2 + _power[3:0] on PA_BOOST
		// so: L=2dBm; H=6dBm, M=14dBm, x=14dBm (PA), X=20dBm(PA+PADAC)
		writeRegister(REG_PA_CONFIG, value);	// Setting output power value
	} else {
		// for the SX1276

		// set MaxPower to 7 -> Pmax=10.8+0.6*MaxPower [dBm] = 15
		value = value | B01110000;

		// then Pout = Pmax-(15-_power[3:0]) if  PaSelect=0 (RFO pin for +14dBm)
		// so L=3dBm; H=7dBm; M=15dBm (but should be limited to 14dBm by RFO pin)

		// and Pout = 17-(15-_power[3:0]) if  PaSelect=1 (PA_BOOST pin for +14dBm)
		// so x= 14dBm (PA);
		// when p=='X' for 20dBm, value is 0x0F and RegPaDacReg=0x87 so 20dBm is enabled

		writeRegister(REG_PA_CONFIG, value);
	}

	_power = value;

	value = readRegister(REG_PA_CONFIG);

	if (value == _power) {
		state = 0;
#if (SX1272_debug_mode > 1)
		Serial.println(F("Output power has been successfully set"));
#endif
	} else {
		state = 1;
	}

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	delay(100);
	return state;
}

int8_t SX1272::setPowerNum(uint8_t pow)
{
	byte st0, stnew;
	int8_t state = 2;
	byte value = 0x00;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setPower'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	if (_modem == LORA) {	// LoRa Stdby mode to write in registers
		stnew = LORA_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
#ifdef ENABLE_FSK
	} else {		// FSK Stdby mode to write in registers
		stnew = FSK_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
#endif
	}

	if ((pow >= 0) && (pow < 15)) {
		_power = pow;
	} else {
		state = -1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("Power value is not valid"));
#endif
	}

	// added by C. Pham
	if (_board == SX1276Chip) {
		value = readRegister(REG_PA_CONFIG);
		// clear OutputPower, but keep current value of PaSelect and MaxPower
		value = value & B11110000;
		value = value + _power;
		_power = value;
	}
	writeRegister(REG_PA_CONFIG, _power);	// Setting output power value
	value = readRegister(REG_PA_CONFIG);

	if (value == _power) {
		state = 0;
#if (SX1272_debug_mode > 1)
		Serial.println(F("Output power has been successfully set"));
#endif
	} else {
		state = 1;
	}

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	delay(100);
	return state;
}

uint8_t SX1272::getPreambleLength()
{
	int8_t state = 2;
	uint8_t p_length;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getPreambleLength'"));
#endif

	state = 1;
	if (_modem == LORA) {	// LORA mode
		p_length = readRegister(REG_PREAMBLE_MSB_LORA);
		// Saving MSB preamble length in LoRa mode
		_preamblelength = (p_length << 8) & 0xFFFF;
		p_length = readRegister(REG_PREAMBLE_LSB_LORA);
		// Saving LSB preamble length in LoRa mode
		_preamblelength = _preamblelength + (p_length & 0xFFFF);
#if (SX1272_debug_mode > 1)
		Serial.print(F("Preamble length configured is "));
		Serial.println(_preamblelength, HEX);
#endif
#ifdef ENABLE_FSK
	} else {		// FSK mode
		p_length = readRegister(REG_PREAMBLE_MSB_FSK);
		// Saving MSB preamble length in FSK mode
		_preamblelength = (p_length << 8) & 0xFFFF;
		p_length = readRegister(REG_PREAMBLE_LSB_FSK);
		// Saving LSB preamble length in FSK mode
		_preamblelength = _preamblelength + (p_length & 0xFFFF);
#if (SX1272_debug_mode > 1)
		Serial.print(F("Preamble length configured is "));
		Serial.println(_preamblelength, HEX);
#endif
#endif
	}
	state = 0;
	return state;
}

uint8_t SX1272::setPreambleLength(uint16_t l)
{
	byte st0, stnew;
	uint8_t p_length;
	int8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setPreambleLength'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	state = 1;
	if (_modem == LORA) {	// LoRa mode

		stnew = LORA_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Set Standby mode to write in registers
		p_length = ((l >> 8) & 0x0FF);
		// Storing MSB preamble length in LoRa mode
		writeRegister(REG_PREAMBLE_MSB_LORA, p_length);
		p_length = (l & 0x0FF);
		// Storing LSB preamble length in LoRa mode
		writeRegister(REG_PREAMBLE_LSB_LORA, p_length);
#ifdef ENABLE_FSK
	} else {		// FSK mode
		stnew = FSK_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Set Standby mode to write in registers
		p_length = ((l >> 8) & 0x0FF);
		// Storing MSB preamble length in FSK mode
		writeRegister(REG_PREAMBLE_MSB_FSK, p_length);
		p_length = (l & 0x0FF);
		// Storing LSB preamble length in FSK mode
		writeRegister(REG_PREAMBLE_LSB_FSK, p_length);
#endif
	}

	state = 0;
#if (SX1272_debug_mode > 1)
	Serial.print(F("Preamble length "));
	Serial.print(l, HEX);
	Serial.println(F(" has been successfully set"));
#endif

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	delay(100);
	return state;
}

uint8_t SX1272::getPayloadLength()
{
	return _payloadlength;
}

int8_t SX1272::setPacketLength()
{
	uint16_t length;

	// added by C. Pham
	// if gateway is in rawFormat mode for packet reception, it will also send in rawFormat
	// unless we switch it back to normal format just for transmission, e.g. for downlink transmission
	if (_rawFormat)
		length = _payloadlength;
	else
		length = _payloadlength + OFFSET_PAYLOADLENGTH;

	return setPacketLength(length);
}

int8_t SX1272::setPacketLength(uint8_t l)
{
	byte st0;
	byte value = 0x00;
	int8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setPacketLength'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	packet_sent.length = l;

	if (_modem == LORA) {

		if (st0 != LORA_STANDBY_MODE) {
			// Set LoRa Standby mode to write in registers
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
		}

		writeRegister(REG_PAYLOAD_LENGTH_LORA, packet_sent.length);	// Storing payload length in LoRa mode
		value = readRegister(REG_PAYLOAD_LENGTH_LORA);
#ifdef ENABLE_FSK
	} else {
		// FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	//  Set FSK Standby mode to write in registers
		writeRegister(REG_PAYLOAD_LENGTH_FSK, packet_sent.length);
		// Storing payload length in FSK mode
		value = readRegister(REG_PAYLOAD_LENGTH_FSK);
#endif
	}

	if (packet_sent.length == value) {
		state = 0;
#if (SX1272_debug_mode > 1)
		Serial.print(F("Packet length "));
		Serial.print(packet_sent.length, DEC);
		Serial.println(F(" has been successfully set"));
#endif
	} else {
		state = 1;
	}

	if (st0 != LORA_STANDBY_MODE) {
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	}

	// comment by C. Pham
	// this delay is included in the send delay overhead
	// TODO: do we really need this delay?
	delay(250);
	return state;
}

uint8_t SX1272::getNodeAddress()
{
	byte st0 = 0;
	uint8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getNodeAddress'"));
#endif

	if (_modem == LORA) {
		st0 = readRegister(REG_OP_MODE);

		// Allowing access to FSK registers while in LoRa standby mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
	}

	_nodeAddress = readRegister(REG_NODE_ADRS);
	state = 1;

	if (_modem == LORA) {
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	}

	state = 0;
#if (SX1272_debug_mode > 1)
	Serial.print(F("Node address configured is "));
	Serial.println(_nodeAddress);
#endif
	return state;
}

int8_t SX1272::setNodeAddress(uint8_t addr)
{
	byte st0, stnew;
	byte value;
	uint8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setNodeAddress'"));
#endif

	if (addr > 255) {
		state = -1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("Node address must be less than 255"));
#endif
	} else {
		// Saving node address
		_nodeAddress = addr;
		st0 = readRegister(REG_OP_MODE);	// Save the previous status

		if (_modem == LORA) {
			stnew = LORA_STANDBY_FSK_REGS_MODE;

			// Allowing access to FSK registers while in LoRa standby mode
			if (st0 != stnew)
				writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
#ifdef ENABLE_FSK
		} else {
			stnew = FSK_STANDBY_MODE;

			//Set FSK Standby mode to write in registers
			if (st0 != stnew)
				writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
#endif
		}

		// Storing node and broadcast address
		writeRegister(REG_NODE_ADRS, addr);
		writeRegister(REG_BROADCAST_ADRS, BROADCAST_0);

		value = readRegister(REG_NODE_ADRS);

		if (st0 != stnew)
			writeRegister(REG_OP_MODE, st0);	// Getting back to previous status

		if (value == _nodeAddress) {
			state = 0;
#if (SX1272_debug_mode > 1)
			Serial.print(F("Node address "));
			Serial.print(addr);
			Serial.println(F(" has been successfully set"));
#endif
		} else {
			state = 1;
#if (SX1272_debug_mode > 1)
			Serial.println(F("There has been an error while setting address"));
#endif
		}
	}
	return state;
}

int8_t SX1272::getSNR()
{
	int8_t state = 2;
	byte value;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getSNR'"));
#endif

	if (_modem == LORA) {	// LoRa mode
		state = 1;
		value = readRegister(REG_PKT_SNR_VALUE);
		_rawSNR = value;

		if (value & 0x80) {
			// Invert and divide by 4
			value = ((~value + 1) & 0xFF) >> 2;
			_SNR = -value;
		} else {
			// Divide by 4
			_SNR = (value & 0xFF) >> 2;
		}
		state = 0;
#if (SX1272_debug_mode > 0)
		Serial.print(F("SNR value is "));
		Serial.println(_SNR, DEC);
#endif
	} else {		// forbidden command if FSK mode
		state = -1;
#if (SX1272_debug_mode > 0)
		Serial.println(F("** SNR does not exist in FSK mode **"));
#endif
	}
	return state;
}

uint8_t SX1272::getRSSI()
{
	uint8_t state = 2;
	int rssi_mean = 0;
	int total = 5;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getRSSI'"));
#endif

	if (_modem == LORA) {
		// get mean value of RSSI

		for (int i = 0; i < total; i++) {
			// with SX1276 we have to add 18 to OFFSET_RSSI to obtain -157
			_RSSI =
			    -(OFFSET_RSSI + (_board == SX1276Chip ? 18 : 0)) +
			    readRegister(REG_RSSI_VALUE_LORA);
			rssi_mean += _RSSI;
		}

		rssi_mean = rssi_mean / total;
		_RSSI = rssi_mean;

		state = 0;
#if (SX1272_debug_mode > 0)
		Serial.print(F("RSSI value is "));
		Serial.println(_RSSI, DEC);
#endif

#ifdef ENABLE_FSK
	} else {
		/// FSK mode
		// get mean value of RSSI
		for (int i = 0; i < total; i++) {
			_RSSI = (readRegister(REG_RSSI_VALUE_FSK) >> 1);
			rssi_mean += _RSSI;
		}
		rssi_mean = rssi_mean / total;
		_RSSI = rssi_mean;

		state = 0;

#if (SX1272_debug_mode > 0)
		Serial.print(F("RSSI value is "));
		Serial.println(_RSSI);
#endif
#endif
	}
	return state;
}

int16_t SX1272::getRSSIpacket()
{
	int8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getRSSIpacket'"));
#endif

	state = 1;
	if (_modem == LORA) {

		state = getSNR();

		if (state == 0) {

			_RSSIpacket = readRegister(REG_PKT_RSSI_VALUE);

			if (_SNR < 0) {
				// commented by C. Pham
				//_RSSIpacket = -NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[_bandwidth] + NOISE_FIGURE + ( double )_SNR;

				// added by C. Pham, using Semtech SX1272 rev3 March 2015
				// for SX1272 we use -139, for SX1276, we use -157
				// then for SX1276 when using low-frequency (i.e. 433MHz) then we use -164
				_RSSIpacket = -(OFFSET_RSSI + (_board == SX1276Chip ? 18 : 0)
					+ (_channel < CH_04_868 ? 7 : 0))
					+ (double)_RSSIpacket + (double)_SNR *0.25;
				state = 0;
			} else {
				//_RSSIpacket = readRegister(REG_PKT_RSSI_VALUE);
				_RSSIpacket = -(OFFSET_RSSI + (_board == SX1276Chip ? 18 : 0)
					+ (_channel < CH_04_868 ? 7 : 0))
					+ (double)_RSSIpacket *16.0 / 15.0;
				state = 0;
			}
#if (SX1272_debug_mode > 0)
			Serial.print(F("RSSI packet value is "));
			Serial.println(_RSSIpacket, DEC);
#endif
		}
	} else {
		state = -1;
#if (SX1272_debug_mode > 0)
		Serial.println(F("RSSI packet does not exist in FSK mode"));
#endif
	}
	return state;
}

uint8_t SX1272::getMaxCurrent()
{
	int8_t state = 2;
	byte value;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getMaxCurrent'"));
#endif

	state = 1;
	_maxCurrent = readRegister(REG_OCP);

	// extract only the OcpTrim value from the OCP register
	_maxCurrent &= B00011111;

	if (_maxCurrent <= 15) {
		value = (45 + (5 * _maxCurrent));
	} else if (_maxCurrent <= 27) {
		value = (-30 + (10 * _maxCurrent));
	} else {
		value = 240;
	}

	_maxCurrent = value;
#if (SX1272_debug_mode > 1)
	Serial.print(F("Maximum current supply configured is "));
	Serial.print(value, DEC);
	Serial.println(F(" mA"));
#endif
	state = 0;
	return state;
}

int8_t SX1272::setMaxCurrent(uint8_t rate)
{
	int8_t state = 2;
	byte st0, stnew;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setMaxCurrent'"));
#endif

	// Maximum rate value = 0x1B, because maximum current supply = 240 mA
	if (rate > 0x1B) {
		state = -1;
#if (SX1272_debug_mode > 1)
		Serial.print(F("Maximum current supply is 240 mA, "));
		Serial.println(F("so max current must be 0x1B"));
#endif
	} else {
		// Enable Over Current Protection
		rate |= B00100000;

		state = 1;
		st0 = readRegister(REG_OP_MODE);	// Save the previous status
		if (_modem == LORA) {
			stnew = LORA_STANDBY_MODE;
			if (st0 != stnew)
				writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
#ifdef ENABLE_FSK
		} else {
			stnew = FSK_STANDBY_MODE;
			if (st0 != stnew)
				writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
#endif
		}

		writeRegister(REG_OCP, rate);	// Modifying maximum current supply

		if (st0 != stnew)
			writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
		state = 0;
	}
	return state;
}

uint8_t SX1272::getRegs()
{
	int8_t state = 2;
	uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getRegs'"));
#endif

	state_f = 1;
	state = getMode();	// Stores the BW, CR and SF.
	if (state == 0) {
		state = getPower();	// Stores the power.
	} else {
		state_f = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting mode **"));
#endif
	}
	if (state == 0) {
		state = getChannel();	// Stores the channel.
	} else {
		state_f = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting power **"));
#endif
	}
	if (state == 0) {
		state = getCRC();	// Stores the CRC configuration.
	} else {
		state_f = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting channel **"));
#endif
	}
	if (state == 0) {
		state = getHeader();	// Stores the header configuration.
	} else {
		state_f = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting CRC **"));
#endif
	}
	if (state == 0) {
		state = getPreambleLength();	// Stores the preamble length.
	} else {
		state_f = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting header **"));
#endif
	}
	if (state == 0) {
		state = getPayloadLength();	// Stores the payload length.
	} else {
		state_f = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting preamble length **"));
#endif
	}
	if (state == 0) {
		state = getNodeAddress();	// Stores the node address.
	} else {
		state_f = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting payload length **"));
#endif
	}
	if (state == 0) {
		state = getMaxCurrent();	// Stores the maximum current supply.
	} else {
		state_f = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting node address **"));
#endif
	}
	if (state == 0) {
		state_f = getTemp();	// Stores the module temperature.
	} else {
		state_f = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting maximum current supply **"));
#endif
	}
	if (state_f != 0) {
#if (SX1272_debug_mode > 1)
		Serial.println(F("** Error getting temperature **"));
		Serial.println();
#endif
	}
	return state_f;
}

uint8_t SX1272::truncPayload(uint16_t length16)
{
	uint8_t state = 2;

	state = 1;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'truncPayload'"));
#endif

	if (length16 > MAX_PAYLOAD) {
		_payloadlength = MAX_PAYLOAD;
	} else {
		_payloadlength = (length16 & 0xFF);
	}
	state = 0;

	return state;
}

uint8_t SX1272::setACK()
{
	uint8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'setACK'"));
#endif

	// added by C. Pham
	// check for enough remaining ToA
	// when operating under duty-cycle mode
	if (_limitToA) {
		if (getRemainingToA() - getToA(ACK_LENGTH) < 0) {
			Serial.print(F("## not enough ToA for ACK at"));
			Serial.println(millis());
			return SX1272_ERROR_TOA;
		}
	}
	// delay(1000);

	clearFlags();		// Initializing flags

	if (_modem == LORA) {	// LoRa mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
#ifdef ENABLE_FSK
	} else {		// FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
#endif
	}

	// Setting ACK length in order to send it
	state = setPacketLength(ACK_LENGTH);
	if (state == 0) {
		// Setting ACK
		ACK.dst = packet_received.src;	// ACK destination is packet source
		ACK.type = PKT_TYPE_ACK;
		ACK.src = packet_received.dst;	// ACK source is packet destination
		ACK.packnum = packet_received.packnum;	// packet number that has been correctly received
		ACK.length = 2;
		ACK.data[0] = _reception;	// CRC of the received packet
		// added by C. Pham
		// store the SNR
		ACK.data[1] = readRegister(REG_PKT_SNR_VALUE);

		// Setting address pointer in FIFO data buffer
		writeRegister(REG_FIFO_ADDR_PTR, 0x80);

		state = 1;

		// Writing ACK to send in FIFO
		writeRegister(REG_FIFO, ACK.dst);	// Writing the destination in FIFO
		writeRegister(REG_FIFO, ACK.type);
		writeRegister(REG_FIFO, ACK.src);	// Writing the source in FIFO
		writeRegister(REG_FIFO, ACK.packnum);	// Writing the packet number in FIFO
		writeRegister(REG_FIFO, ACK.length);	// Writing the packet length in FIFO
		writeRegister(REG_FIFO, ACK.data[0]);	// Writing the ACK in FIFO
		writeRegister(REG_FIFO, ACK.data[1]);	// Writing the ACK in FIFO

		//#if (SX1272_debug_mode > 0)
		Serial.println(F("## ACK set and written in FIFO ##"));
		// Print the complete ACK if debug_mode
		Serial.println(F("## ACK to send:"));
		Serial.print(F("Destination: "));
		Serial.println(ACK.dst);	// Printing destination
		Serial.print(F("Source: "));
		Serial.println(ACK.src);	// Printing source
		Serial.print(F("ACK number: "));
		Serial.println(ACK.packnum);	// Printing ACK number
		Serial.print(F("ACK length: "));
		Serial.println(ACK.length);	// Printing ACK length
		Serial.print(F("ACK payload: "));
		Serial.println(ACK.data[0]);	// Printing ACK payload
		Serial.print(F("ACK SNR last rcv pkt: "));
		Serial.println(_SNR);
		Serial.println(F("##"));
		//#endif

		state = 0;
		_reception = CORRECT_PACKET;	// Updating value to next packet

		// comment by C. Pham
		// TODO: do we really need this delay?
		delay(500);
	}
	return state;
}

uint8_t SX1272::receive()
{
	uint8_t state = 1;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'receive'"));
#endif

	// Initializing packet_received struct
	memset(&packet_received, 0x00, sizeof(packet_received));
	packet_received.data = packet_data;

	// Setting Testmode
	// commented by C. Pham
	//writeRegister(0x31,0x43);

	// Set LowPnTxPllOff
	// modified by C. Pham from 0x09 to 0x08
	writeRegister(REG_PA_RAMP, 0x08);

	//writeRegister(REG_LNA, 0x23);                     // Important in reception
	// modified by C. Pham
	writeRegister(REG_LNA, LNA_MAX_GAIN);
	writeRegister(REG_FIFO_ADDR_PTR, 0x00);	// Setting address pointer in FIFO data buffer
	// change RegSymbTimeoutLsb
	// comment by C. Pham
	// single_chan_pkt_fwd uses 00 00001000
	// why here we have 11 11111111
	// change RegSymbTimeoutLsb
	//writeRegister(REG_SYMB_TIMEOUT_LSB, 0xFF);

	// modified by C. Pham
	if (_spreadingFactor == SF_10 || _spreadingFactor == SF_11
	    || _spreadingFactor == SF_12) {
		writeRegister(REG_SYMB_TIMEOUT_LSB, 0x05);
	} else {
		writeRegister(REG_SYMB_TIMEOUT_LSB, 0x08);
	}
	//end

	writeRegister(REG_FIFO_RX_BYTE_ADDR, 0x00);	// Setting current value of reception buffer pointer
	//clearFlags();                                             // Initializing flags
	//state = 1;
	if (_modem == LORA) {	// LoRa mode
		state = setPacketLength(MAX_LENGTH);	// With MAX_LENGTH gets all packets with length < MAX_LENGTH
		writeRegister(REG_OP_MODE, LORA_RX_MODE);	// LORA mode - Rx
#if (SX1272_debug_mode > 1)
		Serial.
		    println(F
			    ("## Receiving LoRa mode activated with success ##"));
		Serial.println();
#endif
	} else {		// FSK mode
		state = setPacketLength();
		writeRegister(REG_OP_MODE, FSK_RX_MODE);	// FSK mode - Rx
#if (SX1272_debug_mode > 1)
		Serial.
		    println(F
			    ("## Receiving FSK mode activated with success ##"));
		Serial.println();
#endif
	}
	return state;
}

uint8_t SX1272::receivePacketMAXTimeout()
{
	return receivePacketTimeout(MAX_TIMEOUT);
}

uint8_t SX1272::receivePacketTimeout()
{
	setTimeout();
	return receivePacketTimeout(_sendTime);
}

#ifdef W_REQUESTED_ACK

// added by C. Pham
// receiver always use receivePacketTimeout()
// sender should either use sendPacketTimeout() or sendPacketTimeoutACK()
uint8_t SX1272::receivePacketTimeout(uint16_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'receivePacketTimeout'"));
#endif

	state = receive();
	if (state == 0) {
		if (availableData(wait)) {
			state = getPacket();
		} else {
			state = 1;
			state_f = 3;	// There is no packet received
		}
	} else {
		state = 1;
		state_f = 1;	// There has been an error with the 'receive' function
	}

	if ((state == 0) || (state == 3) || (state == 5)) {
		if (_reception == INCORRECT_PACKET) {
			state_f = 4;	// The packet has been incorrectly received
		} else if (_reception == INCORRECT_PACKET_TYPE) {
			state_f = 5;	// The packet type has not been recognized
		} else {
			state_f = 0;	// The packet has been correctly received
			// added by C. Pham
			// we get the SNR and RSSI of the received packet for future usage
			//getSNR();
			getRSSIpacket();
		}

		// need to send an ACK
		if (state == 5 && state_f == 0) {

			state = setACK();

			if (state == 0) {
				state = sendWithTimeout();
				if (state == 0) {
					state_f = 0;
#if (SX1272_debug_mode > 1)
					Serial.
					    println(F
						    ("This last packet was an ACK, so ..."));
					Serial.
					    println(F("ACK successfully sent"));
					Serial.println();
#endif
				} else {
					state_f = 1;	// There has been an error with the 'sendWithTimeout' function
				}
			} else {
				state_f = 1;	// There has been an error with the 'setACK' function
			}
		}
	} else {
		// we need to conserve state_f=3 to indicate that no packet has been received after timeout
		//state_f = 1;
	}
	return state_f;
}
#else

uint8_t SX1272::receivePacketTimeout(uint16_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'receivePacketTimeout'"));
#endif

	state = receive();
	if (state == 0) {
		if (availableData(wait)) {
			state_f = getPacket();
		} else {
			state_f = 1;
		}
	} else {
		state_f = state;
	}
	return state_f;
}
#endif

uint8_t SX1272::receivePacketMAXTimeoutACK()
{
	return receivePacketTimeoutACK(MAX_TIMEOUT);
}

uint8_t SX1272::receivePacketTimeoutACK()
{
	setTimeout();
	return receivePacketTimeoutACK(_sendTime);
}

uint8_t SX1272::receivePacketTimeoutACK(uint16_t wait)
{
	return 0;
}

uint8_t SX1272::receiveAll()
{
	return receiveAll(MAX_TIMEOUT);
}

uint8_t SX1272::receiveAll(uint16_t wait)
{
	uint8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'receiveAll'"));
#endif

#ifdef ENABLE_FSK
	if (_modem == FSK) {	// FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
		config1 = readRegister(REG_PACKET_CONFIG1);
		config1 = config1 & B11111001;	// clears bits 2-1 from REG_PACKET_CONFIG1
		writeRegister(REG_PACKET_CONFIG1, config1);	// AddressFiltering = None
	}
#endif

#if (SX1272_debug_mode > 1)
	Serial.println(F("## Address filtering desactivated ##"));
#endif
	state = receive();	// Setting Rx mode
	if (state == 0) {
		state = getPacket(wait);	// Getting all packets received in wait
	}
	return state;
}

boolean SX1272::availableData()
{
	return availableData(MAX_TIMEOUT);
}

boolean SX1272::availableData(uint16_t wait)
{
	byte value;
	byte header = 0;
	boolean forme = false;
	boolean _hreceived = false;
	//unsigned long previous;
	unsigned long exitTime;

#if (SX1272_debug_mode > 0)
	Serial.println(F("Starting 'availableData'"));
#endif

	exitTime = millis() + (unsigned long)wait;

	//previous = millis();
	if (_modem == LORA) {	// LoRa mode
		value = readRegister(REG_IRQ_FLAGS);
		// Wait to ValidHeader interrupt
		//while( (bitRead(value, 4) == 0) && (millis() - previous < (unsigned long)wait) )
		while ((bitRead(value, 4) == 0) && (millis() < exitTime)) {
			value = readRegister(REG_IRQ_FLAGS);
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP32
			yield();
#else
			// adding this small delay decreases the CPU load of the lora_gateway process to 4~5% instead of nearly 100%
			// suggested by rertini (https://github.com/CongducPham/LowCostLoRaGw/issues/211)
			// tests have shown no side effects
			delay(1);
#endif
			// Condition to avoid an overflow (DO NOT REMOVE)
			//if( millis() < previous )
			//{
			//    previous = millis();
			//}
		}		// end while (millis)

		if (bitRead(value, 4) == 1) {	// header received
			_starttime = millis();

#if (SX1272_debug_mode > 0)
			Serial.println(F("Valid Header received in LoRa mode"));
#endif

#ifdef SX1272_led_send_receive
			digitalWrite(SX1272_led_receive, HIGH);
#endif
			_hreceived = true;

#ifdef W_NET_KEY
			// actually, need to wait until 3 bytes have been received
			//while( (header < 3) && (millis() - previous < (unsigned long)wait) )
			while ((header < 3) && (millis() < exitTime))
#else
			//while( (header == 0) && (millis() - previous < (unsigned long)wait) )
			while ((header == 0) && (millis() < exitTime))
#endif
			{	// Waiting to read first payload bytes from packet
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP32
				yield();
#endif
				header = readRegister(REG_FIFO_RX_BYTE_ADDR);
				// Condition to avoid an overflow (DO NOT REMOVE)
				//if( millis() < previous )
				//{
				//    previous = millis();
				//}
			}

			if (header != 0) {	// Reading first byte of the received packet
#ifdef W_NET_KEY
				// added by C. Pham
				// if we actually wait for an ACK, there is no net key before ACK data
				if (_requestACK == 0) {
					_the_net_key_0 = readRegister(REG_FIFO);
					_the_net_key_1 = readRegister(REG_FIFO);
				}
#endif
				_destination = readRegister(REG_FIFO);
			}
		} else {
			forme = false;
			_hreceived = false;
#if (SX1272_debug_mode > 0)
			Serial.println(F("** The timeout has expired **"));
#endif
		}
	} else {		// FSK mode
		value = readRegister(REG_IRQ_FLAGS2);
		// Wait to Payload Ready interrupt
		//while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
		while ((bitRead(value, 2) == 0) && (millis() < exitTime)) {
			value = readRegister(REG_IRQ_FLAGS2);
			// Condition to avoid an overflow (DO NOT REMOVE)
			//if( millis() < previous )
			//{
			//    previous = millis();
			//}
		}		// end while (millis)

		if (bitRead(value, 2) == 1)	{
			_hreceived = true;
#if (SX1272_debug_mode > 0)
			Serial.println(F("Valid Preamble detected in FSK mode"));
#endif
			// Reading first byte of the received packet
			_destination = readRegister(REG_FIFO);
		} else {
			forme = false;
			_hreceived = false;
#if (SX1272_debug_mode > 0)
			Serial.println(F("The timeout has expired"));
#endif
		}
	}
	// We use _hreceived because we need to ensure that _destination value is correctly
	// updated and is not the _destination value from the previously packet
	if (_hreceived == true) {	// Checking destination
#if (SX1272_debug_mode > 0)
		Serial.println(F("Checking destination"));
#endif

#ifdef W_NET_KEY
		forme = true;

		// if we wait for an ACK, then we do not check for net key
		if (_requestACK == 0)
			if (_the_net_key_0 != _my_netkey[0]
			    || _the_net_key_1 != _my_netkey[1]) {
				//#if (SX1272_debug_mode > 0)
				Serial.println(F("Wrong net key"));
				//#endif
				forme = false;
			} else {
				//#if (SX1272_debug_mode > 0)
				Serial.println(F("Good net key"));
				//#endif
			}

		if (forme && ((_destination == _nodeAddress)
			|| (_destination == BROADCAST_0)))
#else
		// modified by C. Pham
		// if _rawFormat, accept all
		if ((_destination == _nodeAddress)
		    || (_destination == BROADCAST_0) || _rawFormat)
#endif
		{		// LoRa or FSK mode
			forme = true;
#if (SX1272_debug_mode > 0)
			Serial.println(F("## Packet received is for me ##"));
#endif
		} else {
			forme = false;
#if (SX1272_debug_mode > 0)
			Serial.println(F("## Packet received is not for me ##"));
#endif

#ifdef SX1272_led_send_receive
			digitalWrite(SX1272_led_receive, LOW);
#endif
		}
	}
	// added by C. Pham
	if (_hreceived == false || forme == false) {
		if (_modem == LORA)	// STANDBY PARA MINIMIZAR EL CONSUMO
		{		// LoRa mode
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
#ifdef ENABLE_FSK
		} else {	//  FSK mode
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
#endif
		}
	}

	return forme;
}

uint8_t SX1272::getPacketMAXTimeout()
{
	return getPacket(MAX_TIMEOUT);
}

int8_t SX1272::getPacket()
{
	return getPacket(MAX_TIMEOUT);
}

/*
 Function: It gets and stores a packet if it is received before ending 'wait' time.
 Returns:  Integer that determines if there has been any error
   // added by C. Pham
   state = 5  --> The command has been executed with no errors and an ACK is requested
   state = 3  --> The command has been executed but packet has been incorrectly received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
int8_t SX1272::getPacket(uint16_t wait)
{
	uint8_t state = 2;
	byte value = 0x00;
	//unsigned long previous;
	unsigned long exitTime;
	boolean p_received = false;

#if (SX1272_debug_mode > 0)
	Serial.println();
	Serial.println(F("Starting 'getPacket'"));
#endif

	//previous = millis();
	exitTime = millis() + (unsigned long)wait;
	if (_modem == LORA) {	// LoRa mode
		value = readRegister(REG_IRQ_FLAGS);
		// Wait until the packet is received (RxDone flag) or the timeout expires
		//while( (bitRead(value, 6) == 0) && (millis() - previous < (unsigned long)wait) )
		while ((bitRead(value, 6) == 0) && (millis() < exitTime)) {
			value = readRegister(REG_IRQ_FLAGS);
			// Condition to avoid an overflow (DO NOT REMOVE)
			//if( millis() < previous )
			//{
			//    previous = millis();
			//}
		}		// end while (millis)

		// modified by C. Pham
		// RxDone
		if ((bitRead(value, 6) == 1)) {
#if (SX1272_debug_mode > 0)
			Serial.println(F("## Packet received in LoRa mode ##"));
#endif

#ifdef SX1272_led_send_receive
			digitalWrite(SX1272_led_receive, LOW);
#endif
			//CrcOnPayload?
			if (bitRead(readRegister(REG_HOP_CHANNEL), 6)) {

				if ((bitRead(value, 5) == 0)) {
					// packet received & CRC correct
					p_received = true;	// packet correctly received
					_reception = CORRECT_PACKET;
#if (SX1272_debug_mode > 0)
					Serial.println(F("** The CRC is correct **"));
#endif
				} else {
					_reception = INCORRECT_PACKET;
					state = 3;
#if (SX1272_debug_mode > 0)
					Serial.println(F("** The CRC is incorrect **"));
#endif
				}
			} else {
				// as CRC is not set we suppose that CRC is correct
				p_received = true;	// packet correctly received
				_reception = CORRECT_PACKET;
#if (SX1272_debug_mode > 0)
				Serial.println(F("## Packet supposed to be correct as CrcOnPayload is off at transmitter ##"));
#endif
			}
		}
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
#ifdef ENABLE_FSK
	} else {		// FSK mode
		value = readRegister(REG_IRQ_FLAGS2);
		//while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
		while ((bitRead(value, 2) == 0) && (millis() < exitTime)) {
			value = readRegister(REG_IRQ_FLAGS2);
			// Condition to avoid an overflow (DO NOT REMOVE)
			//if( millis() < previous )
			//{
			//    previous = millis();
			//}
		}		// end while (millis)

		if (bitRead(value, 2) == 1) {	// packet received
			if (bitRead(value, 1) == 1) {	// CRC correct
				_reception = CORRECT_PACKET;
				p_received = true;
#if (SX1272_debug_mode > 0)
				Serial.
				    println(F
					    ("## Packet correctly received in FSK mode ##"));
#endif
			} else {	// CRC incorrect
				_reception = INCORRECT_PACKET;
				state = 3;
				p_received = false;
#if (SX1272_debug_mode > 0)
				Serial.
				    println(F
					    ("## Packet incorrectly received in FSK mode ##"));
#endif
			}
		} else {
#if (SX1272_debug_mode > 0)
			Serial.println(F("** The timeout has expired **"));
			Serial.println();
#endif
		}
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
#endif
	}

	if (p_received == true) {
		// Store the packet
		if (_modem == LORA) {
			// comment by C. Pham
			// set the FIFO addr to 0 to read again all the bytes
			writeRegister(REG_FIFO_ADDR_PTR, 0x00);	// Setting address pointer in FIFO data buffer

#ifdef W_NET_KEY
			// added by C. Pham
			packet_received.netkey[0] = readRegister(REG_FIFO);
			packet_received.netkey[1] = readRegister(REG_FIFO);
#endif
			//modified by C. Pham
			if (!_rawFormat)
				packet_received.dst = readRegister(REG_FIFO);	// Storing first byte of the received packet
			else
				packet_received.dst = 0;
		} else {
			value = readRegister(REG_PACKET_CONFIG1);
			if ((bitRead(value, 2) == 0)
			    && (bitRead(value, 1) == 0)) {
				packet_received.dst = readRegister(REG_FIFO);	// Storing first byte of the received packet
			} else {
				packet_received.dst = _destination;	// Storing first byte of the received packet
			}
		}

		// modified by C. Pham
		if (!_rawFormat) {
			packet_received.type = readRegister(REG_FIFO);	// Reading second byte of the received packet
			// check packet type to discard unknown packet type
			if (((packet_received.type & PKT_TYPE_MASK) !=
			     PKT_TYPE_DATA)
			    && ((packet_received.type & PKT_TYPE_MASK) !=
				PKT_TYPE_ACK)) {
				_reception = INCORRECT_PACKET_TYPE;
				state = 3;
#if (SX1272_debug_mode > 0)
				Serial.
				    println(F
					    ("** The packet type is incorrect **"));
#endif
				return state;
			}
			packet_received.src = readRegister(REG_FIFO);	// Reading second byte of the received packet
			packet_received.packnum = readRegister(REG_FIFO);	// Reading third byte of the received packet
			//packet_received.length = readRegister(REG_FIFO);  // Reading fourth byte of the received packet
		} else {
			packet_received.type = 0;
			packet_received.src = 0;
			packet_received.packnum = 0;
		}

		if (_reception == CORRECT_PACKET) {

			packet_received.length = readRegister(REG_RX_NB_BYTES);

			if (_modem == LORA) {
				if (_rawFormat) {
					_payloadlength = packet_received.length;
				} else
					_payloadlength =
					    packet_received.length -
					    OFFSET_PAYLOADLENGTH;
			}
			if (packet_received.length > (MAX_LENGTH + 1)) {
#if (SX1272_debug_mode > 0)
				Serial.
				    println(F
					    ("Corrupted packet, length must be less than 256"));
#endif
			} else {
				for (unsigned int i = 0; i < _payloadlength;
				     i++) {
					packet_received.data[i] = readRegister(REG_FIFO);	// Storing payload
				}

				// commented by C. Pham
				//packet_received.retry = readRegister(REG_FIFO);

				// Print the packet if debug_mode
#if (SX1272_debug_mode > 0)
				Serial.println(F("## Packet received:"));
				Serial.print(F("Destination: "));
				Serial.println(packet_received.dst);	// Printing destination
				Serial.print(F("Type: "));
				Serial.println(packet_received.type);	// Printing type
				Serial.print(F("Source: "));
				Serial.println(packet_received.src);	// Printing source
				Serial.print(F("Packet number: "));
				Serial.println(packet_received.packnum);	// Printing packet number
				Serial.print(F("Packet length: "));
				Serial.println(packet_received.length);	// Printing packet length
				Serial.print(F("Data: "));
				for (unsigned int i = 0; i < _payloadlength;
				     i++) {
					Serial.print((char)packet_received.data[i]);	// Printing payload
				}
				Serial.println();
				//Serial.print(F("Retry number: "));
				//Serial.println(packet_received.retry);                        // Printing number retry
				Serial.println(F("##"));
				Serial.println();
#endif
				state = 0;

#ifdef W_REQUESTED_ACK
				// added by C. Pham
				// need to send an ACK
				if (packet_received.type & PKT_FLAG_ACK_REQ) {
					state = 5;
					_requestACK_indicator = 1;
				} else
					_requestACK_indicator = 0;
#endif
			}
		}
	} else {
		//state = 1;
		if ((_reception == INCORRECT_PACKET)
		    && (_retries < _maxRetries)) {
			// comment by C. Pham
			// what is the purpose of incrementing retries here?
			// bug? not needed?
			_retries++;
#if (SX1272_debug_mode > 0)
			Serial.
			    println(F
				    ("## Retrying to send the last packet ##"));
			Serial.println();
#endif
		}
	}
	if (_modem == LORA) {
		writeRegister(REG_FIFO_ADDR_PTR, 0x00);	// Setting address pointer in FIFO data buffer
	}
	clearFlags();		// Initializing flags
	if (wait > MAX_WAIT) {
		state = -1;
#if (SX1272_debug_mode > 0)
		Serial.
		    println(F
			    ("** The timeout must be smaller than 12.5 seconds **"));
		Serial.println();
#endif
	}

	return state;
}

/*
 Function: It sets the packet destination.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   dest: destination value of the packet sent.
*/
int8_t SX1272::setDestination(uint8_t dest)
{
	int8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'setDestination'"));
#endif

	state = 1;
	_destination = dest;	// Storing destination in a global variable
	packet_sent.dst = dest;	// Setting destination in packet structure
	packet_sent.src = _nodeAddress;	// Setting source in packet structure
	packet_sent.packnum = _packetNumber;	// Setting packet number in packet structure
	_packetNumber++;
	state = 0;

#if (SX1272_debug_mode > 1)
	Serial.print(F("## Destination "));
	Serial.print(_destination, HEX);
	Serial.println(F(" successfully set ##"));
	Serial.print(F("## Source "));
	Serial.print(packet_sent.src, DEC);
	Serial.println(F(" successfully set ##"));
	Serial.print(F("## Packet number "));
	Serial.print(packet_sent.packnum, DEC);
	Serial.println(F(" successfully set ##"));
	Serial.println();
#endif
	return state;
}

/*
 Function: It sets the timeout according to the configured mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setTimeout()
{
	uint8_t state = 2;
	//uint16_t delay;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'setTimeout'"));
#endif

	state = 1;

	// changed by C. Pham
	// we always use MAX_TIMEOUT
	_sendTime = MAX_TIMEOUT;

#if (SX1272_debug_mode > 1)
	Serial.print(F("Timeout to send/receive is: "));
	Serial.println(_sendTime, DEC);
#endif
	state = 0;
	return state;
}

/*
 Function: It sets an uint8_t array payload packet in a packet struct.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPayload(uint8_t * payload)
{
	uint8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'setPayload'"));
#endif

	state = 1;
	if ((_modem == FSK) && (_payloadlength > MAX_PAYLOAD_FSK)) {
		_payloadlength = MAX_PAYLOAD_FSK;
		state = 1;
#if (SX1272_debug_mode > 1)
		Serial.
		    println(F
			    ("In FSK, payload length must be less than 60 bytes."));
		Serial.println();
#endif
	}
	for (unsigned int i = 0; i < _payloadlength; i++) {
		packet_sent.data[i] = payload[i];	// Storing payload in packet structure
	}
	// set length with the actual counter value
	state = setPacketLength();	// Setting packet length in packet structure
	return state;
}

/*
 Function: It sets a packet struct in FIFO in order to sent it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPacket(uint8_t dest, uint8_t * payload)
{
	int8_t state = 2;
	byte st0;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setPacket'"));
#endif

	// added by C. Pham
	// check for enough remaining ToA
	// when operating under duty-cycle mode
	if (_limitToA) {
		// here truncPayload() should have been called before in
		// sendPacketTimeout(uint8_t dest, uint8_t *payload, uint16_t length16)
		uint16_t length16 = _payloadlength;

		if (!_rawFormat)
			length16 = length16 + OFFSET_PAYLOADLENGTH;

		if (getRemainingToA() - getToA(length16) < 0) {
			Serial.print(F("## not enough ToA at "));
			Serial.println(millis());
			return SX1272_ERROR_TOA;
		}
	}

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	clearFlags();		// Initializing flags

	if (_modem == LORA) {	// LoRa mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
#ifdef ENABLE_FSK
	} else {		// FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
#endif
	}

	_reception = CORRECT_PACKET;	// Updating incorrect value to send a packet (old or new)
	if (_retries == 0) {	// Sending new packet
		state = setDestination(dest);	// Setting destination in packet structure
		packet_sent.retry = _retries;
		if (state == 0) {
			state = setPayload(payload);
		}
	} else {
		// comment by C. Pham
		// why to increase the length here?
		// bug?
		if (_retries == 1) {
			packet_sent.length++;
		}
		state = setPacketLength();
		packet_sent.retry = _retries;
#if (SX1272_debug_mode > 0)
		Serial.print(F("** Retrying to send last packet "));
		Serial.print(_retries, DEC);
		Serial.println(F(" time **"));
#endif
	}

	// added by C. Pham
	// set the type to be a data packet
	packet_sent.type |= PKT_TYPE_DATA;

#ifdef W_REQUESTED_ACK
	// added by C. Pham
	// indicate that an ACK should be sent by the receiver
	if (_requestACK)
		packet_sent.type |= PKT_FLAG_ACK_REQ;
#endif

	writeRegister(REG_FIFO_ADDR_PTR, 0x80);	// Setting address pointer in FIFO data buffer
	if (state == 0) {
		state = 1;
		// Writing packet to send in FIFO
#ifdef W_NET_KEY
		// added by C. Pham
		packet_sent.netkey[0] = _my_netkey[0];
		packet_sent.netkey[1] = _my_netkey[1];
		//#if (SX1272_debug_mode > 0)
		Serial.println(F("## Setting net key ##"));
		//#endif
		writeRegister(REG_FIFO, packet_sent.netkey[0]);
		writeRegister(REG_FIFO, packet_sent.netkey[1]);
#endif
		// added by C. Pham
		// we can skip the header for instance when we want to generate
		// at a higher layer a LoRaWAN packet
		if (!_rawFormat) {
			writeRegister(REG_FIFO, packet_sent.dst);	// Writing the destination in FIFO
			// added by C. Pham
			writeRegister(REG_FIFO, packet_sent.type);	// Writing the packet type in FIFO
			writeRegister(REG_FIFO, packet_sent.src);	// Writing the source in FIFO
			writeRegister(REG_FIFO, packet_sent.packnum);	// Writing the packet number in FIFO
		}
		// commented by C. Pham
		//writeRegister(REG_FIFO, packet_sent.length);  // Writing the packet length in FIFO
		for (unsigned int i = 0; i < _payloadlength; i++) {
			writeRegister(REG_FIFO, packet_sent.data[i]);	// Writing the payload in FIFO
		}
		// commented by C. Pham
		//writeRegister(REG_FIFO, packet_sent.retry);           // Writing the number retry in FIFO
		state = 0;
#if (SX1272_debug_mode > 0)
		Serial.println(F("## Packet set and written in FIFO ##"));
		// Print the complete packet if debug_mode
		Serial.println(F("## Packet to send: "));
		Serial.print(F("Destination: "));
		Serial.println(packet_sent.dst);	// Printing destination
		Serial.print(F("Packet type: "));
		Serial.println(packet_sent.type);	// Printing packet type
		Serial.print(F("Source: "));
		Serial.println(packet_sent.src);	// Printing source
		Serial.print(F("Packet number: "));
		Serial.println(packet_sent.packnum);	// Printing packet number
		Serial.print(F("Packet length: "));
		Serial.println(packet_sent.length);	// Printing packet length
		Serial.print(F("Data: "));
		for (unsigned int i = 0; i < _payloadlength; i++) {
			Serial.print((char)packet_sent.data[i]);	// Printing payload
		}
		Serial.println();
		//Serial.print(F("Retry number: "));
		//Serial.println(packet_sent.retry);                    // Printing retry number
		Serial.println(F("##"));
#endif
	}
	writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	return state;
}

uint8_t SX1272::sendWithMAXTimeout()
{
	return sendWithTimeout(MAX_TIMEOUT);
}

uint8_t SX1272::sendWithTimeout()
{
	setTimeout();
	return sendWithTimeout(_sendTime);
}

uint8_t SX1272::sendWithTimeout(uint16_t wait)
{
	uint8_t state = 2;
	byte value = 0x00;
	//unsigned long previous;
	unsigned long exitTime;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'sendWithTimeout'"));
#endif

#ifdef SX1272_led_send_receive
	digitalWrite(SX1272_led_send, HIGH);
#endif
	// clearFlags();    // Initializing flags

	// wait to TxDone flag
	//previous = millis();
	exitTime = millis() + (unsigned long)wait;
	if (_modem == LORA) {	// LoRa mode
		clearFlags();	// Initializing flags

		writeRegister(REG_OP_MODE, LORA_TX_MODE);	// LORA mode - Tx

#if (SX1272_debug_mode > 1)
		value = readRegister(REG_OP_MODE);

		if (value & LORA_TX_MODE == LORA_TX_MODE)
			Serial.println(F("OK"));
		else
			Serial.println(F("ERROR"));
#endif
		value = readRegister(REG_IRQ_FLAGS);
		// Wait until the packet is sent (TX Done flag) or the timeout expires
		//while ((bitRead(value, 3) == 0) && (millis() - previous < wait))
		while ((bitRead(value, 3) == 0) && (millis() < exitTime)) {
			value = readRegister(REG_IRQ_FLAGS);
			// Condition to avoid an overflow (DO NOT REMOVE)
			//if( millis() < previous )
			//{
			//    previous = millis();
			//}
		}
		state = 1;
	} else {		// FSK mode
		writeRegister(REG_OP_MODE, FSK_TX_MODE);	// FSK mode - Tx

		value = readRegister(REG_IRQ_FLAGS2);
		// Wait until the packet is sent (Packet Sent flag) or the timeout expires
		//while ((bitRead(value, 3) == 0) && (millis() - previous < wait))
		while ((bitRead(value, 3) == 0) && (millis() < exitTime)) {
			value = readRegister(REG_IRQ_FLAGS2);
			// Condition to avoid an overflow (DO NOT REMOVE)
			//if( millis() < previous )
			//{
			//    previous = millis();
			//}
		}
		state = 1;
	}

#ifdef SX1272_led_send_receive
	digitalWrite(SX1272_led_send, LOW);
#endif

	if (bitRead(value, 3) == 1) {
		state = 0;	// Packet successfully sent
#if (SX1272_debug_mode > 1)
		Serial.println(F("## Packet successfully sent ##"));
		Serial.println();
#endif
		// added by C. Pham
		// normally there should be enough remaing ToA as the test has been done earlier
		if (_limitToA)
			removeToA(_currentToA);
	} else {
		if (state == 1) {
#if (SX1272_debug_mode > 1)
			Serial.println(F("** Timeout has expired **"));
			Serial.println();
#endif
		} else {
#if (SX1272_debug_mode > 1)
			Serial.
			    println(F
				    ("** There has been an error and packet has not been sent **"));
			Serial.println();
#endif
		}
	}

	clearFlags();		// Initializing flags
	return state;
}

uint8_t SX1272::sendPacketMAXTimeout(uint8_t dest, uint8_t * payload,
				     uint16_t length16)
{
	return sendPacketTimeout(dest, payload, length16, MAX_TIMEOUT);
}

uint8_t SX1272::sendPacketTimeout(uint8_t dest, uint8_t * payload,
				  uint16_t length16)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'sendPacketTimeout'"));
#endif

	state = truncPayload(length16);

	if (state == 0) {
		state_f = setPacket(dest, payload);	// Setting a packet with 'dest' destination
	}			// and writing it in FIFO.
	else {
		state_f = state;
	}
	if (state_f == 0) {
		state_f = sendWithTimeout();	// Sending the packet
	}
	return state_f;
}

uint8_t SX1272::sendPacketTimeout(uint8_t dest, uint8_t * payload,
				  uint16_t length16, uint16_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'sendPacketTimeout'"));
#endif

	state = truncPayload(length16);
	if (state == 0) {
		state_f = setPacket(dest, payload);	// Setting a packet with 'dest' destination
	} else {
		state_f = state;
	}
	if (state_f == 0)	// and writing it in FIFO.
	{
		state_f = sendWithTimeout(wait);	// Sending the packet
	}
	return state_f;
}

uint8_t SX1272::sendPacketMAXTimeoutACK(uint8_t dest, uint8_t * payload,
					uint16_t length16)
{
	return sendPacketTimeoutACK(dest, payload, length16, MAX_TIMEOUT);
}

uint8_t SX1272::sendPacketTimeoutACK(uint8_t dest, uint8_t * payload,
				     uint16_t length16)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'sendPacketTimeoutACK'"));
#endif

#ifdef W_REQUESTED_ACK
	_requestACK = 1;
#endif
	// Sending packet to 'dest' destination
	state = sendPacketTimeout(dest, payload, length16);

	// Trying to receive the ACK
	if (state == 0) {
		state = receive();	// Setting Rx mode to wait an ACK
	} else {
		state_f = state;
	}
	if (state == 0) {
		// added by C. Pham
		Serial.println(F("wait for ACK"));

		if (availableData()) {
			state_f = getACK();	// Getting ACK
		} else {
			state_f = SX1272_ERROR_ACK;
			// added by C. Pham
			Serial.println(F("no ACK"));
		}
	} else {
		state_f = state;
	}

#ifdef W_REQUESTED_ACK
	_requestACK = 0;
#endif
	return state_f;
}

uint8_t SX1272::sendPacketTimeoutACK(uint8_t dest, uint8_t * payload,
				     uint16_t length16, uint16_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'sendPacketTimeoutACK'"));
#endif

#ifdef W_REQUESTED_ACK
	_requestACK = 1;
#endif
	state = sendPacketTimeout(dest, payload, length16, wait);	// Sending packet to 'dest' destination

	if (state == 0) {
		state = receive();	// Setting Rx mode to wait an ACK
	} else {
		state_f = 1;
	}
	if (state == 0) {
		// added by C. Pham
		Serial.println(F("wait for ACK"));

		if (availableData()) {
			state_f = getACK();	// Getting ACK
		} else {
			state_f = SX1272_ERROR_ACK;
			// added by C. Pham
			Serial.println(F("no ACK"));
		}
	} else {
		state_f = 1;
	}

#ifdef W_REQUESTED_ACK
	_requestACK = 0;
#endif
	return state_f;
}

uint8_t SX1272::getACK()
{
	return getACK(MAX_TIMEOUT);
}

/*
 Function: It gets and stores an ACK if it is received, before ending 'wait' time.
 Returns: Integer that determines if there has been any error
   state = 2  --> The ACK has not been received
   state = 1  --> The N-ACK has been received with no errors
   state = 0  --> The ACK has been received with no errors
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
uint8_t SX1272::getACK(uint16_t wait)
{
	uint8_t state = 2;
	byte value = 0x00;
	//unsigned long previous;
	unsigned long exitTime;
	boolean a_received = false;

	//#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getACK'"));
	//#endif

	//previous = millis();
	exitTime = millis() + (unsigned long)wait;
	if (_modem == LORA) {	// LoRa mode
		value = readRegister(REG_IRQ_FLAGS);
		// Wait until the ACK is received (RxDone flag) or the timeout expires
		//while ((bitRead(value, 6) == 0) && (millis() - previous < wait))
		while ((bitRead(value, 6) == 0) && (millis() < exitTime)) {
			value = readRegister(REG_IRQ_FLAGS);
			//if( millis() < previous )
			//{
			//    previous = millis();
			//}
		}
		if (bitRead(value, 6) == 1) {	// ACK received
			// comment by C. Pham
			// not really safe because the received packet may not be an ACK
			// probability is low if using unicast to gateway, but if broadcast
			// can get a packet from another node!!
			a_received = true;
		}
		// Standby para minimizar el consumo
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
#ifdef ENABLE_FSK
	} else {		// FSK mode
		value = readRegister(REG_IRQ_FLAGS2);
		// Wait until the packet is received (RxDone flag) or the timeout expires
		//while ((bitRead(value, 2) == 0) && (millis() - previous < wait))
		while ((bitRead(value, 2) == 0) && (millis() < exitTime)) {
			value = readRegister(REG_IRQ_FLAGS2);
			//if( millis() < previous )
			//{
			//    previous = millis();
			//}
		}
		if (bitRead(value, 2) == 1) {	// ACK received
			a_received = true;
		}
		// Standby para minimizar el consumo
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
#endif
	}

	// not safe because the received packet may not be an ACK!
	if (a_received) {
		// Storing the received ACK
		ACK.dst = _destination;
		ACK.type = readRegister(REG_FIFO);
		ACK.src = readRegister(REG_FIFO);
		ACK.packnum = readRegister(REG_FIFO);
		ACK.length = readRegister(REG_FIFO);
		ACK.data[0] = readRegister(REG_FIFO);
		ACK.data[1] = readRegister(REG_FIFO);

		if (ACK.type == PKT_TYPE_ACK) {

			// Checking the received ACK
			if (ACK.dst == packet_sent.src) {
				if (ACK.src == packet_sent.dst) {
					if (ACK.packnum == packet_sent.packnum) {
						if (ACK.length == 2) {
							if (ACK.data[0] ==
							    CORRECT_PACKET) {
								state = 0;
								//#if (SX1272_debug_mode > 0)
								// Printing the received ACK
								Serial.
								    println(F
									    ("## ACK received:"));
								Serial.
								    print(F
									  ("Destination: "));
								Serial.println(ACK.dst);	// Printing destination
								Serial.
								    print(F
									  ("Source: "));
								Serial.println(ACK.src);	// Printing source
								Serial.
								    print(F
									  ("ACK number: "));
								Serial.println(ACK.packnum);	// Printing ACK number
								Serial.
								    print(F
									  ("ACK length: "));
								Serial.println(ACK.length);	// Printing ACK length
								Serial.
								    print(F
									  ("ACK payload: "));
								Serial.println(ACK.data[0]);	// Printing ACK payload
								Serial.
								    print(F
									  ("ACK SNR of rcv pkt at gw: "));

								value =
								    ACK.data[1];

								if (value & 0x80)	// The SNR sign bit is 1
								{
									// Invert and divide by 4
									value =
									    ((~value + 1) & 0xFF) >> 2;
									_rcv_snr_in_ack
									    =
									    -value;
								} else {
									// Divide by 4
									_rcv_snr_in_ack
									    =
									    (value
									     &
									     0xFF)
									    >>
									    2;
								}

								Serial.
								    println
								    (_rcv_snr_in_ack);
								Serial.
								    println(F
									    ("##"));
								Serial.
								    println();
								//#endif
							} else {
								state = 1;
#if (SX1272_debug_mode > 0)
								Serial.
								    println(F
									    ("** N-ACK received **"));
								Serial.
								    println();
#endif
							}
						} else {
							state = 1;
#if (SX1272_debug_mode > 0)
							Serial.
							    println(F
								    ("** ACK length incorrectly received **"));
							Serial.println();
#endif
						}
					} else {
						state = 1;
#if (SX1272_debug_mode > 0)
						Serial.
						    println(F
							    ("** ACK number incorrectly received **"));
						Serial.println();
#endif
					}
				} else {
					state = 1;
#if (SX1272_debug_mode > 0)
					Serial.
					    println(F
						    ("** ACK source incorrectly received **"));
					Serial.println();
#endif
				}
			}
		} else {
			state = 1;
#if (SX1272_debug_mode > 0)
			Serial.
			    println(F
				    ("** ACK destination incorrectly received **"));
			Serial.println();
#endif
		}
	} else {
		state = 1;
#if (SX1272_debug_mode > 0)
		Serial.println(F("** ACK lost **"));
		Serial.println();
#endif
	}
	clearFlags();		// Initializing flags
	return state;
}

uint8_t SX1272::getTemp()
{
	byte st0;
	uint8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getTemp'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	if (_modem == LORA) {	// Allowing access to FSK registers while in LoRa standby mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
	}

	state = 1;
	// Saving temperature value
	_temp = readRegister(REG_TEMP);
	if (_temp & 0x80)	// The SNR sign bit is 1
	{
		// Invert and divide by 4
		_temp = ((~_temp + 1) & 0xFF);
	} else {
		// Divide by 4
		_temp = (_temp & 0xFF);
	}

#if (SX1272_debug_mode > 1)
	Serial.print(F("## Temperature is: "));
	Serial.print(_temp);
	Serial.println(F(" ##"));
	Serial.println();
#endif

	if (_modem == LORA) {
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	}

	state = 0;
	return state;
}

void SX1272::setPacketType(uint8_t type)
{
	packet_sent.type = type;

	if (type & PKT_FLAG_ACK_REQ)
		_requestACK = 1;
}

uint8_t SX1272::doCAD(uint8_t counter)
{
	uint8_t state = 2;
	byte value = 0x00;
	unsigned long startCAD, endCAD, startDoCad, endDoCad;
	//unsigned long previous;
	unsigned long exitTime;
	uint16_t wait = 100;
	bool failedCAD = false;
	uint8_t retryCAD = 3;
	uint8_t save_counter;
	byte st0, stnew;
	int rssi_count = 0;
	int rssi_mean = 0;
	double bw = 0.0;
	bool hasRSSI = false;
	unsigned long startRSSI = 0;

	bw = (_bandwidth ==
	      BW_125) ? 125e3 : ((_bandwidth == BW_250) ? 250e3 : 500e3);
	// Symbol rate : time for one symbol (usecs)
	double rs = bw / (1 << _spreadingFactor);
	double ts = 1 / rs;
	ts = ts * 1000000.0;

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

#ifdef DEBUG_CAD
	Serial.println(F("SX1272::Starting 'doCAD'"));
#endif

	save_counter = counter;

	startDoCad = millis();

	if (_modem == LORA) {	// LoRa mode
		stnew = LORA_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);

		do {

			hasRSSI = false;

			clearFlags();	// Initializing flags

			// wait to CadDone flag
			// previous = millis();
			startCAD = millis();
			exitTime = millis() + (unsigned long)wait;

			stnew = LORA_CAD_MODE;
			writeRegister(REG_OP_MODE, LORA_CAD_MODE);	// LORA mode - Cad

			startRSSI = micros();

			value = readRegister(REG_IRQ_FLAGS);
			// Wait until CAD ends (CAD Done flag) or the timeout expires
			//while ((bitRead(value, 2) == 0) && (millis() - previous < wait))
			while ((bitRead(value, 2) == 0)
			       && (millis() < exitTime)) {
				// only one reading per CAD
				if (micros() - startRSSI > ts + 240 && !hasRSSI) {
					_RSSI =
					    -(OFFSET_RSSI +
					      (_board ==
					       SX1276Chip ? 18 : 0)) +
					    readRegister(REG_RSSI_VALUE_LORA);
					rssi_mean += _RSSI;
					rssi_count++;
					hasRSSI = true;
				}

				value = readRegister(REG_IRQ_FLAGS);
				// Condition to avoid an overflow (DO NOT REMOVE)
				//if( millis() < previous )
				//{
				//    previous = millis();
				//}
			}
			state = 1;

			endCAD = millis();

			if (bitRead(value, 2) == 1) {
				state = 0;	// CAD successfully performed
#ifdef DEBUG_CAD
				Serial.print(F("SX1272::CAD duration "));
				Serial.println(endCAD - startCAD);
				Serial.
				    println(F
					    ("SX1272::CAD successfully performed"));
#endif

				value = readRegister(REG_IRQ_FLAGS);

				// look for the CAD detected bit
				if (bitRead(value, 0) == 1) {
					// we detected activity
					failedCAD = true;
#ifdef DEBUG_CAD
					Serial.
					    print(F
						  ("SX1272::CAD exits after "));
					Serial.println(save_counter - counter);
#endif
				}

				counter--;
			} else {
#ifdef DEBUG_CAD
				Serial.print(F("SX1272::CAD duration "));
				Serial.println(endCAD - startCAD);
#endif
				if (state == 1) {
#ifdef DEBUG_CAD
					Serial.
					    println(F
						    ("SX1272::Timeout has expired"));
#endif
				} else {
#ifdef DEBUG_CAD
					Serial.
					    println(F
						    ("SX1272::Error and CAD has not been performed"));
#endif
				}

				retryCAD--;

				// to many errors, so exit by indicating that channel is not free
				if (!retryCAD)
					failedCAD = true;
			}

		} while (counter && !failedCAD);

		rssi_mean = rssi_mean / rssi_count;
		_RSSI = rssi_mean;
	}

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);

	endDoCad = millis();

	clearFlags();		// Initializing flags

#ifdef DEBUG_CAD
	Serial.print(F("SX1272::doCAD duration "));
	Serial.println(endDoCad - startDoCad);
#endif

	if (failedCAD)
		return 2;

	return state;
}

//#define DEBUG_GETTOA

#ifdef DEBUG_GETTOA

void printDouble(double val, byte precision)
{
	// prints val with number of decimal places determine by precision
	// precision is a number from 0 to 6 indicating the desired decimial places
	// example: lcdPrintDouble( 3.1415, 2); // prints 3.14 (two decimal places)

	if (val < 0.0) {
		Serial.print('-');
		val = -val;
	}

	Serial.print(int (val));	//prints the int part
	if (precision > 0) {
		Serial.print(".");	// print the decimal point
		unsigned long frac;
		unsigned long mult = 1;
		byte padding = precision - 1;
		while (precision--)
			mult *= 10;

		if (val >= 0)
			frac = (val - int (val))*mult;
		else
			frac = (int (val) - val)*mult;
		unsigned long frac1 = frac;
		while (frac1 /= 10)
			padding--;
		while (padding--)
			Serial.print("0");
		Serial.print(frac, DEC);
	}
}

#endif

uint16_t SX1272::getToA(uint8_t pl)
{

	uint8_t DE = 0;
	uint32_t airTime = 0;

	double bw = 0.0;

	bw = (_bandwidth ==
	      BW_125) ? 125e3 : ((_bandwidth == BW_250) ? 250e3 : 500e3);

#ifdef DEBUG_GETTOA
	Serial.print(F("SX1272::bw is "));
	Serial.println(bw);

	Serial.print(F("SX1272::SF is "));
	Serial.println(_spreadingFactor);
#endif

	//double ts=pow(2,_spreadingFactor)/bw;

	////// from LoRaMAC SX1272GetTimeOnAir()

	// Symbol rate : time for one symbol (secs)
	double rs = bw / (1 << _spreadingFactor);
	double ts = 1 / rs;

	// must add 4 to the programmed preamble length to get the effective preamble length
	double tPreamble = ((_preamblelength + 4) + 4.25) * ts;

#ifdef DEBUG_GETTOA
	Serial.print(F("SX1272::ts is "));
	printDouble(ts, 6);
	Serial.println();
	Serial.print(F("SX1272::tPreamble is "));
	printDouble(tPreamble, 6);
	Serial.println();
#endif

	// for low data rate optimization
	if ((_bandwidth == BW_125) && _spreadingFactor == 12)
		DE = 1;

	// Symbol length of payload and time
	double tmp = (8 * pl - 4 * _spreadingFactor + 28 + 16 - 20 * _header) /
	    (double)(4 * (_spreadingFactor - 2 * DE));

#ifdef DEBUG_GETTOA
	Serial.print(F("SX1272::tmp is "));
	printDouble(tmp, 6);
	Serial.println();
#endif

	tmp = ceil(tmp) * (_codingRate + 4);

	double nPayload = 8 + ((tmp > 0) ? tmp : 0);

#ifdef DEBUG_GETTOA
	Serial.print(F("SX1272::nPayload is "));
	Serial.println(nPayload);
#endif

	double tPayload = nPayload * ts;
	// Time on air
	double tOnAir = tPreamble + tPayload;
	// in us secs
	airTime = floor(tOnAir * 1e6 + 0.999);

	//////

#ifdef DEBUG_GETTOA
	Serial.print(F("SX1272::airTime is "));
	Serial.println(airTime);
#endif
	// return in ms
	_currentToA = ceil(airTime / 1000) + 1;
	return _currentToA;
}

// need to set _send_cad_number to a value > 0
// we advise using _send_cad_number=3 for a SIFS and _send_cad_number=9 for a DIFS
// prior to send any data
void SX1272::CarrierSense()
{

	int e;
	bool carrierSenseRetry = false;
	uint8_t retries = 3;
	uint8_t DIFSretries = 8;

	Serial.print(F("--> CS1\n"));

	if (_send_cad_number && _enableCarrierSense) {

		do {
			DIFSretries = 8;
			do {

				// check for free channel (SIFS/DIFS)
				_startDoCad = millis();
				e = doCAD(_send_cad_number);
				_endDoCad = millis();

				Serial.print(F("--> CAD "));
				Serial.print(_endDoCad - _startDoCad);
				Serial.println();

				if (!e) {
					Serial.print(F("OK1\n"));

					if (_extendedIFS) {
						// wait for random number of CAD
						uint8_t w = random(1, 8);

						Serial.
						    print(F("--> wait for "));
						Serial.print(w);
						Serial.print(F(" CAD = "));
						Serial.
						    print(sx1272_CAD_value
							  [_loraMode] * w);
						Serial.println();

						delay(sx1272_CAD_value
						      [_loraMode] * w);

						// check for free channel (SIFS/DIFS) once again
						_startDoCad = millis();
						e = doCAD(_send_cad_number);
						_endDoCad = millis();

						Serial.print(F("--> CAD "));
						Serial.print(_endDoCad -
							     _startDoCad);
						Serial.println();

						if (!e)
							Serial.print(F("OK2"));
						else
							Serial.print(F("#2"));

						Serial.println();
					}
				} else {
					Serial.print(F("#1\n"));

					// wait for random number of DIFS
					uint8_t w = random(1, 8);

					Serial.print(F("--> wait for "));
					Serial.print(w);
					Serial.print(F(" DIFS=3SIFS= "));
					Serial.
					    print(sx1272_SIFS_value[_loraMode] *
						  3 * w);
					Serial.println();

					delay(sx1272_SIFS_value[_loraMode] * 3 *
					      w);

					Serial.print(F("--> retry\n"));
				}

			} while (e && --DIFSretries);

			// CAD is OK, but need to check RSSI
			if (_RSSIonSend) {

				e = getRSSI();
				uint8_t rssi_retry_count = 8;

				if (!e) {

					do {
						getRSSI();
						Serial.print(F("--> RSSI "));
						Serial.print(_RSSI);
						Serial.println();
						rssi_retry_count--;
						delay(1);
					} while (_RSSI > -90
						 && rssi_retry_count);
				} else
					Serial.print(F("--> RSSI error\n"));

				if (!rssi_retry_count)
					carrierSenseRetry = true;
				else
					carrierSenseRetry = false;
			}
		} while (carrierSenseRetry && --retries);
	}
}

int8_t SX1272::getSyncWord()
{
	int8_t state = 2;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'getSyncWord'"));
#endif

	if (_modem == FSK) {
		state = -1;	// sync word is not available in FSK mode
#if (SX1272_debug_mode > 1)
		Serial.println(F("** FSK mode hasn't sync word **"));
#endif
	} else {
		_syncWord = readRegister(REG_SYNC_WORD);

		state = 0;

#if (SX1272_debug_mode > 1)
		Serial.print(F("## Sync word is "));
		Serial.print(_syncWord, HEX);
		Serial.println(F(" ##"));
#endif
	}
	return state;
}

int8_t SX1272::setSyncWord(uint8_t sw)
{
	byte st0, stnew;
	int8_t state = 2;
	byte config1;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setSyncWord'"));
#endif

	st0 = readRegister(REG_OP_MODE);

	if (_modem == FSK) {
#if (SX1272_debug_mode > 1)
		Serial.print(F("Notice that FSK hasn't sync word parameter, "));
		Serial.println(F("transfter to LoRa mode"));
#endif
		state = setLORA();
	}
	stnew = LORA_STANDBY_MODE;
	if (st0 != stnew)
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);

	writeRegister(REG_SYNC_WORD, sw);

	delay(100);

	config1 = readRegister(REG_SYNC_WORD);

	if (config1 == sw) {
		state = 0;
		_syncWord = sw;
#if (SX1272_debug_mode > 1)
		Serial.print(F("Sync Word "));
		Serial.print(sw, HEX);
		Serial.println(F(" has been successfully set"));
#endif
	} else {
		state = 1;
#if (SX1272_debug_mode > 1)
		Serial.println(F("Setting Sync Word error"));
#endif
	}

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);

	delay(100);

	return state;
}

int8_t SX1272::setSleepMode()
{

	int8_t state = 2;
	byte value;

	writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
	// proposed by escyes
	// https://github.com/CongducPham/LowCostLoRaGw/issues/53#issuecomment-289237532
	//
	// inserted to avoid REG_OP_MODE stay = 0x40 (no sleep mode)
	delay(100);
	writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);	// LoRa sleep mode

	//delay(50);

	value = readRegister(REG_OP_MODE);

	//Serial.print(F("## REG_OP_MODE 0x"));
	//Serial.println(value, HEX);

	if (value == LORA_SLEEP_MODE)
		state = 0;
	else
		state = 1;

	return state;
}

int8_t SX1272::setPowerDBM(uint8_t dbm)
{
	byte st0, stnew;
	int8_t state = 2;
	byte value = 0x00;

	byte RegPaDacReg = (_board == SX1272Chip) ? 0x5A : 0x4D;

#if (SX1272_debug_mode > 1)
	Serial.println(F("Starting 'setPowerDBM'"));
#endif

	st0 = readRegister(REG_OP_MODE);

	if (_modem == LORA) {
		stnew = LORA_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
#ifdef ENABLE_FSK
	} else {
		stnew = FSK_STANDBY_MODE;
		if (st0 != stnew)
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
#endif
	}

	if (dbm == 20) {
		return setPower('X');
	}

	if (dbm > 14)
		return state;

	// disable high power output in all other cases
	writeRegister(RegPaDacReg, 0x84);

	if (dbm > 10)
		// set RegOcp for OcpOn and OcpTrim
		// 130mA
		setMaxCurrent(0x10);
	else
		// 100mA
		setMaxCurrent(0x0B);

	if (_board == SX1272Chip) {
		// Pout = -1 + _power[3:0] on RFO
		// Pout = 2 + _power[3:0] on PA_BOOST
		if (_needPABOOST) {
			value = dbm - 2;
			// we set the PA_BOOST pin
			value = value | B10000000;
		} else
			value = dbm + 1;

		writeRegister(REG_PA_CONFIG, value);
	} else {
		// for the SX1276
		uint8_t pmax = 15;

		// then Pout = Pmax-(15-_power[3:0]) if  PaSelect=0 (RFO pin for +14dBm)
		// so L=3dBm; H=7dBm; M=15dBm (but should be limited to 14dBm by RFO pin)

		// and Pout = 17-(15-_power[3:0]) if  PaSelect=1 (PA_BOOST pin for +14dBm)
		// so x= 14dBm (PA);
		// when p=='X' for 20dBm, value is 0x0F and RegPaDacReg=0x87 so 20dBm is enabled

		if (_needPABOOST) {
			value = dbm - 17 + 15;
			// we set the PA_BOOST pin
			value = value | B10000000;
		} else
			value = dbm - pmax + 15;

		// set MaxPower to 7 -> Pmax=10.8+0.6*MaxPower [dBm] = 15
		value = value | B01110000;

		writeRegister(REG_PA_CONFIG, value);
	}

	_power = value;

	value = readRegister(REG_PA_CONFIG);

	if (value == _power) {
		state = 0;
#if (SX1272_debug_mode > 1)
		Serial.println(F("## Output power has been successfully set ##"));
#endif
	} else {
		state = 1;
	}

	if (st0 != stnew)
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	delay(100);
	return state;
}

long SX1272::limitToA()
{

	// first time we set limitToA?
	// in this design, once you set _limitToA to true
	// it is not possible to set it back to false
	if (_limitToA == false) {
		_startToAcycle = millis();
		_remainingToA = MAX_DUTY_CYCLE_PER_HOUR;
		// we are handling millis() rollover by calculating the end of cycle time
		_endToAcycle = _startToAcycle + DUTYCYCLE_DURATION;
	}

	_limitToA = true;
	return getRemainingToA();
}

long SX1272::getRemainingToA()
{

	if (_limitToA == false)
		return MAX_DUTY_CYCLE_PER_HOUR;

	// we compare to the end of cycle so that millis() rollover is taken into account
	// using unsigned long modulo operation
	if ((millis() > _endToAcycle)) {
		_startToAcycle = _endToAcycle;
		_remainingToA = MAX_DUTY_CYCLE_PER_HOUR;
		_endToAcycle = _startToAcycle + DUTYCYCLE_DURATION;

		Serial.println(F("## new cycle for ToA ##"));
		Serial.print(F("cycle begins at "));
		Serial.print(_startToAcycle);
		Serial.print(F(" cycle ends at "));
		Serial.print(_endToAcycle);
		Serial.print(F(" remaining ToA is "));
		Serial.print(_remainingToA);
		Serial.println();
	}

	return _remainingToA;
}

long SX1272::removeToA(uint16_t toa)
{

	// first, update _remainingToA
	getRemainingToA();

	if (_limitToA) {
		_remainingToA -= toa;
	}

	return _remainingToA;
}

int8_t SX1272::setFreqHopOn()
{

	double bw = 0.0;
	bw = (_bandwidth ==
	      BW_125) ? 125e3 : ((_bandwidth == BW_250) ? 250e3 : 500e3);
	// Symbol rate : time for one symbol (secs)
	double rs = bw / (1 << _spreadingFactor);
	double ts = 1 / rs;

	return 0;
}

void SX1272::setCSPin(uint8_t cs)
{
	//need to call this function before the ON() function
	_SX1272_SS = cs;
}

SX1272 sx1272 = SX1272();
