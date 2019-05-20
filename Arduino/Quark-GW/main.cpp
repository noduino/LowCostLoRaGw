/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
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
#include <Ethernet.h>
#include <SPIFlash.h>
#include <EEPROM.h>
#include <DES.h>
#include <TextFinder.h>

#include <avr/wdt.h>

//#define		DEBUG_SERVER
#define		DEBUG
//#define		DEBUG_EEPROM
//#define		DEBUG_TOKEN
#define		NETCORE_ROUTER_FIXUP
#define		NO_DHCP_ROBUST
//#define		WITH_FLASH

#define		TRYNUM	5
#define		FW_VER	"1.0.0"

byte mac[7];
byte dkey[9];
byte uuid[25];
byte token[25];

IPAddress ip(192,168,1,3);
IPAddress gw_ip5(10,0,0,2);
IPAddress ip9(10,0,0,254);

char cos_serv[] = "api.noduino.org";
EthernetClient client;

TextFinder finder(client);

String body = "";

// same sec message interval
const uint16_t msg_group_len = 2000;

unsigned long last_post_time = 0;

int push_data(uint64_t data, char serv[]);
int check_ctrl(char serv[]);
void process_cmd(int ch, int proto, int clk, int bitlen, uint64_t value);
byte wan_ok();
#ifdef NO_DHCP_ROBUST
byte try_static_ip();
#endif

#ifdef WITH_FLASH
SPIFlash flash(5, 0xEF40);
byte buf[8];
#endif

// get the uuid, key and mac from eeprom
void init_devid()
{
	int i;
	// read the uuid
	for (i=0; i<19; i++) {
		uuid[i] = EEPROM.read(0x10 + i);
	}
	uuid[19] = 0x0;

	// read the key
	for (i=0; i<8; i++) {
		dkey[i] = EEPROM.read(0x23 + i);
	}
	dkey[8] = 0;

	// read the mac
	for (i=0; i<6; i++) {
		mac[i] = EEPROM.read(0x30 + i);
	}
}

DES des;

void gen_token(byte *out, byte *uuid, byte *key)
{
	unsigned long seed = millis();
	uuid[20] = seed & 0xff;
	uuid[21] = (seed >> 8) & 0xff;
	uuid[22] = (seed >> 16) & 0xff;
	uuid[23] = (seed >> 24) & 0xff;
#ifdef DEBUG_TOKEN
	Serial.printf("seed = 0x%08X\n", seed);
#endif
	des.encrypt(out+16, uuid, key);
	des.encrypt(out+8, uuid+8, key);
	des.encrypt(out, uuid+16, key);
}

// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
void setup_wdt(int i)
{
	byte bb;

	if (i > 9) i = 9;

	bb = i & 7;

	if (i > 7) bb |= (1 << 5);
	bb |= (1<<WDCE);

	noInterrupts();
	MCUSR &= ~(1<<WDRF);
	// start timed sequence
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	// set new watchdog timeout value
	WDTCSR = bb;
	WDTCSR |= _BV(WDIE);
	interrupts();
}

uint8_t wdt_off()
{
	noInterrupts();
	wdt_reset();

	// clear WDRF in MCUSR
	MCUSR &= ~(1<<WDRF);
	// write logical one to WDCE and WDE
	// keep old prescaler setting to prevent unintentional time-out
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	// turn off wdt
	WDTCSR = 0x00;

	interrupts();
	return 1;
}

void chip_reset()
{
	//wdt_enable(WDTO_250MS);
	setup_wdt(3);
	while(1);
}

void setup() {
	// disable the watchdog
	// wdt_disable();

#ifdef DEBUG
	Serial.begin(9600);
	Serial.println("Power On now!");
#endif

#ifdef WITH_FLASH
	// Pull up pin D5, this is CS signal (active LOW) of SPI flash
	pinMode(5, OUTPUT);
	digitalWrite(5, HIGH);
#endif

	// We use D5 as the power switch of CH1 TX (315)
	// HIGH to disable the power of CH1 TX
	pinMode(5, OUTPUT);
	digitalWrite(5, HIGH);

#ifdef WITH_FLASH
	// Init SPI flash
	if (flash.initialize()) {
		Serial.println("SPI Flash init OK");

		for(int i=0; i<8; i++) {
			Serial.print(flash.readByte(i), HEX);
		}
		Serial.println();

		byte xx[4] = "xxx";
		flash.blockErase4K(0);
		while(flash.busy());
		flash.writeBytes(0, xx, 3);
		while(flash.busy());
		Serial.println("SPI Write Done");

	} else {
		Serial.println("SPI Flash Init failed");
	}

#ifdef DEBUG
	flash.readBytes(0, buf, 8);
	for(int i=0; i<8; i++) {
		Serial.print(buf[i], HEX);
	}
	Serial.println();
#endif
#endif // FLASH

	init_devid();
#ifdef DEBUG_EEPROM
	Serial.printf("uuid = %s\n", (char *)uuid);
	Serial.printf("dkey = %s\n", (char *)dkey);
	Serial.print("mac = ");
	for(int i=0; i<6; i++) {
		Serial.printf("%02X ", mac[i]);
	}
	Serial.println();
#endif

	body.reserve(100);

	setup_wdt(9);
	// start the Ethernet connection:
	if (Ethernet.begin(mac) == 0) {
#ifdef DEBUG
		Serial.println("Failed to configure Ethernet using DHCP");
#endif
		// DHCP failed, so try a fixed IP
		// If static ip failed, it's network issue
		// waitting for the network is OK
#ifdef NO_DHCP_ROBUST
		if (try_static_ip() == 0) {
#endif
			chip_reset();
#ifdef NO_DHCP_ROBUST
		}
#endif
	}

	// setup ip must be in 8s otherwise chip would be reset
	wdt_reset();

#ifdef DEBUG
	Serial.print("My address:");
	Serial.println(Ethernet.localIP());
#endif

	// push a message to say "I'm online"
#ifdef NETCORE_ROUTER_FIXUP
	if(wan_ok() == 0) {
#ifdef DEBUG
		Serial.println("wan is offline");
#endif
	}
#endif
	body = "{";
	push_data(0x0, cos_serv);

	wdt_reset();
}

#ifdef NO_DHCP_ROBUST
byte try_static_ip()
{
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	ip[3] = 254;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	ip[3] = 111;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	// Try 192.168.0.111/254/3
	ip[2] = 0;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	ip[3] = 254;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	ip[3] = 3;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	// Try 192.168.10.3/111/254
	ip[2] = 10;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}
	ip[3] = 111;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}
	ip[3] = 254;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}

	// Try 192.168.18.3/111/254
	ip[2] = 18;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}
	ip[3] = 3;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}
	ip[3] = 111;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}

	// Try 10.0.0.254
	Ethernet.begin(mac, ip9, gw_ip5, gw_ip5);
	if (wan_ok()) {
		return 1;
	}

	return 0;
}
#endif

byte wan_ok()
{
	wdt_off();
	if (client.connect(cos_serv, 998)) {
		client.stop();
		setup_wdt(9);
		return 1;
	} else {
		client.stop();
		setup_wdt(9);
		return 0;
	}
}

uint64_t old_val = 0;
int rx_ch, rx_proto, rx_delay, rx_bitlen;

void loop() {

	int try_num = 0;
	wdt_reset();

	//int rx_flag = radio_available(0);
	bool rx_flag = true;

	if (rx_flag) {

		//uint64_t radio0_new_val = fetch_rx_data();
		uint64_t radio0_new_val = 0x55aa;

		rx_ch = 0;
		//rx_proto = radio_get_rx_proto(0);
		//rx_bitlen = radio_get_rx_bitlen(0);

		// begin new monitor
		//radio_resetAvailable(0);

		// Same sec event (door open) need interval 2s
		if((radio0_new_val != old_val) ||
			(millis() - last_post_time > msg_group_len)) {
			// It's a new msg group, mark it with a name
			old_val = radio0_new_val;

			// make sure push the data success
			try_num = 0;
#ifdef NETCORE_ROUTER_FIXUP
			if(wan_ok() == 0) {
#ifdef DEBUG
				Serial.println("wan is offline");
#endif
			}
			wdt_reset();
#endif
			while (push_data(radio0_new_val, cos_serv) == -1) {
				wdt_reset();
				try_num++;
#ifdef DEBUG
				Serial.print("pushed data failed. Try num:");
				Serial.println(try_num);
#endif
				if (try_num >= TRYNUM) break;
				delay(1200);		// delay 1.2s
#ifdef DEBUG
				Serial.println("try next");
#endif
			}
		}
	}


	//check_ctrl(cos_serv);
	delay(1100);
}

// this method makes a HTTP connection to the server:
//int push_data(uint32_t data, IPAddress &serv) {
int push_data(uint64_t data, char serv[]) {

	if (data != 0) {
		char data_buf[17];
		sprintf(data_buf, "%04X", (data>>48) & 0xffff);
		sprintf(data_buf+4, "%04X", (data>>32) & 0xffff);
		sprintf(data_buf+8, "%04X", (data>>16) & 0xffff);
		sprintf(data_buf+12, "%04X", (data) & 0xffff);

		body = "{\"meta\":\"";
		body += rx_ch;
		body += ",";
		body += rx_proto;
		body += ",";
		body += rx_delay;
		body += ",";
		body += rx_bitlen;
		//ch, proto, delay, bitlen (,value)

		body += "\",\"value\":\"";

		body += data_buf;
	} else {
		body = "{\"value\":\"0";
	}

	body += "\",\"fwver\":\"";
	body += FW_VER;
	body += "\"}";

#ifdef	DEBUG
	Serial.println("Try push data");
#endif
	// if there's a successful connection:
	if (!client.connected() && wdt_off() && client.connect(serv, 998)) {
		setup_wdt(9);
#ifdef	DEBUG
		Serial.println("Connecting to Cloud ...");
#endif
		gen_token(token, uuid, dkey);

		// Send the HTTP PUT request:
		client.println("POST /v2/node/state HTTP/1.1");
		client.println("Accept: */ *");

		client.print("nodid: ");
		for(int i = 0; i < 19; i++)
			client.printf("%c", uuid[i]);
		client.println();

		client.print("token: ");
		for(int i = 0; i < 24; i++)
			client.printf("%02X", token[i]);
		client.println();

		client.print("Content-Length: ");
		// calculate the length of the sensor reading in bytes:
		client.println(body.length());

		// last pieces of the HTTP PUT request:
		client.println("Content-Type: text/html");
		client.println("Connection: close");
		client.println();

		// here's the actual content of the PUT request:
		client.println(body);
		client.println();
#ifdef DEBUG
#ifdef DEBUG_SERVER
		Serial.println(body);
#endif
		Serial.println("Messages pushed.");
#endif
		// note the time that the connection was made or attempted:
		last_post_time = millis();

		wdt_reset();

		if(client.connected()) {

			wdt_off();

			if (finder.findUntil("200 OK", "\n\r")) {
				setup_wdt(9);
			#ifdef DEBUG
				Serial.println("Server Response OK");
			#endif
				client.stop();
				return 0;
			} else {
				setup_wdt(9);
			#ifdef DEBUG
				Serial.println("Server Response Failed!");
			#endif
				client.stop();
				return -1;
			}
		} else {
			#ifdef DEBUG
				Serial.println("Can't get the server response. Client is not Connected!");
			#endif
				client.stop();
				return -1;
		}

	} else {
#ifdef DEBUG
		// if you couldn't make a connection:
		Serial.println("Connection failed");
#endif
		setup_wdt(9);
		client.stop();
		return -1;
	}
}

int check_ctrl(char serv[]) {
	if (!client.connected() && wdt_off() && client.connect(serv, 998)) {
		setup_wdt(9);
		gen_token(token, uuid, dkey);
		client.println("GET /v2/node/ctrl HTTP/1.1");
		client.println("Accept: */ *");
		client.print("nodid: ");
		for(int i = 0; i < 19; i++)
			client.printf("%c", uuid[i]);
		client.println();
		client.print("token: ");

		for(int i = 0; i < 24; i++)
			client.printf("%02X", token[i]);
		client.println();
		client.println("Content-Length: 2");
		client.println("Content-Type: text/html");
		client.println("Connection: close");
		client.println();
		client.println("{}");
		client.println();

		TextFinder find(client);
		int nloop = 0;
		int ch, proto, clk, bitlen;
		uint64_t value;
		while(client.connected()) {

			if (find.find("meta")) {
				ch = find.getValue();
				proto = find.getValue();
				clk = find.getValue();
				bitlen = find.getValue();
				value = find.getValue64();
				client.stop();
#ifdef DEBUG
				char data_buf[17];
				sprintf(data_buf, "%04X", (value>>48) & 0xffff);
				sprintf(data_buf+4, "%04X", (value>>32) & 0xffff);
				sprintf(data_buf+8, "%04X", (value>>16) & 0xffff);
				sprintf(data_buf+12, "%04X", (value) & 0xffff);

				Serial.println(ch);
				Serial.println(proto);
				Serial.println(clk);
				Serial.println(bitlen);

				Serial.println(data_buf);
#endif
				process_cmd(ch, proto, clk, bitlen, value);
				return 0;
			} else {
				client.stop();
				return -1;
			}

			delay(1);
			nloop++;
			if (nloop > 5000) {
				client.stop();
				return -1;
			}
		}
	}
	client.stop();
}

void switch_ch0_tx()
{
	// enable the tx433 power
	// and enable the TX ANT
	digitalWrite(A3, HIGH);

	//radio_disable_rx(1);
	//radio_enable_tx(A4);

	delay(300);
}

void switch_ch0_rx()
{

	// disable the tx433 power
	digitalWrite(A3, LOW);

	// Receiver on inerrupt 0 => that is pin #2
	//radio_enable_rx(0);

	// Receiver on inerrupt 1 => that is pin #3
	//radio_enable_rx(1);
}

void enable_ch1_tx()
{
	// enable the tx315 power
	digitalWrite(5, LOW);

	//radio_disable_rx(0);
	//radio_disable_rx(1);
	//radio_enable_tx(A5);

	delay(300);
}

void disable_ch1_tx()
{
	// disable the tx315 power
	digitalWrite(5, HIGH);
}

void process_cmd(int ch, int proto, int clk, int bitlen, uint64_t value) {

	// disable the interrupts
	noInterrupts();

	if (ch == 0) {
		// 433 tx
		switch_ch0_tx();

		//radio_send64(value, bitlen);

		switch_ch0_rx();
	} else if (ch == 1) {
		// 315 tx
		enable_ch1_tx();

		//radio_send64(value, bitlen);

		disable_ch1_tx();
	}

	// enable the interrupts
	interrupts();
}
