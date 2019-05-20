/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
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

#ifndef __QUARK_PRESS__
#define __QUARK_PRESS__

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

extern char nomenclature_str[4];
void pressure_init();
double pressure_get_value();

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE READ PIN AND THE POWER PIN FOR THE TEMP. SENSOR
#define PIN_READ		A1

// use digital 9 to power the temperature sensor if needed
#define PIN_POWER		7

///////////////////////////////////////////////////////////////////

#if defined ARDUINO_AVR_PRO || defined ARDUINO_AVR_MINI || defined ARDUINO_SAM_DUE
  // if you have a Pro Mini running at 5V, then change here
  // these boards work in 3.3V
  // Nexus board from Ideetron is a Mini
  // __MK66FX1M0__ is for Teensy36
  // __MK64FX512__  is for Teensy35
  // __MK20DX256__ is for Teensy31/32
  // __MKL26Z64__ is for TeensyLC
  // __SAMD21G18A__ is for Zero/M0 and FeatherM0 (Cortex-M0)
#define VOLTAGE_SCALE  3300.0
#else				// ARDUINO_AVR_NANO || defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MEGA2560
  // also for all other boards, so change here if required.
#define VOLTAGE_SCALE  5000.0
#endif

#endif
