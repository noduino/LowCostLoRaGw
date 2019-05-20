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

#include "pressure.h"

void pressure_init()
{
	// for the temperature sensor
	pinMode(PIN_READ, INPUT);
	pinMode(PIN_POWER, OUTPUT);
}

double pressure_get_value()
{
	//read the raw sensor value
	int value = analogRead(PIN_READ);
	double sensor_value;

	Serial.print(F("Reading "));
	Serial.println(value);

	sensor_value = (value * VOLTAGE_SCALE / 1024.0) / 10;

	return sensor_value;
}
