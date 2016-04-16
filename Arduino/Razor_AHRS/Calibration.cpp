/*
 * Calibration.cpp -- AHRS EEPROM-based calibration, implementation.
 * Created on: Jul 28, 2014
 * Copyright (C) 2014 Pat Deegan.
 * Contact: http://flyingcarsandstuff.com/
 *
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *
 */

#include "Calibration.h"

#ifdef CALIBRATION_USE_EEPROM
#include <string.h>
#include <avr/eeprom.h>
#include <Arduino.h>

CalibrationValues Calibration;
extern bool output_errors;
char readChar();

CalibrationValues::CalibrationValues() :
		timeout_set(false) {
	load();
	initValues();

}

void CalibrationValues::save() {

	initValues();
	eeprom_write_block(&(calib_values), (void*) CALIBRATION_EEPROM_ADDRESS,
			sizeof(calib_values));

}

void CalibrationValues::load() {
	char expectedHeader[] = "calib";
	eeprom_read_block(&(calib_values), (void*) CALIBRATION_EEPROM_ADDRESS,
			sizeof(calib_values));

	if (strncmp(calib_values.header, expectedHeader, 5) != 0) {
		// never initialized... do so now.
		strncpy(calib_values.header, expectedHeader, 5);

		calib_values.accel.x.min = ACCEL_X_MIN;
		calib_values.accel.x.max = ACCEL_X_MAX;
		calib_values.accel.y.min = ACCEL_Y_MIN;
		calib_values.accel.y.max = ACCEL_Y_MAX;
		calib_values.accel.z.min = ACCEL_Z_MIN;
		calib_values.accel.z.max = ACCEL_Z_MAX;

		calib_values.magn.x.min = MAGN_X_MIN;
		calib_values.magn.x.max = MAGN_X_MAX;
		calib_values.magn.y.min = MAGN_Y_MIN;
		calib_values.magn.y.max = MAGN_Y_MAX;
		calib_values.magn.z.min = MAGN_Z_MIN;
		calib_values.magn.z.max = MAGN_Z_MAX;

		calib_values.gyro.x = 0.0f;
		calib_values.gyro.y = 0.0f;
		calib_values.gyro.z = 0.0f;

		memset(calib_values.magn_ellipsoid_center, 0, sizeof(float) * 3);
		memset(calib_values.magn_ellipsoid_transform, 0, sizeof(float) * 3 * 3);
		save();

	}

	initValues();

}

void CalibrationValues::initValues() {
	// Sensor calibration scale and offset values, calculated based on stored data
	accel_offset.x = ((calib_values.accel.x.min + calib_values.accel.x.max)
			/ 2.0f);
	accel_offset.y = ((calib_values.accel.y.min + calib_values.accel.y.max)
			/ 2.0f);
	accel_offset.z = ((calib_values.accel.z.min + calib_values.accel.z.max)
			/ 2.0f);

	accel_scale.x = (GRAVITY / (calib_values.accel.x.max - accel_offset.x));
	accel_scale.y = (GRAVITY / (calib_values.accel.y.max - accel_offset.y));
	accel_scale.z = (GRAVITY / (calib_values.accel.z.max - accel_offset.z));

	magn_offset.x =
			((calib_values.magn.x.min + calib_values.magn.x.max) / 2.0f);
	magn_offset.y =
			((calib_values.magn.y.min + calib_values.magn.y.max) / 2.0f);
	magn_offset.z =
			((calib_values.magn.z.min + calib_values.magn.z.max) / 2.0f);

	magn_scale.x = (100.0f / (calib_values.magn.x.max - magn_offset.x));
	magn_scale.y = (100.0f / (calib_values.magn.y.max - magn_offset.y));
	magn_scale.z = (100.0f / (calib_values.magn.z.max - magn_offset.z));



}

void CalibrationValues::handleRequest(CalibrationRequestCode code) {
	switch (code) {
	case 'h':
		// fall-through
	case 'H':
		showHelp();
		break;
	case 'd':
		// dump!
		dumpValues(false);
		break;
	case 'D':
		// dump, with calculated
		dumpValues(true);
		break;
	case 'g':
		getValue();
		break;
	case 's':
		setValue();
		break;
	default:
		Serial.println(F("Unknown request"));
		break;
	}
}

void CalibrationValues::showHelp()
{
	Serial.println(' ');
	Serial.println(' ');
	Serial.println(F("***** Calibration Help ***** "));
	Serial.println(F("All commands prefixed with '#c' followed by:"));
	Serial.println(F("\t* get or set [g/s]"));
	Serial.println(F("\t* accel, magn, or gyro [a/m/g]"));
	Serial.println(F("\t* coord: x,y, or z [x/y/z]"));
	Serial.println(F("For accel and magnetometer:"));
	Serial.println(F("\t* min or max [m/M]"));
	Serial.println(' ');
	Serial.println(F("Examples: "));
	Serial.println(F("Get acceleration x max: \t#cgaxM"));
	Serial.println(F("Set acceleration y min: \t#csaym -244.00"));
	Serial.println(F("Set gyro z:\t\t\t#csgz -39.9"));
	Serial.println(' ');
	Serial.println(F("Current calibration values may be dumped with #cd or #cD (includes calculated values)"));
	Serial.println(' ');


}
void dumpThreeSpaceVals(const ThreeSpaceVals & v)
{

	Serial.print(F("\tx: "));
	Serial.println(v.x, 3);
	Serial.print(F("\ty: "));
	Serial.println(v.y, 3);
	Serial.print(F("\tz: "));
	Serial.println(v.z, 3);


}

void dumpThreeSpaceMinMax(const ThreeSpaceMinMax & mm) {
	Serial.print(F("x:\tmin: "));
	Serial.println(mm.x.min, 3);
	Serial.print(F("\tmax: "));
	Serial.println(mm.x.max, 3);
	Serial.print(F("y:\tmin: "));
	Serial.println(mm.y.min, 3);
	Serial.print(F("\tmax: "));
	Serial.println(mm.y.max, 3);
	Serial.print(F("z:\tmin: "));
	Serial.println(mm.z.min, 3);
	Serial.print(F("\tmax: "));
	Serial.println(mm.z.max, 3);
}
void CalibrationValues::dumpValues(bool includeCalculated) {

	Serial.println(F("*** Current Calibration ***"));
	Serial.println(F("* Accelerometers"));
	dumpThreeSpaceMinMax(calib_values.accel);
	Serial.println(F("* Magnetometers"));
	dumpThreeSpaceMinMax(calib_values.magn);
	Serial.println(F("* Gyros"));
	dumpThreeSpaceVals(calib_values.gyro);

#if CALIBRATION__MAGN_USE_EXTENDED == true

	Serial.println(F("* Ellipsoid Center"));
	Serial.print(F("{ "));
	for (int i=0; i<3; i++)
	{
		Serial.print(calib_values.magn_ellipsoid_center[i], 3);
		Serial.print(' ');
	}
	Serial.println('}');

	Serial.println(F("* Ellipsoid Transform"));
	Serial.println('{');
	for (int i=0; i<3; i++)
	{
		Serial.print(F("  { "));
		for (int j=0; j<3; j++)
		{
			Serial.print(calib_values.magn_ellipsoid_transform[i][j], 3);
			Serial.print(' ');
		}
		Serial.println('}');
	}
	Serial.println('}');
#endif
	if (includeCalculated) {
		Serial.println(F("*** Calculated Values ***"));

		Serial.println(F("* Accel Offsets"));
		dumpThreeSpaceVals(accel_offset);
		Serial.println(F("* Accel Scale"));
		dumpThreeSpaceVals(accel_scale);
		Serial.println(F("* Magn Offsets"));
		dumpThreeSpaceVals(magn_offset);
		Serial.println(F("* Magn Scale"));
		dumpThreeSpaceVals(magn_scale);
	}


}

void CalibrationValues::getValue() {
	float * valOfInterest = valueOfInterest();

	if (valOfInterest == NULL) {
		return;
	}

	Serial.println((*valOfInterest), 3);
	return;

}
void CalibrationValues::setValue() {

	if (!timeout_set) {
		timeout_set = true;
		Serial.setTimeout(10000);
	}
	float * valOfInterest = valueOfInterest();

	if (valOfInterest == NULL) {
		return;
	}

	float newVal = Serial.parseFloat();

	*valOfInterest = newVal;

	if (output_errors)
		Serial.println(F("Saved"));
	save();

}

float * CalibrationValues::valueOfInterest() {
	ThreeSpaceMinMax * mm = NULL;
	ThreeSpaceVals * v = NULL;
	RawValsMinMax * coordMinMax = NULL;

	char type = readChar();
	switch (type) {
	case 'a':
		// accel
		mm = &(calib_values.accel);
		break;
	case 'm':
		mm = &(calib_values.magn);
		break;
	case 'g':
		v = &(calib_values.gyro);
		break;
	default:
		if (output_errors)
			Serial.println(F("Unknown type [a/m/g]"));
		return NULL;
		break;

	}

	char coord = readChar();
	if (v) {
		// simple
		switch (coord) {
		case 'x':
			return &(v->x);
			break;
		case 'y':
			return &(v->y);
			break;
		case 'z':
			return &(v->z);
			break;
		default:
			if (output_errors)
				Serial.println(F("Unknown coord [x/y/z]"));
			return NULL;
			break;
		}
	}

	if (!mm) {
		if (output_errors)
			Serial.println(F("Shouldn't happen"));
		return NULL;
	}

	switch (coord) {
	case 'x':
		coordMinMax = &(mm->x);
		break;
	case 'y':
		coordMinMax = &(mm->y);
		break;
	case 'z':
		coordMinMax = &(mm->z);
		break;
	default:
		if (output_errors)
			Serial.println(F("Unknown coord [x/y/z]"));

		return NULL;
		break;
	}

	char minOrMax = readChar();
	if (minOrMax == 'm') {
		// min
		return &(coordMinMax->min);

	}

	if (minOrMax == 'M') {
		// max
		return &(coordMinMax->max);
	}

	if (output_errors)
		Serial.println(F("Unknows selection [m/M]"));
	return NULL;

}


#endif /* CALIBRATION_USE_EEPROM */


