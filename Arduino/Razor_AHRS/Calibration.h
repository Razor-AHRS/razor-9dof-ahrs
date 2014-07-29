/*
 * Calibration.h -- AHRS EEPROM-based calibration.
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
 * ************************* OVERVIEW *************************
 *
 * Compiling and installing AHRS on the Razor, just to set calibration values,
 * is a pain.
 *
 * Using the CalibrationValues class defined here, a global Calibration instance
 * is available, which manages the various sensor calibration values and stores them
 * in eeprom.  The main driver was also augmented to allow this class to handle
 * "#c"alibration messages from the serial line, so that you can get or set
 * any of the values at any time.
 *
 * To enable, ensure CALIBRATION_USE_EEPROM is defined below.  Then, connect using
 * the terminal/serial monitor and:
 *
 * 	* #o0 (disable output streaming, so you can see)
 * 	* #cd (dump the current calibration values)
 * 	* #ch (see the calibration help and set any values as required)
 *
 *
 * If you want to leave things as before the eeprom-based calibration, simply leave
 * CALIBRATION_USE_EEPROM undefined and configure all the #defines below, as before.
 *
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_


// CALIBRATION_USE_EEPROM
// Ensure this is defined if you want to be able to
// configure sensor calibration values via the serial
// line terminal.
#define CALIBRATION_USE_EEPROM




/* Values here used when CALIBRATION_USE_EEPROM is NOT defined and as
 * defaults when it is being used.
 */

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -250)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 250)
#define ACCEL_Z_MIN ((float) -250)
#define ACCEL_Z_MAX ((float) 250)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

#ifndef CALIBRATION_USE_EEPROM
// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)
#endif

// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {0, 0, 0};
//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

/*
 // Calibration example:

 // "accel x,y,z (min/max) = -277.00/264.00  -256.00/278.00  -299.00/235.00"
 #define ACCEL_X_MIN ((float) -277)
 #define ACCEL_X_MAX ((float) 264)
 #define ACCEL_Y_MIN ((float) -256)
 #define ACCEL_Y_MAX ((float) 278)
 #define ACCEL_Z_MIN ((float) -299)
 #define ACCEL_Z_MAX ((float) 235)

 // "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
 //#define MAGN_X_MIN ((float) -511)
 //#define MAGN_X_MAX ((float) 581)
 //#define MAGN_Y_MIN ((float) -516)
 //#define MAGN_Y_MAX ((float) 568)
 //#define MAGN_Z_MIN ((float) -489)
 //#define MAGN_Z_MAX ((float) 486)

 // Extended magn
 #define CALIBRATION__MAGN_USE_EXTENDED true
 const float magn_ellipsoid_center[3] = {91.5, -13.5, -48.1};
 const float magn_ellipsoid_transform[3][3] = {{0.902, -0.00354, 0.000636}, {-0.00354, 0.9, -0.00599}, {0.000636, -0.00599, 1}};

 // Extended magn (with Sennheiser HD 485 headphones)
 //#define CALIBRATION__MAGN_USE_EXTENDED true
 //const float magn_ellipsoid_center[3] = {72.3360, 23.0954, 53.6261};
 //const float magn_ellipsoid_transform[3][3] = {{0.879685, 0.000540833, -0.0106054}, {0.000540833, 0.891086, -0.0130338}, {-0.0106054, -0.0130338, 0.997494}};

 //"gyro x,y,z (current/average) = -40.00/-42.05  98.00/96.20  -18.00/-18.36"
 #define GYRO_AVERAGE_OFFSET_X ((float) -42.05)
 #define GYRO_AVERAGE_OFFSET_Y ((float) 96.20)
 #define GYRO_AVERAGE_OFFSET_Z ((float) -18.36)
 */







#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration


#ifdef CALIBRATION_USE_EEPROM

/* EEPROM calibration is enabled! */


// this is the eeprom addy at which will store our structure.
#define CALIBRATION_EEPROM_ADDRESS		100



#define CALIB_STRUCT_DECLARATION	struct  __attribute__ ((__packed__))


// RawValsMinMax -- stuff that has a max and a min.
typedef CALIB_STRUCT_DECLARATION RawValsMinMaxStruct {
	float min;
	float max;
} RawValsMinMax;

// ThreeSpaceMinMax -- x,y and z values with a min and max
typedef CALIB_STRUCT_DECLARATION ThreeSpaceMinMaxStruct {
	RawValsMinMax x;
	RawValsMinMax y;
	RawValsMinMax z;
} ThreeSpaceMinMax;


// ThreeSpaceVals - simple x,y and z related floats
typedef CALIB_STRUCT_DECLARATION ThreeSpaceValsStruct {
	float x;
	float y;
	float z;
} ThreeSpaceVals;


// RawCalibValues -- the data will be storing in eeprom
typedef CALIB_STRUCT_DECLARATION RawCalibValuesStruct {
	char header[5]; // used to verify initialization

	ThreeSpaceMinMax accel; // accelerometer min/maxes
	ThreeSpaceMinMax magn; 	// magnometer min/maxes
	ThreeSpaceVals gyro;	// gyro drifts

	float magn_ellipsoid_center[3]; // more or less supported, as of yet...
	float magn_ellipsoid_transform[3][3];

} RawCalibValues;

typedef char CalibrationRequestCode;


/*
 * CalibrationValues
 *
 * The CalibrationValues type is how we interact with the calibration
 * system.  It provides access to the raw/configured values as well
 * as any derived (calculated) values (like the accel scale, etc.).
 *
 * This class also handles user interaction, through handleRequest()
 * and other methods, so that values may be queried and set via
 * the serial line.
 *
 */
class CalibrationValues {
public:
	CalibrationValues();

	inline float accel_x_min() {
		return calib_values.accel.x.min;
	}
	inline float accel_x_max() {
		return calib_values.accel.x.max;
	}
	inline float accel_y_min() {
		return calib_values.accel.y.min;
	}
	inline float accel_y_max() {
		return calib_values.accel.y.max;
	}
	inline float accel_z_min() {
		return calib_values.accel.z.min;
	}
	inline float accel_z_max() {
		return calib_values.accel.z.max;
	}

	inline float magn_x_min() {
		return calib_values.magn.x.min;
	}
	inline float magn_x_max() {
		return calib_values.magn.x.max;
	}
	inline float magn_y_min() {
		return calib_values.magn.y.min;
	}
	inline float magn_y_max() {
		return calib_values.magn.y.max;
	}
	inline float magn_z_min() {
		return calib_values.magn.z.min;
	}
	inline float magn_z_max() {
		return calib_values.magn.z.max;
	}

	inline float gyro_avg_offset_x() {
		return calib_values.gyro.x;
	}
	inline float gyro_avg_offset_y() {
		return calib_values.gyro.y;
	}
	inline float gyro_avg_offset_z() {
		return calib_values.gyro.z;
	}

	inline float * magn_ellipsoid_center() {
		return calib_values.magn_ellipsoid_center;
	}
	inline float * magn_ellipsoid_transform() {
		return &(calib_values.magn_ellipsoid_transform[0][0]);
	}

	void set_accel_x_min(float setTo) {
		calib_values.accel.x.min = setTo;
	}
	void set_accel_x_max(float setTo) {
		calib_values.accel.x.max = setTo;
	}
	void set_accel_y_min(float setTo) {
		calib_values.accel.y.min = setTo;
	}
	void set_accel_y_max(float setTo) {
		calib_values.accel.y.max = setTo;
	}
	void set_accel_z_min(float setTo) {
		calib_values.accel.z.min = setTo;
	}
	void set_accel_z_max(float setTo) {
		calib_values.accel.z.max = setTo;
	}

	void set_magn_x_min(float setTo) {
		calib_values.magn.x.min = setTo;
	}
	void set_magn_x_max(float setTo) {
		calib_values.magn.x.max = setTo;
	}
	void set_magn_y_min(float setTo) {
		calib_values.magn.y.min = setTo;
	}
	void set_magn_y_max(float setTo) {
		calib_values.magn.y.max = setTo;
	}
	void set_magn_z_min(float setTo) {
		calib_values.magn.z.min = setTo;
	}
	void set_magn_z_max(float setTo) {
		calib_values.magn.z.max = setTo;
	}

	void set_gyro_avg_offset_x(float setTo) {
		calib_values.gyro.x = setTo;
	}
	void set_gyro_avg_offset_y(float setTo) {
		calib_values.gyro.y = setTo;
	}
	void set_gyro_avg_offset_z(float setTo) {
		calib_values.gyro.z = setTo;
	}

	ThreeSpaceVals accel_offset;
	ThreeSpaceVals accel_scale;
	ThreeSpaceVals magn_offset;
	ThreeSpaceVals magn_scale;

	void handleRequest(CalibrationRequestCode code);

	// save()
	// actually store the data -- don't need to
	// call this except if you've modified values
	// pointed to by magn_ellipsoid_center()/magn_ellipsoid_transform()
	void save();

private:
	RawCalibValues calib_values;
	bool timeout_set;

	void load(); // load from eeprom
	void initValues(); // calculate derived values

	void dumpValues(bool includeCalculated); // output current config
	void setValue();
	void getValue();
	void showHelp();


	// valueOfInterest() returns the float the user is getting/setting
	// based on Serial interaction
	float * valueOfInterest();

};

// Calibration is our global calib config instance
extern CalibrationValues Calibration;

// We've kept the same names for everything to minimize impact on the
// rest of the code.  Here, they're simply aliases.

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET Calibration.accel_offset.x
#define ACCEL_Y_OFFSET Calibration.accel_offset.y
#define ACCEL_Z_OFFSET Calibration.accel_offset.z
#define ACCEL_X_SCALE  Calibration.accel_scale.x
#define ACCEL_Y_SCALE  Calibration.accel_scale.y
#define ACCEL_Z_SCALE  Calibration.accel_scale.z

#define MAGN_X_OFFSET  Calibration.magn_offset.x
#define MAGN_Y_OFFSET  Calibration.magn_offset.y
#define MAGN_Z_OFFSET  Calibration.magn_offset.z
#define MAGN_X_SCALE   Calibration.magn_scale.x
#define MAGN_Y_SCALE   Calibration.magn_scale.y
#define MAGN_Z_SCALE   Calibration.magn_scale.z

#define GYRO_AVERAGE_OFFSET_X Calibration.gyro_avg_offset_x()
#define GYRO_AVERAGE_OFFSET_Y Calibration.gyro_avg_offset_y()
#define GYRO_AVERAGE_OFFSET_Z Calibration.gyro_avg_offset_z()

#define MAGN_ELLIPS_CENTER		(Calibration.magn_ellipsoid_center())
#define MAGN_ELLIPS_XFORM		(Calibration.magn_ellipsoid_transform())

#else
// We are *not* using the eeprom calibration system...
// These are the original values used by the code.

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

#define MAGN_ELLIPS_CENTER		magn_ellipsoid_center
#define MAGN_ELLIPS_XFORM		magn_ellipsoid_transform

#endif

#endif /* CALIBRATION_H_ */
