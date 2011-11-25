/***************************************************************************************************************
* Razor AHRS Firmware v1.3.0
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun 9DOF Razor IMU (SEN-10125 and SEN-10736)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
*
*
* History:
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
*
*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update/) by David Malik (david.zsolt.malik@gmail.com)
*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
*
*   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
*       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
*       * Added sensor calibration (improves precision and responsiveness a lot!).
*       * Added binary yaw/pitch/roll output.
*       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
*       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
*       * Wrote new easier to use test program (using Processing).
*       * Added support for new version of "9DOF Razor IMU": SEN-10736.
*       --> The output of this code is not compatible with the older versions!
*       --> A Processing sketch to test the tracker is available.
*
* TODOs:
*   * Put calibration values into EEPROM instead of hardcoding them using #define.
*   * Use self-test and temperature-compensation features of the sensors.
*   * Test if gyro works better with lower filter bandwidth (DLPF_CF) and/or lower sample rate (SMPLRT_DIV).
*   * Runtime hardware version detection.
***************************************************************************************************************/

/*
  9DOF Razor IMU hardware version - SEN-10125 and SEN-10736

  ATMega328@3.3V, 8MHz

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10125
  HMC5883L : Magnetometer on SEN-10736
  ITG-3200 : Gyro

  Programmer  : 3.3v FTDI
  Arduino IDE : Select board "Arduino Pro or Pro Mini (3.3v, 8Mhz) w/ATmega328"
*/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (to the FTDI connector)
    Y axis pointing to the right
    and Z axis pointing down.
    
  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up
  
  Transformation order: first yaw then pitch then roll.
*/

/*
  Commands that the firmware understands:
  
  "#o<param>" - Set output parameter. The available options are:
      "#o0" - Disable continuous streaming output.
      "#o1" - Enable continuous streaming output.
      "#ob" - Output angles in binary format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in text format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).
      "#os" - Output (calibrated) sensor values for all 9 axes. One frame consist of
              three lines - one for each sensor.
      "#oc" - Go to calibration output mode.
      "#on" - When in calibration mode, go on to calibrate next sensor.
    
  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only.
  "#s" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present when streaming is off. The tracker will
         send "#SYNCH\r\n" in response (so it's possible to read using a readLine() function).
          
  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)
  
  Newline character are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.
*/



/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your Razor 9DOF hardware version here by uncommenting one line!
//#define HW__RAZOR_VERSION 10125   // Meaning SparkFun "SEN-10125", which uses HMC5843 magnetometer
//#define HW__RAZOR_VERSION 10736 // Meaning SparkFun "SEN-10736", which uses HMC5883L magnetometer


// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 57600

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Output mode
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES_TEXT 1 // Outputs yaw/pitch/roll in degrees as text
#define OUTPUT__MODE_ANGLES_BINARY 2 // Outputs yaw/pitch/roll in degrees as binary float
#define OUTPUT__MODE_SENSORS_TEXT 3 // Outputs (calibrated) sensor values for all 9 axes as text
// Select your startup output mode here!
int output_mode = OUTPUT__MODE_ANGLES_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

// Bluetooth
// Set this to true, if you have a Rovering Networks Bluetooth Module attached.
// The connect/disconnect message prefix of the module has to be set to "#".
// (Refer to manual, it can be set like this: SO,#)
// Using this has two effects:
//  1. The status LED will reflect the connection status (on/off)
//  2. Output via serial port will only happen as long as we're connected.
//     That way it's very easy to synchronize receiver and sender just by connecting/disconnecting.
// NOTE: When using this, OUTPUT__STARTUP_STREAM_ON has no effect!
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false


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

// Magnetometer
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

// Gyroscpe
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)

/*
// Calibration example:
// "accel x,y,z (min/max) = -270.00/266.00  -251.00/277.00  -296.00/231.00"
#define ACCEL_X_MIN ((float) -270)
#define ACCEL_X_MAX ((float) 266)
#define ACCEL_Y_MIN ((float) -251)
#define ACCEL_Y_MAX ((float) 277)
#define ACCEL_Z_MIN ((float) -296)
#define ACCEL_Z_MAX ((float) 231)

// "magn x,y,z (min/max) = -564.00/656.00  -585.00/635.00  -550.00/564.00"
#define MAGN_X_MIN ((float) -564)
#define MAGN_X_MAX ((float) 656)
#define MAGN_Y_MIN ((float) -585)
#define MAGN_Y_MAX ((float) 635)
#define MAGN_Z_MIN ((float) -550)
#define MAGN_Z_MAX ((float) 564)

//"gyro x,y,z (current/average) = -29.00/-27.98  102.00/100.51  -5.00/-5.85"
#define GYRO_AVERAGE_OFFSET_X ((float) -27.98)
#define GYRO_AVERAGE_OFFSET_Y ((float) 100.51)
#define GYRO_AVERAGE_OFFSET_Z ((float) -5.85)
*/


// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/










// Check if hardware version is defined
#ifndef HW__RAZOR_VERSION
  // Generate compile error
  #error YOU HAVE TO DEFINE YOUR RAZOR HARDWARE VERSION! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.pde!
#endif

#include <Wire.h>

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


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Sensor variables
float accel[3];
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

int sensor_sign[9] = {-1, -1, -1, 1, 1, 1, -1, -1, -1};  // Correct directions x, y, z - gyros, accels, magnetometer

// DCM variables
float MAG_Heading;
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
long timestamp;
long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// Output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// TODO re-init global vars?
void reset_sensor_fusion() {
  delay(20);
  read_sensors();
  timestamp = millis();
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Called if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;
  
  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
  
  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

void setup()
{
  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);
  
  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  delay(20);
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  delay(20);
  reset_sensor_fusion();

  // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif
}

// Main loop
void loop()
{
  // Read incoming control messages
  if (Serial.available() >= 2)
  {
    if (Serial.read() == '#') // Start of new control message
    {
      int command = Serial.read(); // Commands
      if (command == 'f') // request one output _f_rame
        output_single_on = true;
      else if (command == 's') // _s_ynch request
      {
        // Send synch message
        Serial.println("#SYNCH");
      }
      else if (command == 'o') // Set _o_utput mode
      {
        while (Serial.available() < 1) { } // Wait for another byte to arrive
        int output_param = Serial.read();
        if (output_param == 'n')  // Calibrate _n_ext sensor
        {
          curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 't') // Output angles as _t_ext
          output_mode = OUTPUT__MODE_ANGLES_TEXT;
        else if (output_param == 'b') // Output angles in _b_inary form
          output_mode = OUTPUT__MODE_ANGLES_BINARY;
        else if (output_param == 'c') // Go to _c_alibration mode
        {
          output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 's') // Output _s_ensor values as text
          output_mode = OUTPUT__MODE_SENSORS_TEXT;
        else if (output_param == '0')
        {
          turn_output_stream_off();
          reset_calibration_session_flag = true;
        }
        else if (output_param == '1')
        {
          reset_calibration_session_flag = true;
          turn_output_stream_on();
        }
      }
#if OUTPUT__HAS_RN_BLUETOOTH == true
      // Read messages from bluetooth module
      // For this to work, the connect/disconnect message prefix of the module has to be set to "#".
      else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
        turn_output_on();
      else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
        turn_output_off();
#endif // OUTPUT__HAS_RN_BLUETOOTH == true
    }
    else
    { } // Skip character
  }

  // Time to read the sensors again?
  if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();

    if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
    {
      check_reset_calibration_session();  // Check if this session needs a reset
      if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
    }
    else if (output_mode == OUTPUT__MODE_SENSORS_TEXT)
    {
      // Apply sensor calibration
      compensate_sensor_errors();
      
      if (output_stream_on || output_single_on) output_sensors();
    }
    else
    {
      // Apply sensor calibration
      compensate_sensor_errors();
    
      // Run DCM algorithm
      Compass_Heading(); // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();
      
      if (output_stream_on || output_single_on) output_angles();
    }
    
    output_single_on = false;
  }
}
