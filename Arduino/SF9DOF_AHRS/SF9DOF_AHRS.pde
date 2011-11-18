/******************************************************************
* Sparkfun 9DOF Razor IMU AHRS
* 9 Degree of Measurement Attitude and Heading Reference System
*
* Version 1.3
* Released under GNU LGPL License
*
* TODO: http://dev.qu.tu-berlin.de/.......
*
* History:
* - Original code by Doug Weibel and Jose Julio, based on ArduIMU v1.5 by Jordi Munoz 
    and William Premerlani, Jose Julio and Doug Weibel
* - Updated by David Malik (david.zsolt.malik@gmail.com) for new Sparkfun 9DOF HardWare (SEN-10125)
* - Updated and extended by Peter Bartz (peter-bartz@gmx.de)
*       - Cleaned up, streamlined and restructured most of the code to make it more comprehensible
* 	- Added sensor calibration (improves accuracy a lot!)
* 	- Added binary roll/pitch/yaw output and support to synch output stream
* 	- Added support to connect to Rovering Networks Bluetooth modules (and compatible)
*       - TODO: add support for SEN-10736
*       - TODO: maybe gyro works better with lower filter bandwidth (DLPF_CF) and sample rate (SMPLRT_DIV)? Ask Sascha...
*       - TODO: use self-calibration and temperature-compensation features of the sensors
*
* TODO: Put calibration values into eeprom instead of hardcoding them using #define
******************************************************************/

/*
  Razor 9DOF Hardware version - SEN-10125 and SEN-10736

  ATMega328@3.3V w/ external 8MHz resonator
  
  High Fuse DA
  Low Fuse FF

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10125
  HMC5883L : Magnetometer on SEN-10736
  ITG-3200 : Gyro

  Programmer  : 3.3v FTDI
  Arduino IDE : Select board "Arduino Pro or Pro Mini (3.3v, 8mhz) w/ATmega328"
*/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (to the FTDI connector)
    Y axis pointing to the right
    and Z axis pointing down.
  Positive pitch : nose up
  Positive roll  : right wing down
  Positive yaw   : clockwise
*/

#include <Wire.h>








/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/

// Select your Razor 9DOF hardware version here by uncommenting one line!
#define HW__RAZOR_VERSION 10125   // Meaning "SEN-10125", which uses HMC5843 magnetometer
//#define HW__RAZOR_VERSION 10736 // Meaning "SEN-10736", which uses HMC5883L magnetometer


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
#define OUTPUT__MODE_ANGLES_TEXT 1 // Outputs roll/pitch/yaw in degrees as text
#define OUTPUT__MODE_ANGLES_BINARY 2 // Outputs roll/pitch/yaw in degrees as binary float
#define OUTPUT__MODE_SENSORS_TEXT 3 // Outputs (calibrated) sensor values for all 9 axes as text
// Select your startup output mode here!
//int output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
int output_mode = OUTPUT__MODE_ANGLES_TEXT;

// Bluetooth
// Turn this on, if you have a Rovering Networks Bluetooth Module attached.
// The connect/disconnect message prefix of the module has to be set to "#".
// (Refer to manual, it can be set like this: SO,#)
// Using this has two effects:
//  1. The status LED will reflect the connection status (on/off)
//  2. Output of roll/pitch/yaw will only happen as long as we're connected.
//     That way it's very easy to synchronize receiver and sender. 
//#define OUTPUT__HAS_RN_BLUETOOTH


// Sensor calibration values
/*****************************************************************/
// Put MIN/MAX and OFFSET readings for your board here!
// How to calibrate? Read the wiki: TODO
#define ACCEL_X_MIN ((float) -250)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 250)
#define ACCEL_Z_MIN ((float) -250)
#define ACCEL_Z_MAX ((float) 250)

#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)

/*
// Calibration example: 
#define ACCEL_X_MIN ((float) -270)
#define ACCEL_X_MAX ((float) 266)
#define ACCEL_Y_MIN ((float) -251)
#define ACCEL_Y_MAX ((float) 277)
#define ACCEL_Z_MIN ((float) -296)
#define ACCEL_Z_MAX ((float) 231)

#define MAGN_X_MIN ((float) -564)
#define MAGN_X_MAX ((float) 656)
#define MAGN_Y_MIN ((float) -585)
#define MAGN_Y_MAX ((float) 635)
#define MAGN_Z_MIN ((float) -550)
#define MAGN_Z_MAX ((float) 564)

#define GYRO_AVERAGE_OFFSET_X ((float) -28.8)
#define GYRO_AVERAGE_OFFSET_Y ((float) 100.6)
#define GYRO_AVERAGE_OFFSET_Z ((float) -5.9)
*/


// DEBUG OPTIONS
/*****************************************************************/
// When uncommented, gyro drift correction will not be applied
//#define DEBUG__NO_DRIFT_CORRECTION

/*****************************************************************/
/****************** END OF USER SETUP AREA!  ********************/
/*****************************************************************/







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
#define MAGN_X_SCALE (10000.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (10000.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (10000.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


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
#define GRAVITY 256.0f // Arbitrary value as "1G reference" used for accelerometer calibration
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
float roll;
float pitch;
float yaw;

// DCM timing in the main loop
long timestamp;
long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// Output-state variables
boolean bluetooth_connected = false;
int curr_calibration_sensor = 1;

// Read every sensor and record a time stamp
// TODO re-init global vars?
void init_sensor_fusion() {
  Read_Gyro();
  Read_Magn();
  Read_Accel();
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

void new_calibration_session()
{
  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
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
  Read_Accel();
  Compass_Init();
  Gyro_Init();
  delay(20);
  
  init_sensor_fusion();

  // Init calibration values
  new_calibration_session();

  compensate_sensor_errors();
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
      if (command == 's') // _s_ynch request
      {
        // Send synch message
        Serial.println();  // Output on new line
        Serial.println("!SYNCH");
      }
      else if (command == 'n') // Calibrate _n_ext sensor
      {
        curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
        new_calibration_session();
      }
      else if (command == 'o') // Set _o_utput mode
      {
        while (Serial.available() < 1) { } // Wait for another byte to arrive
        int mode = Serial.read();
        if (mode == 'n')  // _n_ext output mode
          output_mode = (output_mode + 1) % 4;
        else if (mode == 't') // Output angles as _t_ext
          output_mode = OUTPUT__MODE_ANGLES_TEXT;
        else if (mode == 'b') // Output angles in _b_inary form
          output_mode = OUTPUT__MODE_ANGLES_BINARY;
        else if (mode == 'c') // Go to _c_alibration mode
        {
          output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
          new_calibration_session();
        }
        else if (mode == 's') // Output _s_ensor values as text
          output_mode = OUTPUT__MODE_SENSORS_TEXT;
      }
#ifdef OUTPUT__HAS_RN_BLUETOOTH
      // Read messages from bluetooth module
      // For this to work, the connect/disconnect message prefix of the module has to be set to "#".
      else if (command == 'C') // Bluetooth "#CONNECT" message
      {
        bluetooth_connected = true;
        digitalWrite(STATUS_LED_PIN, HIGH);
        init_sensor_fusion();
      }
      else if (command == 'D') // Bluetooth "#DISCONNECT" message
      {
        bluetooth_connected = false;
        digitalWrite(STATUS_LED_PIN, LOW);
      }
#endif // OUTPUT__HAS_RN_BLUETOOTH
    }
    else
    { } // Skip character
  }


#ifdef OUTPUT__HAS_RN_BLUETOOTH
  // Do not read sensors and send data if bluetooth not connected
  if (!bluetooth_connected) {
    delay(10);
    return;
  }
#endif // OUTPUT__HAS_RN_BLUETOOTH


  // Time to read the sensors again?
  if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    Read_Gyro(); // Read gyroscope
    Read_Accel(); // Read accelerometer
    Read_Magn(); // Read magnetometer

    if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
    {
      output_calibration(curr_calibration_sensor);
    }
    else if (output_mode == OUTPUT__MODE_SENSORS_TEXT)
    {
      // Apply sensor calibration
      compensate_sensor_errors();
      
      output_sensors();
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
      
      output_angles();
    }
  }
}


