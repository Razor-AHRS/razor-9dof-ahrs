/*
  RazorIMU_9DOF.h - Library to deal with Razor 9 DOF IMU
  Copyright (c) 2018 RoboManipal. All right reserved
  File created by : Shashank Goyal
*/

// Guard code to prevent multiple imports
#ifndef IMU_9DOF__H
#define  IMU_9DOF__H

// Main arduino code
#include "Arduino.h"
// Versatile for serial
#include "Stream.h"
// Debugger
#include "DebuggerSerial.h"

// Axis values
#define PITCH 0
#define ROLL 1
#define YAW 2

// Serial method of connection
class RazorIMU_9DOF {
    // Raw values of Pitch, Roll and Yaw as received from the sensors
    float YPR_values[3];
    // Serial line to which the IMU is attached
    Stream *IMU_Serial;
public:
    // ##################   Constructors   ######################
    RazorIMU_9DOF();
    RazorIMU_9DOF(Stream *AttachedSerial);
    // ###############  Public member functions   ################
    // Attach IMU to serial
    void AttachIMUSerial(Stream *AttachedSerial);
    // Parse data from serial (raw) - Receive and store it
    void UpdateData();
    // #############   Retrieving data from object   ############
    float GetRoll();
    float GetPitch();
    float GetYaw();
};

// End of guard code
#endif
