/*
  RazorIMU_9DOF.h - Library to deal with Razor 9 DOF IMU
  Copyright (c) 2018 RoboManipal. All right reserved
  File created by : Shashank Goyal
*/

// Include library
#include "RazorIMU_9DOF.h"

// ################### Constructors #######################
RazorIMU_9DOF::RazorIMU_9DOF() {
	// Empty constructor
}
RazorIMU_9DOF::RazorIMU_9DOF(Stream *AttachedSerial) {
	// Connect IMU Serial
	this->AttachIMUSerial(AttachedSerial);
}

// ############# Member Functions #########################
// Attach Serial
void RazorIMU_9DOF::AttachIMUSerial(Stream *AttachedSerial) {
	// Connect IMU Serial
	this->IMU_Serial = AttachedSerial;
}

// Get raw data from the serial
void RazorIMU_9DOF::UpdateData() 
{
	// Check if Output is available from the IMUSerial
	if(this->IMU_Serial->available())
	{
		// Output Format : #YPR=<YAW>,<PITCH>,<ROLL>
		// Reads the String : #YPR=
		this->IMU_Serial->readStringUntil('=');
		// Parses the Yaw Value and Reads the Comma.
		YPR_values[YAW] = ((this->IMU_Serial->readStringUntil(',')).toFloat());
		// Parses the Pitch Value and Reads the Comma.
		YPR_values[PITCH] = ((this->IMU_Serial->readStringUntil(',')).toFloat());
		// Parses the Roll Value and Reads the Comma.
		YPR_values[ROLL] = ((this->IMU_Serial->readStringUntil('\n')).toFloat());
	}
}

// Getting the values from the Class Object
float RazorIMU_9DOF::GetPitch() {
	// Pitch
	return this->YPR_values[PITCH];
}
float RazorIMU_9DOF::GetRoll() {
	// Roll
	return this->YPR_values[ROLL];
}
float RazorIMU_9DOF::GetYaw() {
	// Yaw
	return this->YPR_values[YAW];
}
