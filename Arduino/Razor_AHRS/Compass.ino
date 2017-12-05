/* This file is part of the Razor AHRS Firmware */

void Compass_Heading()
{
  float mag_x = 0;
  float mag_y = 0;
  float cos_roll = 0;
  float sin_roll = 0;
  float cos_pitch = 0;
  float sin_pitch = 0;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated magnetic field X
  mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x);
}
