/* This file is part of the Razor AHRS Firmware */

// Output angles: yaw, pitch, roll
void output_angles()
{
  if (output_format == OUTPUT__FORMAT_BINARY)
  {
    float ypr[3];  
    ypr[0] = TO_DEG(yaw);
    ypr[1] = TO_DEG(pitch);
    ypr[2] = TO_DEG(roll);
    LOG_PORT.write((byte*) ypr, 12);  // No new-line
  }
  else if (output_format == OUTPUT__FORMAT_TEXT)
  {
    LOG_PORT.print("#YPR=");
    LOG_PORT.print(TO_DEG(yaw)); LOG_PORT.print(",");
    LOG_PORT.print(TO_DEG(pitch)); LOG_PORT.print(",");
    LOG_PORT.print(TO_DEG(roll)); LOG_PORT.println();
  }
}

void output_calibration(int calibration_sensor)
{
  if (calibration_sensor == 0)  // Accelerometer
  {
    // Output MIN/MAX values
    LOG_PORT.print("accel x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
      if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
      LOG_PORT.print(accel_min[i]);
      LOG_PORT.print("/");
      LOG_PORT.print(accel_max[i]);
      if (i < 2) LOG_PORT.print("  ");
      else LOG_PORT.println();
    }
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {
    // Output MIN/MAX values
    LOG_PORT.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
      if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
      LOG_PORT.print(magnetom_min[i]);
      LOG_PORT.print("/");
      LOG_PORT.print(magnetom_max[i]);
      if (i < 2) LOG_PORT.print("  ");
      else LOG_PORT.println();
    }
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {
    // Average gyro values
    for (int i = 0; i < 3; i++)
      gyro_average[i] += gyro[i];
    gyro_num_samples++;
      
    // Output current and averaged gyroscope values
    LOG_PORT.print("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      LOG_PORT.print(gyro[i]);
      LOG_PORT.print("/");
      LOG_PORT.print(gyro_average[i] / (float) gyro_num_samples);
      if (i < 2) LOG_PORT.print("  ");
      else LOG_PORT.println();
    }
  }
}

void output_sensors_text(char raw_or_calibrated)
{
  LOG_PORT.print("#A-"); LOG_PORT.print(raw_or_calibrated); LOG_PORT.print('=');
  LOG_PORT.print(accel[0]); LOG_PORT.print(",");
  LOG_PORT.print(accel[1]); LOG_PORT.print(",");
  LOG_PORT.print(accel[2]); LOG_PORT.println();

  LOG_PORT.print("#M-"); LOG_PORT.print(raw_or_calibrated); LOG_PORT.print('=');
  LOG_PORT.print(magnetom[0]); LOG_PORT.print(",");
  LOG_PORT.print(magnetom[1]); LOG_PORT.print(",");
  LOG_PORT.print(magnetom[2]); LOG_PORT.println();

  LOG_PORT.print("#G-"); LOG_PORT.print(raw_or_calibrated); LOG_PORT.print('=');
  LOG_PORT.print(gyro[0]); LOG_PORT.print(",");
  LOG_PORT.print(gyro[1]); LOG_PORT.print(",");
  LOG_PORT.print(gyro[2]); LOG_PORT.println();
}

void output_both_angles_and_sensors_text()
{
  LOG_PORT.print("#YPRAG=");
  LOG_PORT.print(TO_DEG(yaw)); LOG_PORT.print(",");
  LOG_PORT.print(TO_DEG(pitch)); LOG_PORT.print(",");
  LOG_PORT.print(TO_DEG(roll)); LOG_PORT.print(",");
  
  LOG_PORT.print(Accel_Vector[0]); LOG_PORT.print(",");
  LOG_PORT.print(Accel_Vector[1]); LOG_PORT.print(",");
  LOG_PORT.print(Accel_Vector[2]); LOG_PORT.print(",");

  LOG_PORT.print(Gyro_Vector[0]); LOG_PORT.print(",");
  LOG_PORT.print(Gyro_Vector[1]); LOG_PORT.print(",");
  LOG_PORT.print(Gyro_Vector[2]); LOG_PORT.println();
}

void output_sensors_binary()
{
  LOG_PORT.write((byte*) accel, 12);
  LOG_PORT.write((byte*) magnetom, 12);
  LOG_PORT.write((byte*) gyro, 12);
}

void output_sensors()
{
  if (output_mode == OUTPUT__MODE_SENSORS_RAW)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('R');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
  {
    // Apply sensor calibration
    compensate_sensor_errors();
    
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('C');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
    {
      output_sensors_binary();
      compensate_sensor_errors();
      output_sensors_binary();
    }
    else if (output_format == OUTPUT__FORMAT_TEXT)
    {
      output_sensors_text('R');
      compensate_sensor_errors();
      output_sensors_text('C');
    }
  }
}

