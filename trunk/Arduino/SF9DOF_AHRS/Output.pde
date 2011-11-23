// Output angles: yaw, pitch, roll
void output_angles()
{
  if (output_mode == OUTPUT__MODE_ANGLES_BINARY)
  {
    float ypr[3];  
    ypr[0] = TO_DEG(yaw);
    ypr[1] = TO_DEG(pitch);
    ypr[2] = TO_DEG(roll);
    Serial.write((byte*) ypr, 12); // No new-line
  }
  else if (output_mode == OUTPUT__MODE_ANGLES_TEXT)
  {
      Serial.print("#YPR=");
      Serial.print(TO_DEG(yaw)); Serial.print(",");
      Serial.print(TO_DEG(pitch)); Serial.print(",");
      Serial.print(TO_DEG(roll)); Serial.println();
  }
}

void output_calibration(int calibration_sensor)
{
  if (calibration_sensor == 0)  // Accelerometer
  {
    // Output MIN/MAX values
    Serial.print("accel min/max (x y z): ");
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
      if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
      Serial.print(accel_min[i]);
      Serial.print("/");
      Serial.print(accel_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {
    // Output MIN/MAX values
    Serial.print("magn min/max (x y z): ");
    for (int i = 0; i < 3; i++) {
      if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
      if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
      Serial.print(magnetom_min[i]);
      Serial.print("/");
      Serial.print(magnetom_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {
    // Average gyro values
    for (int i = 0; i < 3; i++)
      gyro_average[i] += gyro[i];
    gyro_num_samples++;
      
    // Output current and averaged gyroscope values
    Serial.print("gyroscope current/average (x y z): ");
    for (int i = 0; i < 3; i++) {
      Serial.print(gyro[i]);
      Serial.print("/");
      Serial.print(gyro_average[i] / (float) gyro_num_samples);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
}

void output_sensors()
{
  Serial.print("accelerometer: ");
  Serial.print(accel[0]); Serial.print("  ");
  Serial.print(accel[1]); Serial.print("  ");
  Serial.print(accel[2]); Serial.println();

  Serial.print("magnetometer: ");
  Serial.print(magnetom[0]); Serial.print("  ");
  Serial.print(magnetom[1]); Serial.print("  ");
  Serial.print(magnetom[2]); Serial.println();

  Serial.print("gyroscope: ");
  Serial.print(gyro[0]); Serial.print("  ");
  Serial.print(gyro[1]); Serial.print("  ");
  Serial.print(gyro[2]); Serial.println();
}

