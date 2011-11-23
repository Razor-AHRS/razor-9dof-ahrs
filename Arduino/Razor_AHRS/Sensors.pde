// I2C code that reads the sensors

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2

void I2C_Init()
{
  Wire.begin();
}

void Accel_Init()
{
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.send(0x2D);  // power register
  Wire.send(0x08);  // measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.send(0x31);  // Data format register
  Wire.send(0x08);  // set to full resolution
  Wire.endTransmission();
  delay(5);	
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.send(0x2C);  // Rate
  Wire.send(0x09);  // set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(ACCEL_ADDRESS); 
  Wire.send(0x32);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(ACCEL_ADDRESS); //start transmission to device
  Wire.requestFrom(ACCEL_ADDRESS, 6);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i == 6)  // All bytes received?
  {
    accel[0] = sensor_sign[4] * ((((int) buff[3]) << 8) | buff[2]);    // X axis (internal sensor y axis)
    accel[1] = sensor_sign[3] * ((((int) buff[1]) << 8) | buff[0]);    // Y axis (internal sensor x axis)
    accel[2] = sensor_sign[5] * ((((int) buff[5]) << 8) | buff[4]);    // Z axis
  }
  else if (output_mode != OUTPUT__MODE_ANGLES_BINARY)
    Serial.println("!ERR: reading accelerometer");
}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  Wire.send(0x02); 
  Wire.send(0x00);   // Set continouos mode (default to 10Hz)
  Wire.endTransmission(); //end transmission
  delay(10);

  Wire.beginTransmission(MAGN_ADDRESS);
  Wire.send(0x00);
  Wire.send(0b00011000);   // Set 50Hz
  Wire.endTransmission(); //end transmission
  delay(5);
}

void Read_Magn()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);    // request 6 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i == 6)  // All bytes received?
  {
#if HW__RAZOR_VERSION == 10125  // SEN-10125 uses HMC5843 magnetometer
    // MSB byte first, then LSB; X, Y, Z
    magnetom[0] = sensor_sign[6] * ((((int) buff[2]) << 8) | buff[3]);    // X axis (internal sensor y axis)
    magnetom[1] = sensor_sign[7] * ((((int) buff[0]) << 8) | buff[1]);    // Y axis (internal sensor x axis)
    magnetom[2] = sensor_sign[8] * ((((int) buff[4]) << 8) | buff[5]);    // Z axis
#elif HW__RAZOR_VERSION == 10736  // SEN-10736 uses HMC5883L magnetometer
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = sensor_sign[6] * ((((int) buff[4]) << 8) | buff[5]);    // X axis (internal sensor y axis)
    magnetom[1] = sensor_sign[7] * ((((int) buff[0]) << 8) | buff[1]);    // Y axis (internal sensor x axis)
    magnetom[2] = sensor_sign[8] * ((((int) buff[2]) << 8) | buff[3]);    // Z axis
#endif
  }
  else if (output_mode != OUTPUT__MODE_ANGLES_BINARY)
    Serial.println("!ERR: reading magnetometer");
}

void Gyro_Init()
{
  /* Power up reset defaults */
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.send(0x3E);
  Wire.send(0x80);
  Wire.endTransmission(); //end transmission
  delay(10);
  
  /* Select full-scale range of the gyro sensors */
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.send(0x16);
  Wire.send(0x18);    // DLPF_CFG = 0, FS_SEL = 3
  Wire.endTransmission(); //end transmission
  delay(10);
  
  /* dont know... */
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.send(0x3E);
  Wire.send(0x00);
  Wire.endTransmission(); //end transmission
  delay(10);
  
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(GYRO_ADDRESS); 
  Wire.send(0x1D);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(GYRO_ADDRESS); //start transmission to device
  Wire.requestFrom(GYRO_ADDRESS, 6);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i == 6)  // All bytes received?
  {
    gyro[0] = sensor_sign[1] * ((((int) buff[2]) << 8) | buff[3]);    // X axis (internal sensor y axis)
    gyro[1] = sensor_sign[0] * ((((int) buff[0]) << 8) | buff[1]);    // Y axis (internal sensor x axis)
    gyro[2] = sensor_sign[2] * ((((int) buff[4]) << 8) | buff[5]);    // Z axis
  }
  else if (output_mode != OUTPUT__MODE_ANGLES_BINARY)
    Serial.println("!ERR: reading gyroscope");
}
