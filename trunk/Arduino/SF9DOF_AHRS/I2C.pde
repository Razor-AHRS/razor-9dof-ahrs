/* ******************************************************* */
/* I2C code for ADXL345 accelerometer                      */
/* and HMC5843 magnetometer                                */
/* ******************************************************* */

int AccelAddress = 0x53;      // 0x53 = 0xA6 / 2
int CompassAddress = 0x1E;    // 0x1E = 0x3C / 2    //0x3D
int GyroAddress = 0x68;       // 0x68 = 0xD0 / 2


void I2C_Init()
{
  Wire.begin();
}

void Accel_Init()
{
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x2D);  // power register
  Wire.send(0x08);  // measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x31);  // Data format register
  Wire.send(0x08);  // set to full resolution
  Wire.endTransmission();
  delay(5);	
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x2C);  // Rate
  Wire.send(0x09);  // set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(AccelAddress); 
  Wire.send(0x32);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(AccelAddress); //start transmission to device
  Wire.requestFrom(AccelAddress, 6);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
    ACC[1] = (((int)buff[1]) << 8) | buff[0];    // Y axis (internal sensor x axis)
    ACC[0] = (((int)buff[3]) << 8) | buff[2];    // X axis (internal sensor y axis)
    ACC[2] = (((int)buff[5]) << 8) | buff[4];    // Z axis
    AN[3] = ACC[0];
    AN[4] = ACC[1];
    AN[5] = ACC[2];
    accel_x = SENSOR_SIGN[3]*(ACC[0]/*-AN_OFFSET[3]*/);
    accel_y = SENSOR_SIGN[4]*(ACC[1]/*-AN_OFFSET[4]*/);
    accel_z = SENSOR_SIGN[5]*(ACC[2]/*-AN_OFFSET[5]*/);
    }
  else
    Serial.println("!ERR: Acc data");
}

void Compass_Init()
{
  Wire.beginTransmission(CompassAddress);
  Wire.send(0x02); 
  Wire.send(0x00);   // Set continouos mode (default to 10Hz)
  Wire.endTransmission(); //end transmission
  delay(10);

  Wire.beginTransmission(CompassAddress);
  Wire.send(0x00);
  Wire.send(0b00011000);   // Set 50Hz
  Wire.endTransmission(); //end transmission
  delay(5);
}

void Read_Compass()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(CompassAddress); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(CompassAddress); 
  Wire.requestFrom(CompassAddress, 6);    // request 6 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Y,Z
    magnetom_x = SENSOR_SIGN[6]*((((int)buff[2]) << 8) | buff[3]);    // X axis (internal sensor y axis)
    magnetom_y = SENSOR_SIGN[7]*((((int)buff[0]) << 8) | buff[1]);    // Y axis (internal sensor x axis)
    magnetom_z = SENSOR_SIGN[8]*((((int)buff[4]) << 8) | buff[5]);    // Z axis
    }
  else
    Serial.println("!ERR: Mag data");
}


/* Gyro stuffs */

void Gyro_Init()
{
  /* Power up reset defaults */
  Wire.beginTransmission(GyroAddress);
  Wire.send(0x3E);
  Wire.send(0x80);
  Wire.endTransmission(); //end transmission
  delay(10);
  
  /* Select full-scale range of the gyro sensors */
  Wire.beginTransmission(GyroAddress);
  Wire.send(0x16);
  Wire.send(0x18);    // DLPF_CFG = 0, FS_SEL = 3
  Wire.endTransmission(); //end transmission
  delay(10);
  
  /* dont know... */
  Wire.beginTransmission(GyroAddress);
  Wire.send(0x3E);
  Wire.send(0x00);
  Wire.endTransmission(); //end transmission
  delay(10);
  
}
/**/

// Reads x,y and z accelerometer registers
void Read_Gyro()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(GyroAddress); 
  Wire.send(0x1D);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(GyroAddress); //start transmission to device
  Wire.requestFrom(GyroAddress, 6);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
    AN[1] = (((int)buff[0]) << 8) | buff[1];    // Y axis (internal sensor x axis)
    AN[0] = (((int)buff[2]) << 8) | buff[3];    // X axis (internal sensor y axis)
    AN[2] = (((int)buff[4]) << 8) | buff[5];    // Z axis
    }
  else
    Serial.println("!ERR: Gyro data");
}

/**/
