/* This file is part of the Razor AHRS Firmware */

#if HW__VERSION_CODE == 14001
#define DMP_SAMPLE_RATE    100 // Logging/DMP sample rate(4-200 Hz)
#define IMU_COMPASS_SAMPLE_RATE 100 // Compass sample rate (4-100 Hz)
#define IMU_AG_SAMPLE_RATE 100 // Accel/gyro sample rate Must be between 4Hz and 1kHz
#define IMU_GYRO_FSR       2000 // Gyro full-scale range (250, 500, 1000, or 2000)
#define IMU_ACCEL_FSR      16 // Accel full-scale range (2, 4, 8, or 16)
#define IMU_AG_LPF         5 // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)
#define ENABLE_GYRO_CALIBRATION true

unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;

bool initIMU(void)
{
	// imu.begin() should return 0 on success. Will initialize
	// I2C bus, and reset MPU-9250 to defaults.
	if (imu.begin() != INV_SUCCESS)
		return false;

	// Set up MPU-9250 interrupt:
	imu.enableInterrupt(); // Enable interrupt output
	imu.setIntLevel(1);    // Set interrupt to active-low
	imu.setIntLatched(1);  // Latch interrupt output

	// Configure sensors:
	// Set gyro full-scale range: options are 250, 500, 1000, or 2000:
	imu.setGyroFSR(gyroFSR);
	// Set accel full-scale range: options are 2, 4, 8, or 16 g 
	imu.setAccelFSR(accelFSR);
	// Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
	imu.setLPF(IMU_AG_LPF);
	// Set gyro/accel sample rate: must be between 4-1000Hz
	// (note: this value will be overridden by the DMP sample rate)
	imu.setSampleRate(IMU_AG_SAMPLE_RATE);
	// Set compass sample rate: between 4-100Hz
	imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE);

	// Configure digital motion processor. Use the FIFO to get
	// data from the DMP.
	unsigned short dmpFeatureMask = 0;
	if (ENABLE_GYRO_CALIBRATION)
	{
		// Gyro calibration re-calibrates the gyro after a set amount
		// of no motion detected
		dmpFeatureMask |= DMP_FEATURE_GYRO_CAL; //added this line
		dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
	}
	else
	{
		// Otherwise add raw gyro readings to the DMP
		dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
	}
	// Add accel and quaternion's to the DMP
	dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
	dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

	// Initialize the DMP, and set the FIFO's update rate:
	imu.dmpBegin(dmpFeatureMask, fifoRate);

	return true; // Return success
}

void loop_imu()
{
	// Check IMU for new data, and log it.
	if (!imu.fifoAvailable())
	{
		num_accel_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading accelerometer");
		num_gyro_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading gyroscope");
		return;
	}

	// Read from the digital motion processor's FIFO.
	if (imu.dmpUpdateFifo() != INV_SUCCESS)
	{
		num_accel_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading accelerometer");
		num_gyro_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading gyroscope");
		return;
	}

	// Read from the compass.
	if (imu.updateCompass() != INV_SUCCESS)
	{
		num_magn_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading magnetometer");
		return;
	}

	// Conversion from g to similar unit as older versions of Razor...
	accel[0] = -250.0f*imu.calcAccel(imu.ax);
	accel[1] = 250.0f*imu.calcAccel(imu.ay);
	accel[2] = 250.0f*imu.calcAccel(imu.az);
	// Conversion from degrees/s to rad/s.
	gyro[0] = imu.calcGyro(imu.gx)*PI/180.0f;
	gyro[1] = -imu.calcGyro(imu.gy)*PI/180.0f;
	gyro[2] = -imu.calcGyro(imu.gz)*PI/180.0f;
	// Conversion from uT to mGauss.
	magnetom[0] = (10.0f*imu.calcMag(imu.my));
	magnetom[1] = (-10.0f*imu.calcMag(imu.mx));
	magnetom[2] = (10.0f*imu.calcMag(imu.mz));
}
#else
// I2C code to read the sensors

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int16_t) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int16_t) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int16_t) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif


void I2C_Init()
{
  Wire.begin();
}

void Accel_Init()
{
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2D);  // Power register
  WIRE_SEND(0x08);  // Measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31);  // Data format register
  WIRE_SEND(0x08);  // Set to full resolution
  Wire.endTransmission();
  delay(5);
  
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C);  // Rate
  WIRE_SEND(0x09);  // Set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  uint8_t buff[6];
  
  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(0x32);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    // No multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    accel[0] = (int16_t)((((uint16_t) buff[3]) << 8) | buff[2]);  // X axis (internal sensor y axis)
    accel[1] = (int16_t)((((uint16_t) buff[1]) << 8) | buff[0]);  // Y axis (internal sensor x axis)
    accel[2] = (int16_t)((((uint16_t) buff[5]) << 8) | buff[4]);  // Z axis (internal sensor z axis)
  }
  else
  {
    num_accel_errors++;
    if (output_errors) LOG_PORT.println("!ERR: reading accelerometer");
  }
}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Read_Magn()
{
  int i = 0;
  uint8_t buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
// 9DOF Razor IMU SEN-10125 using HMC5843 magnetometer
#if HW__VERSION_CODE == 10125
    // MSB byte first, then LSB; X, Y, Z
    magnetom[0] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));  // X axis (internal sensor -y axis)
    magnetom[1] = -1 * (int16_t)(((((uint16_t) buff[0]) << 8) | buff[1]));  // Y axis (internal sensor -x axis)
    magnetom[2] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));  // Z axis (internal sensor -z axis)
// 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10736
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));  // X axis (internal sensor -y axis)
    magnetom[1] = -1 * (int16_t)(((((uint16_t) buff[0]) << 8) | buff[1]));  // Y axis (internal sensor -x axis)
    magnetom[2] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));  // Z axis (internal sensor -z axis)
// 9DOF Sensor Stick SEN-10183 and SEN-10321 using HMC5843 magnetometer
#elif (HW__VERSION_CODE == 10183) || (HW__VERSION_CODE == 10321)
    // MSB byte first, then LSB; X, Y, Z
    magnetom[0] = (((uint16_t) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
    magnetom[1] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));  // Y axis (internal sensor -y axis)
    magnetom[2] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));  // Z axis (internal sensor -z axis)
// 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10724
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = (int16_t)((((uint16_t) buff[0]) << 8) | buff[1]);         // X axis (internal sensor x axis)
    magnetom[1] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));  // Y axis (internal sensor -y axis)
    magnetom[2] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));  // Z axis (internal sensor -z axis)
#endif
  }
  else
  {
    num_magn_errors++;
    if (output_errors) LOG_PORT.println("!ERR: reading magnetometer");
  }
}

void Gyro_Init()
{
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);
  
  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);
  
  // Set sample rato to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x15);
  WIRE_SEND(0x0A);  //  SMPLRT_DIV = 10 (50Hz)
  Wire.endTransmission();
  delay(5);

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
  int i = 0;
  uint8_t buff[6];
  
  Wire.beginTransmission(GYRO_ADDRESS); 
  WIRE_SEND(0x1D);  // Sends address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    gyro[0] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));    // X axis (internal sensor -y axis)
    gyro[1] = -1 * (int16_t)(((((uint16_t) buff[0]) << 8) | buff[1]));    // Y axis (internal sensor -x axis)
    gyro[2] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));    // Z axis (internal sensor -z axis)
  }
  else
  {
    num_gyro_errors++;
    if (output_errors) LOG_PORT.println("!ERR: reading gyroscope");
  }
}
#endif // HW__VERSION_CODE
