/* This file is part of the Razor AHRS Firmware */

#if (HW__VERSION_CODE == 16832) || (HW__VERSION_CODE == 15846)
bool imuAccDLPF = true; // IMU accelerometer Digital Low Pass Filter.
bool imuGyroDLPF = true; // IMU gyro Digital Low Pass Filter.
// IMU accelerometer full scale (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
// 0: +/- 2g
// 1: +/- 4g
// 2: +/- 8g
// 3: +/- 16g
int imuAccFSS = gpm16;
// IMU accelerometer DLPF bandwidth (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
// 0: 246.0 (3dB)  265.0 (Nyquist) (Hz)
// 1: 246.0 (3dB)  265.0 (Nyquist) (Hz)
// 2: 111.4 (3dB)  136.0 (Nyquist) (Hz)
// 3: 50.4  (3dB)  68.8  (Nyquist) (Hz)
// 4: 23.9  (3dB)  34.4  (Nyquist) (Hz)
// 5: 11.5  (3dB)  17.0  (Nyquist) (Hz)
// 6: 5.7   (3dB)  8.3   (Nyquist) (Hz)
// 7: 473   (3dB)  499   (Nyquist) (Hz)
int imuAccDLPFBW = 6;//acc_d50bw4_n68bw8;
// IMU gyro full scale (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
// 0: +/- 250dps
// 1: +/- 500dps
// 2: +/- 1000dps
// 3: +/- 2000dps
int imuGyroFSS = dps2000;
// IMU gyro DLPF bandwidth (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
// 0: 196.6 (3dB)  229.8 (Nyquist) (Hz)
// 1: 151.8 (3dB)  187.6 (Nyquist) (Hz)
// 2: 119.5 (3dB)  154.3 (Nyquist) (Hz)
// 3: 51.2  (3dB)  73.3  (Nyquist) (Hz)
// 4: 23.9  (3dB)  35.9  (Nyquist) (Hz)
// 5: 11.6  (3dB)  17.8  (Nyquist) (Hz)
// 6: 5.7   (3dB)  8.9   (Nyquist) (Hz)
// 7: 361.4 (3dB)  376.5 (Nyquist) (Hz)
int imuGyroDLPFBW = 6;//gyr_d51bw2_n73bw3;

void qwiicPowerOn()
{
	pinMode(PIN_QWIIC_POWER, OUTPUT);
	digitalWrite(PIN_QWIIC_POWER, LOW);
}

void qwiicPowerOff()
{
	pinMode(PIN_QWIIC_POWER, OUTPUT);
	digitalWrite(PIN_QWIIC_POWER, HIGH);
}

void microSDPowerOn()
{
	pinMode(PIN_MICROSD_POWER, OUTPUT);
	digitalWrite(PIN_MICROSD_POWER, LOW);
}

void microSDPowerOff()
{
	pinMode(PIN_MICROSD_POWER, OUTPUT);
	digitalWrite(PIN_MICROSD_POWER, HIGH);
}

void imuPowerOn()
{
	pinMode(PIN_IMU_POWER, OUTPUT);
	digitalWrite(PIN_IMU_POWER, HIGH);
}

void imuPowerOff()
{
	pinMode(PIN_IMU_POWER, OUTPUT);
	digitalWrite(PIN_IMU_POWER, LOW);
}

bool enableCIPOpullUp()
{
	//Add CIPO pull-up
	ap3_err_t retval = AP3_OK;
	am_hal_gpio_pincfg_t cipoPinCfg = AP3_GPIO_DEFAULT_PINCFG;
	cipoPinCfg.uFuncSel = AM_HAL_PIN_6_M0MISO;
	cipoPinCfg.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA;
	cipoPinCfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL;
	cipoPinCfg.uIOMnum = AP3_SPI_IOM;
	cipoPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
	padMode(MISO, cipoPinCfg, &retval);
	return (retval == AP3_OK);
}

bool beginIMU(void)
{
	// For low power...
	power_adc_disable(); // Power down ADC. It it started by default before setup().
	//Disable all pads
	//for (int x = 0; x < 50; x++)
	//	am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
	qwiicPowerOff();
	microSDPowerOff();

#ifdef USE_SPI
	SPI_PORT.begin();
#else
	WIRE_PORT.begin();
	WIRE_PORT.setClock(400000);
#endif

	enableCIPOpullUp(); // Enable CIPO pull-up on the OLA

	pinMode(PIN_IMU_CHIP_SELECT, OUTPUT);
	digitalWrite(PIN_IMU_CHIP_SELECT, HIGH); //Be sure IMU is deselected

	//Reset ICM by power cycling it
	imuPowerOff();

	delay(10);

	imuPowerOn(); // Enable power for the OLA IMU

	delay(100); // Wait for the IMU to power up

	bool initialized = false;
	while (!initialized) {
#ifdef USE_SPI
		myICM.begin(CS_PIN, SPI_PORT);
#else
		myICM.begin(WIRE_PORT, AD0_VAL);
#endif
		if (myICM.status != ICM_20948_Stat_Ok) {
			if (output_errors) LOG_PORT.println("!ERR: initializing ICM");
			delay(500);
		}
		else {
			initialized = true;
		}
	}

	delay(25); // See https://github.com/sparkfun/OpenLog_Artemis/issues/18...

	if (myICM.enableDLPF(ICM_20948_Internal_Acc, imuAccDLPF) != ICM_20948_Stat_Ok)
	{
		if (output_errors) LOG_PORT.println("!ERR: configuring DLPF");
	}
	if (myICM.enableDLPF(ICM_20948_Internal_Gyr, imuGyroDLPF) != ICM_20948_Stat_Ok)
	{
		if (output_errors) LOG_PORT.println("!ERR: configuring DLPF");
	}
	ICM_20948_dlpcfg_t dlpcfg;
	dlpcfg.a = imuAccDLPFBW;
	dlpcfg.g = imuGyroDLPFBW;
	if (myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg) != ICM_20948_Stat_Ok)
	{
		if (output_errors) LOG_PORT.println("!ERR: configuring DLPFBW");
	}
	ICM_20948_fss_t FSS;
	FSS.a = imuAccFSS;
	FSS.g = imuGyroFSS;
	if (myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS) != ICM_20948_Stat_Ok)
	{
		if (output_errors) LOG_PORT.println("!ERR: configuring FSS");
	}

	return true;
}

void loop_imu()
{
	if (!myICM.dataReady())
	{
		num_accel_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading accelerometer");
		num_gyro_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading gyroscope");
		num_magn_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading magnetometer");
		return;
	}

	myICM.getAGMT(); // The values are only updated when you call 'getAGMT'

	// Conversion from g to similar unit as older versions of Razor...
	accel[0] = 250.0f*myICM.accY()/1000.0;
	accel[1] = 250.0f*myICM.accX()/1000.0;
	accel[2] = 250.0f*myICM.accZ()/1000.0;
	// Conversion from degrees/s to rad/s.
	gyro[0] = -myICM.gyrY()*PI/180.0f;
	gyro[1] = -myICM.gyrX()*PI/180.0f;
	gyro[2] = -myICM.gyrZ()*PI/180.0f;
	// Conversion from uT to mGauss.
	magnetom[0] = (10.0f*myICM.magY());
	magnetom[1] = (-10.0f*myICM.magX());
	magnetom[2] = (10.0f*myICM.magZ());
}
#elif HW__VERSION_CODE == 14001
/*
* Known Bug -
* DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
* specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
* a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
* there will be a 25Hz interrupt from the MPU device.
*
* There is a known issue in which if you do not enable DMP_FEATURE_TAP
* then the interrupts will be at 200Hz even if fifo rate
* is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
*
* DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
*/
#define DMP_SAMPLE_RATE    100 // Logging/DMP sample rate(4-200 Hz)
#define IMU_COMPASS_SAMPLE_RATE 100 // Compass sample rate (4-100 Hz)
#define IMU_AG_SAMPLE_RATE 100 // Accel/gyro sample rate Must be between 4Hz and 1kHz
#if DEBUG__USE_DEFAULT_GYRO_FSR_M0 == true
#define IMU_GYRO_FSR       2000 // Gyro full-scale range (250, 500, 1000, or 2000)
#else
#define IMU_GYRO_FSR       2000 // Gyro full-scale range (250, 500, 1000, or 2000)
#endif // DEBUG__USE_DEFAULT_GYRO_FSR_M0
#if DEBUG__USE_DEFAULT_ACCEL_FSR_M0 == true
#define IMU_ACCEL_FSR      2 // Accel full-scale range (2, 4, 8, or 16)
#else
#define IMU_ACCEL_FSR      16 // Accel full-scale range (2, 4, 8, or 16)
#endif // DEBUG__USE_DEFAULT_ACCEL_FSR_M0
#define IMU_AG_LPF         5 // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)
#define ENABLE_GYRO_CALIBRATION true

unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;

bool beginIMU(void)
{
#if DEBUG__ENABLE_INTERRUPT_M0 == true
	// Set up MPU-9250 interrupt input (active-low)
	pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
#endif // DEBUG__ENABLE_INTERRUPT_M0

	// imu.begin() should return 0 on success. Will initialize
	// I2C bus, and reset MPU-9250 to defaults.
	if (imu.begin() != INV_SUCCESS)
		return false;

#if DEBUG__ENABLE_INTERRUPT_M0 == true
	// Use enableInterrupt() to configure the MPU-9250's 
	// interrupt output as a "data ready" indicator.
	imu.enableInterrupt();

	// The interrupt level can either be active-high or low.
	// Configure as active-low, since we'll be using the pin's
	// internal pull-up resistor.
	// Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
	imu.setIntLevel(INT_ACTIVE_LOW);

	// The interrupt can be set to latch until data has
	// been read, or to work as a 50us pulse.
	// Use latching method -- we'll read from the sensor
	// as soon as we see the pin go LOW.
	// Options are INT_LATCHED or INT_50US_PULSE
	imu.setIntLatched(INT_LATCHED);
#endif // DEBUG__ENABLE_INTERRUPT_M0

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

#if DEBUG__USE_DMP_M0 == true
	// Configure digital motion processor. Use the FIFO to get
	// data from the DMP.
	unsigned short dmpFeatureMask = 0;
	//dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
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
	// Because of known issue without DMP_FEATURE_TAP...
	dmpFeatureMask |= DMP_FEATURE_TAP;

	// Initialize the DMP, and set the FIFO's update rate:
	imu.dmpBegin(dmpFeatureMask, fifoRate);
#else
#if DEBUG__ENABLE_FIFO_M0 == true
	// Use configureFifo to set which sensors should be stored
	// in the buffer.  
	// Parameter to this function can be: INV_XYZ_GYRO, 
	// INV_XYZ_ACCEL, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
	imu.configureFifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);

	// Compass needs to be enabled again here due to a side effect of mpu_configure_fifo()...
	imu.setSensors(INV_XYZ_GYRO|INV_XYZ_ACCEL|INV_XYZ_COMPASS);
#endif // DEBUG__ENABLE_FIFO_M0
#endif // DEBUG__USE_DMP_M0

	return true; // Return success
}

void loop_imu()
{
	// Check IMU for new data, and log it.
#if DEBUG__USE_DMP_M0 == true
	if (!imu.fifoAvailable())
#else
#if DEBUG__ENABLE_FIFO_M0 == true
	if (!imu.fifoAvailable())
#else
	if (!imu.dataReady())
#endif // DEBUG__ENABLE_FIFO_M0
#endif // DEBUG__USE_DMP_M0
	{
		num_accel_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading accelerometer");
		num_gyro_errors++;
		if (output_errors) LOG_PORT.println("!ERR: reading gyroscope");
		return;
	}

	// Get the latest data...
#if DEBUG__USE_DMP_M0 == true
	while (imu.fifoAvailable())
	{
		// Read from the digital motion processor's FIFO.
		if (imu.dmpUpdateFifo() != INV_SUCCESS)
#else
#if DEBUG__ENABLE_FIFO_M0 == true
	while (imu.fifoAvailable())
	{
		if (imu.updateFifo() != INV_SUCCESS)
#else
	{
		if (imu.update(UPDATE_ACCEL|UPDATE_GYRO) != INV_SUCCESS)
#endif // DEBUG__ENABLE_FIFO_M0
#endif // DEBUG__USE_DMP_M0
		{
			num_accel_errors++;
			if (output_errors) LOG_PORT.println("!ERR: reading accelerometer");
			num_gyro_errors++;
			if (output_errors) LOG_PORT.println("!ERR: reading gyroscope");
			return;
		}
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

#if DEBUG__USE_ONLY_DMP_M0 == true
void Euler_angles_only_DMP_M0(void)
{
	Gyro_Vector[0] = GYRO_SCALED_RAD(gyro[0]); //gyro x roll
	Gyro_Vector[1] = GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
	Gyro_Vector[2] = GYRO_SCALED_RAD(gyro[2]); //gyro z yaw

	Accel_Vector[0] = accel[0];
	Accel_Vector[1] = accel[1];
	Accel_Vector[2] = accel[2];

	imu.computeEulerAngles();

	// Convert from NWU to NED...
	pitch = -imu.pitch*PI/180.0f;
	roll = imu.roll*PI/180.0f;

	Compass_Heading(); // Calculate magnetic heading

	float magyaw = -MAG_Heading;
	float imuyaw = imu.yaw*PI/180.0f;
	float fusionyaw = 0;
	float magfusioncoef = 0.0f;
	if (initialmagyaw == -10000) initialmagyaw = magyaw;
	if (initialimuyaw == -10000) initialimuyaw = imuyaw;
	if (!DEBUG__NO_DRIFT_CORRECTION)
		fusionyaw = magyaw*180.0f/PI;
	else
		fusionyaw = atan2((1-magfusioncoef)*sin(imuyaw+initialmagyaw-initialimuyaw)+magfusioncoef*sin(magyaw),
		(1-magfusioncoef)*cos(imuyaw+initialmagyaw-initialimuyaw)+magfusioncoef*cos(magyaw))*180.0f/PI;

	// Convert from NWU to NED...
	yaw = -fusionyaw*PI/180.0f;
}
#endif // DEBUG__USE_ONLY_DMP_M0
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
