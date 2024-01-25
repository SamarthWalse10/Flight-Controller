#include <Wire.h>

double yawRate, pitchRate, rollRate;
double accX, accY, accZ;
double rollAngle, pitchAngle;

float accZInertial;
float verticalVelocity;

uint16_t par_T1, par_T2, par_P5, par_P6;
int16_t par_P1, par_P2, par_P9;
int8_t par_T3, par_P3, par_P4, par_P7, par_P8, par_P10, par_P11;
double T_fine;
double T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
double altitudeBarometer, altitudeBarometerStartUp;
int rateCalibrationNumber;

int arrayLen = 4500;
// double* yawRateArray = new double[arrayLen]{ 0 };
// double* pitchRateArray = new double[arrayLen]{ 0 };
// double* rollRateArray = new double[arrayLen]{ 0 };
// double* accXArray = new double[arrayLen]{ 0 };
// double* accYArray = new double[arrayLen]{ 0 };
// double* accZArray = new double[arrayLen]{ 0 };
double *accZInertialArray = new double[arrayLen]{ 0 };
double *altitudeBarometerArray = new double[arrayLen]{ 0 };

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);  // Set I2C frequency to 400kHz
  Wire.begin();           // Initialize I2C communication as Master
  delay(500);             // Wait for the IMU to power up

  Wire.beginTransmission(0x68);  // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x6B);              // Request access to MPU6050_PWR_MGMT_1 register(0x6B)
  Wire.write(0x00);              // Set the register bits as 00000000 to reset the IMU
  Wire.endTransmission();        // End the transmission

  int rslt = write_register(0x7E, 0xB6);
  if (rslt == 0) Serial.println("Soft reset command successfully sent.");
  else Serial.println("Error sending soft reset command.");
  delay(2);  // giving sensor time to reset
  uint8_t calib_data[21], k = 0;
  Wire.beginTransmission(0x77);
  Wire.write(0x31);
  rslt = Wire.endTransmission();
  Wire.requestFrom(0x77, 21);
  while (Wire.available()) calib_data[k++] = Wire.read();
  par_T1 = (calib_data[1] << 8) | calib_data[0];
  par_T2 = (calib_data[3] << 8) | calib_data[2];
  par_T3 = (int8_t)calib_data[4];
  par_P1 = (int16_t)((calib_data[6] << 8) | calib_data[5]);
  par_P2 = (int16_t)((calib_data[8] << 8) | calib_data[7]);
  par_P3 = (int8_t)calib_data[9];
  par_P4 = (int8_t)calib_data[10];
  par_P5 = ((calib_data[12] << 8) | calib_data[11]);
  par_P6 = ((calib_data[14] << 8) | calib_data[13]);
  par_P7 = (int8_t)calib_data[15];
  par_P8 = (int8_t)calib_data[16];
  par_P9 = (int16_t)((calib_data[18] << 8) | calib_data[17]);
  par_P10 = (int8_t)calib_data[19];
  par_P11 = (int8_t)calib_data[20];
  T1 = (double)(par_T1 / pow(2, -8));
  T2 = (double)(par_T2 / pow(2, 30));
  T3 = (double)(par_T3 / pow(2, 48));
  P1 = (double)((par_P1 - pow(2, 14)) / pow(2, 20));
  P2 = (double)((par_P2 - pow(2, 14)) / pow(2, 29));
  P3 = (double)(par_P3 / pow(2, 32));
  P4 = (double)(par_P4 / pow(2, 37));
  P5 = (double)(par_P5 / pow(2, -3));
  P6 = (double)(par_P6 / pow(2, 6));
  P7 = (double)(par_P7 / pow(2, 8));
  P8 = (double)(par_P8 / pow(2, 15));
  P9 = (double)(par_P9 / pow(2, 48));
  P10 = (double)(par_P10 / pow(2, 48));
  P11 = (double)(par_P11 / pow(2, 65));
  if (rslt == 0) Serial.println("Calibration data successfully read.");
  else Serial.println("Error reading calibration data.");
  rslt = write_register(0x1B, 0x03);
  if (rslt == 0) Serial.println("Power ENABLE command successfully sent.");
  else Serial.println("Error sending power ENABLE command.");
  uint8_t reg_addr[3] = { 0x1C, 0x1D, 0x1F };
  uint8_t reg_val[3] = { 0x03, 0x02, 0x02 };
  rslt = burstWrite_register(reg_addr, reg_val, 3);
  if (rslt == 0) Serial.println("OSR, ODR, IIR commands successfully sent.");
  else Serial.println("Error sending OSR, ODR, IIR commands.");
  rslt = write_register(0x1B, 0x33);
  if (rslt == 0) Serial.println("Power MODE command successfully sent.");
  else Serial.println("Error sending power MODE command.");
  delay(5);  // giving sensor time to change mode
  for (rateCalibrationNumber = 0; rateCalibrationNumber < 4000; rateCalibrationNumber++) {
    readBarometer();
    delay(1);
  }
  for (rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++) {
    readBarometer();
    altitudeBarometerStartUp += altitudeBarometer;
    delay(1);
  }
  altitudeBarometerStartUp /= 2000.00;


  for (int j = 0; j < 100; j++) {
    for (int calibNum = 0; calibNum < arrayLen; calibNum++) {  // Calibrate the IMU by reading the gyro data 2000 times
      readIMU();                                               // Read the gyro data
      accZInertial = -sin(pitchAngle * (3.142 / 180)) * accX + cos(pitchAngle * (3.142 / 180)) * sin(rollAngle * (3.142 / 180)) * accY + cos(pitchAngle * (3.142 / 180)) * cos(rollAngle * (3.142 / 180)) * accZ;
      accZInertial = (accZInertial - 1) * 9.81 * 100.00;
      verticalVelocity = verticalVelocity + accZInertial * 0.004;
      // yawRateArray[calibNum] = yawRate;
      // pitchRateArray[calibNum] = pitchRate;
      // rollRateArray[calibNum] = rollRate;
      // accXArray[calibNum] = accX;
      // accYArray[calibNum] = accY;
      // accZArray[calibNum] = accZ;
      accZInertialArray[calibNum] = accZInertial;
      readBarometer();
      altitudeBarometer -= altitudeBarometerStartUp;
      altitudeBarometerArray[calibNum] = altitudeBarometer;
      delay(1);  // Add a 1ms delay to avoid overloading the IMU
    }
    // double meanYawRate = getMean(yawRateArray, arrayLen);
    // double stdYawRate = getStdDev(yawRateArray, arrayLen);
    // Serial.println("YawRate measurements   => Mean: " + String(meanYawRate) + "   => Standard Deviation: " + String(stdYawRate));
    // double meanPitchRate = getMean(pitchRateArray, arrayLen);
    // double stdPitchRate = getStdDev(pitchRateArray, arrayLen);
    // Serial.println("PitchRate measurements   => Mean: " + String(meanPitchRate) + "   => Standard Deviation: " + String(stdPitchRate));
    // double meanRollRate = getMean(rollRateArray, arrayLen);
    // double stdRollRate = getStdDev(rollRateArray, arrayLen);
    // Serial.println("RollRate measurements   => Mean: " + String(meanRollRate) + "   => Standard Deviation: " + String(stdRollRate));
    // double meanAccX = getMean(accXArray, arrayLen);
    // double stdAccX = getStdDev(accXArray, arrayLen);
    // Serial.println("AccX measurements   => Mean: " + String(meanAccX) + "   => Standard Deviation: " + String(stdAccX));
    // double meanAccY = getMean(accYArray, arrayLen);
    // double stdAccY = getStdDev(accYArray, arrayLen);
    // Serial.println("AccY measurements   => Mean: " + String(meanAccY) + "   => Standard Deviation: " + String(stdAccY));
    // double meanAccZ = getMean(accZArray, arrayLen);
    // double stdAccZ = getStdDev(accZArray, arrayLen);
    // Serial.println("AccZ measurements   => Mean: " + String(meanAccZ) + "   => Standard Deviation: " + String(stdAccZ));
    double meanAccZInertial = getMean(accZInertialArray, arrayLen);
    double stdAccZInertial = getStdDev(accZInertialArray, arrayLen);
    Serial.println("AccZInertial measurements   => Mean: " + String(meanAccZInertial) + "   => Standard Deviation: " + String(stdAccZInertial));
    double meanAltitudeBarometer = getMean(altitudeBarometerArray, arrayLen);
    double stdAltitudeBarometer = getStdDev(altitudeBarometerArray, arrayLen);
    Serial.println("AltitudeBarometer measurements   => Mean: " + String(meanAltitudeBarometer) + "   => Standard Deviation: " + String(stdAltitudeBarometer));
    Serial.println();
  }

  digitalWrite(13, LOW);
}

void loop() {
}

uint8_t write_register(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x77);
  Wire.write(reg);
  Wire.write(val);
  uint8_t result = Wire.endTransmission();
  return result;  // 0 if success, 1 if error
}

uint8_t burstWrite_register(uint8_t *regs, uint8_t *vals, uint8_t len) {
  Wire.beginTransmission(0x77);
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(regs[i]);
    Wire.write(vals[i]);
  }
  uint8_t result = Wire.endTransmission();
  return result;  // 0 if success, 1 if error
}

void readIMU() {
  Wire.beginTransmission(0x68);                                                // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x1A);                                                            // Request access to Gyro configuration register(0x1A)
  Wire.write(0x05);                                                            // Set LPF with 10Hz bandwidth & 1KHz sample rate for Gyro & Accel
  Wire.endTransmission();                                                      // End the transmission
  Wire.beginTransmission(0x68);                                                // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x1C);                                                            // Request access to Accel configuration register(0x1C)
  Wire.write(0x10);                                                            // Set sensitivity 4096LSB/g & full scale range +-8g
  Wire.endTransmission();                                                      // End the transmission
  Wire.beginTransmission(0x68);                                                // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x3B);                                                            // Request access to ACCEL_XOUT data register(0x3B)
  Wire.endTransmission();                                                      // End the transmission
  Wire.requestFrom(0x68, 6);                                                   // Request 6 bytes of data from MPU6050
  int16_t accXLSB = Wire.read() << 8 | Wire.read();                            // Read first 2 bytes of data (ACCEL_XOUT[15:8] & ACCEL_XOUT[7:0])
  int16_t accYLSB = Wire.read() << 8 | Wire.read();                            // Read next 2 bytes of data (ACCEL_YOUT[15:8] & ACCEL_YOUT[7:0])
  int16_t accZLSB = Wire.read() << 8 | Wire.read();                            // Read next 2 bytes of data (ACCEL_ZOUT[15:8] & ACCEL_ZOUT[7:0])
  Wire.beginTransmission(0x68);                                                // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x1B);                                                            // Request access to Gyro configuration register(0x1B)
  Wire.write(0x08);                                                            // Set sensitivity 65.5LSB/(deg/s) & full scale range +-500deg/s
  Wire.endTransmission();                                                      // End the transmission
  Wire.beginTransmission(0x68);                                                // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x43);                                                            // Request access to GYRO_XOUT data register(0x43)
  Wire.endTransmission();                                                      // End the transmission
  Wire.requestFrom(0x68, 6);                                                   // Request 6 bytes of data from MPU6050
  int16_t gyroX = Wire.read() << 8 | Wire.read();                              // Read first 2 bytes of data (GYRO_XOUT[15:8] & GYRO_XOUT[7:0])
  int16_t gyroY = Wire.read() << 8 | Wire.read();                              // Read next 2 bytes of data (GYRO_YOUT[15:8] & GYRO_YOUT[7:0])
  int16_t gyroZ = Wire.read() << 8 | Wire.read();                              // Read next 2 bytes of data (GYRO_ZOUT[15:8] & GYRO_ZOUT[7:0])
  yawRate = (double)gyroX / 65.5;                                              // Convert the gyro raw data into deg/s
  pitchRate = (double)gyroY / 65.5;                                            // Convert the gyro raw data into deg/s
  rollRate = (double)gyroZ / 65.5;                                             // Convert the gyro raw data into deg/s
  accX = (double)accXLSB / 4096 - 0.03;                                        // *(double)accXLSB / 4096 + biasX;                      // Convert the accel raw data into g
  accY = (double)accYLSB / 4096 + 0.02;                                        // *(double)accYLSB / 4096 + biasY;                      // Convert the accel raw data into g
  accZ = (double)accZLSB / 4096 + 0.03;                                        // *(double)accZLSB / 4096 + biasZ;                      // Convert the accel raw data into g
  rollAngle = atan(accY / sqrt(accX * accX + accZ * accZ)) * (180 / 3.142);    // Calculate the roll angle
  pitchAngle = -atan(accX / sqrt(accY * accY + accZ * accZ)) * (180 / 3.142);  // Calculate the pitch angle
}

void readBarometer() {
  uint8_t press_xlsb, press_lsb, press_msb, temp_xlsb, temp_lsb, temp_msb;
  uint8_t temp[6], j = 0;
  Wire.beginTransmission(0x77);
  Wire.write(0x04);  // read pressure and temperature data
  Wire.endTransmission();
  Wire.requestFrom(0x77, 6);
  while (Wire.available()) temp[j++] = Wire.read();
  press_xlsb = temp[0];
  press_lsb = temp[1];
  press_msb = temp[2];
  temp_xlsb = temp[3];
  temp_lsb = temp[4];
  temp_msb = temp[5];
  uint32_t uncomp_temp = (temp_msb << 16) | (temp_lsb << 8) | temp_xlsb;
  uint32_t uncomp_press = (press_msb << 16) | (press_lsb << 8) | press_xlsb;
  double partial_data1_t, partial_data2_t;
  partial_data1_t = (double)(uncomp_temp - T1);
  partial_data2_t = (double)(partial_data1_t * T2);
  T_fine = (double)(partial_data2_t + (partial_data1_t * partial_data1_t) * T3);
  double comp_press, partial_data1_p, partial_data2_p, partial_data3_p, partial_data4_p, partial_out1_p, partial_out2_p;
  partial_data1_p = (double)(P6 * T_fine);
  partial_data2_p = (double)(P7 * (T_fine * T_fine));
  partial_data3_p = (double)(P8 * (T_fine * T_fine * T_fine));
  partial_out1_p = (double)(P5 + partial_data1_p + partial_data2_p + partial_data3_p);
  partial_data1_p = (double)(P2 * T_fine);
  partial_data2_p = (double)(P3 * (T_fine * T_fine));
  partial_data3_p = (double)(P4 * (T_fine * T_fine * T_fine));
  partial_out2_p = (double)((double)uncomp_press * (P1 + partial_data1_p + partial_data2_p + partial_data3_p));
  partial_data1_p = (double)uncomp_press * (double)uncomp_press;
  partial_data2_p = (double)(P9 + (P10 * T_fine));
  partial_data3_p = (double)(partial_data1_p * partial_data2_p);
  partial_data4_p = (double)(partial_data3_p + ((double)uncomp_press * (double)uncomp_press * (double)uncomp_press) * P11);
  comp_press = (double)(partial_out1_p + partial_out2_p + partial_data4_p);
  double pressure = comp_press / 100.0F;
  altitudeBarometer = (double)(44330.00 * (1.00 - pow(pressure / 1013.25, 1.00 / 5.255)) * 100.00);
}

double getMean(double *val, int arrayCount) {
  long double total = 0;
  for (int i = 0; i < arrayCount; i++) {
    total = total + val[i];
  }
  double avg = total / (double)arrayCount;
  return avg;
}

double getStdDev(double *val, int arrayCount) {
  double avg = getMean(val, arrayCount);
  long double total = 0;
  for (int i = 0; i < arrayCount; i++) {
    total = total + (val[i] - avg) * (val[i] - avg);
  }

  double variance = total / (double)arrayCount;
  double stdDev = sqrt(variance);
  return stdDev;
}

// Angular Velocities[°/s]
// YawRate measurements   => Mean: -4.97   => Standard Deviation: 0.02
// PitchRate measurements   => Mean: -1.84   => Standard Deviation: 0.02
// RollRate measurements   => Mean: 0.34   => Standard Deviation: 0.02

// Accelerations[g]
// AccX measurements   => Mean: -0.02   => Standard Deviation: 0.00
// AccY measurements   => Mean: -0.01   => Standard Deviation: 0.00
// AccZ measurements   => Mean: 1.01   => Standard Deviation: 0.00

// Accelerations along Z in inertial frame[cm/s^2]
// AccZInertial measurements   => Mean: 5   => Standard Deviation: 0.95 (stationary)
// AccZInertial measurements   => Mean: 5   => Standard Deviation: 11.00 (small quick vibrations)

// Altitude from barometer[cm]
// AltitudeBarometer measurements   => Mean: (8,8.64,10.75)   => Standard Deviation: (8,8.3,8.6)
