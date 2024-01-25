#include <Wire.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

uint32_t loopTime;

float rollRate, pitchRate, yawRate;
float rollRateOffset, pitchRateOffset, yawRateOffset;
float rollAngle, pitchAngle;
float accX, accY, accZ;

float accZInertial;

uint16_t par_T1, par_T2, par_P5, par_P6;
int16_t par_P1, par_P2, par_P9;
int8_t par_T3, par_P3, par_P4, par_P7, par_P8, par_P10, par_P11;
double T_fine;
double T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
double altitudeBarometer, altitudeBarometerStartUp;
int rateCalibrationNumber;

float kalmanAltitude, kalmanVerticalVelocity;
BLA::Matrix<2, 2> F;
BLA::Matrix<2, 1> G;
BLA::Matrix<2, 2> P;
BLA::Matrix<2, 2> Q;
BLA::Matrix<2, 1> S;
BLA::Matrix<1, 2> H;
BLA::Matrix<2, 2> I;
BLA::Matrix<1, 1> acc;
BLA::Matrix<2, 1> K;
BLA::Matrix<1, 1> R;
BLA::Matrix<1, 1> L;
BLA::Matrix<1, 1> M;

void kalman2D(void) {
  acc = { accZInertial };
  S = F * S + G * acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;
  K = P * ~H * Invert(L);
  M = { (float)altitudeBarometer };
  S = S + K * (M - H * S);
  kalmanAltitude = S(0, 0);
  kalmanVerticalVelocity = S(1, 0);
  P = (I - K * H) * P;
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
  yawRate = (float)gyroZ / 65.5;                                               // Convert the gyro raw data into deg/s
  pitchRate = (float)gyroY / 65.5;                                             // Convert the gyro raw data into deg/s
  rollRate = (float)gyroX / 65.5;                                              // Convert the gyro raw data into deg/s
  accX = (float)accXLSB / 4096 - 0.03;                                         // *(float)accXLSB / 4096 + biasX;                      // Convert the accel raw data into g
  accY = (float)accYLSB / 4096 + 0.02;                                         // *(float)accYLSB / 4096 + biasY;                      // Convert the accel raw data into g
  accZ = (float)accZLSB / 4096 + 0.03;                                         // *(float)accZLSB / 4096 + biasZ;                      // Convert the accel raw data into g
  rollAngle = atan(accY / sqrt(accX * accX + accZ * accZ)) * (180 / 3.142);    // Calculate the roll angle
  pitchAngle = -atan(accX / sqrt(accY * accY + accZ * accZ)) * (180 / 3.142);  // Calculate the pitch angle
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

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // ***************************************************************** MPU6050 SETUP CODE START *****************************************************************
  Wire.beginTransmission(0x68);  // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x6B);              // Request access to MPU6050_PWR_MGMT_1 register(0x6B)
  Wire.write(0x00);              // Set the register bits as 00000000 to reset the IMU
  Wire.endTransmission();        // End the transmission
  // ***************************************************************** MPU6050 SETUP CODE END *****************************************************************

  // ***************************************************************** altitude_bmp390_final SETUP CODE START *****************************************************************
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
  // ***************************************************************** altitude_bmp390_final SETUP CODE END *****************************************************************

  for (rateCalibrationNumber = 0; rateCalibrationNumber < 4000; rateCalibrationNumber++) {
    readBarometer();
    delay(1);
  }
  for (rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++) {
    readBarometer();
    altitudeBarometerStartUp += altitudeBarometer;
    readIMU();
    yawRateOffset += yawRate;
    pitchRateOffset += pitchRate;
    rollRateOffset += rollRate;
    delay(1);
  }
  altitudeBarometerStartUp /= 2000.00;
  yawRateOffset /= 2000.00;
  pitchRateOffset /= 2000.00;
  rollRateOffset /= 2000.00;

  float stdAccZInertial = 10;       // [cm/s^2] -> 5
  float stdAltitudeBarometer = 20;  // [cm]     -> 8
  F = { 1, 0.004, 0, 1 };
  G = { 0.5 * 0.004 * 0.004, 0.004 };
  H = { 1, 0 };
  I = { 1, 0, 0, 1 };
  Q = G * ~G * stdAccZInertial * stdAccZInertial;
  R = { stdAltitudeBarometer * stdAltitudeBarometer };
  P = { 0, 0, 0, 0 };
  S = { 0, 0 };

  loopTime = micros();
}

void loop() {
  readIMU();
  accZInertial = -sin(pitchAngle * (3.142 / 180)) * accX + cos(pitchAngle * (3.142 / 180)) * sin(rollAngle * (3.142 / 180)) * accY + cos(pitchAngle * (3.142 / 180)) * cos(rollAngle * (3.142 / 180)) * accZ;
  accZInertial = (accZInertial - 1) * 9.81 * 100;

  readBarometer();
  altitudeBarometer -= altitudeBarometerStartUp;

  kalman2D();

  // Serial.print("Altitude[cm]: ");
  Serial.print(kalmanAltitude);
  Serial.print(" , ");
  // Serial.print(" Vertical Velocity[cm/s]: ");
  Serial.print(kalmanVerticalVelocity);
  Serial.print(" , ");
  // Serial.print(" Refrence Altitude Mark[cm]: ");
  Serial.print(int(0));
  Serial.print(" , ");
  Serial.print(int(25));
  Serial.print(" , ");
  Serial.println(int(-25));

  while (micros() - loopTime < 4000) {};
  loopTime = micros();
}
