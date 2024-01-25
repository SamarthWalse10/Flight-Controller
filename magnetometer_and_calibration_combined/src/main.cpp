#include <Wire.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

uint32_t loopTimer;

int calibNum;
float yawRate, pitchRate, rollRate;
float yawRateOffset, pitchRateOffset, rollRateOffset;
float accX, accY, accZ;
float rollAngle, pitchAngle;
float kalmanRollAngle = 0, kalmanUncertaintyRollAngle = 2 * 2;
float kalmanPitchAngle = 0, kalmanUncertaintyPitchAngle = 2 * 2;
float kalman1DOutput[] = { 0, 0 };

Eigen::Vector<float, 3> Mag;
Eigen::Vector<double, 3> biasMat;
Eigen::Matrix<double, 3, 3> scalMat;
float headingVal;
float MagX_dampened, MagY_dampened;
float magDeclination = 0.0;


void getCalibParams(double* flattenRawData, int noOfRows, float R) {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>* D = new Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>(noOfRows, 10);
  for (int i = 0; i < noOfRows; i++) {
    (*D)(i, 0) = pow(flattenRawData[(3 * i) + 0], 2);
    (*D)(i, 1) = pow(flattenRawData[(3 * i) + 1], 2);
    (*D)(i, 2) = pow(flattenRawData[(3 * i) + 2], 2);
    (*D)(i, 3) = 2 * flattenRawData[(3 * i) + 1] * flattenRawData[(3 * i) + 2];
    (*D)(i, 4) = 2 * flattenRawData[(3 * i) + 0] * flattenRawData[(3 * i) + 2];
    (*D)(i, 5) = 2 * flattenRawData[(3 * i) + 0] * flattenRawData[(3 * i) + 1];
    (*D)(i, 6) = 2 * flattenRawData[(3 * i) + 0];
    (*D)(i, 7) = 2 * flattenRawData[(3 * i) + 1];
    (*D)(i, 8) = 2 * flattenRawData[(3 * i) + 2];
    (*D)(i, 9) = 1;
  }
  Eigen::Matrix<double, 10, 10> S = (*D).transpose() * (*D);
  delete D;
  D = nullptr;
  if (D != nullptr) Serial.println("D Matrix freeing failed");
  Eigen::Matrix<double, 6, 6> S11;
  Eigen::Matrix<double, 6, 4> S12;
  Eigen::Matrix<double, 4, 4> S22;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) S11(i, j) = S(i, j);
    for (int j = 0; j < 4; j++) S12(i, j) = S(i, j + 6);
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) S22(i, j) = S(i + 6, j + 6);
  }
  Eigen::Matrix<double, 4, 6> S22a = S22.inverse() * S12.transpose();
  Eigen::Matrix<double, 6, 6> S22b = S12 * S22a;
  Eigen::Matrix<double, 6, 6> SS = S11 - S22b;
  Eigen::Matrix<double, 6, 6> C;
  C << -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -4.0;
  Eigen::Matrix<double, 6, 6> E = C.inverse() * SS;
  Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> solver1(E);
  Eigen::Vector<double, 6> eigVals = solver1.eigenvalues().real();
  Eigen::Matrix<double, 6, 6> eigVecs = solver1.eigenvectors().real();
  int maxEigValIndex;
  eigVals.maxCoeff(&maxEigValIndex);
  Eigen::Vector<double, 6> V1 = eigVecs.col(maxEigValIndex);
  if (V1[0] < 0.0) V1 = -V1;
  Eigen::Vector<double, 4> V2 = S22a * V1;
  Eigen::Vector<double, 10> V3;
  V3 << V1[0], V1[1], V1[2], V1[3], V1[4], V1[5], -V2[0], -V2[1], -V2[2], -V2[3];
  double a = V3[0], b = V3[1], c = V3[2], d = V3[5], e = V3[4], f = V3[3], g = V3[6], h = V3[7], i = V3[8], j = V3[9];
  Eigen::Matrix<double, 3, 3> Q;
  Q << a, d, e, d, b, f, e, f, c;
  Eigen::Vector<double, 3> U;
  U << g, h, i;
  biasMat = -Q.inverse() * U;
  Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> solver2(Q);
  Eigen::Vector<double, 3> lmb = solver2.eigenvalues().real();
  Eigen::Matrix<double, 3, 3> V = solver2.eigenvectors().real();
  double lmb1 = lmb[0];
  double lmb2 = lmb[1];
  double lmb3 = lmb[2];
  Eigen::Matrix<double, 3, 3> sqrtEigDiag;
  sqrtEigDiag << sqrt(lmb1), 0, 0, 0, sqrt(lmb2), 0, 0, 0, sqrt(lmb3);
  double constTerm = R / sqrt((biasMat.transpose() * Q * biasMat - j));
  scalMat = constTerm * (V * sqrtEigDiag * V.inverse());
}

void readMagnetometer() {
  uint8_t data[6], i = 0;
  Wire.beginTransmission(0x0D);
  Wire.write(0x00);  // reading the 6 data registers from 0x00 to 0x05
  Wire.endTransmission();
  Wire.requestFrom(0x0D, 6);
  while (Wire.available())
    data[i++] = Wire.read();
  uint8_t x_lsb = data[0];
  uint8_t x_msb = data[1];
  uint8_t y_lsb = data[2];
  uint8_t y_msb = data[3];
  uint8_t z_lsb = data[4];
  uint8_t z_msb = data[5];
  Mag[0] = (int16_t)(x_msb << 8 | x_lsb);
  Mag[1] = (int16_t)(y_msb << 8 | y_lsb);
  Mag[2] = (int16_t)(z_msb << 8 | z_lsb);
  Mag[0] = Mag[0] / 3000;  // for RNG of +-8Gauss => sesitivity is 3000 LSB/Gauss | for RNG of +-2Gauss => sesitivity is 12000 LSB/Gauss
  Mag[1] = Mag[1] / 3000;
  Mag[2] = Mag[2] / 3000;
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

void kalman1D(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {
  float gyroAngleRateStd = 0.02;  // [°/s]
  float accAngleStd = 0.015;      // [°]
  // float gyroAngleRateStd = 1e-8;
  // float accAngleStd = 0.00962361;
  kalmanState = kalmanState + 0.004 * kalmanInput;                                                 // Predict the current state of the system
  kalmanUncertainty = kalmanUncertainty + 0.004 * 0.004 * gyroAngleRateStd * gyroAngleRateStd;     // Calculate the uncertainty of the predicted state
  float kalmanGain = kalmanUncertainty * 1 / (1 * kalmanUncertainty + accAngleStd * accAngleStd);  // Calculate the kalmanGain from the uncertainties on the predictions and measurements
  kalmanState = kalmanState + kalmanGain * (kalmanMeasurement - kalmanState);                      // Update the predicted state of the system with the measurement of the state through the kalman gain
  kalmanUncertainty = (1 - kalmanGain) * kalmanUncertainty;                                        // Update the uncertainty of the predicted state
  kalman1DOutput[0] = kalmanState;                                                                 // Set the kalmanAngle prediction to kalman1DOutput[0]
  kalman1DOutput[1] = kalmanUncertainty;                                                           // Set the uncertainty of kalmanAngle prediction to kalman1DOutput[1]
}


void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x0D);
  Wire.write(0x0B);  // SET/RESET period register to recomended value of 0x01
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(0x0D);
  Wire.write(0x09);  // set OSR=512 , fullscale range(RNG)=+-8Gauss , ODR=200Hz , MODE=continuous
  Wire.write(0x1D);
  Wire.endTransmission();
  Mag << 0.0, 0.0, 0.0;
  biasMat << 0.50229589, -0.27320167, 0.10544156;
  scalMat << 2.57489066, 0.04358851, -0.05691189, 0.04358851, 2.8065306, -0.01236954, -0.05691189, -0.01236954, 2.55004887;

  Wire.beginTransmission(0x68);                      // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x6B);                                  // Request access to MPU6050_PWR_MGMT_1 register(0x6B)
  Wire.write(0x00);                                  // Set the register bits as 00000000 to reset the IMU
  Wire.endTransmission();                            // End the transmission
  for (calibNum = 0; calibNum < 2000; calibNum++) {  // Calibrate the IMU by reading the gyro data 2000 times
    readIMU();                                       // Read the gyro data
    yawRateOffset += yawRate;                        // Add the yaw rate to yawRateOffset
    pitchRateOffset += pitchRate;                    // Add the pitch rate to pitchRateOffset
    rollRateOffset += rollRate;                      // Add the roll rate to rollRateOffset
    delay(1);                                        // Add a 1ms delay to avoid overloading the IMU
  }
  yawRateOffset /= 2000;    // Divide the yawRateOffset by 2000 to get the average of yawRateOffset
  pitchRateOffset /= 2000;  // Divide the pitchRateOffset by 2000 to get the average of pitchRateOffset
  rollRateOffset /= 2000;   // Divide the rollRateOffset by 2000 to get the average of rollRateOffset

  digitalWrite(13, HIGH);
  int rowsCnt = 5000;
  double flattenRawData[rowsCnt * 3];
  for (int i = 0; i < rowsCnt; i++) {
    readMagnetometer();
    flattenRawData[(3 * i) + 0] = Mag[0];
    flattenRawData[(3 * i) + 1] = Mag[1];
    flattenRawData[(3 * i) + 2] = Mag[2];
    delay(2);
  }
  digitalWrite(13, LOW);
  getCalibParams(flattenRawData, rowsCnt, 1.0);

  loopTimer = micros();
}

void loop() {
  readIMU();
  yawRate -= yawRateOffset;
  pitchRate -= pitchRateOffset;
  rollRate -= rollRateOffset;
  kalman1D(kalmanRollAngle, kalmanUncertaintyRollAngle, rollRate, rollAngle);      // Calculate the roll angle using kalman filter
  kalmanRollAngle = kalman1DOutput[0];                                             // Set the kalmanRollAngle to kalman1DOutput[0]
  kalmanUncertaintyRollAngle = kalman1DOutput[1];                                  // Set the uncertainty of kalmanRollAngle to kalman1DOutput[1]
  kalman1D(kalmanPitchAngle, kalmanUncertaintyPitchAngle, pitchRate, pitchAngle);  // Calculate the pitch angle using kalman filter
  kalmanPitchAngle = kalman1DOutput[0];                                            // Set the kalmanPitchAngle to kalman1DOutput[0]
  kalmanUncertaintyPitchAngle = kalman1DOutput[1];                                 // Set the uncertainty of kalmanPitchAngle to kalman1DOutput[1]

  readMagnetometer();
  Mag = scalMat.cast<float>() * (Mag - biasMat.cast<float>());
  float magX_hor = Mag[0] * cos(-kalmanRollAngle * (3.142 / 180)) + Mag[1] * sin(kalmanPitchAngle * (3.142 / 180)) * sin(-kalmanRollAngle * (3.142 / 180)) - Mag[2] * cos(kalmanPitchAngle * (3.142 / 180)) * sin(-kalmanRollAngle * (3.142 / 180));  // compensating for tilt
  float magY_hor = Mag[1] * cos(kalmanPitchAngle * (3.142 / 180)) + Mag[2] * sin(kalmanPitchAngle * (3.142 / 180));  // compensating for tilt
  headingVal = atan2(magY_hor, magX_hor) * (180 / 3.142);  // Magnetic North is shown along X-axis of the magnetometer // compensated tilt
  // MagX_dampened = MagX_dampened * 0.9 + magX_hor * 0.1;
  // MagY_dampened = MagY_dampened * 0.9 + magY_hor * 0.1;
  // headingVal = atan2(MagY_dampened, MagX_dampened) * (180/3.142);  // Magnetic North is shown along X-axis of the magnetometer  // compensated tilt and dampened
  headingVal += magDeclination;
  if (headingVal < 0) headingVal += 360;

  // // Serial.print("MagX[G]: ");
  // Serial.print(Mag[0]);
  // Serial.print(" , ");
  // // Serial.print("MagY[G]: ");
  // Serial.print(Mag[1]);
  // Serial.print(" , ");
  // // Serial.print("MagZ[G]: ");
  // Serial.print(Mag[2]);
  // Serial.print(" , ");
  // // Serial.print("Total Mag[G]: ");
  // Serial.print(sqrt(pow(Mag[0], 2) * pow(Mag[1], 2) * pow(Mag[2], 2)));
  // Serial.print(" , ");
  // // Serial.print("Heading Value[deg]: ");
  Serial.println(headingVal);

  while (micros() - loopTimer < 4000) {};
  loopTimer = micros();
}
