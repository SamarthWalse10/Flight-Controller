#include <Wire.h>

uint32_t loopTimer;

int calibNum;
float yawRate, pitchRate, rollRate;
float yawRateOffset, pitchRateOffset, rollRateOffset;
float accX, accY, accZ;
float rollAngle, pitchAngle;

float kalmanRollAngle = 0, kalmanUncertaintyRollAngle = 2 * 2;
float kalmanPitchAngle = 0, kalmanUncertaintyPitchAngle = 2 * 2;
float kalman1DOutput[] = { 0, 0 };  // {kalmanAngle prediction, Uncertainty of kalmanAngle prediction}

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);         // Set I2C frequency to 400kHz
  Wire.begin();                  // Initialize I2C communication as Master
  delay(500);                    // Wait for the IMU to power up
  Wire.beginTransmission(0x68);  // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x6B);              // Request access to MPU6050_PWR_MGMT_1 register(0x6B)
  Wire.write(0x00);              // Set the register bits as 00000000 to reset the IMU
  Wire.endTransmission();        // End the transmission

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

  loopTimer = micros();
}

void loop() {
  readIMU();                     // Read the IMU Accel & Gyro data and update Angles & Rotation rates
  yawRate -= yawRateOffset;      // Subtract the yawRateOffset from yawRate
  pitchRate -= pitchRateOffset;  // Subtract the pitchRateOffset from pitchRate
  rollRate -= rollRateOffset;    // Subtract the rollRateOffset from rollRate

  kalman1D(kalmanRollAngle, kalmanUncertaintyRollAngle, rollRate, rollAngle);      // Calculate the roll angle using kalman filter
  kalmanRollAngle = kalman1DOutput[0];                                             // Set the kalmanRollAngle to kalman1DOutput[0]
  kalmanUncertaintyRollAngle = kalman1DOutput[1];                                  // Set the uncertainty of kalmanRollAngle to kalman1DOutput[1]
  kalman1D(kalmanPitchAngle, kalmanUncertaintyPitchAngle, pitchRate, pitchAngle);  // Calculate the pitch angle using kalman filter
  kalmanPitchAngle = kalman1DOutput[0];                                            // Set the kalmanPitchAngle to kalman1DOutput[0]
  kalmanUncertaintyPitchAngle = kalman1DOutput[1];                                 // Set the uncertainty of kalmanPitchAngle to kalman1DOutput[1]

  // Serial.print("Yaw Rate [°/s]: ");
  // Serial.print(yawRate);
  // Serial.print(",");
  // Serial.print("Pitch Rate [°/s]: ");
  // Serial.print(pitchRate);
  // Serial.print(",");
  // Serial.print("Roll Rate [°/s]: ");
  // Serial.print(rollRate);
  // Serial.print(",");
  // Serial.print("Accel X[g]: ");
  // Serial.print(accX);
  // Serial.print(",");
  // Serial.print("Accel Y[g]: ");
  // Serial.print(accY);
  // Serial.print(",");
  // Serial.print("Accel Z[g]: ");
  // Serial.print(accZ);
  // Serial.print(",");
  // Serial.print("Roll Angle [°]: ");
  // Serial.print(rollAngle);
  // Serial.print(",");
  // Serial.print("Pitch Angle [°]: ");
  // Serial.print(pitchAngle);
  // Serial.print(",");
  // Serial.print("Pressure [kPa]: ");
  // Serial.print(press);
  // Serial.print(",");
  // Serial.println();

  // Serial.print("Roll Angle [°]: ");
  Serial.print(rollAngle);
  Serial.print(" , ");
  // Serial.print("Kalman Roll Angle [°]: ");
  Serial.print(kalmanRollAngle);
  Serial.print(" , ");
  // Serial.print("Pitch Angle [°]: ");
  Serial.print(pitchAngle);
  Serial.print(" , ");
  // Serial.print("Kalman Pitch Angle [°]: ");
  Serial.print(kalmanPitchAngle);
  Serial.print(" , ");
  Serial.println();

  // Serial.print("Accel X[g]: ");
  // Serial.print(accX);
  // Serial.print("     ,     ");
  // Serial.print("Accel Y[g]: ");
  // Serial.print(accY);
  // Serial.print("     ,     ");
  // Serial.print("Accel Z[g]: ");
  // Serial.print(accZ);
  // Serial.print("     ,     ");
  // Serial.println();

  while (micros() - loopTimer < 4000)
    ;  // Wait for 4ms* loop time
  loopTimer = micros();
}

void readIMU() {
  Wire.beginTransmission(0x68);  // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x1A);              // Request access to Gyro configuration register(0x1A)
  Wire.write(0x05);              // Set LPF with 10Hz bandwidth & 1KHz sample rate for Gyro & Accel
  Wire.endTransmission();        // End the transmission

  Wire.beginTransmission(0x68);  // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x1C);              // Request access to Accel configuration register(0x1C)
  Wire.write(0x10);              // Set sensitivity 4096LSB/g & full scale range +-8g
  Wire.endTransmission();        // End the transmission

  Wire.beginTransmission(0x68);                      // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x3B);                                  // Request access to ACCEL_XOUT data register(0x3B)
  Wire.endTransmission();                            // End the transmission
  Wire.requestFrom(0x68, 6);                         // Request 6 bytes of data from MPU6050
  int16_t accXLSB = Wire.read() << 8 | Wire.read();  // Read first 2 bytes of data (ACCEL_XOUT[15:8] & ACCEL_XOUT[7:0])
  int16_t accYLSB = Wire.read() << 8 | Wire.read();  // Read next 2 bytes of data (ACCEL_YOUT[15:8] & ACCEL_YOUT[7:0])
  int16_t accZLSB = Wire.read() << 8 | Wire.read();  // Read next 2 bytes of data (ACCEL_ZOUT[15:8] & ACCEL_ZOUT[7:0])

  Wire.beginTransmission(0x68);  // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x1B);              // Request access to Gyro configuration register(0x1B)
  Wire.write(0x08);              // Set sensitivity 65.5LSB/(deg/s) & full scale range +-500deg/s
  Wire.endTransmission();        // End the transmission

  Wire.beginTransmission(0x68);                    // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x43);                                // Request access to GYRO_XOUT data register(0x43)
  Wire.endTransmission();                          // End the transmission
  Wire.requestFrom(0x68, 6);                       // Request 6 bytes of data from MPU6050
  int16_t gyroX = Wire.read() << 8 | Wire.read();  // Read first 2 bytes of data (GYRO_XOUT[15:8] & GYRO_XOUT[7:0])
  int16_t gyroY = Wire.read() << 8 | Wire.read();  // Read next 2 bytes of data (GYRO_YOUT[15:8] & GYRO_YOUT[7:0])
  int16_t gyroZ = Wire.read() << 8 | Wire.read();  // Read next 2 bytes of data (GYRO_ZOUT[15:8] & GYRO_ZOUT[7:0])

  yawRate = (float)gyroZ / 65.5;    // Convert the gyro raw data into deg/s
  pitchRate = (float)gyroY / 65.5;  // Convert the gyro raw data into deg/s
  rollRate = (float)gyroX / 65.5;   // Convert the gyro raw data into deg/s

  accX = (float)accXLSB / 4096 - 0.03;                                         // *(float)accXLSB / 4096 + biasX;                      // Convert the accel raw data into g
  accY = (float)accYLSB / 4096 + 0.02;                                         // *(float)accYLSB / 4096 + biasY;                      // Convert the accel raw data into g
  accZ = (float)accZLSB / 4096 + 0.03;                                         // *(float)accZLSB / 4096 + biasZ;                      // Convert the accel raw data into g
  rollAngle = atan(accY / sqrt(accX * accX + accZ * accZ)) * (180 / 3.142);    // Calculate the roll angle
  pitchAngle = -atan(accX / sqrt(accY * accY + accZ * accZ)) * (180 / 3.142);  // Calculate the pitch angle
}

void kalman1D(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {
  float gyroAngleRateStd = 0.02;  // [°/s]
  float accAngleStd = 0.015;       // [°]
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
