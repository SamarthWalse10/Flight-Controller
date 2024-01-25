#include <Wire.h>

uint32_t loopTimer;

float yawRate, pitchRate, rollRate;
float pitchAngle, rollAngle;
float accX, accY, accZ;

float accZInertial;
float verticalVelocity;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);         // Set I2C frequency to 400kHz
  Wire.begin();                  // Initialize I2C communication as Master
  delay(250);                    // Wait for the IMU to power up
  Wire.beginTransmission(0x68);  // Start communication with MPU6050(I2C address 0x68)
  Wire.write(0x6B);              // Request access to MPU6050_PWR_MGMT_1 register(0x6B)
  Wire.write(0x00);              // Set the register bits as 00000000 to reset the IMU
  Wire.endTransmission();        // End the transmission

  loopTimer = micros();
}

void loop() {
  readIMU();
  accZInertial = -sin(pitchAngle * (3.142 / 180)) * accX + cos(pitchAngle * (3.142 / 180)) * sin(rollAngle * (3.142 / 180)) * accY + cos(pitchAngle * (3.142 / 180)) * cos(rollAngle * (3.142 / 180)) * accZ;
  // accZInertial = (accZInertial*9.81*100.00) - (9.81*100.00)
  accZInertial = (accZInertial - 1) * 9.81 * 100.00;
  verticalVelocity = verticalVelocity + accZInertial * 0.004;

  // Serial.print("accZInertial [cm/s^2]: ");
  Serial.print(accZInertial);
  Serial.print(" , ");
  // Serial.print(" Vertical Velocity [cm/s]: ");
  Serial.println(verticalVelocity);

  while (micros() - loopTimer < 4000)
    ;  // 250Hz loop
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
