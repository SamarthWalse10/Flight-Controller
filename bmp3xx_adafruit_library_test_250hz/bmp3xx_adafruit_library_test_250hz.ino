// final based on middle one of "bmp390_test_scratch_teensy4.0"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

uint32_t loopTimer;

Adafruit_BMP3XX bmp;
float altitudeBarometerBMP, altitudeBarometerStartUpBMP;
int rateCalibrationNumber;

uint8_t read_register(uint8_t reg) {
  Wire.beginTransmission(0x77);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(0x77, 1);
  uint8_t val = Wire.read();
  return val;
}

void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  for (rateCalibrationNumber = 0; rateCalibrationNumber < 4000; rateCalibrationNumber++) {
    bmp.readAltitude(1013.25);
    delay(1);
  }
  for (rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++) {
    altitudeBarometerStartUpBMP += (bmp.readAltitude(1013.25) * 100.00);
    delay(1);
  }
  altitudeBarometerStartUpBMP /= 2000.00;

  loopTimer = micros();
}

void loop() {
  altitudeBarometerBMP = (bmp.readAltitude(1013.25) * 100.00) - altitudeBarometerStartUpBMP;

  // Serial.print("Altitude BMP[cm]: ");
  Serial.print(altitudeBarometerBMP);
  Serial.print(" , ");
  // Serial.print(" Refrence Altitude Mark: ");
  Serial.println(int(0));

  while (micros() - loopTimer < 4000) {};
  loopTimer = micros();
}
