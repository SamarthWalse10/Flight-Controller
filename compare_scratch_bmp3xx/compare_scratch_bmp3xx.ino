// final based on middle one of "bmp390_test_scratch_teensy4.0"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

uint32_t loopTimer;

uint16_t par_T1, par_T2, par_P5, par_P6;
int16_t par_P1, par_P2, par_P9;
int8_t par_T3, par_P3, par_P4, par_P7, par_P8, par_P10, par_P11;
double T_fine;
double T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;

double altitudeBarometer, altitudeBarometerStartUp;
Adafruit_BMP3XX bmp;
double altitudeBarometerBMP, altitudeBarometerStartUpBMP;
int rateCalibrationNumber;

uint8_t read_register(uint8_t reg) {
  Wire.beginTransmission(0x77);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(0x77, 1);
  uint8_t val = Wire.read();
  return val;
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

void barometerSignal() {
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

void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // ***************************************************************** BMP3XX ADAFRUIT LIBRARY CODE START *****************************************************************
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }

  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  // ***************************************************************** BMP3XX ADAFRUIT LIBRARY CODE END *****************************************************************

  // ***************************************************************** altitude_bmp390_final SCRATCH CODE START *****************************************************************
  Serial.println("Value of 0x00 CHIP_ID register: " + String(read_register(0x00), HEX));
  Serial.println(String(read_register(0x00), HEX) == String("60") ? "Correct chip id." : "Incorrect chip id.");
  Serial.println();

  Serial.println("Value of 0x03 sensor status register: " + String(read_register(0x03), HEX));
  if (read_register(0x03) & uint8_t(0x10)) Serial.println("Device ready to accept new commands.");
  else Serial.println("Device not ready to accept new commands.");
  Serial.println();

  Serial.println("Writing to 0x7E CMD register to send soft reset command");
  int rslt = write_register(0x7E, 0xB6);
  if (rslt == 0) Serial.println("Soft reset command successfully sent.");
  else Serial.println("Error sending soft reset command.");
  delay(2);
  Serial.println("Value of 0x02 ERR_REG(cmd_err) to check if there was an error during soft reset: " + String(read_register(0x02), HEX));
  if (read_register(0x02) & uint8_t(0x02)) Serial.println("Error during soft reset.");
  else Serial.println("No error during soft reset.");
  Serial.println();

  Serial.println("Reading calibration data from 0x31 register address onwards");
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
  Serial.println("par_T1: " + String(par_T1) + "  par_T2: " + String(par_T2) + "  par_T3: " + String(par_T3) + "  par_P1: " + String(par_P1) + "  par_P2: " + String(par_P2) + "  par_P3: " + String(par_P3) + "  par_P4: " + String(par_P4) + "  par_P5: " + String(par_P5) + "  par_P6: " + String(par_P6) + "  par_P7: " + String(par_P7) + "  par_P8: " + String(par_P8) + "  par_P9: " + String(par_P9) + "  par_P10: " + String(par_P10) + "  par_P11: " + String(par_P11));
  Serial.println("T1: " + String(T1) + "  T2: " + String(T2) + "  T3: " + String(T3) + "  P1: " + String(P1) + "  P2: " + String(P2) + "  P3: " + String(P3) + "  P4: " + String(P4) + "  P5: " + String(P5) + "  P6: " + String(P6) + "  P7: " + String(P7) + "  P8: " + String(P8) + "  P9: " + String(P9) + "  P10: " + String(P10) + "  P11: " + String(P11));
  Serial.println();

  Serial.println("Writing to 0x1B PWR_CTRL(temp_en and press_en) register to set temp enable and press enable");
  // Serial.println("Value of 0x1B register before writing: " + String(read_register(0x1B), HEX));
  rslt = write_register(0x1B, 0x03);
  if (rslt == 0) Serial.println("Power ENABLE command successfully sent.");
  else Serial.println("Error sending power ENABLE command.");
  // Serial.println("Value of 0x1B register after writing: " + String(read_register(0x1B), HEX));
  Serial.println();

  Serial.println("Burst Writing to { 0x1C OSR(osr_t and osr_p) } , { 0x1D ODR(odr_sel) } , { 0x1F CONFIG(iir_filter) } registers");
  // Serial.println("Value of 0x1C,0x1D,0x1F registers before writing are: " + String(read_register(0x1C), HEX) + "," + String(read_register(0x1D), HEX) + "," + String(read_register(0x1F), HEX));
  uint8_t reg_addr[3] = { 0x1C, 0x1D, 0x1F };
  uint8_t reg_val[3] = { 0x03, 0x02, 0x02 };
  rslt = burstWrite_register(reg_addr, reg_val, 3);
  if (rslt == 0) Serial.println("OSR, ODR, IIR commands successfully sent.");
  else Serial.println("Error sending OSR, ODR, IIR commands.");
  // Serial.println("Value of 0x1C,0x1D,0x1F registers after writing are: " + String(read_register(0x1C), HEX) + "," + String(read_register(0x1D), HEX) + "," + String(read_register(0x1F), HEX));
  Serial.println();

  Serial.println("Value of 0x19 INT_CTRL register is: " + String(read_register(0x19), HEX) + "   (required value is: 22)");
  Serial.println("Value of 0x1A IF_CONF register is: " + String(read_register(0x1A), HEX) + "   (required value is: 0)");
  Serial.println();

  Serial.println("Writing to 0x1B PWR_CTRL(mode) register to set mode to NORMAL");
  // Serial.println("Value of 0x1B register before writing: " + String(read_register(0x1B), HEX));
  rslt = write_register(0x1B, 0x33);
  if (rslt == 0) Serial.println("Power MODE command successfully sent.");
  else Serial.println("Error sending power MODE command.");
  delay(5);
  // Serial.println("Value of 0x1B register after writing: " + String(read_register(0x1B), HEX));
  Serial.println();

  // delay(10000);
  Serial.println("Value of 0x00 INT_CTRL register is: " + String(read_register(0x00), HEX));
  Serial.println("Value of 0x1B PWR_CTRL register is: " + String(read_register(0x1B), HEX));
  Serial.println("Value of 0x1C OSR register is: " + String(read_register(0x1C), HEX));
  Serial.println("Value of 0x1D ODR register is: " + String(read_register(0x1D), HEX));
  Serial.println("Value of 0x1F CONFIG_IIR register is: " + String(read_register(0x1F), HEX));
  Serial.println("Value of 0x19 INT_CTRL register is: " + String(read_register(0x19), HEX));
  Serial.println("Value of 0x1A IF_CONF register is: " + String(read_register(0x1A), HEX));
  Serial.println();
  // ***************************************************************** altitude_bmp390_final SCRATCH CODE END *****************************************************************

  for (rateCalibrationNumber = 0; rateCalibrationNumber < 4000; rateCalibrationNumber++) {
    barometerSignal();
    bmp.readAltitude(1013.25);
    delay(1);
  }
  for (rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++) {
    barometerSignal();
    altitudeBarometerStartUp += altitudeBarometer;
    altitudeBarometerStartUpBMP += (bmp.readAltitude(1013.25) * 100.00);
    delay(1);
  }
  altitudeBarometerStartUp /= 2000.00;
  altitudeBarometerStartUpBMP /= 2000.00;

  loopTimer = micros();
}

void loop() {
  altitudeBarometerBMP = (bmp.readAltitude(1013.25) * 100.00) - altitudeBarometerStartUpBMP;
  barometerSignal();
  altitudeBarometer -= altitudeBarometerStartUp;

  // Serial.print("Altitude Scratch[cm]: ");
  Serial.print(altitudeBarometer);
  Serial.print(" , ");
  // Serial.print(" Altitude BMP[cm]: ");
  Serial.print(altitudeBarometerBMP);
  Serial.print(" , ");
  // Serial.print(" Refrence Altitude Mark[cm]: ");
  Serial.println(int(0));

  while (micros() - loopTimer < 4000) {};  // NOTE: even if we have set 4ms(250hz) timer for each loop, but still due to adafruit library this may not be possible and time for each loop may be >4ms(<250hz), for this example.
  loopTimer = micros();
}
