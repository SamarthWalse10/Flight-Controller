// #include <Arduino.h>
// #include <Wire.h>

// uint16_t par_T1, par_T2, par_P5, par_P6;
// int16_t par_P1, par_P2, par_P9;
// int8_t par_T3, par_P3, par_P4, par_P7, par_P8, par_P10, par_P11;
// int64_t T_fine;
// float T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;

// float altitudeBarometer, altitudeBarometerStartUp;
// int rateCalibrationNumber;

// uint8_t read_register(uint8_t reg)
// {
//   Wire.beginTransmission(0x77);
//   Wire.write(reg);
//   Wire.endTransmission();
//   Wire.requestFrom(0x77, 1);
//   uint8_t val = Wire.read();
//   return val;
// }

// void barometerSignal()
// {
//   Serial.println();

//   // Wire.beginTransmission(0x77);
//   // Wire.write(0x04); // read pressure and temperature data
//   // int pressureNtemperatureTransmissionResult = Wire.endTransmission();
//   // if (pressureNtemperatureTransmissionResult == 0)
//   // {
//   //   Serial.println("Pressure and temperature command successfully sent.");
//   // }
//   // else
//   // {
//   //   Serial.println("Error sending pressure and temperature command.");
//   // }
//   // Wire.requestFrom(0x77, 6);
//   // uint8_t press_xlsb = Wire.read();
//   // uint8_t press_lsb = Wire.read();
//   // uint8_t press_msb = Wire.read();
//   // uint8_t temp_xlsb = Wire.read();
//   // uint8_t temp_lsb = Wire.read();
//   // uint8_t temp_msb = Wire.read();
//   uint8_t press_xlsb, press_lsb, press_msb, temp_xlsb, temp_lsb, temp_msb;
//   uint8_t temp[6], j = 0;
//   Wire.beginTransmission(0x77);
//   Wire.write(0x04); // read pressure and temperature data
//   int pressureNtemperatureTransmissionResult = Wire.endTransmission();
//   if (pressureNtemperatureTransmissionResult == 0)
//   {
//     Serial.println("Pressure and temperature command successfully sent.");
//   }
//   else
//   {
//     Serial.println("Error sending pressure and temperature command.");
//   }
//   Wire.requestFrom(0x77, 6);
//   while (Wire.available())
//   {
//     temp[j] = Wire.read();
//     j++;
//   }
//   press_xlsb = temp[0];
//   press_lsb = temp[1];
//   press_msb = temp[2];
//   temp_xlsb = temp[3];
//   temp_lsb = temp[4];
//   temp_msb = temp[5];

//   Serial.println("press_msb: " + String(press_msb) + "  press_lsb: " + String(press_lsb) + "  press_xlsb: " + String(press_xlsb) + "  temp_msb: " + String(temp_msb) + "  temp_lsb: " + String(temp_lsb) + "  temp_xlsb: " + String(temp_xlsb));

//   uint32_t uncomp_temp = (temp_msb << 16) | (temp_lsb << 8) | temp_xlsb;
//   uint32_t uncomp_press = (press_msb << 16) | (press_lsb << 8) | press_xlsb;

//   Serial.println("uncomp_temp: " + String(uncomp_temp) + "  uncomp_press: " + String(uncomp_press));

//   float partial_data1_t, partial_data2_t;
//   partial_data1_t = (float)(uncomp_temp - T1);
//   partial_data2_t = (float)(partial_data1_t * T2);
//   T_fine = (int64_t)(partial_data2_t + (partial_data1_t * partial_data1_t) * T3);

//   float comp_press, partial_data1_p, partial_data2_p, partial_data3_p, partial_data4_p, partial_out1_p, partial_out2_p;
//   partial_data1_p = (float)(P6 * T_fine);
//   partial_data2_p = (float)(P7 * (T_fine * T_fine));
//   partial_data3_p = (float)(P8 * (T_fine * T_fine * T_fine));
//   partial_out1_p = (float)(P5 + partial_data1_p + partial_data2_p + partial_data3_p);
//   partial_data1_p = (float)(P2 * T_fine);
//   partial_data2_p = (float)(P3 * (T_fine * T_fine));
//   partial_data3_p = (float)(P4 * (T_fine * T_fine * T_fine));
//   partial_out2_p = (float)((float)uncomp_press * (P1 + partial_data1_p + partial_data2_p + partial_data3_p));
//   partial_data1_p = (float)uncomp_press * (float)uncomp_press;
//   partial_data2_p = (float)(P9 + (P10 * T_fine));
//   partial_data3_p = (float)(partial_data1_p * partial_data2_p);
//   partial_data4_p = (float)(partial_data3_p + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * P11);
//   comp_press = (float)(partial_out1_p + partial_out2_p + partial_data4_p);

//   float pressure = comp_press / 100.0F;
//   altitudeBarometer = 44330.0 * (1.0 - pow(pressure / 1013.25, 1 / 5.255));
// }

// void setup()
// {
//   Serial.begin(9600);
//   Wire.setClock(400000);
//   Wire.begin();
//   delay(250);

//   Serial.println("Value of 0x7E register before soft reset: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register before soft reset: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register before soft reset: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register before soft reset: " + String(read_register(0x1F), HEX));
//   Wire.beginTransmission(0x77);
//   Wire.write(0x7E); // soft reset
//   Wire.write(0xB6);
//   int resetTransmissionResult = Wire.endTransmission();
//   if (resetTransmissionResult == 0)
//   {
//     Serial.println("Soft reset command successfully sent.");
//   }
//   else
//   {
//     Serial.println("Error sending soft reset command.");
//   }
//   delay(5);
//   Serial.println("Value of 0x7E register after soft reset: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register after soft reset: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register after soft reset: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register after soft reset: " + String(read_register(0x1F), HEX));

//   // Serial.println("Value of 0x7E register before oversampling rate command: " + String(read_register(0x7E), HEX));
//   // Serial.println("Value of 0x1B register before oversampling rate command: " + String(read_register(0x1B), HEX));
//   // Serial.println("Value of 0x1C register before oversampling rate command: " + String(read_register(0x1C), HEX));
//   // Serial.println("Value of 0x1F register before oversampling rate command: " + String(read_register(0x1F), HEX));
//   // Wire.beginTransmission(0x77);
//   // Wire.write(0x1C); // set oversampling rate(OSR) to 1x for temp and 8x for press
//   // Wire.write(0x02);
//   // int oversamplingRateTransmissionResult = Wire.endTransmission();
//   // if (oversamplingRateTransmissionResult == 0)
//   // {
//   //   Serial.println("Oversampling rate command successfully sent.");
//   // }
//   // else
//   // {
//   //   Serial.println("Error sending oversampling rate command.");
//   // }
//   // delay(5);
//   // Serial.println("Value of 0x7E register after oversampling rate command: " + String(read_register(0x7E), HEX));
//   // Serial.println("Value of 0x1B register after oversampling rate command: " + String(read_register(0x1B), HEX));
//   // Serial.println("Value of 0x1C register after oversampling rate command: " + String(read_register(0x1C), HEX));
//   // Serial.println("Value of 0x1F register after oversampling rate command: " + String(read_register(0x1F), HEX));

//   Serial.println("Value of 0x7E register before power mode command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register before power mode command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register before power mode command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register before power mode command: " + String(read_register(0x1F), HEX));
//   Wire.beginTransmission(0x77);
//   Wire.write(0x1B); // set power mode to normal and enable temp and press read
//   Wire.write(0x33);
//   int powerModeTransmissionResult = Wire.endTransmission();
//   if (powerModeTransmissionResult == 0)
//   {
//     Serial.println("Power mode command successfully sent.");
//   }
//   else
//   {
//     Serial.println("Error sending power mode command.");
//   }
//   delay(5);
//   Serial.println("Value of 0x7E register after power mode command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register after power mode command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register after power mode command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register after power mode command: " + String(read_register(0x1F), HEX));

//   Serial.println("Value of 0x7E register before oversampling rate command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register before oversampling rate command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register before oversampling rate command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register before oversampling rate command: " + String(read_register(0x1F), HEX));
//   Wire.beginTransmission(0x77);
//   Wire.write(0x1C); // set oversampling rate(OSR) to 1x for temp and 8x for press
//   Wire.write(0x03);
//   int oversamplingRateTransmissionResult = Wire.endTransmission();
//   if (oversamplingRateTransmissionResult == 0)
//   {
//     Serial.println("Oversampling rate command successfully sent.");
//   }
//   else
//   {
//     Serial.println("Error sending oversampling rate command.");
//   }
//   delay(5);
//   Serial.println("Value of 0x7E register after oversampling rate command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register after oversampling rate command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register after oversampling rate command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register after oversampling rate command: " + String(read_register(0x1F), HEX));

//   Serial.println("Value of 0x7E register before iir filter setting command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register before iir filter setting command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register before iir filter setting command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register before iir filter setting command: " + String(read_register(0x1F), HEX));
//   Wire.beginTransmission(0x77);
//   Wire.write(0x1F); // set IIR filter coefficient to 3
//   Wire.write(0x04);
//   int filterCoefficientTransmissionResult = Wire.endTransmission();
//   if (filterCoefficientTransmissionResult == 0)
//   {
//     Serial.println("Filter coefficient command successfully sent.");
//   }
//   else
//   {
//     Serial.println("Error sending filter coefficient command.");
//   }
//   delay(5);
//   Serial.println("Value of 0x7E register after iir filter setting command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register after iir filter setting command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register after iir filter setting command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register after iir filter setting command: " + String(read_register(0x1F), HEX));

//   uint8_t data[21], i = 0;
//   Wire.beginTransmission(0x77); // read calibration data and store in their respective variables
//   Wire.write(0x31);
//   int calibrationTransmissionResult = Wire.endTransmission();
//   if (calibrationTransmissionResult == 0)
//   {
//     Serial.println("Calibration command successfully sent.");
//   }
//   else
//   {
//     Serial.println("Error sending calibration command.");
//   }
//   Wire.requestFrom(0x77, 21);
//   while (Wire.available())
//   {
//     data[i] = Wire.read();
//     i++;
//   }
//   par_T1 = (data[1] << 8) | data[0];
//   par_T2 = (data[3] << 8) | data[2];
//   par_T3 = (int8_t)data[4];
//   par_P1 = (int16_t)((data[6] << 8) | data[5]);
//   par_P2 = (int16_t)((data[8] << 8) | data[7]);
//   par_P3 = (int8_t)data[9];
//   par_P4 = (int8_t)data[10];
//   par_P5 = ((data[12] << 8) | data[11]);
//   par_P6 = ((data[14] << 8) | data[13]);
//   par_P9 = (int16_t)((data[16] << 8) | data[15]);
//   par_P10 = (int8_t)data[17];
//   par_P11 = (int8_t)data[18];
//   par_P7 = (int8_t)data[19];
//   par_P8 = (int8_t)data[20];
//   par_P9 = (int16_t)((data[16] << 8) | data[15]);
//   par_P10 = (int8_t)data[17];
//   par_P11 = (int8_t)data[18];
//   delay(250);
//   // T1 = (float)par_T1 / (float)(2^(-8));
//   // T2 = (float)par_T2 / (float)(2^30);
//   // T3 = (float)par_T3 / (float)(2^48);
//   // P1 = (float)(par_P1 - (2^14)) / (float)(2^20);
//   // P2 = (float)(par_P2 - (2^14)) / (float)(2^29);
//   // P3 = (float)par_P3 / (float)(2^32);
//   // P4 = (float)par_P4 / (float)(2^37);
//   // P5 = (float)par_P5 / (float)(2^(-3));
//   // P6 = (float)par_P6 / (float)(2^6);
//   // P7 = (float)par_P7 / (float)(2^8);
//   // P8 = (float)par_P8 / (float)(2^15);
//   // P9 = (float)par_P9 / (float)(2^48);
//   // P10 = (float)par_P10 / (float)(2^48);
//   // P11 = (float)par_P11 / (float)(2^65);
//   // T1 = (float)(par_T1 / (1 << 8));
//   // T2 = (float)(par_T2 / (1 << 30));
//   // T3 = (float)(par_T3 / (1 << 48));
//   // P1 = (float)((par_P1 - (1 << 14)) / (1 << 20));
//   // P2 = (float)((par_P2 - (1 << 14)) / (1 << 29));
//   // P3 = (float)(par_P3 / (1 << 32));
//   // P4 = (float)(par_P4 / (1 << 37));
//   // P5 = (float)(par_P5 / (1 << (-3)));
//   // P6 = (float)(par_P6 / (1 << 6));
//   // P7 = (float)(par_P7 / (1 << 8));
//   // P8 = (float)(par_P8 / (1 << 15));
//   // P9 = (float)(par_P9 / (1 << 48));
//   // P10 = (float)(par_P10 / (1 << 48));
//   // P11 = (float)(par_P11 / (1 << 65));
//   // T1 = (float)(par_T1 / pow(2, -8));
//   // T2 = (float)(par_T2 / pow(2, 30));
//   // T3 = (float)(par_T3 / pow(2, 48));
//   // P1 = (float)((par_P1 - pow(2, 14)) / pow(2, 20));
//   // P2 = (float)((par_P2 - pow(2, 14)) / pow(2, 29));
//   // P3 = (float)(par_P3 / pow(2, 32));
//   // P4 = (float)(par_P4 / pow(2, 37));
//   // P5 = (float)(par_P5 / pow(2, -3));
//   // P6 = (float)(par_P6 / pow(2, 6));
//   // P7 = (float)(par_P7 / pow(2, 8));
//   // P8 = (float)(par_P8 / pow(2, 15));
//   // P9 = (float)(par_P9 / pow(2, 48));
//   // P10 = (float)(par_P10 / pow(2, 48));
//   // P11 = (float)(par_P11 / pow(2, 65));
//   T1 = (float)(par_T1 / (2 ^ (-8)));
//   T2 = (float)(par_T2 / (2 ^ 30));
//   T3 = (float)(par_T3 / (2 ^ 48));
//   P1 = (float)((par_P1 - (2 ^ 14)) / (2 ^ 20));
//   P2 = (float)((par_P2 - (2 ^ 14)) / (2 ^ 29));
//   P3 = (float)(par_P3 / (2 ^ 32));
//   P4 = (float)(par_P4 / (2 ^ 37));
//   P5 = (float)(par_P5 / (2 ^ (-3)));
//   P6 = (float)(par_P6 / (2 ^ 6));
//   P7 = (float)(par_P7 / (2 ^ 8));
//   P8 = (float)(par_P8 / (2 ^ 15));
//   P9 = (float)(par_P9 / (2 ^ 48));
//   P10 = (float)(par_P10 / (2 ^ 48));
//   P11 = (float)(par_P11 / (2 ^ 65));
//   delay(250);

//   Serial.println("par_T1: " + String(par_T1) + "  par_T2: " + String(par_T2) + "  par_T3: " + String(par_T3) + "  par_P1: " + String(par_P1) + "  par_P2: " + String(par_P2) + "  par_P3: " + String(par_P3) + "  par_P4: " + String(par_P4) + "  par_P5: " + String(par_P5) + "  par_P6: " + String(par_P6) + "  par_P7: " + String(par_P7) + "  par_P8: " + String(par_P8) + "  par_P9: " + String(par_P9) + "  par_P10: " + String(par_P10) + "  par_P11: " + String(par_P11));
//   Serial.println("T1: " + String(T1) + "  T2: " + String(T2) + "  T3: " + String(T3) + "  P1: " + String(P1) + "  P2: " + String(P2) + "  P3: " + String(P3) + "  P4: " + String(P4) + "  P5: " + String(P5) + "  P6: " + String(P6) + "  P7: " + String(P7) + "  P8: " + String(P8) + "  P9: " + String(P9) + "  P10: " + String(P10) + "  P11: " + String(P11));

//   for (rateCalibrationNumber = 0; rateCalibrationNumber < 3000; rateCalibrationNumber++)
//   {
//     barometerSignal();
//     if (rateCalibrationNumber > 1000)
//     {
//       altitudeBarometerStartUp += altitudeBarometer;
//     }
//     delay(1);
//   }
//   altitudeBarometerStartUp /= 2000;
// }

// void loop()
// {
//   // barometerSignal();
//   // altitudeBarometer -= altitudeBarometerStartUp;
//   // Serial.println("Altitude [cm]: " + String(altitudeBarometer));
//   // delay(50);
// }





































































































































































#include <Arduino.h>
#include <Wire.h>

uint32_t loopTimer;

uint16_t par_T1, par_T2, par_P5, par_P6;
int16_t par_P1, par_P2, par_P9;
int8_t par_T3, par_P3, par_P4, par_P7, par_P8, par_P10, par_P11;
int64_t T_fine;
float T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;

float altitudeBarometer, altitudeBarometerStartUp;
int rateCalibrationNumber;

uint8_t read_register(uint8_t reg)
{
  Wire.beginTransmission(0x77);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(0x77, 1);
  uint8_t val = Wire.read();
  return val;
}

uint8_t write_register(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(0x77);
  Wire.write(reg);
  Wire.write(val);
  uint8_t result = Wire.endTransmission();
  return result; // 0 if success, 1 if error
}

uint8_t burstWrite_register(uint8_t *regs, uint8_t *vals, uint8_t len)
{
  Wire.beginTransmission(0x77);
  for (uint8_t i = 0; i < len; i++)
  {
    Wire.write(regs[i]);
    Wire.write(vals[i]);
  }
  uint8_t result = Wire.endTransmission();
  return result; // 0 if success, 1 if error
}

void barometerSignal()
{
  // Serial.println();

  // Serial.println("Value of 0x7E register: " + String(read_register(0x7E), HEX));
  // Serial.println("Value of 0x1B register: " + String(read_register(0x1B), HEX));
  // Serial.println("Value of 0x1C register: " + String(read_register(0x1C), HEX));
  // Serial.println("Value of 0x1D register: " + String(read_register(0x1D), HEX));
  // Serial.println("Value of 0x1F register: " + String(read_register(0x1F), HEX));
  // Wire.beginTransmission(0x77);
  // Wire.write(0x1B); // set power mode to normal and enable temp and press read
  // Wire.write(0x13);
  // Wire.endTransmission();
  // int powerModeTransmissionResult = Wire.endTransmission();
  // if (powerModeTransmissionResult == 0)
  // {
  //   Serial.println("Power mode command successfully sent.");
  // }
  // else
  // {
  //   Serial.println("Error sending power mode command.");
  // }
  // delay(5);
  // Serial.println("Value of 0x7E register: " + String(read_register(0x7E), HEX));
  // Serial.println("Value of 0x1B register: " + String(read_register(0x1B), HEX));
  // Serial.println("Value of 0x1C register: " + String(read_register(0x1C), HEX));
  // Serial.println("Value of 0x1D register: " + String(read_register(0x1D), HEX));
  // Serial.println("Value of 0x1F register: " + String(read_register(0x1F), HEX));

  // Wire.beginTransmission(0x77);
  // Wire.write(0x04); // read pressure and temperature data
  // int pressureNtemperatureTransmissionResult = Wire.endTransmission();
  // if (pressureNtemperatureTransmissionResult == 0)
  // {
  //   Serial.println("Pressure and temperature command successfully sent.");
  // }
  // else
  // {
  //   Serial.println("Error sending pressure and temperature command.");
  // }
  // Wire.requestFrom(0x77, 6);
  // uint8_t press_xlsb = Wire.read();
  // uint8_t press_lsb = Wire.read();
  // uint8_t press_msb = Wire.read();
  // uint8_t temp_xlsb = Wire.read();
  // uint8_t temp_lsb = Wire.read();
  // uint8_t temp_msb = Wire.read();
  uint8_t press_xlsb, press_lsb, press_msb, temp_xlsb, temp_lsb, temp_msb;
  uint8_t temp[6], j = 0;
  Wire.beginTransmission(0x77);
  Wire.write(0x04); // read pressure and temperature data
  Wire.endTransmission();
  // int pressureNtemperatureTransmissionResult = Wire.endTransmission();
  // if (pressureNtemperatureTransmissionResult == 0)
  // {
  //   Serial.println("Pressure and temperature command successfully sent.");
  // }
  // else
  // {
  //   Serial.println("Error sending pressure and temperature command.");
  // }
  Wire.requestFrom(0x77, 6);
  while (Wire.available()) temp[j++] = Wire.read();
  press_xlsb = temp[0];
  press_lsb = temp[1];
  press_msb = temp[2];
  temp_xlsb = temp[3];
  temp_lsb = temp[4];
  temp_msb = temp[5];

  // Serial.println("press_msb: " + String(press_msb) + "  press_lsb: " + String(press_lsb) + "  press_xlsb: " + String(press_xlsb) + "  temp_msb: " + String(temp_msb) + "  temp_lsb: " + String(temp_lsb) + "  temp_xlsb: " + String(temp_xlsb));

  uint32_t uncomp_temp = (temp_msb << 16) | (temp_lsb << 8) | temp_xlsb;
  uint32_t uncomp_press = (press_msb << 16) | (press_lsb << 8) | press_xlsb;

  // Serial.println("uncomp_temp: " + String(uncomp_temp) + "  uncomp_press: " + String(uncomp_press));

  float partial_data1_t, partial_data2_t;
  partial_data1_t = (float)(uncomp_temp - T1);
  partial_data2_t = (float)(partial_data1_t * T2);
  T_fine = (int64_t)(partial_data2_t + (partial_data1_t * partial_data1_t) * T3);

  float comp_press, partial_data1_p, partial_data2_p, partial_data3_p, partial_data4_p, partial_out1_p, partial_out2_p;
  partial_data1_p = (float)(P6 * T_fine);
  partial_data2_p = (float)(P7 * (T_fine * T_fine));
  partial_data3_p = (float)(P8 * (T_fine * T_fine * T_fine));
  partial_out1_p = (float)(P5 + partial_data1_p + partial_data2_p + partial_data3_p);
  partial_data1_p = (float)(P2 * T_fine);
  partial_data2_p = (float)(P3 * (T_fine * T_fine));
  partial_data3_p = (float)(P4 * (T_fine * T_fine * T_fine));
  partial_out2_p = (float)((float)uncomp_press * (P1 + partial_data1_p + partial_data2_p + partial_data3_p));
  partial_data1_p = (float)uncomp_press * (float)uncomp_press;
  partial_data2_p = (float)(P9 + (P10 * T_fine));
  partial_data3_p = (float)(partial_data1_p * partial_data2_p);
  partial_data4_p = (float)(partial_data3_p + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * P11);
  comp_press = (float)(partial_out1_p + partial_out2_p + partial_data4_p);

  float pressure = comp_press / 100.0F;
  altitudeBarometer = 44330.0 * (1.0 - pow(pressure / 1013.25, 1 / 5.255)) * 100.0;

  // Serial.println(altitudeBarometer);
  // Serial.println("comp_press: " + String(comp_press) + "  pressure: " + String(pressure) + "  altitudeBarometer: " + String(altitudeBarometer));
}

void setup()
{
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

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
  T1 = (float)(par_T1 / pow(2, -8));
  T2 = (float)(par_T2 / pow(2, 30));
  T3 = (float)(par_T3 / pow(2, 48));
  P1 = (float)((par_P1 - pow(2, 14)) / pow(2, 20));
  P2 = (float)((par_P2 - pow(2, 14)) / pow(2, 29));
  P3 = (float)(par_P3 / pow(2, 32));
  P4 = (float)(par_P4 / pow(2, 37));
  P5 = (float)(par_P5 / pow(2, -3));
  P6 = (float)(par_P6 / pow(2, 6));
  P7 = (float)(par_P7 / pow(2, 8));
  P8 = (float)(par_P8 / pow(2, 15));
  P9 = (float)(par_P9 / pow(2, 48));
  P10 = (float)(par_P10 / pow(2, 48));
  P11 = (float)(par_P11 / pow(2, 65));
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
  uint8_t reg_addr[3] = {0x1C, 0x1D, 0x1F};
  uint8_t reg_val[3] = {0x03, 0x02, 0x02};
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
  delay(5);
  if (rslt == 0) Serial.println("Power MODE command successfully sent.");
  else Serial.println("Error sending power MODE command.");
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

  for (rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++)
  {
    barometerSignal();
    altitudeBarometerStartUp += altitudeBarometer;
    delay(1);
  }
  altitudeBarometerStartUp /= 2000;

  Serial.println("Value of 0x00 INT_CTRL register is: " + String(read_register(0x00), HEX));
  Serial.println("Value of 0x1B PWR_CTRL register is: " + String(read_register(0x1B), HEX));
  Serial.println("Value of 0x1C OSR register is: " + String(read_register(0x1C), HEX));
  Serial.println("Value of 0x1D ODR register is: " + String(read_register(0x1D), HEX));
  Serial.println("Value of 0x1F CONFIG_IIR register is: " + String(read_register(0x1F), HEX));
  Serial.println("Value of 0x19 INT_CTRL register is: " + String(read_register(0x19), HEX));
  Serial.println("Value of 0x1A IF_CONF register is: " + String(read_register(0x1A), HEX));
  Serial.println();

  loopTimer = micros();
}

void loop()
{
  // barometerSignal();
  // altitudeBarometer -= altitudeBarometerStartUp;
  // Serial.println("Altitude [cm]: " + String(altitudeBarometer));
  // delay(50);

  // barometerSignal();
  // Serial.println(altitudeBarometer);
  // while (micros() - loopTimer < 4000)
  //   ;
  loopTimer = micros();
}

































































































// #include <Arduino.h>
// #include <Wire.h>

// uint32_t loopTimer;

// uint16_t par_T1, par_T2, par_P5, par_P6;
// int16_t par_P1, par_P2, par_P9;
// int8_t par_T3, par_P3, par_P4, par_P7, par_P8, par_P10, par_P11;
// int64_t T_fine;
// float T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;

// float altitudeBarometer, altitudeBarometerStartUp;
// int rateCalibrationNumber;

// uint8_t read_register(uint8_t reg)
// {
//   Wire.beginTransmission(0x77);
//   Wire.write(reg);
//   Wire.endTransmission();
//   Wire.requestFrom(0x77, 1);
//   uint8_t val = Wire.read();
//   return val;
// }

// void barometerSignal()
// {
//   // Serial.println();

//   // Serial.println("Value of 0x7E register before power mode command: " + String(read_register(0x7E), HEX));
//   // Serial.println("Value of 0x1B register before power mode command: " + String(read_register(0x1B), HEX));
//   // Serial.println("Value of 0x1C register before power mode command: " + String(read_register(0x1C), HEX));
//   // Serial.println("Value of 0x1F register before power mode command: " + String(read_register(0x1F), HEX));
//   // Wire.beginTransmission(0x77);
//   // Wire.write(0x1B); // set power mode to normal and enable temp and press read
//   // Wire.write(0x13);
//   // Wire.endTransmission();
//   // int powerModeTransmissionResult = Wire.endTransmission();
//   // if (powerModeTransmissionResult == 0)
//   // {
//   //   Serial.println("Power mode command successfully sent.");
//   // }
//   // else
//   // {
//   //   Serial.println("Error sending power mode command.");
//   // }
//   // delay(5);
//   // Serial.println("Value of 0x7E register after power mode command: " + String(read_register(0x7E), HEX));
//   // Serial.println("Value of 0x1B register after power mode command: " + String(read_register(0x1B), HEX));
//   // Serial.println("Value of 0x1C register after power mode command: " + String(read_register(0x1C), HEX));
//   // Serial.println("Value of 0x1F register after power mode command: " + String(read_register(0x1F), HEX));

//   // Wire.beginTransmission(0x77);
//   // Wire.write(0x04); // read pressure and temperature data
//   // int pressureNtemperatureTransmissionResult = Wire.endTransmission();
//   // if (pressureNtemperatureTransmissionResult == 0)
//   // {
//   //   Serial.println("Pressure and temperature command successfully sent.");
//   // }
//   // else
//   // {
//   //   Serial.println("Error sending pressure and temperature command.");
//   // }
//   // Wire.requestFrom(0x77, 6);
//   // uint8_t press_xlsb = Wire.read();
//   // uint8_t press_lsb = Wire.read();
//   // uint8_t press_msb = Wire.read();
//   // uint8_t temp_xlsb = Wire.read();
//   // uint8_t temp_lsb = Wire.read();
//   // uint8_t temp_msb = Wire.read();
//   uint8_t press_xlsb, press_lsb, press_msb, temp_xlsb, temp_lsb, temp_msb;
//   uint8_t temp[6], j = 0;
//   Wire.beginTransmission(0x77);
//   Wire.write(0x04); // read pressure and temperature data
//   Wire.endTransmission();
//   // int pressureNtemperatureTransmissionResult = Wire.endTransmission();
//   // if (pressureNtemperatureTransmissionResult == 0)
//   // {
//   //   Serial.println("Pressure and temperature command successfully sent.");
//   // }
//   // else
//   // {
//   //   Serial.println("Error sending pressure and temperature command.");
//   // }
//   Wire.requestFrom(0x77, 6);
//   while (Wire.available())
//   {
//     temp[j] = Wire.read();
//     j++;
//   }
//   press_xlsb = temp[0];
//   press_lsb = temp[1];
//   press_msb = temp[2];
//   temp_xlsb = temp[3];
//   temp_lsb = temp[4];
//   temp_msb = temp[5];

//   // Serial.println("press_msb: " + String(press_msb) + "  press_lsb: " + String(press_lsb) + "  press_xlsb: " + String(press_xlsb) + "  temp_msb: " + String(temp_msb) + "  temp_lsb: " + String(temp_lsb) + "  temp_xlsb: " + String(temp_xlsb));

//   uint32_t uncomp_temp = (temp_msb << 16) | (temp_lsb << 8) | temp_xlsb;
//   uint32_t uncomp_press = (press_msb << 16) | (press_lsb << 8) | press_xlsb;

//   // Serial.println("uncomp_temp: " + String(uncomp_temp) + "  uncomp_press: " + String(uncomp_press));

//   float partial_data1_t, partial_data2_t;
//   partial_data1_t = (float)(uncomp_temp - T1);
//   partial_data2_t = (float)(partial_data1_t * T2);
//   T_fine = (int64_t)(partial_data2_t + (partial_data1_t * partial_data1_t) * T3);

//   float comp_press, partial_data1_p, partial_data2_p, partial_data3_p, partial_data4_p, partial_out1_p, partial_out2_p;
//   partial_data1_p = (float)(P6 * T_fine);
//   partial_data2_p = (float)(P7 * (T_fine * T_fine));
//   partial_data3_p = (float)(P8 * (T_fine * T_fine * T_fine));
//   partial_out1_p = (float)(P5 + partial_data1_p + partial_data2_p + partial_data3_p);
//   partial_data1_p = (float)(P2 * T_fine);
//   partial_data2_p = (float)(P3 * (T_fine * T_fine));
//   partial_data3_p = (float)(P4 * (T_fine * T_fine * T_fine));
//   partial_out2_p = (float)((float)uncomp_press * (P1 + partial_data1_p + partial_data2_p + partial_data3_p));
//   partial_data1_p = (float)uncomp_press * (float)uncomp_press;
//   partial_data2_p = (float)(P9 + (P10 * T_fine));
//   partial_data3_p = (float)(partial_data1_p * partial_data2_p);
//   partial_data4_p = (float)(partial_data3_p + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * P11);
//   comp_press = (float)(partial_out1_p + partial_out2_p + partial_data4_p);

//   float pressure = comp_press / 100.0F;
//   altitudeBarometer = 44330.0 * (1.0 - pow(pressure / 1013.25, 1 / 5.255)) * 100.0;

//   // Serial.println(altitudeBarometer);
//   // Serial.println("comp_press: " + String(comp_press) + "  pressure: " + String(pressure) + "  altitudeBarometer: " + String(altitudeBarometer));
// }

// void setup()
// {
//   Serial.begin(9600);
//   Wire.setClock(400000);
//   Wire.begin();
//   delay(250);

//   Serial.println("Value of 0x7E register before soft reset: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register before soft reset: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register before soft reset: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register before soft reset: " + String(read_register(0x1F), HEX));
//   Wire.beginTransmission(0x77);
//   Wire.write(0x7E); // soft reset
//   Wire.write(0xB6);
//   // Wire.endTransmission();
//   int resetTransmissionResult = Wire.endTransmission();
//   if (resetTransmissionResult == 0)
//   {
//     Serial.println("Soft reset command successfully sent.");
//   }
//   else
//   {
//     Serial.println("Error sending soft reset command.");
//   }
//   // delay(5);
//   Serial.println("Value of 0x7E register after soft reset: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register after soft reset: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register after soft reset: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register after soft reset: " + String(read_register(0x1F), HEX));

//   Serial.println("Value of 0x7E register before power mode command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register before power mode command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register before power mode command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register before power mode command: " + String(read_register(0x1F), HEX));
//   Wire.beginTransmission(0x77);
//   Wire.write(0x1B); // set power mode to normal and enable temp and press read
//   Wire.write(0x33);
//   // Wire.endTransmission();
//   int powerModeTransmissionResult = Wire.endTransmission();
//   if (powerModeTransmissionResult == 0)
//   {
//     Serial.println("Power mode command successfully sent.");
//   }
//   else
//   {
//     Serial.println("Error sending power mode command.");
//   }
//   // delay(5);
//   Serial.println("Value of 0x7E register after power mode command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register after power mode command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register after power mode command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register after power mode command: " + String(read_register(0x1F), HEX));

//   // Serial.println("Value of 0x7E register before oversampling rate command: " + String(read_register(0x7E), HEX));
//   // Serial.println("Value of 0x1B register before oversampling rate command: " + String(read_register(0x1B), HEX));
//   // Serial.println("Value of 0x1C register before oversampling rate command: " + String(read_register(0x1C), HEX));
//   // Serial.println("Value of 0x1F register before oversampling rate command: " + String(read_register(0x1F), HEX));
//   // Wire.beginTransmission(0x77);
//   // Wire.write(0x1C); // set oversampling rate(OSR) to 1x for temp and 8x for press
//   // Wire.write(0x03); // *****to set*****
//   // // Wire.endTransmission();
//   // int oversamplingRateTransmissionResult = Wire.endTransmission();
//   // if (oversamplingRateTransmissionResult == 0)
//   // {
//   //   Serial.println("Oversampling rate command successfully sent.");
//   // }
//   // else
//   // {
//   //   Serial.println("Error sending oversampling rate command.");
//   // }
//   // // delay(5);
//   // Serial.println("Value of 0x7E register after oversampling rate command: " + String(read_register(0x7E), HEX));
//   // Serial.println("Value of 0x1B register after oversampling rate command: " + String(read_register(0x1B), HEX));
//   // Serial.println("Value of 0x1C register after oversampling rate command: " + String(read_register(0x1C), HEX));
//   // Serial.println("Value of 0x1F register after oversampling rate command: " + String(read_register(0x1F), HEX));

//   Serial.println("Value of 0x7E register before iir filter setting command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register before iir filter setting command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register before iir filter setting command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register before iir filter setting command: " + String(read_register(0x1F), HEX));
//   Wire.beginTransmission(0x77);
//   Wire.write(0x1F); // set IIR filter coefficient to 3
//   Wire.write(0x04);
//   // Wire.endTransmission();
//   int filterCoefficientTransmissionResult = Wire.endTransmission();
//   if (filterCoefficientTransmissionResult == 0)
//   {
//     Serial.println("Filter coefficient command successfully sent.");
//   }
//   else
//   {
//     Serial.println("Error sending filter coefficient command.");
//   }
//   // delay(5);
//   Serial.println("Value of 0x7E register after iir filter setting command: " + String(read_register(0x7E), HEX));
//   Serial.println("Value of 0x1B register after iir filter setting command: " + String(read_register(0x1B), HEX));
//   Serial.println("Value of 0x1C register after iir filter setting command: " + String(read_register(0x1C), HEX));
//   Serial.println("Value of 0x1F register after iir filter setting command: " + String(read_register(0x1F), HEX));

//   uint8_t data[21], i = 0;
//   Wire.beginTransmission(0x77); // read calibration data and store in their respective variables
//   Wire.write(0x31);
//   Wire.endTransmission();
//   // int calibrationTransmissionResult = Wire.endTransmission();
//   // if (calibrationTransmissionResult == 0)
//   // {
//   //   Serial.println("Calibration command successfully sent.");
//   // }
//   // else
//   // {
//   //   Serial.println("Error sending calibration command.");
//   // }
//   Wire.requestFrom(0x77, 21);
//   while (Wire.available())
//   {
//     data[i] = Wire.read();
//     i++;
//   }
//   // uint16_t par_T1, par_T2, par_P5, par_P6;
//   // int16_t par_P1, par_P2, par_P9;
//   // int8_t par_T3, par_P3, par_P4, par_P7, par_P8, par_P10, par_P11;
//   par_T1 = (data[1] << 8) | data[0];
//   par_T2 = (data[3] << 8) | data[2];
//   par_T3 = (int8_t)data[4];
//   par_P1 = (int16_t)((data[6] << 8) | data[5]);
//   par_P2 = (int16_t)((data[8] << 8) | data[7]);
//   par_P3 = (int8_t)data[9];
//   par_P4 = (int8_t)data[10];
//   par_P5 = ((data[12] << 8) | data[11]);
//   par_P6 = ((data[14] << 8) | data[13]);
//   par_P7 = (int8_t)data[15];
//   par_P8 = (int8_t)data[16];
//   par_P9 = (int16_t)((data[18] << 8) | data[17]);
//   par_P10 = (int8_t)data[19];
//   par_P11 = (int8_t)data[20];
//   delay(250);
//   // expected raw values
//   // T1: 27648  T2: 19342  T3: -7  P1: 7038  P2: 5258  P3: 6  P4: 1  P5: 19361  P6: 24515  P7: 3  P8: -6  P9: 4017  P10: 7  P11: -11
//   // giving raw values
//   // par_T1: 27648  par_T2: 19342  par_T3: -7  par_P1: 7038  par_P2: 5258  par_P3: 6  par_P4: 1  par_P5: 19361  par_P6: 24515
//   // par_P7: 3  par_P8: -6  par_P9: 4017  par_P10: 7  par_P11: -11

//   // not giving any values
//   // T1 = (float)par_T1 / (float)(2^(-8));
//   // T2 = (float)par_T2 / (float)(2^30);
//   // T3 = (float)par_T3 / (float)(2^48);
//   // P1 = (float)(par_P1 - (2^14)) / (float)(2^20);
//   // P2 = (float)(par_P2 - (2^14)) / (float)(2^29);
//   // P3 = (float)par_P3 / (float)(2^32);
//   // P4 = (float)par_P4 / (float)(2^37);
//   // P5 = (float)par_P5 / (float)(2^(-3));
//   // P6 = (float)par_P6 / (float)(2^6);
//   // P7 = (float)par_P7 / (float)(2^8);
//   // P8 = (float)par_P8 / (float)(2^15);
//   // P9 = (float)par_P9 / (float)(2^48);
//   // P10 = (float)par_P10 / (float)(2^48);
//   // P11 = (float)par_P11 / (float)(2^65);

//   // not working at all
//   // T1 = (float)(par_T1 / (1 << 8));
//   // T2 = (float)(par_T2 / (1 << 30));
//   // T3 = (float)(par_T3 / (1 << 48));
//   // P1 = (float)((par_P1 - (1 << 14)) / (1 << 20));
//   // P2 = (float)((par_P2 - (1 << 14)) / (1 << 29));
//   // P3 = (float)(par_P3 / (1 << 32));
//   // P4 = (float)(par_P4 / (1 << 37));
//   // P5 = (float)(par_P5 / (1 << (-3)));
//   // P6 = (float)(par_P6 / (1 << 6));
//   // P7 = (float)(par_P7 / (1 << 8));
//   // P8 = (float)(par_P8 / (1 << 15));
//   // P9 = (float)(par_P9 / (1 << 48));
//   // P10 = (float)(par_P10 / (1 << 48));
//   // P11 = (float)(par_P11 / (1 << 65));

//   // giving values around (~557.00 , ~537.00)
//   T1 = (float)(par_T1 / pow(2, -8));
//   T2 = (float)(par_T2 / pow(2, 30));
//   T3 = (float)(par_T3 / pow(2, 48));
//   P1 = (float)((par_P1 - pow(2, 14)) / pow(2, 20));
//   P2 = (float)((par_P2 - pow(2, 14)) / pow(2, 29));
//   P3 = (float)(par_P3 / pow(2, 32));
//   P4 = (float)(par_P4 / pow(2, 37));
//   P5 = (float)(par_P5 / pow(2, -3));
//   P6 = (float)(par_P6 / pow(2, 6));
//   P7 = (float)(par_P7 / pow(2, 8));
//   P8 = (float)(par_P8 / pow(2, 15));
//   P9 = (float)(par_P9 / pow(2, 48));
//   P10 = (float)(par_P10 / pow(2, 48));
//   P11 = (float)(par_P11 / pow(2, 65));

//   // giving very large negative values (~-19163244.00)
//   // T1 = (float)(par_T1 / (2 ^ (-8)));
//   // T2 = (float)(par_T2 / (2 ^ 30));
//   // T3 = (float)(par_T3 / (2 ^ 48));
//   // P1 = (float)((par_P1 - (2 ^ 14)) / (2 ^ 20));
//   // P2 = (float)((par_P2 - (2 ^ 14)) / (2 ^ 29));
//   // P3 = (float)(par_P3 / (2 ^ 32));
//   // P4 = (float)(par_P4 / (2 ^ 37));
//   // P5 = (float)(par_P5 / (2 ^ (-3)));
//   // P6 = (float)(par_P6 / (2 ^ 6));
//   // P7 = (float)(par_P7 / (2 ^ 8));
//   // P8 = (float)(par_P8 / (2 ^ 15));
//   // P9 = (float)(par_P9 / (2 ^ 48));
//   // P10 = (float)(par_P10 / (2 ^ 48));
//   // P11 = (float)(par_P11 / (2 ^ 65));
//   delay(250);

//   // Serial.println("par_T1: " + String(par_T1) + "  par_T2: " + String(par_T2) + "  par_T3: " + String(par_T3) + "  par_P1: " + String(par_P1) + "  par_P2: " + String(par_P2) + "  par_P3: " + String(par_P3) + "  par_P4: " + String(par_P4) + "  par_P5: " + String(par_P5) + "  par_P6: " + String(par_P6) + "  par_P7: " + String(par_P7) + "  par_P8: " + String(par_P8) + "  par_P9: " + String(par_P9) + "  par_P10: " + String(par_P10) + "  par_P11: " + String(par_P11));
//   // Serial.println("T1: " + String(T1) + "  T2: " + String(T2) + "  T3: " + String(T3) + "  P1: " + String(P1) + "  P2: " + String(P2) + "  P3: " + String(P3) + "  P4: " + String(P4) + "  P5: " + String(P5) + "  P6: " + String(P6) + "  P7: " + String(P7) + "  P8: " + String(P8) + "  P9: " + String(P9) + "  P10: " + String(P10) + "  P11: " + String(P11));

//   for (rateCalibrationNumber = 0; rateCalibrationNumber < 3000; rateCalibrationNumber++)
//   {
//     barometerSignal();
//     if (rateCalibrationNumber > 1000)
//     {
//       altitudeBarometerStartUp += altitudeBarometer;
//     }
//     delay(1);
//   }
//   altitudeBarometerStartUp /= 2000;

//   loopTimer = micros();
// }

// void loop()
// {
//   barometerSignal();
//   altitudeBarometer -= altitudeBarometerStartUp;
//   // Serial.print("Altitude [cm]: ");
//   Serial.println(altitudeBarometer);
//   // delay(50);

//   // barometerSignal();
//   // Serial.println(altitudeBarometer);
//   while (micros() - loopTimer < 4000);
//   loopTimer = micros();
// }
