// MLX90641.cpp file for the MLX90641.h library, version 1.0.1
// Author: D. Dubins
// Co-authors: ChatGPT 3.0, Perplexity.AI
// Date: 02-Dec-25
// Transferability: This library is designed to work on the ESP32 (and possibly the ATmega2560)
// Author: D.Dubins
// Date: 19-Dec-24
// Last Updated: 23-Dec-24
// Notes: the MLX90641 operating voltage is 3-3.6V (typical: 3.3V).
// Use a logic shifter, or connect to an MCU that operates at 3.3V (e.g. NodeMCU).
// After the device powers up and sends data, a thermal stabilization time is required
// before the device can reach the specified accuracy (up to 3 min) - 12.2.2
// Wiring: ("/\" is notch in device case, pins facing you)
//
//       _____/\______
//     /              \
//    /  4:SCL  1:SDA  \
//   |                  |
//   |                  |
//    \  3:GND  2:3.3V /
//     \______________/
//
// ESP32 - MLX90641:
// --------------------------------------
// SDA - D21 (GPIO21) - SDA
// SCL - D22 (GPIO22) - SCL
// GND -  GND
// 3.3V - VDD

#include <Wire.h>
#include "MLX90641.h"

MLX90641::MLX90641()
{ 
	Vdd = 0.0;                           // to hold calculated Vdd (measured sensor operating voltage)
	Vdd_25 = 0;                          // to store Vdd at 25°C
	K_Vdd = 0;                           // to store K_Vdd
	Ta = 0.0;                            // calculated Ta (ambient temperature)
	Kgain = 0.0;                         // Kgain coefficient
	for (int i = 0; i < NUM_PIXELS; ++i) {
		pix_OS_ref_SP0[i]=0;	         // pixel offset reference sp0
		pix_OS_ref_SP1[i]=0;	         // pixel offset reference sp1
		alpha_pixel[i]=0.f;		         // pixel sensitivity
		Kta[i]=0.f;                      // Kta[i,j] coefficients
		Kv[i]=0.f;                       // Kv[i,j] coefficients
		V_IR_compensated[i] = 0.f;       // V_IR_compensated values
	    T_o[i]=0.f;                      // Matrix to hold final T_o[i] values
	}
	KsTa=0.f;                            // KsTa coefficient
	CT1=0;                               // Corner temperatures
	CT2=0;                               // Corner temperatures
	CT3=0;                               // Corner temperatures
	CT4=0;                               // Corner temperatures
	CT5=0;                               // Corner temperatures
	CT6=0;                               // Corner temperatures
	CT7=0;                               // Corner temperatures
	CT8=0;                               // Corner temperatures
	KsTo1=0.f;                           // KsTo coefficients
	KsTo2=0.f;                           // KsTo coefficients
	KsTo3=0.f;                           // KsTo coefficients
	KsTo4=0.f;                           // KsTo coefficients
	KsTo5=0.f;                           // KsTo coefficients
	KsTo6=0.f;                           // KsTo coefficients
	KsTo7=0.f;                           // KsTo coefficients
	KsTo8=0.f;                           // KsTo coefficients
	Alpha_cr1=0.f;                       // Alpha correction coefficients for each range
	Alpha_cr2=0.f;                       // Alpha correction coefficients for each range
	Alpha_cr3=0.f;                       // Alpha correction coefficients for each range
	Alpha_cr4=0.f;                       // Alpha correction coefficients for each range
	Alpha_cr5=0.f;                       // Alpha correction coefficients for each range
	Alpha_cr6=0.f;                       // Alpha correction coefficients for each range
	Alpha_cr7=0.f;                       // Alpha correction coefficients for each range
	Alpha_cr8=0.f;                       // Alpha correction coefficients for each range
	alpha_reference_row1=0.f;            // Alpha references for sensitivity adjustment
	alpha_reference_row2=0.f;            // Alpha references for sensitivity adjustment
	alpha_reference_row3=0.f;            // Alpha references for sensitivity adjustment
	alpha_reference_row4=0.f;            // Alpha references for sensitivity adjustment
	alpha_reference_row5=0.f;            // Alpha references for sensitivity adjustment
	alpha_reference_row6=0.f;            // Alpha references for sensitivity adjustment
	Emissivity=1.0;                      // Emissivity coefficient (default: 1)
	alpha_CP=0.f;                        // Sensitivity alpha_CP coefficient
	pix_OS_ref_CP=0;                     // Offset CP (known in datasheet as Off_CP or pix_OS_ref_CP)
	Kv_CP=0.f;                           // Kv CP coefficient, because there should be one of those for sure.
	KTa_CP=0.f;                          // KTa_CP coefficient
	TGC=1.0;                             // TGC Coefficient
}

// Read the device EEPROM
bool MLX90641::readEEPROMBlock(uint16_t startAddr, uint16_t numWords, uint16_t *dest) {
  // When reading the word from I2C:
  // The first byte you read is bits 15–8 (the high byte).
  // The second byte you read is bits 7–0 (the low byte).
  // This routine will read the EEPROM contents, with the Hamming bits included.
  // It will contain all the calibration data, such as per-pixel offsets, sensitivites,
  // temperature compensation coefficients, and other parameters essential to convert
  // the raw IR sensor readings into accurate temperature values.
  for (uint16_t i = 0; i < numWords; i++) {
    Wire.beginTransmission(MLX90641_ADDR);
    uint16_t addr = startAddr + i;
    Wire.write(addr >> 8);
    Wire.write(addr & 0xFF);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MLX90641_ADDR, (uint8_t)2) != 2) return false;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    dest[i] = ((uint16_t)hi << 8) | lo;
    delayMicroseconds(5);  // small delay to help with reading at high I2C speeds
  }
  return true;
}

// Check if new data is available
bool MLX90641::isNewDataAvailable() {
  Wire.beginTransmission(MLX90641_ADDR);
  Wire.write(STATUS_ADDR >> 8);
  Wire.write(STATUS_ADDR & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(MLX90641_ADDR, 2);
  if (Wire.available() < 2) return false;

  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  uint16_t status = (hi << 8) | lo;

  return (status & (1 << 3)) != 0;
}

// Clear the new data available bit (must be done after each read)
bool MLX90641::clearNewDataBit() {
  Wire.beginTransmission(MLX90641_ADDR);
  Wire.write(STATUS_ADDR >> 8);
  Wire.write(STATUS_ADDR & 0xFF);
  Wire.write(0xFF);  // High byte
  Wire.write(0xFF);  // Low byte
  return (Wire.endTransmission() == 0);
}

// Read a 16-bit unsigned integer from RAM or EEPROM at the address readByte:
uint16_t MLX90641::readAddr_unsigned(const uint16_t readByte) {
  Wire.beginTransmission(MLX90641_ADDR);
  Wire.write(readByte >> 8);    // MSB of VDD_ADDR
  Wire.write(readByte & 0xFF);  // LSB of VDD_ADDR
  if (Wire.endTransmission(false) != 0) return -999;
  Wire.requestFrom(MLX90641_ADDR, 2);
  if (Wire.available() < 2) return -999;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  uint16_t raw = (uint16_t)((hi << 8) | lo);
  return raw;
}

// Read a 16-bit signed integer from RAM or EEPROM at the address readByte:
int16_t MLX90641::readAddr_signed(const uint16_t readByte) {
  Wire.beginTransmission(MLX90641_ADDR);
  Wire.write(readByte >> 8);    // MSB of VDD_ADDR
  Wire.write(readByte & 0xFF);  // LSB of VDD_ADDR
  if (Wire.endTransmission(false) != 0) return -999;
  Wire.requestFrom(MLX90641_ADDR, 2);
  if (Wire.available() < 2) return -999;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  int16_t raw = (int16_t)((hi << 8) | lo);
  return raw;
}

// Read a 16-bit unsigned integer from eeData at the address addr:
uint16_t MLX90641::readEEPROM_unsigned(uint16_t addr) {
  if (addr < 0x2400 || addr >= (0x2400 + EEPROM_WORDS)) {
    // Address invalid or out of bounds
    return 0;
  }
  return eeData[addr - 0x2400];
}

// Read a 16-bit signed integer from eeData at the address addr:
int16_t MLX90641::readEEPROM_signed(uint16_t addr) {  // helper function to read signed integer from eeData
  return (int16_t)MLX90641::readEEPROM_unsigned(addr);
}

float MLX90641::readVdd() {  //(From 11.1.1, worked example in 11.2.2.2)
  //Note: Resolution correction is not needed if you are sticking with the defaults of the device.
  uint16_t Resolution_EE = (readEEPROM_unsigned(0x2433) & 0x0600) / 512;           // 11.1.18: Cal resolution is bits 10 and 9 at address 2433 (Figure 14) - Example 11.2.2.1
  uint16_t Resolution_REG = (readAddr_unsigned(0x800D) & 0x0C00) / 1024;           // 11.2.2.1: Cal resolution is bits 10 and 9 at address 0x800D from memory (Figure 14) - Example 11.2.2.1
  float Resolution_corr = two_to_the(Resolution_EE) / two_to_the(Resolution_REG);  // 2^Res_EE/2^Res_REG. this number should be 1 by default
  // example value of Vdd reading from datasheet: 0xCB8A (-13430) (Table 10)
  K_Vdd = readEEPROM_signed(0x2427) & 0x07FF;   // K_Vdd register in EEPROM is 0x2427.
  if (K_Vdd > 1023) K_Vdd = K_Vdd - 2048;       // impose limits
  K_Vdd = K_Vdd * 32;                           // Multiply by 2^5. Example K_Vdd: -3136 (Table 11)
  Vdd_25 = readEEPROM_signed(0x2426) & 0x07FF;  // Vdd_25 register in EEPROM is 0x2426. Example number: -13568 (Table 11)
  if (Vdd_25 > 1023) Vdd_25 = Vdd_25 - 2048;
  Vdd_25 = Vdd_25 * 32;                 // multiply by 2^5. Example value Vdd_25: -13568 (Table 11)
  int16_t x = readAddr_signed(0x05AA);  // Vdd register in RAM is 0x05AA
  if (x > 32767) x = x - 65536;
  float Vdd_calc = (float)(((Resolution_corr * x - Vdd_25) / K_Vdd) + 3.3);  // final calculation for Vdd
#ifdef DEBUG
  Serial.print("readVDD() Resolution_EE: ");
  Serial.print(Resolution_EE);
  Serial.println(", example value: 2");  // 11.2.2.1
  Serial.print("readVDD() Resolution_REG: ");
  Serial.print(Resolution_REG);
  Serial.println(", example value: 2");  // 11.2.2.1
  Serial.print("readVDD() Resolution_corr: ");
  Serial.print(Resolution_corr);
  Serial.println(", example value: 1");  // 11.2.2.1
  Serial.print("readVDD() K_Vdd: ");
  Serial.print(K_Vdd);
  Serial.println(", example value: -3136");  // 11.2.2.1
  Serial.print("readVDD() Vdd: ");
  Serial.print(Vdd_calc, 6);
  Serial.println(", example value: 3.25599");  // 11.2.2.2
  Serial.println("Finished: read Vdd.");
#endif
  //Now we have everything we need to calculate Vdd:
  return Vdd_calc;
}

float MLX90641::readTa() {  // Read ambient temperature, datasheet, 11.1.2
                  //Kv_PTAT is in 0x242A (fixed scale 3) and 0x242B (fixed scale 12). Example: 0.005615234 (Table 11.2.1.2)
  int16_t Kv_PTAT = readEEPROM_signed(0x242B) & 0x07FF;
  if (Kv_PTAT > 1023) Kv_PTAT = Kv_PTAT - 2048;            // impose limits
  float Kv_PTAT_f = (float)Kv_PTAT / 4096.0;               // divide Kv_PTAT by 2^12 (float math, example in 11.2.2.3)
  int16_t Kt_PTAT = readEEPROM_signed(0x242A) & 0x07FF;    // read Kt_PTAT from 0x242A
  if (Kt_PTAT > 1023) Kt_PTAT = Kt_PTAT - 2048;            // impose limits
  float Kt_PTAT_f = (float)Kt_PTAT / 8.0;                  // divide by 2^3
  int16_t Vdd_i = readAddr_signed(0x05AA);                   // Read Vdd again from RAM, address 0x05AA
  if (Vdd_i > 32767) Vdd_i = Vdd_i - 65536;                      // impose limits
  float dV = ((float)Vdd_i - (float)Vdd_25) / (float)K_Vdd;  // calculate the change in voltage dV
  uint16_t V_PTAT25 = 32 * (readEEPROM_unsigned(0x2428) & 0x07FF) + (readEEPROM_unsigned(0x2429) & 0x07FF);
  int16_t V_PTAT = readAddr_signed(0x05A0);                                              // get V_PTAT at addr 0x05A0
  if (V_PTAT > 32767) V_PTAT = V_PTAT - 65536;                                           // impose limits
  int16_t V_BE = readAddr_signed(0x0580);                                                // get V_BE at addr 0x0580
  if (V_BE > 32767) V_BE = V_BE - 65536;                                                 // impose limits
  float Alpha_PTAT = (readEEPROM_unsigned(0x242C) & 0x07FF) / 128.0;                     // divide answer by 2^7 (=128)
  float V_PTATart = ((float)V_PTAT / ((float)V_PTAT * Alpha_PTAT + V_BE)) * 262144.0;    // multiply by 2^18 = 262144
  float Ta_calc = ((V_PTATart / (1.0 + Kv_PTAT_f * dV) - V_PTAT25) / Kt_PTAT_f) + 25.0;  // final calculation for Ta
#ifdef DEBUG
  Serial.print("readTa() Ta: ");
  Serial.print(Ta_calc, 2);
  Serial.println(", example value: 21.0");  // 11.2.2.3 (gives 42.75 °C, but my lab isn't that hot)
  Serial.println("Finished: read Ta, ambient temperature.");
#endif
  return Ta_calc;  // return calculated ambient temperature
}

float MLX90641::readKgain() {  // calculate the Kgain coefficient, datasheet 11.1.7. This needs to be calculated once per frame, because it might change in RAM.
  uint16_t GAIN = 32 * (readEEPROM_unsigned(0x2424) & 0x07FF) + (readEEPROM_unsigned(0x2425) & 0x07FF);
  int16_t x = readAddr_signed(0x058A);        // example value: 9734
  if (x > 32767) x = x - 65536;               // impose limits
  float Kgain_calc = (float)GAIN / (float)x;  // final calculation for Kgain
#ifdef DEBUG
  Serial.print("readKgain() GAIN: ");
  Serial.println(GAIN);  // example value: 9972 (11.2.2.4)
  Serial.print("readKgain() Kgain: ");
  Serial.print(Kgain_calc, 8);
  Serial.println(", example value: 1.02445038");  // 11.2.2.4
  Serial.println("Finished: read Kgain coefficient.");
#endif
  return Kgain_calc;
}

void MLX90641::readPixelOffset() {  // this function fills up the pixel offset variables: pix_OS_ref_SP0 and pix_OS_ref_SP1. datasheet 11.1.3
  // There are 2 pixel offset subpages, which have the memory dimensions 16 x 12 (no coincidences here.)
  // Subpage 1 starts at 0x2440 (words 0-F) and goes to 0x24F0 (words 0-F).
  // Subpage 2 starts at 0x2680 (words 0-F) and goes to 0x2730.
  // These are all in the EEPROM, which has been read into memory already.
  uint16_t Offset_scale = (readEEPROM_unsigned(0x2410) & 0x07E0) / 32;                                        // divide by 2^5. Example value: 0/32 = 0.
  int16_t Offset_average = 32 * (readEEPROM_signed(0x2411) & 0x07FF) + (readEEPROM_signed(0x2412) & 0x07FF);  // this is Pix_ox_r1 in Table 11.2.1.2
  if (Offset_average > 32767) Offset_average = Offset_average - 65536;                                        // impose limits
  // parse Pixel offset - subpage 1
  for (uint16_t i = 0; i < 192; i++) {
    int16_t offset_SP0 = readEEPROM_signed(0x2440 + i) & 0x07FF;                 // get offset_SP0 (subpage 1, starting at 0x2440)
    if (offset_SP0 > 1023) offset_SP0 = offset_SP0 - 2048;                       // impose limits. Example value: -670 LSB (11.2.2.5.2)
    int16_t offset_SP1 = readEEPROM_signed(0x2680 + i) & 0x07FF;                 // get offset_SP1 (subpage 2, starting at 0x2680)
    if (offset_SP1 > 1023) offset_SP1 = offset_SP1 - 2048;                       // impose limits
    pix_OS_ref_SP0[i] = Offset_average + offset_SP0 * two_to_the(Offset_scale);  // pix_OS_ref_SP0[i] = Offset_average + offset_SP0[i] * 2^Offset_scale
    pix_OS_ref_SP1[i] = Offset_average + offset_SP1 * two_to_the(Offset_scale);  // pix_OS_ref_SP1[i] = Offset_average + offset_SP1[i] * 2^Offset_scale
  }
#ifdef DEBUG
  Serial.print("readPixelOffset() pix_OS_ref_SP0[95]: ");
  Serial.print(pix_OS_ref_SP0[95]);
  Serial.println(", example value: -673 LSB");  // 11.2.2.5.2
  Serial.print("readPixelOffset() pix_OS_ref_SP1[95]: ");
  Serial.print(pix_OS_ref_SP1[95]);
  Serial.println(", example value: -671 LSB");  // 11.2.2.5.3
  Serial.println("Finished: read pixel offsets.");
#endif
}

void MLX90641::readAlpha() {  // this function restores the sensitivity from EEPROM data (11.1.4), and fills alpha_pixel[].
  // Sensitivity is divided into 6 ranges (1…32, 33…64 and so on) and for each range we store a reference value.
  // Pixel sensitivity (alpha) is stored in RAM, from 0x2500 to 0x25C0
  int16_t alpha_scale_row1 = ((readEEPROM_signed(0x2419) & 0x07E0) / 32) + 20;  // row 1
  alpha_reference_row1 = (float)(readEEPROM_signed(0x241C) & 0x07FF) / (float)pow(2, (uint32_t)alpha_scale_row1);
  int16_t alpha_scale_row2 = (readEEPROM_signed(0x2419) & 0x001F) + 20;  // row 2
  alpha_reference_row2 = (float)(readEEPROM_signed(0x241D) & 0x07FF) / (float)pow(2, (uint32_t)alpha_scale_row2);
  int16_t alpha_scale_row3 = ((readEEPROM_signed(0x241A) & 0x07E0) / 32) + 20;  // row 3
  alpha_reference_row3 = (float)(readEEPROM_signed(0x241E) & 0x07FF) / (float)pow(2, (uint32_t)alpha_scale_row3);
  int16_t alpha_scale_row4 = (readEEPROM_signed(0x241A) & 0x001F) + 20;  // row 4
  alpha_reference_row4 = (float)(readEEPROM_signed(0x241F) & 0x07FF) / (float)pow(2, (uint32_t)alpha_scale_row4);
  int16_t alpha_scale_row5 = ((readEEPROM_signed(0x241B) & 0x07E0) / 32) + 20;  // row 5
  alpha_reference_row5 = (float)(readEEPROM_signed(0x2420) & 0x07FF) / (float)pow(2, (uint32_t)alpha_scale_row5);
  int16_t alpha_scale_row6 = (readEEPROM_signed(0x241B) & 0x001F) + 20;  // row 6
  alpha_reference_row6 = (float)(readEEPROM_signed(0x2421) & 0x07FF) / (float)pow(2, (uint32_t)alpha_scale_row6);
  // Sensitivity Max value for row 1 (pixels 1…32) is stored at EEPROM address 0x241C
  for (int i = 0; i < 32; i++) alpha_pixel[i] = alpha_reference_row1 * (float)(readEEPROM_signed(0x2500 + i) & 0x07FF) / 2047.0;
  // Sensitivity Max value for row 2 (pixels 33…64) is stored at EEPROM address 0x241D
  for (int i = 32; i < 64; i++) alpha_pixel[i] = alpha_reference_row2 * (float)(readEEPROM_signed(0x2500 + i) & 0x07FF) / 2047.0;
  // Sensitivity Max value for row 3 (pixels 65…96) is stored at EEPROM address 0x241E
  for (int i = 64; i < 96; i++) alpha_pixel[i] = alpha_reference_row3 * (float)(readEEPROM_signed(0x2500 + i) & 0x07FF) / 2047.0;
  // Sensitivity Max value for row 4 (pixels 97…128) is stored at EEPROM address 0x241F
  for (int i = 96; i < 128; i++) alpha_pixel[i] = alpha_reference_row4 * (float)(readEEPROM_signed(0x2500 + i) & 0x07FF) / 2047.0;
  // Sensitivity Max value for row 5 (pixels 129…160) is stored at EEPROM address 0x2420
  for (int i = 128; i < 160; i++) alpha_pixel[i] = alpha_reference_row5 * (float)(readEEPROM_signed(0x2500 + i) & 0x07FF) / 2047.0;
  // Sensitivity Max value for row 6 (pixels 161…192) is stored at EEPROM address 0x2421
  for (int i = 160; i < 192; i++) alpha_pixel[i] = alpha_reference_row6 * (float)(readEEPROM_signed(0x2500 + i) & 0x07FF) / 2047.0;
  // Read alpha_pixel (Pixel sensitivities, starting at address 0x2550) - 11.2.2.8
  for (int i = 0; i < NUM_PIXELS; i++) {
    alpha_pixel[i] = readEEPROM_unsigned(0x2550 + i) & 0x07FF;  //11.2.2.8
  }
#ifdef DEBUG
  Serial.print("readAlpha() alpha_scale_row3: ");
  Serial.print(alpha_scale_row3);
  Serial.println(", example value: 32");  // 11.2.2.8
  Serial.print("readAlpha() alpha_reference_row3: ");
  Serial.print(float2exp(alpha_reference_row3, 6));
  Serial.println(", example value: 0.000000345520675182343");  // 11.2.2.8
  Serial.print("readAlpha() alpha_pixel[95]: ");
  Serial.print(float2exp(alpha_pixel[95], 6));
  Serial.println(", example value: 0.000000345520675182343");  // 11.2.2.8
  Serial.println("Finished: read pixel sensitivities.");
#endif
}

// To restore the Kta coefficients, 11.1.5 (fills Kta[]). In EEPROM, Kta values range from 0x25C0 to 0x267F.
void MLX90641::readKta() {
  uint16_t Kta_scale1 = (readEEPROM_unsigned(0x2416) & 0x07E0) / 32;  // divide by 2^5
  uint16_t Kta_scale2 = (readEEPROM_unsigned(0x2416) & 0x001F);
  int16_t Kta_average = readEEPROM_signed(0x2415) & 0x07FF;
  if (Kta_average > 1023) Kta_average = Kta_average - 2048;  // impose limits
  for (uint16_t i = 0; i < 192; i++) {
    int16_t Kta_EE = (readEEPROM_signed(0x25C0 + i) & 0x07E0) / 32;  // divide by 2^5
    if (Kta_EE > 31) Kta_EE = Kta_EE - 64;                           // impose limits
    MLX90641::Kta[i] = ((float)Kta_EE * two_to_the(Kta_scale2) + (float)Kta_average) / two_to_the(Kta_scale1);
  }
#ifdef DEBUG
  Serial.print("readKta() Kta_average: ");
  Serial.print(Kta_average);
  Serial.println(", example value: 765");  // 11.2.2.5.3
  Serial.print("readKta() Kta_scale1: ");
  Serial.print(Kta_scale1);
  Serial.println(", example value: 18 (unsigned)");  // 11.2.2.5.3
  Serial.print("readKta() Kta_scale2: ");
  Serial.print(Kta_scale2);
  Serial.println(", example value: 3 (unsigned)");  // 11.2.2.5.3
  Serial.print("readKta() Kta[95]: ");
  Serial.print(MLX90641::Kta[95], 9);
  Serial.println(", example value: 0.003101349");  // 11.2.2.5.3
  Serial.println("Finished: read Kta coefficients.");
#endif
}

// To restore the Kv coefficients, 11.1.6 (fills Kv[]). In EEPROM, Kv values range from 0x25C0 to 0x267F.
void MLX90641::readKv() {
  uint16_t Kv_scale1 = (readEEPROM_unsigned(0x2418) & 0x07E0) / 32;  // divide by 2^5
  uint16_t Kv_scale2 = (readEEPROM_unsigned(0x2418) & 0x001F);
  int16_t Kv_average = readEEPROM_signed(0x2417) & 0x07FF;
  if (Kv_average > 1023) Kv_average = Kv_average - 2048;  // impose limits
  for (uint16_t i = 0; i < 192; i++) {
    int16_t Kv_EE = (readEEPROM_signed(0x25C0 + i) & 0x001F);
    if (Kv_EE > 15) Kv_EE = Kv_EE - 32;  //impose limits
    Kv[i] = (((float)Kv_EE * two_to_the(Kv_scale2) + (float)Kv_average)) / two_to_the(Kv_scale1);
  }
#ifdef DEBUG
  Serial.print("readKv() Kv_average: ");
  Serial.print(Kv_average);
  Serial.println(", example value: 666");  // 11.2.2.5.3
  Serial.print("readKv() Kv_scale1: ");
  Serial.print(Kv_scale1);
  Serial.println(", example value: 11 (unsigned)");  // 11.2.2.5.3
  Serial.print("readKta() Kv_scale2: ");
  Serial.print(Kv_scale2);
  Serial.println(", example value: 4 (unsigned)");  // 11.2.2.5.3
  Serial.print("readKv() Kv[95]: ");
  Serial.print(Kv[95], 9);
  Serial.println(", example value: 0.3251953");  // 11.2.2.5.3
  Serial.println("Finished: read Kv coefficients.");
#endif
}

// To restore the KsTa coefficient, 11.1.8
float MLX90641::readKsTa() {
  int16_t ksta = readEEPROM_signed(0x2422) & 0x07FF;  // read KsTa at address 0x2422
  if (ksta > 1023) ksta = ksta - 2048;                // impose limits
  float ksta_calc = ((float)ksta / 32768.0);          // final calculation for KsTa
#ifdef DEBUG
  Serial.print("readKsTa() KsTa: ");
  Serial.print(ksta_calc, 12);
  Serial.println(", example value: -0.002197265625");  // 11.2.2.8
  Serial.println("Finished: read KsTa coefficient.");
#endif
  return ksta_calc;  // divide answer by 2^15
}

// To restore the corner temperatures (CT1..CT8), 11.1.9
void MLX90641::readCT() {
  CT1 = -40;  // hard-coded
  CT2 = -20;  // hard-coded
  CT3 = 0;    // hard-coded
  CT4 = 80;   // hard-coded
  CT5 = 120;  // hard-coded
  CT6 = readEEPROM_unsigned(0x243A) & 0x07FF;
  CT7 = readEEPROM_unsigned(0x243C) & 0x07FF;
  CT8 = readEEPROM_unsigned(0x243E) & 0x07FF;
#ifdef DEBUG
  Serial.print("readCT() CT6: ");
  Serial.print(CT6);
  Serial.println(", example value: 200");  // 11.2.2.9.1.1
  Serial.print("readCT() CT7: ");
  Serial.print(CT7);
  Serial.println(", example value: 400");  // 11.2.2.9.1.1
  Serial.print("readCT() CT8: ");
  Serial.print(CT8);
  Serial.println(", example value: 600");  // 11.2.2.9.1.1
  Serial.println("Finished: read corner temperatures.");
#endif
}

// To restore the KsTo coefficients, 11.1.10
void MLX90641::readKsTo() {
  // Addresses: KsTo1..KsTo8 are 0x2435 ..0x2439, and 0x243B, 0x243D, 0x243F
  int16_t x;                                                   // to hold numerators
  uint16_t KsTo_scale = readEEPROM_unsigned(0x2434) & 0x07FF;  // unsigned
  x = (readEEPROM_signed(0x2435) & 0x07FF);
  if (x > 1023) x = x - 2048;
  KsTo1 = (float)x / two_to_the(KsTo_scale);
  x = (readEEPROM_signed(0x2436) & 0x07FF);
  if (x > 1023) x = x - 2048;
  KsTo2 = (float)x / two_to_the(KsTo_scale);
  x = (readEEPROM_signed(0x2437) & 0x07FF);
  if (x > 1023) x = x - 2048;
  KsTo3 = (float)x / two_to_the(KsTo_scale);
  x = (readEEPROM_signed(0x2438) & 0x07FF);
  if (x > 1023) x = x - 2048;
  KsTo4 = (float)x / two_to_the(KsTo_scale);
  x = (readEEPROM_signed(0x2439) & 0x07FF);
  if (x > 1023) x = x - 2048;
  KsTo5 = (float)x / two_to_the(KsTo_scale);
  x = (readEEPROM_signed(0x243B) & 0x07FF);
  if (x > 1023) x = x - 2048;
  KsTo6 = (float)x / two_to_the(KsTo_scale);
  x = (readEEPROM_signed(0x243D) & 0x07FF);
  if (x > 1023) x = x - 2048;
  KsTo7 = (float)x / two_to_the(KsTo_scale);
  x = (readEEPROM_signed(0x243F) & 0x07FF);
  if (x > 1023) x = x - 2048;
  KsTo8 = (float)x / two_to_the(KsTo_scale);

#ifdef DEBUG
  Serial.print("readKsTo() KsTo_scale: ");
  Serial.print(KsTo_scale);
  Serial.println(", example value: 20");  // 11.2.2.9
  Serial.print("readKsTo() KsTo1: ");
  Serial.print(KsTo1, 7);
  Serial.println(", example value: -0.000699997");  // 11.2.2.9.1.2
  Serial.print("readKsTo() KsTo2: ");
  Serial.print(KsTo2, 7);
  Serial.println(", example value: -0.000699997");  // 11.2.2.9.1.2
  Serial.print("readKsTo() KsTo3: ");
  Serial.print(KsTo3, 7);
  Serial.println(", example value: -0.000699997");  // 11.2.2.9
  Serial.print("readKsTo() KsTo4: ");
  Serial.print(KsTo4, 7);
  Serial.println(", example value: -0.000699997");  // 11.2.2.9.1.2
  Serial.print("readKsTo() KsTo5: ");
  Serial.print(KsTo5, 7);
  Serial.println(", example value: -0.000699997");  // 11.2.2.9.1.2
  Serial.print("readKsTo() KsTo6: ");
  Serial.print(KsTo6, 7);
  Serial.println(", example value: -0.000699997");  // 11.2.2.9.1.2
  Serial.print("readKsTo() KsTo7: ");
  Serial.print(KsTo7, 7);
  Serial.println(", example value: -0.000699997");  // 11.2.2.9.1.2
  Serial.print("readKsTo() KsTo8: ");
  Serial.print(KsTo8, 7);
  Serial.println(", example value: -0.000699997");  // 11.2.2.9.1.2
  Serial.println("Finished: read KsTo coefficients.");
#endif
}

// To restore the Sensitivity Correction coefficients for each temperature range, 11.1.11
void MLX90641::readAlphaCorrRange() {
  Alpha_cr2 = 1.0 / (1.0 + KsTo2 * (float)(CT3 - (CT2)));
  Alpha_cr1 = Alpha_cr2 / (1.0 + KsTo1 * (float)(CT2 - (CT1)));
  Alpha_cr3 = 1.0;  // hard-coded
  Alpha_cr4 = (1.0 + KsTo3 * (float)(CT4 - CT3));
  Alpha_cr5 = (1.0 + KsTo4 * (float)(CT5 - CT4)) * Alpha_cr4;
  Alpha_cr6 = (1.0 + KsTo5 * (float)(CT6 - CT5)) * Alpha_cr5;
  Alpha_cr7 = (1.0 + KsTo6 * (float)(CT7 - CT6)) * Alpha_cr6;
  Alpha_cr8 = (1.0 + KsTo7 * (float)(CT8 - CT7)) * Alpha_cr7;
#ifdef DEBUG
  Serial.print("readAlphaCorrRange() Alpha_cr1: ");
  Serial.print(Alpha_cr1, 9);
  Serial.println(", example value: 1.028599");  // 11.2.2.9.1.1
  Serial.print("readAlphaCorrRange() Alpha_cr2: ");
  Serial.print(Alpha_cr2, 9);
  Serial.println(", example value: 1.014198721");  // 11.2.2.9.1.1
  Serial.print("readAlphaCorrRange() Alpha_cr3: ");
  Serial.print(Alpha_cr3, 2);
  Serial.println(", example value: 1");  // 11.2.2.9.1.1
  Serial.print("readAlphaCorrRange() Alpha_cr4: ");
  Serial.print(Alpha_cr4, 6);
  Serial.println(", example value: 0.94400024");  // 11.2.2.9.1.1
  Serial.print("readAlphaCorrRange() Alpha_cr5: ");
  Serial.print(Alpha_cr5, 6);
  Serial.println(", example value: 0.917568347");  // 11.2.2.9.1.1
  Serial.print("readAlphaCorrRange() Alpha_cr6: ");
  Serial.print(Alpha_cr6, 6);
  Serial.println(", example value: 0.86618474");  // 11.2.2.9.1.1
  Serial.print("readAlphaCorrRange() Alpha_cr7: ");
  Serial.print(Alpha_cr7, 6);
  Serial.println(", example value: 0.744919396");  // 11.2.2.9.1.1
  Serial.print("readAlphaCorrRange() Alpha_cr8: ");
  Serial.print(Alpha_cr8, 6);
  Serial.println(", example value: 0.640631128");  // 11.2.2.9.1.1
  Serial.println("Finished: read sensitivity correction coefficients.");
#endif
}

// To restore the Emissivity coefficient, 11.1.12
float MLX90641::readEmissivity() {
  int16_t em = readEEPROM_signed(0x2423) & 0x07FF;  // read Emissivity at address 0x2423
  if (em > 1023) em = em - 2048;                    // impose limits
  float emissivity_calc = ((float)em / 512.0);      // final calculation for emissivity (divide by 2^9)
#ifdef DEBUG
  Serial.print("readEmissivity() em: ");
  Serial.print(em);
  Serial.println(", example value: 486");  // 11.2.2.5.4
  Serial.print("readEmissivity() Emissivity: ");
  Serial.print(emissivity_calc, 6);
  Serial.println(", example value: 0.949218");  // 11.2.2.5.4
  Serial.println("Finished: read Emissivity coefficient.");
#endif
  return emissivity_calc;
}

// To restore Sensitivity alpha_CP, 11.1.13
float MLX90641::readAlpha_CP() {
  int16_t alpha_scale_CP = readEEPROM_signed(0x242E) & 0x07FF;  // read alpha_scale_CP at address 0x242E
  float numerator = (float)(readEEPROM_signed(0x242D) & 0x07FF);
  float alphacp_calc = numerator / two_to_the(alpha_scale_CP);  // final calculation for alpha_CP. Divide by 2^alpha_scale_CP
#ifdef DEBUG
  Serial.print("readAlpha_CP() numerator: ");
  Serial.print(numerator);
  Serial.println(", example value: 830");  // 11.2.2.8
  Serial.print("readAlpha_CP() Alpha_scale_CP: ");
  Serial.print(alpha_scale_CP);
  Serial.println(", example value: 38");  // 11.2.2.8
  Serial.print("readAlpha_CP() alpha_CP: ");
  Serial.print(float2exp(alphacp_calc, 5));
  Serial.println(", example value: 3.01952240988612E-9");  // 11.2.2.8. This is also called "alpha_CP in the example."
  Serial.println("Finished: read alpha_CP.");
#endif
  return alphacp_calc;
}

// To restore offset of the CP, 11.1.14, example 11.2.2.6.2
int16_t MLX90641::readOff_CP() {
  int16_t offcp = 32 * (readEEPROM_signed(0x242F) & 0x07FF) + (readEEPROM_signed(0x2430) & 0x07FF);  // read signed offset cp
  if (offcp > 32767) offcp = offcp - 65536;                                                          // impose limits
#ifdef DEBUG
  Serial.print("readOff_CP() pix_OS_ref_CP: ");
  Serial.print(offcp);
  Serial.println(", example value: -119");  // 11.2.2.6.2 (pix_osref_CP)
  Serial.println("Finished: read Off_CP.");
#endif
  return offcp;
}

// To restore Kv_CP coefficient, 11.1.15, example 11.2.2.6.2
float MLX90641::readKv_CP() {
  int16_t Kv_CP_EE = readEEPROM_signed(0x2432) & 0x003F;            // read Kv_CP_EE
  if (Kv_CP_EE > 31) Kv_CP_EE = Kv_CP_EE - 64;                      // impose limits
  uint16_t Kv_scale = (readEEPROM_unsigned(0x2432) & 0x07C0) / 64;  // divide by 2^6
  float kvcp_calc = (float)Kv_CP_EE / two_to_the(Kv_scale);              // final calculation for Kv_CP_EE. Divide by 2^Kv_scale
#ifdef DEBUG
  Serial.print("readKv_CP() Kv_CP: ");
  Serial.print(kvcp_calc, 4);
  Serial.println(", example value: 0.3125");  // 11.2.2.6.2
  Serial.println("Finished: read Kv_CP.");
#endif
  return kvcp_calc;
}

// To restore KTa_CP coefficient, 11.1.16, example 11.2.2.6.2
float MLX90641::readKTa_CP() {
  int16_t KTa_CP_EE = readEEPROM_signed(0x2431) & 0x003F;             // read KTa_CP_EE (signed)
  if (KTa_CP_EE > 31) KTa_CP_EE = KTa_CP_EE - 64;                     // impose limits
  uint16_t KTa_scale1 = (readEEPROM_unsigned(0x2431) & 0x07C0) / 64;  // divide by 2^6
  float ktacp_calc = (float)KTa_CP_EE / two_to_the(KTa_scale1);       // final calulation for KTa_CP. Divide by 2^Kv_scale
#ifdef DEBUG
  Serial.print("readKTa_CP() KTa_CP: ");
  Serial.print(ktacp_calc, 10);
  Serial.println(", example value: 0.0023193359");  // 11.2.2.6.2
  Serial.println("Finished: read KTa_CP.");
#endif
  return ktacp_calc;
}

// To restore TGC coefficient, 11.1.17, example 11.2.2.7
float MLX90641::readTGC() {
  int16_t TGC_EE = readEEPROM_signed(0x2433) & 0x01FF;  // read TGC (signed)
  if (TGC_EE > 255) TGC_EE = TGC_EE - 512;              // impose limits
  float tgc_calc = (float)(TGC_EE / 64);                // final calculation for TCG. Divide by 2^6
#ifdef DEBUG
  Serial.print("readTGC() TCG_EE: ");
  Serial.print(TGC_EE);
  Serial.println(", example value: 0");  // 11.2.2.7
  Serial.print("readTGC() TCG: ");
  Serial.print(tgc_calc);
  Serial.println(", example value: 0");  // 11.2.2.7
  Serial.println("Finished: read TCG.");
#endif
  return tgc_calc;
}

// After importing and calculating all constants, we are ready to take a temperature reading.
void MLX90641::readTempC() {      // take a temperature reading of all pixels
  Kgain = readKgain();  // This needs to happen in the loop
  Vdd = readVdd();      // Re-read Vdd
  Ta = readTa();        // Re-read Ta
  // Gain compensation - 11.2.2.5.1
  // The pixel data is a bit wonky in memory. Here is the map: (10.6.2)
  // Pixels 1..32 subpage 0: 0x0400..0x041F
  // Pixels 1..32 subpage 1: 0x0420..0x043F
  // Pixels 33..64 subpage 0: 0x0440..0x045F
  // Pixels 33..64 subpage 1: 0x0460..0x047F
  // Pixels 65..96 subpage 0: 0x0480..0x049F
  // Pixels 65..96 subpage 1: 0x04A0..0x04BF
  // Pixels 97..128 subpage 0: 0x04C0..0x04DF
  // Pixels 65..96 subpage 1: 0x04E0..0x04FF
  // Pixels 129..160 subpage 0: 0x0500..0x051F
  // Pixels 129..160 subpage 1: 0x0520..0x053F
  // Pixels 161..192 subpage 0: 0x0540..0x055F
  // Pixels 161..192 subpage 1: 0x0560..0x057F

  uint8_t subpage = readAddr_unsigned(STATUS_ADDR) & 0x01;  // read current subpage
  float alpha_comp[NUM_PIXELS] = { 0.0 };

  // Compensating gain of CP pixel - 11.2.2.6.1 - only need this once
  int16_t CP = readAddr_signed(0x0588);   // read CP at address 0x0580 (Example data: -105)
  if (CP < 32767) CP = CP - 65536;        //impose limits
  float CP_pix_gain = (float)CP * Kgain;  // final equation for CP_pix_gain

  if (subpage == 0) {
    // Gain compensation - 11.2.2.5.1
    float pix_gain_S0[NUM_PIXELS] = { 0.0 };  // to store pixel gain.

    for (int i = 0; i < NUM_PIXELS; i++) {
      int16_t x1 = readAddr_signed(pix_addr_S0(i));  // read pixel data (sp0) for pixel
      if (x1 < 32767) x1 = x1 - 65536;               //impose limits
      pix_gain_S0[i] = (float)x1 * Kgain;
    }

    // IR data compensation - 11.2.2.5.3. Ta0 = 25 (°C), VddV0=3.3
	float pix_OS_SP0[NUM_PIXELS] = { 0.0 };
    for (int i = 0; i < NUM_PIXELS; i++) {
      pix_OS_SP0[i] = pix_gain_S0[i] - (float)pix_OS_ref_SP0[i] * (1.0 + MLX90641::Kta[i] * (Ta - 25.0)) * (1.0 + Kv[i] * (Vdd - 3.3));
    }

    // Compensating offset, Ta and Vdd of CP pixel - 11.2.2.6.2
    float CP_pix_OS = CP_pix_gain - pix_OS_ref_CP * (1.0 + KTa_CP * (Ta - 25.0)) * (1.0 + Kv_CP * (Vdd - 3.3));
    for (int i = 0; i < NUM_PIXELS; i++) {
      V_IR_compensated[i] = (pix_OS_SP0[i] - (TGC * CP_pix_OS)) / Emissivity;  //11.2.2.7
    }

    // Normalizing to sensitivity - 11.2.2.8
    float alpha_SP0[NUM_PIXELS] = { 0.0 };
    // Scaling for Row 1:
    for (int i = 0; i < 32; i++) {
      alpha_SP0[i] = (float)alpha_reference_row1 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Scaling for Row 2:
    for (int i = 32; i < 64; i++) {
      alpha_SP0[i] = (float)alpha_reference_row2 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Scaling for Row 3:
    for (int i = 64; i < 96; i++) {
      alpha_SP0[i] = (float)alpha_reference_row3 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Scaling for Row 4:
    for (int i = 96; i < 128; i++) {
      alpha_SP0[i] = (float)alpha_reference_row4 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Scaling for Row 5:
    for (int i = 128; i < 160; i++) {
      alpha_SP0[i] = (float)alpha_reference_row5 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Scaling for Row 6:
    for (int i = 160; i < 192; i++) {
      alpha_SP0[i] = (float)alpha_reference_row6 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Define and calculate alpha_comp[i] - 11.2.2.8
    for (int i = 0; i < NUM_PIXELS; i++) {
      alpha_comp[i] = (alpha_SP0[i] - TGC * alpha_CP) * (1.0 + KsTa * (Ta - 25.0));
    }
  }  // end subpage 0

  // Now to read Subpage 1
  if (subpage == 1) {
    // Gain compensation - 11.2.2.5.1
    float pix_gain_S1[NUM_PIXELS] = { 0.0 };  // to store pixel gain.

    for (int i = 0; i < NUM_PIXELS; i++) {
      int16_t x1 = readAddr_signed(pix_addr_S1(i));  // read pixel data (sp0) for pixel
      if (x1 < 32767) x1 = x1 - 65536;               //impose limits
      pix_gain_S1[i] = (float)x1 * Kgain;
    }

    // IR data compensation - 11.2.2.5.3. Ta0 = 25 (°C), VddV0=3.3
	float pix_OS_SP1[NUM_PIXELS] = { 0.0 };
    for (int i = 0; i < NUM_PIXELS; i++) {
      pix_OS_SP1[i] = pix_gain_S1[i] - (float)pix_OS_ref_SP1[i] * (1.0 + MLX90641::Kta[i] * (Ta - 25.0)) * (1.0 + Kv[i] * (Vdd - 3.3));
    }

    // Compensating offset, Ta and Vdd of CP pixel - 11.2.2.6.2
    float CP_pix_OS = CP_pix_gain - pix_OS_ref_CP * (1.0 + KTa_CP * (Ta - 25.0)) * (1.0 + Kv_CP * (Vdd - 3.3));
    for (int i = 0; i < NUM_PIXELS; i++) {
      V_IR_compensated[i] = (pix_OS_SP1[i] - (TGC * CP_pix_OS)) / Emissivity;  //11.2.2.7
    }

    // Normalizing to sensitivity - 11.2.2.8
    float alpha_SP1[NUM_PIXELS] = { 0.0 };
    // Scaling for Row 1:
    for (int i = 0; i < 32; i++) {
      alpha_SP1[i] = (float)alpha_reference_row1 * (float)alpha_pixel[i] / 2047.0f;  // 2^11 - 1 = 2047.0f
    }
    // Scaling for Row 2:
    for (int i = 32; i < 64; i++) {
      alpha_SP1[i] = (float)alpha_reference_row2 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Scaling for Row 3:
    for (int i = 64; i < 96; i++) {
      alpha_SP1[i] = (float)alpha_reference_row3 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Scaling for Row 4:
    for (int i = 96; i < 128; i++) {
      alpha_SP1[i] = (float)alpha_reference_row4 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Scaling for Row 5:
    for (int i = 128; i < 160; i++) {
      alpha_SP1[i] = (float)alpha_reference_row5 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Scaling for Row 6:
    for (int i = 160; i < 192; i++) {
      alpha_SP1[i] = (float)alpha_reference_row6 * (float)alpha_pixel[i] / 2047.0f;
    }
    // Define and calculate alpha_comp[i] - 11.2.2.8
    for (int i = 0; i < NUM_PIXELS; i++) {
      alpha_comp[i] = (alpha_SP1[i] - TGC * alpha_CP) * (1.0 + KsTa * (Ta - 25.0));
    }
  }  // end subpage 1
  // Calculating To for basic temperature range (0-80°C) - 11.2.2.9
  // From the datasheet: The IR signal received by the sensor has two components:
  // 1. IR signal emitted by the object
  // 2. IR signal reflected from the object (the source of this signal is surrounding environment of the sensor)
  // In order to compensate correctly for the emissivity and achieve best accuracy we need to know the surrounding
  // temperature which is responsible for the second component of the IR signal namely the reflected part - 𝑇𝑟.  In case
  // this 𝑇𝑟 temperature is not available and cannot be provided it might be replaced by 𝑇𝑟≈𝑇𝑎−5.
  float Ta_K4 = powf((Ta + 273.15), 4.0);               // powf() returns the a^b where a, b are both float numbers
  float Tr_K4 = powf((Ta + 268.15), 4.0);               // assume Tr = Ta - 5.0 (surrounding air)
  float Ta_r = Tr_K4 - ((Tr_K4 - Ta_K4) / Emissivity);  // this is T_a-r in the datasheet
  float S_x[NUM_PIXELS] = { 0.0 };                      // define matrix to hold Sx values
  // Edit the following formulas accordingly based on the temperature range the sensor will be measuring: 11.2.2.9.1
  //If 𝑇𝑂(𝑖,𝑗) < -20°C we are in range 1 and we will use the parameters (𝐾𝑠𝑇𝑜1, 𝐴𝑙𝑝ℎ𝑎𝑐𝑜𝑟𝑟𝑟𝑎𝑛𝑔𝑒1 and 𝐶𝑇1 = −40°𝐶)
  //If -20°C < 𝑇𝑂(𝑖,𝑗) < -40°C we are in range 2 and we will use the parameters (𝐾𝑠𝑇𝑜2, 𝐴𝑙𝑝ℎ𝑎𝑐𝑜𝑟𝑟𝑟𝑎𝑛𝑔𝑒2 and 𝐶𝑇2 = −20°𝐶)
  // If 0°C < 𝑇𝑂(𝑖,𝑗) < 80°C we are in range 3 and we will use the parameters (𝐾𝑠𝑇𝑜3, 𝐴𝑙𝑝ℎ𝑎𝑐𝑜𝑟𝑟𝑟𝑎𝑛𝑔𝑒3 and 𝐶𝑇3 = 0°𝐶)
  // If 80°C < 𝑇𝑂(𝑖,𝑗) < 120°C we are in range 4 and we will use the parameters (𝐾𝑠𝑇𝑜4, 𝐴𝑙𝑝ℎ𝑎𝑐𝑜𝑟𝑟𝑟𝑎𝑛𝑔𝑒4 and 𝐶𝑇4 = 80°𝐶)
  // If 120°C < 𝑇𝑂(𝑖,𝑗) < CT6°C we are in range 5 and we will use the parameters (𝐾𝑠𝑇𝑜5, 𝐴𝑙𝑝ℎ𝑎𝑐𝑜𝑟𝑟𝑟𝑎𝑛𝑔𝑒5 and 𝐶𝑇5 = 120°𝐶)
  // If CT6°C < 𝑇𝑂(𝑖,𝑗) < CT7°C we are in range 6 and we will use the parameters (𝐾𝑠𝑇𝑜6, 𝐴𝑙𝑝ℎ𝑎𝑐𝑜𝑟𝑟𝑟𝑎𝑛𝑔𝑒6 and 𝐶𝑇6 = 200°𝐶)
  // If CT7°C < 𝑇𝑂(𝑖,𝑗) < CT8°C we are in range 7 and we will use the parameters (𝐾𝑠𝑇𝑜7, 𝐴𝑙𝑝ℎ𝑎𝑐𝑜𝑟𝑟𝑟𝑎𝑛𝑔𝑒7 and 𝐶𝑇7 = 400°𝐶)
  // If CT8°C < 𝑇𝑂(𝑖,𝑗)  we are in range 8 and we will use the parameters (𝐾𝑠𝑇𝑜8, 𝐴𝑙𝑝ℎ𝑎𝑐𝑜𝑟𝑟𝑟𝑎𝑛𝑔𝑒8 and 𝐶𝑇8 = 600°𝐶)

  for (int i = 0; i < NUM_PIXELS; i++) {
    if (alpha_comp[i] < 1.0e-6) alpha_comp[i] = 1.0e-6;                                                                 // protects against small alpha_comp[] values
    S_x[i] = KsTo3 * MLX90641::fourth_root(powf(alpha_comp[i], 3.0) * V_IR_compensated[i] + powf(alpha_comp[i], 4.0) * Ta_r);     // formula for S_x[i]
    T_o[i] = MLX90641::fourth_root((V_IR_compensated[i] / (alpha_comp[i] * (1.0 - (KsTo3 * 273.15)) + S_x[i])) + Ta_r) - 273.15;  // formula for T_o[i]
    // Apply post-hoc calibration equation - calibrate to desired surface (comment out if not needed)
    //T_o[i] = T_o[i] + OFFSET;  // Only use OFFSET term for temperature adjustment
    T_o[i] = T_o[i] * CAL_SLOPE + CAL_INT + OFFSET;  // adjust T_o based on calibration + OFFSET
    float inner = (V_IR_compensated[i] / (alpha_comp[i] * (1.0 - (KsTo3 * 273.15)) + S_x[i])) + Ta_r;
#ifdef DEBUG
    if (inner < 0 || isnan(inner)) {
      Serial.print("BAD INNER @ " + (String)i + ", " + (String)inner);
      Serial.print("Pixel ");
      Serial.print(i);
      Serial.print(" S_x[i] =");
      Serial.print(S_x[i], 8);
      Serial.print(" alpha_comp = ");
      Serial.print(alpha_comp[i], 8);
      Serial.print(" V_IR_comp = ");
      Serial.println(V_IR_compensated[i], 8);
    }
#endif
  }

#ifdef DEBUG
  Serial.print("Subpage: ");
  Serial.println(subpage);
  Serial.print("readTempC() Ta_K4: ");
  Serial.print("Ta_K4/1e9 = ");
  Serial.print(Ta_K4 / 1e9, 6);
  Serial.println(", example value: 9866871831.80621 ");  // 11.2.2.8
  Serial.print("readTempC() Tr_K4: ");
  Serial.print("Tr_K4/1e9 = ");
  Serial.print(Tr_K4 / 1e9, 6);
  Serial.println(", example value: 9253097577.685506 ");  // 11.2.2.8
  Serial.print("readTempC() Ta_r/1e9 = ");
  Serial.print(Ta_r / 1e9, 6);
  Serial.println(", example value: 9899175739.92 ");  // 11.2.2.8
  Serial.print("readTempC() S_x[95] * 1e8 = ");
  Serial.print(S_x[95] * 1e8, 6);
  Serial.println(", example value: -8.18463664533495E-08");  // 11.2.2.8
  Serial.print("readTempC() T_o[95] = ");
  Serial.print(T_o[95], 1);
  Serial.println(", example value: 80.12");  // 11.2.2.8

  Serial.println("Finished: basic temperature range.");
#endif
}

// To print a number to the Serial Monitor in exponential format (for debugging)
String MLX90641::float2exp(float num, byte sigDigits) {
  if (num == 0) return "0.00e+0";
  if (isnan(num)) return "NaN";
  int exponent = floor(log10(abs(num)));   //find order
  float scaled = num / pow(10, exponent);  //scale #
  //Round scaled to (sigDigits-1) decimal places:
  float rounded = round(scaled * pow(10, sigDigits - 1)) / pow(10, sigDigits - 1);
  //Handle rounding edge case
  //where 9.999... rounds to 10.0
  if (rounded >= 10.0) {
    rounded /= 10.0;
    exponent++;
  }
  //Build string
  //controls how many decimal places show:
  String expStr = String(rounded, sigDigits - 1);
  expStr += "e" + String((exponent >= 0 ? "+" : ""));
  expStr += String(exponent);
  return expStr;
}

// safer way to 2^ (won't overflow for big numbers)
float MLX90641::two_to_the(uint32_t n) {
  return (float)pow(2.0, (double)n);
}

// fourth root done with two square roots
float MLX90641::fourth_root(float n) {
  return sqrtf(sqrtf(n));  // this is x^(1/4)
}

// to retrieve pixel address, subpage 0
uint16_t MLX90641::pix_addr_S0(uint16_t pxl) {
  uint16_t x = FRAME_ADDR;          // base memory for pix, sp0
  if (pxl >= NUM_PIXELS) return 0;  // or assert / error print
  return x + pxl + 32 * (pxl / 32);
}

// to retrieve pixel address, subpage 1
uint16_t MLX90641::pix_addr_S1(uint16_t pxl) {
  uint16_t x = 0x0420;              // base memory for pix, sp0
  if (pxl >= NUM_PIXELS) return 0;  // or assert / error print
  return x + pxl + 32 * (pxl / 32);
}

// To set the refresh rate - 10.4, 12.2.1, and Figure 11
bool MLX90641::setRefreshRate(uint8_t rate) {
  // MLX90641 refresh rates (Control register 0x800D bits 10:7):
  // Bit    Freq      Sec/frame          POR Delay (ms)  Sample Every (ms)
  // 0x00 = 0.5 Hz    2 sec              4080 ms         2400 ms
  // 0x01 = 1 Hz      1 sec/frame        2080 ms         1200 ms
  // 0x02 = 2 Hz      0.5 sec/frame      1080 ms         600 ms (default)
  // 0x03 = 4 Hz      0.25 sec/frame     580 ms          300 ms
  // 0x04 = 8 Hz      0.125 sec/frame    330 ms          150 ms
  // 0x05 = 16 Hz     0.0625 sec/frame   205 ms           75 ms
  // 0x06 = 32 Hz     0.03125 sec/frame  143 ms           38 ms
  // 0x07 = 64 Hz     0.015625 sec/frame 112 ms           19 ms
  if (rate > 0x07) return false;  // Invalid rate

  // Read current config (0x800D)
  uint16_t config = readAddr_unsigned(0x800D);  //read control word
  if (config == 0xFFFF) return false;           // Read error

  // Clear RR bits (10:7), set new rate
  config &= ~(0x07 << 7);  // Mask bits 10:7 → 0000 0111 0000 0000
  config |= (rate << 7);   // Set new RR value

  // Write back to 0x800D
  Wire.beginTransmission(MLX90641_ADDR);
  Wire.write(0x80);
  Wire.write(0x0D);           // Config register address
  Wire.write(config >> 8);    // High byte
  Wire.write(config & 0xFF);  // Low byte
#ifdef DEBUG
  Serial.println("setRefreshRate() refreshrate");
  Serial.print("refresh rate set to 0x0");
  Serial.println(rate, HEX);
#endif
  return (Wire.endTransmission() == 0);
}

