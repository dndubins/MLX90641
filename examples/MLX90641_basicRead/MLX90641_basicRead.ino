// MLX90641_basicRead.ino file for the MLX90641.h library, version 1.0.4
// Author: D. Dubins
// Lots of help from: ChatGPT 3.0, Perplexity.AI
// Date: 17-Dec-25
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
//
// MLX90641 refresh rates (Control register 0x800D bits 10:7):
// -----------------------------------------------------------
// Bit    Freq      Sec/frame          POR Delay (ms)  Sample Every (ms)
// 0x00 = 0.5 Hz    2 sec              4080 ms         2400 ms
// 0x01 = 1 Hz      1 sec/frame        2080 ms         1200 ms
// 0x02 = 2 Hz      0.5 sec/frame      1080 ms         600 ms (default)
// 0x03 = 4 Hz      0.25 sec/frame     580 ms          300 ms
// 0x04 = 8 Hz      0.125 sec/frame    330 ms          150 ms
// 0x05 = 16 Hz     0.0625 sec/frame   205 ms           75 ms
// 0x06 = 32 Hz     0.03125 sec/frame  143 ms           38 ms
// 0x07 = 64 Hz     0.015625 sec/frame 112 ms           19 ms

#include <Wire.h>
#include "MLX90641.h"

//#define HEATMAP							// Uncomment for simple ASCII heatmap output to Serial Monitor instead of temperatures
//#define DEBUG                             // Show calculated and example values for calibration constants
#define OFFSET 0.0                          // Post-hoc cheap temperature adjustment (shift)
#define I2C_SPEED 100000                    // Set I2C clock speed (safe speed is 100 kHz, up to 400 kHz possible)
#define REFRESH_RATE 0x03                   // 0x00 (0.5 Hz) to 0x07 (64 Hz). Default: 0x03 (4 Hz)
#define SAMPLE_DELAY 300                    // delay between reading samples (see refresh rate table)
#define POR_DELAY SAMPLE_DELAY * 2.0 * 1.2  // delay required after power on reset (see refresh rate table)
#define CAL_INT -45.4209807273067           // Intercept of T_meas vs. T_o calibration curve (post-hoc calibration). My value: -45.4209807273067
#define CAL_SLOPE 2.64896693658985          // Slope of T_meas vs. T_o calibration curve (post-hoc calibration). My value: 2.64896693658985

MLX90641 myIRcam;  // declare an instance of class MLX90641

void setup() {
  Serial.begin(115200);                        // Start the Serial Monitor at 115200 bps
  Wire.begin(21, 22);                          // SDA, SCL for the ESP32 (SDA: GPIO 21, SDL: GPIO22). Change these to your I2C pins if using a different bus.
  Wire.setClock(I2C_SPEED);                    // set I2C clock speed (slower=more stable)
  if (myIRcam.setRefreshRate(REFRESH_RATE)) {  // set the page refresh rate (sampling frequency)
    Serial.println("Refresh rate adjusted.");
  } else {
    Serial.println("Error on adjusting refresh rate.");
  }
  delay(POR_DELAY);  // Power on reset delay (POR), see table above
  Serial.println("MLX90641 ESP32 Calibrated Read");
  // Read full EEPROM (0x2400..0x272F)
  if (!myIRcam.readEEPROMBlock(0x2400, EEPROM_WORDS, myIRcam.eeData)) {
    Serial.println("EEPROM read failed!");
    while (1) delay(1000);
  }

  // Mark bad pixels separately here (row indexes 0...11, col indexes 0..15)
  //myIRcam.badPixels[pixelAddr(9,14)]=true;    // mark pixel bad at row 9, column 14
  //myIRcam.badPixels[pixelAddr(11,0)]=true;    // mark pixel bad at row 11, column 0

  // Check EEPROM data:
#ifdef DEBUG
  Serial.println("setup() First 16 words of EEPROM:");
  for (int i = 0; i < 16; i++) {
    Serial.println("EEPROM value at address: 0x" + String(0x2400 + i, HEX) + ", value: 0x" + String(eeData[i], HEX));
  }
  Serial.println("setup() Suspicious EEPROM value check:");
  for (int i = 0; i < EEPROM_WORDS; i++) {
    if (myIRcam.eeData[i] == 0x0000 || myIRcam.eeData[i] == 0xFFFF) {
      Serial.println("EEPROM value suspicious at address: 0x" + String(0x2400 + i, HEX) + ", value: 0x" + String(eeData[i], HEX));
    }
  }
#endif
  myIRcam.Vdd = myIRcam.readVdd();  // This should be close to 3.3V. Can read once in setup.
  myIRcam.Ta = myIRcam.readTa();    // should happen inside the loop
  Serial.print("Ambient temperature on start: ");
  Serial.println(myIRcam.Ta, 1);      // This should be close to ambient temperature (21°C?)
  myIRcam.readPixelOffset();          // only needs to be read once
  myIRcam.readAlpha();                // read sensitivities (fills alpha_pixel[])
  myIRcam.readKta();                  // read Kta coefficients (fills Kta[])
  myIRcam.readKv();                   // read Kv coefficients (fills Kv[])
  myIRcam.KsTa = myIRcam.readKsTa();  // read KsTa coefficient
  Serial.println("Finished: read KsTA.");
  myIRcam.readCT();                               // read 8 corner temperatures
  myIRcam.readKsTo();                             // read 8 KsTo coefficients
  myIRcam.readAlphaCorrRange();                   // read sensitivity correction coefficients
  myIRcam.Emissivity = myIRcam.readEmissivity();  // read Emissivity coefficient (comment out for hard-coding)
  //myIRcam.Emissivity = 0.95;                    // over-write Emissivity with hard-coded value here (e.g. 0.95)
  myIRcam.alpha_CP = myIRcam.readAlpha_CP();      // read Sensitivity alpha_CP coefficient
  myIRcam.pix_OS_ref_CP = myIRcam.readOff_CP();   // read offset CP (also called pix_OS_ref_CP)
  myIRcam.Kv_CP = myIRcam.readKv_CP();            // read Kv CP
  myIRcam.KTa_CP = myIRcam.readKTa_CP();          // read KTa_CP
  myIRcam.TGC = myIRcam.readTGC();                // read TGC - do this last (leaves setup function for some odd reason)
  /*#ifdef DEBUG                  // uncomment this if you need a pixel map (or consult the datasheet)
  Serial.println("Printing pixel address memory map: ");
  for (int i = 0; i < 192; i++) {
    Serial.print(i);
    Serial.print(", 0x0");
    Serial.print(myIRcam.pix_addr_S0(i), HEX);
    Serial.print(", 0x0");
    Serial.println(myIRcam.pix_addr_S1(i), HEX);
    delay(100);
  }
  #endif*/
}

void loop() {
  unsigned long pollStart = millis();
  while (millis() - pollStart < 500) {  // 500ms max wait (adjust to > 1000 * 1.2 * (1/refresh_rate in Hz))
    if (myIRcam.isNewDataAvailable()) break;
    delay(10);  // Yield to prevent watchdog/I2C lockup
  }
  if (myIRcam.isNewDataAvailable()) {
    myIRcam.clearNewDataBit();

    myIRcam.readTempC();  // read the temperature

    Serial.println("\n16x12 Thermal Frame (Celsius calibrated):");
    for (int r = 0; r < 12; r++) {    // rows
      for (int c = 0; c < 16; c++) {  // columns
        // Two options here: final temperatures, or ASCII heat map:
        float lim1 = 23.0;  // low limit for heat map
        float lim2 = 25.0;  // middle limit for heat map
        float lim3 = 27.0;  // high limit for heat map
#ifdef HEATMAP
        if (myIRcam.T_o[r * 16 + c] > lim3) Serial.print("H");  // hot
        if (myIRcam.T_o[r * 16 + c] > lim2 && myIRcam.T_o[r * 16 + c] <= lim3) Serial.print("*");
        if (myIRcam.T_o[r * 16 + c] > lim1 && myIRcam.T_o[r * 16 + c] <= lim2) Serial.print(".");
        if (myIRcam.T_o[r * 16 + c] <= lim1) Serial.print("C");  // cold
#else
        Serial.print(myIRcam.T_o[r * 16 + c], 1);  // putting the data in a 16x12 grid
#endif
        Serial.print(" ");  // print divider for data (needed for both display modes)
      }
      Serial.println();
    }
    Serial.print("Ambient Temp (Ta): ");
    Serial.println(myIRcam.Ta, 1);
    // Calculate average temperature across all pixels
    float avg = 0.0;
    for (int i = 0; i < NUM_PIXELS; i++) {
      avg += myIRcam.T_o[i];
    }
    avg /= (float)NUM_PIXELS;
    Serial.print("Average Value: ");
    Serial.println(avg);
  } else {
    Serial.println("Timeout: No new data");
    return;  // Skip this frame
  }
  delay(SAMPLE_DELAY);  // wait for new reading (adjust to desired sample frequency, see refresh rate table in setRefreshRate() for ranges)
#ifdef DEBUG            // when debugging, it helps to only see the first reading, so you can scroll through the constants.
  while (1)
    ;
#endif
}

// helper function to convert pixel row, col to 1D index array
int pixelAddr(int row, int col){
  return (row*16)+col; // convert row, col to 1D array index in 1D array of pixels (e.g. badPixels[])
}
