// MLX90641_NeoPixel.ino file for the MLX90641.h library, version 1.0.4
// Author: D. Dubins
// Date: 05-Jan-26
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
// ESP32 - NeoPixel:
// -----------------
// Vin - +5V
// D13 - Data
// GND - GND
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
#include <FastLED.h>                        // FastLED by Daniel Garcia, v3.10.3

//#define DEBUG                             // Show calculated and example values for calibration constants
#define OFFSET 0.0                          // Post-hoc cheap temperature adjustment (shift)
#define I2C_SPEED 100000                    // Set I2C clock speed (safe speed is 100 kHz, up to 400 kHz possible)
#define REFRESH_RATE 0x03                   // 0x00 (0.5 Hz) to 0x07 (64 Hz). Default: 0x03 (4 Hz)
#define SAMPLE_DELAY 300                    // delay between reading samples (see refresh rate table).
#define POR_DELAY SAMPLE_DELAY * 2.0 * 1.2  // delay required after power on reset (see refresh rate table)
#define CAL_INT -45.4209807273067           // Intercept of T_meas vs. T_o calibration curve (post-hoc calibration). My value: -45.4209807273067
#define CAL_SLOPE 2.64896693658985          // Slope of T_meas vs. T_o calibration curve (post-hoc calibration). My value: 2.64896693658985

MLX90641 myIRcam;  // declare an instance of class MLX90641

// NeoPixel Code
#define LED_PIN 13     // GPIO13 (D13) for LED pin to NeoPixel
#define NUM_LEDS 256   // number of LEDs in the NeoPixel (16x16). Update this to match size of NeoPixel.
#define BRIGHTNESS 10  // 0-255 (brightest)
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
TBlendType currentBlending;

bool x_invert = false;  // true: invert x on LED
bool y_invert = false;  // true: invert y on LED

// Limits for colour visualization
#define TCOLD 23.0  // cold temperature threshold
#define TMID 24.0   // intermediate temperature threshold
#define THOT 25.0   // hot temperature threshold

byte LEDpixelMap[16][16] = {  // 16x16 = 256 pixels on the LCD screen (snake trail)
  { 0, 31, 32, 63, 64, 95, 96, 127, 128, 159, 160, 191, 192, 223, 224, 255 },
  { 1, 30, 33, 62, 65, 94, 97, 126, 129, 158, 161, 190, 193, 222, 225, 254 },
  { 2, 29, 34, 61, 66, 93, 98, 125, 130, 157, 162, 189, 194, 221, 226, 253 },
  { 3, 28, 35, 60, 67, 92, 99, 124, 131, 156, 163, 188, 195, 220, 227, 252 },
  { 4, 27, 36, 59, 68, 91, 100, 123, 132, 155, 164, 187, 196, 219, 228, 251 },
  { 5, 26, 37, 58, 69, 90, 101, 122, 133, 154, 165, 186, 197, 218, 229, 250 },
  { 6, 25, 38, 57, 70, 89, 102, 121, 134, 153, 166, 185, 198, 217, 230, 249 },
  { 7, 24, 39, 56, 71, 88, 103, 120, 135, 152, 167, 184, 199, 216, 231, 248 },
  { 8, 23, 40, 55, 72, 87, 104, 119, 136, 151, 168, 183, 200, 215, 232, 247 },
  { 9, 22, 41, 54, 73, 86, 105, 118, 137, 150, 169, 182, 201, 214, 233, 246 },
  { 10, 21, 42, 53, 74, 85, 106, 117, 138, 149, 170, 181, 202, 213, 234, 245 },
  { 11, 20, 43, 52, 75, 84, 107, 116, 139, 148, 171, 180, 203, 212, 235, 244 },
  { 12, 19, 44, 51, 76, 83, 108, 115, 140, 147, 172, 179, 204, 211, 236, 243 },
  { 13, 18, 45, 50, 77, 82, 109, 114, 141, 146, 173, 178, 205, 210, 237, 242 },
  { 14, 17, 46, 49, 78, 81, 110, 113, 142, 145, 174, 177, 206, 209, 238, 241 },
  { 15, 16, 47, 48, 79, 80, 111, 112, 143, 144, 175, 176, 207, 208, 239, 240 }
};

void setup() {
  Serial.begin(115200);  // Start the Serial Monitor at 115200 bps
  delay(3000);           // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);  // 0-255 (255: brightest)
  currentBlending = NOBLEND;          // Options are LINEARBLEND (fading) or NOBLEND (discrete fast changes)

  // Set up MLX90641:
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
  myIRcam.Emissivity = myIRcam.readEmissivity();  // read Emissivity coefficient
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
  LEDMatrixClear();  // fill LEDpixelMap[][] with black and clear the screen
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

    // calculate pixel colour map, LEDpixelMap[][]
    int xadj, yadj;
    for (int x = 0; x < 12; x++) {              // x is pixel row (0 ... 11)
      for (int y = 0; y < 16; y++) {            // y is pixel col (0 ... 15)
        (x_invert) ? xadj = 11 - x : xadj = x;  // check for inversion flags
        (y_invert) ? yadj = 15 - y : yadj = y;
        if (myIRcam.T_o[xadj * 16 + yadj] < TCOLD) {        // if pixel < TCOLD
          leds[LEDpixelMap[xadj + 2][yadj]] = CRGB::Blue;   // the +2 here is to centre the image on the 16x16 screen
        } else if (myIRcam.T_o[xadj * 16 + yadj] < TMID) {  // if pixel < THOT
          leds[LEDpixelMap[xadj + 2][yadj]] = CRGB::Cyan;
        } else if (myIRcam.T_o[xadj * 16 + yadj] < THOT) {  // if pixel < THOT
          leds[LEDpixelMap[xadj + 2][yadj]] = CRGB::Yellow;
        } else {
          leds[LEDpixelMap[xadj + 2][yadj]] = CRGB::Red;
        }
      }
    }
    FastLED.show();  // show pixel map
  } else {
    Serial.println("Timeout: No new data");
    return;  // Skip this frame
  }
  delay(SAMPLE_DELAY);  // wait for new reading (adjust to desired sample frequency, see refresh rate table in setRefreshRate() for ranges)

#ifdef DEBUG  // when debugging, it helps to only see the first reading, so you can scroll through the constants.
  while(1);
#endif
}

// helper function to convert pixel row, col to 1D index array
int pixelAddr(int row, int col) {
  return (row * 16) + col;  // convert row, col to 1D array index in 1D array of pixels (e.g. badPixels[])
}

void LEDpixelMap_Black() {
  //clears the 8x8 LCD screen (shifts out zeros)
  for (int r = 0; r < 16; r++) {    // by row  (clear out entire 16x16 screen)
    for (int c = 0; c < 16; c++) {  // by column (clear out entire 16x16 screen)
      leds[LEDpixelMap[r][c]] = CRGB::Black;
    }
  }
}

void LEDMatrixClear() {
  LEDpixelMap_Black();
  FastLED.show();
}
