#ifndef MLX90641_h
#define MLX90641_h

#include <Arduino.h>

// USER CONFIGURATION - Override these in your .ino AFTER #include "MLX90641.h"
//#define DEBUG                             // show calculated and example values for calibration constants
#define OFFSET 0.0                          // posthoc cheap temperature adjustment (shift)
#define MLX90641_ADDR 0x33                  // I2C bit address of the MLX90641
#define NUM_PIXELS 192                      // number of pixels
#define BLOCK_SIZE 8                        // block size for reading words
#define I2C_SPEED 100000                    // safe speed is 100 kHz
#define REFRESH_RATE 0x03                   // 0x00 (0.5 Hz) to 0x07 (64 Hz). Default: 0x03 (4 Hz)
#define SAMPLE_DELAY 300                    // delay between reading samples (see setRefreshRate() table)
#define POR_DELAY SAMPLE_DELAY * 2.0 * 1.2  // delay required after power on reset (see setRefreshRate() table)
#define FRAME_ADDR 0x0400                   // Starting address for pixel data in RAM
#define STATUS_ADDR 0x8000                  // Address for Status Register
#define CAL_INT -45.4209807273067           // Intercept of T_meas vs. T_o calibration curve (post-hoc calibration). My value: -45.4209807273067 
#define CAL_SLOPE 2.64896693658985          // Slope of T_meas vs. T_o calibration curve (post-hoc calibration). My value: 2.64896693658985 
#define EEPROM_WORDS 832                    // MLX90641 EEPROM size in 16-bit words

// Static Variables:
static uint16_t eeData[EEPROM_WORDS];       // to hold the EEPROM contents

class MLX90641 {
	public:
	MLX90641();
	// Non-Static Variables: (not common across all sensors)
	uint16_t eeData[EEPROM_WORDS];              // to hold the EEPROM contents
	float Vdd;                                  // to hold calculated Vdd (measured sensor operating voltage)
	int16_t Vdd_25;                      // to store Vdd at 25°C
	int16_t K_Vdd;                       // to store K_Vdd
	float Ta;                            // calculated Ta (ambient temperature)
	float Kgain;                         // Kgain coefficient
	int16_t pix_OS_ref_SP0[NUM_PIXELS];  // pixel offset reference sp0
	int16_t pix_OS_ref_SP1[NUM_PIXELS];  // pixel offset reference sp1
	float pix_OS_SP0[NUM_PIXELS];        // pixel offset sp0
	float pix_OS_SP1[NUM_PIXELS];        // pixel offset sp1
	float alpha_pixel[NUM_PIXELS];       // pixel sensitivity
	float Kta[NUM_PIXELS];               // Kta[i,j] coefficients
	float Kv[NUM_PIXELS];                // Kv[i,j] coefficients
	float KsTa;                          // KsTa coefficient
	int16_t CT1;                         // Corner temperatures
	int16_t CT2;                         // Corner temperatures
	int16_t CT3;                         // Corner temperatures
	int16_t CT4;                         // Corner temperatures
	int16_t CT5;                         // Corner temperatures
	int16_t CT6;                         // Corner temperatures
	int16_t CT7;                         // Corner temperatures
	int16_t CT8;                         // Corner temperatures
	float KsTo1;                         // KsTo coefficients
	float KsTo2;                         // KsTo coefficients
	float KsTo3;                         // KsTo coefficients
	float KsTo4;                         // KsTo coefficients
	float KsTo5;                         // KsTo coefficients
	float KsTo6;                         // KsTo coefficients
	float KsTo7;                         // KsTo coefficients
	float KsTo8;                         // KsTo coefficients
	float Alpha_cr1;                     // Alpha correction coefficients for each range
	float Alpha_cr2;                     // Alpha correction coefficients for each range
	float Alpha_cr3;                     // Alpha correction coefficients for each range
	float Alpha_cr4;                     // Alpha correction coefficients for each range
	float Alpha_cr5;                     // Alpha correction coefficients for each range
	float Alpha_cr6;                     // Alpha correction coefficients for each range
	float Alpha_cr7;                     // Alpha correction coefficients for each range
	float Alpha_cr8;                     // Alpha correction coefficients for each range
	float alpha_reference_row1;          // Alpha references for sensitivity adjustment
	float alpha_reference_row2;          // Alpha references for sensitivity adjustment
	float alpha_reference_row3;          // Alpha references for sensitivity adjustment
	float alpha_reference_row4;          // Alpha references for sensitivity adjustment
	float alpha_reference_row5;          // Alpha references for sensitivity adjustment
	float alpha_reference_row6;          // Alpha references for sensitivity adjustment
	float Emissivity;                    // Emissivity coefficient (default: 1)
	float alpha_CP;                      // Sensitivity alpha_CP coefficient
	int16_t pix_OS_ref_CP;               // Offset CP (known in datasheet as Off_CP or pix_OS_ref_CP)
	float Kv_CP;                         // Kv CP coefficient, because there should be one of those for sure.
	float KTa_CP;                        // KTa_CP coefficient
	float TGC;                           // TGC Coefficient
	float V_IR_compensated[NUM_PIXELS];  // V_IR_compensated values
	float T_o[NUM_PIXELS];               // Matrix to hold final T_o[i] values
	
	// Functions:
	bool readEEPROMBlock(uint16_t startAddr, uint16_t numWords, uint16_t *dest); // Read the device EEPROM
	bool isNewDataAvailable(); // Check if new data is available
	bool clearNewDataBit(); // Clear the new data available bit (must be done after each read)
	uint16_t readAddr_unsigned(const uint16_t readByte); // Read a 16-bit unsigned integer from RAM or EEPROM at the address readByte
	int16_t readAddr_signed(const uint16_t readByte); // Read a 16-bit signed integer from RAM or EEPROM at the address readByte
	uint16_t readEEPROM_unsigned(uint16_t addr); // Read a 16-bit unsigned integer from eeData at the address addr
	int16_t readEEPROM_signed(uint16_t addr); // Read a 16-bit signed integer from eeData at the address addr
	float readVdd(); // read Vdd (From 11.1.1, worked example in 11.2.2.2) 
	float readTa(); // Read ambient temperature, datasheet, 11.1.2
	float readKgain(); // calculate the Kgain coefficient, datasheet 11.1.7. This needs to be calculated once per frame, because it might change in RAM.
	void readPixelOffset(); // this function fills up the pixel offset variables: pix_OS_ref_SP0 and pix_OS_ref_SP1. datasheet 11.1.3
	void readAlpha(); // this function restores the sensitivity from EEPROM data (11.1.4), and fills alpha_pixel[].
	void readKta(); // To restore the Kta coefficients, 11.1.5 (fills Kta[]). In EEPROM, Kta values range from 0x25C0 to 0x267F.
	void readKv(); // To restore the Kv coefficients, 11.1.6 (fills Kv[]). In EEPROM, Kv values range from 0x25C0 to 0x267F.
	float readKsTa(); // To restore the KsTa coefficient, 11.1.8
	void readCT(); // To restore the corner temperatures (CT1..CT8), 11.1.9
	void readKsTo(); // To restore the KsTo coefficients, 11.1.10
	void readAlphaCorrRange(); // To restore the Sensitivity Correction coefficients for each temperature range, 11.1.11
	float readEmissivity(); // To restore the Emissivity coefficient, 11.1.12
	float readAlpha_CP(); // To restore Sensitivity alpha_CP, 11.1.13
	int16_t readOff_CP(); // To restore offset of the CP, 11.1.14, example 11.2.2.6.2
	float readKv_CP(); // To restore Kv_CP coefficient, 11.1.15, example 11.2.2.6.2
	float readKTa_CP(); // To restore KTa_CP coefficient, 11.1.16, example 11.2.2.6.2
	float readTGC(); // To restore TGC coefficient, 11.1.17, example 11.2.2.7
	void readTempC(); // After importing and calculating all constants, we are ready to take a temperature reading.
	String float2exp(float num, byte sigDigits); // To print a number to the Serial Monitor in exponential format (for debugging)
	float two_to_the(uint32_t n); // safer way to 2^ (won't overflow for big numbers)
	float fourth_root(float n); // fourth root done with two square roots
	uint16_t pix_addr_S0(uint16_t pxl); // to retrieve pixel address, subpage 0
	uint16_t pix_addr_S1(uint16_t pxl); // to retrieve pixel address, subpage 1
	bool setRefreshRate(uint8_t rate); // // To set the refresh rate - 10.4, 12.2.1, and Figure 11
};

#endif