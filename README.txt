Readme file for MLX90641 16x12 IR Array

MLX90641 is a library that can control the MLX90641 16x12 IR Array (by Melexis) using an ESP32 MCU.

I wrote this sketch because the drivers written by Melexis available at https://github.com/melexis/mlx90641-library did not compile for the ESP32.

* The example sketch "MLX90641_basicRead.ino" illustrates a simple reading with basic temperature output to the Serial Monitor.
* The example sketch "MLX90641_processing.ino" formats the output for a processing sketch, to draw a heat map.
* The example sketch "extras/MLX90641_heatmap.pde" is a Processing (https://processing.org/) sketch to make a colour heat map, with a simple control panel.
* The file "extras/FAB-MLX90641-001-A.1.zip" is a Gerber and Drill file if you would like to print a PCB for the sensor.

Datasheet: Melexis. "MLX90641 16x12 IR Array Datasheet", Revision 4 - September 14, 2023. 3901090641

* According to Melexis, each sensor may have up to 1 bad pixel, with either no output or out of specification temperature reading. Pixels might also go bad throughout the working life of the sensor. Once a pixel is flagged, the library will replace its value by taking the average of neighbouring pixels. I provided an example of how to manually flag a defective pixel in the example sketches:

	myIRcam.badPixels[pixelAddr(9,14)]=true;    // mark pixel bad at row 9, column 14

* The library has an automatic way of identifying and flagging these bad pixels. I'm not sure how well this will work, because my sensor did not have any bad pixels (yay!).
* For those of you who don't like libraries, I included a library-free version in a self-contained sketch, "MLX90641.ino". I did not include bad pixel handling in this sketch.

The functions available in the library include:
	bool readEEPROMBlock(uint16_t startAddr, uint16_t numWords, uint16_t *dest); // Read the device EEPROM
	bool isNewDataAvailable(); // Check if new data is available
	bool clearNewDataBit(); // Clear the new data available bit (must be done after each read)
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
To use the library, copy the download to the Library directory.
 
Technical notes:
- This MLX90641.h library was designed only for the ESP32. Feel free to adapt it to other MCUs.

Acknowledgements: 
- A big thank-you to Howard Qiu for introducing me to this sensor, and for the discussions we had about it. This project was a lot of fun!









