#ifndef CompassCMPS14_h
  #define CompassCMPS14_h

  #include "Arduino.h"
  
  // CMPS14 compass registers
  #define _i2cAddress 0x60

  #define CONTROL_Register 0

  #define BEARING_Register 2 
  #define PITCH_Register 4 
  #define ROLL_Register 5

  #define MAGNETX_Register  6
  #define MAGNETY_Register  8
  #define MAGNETZ_Register 10

  #define ACCELEROX_Register 12
  #define ACCELEROY_Register 14
  #define ACCELEROZ_Register 16

  #define GYROX_Register 18
  #define GYROY_Register 20
  #define GYROZ_Register 22

  #define Calibration_Register 0x1E

  #define ONE_BYTE   1
  #define TWO_BYTES  2
  #define FOUR_BYTES 4
  #define SIX_BYTES  6

  #define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"

  #define BYTE_TO_BINARY(byte)  \
    (byte & 0x80 ? '1' : '0'), \
    (byte & 0x40 ? '1' : '0'), \
    (byte & 0x20 ? '1' : '0'), \
    (byte & 0x10 ? '1' : '0'), \
    (byte & 0x08 ? '1' : '0'), \
    (byte & 0x04 ? '1' : '0'), \
    (byte & 0x02 ? '1' : '0'), \
    (byte & 0x01 ? '1' : '0')

  class CompassCMPS14{
    public:
		
	void begin();
	void ReadCompass();
	void ReadAccelerator();
	void ReadGyro();
	void ReadMagnet();
	void changeAddress(byte, byte);
	void setCalibration(byte);	

	int16_t getBearing();
	byte getPitch();
	byte getRoll();
	byte getCalibration();
		
	int16_t getAcceleroX();
	int16_t getAcceleroY();
	int16_t getAcceleroZ();

	int16_t getGyroX();
        int16_t getGyroY();
        int16_t getGyroZ();

	int16_t getMagnetX();
	int16_t getMagnetY();
	int16_t getMagnetZ();
	
	byte calibration;	
	int bearing;
        int nReceived;
	signed char pitch;
	signed char roll;

	float magnetX = 0;
  	float magnetY = 0;
  	float magnetZ = 0;
	float magnetScale = 2.0f/32768.0f; // 2 Gauss

  	float accelX = 0;
  	float accelY = 0;
  	float accelZ = 0;
  	// The acceleration along the X-axis, presented in mg 
  	// See BNO080_Datasheet_v1.3 page 21
  	float accelScale = 9.80592991914f/1000.0f; // 1 m/s^2
  
  	float gyroX = 0;
  	float gyroY = 0;
  	float gyroZ = 0;
  	// 16bit signed integer 32,768
  	// Max 2000 degrees per second - page 6
  	float gyroScale = 1.0f/16.0f; // 1 Dps
  		
  private:
	byte _byteHigh;
	byte _byteLow;
  	byte _fine;

  };

#endif