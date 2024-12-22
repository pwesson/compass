#ifndef BNO080_h
  #define BNO080_h

  #include "Arduino.h"

  // BNO080 registers
  #define _i2cAddress 0x4B

  #define I2C_BUFFER_SIZE 32
  #define PACKET_BUFFER_SIZE 1000

  struct Quaternion {
    float real, i, j, k;
    byte status;
    byte sequence_number;
    int16_t accuracy;
  };

  struct Axis {
    float X, Y, Z;
    byte status;
    byte sequence_number;
  };

  struct byteReading {
    byte value;
    byte status;
    byte sequence_number;
  };

  struct floatReading {
    float value;
    byte status;
    byte sequence_number;
  };

  struct calibration_status {
    byte channel_sequencialNumber;
    byte command_sequencialNumber;
    byte response_sequentialNumber;
    byte status;
    byte accel_calibration_enabled;
    byte gyro_calibration_enabled;
    byte magnet_calibration_enabled;
  };

  struct Euler {
    float roll, pitch, yaw;
    float roll2, pitch2, yaw2;
  };

  class BNO080{
    public:
		
	void SetupBMO080();
	void Restart();
	void calibrate(byte);
	void calibrateAGM(bool , bool , bool );
	void saveCalibration();
	void SetPacketInterval(int reportID, uint16_t );
	float Convert2float(int16_t , float );
	int receivePacket();	
	int processReport(int , uint16_t );
	int readRegister(int , int* );
	int readRegisters(int , int , int* );
	int writeRegister(int , int );
	int writeBytes(int , int* );
	void Quanternion2Euler(Quaternion );
	void printStatus(byte );

	byte input = 0;
	byte tap_status = 0;
	byte tap_detector = 0;

	int nZeroReadings = 0;
	int nReceived = 0;
	int packetBuffer[PACKET_BUFFER_SIZE];

	int channel1_sequencialNumber = 0;
	int channel2_sequencialNumber = 0;
	int command_sequencialNumber = 0;

	uint16_t packetBufferPtr;

	Axis accel;
	Axis gyro;
	Axis magnet;
	Axis linearAccel;
	Axis accel_raw;
	Axis gyro_raw;
	Axis magnet_raw;
	Axis gravity;
	Axis gyro_uncalibrated;
	Axis gyro_uncalibrated_bias;
	Axis magnet_uncalibrated_soft_iron;
	Axis magnet_uncalibrated_hard_iron;

	calibration_status calibration;

	byteReading stability_classification;	

	floatReading step_counter;
	floatReading significant_motion;

	Quaternion quantRotationVec;
	Quaternion quantGameRotationVec;
	Quaternion quantGeoRotationVec;

	Euler euler;
  		
  private:
	byte _byteHigh;
	byte _byteLow;
  	byte _fine;

  };

#endif