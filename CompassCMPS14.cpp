// Digital Compass CMPS14
// Copyright (C) 2021 https://www.roboticboat.uk
// 3566d302-09ed-4ff3-8cc4-9aeffbf323d0
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.


#include "CompassCMPS14.h"

#include <Wire.h>

// Register Function
// 0        Command register (write) / Software version (read)

// 1        Compass Bearing as a byte, i.e. 0-255 for a full circle
// 2,3      Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees. Register 2 being the high byte

// 4        Pitch angle - signed byte giving angle in degrees from the horizontal plane, Kalman filtered with Gyro
// 5        Roll angle - signed byte giving angle in degrees from the horizontal plane, Kalman filtered with Gyro

// 6,7      Magnetometer X axis raw output, 16 bit signed integer with register 6 being the upper 8 bits
// 8,9      Magnetometer Y axis raw output, 16 bit signed integer with register 8 being the upper 8 bits
// 10,11    Magnetometer Z axis raw output, 16 bit signed integer with register 10 being the upper 8 bits

// 12,13    Accelerometer  X axis raw output, 16 bit signed integer with register 12 being the upper 8 bits
// 14,15    Accelerometer  Y axis raw output, 16 bit signed integer with register 14 being the upper 8 bits
// 16,17    Accelerometer  Z axis raw output, 16 bit signed integer with register 16 being the upper 8 bits

// 18,19    Gyro X axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
// 20,21    Gyro Y axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
// 22,23    Gyro Z axis raw output, 16 bit signed integer with register 22 being the upper 8 bits


int16_t CompassCMPS14::getBearing()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(BEARING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate full bearing
  bearing = ((_byteHigh<<8) + _byteLow) / 10;
  
  return bearing;
}

byte CompassCMPS14::getPitch()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(PITCH_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}

  // Request 1 byte from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;
  
  // Read the values
  pitch = Wire.read();

  return pitch;
}

byte CompassCMPS14::getRoll()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ROLL_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 1 byte from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;

  // Read the values
  roll = Wire.read();

  return roll ;
}

int16_t CompassCMPS14::getGyroX()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoX
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS14::getGyroY()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoY
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS14::getGyroZ()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoZ
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS14::getAcceleroX()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS14::getAcceleroY()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS14::getAcceleroZ()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);

}

int16_t CompassCMPS14::getMagnetX()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS14::getMagnetY()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS14::getMagnetZ()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

void CompassCMPS14::ReadCompass()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(BEARING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){bearing = 0; pitch = 0;  roll = 0; return;}
  
  // Request 4 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, FOUR_BYTES);

  // Something has gone wrong
  if (nReceived != FOUR_BYTES) {bearing = 0; pitch = 0;  roll = 0; return;}
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  bearing = ((_byteHigh<<8) + _byteLow) / 10;

  // Read the values
  pitch = Wire.read();

  // Read the values
  roll = Wire.read();

}


void CompassCMPS14::ReadAccelerator()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){accelX = 0; accelY = 0; accelZ = 0; return;}
  
  // Request 6 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, SIX_BYTES);

  // Timed out so return
  if (nReceived != SIX_BYTES) {accelX = 0; accelY = 0; accelZ = 0; return;}
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  accelX = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * accelScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  accelY = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * accelScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  accelZ = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * accelScale;

}

void CompassCMPS14::ReadGyro()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){gyroX = 0; gyroY = 0; gyroZ = 0; return;}
  
  // Request 6 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, SIX_BYTES);

  // Timed out so return
  if (nReceived != SIX_BYTES) {gyroX = 0; gyroY = 0; gyroZ = 0; return;}
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  gyroX = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * gyroScale;

    // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  gyroY = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * gyroScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  gyroZ = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * gyroScale;

}

void CompassCMPS14::ReadMagnet()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){magnetX = 0; magnetY = 0; magnetZ = 0; return;}
  
  // Request 6 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, SIX_BYTES);

  // Timed out so return
  if (nReceived != SIX_BYTES) {magnetX = 0; magnetY = 0; magnetZ = 0; return;}
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  magnetX = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * magnetScale;

    // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  magnetY = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * magnetScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  magnetZ = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * magnetScale;

}


void CompassCMPS14::changeAddress(byte i2cAddress, byte newi2cAddress)
{
  // Reset the address on the i2c network
  // Ensure that you have only this module connected on the i2c network
  // The 7 bit i2c address must end with a 0. (even numbers please)
  // For example changeAddress(0x60, 0x64)

  // Address 0x60, 1 long flash, 0 short flashes
  // Address 0x62, 1 long flash, 1 short flashes
  // Address 0x64, 1 long flash, 2 short flashes
  // Address 0x66, 1 long flash, 3 short flashes
  // Address 0x68, 1 long flash, 4 short flashes
  // Address 0x6A, 1 long flash, 5 short flashes
  // Address 0x6C, 1 long flash, 6 short flashes
  // Address 0x6E, 1 long flash, 7 short flashes

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xA0));
  
  // End the transmission
  int nackCatcher = Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xAA));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xA5));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(newi2cAddress);

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return;}

}

byte CompassCMPS14::getCalibration()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(Calibration_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 1 byte from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;

  // Read the values
  calibration = Wire.read();

  return calibration;
}

void CompassCMPS14::setCalibration(byte a)
{

  // Do we start the calibration?
  if (a == 'g' || a == 'a' || a == 'm' || a == 'x'){

    // Begin communication with CMPS14
    Wire.beginTransmission(_i2cAddress);

    // Want the Command Register
    Wire.write(byte(0x00));

    // Send some data
    Wire.write(byte(0x98));

    // End the transmission
    int nackCatcher = Wire.endTransmission();

    // Return if we have a connection problem 
    if(nackCatcher != 0){return;}

    delay(20);

    // Begin communication with CMPS14
    Wire.beginTransmission(_i2cAddress);

    // Want the Command Register
    Wire.write(byte(0x00));

    // Send some data
    Wire.write(byte(0x95));

    // End the transmission
    nackCatcher = Wire.endTransmission();

    // Return if we have a connection problem 
    if(nackCatcher != 0){return;}

    delay(20);

    // Begin communication with CMPS14
    Wire.beginTransmission(_i2cAddress);

    // Want the Command Register
    Wire.write(byte(0x00));

    // Send some data
    Wire.write(byte(0x99));

    // End the transmission
    nackCatcher = Wire.endTransmission();

    // Return if we have a connection problem 
    if(nackCatcher != 0){return;}

    delay(20);

    // Begin communication with CMPS14
    Wire.beginTransmission(_i2cAddress);

    // Want the Command Register
    Wire.write(byte(0x00));

    // Send some data
    switch (a){

      case 'g':
          Wire.write(byte(B10000100));
          break;

      case 'a':
          Wire.write(byte(B10000010));
          break;

      case 'm':
          Wire.write(byte(B10000001));
          break;

      case 'x':
          Wire.write(byte(B10000000));
          break;
    }

    // End the transmission
    nackCatcher = Wire.endTransmission();

  }
}