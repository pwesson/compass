// Receive digital compass readings from CMPS10
// Copyright (C) 2021 https://www.roboticboat.uk
// d863b1ba-65cb-48d6-8fe2-d1fa2f3bf4c3
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
 
#include <Wire.h>

// Register Function
// 0        Software version
// 1        Compass Bearing as a byte, i.e. 0-255 for a full circle
// 2,3      Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees.
// 4        Pitch angle - signed byte giving angle in degrees from the horizontal plane
// 5        Roll angle - signed byte giving angle in degrees from the horizontal plane
// 6        Unused
// 7        Unused
// 8        Unused
// 9        Unused
//
// 10,11    Magnetometer X axis raw output, 16 bit signed integer with register 10 being the upper 8 bits
// 12,13    Magnetometer Y axis raw output, 16 bit signed integer with register 12 being the upper 8 bits
// 14,15    Magnetometer Z axis raw output, 16 bit signed integer with register 14 being the upper 8 bits
//
// 16,17    Accelerometer  X axis raw output, 16 bit signed integer with register 16 being the upper 8 bits
// 18,19    Accelerometer  Y axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
// 20,21    Accelerometer  Z axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
//
// 22       Command register

// Address of CMPS10 on I2C
#define i2cAddress 0x60
#define BEARING_Register 2 
#define FOUR_BYTES 4

//Allocate Memory
byte byteHigh, byteLow, fine;
int bearing;
int nReceived;
char pitch, roll;
  
void setup()
{
  // Initialize the serial port to the User
  // Set this up early in the code, so the User sees all messages
  Serial.begin(9600);

  // Startup the i2c network
  Wire.begin();
}
 
void loop()
{
  
  // Begin communication with CMPS10
  Wire.beginTransmission(i2cAddress); 
  Wire.write(BEARING_Register); // Start read
  Wire.endTransmission();

  // Request 4 bytes from CMPS10
  nReceived = Wire.requestFrom(i2cAddress, FOUR_BYTES);

  if (nReceived == FOUR_BYTES){

    // Read the values
    byteHigh = Wire.read(); 
    byteLow = Wire.read();
    pitch = Wire.read();
    roll = Wire.read();
 
    // Calculate full bearing
    bearing = ((byteHigh<<8) + byteLow) / 10;

  }
    
  // Print data to Serial Monitor window
  Serial.print("$CMP,");
  Serial.print(bearing, DEC);
  Serial.print(",");
  Serial.print(pitch, DEC); 
  Serial.print(",");
  Serial.println(roll, DEC);

  // Delay
  delay(100);
}
