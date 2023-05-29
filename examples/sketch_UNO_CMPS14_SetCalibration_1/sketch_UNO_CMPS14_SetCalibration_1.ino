// Teensy 4.1 with CMPS14 i2c (calibration)
// Copyright (C) 2021 https://www.roboticboat.uk
// ced36f2e-2de1-495f-9836-33642df9cc2a
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


// Arduino 1.8.13 IDE

#include <Wire.h>

#define _i2cAddress         0x60
#define calibrationQuality  0x1E

// https://stackoverflow.com/questions/111928 (nice trick)
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

// Timer
unsigned long mytime;

// Character array
char Message[100];

void setup() {

  // Keep the User informed
  Serial.begin(9600);

  // Set i2c network
  Wire.begin();

  Serial.println("----------------------");
  Serial.println("   Calibrate CMPS14"); 
  Serial.println("----------------------");

}

void loop() {

  // Timer
  mytime = millis();
  
  // Check the Serial port
  while(Serial.available()) 
  {
    // Read User input
    byte a = Serial.read();

    // Do we start the calibration?
    if (a == 'g' || a == 'a' || a == 'm' || a == 'x'){

      writeToCMPS14(byte(0x98));
      writeToCMPS14(byte(0x95));
      writeToCMPS14(byte(0x99));

      // Begin communication with CMPS14
      Wire.beginTransmission(_i2cAddress);

      // Want the Command Register
      Wire.write(byte(0x00));

      // Send some data
      switch (a){

        case 'g':
          Serial.println("Gyro");
          Wire.write(byte(B10000100));
          break;

        case 'a':
          Serial.println("Accel");
          Wire.write(byte(B10000010));
          break;

        case 'm':
          Serial.println("Magnet");
          Wire.write(byte(B10010001));
          break;

        case 'x':
          Serial.println("Stop auto calibration");
          Wire.write(byte(B10000000));
          break;

      }

      // End the transmission
      int nackCatcher = Wire.endTransmission();

      // Is connection ok? 
      if(nackCatcher != 0){

        Serial.println("communication error here");
      }

    }

    // Store the calibration
    if (a == 's'){

      // Update the User
      Serial.println("Save");

      writeToCMPS14(byte(0xF0));
      writeToCMPS14(byte(0xF5));
      writeToCMPS14(byte(0xF6));      
    }

    // Reset the calibration
    if (a == 'r'){

      // Update the User
      Serial.println("Reset");
      
      writeToCMPS14(byte(0xE0));
      writeToCMPS14(byte(0xE5));
      writeToCMPS14(byte(0xE2));      

      delay(500);
    }
  
  }

  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(calibrationQuality);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0) return;
  
  // Request 1 byte from CMPS14
  int nReceived = Wire.requestFrom(_i2cAddress, 1);

  // Timed out so return
  if (nReceived != 1) return;
  
  // Read the values
  byte calibration = Wire.read();

  // Update the User
  Serial.print(mytime);
  Serial.print(",");
  Serial.print("System|Gyro|Accel|Magnet\t");
  sprintf(Message,"Calibration " BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(calibration));
  Serial.print(Message);

  // Wait 100ms so the output is slower
  delay(100);
}

void writeToCMPS14(byte n){

  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
    
  // Want the Command Register
  Wire.write(byte(0x00));

  // Send some data    
  Wire.write(n);
  
  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){

    Serial.println("communication error");
  }
  else
  { 
    Serial.println("OK");
  }

  // Wait 100ms
  delay(100);
}
