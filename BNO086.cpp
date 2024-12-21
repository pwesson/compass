// Digital Compass BNO086
// Copyright (C) 2024 https://www.roboticboat.uk
// e1884045-2825-47dd-8cdb-80039b9e278b
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


#include "BNO086.h"

#include <Wire.h>

void BNO086::SetupBMO086()
{
  delay(1000);

  // Restart the BMO080 module
  Restart();
  
  // Set Accelerator, reportID 0x01 at 50ms intervals
  SetPacketInterval(0x01, 50);

  // Set Gyroscope, reportID 0x02 at 50ms intervals
  SetPacketInterval(0x02, 50);

  // Set Magnet, reportID 0x03 at 50ms intervals
  SetPacketInterval(0x03, 50);

  // Set Linear Accelerator, reportID 0x04 at 50ms intervals
  //SetPacketInterval(0x04, 50);

  // Set Raw Accelerator, reportID 0x14 at 50ms intervals
  SetPacketInterval(0x14, 50);

  // Set Raw Gyroscope, reportID 0x15 at 50ms intervals
  SetPacketInterval(0x15, 50);

  // Set Raw Gyroscope, reportID 0x16 at 50ms intervals
  SetPacketInterval(0x16, 50);

  // Set Rotation Vector, reportID 0x08 at 50ms intervals
  //SetPacketInterval(0x05, 50);

  // Set Game Rotation Vector, reportID 0x08 at 50ms intervals
  SetPacketInterval(0x08, 50);

  // Set Geomagnetic Rotation Vector, reportID 0x09 at 50ms intervals
  //SetPacketInterval(0x09, 50);

}

void BNO086::Restart() {

  // Allocate memory
  int restart_Message[5];

  // Size of data transmission
  // Least significant byte (LSB) size of data
  restart_Message[0] = 0x05;

  // Most significant byte (MSB) size of data
  restart_Message[1] = 0x00;

  // Channel number hard coded at 1
  restart_Message[2] = 0x01;

  // Channel 1 sequence number
  restart_Message[3] = channel1_sequencialNumber++;

  // The Command Reference
  restart_Message[4] = 0x01;

  // Send the packet
  int status = writeBytes(5, restart_Message);
  
  if (status != 0){

    Serial.println("Restart status error");
  }

  // Wait for the BNO080 to restart. This delay is essential
  delay(500);
}

void BNO086::calibrate(byte input)
{
  // Do we calibration the accelerator?
  if (input == 'a'){
    calibrateAGM(true, false, false);
    Serial.println("Calibration accelerator");
    delay(500);
  }

  // Do we calibration the gyroscrope?
  if (input == 'g'){
    calibrateAGM(false, true, false);
    Serial.println("Calibration gyroscope");
    delay(500);
  }

  // Do we calibration the magnetometer?
  if (input == 'm'){
    calibrateAGM(false, false, true);
    Serial.println("Calibration magnetometer");
    delay(500);
  }

  // Do we stop calibration?
  if (input == 'x'){
    calibrateAGM(false, false, false);
    Serial.println("Stopped calibration");
    delay(500);
  }

  // Do we stop calibration?
  if (input == 's'){
    saveCalibration();
    Serial.println("Saved");
  }
}

void BNO086::calibrateAGM(bool isAccel, bool isGyro, bool isMagnet)
{
  // Allocate memory
  int calibrate_Message[16];
  
  // Create header. Least significant byte (LSB) size of data
  calibrate_Message[0] = (16) & 0xFF;

  // Most significant byte (MSB) size of data
  calibrate_Message[1] = ((16) >> 8) & 0xFF;

  // Channel number hard coded at 2
  calibrate_Message[2] = 0x02;

  // Sequence number
  calibrate_Message[3] = channel2_sequencialNumber++;

  // The Command Reference
  calibrate_Message[4] = 0xF2;

  // Command sequence number
  calibrate_Message[5] = command_sequencialNumber++;
  
  // Command - ME Calibration Command
  calibrate_Message[6] = 0x07;

  // Calibrate accelerator
  calibrate_Message[7] = 0;
  if (isAccel) calibrate_Message[7] = 1;

  // Calibrate gyroscope
  calibrate_Message[8] = 0;
  if (isGyro) calibrate_Message[8] = 1;

  // Calibrate magnetometer
  calibrate_Message[9] = 0;
  if (isMagnet) calibrate_Message[9] = 1;

  // Subcommand. Get ME Calibration
  calibrate_Message[10] = 0x01;
  
  calibrate_Message[11] = 0;
  calibrate_Message[12] = 0;
  calibrate_Message[13] = 0;
  calibrate_Message[14] = 0;
  calibrate_Message[15] = 0;
  
  // Send the packet
  int status = writeBytes(16, calibrate_Message);

  if (status != 0){

    Serial.print("calibrate Message status error");
    delay(500);
  }
}

void BNO086::saveCalibration(){

// Allocate memory
  int save_Message[16];
  
  // Create header. Least significant byte (LSB) size of data
  save_Message[0] = (16) & 0xFF;

  // Most significant byte (MSB) size of data
  save_Message[1] = ((16) >> 8) & 0xFF;

  // Channel number hard coded at 2
  save_Message[2] = 0x02;

  // Sequence number
  save_Message[3] = channel2_sequencialNumber++;

  // SHTP Command Reference
  save_Message[4] = 0xF2;

  // Command sequence number
  save_Message[5] = command_sequencialNumber++;
  
  // Command - Dynamic Calibration Data (DCD)
  save_Message[6] = 0x06;

  // Calibrate accelerator
  save_Message[7] = 0;
  save_Message[8] = 0;
  save_Message[9] = 0;
  save_Message[10] = 0;
  save_Message[11] = 0;
  save_Message[12] = 0;
  save_Message[13] = 0;
  save_Message[14] = 0;
  save_Message[15] = 0;
  
  // Send the packet
  int status = writeBytes(16, save_Message);

  if (status != 0){

    Serial.print("calibrate Message status error");
    delay(500);
  }

}


void BNO086::SetPacketInterval(int reportID, uint16_t milliSeconds)
{
  // Allocate memory
  int set_packet_Message[21];

  // Create header. Least significant byte (LSB) size of data
  set_packet_Message[0] = (21) & 0xFF;

  // Most significant byte (MSB) size of data
  set_packet_Message[1] = ((21) >> 8) & 0xFF;

  // Channel number hard coded at 2
  set_packet_Message[2] = 0x02;

  // Sequencial 8-bit number to indicate following packet
  set_packet_Message[3] = channel2_sequencialNumber++;

  // The Command Reference. Set features of the output
  set_packet_Message[4] = 0xFD;

  // ReportID reference
  set_packet_Message[5] = reportID;

  // Not set
  set_packet_Message[6] = 0;
  set_packet_Message[7] = 0;
  set_packet_Message[8] = 0;

  // Time interval in microseconds
  uint32_t microSeconds = (uint32_t)milliSeconds * 1000;

  // Set the microsecond interval
  set_packet_Message[9] = microSeconds & 0xFF;
  set_packet_Message[10] = (microSeconds >> 8) & 0xFF;
  set_packet_Message[11] = (microSeconds >> 16) & 0xFF;
  set_packet_Message[12] = (microSeconds >> 24) & 0xFF;

  // Not set
  set_packet_Message[13] = 0;
  set_packet_Message[14] = 0;
  set_packet_Message[15] = 0;
  set_packet_Message[16] = 0;
  set_packet_Message[17] = 0;
  set_packet_Message[18] = 0;
  set_packet_Message[19] = 0;
  set_packet_Message[20] = 0;

  // Send the packet
  int status = writeBytes(21, set_packet_Message);

  if (status != 0){

    Serial.print("SetPacketInterval status error");
    delay(500);
  }
}

float BNO086::Convert2float(int16_t intvalue, float divisor) {
  float fvalue = intvalue;
  return fvalue / divisor;
}

int BNO086::receivePacket()
{
  // Header bytes to receive
  int numHeaderBytes = 4;
  int headerBuffer[numHeaderBytes];

  int dataBuffer[I2C_BUFFER_SIZE];

  // Reset packet buffer pointer
  packetBufferPtr = 0;

  // Number of bytes to receive
  nReceived = Wire.requestFrom(_i2cAddress, numHeaderBytes);

  // Read the data into the headerBuffer
  for (int i = 0; i < numHeaderBytes; i++)
  {
    headerBuffer[i] = Wire.read();
    //sprintf (showHex, "%02x", headerBuffer[i]);
    //Serial.print(showHex);
  }

  // Section 1.3 Host communication 
  // 1.3.1 SHTP
  // Bit 15 = continuation flag
  if (bitRead(headerBuffer[1], 7) == 1){
    //Serial.print("Continuation ");
    bitSet(headerBuffer[1],7) = 0;
  }

  // Data package length
  uint16_t numDataBytes = (uint16_t)(headerBuffer[1] << 8 | headerBuffer[0]);

  if (numDataBytes == 0) return (int)8;

  numDataBytes = numDataBytes - numHeaderBytes;

  // Need to break up into 32 byte reads
  uint16_t bytesRemaining = numDataBytes;

  while (bytesRemaining  > 0)
  {
    uint16_t dataBytes = bytesRemaining;

    if (dataBytes > I2C_BUFFER_SIZE - 4) dataBytes = I2C_BUFFER_SIZE - 4;

    // Number of bytes to receive
    nReceived = Wire.requestFrom(_i2cAddress, (int)(dataBytes + 4));

    if (nReceived != dataBytes + 4) {
      Serial.println("failed 4");
      return (int)55;
    }

    // Skip over the SHTP header bytes 0,1,2 & 3
    for (int i = 0; i < 4; i++) Wire.read();

    // Read the data into the headerBuffer
    for (int i = 0; i < dataBytes; i++)
    {
      dataBuffer[i] = Wire.read();
      packetBuffer[packetBufferPtr] = dataBuffer[i];
      packetBufferPtr++;
      if (packetBufferPtr == PACKET_BUFFER_SIZE) packetBufferPtr = PACKET_BUFFER_SIZE - 1;
    }

    bytesRemaining = bytesRemaining - dataBytes;
  }

  // Lets try and process this packetBuffer.
  uint16_t tmp = 5;
  int inc = 0;
  while (tmp < packetBufferPtr) {
    //Serial.print("[");
    //sprintf (showHex, "%02x", packetBuffer[tmp]);
    //Serial.print(showHex);
    //Serial.print(": size ");
    inc = processReport(packetBuffer[tmp], tmp);
    //Serial.print(inc);
    //Serial.print("] ");
    if (inc == 0) break;
    tmp = tmp + inc;
  }
  //Serial.println("");

  return (int)0;
}

int BNO086::processReport(int reportID, uint16_t tmp)
{
  // Want to know the number of bytes in the report
  switch (reportID) {
    case 0x01:
      // Accelerometer             (0x01)
      // The Q point is 8. 2^8=256 (units of reading)
      accel.sequence_number = packetBuffer[tmp + 1] ;
      accel.status = packetBuffer[tmp + 2] & 0x03;
      accel.X = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 256.0f);
      accel.Y = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 256.0f);
      accel.Z = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 256.0f);
      // Return number of bytes in the report just processed
      return (int)10;

    case 0x02:
      // Gyroscope Calibrated      (0x02)
      // The Q point is 9. 2^9=512 (units of reading)
      gyro.sequence_number = packetBuffer[tmp + 1] ;
      gyro.status = packetBuffer[tmp + 2] & 0x03;
      gyro.X = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 512.0f);
      gyro.Y = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 512.0f);
      gyro.Z = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 512.0f);
      // Return number of bytes in the report just processed
      return (int)10;

    case 0x03:
      // Magnetic Field Calibrated (0x03)
      // The Q point is 4. 2^4 = 16 (units of reading)
      magnet.sequence_number = packetBuffer[tmp + 1] ;
      magnet.status = packetBuffer[tmp + 2] & 0x03;
      magnet.X = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 16.0f);
      magnet.Y = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 16.0f);
      magnet.Z = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 16.0f);
      // Return number of bytes in the report just processed
      return (int)10;

    case 0x04:
      // Linear Acceleration       (0x04)
      // The Q point is 8. 2^8=256 (units of reading)
      linearAccel.sequence_number = packetBuffer[tmp + 1] ;
      linearAccel.status = packetBuffer[tmp + 2] & 0x03;
      linearAccel.X = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 256.0f);
      linearAccel.Y = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 256.0f);
      linearAccel.Z = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 256.0f);
      // Return number of bytes in the report just processed
      return (int)10;

    case 0x05:
      // Rotation Vector           (0x05)
      // The Q point is 14. 2^14 = 16384 (units of reading)
      quantRotationVec.sequence_number = packetBuffer[tmp + 1] ;
      quantRotationVec.status = packetBuffer[tmp + 2] & 0x03;
      quantRotationVec.i = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 16384.0f);
      quantRotationVec.j = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 16384.0f);
      quantRotationVec.k = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 16384.0f);
      quantRotationVec.real = Convert2float((int16_t) packetBuffer[tmp + 11] << 8 | packetBuffer[tmp + 10], 16384.0f);
      quantRotationVec.accuracy = (int16_t) packetBuffer[tmp + 13] << 8 | packetBuffer[tmp + 12];
      // Return number of bytes in the report just processed
      return (int)14;

    case 0x06:
      // Gravity                   (0x06)
      // The Q point is 8. 2^8=256 (units of reading)
      gravity.sequence_number = packetBuffer[tmp + 1] ;
      gravity.status = packetBuffer[tmp + 2] & 0x03;
      gravity.X = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 256.0f);
      gravity.Y = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 256.0f);
      gravity.Z = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 256.0f);
      // Return number of bytes in the report just processed
      return (int)10;

    case 0x07:
      // Gyroscope Uncalibrated      (0x07)
      // The Q point is 9. 2^9=512 (units of reading)
      gyro_uncalibrated.sequence_number = packetBuffer[tmp + 1] ;
      gyro_uncalibrated.status = packetBuffer[tmp + 2] & 0x03;
      gyro_uncalibrated.X = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 512.0f);
      gyro_uncalibrated.Y = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 512.0f);
      gyro_uncalibrated.Z = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 512.0f);

      gyro_uncalibrated_bias.sequence_number = packetBuffer[tmp + 1] ;
      gyro_uncalibrated_bias.status = packetBuffer[tmp + 2] & 0x03;
      gyro_uncalibrated_bias.X = Convert2float((int16_t) packetBuffer[tmp + 11] << 8 | packetBuffer[tmp + 10], 512.0f);
      gyro_uncalibrated_bias.Y = Convert2float((int16_t) packetBuffer[tmp + 13] << 8 | packetBuffer[tmp + 12], 512.0f);
      gyro_uncalibrated_bias.Z = Convert2float((int16_t) packetBuffer[tmp + 15] << 8 | packetBuffer[tmp + 14], 512.0f);
      // Return number of bytes in the report just processed
      return (int)16;

    case 0x08:
      // Game Rotation Vector      (0x08)
      // The Q point is 14. 2^14 = 16384 (units of reading)
      quantGameRotationVec.sequence_number = packetBuffer[tmp + 1] ;
      quantGameRotationVec.status = packetBuffer[tmp + 2] & 0x03;
      quantGameRotationVec.i = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 16384.0f);
      quantGameRotationVec.j = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 16384.0f);
      quantGameRotationVec.k = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 16384.0f);
      quantGameRotationVec.real = Convert2float((int16_t) packetBuffer[tmp + 11] << 8 | packetBuffer[tmp + 10], 16384.0f);
      quantGameRotationVec.accuracy = -1;

      // Return number of bytes in the report just processed
      return (int)12;

    case 0x09:
      // Geomagnetic Rotation Vec  (0x09)
      // The Q point is 14. 2^14 = 16384 (units of reading)
      quantGeoRotationVec.sequence_number = packetBuffer[tmp + 1] ;
      quantGeoRotationVec.status = packetBuffer[tmp + 2] & 0x03;
      quantGeoRotationVec.i = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 16384.0f);
      quantGeoRotationVec.j = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 16384.0f);
      quantGeoRotationVec.k = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 16384.0f);
      quantGeoRotationVec.real = Convert2float((int16_t) packetBuffer[tmp + 11] << 8 | packetBuffer[tmp + 10], 16384.0f);
      quantGeoRotationVec.accuracy = (int16_t) packetBuffer[tmp + 13] << 8 | packetBuffer[tmp + 12];

      // Return number of bytes in the report just processed
      return (int)14;

    case 0x0F:
      // Magnetic Field Uncalibrated (0x0F)
      // The Q point is 4. 2^4 = 16 (units of reading)
      magnet_uncalibrated_soft_iron.sequence_number = packetBuffer[tmp + 1] ;
      magnet_uncalibrated_soft_iron.status = packetBuffer[tmp + 2] & 0x03;
      magnet_uncalibrated_soft_iron.X = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 16.0f);
      magnet_uncalibrated_soft_iron.Y = Convert2float((int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6], 16.0f);
      magnet_uncalibrated_soft_iron.Z = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 16.0f);
      
      magnet_uncalibrated_hard_iron.sequence_number = packetBuffer[tmp + 1] ;
      magnet_uncalibrated_hard_iron.status = packetBuffer[tmp + 2] & 0x03;
      magnet_uncalibrated_hard_iron.X = Convert2float((int16_t) packetBuffer[tmp + 11] << 8 | packetBuffer[tmp + 10], 16.0f);
      magnet_uncalibrated_hard_iron.Y = Convert2float((int16_t) packetBuffer[tmp + 13] << 8 | packetBuffer[tmp + 12], 16.0f);
      magnet_uncalibrated_hard_iron.Z = Convert2float((int16_t) packetBuffer[tmp + 15] << 8 | packetBuffer[tmp + 14], 16.0f);
      // Return number of bytes in the report just processed
      return (int)16;

    case 0x10:
      // Tap Detector              (0x10)
      // Bit 0 = X axis tap.
      // Bit 1 = X axis positive tap
      // Bit 2 = Y axis tap
      // Bit 3 = Y axis positive tap
      // Bit 4 = Z axis tap
      // Bit 5 = Z axis positive tap
      // Bit 6 = double tap
      tap_status = packetBuffer[tmp + 2] & 0x03;
      tap_detector = packetBuffer[tmp + 4];
      // Return number of bytes in the report just processed
      return (int)5;

    case 0x11:
      // Step Counter              (0x11)
      // The Q point is 0. 2^0 = 1 (units of reading)
      // Note the step counter will loop
      step_counter.sequence_number = packetBuffer[tmp + 1] ;
      step_counter.status = packetBuffer[tmp + 2] & 0x03;
      step_counter.value = Convert2float((int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8], 1.0f);
      // Return number of bytes in the report just processed
      return (int)12;

    case 0x12:
      // Significant Motion        (0x12)
      // 1 = significant motion detected
      significant_motion.sequence_number = packetBuffer[tmp + 1] ;
      significant_motion.status = packetBuffer[tmp + 2] & 0x03;
      significant_motion.value = Convert2float((int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4], 1.0f);
      // Return number of bytes in the report just processed
      return (int)6;

    case 0x13:
      // Stability Classifier      (0x13)
      // 0 = unknown
      // 1 = stable
      // 2 = Some very minor movements (Gyro calibration must be enabled)
      // 3 = Stable
      // 4 = Motion
      // 5+ = reserved
      stability_classification.sequence_number = packetBuffer[tmp + 1] ;
      stability_classification.status = packetBuffer[tmp + 2] & 0x03;
      stability_classification.value = packetBuffer[tmp + 4];
      // Return number of bytes in the report just processed
      return (int)6;

    case 0x14:
      // Raw Accelerometer         (0x14)
      // The units are ADCs
      accel_raw.sequence_number = packetBuffer[tmp + 1] ;
      accel_raw.status = packetBuffer[tmp + 2] & 0x03;
      accel_raw.X = (int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4];
      accel_raw.Y = (int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6];
      accel_raw.Z = (int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8];
      // Return number of bytes in the report just processed
      return (int)16;

    case 0x15:
      // Raw Gyroscope             (0x15)
      // The units are ADCs
      gyro_raw.sequence_number = packetBuffer[tmp + 1] ;
      gyro_raw.status = packetBuffer[tmp + 2] & 0x03;
      gyro_raw.X = (int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4];
      gyro_raw.Y = (int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6];
      gyro_raw.Z = (int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8];
      // Return number of bytes in the report just processed
      return (int)16;

    case 0x16:
      // Raw Magnetometer          (0x16)
      // The units are ADCs
      magnet_raw.sequence_number = packetBuffer[tmp + 1] ;
      magnet_raw.status = packetBuffer[tmp + 2] & 0x03;
      magnet_raw.X = (int16_t) packetBuffer[tmp + 5] << 8 | packetBuffer[tmp + 4];
      magnet_raw.Y = (int16_t) packetBuffer[tmp + 7] << 8 | packetBuffer[tmp + 6];
      magnet_raw.Z = (int16_t) packetBuffer[tmp + 9] << 8 | packetBuffer[tmp + 8];
      // Return number of bytes in the report just processed
      return (int)16;
      
    case 0x18:
      // Step Detector             (0x18)
      return (int)8;
    case 0x19:
      // Shake Detector            (0x19)
      return (int)6;
    case 0x1A:
      // Flip Detector             (0x1A)
      return (int)6;
    case 0x1B:
      // Pickup Detector           (0x1B)
      return (int)6;
    case 0x1C:
      // Stability Detector        (0x1C)
      return (int)6;
    case 0x1F:
      // Sleep Detector            (0x1F)
      return (int)6;
    case 0x20:
      // Tilt Detector             (0x20)
      return (int)6;
    case 0x28:
      // ARVR-Stabilized Rotation Vector  (0x28)
      return (int)14;
    case 0x29:
      // ARVR-Stabilized Game Rotation Vector (0x29)
      return (int)12;
    case 0x2A:
      // Gyro-Integrated Rotation Vector    (0x2A)
      return (int)14;

    case 0xF1:
      // Command Response    (0xF1)
      // Currently a catch-all for all command responses
      calibration.channel_sequencialNumber = packetBuffer[tmp + 1];
      calibration.command_sequencialNumber = packetBuffer[tmp + 3];
      calibration.response_sequentialNumber = packetBuffer[tmp + 4];
      calibration.status = packetBuffer[tmp + 5];
      calibration.accel_calibration_enabled = packetBuffer[tmp + 6];
      calibration.gyro_calibration_enabled = packetBuffer[tmp + 7];
      calibration.magnet_calibration_enabled = packetBuffer[tmp + 8];
      // Return number of bytes in the report just processed
      return (int)16;

    case 0xFA:
      // Timestamp Rebase          (0xFA)
      return (int)5;
    default:
      return 0;
  }
}


//========== i2c commands ===============
//
// NACK errors are
// 0: OK - successful send
// 1: Send receiveBuffer too large for the twi receiveBuffer.
// 2: Register address was sent and a NACK received. Master should send a STOP condition.
// 3: Data was sent and a NACK received. Slave has sent all data. Master can send a STOP condition.
// 4: A different twi error took place
// 5: Time out on waiting for data to be received.
// 6: Checking data was correctly written failed

int BNO086::readRegister(int registerAddress, int* dest)
{
  // Initialise the transmit message buffer
  Wire.beginTransmission(_i2cAddress);

  // Add the i2c module Register address to transmit message buffer
  Wire.write(registerAddress);

  // Transmit message and accept NACK response
  int nackCatcher = Wire.endTransmission(false);

  // Return if we have a connection problem
  if (nackCatcher != 0) return nackCatcher;

  // Number of bytes to receive
  nReceived = Wire.requestFrom(_i2cAddress, (int)1);

  if (nReceived == 0) {
    Serial.println("failed 1");
    return (int)5;
  }

  // Read the data
  *dest = Wire.read();

  // All ok
  return (int)0;
}

int BNO086::readRegisters(int registerAddress, int numBytes, int* dest)
{
  // Initialise the transmit message buffer
  Wire.beginTransmission(_i2cAddress);

  // Add the i2c module Register address to transmit message buffer
  Wire.write(registerAddress);

  // Transmit message and accept NACK response
  int nackCatcher = Wire.endTransmission(false);

  // Return if we have a connection problem
  if (nackCatcher != 0) return nackCatcher;

  // Number of bytes to receive
  nReceived = Wire.requestFrom(_i2cAddress, numBytes);

  int i = 0;

  // read the data into the receiveBuffer
  while ( Wire.available() )
  {
    dest[i++] = Wire.read();
  }

  // All ok
  return (int)0;
}

int BNO086::writeRegister(int registerAddress, int dataByte) {

  // Initialise the transmit message buffer
  Wire.beginTransmission(_i2cAddress);

  // Add the i2c module Register address to transmit message buffer
  Wire.write(registerAddress);

  // Add the command data to the transmit message buffer
  Wire.write(dataByte);

  // Transmit message and accept NACK response
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) return nackCatcher;

  // Wait for 10ms
  delay(10);

  // Allocate memory.
  int checkByte;

  // We are now going to check the operation was successful
  // This is not really required, but most of the time we are debugging
  // Read the Register
  readRegister(registerAddress, &checkByte);

  // Check the data read back is the same
  if (checkByte == dataByte) return (int)0;

  // Update the User
  Serial.println("ERROR: i2c module register not accepting writes");

  // An error happened
  return (int)6;
}

int BNO086::writeBytes(int numBytes, int* dataBytes) {

  // Initialise the transmit message buffer
  Wire.beginTransmission(_i2cAddress);

  // Loop over the bytes to send
  for (int i = 0; i < numBytes; i++)
  {
    // Add the command data to the transmit message buffer
    Wire.write(dataBytes[i]);
  }

  // Transmit message and accept NACK response
  // The Wire.endTransmission() is used when writing data only
  int nackCatcher = Wire.endTransmission();

  // Return nack
  return nackCatcher;
}



void BNO086::Quanternion2Euler(Quaternion q)
{
  // Calculates the Euler ZYX angles
  // Derived by comparing the Quaternion and Euler rotation matrices
  // Return angles in degrees (not radians)

  float old_roll = euler.roll;
  float old_pitch = euler.pitch;
  float old_yaw = euler.yaw;

  float norm = sqrt(q.real * q.real + q.i * q.i + q.j * q.j + q.k * q.k);
  float   dqw = q.real / norm;
  float   dqx = q.i / norm;
  float   dqy = q.j / norm;
  float   dqz = q.k / norm;

  float ysqr = dqy * dqy;

  // roll (x-axis rotation)
  float t0 = +2.0 * (dqw * dqx + dqy * dqz);
  float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
  euler.roll = atan2(t0, t1) * 180.0f / PI;

  // pitch (y-axis rotation)
  float t2 = +2.0 * (dqw * dqy - dqz * dqx);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  euler.pitch = asin(t2) * 180.0f / PI;

  // yaw (z-axis rotation)
  float t3 = +2.0 * (dqw * dqz + dqx * dqy);
  float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
  euler.yaw = atan2(t3, t4) * 180.0f / PI;

  euler.yaw = -euler.yaw;
  if (euler.yaw > 360) euler.yaw -= 360;
  if (euler.yaw < 0) euler.yaw += 360;

  // Data quality check
  if (isnan(euler.roll)) {euler.roll = old_roll;}
  if (isnan(euler.pitch)) {euler.pitch = old_pitch;}
  if (isnan(euler.yaw)) {euler.yaw = old_yaw;}

}

void BNO086::printStatus(byte level) {

  // Want to know the number of bytes in the report
  switch (level) {
    case 0x00:
      Serial.print("D");
      return;
  case 0x01:
      Serial.print("C");
      return;
  case 0x02:
      Serial.print("B");
      return;
  case 0x03:
      Serial.print("A");
      return;
  } 
}
