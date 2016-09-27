/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  LIDARLite.cpp

  This library provides quick access to the basic functions of LIDAR-Lite
  via the Arduino interface. Additionally, it can provide a user of any
  platform with a template for their own application code.

  Copyright (c) 2016 Garmin Ltd. or its subsidiaries.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

------------------------------------------------------------------------------*/

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include "LIDARLite.h"

/*------------------------------------------------------------------------------
  Constructor

  Use LIDARLite::begin to initialize.
------------------------------------------------------------------------------*/
LIDARLite::LIDARLite(){}

/*------------------------------------------------------------------------------
  Begin

  Starts the sensor and I2C.

  Parameters
  ------------------------------------------------------------------------------
  configuration: Default 0. Selects one of several preset configurations.
  fasti2c: Default 100 kHz. I2C base frequency.
    If true I2C frequency is set to 400kHz.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::begin(int configuration, bool fasti2c, char lidarliteAddress)
{
  Wire.begin(); // Start I2C
  if(fasti2c)
  {
    #if ARDUINO >= 157
      Wire.setClock(400000UL); // Set I2C frequency to 400kHz, for Arduino Due
    #else
      TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
    #endif
  }
  configure(configuration, lidarliteAddress); // Configuration settings
} /* LIDARLite::begin */

/*------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Default mode, balanced performance.
    1: Short range, high speed. Uses 0x1d maximum acquisition count.
    2: Default range, higher speed short range. Turns on quick termination
        detection for faster measurements at short range (with decreased
        accuracy)
    3: Maximum range. Uses 0xff maximum acquisition count.
    4: High sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for high sensitivity and noise.
    5: Low sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for low sensitivity and noise.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::configure(int configuration, char lidarliteAddress)
{
  switch (configuration)
  {
    case 0: // Default mode, balanced performance
      write(0x02,0x80,lidarliteAddress); // Default
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0x00,lidarliteAddress); // Default
    break;

    case 1: // Short range, high speed
      write(0x02,0x1d,lidarliteAddress);
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0x00,lidarliteAddress); // Default
    break;

    case 2: // Default range, higher speed short range
      write(0x02,0x80,lidarliteAddress); // Default
      write(0x04,0x00,lidarliteAddress);
      write(0x1c,0x00,lidarliteAddress); // Default
    break;

    case 3: // Maximum range
      write(0x02,0xff,lidarliteAddress);
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0x00,lidarliteAddress); // Default
    break;

    case 4: // High sensitivity detection, high erroneous measurements
      write(0x02,0x80,lidarliteAddress); // Default
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0x20,lidarliteAddress);
    break;

    case 5: // Low sensitivity detection, low erroneous measurementss
      write(0x02,0x80,lidarliteAddress); // Default
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0x60,lidarliteAddress);
    break;
  }
} /* LIDARLite::configure */

/*------------------------------------------------------------------------------
  Reset

  Reset device. The device reloads default register settings, including the
  default I2C address. Re-initialization takes approximately 22ms.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::reset(char lidarliteAddress)
{
  write(0x00,0x00,lidarliteAddress);
} /* LIDARLite::reset */

/*------------------------------------------------------------------------------
  Distance

  Take a distance measurement and read the result.

  Process
  ------------------------------------------------------------------------------
  1.  Write 0x04 or 0x03 to register 0x00 to initiate an aquisition.
  2.  Read register 0x01 (this is handled in the read() command)
      - if the first bit is "1" then the sensor is busy, loop until the first
        bit is "0"
      - if the first bit is "0" then the sensor is ready
  3.  Read two bytes from register 0x8f and save
  4.  Shift the first value from 0x8f << 8 and add to second value from 0x8f.
      The result is the measured distance in centimeters.

  Parameters
  ------------------------------------------------------------------------------
  biasCorrection: Default true. Take aquisition with receiver bias
    correction. If set to false measurements will be faster. Receiver bias
    correction must be performed periodically. (e.g. 1 out of every 100
    readings).
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
int LIDARLite::distance(bool biasCorrection, char lidarliteAddress)
{
  if(biasCorrection)
  {
    // Take acquisition & correlation processing with receiver bias correction
    write(0x00,0x04,lidarliteAddress);
  }
  else
  {
    // Take acquisition & correlation processing without receiver bias correction
    write(0x00,0x03,lidarliteAddress);
  }
  // Array to store high and low bytes of distance
  byte distanceArray[2];
  // Read two bytes from register 0x8f (autoincrement for reading 0x0f and 0x10)
  read(0x8f,2,distanceArray,true,lidarliteAddress);
  // Shift high byte and add to low byte
  int distance = (distanceArray[0] << 8) + distanceArray[1];
  return(distance);
} /* LIDARLite::distance */

/*------------------------------------------------------------------------------
  Write

  Perform I2C write to device.

  Parameters
  ------------------------------------------------------------------------------
  myAddress: register address to write to.
  myValue: value to write.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::write(char myAddress, char myValue, char lidarliteAddress)
{
  Wire.beginTransmission((int)lidarliteAddress);
  Wire.write((int)myAddress); // Set register for write
  Wire.write((int)myValue); // Write myValue to register

  // A nack means the device is not responding, report the error over serial
  int nackCatcher = Wire.endTransmission();
  if(nackCatcher != 0)
  {
    Serial.println("> nack");
  }

  delay(1); // 1 ms delay for robustness with successive reads and writes
} /* LIDARLite::write */

/*------------------------------------------------------------------------------
  Read

  Perform I2C read from device. Will detect an unresponsive device and report
  the error over serial. The optional busy flag monitoring
  can be used to read registers that are updated at the end of a distance
  measurement to obtain the new data.

  Parameters
  ------------------------------------------------------------------------------
  myAddress: register address to read from.
  numOfBytes: numbers of bytes to read. Can be 1 or 2.
  arrayToSave: an array to store the read values.
  monitorBusyFlag: if true, the routine will repeatedly read the status
    register until the busy flag (LSB) is 0.
------------------------------------------------------------------------------*/
void LIDARLite::read(char myAddress, int numOfBytes, byte arrayToSave[2], bool monitorBusyFlag, char lidarliteAddress)
{
  int busyFlag = 0; // busyFlag monitors when the device is done with a measurement
  if(monitorBusyFlag)
  {
    busyFlag = 1; // Begin read immediately if not monitoring busy flag
  }
  int busyCounter = 0; // busyCounter counts number of times busy flag is checked, for timeout

  while(busyFlag != 0) // Loop until device is not busy
  {
    // Read status register to check busy flag
    Wire.beginTransmission((int)lidarliteAddress);
    Wire.write(0x01); // Set the status register to be read

    // A nack means the device is not responding, report the error over serial
    int nackCatcher = Wire.endTransmission();
    if(nackCatcher != 0)
    {
      Serial.println("> nack");
    }

    Wire.requestFrom((int)lidarliteAddress,1); // Read register 0x01
    busyFlag = bitRead(Wire.read(),0); // Assign the LSB of the status register to busyFlag

    busyCounter++; // Increment busyCounter for timeout

    // Handle timeout condition, exit while loop and goto bailout
    if(busyCounter > 9999)
    {
      goto bailout;
    }
  }

  // Device is not busy, begin read
  if(busyFlag == 0)
  {
    Wire.beginTransmission((int)lidarliteAddress);
    Wire.write((int)myAddress); // Set the register to be read

    // A nack means the device is not responding, report the error over serial
    int nackCatcher = Wire.endTransmission();
    if(nackCatcher != 0)
    {
      Serial.println("> nack");
    }

    // Perform read of 1 or 2 bytes, save in arrayToSave
    Wire.requestFrom((int)lidarliteAddress, numOfBytes);
    int i = 0;
    if(numOfBytes <= Wire.available())
    {
      while(i < numOfBytes)
      {
        arrayToSave[i] = Wire.read();
        i++;
      }
    }
  }

  // bailout reports error over serial
  if(busyCounter > 9999)
  {
    bailout:
      busyCounter = 0;
      Serial.println("> Bailout");
  }
} /* LIDARLite::read */

/*------------------------------------------------------------------------------
  Correlation Record To Serial

  The correlation record used to calculate distance can be read from the device.
  It has a bipolar wave shape, transitioning from a positive going portion to a
  roughly symmetrical negative going pulse. The point where the signal crosses
  zero represents the effective delay for the reference and return signals.

  Process
  ------------------------------------------------------------------------------
  1.  Take a distance reading (there is no correlation record without at least
      one distance reading being taken)
  2.  Select memory bank by writing 0xc0 to register 0x5d
  3.  Set test mode select by writing 0x07 to register 0x40
  4.  For as many readings as you want to take (max is 1024)
      1.  Read two bytes from 0xd2
      2.  The Low byte is the value from the record
      3.  The high byte is the sign from the record

  Parameters
  ------------------------------------------------------------------------------
  separator: the separator between serial data words
  numberOfReadings: Default: 256. Maximum of 1024
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::correlationRecordToSerial(char separator, int numberOfReadings, char lidarliteAddress)
{

  // Array to store read values
  byte correlationArray[2];
  // Var to store value of correlation record
  int correlationValue = 0;
  //  Selects memory bank
  write(0x5d,0xc0,lidarliteAddress);
  // Test mode enable
  write(0x40, 0x07,lidarliteAddress);
  for(int i = 0; i<numberOfReadings; i++){
    // Select single byte
    read(0xd2,2,correlationArray,false,lidarliteAddress);
    //  Low byte is the value of the correlation record
    correlationValue = correlationArray[0];
    // if upper byte lsb is set, the value is negative
    if((int)correlationArray[1] == 1){
      correlationValue |= 0xff00;
    }
    Serial.print((int)correlationValue);
    Serial.print(separator);
  }
  // test mode disable
  write(0x40,0x00,lidarliteAddress);
} /* LIDARLite::correlationRecordToSerial */
