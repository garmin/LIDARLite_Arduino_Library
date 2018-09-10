/*------------------------------------------------------------------------------

  LIDARLite_v3HP Arduino Library
  LIDARLite_v3HP.cpp

  This library provides quick access to the basic functions of LIDAR-Lite
  via the Arduino interface. Additionally, it can provide a user of any
  platform with a template for their own application code.

  Copyright (c) 2018 Garmin Ltd. or its subsidiaries.

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
#include <stdint.h>
#include "LIDARLite_v3HP.h"

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
void LIDARLite_v3HP::configure(uint8_t configuration, uint8_t lidarliteAddress)
{
    uint8_t sigCountMax;
    uint8_t acqConfigReg;
    uint8_t refCountMax;
    uint8_t thresholdBypass;

    switch (configuration)
    {
        case 0: // Default mode, balanced performance
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case 1: // Short range, high speed
            sigCountMax     = 0x1d;
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case 2: // Default range, higher speed short range
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x00;
            refCountMax     = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case 3: // Maximum range
            sigCountMax     = 0xff;
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case 4: // High sensitivity detection, high erroneous measurements
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x80;
            break;

        case 5: // Low sensitivity detection, low erroneous measurements
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0xb0;
            break;

        case 6: // Short range, high speed, higher error
            sigCountMax     = 0x04;
            acqConfigReg    = 0x01; // turn off short_sig, mode pin = status output mode
            refCountMax     = 0x03;
            thresholdBypass = 0x00;
            break;
    }

    write(0x02, &sigCountMax    , 1, lidarliteAddress);
    write(0x04, &acqConfigReg   , 1, lidarliteAddress);
    write(0x12, &refCountMax    , 1, lidarliteAddress);
    write(0x1c, &thresholdBypass, 1, lidarliteAddress);
} /* LIDARLite_v3HP::configure */

/*------------------------------------------------------------------------------
  Set I2C Address

  Set Alternate I2C Device Address. See Operation Manual for additional info.

  Parameters
  ------------------------------------------------------------------------------
  newAddress: desired secondary I2C device address
  disableDefault: a non-zero value here means the default 0x62 I2C device
    address will be disabled.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v3HP::setI2Caddr (uint8_t newAddress, uint8_t disableDefault,
                                 uint8_t lidarliteAddress)
{
    uint8_t dataBytes[2];

    // Read UNIT_ID serial number bytes and write them into I2C_ID byte locations
    read (0x16, dataBytes, 2, lidarliteAddress);
    write(0x18, dataBytes, 2, lidarliteAddress);

    // Write the new I2C device address to registers
    // left shift by one to work around data alignment issue in v3HP
    dataBytes[0] = (newAddress << 1);
    write(0x1a, dataBytes, 1, lidarliteAddress);

    // Enable the new I2C device address using the default I2C device address
    read (0x1e, dataBytes, 1, lidarliteAddress);
    dataBytes[0] = dataBytes[0] | (1 << 4); // set bit to enable the new address
    write(0x1e, dataBytes, 1, lidarliteAddress);

    // If desired, disable default I2C device address (using the new I2C device address)
    if (disableDefault)
    {
        read (0x1e, dataBytes, 1, newAddress);
        dataBytes[0] = dataBytes[0] | (1 << 3); // set bit to disable default address
        write(0x1e, dataBytes, 1, newAddress);
    }
} /* LIDARLite_v3HP::setI2Caddr */

/*------------------------------------------------------------------------------
  Take Range

  Initiate a distance measurement by writing to register 0x00.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v3HP::takeRange(uint8_t lidarliteAddress)
{
    uint8_t dataByte = 0x01;

    write(0x00, &dataByte, 1, lidarliteAddress);
} /* LIDARLite_v3HP::takeRange */

/*------------------------------------------------------------------------------
  Wait for Busy Flag

  Blocking function to wait until the Lidar Lite's internal busy flag goes low

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v3HP::waitForBusy(uint8_t lidarliteAddress)
{
    uint16_t busyCounter = 0; // busyCounter counts number of times busy flag is checked, for timeout
    uint8_t  busyFlag    = 1; // busyFlag monitors when the device is done with a measurement

    while (busyFlag)      // Loop until device is not busy
    {
        // Handle timeout condition, exit while loop and goto bailout
        if (busyCounter > 9999)
        {
            break;
        }

        busyFlag = getBusyFlag(lidarliteAddress);

        // Increment busyCounter for timeout
        busyCounter++;
    }

    // bailout reports error over serial
    if (busyCounter > 9999)
    {
        Serial.println("> bailing out of waitForBusy()");
    }
} /* LIDARLite_v3HP::waitForBusy */

/*------------------------------------------------------------------------------
  Get Busy Flag

  Read BUSY flag from device registers. Function will return 0x00 if not busy.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
uint8_t LIDARLite_v3HP::getBusyFlag(uint8_t lidarliteAddress)
{
    uint8_t  busyFlag; // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    read(0x01, &busyFlag, 1, lidarliteAddress);

    // STATUS bit 0 is busyFlag
    busyFlag &= 0x01;

    return busyFlag;
} /* LIDARLite_v3HP::getBusyFlag */

/*------------------------------------------------------------------------------
  Read Distance

  Read and return result of distance measurement.

  Process
  ------------------------------------------------------------------------------
  1.  Read two bytes from register 0x8f and save
  2.  Shift the first value from 0x8f << 8 and add to second value from 0x8f.
      The result is the measured distance in centimeters.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
uint16_t LIDARLite_v3HP::readDistance(uint8_t lidarliteAddress)
{
    uint16_t  distance;
    uint8_t dataBytes[2];

    // Read two bytes from register 0x0f and 0x10 (autoincrement)
    read(0x0f, dataBytes, 2, lidarliteAddress);

    // Shift high byte and add to low byte
    distance = (dataBytes[0] << 8) | dataBytes[1];

    return (distance);
} /* LIDARLite_v3HP::readDistance */

/*------------------------------------------------------------------------------
  Write

  Perform I2C write to device. The I2C peripheral in the LidarLite v3 HP
  will receive multiple bytes in one I2C transmission. The first byte is
  always the register address. The the bytes that follow will be written
  into the specified register address first and then the internal address
  in the Lidar Lite will be auto-incremented for all following bytes.

  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to write to
  dataBytes: pointer to array of bytes to write
  numBytes:  number of bytes in 'dataBytes' array to write
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v3HP::write(uint8_t regAddr, uint8_t * dataBytes,
                           uint16_t numBytes, uint8_t lidarliteAddress)
{
    int nackCatcher;

    Wire.beginTransmission((int) lidarliteAddress);

    // Wire.write Syntax
    // -----------------------------------------------------------------
    // Wire.write(value)         - a value to send as a single byte
    // Wire.write(string)        - a string to send as a series of bytes
    // Wire.write(data, length)  - an array of data to send as bytes

    // First byte of every write sets the LidarLite's internal register address pointer
    Wire.write((int) regAddr);

    // Subsequent bytes are data writes
    Wire.write(dataBytes, (int) numBytes);

    // A nack means the device is not responding. Report the error over serial.
    nackCatcher = Wire.endTransmission();
    if (nackCatcher != 0)
    {
        Serial.println("> nack");
    }

    delayMicroseconds(100); // 100 us delay for robustness with successive reads and writes
} /* LIDARLite_v3HP::write */

/*------------------------------------------------------------------------------
  Read

  Perform I2C read from device.  The I2C peripheral in the LidarLite v3 HP
  will send multiple bytes in one I2C transmission. The register address must
  be set up by a previous I2C write. The bytes that follow will be read
  from the specified register address first and then the internal address
  pointer in the Lidar Lite will be auto-incremented for following bytes.

  Will detect an unresponsive device and report the error over serial.

  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to write to
  dataBytes: pointer to array of bytes to write
  numBytes:  number of bytes in 'dataBytes' array to write
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v3HP::read(uint8_t regAddr, uint8_t * dataBytes,
                          uint16_t numBytes, uint8_t lidarliteAddress)
{
    uint16_t i = 0;
    int nackCatcher = 0;

    // Set the internal register address pointer in the Lidar Lite
    Wire.beginTransmission((int) lidarliteAddress);
    Wire.write((int) regAddr); // Set the register to be read

    // A nack means the device is not responding, report the error over serial
    nackCatcher = Wire.endTransmission(false); // false means perform repeated start
    if (nackCatcher != 0)
    {
        Serial.println("> nack");
    }

    // Perform read, save in dataBytes array
    Wire.requestFrom((int)lidarliteAddress, (int) numBytes);
    if ((int) numBytes <= Wire.available())
    {
        while (i < numBytes)
        {
            dataBytes[i] = (uint8_t) Wire.read();
            i++;
        }
    }

} /* LIDARLite_v3HP::read */

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
  2.  Set test mode select by writing 0x07 to register 0x40
  3.  For as many readings as you want to take (max is 1024)
      1.  Read two bytes from 0x52
      2.  The Low byte is the value from the record
      3.  The high byte is the sign from the record

  Parameters
  ------------------------------------------------------------------------------
  separator: the separator between serial data words
  numberOfReadings: Default: 1024. Maximum of 1024
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v3HP::correlationRecordToSerial(
    uint16_t numberOfReadings, uint8_t lidarliteAddress)
{
    uint16_t  i = 0;
    uint8_t   dataBytes[2];          // Array to store read / write data
    int16_t   correlationValue = 0;  // Var to store value of correlation record

    // Test mode enable
    dataBytes[0] = 0x07;
    write(0x40, dataBytes, 1, lidarliteAddress);

    for (i=0 ; i<numberOfReadings ; i++)
    {
        read(0x52, dataBytes, 2, lidarliteAddress);

        //  Low byte is the value of the correlation record
        correlationValue = (uint16_t) dataBytes[0];

        // if upper byte lsb is one, the value is negative
        // so here we test to artifically sign extend the data
        if ( (int) dataBytes[1] == 1)
        {
            correlationValue |= 0xff00;
        }
        Serial.println(correlationValue);
    }

    // test mode disable
    dataBytes[0] = 0;
    write(0x40, dataBytes, 1, lidarliteAddress);
} /* LIDARLite_v3HP::correlationRecordToSerial */
