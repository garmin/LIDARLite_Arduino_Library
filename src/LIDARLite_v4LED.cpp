/*------------------------------------------------------------------------------

  LIDARLite_v4LED Arduino Library
  LIDARLite_v4LED.cpp

  This library provides quick access to the basic functions of LIDAR-Lite
  via the Arduino interface. Additionally, it can provide a user of any
  platform with a template for their own application code.

  Copyright (c) 2019 Garmin Ltd. or its subsidiaries.

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
#include "LIDARLite_v4LED.h"

/*------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Maximum range. Uses maximum acquisition count.
    1: Balanced performance.
    2: Short range, high speed. Reduces maximum acquisition count.
    3: Mid range, higher speed. Turns on quick termination
         detection for faster measurements at short range (with decreased
         accuracy)
    4: Maximum range, higher speed on short range targets. Turns on quick
         termination detection for faster measurements at short range (with
         decreased accuracy)
    5: Very short range, higher speed, high error. Reduces maximum
         acquisition count to a minimum for faster rep rates on very
         close targets with high error.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::configure(uint8_t configuration, uint8_t lidarliteAddress)
{
    uint8_t sigCountMax;
    uint8_t acqConfigReg;

    switch (configuration)
    {
        case 0: // Default mode - Maximum range
            sigCountMax     = 0xff;
            acqConfigReg    = 0x08;
            break;

        case 1: // Balanced performance
            sigCountMax     = 0x80;
            acqConfigReg    = 0x08;
            break;

        case 2: // Short range, high speed
            sigCountMax     = 0x18;
            acqConfigReg    = 0x00;
            break;

        case 3: // Mid range, higher speed on short range targets
            sigCountMax     = 0x80;
            acqConfigReg    = 0x00;
            break;

        case 4: // Maximum range, higher speed on short range targets
            sigCountMax     = 0xff;
            acqConfigReg    = 0x00;
            break;

        case 5: // Very short range, higher speed, high error
            sigCountMax     = 0x04;
            acqConfigReg    = 0x00;
            break;
    }

    write(0x05, &sigCountMax    , 1, lidarliteAddress);
    write(0xE5, &acqConfigReg   , 1, lidarliteAddress);
} /* LIDARLite_v4LED::configure */

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
void LIDARLite_v4LED::setI2Caddr (uint8_t newAddress, uint8_t disableDefault,
                                  uint8_t lidarliteAddress)
{
    uint8_t dataBytes[5];

    // Read 4-byte device serial number
    read (0x16, dataBytes, 4, lidarliteAddress);

    // Append the desired I2C address to the end of the serial number byte array
    dataBytes[4] = newAddress;

    // Write the serial number and new address in one 5-byte transaction
    write(0x16, dataBytes, 5, lidarliteAddress);

    // If desired, disable default I2C device address (using the new I2C device address)
    if (disableDefault)
    {
        dataBytes[0] = 0x01; // set bit to disable default address
        write(0x1b, dataBytes, 1, newAddress);
    }
} /* LIDARLite_v4LED::setI2Caddr */

/*------------------------------------------------------------------------------
  Take Range

  Initiate a distance measurement by writing to register 0x00.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::takeRange(uint8_t lidarliteAddress)
{
    uint8_t dataByte = 0x04;

    write(0x00, &dataByte, 1, lidarliteAddress);
} /* LIDARLite_v4LED::takeRange */

/*------------------------------------------------------------------------------
  Wait for Busy Flag

  Blocking function to wait until the Lidar Lite's internal busy flag goes low

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::waitForBusy(uint8_t lidarliteAddress)
{
    uint8_t  busyFlag;

    do
    {
        busyFlag = getBusyFlag(lidarliteAddress);
    } while (busyFlag);

} /* LIDARLite_v4LED::waitForBusy */

/*------------------------------------------------------------------------------
  Get Busy Flag

  Read BUSY flag from device registers. Function will return 0x00 if not busy.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
uint8_t LIDARLite_v4LED::getBusyFlag(uint8_t lidarliteAddress)
{
    uint8_t  statusByte = 0;
    uint8_t  busyFlag; // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    read(0x01, &statusByte, 1, lidarliteAddress);

    // STATUS bit 0 is busyFlag
    busyFlag = statusByte & 0x01;

    return busyFlag;
} /* LIDARLite_v4LED::getBusyFlag */

/*------------------------------------------------------------------------------
  Take Range using Trigger / Monitor Pins

  Initiate a distance measurement by toggling the trigger pin

  Parameters
  ------------------------------------------------------------------------------
  triggerPin: digital output pin connected to trigger input of LIDAR-Lite
  monitorPin: digital input pin connected to monitor output of LIDAR-Lite
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::takeRangeGpio(uint8_t triggerPin, uint8_t monitorPin)
{
    uint8_t busyFlag;

    if (digitalRead(triggerPin))
        digitalWrite(triggerPin, LOW);
    else
        digitalWrite(triggerPin, HIGH);

    // When LLv4 receives trigger command it will drive monitor pin low.
    // Wait for LLv4 to acknowledge receipt of command before moving on.
    do
    {
        busyFlag = getBusyFlagGpio(monitorPin);
    } while (!busyFlag);
} /* LIDARLite_v4LED::takeRangeGpio */

/*------------------------------------------------------------------------------
  Wait for Busy Flag using Trigger / Monitor Pins

  Blocking function to wait until the Lidar Lite's internal busy flag goes low

  Parameters
  ------------------------------------------------------------------------------
  monitorPin: digital input pin connected to monitor output of LIDAR-Lite
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::waitForBusyGpio(uint8_t monitorPin)
{
    uint8_t  busyFlag;

    do
    {
        busyFlag = getBusyFlagGpio(monitorPin);
    } while (busyFlag);

} /* LIDARLite_v4LED::waitForBusyGpio */

/*------------------------------------------------------------------------------
  Get Busy Flag using Trigger / Monitor Pins

  Check BUSY status via Monitor pin. Function will return 0x00 if not busy.

  Parameters
  ------------------------------------------------------------------------------
  monitorPin: digital input pin connected to monitor output of LIDAR-Lite
------------------------------------------------------------------------------*/
uint8_t LIDARLite_v4LED::getBusyFlagGpio(uint8_t monitorPin)
{
    uint8_t  busyFlag; // busyFlag monitors when the device is done with a measurement

    // Check busy flag via monitor pin
    if (digitalRead(monitorPin))
        busyFlag = 1;
    else
        busyFlag = 0;

    return busyFlag;
} /* LIDARLite_v4LED::getBusyFlagGpio */

/*------------------------------------------------------------------------------
  Read Distance

  Read and return the result of the most recent distance measurement.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
uint16_t LIDARLite_v4LED::readDistance(uint8_t lidarliteAddress)
{
    uint16_t  distance;
    uint8_t * dataBytes = (uint8_t *) &distance;

    // Read two bytes from registers 0x10 and 0x11
    read(0x10, dataBytes, 2, lidarliteAddress);

    return (distance);
} /* LIDARLite_v4LED::readDistance */

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
void LIDARLite_v4LED::write(uint8_t regAddr,  uint8_t * dataBytes,
                            uint8_t numBytes, uint8_t lidarliteAddress)
{
    int nackCatcher;

    Wire.beginTransmission((int) lidarliteAddress);

    // First byte of every write sets the LidarLite's internal register address pointer
    Wire.write((int) regAddr);

    // Subsequent bytes are data writes
    Wire.write(dataBytes, (int) numBytes);

    // A nack means the device is not responding. Report the error over serial.
    nackCatcher = Wire.endTransmission();
    if (nackCatcher != 0)
    {
        // handle nack issues in here
    }
} /* LIDARLite_v4LED::write */

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
void LIDARLite_v4LED::read(uint8_t regAddr,  uint8_t * dataBytes,
                           uint8_t numBytes, uint8_t lidarliteAddress)
{
    uint8_t   i = 0;
    uint8_t   numHere;

    // This single function performs the following actions -
    //     1) I2C START
    //     2) I2C write to set the address
    //     3) REPEATED START
    //     4) I2C read to fetch the required data
    //     5) I2C STOP
    Wire.requestFrom
    (
        lidarliteAddress, // Slave address
        numBytes,         // number of consecutive bytes to read
        regAddr,          // address of first register to read
        1,                // number of bytes in regAddr
        true              // true = set STOP condition following I2C read
    );

    numHere = Wire.available();

    while (i < numHere)
    {
        dataBytes[i] = Wire.read();
        i++;
    }

} /* LIDARLite_v4LED::read */

/*------------------------------------------------------------------------------
  Correlation Record Read

  The correlation record used to calculate distance can be read from the device.
  It has a bipolar wave shape, transitioning from a positive going portion to a
  roughly symmetrical negative going pulse. The point where the signal crosses
  zero represents the effective delay for the reference and return signals.

  Process
  ------------------------------------------------------------------------------
  1.  Take a distance reading (there is no correlation record without at least
      one distance reading being taken)
  2.  For as many points as you want to read from the record (max is 192) read
      the two byte signed correlation data point from 0x52

  Parameters
  ------------------------------------------------------------------------------
  correlationArray: pointer to memory location to store the correlation record
                    ** Two bytes for every correlation value must be
                       allocated by calling function
  numberOfReadings: Default = 192. Maximum = 192
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::correlationRecordRead(
    int16_t * correlationArray, uint8_t numberOfReadings, uint8_t lidarliteAddress)
{
    uint8_t   i;
    int16_t   correlationValue;
    uint8_t * dataBytes = (uint8_t *) &correlationValue;

    for (i=0 ; i<numberOfReadings ; i++)
    {
        read(0x52, dataBytes, 2, lidarliteAddress);
        correlationArray[i] = correlationValue;
    }
} /* LIDARLite_v4LED::correlationRecordRead */
