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
  Reset Reference Filter

  In some scenarios, power-on transients in the LIDAR-Lite v3HP can result in
  initial measurements that may be a few centimeters short of actual distance.
  This symptom will eventually rectify itself after a few hundred measurements.
  This symptom can also be rectified more quickly by resetting the unit's internal
  reference filter. The process here illustrates how to disable the internal
  reference filter, trigger a measurement (forcing re-init of the reference
  filter), and then re-enable the filter.

  Process
  ------------------------------------------------------------------------------
  1.  Disable the LIDAR-Lite reference filter
  2.  Set reference integration count to max
  3.  Trigger a measurement
  4.  Restore reference integration count
  5.  Re-enable reference filter

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v3HP::resetReferenceFilter(uint8_t lidarliteAddress)
{
    uint8_t dataBytes[2];
    uint8_t acqConfigReg;
    uint8_t refCountMax;

    // Set bit 4 of the acquisition configuration register (disable reference filter)
    read(0x04, dataBytes, 1, lidarliteAddress);  // Read address 0x04 (acquisition config register)
    acqConfigReg = dataBytes[0];                 // store for later restoration
    dataBytes[0] = dataBytes[0] | 0x10;          // turn on disable of ref filter
    write(0x04, dataBytes, 1, lidarliteAddress); // write it back

    // Set reference integration count to max
    read(0x12, dataBytes, 1, lidarliteAddress);  // Read address 0x12 (ref integration count)
    refCountMax = dataBytes[0];                  // store for later restoration
    dataBytes[0] = 0xff;                         // we want to reference to overflow quickly
    write(0x12, dataBytes, 1, lidarliteAddress); // write ref integration count

    // Trigger a measurement
    waitForBusy(lidarliteAddress);
    takeRange(lidarliteAddress);
    waitForBusy(lidarliteAddress);
    // ... no need to read the distance, it is immaterial

    // Restore previous reference integration count
    dataBytes[0] = refCountMax;
    write(0x12, dataBytes, 1, lidarliteAddress);

    // Restore previous acquisition configuration register (re-enabling reference filter)
    dataBytes[0] = acqConfigReg;
    write(0x04, dataBytes, 1, lidarliteAddress);
} /* LIDARLite_v3HP::resetReferenceFilter */

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
void LIDARLite_v3HP::write(uint8_t regAddr,  uint8_t * dataBytes,
                           uint8_t numBytes, uint8_t lidarliteAddress)
{
    Wire.beginTransmission(lidarliteAddress);

    // Wire.write Syntax
    // -----------------------------------------------------------------
    // Wire.write(value)         - a value to send as a single byte
    // Wire.write(string)        - a string to send as a series of bytes
    // Wire.write(data, length)  - an array of data to send as bytes

    // First byte of every write sets the LidarLite's internal register address pointer
    Wire.write(regAddr);

    // Subsequent bytes are data writes
    Wire.write(dataBytes, numBytes);

    // A nack means the device is not responding. Report the error over serial.
    if ( Wire.endTransmission() )
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
void LIDARLite_v3HP::read(uint8_t regAddr,  uint8_t * dataBytes,
                          uint8_t numBytes, uint8_t lidarliteAddress)
{
    // This single function performs the following actions -
    //     1) I2C START
    //     2) I2C write to set the address
    //     3) I2C REPEATED START
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

    uint8_t  numHere = Wire.available();
    uint8_t  i       = 0;

    while (i < numHere)
    {
        dataBytes[i] = Wire.read();
        i++;
    }

    delayMicroseconds(100); // 100 us delay for robustness with successive reads and writes
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

/*------------------------------------------------------------------------------
  Correlation Peak Stack Read

  Following a distance measurement, the correlation record is searched and a peak
  stack table is built. The nine total entries in the peak stack table are
  readable via registers, as illustrated below. This function reads all stack
  entries, calculates distances from those entries, and then returns arrays
  containing the peak magnitudes and associated distances.

  - The first stack entry represents internal reference measurement data.
  - The next eight entries represent the largest peaks in the correlation record.
      - The eight range entries are sorted largest peak to smallest peak.
      - The first of these eight entries is assumed to be the intended target
        and is used to calculate the distance data stored in I2C registers.
      - The remaining entries most often represent system noise.

  Each stack entry is comprised of three 16-bit values (6 total bytes)
      1st value = Peak Strength (Magnitude of Peak)
      2nd value = Zero crossing address (Coarse Distance)
      3rd value = Interpolated value (Fine Distance)

  Note: There are calibration and compensation circuits not available
        outside the LIDAR-Lite system, but the following shows how to
        quickly extract multiple signatures from the correlation results.
        Using this method does not result in data that matches the
        device distance registers exactly, but it is representative.

  Note: Using this method, some negative distances can be produced
        if noise artifacts exist very early in the correlation record.
        This is normal. Any negative distances should be ignored.

  Note: LIDAR-Lite must be idle (not BUSY) in order to perform this function.

  Process
  ------------------------------------------------------------------------------
  1.  Take a distance reading (there is no peak stack data available before
      at least one distance reading is performed)
  2.  Read stack entries

  Parameters
  ------------------------------------------------------------------------------
  peakArray: Pointer to an array of eight 16-bit unsigned data values
             for storing the Peak Strengths (Magnitude) from the stack
  distArray: Pointer to an array of eight 16-bit unsigned data values
             for storing the effective distance associated with each peak
  lidarliteAddress: Default 0x62. Fill in the new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite_v3HP::peakStackRead(
    int16_t * peakArray, int16_t * distArray, uint8_t lidarliteAddress)
{
    #define RECORD_OFFSET ((63*18) + 1)
    #define FREQ_ADJUST   (1.067)

    uint8_t   idx;
    uint8_t   dataBytes[2];
    int16_t   peakVal;
    int16_t   coarseDist;  // Zero crossing address (Coarse Distance)
    int16_t   fineDist; // Interpolated value (Fine Distance)
    int16_t   LLref;
    int16_t   LLtarget;

    // Reset the peak stack internal address pointer
    dataBytes[0] = 1;
    write(0x26, dataBytes, 1, lidarliteAddress);

    // Each time through the loop read one peak stack "entry."
    // First time through the loop always retrieves reference data entry.
    for (idx = 0 ; idx < 9 ; idx++)
    {
        // *** Read "Peak Strength"
        read(0x26, dataBytes, 2, lidarliteAddress);
        peakVal    = (int16_t) ((dataBytes[0] << 8) + dataBytes[1]);

        // *** Read "Zero Crossing Address"
        read(0x26, dataBytes, 2, lidarliteAddress);
        coarseDist = (int16_t) ((dataBytes[0] << 8) + dataBytes[1]);

        // *** Read "Interpolated Value"
        read(0x26, dataBytes, 2, lidarliteAddress);
        fineDist   = (int16_t) ((dataBytes[0] << 8) + dataBytes[1]);

        // ---------- Calculate distance from peak stack data -------------

        LLtarget = (coarseDist * 18) + fineDist;

        // The first entry in the peak stack contains reference data
        if (idx == 0)
            LLref  = LLtarget;

        // The remaining eight entries contain data corresponding to the
        // eight largest peaks in the correlation record
        if (idx != 0)
        {
            peakArray[idx-1] = peakVal;
            distArray[idx-1] = ((LLtarget - RECORD_OFFSET) - LLref) * FREQ_ADJUST;
        }
    }
} /* LIDARLite_v3HP::peakStackRead */

