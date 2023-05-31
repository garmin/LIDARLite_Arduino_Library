/*------------------------------------------------------------------------------

  LIDARLite_v4LED Arduino Library
  LIDARLite_v4LED.h

  This library provides quick access to all the basic functions of LIDAR-Lite
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
#ifndef LIDARLite_v4LED_h
#define LIDARLite_v4LED_h

#define LEGACY_I2C             1 

#define LIDARLITE_ADDR_DEFAULT 0x62

#include <Arduino.h>
#include <stdint.h>

class LIDARLite_v4LED
{
  public:
                LIDARLite_v4LED();
                LIDARLite_v4LED(TwoWire *port);
      void      configure   (uint8_t configuration = 0, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

      void      setI2Caddr  (uint8_t newAddress, uint8_t disableDefault, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      uint16_t  readDistance(uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      void      waitForBusy (uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      uint8_t   getBusyFlag (uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      void      takeRange   (uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      void      waitForBusyGpio (uint8_t monitorPin);
      uint8_t   getBusyFlagGpio (uint8_t monitorPin);
      void      takeRangeGpio   (uint8_t triggerPin, uint8_t monitorPin);

      void      write (uint8_t regAddr, uint8_t * dataBytes, uint8_t numBytes, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      void      read  (uint8_t regAddr, uint8_t * dataBytes, uint8_t numBytes, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

      void      correlationRecordRead (int16_t * correlationArray, uint8_t numberOfReadings = 192, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
  private:
      TwoWire *i2cPort;
};

#endif
